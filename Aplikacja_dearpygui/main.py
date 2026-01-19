import threading
import time
from collections import deque

import serial
import serial.tools.list_ports
from dearpygui import dearpygui as dpg


class SerialClient:
    def __init__(self):
        self.ser = None
        self.running = False
        self.thread = None
        self.log_callback = None

    def set_log_callback(self, cb):
        self.log_callback = cb

    def list_ports(self):
        return [f"{p.device} - {p.description}" for p in serial.tools.list_ports.comports()]

    def connect(self, port, baud=115200):
        self.disconnect()
        device = port.split(" - ")[0]
        self.ser = serial.Serial(device, baudrate=baud, timeout=0.05)
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def send(self, msg: str):
        if not self.ser or not self.ser.is_open:
            return
        payload = msg if msg.endswith("\n") else msg + "\n"
        self.ser.write(payload.encode("utf-8"))

    def _reader(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                line = ""
            if line:
                if self.log_callback:
                    self.log_callback(line)
            time.sleep(0.01)


def main():
    client = SerialClient()
    log_buffer = deque(maxlen=500)

    dpg.create_context()

    with dpg.font_registry():
        default_font = dpg.add_font(None, 16)  # use system default, size 16

    def append_log(msg: str):
        log_buffer.append(msg)
        dpg.configure_item("log_text", default_value="\n".join(log_buffer))

    client.set_log_callback(append_log)

    def refresh_ports(sender=None, app_data=None):
        ports = client.list_ports()
        dpg.configure_item("ports_combo", items=ports)
        if ports:
            dpg.set_value("ports_combo", ports[0])

    def on_connect(sender=None, app_data=None):
        current = dpg.get_value("ports_combo")
        if not current:
            append_log("[warn] brak wybranego portu")
            return
        try:
            client.connect(current)
            append_log(f"[info] połączono z {current}")
        except Exception as e:
            append_log(f"[error] {e}")

    def on_disconnect(sender=None, app_data=None):
        client.disconnect()
        append_log("[info] rozłączono")

    def on_set_change(sender, app_data):
        val = app_data
        dpg.set_value("set_label", f"SET: {val:.1f}")
        client.send(f"SET:{val:.1f}")

    with dpg.window(label="Ball on Beam (Dear PyGui)", width=900, height=600) as main_win:
        with dpg.group(horizontal=True):
            dpg.add_combo(tag="ports_combo", width=300)
            dpg.add_button(label="Odśwież", callback=refresh_ports)
            dpg.add_button(label="Połącz", callback=on_connect)
            dpg.add_button(label="Rozłącz", callback=on_disconnect)

        dpg.add_separator()

        dpg.add_text("SET: 0.0", tag="set_label")
        dpg.add_slider_float(label="Setpoint", min_value=0.0, max_value=250.0, default_value=125.0, width=400, callback=on_set_change)

        dpg.add_separator()
        dpg.add_text("Log UART")
        dpg.add_input_text(tag="log_text", multiline=True, readonly=True, width=860, height=400)

    dpg.bind_font(default_font)
    dpg.create_viewport(title="Ball on Beam", width=920, height=640)
    dpg.setup_dearpygui()
    refresh_ports()
    dpg.show_viewport()
    dpg.set_primary_window(main_win, True)
    dpg.start_dearpygui()
    dpg.destroy_context()
    client.disconnect()


if __name__ == "__main__":
    main()
