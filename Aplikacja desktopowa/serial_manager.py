import sys
import time
import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, QThread, Signal, Slot, QMutex
from utils.crc8 import calculate_crc8


class SerialWorker(QObject):
    data_received = Signal(str)
    connection_lost = Signal(str)

    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port_name = port
        self.baudrate = baudrate
        self.serial_port = None
        self.is_running = True

    def run(self):
        try:
            self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=1)
            self.serial_port.dtr = True
            self.serial_port.rts = True
            self.serial_port.reset_input_buffer()

            while self.is_running and self.serial_port.is_open:
                if self.serial_port.in_waiting > 0:
                    try:
                        line = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
                        if line:
                            self.data_received.emit(line)
                    except Exception as e:
                        print(f"Read error: {e}")
                else:
                    QThread.msleep(5)

        except serial.SerialException as e:
            self.connection_lost.emit(str(e))
        except Exception as e:
            self.connection_lost.emit(str(e))
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

    def stop(self):
        self.is_running = False

    def write(self, data: bytes):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(data)
            except Exception as e:
                print(f"Write error: {e}")


class SerialManager(QObject):
    connected = Signal(bool)
    rx_log = Signal(str, str)
    tx_log = Signal(str)
    new_data = Signal(object)
    ports_listed = Signal(object)

    def __init__(self):
        super().__init__()
        self.worker = None
        self.thread = None
        self.last_data = {"distance": 0, "filtered": 0, "error": 0, "control": 0, "setpoint": 125, "freq": 0, "stm_time": 0, "beam_angle": 0.0}
        self.sample_count = 0
        self.last_time = time.time()

    def list_ports(self):
        ports = []
        for p in serial.tools.list_ports.comports():
            desc = p.description if p.description else "Generic Serial"
            ports.append(f"{p.device} - {desc}")
        port_list_sorted = sorted(ports)
        self.ports_listed.emit(port_list_sorted)
        return port_list_sorted

    def connect_serial(self, port_name):
        if self.thread and self.thread.isRunning():
            return

        real_port = port_name.split(" - ")[0]
        self.thread = QThread()
        self.worker = SerialWorker(real_port)
        self.worker.moveToThread(self.thread)

        self.thread.started.connect(self.worker.run)
        self.worker.data_received.connect(self.handle_line)
        self.worker.connection_lost.connect(self.handle_error)

        self.thread.start()
        self.connected.emit(True)
        self.rx_log.emit(f"Connected to {port_name}", "success")

    def disconnect_serial(self):
        if self.worker:
            self.worker.stop()
        if self.thread:
            self.thread.quit()
            self.thread.wait()

        self.thread = None
        self.worker = None
        self.connected.emit(False)
        self.rx_log.emit("Disconnected", "info")

    def send_command(self, cmd):
        if self.worker and self.thread and self.thread.isRunning():
            crc = calculate_crc8(cmd)
            msg = f"{cmd};C:{crc:02X}\n"
            self.worker.write(msg.encode("utf-8"))
            self.tx_log.emit(msg.strip())
        else:
            self.rx_log.emit("Cannot send: Serial not connected", "error")

    def send_setpoint(self, val):
        self.send_command(f"S:{val:.1f}")
        self.last_data["setpoint"] = val

    def send_pid(self, kp, ki, kd):
        self.send_pid_p(kp)
        QThread.msleep(500)
        self.send_pid_i(ki)
        QThread.msleep(500)
        self.send_pid_d(kd)

    def send_pid_p(self, val):
        self.send_command(f"P:{val:.4f}")

    def send_pid_i(self, val):
        self.send_command(f"I:{val:.5f}")

    def send_pid_d(self, val):
        self.send_command(f"D:{val:.1f}")

    def send_lqr_k1(self, val):
        self.send_command(f"L1:{val:.2f}")

    def send_lqr_k2(self, val):
        self.send_command(f"L2:{val:.2f}")

    def send_lqr_k3(self, val):
        self.send_command(f"L3:{val:.2f}")

    def send_calibration(self, index, raw, target):
        self.send_command(f"CAL{index}:{raw:.1f},{target:.1f}")

    def send_control_mode(self, mode):
        self.send_command(f"M:{mode}")

    def send_pid_mode(self, mode):
        self.send_command(f"X:{mode}")

    def send_regulator_state(self, state):
        self.send_command(f"R:{state}")

    def send_sampling_rate(self, dt_ms):
        self.send_command(f"T:{int(dt_ms)}")

    def send_vision_data(self, ball_position_mm: float, beam_angle_deg: float):
        if self.worker and self.thread and self.thread.isRunning():
            data = f"V:{ball_position_mm:.1f};B:{beam_angle_deg:.2f}"
            crc = calculate_crc8(data)
            msg = f"{data};C:{crc:02X}\n"
            self.worker.write(msg.encode("utf-8"))

    @Slot(str)
    def handle_line(self, line):
        valid = False
        payload = line
        crc_val_str = "00"

        if ";C:" in line:
            parts = line.split(";C:")
            if len(parts) == 2:
                data_part = parts[0]
                try:
                    crc_recv = int(parts[1], 16)
                    crc_calc = calculate_crc8(data_part)
                    if crc_calc == crc_recv:
                        valid = True
                        payload = data_part
                        crc_val_str = f"{crc_recv:02X}"
                        self.rx_log.emit(f"RX [OK]: {line.strip()} (CRC:{crc_val_str})", "success")
                    else:
                        self.rx_log.emit(f"CRC Error: {line}", "error")
                except ValueError:
                    self.rx_log.emit(f"Fmt Error: {line}", "error")
        else:
            self.rx_log.emit(f"RAW: {line}", "info")

        if valid:
            self.sample_count += 1
            now = time.time()
            if now - self.last_time >= 1.0:
                self.last_data["freq"] = self.sample_count
                self.sample_count = 0
                self.last_time = now

            segments = payload.split(";")
            current_data = self.last_data.copy()

            for seg in segments:
                if seg.startswith("D:"):
                    try:
                        current_data["distance"] = max(0, float(seg[2:]))
                    except:
                        pass
                elif seg.startswith("F:"):
                    try:
                        current_data["filtered"] = float(seg[2:])
                    except:
                        pass
                elif seg.startswith("E:"):
                    try:
                        current_data["error"] = float(seg[2:])
                    except:
                        pass
                elif seg.startswith("A:"):
                    try:
                        current_data["control"] = float(seg[2:])
                    except:
                        pass
                elif seg.startswith("V:"):
                    try:
                        current_data["avg_error"] = float(seg[2:])
                    except:
                        pass
                elif seg.startswith("T:"):
                    try:
                        current_data["stm_time"] = int(seg[2:])
                    except:
                        pass
                elif seg.startswith("Z:"):
                    try:
                        current_data["setpoint"] = float(seg[2:])
                    except:
                        pass
                elif seg.startswith("B:"):
                    try:
                        current_data["beam_angle"] = float(seg[2:]) / 100.0
                    except:
                        pass

            self.last_data = current_data
            self.new_data.emit(current_data)

    @Slot(str)
    def handle_error(self, msg):
        self.rx_log.emit(f"Error: {msg}", "error")
        self.disconnect_serial()
