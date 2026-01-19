import sys
import time
import csv
import os
from datetime import datetime
from PySide6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QSplitter, QFileDialog, QTabWidget, QApplication, QMessageBox, QFrame, QDoubleSpinBox
from PySide6.QtCore import Qt, QTimer, QThread, Signal
from PySide6.QtGui import QIcon

from serial_manager import SerialManager
from widgets.metrics_panel import MetricsPanel
from widgets.control_panel import ControlPanel
from widgets.charts_panel import ChartsPanel
from widgets.charts_panel import ChartsPanel
from widgets.terminal import Terminal
from utils.metrics import calculate_metrics

# Reverted Scipy Impor - Using embedded signal


class MainWindow(QMainWindow):
    def __init__(self):
            if port:
                self.serial.connect_serial(port.split(" - ")[0])
        # Global Connection Row (visible on all tabs)
            self.serial.disconnect_serial()
        self.root_layout.addLayout(self.connection_row)


        snapshot["timestamp"] = time.time()
        self.control_panel = self.control_tab.control_panel
        self.data_history.append(snapshot)
        if len(self.data_history) > 1000:
            self.data_history.pop(0)

        self.control_tab.on_new_data(data)
        self.test_tab.on_new_data(data)

        # Export Button
        self.btn_test_export.setStyleSheet(
            """
        self.control_tab.refresh(self.data_history)
        self.test_tab.refresh()

        # Open file dialog
        filepath, _ = QFileDialog.getSaveFileName(self, "Zapisz nagranie", default_filename, "CSV Files (*.csv);;All Files (*)")

        if filepath:
            try:
                with open(filepath, "w", newline="", encoding="utf-8") as f:
                    fieldnames = ["time", "distance", "filtered", "setpoint", "error", "control"]
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(self.recording_data)

                self.terminal.append_rx(f"Zapisano {len(self.recording_data)} próbek do: {filepath}", "success")
            except Exception as e:
                self.terminal.append_rx(f"Błąd zapisu: {e}", "error")

    def _update_ports(self, ports):
        current = self.combo_ports.currentText()
        self.combo_ports.clear()
        self.combo_ports.addItems(ports)

        # Auto-select "STM" port if not currently connected
        if not self.serial.thread:  # only if not connected
            for i, p in enumerate(ports):
                if "STM" in p.upper() or "ST-LINK" in p.upper():
                    self.combo_ports.setCurrentIndex(i)
                    return

        if current in ports:
            self.combo_ports.setCurrentText(current)

    def _toggle_regulator(self):
        if not self.regulator_running:
            # Start regulator
            self.regulator_running = True
            self.serial.send_regulator_state(1)

            self.btn_regulator.setText("STOP")
            self.btn_regulator.setStyleSheet(
                """
                QPushButton { 
                    background-color: #ef4444; 
                    color: white; 
                    border: none; 
                    border-radius: 3px;
                    font-weight: bold;
                    font-size: 10px;
                }
                QPushButton:hover { background-color: #dc2626; }
                QPushButton:disabled { background-color: #444; color: #666; }
            """
            )
            # LEDs: green ON, red OFF
            self.led_green.setStyleSheet("background-color: #4ade80; border-radius: 8px;")
            self.led_red.setStyleSheet("background-color: #3a1a1a; border-radius: 8px;")
        else:
            # Stop regulator
            self.regulator_running = False
            self.serial.send_regulator_state(0)

            self.btn_regulator.setText("START")
            self.btn_regulator.setStyleSheet(
                """
                QPushButton { 
                    background-color: #22c55e; 
                    color: white; 
                    border: none; 
                    border-radius: 3px;
                    font-weight: bold;
                    font-size: 10px;
                }
                QPushButton:hover { background-color: #16a34a; }
                QPushButton:disabled { background-color: #444; color: #666; }
            """
            )
            # LEDs: red ON, green OFF
            self.led_red.setStyleSheet("background-color: #ef4444; border-radius: 8px;")
            self.led_green.setStyleSheet("background-color: #1a3a1a; border-radius: 8px;")

    def _toggle_connection(self):
        if self.btn_connect.text() == "Połącz":
            port = self.combo_ports.currentText()
            import sys
            import time
            import os
            from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
                                           QComboBox, QTabWidget, QApplication, 
                                           QMessageBox)
            from PySide6.QtCore import Qt, QTimer, QThread, Signal
            from PySide6.QtGui import QIcon

            from serial_manager import SerialManager
            from tabs.control_tab import ControlTab
            from tabs.test_tab import TestTab
            self.serial.send_command("L:100.0")  # Center Servo (Angle)

            # Enable Test Tab controls
            self.btn_set_angle.setEnabled(True)
            self.btn_center_servo.setEnabled(True)
            self.btn_test_start.setEnabled(True)
        else:
            self.btn_connect.setText("Połącz")
            self.btn_connect.setObjectName("connectBtn")
            self.lbl_status.setText("ROZŁĄCZONY")
            self.lbl_status.setStyleSheet("color: #ef4444; font-weight: bold; font-size: 10px;")
            self.combo_ports.setEnabled(True)
            self.btn_refresh.setEnabled(True)
            self.btn_connect.setStyle(self.btn_connect.style())

            # Disable Test Tab controls
            self.btn_set_angle.setEnabled(False)
            self.btn_center_servo.setEnabled(False)
            self.btn_test_start.setEnabled(False)

    def _on_new_data(self, data):
        # Always append to history for charting
        # Note: serial_manager emits existing dict reference but updated values,
        # so we should copy it for history to avoid mutations affecting old history
        snapshot = data.copy()
        current_time = time.time()
        snapshot["timestamp"] = current_time

        self.data_history.append(snapshot)
        if len(self.data_history) > 1000:  # Keep more history in desktop
            self.data_history.pop(0)

        # Recording logic - use STM32's internal time (HAL_GetTick in ms)
        if self.is_recording:
            stm_time_ms = data.get("stm_time", 0)
            # Convert to seconds and make relative to first sample
            if not hasattr(self, "recording_stm_start") or self.recording_stm_start is None:
                self.recording_stm_start = stm_time_ms
            relative_time = (stm_time_ms - self.recording_stm_start) / 1000.0

            record = {"time": round(relative_time, 4), "distance": data.get("distance", 0), "filtered": data.get("filtered", 0), "setpoint": data.get("setpoint", 0), "error": data.get("error", 0), "control": data.get("control", 0)}
            self.recording_data.append(record)

            # Update sample count in UI
            self.lbl_rec_samples.setText(str(len(self.recording_data)))

        # Update Control Panel immediate values
        self.control_panel.update_data(data)

        # Collect data for Test Identification tab if recording
        if hasattr(self, "test_recording") and self.test_recording:
            stm_time_ms = data.get("stm_time", 0)
            # Use STM32 internal time for accurate sampling
            if not hasattr(self, "test_stm_start") or self.test_stm_start is None:
                self.test_stm_start = stm_time_ms
            relative_time = (stm_time_ms - self.test_stm_start) / 1000.0

            test_record = {"time": round(relative_time, 4), "distance": data.get("distance", 0), "filtered": data.get("filtered", 0), "control": data.get("control", 0), "setpoint": data.get("setpoint", 0)}
            self.test_data.append(test_record)

    def _update_ui_tick(self):
        # Called at 10Hz to refresh charts and metrics
        # This decouples high-freq serial data from UI rendering
        if not self.data_history:
            return

        latest = self.data_history[-1]

        # Metrics
        computed = calculate_metrics(self.data_history)
        self.metrics_panel.update_metrics(latest, computed)

        # Charts
        self.charts_panel.update_charts(self.data_history)

        # Update Test Identification tab charts if recording
        if hasattr(self, "test_recording") and self.test_recording:
            self._update_test_charts()

    def _update_test_charts(self):
        """Update charts on Test Identification tab"""
        if not self.test_data:
            return

        view_data = self.test_data[-200:]

        dists = [d.get("distance", 0) for d in view_data]
        filts = [d.get("filtered", 0) for d in view_data]
        angles = [d.get("control", 0) for d in view_data]

        self.csv_curve_dist.setData(dists)
        self.csv_curve_filt.setData(filts)
        self.csv_curve_angle.setData(angles)

        # Update sample count
        self.lbl_test_samples.setText(f"Próbki: {len(self.test_data)}")

    def _start_identification_test(self):
        """Start/stop identification test recording"""
        if not self.test_recording:
            # Start test
            self.test_recording = True
            self.test_data = []
            self.test_start_time = time.time()
            self.test_stm_start = None  # Reset STM32 time reference

            # Send start command to STM32
            self.serial.send_command("TEST:START")

            self.btn_test_start.setText("⏹ STOP TEST")
            self.btn_test_start.setStyleSheet(
                """
                QPushButton { 
                    background-color: #ef4444; 
                    color: white; 
                    border: none; 
                    border-radius: 4px;
                    font-weight: bold;
                    font-size: 14px;
                }
                QPushButton:hover { background-color: #dc2626; }
            """
            )
            self.lbl_test_status.setText("Status: Nagrywanie...")
            self.lbl_test_status.setStyleSheet("color: #22c55e; font-size: 11px;")
        else:
            # Stop test
            self.test_recording = False

            # Send stop command to STM32
            self.serial.send_command("TEST:STOP")

            self.btn_test_start.setText("▶ START TEST")
            self.btn_test_start.setStyleSheet(
                """
                QPushButton { 
                    background-color: #22c55e; 
                    color: white; 
                    border: none; 
                    border-radius: 4px;
                    font-weight: bold;
                    font-size: 14px;
                }
                QPushButton:hover { background-color: #16a34a; }
            """
            )
            self.lbl_test_status.setText(f"Status: Zakończono ({len(self.test_data)} próbek)")
            self.lbl_test_status.setStyleSheet("color: #94a3b8; font-size: 11px;")

    def _handle_rx_log(self, msg, type):
        # Detect end of test sequence
        if "TEST:FINISHED" in msg:
            if self.test_recording:
                self._start_identification_test()  # Toggle off -> executes stop logic
                self.terminal.append_rx("Wykryto koniec sekwencji (STM32). Zatrzymano nagrywanie.", "success")

    def _center_servo(self):
        """Center the servo so user can place the ball"""
        self.serial.send_regulator_state(0)  # Disable regulator
        self.serial.send_command("L:100.0")  # Center servo angle
        self.terminal.append_rx("Wycentrowano belkę. Możesz położyć piłeczkę.", "info")

        # Update regulator button state in UI
        self.regulator_running = False
        self.btn_regulator.setText("START")
        self.btn_regulator.setStyleSheet(
            """
            QPushButton { 
                background-color: #22c55e; 
                color: white; 
                border: none; 
                border-radius: 3px;
                font-weight: bold;
                font-size: 10px;
            }
            QPushButton:hover { background-color: #16a34a; }
            QPushButton:disabled { background-color: #444; color: #666; }
        """
        )
        self.led_red.setStyleSheet("background-color: #ef4444; border-radius: 8px;")
        self.led_green.setStyleSheet("background-color: #1a3a1a; border-radius: 8px;")

    def _set_manual_test_angle(self):
        """Set manual servo angle and disable regulator"""
        val = self.spin_test_angle.value()
        self.serial.send_regulator_state(0)  # Disable regulator
        QThread.msleep(100)  # Wait for STM32 to process R:0
        self.serial.send_command(f"L:{val:.2f}")  # Set angle
        self.terminal.append_rx(f"Ustawiono kąt: {val:.2f}° (Regulator OFF)", "info")

        # UI Sync
        self.regulator_running = False
        self.btn_regulator.setText("START")
        self.btn_regulator.setStyleSheet(
            """
            QPushButton { 
                background-color: #22c55e; 
                color: white; 
                border: none; 
                border-radius: 3px;
                font-weight: bold;
                font-size: 10px;
            }
            QPushButton:hover { background-color: #16a34a; }
            QPushButton:disabled { background-color: #444; color: #666; }
        """
        )
        self.led_red.setStyleSheet("background-color: #ef4444; border-radius: 8px;")
        self.led_green.setStyleSheet("background-color: #1a3a1a; border-radius: 8px;")

    def _export_test_data(self):
        """Export recorded test data to CSV"""
        if not self.test_data:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"test_identyfikacji_{timestamp}.csv"

        filepath, _ = QFileDialog.getSaveFileName(self, "Zapisz dane testu", default_filename, "CSV Files (*.csv);;All Files (*)")

        if filepath:
            try:
                with open(filepath, "w", newline="", encoding="utf-8") as f:
                    fieldnames = ["time", "distance", "filtered", "control", "setpoint"]
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(self.test_data)

                self.lbl_test_status.setText(f"Zapisano: {os.path.basename(filepath)}")
                self.lbl_test_status.setStyleSheet("color: #22c55e; font-size: 11px;")
            except Exception as e:
                self.lbl_test_status.setText(f"Błąd: {e}")
                self.lbl_test_status.setStyleSheet("color: #ef4444; font-size: 11px;")

    def _clear_test_data(self):
        """Clear recorded test data"""
        self.test_data = []
        self.test_recording = False

        # Clear charts
        self.csv_curve_dist.setData([])
        self.csv_curve_filt.setData([])
        self.csv_curve_angle.setData([])

        self.lbl_test_samples.setText("Próbki: 0")
        self.lbl_test_status.setText("Status: Oczekiwanie")
        self.lbl_test_status.setStyleSheet("color: #94a3b8; font-size: 11px;")

        self.btn_test_start.setText("▶ START TEST")
        self.btn_test_start.setStyleSheet(
            """
            QPushButton { 
                background-color: #22c55e; 
                color: white; 
                border: none; 
                border-radius: 4px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover { background-color: #16a34a; }
        """
        )
