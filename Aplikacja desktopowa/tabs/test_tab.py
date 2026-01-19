import csv
import os
import time
from datetime import datetime

import pyqtgraph as pg
from PySide6.QtCore import Qt, QThread
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QLabel, QPushButton, QDoubleSpinBox, QFrame, QFileDialog

from widgets.terminal import Terminal


class TestTab(QWidget):
    def __init__(self, serial, parent=None):
        super().__init__(parent)
        self.serial = serial

        self.test_recording = False
        self.test_data = []
        self.test_start_time = 0.0
        self.test_stm_start = None

        self._build_ui()
        self._connect_logs()

    def _build_ui(self):
        pg.setConfigOption("background", "#1e293b")
        pg.setConfigOption("foreground", "#94a3b8")
        pg.setConfigOptions(antialias=True)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 10, 0, 0)
        layout.setSpacing(10)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(2)

        left_widget = QWidget()
        left_widget.setProperty("class", "card")
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(15, 15, 15, 15)
        left_layout.setSpacing(10)

        header = QLabel("TEST IDENTYFIKACJI")
        header.setStyleSheet("color: #94a3b8; font-weight: bold; font-size: 12px;")
        left_layout.addWidget(header)

        info_label = QLabel("Sygnał sterujący wgrany na STM32.\nPrzycisk START uruchamia test.")
        info_label.setStyleSheet("color: #666; font-size: 10px;")
        info_label.setWordWrap(True)
        left_layout.addWidget(info_label)

        manual_frame = QFrame()
        manual_frame.setStyleSheet("background-color: rgba(255,255,255,0.05); border-radius: 4px; padding: 5px;")
        manual_layout = QHBoxLayout(manual_frame)
        manual_layout.setContentsMargins(5, 5, 5, 5)

        lbl_angle = QLabel("Kąt:")
        lbl_angle.setStyleSheet("color: #ccc;")

        self.spin_test_angle = QDoubleSpinBox()
        self.spin_test_angle.setRange(0, 200)
        self.spin_test_angle.setValue(100.0)
        self.spin_test_angle.setSuffix(" °")
        self.spin_test_angle.setStyleSheet("background-color: #222; color: white; border: 1px solid #444;")

        self.btn_set_angle = QPushButton("Ustaw")
        self.btn_set_angle.setFixedWidth(60)
        self.btn_set_angle.setStyleSheet(
            """
            QPushButton { background-color: #64748b; color: white; border: none; border-radius: 3px; padding: 4px; }
            QPushButton:hover { background-color: #475569; }
            QPushButton:pressed { background-color: #334155; }
            """
        )
        self.btn_set_angle.clicked.connect(self._set_manual_test_angle)
        self.btn_set_angle.setEnabled(False)

        manual_layout.addWidget(lbl_angle)
        manual_layout.addWidget(self.spin_test_angle)
        manual_layout.addWidget(self.btn_set_angle)
        left_layout.addWidget(manual_frame)

        self.btn_center_servo = QPushButton("⬌ WYCENTRUJ BELKĘ")
        self.btn_center_servo.setFixedHeight(35)
        self.btn_center_servo.setStyleSheet(
            """
            QPushButton {
                background-color: #3b82f6;
                color: white;
                border: none;
                border-radius: 4px;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover { background-color: #2563eb; }
            """
        )
        self.btn_center_servo.clicked.connect(self._center_servo)
        left_layout.addWidget(self.btn_center_servo)

        self.btn_test_start = QPushButton("▶ START TEST")
        self.btn_test_start.setFixedHeight(40)
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
            QPushButton:disabled { background-color: #444; color: #666; }
            """
        )
        self.btn_test_start.clicked.connect(self._start_identification_test)
        left_layout.addWidget(self.btn_test_start)

        self.lbl_test_status = QLabel("Status: Oczekiwanie")
        self.lbl_test_status.setStyleSheet("color: #94a3b8; font-size: 11px;")
        left_layout.addWidget(self.lbl_test_status)

        self.lbl_test_samples = QLabel("Próbki: 0")
        self.lbl_test_samples.setStyleSheet("color: #3b82f6; font-size: 11px; font-weight: bold;")
        left_layout.addWidget(self.lbl_test_samples)

        self.btn_test_export = QPushButton("💾 EKSPORT CSV")
        self.btn_test_export.setFixedHeight(35)
        self.btn_test_export.setStyleSheet(
            """
            QPushButton {
                background-color: #3b82f6;
                color: white;
                border: none;
                border-radius: 4px;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover { background-color: #2563eb; }
            """
        )
        self.btn_test_export.clicked.connect(self._export_test_data)
        left_layout.addWidget(self.btn_test_export)

        self.btn_test_clear = QPushButton("🗑 WYCZYŚĆ")
        self.btn_test_clear.setFixedHeight(30)
        self.btn_test_clear.setStyleSheet(
            """
            QPushButton {
                background-color: #64748b;
                color: white;
                border: none;
                border-radius: 4px;
                font-size: 11px;
            }
            QPushButton:hover { background-color: #475569; }
            """
        )
        self.btn_test_clear.clicked.connect(self._clear_test_data)
        left_layout.addWidget(self.btn_test_clear)

        left_layout.addStretch()

        lbl_term = QLabel("Logi:")
        lbl_term.setStyleSheet("color: #666; font-size: 10px; margin-top: 10px;")
        left_layout.addWidget(lbl_term)

        self.terminal = Terminal()
        self.terminal.setMaximumHeight(200)
        left_layout.addWidget(self.terminal)

        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)

        chart_header = QLabel("WYKRESY W CZASIE RZECZYWISTYM")
        chart_header.setStyleSheet("color: #94a3b8; font-weight: bold; font-size: 12px; margin-left: 5px;")
        right_layout.addWidget(chart_header)

        self.csv_plot_dist = pg.PlotWidget()
        self.csv_plot_dist.showGrid(x=False, y=True, alpha=0.3)
        self.csv_plot_dist.setYRange(0, 260, padding=0)
        self.csv_plot_dist.setMouseEnabled(x=False, y=False)
        self.csv_plot_dist.setTitle("Odległość [mm]", color="#94a3b8", size="10pt")

        self.csv_curve_dist = self.csv_plot_dist.plot(pen=pg.mkPen(color="#22c55e", width=2))
        self.csv_curve_filt = self.csv_plot_dist.plot(pen=pg.mkPen(color="#3b82f6", width=2))

        right_layout.addWidget(self.csv_plot_dist, stretch=1)

        self.csv_plot_angle = pg.PlotWidget()
        self.csv_plot_angle.showGrid(x=False, y=True, alpha=0.3)
        self.csv_plot_angle.setMouseEnabled(x=False, y=True)
        self.csv_plot_angle.setTitle("Kąt Serwa [°]", color="#94a3b8", size="10pt")

        self.csv_curve_angle = self.csv_plot_angle.plot(pen=pg.mkPen(color="#f39c12", width=2))
        right_layout.addWidget(self.csv_plot_angle, stretch=1)

        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 3)

        layout.addWidget(splitter, stretch=1)

    def _connect_logs(self):
        self.serial.rx_log.connect(lambda msg, t: self.terminal.append_rx(msg, t))
        self.serial.tx_log.connect(self.terminal.append_tx)
        self.serial.rx_log.connect(self.handle_rx_log)

    def on_connected_changed(self, connected):
        self.btn_set_angle.setEnabled(connected)
        self.btn_center_servo.setEnabled(connected)
        self.btn_test_start.setEnabled(connected)

    def on_new_data(self, data):
        if not self.test_recording:
            return

        stm_time_ms = data.get("stm_time", 0)
        if self.test_stm_start is None:
            self.test_stm_start = stm_time_ms
        relative_time = (stm_time_ms - self.test_stm_start) / 1000.0

        test_record = {"time": round(relative_time, 4), "distance": data.get("distance", 0), "filtered": data.get("filtered", 0), "control": data.get("control", 0), "setpoint": data.get("setpoint", 0)}
        self.test_data.append(test_record)

    def refresh(self):
        if not self.test_recording or not self.test_data:
            return

        view_data = self.test_data[-200:]

        dists = [d.get("distance", 0) for d in view_data]
        filts = [d.get("filtered", 0) for d in view_data]
        angles = [d.get("control", 0) for d in view_data]

        self.csv_curve_dist.setData(dists)
        self.csv_curve_filt.setData(filts)
        self.csv_curve_angle.setData(angles)

        self.lbl_test_samples.setText(f"Próbki: {len(self.test_data)}")

    def _start_identification_test(self):
        if not self.test_recording:
            self.test_recording = True
            self.test_data = []
            self.test_start_time = time.time()
            self.test_stm_start = None

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
            self.test_recording = False
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

    def handle_rx_log(self, msg, _type=None):
        if "TEST:FINISHED" in msg and self.test_recording:
            self._start_identification_test()
            self.terminal.append_rx("Wykryto koniec sekwencji (STM32). Zatrzymano nagrywanie.", "success")

    def _center_servo(self):
        self.serial.send_regulator_state(0)
        self.serial.send_command("L:100.0")
        self.terminal.append_rx("Wycentrowano belkę. Możesz położyć piłeczkę.", "info")

    def _set_manual_test_angle(self):
        val = self.spin_test_angle.value()
        self.serial.send_regulator_state(0)
        QThread.msleep(100)
        self.serial.send_command(f"L:{val:.2f}")
        self.terminal.append_rx(f"Ustawiono kąt: {val:.2f}° (Regulator OFF)", "info")

    def _export_test_data(self):
        if not self.test_data:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"test_identyfikacji_{timestamp}.csv"

        filepath, _ = QFileDialog.getSaveFileName(self, "Zapisz dane testu", default_filename, "CSV Files (*.csv);;All Files (*)")

        if not filepath:
            return

        with open(filepath, "w", newline="", encoding="utf-8") as f:
            fieldnames = ["time", "distance", "filtered", "control", "setpoint"]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.test_data)

        self.lbl_test_status.setText(f"Zapisano: {os.path.basename(filepath)}")
        self.lbl_test_status.setStyleSheet("color: #22c55e; font-size: 11px;")

    def _clear_test_data(self):
        self.test_data = []
        self.test_recording = False

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
