import csv
import time
from datetime import datetime
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QLabel, QPushButton, QFileDialog
from PySide6.QtCore import Qt

from widgets.metrics_panel import MetricsPanel
from widgets.control_panel import ControlPanel
from widgets.charts_panel import ChartsPanel
from widgets.terminal import Terminal
from utils.metrics import calculate_metrics


class ControlTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.is_recording = False
        self.recording_data = []
        self.recording_start_time = 0.0
        self.recording_stm_start = None

        self.metrics_panel = MetricsPanel()
        self.control_panel = ControlPanel()
        self.terminal = Terminal()
        self.charts_panel = ChartsPanel()

        self._build_layout()

    def _build_layout(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 10, 0, 0)
        main_layout.setSpacing(10)

        top_row_layout = QHBoxLayout()
        top_row_layout.setSpacing(8)

        self.rec_tile = self._create_recording_tile()
        top_row_layout.addWidget(self.rec_tile)
        top_row_layout.addWidget(self.metrics_panel)

        main_layout.addLayout(top_row_layout)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(2)

        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 5, 0)
        left_layout.setSpacing(10)
        left_layout.addWidget(self.control_panel)
        left_layout.addWidget(self.terminal, stretch=1)

        splitter.addWidget(left_widget)
        splitter.addWidget(self.charts_panel)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)

        main_layout.addWidget(splitter, stretch=1)

    def _create_recording_tile(self):
        tile = QWidget()
        tile.setProperty("class", "card")
        tile.setFixedWidth(160)

        layout = QVBoxLayout(tile)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        self.lbl_rec_status = QLabel("REC: STOP")
        self.lbl_rec_status.setStyleSheet("color: #94a3b8; font-weight: bold; font-size: 10px;")
        layout.addWidget(self.lbl_rec_status)

        row_controls = QHBoxLayout()
        row_controls.setSpacing(4)

        self.btn_rec_toggle = QPushButton("▶")
        self.btn_rec_toggle.setFixedSize(32, 26)
        self.btn_rec_toggle.setStyleSheet(
            """
            QPushButton {
                background-color: #22c55e;
                color: white;
                border: none;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #16a34a; }
            """
        )
        self.btn_rec_toggle.clicked.connect(self._toggle_recording)

        self.lbl_rec_samples = QLabel("0")
        self.lbl_rec_samples.setStyleSheet("color: #94a3b8; font-size: 10px;")

        row_controls.addWidget(self.btn_rec_toggle)
        row_controls.addWidget(self.lbl_rec_samples)
        row_controls.addStretch()

        layout.addLayout(row_controls)
        return tile

    def _toggle_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.recording_data = []
            self.recording_start_time = time.time()
            self.recording_stm_start = None

            self.btn_rec_toggle.setText("⏹")
            self.btn_rec_toggle.setStyleSheet(
                """
                QPushButton {
                    background-color: #ef4444;
                    color: white;
                    border: none;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #dc2626; }
                """
            )
            self.lbl_rec_status.setText("REC: ●")
            self.lbl_rec_status.setStyleSheet("color: #ef4444; font-weight: bold; font-size: 10px;")
            self.lbl_rec_samples.setText("0")
        else:
            self.is_recording = False
            self.btn_rec_toggle.setText("▶")
            self.btn_rec_toggle.setStyleSheet(
                """
                QPushButton {
                    background-color: #22c55e;
                    color: white;
                    border: none;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #16a34a; }
                """
            )
            self.lbl_rec_status.setText("REC: STOP")
            self.lbl_rec_status.setStyleSheet("color: #94a3b8; font-weight: bold; font-size: 10px;")

            if self.recording_data:
                self._save_recording()

    def _save_recording(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"nagranie_{timestamp}.csv"

        filepath, _ = QFileDialog.getSaveFileName(self, "Zapisz nagranie", default_filename, "CSV Files (*.csv);;All Files (*)")

        if not filepath:
            return

        with open(filepath, "w", newline="", encoding="utf-8") as f:
            fieldnames = ["time", "distance", "filtered", "setpoint", "error", "control"]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.recording_data)

        self.terminal.append_rx(f"Zapisano {len(self.recording_data)} próbek do: {filepath}", "success")

    def on_new_data(self, data):
        self.control_panel.update_data(data)

        if not self.is_recording:
            return

        stm_time_ms = data.get("stm_time", 0)
        if self.recording_stm_start is None:
            self.recording_stm_start = stm_time_ms
        relative_time = (stm_time_ms - self.recording_stm_start) / 1000.0

        record = {"time": round(relative_time, 4), "distance": data.get("distance", 0), "filtered": data.get("filtered", 0), "setpoint": data.get("setpoint", 0), "error": data.get("error", 0), "control": data.get("control", 0)}
        self.recording_data.append(record)
        self.lbl_rec_samples.setText(str(len(self.recording_data)))

    def refresh(self, data_history):
        if not data_history:
            return

        latest = data_history[-1]
        computed = calculate_metrics(data_history)
        self.metrics_panel.update_metrics(latest, computed)
        self.charts_panel.update_charts(data_history)
