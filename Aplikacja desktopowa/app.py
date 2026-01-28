import sys
import time
import csv
import os
from datetime import datetime
from PySide6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QSplitter, QFileDialog, QTabWidget, QApplication, QMessageBox, QFrame, QDoubleSpinBox, QCheckBox, QSpinBox
from PySide6.QtCore import Qt, QTimer, QThread, Signal
from PySide6.QtGui import QIcon

from serial_manager import SerialManager
from widgets.metrics_panel import MetricsPanel
from widgets.control_panel import ControlPanel
from widgets.charts_panel import ChartsPanel
from widgets.charts_panel import ChartsPanel
from widgets.terminal import Terminal
from widgets.opencv_panel import OpenCVPanel
from utils.metrics import calculate_metrics

# Reverted Scipy Impor - Using embedded signal


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Panel Sterowania Ball on Beam")
        self.resize(1280, 800)

        # --- Managers ---
        self.serial = SerialManager()
        self.data_history = []

        # --- Recording State ---
        self.is_recording = False
        self.recording_data = []
        self.recording_start_time = 0

        # --- UI Setup ---
        # Create main central widget with connection bar always visible
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.root_layout = QVBoxLayout(self.central_widget)
        self.root_layout.setContentsMargins(10, 10, 10, 10)
        self.root_layout.setSpacing(10)

        # Global Connection Row (visible on all tabs)
        self.connection_row = QHBoxLayout()
        self.connection_row.setSpacing(8)

        self.conn_tile = self._create_connection_tile()
        self.connection_row.addWidget(self.conn_tile)

        # Regulator Start/Stop Tile
        self.regulator_tile = self._create_regulator_tile()
        self.connection_row.addWidget(self.regulator_tile)

        # Recording Tile (global - available on all tabs)
        self.rec_tile = self._create_recording_tile()
        self.connection_row.addWidget(self.rec_tile)

        # Sampling Rate Control (global)
        self.sampling_tile = self._create_sampling_tile()
        self.connection_row.addWidget(self.sampling_tile)

        # Angle Control (global)
        self.angle_tile = self._create_angle_tile()
        self.connection_row.addWidget(self.angle_tile)

        self.connection_row.addStretch()

        self.root_layout.addLayout(self.connection_row)

        # Tab widget below connection bar
        self.tab_widget = QTabWidget()
        self.root_layout.addWidget(self.tab_widget, stretch=1)

        # --- Tab 1: Sterowanie (main control panel) ---
        self.control_tab = QWidget()
        self.main_layout = QVBoxLayout(self.control_tab)
        self.main_layout.setContentsMargins(0, 10, 0, 0)
        self.main_layout.setSpacing(10)

        # 1. Top Row: Metrics (recording moved to global bar)
        self.top_row_layout = QHBoxLayout()
        self.top_row_layout.setSpacing(8)

        # 1.1 Metrics Panel
        self.metrics_panel = MetricsPanel()
        self.top_row_layout.addWidget(self.metrics_panel)

        self.main_layout.addLayout(self.top_row_layout)

        # 2. Main Content Splitter (Control | Charts)
        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(2)

        # Left Panel (Control + Terminal)
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 5, 0)
        left_layout.setSpacing(10)

        self.control_panel = ControlPanel()
        self.terminal = Terminal()

        left_layout.addWidget(self.control_panel)
        left_layout.addWidget(self.terminal, stretch=1)

        # Right Panel (Charts)
        self.charts_panel = ChartsPanel()

        splitter.addWidget(left_widget)
        splitter.addWidget(self.charts_panel)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)

        self.main_layout.addWidget(splitter, stretch=1)

        # Add OpenCV Tab first
        self.opencv_panel = OpenCVPanel()
        self.tab_widget.addTab(self.opencv_panel, "OpenCV")

        # Add control tab
        self.tab_widget.addTab(self.control_tab, "Sterowanie")

        # --- Tab 2: CSV Signal Player ---
        self._create_csv_player_tab()

        # --- Wire Signals ---

        # Serial -> UI
        self.serial.ports_listed.connect(self._update_ports)
        self.serial.connected.connect(self._on_connected_changed)
        self.serial.rx_log.connect(self.terminal.append_rx)
        self.serial.tx_log.connect(self.terminal.append_tx)
        self.serial.new_data.connect(self._on_new_data)

        # Control UI -> Serial
        # self.control_panel.pid_update.connect(self.serial.send_pid) # Removed batch update
        self.control_panel.kp_update.connect(self.serial.send_pid_p)
        self.control_panel.ki_update.connect(self.serial.send_pid_i)
        self.control_panel.kd_update.connect(self.serial.send_pid_d)

        self.control_panel.setpoint_update.connect(self.serial.send_setpoint)
        self.control_panel.calibration_update.connect(self.serial.send_calibration)
        self.control_panel.mode_update.connect(self.serial.send_control_mode)
        self.control_panel.pid_mode_update.connect(self.serial.send_pid_mode)

        # Connection Tile Interactions
        self.btn_refresh.clicked.connect(self.serial.list_ports)
        self.btn_connect.clicked.connect(self._toggle_connection)

        # Timer for chart updates (throttled 10 FPS)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_ui_tick)
        self.update_timer.start(100)

        self.serial.list_ports()

        # Connect rx_log to handler for special messages
        self.serial.rx_log.connect(self._handle_rx_log)

        # Bezposrednie wysylanie setpointu
        self.control_panel.setpoint_update.disconnect()
        self.control_panel.setpoint_update.connect(self._on_setpoint_change)

        # OpenCV -> STM32: Wysyłanie pozycji piłeczki i kąta belki
        self.opencv_panel.ball_on_beam_update.connect(self._on_vision_ball_update)
        self.opencv_panel.beam_angle_update.connect(self._on_vision_angle_update)

        # Przechowywanie ostatnich danych z wizji
        self._vision_ball_pos_mm = -1.0
        self._vision_beam_angle = 0.0

        # Timer do wysyłania danych wizyjnych (30 Hz)
        self.vision_send_timer = QTimer()
        self.vision_send_timer.timeout.connect(self._send_vision_data_to_stm)
        self.vision_send_timer.start(33)  # ~30 FPS

    def _on_setpoint_change(self, val):
        self.serial.send_setpoint(val)

    def _on_vision_ball_update(self, position_ratio: float):
        """Handler dla aktualizacji pozycji piłeczki z OpenCV (0.0-1.0 lub -1 jeśli nie wykryta)"""
        if position_ratio >= 0:
            self._vision_ball_pos_mm = position_ratio * 250.0  # Konwersja na mm (0-250)
        else:
            self._vision_ball_pos_mm = -1.0  # Piłeczka nie wykryta

    def _on_vision_angle_update(self, angle_deg: float):
        """Handler dla aktualizacji kąta belki z OpenCV (w stopniach)"""
        self._vision_beam_angle = angle_deg

    def _send_vision_data_to_stm(self):
        """Wysyła dane z wizji do STM32 przez UART (30 Hz)"""
        if self._vision_ball_pos_mm >= 0:
            self.serial.send_vision_data(self._vision_ball_pos_mm, self._vision_beam_angle)

    def _create_connection_tile(self):
        tile = QWidget()
        tile.setProperty("class", "card")
        tile.setFixedWidth(320)

        layout = QHBoxLayout(tile)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(6)

        # Status Label
        self.lbl_status = QLabel("ROZŁĄCZONY")
        self.lbl_status.setStyleSheet("color: #ef4444; font-weight: bold; font-size: 10px;")
        self.lbl_status.setFixedWidth(75)
        layout.addWidget(self.lbl_status)

        # Port Combo
        self.combo_ports = QComboBox()
        self.combo_ports.setFixedHeight(26)
        self.combo_ports.setStyleSheet(
            """
            QComboBox { 
                background-color: rgba(255,255,255,0.05); 
                border: 1px solid #444; 
                padding: 2px; 
                color: white;
                font-size: 10px;
            }
            QComboBox:disabled { color: #555; border-color: #222; }
        """
        )
        layout.addWidget(self.combo_ports, stretch=1)

        # Refresh Button
        self.btn_refresh = QPushButton("↻")
        self.btn_refresh.setFixedSize(26, 26)
        self.btn_refresh.setStyleSheet("background-color: rgba(255,255,255,0.05); border: 1px solid #444; font-size: 12px;")
        layout.addWidget(self.btn_refresh)

        # Connect Button
        self.btn_connect = QPushButton("Połącz")
        self.btn_connect.setObjectName("connectBtn")
        self.btn_connect.setFixedHeight(26)
        layout.addWidget(self.btn_connect)

        return tile

    def _create_regulator_tile(self):
        tile = QWidget()
        tile.setProperty("class", "card")
        tile.setFixedWidth(140)

        layout = QHBoxLayout(tile)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(6)

        # LED indicators (red = stop, green = run) - using fixed size widgets
        self.led_red = QLabel()
        self.led_red.setFixedSize(16, 16)
        self.led_red.setStyleSheet("background-color: #3a1a1a; border-radius: 8px;")  # Dim (inactive)
        layout.addWidget(self.led_red)

        self.led_green = QLabel()
        self.led_green.setFixedSize(16, 16)
        self.led_green.setStyleSheet("background-color: #4ade80; border-radius: 8px;")  # Active (running)
        layout.addWidget(self.led_green)

        # Start/Stop Button
        self.btn_regulator = QPushButton("STOP")
        self.btn_regulator.setObjectName("startBtn")
        self.btn_regulator.setFixedSize(60, 26)
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
        self.btn_regulator.clicked.connect(self._toggle_regulator)
        layout.addWidget(self.btn_regulator)

        # Track regulator state
        self.regulator_running = True

        return tile

    def _create_sampling_tile(self):
        tile = QFrame()
        tile.setProperty("class", "card")
        tile.setFixedHeight(50)
        layout = QHBoxLayout(tile)
        layout.setContentsMargins(10, 5, 10, 5)
        layout.setSpacing(5)

        self.sampling_preset_btns = []
        for ms in [10, 20, 30, 50, 100]:
            btn = QPushButton(f"{ms}")
            btn.setFixedSize(45, 28)
            btn.setStyleSheet("QPushButton { background-color: #581c87; color: white; border: none; border-radius: 3px; font-size: 11px; font-weight: bold; }" "QPushButton:hover { background-color: #7e22ce; }")
            btn.clicked.connect(lambda checked, v=ms: self._set_sampling_preset(v))
            btn.setEnabled(False)
            layout.addWidget(btn)
            self.sampling_preset_btns.append(btn)

        self.spin_sampling_custom = QSpinBox()
        self.spin_sampling_custom.setRange(5, 500)
        self.spin_sampling_custom.setValue(30)
        self.spin_sampling_custom.setStyleSheet("background-color: #222; color: white; border: 1px solid #581c87; font-size: 10px;")
        self.spin_sampling_custom.setFixedWidth(55)
        layout.addWidget(self.spin_sampling_custom)

        self.btn_sampling_send = QPushButton("OK")
        self.btn_sampling_send.setFixedSize(32, 28)
        self.btn_sampling_send.setStyleSheet("QPushButton { background-color: #7e22ce; color: white; border: none; border-radius: 3px; font-size: 10px; font-weight: bold; }" "QPushButton:hover { background-color: #9333ea; }")
        self.btn_sampling_send.clicked.connect(self._send_custom_sampling)
        self.btn_sampling_send.setEnabled(False)
        layout.addWidget(self.btn_sampling_send)

        return tile

    def _send_custom_sampling(self):
        ms = self.spin_sampling_custom.value()
        self.serial.send_sampling_rate(ms)
        self.terminal.append_tx(f"Próbkowanie: {ms} ms")

    def _set_sampling_preset(self, ms):
        self.serial.send_sampling_rate(ms)
        self.terminal.append_tx(f"Próbkowanie: {ms} ms")

    def _create_angle_tile(self):
        tile = QFrame()
        tile.setProperty("class", "card")
        tile.setFixedHeight(50)
        layout = QHBoxLayout(tile)
        layout.setContentsMargins(10, 5, 10, 5)
        layout.setSpacing(5)

        self.angle_preset_btns = []
        for angle in [60, 80, 100, 120, 140]:
            btn = QPushButton(str(angle))
            btn.setFixedSize(45, 28)
            btn.setStyleSheet("QPushButton { background-color: #334155; color: white; border: none; border-radius: 3px; font-size: 11px; font-weight: bold; }" "QPushButton:hover { background-color: #475569; }")
            btn.clicked.connect(lambda checked, a=angle: self._set_angle_preset(a))
            btn.setEnabled(False)
            layout.addWidget(btn)
            self.angle_preset_btns.append(btn)

        self.spin_angle_custom = QSpinBox()
        self.spin_angle_custom.setRange(0, 200)
        self.spin_angle_custom.setValue(100)
        self.spin_angle_custom.setStyleSheet("background-color: #222; color: white; border: 1px solid #475569; font-size: 10px;")
        self.spin_angle_custom.setFixedWidth(55)
        layout.addWidget(self.spin_angle_custom)

        self.btn_angle_send = QPushButton("OK")
        self.btn_angle_send.setFixedSize(32, 28)
        self.btn_angle_send.setStyleSheet("QPushButton { background-color: #475569; color: white; border: none; border-radius: 3px; font-size: 10px; font-weight: bold; }" "QPushButton:hover { background-color: #64748b; }")
        self.btn_angle_send.clicked.connect(self._send_custom_angle)
        self.btn_angle_send.setEnabled(False)
        layout.addWidget(self.btn_angle_send)

        return tile

    def _send_custom_angle(self):
        angle = self.spin_angle_custom.value()
        self.serial.send_command(f"L:{angle}")
        self.terminal.append_tx(f"Kąt serwa: {angle}")

    def _set_angle_preset(self, angle):
        self.serial.send_command(f"L:{angle}")
        self.terminal.append_tx(f"Kąt serwa: {angle}")

    def _create_csv_player_tab(self):
        """Create Tab 2: Test Runner with charts and export"""
        import pyqtgraph as pg

        self.csv_tab = QWidget()
        csv_layout = QVBoxLayout(self.csv_tab)
        csv_layout.setContentsMargins(0, 10, 0, 0)
        csv_layout.setSpacing(10)

        # Main content splitter (same layout as tab 1)
        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(2)

        # --- Left Panel: Test Controls ---
        left_widget = QWidget()
        left_widget.setProperty("class", "card")
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(15, 15, 15, 15)
        left_layout.setSpacing(10)

        # Make left layout stretchable to push terminal to bottom
        # We will add a scroll area if needed, but for now just layout

        # Header
        header = QLabel("TEST IDENTYFIKACJI")
        header.setStyleSheet("color: #94a3b8; font-weight: bold; font-size: 12px;")
        left_layout.addWidget(header)

        # Info
        info_label = QLabel("Sygnał sterujący wgrany na STM32.\nPrzycisk START uruchamia test.")
        info_label.setStyleSheet("color: #666; font-size: 10px;")
        info_label.setWordWrap(True)
        info_label.setWordWrap(True)
        left_layout.addWidget(info_label)

        # Center Servo Button
        self.btn_center_servo = QPushButton("WYCENTRUJ BELKE")
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

        # Start Test Button (Embedded)
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

        # Recording status
        self.lbl_test_status = QLabel("Status: Oczekiwanie")
        self.lbl_test_status.setStyleSheet("color: #94a3b8; font-size: 11px;")
        left_layout.addWidget(self.lbl_test_status)

        self.lbl_test_samples = QLabel("Próbki: 0")
        self.lbl_test_samples.setStyleSheet("color: #3b82f6; font-size: 11px; font-weight: bold;")
        left_layout.addWidget(self.lbl_test_samples)

        # Export Button
        self.btn_test_export = QPushButton("EKSPORT CSV")
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

        # Clear Button
        self.btn_test_clear = QPushButton("WYCZYSC")
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

        # --- Terminal for Tab 2 ---
        left_layout.addStretch()  # Push terminal to bottom of left panel

        lbl_term = QLabel("Logi:")
        lbl_term.setStyleSheet("color: #666; font-size: 10px; margin-top: 10px;")
        left_layout.addWidget(lbl_term)

        self.terminal_test = Terminal()
        self.terminal_test.setMaximumHeight(200)  # Limit height in left panel
        left_layout.addWidget(self.terminal_test)

        # Connect signals to this terminal too
        self.serial.rx_log.connect(lambda msg, t: self.terminal_test.append_rx(msg, t))
        self.serial.tx_log.connect(self.terminal_test.append_tx)

        # --- Right Panel: Charts with Checkboxes ---
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(5)

        # Header with checkboxes
        header_row = QHBoxLayout()
        header_row.setSpacing(10)

        chart_header = QLabel("WYKRESY")
        chart_header.setStyleSheet("color: #94a3b8; font-weight: bold; font-size: 12px;")
        header_row.addWidget(chart_header)

        # Checkboxes for chart visibility
        self.chk_show_distance = QCheckBox("Odległość")
        self.chk_show_distance.setChecked(True)
        self.chk_show_distance.setStyleSheet("color: #22c55e; font-size: 10px;")
        self.chk_show_distance.stateChanged.connect(self._update_test_chart_visibility)
        header_row.addWidget(self.chk_show_distance)

        self.chk_show_setpoint = QCheckBox("Setpoint")
        self.chk_show_setpoint.setChecked(True)
        self.chk_show_setpoint.setStyleSheet("color: #f39c12; font-size: 10px;")
        self.chk_show_setpoint.stateChanged.connect(self._update_test_chart_visibility)
        header_row.addWidget(self.chk_show_setpoint)

        self.chk_show_error = QCheckBox("Uchyb")
        self.chk_show_error.setChecked(True)
        self.chk_show_error.setStyleSheet("color: #ef4444; font-size: 10px;")
        self.chk_show_error.stateChanged.connect(self._update_test_chart_visibility)
        header_row.addWidget(self.chk_show_error)

        self.chk_show_control = QCheckBox("Sterowanie")
        self.chk_show_control.setChecked(True)
        self.chk_show_control.setStyleSheet("color: #3498db; font-size: 10px;")
        self.chk_show_control.stateChanged.connect(self._update_test_chart_visibility)
        header_row.addWidget(self.chk_show_control)

        self.chk_show_beam = QCheckBox("Kąt belki")
        self.chk_show_beam.setChecked(True)
        self.chk_show_beam.setStyleSheet("color: #f59e0b; font-size: 10px;")
        self.chk_show_beam.stateChanged.connect(self._update_test_chart_visibility)
        header_row.addWidget(self.chk_show_beam)

        header_row.addStretch()
        right_layout.addLayout(header_row)

        # PyQtGraph setup
        pg.setConfigOption("background", "#1e293b")
        pg.setConfigOption("foreground", "#94a3b8")
        pg.setConfigOptions(antialias=True)

        # Chart 1: Distance & Setpoint (Main)
        self.csv_plot_dist = pg.PlotWidget()
        self.csv_plot_dist.showGrid(x=False, y=True, alpha=0.3)
        self.csv_plot_dist.setYRange(0, 260, padding=0)
        self.csv_plot_dist.setMouseEnabled(x=False, y=False)
        self.csv_plot_dist.setTitle("Odległość / Setpoint [mm]", color="#94a3b8", size="10pt")

        self.csv_curve_setpoint = self.csv_plot_dist.plot(pen=pg.mkPen(color="#f39c12", width=2, style=Qt.DashLine), name="Setpoint")
        self.csv_curve_dist = self.csv_plot_dist.plot(pen=pg.mkPen(color="#22c55e", width=2, style=Qt.DotLine), name="Dystans")
        self.csv_curve_filt = self.csv_plot_dist.plot(pen=pg.mkPen(color="#3b82f6", width=2), name="Filtrowany")

        right_layout.addWidget(self.csv_plot_dist, stretch=2)

        # Chart 2: Error
        self.csv_plot_error = pg.PlotWidget()
        self.csv_plot_error.showGrid(x=False, y=True, alpha=0.3)
        self.csv_plot_error.setMouseEnabled(x=False, y=True)
        self.csv_plot_error.setTitle("Uchyb Regulacji", color="#94a3b8", size="10pt")

        self.csv_curve_error = self.csv_plot_error.plot(pen=pg.mkPen(color="#ef4444", width=2), fillLevel=0, brush=(239, 68, 68, 50))

        right_layout.addWidget(self.csv_plot_error, stretch=1)

        # Chart 3: Control & Beam Angle
        self.csv_plot_ctrl = pg.PlotWidget()
        self.csv_plot_ctrl.showGrid(x=False, y=True, alpha=0.3)
        self.csv_plot_ctrl.setMouseEnabled(x=False, y=True)
        self.csv_plot_ctrl.setTitle("Sterowanie / Kąt Belki", color="#94a3b8", size="10pt")

        self.csv_curve_ctrl = self.csv_plot_ctrl.plot(pen=pg.mkPen(color="#3498db", width=2), name="Sterowanie")
        self.csv_curve_beam = self.csv_plot_ctrl.plot(pen=pg.mkPen(color="#f59e0b", width=2, style=Qt.DashLine), name="Kąt belki")

        right_layout.addWidget(self.csv_plot_ctrl, stretch=1)

        # Add to splitter
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 3)

        csv_layout.addWidget(splitter, stretch=1)

        self.tab_widget.addTab(self.csv_tab, "Test Identyfikacji")

        # Test data state
        self.test_recording = False
        self.test_data = []
        self.test_start_time = 0

    def _create_recording_tile(self):
        tile = QWidget()
        tile.setProperty("class", "card")
        tile.setFixedWidth(160)  # Compact width

        layout = QVBoxLayout(tile)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        # Status Label (shorter text)
        self.lbl_rec_status = QLabel("REC: STOP")
        self.lbl_rec_status.setStyleSheet("color: #94a3b8; font-weight: bold; font-size: 10px;")
        layout.addWidget(self.lbl_rec_status)

        # Controls Row
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
            # Start recording
            self.is_recording = True
            self.recording_data = []
            self.recording_start_time = time.time()
            self.recording_stm_start = None  # Reset STM32 time reference

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
            # Stop recording and save
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

            # Save to CSV
            if self.recording_data:
                self._save_recording()

    def _save_recording(self):
        # Generate default filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"nagranie_{timestamp}.csv"

        # Open file dialog
        filepath, _ = QFileDialog.getSaveFileName(self, "Zapisz nagranie", default_filename, "CSV Files (*.csv);;All Files (*)")

        if filepath:
            try:
                with open(filepath, "w", newline="", encoding="utf-8") as f:
                    fieldnames = ["time", "distance", "filtered", "setpoint", "error", "control", "beam_angle"]
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
            if port:
                # Extract device name if format is "COMx - Desc"
                # Assuming serial_manager expects just COMx or handles full string?
                # serial_manager uses logic: `self.serial = serial.Serial(port_value, ...)`
                # list_ports returns just device name usually?
                # Let's check serial_manager: `ports = [p.device for p in serial.tools.list_ports.comports()]`
                # So it returns "COM3", "COM4".
                # BUT wait, the user said "neuich lista comow niech ma tez tytylu urzadzen".
                # I need to modify serial_manager to return formatted strings or handle the split here.
                # Let's modify serial_manager list_ports first? Or just do it here if possible.
                # serial_manager just returns `p.device`.
                # I should upgrade serial_manager to return full desc.

                # For now, pass what we have.
                self.serial.connect_serial(port.split(" - ")[0])
        else:
            self.serial.disconnect_serial()

    def _on_connected_changed(self, connected):
        if connected:
            self.btn_connect.setText("Rozłącz")
            self.btn_connect.setObjectName("disconnectBtn")
            self.lbl_status.setText("POŁĄCZONO")
            self.lbl_status.setStyleSheet("color: #4ade80; font-weight: bold; font-size: 10px;")
            self.combo_ports.setEnabled(False)
            self.btn_refresh.setEnabled(False)
            self.btn_connect.setStyle(self.btn_connect.style())

            # Reset STM32 state on connect
            QThread.msleep(100)
            self.serial.send_setpoint(125)
            self.serial.send_command("L:100.0")  # Center Servo (Angle)

            # Enable Test Tab controls
            self.btn_center_servo.setEnabled(True)
            self.btn_test_start.setEnabled(True)
            for btn in self.angle_preset_btns:
                btn.setEnabled(True)
            for btn in self.sampling_preset_btns:
                btn.setEnabled(True)
            self.btn_angle_send.setEnabled(True)
            self.btn_sampling_send.setEnabled(True)
        else:
            self.btn_connect.setText("Połącz")
            self.btn_connect.setObjectName("connectBtn")
            self.lbl_status.setText("ROZŁĄCZONY")
            self.lbl_status.setStyleSheet("color: #ef4444; font-weight: bold; font-size: 10px;")
            self.combo_ports.setEnabled(True)
            self.btn_refresh.setEnabled(True)
            self.btn_connect.setStyle(self.btn_connect.style())

            # Disable Test Tab controls
            self.btn_center_servo.setEnabled(False)
            self.btn_test_start.setEnabled(False)
            for btn in self.angle_preset_btns:
                btn.setEnabled(False)
            for btn in self.sampling_preset_btns:
                btn.setEnabled(False)
            self.btn_angle_send.setEnabled(False)
            self.btn_sampling_send.setEnabled(False)

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

            record = {
                "time": round(relative_time, 4),
                "distance": data.get("distance", 0),
                "filtered": data.get("filtered", 0),
                "setpoint": data.get("setpoint", 0),
                "error": data.get("error", 0),
                "control": data.get("control", 0),
                "beam_angle": data.get("beam_angle", 0),
            }
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

            test_record = {
                "time": round(relative_time, 4),
                "distance": data.get("distance", 0),
                "filtered": data.get("filtered", 0),
                "setpoint": data.get("setpoint", 0),
                "error": data.get("error", 0),
                "control": data.get("control", 0),
                "beam_angle": data.get("beam_angle", 0),
            }
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

        # Charts (main tab)
        self.charts_panel.update_charts(self.data_history)

        # Update Test Identification tab charts (always, not just when recording)
        self._update_test_charts()

    def _update_test_charts(self):
        """Update charts on Test Identification tab"""
        if not self.data_history:
            return

        view_data = self.data_history[-200:]

        dists = [d.get("distance", 0) for d in view_data]
        filts = [d.get("filtered", 0) for d in view_data]
        sets = [d.get("setpoint", 0) for d in view_data]
        errs = [d.get("error", 0) for d in view_data]
        ctrls = [d.get("control", 0) for d in view_data]
        beams = [d.get("beam_angle", 0) * 10 for d in view_data]  # Scale for visibility

        # Update all curves
        self.csv_curve_dist.setData(dists)
        self.csv_curve_filt.setData(filts)
        self.csv_curve_setpoint.setData(sets)
        self.csv_curve_error.setData(errs)
        self.csv_curve_ctrl.setData(ctrls)
        self.csv_curve_beam.setData(beams)

        # Update sample count if test is running
        if self.test_recording:
            self.lbl_test_samples.setText(f"Próbki: {len(self.test_data)}")

    def _update_test_chart_visibility(self):
        """Update chart visibility based on checkboxes"""
        # Distance chart
        self.csv_curve_dist.setVisible(self.chk_show_distance.isChecked())
        self.csv_curve_filt.setVisible(self.chk_show_distance.isChecked())

        # Setpoint
        self.csv_curve_setpoint.setVisible(self.chk_show_setpoint.isChecked())

        # Error chart
        self.csv_plot_error.setVisible(self.chk_show_error.isChecked())

        # Control chart
        self.csv_curve_ctrl.setVisible(self.chk_show_control.isChecked())

        # Beam angle
        self.csv_curve_beam.setVisible(self.chk_show_beam.isChecked())

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
                    fieldnames = ["time", "distance", "filtered", "setpoint", "error", "control", "beam_angle"]
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
        self.csv_curve_setpoint.setData([])
        self.csv_curve_error.setData([])
        self.csv_curve_ctrl.setData([])
        self.csv_curve_beam.setData([])

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
