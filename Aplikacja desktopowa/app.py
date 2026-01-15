import sys
import time
import csv
import os
from datetime import datetime
from PySide6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QSplitter, QFileDialog, QTabWidget
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QIcon

from serial_manager import SerialManager
from widgets.metrics_panel import MetricsPanel
from widgets.control_panel import ControlPanel
from widgets.charts_panel import ChartsPanel
from widgets.terminal import Terminal
from utils.metrics import calculate_metrics

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
        
        # 1. Top Row: Recording Tile + Metrics (connection moved to global bar)
        self.top_row_layout = QHBoxLayout()
        self.top_row_layout.setSpacing(8)
        
        # 1.1 Recording Tile
        self.rec_tile = self._create_recording_tile()
        self.top_row_layout.addWidget(self.rec_tile)
        
        # 1.2 Metrics Panel
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
        
        # Add first tab
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
        self.control_panel.derivative_mode_update.connect(self.serial.send_derivative_mode)
        
        # Connection Tile Interactions
        self.btn_refresh.clicked.connect(self.serial.list_ports)
        self.btn_connect.clicked.connect(self._toggle_connection)
        
        # Timer for chart updates (throttled 10 FPS)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_ui_tick)
        self.update_timer.start(100)
        
        # Initial port list
        self.serial.list_ports()

        # Bezposrednie wysylanie setpointu
        self.control_panel.setpoint_update.disconnect()
        self.control_panel.setpoint_update.connect(self._on_setpoint_change)

    def _on_setpoint_change(self, val):
        self.serial.send_setpoint(val)




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
        self.combo_ports.setStyleSheet("""
            QComboBox { 
                background-color: rgba(255,255,255,0.05); 
                border: 1px solid #444; 
                padding: 2px; 
                color: white;
                font-size: 10px;
            }
            QComboBox:disabled { color: #555; border-color: #222; }
        """)
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
        self.led_red.setStyleSheet("background-color: #ef4444; border-radius: 8px;")  # Active (stop state)
        layout.addWidget(self.led_red)
        
        self.led_green = QLabel()
        self.led_green.setFixedSize(16, 16)
        self.led_green.setStyleSheet("background-color: #1a3a1a; border-radius: 8px;")  # Dim (inactive)
        layout.addWidget(self.led_green)
        
        # Start/Stop Button
        self.btn_regulator = QPushButton("START")
        self.btn_regulator.setObjectName("startBtn")
        self.btn_regulator.setFixedSize(60, 26)
        self.btn_regulator.setStyleSheet("""
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
        """)
        self.btn_regulator.clicked.connect(self._toggle_regulator)
        layout.addWidget(self.btn_regulator)
        
        # Track regulator state
        self.regulator_running = False
        
        return tile

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
        
        # Header
        header = QLabel("TEST IDENTYFIKACJI")
        header.setStyleSheet("color: #94a3b8; font-weight: bold; font-size: 12px;")
        left_layout.addWidget(header)
        
        # Info
        info_label = QLabel("Sygnał sterujący wgrany na STM32.\nPrzycisk START uruchamia test.")
        info_label.setStyleSheet("color: #666; font-size: 10px;")
        info_label.setWordWrap(True)
        left_layout.addWidget(info_label)
        
        # Start Test Button
        self.btn_test_start = QPushButton("▶ START TEST")
        self.btn_test_start.setFixedHeight(40)
        self.btn_test_start.setStyleSheet("""
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
        """)
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
        self.btn_test_export = QPushButton("💾 EKSPORT CSV")
        self.btn_test_export.setFixedHeight(35)
        self.btn_test_export.setStyleSheet("""
            QPushButton { 
                background-color: #3b82f6; 
                color: white; 
                border: none; 
                border-radius: 4px;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover { background-color: #2563eb; }
        """)
        self.btn_test_export.clicked.connect(self._export_test_data)
        left_layout.addWidget(self.btn_test_export)
        
        # Clear Button
        self.btn_test_clear = QPushButton("🗑 WYCZYŚĆ")
        self.btn_test_clear.setFixedHeight(30)
        self.btn_test_clear.setStyleSheet("""
            QPushButton { 
                background-color: #64748b; 
                color: white; 
                border: none; 
                border-radius: 4px;
                font-size: 11px;
            }
            QPushButton:hover { background-color: #475569; }
        """)
        self.btn_test_clear.clicked.connect(self._clear_test_data)
        left_layout.addWidget(self.btn_test_clear)
        
        left_layout.addStretch()
        
        # --- Right Panel: Charts ---
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)
        
        # Header
        chart_header = QLabel("WYKRESY W CZASIE RZECZYWISTYM")
        chart_header.setStyleSheet("color: #94a3b8; font-weight: bold; font-size: 12px; margin-left: 5px;")
        right_layout.addWidget(chart_header)
        
        # PyQtGraph setup
        pg.setConfigOption('background', '#1e293b')
        pg.setConfigOption('foreground', '#94a3b8')
        pg.setConfigOptions(antialias=True)
        
        # Chart 1: Distance
        self.csv_plot_dist = pg.PlotWidget()
        self.csv_plot_dist.showGrid(x=False, y=True, alpha=0.3)
        self.csv_plot_dist.setYRange(0, 260, padding=0)
        self.csv_plot_dist.setMouseEnabled(x=False, y=False)
        self.csv_plot_dist.setTitle("Odległość [mm]", color="#94a3b8", size="10pt")
        
        self.csv_curve_dist = self.csv_plot_dist.plot(pen=pg.mkPen(color='#22c55e', width=2))
        self.csv_curve_filt = self.csv_plot_dist.plot(pen=pg.mkPen(color='#3b82f6', width=2))
        
        right_layout.addWidget(self.csv_plot_dist, stretch=1)
        
        # Chart 2: Servo Angle
        self.csv_plot_angle = pg.PlotWidget()
        self.csv_plot_angle.showGrid(x=False, y=True, alpha=0.3)
        self.csv_plot_angle.setMouseEnabled(x=False, y=True)
        self.csv_plot_angle.setTitle("Kąt Serwa [°]", color="#94a3b8", size="10pt")
        
        self.csv_curve_angle = self.csv_plot_angle.plot(pen=pg.mkPen(color='#f39c12', width=2))
        
        right_layout.addWidget(self.csv_plot_angle, stretch=1)
        
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
        self.btn_rec_toggle.setStyleSheet("""
            QPushButton { 
                background-color: #22c55e; 
                color: white; 
                border: none; 
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #16a34a; }
        """)
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
            
            self.btn_rec_toggle.setText("⏹")
            self.btn_rec_toggle.setStyleSheet("""
                QPushButton { 
                    background-color: #ef4444; 
                    color: white; 
                    border: none; 
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #dc2626; }
            """)
            self.lbl_rec_status.setText("REC: ●")
            self.lbl_rec_status.setStyleSheet("color: #ef4444; font-weight: bold; font-size: 10px;")
            self.lbl_rec_samples.setText("0")
        else:
            # Stop recording and save
            self.is_recording = False
            
            self.btn_rec_toggle.setText("▶")
            self.btn_rec_toggle.setStyleSheet("""
                QPushButton { 
                    background-color: #22c55e; 
                    color: white; 
                    border: none; 
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #16a34a; }
            """)
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
        filepath, _ = QFileDialog.getSaveFileName(
            self,
            "Zapisz nagranie",
            default_filename,
            "CSV Files (*.csv);;All Files (*)"
        )
        
        if filepath:
            try:
                with open(filepath, 'w', newline='', encoding='utf-8') as f:
                    fieldnames = ['time', 'distance', 'filtered', 'setpoint', 'error', 'control']
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
        if not self.serial.thread: # only if not connected
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
            self.btn_regulator.setStyleSheet("""
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
            """)
            # LEDs: green ON, red OFF
            self.led_green.setStyleSheet("background-color: #4ade80; border-radius: 8px;")
            self.led_red.setStyleSheet("background-color: #3a1a1a; border-radius: 8px;")
        else:
            # Stop regulator
            self.regulator_running = False
            self.serial.send_regulator_state(0)
            
            self.btn_regulator.setText("START")
            self.btn_regulator.setStyleSheet("""
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
            """)
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
                # I will quick-fix serial_manager to return "COMx - Desc" in next step or assume user is OK with COMx for now 
                # OR better: The user asked "niech w dropdownie automatycznie bedzie wybrany ten co ma w tytule stm".
                # This implies the dropdown DOES contain titles.
                # Currently serial_manager returns only `p.device`.
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
        else:
            self.btn_connect.setText("Połącz")
            self.btn_connect.setObjectName("connectBtn")
            self.lbl_status.setText("ROZŁĄCZONY")
            self.lbl_status.setStyleSheet("color: #ef4444; font-weight: bold; font-size: 10px;")
            self.combo_ports.setEnabled(True)
            self.btn_refresh.setEnabled(True)
            
    def _on_new_data(self, data):
        # Always append to history for charting
        # Note: serial_manager emits existing dict reference but updated values, 
        # so we should copy it for history to avoid mutations affecting old history
        snapshot = data.copy()
        current_time = time.time()
        snapshot['timestamp'] = current_time
        
        self.data_history.append(snapshot)
        if len(self.data_history) > 1000: # Keep more history in desktop
            self.data_history.pop(0)
        
        # Recording logic - collect data with relative timestamp
        if self.is_recording:
            relative_time = current_time - self.recording_start_time
            record = {
                'time': round(relative_time, 4),
                'distance': data.get('distance', 0),
                'filtered': data.get('filtered', 0),
                'setpoint': data.get('setpoint', 0),
                'error': data.get('error', 0),
                'control': data.get('control', 0)
            }
            self.recording_data.append(record)
            
            # Update sample count in UI
            self.lbl_rec_samples.setText(str(len(self.recording_data)))
            
        # Update Control Panel immediate values
        self.control_panel.update_data(data)
        
        # Collect data for Test Identification tab if recording
        if hasattr(self, 'test_recording') and self.test_recording:
            relative_time = current_time - self.test_start_time
            test_record = {
                'time': round(relative_time, 4),
                'distance': data.get('distance', 0),
                'filtered': data.get('filtered', 0),
                'control': data.get('control', 0),
                'setpoint': data.get('setpoint', 0)
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
        
        # Charts
        self.charts_panel.update_charts(self.data_history)
        
        # Update Test Identification tab charts if recording
        if hasattr(self, 'test_recording') and self.test_recording:
            self._update_test_charts()
    
    def _update_test_charts(self):
        """Update charts on Test Identification tab"""
        if not self.test_data:
            return
            
        view_data = self.test_data[-200:]
        
        dists = [d.get('distance', 0) for d in view_data]
        filts = [d.get('filtered', 0) for d in view_data]
        angles = [d.get('control', 0) for d in view_data]
        
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
            
            # Send start command to STM32
            self.serial.send_command("TEST:START")
            
            self.btn_test_start.setText("⏹ STOP TEST")
            self.btn_test_start.setStyleSheet("""
                QPushButton { 
                    background-color: #ef4444; 
                    color: white; 
                    border: none; 
                    border-radius: 4px;
                    font-weight: bold;
                    font-size: 14px;
                }
                QPushButton:hover { background-color: #dc2626; }
            """)
            self.lbl_test_status.setText("Status: Nagrywanie...")
            self.lbl_test_status.setStyleSheet("color: #22c55e; font-size: 11px;")
        else:
            # Stop test
            self.test_recording = False
            
            # Send stop command to STM32
            self.serial.send_command("TEST:STOP")
            
            self.btn_test_start.setText("▶ START TEST")
            self.btn_test_start.setStyleSheet("""
                QPushButton { 
                    background-color: #22c55e; 
                    color: white; 
                    border: none; 
                    border-radius: 4px;
                    font-weight: bold;
                    font-size: 14px;
                }
                QPushButton:hover { background-color: #16a34a; }
            """)
            self.lbl_test_status.setText(f"Status: Zakończono ({len(self.test_data)} próbek)")
            self.lbl_test_status.setStyleSheet("color: #94a3b8; font-size: 11px;")
    
    def _export_test_data(self):
        """Export recorded test data to CSV"""
        if not self.test_data:
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"test_identyfikacji_{timestamp}.csv"
        
        filepath, _ = QFileDialog.getSaveFileName(
            self,
            "Zapisz dane testu",
            default_filename,
            "CSV Files (*.csv);;All Files (*)"
        )
        
        if filepath:
            try:
                with open(filepath, 'w', newline='', encoding='utf-8') as f:
                    fieldnames = ['time', 'distance', 'filtered', 'control', 'setpoint']
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
        self.btn_test_start.setStyleSheet("""
            QPushButton { 
                background-color: #22c55e; 
                color: white; 
                border: none; 
                border-radius: 4px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover { background-color: #16a34a; }
        """)
