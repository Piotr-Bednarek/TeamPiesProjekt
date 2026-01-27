import cv2
import json
import os
import numpy as np
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSlider, QGroupBox, QGridLayout, QTabWidget, QMessageBox, QTextEdit, QCheckBox, QComboBox
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap, QTextCursor
from datetime import datetime


class OpenCVPanel(QWidget):
    ball_position_update = Signal(int, int)  # Signal for ball position (x, y)
    aruco_markers_update = Signal(object)  # Signal for ArUco markers {id: (x, y)}
    ball_on_beam_update = Signal(float)  # Signal for ball position on beam (0.0 to 1.0, -1 if not detected)
    beam_angle_update = Signal(float)  # Signal for angle between beams in degrees

    def __init__(self):
        super().__init__()

        # Camera variables
        self.cap = None
        self.camera_active = False
        self.current_fps = 0.0
        self.fps_counter = 0
        self.fps_start_time = 0

        # Camera settings (fixed 640x480 @ 30 FPS)
        self.camera_width = 640
        self.camera_height = 480
        self.camera_fps = 30

        # Cached camera settings for faster reconnection
        self._last_camera_index = None
        self._last_camera_backend = None

        # Ball detection parameters (HSV)
        self.h_min = 12
        self.h_max = 25
        self.s_min = 40
        self.s_max = 120
        self.v_min = 80
        self.v_max = 255
        self.blur_size = 9
        self.min_area = 100

        # ArUco detection parameters
        self.aruco_min_area = 500  # minimum marker area in px^2
        self.aruco_adaptive_const = 5
        self.aruco_min_size = 0.01

        # Enhanced ArUco detection parameters
        self.aruco_upscale_enabled = True  # Enable super-resolution upscaling
        self.aruco_upscale_factor = 1.5  # Upscaling factor (1.0 = no scaling, 2.0 = double)
        self.aruco_sharpen_enabled = True  # Enable sharpening kernel
        self.aruco_apriltag_refine = True  # Use APRILTAG refinement method (more robust)

        # Sharpening kernel (emphasizes edges)
        self._sharpen_kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]], dtype=np.float32)

        # ArUco display parameters (only for visualization, not detection)
        self.aruco_brightness = 0  # -50 to +50
        self.aruco_contrast = 1.0  # 0.5 to 2.0
        self.aruco_gamma = 1.5  # 0.5 to 3.0
        self._gamma_lut = None  # Cached gamma LUT table
        self._update_gamma_lut()

        # Calibration for ball position (to correct perspective)
        self.calib_min = 0.0  # raw value when ball is at 0mm
        self.calib_max = 1.0  # raw value when ball is at 250mm
        self.beam_length_mm = 250  # beam length in millimeters

        # Angle offset calibration
        self.angle_offset = 0.0  # offset to add to measured angle (degrees)
        self.angle_method = 0  # 0 = relative to horizontal, 1 = relative to reference line (ID1-ID3)

        # Ball position
        self.ball_x = 0
        self.ball_y = 0
        self.ball_detected = False
        self._last_ball_state = False  # For logging state changes

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshConstant = 5
        self.aruco_params.minMarkerLengthRatioOriginalImg = 0.01
        self.aruco_params.detectInvertedMarker = True

        # Advanced detection parameters - RELAXED for better small marker detection
        # Try APRILTAG refinement first (more robust for small/blurry markers)
        try:
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        except AttributeError:
            # Fallback for older OpenCV versions
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.errorCorrectionRate = 0.6

        # RELAXED parameters for distorted/small markers
        self.aruco_params.polygonalApproxAccuracyRate = 0.08  # Default 0.03, allow more distorted shapes
        self.aruco_params.minMarkerPerimeterRate = 0.015  # Very low for small markers
        self.aruco_params.maxMarkerPerimeterRate = 4.0  # Default
        self.aruco_params.minCornerDistanceRate = 0.03  # Reduced from 0.05
        self.aruco_params.minDistanceToBorder = 2  # Reduced from 3

        # Perspective correction for small markers
        self.aruco_params.perspectiveRemovePixelPerCell = 10  # Default 4, more pixels for analysis
        self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.15  # Margin of error inside marker

        # Store advanced params for UI
        self.aruco_error_correction = 0.6
        self.aruco_corner_refine = True
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Current view mode (0 = ball, 1 = aruco, 2 = measurements)
        self.current_tab = 0

        # Storage for ArUco markers (shared between tabs)
        self.aruco_centers = {}
        self._held_aruco_centers = {}  # Held markers when temporarily lost
        self._aruco_hold_frames = {}  # Counter for how many frames marker is missing
        self._aruco_hold_max = 15  # Max frames to hold marker position
        self._last_aruco_ids = set()  # For logging state changes

        # Measurement results
        self.ball_position_on_beam = -1.0  # 0.0 to 1.0, -1 if not detected
        self.ball_position_mm = -1.0  # Position in millimeters
        self.beam_angle = 0.0  # Angle between beams in degrees

        self._setup_ui()

        # Timer for camera feed
        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self._process_frame)

        # Auto-load parameters on startup
        self._auto_load_parameters()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Top control panel
        top_layout = QHBoxLayout()

        # Camera controls
        cam_group = QGroupBox("Sterowanie kamerą")
        cam_layout = QHBoxLayout(cam_group)

        # Camera selection dropdown
        cam_layout.addWidget(QLabel("Kamera:"))
        self.combo_camera = QComboBox()
        self.combo_camera.setFixedWidth(120)
        self.combo_camera.addItem("Auto", -1)
        self.combo_camera.addItem("Kamera 0", 0)
        self.combo_camera.addItem("Kamera 1", 1)
        self.combo_camera.addItem("Kamera 2", 2)
        self.combo_camera.addItem("Kamera 3", 3)
        self.combo_camera.setCurrentIndex(0)  # Auto by default
        cam_layout.addWidget(self.combo_camera)

        self.btn_start_camera = QPushButton("Start Camera")
        self.btn_start_camera.setFixedHeight(32)
        self.btn_start_camera.clicked.connect(self._toggle_camera)
        cam_layout.addWidget(self.btn_start_camera)

        self.lbl_camera_status = QLabel("Camera: OFF")
        self.lbl_camera_status.setStyleSheet("color: #ef4444; font-weight: bold;")
        cam_layout.addWidget(self.lbl_camera_status)

        self.lbl_fps = QLabel("FPS: 0.0")
        self.lbl_fps.setStyleSheet("color: #fbbf24;")
        cam_layout.addWidget(self.lbl_fps)

        self.lbl_ball_pos = QLabel("Ball: Not detected")
        self.lbl_ball_pos.setStyleSheet("color: #60a5fa;")
        cam_layout.addWidget(self.lbl_ball_pos)

        self.lbl_aruco_status = QLabel("ArUco: 0 markers")
        self.lbl_aruco_status.setStyleSheet("color: #a78bfa;")
        cam_layout.addWidget(self.lbl_aruco_status)

        cam_layout.addStretch()
        top_layout.addWidget(cam_group)

        layout.addLayout(top_layout)

        # Main content area with video and tabs
        content_layout = QHBoxLayout()

        # Left side: Video display
        video_layout = QVBoxLayout()

        self.lbl_video = QLabel("Obraz z kamery pojawi się tutaj")
        self.lbl_video.setMinimumSize(640, 480)
        self.lbl_video.setAlignment(Qt.AlignCenter)
        self.lbl_video.setStyleSheet("background-color: #1e1e1e; border: 2px solid #444;")
        video_layout.addWidget(self.lbl_video)

        content_layout.addLayout(video_layout, stretch=3)

        # Right side: Tab widget + global log
        right_layout = QVBoxLayout()

        self.tab_widget = QTabWidget()
        self.tab_widget.currentChanged.connect(self._on_tab_changed)

        # Tab 1: Ball detection (HSV)
        ball_tab = self._create_ball_tab()
        self.tab_widget.addTab(ball_tab, "Detekcja Pileczki")

        # Tab 2: ArUco detection
        aruco_tab = self._create_aruco_tab()
        self.tab_widget.addTab(aruco_tab, "Detekcja ArUco")

        # Tab 3: Combined measurements
        measurements_tab = self._create_measurements_tab()
        self.tab_widget.addTab(measurements_tab, "Pomiary")

        right_layout.addWidget(self.tab_widget, stretch=3)

        # Global log panel at the bottom
        global_log_group = QGroupBox("Log systemowy")
        global_log_layout = QVBoxLayout(global_log_group)
        global_log_layout.setContentsMargins(5, 5, 5, 5)
        self.global_log = QTextEdit()
        self.global_log.setReadOnly(True)
        self.global_log.setMinimumHeight(100)
        self.global_log.setMaximumHeight(150)
        self.global_log.setStyleSheet("background-color: #0d0d0d; color: #22c55e; font-family: 'Consolas', monospace; font-size: 11px;")
        global_log_layout.addWidget(self.global_log)
        right_layout.addWidget(global_log_group)

        content_layout.addLayout(right_layout, stretch=1)

        layout.addLayout(content_layout)

    def _create_ball_tab(self):
        """Create the ball detection tab with HSV controls"""
        tab = QWidget()
        sliders_layout = QVBoxLayout(tab)

        # HSV Range sliders
        hsv_group = QGroupBox("Zakres HSV (detekcja koloru)")
        hsv_layout = QGridLayout(hsv_group)

        # H Min/Max
        hsv_layout.addWidget(QLabel("H Min:"), 0, 0)
        self.slider_h_min = self._create_slider(0, 180, self.h_min)
        self.slider_h_min.valueChanged.connect(self._on_h_min_changed)
        hsv_layout.addWidget(self.slider_h_min, 0, 1)
        self.lbl_h_min = QLabel(str(self.h_min))
        hsv_layout.addWidget(self.lbl_h_min, 0, 2)

        hsv_layout.addWidget(QLabel("H Max:"), 1, 0)
        self.slider_h_max = self._create_slider(0, 180, self.h_max)
        self.slider_h_max.valueChanged.connect(self._on_h_max_changed)
        hsv_layout.addWidget(self.slider_h_max, 1, 1)
        self.lbl_h_max = QLabel(str(self.h_max))
        hsv_layout.addWidget(self.lbl_h_max, 1, 2)

        # S Min/Max
        hsv_layout.addWidget(QLabel("S Min:"), 2, 0)
        self.slider_s_min = self._create_slider(0, 255, self.s_min)
        self.slider_s_min.valueChanged.connect(self._on_s_min_changed)
        hsv_layout.addWidget(self.slider_s_min, 2, 1)
        self.lbl_s_min = QLabel(str(self.s_min))
        hsv_layout.addWidget(self.lbl_s_min, 2, 2)

        hsv_layout.addWidget(QLabel("S Max:"), 3, 0)
        self.slider_s_max = self._create_slider(0, 255, self.s_max)
        self.slider_s_max.valueChanged.connect(self._on_s_max_changed)
        hsv_layout.addWidget(self.slider_s_max, 3, 1)
        self.lbl_s_max = QLabel(str(self.s_max))
        hsv_layout.addWidget(self.lbl_s_max, 3, 2)

        # V Min/Max
        hsv_layout.addWidget(QLabel("V Min:"), 4, 0)
        self.slider_v_min = self._create_slider(0, 255, self.v_min)
        self.slider_v_min.valueChanged.connect(self._on_v_min_changed)
        hsv_layout.addWidget(self.slider_v_min, 4, 1)
        self.lbl_v_min = QLabel(str(self.v_min))
        hsv_layout.addWidget(self.lbl_v_min, 4, 2)

        hsv_layout.addWidget(QLabel("V Max:"), 5, 0)
        self.slider_v_max = self._create_slider(0, 255, self.v_max)
        self.slider_v_max.valueChanged.connect(self._on_v_max_changed)
        hsv_layout.addWidget(self.slider_v_max, 5, 1)
        self.lbl_v_max = QLabel(str(self.v_max))
        hsv_layout.addWidget(self.lbl_v_max, 5, 2)

        sliders_layout.addWidget(hsv_group)

        # Processing parameters
        proc_group = QGroupBox("Parametry przetwarzania")
        proc_layout = QGridLayout(proc_group)

        proc_layout.addWidget(QLabel("Blur Size:"), 0, 0)
        self.slider_blur = self._create_slider(1, 21, self.blur_size, step=2)
        self.slider_blur.valueChanged.connect(self._on_blur_changed)
        proc_layout.addWidget(self.slider_blur, 0, 1)
        self.lbl_blur = QLabel(str(self.blur_size))
        proc_layout.addWidget(self.lbl_blur, 0, 2)

        proc_layout.addWidget(QLabel("Min Area:"), 1, 0)
        self.slider_min_area = self._create_slider(10, 3000, self.min_area)
        self.slider_min_area.valueChanged.connect(self._on_min_area_changed)
        proc_layout.addWidget(self.slider_min_area, 1, 1)
        self.lbl_min_area = QLabel(str(self.min_area))
        proc_layout.addWidget(self.lbl_min_area, 1, 2)

        sliders_layout.addWidget(proc_group)

        # Save/Load buttons for ball detection
        btn_layout = QHBoxLayout()
        self.btn_save_ball = QPushButton("Zapisz parametry")
        self.btn_save_ball.clicked.connect(self._save_parameters)
        btn_layout.addWidget(self.btn_save_ball)
        self.btn_load_ball = QPushButton("Wczytaj parametry")
        self.btn_load_ball.clicked.connect(self._load_parameters)
        btn_layout.addWidget(self.btn_load_ball)
        sliders_layout.addLayout(btn_layout)

        # Info about min_area parameter
        info_label = QLabel("Min Area: minimalna powierzchnia konturu [px²] aby uznać za piłeczkę")
        info_label.setStyleSheet("color: #888; font-size: 10px; margin-top: 5px;")
        info_label.setWordWrap(True)
        sliders_layout.addWidget(info_label)

        sliders_layout.addStretch()

        return tab

    def _create_aruco_tab(self):
        """Create the ArUco detection tab with brightness/contrast/gamma controls"""
        tab = QWidget()
        sliders_layout = QVBoxLayout(tab)

        # ArUco parameters
        aruco_group = QGroupBox("Parametry detekcji ArUco")
        aruco_layout = QGridLayout(aruco_group)

        # Adaptive threshold constant
        aruco_layout.addWidget(QLabel("Adaptive Const:"), 0, 0)
        self.slider_aruco_adaptive = self._create_slider(1, 30, self.aruco_adaptive_const)
        self.slider_aruco_adaptive.valueChanged.connect(self._on_aruco_adaptive_changed)
        aruco_layout.addWidget(self.slider_aruco_adaptive, 0, 1)
        self.lbl_aruco_adaptive = QLabel(str(self.aruco_adaptive_const))
        aruco_layout.addWidget(self.lbl_aruco_adaptive, 0, 2)

        # Min marker size
        aruco_layout.addWidget(QLabel("Min Size (%):"), 1, 0)
        self.slider_aruco_min_size = self._create_slider(1, 100, int(self.aruco_min_size * 1000))
        self.slider_aruco_min_size.valueChanged.connect(self._on_aruco_min_size_changed)
        aruco_layout.addWidget(self.slider_aruco_min_size, 1, 1)
        self.lbl_aruco_min_size = QLabel(f"{self.aruco_min_size:.3f}")
        aruco_layout.addWidget(self.lbl_aruco_min_size, 1, 2)

        # Min area filter
        aruco_layout.addWidget(QLabel("Min Area (px²):"), 2, 0)
        self.slider_aruco_min_area = self._create_slider(0, 5000, self.aruco_min_area)
        self.slider_aruco_min_area.valueChanged.connect(self._on_aruco_min_area_changed)
        aruco_layout.addWidget(self.slider_aruco_min_area, 2, 1)
        self.lbl_aruco_min_area = QLabel(str(self.aruco_min_area))
        aruco_layout.addWidget(self.lbl_aruco_min_area, 2, 2)

        # Error correction rate
        aruco_layout.addWidget(QLabel("Error Correction:"), 3, 0)
        self.slider_error_correction = self._create_slider(0, 100, int(self.aruco_error_correction * 100))
        self.slider_error_correction.valueChanged.connect(self._on_error_correction_changed)
        aruco_layout.addWidget(self.slider_error_correction, 3, 1)
        self.lbl_error_correction = QLabel(f"{self.aruco_error_correction:.2f}")
        aruco_layout.addWidget(self.lbl_error_correction, 3, 2)

        # Corner refinement checkbox
        self.chk_corner_refine = QCheckBox("Subpixel Corner Refinement")
        self.chk_corner_refine.setChecked(self.aruco_corner_refine)
        self.chk_corner_refine.stateChanged.connect(self._on_corner_refine_changed)
        aruco_layout.addWidget(self.chk_corner_refine, 4, 0, 1, 3)

        sliders_layout.addWidget(aruco_group)

        # ENHANCED: Super-resolution and preprocessing options
        enhanced_group = QGroupBox("Ulepszona detekcja (Super-Resolution)")
        enhanced_layout = QGridLayout(enhanced_group)

        # Upscaling checkbox and slider
        self.chk_upscale = QCheckBox("Upscaling (powiększenie)")
        self.chk_upscale.setChecked(self.aruco_upscale_enabled)
        self.chk_upscale.stateChanged.connect(self._on_upscale_changed)
        self.chk_upscale.setToolTip("Powiększa obraz przed detekcją - lepsze wyniki dla małych markerów")
        enhanced_layout.addWidget(self.chk_upscale, 0, 0)

        enhanced_layout.addWidget(QLabel("Skala:"), 0, 1)
        self.slider_upscale_factor = self._create_slider(10, 30, int(self.aruco_upscale_factor * 10))
        self.slider_upscale_factor.valueChanged.connect(self._on_upscale_factor_changed)
        enhanced_layout.addWidget(self.slider_upscale_factor, 0, 2)
        self.lbl_upscale_factor = QLabel(f"{self.aruco_upscale_factor:.1f}x")
        enhanced_layout.addWidget(self.lbl_upscale_factor, 0, 3)

        self.chk_sharpen = QCheckBox("Wyostrzanie krawędzi")
        self.chk_sharpen.setChecked(self.aruco_sharpen_enabled)
        self.chk_sharpen.stateChanged.connect(self._on_sharpen_changed)
        self.chk_sharpen.setToolTip("Podkreśla krawędzie - lepsze wykrywanie ramek markerów")
        enhanced_layout.addWidget(self.chk_sharpen, 1, 0, 1, 4)

        self.chk_apriltag = QCheckBox("Metoda APRILTAG (robustna)")
        self.chk_apriltag.setChecked(self.aruco_apriltag_refine)
        self.chk_apriltag.stateChanged.connect(self._on_apriltag_changed)
        self.chk_apriltag.setToolTip("Bardziej odporna metoda doprecyzowywania narożników (wolniejsza)")
        enhanced_layout.addWidget(self.chk_apriltag, 2, 0, 1, 4)

        sliders_layout.addWidget(enhanced_group)

        display_group = QGroupBox("Przetwarzanie obrazu")
        display_layout = QGridLayout(display_group)

        display_layout.addWidget(QLabel("Jasnosc:"), 0, 0)
        self.slider_aruco_brightness = self._create_slider(-100, 100, self.aruco_brightness)
        self.slider_aruco_brightness.valueChanged.connect(self._on_aruco_brightness_changed)
        display_layout.addWidget(self.slider_aruco_brightness, 0, 1)
        self.lbl_aruco_brightness = QLabel(str(self.aruco_brightness))
        display_layout.addWidget(self.lbl_aruco_brightness, 0, 2)

        # Contrast
        display_layout.addWidget(QLabel("Kontrast:"), 1, 0)
        self.slider_aruco_contrast = self._create_slider(10, 300, int(self.aruco_contrast * 100))
        self.slider_aruco_contrast.valueChanged.connect(self._on_aruco_contrast_changed)
        display_layout.addWidget(self.slider_aruco_contrast, 1, 1)
        self.lbl_aruco_contrast = QLabel(f"{self.aruco_contrast:.2f}")
        display_layout.addWidget(self.lbl_aruco_contrast, 1, 2)

        # Gamma
        display_layout.addWidget(QLabel("Gamma:"), 2, 0)
        self.slider_aruco_gamma = self._create_slider(10, 300, int(self.aruco_gamma * 100))
        self.slider_aruco_gamma.valueChanged.connect(self._on_aruco_gamma_changed)
        display_layout.addWidget(self.slider_aruco_gamma, 2, 1)
        self.lbl_aruco_gamma = QLabel(f"{self.aruco_gamma:.2f}")
        display_layout.addWidget(self.lbl_aruco_gamma, 2, 2)

        sliders_layout.addWidget(display_group)

        # Save/Load buttons for ArUco detection
        btn_layout = QHBoxLayout()
        self.btn_save_aruco = QPushButton("Zapisz parametry")
        self.btn_save_aruco.clicked.connect(self._save_parameters)
        btn_layout.addWidget(self.btn_save_aruco)
        self.btn_load_aruco = QPushButton("Wczytaj parametry")
        self.btn_load_aruco.clicked.connect(self._load_parameters)
        btn_layout.addWidget(self.btn_load_aruco)
        sliders_layout.addLayout(btn_layout)

        # Log panel for ArUco detection
        log_group = QGroupBox("Log detekcji")
        log_layout = QVBoxLayout(log_group)
        log_layout.setContentsMargins(5, 5, 5, 5)
        self.aruco_log = QTextEdit()
        self.aruco_log.setReadOnly(True)
        self.aruco_log.setMinimumHeight(80)
        self.aruco_log.setMaximumHeight(100)
        self.aruco_log.setStyleSheet("background-color: #1a1a1a; color: #a78bfa; font-family: 'Consolas', monospace; font-size: 11px;")
        log_layout.addWidget(self.aruco_log)
        sliders_layout.addWidget(log_group)

        return tab

    def _create_measurements_tab(self):
        """Create the measurements tab combining ball and ArUco detection"""
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # Measurements display
        meas_group = QGroupBox("Wyniki pomiarow")
        meas_layout = QGridLayout(meas_group)

        # Ball position on beam (in mm)
        meas_layout.addWidget(QLabel("Pozycja pileczki na belce:"), 0, 0)
        self.lbl_ball_on_beam = QLabel("-- mm")
        self.lbl_ball_on_beam.setStyleSheet("font-weight: bold; font-size: 14px; color: #60a5fa;")
        meas_layout.addWidget(self.lbl_ball_on_beam, 0, 1)

        # Raw position (before calibration)
        meas_layout.addWidget(QLabel("Pozycja surowa:"), 1, 0)
        self.lbl_ball_raw_percent = QLabel("-- %")
        self.lbl_ball_raw_percent.setStyleSheet("color: #888;")
        meas_layout.addWidget(self.lbl_ball_raw_percent, 1, 1)

        # Beam angle
        meas_layout.addWidget(QLabel("Kat belki:"), 2, 0)
        self.lbl_beam_angle = QLabel("-- °")
        self.lbl_beam_angle.setStyleSheet("font-weight: bold; font-size: 14px; color: #fbbf24;")
        meas_layout.addWidget(self.lbl_beam_angle, 2, 1)

        # Ball raw position
        meas_layout.addWidget(QLabel("Pozycja pileczki (px):"), 3, 0)
        self.lbl_ball_raw = QLabel("(-, -)")
        self.lbl_ball_raw.setStyleSheet("color: #888;")
        meas_layout.addWidget(self.lbl_ball_raw, 3, 1)

        # ArUco markers status
        meas_layout.addWidget(QLabel("Znaczniki ArUco:"), 4, 0)
        self.lbl_aruco_markers = QLabel("Brak")
        self.lbl_aruco_markers.setStyleSheet("color: #888;")
        meas_layout.addWidget(self.lbl_aruco_markers, 4, 1)

        layout.addWidget(meas_group)

        # Calibration group
        calib_group = QGroupBox("Kalibracja pozycji (0-250mm)")
        calib_layout = QGridLayout(calib_group)

        # Calibration labels
        calib_layout.addWidget(QLabel("Min (0mm):"), 0, 0)
        self.lbl_calib_min = QLabel(f"{self.calib_min:.3f}")
        self.lbl_calib_min.setStyleSheet("font-weight: bold;")
        calib_layout.addWidget(self.lbl_calib_min, 0, 1)

        calib_layout.addWidget(QLabel("Max (250mm):"), 1, 0)
        self.lbl_calib_max = QLabel(f"{self.calib_max:.3f}")
        self.lbl_calib_max.setStyleSheet("font-weight: bold;")
        calib_layout.addWidget(self.lbl_calib_max, 1, 1)

        # Calibration buttons
        self.btn_calib_min = QPushButton("Ustaw 0mm (poczatek)")
        self.btn_calib_min.clicked.connect(self._calibrate_min)
        calib_layout.addWidget(self.btn_calib_min, 0, 2)

        self.btn_calib_max = QPushButton("Ustaw 250mm (koniec)")
        self.btn_calib_max.clicked.connect(self._calibrate_max)
        calib_layout.addWidget(self.btn_calib_max, 1, 2)

        self.btn_calib_reset = QPushButton("Reset kalibracji")
        self.btn_calib_reset.clicked.connect(self._calibrate_reset)
        calib_layout.addWidget(self.btn_calib_reset, 2, 0, 1, 3)

        layout.addWidget(calib_group)

        # Angle offset calibration group
        angle_group = QGroupBox("Kalibracja kata")
        angle_layout = QGridLayout(angle_group)

        # Angle measurement method
        angle_layout.addWidget(QLabel("Metoda pomiaru:"), 0, 0)
        from PySide6.QtWidgets import QComboBox

        self.combo_angle_method = QComboBox()
        self.combo_angle_method.addItem("Wzgledem poziomu obrazu (ID2-ID0)")
        self.combo_angle_method.addItem("Wzgledem linii ref. (ID1-ID3)")
        self.combo_angle_method.setCurrentIndex(self.angle_method)
        self.combo_angle_method.currentIndexChanged.connect(self._on_angle_method_changed)
        angle_layout.addWidget(self.combo_angle_method, 0, 1, 1, 2)

        # Angle offset slider
        angle_layout.addWidget(QLabel("Offset kata:"), 1, 0)
        self.slider_angle_offset = self._create_slider(-150, 150, int(self.angle_offset * 10))
        self.slider_angle_offset.valueChanged.connect(self._on_angle_offset_changed)
        angle_layout.addWidget(self.slider_angle_offset, 1, 1)
        self.lbl_angle_offset = QLabel(f"{self.angle_offset:.1f}°")
        self.lbl_angle_offset.setStyleSheet("font-weight: bold; min-width: 50px;")
        angle_layout.addWidget(self.lbl_angle_offset, 1, 2)

        # Button to set current angle as zero
        self.btn_angle_zero = QPushButton("Ustaw obecny kat jako 0°")
        self.btn_angle_zero.clicked.connect(self._calibrate_angle_zero)
        angle_layout.addWidget(self.btn_angle_zero, 2, 0, 1, 2)

        self.btn_angle_reset = QPushButton("Reset")
        self.btn_angle_reset.clicked.connect(self._calibrate_angle_reset)
        angle_layout.addWidget(self.btn_angle_reset, 2, 2)

        layout.addWidget(angle_group)

        layout.addStretch()

        return tab

    def _calibrate_min(self):
        """Set calibration minimum (0mm position)"""
        if hasattr(self, "_raw_ball_position") and self._raw_ball_position >= 0:
            self.calib_min = self._raw_ball_position
            self.lbl_calib_min.setText(f"{self.calib_min:.3f}")
            self._log_ball(f"Kalibracja 0mm ustawiona: {self.calib_min:.3f}")
            self._log_global(f"Kalibracja 0mm ustawiona: {self.calib_min:.3f}")

    def _calibrate_max(self):
        """Set calibration maximum (250mm position)"""
        if hasattr(self, "_raw_ball_position") and self._raw_ball_position >= 0:
            self.calib_max = self._raw_ball_position
            self.lbl_calib_max.setText(f"{self.calib_max:.3f}")
            self._log_ball(f"Kalibracja 250mm ustawiona: {self.calib_max:.3f}")
            self._log_global(f"Kalibracja 250mm ustawiona: {self.calib_max:.3f}")

    def _calibrate_reset(self):
        """Reset calibration to defaults"""
        self.calib_min = 0.0
        self.calib_max = 1.0
        self.lbl_calib_min.setText(f"{self.calib_min:.3f}")
        self.lbl_calib_max.setText(f"{self.calib_max:.3f}")
        self._log_ball("Kalibracja zresetowana")
        self._log_global("Kalibracja zresetowana")

    def _on_angle_offset_changed(self, val):
        """Handle angle offset slider change"""
        self.angle_offset = val / 10.0
        self.lbl_angle_offset.setText(f"{self.angle_offset:.1f}°")

    def _on_angle_method_changed(self, index):
        """Handle angle measurement method change"""
        self.angle_method = index
        method_name = "poziom obrazu" if index == 0 else "linia ref. ID1-ID3"
        self._log_global(f"Metoda pomiaru kata: {method_name}", "angle")

    def _calibrate_angle_zero(self):
        """Set current angle as zero (calculate offset)"""
        if hasattr(self, "_raw_beam_angle"):
            self.angle_offset = -self._raw_beam_angle
            self.slider_angle_offset.setValue(int(self.angle_offset * 10))
            self.lbl_angle_offset.setText(f"{self.angle_offset:.1f}°")
            self._log_global(f"Offset kata ustawiony: {self.angle_offset:.1f}°", "angle")

    def _calibrate_angle_reset(self):
        """Reset angle offset to zero"""
        self.angle_offset = 0.0
        self.slider_angle_offset.setValue(0)
        self.lbl_angle_offset.setText("0.0°")
        self._log_global("Offset kata zresetowany")

    def _apply_calibration(self, raw_value):
        """Apply calibration to convert raw position to calibrated position"""
        if self.calib_max == self.calib_min:
            return raw_value
        calibrated = (raw_value - self.calib_min) / (self.calib_max - self.calib_min)
        return max(0.0, min(1.0, calibrated))

    def _get_params_file_path(self):
        """Get the path to the parameters file"""
        return os.path.join(os.path.dirname(__file__), "..", "opencv_params.json")

    def _save_parameters(self):
        """Save all detection parameters to a JSON file"""
        params = {
            "ball_detection": {"h_min": self.h_min, "h_max": self.h_max, "s_min": self.s_min, "s_max": self.s_max, "v_min": self.v_min, "v_max": self.v_max, "blur_size": self.blur_size, "min_area": self.min_area},
            "aruco_detection": {
                "adaptive_const": self.aruco_adaptive_const,
                "min_size": self.aruco_min_size,
                "min_area": self.aruco_min_area,
                "brightness": self.aruco_brightness,
                "contrast": self.aruco_contrast,
                "gamma": self.aruco_gamma,
                "error_correction": self.aruco_error_correction,
                "corner_refine": self.aruco_corner_refine,
                # Enhanced detection params
                "upscale_enabled": self.aruco_upscale_enabled,
                "upscale_factor": self.aruco_upscale_factor,
                "sharpen_enabled": self.aruco_sharpen_enabled,
                "apriltag_refine": self.aruco_apriltag_refine,
            },
            "calibration": {"min": self.calib_min, "max": self.calib_max, "angle_offset": self.angle_offset, "angle_method": self.angle_method},
        }
        try:
            with open(self._get_params_file_path(), "w") as f:
                json.dump(params, f, indent=4)
            QMessageBox.information(self, "Sukces", "Parametry zostaly zapisane.")
        except Exception as e:
            QMessageBox.warning(self, "Blad", f"Nie udalo sie zapisac parametrow: {e}")

    def _load_parameters(self):
        """Load all detection parameters from a JSON file"""
        try:
            with open(self._get_params_file_path(), "r") as f:
                params = json.load(f)

            # Load ball detection params
            ball = params.get("ball_detection", {})
            if "h_min" in ball:
                self.slider_h_min.setValue(ball["h_min"])
            if "h_max" in ball:
                self.slider_h_max.setValue(ball["h_max"])
            if "s_min" in ball:
                self.slider_s_min.setValue(ball["s_min"])
            if "s_max" in ball:
                self.slider_s_max.setValue(ball["s_max"])
            if "v_min" in ball:
                self.slider_v_min.setValue(ball["v_min"])
            if "v_max" in ball:
                self.slider_v_max.setValue(ball["v_max"])
            if "blur_size" in ball:
                self.slider_blur.setValue(ball["blur_size"])
            if "min_area" in ball:
                self.slider_min_area.setValue(ball["min_area"])

            # Load ArUco detection params
            aruco = params.get("aruco_detection", {})
            if "adaptive_const" in aruco:
                self.slider_aruco_adaptive.setValue(aruco["adaptive_const"])
            if "min_size" in aruco:
                self.slider_aruco_min_size.setValue(int(aruco["min_size"] * 1000))
            if "min_area" in aruco:
                self.slider_aruco_min_area.setValue(aruco["min_area"])
            if "brightness" in aruco:
                self.slider_aruco_brightness.setValue(aruco["brightness"])
            if "contrast" in aruco:
                self.slider_aruco_contrast.setValue(int(aruco["contrast"] * 100))
            if "gamma" in aruco:
                self.slider_aruco_gamma.setValue(int(aruco["gamma"] * 100))
                self._update_gamma_lut()
            if "error_correction" in aruco:
                self.slider_error_correction.setValue(int(aruco["error_correction"] * 100))
            if "corner_refine" in aruco:
                self.chk_corner_refine.setChecked(aruco["corner_refine"])
            # Enhanced detection params
            if "upscale_enabled" in aruco:
                self.chk_upscale.setChecked(aruco["upscale_enabled"])
            if "upscale_factor" in aruco:
                self.slider_upscale_factor.setValue(int(aruco["upscale_factor"] * 10))
            if "sharpen_enabled" in aruco:
                self.chk_sharpen.setChecked(aruco["sharpen_enabled"])
            if "apriltag_refine" in aruco:
                self.chk_apriltag.setChecked(aruco["apriltag_refine"])

            # Load calibration params
            calib = params.get("calibration", {})
            if "min" in calib:
                self.calib_min = calib["min"]
                self.lbl_calib_min.setText(f"{self.calib_min:.3f}")
            if "max" in calib:
                self.calib_max = calib["max"]
                self.lbl_calib_max.setText(f"{self.calib_max:.3f}")
            if "angle_offset" in calib:
                self.angle_offset = calib["angle_offset"]
                self.slider_angle_offset.setValue(int(self.angle_offset * 10))
                self.lbl_angle_offset.setText(f"{self.angle_offset:.1f}°")
            if "angle_method" in calib:
                self.angle_method = calib["angle_method"]
                self.combo_angle_method.setCurrentIndex(self.angle_method)

            QMessageBox.information(self, "Sukces", "Parametry zostaly wczytane.")
        except FileNotFoundError:
            QMessageBox.warning(self, "Blad", "Plik z parametrami nie istnieje.")
        except Exception as e:
            QMessageBox.warning(self, "Blad", f"Nie udalo sie wczytac parametrow: {e}")

    def _on_tab_changed(self, index):
        """Handle tab change"""
        self.current_tab = index

    def _log_ball(self, message, level="info"):
        """Add a log message to the global log panel (ball detection)"""
        # Use global log with ball-specific color
        self._log_global(f"[BALL] {message}", "ball")

    def _log_aruco(self, message, level="info"):
        """Add a log message to the ArUco detection log panel"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        colors = {"info": "#a78bfa", "warn": "#fbbf24", "error": "#ef4444"}
        color = colors.get(level, "#a78bfa")
        self.aruco_log.append(f'<span style="color:#666">[{timestamp}]</span> <span style="color:{color}">{message}</span>')
        # Keep only last 50 lines
        if self.aruco_log.document().blockCount() > 50:
            cursor = self.aruco_log.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.Start)
            cursor.movePosition(QTextCursor.MoveOperation.Down, QTextCursor.MoveMode.KeepAnchor)
            cursor.removeSelectedText()

    def _log_global(self, message, level="info"):
        """Add a log message to the global terminal log panel"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        colors = {"info": "#10b981", "warn": "#fbbf24", "error": "#ef4444", "camera": "#60a5fa", "ball": "#f472b6", "aruco": "#a78bfa", "angle": "#fbbf24"}
        color = colors.get(level, "#10b981")
        self.global_log.append(f'<span style="color:#666">[{timestamp}]</span> <span style="color:{color}">{message}</span>')
        # Keep only last 100 lines
        if self.global_log.document().blockCount() > 100:
            cursor = self.global_log.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.Start)
            cursor.movePosition(QTextCursor.MoveOperation.Down, QTextCursor.MoveMode.KeepAnchor)
            cursor.removeSelectedText()

    def _auto_load_parameters(self):
        """Automatically load parameters from file at startup (silent)"""
        try:
            with open(self._get_params_file_path(), "r") as f:
                params = json.load(f)

            # Load ball detection params
            ball = params.get("ball_detection", {})
            if "h_min" in ball:
                self.h_min = ball["h_min"]
            if "h_max" in ball:
                self.h_max = ball["h_max"]
            if "s_min" in ball:
                self.s_min = ball["s_min"]
            if "s_max" in ball:
                self.s_max = ball["s_max"]
            if "v_min" in ball:
                self.v_min = ball["v_min"]
            if "v_max" in ball:
                self.v_max = ball["v_max"]
            if "blur_size" in ball:
                self.blur_size = ball["blur_size"]
            if "min_area" in ball:
                self.min_area = ball["min_area"]

            # Load ArUco detection params
            aruco = params.get("aruco_detection", {})
            if "adaptive_const" in aruco:
                self.aruco_adaptive_const = aruco["adaptive_const"]
                self.aruco_params.adaptiveThreshConstant = self.aruco_adaptive_const
            if "min_size" in aruco:
                self.aruco_min_size = aruco["min_size"]
                self.aruco_params.minMarkerLengthRatioOriginalImg = self.aruco_min_size
            if "min_area" in aruco:
                self.aruco_min_area = aruco["min_area"]
            if "error_correction" in aruco:
                self.aruco_error_correction = aruco["error_correction"]
                self.aruco_params.errorCorrectionRate = self.aruco_error_correction
            if "corner_refine" in aruco:
                self.aruco_corner_refine = aruco["corner_refine"]
                if self.aruco_corner_refine:
                    if self.aruco_apriltag_refine:
                        try:
                            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
                        except AttributeError:
                            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
                    else:
                        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
                else:
                    self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
            if "brightness" in aruco:
                self.aruco_brightness = aruco["brightness"]
            if "contrast" in aruco:
                self.aruco_contrast = aruco["contrast"]
            if "gamma" in aruco:
                self.aruco_gamma = aruco["gamma"]
                self._update_gamma_lut()
            # Enhanced detection params
            if "upscale_enabled" in aruco:
                self.aruco_upscale_enabled = aruco["upscale_enabled"]
            if "upscale_factor" in aruco:
                self.aruco_upscale_factor = aruco["upscale_factor"]
            if "sharpen_enabled" in aruco:
                self.aruco_sharpen_enabled = aruco["sharpen_enabled"]
            if "apriltag_refine" in aruco:
                self.aruco_apriltag_refine = aruco["apriltag_refine"]

            # Rebuild ArUco detector with loaded params
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

            # Load calibration params
            calib = params.get("calibration", {})
            if "min" in calib:
                self.calib_min = calib["min"]
            if "max" in calib:
                self.calib_max = calib["max"]
            if "angle_offset" in calib:
                self.angle_offset = calib["angle_offset"]
            if "angle_method" in calib:
                self.angle_method = calib["angle_method"]

            self._log_global("Parametry wczytane automatycznie z pliku", "info")
        except FileNotFoundError:
            self._log_global("Brak pliku parametrow - uzyto domyslnych", "warn")
        except Exception as e:
            self._log_global(f"Blad wczytywania parametrow: {e}", "error")

    def _create_slider(self, min_val, max_val, default_val, step=1):
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(min_val)
        slider.setMaximum(max_val)
        slider.setValue(default_val)
        slider.setSingleStep(step)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval((max_val - min_val) // 10 if (max_val - min_val) > 10 else 1)
        return slider

    # Ball detection slider callbacks
    def _on_h_min_changed(self, val):
        self.h_min = val
        self.lbl_h_min.setText(str(val))

    def _on_h_max_changed(self, val):
        self.h_max = val
        self.lbl_h_max.setText(str(val))

    def _on_s_min_changed(self, val):
        self.s_min = val
        self.lbl_s_min.setText(str(val))

    def _on_s_max_changed(self, val):
        self.s_max = val
        self.lbl_s_max.setText(str(val))

    def _on_v_min_changed(self, val):
        self.v_min = val
        self.lbl_v_min.setText(str(val))

    def _on_v_max_changed(self, val):
        self.v_max = val
        self.lbl_v_max.setText(str(val))

    def _on_blur_changed(self, val):
        # Ensure blur is odd
        if val % 2 == 0:
            val += 1
            self.slider_blur.setValue(val)
        self.blur_size = val
        self.lbl_blur.setText(str(val))

    def _on_min_area_changed(self, val):
        self.min_area = val
        self.lbl_min_area.setText(str(val))

    # ArUco detection slider callbacks
    def _on_aruco_adaptive_changed(self, val):
        self.aruco_adaptive_const = max(1, val)
        self.aruco_params.adaptiveThreshConstant = self.aruco_adaptive_const
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.lbl_aruco_adaptive.setText(str(val))

    def _on_aruco_min_size_changed(self, val):
        self.aruco_min_size = val / 1000.0
        self.aruco_params.minMarkerLengthRatioOriginalImg = self.aruco_min_size
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.lbl_aruco_min_size.setText(f"{self.aruco_min_size:.3f}")

    def _on_aruco_min_area_changed(self, val):
        self.aruco_min_area = val
        self.lbl_aruco_min_area.setText(str(val))

    def _on_error_correction_changed(self, val):
        self.aruco_error_correction = val / 100.0
        self.aruco_params.errorCorrectionRate = self.aruco_error_correction
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.lbl_error_correction.setText(f"{self.aruco_error_correction:.2f}")

    def _on_corner_refine_changed(self, state):
        self.aruco_corner_refine = state == 2  # Qt.Checked = 2
        if self.aruco_corner_refine:
            if self.aruco_apriltag_refine:
                try:
                    self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
                except AttributeError:
                    self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            else:
                self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        else:
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    # Enhanced ArUco detection callbacks
    def _on_upscale_changed(self, state):
        self.aruco_upscale_enabled = state == 2
        self._log_global(f"Upscaling: {'ON' if self.aruco_upscale_enabled else 'OFF'}", "aruco")

    def _on_upscale_factor_changed(self, val):
        self.aruco_upscale_factor = val / 10.0
        self.lbl_upscale_factor.setText(f"{self.aruco_upscale_factor:.1f}x")

    def _on_sharpen_changed(self, state):
        self.aruco_sharpen_enabled = state == 2
        self._log_global(f"Wyostrzanie: {'ON' if self.aruco_sharpen_enabled else 'OFF'}", "aruco")

    def _on_apriltag_changed(self, state):
        self.aruco_apriltag_refine = state == 2
        if self.aruco_corner_refine:
            if self.aruco_apriltag_refine:
                try:
                    self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
                    self._log_global("Metoda doprecyzowania: APRILTAG", "aruco")
                except AttributeError:
                    self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
                    self._log_global("APRILTAG niedostępny - używam SUBPIX", "warn")
            else:
                self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
                self._log_global("Metoda doprecyzowania: SUBPIX", "aruco")
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def _on_aruco_brightness_changed(self, val):
        self.aruco_brightness = val
        self.lbl_aruco_brightness.setText(str(val))

    def _on_aruco_contrast_changed(self, val):
        self.aruco_contrast = val / 100.0
        self.lbl_aruco_contrast.setText(f"{self.aruco_contrast:.2f}")

    def _on_aruco_gamma_changed(self, val):
        self.aruco_gamma = val / 100.0
        self.lbl_aruco_gamma.setText(f"{self.aruco_gamma:.2f}")
        self._update_gamma_lut()

    def _update_gamma_lut(self):
        """Update cached gamma LUT table"""
        if self.aruco_gamma != 1.0:
            inv_gamma = 1.0 / self.aruco_gamma
            self._gamma_lut = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)]).astype(np.uint8)
        else:
            self._gamma_lut = None

    def _toggle_camera(self):
        if self.camera_active:
            self._stop_camera()
        else:
            self._start_camera()

    def _start_camera(self):
        """Initialize and start the camera"""
        self._log_ball("Laczenie z kamera...")
        self._log_aruco("Laczenie z kamera...")
        self._log_global("Laczenie z kamera...", "camera")
        self.cap = self._find_camera()

        if self.cap is None:
            self.lbl_camera_status.setText("Camera: ERROR")
            self.lbl_camera_status.setStyleSheet("color: #ef4444; font-weight: bold;")
            self._log_ball("Nie znaleziono kamery!", "error")
            self._log_aruco("Nie znaleziono kamery!", "error")
            self._log_global("Nie znaleziono kamery!", "error")
            return

        self.camera_active = True
        self.fps_counter = 0
        self.fps_start_time = cv2.getTickCount() / cv2.getTickFrequency()

        self.btn_start_camera.setText("Stop Camera")
        self.lbl_camera_status.setText("Camera: ON")
        self.lbl_camera_status.setStyleSheet("color: #10b981; font-weight: bold;")

        cam_info = f"Kamera uruchomiona (idx:{self._last_camera_index})"
        self._log_ball(cam_info)
        self._log_aruco(cam_info)
        self._log_global(cam_info, "camera")

        self.camera_timer.start(1)  # Process as fast as possible

    def _stop_camera(self):
        """Stop the camera and release resources"""
        self.camera_active = False
        self.camera_timer.stop()

        if self.cap is not None:
            self.cap.release()
            self.cap = None

        self.btn_start_camera.setText("Start Camera")
        self.lbl_camera_status.setText("Camera: OFF")
        self.lbl_camera_status.setStyleSheet("color: #ef4444; font-weight: bold;")
        self.lbl_video.setText("Obraz z kamery pojawi się tutaj")
        self.lbl_fps.setText("FPS: 0.0")

        self._log_ball("Kamera zatrzymana", "warn")
        self._log_aruco("Kamera zatrzymana", "warn")
        self._log_global("Kamera zatrzymana", "warn")

    def _find_camera(self):
        """Find and initialize the camera with caching for faster reconnection"""
        # Get selected camera from dropdown
        selected_index = self.combo_camera.currentData()

        # If specific camera selected (not Auto)
        if selected_index is not None and selected_index >= 0:
            for backend in [cv2.CAP_DSHOW, cv2.CAP_MSMF]:
                cap = self._try_open_camera(selected_index, backend)
                if cap is not None:
                    self._last_camera_index = selected_index
                    self._last_camera_backend = backend
                    return cap
            self._log_global(f"Nie można otworzyć kamery {selected_index}", "error")
            return None

        # Auto mode - try cached settings first (fastest path)
        if self._last_camera_index is not None:
            cap = self._try_open_camera(self._last_camera_index, self._last_camera_backend)
            if cap is not None:
                return cap
            # Cache invalid, reset it
            self._last_camera_index = None
            self._last_camera_backend = None

        # Scan for camera: prioritize DSHOW (faster for DirectShow cameras like PS3 Eye)
        for i in [1, 0, 2, 3]:
            for backend in [cv2.CAP_DSHOW, cv2.CAP_MSMF]:
                cap = self._try_open_camera(i, backend)
                if cap is not None:
                    # Cache working settings
                    self._last_camera_index = i
                    self._last_camera_backend = backend
                    return cap
        return None

    def _try_open_camera(self, index, backend):
        """Try to open camera at given index with given backend"""
        try:
            cap = cv2.VideoCapture(index, backend)
            if not cap.isOpened():
                return None

            # Configure camera
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
            cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            # Quick validation - single frame test
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                return cap

            cap.release()
        except Exception:
            pass
        return None

    def _process_frame(self):
        """Process a single frame from the camera"""
        if not self.camera_active or self.cap is None:
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        # Calculate FPS
        self.fps_counter += 1
        current_time = cv2.getTickCount() / cv2.getTickFrequency()
        elapsed = current_time - self.fps_start_time
        if elapsed >= 1.0:
            self.current_fps = self.fps_counter / elapsed
            self.fps_counter = 0
            self.fps_start_time = current_time
            self.lbl_fps.setText(f"FPS: {self.current_fps:.1f}")

        # Process based on current tab
        if self.current_tab == 0:
            # Ball detection mode
            combined = self._process_ball_detection(frame)
        elif self.current_tab == 1:
            # ArUco detection mode
            combined = self._process_aruco_detection(frame)
        else:
            # Combined measurements mode
            combined = self._process_measurements(frame)

        # Convert to QPixmap and display
        self._display_frame(combined)

    def _process_ball_detection(self, frame):
        """Process frame for ball detection using HSV"""
        # Apply blur and convert to HSV
        blur = cv2.GaussianBlur(frame, (self.blur_size, self.blur_size), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # Create mask
        mask = cv2.inRange(hsv, (self.h_min, self.s_min, self.v_min), (self.h_max, self.s_max, self.v_max))

        # Morphology
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw on frame
        final_frame = frame.copy()
        self.ball_detected = False
        detected_area = 0

        if contours:
            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            detected_area = int(area)

            if area > self.min_area:
                ((x, y), radius) = cv2.minEnclosingCircle(cnt)
                self.ball_x = int(x)
                self.ball_y = int(y)
                self.ball_detected = True

                cv2.circle(final_frame, (self.ball_x, self.ball_y), int(radius), (0, 255, 0), 2)
                cv2.circle(final_frame, (self.ball_x, self.ball_y), 5, (0, 0, 255), -1)

                # Position text
                cv2.putText(final_frame, f"Ball: ({self.ball_x}, {self.ball_y})", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

                # Update label
                self.lbl_ball_pos.setText(f"Ball: ({self.ball_x}, {self.ball_y})")
                self.lbl_ball_pos.setStyleSheet("color: #10b981;")

                # Emit signal
                self.ball_position_update.emit(self.ball_x, self.ball_y)

        # Log state changes only
        if self.ball_detected and not self._last_ball_state:
            self._log_ball(f"Pileczka wykryta! pos=({self.ball_x}, {self.ball_y}) area={detected_area}px")
            self._log_global(f"Pileczka wykryta @ ({self.ball_x}, {self.ball_y})", "ball")
        elif not self.ball_detected and self._last_ball_state:
            if detected_area > 0:
                self._log_ball(f"Pileczka zgubiona (area={detected_area} < min={self.min_area})", "warn")
            else:
                self._log_ball("Pileczka zgubiona (brak konturu)", "warn")
            self._log_global("Pileczka zgubiona", "warn")
        self._last_ball_state = self.ball_detected

        if not self.ball_detected:
            self.lbl_ball_pos.setText("Ball: Not detected")
            self.lbl_ball_pos.setStyleSheet("color: #ef4444;")

        # Add labels
        cv2.putText(final_frame, f"FPS: {self.current_fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "Oryginal", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Convert mask to BGR for display
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.putText(mask_bgr, "Maska HSV", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        # cv2.putText(final_frame, "Detekcja Pileczki", (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Create combined view
        top_row = np.hstack([frame, mask_bgr])
        final_resized = cv2.resize(final_frame, (top_row.shape[1], frame.shape[0]))
        combined = np.vstack([top_row, final_resized])

        return combined

    def _process_aruco_detection(self, frame):
        """Process frame for ArUco marker detection with enhanced processing"""
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply brightness/contrast adjustments for detection
        if self.aruco_contrast != 1.0 or self.aruco_brightness != 0:
            gray_adjusted = cv2.convertScaleAbs(gray, alpha=self.aruco_contrast, beta=self.aruco_brightness)
        else:
            gray_adjusted = gray

        # Apply cached gamma correction
        if self._gamma_lut is not None:
            gray_adjusted = cv2.LUT(gray_adjusted, self._gamma_lut)

        # ENHANCED: Apply sharpening kernel (emphasizes edges for better marker detection)
        if self.aruco_sharpen_enabled:
            gray_adjusted = cv2.filter2D(gray_adjusted, -1, self._sharpen_kernel)

        # ENHANCED: Super-resolution upscaling for small markers
        scale_factor = self.aruco_upscale_factor if self.aruco_upscale_enabled else 1.0
        if scale_factor > 1.0:
            h, w = gray_adjusted.shape[:2]
            # Use INTER_LANCZOS4 for best quality upscaling (better than CUBIC for edge detection)
            gray_upscaled = cv2.resize(gray_adjusted, (int(w * scale_factor), int(h * scale_factor)), interpolation=cv2.INTER_LANCZOS4)
            # Detect markers on upscaled image
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray_upscaled)
            # IMPORTANT: Scale detected corner coordinates back down to original resolution
            if corners:
                for corner in corners:
                    corner /= scale_factor
        else:
            # Standard detection without upscaling
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray_adjusted)

        # Filter markers by minimum area
        filtered_corners = []
        filtered_ids = []
        if ids is not None and len(ids) > 0:
            for i, marker_id in enumerate(ids):
                area = cv2.contourArea(corners[i])
                if area >= self.aruco_min_area:
                    filtered_corners.append(corners[i])
                    filtered_ids.append(marker_id)

        # Create display frame
        frame_with_aruco = frame.copy()
        centers = {}
        num_markers = 0
        current_ids = set()

        if filtered_ids:
            num_markers = len(filtered_ids)
            frame_with_aruco = cv2.aruco.drawDetectedMarkers(frame_with_aruco, filtered_corners, np.array(filtered_ids))

            # Calculate centers and draw text
            for i, marker_id in enumerate(filtered_ids):
                corner = filtered_corners[i][0]
                center_x = int((corner[0][0] + corner[2][0]) / 2)
                center_y = int((corner[0][1] + corner[2][1]) / 2)
                mid = int(marker_id[0])
                centers[mid] = (center_x, center_y)
                current_ids.add(mid)

                cv2.putText(frame_with_aruco, f"ID:{marker_id[0]}", (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)

            # Draw lines between marker pairs
            if 2 in centers and 0 in centers:
                cv2.line(frame_with_aruco, centers[2], centers[0], (255, 0, 255), 2)  # Magenta
            if 1 in centers and 3 in centers:
                cv2.line(frame_with_aruco, centers[1], centers[3], (0, 255, 255), 2)  # Yellow

            # Emit signal
            self.aruco_markers_update.emit(centers)
        else:
            cv2.putText(frame_with_aruco, "No ArUco markers detected", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

        # Log ArUco state changes
        new_ids = current_ids - self._last_aruco_ids
        lost_ids = self._last_aruco_ids - current_ids
        if new_ids:
            for mid in sorted(new_ids):
                pos = centers.get(mid, (0, 0))
                self._log_aruco(f"Marker ID:{mid} wykryty @ ({pos[0]}, {pos[1]})")
                self._log_global(f"ArUco ID:{mid} wykryty @ ({pos[0]}, {pos[1]})", "aruco")
        if lost_ids:
            for mid in sorted(lost_ids):
                self._log_aruco(f"Marker ID:{mid} zgubiony", "warn")
                self._log_global(f"ArUco ID:{mid} zgubiony", "warn")
        self._last_aruco_ids = current_ids

        # Update status label
        self.lbl_aruco_status.setText(f"ArUco: {num_markers} markers")
        if num_markers > 0:
            self.lbl_aruco_status.setStyleSheet("color: #10b981;")
        else:
            self.lbl_aruco_status.setStyleSheet("color: #ef4444;")

        # Add labels
        cv2.putText(frame_with_aruco, f"FPS: {self.current_fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame_with_aruco, f"MinArea: {self.aruco_min_area}px^2", (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, "Oryginal", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Gray image for debug view (with adjustments used for detection)
        gray_bgr = cv2.cvtColor(gray_adjusted, cv2.COLOR_GRAY2BGR)
        cv2.putText(gray_bgr, "Detekcja (B/C/G)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame_with_aruco, "ArUco Detection", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Create combined view
        top_row = np.hstack([frame, gray_bgr])
        frame_with_aruco_resized = cv2.resize(frame_with_aruco, (top_row.shape[1], frame.shape[0]))
        combined = np.vstack([top_row, frame_with_aruco_resized])

        return combined

    def _project_point_on_line(self, point, line_start, line_end):
        """
        Project a point onto a line segment and return the position (0.0 to 1.0).
        Returns -1 if projection is outside the line segment.
        """
        px, py = point
        x1, y1 = line_start
        x2, y2 = line_end

        # Line vector
        dx = x2 - x1
        dy = y2 - y1

        # Line length squared
        line_len_sq = dx * dx + dy * dy
        if line_len_sq == 0:
            return -1

        # Parameter t for projection point
        t = ((px - x1) * dx + (py - y1) * dy) / line_len_sq

        # Clamp to [0, 1] range (point on segment)
        return max(0.0, min(1.0, t))

    def _calculate_angle_to_horizontal(self, line_start, line_end):
        """
        Calculate the angle of a line relative to the horizontal axis.
        Returns angle in degrees, positive = counterclockwise from horizontal.
        """
        dx = line_end[0] - line_start[0]
        dy = line_end[1] - line_start[1]
        # atan2 gives angle from positive X axis, but Y is inverted in image coords
        # so we negate dy to get conventional angle (positive = CCW)
        angle_rad = np.arctan2(-dy, dx)
        return np.degrees(angle_rad)

    def _calculate_angle_between_lines(self, line1_start, line1_end, line2_start, line2_end):
        """
        Calculate the angle between two lines in degrees.
        Returns angle in range [-90, 90] degrees.
        """
        # Direction vectors
        v1 = np.array([line1_end[0] - line1_start[0], line1_end[1] - line1_start[1]], dtype=float)
        v2 = np.array([line2_end[0] - line2_start[0], line2_end[1] - line2_start[1]], dtype=float)

        # Normalize vectors
        len1 = np.linalg.norm(v1)
        len2 = np.linalg.norm(v2)
        if len1 == 0 or len2 == 0:
            return 0.0

        v1 = v1 / len1
        v2 = v2 / len2

        # Dot product gives cos(angle)
        dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle_rad = np.arccos(dot)

        # Cross product to determine sign (2D cross product is scalar)
        cross = v1[0] * v2[1] - v1[1] * v2[0]

        angle_deg = np.degrees(angle_rad)
        if cross < 0:
            angle_deg = -angle_deg

        return angle_deg

    def _process_measurements(self, frame):
        """Process frame for combined ball + ArUco measurements"""
        # --- Ball detection ---
        blur = cv2.GaussianBlur(frame, (self.blur_size, self.blur_size), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (self.h_min, self.s_min, self.v_min), (self.h_max, self.s_max, self.v_max))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.ball_detected = False
        if contours:
            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            if area > self.min_area:
                ((x, y), radius) = cv2.minEnclosingCircle(cnt)
                self.ball_x = int(x)
                self.ball_y = int(y)
                self.ball_detected = True

        # --- ArUco detection with hold (using enhanced processing) ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply brightness/contrast adjustments
        if self.aruco_contrast != 1.0 or self.aruco_brightness != 0:
            gray_adjusted = cv2.convertScaleAbs(gray, alpha=self.aruco_contrast, beta=self.aruco_brightness)
        else:
            gray_adjusted = gray

        # Apply cached gamma correction
        if self._gamma_lut is not None:
            gray_adjusted = cv2.LUT(gray_adjusted, self._gamma_lut)

        # ENHANCED: Apply sharpening kernel
        if self.aruco_sharpen_enabled:
            gray_adjusted = cv2.filter2D(gray_adjusted, -1, self._sharpen_kernel)

        # ENHANCED: Super-resolution upscaling
        scale_factor = self.aruco_upscale_factor if self.aruco_upscale_enabled else 1.0
        if scale_factor > 1.0:
            h, w = gray_adjusted.shape[:2]
            gray_upscaled = cv2.resize(gray_adjusted, (int(w * scale_factor), int(h * scale_factor)), interpolation=cv2.INTER_LANCZOS4)
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray_upscaled)
            if corners:
                for corner in corners:
                    corner /= scale_factor
        else:
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray_adjusted)

        current_aruco_centers = {}
        if ids is not None and len(ids) > 0:
            for i, marker_id in enumerate(ids):
                area_aruco = cv2.contourArea(corners[i])
                if area_aruco >= self.aruco_min_area:
                    corner = corners[i][0]
                    center_x = int((corner[0][0] + corner[2][0]) / 2)
                    center_y = int((corner[0][1] + corner[2][1]) / 2)
                    mid = int(marker_id[0])
                    current_aruco_centers[mid] = (center_x, center_y)
                    # Reset hold counter when marker is detected
                    self._aruco_hold_frames[mid] = 0
                    # Update held position
                    self._held_aruco_centers[mid] = (center_x, center_y)

        # Apply hold for markers that were recently lost
        self.aruco_centers = dict(current_aruco_centers)
        for mid in list(self._held_aruco_centers.keys()):
            if mid not in current_aruco_centers:
                self._aruco_hold_frames[mid] = self._aruco_hold_frames.get(mid, 0) + 1
                if self._aruco_hold_frames[mid] <= self._aruco_hold_max:
                    # Use held position
                    self.aruco_centers[mid] = self._held_aruco_centers[mid]
                else:
                    # Hold expired, remove
                    del self._held_aruco_centers[mid]
                    del self._aruco_hold_frames[mid]

        # --- Calculate measurements ---
        self._raw_ball_position = -1.0
        self.ball_position_on_beam = -1.0
        self.ball_position_mm = -1.0
        self._raw_beam_angle = 0.0
        self.beam_angle = 0.0

        # Ball position on beam (raw projection)
        if self.ball_detected and 2 in self.aruco_centers and 0 in self.aruco_centers:
            self._raw_ball_position = self._project_point_on_line((self.ball_x, self.ball_y), self.aruco_centers[2], self.aruco_centers[0])
            # Apply calibration to get corrected position
            self.ball_position_on_beam = self._apply_calibration(self._raw_ball_position)
            # Convert to mm
            self.ball_position_mm = self.ball_position_on_beam * self.beam_length_mm

        # Angle measurement (using selected method)
        if 2 in self.aruco_centers and 0 in self.aruco_centers:
            if self.angle_method == 0:
                # Method 0: Angle relative to horizontal axis (no perspective issues)
                self._raw_beam_angle = self._calculate_angle_to_horizontal(self.aruco_centers[2], self.aruco_centers[0])
            elif 1 in self.aruco_centers and 3 in self.aruco_centers:
                # Method 1: Angle relative to reference line (ID1-ID3)
                self._raw_beam_angle = self._calculate_angle_between_lines(self.aruco_centers[2], self.aruco_centers[0], self.aruco_centers[1], self.aruco_centers[3])
            # Apply angle offset
            self.beam_angle = self._raw_beam_angle + self.angle_offset

        # --- Update UI labels (mm instead of %) ---
        if self.ball_position_mm >= 0:
            self.lbl_ball_on_beam.setText(f"{self.ball_position_mm:.1f} mm")
            self.lbl_ball_on_beam.setStyleSheet("font-weight: bold; font-size: 14px; color: #10b981;")
            self.ball_on_beam_update.emit(self.ball_position_on_beam)
        else:
            self.lbl_ball_on_beam.setText("-- mm")
            self.lbl_ball_on_beam.setStyleSheet("font-weight: bold; font-size: 14px; color: #ef4444;")
            self.ball_on_beam_update.emit(-1.0)

        # Raw position (before calibration)
        if self._raw_ball_position >= 0:
            self.lbl_ball_raw_percent.setText(f"{self._raw_ball_position * 100:.1f} %")
        else:
            self.lbl_ball_raw_percent.setText("-- %")

        self.lbl_beam_angle.setText(f"{self.beam_angle:.2f} °")
        self.beam_angle_update.emit(self.beam_angle)

        if self.ball_detected:
            self.lbl_ball_raw.setText(f"({self.ball_x}, {self.ball_y})")
        else:
            self.lbl_ball_raw.setText("(-, -)")

        # Show which markers are held vs live
        markers_parts = []
        for k in sorted(self.aruco_centers.keys()):
            if k in current_aruco_centers:
                markers_parts.append(f"ID:{k}")
            else:
                markers_parts.append(f"ID:{k}(H)")  # H = held
        self.lbl_aruco_markers.setText(", ".join(markers_parts) if markers_parts else "Brak")

        # --- Draw visualization ---
        final_frame = frame.copy()

        # Draw ArUco markers and lines
        if 2 in self.aruco_centers and 0 in self.aruco_centers:
            cv2.line(final_frame, self.aruco_centers[2], self.aruco_centers[0], (255, 0, 255), 2)  # Magenta
        if 1 in self.aruco_centers and 3 in self.aruco_centers:
            cv2.line(final_frame, self.aruco_centers[1], self.aruco_centers[3], (0, 255, 255), 2)  # Yellow

        # Draw marker centers (different color for held markers)
        for marker_id, center in self.aruco_centers.items():
            if marker_id in current_aruco_centers:
                cv2.circle(final_frame, center, 5, (0, 255, 0), -1)  # Green = live
            else:
                cv2.circle(final_frame, center, 5, (0, 165, 255), -1)  # Orange = held
            cv2.putText(final_frame, f"ID:{marker_id}", (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

        # Draw ball
        if self.ball_detected:
            cv2.circle(final_frame, (self.ball_x, self.ball_y), 10, (0, 255, 0), 2)
            cv2.circle(final_frame, (self.ball_x, self.ball_y), 3, (0, 0, 255), -1)

            # Draw projected position on beam
            if self.ball_position_on_beam >= 0 and 2 in self.aruco_centers and 0 in self.aruco_centers:
                p2 = self.aruco_centers[2]
                p0 = self.aruco_centers[0]
                proj_x = int(p2[0] + self.ball_position_on_beam * (p0[0] - p2[0]))
                proj_y = int(p2[1] + self.ball_position_on_beam * (p0[1] - p2[1]))
                cv2.circle(final_frame, (proj_x, proj_y), 6, (255, 255, 0), -1)  # Cyan
                cv2.line(final_frame, (self.ball_x, self.ball_y), (proj_x, proj_y), (255, 255, 0), 1)  # Line from ball to projection

        # Add info text (mm instead of %)
        cv2.putText(final_frame, f"FPS: {self.current_fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(final_frame, f"Pozycja: {self.ball_position_mm:.1f}mm" if self.ball_position_mm >= 0 else "Pozycja: --", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(final_frame, f"Kat: {self.beam_angle:.2f} deg", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

        # Convert mask to BGR
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.putText(mask_bgr, "Maska HSV", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, "Oryginal", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Create combined view
        top_row = np.hstack([frame, mask_bgr])
        final_resized = cv2.resize(final_frame, (top_row.shape[1], frame.shape[0]))
        combined = np.vstack([top_row, final_resized])

        return combined

    def _display_frame(self, frame):
        """Convert OpenCV frame to QPixmap and display"""
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)

        # Scale to fit label while maintaining aspect ratio
        scaled_pixmap = pixmap.scaled(self.lbl_video.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.lbl_video.setPixmap(scaled_pixmap)

    def closeEvent(self, event):
        """Clean up when widget is closed"""
        if self.camera_active:
            self._stop_camera()
        event.accept()
