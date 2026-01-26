import cv2
import json
import os
import numpy as np
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSlider, QGroupBox, QGridLayout, QTabWidget, QMessageBox
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap


class OpenCVPanel(QWidget):
    ball_position_update = Signal(int, int)  # Signal for ball position (x, y)
    aruco_markers_update = Signal(dict)  # Signal for ArUco markers {id: (x, y)}

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
        self.aruco_brightness = 0  # -50 to +50
        self.aruco_contrast = 1.0  # 0.5 to 2.0
        self.aruco_gamma = 1.5  # 0.5 to 3.0
        self.aruco_min_area = 500  # minimum marker area in px^2
        self.aruco_adaptive_const = 5
        self.aruco_min_size = 0.01

        # Ball position
        self.ball_x = 0
        self.ball_y = 0
        self.ball_detected = False

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshConstant = 5
        self.aruco_params.minMarkerLengthRatioOriginalImg = 0.01
        self.aruco_params.detectInvertedMarker = True
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Current view mode (0 = ball, 1 = aruco)
        self.current_tab = 0

        self._setup_ui()

        # Timer for camera feed
        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self._process_frame)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Top control panel
        top_layout = QHBoxLayout()

        # Camera controls
        cam_group = QGroupBox("Sterowanie kamerą")
        cam_layout = QHBoxLayout(cam_group)

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

        # Main content area
        content_layout = QHBoxLayout()

        # Left side: Video display
        video_layout = QVBoxLayout()

        self.lbl_video = QLabel("Obraz z kamery pojawi się tutaj")
        self.lbl_video.setMinimumSize(640, 480)
        self.lbl_video.setAlignment(Qt.AlignCenter)
        self.lbl_video.setStyleSheet("background-color: #1e1e1e; border: 2px solid #444;")
        video_layout.addWidget(self.lbl_video)

        content_layout.addLayout(video_layout, stretch=3)

        # Right side: Tab widget for different detection modes
        self.tab_widget = QTabWidget()
        self.tab_widget.currentChanged.connect(self._on_tab_changed)

        # Tab 1: Ball detection (HSV)
        ball_tab = self._create_ball_tab()
        self.tab_widget.addTab(ball_tab, "Detekcja Pileczki")

        # Tab 2: ArUco detection
        aruco_tab = self._create_aruco_tab()
        self.tab_widget.addTab(aruco_tab, "Detekcja ArUco")

        content_layout.addWidget(self.tab_widget, stretch=1)

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
        self.slider_min_area = self._create_slider(10, 1000, self.min_area)
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

        sliders_layout.addStretch()

        return tab

    def _create_aruco_tab(self):
        """Create the ArUco detection tab with brightness/contrast/gamma controls"""
        tab = QWidget()
        sliders_layout = QVBoxLayout(tab)

        # Image adjustment
        adjust_group = QGroupBox("Korekcja obrazu")
        adjust_layout = QGridLayout(adjust_group)

        # Brightness (-50 to +50, default 0)
        adjust_layout.addWidget(QLabel("Jasność:"), 0, 0)
        self.slider_aruco_brightness = self._create_slider(-50, 50, self.aruco_brightness)
        self.slider_aruco_brightness.valueChanged.connect(self._on_aruco_brightness_changed)
        adjust_layout.addWidget(self.slider_aruco_brightness, 0, 1)
        self.lbl_aruco_brightness = QLabel(str(self.aruco_brightness))
        adjust_layout.addWidget(self.lbl_aruco_brightness, 0, 2)

        # Contrast (0.5 to 2.0, slider 50-200, default 100)
        adjust_layout.addWidget(QLabel("Kontrast:"), 1, 0)
        self.slider_aruco_contrast = self._create_slider(50, 200, int(self.aruco_contrast * 100))
        self.slider_aruco_contrast.valueChanged.connect(self._on_aruco_contrast_changed)
        adjust_layout.addWidget(self.slider_aruco_contrast, 1, 1)
        self.lbl_aruco_contrast = QLabel(f"{self.aruco_contrast:.2f}")
        adjust_layout.addWidget(self.lbl_aruco_contrast, 1, 2)

        # Gamma (0.5 to 3.0, slider 50-300, default 150)
        adjust_layout.addWidget(QLabel("Gamma:"), 2, 0)
        self.slider_aruco_gamma = self._create_slider(50, 300, int(self.aruco_gamma * 100))
        self.slider_aruco_gamma.valueChanged.connect(self._on_aruco_gamma_changed)
        adjust_layout.addWidget(self.slider_aruco_gamma, 2, 1)
        self.lbl_aruco_gamma = QLabel(f"{self.aruco_gamma:.2f}")
        adjust_layout.addWidget(self.lbl_aruco_gamma, 2, 2)

        sliders_layout.addWidget(adjust_group)

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

        sliders_layout.addWidget(aruco_group)

        # Info label
        info_label = QLabel("Linie:\n- Magenta: ID 2 -> ID 0\n- Zolta: ID 1 -> ID 3")
        info_label.setStyleSheet("color: #888; font-size: 11px; padding: 5px;")
        sliders_layout.addWidget(info_label)

        # Save/Load buttons for ArUco detection
        btn_layout = QHBoxLayout()
        self.btn_save_aruco = QPushButton("Zapisz parametry")
        self.btn_save_aruco.clicked.connect(self._save_parameters)
        btn_layout.addWidget(self.btn_save_aruco)
        self.btn_load_aruco = QPushButton("Wczytaj parametry")
        self.btn_load_aruco.clicked.connect(self._load_parameters)
        btn_layout.addWidget(self.btn_load_aruco)
        sliders_layout.addLayout(btn_layout)

        sliders_layout.addStretch()

        return tab

    def _get_params_file_path(self):
        """Get the path to the parameters file"""
        return os.path.join(os.path.dirname(__file__), "..", "opencv_params.json")

    def _save_parameters(self):
        """Save all detection parameters to a JSON file"""
        params = {
            "ball_detection": {
                "h_min": self.h_min,
                "h_max": self.h_max,
                "s_min": self.s_min,
                "s_max": self.s_max,
                "v_min": self.v_min,
                "v_max": self.v_max,
                "blur_size": self.blur_size,
                "min_area": self.min_area
            },
            "aruco_detection": {
                "brightness": self.aruco_brightness,
                "contrast": self.aruco_contrast,
                "gamma": self.aruco_gamma,
                "adaptive_const": self.aruco_adaptive_const,
                "min_size": self.aruco_min_size,
                "min_area": self.aruco_min_area
            }
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
            if "brightness" in aruco:
                self.slider_aruco_brightness.setValue(aruco["brightness"])
            if "contrast" in aruco:
                self.slider_aruco_contrast.setValue(int(aruco["contrast"] * 100))
            if "gamma" in aruco:
                self.slider_aruco_gamma.setValue(int(aruco["gamma"] * 100))
            if "adaptive_const" in aruco:
                self.slider_aruco_adaptive.setValue(aruco["adaptive_const"])
            if "min_size" in aruco:
                self.slider_aruco_min_size.setValue(int(aruco["min_size"] * 1000))
            if "min_area" in aruco:
                self.slider_aruco_min_area.setValue(aruco["min_area"])

            QMessageBox.information(self, "Sukces", "Parametry zostaly wczytane.")
        except FileNotFoundError:
            QMessageBox.warning(self, "Blad", "Plik z parametrami nie istnieje.")
        except Exception as e:
            QMessageBox.warning(self, "Blad", f"Nie udalo sie wczytac parametrow: {e}")

    def _on_tab_changed(self, index):
        """Handle tab change"""
        self.current_tab = index

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
    def _on_aruco_brightness_changed(self, val):
        self.aruco_brightness = val
        self.lbl_aruco_brightness.setText(str(val))

    def _on_aruco_contrast_changed(self, val):
        self.aruco_contrast = val / 100.0
        self.lbl_aruco_contrast.setText(f"{self.aruco_contrast:.2f}")

    def _on_aruco_gamma_changed(self, val):
        self.aruco_gamma = val / 100.0
        self.lbl_aruco_gamma.setText(f"{self.aruco_gamma:.2f}")

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

    def _toggle_camera(self):
        if self.camera_active:
            self._stop_camera()
        else:
            self._start_camera()

    def _start_camera(self):
        """Initialize and start the camera"""
        self.cap = self._find_camera()

        if self.cap is None:
            self.lbl_camera_status.setText("Camera: ERROR")
            self.lbl_camera_status.setStyleSheet("color: #ef4444; font-weight: bold;")
            return

        self.camera_active = True
        self.fps_counter = 0
        self.fps_start_time = cv2.getTickCount() / cv2.getTickFrequency()

        self.btn_start_camera.setText("Stop Camera")
        self.lbl_camera_status.setText("Camera: ON")
        self.lbl_camera_status.setStyleSheet("color: #10b981; font-weight: bold;")

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

    def _find_camera(self):
        """Find and initialize the PS3 Eye camera (fixed 640x480 @ 30 FPS)"""
        for i in [1, 0]:
            for backend in [cv2.CAP_DSHOW, None]:
                if backend is not None:
                    cap = cv2.VideoCapture(i, backend)
                else:
                    cap = cv2.VideoCapture(i)

                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
                    cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                    # Test if camera works
                    for _ in range(5):
                        ret, frame = cap.read()
                        if ret and frame is not None:
                            return cap
                    cap.release()
        return None

    def _apply_gamma_correction(self, image, gamma):
        """Apply gamma correction for low-light visibility improvement"""
        if gamma == 1.0:
            return image
        inv_gamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype(np.uint8)
        return cv2.LUT(image, table)

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
        else:
            # ArUco detection mode
            combined = self._process_aruco_detection(frame)

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

        if contours:
            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)

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
        """Process frame for ArUco marker detection"""
        # Apply brightness/contrast/gamma adjustments for display
        adjusted_frame = np.clip(frame.astype(float) * self.aruco_contrast + self.aruco_brightness, 0, 255).astype(np.uint8)
        adjusted_frame = self._apply_gamma_correction(adjusted_frame, self.aruco_gamma)

        # Convert to grayscale for ArUco detection (use CLEAN grayscale - no preprocessing!)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)

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
        frame_with_aruco = adjusted_frame.copy()
        centers = {}
        num_markers = 0

        if filtered_ids:
            num_markers = len(filtered_ids)
            frame_with_aruco = cv2.aruco.drawDetectedMarkers(frame_with_aruco, filtered_corners, np.array(filtered_ids))

            # Calculate centers and draw text
            for i, marker_id in enumerate(filtered_ids):
                corner = filtered_corners[i][0]
                center_x = int((corner[0][0] + corner[2][0]) / 2)
                center_y = int((corner[0][1] + corner[2][1]) / 2)
                centers[int(marker_id[0])] = (center_x, center_y)

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

        # Update status label
        self.lbl_aruco_status.setText(f"ArUco: {num_markers} markers")
        if num_markers > 0:
            self.lbl_aruco_status.setStyleSheet("color: #10b981;")
        else:
            self.lbl_aruco_status.setStyleSheet("color: #ef4444;")

        # Add labels
        cv2.putText(frame_with_aruco, f"FPS: {self.current_fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame_with_aruco, f"MinArea: {self.aruco_min_area}px^2", (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(adjusted_frame, "Adjusted (Brightness/Contrast/Gamma)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Gray image for debug view
        gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.putText(gray_bgr, "Grayscale (Clean)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame_with_aruco, "ArUco Detection", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Create combined view
        top_row = np.hstack([adjusted_frame, gray_bgr])
        frame_with_aruco_resized = cv2.resize(frame_with_aruco, (top_row.shape[1], frame.shape[0]))
        combined = np.vstack([top_row, frame_with_aruco_resized])

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
