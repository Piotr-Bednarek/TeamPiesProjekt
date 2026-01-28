import os
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QSlider, QHBoxLayout, QPushButton, QButtonGroup, QGridLayout, QFrame, QRadioButton, QComboBox, QLineEdit, QMessageBox, QInputDialog
from PySide6.QtCore import Qt, Signal, QTimer
from widgets.beam_visualizer import BeamVisualizer


class SliderControl(QWidget):
    value_changed = Signal(float)
    value_committed = Signal(float)

    def __init__(self, label, min_val, max_val, step, init_val, unit=""):
        super().__init__()
        self.step = step
        self.unit = unit
        self.factor = 1.0 / step if step < 1 else 1.0

        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(10)

        self.lbl_name = QLabel(label)
        self.lbl_name.setStyleSheet("font-weight: bold; color: #94a3b8;")
        self.lbl_name.setFixedWidth(30)

        self.lbl_val = QLabel(f"{init_val} {unit}")
        self.lbl_val.setFixedWidth(50)
        self.lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_val.setStyleSheet("color: #3b82f6; font-family: monospace; font-weight: bold;")

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(int(min_val * self.factor))
        self.slider.setMaximum(int(max_val * self.factor))
        self.slider.setValue(int(init_val * self.factor))

        self.slider.valueChanged.connect(self._on_change)
        self.slider.sliderReleased.connect(self._on_release)

        layout.addWidget(self.lbl_name)
        layout.addWidget(self.slider)
        layout.addWidget(self.lbl_val)

        self.setObjectName("sliderContainer")
        self.setStyleSheet(
            """
            QWidget#sliderContainer {
                background-color: rgba(255, 255, 255, 0.03);
                border: 1px solid rgba(255, 255, 255, 0.08);
                border-radius: 6px;
            }
        """
        )

    def _on_change(self, val):
        real_val = val / self.factor
        if self.step < 1:
            fmt = "{:.4f}" if self.step < 0.001 else "{:.2f}"
        else:
            fmt = "{:.0f}"
        self.lbl_val.setText(f"{fmt.format(real_val)} {self.unit}")
        self.value_changed.emit(real_val)

    def _on_release(self):
        real_val = self.slider.value() / self.factor
        self.value_committed.emit(real_val)

    def set_value(self, val):
        self.slider.setValue(int(val * self.factor))


class ControlPanel(QWidget):
    pid_update = Signal(float, float, float)
    kp_update = Signal(float)
    ki_update = Signal(float)
    kd_update = Signal(float)
    setpoint_update = Signal(float)
    calibration_update = Signal(int, float, float)
    mode_update = Signal(int)
    pid_mode_update = Signal(int)

    def __init__(self):
        super().__init__()

        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(5, 5, 5, 5)
        self.layout.setSpacing(8)

        self.setObjectName("panel")
        self.setProperty("class", "card")

        source_frame = QFrame()
        source_frame.setStyleSheet("background-color: rgba(255,255,255,0.05); border-radius: 4px; padding: 2px;")
        source_layout = QHBoxLayout(source_frame)
        source_layout.setContentsMargins(5, 2, 5, 2)

        lbl_source = QLabel("Źródło:")
        lbl_source.setStyleSheet("font-weight: bold; color: #ccc;")

        self.rb_gui = QRadioButton("GUI (Suwak)")
        self.rb_gui.setChecked(True)
        self.rb_analog = QRadioButton("Potencjometr")
        self.rb_sinus = QRadioButton("Sinus")

        self.mode_group = QButtonGroup(self)
        self.mode_group.addButton(self.rb_gui, 0)
        self.mode_group.addButton(self.rb_analog, 1)
        self.mode_group.addButton(self.rb_sinus, 2)
        self.mode_group.idClicked.connect(self.mode_update.emit)

        source_layout.addWidget(lbl_source)
        source_layout.addStretch()
        source_layout.addWidget(self.rb_gui)
        source_layout.addWidget(self.rb_analog)
        source_layout.addWidget(self.rb_sinus)

        self.layout.addWidget(source_frame)

        self.viz = BeamVisualizer()
        self.viz.setpoint_changed.connect(self._on_viz_setpoint)
        self.layout.addWidget(self.viz)

        self.cal_points = [None] * 5
        self.cal_targets = [0, 62.5, 125, 187.5, 250]
        self.cal_btns = []

        # === Controller Mode Selection ===
        controller_mode_group = QFrame()
        controller_mode_group.setObjectName("controllerModeGroup")
        controller_mode_group.setStyleSheet("#controllerModeGroup { background-color: #1a1f2e; border: 1px solid #334155; border-radius: 8px; }")
        controller_mode_layout = QVBoxLayout(controller_mode_group)
        controller_mode_layout.setContentsMargins(5, 5, 5, 5)
        controller_mode_layout.setSpacing(5)

        controller_mode_label = QLabel("TRYB REGULATORA")
        controller_mode_label.setStyleSheet("color: #a78bfa; font-weight: bold; font-size: 11px;")
        controller_mode_layout.addWidget(controller_mode_label)

        controller_mode_buttons = QHBoxLayout()
        self.rb_custom_pid = QRadioButton("Custom PID")
        self.rb_lqr = QRadioButton("LQR")
        self.rb_custom_pid.setChecked(True)

        self.controller_mode_group = QButtonGroup(self)
        self.controller_mode_group.addButton(self.rb_custom_pid, 0)
        self.controller_mode_group.addButton(self.rb_lqr, 1)
        self.controller_mode_group.idClicked.connect(self._on_controller_mode_change)

        controller_mode_buttons.addWidget(self.rb_custom_pid)
        controller_mode_buttons.addWidget(self.rb_lqr)
        controller_mode_layout.addLayout(controller_mode_buttons)

        self.layout.addWidget(controller_mode_group)

        # === PID Parameters ===
        pid_group = QFrame()
        pid_group.setObjectName("pidGroup")
        pid_group.setStyleSheet("#pidGroup { background-color: #1a1f2e; border: 1px solid #334155; border-radius: 8px; }")
        pid_layout = QVBoxLayout(pid_group)
        pid_layout.setContentsMargins(5, 5, 5, 5)
        pid_layout.setSpacing(5)

        pid_header = QHBoxLayout()
        pid_label = QLabel("PARAMETRY REGULATORA (PID)")
        pid_label.setStyleSheet("color: #3b82f6; font-weight: bold; font-size: 11px;")

        pid_header.addWidget(pid_label)
        pid_header.addStretch()

        pid_layout.addLayout(pid_header)

        self.sli_kp = SliderControl("Kp", 0.0, 2.0, 0.01, 0.44)
        self.sli_ki = SliderControl("Ki", 0.0, 0.01, 0.0001, 0.0053)
        self.sli_kd = SliderControl("Kd", 0.0, 10.0, 0.1, 5.0)

        self.sli_kp.value_committed.connect(self._on_kp_change)
        self.sli_ki.value_committed.connect(self._on_ki_change)
        self.sli_kd.value_committed.connect(self._on_kd_change)

        pid_layout.addWidget(self.sli_kp)
        pid_layout.addWidget(self.sli_ki)
        pid_layout.addWidget(self.sli_kd)

        self.layout.addWidget(pid_group)

        # === Presets ===
        self.presets_file = os.path.join(os.path.dirname(__file__), "..", "pid_presets.txt")
        self.presets = {}
        self._load_presets()

        presets_group = QFrame()
        presets_group.setObjectName("presetsGroup")
        presets_group.setStyleSheet("#presetsGroup { background-color: #1a1f2e; border: 1px solid #334155; border-radius: 8px; }")
        presets_layout = QVBoxLayout(presets_group)
        presets_layout.setContentsMargins(5, 5, 5, 5)
        presets_layout.setSpacing(5)

        presets_label = QLabel("ZESTAWY PID")
        presets_label.setStyleSheet("color: #10b981; font-weight: bold; font-size: 11px;")
        presets_layout.addWidget(presets_label)

        dropdown_row = QHBoxLayout()
        self.presets_combo = QComboBox()
        self.presets_combo.setMinimumWidth(120)
        self.presets_combo.addItem("-- Wybierz --")
        for name in self.presets.keys():
            self.presets_combo.addItem(name)
        self.presets_combo.currentTextChanged.connect(self._on_preset_selected)

        self.btn_apply_preset = QPushButton("Zastosuj")
        self.btn_apply_preset.setStyleSheet("background-color: #3b82f6;")
        self.btn_apply_preset.clicked.connect(self._apply_preset)

        dropdown_row.addWidget(self.presets_combo, 1)
        dropdown_row.addWidget(self.btn_apply_preset)
        presets_layout.addLayout(dropdown_row)

        action_row = QHBoxLayout()
        self.btn_save_preset = QPushButton("Zapisz obecne")
        self.btn_save_preset.setStyleSheet("background-color: #22c55e;")
        self.btn_save_preset.clicked.connect(self._save_preset)

        self.btn_delete_preset = QPushButton("Usuń")
        self.btn_delete_preset.setStyleSheet("background-color: #ef4444;")
        self.btn_delete_preset.clicked.connect(self._delete_preset)

        action_row.addWidget(self.btn_save_preset, 1)
        action_row.addWidget(self.btn_delete_preset)
        presets_layout.addLayout(action_row)

        self.layout.addWidget(presets_group)
        self.layout.addStretch()

        self.current_raw_distance = 0

    def _on_viz_setpoint(self, val):
        self.setpoint_update.emit(val)

    def _on_kp_change(self, val):
        self.kp_update.emit(val)

    def _on_ki_change(self, val):
        self.ki_update.emit(val)

    def _on_kd_change(self, val):
        self.kd_update.emit(val)

    def _on_controller_mode_change(self, mode_id):
        self.pid_mode_update.emit(mode_id)

    def _on_mode_change(self, btn_id):
        self.pid_mode_update.emit(btn_id)

    def _load_presets(self):
        self.presets = {}
        if os.path.exists(self.presets_file):
            try:
                with open(self.presets_file, "r", encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if not line or line.startswith("#"):
                            continue
                        parts = line.split(";")
                        if len(parts) == 4:
                            name, kp, ki, kd = parts
                            self.presets[name] = (float(kp), float(ki), float(kd))
            except Exception as e:
                print(f"Błąd wczytywania presetów: {e}")

    def _save_presets_to_file(self):
        try:
            with open(self.presets_file, "w", encoding="utf-8") as f:
                f.write("# Zestawy PID: nazwa;Kp;Ki;Kd\n")
                for name, (kp, ki, kd) in self.presets.items():
                    f.write(f"{name};{kp};{ki};{kd}\n")
        except Exception as e:
            QMessageBox.critical(self, "Błąd", f"Nie można zapisać pliku: {e}")

    def _on_preset_selected(self, name):
        pass

    def _apply_preset(self):
        name = self.presets_combo.currentText()
        if name == "-- Wybierz --" or name not in self.presets:
            return

        kp, ki, kd = self.presets[name]

        self.sli_kp.set_value(kp)
        self.sli_ki.set_value(ki)
        self.sli_kd.set_value(kd)

        self.btn_apply_preset.setEnabled(False)
        self.btn_apply_preset.setText("Wysyłanie...")

        self.kp_update.emit(kp)
        QTimer.singleShot(500, lambda: self.ki_update.emit(ki))
        QTimer.singleShot(1000, lambda: self.kd_update.emit(kd))
        QTimer.singleShot(1500, lambda: self._finish_apply_preset())

    def _finish_apply_preset(self):
        self.btn_apply_preset.setEnabled(True)
        self.btn_apply_preset.setText("Zastosuj")

    def _save_preset(self):
        name, ok = QInputDialog.getText(self, "Zapisz zestaw PID", "Podaj nazwę dla zestawu:")
        if not ok or not name.strip():
            return

        name = name.strip()

        kp = self.sli_kp.slider.value() / self.sli_kp.factor
        ki = self.sli_ki.slider.value() / self.sli_ki.factor
        kd = self.sli_kd.slider.value() / self.sli_kd.factor

        if name in self.presets:
            reply = QMessageBox.question(self, "Potwierdź", f"Zestaw '{name}' już istnieje. Nadpisać?", QMessageBox.Yes | QMessageBox.No)
            if reply != QMessageBox.Yes:
                return
        else:
            self.presets_combo.addItem(name)

        self.presets[name] = (kp, ki, kd)
        self._save_presets_to_file()
        self.presets_combo.setCurrentText(name)

    def _delete_preset(self):
        name = self.presets_combo.currentText()
        if name == "-- Wybierz --" or name not in self.presets:
            QMessageBox.warning(self, "Uwaga", "Wybierz zestaw do usunięcia.")
            return

        reply = QMessageBox.question(self, "Potwierdź", f"Czy na pewno usunąć zestaw '{name}'?", QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return

        del self.presets[name]
        self._save_presets_to_file()

        idx = self.presets_combo.findText(name)
        if idx >= 0:
            self.presets_combo.removeItem(idx)
        self.presets_combo.setCurrentIndex(0)

    def _on_cal_click(self, idx):
        raw = self.current_raw_distance
        self.cal_points[idx] = raw
        self.cal_btns[idx].setText(f"{raw:.0f}")
        self.cal_btns[idx].setChecked(True)

        if all(p is not None for p in self.cal_points):
            self.btn_save_cal.setEnabled(True)
            self.btn_save_cal.setText("Zapisz kalibrację (Gotowe)")

    def _on_save_cal(self):
        self.btn_save_cal.setEnabled(False)
        self.btn_save_cal.setText("Wysyłanie...")

        self._pending_cal_points = []
        for i, raw in enumerate(self.cal_points):
            if raw is not None:
                self._pending_cal_points.append((i, raw, self.cal_targets[i]))

        self._send_next_cal(0)

    def _send_next_cal(self, idx):
        if idx < len(self._pending_cal_points):
            i, raw, target = self._pending_cal_points[idx]
            self.calibration_update.emit(i, raw, target)
            self.btn_save_cal.setText(f"Wysyłanie {idx+1}/{len(self._pending_cal_points)}...")
            QTimer.singleShot(500, lambda: self._send_next_cal(idx + 1))
        else:
            self.btn_save_cal.setText("Wysłano!")
            self.btn_save_cal.setEnabled(True)
            QTimer.singleShot(2000, lambda: self.btn_save_cal.setText("Zapisz kalibrację (Gotowe)"))

    def update_data(self, data):
        self.viz.set_data(data.get("filtered", 0), data.get("setpoint", 125))
        self.current_raw_distance = data.get("distance", 0)
