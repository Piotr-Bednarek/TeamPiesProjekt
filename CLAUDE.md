# CLAUDE.md — Ball on Beam Control System

## Project Overview

Projekt zaliczeniowy z automatyki: system automatycznego równoważenia kulki na belce.

**Autorzy**: Piotr Bednarek, Jan Andrzejewski, Mateusz Banaszak (2026)

**Cel**: Utrzymanie kulki w zadanej pozycji na pochylonej belce za pomocą serwomechanizmu sterowanego przez mikrokontroler STM32, z regulatorem PID lub LQR.

---

## Repository Structure

```
STM_PROJEKT_ZALICZENIOWY/
├── 1-desktop-app/      # Aplikacja desktopowa (Python/PySide6)
├── 2-firmware/         # Firmware STM32F767ZI (C, FreeRTOS)
├── 3-docs/             # Dokumentacja i raporty (LaTeX/PDF)
├── 4-cad/              # Modele CAD i schematy elektryczne
├── 5-matlab/           # Modele Simulink i skrypty identyfikacji systemu
├── LQR/                # Notebook Jupyter do projektowania LQR
├── data/               # Nagrania eksperymentalne (CSV)
├── photos/             # Zdjęcia sprzętowe
└── archiwum/           # Archiwum (stare wersje firmware, web-app React, C++)
```

---

## Components

### 1. Firmware (`2-firmware/`)

- **MCU**: STM32F767ZI (Cortex-M7, 216 MHz)
- **IDE**: STM32CubeIDE (Eclipse CDT, managed Makefile)
- **RTOS**: FreeRTOS
- **HAL**: STM32F7xx HAL + CMSIS

**Kluczowe moduły**:
| Plik | Opis |
|------|------|
| `Core/Src/main.c` | Główna logika sterowania |
| `Core/Src/servo_pid.c` | Regulator PID z anti-windup |
| `Core/Src/lqr.c` | Regulator LQR (3-stanowy: pozycja, prędkość, kąt belki) |
| `Core/Src/vl53l0x.c` | Sterownik czujnika odległości VL53L0X (I2C) |
| `Core/Src/servo.c` | Sterowanie serwomechanizmem MG90S (PWM ~50 Hz) |
| `Core/Src/filters.c` | Filtry EMA (wykładnicza średnia ruchoma) |
| `Core/Src/calibration.c` | Kalibracja czujnika |
| `Core/Src/freertos.c` | Zarządzanie zadaniami RTOS |

**Protokół UART** (115200 baud, DMA):
- Ramki binarne z checksumą CRC8
- Telemetria: odległość, pozycja filtrowana, błąd, sygnał sterujący, kąt serwomechanizmu, znacznik czasu
- Komendy: zmiana setpointu, aktualizacja nastaw PID/LQR, kalibracja, przełączanie trybu

**Build**: Otworzyć w STM32CubeIDE → Build Project → Debug as STM32 Embedded C/C++

---

### 2. Desktop App (`1-desktop-app/`)

- **Język**: Python 3
- **GUI**: PySide6 (Qt 6)
- **Wykresy**: pyqtgraph (real-time)
- **Komunikacja**: pyserial
- **Wizja**: OpenCV (podgląd kamery + detekcja kulki)

**Uruchamianie**:
```bash
cd 1-desktop-app
pip install -r requirements.txt
python main.py
```

**Kluczowe pliki**:
| Plik | Opis |
|------|------|
| `main.py` | Punkt wejścia |
| `app.py` | Główne okno aplikacji |
| `serial_manager.py` | Obsługa komunikacji szeregowej |
| `widgets/control_panel.py` | Panel PID/LQR |
| `widgets/charts_panel.py` | Wykresy danych |
| `widgets/opencv_panel.py` | Podgląd kamery |
| `widgets/beam_visualizer.py` | Wizualizacja 3D belki |
| `utils/lqr_calculator.py` | Obliczanie wzmocnień LQR |
| `utils/crc8.py` | Checksum CRC8 |
| `presets.json` | Presety nastaw PID i LQR |
| `opencv_params.json` | Parametry detekcji OpenCV |

---

### 3. Dokumentacja (`3-docs/`)

- `2-opis-zadania/` — specyfikacja zadania (LaTeX + PDF)
- `3-raport/` — raport końcowy (LaTeX + PDF)
- `2-estymacja-modelu/` — identyfikacja układu
- `research/` — materiały badawcze

---

### 4. MATLAB/Simulink (`5-matlab/`)

- `ballonbeam_matlab1.slx` — model Simulink
- `generowanie_APRBS1.m` — generator sygnału APRBS do identyfikacji
- `identyfikacja_skok.m` — identyfikacja odpowiedzi skokowej
- `par_regulatora_APRBS.m` — obliczanie parametrów regulatora
- `identyfikacja-*.csv` — dane z 7 eksperymentów identyfikacyjnych

---

### 5. LQR (`LQR/`)

- `lqr.ipynb` — Jupyter notebook: projektowanie i strojenie regulatora LQR

---

## Hardware

| Element | Opis |
|---------|------|
| STM32F767ZI (Nucleo-F767ZI) | Mikrokontroler główny |
| VL53L0X | Czujnik odległości ToF (I2C), mierzy pozycję kulki |
| MG90S | Serwomechanek (PWM), wychyla belkę |

---

## Control Architecture

```
Czujnik VL53L0X → EMA filtr → Regulator (PID lub LQR) → Servo MG90S
                      ↑
              UART (setpoint, nastawy)
                      ↓
              UART (telemetria)
                      ↓
          Aplikacja desktopowa (Python)
```

**PID**: klasyczne sterowanie zwrotne, nastawy Kp/Ki/Kd, pochodna na pomiarze, anti-windup

**LQR**: optymalne sterowanie w przestrzeni stanów, wektor stanu [pozycja_kulki, prędkość_kulki, kąt_belki], macierze wag Q i R

---

## Notes

- Archiwum (`archiwum/`) zawiera poprzednie wersje — nie edytować
- Web-app React jest zarchiwizowana, zastąpiona przez aplikację Python
- Dane eksperymentalne w `data/recordings/` (format CSV)
- Gitignore wyklucza artefakty build i pliki tymczasowe LaTeX
