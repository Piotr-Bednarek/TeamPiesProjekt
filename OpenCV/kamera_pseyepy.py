import cv2
import numpy as np
import time
import sys

# Dodaj ścieżkę do pseyepy
sys.path.insert(0, r"c:\Users\Piotr\STM32CubeIDE\workspace_1.19.0\STM_PROJEKT_ZALICZENIOWY\pseyepy")

from pseyepy import Camera

# ==================== USTAWIENIA KAMERY ====================
# Rozdzielczości dla PS3 Eye:
# Camera.RES_LARGE = 640x480 @ max 60 FPS
# Camera.RES_SMALL = 320x240 @ max 120+ FPS (dla maksymalnego FPS)

RESOLUTION = Camera.RES_SMALL  # Zmień na Camera.RES_LARGE dla 640x480
CAMERA_FPS = 120  # Max 60 dla RES_LARGE, max ~150 dla RES_SMALL
USE_COLOR = True  # True = kolor, False = grayscale (szybsze)
# ===========================================================

print("Initializing PS3 Eye camera with pseyepy...")
print(f"  -> Resolution: {'320x240' if RESOLUTION == Camera.RES_SMALL else '640x480'}")
print(f"  -> Requested FPS: {CAMERA_FPS}")
print(f"  -> Color: {USE_COLOR}")

try:
    # Inicjalizacja kamery
    cam = Camera(0, fps=CAMERA_FPS, resolution=RESOLUTION, colour=USE_COLOR)  # Indeks kamery (0 = pierwsza)

    # Opcjonalne ustawienia kamery
    cam.exposure = 120  # 0-255
    cam.gain = 30  # 0-63
    cam.auto_gain = False
    cam.auto_exposure = False

    print("Camera initialized successfully!")
    print("Camera feed active. Press 'q' to exit.")

except Exception as e:
    print(f"\nCRITICAL ERROR: Could not initialize camera: {e}")
    print("Make sure:")
    print("  1. PS3 Eye is connected via USB")
    print("  2. CL-Eye driver is NOT running (close CL-Eye Test)")
    print("  3. pseyepy is properly installed")
    sys.exit(1)

# Zmienne do liczenia FPS
fps_counter = 0
fps_start_time = time.time()
current_fps = 0.0

# Rozmiar ramki
if RESOLUTION == Camera.RES_SMALL:
    frame_width, frame_height = 320, 240
else:
    frame_width, frame_height = 640, 480

while True:
    # Odczyt klatki z kamery
    frame, timestamp = cam.read()

    if frame is None:
        print("Failed to grab frame")
        continue

    # pseyepy zwraca RGB, OpenCV używa BGR
    if USE_COLOR:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    else:
        # Grayscale - konwertuj do BGR dla wyświetlania
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    # Obliczanie FPS
    fps_counter += 1
    elapsed_time = time.time() - fps_start_time
    if elapsed_time >= 0.5:  # Aktualizuj FPS co 0.5 sekundy dla szybszej aktualizacji
        current_fps = fps_counter / elapsed_time
        fps_counter = 0
        fps_start_time = time.time()

    # Przetwarzanie obrazu
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Zakres dla pomarańczowej piłki (dostosuj do swojej piłki)
    mask = cv2.inRange(hsv, (5, 100, 100), (15, 255, 255))

    # Wykrywanie konturów
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    final_frame = frame.copy()

    ball_detected = False
    ball_x, ball_y, ball_radius = 0, 0, 0

    if contours:
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) > 100:
            ((ball_x, ball_y), ball_radius) = cv2.minEnclosingCircle(cnt)
            cv2.circle(final_frame, (int(ball_x), int(ball_y)), int(ball_radius), (0, 255, 0), 2)
            cv2.circle(final_frame, (int(ball_x), int(ball_y)), 5, (0, 0, 255), -1)
            ball_detected = True

    # Dodanie napisu z FPS na obrazie
    fps_text = f"FPS: {current_fps:.1f}"
    cv2.putText(final_frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

    # Dodanie informacji o rozdzielczości
    res_text = f"{frame_width}x{frame_height}"
    cv2.putText(final_frame, res_text, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

    # Dodanie pozycji piłki jeśli wykryta
    if ball_detected:
        pos_text = f"Ball: ({int(ball_x)}, {int(ball_y)})"
        cv2.putText(final_frame, pos_text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    # Konwersja maski do BGR żeby można było połączyć z innymi obrazami
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # Dodanie etykiet na każdy obraz
    cv2.putText(frame, "Oryginal", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(mask_bgr, "Maska", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(final_frame, "Wykrywanie", (10, frame_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    # Połączenie obrazów w jedno okno
    top_row = np.hstack([frame, mask_bgr])
    final_resized = cv2.resize(final_frame, (top_row.shape[1], frame_height))
    combined = np.vstack([top_row, final_resized])

    # Wyświetlanie w jednym oknie
    cv2.imshow("Ball Tracking - PS3 Eye (pseyepy)", combined)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    elif key == ord("+"):  # Zwiększ exposure
        cam.exposure = min(255, cam.exposure[0] + 10)
        print(f"Exposure: {cam.exposure[0]}")
    elif key == ord("-"):  # Zmniejsz exposure
        cam.exposure = max(0, cam.exposure[0] - 10)
        print(f"Exposure: {cam.exposure[0]}")
    elif key == ord("g"):  # Zwiększ gain
        cam.gain = min(63, cam.gain[0] + 5)
        print(f"Gain: {cam.gain[0]}")
    elif key == ord("f"):  # Zmniejsz gain
        cam.gain = max(0, cam.gain[0] - 5)
        print(f"Gain: {cam.gain[0]}")

print("\nClosing camera...")
cam.end()
cv2.destroyAllWindows()
print("Done.")
