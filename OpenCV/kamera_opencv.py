import cv2
import numpy as np
import time

# ==================== USTAWIENIA KAMERY ====================
# Dostępne rozdzielczości dla PS3 Eye:
# 640x480 @ max 60 FPS
# 320x240 @ max 120 FPS (dla maksymalnego FPS)
CAMERA_WIDTH = 320  # Zmień na 640 dla wyższej rozdzielczości
CAMERA_HEIGHT = 240  # Zmień na 480 dla wyższej rozdzielczości
CAMERA_FPS = 120  # Zmień na 60 dla 640x480

# Inne popularne rozdzielczości:
# 160x120, 176x144, 320x240, 352x288, 640x480
# ===========================================================

# ==================== USTAWIENIA DETEKCJI KOLORU ====================
# Kolor piłeczki: c9ae82 (beżowo-brązowy)
# RGB: (201, 174, 130) -> BGR: (130, 174, 201)
# HSV range - dostrojone dla koloru c9ae82
H_MIN, H_MAX = 12, 25  # Hue (odcień)
S_MIN, S_MAX = 40, 120  # Saturation (nasycenie)
V_MIN, V_MAX = 80, 255  # Value (jasność)
# ===================================================================


def find_camera():
    # CL-Eye usually works best with DSHOW on index 0 or 1
    for i in [1]:
        for backend in [cv2.CAP_DSHOW, None]:
            backend_str = "DSHOW" if backend == cv2.CAP_DSHOW else "AUTO"
            print(f"Trying camera index {i} with {backend_str}...")

            if backend is not None:
                cap = cv2.VideoCapture(i, backend)
            else:
                cap = cv2.VideoCapture(i)

            if cap.isOpened():
                # Ustawienie rozdzielczości i FPS
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
                cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

                # Wyłączenie buforowania dla niższego opóźnienia
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                # Try to get a frame to confirm it's working
                # Some cameras need a few reads to "get going"
                for _ in range(5):
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        # Success!
                        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        actual_fps = cap.get(cv2.CAP_PROP_FPS)

                        print(f"  -> SUCCESS! Found camera at index {i} with {backend_str}")
                        print(f"  -> Requested: {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS} FPS")
                        print(f"  -> Actual: {actual_width}x{actual_height} @ {actual_fps} FPS")
                        print(f"  -> Avg Brightness: {np.mean(frame):.2f}")
                        return cap
                cap.release()
    return None


cap = find_camera()

if cap is None:
    print("\nCRITICAL ERROR: No camera could be opened in OpenCV.")
    print("Close CL-Eye Test before running this script!")
    exit()

print("Camera feed active. Press 'q' to exit.")

# Zmienne do liczenia FPS
fps_counter = 0
fps_start_time = time.time()
current_fps = 0.0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Obliczanie FPS
    fps_counter += 1
    elapsed_time = time.time() - fps_start_time
    if elapsed_time >= 1.0:  # Aktualizuj FPS co sekundę
        current_fps = fps_counter / elapsed_time
        fps_counter = 0
        fps_start_time = time.time()

    # Przetwarzanie (uproszczone do testu)
    blur = cv2.GaussianBlur(frame, (9, 9), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Zakres dla pomarańczowej piłki (przykładowy)
    mask = cv2.inRange(hsv, (5, 100, 100), (15, 255, 255))

    # Wykrywanie konturów
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    final_frame = frame.copy()

    if contours:
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) > 100:
            ((x, y), radius) = cv2.minEnclosingCircle(cnt)
            cv2.circle(final_frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.circle(final_frame, (int(x), int(y)), 5, (0, 0, 255), -1)

    # Dodanie napisu z FPS na obrazie (rzeczywisty zmierzony FPS)
    fps_text = f"Real FPS: {current_fps:.1f}"
    cv2.putText(final_frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

    # Dodanie informacji o rozdzielczości
    res_text = f"{frame.shape[1]}x{frame.shape[0]}"
    cv2.putText(final_frame, res_text, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

    # Konwersja maski do BGR żeby można było połączyć z innymi obrazami
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # Dodanie etykiet na każdy obraz
    cv2.putText(frame, "Oryginal", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(mask_bgr, "Maska", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(final_frame, "Wykrywanie", (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    # Połączenie obrazów w jedno okno (górny rząd: oryginal + maska, dolny: wykrywanie powiększone)
    top_row = np.hstack([frame, mask_bgr])
    # Powiększenie final_frame do szerokości górnego rzędu
    final_resized = cv2.resize(final_frame, (top_row.shape[1], frame.shape[0]))

    # Połączenie pionowe
    combined = np.vstack([top_row, final_resized])

    # Wyświetlanie w jednym oknie
    cv2.imshow("Ball Tracking - PS3 Eye", combined)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
