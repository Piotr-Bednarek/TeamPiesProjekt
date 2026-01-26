import cv2
import numpy as np
import time

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

MAX_DISPLAY_WIDTH = 1920
MAX_DISPLAY_HEIGHT = 1080

H_MIN, H_MAX = 10, 27  # Hue (odcień)
S_MIN, S_MAX = 61, 141  # Saturation (nasycenie)
V_MIN, V_MAX = 112, 218  # Value (jasność)


def find_camera():
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

                # Zwiększenie jasności kamery
                cap.set(cv2.CAP_PROP_BRIGHTNESS, 50)
                cap.set(cv2.CAP_PROP_CONTRAST, 50)
                cap.set(cv2.CAP_PROP_GAIN, 100)

                # Try to get a frame to confirm it's working
                for _ in range(5):
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        print(f"  -> SUCCESS! Found camera at index {i}")
                        print(f"  -> Resolution: {frame.shape[1]}x{frame.shape[0]} @ ~30 FPS")
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

# Zmienne globalne dla trackbarów
h_min, h_max = H_MIN, H_MAX
s_min, s_max = S_MIN, S_MAX
v_min, v_max = V_MIN, V_MAX

# Zmienne do regulacji jasności/kontrastu
brightness_adjust = 0
contrast_adjust = 1.0
gamma_value = 1.5  # Domyślne gamma dla słabego oświetlenia
aruco_min_area_px = 300  # Minimalna powierzchnia markera w pikselach
display_scale = 1.0  # Skala powiększenia obrazu do podglądu

# Inicjalizacja detektora ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
# Dostosowanie parametrów detektora do lepszej detekcji
aruco_params.adaptiveThreshConstant = 5  # Bardziej wrażliwy na zmienne oświetlenie
aruco_params.minMarkerLengthRatioOriginalImg = 0.01  # Bardzo małe markery
aruco_params.polygonalApproxAccuracyRate = 0.03  # Bardziej elastyczne
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
aruco_params.detectInvertedMarker = True  # Pozwala na odwrócone markery
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)


def on_trackbar(x):
    global h_min, h_max, s_min, s_max, v_min, v_max
    h_min = cv2.getTrackbarPos("H min", "Ball Tracking - PS3 Eye")
    h_max = cv2.getTrackbarPos("H max", "Ball Tracking - PS3 Eye")
    s_min = cv2.getTrackbarPos("S min", "Ball Tracking - PS3 Eye")
    s_max = cv2.getTrackbarPos("S max", "Ball Tracking - PS3 Eye")
    v_min = cv2.getTrackbarPos("V min", "Ball Tracking - PS3 Eye")
    v_max = cv2.getTrackbarPos("V max", "Ball Tracking - PS3 Eye")


def on_aruco_trackbar(x):
    global aruco_params, brightness_adjust, contrast_adjust, aruco_min_area_px, display_scale
    adaptive_const = cv2.getTrackbarPos("ArUco Adaptive", "ArUco Settings")
    min_marker_size = cv2.getTrackbarPos("ArUco Min Size", "ArUco Settings") / 1000.0
    brightness_adjust = cv2.getTrackbarPos("Brightness", "ArUco Settings") - 50
    contrast_adjust = cv2.getTrackbarPos("Contrast", "ArUco Settings") / 50.0
    aruco_min_area_px = cv2.getTrackbarPos("ArUco Min Area", "ArUco Settings")
    display_scale = cv2.getTrackbarPos("Display Scale", "ArUco Settings") / 100.0

    aruco_params.adaptiveThreshConstant = max(1, adaptive_const)
    aruco_params.minMarkerLengthRatioOriginalImg = min_marker_size


def apply_gamma_correction(image, gamma):
    """Gamma correction dla poprawy widzialności w słabym oświetleniu"""
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype(np.uint8)
    return cv2.LUT(image, table)


def scale_image_to_fit(image, max_width=MAX_DISPLAY_WIDTH, max_height=MAX_DISPLAY_HEIGHT):
    """Skaluje obraz aby zmieścił się w określonych wymiarach bez zmiany proporcji"""
    height, width = image.shape[:2]

    # Oblicz skalę aby obraz zmieścił się w maksymalnych wymiarach
    scale = min(max_width / width, max_height / height)

    if scale < 1.0:  # Tylko skaluj w dół, nie w górę
        new_width = int(width * scale)
        new_height = int(height * scale)
        return cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)

    return image


# Stwórz okno z trackbarami HSV
cv2.namedWindow("Ball Tracking - PS3 Eye")
cv2.createTrackbar("H min", "Ball Tracking - PS3 Eye", H_MIN, 180, on_trackbar)
cv2.createTrackbar("H max", "Ball Tracking - PS3 Eye", H_MAX, 180, on_trackbar)
cv2.createTrackbar("S min", "Ball Tracking - PS3 Eye", S_MIN, 255, on_trackbar)
cv2.createTrackbar("S max", "Ball Tracking - PS3 Eye", S_MAX, 255, on_trackbar)
cv2.createTrackbar("V min", "Ball Tracking - PS3 Eye", V_MIN, 255, on_trackbar)
cv2.createTrackbar("V max", "Ball Tracking - PS3 Eye", V_MAX, 255, on_trackbar)

# Stwórz okno z trackbarami ArUco
cv2.namedWindow("ArUco Settings")
cv2.createTrackbar("ArUco Adaptive", "ArUco Settings", 5, 30, on_aruco_trackbar)
cv2.createTrackbar("ArUco Min Size", "ArUco Settings", 10, 100, on_aruco_trackbar)
cv2.createTrackbar("Brightness", "ArUco Settings", 50, 100, on_aruco_trackbar)
cv2.createTrackbar("Contrast", "ArUco Settings", 50, 100, on_aruco_trackbar)
cv2.createTrackbar("Gamma", "ArUco Settings", 75, 100, on_aruco_trackbar)  # 1.5 domyślnie
cv2.createTrackbar("ArUco Min Area", "ArUco Settings", aruco_min_area_px, 5000, on_aruco_trackbar)
cv2.createTrackbar("Display Scale", "ArUco Settings", 100, 200, on_aruco_trackbar)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Regulacja jasności, kontrastu i gamma
    frame = np.clip(frame.astype(float) * contrast_adjust + brightness_adjust, 0, 255).astype(np.uint8)
    gamma_value = cv2.getTrackbarPos("Gamma", "ArUco Settings") / 50.0
    frame = apply_gamma_correction(frame, gamma_value)

    # Obliczanie FPS
    fps_counter += 1
    elapsed_time = time.time() - fps_start_time
    if elapsed_time >= 1.0:  # Aktualizuj FPS co sekundę
        current_fps = fps_counter / elapsed_time
        fps_counter = 0
        fps_start_time = time.time()

    # Przetwarzanie - konwersja do HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Rozmycie dla redukcji szumu
    hsv_blur = cv2.GaussianBlur(hsv, (5, 5), 0)

    # Maska dla koloru c9ae82
    mask = cv2.inRange(hsv_blur, (h_min, s_min, v_min), (h_max, s_max, v_max))

    # Morfologia - rozszerzenie i erozja dla czyszczenia maski
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Wykrywanie konturów
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    final_frame = frame.copy()

    ball_detected = False
    # Jeśli znaleziono kontury, znajduj największy (piłeczkę)
    if contours:
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) > 50:  # Minimalny rozmiar
            ((x, y), radius) = cv2.minEnclosingCircle(cnt)
            x, y, radius = int(x), int(y), int(radius)

            # Rysowanie znalezionego okręgu
            cv2.circle(final_frame, (x, y), radius, (0, 255, 0), 2)  # Zielony kolor okręgu
            cv2.circle(final_frame, (x, y), 5, (0, 0, 255), -1)  # Czerwony punkt środka

            # Informacja o pozycji
            pos_text = f"Ball: ({x}, {y}) r={radius}"
            cv2.putText(final_frame, pos_text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
            ball_detected = True

    # Konwersja maski do BGR dla wyświetlania obok
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # Dodanie napisu z FPS na obrazie (rzeczywisty zmierzony FPS)
    fps_text = f"Real FPS: {current_fps:.1f}"
    cv2.putText(final_frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

    # Dodanie informacji o rozdzielczości
    res_text = f"{frame.shape[1]}x{frame.shape[0]}"
    cv2.putText(final_frame, res_text, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

    # Dodanie etykiet na każdy obraz
    cv2.putText(frame, "Oryginal", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(mask_bgr, "Maska HSV (c9ae82)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(final_frame, "Detekcja Piłeczki", (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    # Wykrywanie znaczników ArUco na czystym obrazie w skali szarości
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco_detector.detectMarkers(gray)

    # Filtrowanie markerów po minimalnej powierzchni
    filtered_corners = []
    filtered_ids = []
    if ids is not None and len(ids) > 0:
        for i, marker_id in enumerate(ids):
            area = cv2.contourArea(corners[i])
            if area >= aruco_min_area_px:
                filtered_corners.append(corners[i])
                filtered_ids.append(marker_id)

    # Rysowanie wykrytych markerów ArUco (po filtrze)
    if filtered_ids:
        frame_with_aruco = cv2.aruco.drawDetectedMarkers(frame.copy(), filtered_corners, np.array(filtered_ids))

        # Wyświetlenie informacji o wykrytych markerach
        centers = {}
        for i, marker_id in enumerate(filtered_ids):
            corner = filtered_corners[i][0]
            center_x = int((corner[0][0] + corner[2][0]) / 2)
            center_y = int((corner[0][1] + corner[2][1]) / 2)
            centers[int(marker_id[0])] = (center_x, center_y)

            # Tekst z ID markera
            cv2.putText(frame_with_aruco, f"ID:{marker_id[0]}", (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)

        # Rysowanie linii pomiędzy parami markerów
        if 2 in centers and 0 in centers:
            cv2.line(frame_with_aruco, centers[2], centers[0], (255, 0, 255), 2)
        if 1 in centers and 3 in centers:
            cv2.line(frame_with_aruco, centers[1], centers[3], (0, 255, 255), 2)
    else:
        frame_with_aruco = frame.copy()
        cv2.putText(frame_with_aruco, "No ArUco markers detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

    cv2.putText(frame_with_aruco, "ArUco Detection", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame_with_aruco, f"MinArea:{aruco_min_area_px}px^2", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    # Połączenie obrazów w jedno okno (górny rząd: oryginal + aruco, dolny: maska + detekcja piłeczki)
    top_row = np.hstack([frame, frame_with_aruco])
    bottom_row = np.hstack([mask_bgr, final_frame])

    # Powiększenie obrazów do tej samej wysokości
    top_row_resized = cv2.resize(top_row, (top_row.shape[1], 240))
    bottom_row_resized = cv2.resize(bottom_row, (bottom_row.shape[1], 240))

    # Połączenie pionowe
    combined = np.vstack([top_row_resized, bottom_row_resized])

    # Opcjonalne powiększenie podglądu (nie wpływa na detekcję, tylko na wyświetlanie)
    if display_scale != 1.0:
        combined = cv2.resize(combined, None, fx=display_scale, fy=display_scale, interpolation=cv2.INTER_CUBIC)

    # Skalowanie połączonego obrazu aby zmieścił się na ekranie
    combined = scale_image_to_fit(combined, MAX_DISPLAY_WIDTH, MAX_DISPLAY_HEIGHT)

    # Wyświetlanie w jednym oknie
    cv2.imshow("Ball Tracking - PS3 Eye", combined)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
