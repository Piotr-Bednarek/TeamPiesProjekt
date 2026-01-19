import cv2
import matplotlib.pyplot as plt
import matplotlib

matplotlib.use("TkAgg")

cap = cv2.VideoCapture(0)

# zmiana ekspozycji kamery (wieksza = wieksze opoznienie obrazu)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
# cap.set(cv2.CAP_PROP_EXPOSURE, -4)

# definicje okien
okna = ["org", "blur", "rgb_hsv", "thresh", "morph_close", "morph_open", "finalne"]

# figure do krokow
fig = plt.figure(figsize=(15, 10))
axes = [fig.add_subplot(3, 3, i + 1) for i in range(7)]

# ukrywamy puste subploty
for i in range(7, 9):
    fig.add_subplot(3, 3, i + 1).axis("off")

plt.tight_layout()


def display_frames(frames, window_names, axes_list):
    for i, (frame, name, ax) in enumerate(zip(frames, window_names, axes_list)):
        # bgr do rgb
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            display_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            display_frame = frame

        ax.clear()
        ax.imshow(display_frame, cmap="gray" if len(frame.shape) == 2 else None)
        ax.set_title(name)
        ax.axis("off")

    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.001)


while True:
    ret, frame = cap.read()
    if not ret:
        break

    # przygotowanie frameow do wyswietlenia
    blur = cv2.GaussianBlur(frame, (9, 9), 0)
    rgb_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(rgb_hsv, (5, 100, 100), (15, 255, 255))
    morph_close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    # morph open dla background noise
    morph_open = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))

    final_frame = frame.copy()

    # wykrywanie konturow
    contours, _ = cv2.findContours(morph_close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # jezeli jest jakis kontur wybieramy ten o najwiekszym polu z nadzieja ze to pilka
        largest_contour = max(contours, key=cv2.contourArea)

        # usuywamy wszystkie inne kontury
        contours = [largest_contour]

        # dopasowanie okregu do wykrytego konturu
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

        # finalne okno z wykrytą piłką
        cv2.drawContours(final_frame, contours, -1, (0, 255, 0), 2)
        # rysowanie dopasowanego okregu
        cv2.circle(final_frame, (int(x), int(y)), int(radius), (255, 0, 0), 2)
        cv2.circle(final_frame, (int(x), int(y)), 5, (0, 0, 255), -1)

    # wysłanie listy framow do wyswietlenia
    frames_to_display = [frame, blur, rgb_hsv, thresh, morph_close, morph_open, final_frame]
    display_frames(frames_to_display, okna, axes)

    if not plt.fignum_exists(1):
        break

cap.release()
plt.close("all")
