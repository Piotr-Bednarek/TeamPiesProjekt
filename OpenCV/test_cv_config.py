import cv2
import numpy as np

def test_config(index, backend, width=None, height=None):
    backend_name = "DSHOW" if backend == cv2.CAP_DSHOW else "MSMF" if backend == cv2.CAP_MSMF else "AUTO"
    print(f"Testing Index {index} with Backend {backend_name}...")
    
    if backend is not None:
        cap = cv2.VideoCapture(index, backend)
    else:
        cap = cv2.VideoCapture(index)
        
    if not cap.isOpened():
        print(f"  [X] Could not open index {index}")
        return False

    if width and height:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        print(f"  Attempted resolution: {width}x{height}")

    # Read few frames
    for i in range(10):
        ret, frame = cap.read()
        if ret:
            brightness = np.mean(frame)
            if brightness > 0.1:
                print(f"  [!] SUCCESS! Frame captured. Avg brightness: {brightness:.2f}")
                cap.release()
                return True
    
    print(f"  [-] Failed. Frame was black or could not be read.")
    cap.release()
    return False

if __name__ == "__main__":
    indices = [0, 1, 2]
    backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF]
    resolutions = [(640, 480), (320, 240)]

    for idx in indices:
        for b in backends:
            # Test default resolution
            test_config(idx, b)
            # Test specific resolutions
            for r in resolutions:
                test_config(idx, b, r[0], r[1])
