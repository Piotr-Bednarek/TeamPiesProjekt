import cv2
import time

def try_ps3eye(index):
    print(f"\n--- Testing Index {index} ---")
    
    # Try with DSHOW backend which is usually required for CL-Eye
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        print(f"Could not open camera {index}")
        return False

    # PS3 Eye specific settings for CL-Eye driver
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # PS3 Eye supports high FPS. Setting it might "wake it up"
    cap.set(cv2.CAP_PROP_FPS, 60)

    print(f"Resolution set to: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
    print(f"FPS set to: {cap.get(cv2.CAP_PROP_FPS)}")

    print("Capturing 30 frames to check for image...")
    found_image = False
    for i in range(30):
        ret, frame = cap.read()
        if ret:
            # Check if frame is not just solid black/gray
            avg = frame.mean()
            if avg > 1.0:
                print(f"Frame {i}: SUCCESS! Avg brightness: {avg:.2f}")
                found_image = True
                cv2.imshow(f"Camera {index} Success", frame)
                cv2.waitKey(1000)
                break
            else:
                if i % 10 == 0:
                    print(f"Frame {i}: Still black (avg {avg:.2f})...")
        else:
            print(f"Frame {i}: Read failed")
        time.sleep(0.1)

    cap.release()
    cv2.destroyAllWindows()
    return found_image

if __name__ == "__main__":
    # Test index 0 and 1
    for idx in [0, 1]:
        if try_ps3eye(idx):
            print(f"\nPositive result on index {idx}!")
            break
    else:
        print("\nAll attempts failed.")
