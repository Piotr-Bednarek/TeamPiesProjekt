import cv2
import numpy as np
import subprocess

def get_camera_names():
    print("--- System Camera Devices (PowerShell) ---")
    ps_command = 'Get-PnpDevice -PresentOnly | Where-Object { $_.Class -eq "Image" -or $_.Class -eq "Camera" } | Select-Object FriendlyName'
    try:
        output = subprocess.check_output(['powershell', '-Command', ps_command]).decode('utf-8')
        print(output.strip())
    except:
        print("Could not fetch names via PowerShell.")
    print("------------------------------------------\n")

def check_cameras():
    get_camera_names()
    
    print("Scanning indices 0 to 10...")
    for i in range(10):
        # We try both DSHOW and Default backends
        for backend in [cv2.CAP_DSHOW, None]:
            b_name = "DSHOW" if backend == cv2.CAP_DSHOW else "AUTO"
            cap = cv2.VideoCapture(i, backend) if backend is not None else cv2.VideoCapture(i)
            
            if cap.isOpened():
                ret, frame = cap.read()
                status = "OK" if ret else "NO_FRAME"
                brightness = np.mean(frame) if ret else 0
                
                print(f"Index {i} ({b_name}): Status={status}, Brightness={brightness:.2f}")
                
                if ret:
                    window_name = f"Index {i} ({b_name})"
                    cv2.imshow(window_name, frame)
                    # Force window to top
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)
                
                cap.release()
            else:
                # Silently skip unopened indices to avoid spam
                pass
    
    print("\nCheck all windows. If a window is open but black, that index might be your PS3 Eye.")
    print("Press any key to close all windows and exit.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    check_cameras()
