import cv2
import serial
import time

# ==== CONFIG ====
SERIAL_PORT = 'COM3'   # change to your ESP32 serial port (e.g., /dev/ttyS0)
BAUD_RATE = 115200
CAMERA_INDEX = 0               # 0 for default PiCam/USB cam
DETECTION_THRESHOLD = 0.5      # fake threshold for mushroom detection simulation
# =================

# Connect to ESP32
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
cap = cv2.VideoCapture(CAMERA_INDEX)

def detect_mushroom(frame):
    """
    Dummy detection function — replace with real model later.
    Here it just checks for 'brownish color' area (simulate mushroom color).
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = (10, 50, 50)
    upper = (30, 255, 255)
    mask = cv2.inRange(hsv, lower, upper)
    ratio = mask.sum() / (mask.size * 255)
    return ratio > DETECTION_THRESHOLD

def capture_and_check():
    found = 0
    for i in range(5):
        ret, frame = cap.read()
        if not ret:
            print("Camera capture failed.")
            continue

        # Save for debugging
        cv2.imwrite(f"mushroom_check_{i+1}.jpg", frame)

        if detect_mushroom(frame):
            print(f"Mushroom detected in image {i+1}")
            found += 1
        else:
            print(f"No mushroom in image {i+1}")
        time.sleep(0.5)

    print(f"\nSummary: {found}/5 images show mushrooms.")
    if found >= 3:
        ser.write(b"mushroom_status:1\n")
        print("Sent status 1 to ESP32 (mushroom FOUND).")
    else:
        ser.write(b"mushroom_status:0\n")
        print("Sent status 0 to ESP32 (NO mushroom).")

def main():
    print("Waiting for ESP32 command...")
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            print("Received:", line)
            if line == "START_OPENCV":
                print("Starting mushroom detection sequence...")
                capture_and_check()
            time.sleep(0.1)

try:
    main()
except KeyboardInterrupt:
    print("Exiting...")
finally:
    cap.release()
    ser.close()
