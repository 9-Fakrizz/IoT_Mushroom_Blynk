import serial
import time
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from datetime import datetime

# =======================
# CONFIGURATION
# =======================
SERIAL_PORT = '/dev/ttyUSB1'   # Change to /dev/ttyACM0 if needed
BAUD_RATE = 115200
MODEL_PATH = 'mushroom_model.tflite'
CONFIDENCE_THRESHOLD = 0.5
CAMERA_ID = 0  # 0 = default camera

# =======================
# FUNCTIONS
# =======================
def connect_serial():
    """Try to connect to serial port with retry"""
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for ESP32 reset
            ser.flushInput()
            print(f"[INFO] Connected to serial port {SERIAL_PORT}")
            return ser
        except serial.SerialException:
            print(f"[WARN] Waiting for ESP32 on {SERIAL_PORT}...")
            time.sleep(2)

def detect_mushroom(frame):
    """Run inference and return (found, confidence)"""
    input_shape = input_details[0]['shape']
    resized = cv2.resize(frame, (input_shape[2], input_shape[1]))
    input_data = np.expand_dims(resized, axis=0).astype(np.float32) / 255.0

    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    output_data = interpreter.get_tensor(output_details[0]['index'])[0]
    confidence = float(output_data if np.isscalar(output_data) else output_data[0])
    return confidence >= CONFIDENCE_THRESHOLD, confidence

# =======================
# INITIAL SETUP
# =======================
print("[INFO] Loading TensorFlow Lite model...")
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

print("[INFO] Connecting to serial...")
ser = connect_serial()

print("[INFO] Opening camera...")
cap = cv2.VideoCapture(CAMERA_ID)
if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

print("[READY] Waiting for command from ESP32...")

# =======================
# MAIN LOOP
# =======================
while True:
    try:
        if ser.in_waiting:
            command = ser.readline().decode(errors='ignore').strip()
            print(f"[{datetime.now().strftime('%H:%M:%S')}] [ESP32] {command}")

            if command == "START_OPENCV":
                print("[INFO] Starting mushroom detection...")
                mushroom_count = 0

                for i in range(10):
                    ret, frame = cap.read()
                    if not ret:
                        print("[WARN] Failed to capture image")
                        continue

                    found, conf = detect_mushroom(frame)
                    label = f"Conf: {conf:.2f}"

                    h, w, _ = frame.shape
                    color = (0, 255, 0) if found else (0, 0, 255)
                    cv2.rectangle(frame, (int(w*0.2), int(h*0.2)), (int(w*0.8), int(h*0.8)), color, 2)
                    cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

                    print(f"  Frame {i+1}: {'Mushroom' if found else 'None'} (conf={conf:.2f})")
                    if found:
                        mushroom_count += 1

                    time.sleep(0.3)

                if mushroom_count >= 6:
                    ser.write(b"mushroom_status:1\n")
                    print("[RESULT] Mushroom FOUND")
                else:
                    ser.write(b"mushroom_status:0\n")
                    print("[RESULT] No mushroom detected")

            elif command == "exit":
                print("[INFO] Exiting loop...")
                break

    except (OSError, serial.SerialException):
        print("[ERROR] Serial disconnected! Attempting to reconnect...")
        try:
            ser.close()
        except Exception:
            pass
        ser = connect_serial()
        print("[INFO] Reconnected to serial port!")

# =======================
# CLEANUP
# =======================
cap.release()
ser.close()
print("[DONE] Closed all connections.")
