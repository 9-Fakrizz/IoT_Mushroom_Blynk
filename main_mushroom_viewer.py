import serial
import time
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from datetime import datetime

# =======================
# CONFIGURATION
# =======================
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
MODEL_PATH = 'mushroom_model.tflite'
CONFIDENCE_THRESHOLD = 0.5
CAMERA_ID = 0  # 0 = default camera

# =======================
# INITIAL SETUP
# =======================
print("[INFO] Loading TensorFlow Lite model...")
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

print("[INFO] Opening serial connection...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)
ser.flushInput()

cap = cv2.VideoCapture(CAMERA_ID)
if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

print("[READY] Waiting for command from ESP32...")

# =======================
# INFERENCE FUNCTION
# =======================
def detect_mushroom(frame):
    input_shape = input_details[0]['shape']
    resized = cv2.resize(frame, (input_shape[2], input_shape[1]))
    input_data = np.expand_dims(resized, axis=0).astype(np.float32) / 255.0

    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    output_data = interpreter.get_tensor(output_details[0]['index'])[0]
    confidence = float(output_data if np.isscalar(output_data) else output_data[0])

    return confidence >= CONFIDENCE_THRESHOLD, confidence

# =======================
# MAIN LOOP
# =======================
while True:
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

                # Draw bounding box
                h, w, _ = frame.shape
                color = (0, 255, 0) if found else (0, 0, 255)
                cv2.rectangle(frame, (int(w*0.2), int(h*0.2)), (int(w*0.8), int(h*0.8)), color, 2)
                cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

                print(f"  Frame {i+1}: {'Mushroom' if found else 'None'} (conf={conf:.2f})")
                if found:
                    mushroom_count += 1

                # ===== SHOW IMAGE WITH BOUNDING BOX =====
                cv2.imshow("Mushroom Detection", frame)
                # Press ESC to stop preview early
                if cv2.waitKey(1) & 0xFF == 27:
                    break

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

# Cleanup
cap.release()
ser.close()
cv2.destroyAllWindows()
print("[DONE] Closed all connections.")
