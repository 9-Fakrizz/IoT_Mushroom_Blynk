import serial
import time
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite


# =======================
# CONFIGURATION
# =======================
SERIAL_PORT = '/dev/ttyUSB0'   # change to /dev/ttyACM0 if needed
BAUD_RATE = 115200
MODEL_PATH = 'mushroom_model.tflite'
CONFIDENCE_THRESHOLD = 0.5
CAMERA_ID = 0  # usually 0 for default camera

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
time.sleep(2)  # wait for ESP32 reset

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

    # Handle both binary and multi-output models
    if len(output_data.shape) == 0:
        confidence = float(output_data)
    else:
        confidence = float(output_data[0])

    return confidence >= CONFIDENCE_THRESHOLD, confidence


# =======================
# MAIN LOOP
# =======================
while True:
    if ser.in_waiting:
        command = ser.readline().decode().strip()
        print(f"[ESP32] {command}")

        if command == "START_OPENCV":
            print("[INFO] Starting mushroom detection...")
            mushroom_count = 0

            for i in range(5):
                ret, frame = cap.read()
                if not ret:
                    print("[WARN] Failed to capture image")
                    continue

                found, conf = detect_mushroom(frame)
                print(f"  Image {i+1}: {'Mushroom' if found else 'None'} (conf={conf:.2f})")

                if found:
                    mushroom_count += 1
                time.sleep(0.5)

            if mushroom_count >= 3:
                ser.write(b"mushroom_found\n")
                print("[RESULT] Mushroom FOUND")
            else:
                ser.write(b"no_mushroom\n")
                print("[RESULT] No mushroom")

        elif command == "exit":
            print("[INFO] Exiting...")
            break

# Cleanup
cap.release()
ser.close()
print("[DONE] Closed all connections.")
