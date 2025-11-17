import serial
import time
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from datetime import datetime

# ========== CONFIG ==========
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
MODEL_PATH = 'mushroom_model.tflite'
CONFIDENCE_THRESHOLD = 0.5
NMS_IOU_THRESHOLD = 0.45
CAMERA_ID = 0  # 0 = default camera

CLASSES = ["harvest", "pinHeads", "tinyButton"]  # from your labels.txt

# ========== SETUP ==========
print("[INFO] Loading TensorFlow Lite model...")
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
print("[INFO] Model input details:", input_details[0])
print("[INFO] Model output details:", output_details[0])

print("[INFO] Opening serial connection...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)
ser.flushInput()

cap = cv2.VideoCapture(CAMERA_ID)
if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

print("[READY] Waiting for command from ESP32...")

# ========== DETECTION HELPERS ==========
def decode_output(output_tensor, model_w, model_h, conf_thres=0.5):
    attrs, num_boxes = output_tensor.shape
    if attrs != 7:
        print("[ERROR] Unexpected output shape. Expected (7, N).")
        return [], [], [], "INVALID"

    boxes, confidences, class_ids = [], [], []

    for i in range(num_boxes):
        cx = float(output_tensor[0, i])
        cy = float(output_tensor[1, i])
        w = float(output_tensor[2, i])
        h = float(output_tensor[3, i])

        if w <= 0 or h <= 0:
            continue
        if cx - w/2 < 0 or cy - h/2 < 0:
            continue

        scores = output_tensor[4:, i]
        cls_id = int(np.argmax(scores))
        conf = float(scores[cls_id])
        if conf < conf_thres:
            continue

        x1 = max(0, cx - w/2)
        y1 = max(0, cy - h/2)
        x2 = cx + w/2
        y2 = cy + h/2

        boxes.append([x1, y1, x2, y2])
        confidences.append(conf)
        class_ids.append(cls_id)

    return boxes, confidences, class_ids, "xywh_no_objectness"

def run_inference_on_frame(frame):
    input_shape = input_details[0]['shape']
    if len(input_shape) == 4:
        if input_shape[1] == 3:  # [1,3,H,W]
            model_h = int(input_shape[2])
            model_w = int(input_shape[3])
        else:
            model_h = int(input_shape[1])
            model_w = int(input_shape[2])
    else:
        model_h = frame.shape[0]
        model_w = frame.shape[1]

    resized = cv2.resize(frame, (model_w, model_h))
    if input_details[0]['dtype'] == np.float32:
        input_data = np.expand_dims(resized.astype(np.float32)/255.0, axis=0)
    else:
        input_data = np.expand_dims(resized, axis=0)

    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])[0]

    boxes, confs, cls_ids, mode = decode_output(output_data, model_w, model_h)

    # NMS
    indices = []
    if len(boxes) > 0:
        try:
            indices = cv2.dnn.NMSBoxes(boxes, confs, CONFIDENCE_THRESHOLD, NMS_IOU_THRESHOLD)
            if isinstance(indices, np.ndarray):
                indices = indices.flatten().tolist()
            elif len(indices) and isinstance(indices[0], (list, tuple, np.ndarray)):
                indices = [int(i[0]) for i in indices]
            else:
                indices = [int(i) for i in indices]
        except Exception as e:
            print("[WARN] NMS failed:", e)
            indices = list(range(len(boxes)))

    detections = [(boxes[i], confs[i], cls_ids[i]) for i in indices]
    return detections, mode

# ========== MAIN LOOP ==========
while True:
    if ser.in_waiting:
        command = ser.readline().decode(errors='ignore').strip()
        print(f"[{datetime.now().strftime('%H:%M:%S')}] [ESP32] {command}")

        if command == "START_OPENCV":
            print("[INFO] Starting detection pass...")
            positive_count = 0

            for i in range(10):
                ret, frame = cap.read()
                if not ret:
                    print("[WARN] Failed to capture image")
                    continue

                detections, decode_mode = run_inference_on_frame(frame)

                '''for (box, score, cid) in detections:
                    if score >= CONFIDENCE_THRESHOLD:
                        positive_count += 1'''

                print(f"[DEBUG] Frame {i+1}: {len(detections)} detections, positive_count={positive_count}")
                time.sleep(0.25)
            if len(detections) != 0:
                positive_count += 1
            
            if positive_count >= 6:
                ser.write(b"mushroom_status:1\n")
                print("[RESULT] Detected (>=6 positive detections)")
            else:
                ser.write(b"mushroom_status:0\n")
                print("[RESULT] Not detected")

        elif command == "exit":
            print("[INFO] Exiting loop...")
            break

# Cleanup
cap.release()
ser.close()
print("[DONE] Closed all connections.")
