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

# Color map for classes (BGR)
CLASS_COLORS = {
    "harvest": (0, 200, 0),
    "pinHeads": (0, 220, 220),
    "tinyButton": (255, 128, 0)
}

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
    """
    Decode your model output:
    output_tensor shape = (7, 8400)
    format:
        row 0 = x
        row 1 = y
        row 2 = w
        row 3 = h
        row 4 = class 0 score
        row 5 = class 1 score
        row 6 = class 2 score
    No objectness.
    """

    attrs, num_boxes = output_tensor.shape

    if attrs != 7:
        print("[ERROR] Unexpected output shape. Expected (7, N).")
        return [], [], [], "INVALID"

    print(f"[DEBUG] decode_output: attrs={attrs}, num_boxes={num_boxes}")

    # Prepare lists
    boxes = []
    confidences = []
    class_ids = []

    # Loop through all anchors
    for i in range(num_boxes):
        cx = float(output_tensor[0, i])
        cy = float(output_tensor[1, i])
        w = float(output_tensor[2, i])
        h = float(output_tensor[3, i])

        # Skip invalid boxes
        if w <= 0 or h <= 0:
            continue
        if cx - w/2 < 0 or cy - h/2 < 0:
            continue

        scores = output_tensor[4:, i]   # class scores
        cls_id = int(np.argmax(scores))
        conf = float(scores[cls_id])
        if conf < conf_thres:
            continue

        # Convert center -> top-left/bottom-right
        x1 = cx - w/2
        y1 = cy - h/2
        x2 = cx + w/2
        y2 = cy + h/2

        boxes.append([x1, y1, x2, y2])
        confidences.append(conf)
        class_ids.append(cls_id)

    return boxes, confidences, class_ids, "xywh_no_objectness"
    
def run_inference_on_frame(frame):
    # Prepare input
    input_shape = input_details[0]['shape']  # e.g. [1, h, w, 3] or [1,3,h,w] depending on model
    # Find width and height positions
    if len(input_shape) == 4:
        if input_shape[1] == 3:  # shape [1,3,H,W]
            model_h = int(input_shape[2])
            model_w = int(input_shape[3])
        else:
            model_h = int(input_shape[1])
            model_w = int(input_shape[2])
    else:
        # fallback
        model_h = frame.shape[0]
        model_w = frame.shape[1]

    resized = cv2.resize(frame, (model_w, model_h))
    # Normalize if float input
    if input_details[0]['dtype'] == np.float32:
        input_data = np.expand_dims(resized.astype(np.float32) / 255.0, axis=0)
    else:
        input_data = np.expand_dims(resized, axis=0)

    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    output_data = interpreter.get_tensor(output_details[0]['index'])[0]  # shape (attrs, num_boxes)
    print("[DEBUG] Output shape:", output_data.shape)
    # print small sample (avoid flooding)
    # print("[DEBUG] Output sample (first cols):", output_data[:, :6])

    # Decode to boxes, confidences, class_ids
    boxes, confs, cls_ids, mode = decode_output(output_data, model_w, model_h)

    # Filter by confidence threshold
    filtered_boxes = []
    filtered_scores = []
    filtered_class_ids = []
    for b, s, cid in zip(boxes, confs, cls_ids):
        if s >= CONFIDENCE_THRESHOLD:
            filtered_boxes.append(b)  # x,y,w,h
            filtered_scores.append(s)
            filtered_class_ids.append(cid)

    print(f"[DEBUG] {len(filtered_boxes)} boxes passed confidence >= {CONFIDENCE_THRESHOLD}")

    # Convert to format for NMS: boxes list of [x,y,w,h]
    indices = []
    if len(filtered_boxes) > 0:
        # cv2.dnn.NMSBoxes expects boxes as list of [x,y,w,h] and scores
        try:
            indices = cv2.dnn.NMSBoxes(filtered_boxes, filtered_scores, CONFIDENCE_THRESHOLD, NMS_IOU_THRESHOLD)
            # cv2 returns nested lists; normalize to flat list
            if isinstance(indices, np.ndarray):
                indices = indices.flatten().tolist()
            elif isinstance(indices, (list, tuple)):
                # sometimes returns list of lists like [[0],[2]] or [[0,1]]
                if len(indices) and isinstance(indices[0], (list, tuple, np.ndarray)):
                    indices = [int(i[0]) for i in indices]
                else:
                    indices = [int(i) for i in indices]
            else:
                indices = []
        except Exception as e:
            print("[WARN] NMS failed:", e)
            indices = list(range(len(filtered_boxes)))

    # Build final detections list
    detections = []
    for idx in indices:
        b = filtered_boxes[idx]
        score = filtered_scores[idx]
        cid = filtered_class_ids[idx]
        detections.append((b, score, cid))

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

                for (box, score, cid) in detections:
                    x1, y1, x2, y2 = box
                    cls_name = CLASSES[cid] if 0 <= cid < len(CLASSES) else f"cls{cid}"
                    color = CLASS_COLORS.get(cls_name, (0, 255, 0))
                    label = f"{cls_name}: {score:.2f}"

                    # Convert to int and clamp to image boundaries
                    x1 = max(0, int(x1))
                    y1 = max(0, int(y1))
                    x2 = min(frame.shape[1]-1, int(x2))
                    y2 = min(frame.shape[0]-1, int(y2))

                    # Skip invalid boxes
                    if x2 <= x1 or y2 <= y1:
                        continue

                    # Draw rectangle and label
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, label, (x1, max(15, y1-5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


                # If no detection, still show a central box like before and show decode mode
                if len(detections) == 0:
                    h_img, w_img = frame.shape[:2]
                    color = (0, 0, 255)
                    cv2.rectangle(frame, (int(w_img*0.2), int(h_img*0.2)), (int(w_img*0.8), int(h_img*0.8)), color, 2)
                    cv2.putText(frame, f"No det (mode={decode_mode})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                else:
                    positive_count += 1
				
                cv2.imshow("Detection Viewer", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

                time.sleep(0.25)

            print(f"positive count = {positive_count}")
            # Decide final result based on positive_count
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
cv2.destroyAllWindows()
print("[DONE] Closed all connections.")
