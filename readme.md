# IoT Mushroom Farm Controller

An integrated system to monitor and manage a mushroom farm environment, combining hardware, AI-based vision, and cloud‑dashboard control via Blynk.

## 🧩 Project Overview

This project uses:

* A microcontroller (e.g., Espressif ESP32) to control relays for heater, fan, mist, LED.
* A distance sensor (e.g., Adafruit VL53L0X) and environmental sensor (e.g., DHT DHT22) on the ESP32.
* A Raspberry Pi running a camera + TensorFlow Lite model to detect mushrooms via USB serial communication.
* Blynk dashboard for remote monitoring and manual control (auto/manual mode).
* A systemd service on the Pi for automatic startup and reliable operation.

---

## 🎯 How It Works

1. The ESP32 boots, connects to WiFi, and links to Blynk cloud.
2. It reads temperature/humidity and distance values, sends these to Blynk virtual pins.
3. If configured in *Auto Mode*, the ESP32 will control relays based on sensor values.
4. When the distance sensor triggers (object nearer than threshold), the ESP32 sends `START_OPENCV` via serial to the Raspberry Pi.
5. The Pi captures a sequence of N frames, runs each through the TFLite model to detect mushrooms.
6. If ≥ M of N frames detect a mushroom → Pi sends back `mushroom_status:1`, else `…:0`.
7. The ESP32 receives the status and updates Blynk (display found status) and possibly triggers further control logic.
8. All components log their status and can be viewed via Blynk or via log files on the Pi.

---

## 🧰 Hardware & Wiring

| Component               | Connection Summary                                                      |
| ----------------------- | ----------------------------------------------------------------------- |
| ESP32                   | Relays on GPIOs 13/14/15/5, distance sensor on GPIO 34, DHT22 on GPIO 4 |
| VL53L0X distance sensor | I2C to ESP32 (SDA/SCL)                                                  |
| DHT22 sensor            | Single‑wire to ESP32 GPIO 4                                             |
| Raspberry Pi            | USB connection to ESP32 for serial; camera connected via CSI or USB     |
| Relays                  | Active‑LOW logic: write LOW to turn on                                  |

---

## 🔧 Software Setup

### On Raspberry Pi

1. Clone repository:

   ```bash
   git clone https://github.com/9‑Fakrizz/IoT_Mushroom_Blynk.git
   cd IoT_Mushroom_Blynk
   ```
2. Create and activate Python venv:

   ```bash
   python3 ‑m venv venv
   source venv/bin/activate
   pip install ‑r requirements.txt
   ```
3. Ensure correct serial device (e.g., `/dev/ttyUSB0`). Update `SERIAL_PORT` in `main_mushroom.py`.
4. Test run:

   ```bash
   source venv/bin/activate
   python main_mushroom.py
   ```

   Confirm it waits for serial command from ESP32.
5. (Optional) Setup systemd service for auto‑start:

   ```ini
   /etc/systemd/system/mushroom.service

   [Unit]
   Description=Mushroom Detection Service
   After=network.target

   [Service]
   WorkingDirectory=/home/ussy/IoT_Mushroom_Blynk
   ExecStart=/bin/bash ‑c 'source /home/ussy/IoT_Mushroom_Blynk/venv/bin/activate && python main_mushroom.py'
   Restart=always
   RestartSec=5
   User=ussy
   StandardOutput=append:/home/ussy/mushroom.log
   StandardError=append:/home/ussy/mushroom.log

   [Install]
   WantedBy=multi‑user.target
   ```

   Then:

   ```bash
   sudo systemctl daemon‑reload
   sudo systemctl enable mushroom.service
   sudo systemctl start mushroom.service
   ```

   View live logs:

   ```bash
   tail ‑f /home/ussy/mushroom.log
   ```

### On ESP32 (Arduino IDE / PlatformIO)

1. Open `main_esp32.ino`.
2. Update WiFi, Blynk Template ID & Token, pin definitions.
3. Upload to your ESP32.
4. In Blynk app: create dashboard with virtual pins:

   * V0: temperature
   * V1: humidity
   * V2: distance
   * V3: Mist relay
   * V4: Fan relay
   * V5: Heater relay
   * V6: LED relay
   * V7: Mushroom found status
   * V8: Auto/Manual mode switch
   * V9: Manual “Check Mushroom” button

---

## 🐞 Known Issues & Troubleshooting

* **Serial port changes**: USB‑serial device can appear as `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc. Use `ls /dev/ttyUSB*` to check. Consider a udev rule for a stable symlink (e.g., `/dev/esp32`).
* **Input/output error on flush**: If the USB device disconnects or is not yet ready, `ser.flushInput()` may crash. Use a retry loop and catch exceptions.
* **No GUI support on Pi**: If running headless (no display), remove or comment out any `cv2.imshow()` or `cv2.waitKey()` calls, as they will fail.
* **Blynk offline**: Ensure your internet is up, credentials are correct, and your device is registered under the correct Template ID.
* **Relays behave backward**: Confirm if your relay board uses active‑LOW logic (i.e., `LOW` means ON). Adjust code accordingly.

---

## 🧮 How to Use

1. Boot both ESP32 and Raspberry Pi. The Pi service will auto‑start and wait for commands.
2. On the Blynk app: choose Auto or Manual.

   * In Manual: you can toggle relays or press “Check Mushroom” (V9) to manually trigger the detection.
   * In Auto: the distance sensor and temperature/humidity logic trigger the detection and control automatically.
3. When the Pi sends a `mushroom_status` result, the Blynk widget updates (green/red indicator).
4. Monitor logs via `journalctl` or `tail ‑f mushroom.log` to debug or track behavior.

---

## 📦 Repository Structure

```
IoT_Mushroom_Blynk/
  ├─ main_mushroom.py         # Raspberry Pi Python script for detection & serial communication  
  ├─ main_esp32.ino           # ESP32 Arduino sketch for sensors & Blynk  
  ├─ camtflite.py / cam.py     # Additional camera scripts or tests  
  ├─ requirements.txt         # Python dependencies (e.g., tflite‐runtime, pyserial, opencv)  
  ├─ readme.md                # This documentation  
  └─ …                        # Other folders/files (e.g., model files, images)  
```

---

## 🧠 Tips & Extensions

* Replace the binary classifier model with an object detection model (e.g., YOLO, SSD) for bounding boxes.
* Add WiFi/OTA capabilities to ESP32 so you can update firmware remotely.
* Store data (temperature, humidity, distance, detection events) in a database or cloud for analytics (Grafana, MQTT, etc.).
* Add alerting: send email/SMS when mushroom detection triggers or environmental thresholds exceed limits.

---

Thank you for checking out this project!
Feel free to improve, fork, and share your enhancements.

---
