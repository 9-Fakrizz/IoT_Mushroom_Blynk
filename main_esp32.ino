#define BLYNK_TEMPLATE_ID "TMPL6Hh_F32MY"
#define BLYNK_TEMPLATE_NAME "MushroomFarm"
#define BLYNK_AUTH_TOKEN "7mQrS98cDi42pt0V4cWTT1RL6cuSfFQS"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_VL53L0X.h>
#include "DHT.h"

// ====== Pin definitions ======
#define RELAY1 13  // Heater
#define RELAY2 14  // LED
#define RELAY3 15  // Fan
#define RELAY4 5   // Mist
#define DHTPIN 27
#define DHTTYPE DHT11

// ====== Sensors ======
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
DHT dht(DHTPIN, DHTTYPE);

// ====== Wi-Fi & Blynk credentials ======
char ssid[] = "Ok";
char pass[] = "q12345678";

// ====== Variables ======
float temp = 0;
float humid = 0;
bool autoMode = false;
bool found_mushroom = false;
unsigned long lastSensorTime = 0;
const unsigned long sensorInterval = 2000; // 2 sec update
const unsigned long interval = 12UL * 60UL * 60UL * 1000UL; // 12 hours in milliseconds
unsigned long currentMillis;

// Relay states
bool relay1State = HIGH;
bool relay2State = HIGH;
bool relay3State = HIGH;
bool relay4State = HIGH;

// ====== Blynk Virtual Pins ======
#define VPIN_TEMP              V0
#define VPIN_HUMID             V1
#define VPIN_DIST              V2
#define VPIN_RELAY4_MIST       V3
#define VPIN_RELAY3_FAN        V4
#define VPIN_RELAY1_HEATER     V5
#define VPIN_RELAY2_led        V6
#define VPIN_FOUND             V7
#define VPIN_MODE              V8
#define VPIN_CHECK_MUSHROOM    V9

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  allRelaysOff();

  if (!lox.begin()) {
    Serial.println("VL53L0X not found. Check wiring!");
    while (1);
  }

  dht.begin();
  Serial.println("Sensors ready.");

  WiFi.begin(ssid, pass);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n WiFi Connected.");

  Blynk.config(BLYNK_AUTH_TOKEN);
  if (!Blynk.connect()) {
    Serial.println(" Blynk not connected yet, will retry automatically.");
  }

  Blynk.virtualWrite(V7, 0); // set mushroom status to 0
  currentMillis = millis();
}

// ====== Loop ======
void loop() {
  Blynk.run();

  // Read sensors every few seconds (non-blocking)
  if (millis() - lastSensorTime >= sensorInterval) {
    lastSensorTime = millis();
    readSensors();
  }

  // Listen for Pi messages
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    if (msg.startsWith("mushroom_status:")) {
      int status = msg.substring(msg.indexOf(':') + 1).toInt();
      if (status == 1) {
        Serial.println("Mushroom FOUND!");
        // update Blynk or LED here
        Blynk.virtualWrite(V7, 1);
        Blynk.logEvent("time_to_harvest", "A mushroom has been detected!");

      } else {
        Serial.println("Mushroom NOT found!");
        Blynk.virtualWrite(V7, 0);
      }
    }
  }

}

// ====== Read Sensors + Control Logic ======
void readSensors() {
  humid = dht.readHumidity();
  temp = dht.readTemperature();

  if (isnan(humid) || isnan(temp)) {
    Serial.println("DHT read failed.");    
    humid = 0;
    temp = 0;
    return;
  }

  Blynk.virtualWrite(VPIN_TEMP, temp);
  Blynk.virtualWrite(VPIN_HUMID, humid);

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  int distance = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;
  Blynk.virtualWrite(VPIN_DIST, distance);

  // --- Auto Control Logic ---
  if (autoMode) {
    // Check if 12 hours have passed
    if (millis() - currentMillis >= interval) {
      Serial.println("Hello 12 hr");
      digitalWrite(RELAY2, !digitalRead(RELAY2));
      Blynk.virtualWrite(V6, digitalRead(RELAY2));
      currentMillis = millis();
    }

    // temp auto
    if (temp > 35) {
      setRelay(RELAY3, 1, relay3State); // Fan ON
      setRelay(RELAY1, 0, relay1State); // Heater OFF
      Blynk.virtualWrite(V4, 1);
      Blynk.virtualWrite(V5, 0);
    } else if (temp < 30) {
      setRelay(RELAY1, 1, relay1State); // Heater ON
      setRelay(RELAY3, 0, relay3State); // Fan OFF
      Blynk.virtualWrite(V4, 0);
      Blynk.virtualWrite(V5, 1);
    } else {
      setRelay(RELAY1, 0, relay1State);
      setRelay(RELAY3, 0, relay3State);
      Blynk.virtualWrite(V4, 0);
      Blynk.virtualWrite(V5, 0);
    }
    
    // humid auto
    if (humid < 65){
      digitalWrite(RELAY4, 0);
      Blynk.virtualWrite(V3, 1);
    }else {
      digitalWrite(RELAY4, 1);
      Blynk.virtualWrite(V3, 0);
    }    
 
  }

  // --- Trigger Pi if mushroom close ---
  if (distance > 0 && distance < 70) {
    Serial.println("START_OPENCV");
    Serial.println("Sent to Pi: START_OPENCV");
  }
}

// ====== Blynk Callbacks ======
BLYNK_WRITE(VPIN_MODE) {
  autoMode = param.asInt();
  Serial.print("Mode: ");
  Serial.println(autoMode ? "AUTO" : "MANUAL");
}

BLYNK_WRITE(VPIN_RELAY1_HEATER) { if (!autoMode) setRelay(RELAY1, param.asInt(), relay1State); }
BLYNK_WRITE(VPIN_RELAY2_led)    { if (!autoMode) setRelay(RELAY2, param.asInt(), relay2State); }
BLYNK_WRITE(VPIN_RELAY3_FAN)    { if (!autoMode) setRelay(RELAY3, param.asInt(), relay3State); }
BLYNK_WRITE(VPIN_RELAY4_MIST)   { if (!autoMode) setRelay(RELAY4, param.asInt(), relay4State); }
BLYNK_WRITE(VPIN_CHECK_MUSHROOM)
{
  int state = param.asInt();  // 1 = check, 0 = ignore
  if (state == 1) {
    Serial.println("Manual mushroom check triggered!");
    checkMushroom();
  }
}

void checkMushroom() {
  Serial.println("Checking mushroom sensor...");
  Serial.println("START_OPENCV");
  Serial.println("Sent to Pi: START_OPENCV");
}

// ====== Helpers ======
void setRelay(int pin, int state, bool &relayState) {
  relayState = state ? LOW : HIGH; // Active LOW
  digitalWrite(pin, relayState);
}

void allRelaysOff() {
  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2, HIGH);
  digitalWrite(RELAY3, HIGH);
  digitalWrite(RELAY4, HIGH);
}
