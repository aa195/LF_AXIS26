#include <QTRSensors.h>

// ---------------- CONFIG ----------------
#define SENSOR_COUNT 8
#define LINE_THRESHOLD 200

QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

// ESP32 ADC pins (safe ones)
const uint8_t qtrPins[SENSOR_COUNT] = {36, 39, 34, 35, 32, 33, 25, 26};

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(1000); // allow serial to start

  Serial.println("ESP32 STARTED");

  qtr.setTypeAnalog();
  qtr.setSensorPins(qtrPins, SENSOR_COUNT);

  // 🔥 CALIBRATION
  Serial.println("Calibrating sensors...");
  for (int i = 0; i < 100; i++) {
    qtr.calibrate();
    delay(20);
  }
  Serial.println("Calibration done!");
}

// ---------------- LOOP ----------------
void loop() {

  Serial.println("---- NEW READING ----");

  // Read raw values
  qtr.read(sensorValues);

  // Print raw sensor values
  Serial.print("Sensors: ");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }

  // Calculate position (0 to 7000)
  uint16_t position = qtr.readLineBlack(sensorValues);

  Serial.print(" | Position: ");
  Serial.print(position);

  // ---------------- LINE DETECTION ----------------
  bool lineDetected = false;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] > LINE_THRESHOLD) {
      lineDetected = true;
      break;
    }
  }

  if (!lineDetected) {
    Serial.print(" | Line LOST");
  } else {
    Serial.print(" | Line OK");
  }

  Serial.println();

  delay(200);
}