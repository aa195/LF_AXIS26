#include <QTRSensors.h>

// ------------------- CONFIG -------------------
#define SENSOR_COUNT 8

QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];    // calibrated (0–1000) for position
uint16_t rawValues[SENSOR_COUNT];       // raw ADC (0–4095) for detection

// Dynamic thresholds (raw ADC scale)
uint16_t rawMin[SENSOR_COUNT];
uint16_t rawMax[SENSOR_COUNT];
uint16_t thresholds[SENSOR_COUNT];

// ------------------- SENSOR PINS -------------------
const uint8_t qtrPins[SENSOR_COUNT] = {36, 39, 34, 35, 32, 33, 25, 26};

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  qtr.setTypeAnalog();
  qtr.setSensorPins(qtrPins, SENSOR_COUNT);

  // Init raw min/max
  for (int i = 0; i < SENSOR_COUNT; i++) {
    rawMin[i] = 4095;
    rawMax[i] = 0;
  }

  // -------- CALIBRATION --------
  Serial.println("Calibrating... move robot over line and white surface");

  for (int i = 0; i < 400; i++) {
    qtr.calibrate();

    uint16_t temp[SENSOR_COUNT];
    qtr.read(temp);
    for (int j = 0; j < SENSOR_COUNT; j++) {
      if (temp[j] < rawMin[j]) rawMin[j] = temp[j];
      if (temp[j] > rawMax[j]) rawMax[j] = temp[j];
    }
    delay(10);
  }

  Serial.println("Calibration Done!");
  Serial.println();

  // -------- DYNAMIC THRESHOLDS --------
  Serial.println("=== Sensor Thresholds ===");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    thresholds[i] = (rawMin[i] + rawMax[i]) / 2;
    Serial.print("S"); Serial.print(i);
    Serial.print(" | Min: ");    Serial.print(rawMin[i]);
    Serial.print(" | Max: ");    Serial.print(rawMax[i]);
    Serial.print(" | Thresh: "); Serial.println(thresholds[i]);
  }
  Serial.println("=========================");
  Serial.println();
}

// ------------------- LOOP -------------------
void loop() {

  // -------- CALIBRATED READ for position --------
  qtr.readCalibrated(sensorValues);
  uint16_t position = qtr.readLineBlack(sensorValues);

  // -------- RAW READ for detection --------
  qtr.read(rawValues);

  // -------- PRINT RAW VALUES --------
  Serial.print("Raw:  ");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(rawValues[i]);
    Serial.print("\t");
  }
  Serial.println();

  // -------- PRINT CALIBRATED VALUES --------
  Serial.print("Cal:  ");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();

  // -------- PRINT BINARY (black/white per sensor) --------
  Serial.print("Line: ");
  int activeSensors = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    bool isBlack = rawValues[i] > thresholds[i];
    if (isBlack) activeSensors++;
    Serial.print(isBlack ? "B" : "W");
    Serial.print("\t");
  }
  Serial.println();

  // -------- PRINT POSITION & ACTIVE COUNT --------
  Serial.print("Pos: "); Serial.print(position);
  Serial.print("  | Active Sensors: "); Serial.print(activeSensors);

  if (activeSensors == 0) {
    Serial.print("  | LINE LOST");
  } else {
    Serial.print("  | LINE OK");
  }

  Serial.println();
  Serial.println("-----------------------------");

  delay(200);
}