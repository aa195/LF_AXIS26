// ================================================================
//  AUTOBOT — AXIS'26 VNIT NAGPUR
//  Finite State Machine implementation
//
//  BTN1 = Cycle through modes (press repeatedly to select)
//  BTN2 = Confirm and START selected mode
//
//  MODES (cycle with BTN1):
//    0 - LINE FOLLOW ONLY  (Round 1)
//    1 - LSRB DRY RUN      (Round 2 explore, left priority)
//    2 - LSRB FINAL RUN    (Round 2 replay, left priority)
//    3 - RSLB DRY RUN      (Round 2 explore, right priority)
//    4 - RSLB FINAL RUN    (Round 2 replay, right priority)
//
//  FSM STATES (active during loop only):
//    FOLLOWING    → PID line following
//    AT_JUNCTION  → nudge forward, classify, decide
//    TURNING      → execute the turn
//    DEAD_END     → pivot to recover
//    FINISHED     → blink LEDs forever
// ================================================================

#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

// ================================================================
//  SENSOR PINS
// ================================================================
#define SENSOR_COUNT 8
const uint8_t qtrPins[SENSOR_COUNT] = {36, 39, 34, 35, 32, 33, 25, 26};

// ================================================================
//  MOTOR PINS
// ================================================================
#define AIN1 18
#define AIN2 5
#define PWMA 4
#define BIN1 19
#define BIN2 21
#define PWMB 22
#define STBY -1

Motor motorA = Motor(AIN1, AIN2, PWMA, -1, STBY);
Motor motorB = Motor(BIN1, BIN2, PWMB, -1, STBY);

// ================================================================
//  BUTTON & LED PINS
// ================================================================
#define LED_LEFT  13
#define LED_RIGHT 14
#define BTN1      12
#define BTN2      27

// ================================================================
//  SPEEDS
// ================================================================
#define BASE_SPEED  180
#define TURN_SPEED  120
#define TURN_SLOW   100
#define BACK_SPEED  150
#define CALIB_SPEED 80

// ================================================================
//  PID CONSTANTS
// ================================================================
float Kp = 0.054;
float Ki = 0.0001;
float Kd = 0.54;

float lastError = 0;
float integral  = 0;

// ================================================================
//  SENSOR STATE
// ================================================================
QTRSensors qtr;
uint16_t   sensorValues[SENSOR_COUNT];
uint16_t   rawValues[SENSOR_COUNT];
uint16_t   rawMin[SENSOR_COUNT];
uint16_t   rawMax[SENSOR_COUNT];
uint16_t   thresholds[SENSOR_COUNT];

// ================================================================
//  MODE DEFINITIONS
// ================================================================
#define MODE_LINE_FOLLOW  0
#define MODE_LSRB_DRY    1
#define MODE_LSRB_FINAL  2
#define MODE_RSLB_DRY    3
#define MODE_RSLB_FINAL  4

int selectedMode = 0;

// ================================================================
//  PATH MEMORY
// ================================================================
#define MAX_PATH 50
char path[MAX_PATH];
int  pathLength  = 0;
int  replayIndex = 0;

// ================================================================
//  FSM STATE DEFINITIONS
// ================================================================
enum BotState {
  STATE_FOLLOWING,
  STATE_AT_JUNCTION,
  STATE_TURNING,
  STATE_DEAD_END,
  STATE_FINISHED
};

BotState currentState  = STATE_FOLLOWING;
char pendingDecision = 0;

// ================================================================
//  FORWARD DECLARATIONS
// ================================================================
void  runFollowing();
void  runAtJunction();
void  runTurning();
void  runDeadEnd();
void  runFinished();

void  calibrateSensors();
void  readSensors();
int   countActive();
bool  leftActive();
bool  rightActive();
bool  centerActive();
bool  isJunction();
bool  isDeadEnd();

void  followLinePID();
void  resetPID();

char  decideJunction(bool useLSRB);
void  optimizePath();

void  doTurnLeft();
void  doTurnRight();
void  doTurnBack();

void  setLeft(int s);
void  setRight(int s);
void  stopMotors();
void  setLED(char direction);
void  clearLEDs();
void  showModeIndicator(int mode);

// ================================================================
//  SETUP
//  1. Auto-calibrate (bot spins to sweep sensors)
//  2. Mode selection loop (BTN1 cycle, BTN2 confirm)
//  3. Start FSM
// ================================================================
void setup() {
  pinMode(BTN1,      INPUT_PULLUP);
  pinMode(BTN2,      INPUT_PULLUP);
  pinMode(LED_LEFT,  OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins(qtrPins, SENSOR_COUNT);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    rawMin[i] = 4095;
    rawMax[i] = 0;
  }

  // Power-on blink
  digitalWrite(LED_LEFT,  HIGH);
  digitalWrite(LED_RIGHT, HIGH);
  delay(300);
  digitalWrite(LED_LEFT,  LOW);
  digitalWrite(LED_RIGHT, LOW);

  calibrateSensors();

  // Calibration done — triple blink
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_LEFT,  HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    delay(150);
    digitalWrite(LED_LEFT,  LOW);
    digitalWrite(LED_RIGHT, LOW);
    delay(150);
  }

  // ---- MODE SELECTION ----
  showModeIndicator(selectedMode);

  while (true) {
    if (digitalRead(BTN1) == LOW) {
      selectedMode = (selectedMode + 1) % 5;
      showModeIndicator(selectedMode);
      delay(350);
    }
    if (digitalRead(BTN2) == LOW) {
      for (int i = 0; i < 2; i++) {
        digitalWrite(LED_LEFT,  HIGH);
        digitalWrite(LED_RIGHT, HIGH);
        delay(200);
        digitalWrite(LED_LEFT,  LOW);
        digitalWrite(LED_RIGHT, LOW);
        delay(200);
      }
      delay(300);
      break;
    }
  }

  clearLEDs();
  resetPID();
  currentState = STATE_FOLLOWING;
}

// ================================================================
//  MAIN LOOP — FSM dispatcher
// ================================================================
void loop() {
  readSensors();

  switch (currentState) {
    case STATE_FOLLOWING:   runFollowing();   break;
    case STATE_AT_JUNCTION: runAtJunction();  break;
    case STATE_TURNING:     runTurning();     break;
    case STATE_DEAD_END:    runDeadEnd();     break;
    case STATE_FINISHED:    runFinished();    break;
  }
}

// ================================================================
//  STATE: FOLLOWING
// ================================================================
void runFollowing() {
  if (selectedMode == MODE_LINE_FOLLOW) {
    followLinePID();
    return;
  }

  if (isDeadEnd()) {
    currentState = STATE_DEAD_END;
    return;
  }

  if (isJunction()) {
    currentState = STATE_AT_JUNCTION;
    return;
  }

  followLinePID();
}

// ================================================================
//  STATE: AT_JUNCTION
//
//  Nudge forward to clear junction body.
//  If sensors still all black after nudge → finish line.
//  Otherwise classify, decide, store in pendingDecision.
//  Transition to TURNING.
// ================================================================
void runAtJunction() {
  // Nudge forward until edge sensors clear
  unsigned long nudgeTimeout = millis() + 700;
  while ((leftActive() || rightActive()) && millis() < nudgeTimeout) {
    qtr.read(rawValues);
    setLeft(BASE_SPEED);
    setRight(BASE_SPEED);
  }

  // Fresh read after nudge
  qtr.read(rawValues);

  // All sensors still black = finish line
  if (countActive() >= 7) {
    stopMotors();
    currentState = STATE_FINISHED;
    return;
  }

  // Decide
  bool useLSRB = (selectedMode == MODE_LSRB_DRY || selectedMode == MODE_LSRB_FINAL);
  char decision;

  if (selectedMode == MODE_LSRB_FINAL || selectedMode == MODE_RSLB_FINAL) {
    if (replayIndex < pathLength) {
      decision = path[replayIndex++];
    } else {
      currentState = STATE_FOLLOWING;
      return;
    }
  } else if (selectedMode == MODE_LSRB_DRY || selectedMode == MODE_RSLB_DRY) {
    decision = decideJunction(useLSRB);
    if (pathLength < MAX_PATH) {
      path[pathLength++] = decision;
      optimizePath();
    }
  } else {
    decision = 'S';
  }

  pendingDecision = decision;
  setLED(decision);
  currentState = STATE_TURNING;
}

// ================================================================
//  STATE: TURNING
//
//  Executes pendingDecision turn.
//  'S' does nothing — nudge already done.
//  Transitions back to FOLLOWING when complete.
// ================================================================
void runTurning() {
  switch (pendingDecision) {
    case 'L': doTurnLeft();  break;
    case 'R': doTurnRight(); break;
    case 'U': doTurnBack();  break;
    case 'S':                break;
  }

  clearLEDs();
  resetPID();
  currentState = STATE_FOLLOWING;
}

// ================================================================
//  STATE: DEAD_END
//
//  Records 'U' in dry run and optimizes path.
//  Pivots until center finds line.
//  Transitions back to FOLLOWING.
// ================================================================
void runDeadEnd() {
  bool useLSRB = (selectedMode == MODE_LSRB_DRY || selectedMode == MODE_LSRB_FINAL);

  if (selectedMode == MODE_LSRB_DRY || selectedMode == MODE_RSLB_DRY) {
    if (pathLength < MAX_PATH) {
      path[pathLength++] = 'U';
      optimizePath();
    }
  }

  unsigned long timeout = millis() + 3000;
  while (!centerActive() && millis() < timeout) {
    qtr.read(rawValues);
    if (useLSRB) { setLeft(-TURN_SLOW); setRight(TURN_SPEED); }
    else          { setLeft(TURN_SPEED); setRight(-TURN_SLOW); }
  }

  stopMotors();
  delay(60);
  resetPID();
  currentState = STATE_FOLLOWING;
}

// ================================================================
//  STATE: FINISHED
//  Blink both LEDs forever.
// ================================================================
void runFinished() {
  stopMotors();
  digitalWrite(LED_LEFT,  HIGH);
  digitalWrite(LED_RIGHT, HIGH);
  delay(400);
  digitalWrite(LED_LEFT,  LOW);
  digitalWrite(LED_RIGHT, LOW);
  delay(400);
}

// ================================================================
//  TURN FUNCTIONS
// ================================================================
void doTurnLeft() {
  if (centerActive()) {
    unsigned long t = millis() + 1500;
    while (centerActive() && millis() < t) {
      qtr.read(rawValues);
      setLeft(-TURN_SLOW);
      setRight(TURN_SPEED);
    }
  }
  unsigned long t = millis() + 2000;
  while (!centerActive() && millis() < t) {
    qtr.read(rawValues);
    setLeft(-TURN_SLOW);
    setRight(TURN_SPEED);
  }
  stopMotors();
  delay(50);
}

void doTurnRight() {
  if (centerActive()) {
    unsigned long t = millis() + 1500;
    while (centerActive() && millis() < t) {
      qtr.read(rawValues);
      setLeft(TURN_SPEED);
      setRight(-TURN_SLOW);
    }
  }
  unsigned long t = millis() + 2000;
  while (!centerActive() && millis() < t) {
    qtr.read(rawValues);
    setLeft(TURN_SPEED);
    setRight(-TURN_SLOW);
  }
  stopMotors();
  delay(50);
}

void doTurnBack() {
  bool useLSRB = (selectedMode == MODE_LSRB_DRY || selectedMode == MODE_LSRB_FINAL);

  // setLeft(-BACK_SPEED);
  // setRight(-BACK_SPEED);
  // delay(250);
  // stopMotors();
  // delay(60);

  // if (centerActive()) {
  //   unsigned long t = millis() + 1500;
  //   while (centerActive() && millis() < t) {
  //     qtr.read(rawValues);
  //     if (useLSRB) { setLeft(-TURN_SLOW); setRight(TURN_SPEED); }
  //     else          { setLeft(TURN_SPEED); setRight(-TURN_SLOW); }
  //   }
  // }
  unsigned long t = millis() + 2500;
  while (!centerActive() && millis() < t) {
    qtr.read(rawValues);
    if (useLSRB) { setLeft(-TURN_SLOW); setRight(TURN_SPEED); }
    else          { setLeft(TURN_SPEED); setRight(-TURN_SLOW); }
  }
  stopMotors();
  delay(50);
}

// ================================================================
//  JUNCTION DECISION
// ================================================================
char decideJunction(bool useLSRB) {
  bool le = leftActive();
  bool re = rightActive();
  bool ce = centerActive();

  if (useLSRB) {
    if (le) return 'L';
    if (ce) return 'S';
    if (re) return 'R';
    return 'U';
  } else {
    if (re) return 'R';
    if (ce) return 'S';
    if (le) return 'L';
    return 'U';
  }
}

// ================================================================
//  PATH OPTIMIZATION
// ================================================================
void optimizePath() {
  if (pathLength < 3 || path[pathLength - 2] != 'U')
    return;

  int totalAngle = 0;
  for (int m = 1; m <= 3; m++) {
    switch (path[pathLength - m]) {
      case 'R': totalAngle += 90;  break;
      case 'L': totalAngle += 270; break;
      case 'U': totalAngle += 180; break;
      case 'S': totalAngle += 0;   break;
    }
  }
  totalAngle %= 360;

  switch (totalAngle) {
    case 0:   path[pathLength - 3] = 'S'; break;
    case 90:  path[pathLength - 3] = 'R'; break;
    case 180: path[pathLength - 3] = 'U'; break;
    case 270: path[pathLength - 3] = 'L'; break;
  }
  pathLength -= 2;
}

// ================================================================
//  PID LINE FOLLOWING
// ================================================================
void followLinePID() {
  int position = qtr.readLineBlack(sensorValues);
  int error    = 3500 - position;

  integral += error;
  integral  = constrain(integral, -5000, 5000);

  float derivative = error - lastError;
  float output     = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int leftSpeed  = constrain(BASE_SPEED + (int)output, -255, 255);
  int rightSpeed = constrain(BASE_SPEED - (int)output, -255, 255);

  setLeft(leftSpeed);
  setRight(rightSpeed);
}

void resetPID() {
  lastError = 0;
  integral  = 0;
}

// ================================================================
//  SENSOR HELPERS
// ================================================================

void calibrateSensors() {
  // ---- AUTO CALIBRATION ----
  for (int i = 0; i < 400; i++) {
    setLeft(CALIB_SPEED);
    setRight(-CALIB_SPEED);

    qtr.calibrate();
    uint16_t temp[SENSOR_COUNT];
    qtr.read(temp);
    for (int j = 0; j < SENSOR_COUNT; j++) {
      if (temp[j] < rawMin[j]) rawMin[j] = temp[j];
      if (temp[j] > rawMax[j]) rawMax[j] = temp[j];
    }
    delay(10);
  }

  stopMotors();

  // Compute thresholds
  for (int i = 0; i < SENSOR_COUNT; i++)
    thresholds[i] = (rawMin[i] + rawMax[i]) / 2;
}

void readSensors() {
  qtr.read(rawValues);
}

int countActive() {
  int c = 0;
  for (int i = 0; i < SENSOR_COUNT; i++)
    if (rawValues[i] > thresholds[i]) c++;
  return c;
}

bool rightActive()  { return rawValues[0] > thresholds[0] || rawValues[1] > thresholds[1]; }
bool leftActive()   { return rawValues[6] > thresholds[6] || rawValues[7] > thresholds[7]; }
bool centerActive() { return rawValues[3] > thresholds[3] || rawValues[4] > thresholds[4]; }
bool isDeadEnd()    { return countActive() == 0; }

bool isJunction() {
  bool le = leftActive();
  bool re = rightActive();
  int  ac = countActive();
  if (le && re)      return true;
  if (le && ac >= 4) return true;
  if (re && ac >= 4) return true;
  return false;
}

// ================================================================
//  MODE INDICATOR
// ================================================================
void showModeIndicator(int mode) {
  clearLEDs();
  delay(100);
  switch (mode) {
    case MODE_LINE_FOLLOW:
      digitalWrite(LED_LEFT,  HIGH);
      digitalWrite(LED_RIGHT, HIGH);
      break;
    case MODE_LSRB_DRY:
      digitalWrite(LED_LEFT, HIGH);
      break;
    case MODE_LSRB_FINAL:
      for (int i = 0; i < 2; i++) {
        digitalWrite(LED_LEFT, HIGH); delay(200);
        digitalWrite(LED_LEFT, LOW);  delay(200);
      }
      break;
    case MODE_RSLB_DRY:
      digitalWrite(LED_RIGHT, HIGH);
      break;
    case MODE_RSLB_FINAL:
      for (int i = 0; i < 2; i++) {
        digitalWrite(LED_RIGHT, HIGH); delay(200);
        digitalWrite(LED_RIGHT, LOW);  delay(200);
      }
      break;
  }
}

// ================================================================
//  LED HELPERS
// ================================================================
void setLED(char direction) {
  clearLEDs();
  if (direction == 'L') digitalWrite(LED_LEFT,  HIGH);
  if (direction == 'R') digitalWrite(LED_RIGHT, HIGH);
}

void clearLEDs() {
  digitalWrite(LED_LEFT,  LOW);
  digitalWrite(LED_RIGHT, LOW);
}

// ================================================================
//  MOTOR HELPERS
// ================================================================
void setLeft(int s)  { motorA.drive(s); }
void setRight(int s) { motorB.drive(s); }
void stopMotors()    { motorA.brake(); motorB.brake(); }
