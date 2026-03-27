#include <SparkFun_TB6612.h>

// ------------------- MOTOR PINS -------------------
#define AIN1 4
#define AIN2 2
#define PWMA 15

#define BIN1 19
#define BIN2 18
#define PWMB 21

#define STBY -1   // STBY tied to 3.3V

// ------------------- MOTOR OFFSET -------------------
// Change to -1 if a motor runs backwards
#define OFFSET_A  1
#define OFFSET_B  1

Motor motorA = Motor(AIN1, AIN2, PWMA, OFFSET_A, STBY);
Motor motorB = Motor(BIN1, BIN2, PWMB, OFFSET_B, STBY);

// ------------------- SPEED CONFIG -------------------
#define SPEED_LOW     80    // ~30%
#define SPEED_MID     180   // ~70%
#define SPEED_HIGH    242   // ~95%
#define TEST_DELAY    2000  // ms per test step

// ------------------- HELPERS -------------------
void runBoth(int speed) {
  motorA.drive(speed);
  motorB.drive(speed);
}

void stopAll() {
  motorA.brake();
  motorB.brake();
}

void pause(int ms, const char* label) {
  Serial.print("  >> "); Serial.println(label);
  delay(ms);
  stopAll();
  Serial.println("  >> STOP");
  delay(500);
}

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("==============================");
  Serial.println("   TB6612FNG Motor Test");
  Serial.println("==============================");
  Serial.println();
}

// ------------------- LOOP -------------------
void loop() {

  // ======== TEST 1: Forward speeds ========
  Serial.println("[ TEST 1 ] Forward - Low Speed (30%)");
  runBoth(SPEED_LOW);
  pause(TEST_DELAY, "Both forward at LOW");

  Serial.println("[ TEST 2 ] Forward - Mid Speed (70%)");
  runBoth(SPEED_MID);
  pause(TEST_DELAY, "Both forward at MID");

  Serial.println("[ TEST 3 ] Forward - High Speed (95%)");
  runBoth(SPEED_HIGH);
  pause(TEST_DELAY, "Both forward at HIGH");

  // ======== TEST 2: Reverse ========
  Serial.println("[ TEST 4 ] Reverse - Mid Speed");
  runBoth(-SPEED_MID);
  pause(TEST_DELAY, "Both reverse at MID");

  // ======== TEST 3: Individual motors ========
  Serial.println("[ TEST 5 ] Motor A only - Forward");
  motorA.drive(SPEED_MID);
  motorB.brake();
  pause(TEST_DELAY, "Motor A only");

  Serial.println("[ TEST 6 ] Motor B only - Forward");
  motorA.brake();
  motorB.drive(SPEED_MID);
  pause(TEST_DELAY, "Motor B only");

  // ======== TEST 4: Turning ========
  Serial.println("[ TEST 7 ] Turn Left (A back, B forward)");
  motorA.drive(-SPEED_MID);
  motorB.drive(SPEED_MID);
  pause(TEST_DELAY, "Pivot Left");

  Serial.println("[ TEST 8 ] Turn Right (A forward, B back)");
  motorA.drive(SPEED_MID);
  motorB.drive(-SPEED_MID);
  pause(TEST_DELAY, "Pivot Right");

  // ======== TEST 5: Ramp up ========
  Serial.println("[ TEST 9 ] Ramp Up (0 → 255)");
  Serial.println("  >> Ramping...");
  for (int spd = 0; spd <= 255; spd += 5) {
    runBoth(spd);
    delay(40);
  }
  pause(1000, "Ramp Up complete");

  // ======== TEST 6: Ramp down ========
  Serial.println("[ TEST 10 ] Ramp Down (255 → 0)");
  Serial.println("  >> Ramping...");
  for (int spd = 255; spd >= 0; spd -= 5) {
    runBoth(spd);
    delay(40);
  }
  pause(1000, "Ramp Down complete");

  // ======== DONE ========
  Serial.println();
  Serial.println("==============================");
  Serial.println("  All tests done! Restarting.");
  Serial.println("==============================");
  Serial.println();
  delay(3000);
}