
#include <Arduino.h>

// =====================================
// PIN CONFIGURATION
// =====================================

const int STEP_PIN = 3;
const int DIR_PIN = 4;
const int ENABLE_PIN = 5;

// =====================================
// MOTOR CONFIGURATION
// =====================================

// Change to your motor's actual specification
const int FULL_STEPS_PER_REV = 200;

// DRV8825 microstepping:
// 1 = full step
// 2 = half step
// 4 = quarter step
// 8 = 1/8 step
// 16 = 1/16 step
// 32 = 1/32 step
const int MICROSTEPS = 1;

const float STEPS_PER_REV =
    FULL_STEPS_PER_REV * MICROSTEPS;

// Change if your arm uses gearing
const float GEAR_RATIO = 1.0;

// Start slowly
const unsigned int STEP_DELAY_US = 4000;

// Test movement
const float TEST_ANGLE = 10.0;

// Safety limit
const float MAX_ANGLE = 90.0;

// =====================================
// MOTOR POSITION
// =====================================

long currentSteps = 0;

// =====================================
// CONVERT ANGLE TO STEPS
// =====================================

long angleToSteps(float angle) {
  return lround(
    angle / 360.0 *
    STEPS_PER_REV * GEAR_RATIO
  );
}

float stepsToAngle(long steps) {
  return steps * 360.0 /
         (STEPS_PER_REV * GEAR_RATIO);
}

// =====================================
// MOVE MOTOR TO TARGET ANGLE
// =====================================

void moveToAngle(float targetAngle) {

  if (targetAngle > MAX_ANGLE ||
      targetAngle < -MAX_ANGLE) {

    Serial.println("ERROR: Angle limit!");
    return;
  }

  long targetSteps = angleToSteps(targetAngle);

  long difference = targetSteps - currentSteps;

  if (difference == 0) {
    return;
  }

  digitalWrite(ENABLE_PIN, LOW);

  // Direction
  if (difference > 0) {
    digitalWrite(DIR_PIN, HIGH);
  } else {
    digitalWrite(DIR_PIN, LOW);
  }

  delayMicroseconds(10);

  long stepsToMove = labs(difference);

  for (long i = 0; i < stepsToMove; i++) {

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);

    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);

    if (difference > 0) {
      currentSteps++;
    } else {
      currentSteps--;
    }
  }

  // Disable motor after moving
  digitalWrite(ENABLE_PIN, HIGH);

  Serial.print("Target: ");
  Serial.print(targetAngle, 2);

  Serial.print(" deg | Estimated: ");
  Serial.print(stepsToAngle(currentSteps), 2);

  Serial.println(" deg");
}

// =====================================
// SETUP
// =====================================

void setup() {

  // Set disabled level before switching
  // the ENABLE pin to output mode
  digitalWrite(ENABLE_PIN, HIGH);
  pinMode(ENABLE_PIN, OUTPUT);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);

  Serial.begin(115200);

  Serial.println();
  Serial.println("DRV8825 STEPPER MOTOR TEST");
  Serial.println("-------------------------");
  Serial.println("Position the arm at 0 degrees");
  Serial.println("before enabling motor power.");
  Serial.println();

  // Program assumes motor starts at 0 deg
  currentSteps = 0;

  delay(3000);
}

// =====================================
// MAIN LOOP
// =====================================

void loop() {

  Serial.println("Moving to +10 degrees");
  moveToAngle(TEST_ANGLE);
  delay(1000);

  Serial.println("Returning to 0 degrees");
  moveToAngle(0);
  delay(1000);

  Serial.println("Moving to -10 degrees");
  moveToAngle(-TEST_ANGLE);
  delay(1000);

  Serial.println("Returning to 0 degrees");
  moveToAngle(0);
  delay(2000);
}
