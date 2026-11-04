#define STEP_PIN 3
#define DIR_PIN 4
#define EN_PIN 5

// MICROSTEPS must match the DRV8825 M0/M1/M2 jumpers (find it with step_test.ino).
// Allowed: 1, 2, 4, 8, 16 or 32 (the static_assert below rejects anything else).
const int MICROSTEPS = 16;   // M0=GND, M1=GND, M2=5V -> 1/16 (user-reported); confirm with step_test
static_assert(MICROSTEPS == 1 || MICROSTEPS == 2 || MICROSTEPS == 4 || MICROSTEPS == 8 || MICROSTEPS == 16 || MICROSTEPS == 32,
              "Set MICROSTEPS to match the M0-M2 jumpers (use step_test.ino to find it)");
const long STEPS_PER_REV = 200L * MICROSTEPS;

// Test move of about 9 degrees, computed from MICROSTEPS (was a fixed 80, which is only 9 deg at 1/16)
const float TEST_DEG = 9.0;
const int TEST_STEPS = (int)(TEST_DEG / 360.0 * STEPS_PER_REV + 0.5);

const unsigned long STEP_DELAY_US = 2000;

void moveMotor(int steps, bool direction) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  delayMicroseconds(5);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void setup() {
  Serial.begin(115200);

  // Write EN HIGH (disabled) BEFORE making it an output; the old order drove EN LOW (enabled) for a moment.
  digitalWrite(EN_PIN, HIGH);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  Serial.println("FURUTA MOTOR TEST");
  Serial.println("Center the horizontal arm first.");
  Serial.println("A = +9 deg then return");
  Serial.println("B = -9 deg then return");
  Serial.println("S = Disable motor");
}

void loop() {
  if (!Serial.available()) return;

  char command = Serial.read();

  if (command == 'S' || command == 's') {
    digitalWrite(EN_PIN, HIGH);
    Serial.println("MOTOR DISABLED");
    return;
  }

  if (command == 'A' || command == 'a') {
    digitalWrite(EN_PIN, LOW);

    moveMotor(TEST_STEPS, true);
    delay(500);
    moveMotor(TEST_STEPS, false);

    digitalWrite(EN_PIN, HIGH);
    Serial.println("TEST A COMPLETE");
  }

  if (command == 'B' || command == 'b') {
    digitalWrite(EN_PIN, LOW);

    moveMotor(TEST_STEPS, false);
    delay(500);
    moveMotor(TEST_STEPS, true);

    digitalWrite(EN_PIN, HIGH);
    Serial.println("TEST B COMPLETE");
  }
}