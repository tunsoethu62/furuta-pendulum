#define STEP_PIN 3
#define DIR_PIN 4
#define EN_PIN 5

const int MICROSTEPS = 16;
const int STEPS_PER_REV = 200 * MICROSTEPS;

// 9 degrees = 80 microsteps
const int TEST_STEPS = 80;

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

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(EN_PIN, HIGH);

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