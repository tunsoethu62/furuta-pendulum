/*
  vref_reader.ino  -  read the DRV8825 current-limit voltage (Vref) with the Uno, no multimeter needed

  NOTHING MOVES: this sketch never pulses STEP. It holds EN (D5) HIGH = driver disabled,
  and leaves STEP (D3) and DIR (D4) LOW. Run it with the MOTOR DISCONNECTED.

  Wiring (power everything OFF before connecting):
    - Uno A0  ->  the DRV8825 Vref test point (the metal top/centre of the small trimpot)
    - Uno GND ->  DRV8825 GND (common ground is required, otherwise the reading is meaningless)
    - VMOT 12 V must be ON while measuring: on the Pololu-style carrier, Vref only exists when VMOT is powered.

  Current limit formula (DRV8825 datasheet: I_limit = Vref / (5 * R_sense)):
    R_sense = 0.100 ohm (most Pololu-style boards, marked "R100")  ->  I_limit = 2 * Vref
    R_sense = 0.200 ohm (some clones, marked "R200")              ->  I_limit = Vref
  Check the two small resistors near the chip to see which one you have.

  Accuracy: the 5 V pin is the ADC reference. USB "5 V" is often 4.7-5.1 V, so expect about +/-5 %.
  If you can, power the Uno from the same supply every time you read.
*/

const uint8_t STEP_PIN = 3;
const uint8_t DIR_PIN  = 4;
const uint8_t EN_PIN   = 5;   // active LOW: HIGH = driver disabled
const uint8_t VREF_PIN = A0;

const float   ADC_REF_V = 5.0;   // change to your measured 5 V if you know it
const uint8_t SAMPLES   = 64;

void setup() {
  // Disable the driver FIRST: write HIGH before switching the pin to output, so it is never driven LOW.
  digitalWrite(EN_PIN, HIGH);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW); pinMode(STEP_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);  pinMode(DIR_PIN, OUTPUT);

  analogReference(DEFAULT);       // 5 V reference
  Serial.begin(115200);
  Serial.println(F("DRV8825 Vref reader (driver disabled, no STEP output)"));
  Serial.println(F("Vref_V, I_limit_A_if_R100, I_limit_A_if_R200"));
}

void loop() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) sum += analogRead(VREF_PIN);
  float counts = sum / (float)SAMPLES;
  float vref = counts * ADC_REF_V / 1023.0;

  Serial.print(vref, 3);       Serial.print(F(", "));
  Serial.print(2.0 * vref, 3); Serial.print(F(", "));
  Serial.print(vref, 3);
  if (counts < 5)    Serial.print(F("   <- ~0 V: check A0 wire, common GND, and that VMOT is ON"));
  if (counts > 1015) Serial.print(F("   <- at the top of the range: wrong point? (Vref is normally below ~1.5 V)"));
  Serial.println();
  delay(500);   // a simple 2 readings per second is fine here; nothing time-critical runs in this sketch
}
