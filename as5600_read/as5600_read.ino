/*
  as5600_read.ino  -  Furuta pendulum, step 1: read ONE AS5600 encoder (read-only)

  Board : Arduino Uno (ATmega328P, 5 V logic)
  Bus   : hardware I2C  ->  SDA = A4, SCL = A5   (the Uno's only hardware I2C pins)
  Sensor: AS5600 at fixed 7-bit address 0x36

  What it does
    - Every 20 ms (50 Hz) it reads the 12-bit RAW ANGLE (0..4095 = one full turn).
    - It "unwraps" the angle, so turning past 4095 -> 0 keeps counting (4096, 4097, ...)
      instead of jumping back to 0.
    - It reads the STATUS register to check the magnet.
    - It prints one CSV line per sample over Serial at 115200 baud:
        t_us,raw,unwrapped_counts,degrees,radians,magnet,status_hex,agc,magnitude
      magnet = OK | WEAK | STRONG | NONE | I2C_ERR
      agc       = automatic gain (0x1A). HIGH agc = weak field (chip turns gain up). In 5 V mode the
                  range is 0..255, in 3.3 V mode 0..128. Near the top of the range = magnet too far/weak.
      magnitude = field magnitude after the gain stage (0x1B/0x1C, 12 bit). Use it to compare magnet positions.

  SAFETY: this sketch never touches the DRV8825 STEP/DIR pins. Nothing moves.
          No pinMode/digitalWrite is done on any pin except I2C.

  Register map (AS5600 datasheet, ams-OSRAM, "Register Description"):
    0x0B STATUS    : bit5 MD = magnet detected, bit4 ML = magnet too weak, bit3 MH = magnet too strong
    0x0C RAW ANGLE : high byte, bits 11..8  (only the low 4 bits are used)
    0x0D RAW ANGLE : low byte,  bits 7..0
    0x1A AGC       : automatic gain control value
    0x1B MAGNITUDE : high byte, bits 11..8;  0x1C MAGNITUDE low byte
*/

#include <Wire.h>

// ---------- settings ----------
const uint8_t  AS5600_ADDR       = 0x36;
const uint8_t  REG_STATUS        = 0x0B;
const uint8_t  REG_RAW_ANGLE_HI  = 0x0C;   // 0x0D (low byte) is read right after it
// Free-decay test: use 10000 (100 Hz) and PRINT_DIAG = false so each line stays short enough for 100 Hz.
const uint32_t SAMPLE_PERIOD_US  = 10000UL; // 10 000 us = 100 Hz  (20000 = 50 Hz)
const bool     PRINT_DIAG        = false;   // true = also print magnet,status_hex,agc,magnitude (use at 50 Hz)
const int16_t  COUNTS_PER_TURN   = 4096;    // 12-bit sensor
const int16_t  HALF_TURN         = 2048;

// ---------- state ----------
uint32_t nextSampleUs   = 0;     // when the next sample is due
int16_t  lastRaw        = -1;    // -1 = no valid reading yet
int32_t  unwrapped      = 0;     // total counts, can go beyond one turn (+ or -)

// Read `count` bytes starting at register `reg`. Returns true on success.
bool readRegisters(uint8_t reg, uint8_t *buf, uint8_t count) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // false = repeated start, keep the bus
  if (Wire.requestFrom(AS5600_ADDR, count) != count) return false;
  for (uint8_t i = 0; i < count; i++) buf[i] = Wire.read();
  return true;
}

// Returns raw angle 0..4095, or -1 if the I2C read failed.
int16_t readRawAngle() {
  uint8_t b[2];
  if (!readRegisters(REG_RAW_ANGLE_HI, b, 2)) return -1;
  return (int16_t)(((uint16_t)(b[0] & 0x0F) << 8) | b[1]);
}

uint8_t lastStatus = 0;   // raw STATUS byte, printed for diagnosis

// Returns a short text describing the magnet, from the STATUS register.
const char *magnetFlag() {
  uint8_t s;
  if (!readRegisters(REG_STATUS, &s, 1)) return "I2C_ERR";
  lastStatus = s;
  bool md = s & (1 << 5);   // magnet detected
  bool ml = s & (1 << 4);   // too weak
  bool mh = s & (1 << 3);   // too strong
  if (!md) return "NONE";
  if (ml)  return "WEAK";
  if (mh)  return "STRONG";
  return "OK";
}

void setup() {
  Serial.begin(115200);
  Wire.begin();             // Uno = I2C master on A4/A5
  Wire.setClock(400000);    // AS5600 supports 400 kHz fast mode; use 100000 if your wires are long
  if (PRINT_DIAG) Serial.println(F("t_us,raw,unwrapped_counts,degrees,radians,magnet,status_hex,agc,magnitude"));
  else            Serial.println(F("t_us,raw,unwrapped_counts,degrees,radians"));
  nextSampleUs = micros();
}

void loop() {
  // Fixed-rate timing without delay(): wait until the next 20 ms tick.
  // The (int32_t) subtraction keeps this correct when micros() wraps after ~71 minutes.
  uint32_t now = micros();
  if ((int32_t)(now - nextSampleUs) < 0) return;
  nextSampleUs += SAMPLE_PERIOD_US;   // schedule from the planned time, not "now", so the rate does not drift

  int16_t raw = readRawAngle();
  if (raw < 0) {                       // read failed: report it, keep the old unwrapped value
    Serial.print(now); Serial.println(F(",,,,,I2C_ERR"));
    return;
  }

  // Unwrap with the shortest-change rule:
  // between two samples the shaft cannot plausibly move more than half a turn,
  // so a jump bigger than 2048 counts means we crossed the 4095 <-> 0 boundary.
  if (lastRaw < 0) {
    unwrapped = raw;                   // first valid reading: start from the raw angle
  } else {
    int16_t change = raw - lastRaw;
    if (change >  HALF_TURN) change -= COUNTS_PER_TURN;   // e.g. 10 -> 4090 really means -16
    if (change < -HALF_TURN) change += COUNTS_PER_TURN;   // e.g. 4090 -> 10 really means +16
    unwrapped += change;
  }
  lastRaw = raw;

  float degrees = unwrapped * (360.0f / COUNTS_PER_TURN);
  float radians = unwrapped * (2.0f * PI / COUNTS_PER_TURN);

  // CSV line. Note: printing ~50 characters at 115200 baud takes ~4.5 ms, fine at 50 Hz.
  Serial.print(now);        Serial.print(',');
  Serial.print(raw);        Serial.print(',');
  Serial.print(unwrapped);  Serial.print(',');
  Serial.print(degrees, 3); Serial.print(',');
  Serial.print(radians, 5);
  if (!PRINT_DIAG) { Serial.println(); return; }
  Serial.print(',');
  Serial.print(magnetFlag());     Serial.print(',');
  Serial.print(lastStatus, HEX);  Serial.print(',');
  uint8_t agc = 0, mag[2] = {0, 0};
  bool okAgc = readRegisters(0x1A, &agc, 1);
  bool okMag = readRegisters(0x1B, mag, 2);
  if (okAgc) Serial.print(agc);
  Serial.print(',');
  if (okMag) Serial.print(((uint16_t)(mag[0] & 0x0F) << 8) | mag[1]);
  Serial.println();
}
