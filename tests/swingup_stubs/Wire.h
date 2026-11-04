#pragma once
#include <stdint.h>
struct FakeWire {
  bool fail = false;
  unsigned readIndex = 0;
  uint16_t raw = 1269;
  void begin() {}
  void setClock(unsigned long) {}
  void setWireTimeout(unsigned long, bool) {}
  void clearWireTimeoutFlag() {}
  bool getWireTimeoutFlag() { return false; }
  void beginTransmission(uint8_t) {}
  void write(uint8_t) {}
  uint8_t endTransmission(bool) { return fail ? 1 : 0; }
  uint8_t requestFrom(uint8_t, uint8_t) { readIndex = 0; return fail ? 0 : 2; }
  uint8_t read() { return readIndex++ == 0 ? raw >> 8 : raw & 255; }
};
static FakeWire Wire;
