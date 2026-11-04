#pragma once
#include <math.h>
#include <stdint.h>
#define PI 3.14159265358979323846
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define _BV(n) (1u << (n))
#define OCIE1A 1
#define WGM12 3
#define CS11 1
#define ISR(name) void name()
#define __builtin_avr_delay_cycles(n) ((void)0)
struct __FlashStringHelper {};
#define F(s) reinterpret_cast<const __FlashStringHelper *>(s)
static uint8_t PORTD, TIMSK1, TCCR1A, TCCR1B;
static uint16_t TCNT1, OCR1A;
static uint32_t fakeUs = 0;
inline void noInterrupts() {}
inline void interrupts() {}
inline void digitalWrite(uint8_t pin, uint8_t v) {
  if (v) PORTD |= _BV(pin); else PORTD &= ~_BV(pin);
}
inline void pinMode(uint8_t, uint8_t) {}
inline uint32_t micros() { return fakeUs; }
inline uint32_t millis() { return fakeUs / 1000; }
inline void delay(uint32_t ms) { fakeUs += ms * 1000; }
inline void delayMicroseconds(uint32_t us) { fakeUs += us; }
inline float degrees(float r) { return r * 180 / PI; }
inline float radians(float d) { return d * PI / 180; }
template<class T> T constrain(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }
struct FakeSerial {
  const char *pending = "";
  void begin(int) {}
  int available() { return *pending != 0; }
  char read() { return *pending++; }
  int availableForWrite() { return 64; }
  unsigned write(const uint8_t *, unsigned n) { return n; }
  template<class T> void print(T) {}
  template<class T> void print(T, int) {}
  template<class T> void println(T) {}
  template<class T> void println(T, int) {}
};
static FakeSerial Serial;
