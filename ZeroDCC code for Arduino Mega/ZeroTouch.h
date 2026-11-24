// ZeroTouch.h - XPT2046 resistive touch for ZeroDCC
//
// Reads the touch panel and turns a press into one "tap" at a screen position in the
// map screen's landscape coordinates (480 x 320, same as DccMap.h draws in).
//
// SETTINGS
//   TOUCH_ENABLED   0 switches touch off completely (nothing is read, no pins are used)
//   Pins and calibration are set below for each screen, picked by ZDCC_SPI_DISPLAY in the sketch:
//     3.95" LCD module for Mega2560 - hardware SPI shared with the SD card (TP_CS 53, TP_IRQ 44)
//     4.0" ST7796S SPI module       - software SPI on T_CLK 22, T_CS 24, T_DIN 26, T_DO 28, T_IRQ 30
//   Calibration is per screen: run TouchCal_395 (or TouchCal_SPI4 for the 4.0" module) and copy
//   the five TOUCH_... lines it prints over that screen's lines below.
//
// IN YOUR SKETCH
//   touchBegin();                 in setup(), BEFORE SD.begin() (keeps the touch chip off the bus);
//                                 it also checks whether a touch screen is fitted - touchFound()
//   if (touchTap(x, y)) { ... }   in loop(); true once per press, x / y in landscape pixels

#pragma once
#include <SPI.h>

// ---------------- Settings ----------------
// The screen is chosen in the sketch with ZDCC_SPI_DISPLAY; pins and calibration follow it.
#define TOUCH_ENABLED   1

#ifndef ZDCC_SPI_DISPLAY
#define ZDCC_SPI_DISPLAY 0
#endif

#if ZDCC_SPI_DISPLAY
// ---- 4.0" ST7796S SPI module: touch on its own pins, software SPI (as TouchTest_SPI4) ----
#define TOUCH_SOFT_SPI  1
const uint8_t TOUCH_CS  = 24;         // T_CS
const uint8_t TOUCH_IRQ = 30;         // T_IRQ (pen down = LOW)
const uint8_t TOUCH_CLK = 22;         // T_CLK
const uint8_t TOUCH_DIN = 26;         // T_DIN
const uint8_t TOUCH_DO  = 28;         // T_DO
// calibration from TouchCal_SPI4 on this module (30 Oct 2026)
#define TOUCH_SWAP    1
#define TOUCH_LEFT    3828
#define TOUCH_RIGHT   377
#define TOUCH_TOP     3701
#define TOUCH_BOTTOM  450

#else
// ---- 3.95" LCD module for Mega2560: touch on hardware SPI, shared with the SD card ----
#define TOUCH_SOFT_SPI  0
const uint8_t TOUCH_CS  = 53;         // TP_CS
const uint8_t TOUCH_IRQ = 44;         // TP_IRQ (pen down = LOW)
const uint8_t TOUCH_CLK = 52;         // not used with hardware SPI
const uint8_t TOUCH_DIN = 51;
const uint8_t TOUCH_DO  = 50;
// calibration: run TouchCal_395 on the new screen and paste its five lines here
#define TOUCH_SWAP    1
#define TOUCH_LEFT    3828
#define TOUCH_RIGHT   377
#define TOUCH_TOP     3701
#define TOUCH_BOTTOM  450
#endif

const int16_t  TOUCH_INSET   = 20;    // calibration targets were 20 px in from each corner
const int16_t  TOUCH_W       = 480;   // landscape screen size
const int16_t  TOUCH_H       = 320;
const uint16_t TOUCH_PRESS_Z = 100;   // pressure above this counts as a real press
const uint8_t  TOUCH_SETTLE  = 30;    // ms the pen must be down before the position is read
const uint8_t  TOUCH_RELEASE = 80;    // ms the pen must be up before the next tap can count

// ---------------- Is there a touch screen? ----------------
// touchBegin() checks at power-up and sets touchStatus:
const uint8_t TOUCH_NONE  = 0;      // no touch controller answered (or TOUCH_ENABLED 0)
const uint8_t TOUCH_OK    = 1;      // touch controller found and the panel reads normally
const uint8_t TOUCH_FAULT = 2;      // controller found, but the panel reads as if a wire is broken
uint8_t touchStatus = TOUCH_NONE;
inline bool touchFound() { return touchStatus == TOUCH_OK; }

// ---------------- Low level ----------------
#if TOUCH_ENABLED

#if TOUCH_SOFT_SPI
static uint8_t touchSoftByte(uint8_t out) {
  uint8_t in = 0;
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(TOUCH_DIN, (out & 0x80) ? HIGH : LOW);
    out <<= 1;
    delayMicroseconds(2);
    digitalWrite(TOUCH_CLK, HIGH);
    delayMicroseconds(2);
    in = (in << 1) | (digitalRead(TOUCH_DO) ? 1 : 0);
    digitalWrite(TOUCH_CLK, LOW);
  }
  return in;
}
#else
static SPISettings touchSPI(1000000, MSBFIRST, SPI_MODE0);
#endif

// one conversion, the 16 bits exactly as the chip sends them: a 0, 12 data bits, then three 0s
static uint16_t touchXferRaw(uint8_t cmd) {
#if TOUCH_SOFT_SPI
  touchSoftByte(cmd);
  uint16_t hi = touchSoftByte(0), lo = touchSoftByte(0);
  return (hi << 8) | lo;
#else
  SPI.transfer(cmd);
  return SPI.transfer16(0);
#endif
}

// one 12-bit conversion; cmd picks the channel
static uint16_t touchXfer(uint8_t cmd) {
#if TOUCH_SOFT_SPI
  touchSoftByte(cmd);
  uint16_t hi = touchSoftByte(0), lo = touchSoftByte(0);
  return (((hi << 8) | lo) >> 3) & 0x0FFF;
#else
  SPI.transfer(cmd);
  return (SPI.transfer16(0) >> 3) & 0x0FFF;
#endif
}

// raw position averaged over 4 readings; true only if pressed firmly at the start and the end
static bool touchRaw(uint16_t &x, uint16_t &y) {
#if !TOUCH_SOFT_SPI
  SPI.beginTransaction(touchSPI);
#endif
  digitalWrite(TOUCH_CS, LOW);
  uint16_t z1 = touchXfer(0xB1);                 // pressure
  touchXfer(0x91); touchXfer(0xD1);              // first readings after switching settle poorly
  uint32_t sx = 0, sy = 0;
  for (uint8_t i = 0; i < 4; i++) {
    sy += touchXfer(0x91);
    sx += touchXfer(0xD1);
  }
  uint16_t z1b = touchXfer(0xB0);                // pressure again, then power down with pen-down on
  digitalWrite(TOUCH_CS, HIGH);
#if !TOUCH_SOFT_SPI
  SPI.endTransaction();
#endif
  x = sx / 4;
  y = sy / 4;
  return z1 > TOUCH_PRESS_Z && z1b > TOUCH_PRESS_Z;
}

#endif  // TOUCH_ENABLED

// ---------------- Public ----------------
// Start the touch controller and check whether there is one (sets touchStatus).
//   - with no controller the data line is held high by its pull-up, so a reading comes back as all
//     1s; a real XPT2046 always ends each reading with three 0 bits and starts it with a 0
//   - its built-in temperature sensor must give a sensible value (rules out a line stuck low)
//   - with the screen not pressed, the pressure reading must be near 0; a broken panel wire
//     (like the first 3.95" module) reads high, and is reported as TOUCH_FAULT
void touchBegin() {
#if TOUCH_ENABLED
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  pinMode(TOUCH_IRQ, INPUT_PULLUP);
#if TOUCH_SOFT_SPI
  pinMode(TOUCH_CLK, OUTPUT); digitalWrite(TOUCH_CLK, LOW);
  pinMode(TOUCH_DIN, OUTPUT); digitalWrite(TOUCH_DIN, LOW);
  pinMode(TOUCH_DO,  INPUT_PULLUP);
#else
  SPI.begin();
  pinMode(MISO, INPUT_PULLUP);                   // shared with the SD card; the pull-up does it no harm
#endif

#if !TOUCH_SOFT_SPI
  SPI.beginTransaction(touchSPI);
#endif
  digitalWrite(TOUCH_CS, LOW);
  touchXferRaw(0x87);                            // temperature, internal reference on: first one settles it
  uint16_t t1 = touchXferRaw(0x87);
  uint16_t t2 = touchXferRaw(0x87);
  uint16_t z1 = touchXfer(0xB1);                 // pressure, screen not pressed
  digitalWrite(TOUCH_CS, HIGH);
#if !TOUCH_SOFT_SPI
  SPI.endTransaction();
#endif

  uint16_t x, y;
  touchRaw(x, y);                                // leaves the chip powered down, ready to signal pen-down
  delayMicroseconds(200);
  bool pressed = digitalRead(TOUCH_IRQ) == LOW;  // someone touching the screen while it starts up

  bool framed = (t1 & 0x8007) == 0 && (t2 & 0x8007) == 0;   // leading 0 and three trailing 0s
  uint16_t temp = (t2 >> 3) & 0x0FFF;
  if (!framed || temp < 100 || temp > 3900) {
    touchStatus = TOUCH_NONE;
  }
  else if (!pressed && z1 > 1000) {
    touchStatus = TOUCH_FAULT;                   // nothing pressing it, yet the pressure reads high
  }
  else {
    touchStatus = TOUCH_OK;
  }
#endif
}

inline bool touchPenDown() {
#if TOUCH_ENABLED
  return digitalRead(TOUCH_IRQ) == LOW;
#else
  return false;
#endif
}

// position under the pen in landscape pixels; false if not pressed firmly enough
bool touchRead(int16_t &sx, int16_t &sy) {
#if TOUCH_ENABLED
  uint16_t rx, ry;
  if (!touchRaw(rx, ry)) return false;
  long a1 = TOUCH_SWAP ? ry : rx;                // raw reading along the screen's width
  long a2 = TOUCH_SWAP ? rx : ry;                // raw reading down the screen's height
  long x = TOUCH_INSET + (a1 - TOUCH_LEFT) * (TOUCH_W - 1 - 2 * TOUCH_INSET) / (TOUCH_RIGHT - TOUCH_LEFT);
  long y = TOUCH_INSET + (a2 - TOUCH_TOP)  * (TOUCH_H - 1 - 2 * TOUCH_INSET) / (TOUCH_BOTTOM - TOUCH_TOP);
  sx = constrain(x, 0, TOUCH_W - 1);
  sy = constrain(y, 0, TOUCH_H - 1);
  return true;
#else
  return false;
#endif
}

// Call every time round loop(). Returns true once per press, with the position of the press.
// The pen has to come off the screen before the next tap counts, so holding a press can't
// repeat. Never waits - each call takes well under a millisecond.
bool touchTap(int16_t &sx, int16_t &sy) {
#if TOUCH_ENABLED
  if (touchStatus != TOUCH_OK) return false;     // no touch screen (or a faulty one): never a tap
  static uint8_t state = 0;                      // 0 waiting, 1 pen down - settling, 2 used - wait for release
  static unsigned long t = 0;
  unsigned long now = millis();
  bool down = touchPenDown();

  switch (state) {
    case 0:
      if (down) { state = 1; t = now; }
      break;

    case 1:
      if (!down) { state = 0; break; }           // too short to count
      if (now - t < TOUCH_SETTLE) break;
      if (touchRead(sx, sy)) { state = 2; t = now; return true; }
      if (now - t > 500) { state = 2; t = now; } // light press that never firmed up: ignore it
      break;

    case 2:
      if (down) t = now;                         // still pressed (or bounced): restart the release timer
      else if (now - t >= TOUCH_RELEASE) state = 0;
      break;
  }
#endif
  return false;
}
