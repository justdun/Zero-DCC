// ZeroZ1Bus.h - Zero DCC RC 4.0
//
//  Shared by Zero DCC (the Mega) and the Rail Drive Nano (ZeroRailDrive.ino) - keep both copies the same.
//
//  1. Zero 1 bus messages and waveform (plain C++, used by the Rail Drive Nano)
//     A complete Zero 1 waveform (MERG Technical Bulletin CC01/1) for Zero 1 accessory modules, the
//     MicroMimic's Track In, and - if wanted - a track with genuine Zero 1 locos:
//       10 ms slots: one data period, then four power periods (+, -, +, -), repeating every 50 ms
//       data period: +V for 2 ms, 32 bits + one extra 0 (each bit one cycle, - half then + half;
//                    0 = 9.47 kHz cycle, 1 = 4.73 kHz cycle), then -V to the end of the slot
//     Accessory commands (1-89) go in the next data periods (each sent Z1BUS_ACC_REPEATS times).
//     The other data periods carry the 16 Zero 1 locos' speeds and directions, four locos at a time.
//
//  2. Zero DCC's link to the Rail Drive Nano (ZDCC_Z1BUS 1): the Mega sends <Z ...> messages on Serial2
//     (power, Zero 1 locos 1-16, accessories 1-89) and reads its replies (overload, current reports);
//     D8 is the "track power on" line to the Nano's D2. The waveform, current sense and overload trip
//     all run on the Nano - the Mega uses no timers or ADC for the Zero 1 bus.
//
//  3. MicroMimic Rail Drive RECEIVER (light pen, ZDCC_LIGHTPEN 1) on the Mega
//     The MicroMimic's light pen sends standard Zero 1 accessory messages on the Rail Drive socket,
//     pin 3. They're decoded here (checksum checked) and passed to Z1Rail_OnAccessory() in the main sketch.
//
//  The message / waveform / decode parts are plain C++ (define Z1BUS_HOST_TEST to use only those).

#ifndef ZERO_Z1BUS_H
#define ZERO_Z1BUS_H

#include <stdint.h>

// ---------------------------------------------------------------- settings
#ifndef Z1BUS_HOST_TEST
#include <Arduino.h>
const uint8_t Z1BUS_PIN_EN   = 8;      // -> Rail Drive Nano D2: "track power on" (high = bus may be on)
const uint8_t RAILDRIVE_PIN  = 2;      // MicroMimic Rail Drive pin 3 - must be an interrupt pin (2, 3, 18-21)
#define RAILDRIVE_EDGE FALLING         // time between falling edges (direct link, or 6N137 as in the notes)
const uint16_t Z1BUS_REPORT_MS = 0;    // calibrating: e.g. 2000 = the Nano reports its current every 2 s; 0 = off
#endif
const uint8_t Z1BUS_ACC_REPEATS = 4;   // each accessory message is sent this many times (50 ms apart)  (RC 3.2: 2 was not always enough for the point modules)

// ---------------------------------------------------------------- timing (timer ticks of 0.5 us)
const uint16_t Z1BUS_SLOT  = 20000;    // 10 ms slot (50 Hz mains in the original)
const uint16_t Z1BUS_LEAD  = 4000;     // 2 ms +V at the start of a data period
const uint16_t Z1BUS_HALF0 = 106;      // half cycle of a 0 bit: 52.8 us (9.47 kHz)
const uint16_t Z1BUS_HALF1 = 211;      // half cycle of a 1 bit: 105.7 us (4.73 kHz)

// ---------------------------------------------------------------- plain C++: messages
// A message is 32 bits, sent bit 0 first. Field n (1-8) is bits 4(n-1) .. 4(n-1)+3, LSB first.

// 4-bit add with end-around carry (CC01/1: the eight fields must add up to 1111)
static inline uint8_t z1Eac(uint8_t a, uint8_t b) {
  uint8_t s = a + b;
  while (s > 15) s = (s & 15) + (s >> 4);
  return s;
}

static inline uint8_t z1Field(uint32_t msg, uint8_t n) {      // n = 1..8
  return (msg >> (4 * (n - 1))) & 0x0F;
}

// Build a message from fields 1-6 and 8; field 7 (the check field) is worked out here
static inline uint32_t z1Build(const uint8_t f[8]) {
  uint8_t s = 0;
  for (uint8_t i = 0; i < 8; i++) if (i != 6) s = z1Eac(s, f[i] & 0x0F);
  uint32_t msg = 0;
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t v = (i == 6) ? (uint8_t)(~s & 0x0F) : (uint8_t)(f[i] & 0x0F);
    msg |= (uint32_t)v << (4 * i);
  }
  return msg;
}

// Accessory 0-99; state 1 = "<-", 0 = "->" (as Zero DCC's Send_Accessory and Z1_Data[24])
static inline uint32_t z1AccessoryMsg(uint8_t acc, uint8_t state) {
  uint8_t f[8] = { 0x2, 0, 0, (uint8_t)(acc / 10), (uint8_t)(acc % 10), (uint8_t)(state ? 0x8 : 0), 0, 0x2 };
  return z1Build(f);
}

// Loco message for tag group 'tag' (0 = locos 1-4 ... 3 = 13-16).
// speed[i] = Zero 1 speed 1 (stop) - 15 (full); fwd bit i = 1 for forward
static inline uint32_t z1LocoMsg(uint8_t tag, const uint8_t speed[4], uint8_t fwd) {
  uint8_t f[8] = { (uint8_t)(0x2 | ((tag & 3) << 2)), 0, 0, 0, 0, (uint8_t)(fwd & 0x0F), 0, 0x2 };
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t s = speed[i];
    f[1 + i] = (s < 1) ? 1 : (s > 15 ? 15 : s);  // 0000 would turn the message into an accessory one
  }
  return z1Build(f);
}

// Loco message with all four locos stopped, reverse
static inline uint32_t z1StopMsg(uint8_t tag) {
  const uint8_t stop[4] = { 1, 1, 1, 1 };
  return z1LocoMsg(tag, stop, 0);
}

// DCC-EX speed 0-126 -> Zero 1 speed 1-15 (the inverse of Zero DCC's map(z1 - 1, 0, 14, 0, 126))
static inline uint8_t z1SpeedFromDcc(int dcc) {
  if (dcc <= 0) return 1;
  if (dcc >= 126) return 15;
  return 1 + (uint8_t)((dcc * 14 + 63) / 126);
}

static inline bool z1ChecksumOK(uint32_t msg) {
  uint8_t s = 0;
  for (uint8_t n = 1; n <= 8; n++) s = z1Eac(s, z1Field(msg, n));
  return s == 15;
}

// Valid accessory message? Fills acc (0-99) and state (1 = "<-")
static inline bool z1DecodeAccessory(uint32_t msg, uint8_t &acc, uint8_t &state) {
  if (z1Field(msg, 1) != 0x2 || z1Field(msg, 2) != 0 || z1Field(msg, 3) != 0) return false;
  if (z1Field(msg, 8) != 0x2 || !z1ChecksumOK(msg)) return false;
  uint8_t tens = z1Field(msg, 4), units = z1Field(msg, 5);
  if (tens > 9 || units > 9) return false;
  acc = tens * 10 + units;
  state = (z1Field(msg, 6) & 0x8) ? 1 : 0;
  return true;
}

// ---------------------------------------------------------------- plain C++: waveform generator
// Each call gives the next piece of the track waveform: its polarity (+1 / -1) and length in ticks.
struct Z1BusGen {
  uint8_t  st = 0;         // 0 lead-in, 1 bits, 2 hold, 3-6 power periods
  uint8_t  bitIdx = 0;     // 0-32 (32 = the extra 0)
  uint8_t  half = 0;
  uint16_t used = 0;       // ticks used so far in the data period
  uint32_t msg = 0;
  uint8_t  tag = 0;        // next loco group, 0-3 in turn
  uint8_t  locoSpeed[16] = { 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1 };   // Zero 1 speeds, loco 1 = [0]
  uint16_t locoFwd = 0;    // bit n-1 = loco n forward
  // accessory queue
  uint32_t q[32];                 // 8 commands of 4 copies
  uint8_t  qHead = 0, qCount = 0;

  bool push(uint32_t m) {
    if (qCount >= 32) return false;
    q[(qHead + qCount) & 31] = m;
    qCount++;
    return true;
  }

  void setLoco(uint8_t n, uint8_t z1Speed, bool forward) {   // n = 1..16
    if (n < 1 || n > 16) return;
    locoSpeed[n - 1] = z1Speed;
    if (forward) locoFwd |= (1u << (n - 1)); else locoFwd &= ~(1u << (n - 1));
  }

  uint32_t nextMessage() {
    if (qCount) {
      uint32_t m = q[qHead];
      qHead = (qHead + 1) & 31;
      qCount--;
      return m;
    }
    uint32_t m = z1LocoMsg(tag, &locoSpeed[tag * 4], (uint8_t)((locoFwd >> (tag * 4)) & 0x0F));
    tag = (tag + 1) & 3;
    return m;
  }

  void step(int8_t &level, uint16_t &ticks) {
    switch (st) {
      case 0:                                   // data period: 2 ms lead-in at +V
        msg = nextMessage();
        level = 1; ticks = Z1BUS_LEAD; used = ticks;
        bitIdx = 0; half = 0; st = 1;
        break;
      case 1: {                                 // bits: - half then + half
        bool one = (bitIdx < 32) && ((msg >> bitIdx) & 1);
        ticks = one ? Z1BUS_HALF1 : Z1BUS_HALF0;
        level = half ? 1 : -1;
        used += ticks;
        if (half) { half = 0; if (++bitIdx > 32) st = 2; }
        else half = 1;
        break;
      }
      case 2:                                   // hold -V to the end of the slot
        level = -1; ticks = Z1BUS_SLOT - used; st = 3;
        break;
      default:                                  // power periods +, -, +, -
        level = (st & 1) ? 1 : -1;              // st 3 +, 4 -, 5 +, 6 -
        ticks = Z1BUS_SLOT;
        st = (st == 6) ? 0 : st + 1;
        break;
    }
  }
};

// ---------------------------------------------------------------- plain C++: Rail Drive decoder
// Fed with the time between same-direction edges of Rail Drive pin 3 (one per bit). The MicroMimic
// sends bits 2-33 (the leading sync 0 isn't on the pin), so the last 32 bits before a gap of over
// 1 ms are bits 2-33: bit 1 is always 0 and bit 33 (the extra 0) must be 0.
struct Z1RailDec {
  uint32_t window = 0;
  uint8_t  count = 0;
  bool     bad = false;

  void addPeriod(uint32_t periodUs) {
    uint8_t b;
    if (periodUs > 10 && periodUs < 150)       b = 0;
    else if (periodUs > 160 && periodUs < 250) b = 1;
    else { bad = true; return; }
    window = (window >> 1) | ((uint32_t)b << 31);
    if (count < 255) count++;
  }

  // Call when the line has been quiet for over 1 ms. True = a whole message, in msg.
  bool finish(uint32_t &msg) {
    bool ok = !bad && count >= 32 && !(window & 0x80000000UL);
    msg = window << 1;                         // bit 0 = the sync 0
    window = 0; count = 0; bad = false;
    return ok;
  }
};

// ================================================================ Arduino Mega part
#ifndef Z1BUS_HOST_TEST
#ifndef ZDCC_Z1BUS
#define ZDCC_Z1BUS 1                    // 1 = Rail Drive Nano on Serial2, 0 = no Zero 1 bus
#endif
#ifndef ZDCC_LIGHTPEN
#define ZDCC_LIGHTPEN 1                 // 1 = MicroMimic light pen on D2, 0 = off
#endif

void Z1Rail_OnAccessory(uint8_t acc, uint8_t state);   // in the main sketch

#if ZDCC_Z1BUS   // ======================================== the Rail Drive Nano on Serial2
#define Z1BUS_SERIAL Serial2
const uint16_t Z1BUS_HEARTBEAT_MS = 500;     // <Z H> when nothing else has been sent for this long
const uint16_t Z1BUS_MISSING_MS   = 3000;    // nothing from the Nano for this long -> reported missing
static bool z1busWanted = false;             // track power on
static uint8_t z1remSpeed[17], z1remDir[17]; // what each Zero 1 loco should be doing (DCC speed 0-126)
static unsigned long z1remLastSent = 0, z1remLastHeard = 0;
static bool z1remPresent = false;            // the Nano is answering
static bool z1remTripped = false;

static void z1remDone() { z1remLastSent = millis(); }
static void z1remPower() {
  Z1BUS_SERIAL.print(F("<Z P "));
  Z1BUS_SERIAL.print(z1busWanted ? 1 : 0);
  Z1BUS_SERIAL.println('>');
  z1remDone();
}
static void z1remLoco(uint8_t n) {
  Z1BUS_SERIAL.print(F("<Z L "));
  Z1BUS_SERIAL.print(n);
  Z1BUS_SERIAL.print(' ');
  Z1BUS_SERIAL.print(z1remSpeed[n]);
  Z1BUS_SERIAL.print(' ');
  Z1BUS_SERIAL.print(z1remDir[n]);
  Z1BUS_SERIAL.println('>');
  z1remDone();
}
static void z1remAll() {                     // everything the Nano should know (after HELLO / LOST)
  if (Z1BUS_REPORT_MS) {
    Z1BUS_SERIAL.print(F("<Z R "));
    Z1BUS_SERIAL.print(Z1BUS_REPORT_MS);
    Z1BUS_SERIAL.println('>');
  }
  for (uint8_t n = 1; n <= 16; n++) z1remLoco(n);
  z1remPower();
}

void Z1Bus_Begin() {
  pinMode(Z1BUS_PIN_EN, OUTPUT);             // D8 -> Nano D2: "track power on"
  digitalWrite(Z1BUS_PIN_EN, LOW);
  Z1BUS_SERIAL.begin(115200);
  for (uint8_t n = 0; n <= 16; n++) { z1remSpeed[n] = 0; z1remDir[n] = 0; }
  z1remAll();
}

void Z1Bus_Power(bool on) {
  z1busWanted = on;
  digitalWrite(Z1BUS_PIN_EN, on ? HIGH : LOW);
  z1remPower();
}

void Z1Bus_Accessory(unsigned int acc, uint8_t state) {
  if (acc < 1 || acc > 89) return;
  Z1BUS_SERIAL.print(F("<Z A "));
  Z1BUS_SERIAL.print(acc);
  Z1BUS_SERIAL.print(' ');
  Z1BUS_SERIAL.print(state ? 1 : 0);
  Z1BUS_SERIAL.println('>');
  z1remDone();
}

void Z1Bus_Loco(int n, int dccSpeed, int direction) {
  if (n < 1 || n > 16) return;
  z1remSpeed[n] = dccSpeed < 0 ? 0 : (dccSpeed > 126 ? 126 : dccSpeed);
  z1remDir[n] = direction ? 1 : 0;
  z1remLoco((uint8_t)n);
}

static void z1remLine(const char *l) {       // one line from the Nano
  z1remLastHeard = millis();
  if (!z1remPresent) { z1remPresent = true; }
  if (l[0] == '#') { Serial.print(F("Rail Drive: ")); Serial.println(l + 1); return; }
  if (strncmp(l, "<Z ", 3) != 0) return;
  const char *c = l + 3;
  if (!strncmp(c, "HELLO", 5)) {
    Serial.print(F("Rail Drive: Nano"));
    Serial.print(c + 5);
    Serial.println(F(" connected - sending power and loco settings"));
    z1remTripped = false;
    z1remAll();
  }
  else if (!strncmp(c, "LOST", 4)) {
    Serial.println(F("Rail Drive: Nano lost Zero DCC's messages - bus was switched off; resending"));
    z1remAll();
  }
  else if (c[0] == 'T') {
    z1remTripped = true;
    Serial.print(F("Zero 1 bus (Rail Drive) "));
    Serial.print(c[2] == 'S' ? F("SHORT") : F("OVERLOAD"));
    Serial.print(F(" - switched off (peak "));
    const char *a = strrchr(c, ' ');
    Serial.print(a ? a + 1 : "?");
    Serial.println(F(" A). Trying again shortly."));
  }
  else if (!strncmp(c, "OK", 2)) {
    z1remTripped = false;
    Serial.println(F("Zero 1 bus (Rail Drive): back on after overload"));
  }
  else if (c[0] == 'I') {                    // "I avg peak volts": amps, amps, sense volts
    Serial.print(F("Zero 1 bus current (Rail Drive): average, peak, sense V = "));
    Serial.println(c + 2);
  }
}

static void z1remPoll() {
  static char buf[64];
  static uint8_t len = 0;
  while (Z1BUS_SERIAL.available()) {
    char ch = Z1BUS_SERIAL.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      buf[len] = 0;
      if (len) z1remLine(buf);
      len = 0;
      continue;
    }
    if (len < sizeof(buf) - 1) buf[len++] = ch;
  }
  unsigned long now = millis();
  if (now - z1remLastSent >= Z1BUS_HEARTBEAT_MS) {
    Z1BUS_SERIAL.println(F("<Z H>"));
    z1remDone();
  }
  if (z1remPresent && now - z1remLastHeard > Z1BUS_MISSING_MS) {
    z1remPresent = false;
    Serial.println(F("Rail Drive: Nano not answering on Serial2 - check its power and wiring"));
  }
}
#else
inline void Z1Bus_Begin() {}
inline void Z1Bus_Power(bool) {}
inline void Z1Bus_Accessory(unsigned int, uint8_t) {}
inline void Z1Bus_Loco(int, int, int) {}
static inline void z1remPoll() {}
#endif

#if ZDCC_LIGHTPEN   // ======================================== MicroMimic Rail Drive receiver (light pen)
static volatile uint32_t z1railLast = 0;
static volatile bool     z1railActive = false;
Z1RailDec Z1RAIL_DEC;                           // filled by the interrupt, read with interrupts off

static void Z1Rail_Edge() {
  uint32_t now = micros();
  if (z1railActive) {
    uint32_t p = now - z1railLast;
    if (p <= 1000) Z1RAIL_DEC.addPeriod(p);
  }
  z1railLast = now;
  z1railActive = true;
}

void Z1Rail_Begin() {
  pinMode(RAILDRIVE_PIN, INPUT_PULLUP);         // idles high (Rail Drive's own pull-up, or the 6N137's)
  attachInterrupt(digitalPinToInterrupt(RAILDRIVE_PIN), Z1Rail_Edge, RAILDRIVE_EDGE);
}

static void z1railPoll() {
  // Rail Drive: a message is complete once the line has been quiet for over 1 ms
  uint32_t msg;
  bool ok = false, done = false;
  noInterrupts();
  if (z1railActive && micros() - z1railLast > 1000) {
    z1railActive = false;
    ok = Z1RAIL_DEC.finish(msg);
    done = true;
  }
  interrupts();
  if (done && ok) {
    uint8_t acc, state;
    if (z1DecodeAccessory(msg, acc, state) && acc != 0) {   // accessory 0 = light pen missed
      Z1Rail_OnAccessory(acc, state);
    }
  }

}
#else
inline void Z1Rail_Begin() {}
static inline void z1railPoll() {}
#endif

// Call from loop(): light pen messages, the Rail Drive Nano's messages
void Z1Bus_Poll() {
  z1railPoll();
  z1remPoll();
}

#endif  // Z1BUS_HOST_TEST
#endif  // ZERO_Z1BUS_H
