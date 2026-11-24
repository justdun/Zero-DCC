//  Zero DCC WiFi for the ESP8266 on a Mega2560+WiFi combo board (Duinotech MEGA r3 with ESP8266)
//
//  WiFi 2.0 - stage 4b
//
//  DCC-EX LINK - WiFi or serial (set in Zero DCC, ZDCC_DCCEX_SERIAL):
//  - WiFi (default): this ESP joins DCC-EX's access point and carries Zero DCC's and the phones'
//    DCC-EX traffic on one connection, as below.
//  - Serial: Zero DCC is wired to DCC-EX on its Serial1 and tells this ESP so ("@DX S"). The ESP then
//    leaves DCC-EX's WiFi alone (access point only) and hands the phones' DCC-EX commands to the Mega
//    ("&<...>"), which passes them to DCC-EX on Serial1; DCC-EX's replies come back as "@<...>" lines.
//
//  WHAT IT DOES
//  - Runs Zero DCC's own WiFi access point: phones (Train Driver etc.) connect to it, port 2560.
//  - Finds DCC-EX's own access point by itself (DCCEX_xxxxxx, password PASS_xxxxxx) and joins it.
//  - Zero DCC's link to DCC-EX, as bridge v1.1: DCC-EX commands from the Mega go to DCC-EX.
//  - Phones (stage 2):
//      loco speed / direction for an address in one of Zero DCC's 16 slots, or for address 1-16
//        (Zero 1 locos 1-16 are cab IDs 1-16) -> Zero DCC (as a Master key:
//        DCC-EX and the Zero 1 bus), track power <1> / <0> (not PROG) -> Zero DCC, <!> -> both;
//      accessories and turnouts all -> DCC-EX; those on address 1-89 -> Zero DCC as well (its Zero 1
//        side: Zero 1 bus, map, recording - Zero DCC doesn't send them to DCC-EX again),
//      turnouts: a phone's DCC turnout <T id DCC linear> (or <T id DCC address sub>) is remembered and
//        passed on to DCC-EX; throwing one whose address is 1-89 (<T id 0/1>) also goes to Zero DCC as
//        that Zero 1 accessory, and Zero DCC's point changes come back to the phones as <H id state>,
//      recording cabs 9990-9999 (<t> and <F>) -> Zero DCC only, never DCC-EX (F1 Play, F2 Record, F0 Stop),
//      while Zero DCC is recording: every loco <t> -> Zero DCC (it sends it on to DCC-EX and records it),
//      every DCC accessory / DCC turnout 1-2040 also goes to Zero DCC (Zero 1 side for 1-89, the map,
//        the recording); Zero DCC's changes on any of them come back to the phones as <H id state>,
//      loco functions <F cab fn state> -> Zero DCC (it keeps the state, records it and sends it on to DCC-EX),
//      everything else -> DCC-EX.
//  - Stage 2b: Zero DCC and all phones share ONE connection to DCC-EX (DCC-EX only allows a few, and
//    phones connecting to DCC-EX directly need theirs). Everything DCC-EX sends on it goes to the Mega
//    and to every phone, so phones see Zero DCC's own changes (power, Zero 1 Master) as well.
//    Zero DCC also sends @ messages for its slot locos and power; without DCC-EX the ESP answers the
//    phone's status questions itself (Zero 1 only).
//  - Every power command from a phone is acted on and logged (no start-up filter: apps differ too much).
//
//  NETWORK NAMES
//  Zero DCC's access point: "ZeroDCC_" + the last 6 characters of this ESP's MAC address,
//  password "Password_" + the same 6 characters (shown in the Mega's serial log at start-up).
//  To use your own, fill in AP_NAME / AP_PASSWORD below (password at least 8 characters).
//  DCC-EX: the strongest access point named "DCCEX_..." is joined, with password "PASS_..." (the same
//  ending), as DCC-EX sets up by default. To use a fixed one, fill in DCCEX_NAME / DCCEX_PASSWORD.
//  If DCC-EX isn't found after SEARCH_TRIES tries (about 10 s each), Zero DCC carries on without it
//  (Zero 1 only) and looks again when track power is turned on, or at the next restart.
//
//  SERIAL LINK TO THE MEGA (Serial3, 115200)
//  Mega -> ESP   <...>        a command for DCC-EX (as bridge v1.1)
//                @<...>       a message for every connected phone (slot loco changes, power)
//                @ZS a1 .. a16  the DCC addresses in Zero DCC's 16 loco slots (kept by the ESP, not sent on)
//                @A n state   accessory n (1-89) changed: sent to phones as <H id state> for its turnouts
//                @R 0|1       Zero DCC is recording (1) or not (0)
//                @DX S | W    DCC-EX is on Zero DCC's serial link (S) / reached over WiFi (W, the default)
//  ESP -> Mega   <...>        a phone command for Zero DCC: <t cab speed dir>, <1>, <0>, <!>,
//                             <a n state> (n 1-89), <F cab fn state>
//                #ZB s n p    status every 2 s and on any change: s = 1 looking for DCC-EX,
//                             2 on DCC-EX's WiFi but can't reach DCC-EX, 3 connected to DCC-EX,
//                             4 DCC-EX not found (Zero 1 only); n = commands from the Mega so far;
//                             p = phones connected
//                #ZL text     a line for the Mega's serial log
//                #<...>       a reply or broadcast from DCC-EX on Zero DCC's own connection
//                &<...>       (serial link) a phone command for DCC-EX - the Mega sends it on Serial1
//                #ZB 5 ...    (serial link) state 5 = DCC-EX on Zero DCC's serial link
//
//  DCC-EX over WiFi acts only on the first command in each network packet, so every command is
//  sent on its own, at least COMMAND_GAP_MS after the previous one on that connection.
//
//  BOARD SETTINGS (Arduino IDE): "Generic ESP8266 Module", Crystal Frequency 26 MHz.
//  To upload: set the combo board's DIP switches for USB <-> ESP8266 with the ESP in flash mode,
//  then set them back to Mega <-> ESP8266 (Serial3) to run.
//
//  STATUS LED (if your ESP has one on GPIO2, active LOW): slow blink = looking for DCC-EX,
//  fast blink = can't reach DCC-EX, steady = connected, off = Zero 1 only

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

// ---------------------------------------------------------------- settings
const char *AP_NAME        = "";                  // "" = ZeroDCC_xxxxxx
const char *AP_PASSWORD    = "";                  // "" = Password_xxxxxx (at least 8 characters)
IPAddress   AP_IP(192, 168, 6, 1);                // Zero DCC's network (DCC-EX uses 192.168.4.1)
const char *DCCEX_NAME     = "";                  // "" = the strongest DCCEX_xxxxxx in range
const char *DCCEX_PASSWORD = "";                  // "" = PASS_xxxxxx to match
const uint16_t PORT        = 2560;                // phones connect here; DCC-EX listens on the same

const uint8_t  SEARCH_TRIES   = 6;                // tries to find and join DCC-EX before giving up
const unsigned long TRY_MS    = 10000;            // time allowed for each try
const uint8_t  MAX_PHONES     = 3;                // phones on Zero DCC's WiFi (they share one DCC-EX connection)
const unsigned long SERIAL_BAUD = 115200;         // must match Serial3.begin() on the Mega
const unsigned long COMMAND_GAP_MS = 20;          // pause between commands to DCC-EX
const int STATUS_LED = 2;                         // on-board LED, active LOW

const char *VERSION = "WiFi 2.0 stage 4b";
const bool DEBUG_PHONES = false;                 // true: log what phones send and what comes back (testing)

// ---------------------------------------------------------------- a queue of whole <...> commands
const uint8_t CMD_LEN = 96;
struct CmdQueue {
  static const uint8_t N = 16;
  char cmd[N][CMD_LEN];
  uint8_t len[N];
  uint8_t head = 0, count = 0;
  bool full() const { return count >= N; }
  bool empty() const { return count == 0; }
  void push(const char *c, uint8_t n) {
    if (full()) return;
    uint8_t i = (head + count) % N;
    memcpy(cmd[i], c, n); len[i] = n; count++;
  }
  void pop() { if (count) { head = (head + 1) % N; count--; } }
  void clear() { head = count = 0; }
};

// collects <...> commands from a byte stream
struct CmdParser {
  char buf[CMD_LEN];
  uint8_t len = 0;
  bool in = false;
  // returns true when a complete command is in buf / len
  bool feed(char c) {
    if (c == '<') { in = true; len = 0; }
    if (!in) return false;
    if (len < CMD_LEN) buf[len++] = c; else { in = false; return false; }   // too long: dropped
    if (c == '>') { in = false; return true; }
    return false;
  }
};

// ---------------------------------------------------------------- state
enum Link : uint8_t { L_SEARCH = 1, L_NOPORT = 2, L_LINKED = 3, L_GAVEUP = 4, L_SERIAL = 5 };
Link link = L_SEARCH;
uint8_t tries = 0;
bool scanning = false, joining = false;
unsigned long tryStarted = 0, nextTryAt = 0;
String joinName;

WiFiClient zdcc;                                  // Zero DCC's own connection to DCC-EX
CmdQueue zdccQ;
unsigned long zdccLastSent = 0, zdccLastConnect = 0;
CmdParser megaParser;
bool megaToPhones = false;                        // the command being read started with '@'
uint16_t commandCount = 0;

WiFiServer server(PORT);
struct Phone {
  WiFiClient c;                                   // the phone
  CmdParser p;
  unsigned long connectedAt = 0;
  bool sawStatus = false;                         // the phone has sent <s> (so it knows the power state)
  bool used = false;
};
Phone phones[MAX_PHONES];
uint8_t phoneCount = 0;

int slotAddr[16];                                 // from Zero DCC: the address in each loco slot
bool slotsKnown = false;
int powerState = -1;                              // last @<p..> from Zero DCC (for phones' questions)
String locoState[16];                             // last @<l ...> for each slot (Zero 1 only)
String megaAt;                                    // an @ line being read from the Mega
bool recording = false;                           // Zero DCC is recording (@R 1)
String recState[10];                              // last <l ...> for recording cabs 9990-9999

bool isRecCab(long cab) { return cab >= 9990 && cab <= 9999; }

// DCC turnouts the phones have defined (<T id DCC ...>): turnout id -> DCC linear address
const uint8_t MAX_TURNOUTS = 48;
struct TurnoutMap { int id; int lin; };
TurnoutMap turnouts[MAX_TURNOUTS];
uint8_t turnoutCount = 0, turnoutNext = 0;

int turnoutLin(int id) {
  for (uint8_t t = 0; t < turnoutCount; t++) if (turnouts[t].id == id) return turnouts[t].lin;
  return -1;
}
void turnoutSet(int id, int lin) {               // lin -1 = forget it
  for (uint8_t t = 0; t < turnoutCount; t++) if (turnouts[t].id == id) { turnouts[t].lin = lin; return; }
  if (lin < 0) return;
  uint8_t t;
  if (turnoutCount < MAX_TURNOUTS) t = turnoutCount++;
  else { t = turnoutNext; turnoutNext = (turnoutNext + 1) % MAX_TURNOUTS; }   // full: reuse the oldest
  turnouts[t].id = id; turnouts[t].lin = lin;
}
bool megaAtLine = false;

String suffix;                                    // last 6 characters of the MAC
String apName, apPass;
uint8_t reportedState = 0, reportedPhones = 255;
uint16_t reportedCount = 0;
unsigned long lastReport = 0;

// ---------------------------------------------------------------- messages to the Mega
void megaLog(const String &s) { Serial.print("#ZL "); Serial.println(s); }

void report(bool force) {
  unsigned long now = millis();
  bool changed = link != reportedState || phoneCount != reportedPhones;
  bool counted = commandCount != reportedCount;
  if (!force && !changed && !(counted && now - lastReport >= 150) && now - lastReport < 2000) return;
  Serial.print("#ZB "); Serial.print((int)link); Serial.print(' ');
  Serial.print(commandCount); Serial.print(' '); Serial.println(phoneCount);
  reportedState = link; reportedPhones = phoneCount; reportedCount = commandCount;
  lastReport = now;
}

void setLed(bool on) { digitalWrite(STATUS_LED, on ? LOW : HIGH); }

// ---------------------------------------------------------------- finding and joining DCC-EX
void startSearch(const char *why) {
  if (link == L_LINKED || link == L_NOPORT) return;
  tries = 0; link = L_SEARCH; scanning = joining = false; nextTryAt = millis();
  megaLog(String("looking for DCC-EX (") + why + ")");
}

void searchStep() {
  unsigned long now = millis();
  if (link != L_SEARCH) return;

  if (!scanning && !joining) {                    // start a try (each try takes TRY_MS)
    if ((long)(now - nextTryAt) < 0) return;
    if (tries >= SEARCH_TRIES) {
      link = L_GAVEUP;
      WiFi.disconnect();
      megaLog("DCC-EX not found - carrying on without it (Zero 1 only)");
      return;
    }
    tries++;
    tryStarted = now;
    nextTryAt = now + TRY_MS;
    if (DCCEX_NAME[0]) {                          // a fixed name: no scan needed
      joinName = DCCEX_NAME;
      WiFi.begin(DCCEX_NAME, DCCEX_PASSWORD[0] ? DCCEX_PASSWORD : "");
      joining = true;
      megaLog(String("try ") + tries + ": joining " + joinName);
    } else {
      WiFi.scanNetworks(true);                    // in the background
      scanning = true;
    }
    return;
  }

  if (scanning) {
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) {
      if (now - tryStarted > TRY_MS) { scanning = false; WiFi.scanDelete(); }
      return;
    }
    scanning = false;
    int best = -1; int32_t bestRssi = -1000;
    for (int i = 0; i < n; i++) {
      String s = WiFi.SSID(i);
      if (s.startsWith("DCCEX_") && WiFi.RSSI(i) > bestRssi) { best = i; bestRssi = WiFi.RSSI(i); }
    }
    if (best < 0) {
      WiFi.scanDelete();
      megaLog(String("try ") + tries + ": no DCC-EX access point in range");
      return;                                     // next try at nextTryAt
    }
    joinName = WiFi.SSID(best);
    String pass = DCCEX_PASSWORD[0] ? String(DCCEX_PASSWORD) : "PASS_" + joinName.substring(6);
    WiFi.scanDelete();
    WiFi.begin(joinName.c_str(), pass.c_str());
    joining = true;
    tryStarted = now;
    nextTryAt = now + TRY_MS;
    megaLog(String("try ") + tries + ": joining " + joinName + " (" + bestRssi + " dBm)");
    return;
  }

  if (joining) {
    if (WiFi.status() == WL_CONNECTED) {
      joining = false;
      link = L_NOPORT;                            // on the network; connect() follows
      megaLog("on " + joinName + ", DCC-EX at " + WiFi.gatewayIP().toString());
      return;
    }
    if (now - tryStarted > TRY_MS) {
      joining = false;
      WiFi.disconnect();
      megaLog(String("try ") + tries + ": couldn't join " + joinName);
    }
  }
}

// ---------------------------------------------------------------- connections to DCC-EX
void toAllPhones(const String &line);
int slotOf(int cab);
bool dccexReady() { return link == L_NOPORT || link == L_LINKED; }

void sendQueued(WiFiClient &up, CmdQueue &q, unsigned long &lastSent) {
  if (q.empty() || !up.connected()) return;
  unsigned long now = millis();
  if (now - lastSent < COMMAND_GAP_MS) return;
  up.write((const uint8_t *)q.cmd[q.head], q.len[q.head]);   // one command per packet
  q.pop();
  lastSent = now;
}

void dccexStep() {
  unsigned long now = millis();
  if (!dccexReady()) return;

  if (WiFi.status() != WL_CONNECTED) {            // lost DCC-EX's WiFi
    zdcc.stop();
    link = L_SEARCH; tries = 0; scanning = joining = false; nextTryAt = now;
    megaLog("lost DCC-EX's WiFi - looking again");
    return;
  }

  if (!zdcc.connected()) {
    if (link == L_LINKED) megaLog("connection to DCC-EX closed - reconnecting");
    link = L_NOPORT;
    if (now - zdccLastConnect > 2000) {
      zdccLastConnect = now;
      zdcc.setTimeout(1000);
      if (zdcc.connect(WiFi.gatewayIP(), PORT)) {
        zdcc.setNoDelay(true);
        link = L_LINKED;
        megaLog("connected to DCC-EX");
      }
    }
    return;
  }

  sendQueued(zdcc, zdccQ, zdccLastSent);

  // replies and broadcasts on the shared connection -> the Mega ("#<...>") and every phone
  static CmdParser replyParser;
  while (zdcc.available()) {
    if (replyParser.feed((char)zdcc.read())) {
      Serial.print('#');
      Serial.write((const uint8_t *)replyParser.buf, replyParser.len);
      Serial.println();
      String line(replyParser.buf); line = line.substring(0, replyParser.len);
      toAllPhones(line + "\n");
      if (DEBUG_PHONES) megaLog("DCC-EX: " + line);
    }
  }
}

// ---------------------------------------------------------------- the Mega
int slotOf(int cab) {                             // the first slot holding this address, or -1
  if (!slotsKnown) return -1;
  for (uint8_t k = 0; k < 16; k++) if (slotAddr[k] == cab) return k;
  return -1;
}

void toAllPhones(const String &line) {           // line ends in \n
  for (uint8_t i = 0; i < MAX_PHONES; i++) if (phones[i].used) phones[i].c.print(line);
}

// a whole "@..." line from the Mega
void megaAtMessage(String m) {
  m.trim();
  if (m.startsWith("ZS")) {                       // the slot list
    int pos = 2;
    for (uint8_t k = 0; k < 16; k++) {
      while (pos < (int)m.length() && m[pos] == ' ') pos++;
      int end = pos;
      while (end < (int)m.length() && m[end] != ' ') end++;
      slotAddr[k] = end > pos ? m.substring(pos, end).toInt() : -1;
      pos = end;
    }
    if (!slotsKnown) megaLog("slot list received from Zero DCC");
    slotsKnown = true;
    return;
  }
  if (m.startsWith("DX ")) {                      // how DCC-EX is reached
    if (m[3] == 'S' && link != L_SERIAL) {
      zdcc.stop();
      zdccQ.clear();
      scanning = joining = false;
      WiFi.scanDelete();
      WiFi.disconnect();
      WiFi.mode(WIFI_AP);                         // Zero DCC's access point only
      link = L_SERIAL;
      megaLog("DCC-EX is on Zero DCC's serial link - not using DCC-EX's WiFi");
      report(true);
    }
    else if (m[3] == 'W' && link == L_SERIAL) {   // back to WiFi: start again
      megaLog("DCC-EX over WiFi again - restarting the WiFi");
      delay(100);
      ESP.restart();
    }
    return;
  }
  if (m.startsWith("R ")) {                       // recording on / off
    bool r = m.substring(2).toInt() != 0;
    if (r != recording) megaLog(r ? "recording: all phone locos go through Zero DCC" : "recording ended");
    recording = r;
    return;
  }
  if (m.startsWith("A ")) {                       // accessory n changed on Zero DCC: <H id state> to phones
    int sp = m.indexOf(' ', 2);
    if (sp < 0) return;
    int acc = m.substring(2, sp).toInt();
    int st = m.substring(sp + 1).toInt() ? 1 : 0;
    for (uint8_t t = 0; t < turnoutCount; t++)
      if (turnouts[t].lin == acc) toAllPhones("<H " + String(turnouts[t].id) + " " + String(st) + ">\n");
    return;
  }
  if (!m.startsWith("<")) return;
  if (m.startsWith("<p")) powerState = (m.length() > 2 && m[2] == '1') ? 1 : 0;
  if (m.startsWith("<l ")) {                      // <l cab 0 speedbyte functions> from Zero DCC
    long cab = m.substring(3).toInt();
    int k = slotOf(cab);
    if (k >= 0) locoState[k] = m;                 // remembered for a phone that asks
    if (isRecCab(cab)) recState[cab - 9990] = m;
  }
  toAllPhones(m + "\n");
}

void megaStep() {
  // read only while there's room to queue another command (the rest waits in the serial buffer)
  while (Serial.available() && !zdccQ.full()) {
    char c = Serial.read();
    if (megaAtLine) {                             // rest of an @ line
      if (c == '\n' || c == '\r') { megaAtLine = false; megaAtMessage(megaAt); megaAt = ""; }
      else if (megaAt.length() < 120) megaAt += c;
      continue;
    }
    if (c == '@' && !megaParser.in) { megaAtLine = true; megaAt = ""; continue; }
    if (megaParser.feed(c)) {
      commandCount++;
      bool powerOn = megaParser.len >= 3 && megaParser.buf[1] == '1' && (megaParser.buf[2] == '>' || megaParser.buf[2] == ' ');
      if (link == L_SERIAL) {}                    // (the Mega sends its own commands on Serial1)
      else if (dccexReady()) zdccQ.push(megaParser.buf, megaParser.len);
      else if (powerOn && link == L_GAVEUP) startSearch("track power on");
    }
  }
}

// ---------------------------------------------------------------- phones
// one complete <...> command from phone i
void toDccex(uint8_t i, const char *cmd, uint8_t len) {
  if (link == L_SERIAL) {                         // DCC-EX on the Mega's Serial1: the Mega passes it on
    Serial.print('&');
    Serial.write((const uint8_t *)cmd, len);
    Serial.println();
    return;
  }
  if (link == L_LINKED && zdcc.connected()) zdccQ.push(cmd, len);
  else if (DEBUG_PHONES) megaLog("phone " + String(i + 1) + ": no connection to DCC-EX - dropped");
}

void toMega(const String &cmd) { Serial.println(cmd); }

// the numbers after the command letter in body ("t 3 20 1" -> 3, 20, 1); returns how many (up to max)
int parseNums(const String &body, long *v, int max) {
  int n = 0, pos = 1;
  while (n < max) {
    while (pos < (int)body.length() && body[pos] == ' ') pos++;
    if (pos >= (int)body.length()) break;
    int end = pos; while (end < (int)body.length() && body[end] != ' ') end++;
    String w = body.substring(pos, end);
    char c0 = w[0];
    if (!(isdigit(c0) || c0 == '-')) break;       // a word (MAIN, PROG ...), not a number
    v[n++] = w.toInt(); pos = end;
  }
  return n;
}

void routePhone(uint8_t i, const char *cmd, uint8_t len) {
  String c(cmd); c = c.substring(0, len);         // "<...>"
  String body = c.substring(1, c.length() - 1); body.trim();
  char op = body.length() ? body[0] : 0;
  bool linked = (link == L_LINKED || link == L_SERIAL);

  if (op == 't') {                                // loco: <t cab speed dir>, <t reg cab speed dir>, <t cab>
    long v[4]; int n = parseNums(body, v, 4);
    long cab = -1, speed = 0, dir = 0;
    if (n == 3) { cab = v[0]; speed = v[1]; dir = v[2]; }
    else if (n == 4) { cab = v[1]; speed = v[2]; dir = v[3]; }
    else if (n == 1) {                            // a phone asking for a loco's state
      if (isRecCab(v[0])) {                       // a recording cab: Zero DCC's own
        String r = recState[v[0] - 9990];
        phones[i].c.print((r.length() ? r : "<l " + String(v[0]) + " 0 0 0>") + "\n");
        return;
      }
      int k = slotOf(v[0]);
      if (!linked && k >= 0 && locoState[k].length()) phones[i].c.print(locoState[k] + "\n");
      else if (linked) toDccex(i, cmd, len);
      return;
    }
    if (cab >= 0 && (slotOf(cab) >= 0 || (cab >= 1 && cab <= 16) || isRecCab(cab) || recording)) {
      // a slot loco, Zero 1 loco 1-16, a recording cab, or any loco while Zero DCC is recording
      if (speed < 0) speed = 0;                   // -1 = emergency stop for this loco
      toMega("<t " + String(cab) + " " + String(speed) + " " + String(dir) + ">");
      return;
    }
    if (linked) toDccex(i, cmd, len);
    return;
  }

  if ((op == '1' || op == '0') && body.indexOf("PROG") < 0) {   // track power (main / both)
    megaLog("phone " + String(i + 1) + ": power " + (op == '1' ? "on" : "off") + " (" + c + ")");
    toMega(String("<") + op + ">");
    return;
  }

  if (op == 'a') {                                // accessory: <a linear state> or <a address sub state>
    long v[3]; int n = parseNums(body, v, 3);
    long lin = -1, state = 0;
    if (n == 2) { lin = v[0]; state = v[1]; }
    else if (n == 3) { lin = (v[0] - 1) * 4 + v[1] + 1; state = v[2]; }
    if (linked) toDccex(i, cmd, len);             // every accessory goes to DCC-EX
    if (lin >= 1 && lin <= 2040)                  // and to Zero DCC too: 1-89 its Zero 1 side, all of them
                                                  // the map and (while recording) the recording
      toMega("<a " + String(lin) + " " + String(state ? 1 : 0) + ">");
    return;
  }

  if (op == 'T') {                                // turnouts
    String w[5]; int nw = 0; int pos = 1;         // the words after T
    while (nw < 5) {
      while (pos < (int)body.length() && body[pos] == ' ') pos++;
      if (pos >= (int)body.length()) break;
      int end = pos; while (end < (int)body.length() && body[end] != ' ') end++;
      w[nw++] = body.substring(pos, end); pos = end;
    }
    if (nw >= 3) {                                // a definition: <T id DCC linear>, <T id DCC addr sub>, <T id SERVO ...>
      int id = w[0].toInt(); int lin = -1;
      if (w[1] == "DCC" && nw == 3) lin = w[2].toInt();
      else if (w[1] == "DCC" && nw == 4) lin = (w[2].toInt() - 1) * 4 + w[3].toInt() + 1;
      turnoutSet(id, lin);
      if (linked) toDccex(i, cmd, len);
      else phones[i].c.print("<O>\n");          // no DCC-EX: accept it here
      return;
    }
    if (nw == 2) {                                // throw / close: <T id 0|1|C|T>
      int id = w[0].toInt();
      int lin = turnoutLin(id);
      if (linked) toDccex(i, cmd, len);           // every turnout goes to DCC-EX
      if (lin >= 1 && lin <= 2040) {              // and to Zero DCC too: 1-89 its Zero 1 side, all of them
                                                  // the map and (while recording) the recording
        char c0 = w[1][0];
        int st = (c0 == '1' || c0 == 'T' || c0 == 't') ? 1 : 0;
        toMega("<a " + String(lin) + " " + String(st) + ">");
      }
      return;
    }
    if (linked) toDccex(i, cmd, len);
    return;
  }

  if (op == 'F') {                                // loco function: <F cab function state>
    long v[3]; int n = parseNums(body, v, 3);
    if (n == 3 && v[0] > 0 && v[1] >= 0 && v[1] <= 31) {
      toMega("<F " + String(v[0]) + " " + String(v[1]) + " " + String(v[2] ? 1 : 0) + ">");
      return;
    }
    if (linked) toDccex(i, cmd, len);
    return;
  }

  if (op == '!') {                                // emergency stop: Zero DCC's slots, and DCC-EX
    toMega("<!>");
    if (linked) toDccex(i, cmd, len);
    return;
  }

  if (op == 's') phones[i].sawStatus = true;

  if (linked) { toDccex(i, cmd, len); return; }

  // no DCC-EX: answer the phone's usual questions here
  if (op == 's') {
    phones[i].c.print("<iDCC-EX V-5.0.0 / ZERO-DCC / WIFI-2.0 / G-ZERODCC>\n");
    phones[i].c.print(powerState == 1 ? "<p1>\n" : "<p0>\n");
  }
  else if (op == 'J' && body.length() > 1) phones[i].c.print(String("<j") + body[1] + ">\n");   // empty roster / lists
  else if (op == '#') phones[i].c.print("<# 16>\n");
}

void phonesStep() {
  unsigned long now = millis();

  // new phone
  WiFiClient nc = server.accept();
  if (nc) {
    int slot = -1;
    for (uint8_t i = 0; i < MAX_PHONES; i++) if (!phones[i].used) { slot = i; break; }
    if (slot < 0) {
      nc.stop();
      megaLog("phone turned away - already " + String(MAX_PHONES) + " connected");
    } else {
      Phone &ph = phones[slot];
      ph.c = nc; ph.c.setNoDelay(true);
      ph.used = true; ph.p.in = false; ph.connectedAt = millis(); ph.sawStatus = false;
      phoneCount++;
      megaLog("phone connected from " + nc.remoteIP().toString() + " (" + phoneCount + " connected)");
    }
  }

  for (uint8_t i = 0; i < MAX_PHONES; i++) {
    Phone &ph = phones[i];
    if (!ph.used) continue;

    if (!ph.c.connected()) {                      // phone gone
      ph.c.stop(); ph.used = false;
      phoneCount--;
      megaLog("phone disconnected (" + String(phoneCount) + " connected)");
      continue;
    }

    // phone commands: some to Zero DCC, the rest to DCC-EX
    static String raw[MAX_PHONES];
    while (ph.c.available() && !zdccQ.full()) {
      char ch = (char)ph.c.read();
      if (DEBUG_PHONES && (uint8_t)ch >= 32 && (uint8_t)ch < 127 && raw[i].length() < 90) raw[i] += ch;
      if (ph.p.feed(ch)) routePhone(i, ph.p.buf, ph.p.len);
    }
    if (DEBUG_PHONES && raw[i].length() && (raw[i].length() >= 90 || !ph.c.available())) {
      megaLog("phone " + String(i + 1) + " sent: " + raw[i]);
      raw[i] = "";
    }
  }
}

// ---------------------------------------------------------------- setup and loop
void setup() {
  Serial.setRxBufferSize(1024);                   // room for Mega commands waiting their turn
  Serial.begin(SERIAL_BAUD);
  pinMode(STATUS_LED, OUTPUT);
  setLed(false);

  String mac = WiFi.softAPmacAddress();           // "AA:BB:CC:DD:EE:FF"
  mac.replace(":", "");
  suffix = mac.substring(6);
  apName = AP_NAME[0] ? String(AP_NAME) : "ZeroDCC_" + suffix;
  apPass = AP_PASSWORD[0] ? String(AP_PASSWORD) : "Password_" + suffix;

  WiFi.persistent(false);                         // don't keep rewriting the settings to flash
  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(false);                   // the search below handles it
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(apName.c_str(), apPass.c_str(), 1, false, 4);
  server.begin();
  server.setNoDelay(true);
  MDNS.begin("zerodcc");                          // zerodcc.local

  delay(200);
  megaLog(String(VERSION) + " - access point " + apName + ", password " + apPass + ", " + AP_IP.toString() + ":" + PORT);
  startSearch("start-up");
  report(true);
}

void loop() {
  unsigned long now = millis();
  searchStep();
  dccexStep();
  megaStep();
  phonesStep();
  MDNS.update();
  report(false);

  if (link == L_SEARCH)      setLed((now / 500) % 2);
  else if (link == L_NOPORT) setLed((now / 100) % 2);
  else if (link == L_LINKED || link == L_SERIAL) setLed(true);
  else                       setLed(false);
}
