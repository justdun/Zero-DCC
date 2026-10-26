//  Zero DCC WiFi bridge for the ESP8266 on a Mega2560+WiFi combo board
//
//  Release Candidate 1.1
//
//  Receives DCC-EX commands from the Mega on the ESP's serial port and
//  forwards them over WiFi to a DCC-EX command station (TCP port 2560).
//  Replies from DCC-EX are passed back to the Mega.
//
//  Board setting in Arduino IDE: "Generic ESP8266 Module" (ESP8266 core installed)
//
//  ONE COMMAND PER PACKET
//  Over WiFi, DCC-EX acts only on the first command in each network packet, and treats
//  a packet that doesn't start with '<' as a WiThrottle message. So the bridge picks out
//  each complete <...> command from the Mega and sends it on its own, with a short gap
//  (COMMAND_GAP_MS) before the next. Line breaks between commands are dropped. Commands
//  waiting their turn are held in the ESP's serial buffer.
//
//  BOARD SETTINGS: Tools -> Crystal Frequency must match the board (26 MHz on the usual
//  combo boards). If it's wrong, the serial speed and the WiFi radio are both off.
//
//  STATUS REPORTS TO THE MEGA
//  Every 2 seconds (and whenever something changes) the bridge sends the Mega a line
//      #ZB <state> <count>
//  state 1 = looking for the WiFi network, 2 = on the WiFi but can't reach DCC-EX,
//  3 = connected to DCC-EX; count = number of commands received from the Mega.
//  ZeroDCC shows this as a coloured dot at the left of its status line, so the link
//  can be checked without a serial monitor. These lines never interrupt a DCC-EX reply.
//
//  STATUS LED (only if your ESP has an LED on GPIO2, active LOW)
//    slow blink = looking for WiFi, fast blink = no DCC-EX, steady = connected,
//    brief flicker off = a command from the Mega was sent to DCC-EX

#include <ESP8266WiFi.h>

const char* WIFI_SSID     = "DCCEX_xxxxxx";   // DCC-EX access point name (or your home network)
const char* WIFI_PASSWORD = "PASS_xxxxxx";    // DCC-EX access point password
IPAddress   DCCEX_IP(192, 168, 4, 1);         // DCC-EX default IP when it runs its own access point
const uint16_t DCCEX_PORT = 2560;

const unsigned long SERIAL_BAUD = 115200;     // must match Serial3.begin() on the Mega
const int STATUS_LED = 2;                     // ESP8266 on-board LED, active LOW
const unsigned long COMMAND_GAP_MS = 20;      // pause between commands sent to DCC-EX

WiFiClient dccex;
unsigned long lastConnectAttempt = 0;
unsigned long flickerUntil = 0;

uint16_t commandCount = 0;                    // commands received from the Mega ('>' characters)
byte reportedState = 0;
uint16_t reportedCount = 0;
unsigned long lastReport = 0;
bool midLine = false;                         // part-way through passing on a DCC-EX reply
unsigned long lastReplyByte = 0;

char command[64];                             // the <...> command being collected from the Mega
byte commandLen = 0;
bool inCommand = false;
unsigned long lastCommandSent = 0;

void setLed(bool on) {
  digitalWrite(STATUS_LED, on ? LOW : HIGH);
}

void countCommands(const uint8_t *data, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (data[i] == '>') commandCount++;
  }
}

void discardSerial() {
  while (Serial.available()) {
    uint8_t c = Serial.read();
    countCommands(&c, 1);
  }
}

// Tell the Mega how the link is doing (never in the middle of a DCC-EX reply)
void report(byte state) {
  unsigned long now = millis();
  if (midLine && now - lastReplyByte > 500) midLine = false;   // a reply was cut off
  if (midLine) return;

  bool changed = (state != reportedState);
  bool counted = (commandCount != reportedCount);
  if (!changed && !(counted && now - lastReport >= 150) && now - lastReport < 2000) return;

  Serial.print("#ZB ");
  Serial.print(state);
  Serial.print(' ');
  Serial.println(commandCount);
  reportedState = state;
  reportedCount = commandCount;
  lastReport = now;
}

// Collect the next complete <...> command from the Mega and send it to DCC-EX on its own.
// Sends at most one command per call, and none until COMMAND_GAP_MS after the last one.
void forwardCommand(unsigned long now) {
  if (now - lastCommandSent < COMMAND_GAP_MS) return;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') {                           // start of a command (restarts a broken one)
      inCommand = true;
      commandLen = 0;
    }
    if (!inCommand) continue;                 // line breaks / anything between commands
    if (commandLen < sizeof(command)) command[commandLen++] = c;
    if (c == '>') {
      inCommand = false;
      if (command[commandLen - 1] == '>') {   // complete (an over-long one is dropped)
        dccex.write((const uint8_t *)command, commandLen);
        commandCount++;
        flickerUntil = now + 60;
        lastCommandSent = now;
      }
      return;                                 // one command per gap
    }
  }
}

void setup() {
  Serial.setRxBufferSize(1024);               // room for commands waiting their turn
  Serial.begin(SERIAL_BAUD);
  pinMode(STATUS_LED, OUTPUT);
  setLed(false);

  WiFi.persistent(false);                     // don't keep rewriting the credentials to flash
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  dccex.setTimeout(1000);                     // give up on a connection attempt after 1 s
}

void loop() {
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    setLed((now / 500) % 2);                  // slow blink: no WiFi yet
    discardSerial();
    inCommand = false;
    midLine = false;
    report(1);
    return;
  }

  if (!dccex.connected()) {
    setLed((now / 100) % 2);                  // fast blink: WiFi OK, no DCC-EX connection
    discardSerial();
    inCommand = false;
    midLine = false;
    report(2);
    if (now - lastConnectAttempt > 2000) {
      lastConnectAttempt = now;
      if (dccex.connect(DCCEX_IP, DCCEX_PORT)) {
        dccex.setNoDelay(true);
      }
    }
    return;
  }

  forwardCommand(now);

  while (dccex.available()) {
    int c = dccex.read();
    Serial.write((uint8_t)c);
    midLine = (c != '\n');
    lastReplyByte = millis();
  }

  report(3);
  setLed((long)(now - flickerUntil) >= 0);    // steady on, except during a flicker
}
