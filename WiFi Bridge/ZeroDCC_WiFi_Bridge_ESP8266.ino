//  Zero DCC WiFi bridge for the ESP8266 on a Mega2560+WiFi combo board
//
//  Receives DCC-EX commands from the Mega on the ESP's serial port and
//  forwards them over WiFi to a DCC-EX command station (TCP port 2560).
//  Replies from DCC-EX are passed back to the Mega.
//
//  Board setting in Arduino IDE: "Generic ESP8266 Module" (ESP8266 core installed)

#include <ESP8266WiFi.h>

const char* WIFI_SSID     = "DCCEX_xxxxxx";   // DCC-EX access point name (or your home network)
const char* WIFI_PASSWORD = "PASS_xxxxxx";    // DCC-EX access point password
IPAddress   DCCEX_IP(192, 168, 4, 1);         // DCC-EX default IP when it runs its own access point
const uint16_t DCCEX_PORT = 2560;

const unsigned long SERIAL_BAUD = 115200;     // must match Serial3.begin() on the Mega
const int STATUS_LED = 2;                     // ESP8266 on-board LED, active LOW

WiFiClient dccex;
unsigned long lastConnectAttempt = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(STATUS_LED, HIGH);
    discardSerial();
    return;
  }

  if (!dccex.connected()) {
    digitalWrite(STATUS_LED, HIGH);
    discardSerial();
    if (millis() - lastConnectAttempt > 2000) {
      lastConnectAttempt = millis();
      if (dccex.connect(DCCEX_IP, DCCEX_PORT)) {
        dccex.setNoDelay(true);
      }
    }
    return;
  }

  digitalWrite(STATUS_LED, LOW);

  uint8_t buf[128];
  size_t n = Serial.available();
  if (n > 0) {
    if (n > sizeof(buf)) n = sizeof(buf);
    n = Serial.readBytes(buf, n);
    dccex.write(buf, n);
  }

  while (dccex.available()) {
    Serial.write(dccex.read());
  }
}

void discardSerial() {
  while (Serial.available()) {
    Serial.read();
  }
}
