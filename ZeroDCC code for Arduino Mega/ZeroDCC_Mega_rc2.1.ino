//  Zero DCC for Arduino Mega
//
//  Release Candidate 2.1
//
//  Hornby Zero 1 decoder and interface to a secondary DCC-EX device via serial connection
//
//  Written and Copyright (C) by Justin M Dunlop (Zero1Guy) 2020-2026
//
//  Please do not distrute without written permission from the author
//
//  NB:  Pin configurations below will need to be followed and/or edited to suit your set up
//
//  Refer to https://github.com/justdun/Zero-DCC for further details and instructons
//    and to https://www.youtube.com/@zero1guy for video demonstrations 
//
//  Update notes:
//  RC1.1 - code improvement to catch potential corruption to incoming Zero 1 data
//        - some arrays reduced in size to meet operational requirement and minimise memory use
//
//  RC1.2 - now broadcasts DCC-EX commands on Serial 0,1,2 & 3 to allow for wired connections plus serial out to WiFi on combo boards.
//
//  RC2.0 - Screen update:
//        - splash and loco screens are drawn in code (ZeroSplash.h, ZeroPanel.h) - no BMP files needed on the SD card
//        - loading status (settings, other settings banks, MicroMimic map file, recordings) shown
//          under the logo on the splash screen and on the serial monitor
//        - track map screen: Accessory 93 ON shows the layout from the SD card file Z_DCCMAP.TXT (DccMap.h),
//          Accessory 93 OFF returns to the loco screen. Points (Accessories 1-89) show green/red dots.
//          If Z_DCCMAP.TXT is missing or has an error at start-up, Accessory 93 is ignored.
//        - Zero 1 / MicroMimic feed (relay) is cut whenever DCC track power is off, and only reconnected
//          when track power is on and no Accessory 90-99 function is running
//        - while track power is off no loco, function or turnout commands are sent to DCC-EX (so it can't
//          get out of step with the layout or with JMRI etc.), and map points and signals don't change.
//          Still sent: power and track-mode commands (Acc 90), programming-track writes (Acc 91) and the
//          speed-0 stop sent when a cab slot is reassigned (Acc 96/97).
//        - change to some of the Accessory 95 logic
//        - while track power is off no loco or turnout commands are sent to DCC-EX (so it can't get out
//          of step with the layout or with JMRI etc.), and map points and signals don't change.
//          Still sent: power and track-mode commands (Acc 90), programming-track writes (Acc 91), the
//          speed-0 stop sent when a cab slot is reassigned (Acc 96/97) and loco functions (Acc 98/99).
//        - Zero 1 / MicroMimic feed (relay) is off at start-up and goes on and off with DCC track power.
//          In Accessory 90, choosing 1 (ON) shows TRACK POWER ON on black; DCC power and the Zero 1
//          feed are only turned on when Acc 90 is closed with 90<- (the status line then turns red).
//          Closing Acc 90 without choosing ON leaves the power as it was.
//          NB: ZeroDCC only knows about power switched with Accessory 90 (DCC-EX does not report back),
//          so these blocks follow the status line on the screen. Every power change is logged on the
//          serial monitor.
//
// RC 2.1 - WiFi combo boards: a coloured dot at the left of the status line shows the ESP8266 bridge's
//          status (white = looking for WiFi, yellow = no DCC-EX, green = connected, grey = stopped
//          reporting, magenta = data on RX3 but not from this bridge, brief black blink = commands
//          arriving from the Mega). No dot = nothing at all received from the ESP on RX3.
//         

#include <SPI.h>
#include <SD.h>

#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

#include "ZeroPanel.h"                      // loco screen background (Zero 1 panel)
#include "ZeroSplash.h"                     // start-up splash screen
#include "DccMap.h"                         // track map screen (Accessory 93)
#include <Fonts/FreeSans9pt7b.h>            // loading status lines on the splash screen

const unsigned long Z1_SplashTime = 3000;   // minimum time the splash screen stays up (ms)
byte Z1_SplashLine = 0;                     // next loading-status line on the splash screen

byte Z1_Pin = 3;                            // NB: This is the pin that receives data from the optocoupler board
byte Z1_Relay_Pin = 5;                      // NB: This is the pin that sends a trigger to the dual relays to cut Zero 1 signal to Zero 1 accessories and MicroMimic
byte Z1_SD_Card_Pin = 48;                   // NB: For the documented TFT screen with built in SD Card Reader this is the default pin setting for the SD card reader, this will need to change if using a different SD card reader
String Z1_Settings_File = "Z_DCC_";
String Z1_Recording_File = "Z_DCCR";
File Z1_PlaybackFile;
bool Z1_PlaybackCommandLoaded = false;
byte Z1_Count = 0;
byte Z1_Data[34];
byte Z1_Tag;
int Z1_LocoNo[ ] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
byte Z1_LocoSpeed[ ] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} ;
byte Z1_LocoDirection[ ] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} ;
byte Z1_ShownSpeed[17];                     // speed / direction currently shown on the loco screen:
byte Z1_ShownDirection[17];                 // the Zero 1 controls, or a recording while it plays back
byte Z1_Accessory99Flag = 0;
byte Z1_Accessory99Function = 0;
byte Z1_Accessory98Flag = 0;
byte Z1_Accessory97Flag = 0;
byte Z1_Accessory97Bank = 1;
byte Z1_Accessory96Flag = 0;
byte Z1_Accessory95Flag = 0;
String Z1_Accessory95File;
byte Z1_Accessory91Flag = 0;
byte Z1_Accessory90Flag = 0;
byte Z1_LocoFN[17][29];
int LocoNumber;
byte j;

String nextcommand;
unsigned long filesize;
unsigned long fileposition;
unsigned long commandtime;
unsigned long rightnow;
unsigned long Z1_PulseDuration;
unsigned long Z1_LastCommandTime;
int AccessoryNo;
byte Z1_TrackType = 2;                      // last track mode set by Accessory 90 (2, 3 or 4)
bool Z1_TrackPowerOn = false;
bool Z1_PowerPending = false;               // ON chosen inside Accessory 90 - applied when Acc 90 is closed with 90<-

// WiFi bridge status (combo boards: the ESP8266 reports "#ZB state count" on Serial3)
byte Z1_BridgeState = 0;                    // 0 never heard, 1 no WiFi, 2 no DCC-EX, 3 connected,
                                            // 8 data on RX3 but no status lines, 9 stopped reporting
unsigned int Z1_BridgeCount = 0;            // commands the bridge has received from the Mega
bool Z1_BridgeCountKnown = false;
unsigned long Z1_BridgeHeard = 0;
unsigned long Z1_BridgeFlashUntil = 0;
bool Z1_BridgeFlashing = false;

volatile uint16_t datapoint[1001];
volatile int savepoint = 1;
int readpoint = 1;
int lastsavepoint = 1;
volatile int chaseflag = 0;

class MultiSerial : public Print {
  public:
    size_t write(uint8_t c) {
      Serial.write(c);
      Serial1.write(c);
      Serial2.write(c);
      Serial3.write(c);
      return 1;
    }
    size_t write(const uint8_t *buffer, size_t size) {
      Serial.write(buffer, size);
      Serial1.write(buffer, size);
      Serial2.write(buffer, size);
      Serial3.write(buffer, size);
      return size;
    }
};

MultiSerial DCC_Out;

Sd2Card card;
SdVolume volume;
SdFile root;
File myfile;

size_t readField(File* file, char* str, size_t size, char* delim) {
  char ch;
  size_t n = 0;
  while ((n + 1) < size && file->read(&ch, 1) == 1) {

    if (ch == '\r') {
      continue;
    }
    str[n++] = ch;
    if (strchr(delim, ch)) {
      break;
    }
  }
  str[n] = '\0';
  return n;
}

#define errorHalt(msg) {Serial.println(F(msg)); while(1);}

// One loading-status line: printed to the serial monitor and shown under the
// logo on the splash screen (landscape, left-aligned with the logo, one line per call)
void Splash_Status(const char *msg) {

  Serial.println(msg);

  int16_t x1, y1;
  uint16_t w, h;
  tft.setRotation(1);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setTextColor(0x0000);
  tft.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(ZS_Y0 - x1, 222 + Z1_SplashLine * 24);      // ZS_Y0 = left edge of the logo
  tft.print(msg);
  tft.setFont();
  tft.setRotation(0);
  Z1_SplashLine++;

}

void Splash_Status(const __FlashStringHelper *msg) {

  char buf[60];
  strncpy_P(buf, (const char *)msg, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  Splash_Status(buf);

}

// Count the other settings banks (Z_DCC_2.TXT .. Z_DCC_10.TXT) and the
// session recordings (Z_DCCR<n>.TXT) in the root of the SD card
void Count_SD_Files(byte &otherSettings, byte &recordings) {

  otherSettings = 0;
  recordings = 0;
  File dir = SD.open("/");
  if (!dir) return;

  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    const char *n = entry.name();
    if (!entry.isDirectory() && strstr_P(n, PSTR(".TXT")) != NULL) {
      if (strncasecmp_P(n, PSTR("Z_DCCR"), 6) == 0 && isdigit(n[6])) {
        recordings++;
      }
      else if (strncasecmp_P(n, PSTR("Z_DCC_"), 6) == 0 && isdigit(n[6])) {
        int bank = atoi(n + 6);
        if (bank >= 2 && bank <= 10) otherSettings++;
      }
    }
    entry.close();
  }
  dir.close();

}

void setup()

{

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  while (!Serial) {
    ;
  }

  Serial.println(F("Welcome to Zero DCC!"));
  Serial.println();
  Serial.println(F("You should only see DCC++EX commands here (or errors if things go wrong)."));
  Serial.println();

  pinMode (digitalPinToInterrupt(Z1_Pin), INPUT);

  pinMode(Z1_Relay_Pin, OUTPUT);
  digitalWrite(Z1_Relay_Pin, HIGH);         // Zero 1 / MicroMimic feed cut until DCC track power is turned on

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GRAY    0x8410
#define Z_DCC   ZP_WHITE                    // text background = label area of the panel
#define Z_DCC_RED ZP_RED                    // status line red = panel red

  uint16_t ID;
  ID = tft.readID();
  if (ID == 0x0D3D3) ID = 0x7796;
  tft.begin(ID);

  drawZeroSplash(tft);
  unsigned long splashStart = millis();

  pinMode(Z1_SD_Card_Pin, OUTPUT);

 if (!SD.begin(Z1_SD_Card_Pin))

  {
    Serial.println(F("SD Card failed."));
    Splash_Status(F("Settings not found - using defaults"));
    Splash_Status(F("No additional settings files found"));
    Splash_Status(F("No MicroMimic file found - function not available"));
    Splash_Status(F("No recordings found"));
  }

  else {

    String Filename = String (Z1_Settings_File + "1.txt");

    if (SD.exists(Filename)) {
      Splash_Status(F("Loading settings file"));
      myfile = SD.open(Filename , FILE_WRITE);
    }
    else {
      Splash_Status(F("Settings not found - using defaults"));
    }
    if (myfile) {

      myfile.seek(0);
      size_t n;
      char str[20];
      j = 1;

      while (true) {

        n = readField(&myfile, str, sizeof(str), ",\n");

        if (n == 0) {
          myfile.close();
          break;
        }

        if (str[n - 1] == ',' || str[n - 1] == '\n') {

          str[n - 1] = 0;
        }

        else {

          Serial.print(myfile.available() ? F("error: ") : F("eof:   "));
        }

        Z1_LocoNo[j++] = atoi(str);

      }
      myfile.close();
    }

    byte otherSettings, recordings;
    Count_SD_Files(otherSettings, recordings);
    char msg[48];

    if (otherSettings == 0) {
      Splash_Status(F("No additional settings files found"));
    }
    else {
      snprintf_P(msg, sizeof(msg), PSTR("%d other settings file%s found"), otherSettings, otherSettings == 1 ? "" : "s");
      Splash_Status(msg);
    }

    if (!SD.exists(DCC_MAP_FILE)) {
      Splash_Status(F("No MicroMimic file found - function not available"));
    }
    else if (dccMapLoad() < 0) {            // track map for Accessory 93 (details on the serial monitor)
      Splash_Status(F("MicroMimic file has errors - function not available"));
    }
    else {
      Splash_Status(F("MicroMimic file found"));
    }

    if (recordings == 0) {
      Splash_Status(F("No recordings found"));
    }
    else {
      snprintf_P(msg, sizeof(msg), PSTR("%d session recording%s found"), recordings, recordings == 1 ? "" : "s");
      Splash_Status(msg);
    }

  }

  while (millis() - splashStart < Z1_SplashTime) {
    ;
  }

  tft.setTextColor(BLACK, Z_DCC);
  tft.setTextSize(2);
  drawZeroPanel(tft);
  Update_Screen_Locos();

  Z1_TrackType = 2;
  Update_Screen_Track_Type();
  Track_Power_Off();

  attachInterrupt (digitalPinToInterrupt(Z1_Pin), Z1_CALC_PULSE, CHANGE);

}

void loop()
{

  interrupts();
  Data_Cycle();
  Read_Bridge();

  if (Z1_Accessory95Flag == 3) {
    Playback_Cycle();
  }

}

void Z1_CALC_PULSE()
{

  if ( digitalRead( Z1_Pin ) == HIGH )
  {

    datapoint[savepoint] = micros();

    if (savepoint == 1000) {
      savepoint = 1;
      chaseflag = 1;
    }
    else {
      savepoint = savepoint + 1;
    }
  }
}

void Data_Cycle()
{

  int lastchaseflag;
  uint16_t thisPulse;
  uint16_t nextPulse;

  noInterrupts();
  lastsavepoint = savepoint;
  lastchaseflag = chaseflag;
  interrupts();

  if (((lastsavepoint > (readpoint + 2) ) && (lastchaseflag == 0))  || ((lastsavepoint < readpoint ) && (lastchaseflag == 1))) {

    noInterrupts();
    thisPulse = datapoint[readpoint];
    if (readpoint == 1000) {
      nextPulse = datapoint[1];
    }
    else {
      nextPulse = datapoint[readpoint + 1];
    }
    interrupts();

    Z1_PulseDuration = (uint16_t)(nextPulse - thisPulse);

    if (readpoint == 1000) {
      readpoint = 1;
      noInterrupts();
      chaseflag = 0;
      interrupts();
    }
    else  {
      readpoint = readpoint + 1;
    }

    delay(1);

    if (Z1_PulseDuration > 1000) {
      if (Z1_Count == 33) {
        decoder();
      }
      Z1_Count = 0;
      }

    else if (Z1_PulseDuration > 10 && Z1_PulseDuration < 150) {
      if (Z1_Count < 33) {
        Z1_Count = Z1_Count + 1;
        Z1_Data[Z1_Count] = 0;
      }
      }

    else if (Z1_PulseDuration > 160 && Z1_PulseDuration < 250) {
      if (Z1_Count < 33) {
        Z1_Count = Z1_Count + 1;
        Z1_Data[Z1_Count] = 1;
      }
      }

  }

}

void decoder() {

  if ((Z1_Data[8] + Z1_Data[7] + Z1_Data[6] + Z1_Data[5]) == 0) {

    AccessoryNo = (((Z1_Data[13] * 1) + (Z1_Data[14] * 2) + (Z1_Data[15] * 4) + (Z1_Data[16] * 8)) * 10) + ((Z1_Data[17] * 1) + (Z1_Data[18] * 2) + (Z1_Data[19] * 4) + (Z1_Data[20] * 8));

    if (Z1_Accessory99Flag == 1 && AccessoryNo != 99 ) {
      Accessory99();
    }

    else if (Z1_Accessory98Flag == 1 && AccessoryNo != 98 ) {
      Accessory98();
    }

    else if (Z1_Accessory97Flag == 1 && AccessoryNo != 97 && Z1_Accessory95Flag == 0) {
      Accessory97();
    }

    else if (Z1_Accessory96Flag == 1 && AccessoryNo != 96 && Z1_Accessory95Flag == 0)  {
      Accessory96();
    }

    // Accessory 95 - record / play back. "95<-" always leaves the function.
    else if (AccessoryNo == 95 && Z1_Accessory95Flag == 3) {    // playing back: 95 either way stops it
      Playback_Stop();
    }

    else if (AccessoryNo == 95 && Z1_Accessory95Flag == 2) {    // recording: 95 either way ends and saves it
      Recording_Stop();
    }

    else if (Z1_Accessory95Flag == 1 ) {                         // waiting for a recording number
      if (AccessoryNo == 95) {
        if (Z1_Data[24] == 1) {                                  // 95<- : cancel
          Accessory95_Cancel();
        }                                                        // 95-> : ignored, still waiting
      }
      else {
        Accessory95();                                           // n-> record, n<- play back
      }
    }

    else if (Z1_Accessory91Flag == 1 && AccessoryNo != 91 && Z1_Accessory95Flag == 0 ) {
      Accessory91();
    }

    else if (Z1_Accessory90Flag == 1 && AccessoryNo != 90 && Z1_Accessory95Flag == 0 ) {
      Accessory90();
    }

    else if (AccessoryNo == 99) {
      if (Z1_Data[24] == 1) {
        LocoNumber = 0;
        Z1_Accessory99Flag = 0;
        Z1_Accessory99Function = 0;
        Relay_Restore();
      }
      else {
        Z1_Accessory99Flag = 1;
        LocoNumber = 0;
        digitalWrite(Z1_Relay_Pin, HIGH);
      }
    }

    else if (AccessoryNo == 98) {
      if (Z1_Data[24] == 1) {
        Z1_Accessory98Flag = 0;
        LocoNumber = 0;
        Relay_Restore();
      }
      else {
        Z1_Accessory98Flag = 1;
        LocoNumber = 1;
        digitalWrite(Z1_Relay_Pin, HIGH);
      }
    }

    else if (AccessoryNo == 97) {
      if (Z1_Data[24] == 1) {
        Z1_Accessory97Flag = 0;
        LocoNumber = 0;
        Relay_Restore();
      }
      else {
        Z1_Accessory97Flag = 1;
        LocoNumber = 0;
        digitalWrite(Z1_Relay_Pin, HIGH);
      }
    }

    else if (AccessoryNo == 95 && Z1_Accessory95Flag == 0) {

      if (Z1_Data[24] == 0) {                                    // 95-> : wait for a recording number
        Z1_Accessory95Flag = 1;
        digitalWrite(Z1_Relay_Pin, HIGH);
        Serial.println(F("Acc 95: choose a recording number (forward = record, back = play) or 95 back to cancel"));
      }                                                          // 95<- : already off, ignored

    }

    else if (AccessoryNo == 96) {
      if (Z1_Data[24] == 1) {
        Z1_Accessory96Flag = 0;
        LocoNumber = 0;
        Relay_Restore();
      }
      else {
        Z1_Accessory96Flag = 1;
        LocoNumber = 0;
        digitalWrite(Z1_Relay_Pin, HIGH);
      }
    }

    else if (AccessoryNo == 91) {
      if (Z1_Data[24] == 1) {
        Z1_Accessory91Flag = 0;
        LocoNumber = 0;
        Relay_Restore();
      }
      else {
        Z1_Accessory91Flag = 1;
        LocoNumber = 0;
        digitalWrite(Z1_Relay_Pin, HIGH);
      }
    }

    else if (AccessoryNo == 90) {
      if (Z1_Data[24] == 1) {
        Z1_Accessory90Flag = 0;
        LocoNumber = 0;
        if (Z1_PowerPending) {              // ON was chosen: DCC power and Zero 1 feed on together
          Z1_PowerPending = false;
          Track_Power_On();
        }
        Relay_Restore();
      }
      else {
        Z1_Accessory90Flag = 1;
        LocoNumber = 0;
        digitalWrite(Z1_Relay_Pin, HIGH);
      }
    }

    else if (AccessoryNo == MAP_SCREEN_ACC) {          // Accessory 93: track map screen
      dccMapAccessory(tft, MAP_SCREEN_ACC, Z1_Data[24] == 0);   // ON shows the track map, OFF returns to the loco screen
    }

    else if (AccessoryNo < 90 && Z1_Accessory91Flag == 0 && Z1_Accessory96Flag == 0 && Z1_Accessory97Flag == 0 && Z1_Accessory98Flag == 0 && Z1_Accessory99Flag == 0 ) {
      Point_Routine();
    }

  }

  else {
    Locomotive_Routine();
  }

  Z1_Count = 0;

}

void Locomotive_Routine() {

  // Track power off: send nothing to DCC-EX and leave the stored speeds alone, so the
  // first Zero 1 update after power comes back on sends every loco's current setting
  if (!Z1_TrackPowerOn) return;

  Z1_Tag = (Z1_Data[3] * 1) + (Z1_Data[4] * 2);

  int LocoSpeed[5];
  int LocoDirection[5];
  j = 1;
  for (j = 1 ; j <= 4 ; j++) {
    int Change = 0;

    LocoSpeed[j] = ((Z1_Data[1 + (j * 4)] * 1) + (Z1_Data[2 + (j * 4)] * 2) + (Z1_Data[3 + (j * 4)] * 4) + (Z1_Data[4 + (j * 4)] * 8) - 1);

    LocoSpeed[j] = map(LocoSpeed[j], 0, 14, 0, 126);

    if (Z1_LocoSpeed [j + (Z1_Tag * 4)] != LocoSpeed[j] )  {
      Change = 1;
      Z1_LocoSpeed[j + (Z1_Tag * 4)] = LocoSpeed[j];
    }

    LocoDirection[j] = Z1_Data [20 + j];

    if (Z1_LocoDirection[j + (Z1_Tag * 4)] != LocoDirection[j]) {
      Change = 1;
      Z1_LocoDirection[j + (Z1_Tag * 4)] = LocoDirection[j];
    }

    if (Change == 1) {
      DCC_Out.print("<t ");
      DCC_Out.print(j + (Z1_Tag * 4));
      DCC_Out.print(" ");
      DCC_Out.print(Z1_LocoNo[j + (Z1_Tag * 4)]);
      DCC_Out.print(" ");
      DCC_Out.print(Z1_LocoSpeed[j + (Z1_Tag * 4)]);
      DCC_Out.print(" ");
      DCC_Out.print(Z1_LocoDirection[j + (Z1_Tag * 4)]);
      DCC_Out.println(">");

      if ( Z1_Accessory95Flag == 2) {
        rightnow = millis();
        myfile = SD.open(Z1_Accessory95File , FILE_WRITE);

        Serial.print("Writing to file:");
        if (myfile) {
          myfile.print(rightnow - Z1_LastCommandTime);
          myfile.print ("\n");
          myfile.print("<t ");
          myfile.print(j + (Z1_Tag * 4));
          myfile.print(" ");
          myfile.print(Z1_LocoNo[j + (Z1_Tag * 4)]);
          myfile.print(" ");
          myfile.print(Z1_LocoSpeed[j + (Z1_Tag * 4)]);
          myfile.print(" ");
          myfile.print(Z1_LocoDirection[j + (Z1_Tag * 4)]);
          myfile.print(">");
          myfile.print( "\n" );
          myfile.close();
        }
        else {

          Serial.println("error opening file");
        }
        Z1_LastCommandTime = rightnow;
      }
      Update_Screen_Speeds(int (j + (Z1_Tag * 4)));
      Change = 0;
    }
  }

}

void Point_Routine() {

  if (!Z1_TrackPowerOn) return;             // track power off: no turnout commands, map unchanged

  if (Z1_Data[24] == 1) {
    DCC_Out.print("<T ");
    DCC_Out.print(AccessoryNo);
    DCC_Out.println(" 1>");

    if ( Z1_Accessory95Flag == 2) {
      rightnow = millis();
      myfile = SD.open(Z1_Accessory95File , FILE_WRITE);

      Serial.print("Writing to file:");
      if (myfile) {
        myfile.print(rightnow - Z1_LastCommandTime);
        myfile.print ("\n");
        myfile.print("<T ");
        myfile.print(AccessoryNo);
        myfile.println(" 1>");
        myfile.print( "\n" );
        myfile.close();
      }
      else {

        Serial.println("error opening file");
      }
      Z1_LastCommandTime = rightnow;
    }

  }

  else {
    DCC_Out.print("<T ");
    DCC_Out.print(AccessoryNo);
    DCC_Out.println(" 0>");

    if ( Z1_Accessory95Flag == 2) {
      rightnow = millis();
      myfile = SD.open(Z1_Accessory95File , FILE_WRITE);

      Serial.print("Writing to file:");
      if (myfile) {
        myfile.print(rightnow - Z1_LastCommandTime);
        myfile.print ("\n");
        myfile.print("<T ");
        myfile.print(AccessoryNo);
        myfile.println(" 0>");
        myfile.print( "\n" );
        myfile.close();
      }
      else {

        Serial.println("error opening file");
      }
      Z1_LastCommandTime = rightnow;
    }

  }

  if (Z1_TrackPowerOn) {                    // nothing can move with track power off, so leave the map as it is
    dccMapAccessory(tft, AccessoryNo, Z1_Data[24] == 1);   // update the point's / signal's dots on the map screen
  }

}

void Accessory90() {

  Serial.print(F("Acc 90 key "));
  Serial.println(AccessoryNo);

  if (AccessoryNo == 1) {                   // ON: chosen now, sent when Acc 90 is closed with 90<-
     Z1_PowerPending = true;
     Serial.println(F("Track power ON chosen - applied when Acc 90 is closed"));
     Draw_Track_Power();
     return;
  }

  Z1_PowerPending = false;                  // any other choice turns power off

  if (AccessoryNo == 2) {
     Z1_TrackType = 2;
     Track_Power_Off();
     DCC_Out.println("<= A MAIN >");
     DCC_Out.println("<= B PROG >");
     Update_Screen_Track_Type();
     }
  else if (AccessoryNo == 3) {
     Z1_TrackType = 3;
     Track_Power_Off();
     DCC_Out.println("<= A MAIN >");
     DCC_Out.println("<= B DC 999 >");
     Update_Screen_Track_Type();
     }
  else if (AccessoryNo == 4) {
     Z1_TrackType = 4;
     Track_Power_Off();
     DCC_Out.println("<= A DC 999 >");
     DCC_Out.println("<= B DC 998 >");
     Update_Screen_Track_Type();
     }

  else {
     Track_Power_Off();
  }

}

void Accessory91() {

  if (Z1_Data[24] == 1) {
    if ( AccessoryNo == 10 ) {
      AccessoryNo = 0;
    }
    if ( LocoNumber == 0 ) {
      LocoNumber = AccessoryNo;
    }
    else {
      LocoNumber = (LocoNumber * 10) + AccessoryNo;
    }
  }

  else if (Z1_Data[24] == 0) {

    if ( LocoNumber > 0 && LocoNumber <= 9999) {

        DCC_Out.print("<W ");
        DCC_Out.print(LocoNumber);
        DCC_Out.println(">");
        delay(500);
      }
      Z1_Accessory91Flag = 0;
      LocoNumber = 0;
      Relay_Restore();
    }

    else {

      Z1_Accessory91Flag = 0;
      LocoNumber = 0;
      Relay_Restore();
    }

}

// Recording number chosen: n-> records to Z_DCCR<n>.txt, n<- plays it back
void Accessory95() {

  Relay_Restore();
  Z1_Accessory95File = String (Z1_Recording_File + String(AccessoryNo, DEC) + ".txt");

  if (Z1_Data[24] == 0) {

    Z1_LastCommandTime = millis();

    if (SD.exists(Z1_Accessory95File)) {
      SD.remove(Z1_Accessory95File);
    }

    myfile = SD.open(Z1_Accessory95File , FILE_WRITE);

    if (myfile) {
      myfile.close();
      Z1_Accessory95Flag = 2;
      Serial.print(F("Recording to "));
      Serial.println(Z1_Accessory95File);
    }
    else {
      Z1_Accessory95Flag = 0;
      Serial.println(F("Recording failed: error opening file"));
    }

  }

  else {

    Z1_Accessory95Flag = 0;
    if (SD.exists(Z1_Accessory95File)) {
      Z1_PlaybackFile = SD.open(Z1_Accessory95File, FILE_READ);
      if (Z1_PlaybackFile) {
        filesize = Z1_PlaybackFile.size();
        fileposition = 0;
        Z1_PlaybackCommandLoaded = false;
        Z1_Accessory95Flag = 3;
        Serial.print(F("Playing "));
        Serial.println(Z1_Accessory95File);
      }
    }
    if (Z1_Accessory95Flag != 3) {
      Serial.print(F("Recording not found: "));
      Serial.println(Z1_Accessory95File);
    }

  }

}

void Recording_Stop() {

  Z1_Accessory95Flag = 0;
  Serial.println(F("Recording stopped."));

}

void Accessory95_Cancel() {

  Z1_Accessory95Flag = 0;
  Relay_Restore();
  Serial.println(F("Acc 95 cancelled."));

}

void Playback_Cycle() {
  if (!Z1_PlaybackCommandLoaded) {
    if (fileposition >= filesize) {
      Playback_Stop();
      return;
    }
    readnextcommand();
    Z1_PlaybackCommandLoaded = true;
  }

  if (millis() >= commandtime) {
    if (Z1_TrackPowerOn) {                  // track power off: the recording keeps time but sends nothing
      DCC_Out.println(nextcommand);
      Playback_Show_Command(nextcommand);
    }
    Z1_PlaybackCommandLoaded = false;
  }
}

// Keep the screens in step with a recording during playback:
//   "<t slot address speed direction>"  -> that cab's speed arrows on the loco screen
//   "<T accessory 0/1>"                 -> the point / signal dots on the map
// The recorded speeds are only shown, not stored as the Zero 1 control settings, so
// the next change on the Zero 1 controller still takes over as normal.
void Playback_Show_Command(const String &cmd) {
  if (cmd.startsWith("<t ")) {
    int slot, address, speed, direction;
    if (sscanf(cmd.c_str(), "<t %d %d %d %d", &slot, &address, &speed, &direction) == 4 && slot >= 1 && slot <= 16) {
      Z1_ShownSpeed[slot] = speed;
      Z1_ShownDirection[slot] = direction;
      Draw_Speed(slot);
    }
    return;
  }
  if (!cmd.startsWith("<T ")) return;
  int sp = cmd.indexOf(' ', 3);
  if (sp < 0) return;
  int acc = cmd.substring(3, sp).toInt();
  if (acc < MAP_FIRST_ACC || acc > MAP_LAST_ACC) return;
  if (!Z1_TrackPowerOn) return;             // map only changes while track power is on
  dccMapAccessory(tft, acc, cmd.charAt(sp + 1) == '1');
}

void readnextcommand() {
  Z1_PlaybackFile.seek(fileposition);

  size_t n;
  char str[20];

  n = readField(&Z1_PlaybackFile, str, sizeof(str), ",\n");
  if (n == 0) { fileposition = filesize; return; }
  if (str[n - 1] == ',' || str[n - 1] == '\n') str[n - 1] = 0;
  rightnow = millis();
  commandtime = rightnow + strtol(str, NULL, 10);

  n = readField(&Z1_PlaybackFile, str, sizeof(str), ",\n");
  if (n == 0) { fileposition = filesize; return; }
  if (str[n - 1] == ',' || str[n - 1] == '\n') str[n - 1] = 0;
  nextcommand = str;

  fileposition = Z1_PlaybackFile.position();
}

void Playback_Stop() {
  if (Z1_PlaybackFile) {
    Z1_PlaybackFile.close();
  }
  Z1_PlaybackCommandLoaded = false;
  Z1_Accessory95Flag = 0;
  Serial.println("Playback stopped.");
}

void Accessory96() {
  String Filename;

  if (Z1_Data[24] == 0 && AccessoryNo < 17) {
    Z1_LocoSpeed[AccessoryNo] = 0;
    Z1_LocoDirection[AccessoryNo] = 0;
    DCC_Out.print("<t ");
    DCC_Out.print(AccessoryNo);
    DCC_Out.print(" ");
    DCC_Out.print(Z1_LocoNo[AccessoryNo]);
    DCC_Out.println(" 0 0>");

    Z1_LocoNo[AccessoryNo] = LocoNumber;
    LocoNumber = 0;
    Update_Screen_Locos();

    Filename = String (Z1_Settings_File + String(Z1_Accessory97Bank, DEC) + ".txt");
    if (SD.exists(Filename)) {
      SD.remove(Filename);
    }
    myfile = SD.open(Filename , FILE_WRITE);

    Serial.print("Writing to file : ");
    if (myfile) {
      for (j = 1; j <= 16; j++) {
        myfile.print(Z1_LocoNo[j]);
        myfile.print( "\n" );
      }

      Serial.println("done.");
    }
    else {

      Serial.println("error opening file");
    }

    myfile.close();

  }

  else if (Z1_Data[24] == 1)  {
    if ( AccessoryNo == 10 ) {
      AccessoryNo = 0;
    }
    if ( LocoNumber == 0 ) {
      LocoNumber = AccessoryNo;
    }
    else {
      LocoNumber = (LocoNumber * 10) + AccessoryNo;
    }
  }

}

void Accessory97() {

  String Filename;

  if ( AccessoryNo <= 10 ) {
    int k;
    Z1_Accessory97Bank = byte(AccessoryNo);

    for (k = 1 ; k <= 16 ; k++) {
      Z1_LocoSpeed[k] = 0;
      Z1_LocoDirection[k] = 0;
      DCC_Out.print("<t ");
      DCC_Out.print(k);
      DCC_Out.print(" ");
      DCC_Out.print(Z1_LocoNo[k]);
      DCC_Out.println(" 0 0>");
    }

    Filename = String (Z1_Settings_File + String(Z1_Accessory97Bank, DEC) + ".txt");
    if (SD.exists(Filename)) {
      myfile = SD.open(Filename , FILE_WRITE);
      if (myfile) {

        myfile.seek(0);
        size_t n;
        char str[20];
        j = 1;

        while (true) {
          n = readField(&myfile, str, sizeof(str), ",\n");

          if (n == 0) break;

          if (str[n - 1] == ',' || str[n - 1] == '\n') {
            Serial.print(str[n - 1] == ',' ? F("comma: ") : F("endl:  "));

            str[n - 1] = 0;
          }

          else {

            Serial.print(myfile.available() ? F("error: ") : F("eof:   "));
          }

          Z1_LocoNo[j++] = atoi(str);

        }
      }

      myfile.close();
    }

    Update_Screen_Locos();
  }

  Z1_Accessory97Flag = 0;
  LocoNumber = 0;
  Relay_Restore();

}

void Accessory98() {

  LocoNumber = AccessoryNo;
  if (Z1_Data[24] == 1) {
    if (Z1_LocoFN[LocoNumber][0] == 0) {
      Z1_LocoFN[LocoNumber][0] = 1;
    }
    else {
      Z1_LocoFN[LocoNumber][0] = 0;
    }

    DCC_Out.print("<F ");
    DCC_Out.print(Z1_LocoNo[LocoNumber]);
    DCC_Out.print(" 0 ");
    DCC_Out.print(Z1_LocoFN[LocoNumber][0]);
    DCC_Out.println(">");

    if ( Z1_Accessory95Flag == 2) {
      rightnow = millis();
      myfile = SD.open(Z1_Accessory95File , FILE_WRITE);

      Serial.print("Writing to file:");
      if (myfile) {
        myfile.print(rightnow - Z1_LastCommandTime);
        myfile.print ("\n");
        myfile.print("<F ");
        myfile.print(Z1_LocoNo[LocoNumber]);
        myfile.print(" 0 ");
        myfile.print(Z1_LocoFN[LocoNumber][0]);
        myfile.print(">");
        myfile.print( "\n" );
        myfile.close();

      }
      else {

        Serial.println("error opening file");
      }
      Z1_LastCommandTime = rightnow;
    }
  }

  if (Z1_Data[24] == 0) {

    if (Z1_LocoFN[LocoNumber][2] == 0) {
      Z1_LocoFN[LocoNumber][2] = 1;
    }
    else {
      Z1_LocoFN[LocoNumber][2] = 0;
    }

    DCC_Out.print("<F ");
    DCC_Out.print(Z1_LocoNo[LocoNumber]);
    DCC_Out.print(" 2 ");
    DCC_Out.print(Z1_LocoFN[LocoNumber][2]);
    DCC_Out.println(">");

    if ( Z1_Accessory95Flag == 2) {
      rightnow = millis();
      myfile = SD.open(Z1_Accessory95File , FILE_WRITE);

      Serial.print("Writing to file:");
      if (myfile) {
        myfile.print(rightnow - Z1_LastCommandTime);
        myfile.print ("\n");
        myfile.print("<F ");
        myfile.print(Z1_LocoNo[LocoNumber]);
        myfile.print(" 2 ");
        myfile.print(Z1_LocoFN[LocoNumber][2]);
        myfile.print(">");
        myfile.print( "\n" );
        myfile.close();

      }
      else {

        Serial.println("error opening file");
      }
      Z1_LastCommandTime = rightnow;
    }

  }

}

void Accessory99() {

  if ( LocoNumber == 0 ) {
    LocoNumber = AccessoryNo;
  }

  else  {
    Z1_Accessory99Function = AccessoryNo;

    if ( Z1_Accessory99Function == 29 )  {
      Z1_Accessory99Function = 0;
    }

    if (Z1_LocoFN[LocoNumber][Z1_Accessory99Function] == 0) {
      Z1_LocoFN[LocoNumber][Z1_Accessory99Function] = 1;
    }
    else {
      Z1_LocoFN[LocoNumber][Z1_Accessory99Function] = 0;
    }

    DCC_Out.print("<F ");
    DCC_Out.print(Z1_LocoNo[LocoNumber]);
    DCC_Out.print(" ");
    DCC_Out.print(Z1_Accessory99Function);
    DCC_Out.print(" ");
    DCC_Out.print(Z1_LocoFN[LocoNumber][Z1_Accessory99Function]);
    DCC_Out.println(">");

    if ( Z1_Accessory95Flag == 2) {
      rightnow = millis();
      myfile = SD.open(Z1_Accessory95File , FILE_WRITE);

      Serial.print("Writing to file:");
      if (myfile) {
        myfile.print(rightnow - Z1_LastCommandTime);
        myfile.print ("\n");
        myfile.print("<F ");
        myfile.print(Z1_LocoNo[LocoNumber]);
        myfile.print(" ");
        myfile.print(Z1_Accessory99Function);
        myfile.print(" ");
        myfile.print(Z1_LocoFN[LocoNumber][Z1_Accessory99Function]);
        myfile.print(">");
        myfile.print( "\n" );
        myfile.close();
      }
      else {
        Serial.println("error opening file");
      }
      Z1_LastCommandTime = rightnow;
    }
  }

}

// Status line: same position on the loco screen and the map screen
int Status_Line_Y() {
  return MAP_STATUS_Y;
}

void Update_Screen_Track_Type() {

  tft.setRotation(1);
  tft.setTextColor(BLACK, Z_DCC_RED);
  tft.setTextSize(2);
  tft.setCursor(40, Status_Line_Y());

  if (Z1_TrackType == 2)       { tft.print("A:MAIN    B:PROG    "); }
  else if (Z1_TrackType == 3)  { tft.print("A:MAIN    B:DC (999)"); }
  else if (Z1_TrackType == 4)  { tft.print("A:DC(999) B:DCX(998)"); }
  tft.setRotation(0);

}

void Draw_Track_Power() {

  tft.setRotation(1);
  tft.setTextSize(2);
  tft.setCursor(290, Status_Line_Y());
  if (Z1_TrackPowerOn) {
    tft.setTextColor(BLACK, Z_DCC_RED);
    tft.print("TRACK POWER ON ");
  }
  else if (Z1_PowerPending) {               // chosen in Acc 90, not on yet
    tft.setTextColor(Z_DCC_RED, BLACK);
    tft.print("TRACK POWER ON ");
  }
  else {
    tft.setTextColor(Z_DCC_RED, BLACK);
    tft.print("TRACK POWER OFF");
  }
  tft.setRotation(0);

}

// Reconnect the Zero 1 / MicroMimic feed at the end of an Accessory 90-99 function,
// but only while DCC track power is on - Zero 1 power goes on and off with DCC power
void Relay_Restore() {

  digitalWrite(Z1_Relay_Pin, Z1_TrackPowerOn ? LOW : HIGH);

}

void Track_Power_On() {

  Z1_TrackPowerOn = true;
  Serial.println(F("Track power ON"));
  Draw_Track_Power();
  DCC_Out.println("<1>");

}

void Track_Power_Off() {

  Z1_TrackPowerOn = false;
  Serial.println(F("Track power OFF"));
  digitalWrite(Z1_Relay_Pin, HIGH);         // Zero 1 / MicroMimic feed off with DCC power
  Draw_Track_Power();
  DCC_Out.println("<0>");

}

// Text sits centred in the sloped panel rows (ZeroPanel.h): zpRowCentreY(row, x) gives the row's
// centre at column x, so each character follows the slope of its row
void Update_Screen_Locos() {
  int k;

  if (dccMapActive()) return;

  tft.setTextColor(BLACK, Z_DCC);
  tft.setTextSize(2);

  for (k = 1; k <= 16; k++)   {
    tft.setCursor(10, zpRowCentreY(k, 34) - 7);       // 4 characters wide, centred at x = 34
    tft.print("    ");
    tft.setCursor(10, zpRowCentreY(k, 34) - 7);
    tft.print(String(Z1_LocoNo[k], DEC));
  }

}

// Zero 1 control change: show the cab's new speed and direction
void Update_Screen_Speeds(int k) {

  Z1_ShownSpeed[k] = Z1_LocoSpeed[k];
  Z1_ShownDirection[k] = Z1_LocoDirection[k];
  Draw_Speed(k);

}

// Draw the speed arrows for cab k from what is currently shown
void Draw_Speed(int k) {
  int l;

  if (dccMapActive()) return;

  tft.setTextColor(BLUE, Z_DCC);
  tft.setTextSize(2);

  for (l = 1; l <= 15; l++) {
    tft.setCursor(((l * 10) + 58), zpRowCentreY(k, (l * 10) + 64) - 7);
    tft.print(" ");
  }

  for (l = 1; l <= ((Z1_ShownSpeed[k] / 9) + 1); l++) {
    tft.setCursor(((l * 10) + 58), zpRowCentreY(k, (l * 10) + 64) - 7);

    if (Z1_ShownDirection[k] == 1) {
      tft.print(">");
    }
    else {
      tft.print("<");
    }
  }

}

// ---- WiFi bridge status dot (left end of the status line) ----
//   no dot  - nothing heard from the ESP8266 (bridge not running, or no link from the ESP back to RX3)
//   white   - bridge running, looking for the WiFi network
//   yellow  - on the WiFi, but can't reach DCC-EX
//   green   - connected to DCC-EX
//   grey    - the bridge has stopped reporting
//   magenta - something is arriving on RX3 but it isn't the bridge's status lines
//             (the ESP is running other firmware or an older bridge, or the baud rate is wrong)
//   blinks black briefly - the bridge has just received commands from the Mega (TX3 -> ESP works)
void Read_Bridge() {

  static char line[24];
  static byte len = 0;
  static bool skip = false;

  while (Serial3.available()) {
    char c = Serial3.read();
    if (Z1_BridgeState == 0) {               // first sign of life on RX3
      Z1_BridgeState = 8;
      Serial.println(F("WiFi bridge: data on RX3, but no status lines yet"));
      Draw_Bridge();
    }
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = 0;
      if (!skip) Bridge_Line(line);
      len = 0;
      skip = false;
    }
    else if (len < sizeof(line) - 1) line[len++] = c;
    else skip = true;                        // too long: a DCC-EX reply, not a status line
  }

  unsigned long now = millis();
  if (Z1_BridgeState >= 1 && Z1_BridgeState <= 3 && now - Z1_BridgeHeard > 6000) {
    Z1_BridgeState = 9;
    Serial.println(F("WiFi bridge: stopped reporting"));
    Draw_Bridge();
  }
  if (Z1_BridgeFlashing && (long)(now - Z1_BridgeFlashUntil) >= 0) {
    Z1_BridgeFlashing = false;
    Draw_Bridge();
  }

}

void Bridge_Line(const char *line) {

  int state;
  unsigned int count;
  if (strncmp_P(line, PSTR("#ZB "), 4) != 0) return;
  if (sscanf_P(line + 4, PSTR("%d %u"), &state, &count) != 2 || state < 1 || state > 3) return;

  Z1_BridgeHeard = millis();
  if (state != Z1_BridgeState) {
    Z1_BridgeState = state;
    if (state == 1) Serial.println(F("WiFi bridge: looking for the WiFi network"));
    else if (state == 2) Serial.println(F("WiFi bridge: on the WiFi, can't reach DCC-EX"));
    else Serial.println(F("WiFi bridge: connected to DCC-EX"));
    Draw_Bridge();
  }
  if (Z1_BridgeCountKnown && count != Z1_BridgeCount) {
    Z1_BridgeFlashing = true;
    Z1_BridgeFlashUntil = millis() + 150;
    Draw_Bridge();
  }
  Z1_BridgeCount = count;
  Z1_BridgeCountKnown = true;

}

void Draw_Bridge() {

  if (Z1_BridgeState == 0) return;
  uint16_t colour;
  if (Z1_BridgeFlashing)          colour = BLACK;
  else if (Z1_BridgeState == 1)   colour = WHITE;
  else if (Z1_BridgeState == 2)   colour = YELLOW;
  else if (Z1_BridgeState == 3)   colour = GREEN;
  else if (Z1_BridgeState == 8)   colour = MAGENTA;
  else                            colour = GRAY;
  tft.setRotation(1);
  tft.fillCircle(14, Status_Line_Y() + 8, 7, BLACK);
  tft.fillCircle(14, Status_Line_Y() + 8, 5, colour);
  tft.setRotation(0);

}

// Called by the map module after it draws the map screen:
// red band right across the top of the screen (from the top edge to a few pixels
// below the text), then the usual status text on top of it
void showMapStatus() {

  tft.setRotation(1);
  tft.fillRect(0, 0, tft.width(), MAP_STATUS_Y + 16 + 4, Z_DCC_RED);
  tft.setRotation(0);
  Update_Screen_Track_Type();
  Draw_Track_Power();
  Draw_Bridge();

}

// Called by the map module when Accessory 93 is turned off
void showLocoScreen() {
  int k;

  drawZeroPanel(tft);
  Update_Screen_Locos();
  for (k = 1; k <= 16; k++) {
    Draw_Speed(k);
  }
  Update_Screen_Track_Type();
  Draw_Track_Power();
  Draw_Bridge();

}
