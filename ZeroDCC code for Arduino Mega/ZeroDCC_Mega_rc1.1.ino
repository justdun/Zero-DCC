//  Zero DCC for Arduino Mega with Wifi
//
//  Release Candidate 1.1
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

#include <SPI.h>
#include <SD.h>

#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
#define BMPIMAGEOFFSET 54
#define BUFFPIXEL      20
#define PALETTEDEPTH   8

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

volatile uint16_t datapoint[1001];
volatile int savepoint = 1;
int readpoint = 1;
int lastsavepoint = 1;
volatile int chaseflag = 0;

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

void setup()

{

  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  Serial.println(F("Welcome to Zero DCC!"));
  Serial.println();
  Serial.println(F("You should only see DCC++EX commands here (or errors if things go wrong)."));
  Serial.println();

  pinMode (digitalPinToInterrupt(Z1_Pin), INPUT);

  pinMode(Z1_Relay_Pin, OUTPUT);

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GRAY    0x8410
#define Z_DCC   0xBDD7
#define Z_DCC_RED 0xA800

  uint16_t ID;
  ID = tft.readID();
  if (ID == 0x0D3D3) ID = 0x7796;
  tft.begin(ID);

  tft.fillScreen(GRAY);
  tft.setTextColor(BLACK, Z_DCC);
  tft.setTextSize(2);

  pinMode(Z1_SD_Card_Pin, OUTPUT);

 if (!SD.begin(Z1_SD_Card_Pin))

  {
    Serial.println(F("SD Card failed."));
    tft.fillScreen(GRAY);
  }

  else {

    uint8_t ret;

    ret = showBMP("Z_DCC_SP.bmp", 0, 0);
    switch (ret) {
      case 0:
        break;
      case 1:
        Serial.println(F("BMP error bad position"));
        break;
      case 2:
        Serial.println(F("BMP error bad BMP ID"));
        break;
      case 3:
        Serial.println(F("BMP error wrong number of planes"));
        break;
      case 4:
        Serial.println(F("BMP error unsupported BMP format"));
        break;
      case 5:
        Serial.println(F("BMP error unsupported palette"));
        break;
      default:
        Serial.println(F("BMP error unknown"));
        break;
    }

    String Filename = String (Z1_Settings_File + "1.txt");

    myfile = SD.open(Filename , FILE_WRITE);
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

    ret = showBMP("Z_DCC.bmp", 0, 0);
    switch (ret) {
      case 0:
        break;
      case 1:
        Serial.println(F("BMP error bad position"));
        break;
      case 2:
        Serial.println(F("BMP error bad BMP ID"));
        break;
      case 3:
        Serial.println(F("BMP error wrong number of planes"));
        break;
      case 4:
        Serial.println(F("BMP error unsupported BMP format"));
        break;
      case 5:
        Serial.println(F("BMP error unsupported palette"));
        break;
      default:
        Serial.println(F("BMP error unknown"));
        break;
    }

  }

  Update_Screen_Locos();

  AccessoryNo = 2;
  Update_Screen_Track_Type();
  Track_Power_Off();

  attachInterrupt (digitalPinToInterrupt(Z1_Pin), Z1_CALC_PULSE, CHANGE);

}

void loop()
{

  interrupts();
  Data_Cycle();

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

    else if (AccessoryNo == 95 && Z1_Accessory95Flag == 3) {
      Playback_Stop();
    }

    else if (AccessoryNo == 95 && Z1_Accessory95Flag == 2) {
      Accessory95();
    }

    else if (Z1_Accessory95Flag == 1 ) {
      Accessory95();
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
        digitalWrite(Z1_Relay_Pin, LOW);
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
        digitalWrite(Z1_Relay_Pin, LOW);
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
        digitalWrite(Z1_Relay_Pin, LOW);
      }
      else {
        Z1_Accessory97Flag = 1;
        LocoNumber = 0;
        digitalWrite(Z1_Relay_Pin, HIGH);
      }
    }

    else if (AccessoryNo == 95 && Z1_Accessory95Flag == 0) {

      Z1_Accessory95Flag = 1;

      digitalWrite(Z1_Relay_Pin, HIGH);

    }

    else if (AccessoryNo == 96) {
      if (Z1_Data[24] == 1) {
        Z1_Accessory96Flag = 0;
        LocoNumber = 0;
        digitalWrite(Z1_Relay_Pin, LOW);
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
        digitalWrite(Z1_Relay_Pin, LOW);
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
        digitalWrite(Z1_Relay_Pin, LOW);
      }
      else {
        Z1_Accessory90Flag = 1;
        LocoNumber = 0;
        digitalWrite(Z1_Relay_Pin, HIGH);
      }
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
      Serial.print("<t ");
      Serial.print(j + (Z1_Tag * 4));
      Serial.print(" ");
      Serial.print(Z1_LocoNo[j + (Z1_Tag * 4)]);
      Serial.print(" ");
      Serial.print(Z1_LocoSpeed[j + (Z1_Tag * 4)]);
      Serial.print(" ");
      Serial.print(Z1_LocoDirection[j + (Z1_Tag * 4)]);
      Serial.println(">");

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

  if (Z1_Data[24] == 1) {
    Serial.print("<T ");
    Serial.print(AccessoryNo);
    Serial.println(" 1>");

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
    Serial.print("<T ");
    Serial.print(AccessoryNo);
    Serial.println(" 0>");

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

}

void Accessory90() {

  if (AccessoryNo == 1) {  Track_Power_On(); }

  else if (AccessoryNo == 2) {
     Track_Power_Off();
     Serial.println("<= A MAIN >");
     Serial.println("<= B PROG >");
     Update_Screen_Track_Type();
     }
  else if (AccessoryNo == 3) {
     Track_Power_Off();
     Serial.println("<= A MAIN >");
     Serial.println("<= B DC 999 >");
     Update_Screen_Track_Type();
     }
  else if (AccessoryNo == 4) {
     Track_Power_Off();
     Serial.println("<= A DC 999 >");
     Serial.println("<= B DC 998 >");
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

        Serial.print("<W ");
        Serial.print(LocoNumber);
        Serial.println(">");
        delay(500);
      }
      Z1_Accessory91Flag = 0;
      LocoNumber = 0;
      digitalWrite(Z1_Relay_Pin, LOW);
    }

    else {

      Z1_Accessory91Flag = 0;
      LocoNumber = 0;
      digitalWrite(Z1_Relay_Pin, LOW);
    }

}

void Accessory95() {

  digitalWrite(Z1_Relay_Pin, LOW);

  if (Z1_Data[24] == 0) {

    if (Z1_Accessory95Flag == 1) {
      Z1_Accessory95Flag = 2;
      Z1_Accessory95File = String (Z1_Recording_File + String(AccessoryNo, DEC) + ".txt");

      Z1_LastCommandTime = millis();

      if (SD.exists(Z1_Accessory95File)) {
        SD.remove(Z1_Accessory95File);
      }

      myfile = SD.open(Z1_Accessory95File , FILE_WRITE);

      Serial.print("Writing to file:  opening ");
      if (myfile) {
        myfile.close();
      }
      else {

        Serial.println("error opening file");
      }

    }

    else {

      Z1_Accessory95Flag = 0;

    }

  }

  else {
    Z1_Accessory95File = String (Z1_Recording_File + String(AccessoryNo, DEC) + ".txt");

    if (SD.exists(Z1_Accessory95File)) {
      Z1_PlaybackFile = SD.open(Z1_Accessory95File, FILE_READ);
      if (Z1_PlaybackFile) {
        filesize = Z1_PlaybackFile.size();
        fileposition = 0;
        Z1_PlaybackCommandLoaded = false;
        Z1_Accessory95Flag = 3;
      }
    }
  }

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
    Serial.println(nextcommand);
    Z1_PlaybackCommandLoaded = false;
  }
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
    Serial.print("<t ");
    Serial.print(AccessoryNo);
    Serial.print(" ");
    Serial.print(Z1_LocoNo[AccessoryNo]);
    Serial.println(" 0 0>");

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
      Serial.print("<t ");
      Serial.print(k);
      Serial.print(" ");
      Serial.print(Z1_LocoNo[k]);
      Serial.println(" 0 0>");
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
  digitalWrite(Z1_Relay_Pin, LOW);

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

    Serial.print("<F ");
    Serial.print(Z1_LocoNo[LocoNumber]);
    Serial.print(" 0 ");
    Serial.print(Z1_LocoFN[LocoNumber][0]);
    Serial.println(">");

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

    Serial.print("<F ");
    Serial.print(Z1_LocoNo[LocoNumber]);
    Serial.print(" 2 ");
    Serial.print(Z1_LocoFN[LocoNumber][2]);
    Serial.println(">");

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

    Serial.print("<F ");
    Serial.print(Z1_LocoNo[LocoNumber]);
    Serial.print(" ");
    Serial.print(Z1_Accessory99Function);
    Serial.print(" ");
    Serial.print(Z1_LocoFN[LocoNumber][Z1_Accessory99Function]);
    Serial.println(">");

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

void Update_Screen_Track_Type() {

  tft.setRotation(1);
  tft.setTextColor(BLACK, Z_DCC_RED);
  tft.setTextSize(2);
  tft.setCursor(40, 0);

  if (AccessoryNo == 2)       { tft.print("A:MAIN    B:PROG    "); }
  else if (AccessoryNo == 3)  { tft.print("A:MAIN    B:DC (999)"); }
  else if (AccessoryNo == 4)  { tft.print("A:DC(999) B:DCX(998)"); }
  tft.setRotation(0);

}

void Track_Power_On() {

  tft.setRotation(1);
  tft.setTextSize(2);
  tft.setCursor(290, 0);
  tft.setTextColor(BLACK, Z_DCC_RED);
  tft.print("TRACK POWER ON ");
  tft.setRotation(0);
  Serial.println("<1>");

}

void Track_Power_Off() {

  tft.setRotation(1);
  tft.setTextSize(2);
  tft.setCursor(290, 0);
  tft.setTextColor(Z_DCC_RED, BLACK);
  tft.print("TRACK POWER OFF");
  tft.setRotation(0);
  Serial.println("<0>");

}

void Update_Screen_Locos() {
  int k;

  tft.setTextColor(BLACK, Z_DCC);
  tft.setTextSize(2);

  for (k = 1; k <= 16; k++)   {
    tft.setCursor(10, ((k - 1) * 25.2) + 11);
    tft.print("    ");
    tft.setCursor(10, ((k - 1) * 25.2) + 11);
    tft.print(String(Z1_LocoNo[k], DEC));
  }

}

void Update_Screen_Speeds(int k) {
  int l;
  tft.setTextColor(BLUE, Z_DCC);
  tft.setTextSize(2);

  for (l = 1; l <= 15; l++) {
    tft.setCursor(((l * 10) + 58) , (((k - 1) * 25.4) + ((l * (0.9 + ((k - 1) * 0.04)))) + 14));

    tft.print(" ");
  }

  for (l = 1; l <= ((Z1_LocoSpeed[k] / 9) + 1); l++) {
    tft.setCursor(((l * 10) + 58), (((k - 1) * 25.4) + ((l * (0.9 + ((k - 1) * 0.04)))) + 14));

    if (Z1_LocoDirection[k] == 1) {
      tft.print(">");
    }
    else {
      tft.print("<");
    }
  }

}

uint16_t read16(File& f) {
  uint16_t result;
  f.read((uint8_t*)&result, sizeof(result));
  return result;
}

uint32_t read32(File& f) {
  uint32_t result;
  f.read((uint8_t*)&result, sizeof(result));
  return result;
}

uint8_t showBMP(char *nm, int x, int y) {
  File bmpFile;
  int bmpWidth, bmpHeight;
  uint8_t bmpDepth;
  uint32_t bmpImageoffset;
  uint32_t rowSize;
  uint8_t sdbuffer[3 * BUFFPIXEL];
  uint16_t lcdbuffer[(1 << PALETTEDEPTH) + BUFFPIXEL], *palette = NULL;
  uint8_t bitmask, bitshift;
  boolean flip = true;
  int w, h, row, col, lcdbufsiz = (1 << PALETTEDEPTH) + BUFFPIXEL, buffidx;
  uint32_t pos;
  boolean is565 = false;

  uint16_t bmpID;
  uint16_t n;
  uint8_t ret;

  if ((x >= tft.width()) || (y >= tft.height()))
    return 1;

  bmpFile = SD.open(nm);
  bmpID = read16(bmpFile);
  (void) read32(bmpFile);
  (void) read32(bmpFile);
  bmpImageoffset = read32(bmpFile);
  (void) read32(bmpFile);
  bmpWidth = read32(bmpFile);
  bmpHeight = read32(bmpFile);
  n = read16(bmpFile);
  bmpDepth = read16(bmpFile);
  pos = read32(bmpFile);
  if (bmpID != 0x4D42) ret = 2;
  else if (n != 1) ret = 3;
  else if (pos != 0 && pos != 3) ret = 4;
  else if (bmpDepth < 16 && bmpDepth > PALETTEDEPTH) ret = 5;
  else {
    bool first = true;
    is565 = (pos == 3);

    rowSize = (bmpWidth * bmpDepth / 8 + 3) & ~3;
    if (bmpHeight < 0) {
      bmpHeight = -bmpHeight;
      flip = false;
    }

    w = bmpWidth;
    h = bmpHeight;
    if ((x + w) >= tft.width())
      w = tft.width() - x;
    if ((y + h) >= tft.height())
      h = tft.height() - y;

    if (bmpDepth <= PALETTEDEPTH) {

      bmpFile.seek(bmpImageoffset - (4 << bmpDepth));
      bitmask = 0xFF;
      if (bmpDepth < 8)
        bitmask >>= bmpDepth;
      bitshift = 8 - bmpDepth;
      n = 1 << bmpDepth;
      lcdbufsiz -= n;
      palette = lcdbuffer + lcdbufsiz;
      for (col = 0; col < n; col++) {
        pos = read32(bmpFile);
        palette[col] = ((pos & 0x0000F8) >> 3) | ((pos & 0x00FC00) >> 5) | ((pos & 0xF80000) >> 8);
      }
    }

    tft.setAddrWindow(x, y, x + w - 1, y + h - 1);
    for (row = 0; row < h; row++) {

      uint8_t r, g, b, *sdptr;
      int lcdidx, lcdleft;
      if (flip)
        pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
      else
        pos = bmpImageoffset + row * rowSize;
      if (bmpFile.position() != pos) {
        bmpFile.seek(pos);
        buffidx = sizeof(sdbuffer);
      }

      for (col = 0; col < w; ) {
        lcdleft = w - col;
        if (lcdleft > lcdbufsiz) lcdleft = lcdbufsiz;
        for (lcdidx = 0; lcdidx < lcdleft; lcdidx++) {
          uint16_t color;

          if (buffidx >= sizeof(sdbuffer)) {
            bmpFile.read(sdbuffer, sizeof(sdbuffer));
            buffidx = 0;
            r = 0;
          }
          switch (bmpDepth) {
            case 24:
              b = sdbuffer[buffidx++];
              g = sdbuffer[buffidx++];
              r = sdbuffer[buffidx++];
              color = tft.color565(r, g, b);
              break;
            case 16:
              b = sdbuffer[buffidx++];
              r = sdbuffer[buffidx++];
              if (is565)
                color = (r << 8) | (b);
              else
                color = (r << 9) | ((b & 0xE0) << 1) | (b & 0x1F);
              break;
            case 1:
            case 4:
            case 8:
              if (r == 0)
                b = sdbuffer[buffidx++], r = 8;
              color = palette[(b >> bitshift) & bitmask];
              r -= bmpDepth;
              b <<= bmpDepth;
              break;
          }
          lcdbuffer[lcdidx] = color;

        }
        tft.pushColors(lcdbuffer, lcdidx, first);
        first = false;
        col += lcdidx;
      }
    }
    tft.setAddrWindow(0, 0, tft.width() - 1, tft.height() - 1);
    ret = 0;
  }
  bmpFile.close();
  return (ret);
}
