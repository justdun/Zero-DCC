// ZeroST7796.h - Adafruit_GFX driver for the 4.0" ST7796S SPI display module (480 x 320)
//
// Lets ZeroDCC run unchanged on the SPI screen (e.g. the Mega Pro test rig): everything the
// sketch, ZeroPanel.h, ZeroSplash.h and DccMap.h draw goes through Adafruit_GFX, so only this
// small driver is different. Uses hardware SPI (SDI/MOSI 51, SCK 52), shared with the SD card.
//
// Wiring as ZeroDCC v6.96 SPI:  CS A5   DC/RS A3   RESET A4   LED A0   SDI 51   SCK 52
//
// If the picture comes out upside down, set ST_ROT_OFFSET to 2.
// If the colours look like a photo negative, set ST_INVERT to true.
// If the screen shows noise, lower ST_SPI_HZ to 4000000.

#pragma once
#include <Adafruit_GFX.h>
#include <SPI.h>

const uint32_t ST_SPI_HZ     = 4000000;     // 8000000 is faster if your wiring copes; 2000000 for long jumper leads
const uint8_t  ST_ROT_OFFSET = 0;
const bool     ST_INVERT     = false;

class ZeroST7796 : public Adafruit_GFX {
  public:
    ZeroST7796(uint8_t cs, uint8_t dc, uint8_t rst, int8_t led)
      : Adafruit_GFX(320, 480), _cs(cs), _dc(dc), _rst(rst), _led(led), _spi(ST_SPI_HZ, MSBFIRST, SPI_MODE0) {}

    // same calls the sketch makes for MCUFRIEND_kbv
    uint16_t readID() { return 0x7796; }

    void begin(uint16_t id = 0) {
      (void)id;
      pinMode(_cs, OUTPUT); digitalWrite(_cs, HIGH);
      pinMode(_dc, OUTPUT); digitalWrite(_dc, HIGH);
      _csReg = portOutputRegister(digitalPinToPort(_cs)); _csBit = digitalPinToBitMask(_cs);
      _dcReg = portOutputRegister(digitalPinToPort(_dc)); _dcBit = digitalPinToBitMask(_dc);
      if (_led >= 0) { pinMode(_led, OUTPUT); digitalWrite(_led, HIGH); }
      SPI.begin();

      pinMode(_rst, OUTPUT);
      digitalWrite(_rst, HIGH); delay(10);
      digitalWrite(_rst, LOW);  delay(20);
      digitalWrite(_rst, HIGH); delay(150);

      select();
      command(0x01); deselect(); delay(150);     // software reset
      select();
      command(0x11); deselect(); delay(150);     // sleep out
      select();
      command(0xF0); data(0xC3);                 // command set enable
      command(0xF0); data(0x96);
      command(0x3A); data(0x55);                 // 16 bits per pixel
      command(ST_INVERT ? 0x21 : 0x20);
      command(0xF0); data(0x3C);                 // command set disable
      command(0xF0); data(0x69);
      command(0x29);                             // display on
      deselect();
      delay(20);
      setRotation(0);
    }

    void setRotation(uint8_t r) override {
      Adafruit_GFX::setRotation(r);              // sets rotation, _width, _height
      static const uint8_t madctl[4] = { 0x48, 0x28, 0x88, 0xE8 };
      select();
      command(0x36); data(madctl[(rotation + ST_ROT_OFFSET) & 3]);
      deselect();
    }

    void drawPixel(int16_t x, int16_t y, uint16_t c) override {
      if (x < 0 || y < 0 || x >= _width || y >= _height) return;
      select();
      window(x, y, x, y);
      SPI.transfer16(c);
      deselect();
    }

    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) override {
      if (w < 0) { x += w + 1; w = -w; }
      if (h < 0) { y += h + 1; h = -h; }
      if (x < 0) { w += x; x = 0; }
      if (y < 0) { h += y; y = 0; }
      if (x + w > _width)  w = _width - x;
      if (y + h > _height) h = _height - y;
      if (w <= 0 || h <= 0) return;
      select();
      window(x, y, x + w - 1, y + h - 1);
      uint32_t n = (uint32_t)w * h;
      uint8_t hi = c >> 8, lo = c;
      while (n--) { SPI.transfer(hi); SPI.transfer(lo); }
      deselect();
    }

    void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t c) override { fillRect(x, y, 1, h, c); }
    void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c) override { fillRect(x, y, w, 1, c); }
    void fillScreen(uint16_t c) override { fillRect(0, 0, _width, _height, c); }

    // Adafruit_GFX draws text and shapes between startWrite / endWrite: hold the bus for the lot
    void startWrite() override { select(); _hold++; }
    void endWrite() override { if (_hold) _hold--; deselect(); }
    void writePixel(int16_t x, int16_t y, uint16_t c) override { drawPixel(x, y, c); }
    void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) override { fillRect(x, y, w, h, c); }
    void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t c) override { fillRect(x, y, 1, h, c); }
    void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c) override { fillRect(x, y, w, 1, c); }

  private:
    uint8_t _cs, _dc, _rst;
    int8_t _led;
    SPISettings _spi;
    volatile uint8_t *_csReg, *_dcReg;
    uint8_t _csBit, _dcBit;
    uint8_t _hold = 0;                           // >0 while inside startWrite / endWrite
    uint8_t _depth = 0;                          // nested select() calls

    void select() {
      if (_depth++ == 0) { SPI.beginTransaction(_spi); *_csReg &= ~_csBit; }
    }
    void deselect() {
      if (_depth && --_depth == 0) { *_csReg |= _csBit; SPI.endTransaction(); }
    }
    void command(uint8_t c) { *_dcReg &= ~_dcBit; SPI.transfer(c); *_dcReg |= _dcBit; }
    void data(uint8_t d) { SPI.transfer(d); }
    void window(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
      command(0x2A); SPI.transfer16(x0); SPI.transfer16(x1);
      command(0x2B); SPI.transfer16(y0); SPI.transfer16(y1);
      command(0x2C);
    }
};
