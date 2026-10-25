// ZeroPanel.h - draws the Zero 1 panel background in code (no bitmap)
// For Adafruit_GFX-based drivers (MCUFRIEND_kbv, Adafruit_ILI9341/HX8357, etc.)
// Layout is for portrait 320 x 480 (rotation 0), matching the original image.
//
// The ruled lines are not parallel: each one slopes and they fan out slightly
// down the screen, following the Zero 1 panel that sits under the LCD. All 18
// lines (top edge, 16 numbered rows, an unnumbered 17th row, bottom edge) come
// from one equation measured from the original Z_DCC.bmp:
//
//   y = A + B*x + k*(C + D*x) + E*k*k      k = 0 (top edge) .. 17 (bottom edge)

#pragma once
#include <Adafruit_GFX.h>
#include <Fonts/FreeSansBold12pt7b.h>

// 8-bit RGB -> RGB565
#ifndef RGB565
#define RGB565(r, g, b) (uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
#endif

// ---- Colours (tweak to taste) ----
const uint16_t ZP_FRAME = RGB565(140, 135, 130);  // grey above the top line / below the bottom line
const uint16_t ZP_WHITE = RGB565(245, 245, 245);  // label area
const uint16_t ZP_RED   = RGB565(232,  62,  40);  // number strip
const uint16_t ZP_LINE  = RGB565( 30,  30,  30);  // ruled lines
const uint16_t ZP_TEXT  = RGB565( 20,  20,  20);  // numbers

// ---- Layout (portrait pixels) ----
const int16_t ZP_W        = 320;
const int16_t ZP_H        = 480;
const int16_t ZP_RED_X    = 221;    // white | red boundary (vertical)
const uint8_t ZP_LINES    = 18;     // k = 0..17
const uint8_t ZP_ROWS     = 16;     // numbered rows
const int16_t ZP_NUM_CX   = 246;    // x centre of the numbers in the red strip
const int8_t  ZP_DIGIT_GAP = 1;     // pixels between the two digits of 10-16 (can be 0 or negative)
const float   ZP_NUM_STRETCH = 1.35;  // height stretch of the numbers (1.0 = normal font, ~23 px tall)

// line equation (measured from the original panel image)
const float ZP_A = 1.928;
const float ZP_B = 0.08641;
const float ZP_C = 24.990;
const float ZP_D = 0.004133;
const float ZP_E = 0.008028;

const uint8_t ZP_BASE_ROT = 0;      // your normal (portrait) rotation

// y of ruled line k (0 = top edge, 17 = bottom edge) at column x
inline float zpLineYf(uint8_t k, int16_t x) {
  return ZP_A + ZP_B * x + k * (ZP_C + ZP_D * x) + ZP_E * k * k;
}
inline int16_t zpLineY(uint8_t k, int16_t x) { return (int16_t)(zpLineYf(k, x) + 0.5f); }

// y of the centre of row 'row' (1 = top numbered row ... 17) at column x.
// Use it to place text: e.g. setCursor(x, zpRowCentreY(row, x + w/2) - 8) for size-2 text.
inline int16_t zpRowCentreY(uint8_t row, int16_t x) {
  return (int16_t)((zpLineYf(row - 1, x) + zpLineYf(row, x)) / 2 + 0.5f);
}

// draw (or redraw) ruled line k across the full width, 2 px thick
void zpDrawLine(Adafruit_GFX &tft, uint8_t k) {
  int16_t y0 = zpLineY(k, 0), y1 = zpLineY(k, ZP_W - 1);
  tft.drawLine(0, y0,     ZP_W - 1, y1,     ZP_LINE);
  tft.drawLine(0, y0 + 1, ZP_W - 1, y1 + 1, ZP_LINE);
}

// Frame, white panel, red strip and ruled lines only
void drawZeroPanelBase(Adafruit_GFX &tft) {
  tft.setRotation(ZP_BASE_ROT);

  // column by column: grey above the top line, panel, grey below the bottom line
  for (int16_t x = 0; x < ZP_W; x++) {
    int16_t top = zpLineY(0, x), bot = zpLineY(ZP_LINES - 1, x);
    if (bot > ZP_H - 1) bot = ZP_H - 1;
    if (top > 0)          tft.drawFastVLine(x, 0, top, ZP_FRAME);
    tft.drawFastVLine(x, top, bot - top + 1, x < ZP_RED_X ? ZP_WHITE : ZP_RED);
    if (bot < ZP_H - 1)   tft.drawFastVLine(x, bot + 1, ZP_H - 1 - bot, ZP_FRAME);
  }

  // 18 sloped ruled lines, 2 px thick
  for (uint8_t k = 0; k < ZP_LINES; k++) zpDrawLine(tft, k);
}

// glyph data for one character of the number font (read from PROGMEM)
struct ZpGlyph { uint16_t off; uint8_t w, h; int8_t xo, yo; };

static ZpGlyph zpGlyph(char c) {
  const GFXfont *f = &FreeSansBold12pt7b;
  GFXglyph *glyphs = (GFXglyph *)pgm_read_ptr(&f->glyph);
  GFXglyph *g = glyphs + ((uint8_t)c - (uint8_t)pgm_read_byte(&f->first));
  ZpGlyph z;
  z.off = pgm_read_word(&g->bitmapOffset);
  z.w   = pgm_read_byte(&g->width);
  z.h   = pgm_read_byte(&g->height);
  z.xo  = (int8_t)pgm_read_byte(&g->xOffset);
  z.yo  = (int8_t)pgm_read_byte(&g->yOffset);
  return z;
}

// Draw one sideways digit, stretched ZP_NUM_STRETCH times in height (its width
// along the row is unchanged). (xr, base) is the left edge of the ink / baseline in the
// sideways view; drawing is done straight onto the portrait screen.
static void zpDrawTallGlyph(Adafruit_GFX &tft, char c, int16_t xr, int16_t base) {
  const uint8_t *bitmap = (const uint8_t *)pgm_read_ptr(&FreeSansBold12pt7b.bitmap);
  ZpGlyph g = zpGlyph(c);
  uint16_t bit = 0;
  for (uint8_t j = 0; j < g.h; j++) {
    // this bitmap row covers sideways-y from y0 to y1 once stretched
    int16_t y0 = base + (int16_t)lround((g.yo + j)     * ZP_NUM_STRETCH);
    int16_t y1 = base + (int16_t)lround((g.yo + j + 1) * ZP_NUM_STRETCH) - 1;
    for (uint8_t i = 0; i < g.w; i++, bit++) {
      if (pgm_read_byte(&bitmap[g.off + (bit >> 3)]) & (0x80 >> (bit & 7))) {
        // sideways (x, y) -> portrait (ZP_W - 1 - y, x)
        tft.fillRect(ZP_W - 1 - y1, xr + i, y1 - y0 + 1, 1, ZP_TEXT);
      }
    }
  }
}

// Sideways numbers 1-16 in the red strip, centred in their sloped rows.
// Two-digit numbers are set tighter than the font's normal spacing
// (ZP_DIGIT_GAP px between the digits) and all digits are stretched upright.
void drawZeroPanelNumbers(Adafruit_GFX &tft) {
  tft.setRotation(ZP_BASE_ROT);

  const int16_t yc = (ZP_W - 1) - ZP_NUM_CX;      // centre across the strip, sideways-y

  for (uint8_t r = 1; r <= ZP_ROWS; r++) {
    char buf[3];
    itoa(r, buf, 10);
    uint8_t n = strlen(buf);

    int16_t total = 0;
    float top = 0, bottom = 0;
    for (uint8_t d = 0; d < n; d++) {
      ZpGlyph g = zpGlyph(buf[d]);
      total += g.w + (d ? ZP_DIGIT_GAP : 0);
      if (g.yo < top) top = g.yo;
      if (g.yo + g.h > bottom) bottom = g.yo + g.h;
    }
    top *= ZP_NUM_STRETCH;  bottom *= ZP_NUM_STRETCH;

    int16_t x    = zpRowCentreY(r, ZP_NUM_CX) + 1 - total / 2;     // left edge of the first digit
    int16_t base = yc - (int16_t)lround((top + bottom) / 2);       // baseline, centred across the strip
    for (uint8_t d = 0; d < n; d++) {
      ZpGlyph g = zpGlyph(buf[d]);
      zpDrawTallGlyph(tft, buf[d], x, base);
      x += g.w + ZP_DIGIT_GAP;
    }
  }
}

void drawZeroPanel(Adafruit_GFX &tft) {
  drawZeroPanelBase(tft);
  drawZeroPanelNumbers(tft);
}
