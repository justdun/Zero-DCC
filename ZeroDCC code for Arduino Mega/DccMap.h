// DccMap.h - track schematic screen for ZeroDCC
//
// Loads a 15 x 9 track map from the SD card file Z_DCCMAP.TXT at start-up and draws it in
// landscape (480 x 320, 32 px squares). Accessory 93 ON shows the map (only if
// the map file loaded without errors - otherwise Acc 93 is ignored).
// Accessory 93 OFF returns to the loco screen. Points and signals can use any
// accessory number from 1 to 2040: 1-89 are set straight from the Zero 1,
// higher numbers through Accessory 92. Each change updates the green/red dots
// on every point or signal that uses that accessory number.
//
// ---------------------------------------------------------------------------
// Z_DCCMAP.TXT MAP FILE FORMAT (plain text, edit on a PC)
//   - One line per row, top row first, up to 9 rows
//     (the top strip of the screen shows the track mode / power status line).
//   - Up to 15 codes per row, separated by spaces or commas.
//     Missing codes at the end of a row are blank.
//   - BLANK on its own line leaves that whole row empty.
//   - Lines starting with ; or # are comments. Upper/lower case doesn't matter.
//   - Lines can be up to 159 characters long.
//
//   Edges of a square:  L = mid-left   R = mid-right
//                       T = mid-top    B = mid-bottom
//
//   .            blank square                  (0 also works)
//   H            horizontal line   L-R         (- also works)
//   V            vertical line     T-B         (| also works)
//   BR LB TL TR  diagonal line between those two edges
//                (BR = middle-bottom to mid-right, LB = mid-left to middle-bottom,
//                 TL = middle-top to mid-left,     TR = mid-right to middle-top)
//
//   Points: base line + diagonal + accessory number (1-2040) + optional A
//   HLT12   horizontal with diagonal mid-left to mid-top,     accessory 12
//   HTR12   horizontal with diagonal mid-top to mid-right
//   HLB12   horizontal with diagonal mid-left to mid-bottom
//   HBR12   horizontal with diagonal mid-bottom to mid-right
//   VBR12   vertical   with diagonal middle-bottom to mid-right
//   VLB12   vertical   with diagonal mid-left to middle-bottom
//   VTL12   vertical   with diagonal middle-top to mid-left
//   VTR12   vertical   with diagonal mid-right to middle-top
//   Add A for the alternative orientation, e.g. HLT12A, VBR1050A
//
//   Signals: any track code + S + accessory number (1-2040) + optional A
//   HS12  VS12  BRS12  LBS12  TLS12  TRS12   (e.g. TRS7A = TR with signal 7, alternative)
//   (the two diagonal letters can be in either order: HLT12 = HTL12)
//
//   Accessory numbers too wide for the space beside the track are drawn in a
//   smaller font, on two lines if needed.
//
// DOTS ON A POINT
//   Default:      accessory OFF -> straight route green, diverging red
//                 accessory ON  -> diverging route green, straight red
//   Alternative:  the other way round.
// DOT ON A SIGNAL (one dot in the middle of the track)
//   Default:      accessory OFF -> green, ON -> red
//   Alternative:  the other way round.
//   Accessories are assumed OFF until they are first set after power-up.
// ---------------------------------------------------------------------------
//
// IN YOUR SKETCH
//   1. #include "DccMap.h"
//   2. Write  void showLocoScreen()  that redraws your normal loco screen
//      (background plus current speeds/arrows). The map calls it on Acc 93 OFF.
//      Also write  void showMapStatus()  that draws the track mode / power line
//      at y = MAP_STATUS_Y in landscape. The map calls it after drawing.
//   3. In setup(), after SD.begin() has succeeded:          dccMapLoad();
//   4. Where you handle a Zero 1 accessory command (1-89 and 93), add:
//          if (dccMapAccessory(tft, acc, on)) return;   // true = Acc 93, nothing else to do
//      For any other point / signal change (Accessory 92, playback), call
//          dccMapSetPoint(tft, acc, on);                // never treats 93 as the map screen
//   5. Skip loco screen updates while the map is showing:
//          if (!dccMapActive()) { ...draw speed / direction... }

#pragma once
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <SD.h>

// ---------------- Settings ----------------
#define DCC_MAP_FILE "Z_DCCMAP.TXT"       // SD library needs an 8.3 name (max 8 chars + .TXT)
const uint8_t  MAP_SCREEN_ACC  = 93;      // accessory that switches to the map
const uint16_t MAP_FIRST_ACC   = 1;
const uint16_t MAP_LAST_ACC    = 2040;    // highest DCC accessory (linear) address; 2041-2044 are broadcast

const uint8_t MAP_COLS = 15;
const uint8_t MAP_ROWS = 9;
const uint8_t MAP_TOP  = 32;              // map starts below the 32 px status strip
const uint8_t MAP_STATUS_Y = 8;           // status text row, centred in the strip
const uint8_t MAP_CELL = 32;              // 15 x 32 = 480, 10 x 32 = 320
const uint8_t MAP_ROT      = 1;           // landscape
const uint8_t MAP_BASE_ROT = 0;           // rotation your other screens use

#ifndef RGB565
#define RGB565(r, g, b) (uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
#endif
const uint16_t MAP_BG    = RGB565(152, 152, 152);  // same grey as the splash
const uint16_t MAP_TRACK = RGB565(  0,   0,   0);
const uint16_t MAP_LABEL = RGB565( 40,  40,  40);
const uint16_t MAP_ON    = RGB565(  0, 200,   0);  // green dot
const uint16_t MAP_OFF   = RGB565(220,   0,   0);  // red dot
const uint16_t MAP_RING  = RGB565(  0,   0,   0);  // dot outline
const uint16_t MAP_ERR   = RGB565(160,   0,   0);
const bool     MAP_SHOW_NUMBERS = true;            // accessory number beside each point

// ---------------- Data ----------------
// shape byte: bits 0-1 base line (0 none, 1 H, 2 V)
//             bits 2-5 diagonal edges (L,R,T,B)
//             bit 6    alternative orientation
//             bit 7    signal (single dot) rather than a point
enum : uint8_t { E_L = 1, E_R = 2, E_T = 4, E_B = 8 };
enum : uint8_t { BASE_NONE = 0, BASE_H = 1, BASE_V = 2 };
const uint8_t SHAPE_ALT = 0x40;
const uint8_t SHAPE_SIG = 0x80;

struct MapCell { uint8_t shape; uint16_t acc; };  // acc 0 = not a point / signal

const uint8_t MAP_CELLS = MAP_ROWS * MAP_COLS;

MapCell  dccMap[MAP_ROWS][MAP_COLS];
uint8_t  dccCellOn[(MAP_CELLS + 7) / 8];          // last known accessory state of each map square
bool     dccMapLoaded = false;
bool     dccMapShown  = false;
char     dccMapError[48] = "";

void showLocoScreen();                            // provided by your sketch
void showMapStatus();                             // provided by your sketch: status line on the map

inline bool dccMapActive() { return dccMapShown; }

// ---------------- Accessory state (per map square) ----------------
inline bool mapCellOn(uint8_t r, uint8_t c) {
  uint8_t i = r * MAP_COLS + c;
  return (dccCellOn[i >> 3] >> (i & 7)) & 1;
}
inline void mapSetCellOn(uint8_t r, uint8_t c, bool on) {
  uint8_t i = r * MAP_COLS + c;
  if (on) dccCellOn[i >> 3] |=  (1 << (i & 7));
  else    dccCellOn[i >> 3] &= ~(1 << (i & 7));
}

// ---------------- Parsing ----------------
static uint8_t mapEdgeBit(char c) {
  switch (c) {
    case 'L': return E_L;
    case 'R': return E_R;
    case 'T': return E_T;
    case 'B': return E_B;
  }
  return 0;
}

// a diagonal joins one side edge (L/R) to one top/bottom edge (T/B)
static bool mapValidDiagonal(uint8_t m) {
  uint8_t side = m & (E_L | E_R), vert = m & (E_T | E_B);
  return (side == E_L || side == E_R) && (vert == E_T || vert == E_B);
}

// accessory number 1-2040 with optional trailing A; p points at the first digit
static bool mapParseAcc(const char *p, uint16_t &acc, bool &alt) {
  uint32_t num = 0; uint8_t digits = 0;
  while (*p >= '0' && *p <= '9' && digits < 6) { num = num * 10 + (*p - '0'); p++; digits++; }
  if (digits == 0 || num < MAP_FIRST_ACC || num > MAP_LAST_ACC) return false;
  alt = false;
  if (*p == 'A') { alt = true; p++; }
  if (*p != '\0') return false;
  acc = (uint16_t)num;
  return true;
}

static bool mapParseToken(const char *t, MapCell &c) {
  c.shape = 0; c.acc = 0;
  size_t n = strlen(t);

  // signal: track code + S + accessory, e.g. HS12, TRS7A
  const char *sp = strchr(t, 'S');
  if (sp && sp > t && sp - t <= 2) {
    char track[3] = {0, 0, 0};
    memcpy(track, t, sp - t);
    MapCell tc;
    bool alt;
    if (!mapParseToken(track, tc) || !tc.shape || tc.acc) return false;
    if (!mapParseAcc(sp + 1, c.acc, alt)) return false;
    c.shape = tc.shape | SHAPE_SIG | (alt ? SHAPE_ALT : 0);
    return true;
  }

  if (!strcmp(t, ".") || !strcmp(t, "0")) return true;
  if (!strcmp(t, "H") || !strcmp(t, "-")) { c.shape = BASE_H; return true; }
  if (!strcmp(t, "V") || !strcmp(t, "|")) { c.shape = BASE_V; return true; }

  if (n == 2) {                                   // plain diagonal
    uint8_t m = mapEdgeBit(t[0]) | mapEdgeBit(t[1]);
    if (!mapValidDiagonal(m)) return false;
    c.shape = m << 2;
    return true;
  }

  if (n >= 4 && (t[0] == 'H' || t[0] == 'V')) {  // point
    uint8_t m = mapEdgeBit(t[1]) | mapEdgeBit(t[2]);
    if (!mapValidDiagonal(m)) return false;
    bool alt;
    if (!mapParseAcc(t + 3, c.acc, alt)) return false;
    c.shape = (t[0] == 'H' ? BASE_H : BASE_V) | (m << 2) | (alt ? SHAPE_ALT : 0);
    return true;
  }
  return false;
}

// parse one text line into map row 'row'; returns false on a bad code
static bool mapParseRow(char *line, uint8_t row, uint16_t lineNo) {
  for (char *s = line; *s; s++) *s = toupper(*s);
  for (uint8_t col = 0; col < MAP_COLS; col++) dccMap[row][col] = {0, 0};

  // first token decides BLANK
  char *tok = strtok(line, " \t,");
  if (tok && !strcmp(tok, "BLANK")) return true;

  uint8_t col = 0;
  while (tok) {
    if (col >= MAP_COLS) {
      Serial.print(F("Map file line ")); Serial.print(lineNo);
      Serial.println(F(": more than 15 codes, extra ignored"));
      break;
    }
    MapCell c;
    if (!mapParseToken(tok, c)) {
      snprintf(dccMapError, sizeof(dccMapError), "Map file line %u: bad code '%s'", lineNo, tok);
      Serial.println(dccMapError);
      return false;
    }
    dccMap[row][col++] = c;
    tok = strtok(NULL, " \t,");
  }
  return true;
}

// Load Z_DCCMAP.TXT from the SD card (call once in setup, after SD.begin succeeds).
// Returns the number of rows read, or -1 if the file is missing or has an error.
// If it fails, Accessory 93 is ignored and the loco screen stays up.
int8_t dccMapLoad() {
  memset(dccMap, 0, sizeof(dccMap));
  memset(dccCellOn, 0, sizeof(dccCellOn));
  dccMapLoaded = false;
  dccMapError[0] = '\0';

  File f = SD.open(DCC_MAP_FILE);
  if (!f) {
    strcpy(dccMapError, "Z_DCCMAP.TXT not found");
    Serial.println(dccMapError);
    return -1;
  }

  char line[160];
  uint8_t len = 0, row = 0;
  uint16_t lineNo = 0;
  bool ok = true, more = true;

  while (more && ok) {
    int ch = f.read();
    more = (ch >= 0);
    if (more && ch != '\n') {
      if (ch != '\r' && len < sizeof(line) - 1) line[len++] = (char)ch;
      continue;
    }
    line[len] = '\0';
    lineNo++;
    // skip leading spaces, comments and empty lines
    char *s = line;
    while (*s == ' ' || *s == '\t') s++;
    if (*s && *s != ';' && *s != '#') {
      if (row < MAP_ROWS) ok = mapParseRow(s, row++, lineNo);
      else { Serial.print(F("Map file: more than 9 rows, line ")); Serial.print(lineNo); Serial.println(F(" ignored")); }
    }
    len = 0;
  }
  f.close();

  if (!ok) { memset(dccMap, 0, sizeof(dccMap)); return -1; }
  dccMapLoaded = true;
  Serial.print(F("Z_DCCMAP.TXT loaded: ")); Serial.print(row); Serial.println(F(" rows"));
  return row;
}

// ---------------- Drawing ----------------
static void mapEdgePoint(int16_t x, int16_t y, uint8_t e, int16_t &px, int16_t &py) {
  const int16_t M = MAP_CELL / 2;
  switch (e) {
    case E_L: px = x;                py = y + M;            break;
    case E_R: px = x + MAP_CELL - 1; py = y + M;            break;
    case E_T: px = x + M;            py = y;                break;
    default:  px = x + M;            py = y + MAP_CELL - 1; break;   // E_B
  }
}

static void mapDiagonal(Adafruit_GFX &tft, int16_t x, int16_t y, uint8_t mask) {
  uint8_t a = mask & (E_L | E_R), b = mask & (E_T | E_B);
  int16_t x0, y0, x1, y1;
  mapEdgePoint(x, y, a, x0, y0);
  mapEdgePoint(x, y, b, x1, y1);
  for (int8_t d = -1; d <= 1; d++) tft.drawLine(x0 + d, y0, x1 + d, y1, MAP_TRACK);  // 3 px wide
}

// dot positions on a point: straight route and diverging route
static void mapDotPositions(int16_t x, int16_t y, uint8_t shape,
                            int16_t &sx, int16_t &sy, int16_t &dx, int16_t &dy) {
  const int16_t M = MAP_CELL / 2;
  uint8_t base = shape & 3, mask = (shape >> 2) & 0x0F;
  // the diagonal shares one edge with the base line (the toe); it leaves by the other
  uint8_t shared = (base == BASE_H) ? (mask & (E_L | E_R)) : (mask & (E_T | E_B));
  uint8_t divEdge = mask & ~shared;
  uint8_t straightEdge = (shared == E_L) ? E_R : (shared == E_R) ? E_L : (shared == E_T) ? E_B : E_T;

  int16_t ex, ey; mapEdgePoint(x, y, straightEdge, ex, ey);
  sx = x + M + (ex - (x + M)) / 2;                  // halfway from centre to the straight exit
  sy = y + M + (ey - (y + M)) / 2;
  int16_t ax, ay, bx, by;
  mapEdgePoint(x, y, shared, ax, ay);
  mapEdgePoint(x, y, divEdge, bx, by);
  dx = ax + (bx - ax) * 3 / 4;                      // three quarters along the diagonal
  dy = ay + (by - ay) * 3 / 4;
}

// single signal dot in the middle of the track
static void mapDrawSignal(Adafruit_GFX &tft, uint8_t r, uint8_t c) {
  const MapCell &cell = dccMap[r][c];
  const int16_t M = MAP_CELL / 2;
  int16_t x = c * MAP_CELL, y = MAP_TOP + r * MAP_CELL, px = x + M, py = y + M;
  uint8_t mask = (cell.shape >> 2) & 0x0F;
  if (mask) {                                       // diagonal: halfway along it
    int16_t ax, ay, bx, by;
    mapEdgePoint(x, y, mask & (E_L | E_R), ax, ay);
    mapEdgePoint(x, y, mask & (E_T | E_B), bx, by);
    px = (ax + bx) / 2; py = (ay + by) / 2;
  }
  bool red = mapCellOn(r, c);
  if (cell.shape & SHAPE_ALT) red = !red;
  tft.fillCircle(px, py, 4, MAP_RING);
  tft.fillCircle(px, py, 3, red ? MAP_OFF : MAP_ON);
}

static void mapDrawDots(Adafruit_GFX &tft, uint8_t r, uint8_t c) {
  const MapCell &cell = dccMap[r][c];
  if (cell.shape & SHAPE_SIG) { mapDrawSignal(tft, r, c); return; }
  int16_t x = c * MAP_CELL, y = MAP_TOP + r * MAP_CELL, sx, sy, dx, dy;
  mapDotPositions(x, y, cell.shape, sx, sy, dx, dy);
  bool diverging = mapCellOn(r, c);
  if (cell.shape & SHAPE_ALT) diverging = !diverging;
  tft.fillCircle(sx, sy, 4, MAP_RING);
  tft.fillCircle(dx, dy, 4, MAP_RING);
  tft.fillCircle(sx, sy, 3, diverging ? MAP_OFF : MAP_ON);
  tft.fillCircle(dx, dy, 3, diverging ? MAP_ON  : MAP_OFF);
}

// small 3 x 5 digits for numbers too wide for the normal 5 x 7 font
// (15 bits per digit, top row first, 3 bits per row)
const uint16_t MAP_TINY_DIGITS[10] PROGMEM = {
  0b111101101101111,  // 0
  0b010110010010111,  // 1
  0b111001111100111,  // 2
  0b111001111001111,  // 3
  0b101101111001001,  // 4
  0b111100111001111,  // 5
  0b111100111101111,  // 6
  0b111001001001001,  // 7
  0b111101111101111,  // 8
  0b111101111001111   // 9
};

static void mapTinyText(Adafruit_GFX &tft, int16_t x, int16_t y, const char *s, uint8_t n) {
  for (uint8_t i = 0; i < n; i++, x += 4) {
    uint16_t g = pgm_read_word(&MAP_TINY_DIGITS[s[i] - '0']);
    for (uint8_t b = 0; b < 15; b++)
      if (g & (0x4000 >> b)) tft.drawPixel(x + b % 3, y + b / 3, MAP_LABEL);
  }
}

// accessory number centred in the box (bx, by, bw, bh): normal font if it fits,
// otherwise the small font on one line, otherwise the small font on two lines
static void mapDrawNumber(Adafruit_GFX &tft, int16_t bx, int16_t by, int16_t bw, int16_t bh, uint16_t acc) {
  char s[6];
  utoa(acc, s, 10);
  uint8_t n = strlen(s);

  int16_t w = 6 * n - 1;                            // 5 x 7 font, size 1
  if (w <= bw) {
    tft.setTextSize(1);
    tft.setTextColor(MAP_LABEL);
    tft.setCursor(bx + (bw - w) / 2, by + (bh - 7) / 2);
    tft.print(s);
    return;
  }
  w = 4 * n - 1;
  if (w <= bw) {
    mapTinyText(tft, bx + (bw - w) / 2, by + (bh - 5) / 2, s, n);
    return;
  }
  uint8_t n1 = (n + 1) / 2, n2 = n - n1;            // e.g. 2040 -> 20 / 40
  int16_t w1 = 4 * n1 - 1, w2 = 4 * n2 - 1, top = by + (bh - 11) / 2;
  mapTinyText(tft, bx + (bw - w1) / 2, top,     s,      n1);
  mapTinyText(tft, bx + (bw - w2) / 2, top + 6, s + n1, n2);
}

// the label goes in the clear part of the square, away from the track and dots
static void mapDrawLabel(Adafruit_GFX &tft, int16_t x, int16_t y, const MapCell &cell) {
  uint8_t base = cell.shape & 3, mask = (cell.shape >> 2) & 0x0F;
  if ((cell.shape & SHAPE_SIG) && !base) {          // signal on a diagonal: opposite corner
    mapDrawNumber(tft, (mask & E_R) ? x + 1 : x + 18, (mask & E_B) ? y + 1 : y + 18, 13, 13, cell.acc);
  } else if ((cell.shape & SHAPE_SIG) && base == BASE_H) {
    mapDrawNumber(tft, x + 1, y + 21, 30, 10, cell.acc);              // below the line
  } else if (cell.shape & SHAPE_SIG) {
    mapDrawNumber(tft, x + 1, y + 1, 11, 11, cell.acc);               // top-left, clear of the dot
  } else if (base == BASE_H) {                      // point: the half without the diagonal
    mapDrawNumber(tft, x + 1, (mask & E_T) ? y + 20 : y + 1, 30, 11, cell.acc);
  } else {
    mapDrawNumber(tft, (mask & E_R) ? x + 1 : x + 21, y + 1, 11, 30, cell.acc);
  }
}

static void mapDrawCell(Adafruit_GFX &tft, uint8_t r, uint8_t c) {
  const MapCell &cell = dccMap[r][c];
  if (!cell.shape) return;
  const int16_t M = MAP_CELL / 2;
  int16_t x = c * MAP_CELL, y = MAP_TOP + r * MAP_CELL;
  uint8_t base = cell.shape & 3, mask = (cell.shape >> 2) & 0x0F;
  if (base == BASE_H) tft.fillRect(x, y + M - 1, MAP_CELL, 3, MAP_TRACK);
  if (base == BASE_V) tft.fillRect(x + M - 1, y, 3, MAP_CELL, MAP_TRACK);
  if (mask) mapDiagonal(tft, x, y, mask);
  if (cell.acc) {
    if (MAP_SHOW_NUMBERS) mapDrawLabel(tft, x, y, cell);
    mapDrawDots(tft, r, c);
  }
}

void dccMapDraw(Adafruit_GFX &tft) {
  tft.setRotation(MAP_ROT);
  tft.fillScreen(MAP_BG);
  if (dccMapLoaded) {
    for (uint8_t r = 0; r < MAP_ROWS; r++)
      for (uint8_t c = 0; c < MAP_COLS; c++) mapDrawCell(tft, r, c);
  } else {
    tft.setTextSize(2);
    tft.setTextColor(MAP_ERR);
    tft.setCursor(12, 150);
    tft.print(dccMapError[0] ? dccMapError : "No track map loaded");
  }
  tft.setRotation(MAP_BASE_ROT);
  showMapStatus();                  // track mode / power line in the top strip
}

// ---------------- Point / signal changes ----------------
// Record the new state of accessory 'acc' (1-2040) on every map square that uses it,
// and redraw those dots if the map is showing. 93 is treated as an ordinary number here.
void dccMapSetPoint(Adafruit_GFX &tft, uint16_t acc, bool on) {
  if (!dccMapLoaded || acc < MAP_FIRST_ACC || acc > MAP_LAST_ACC) return;
  bool draw = dccMapShown;
  if (draw) tft.setRotation(MAP_ROT);
  for (uint8_t r = 0; r < MAP_ROWS; r++)
    for (uint8_t c = 0; c < MAP_COLS; c++)
      if (dccMap[r][c].acc == acc) {
        mapSetCellOn(r, c, on);
        if (draw) mapDrawDots(tft, r, c);
      }
  if (draw) tft.setRotation(MAP_BASE_ROT);
}

// Call for every Zero 1 accessory command. Returns true if it was the map
// screen accessory (93), so the caller has nothing else to do with it.
bool dccMapAccessory(Adafruit_GFX &tft, uint16_t acc, bool on) {
  if (acc == MAP_SCREEN_ACC) {
    if (on) {
      if (!dccMapLoaded) {          // no valid map file at start-up: stay on the loco screen
        Serial.println(F("Acc 93 ignored: no valid map file loaded"));
        return true;
      }
      dccMapShown = true;
      dccMapDraw(tft);
    } else if (dccMapShown) {
      dccMapShown = false;
      tft.setRotation(MAP_BASE_ROT);
      showLocoScreen();
    }
    return true;
  }

  dccMapSetPoint(tft, acc, on);
  return false;
}
