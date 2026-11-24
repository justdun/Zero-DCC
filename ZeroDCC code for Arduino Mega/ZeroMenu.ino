// ZeroMenu.ino - the menu screen for ZeroDCC (the screen after the Acc 93 map)
//
// A second tab of the ZeroDCC sketch: keep it in the same folder as ZeroDCC_Mega_rc3.0.ino and the
// Arduino IDE builds the two together.
//
// Every menu choice does exactly what the Zero 1 keys would: choosing Accessory 9x starts that
// function (cutting the Zero 1 / MicroMimic feed as 9x-> does), its Exit button ends it (as 9x<-).
// The Zero 1 keys keep working at the same time. Using the keys never opens the menu by itself.
// To leave the menu, use the arrows at the screen edges. A function that is still running when you
// leave stays running, and the menu shows the same page when you come back.
//
// Drawn in landscape (480 x 320, status strip across the top), like the map.

// ---------------- Colours ----------------
const uint16_t MN_BG    = MAP_BG;                   // grey / silver, as the map
const uint16_t MN_BTN   = ZP_WHITE;                 // ordinary button
const uint16_t MN_ON    = 0xFFE0;                   // yellow: selected / active / switched on
const uint16_t MN_EXIT  = RGB565(245, 165, 150);    // exit / back / stop buttons
const uint16_t MN_OFF   = RGB565(195, 195, 195);    // not available
const uint16_t MN_TEXT  = 0x0000;
const uint16_t MN_DIM   = RGB565(110, 110, 110);
const uint16_t MN_WARN  = RGB565(170, 0, 0);
const uint16_t MN_EDIT  = 0x07FF;                   // cyan: CAB ID being edited
const uint16_t MN_HEAD  = RGB565(0, 0, 140);        // page title (which accessory you're in)

// ---------------- Pages ----------------
const byte PG_MAIN = 0, PG_90 = 1, PG_91 = 2, PG_92 = 3, PG_95 = 4, PG_95_PLAY = 5, PG_95_REC = 6,
           PG_95_CONFIRM = 7, PG_95_RECORDING = 8, PG_96 = 9, PG_97 = 10, PG_98 = 11, PG_99 = 12, PG_99_FN = 13,
           PG_91_WARN = 14, PG_91_CLEAR = 15, PG_91_PLACE = 16, PG_91_NUMBER = 17, PG_91_CONFIRM = 18,
           PG_91_PROG = 19, PG_91_DONE = 20, PG_91_TYPE = 21, PG_91_SETUP = 22, PG_91_BLOCK = 23;

// ---------------- Button ids ----------------
// 0-9 keypad digits, 10 X, 11 <-, 12 ->, 13 Select, 20-29 main menu (Acc 90-99), 31-34 options,
// 40-49 list entries (recordings / rosters), 50-81 F0-F31, 90+ the rest
const byte ID_X = 10, ID_LEFT = 11, ID_RIGHT = 12, ID_SELECT = 13;
const byte ID_MAIN = 20, ID_OPT = 30, ID_LIST = 40, ID_FN = 50;
const byte ID_EXIT = 90, ID_BACK = 91, ID_PLAY = 92, ID_REC = 93, ID_YES = 94, ID_NO = 95, ID_OK = 96,
           ID_ENDREC = 97, ID_STOPPLAY = 98, ID_DONE = 99;

struct MenuBtn { int16_t x, y, w, h; byte id; };
const byte MENU_MAX_BTN = 36;
MenuBtn Menu_Btn[MENU_MAX_BTN];
byte Menu_Btns = 0;

byte Menu_Page = PG_MAIN;
long Menu_Watch = -1;                // what the page showed last, to redraw when something changes
byte Menu_Sel = 0;                   // 95: recording being confirmed; 96: slot being edited (0 = none)
unsigned int Menu_Entry = 0;         // 96 / 91: number on the keypad
byte Menu_FnSlot = 0;                // 99: slot whose functions are shown
unsigned int Menu_LastAcc = 0;       // 92: last command sent
byte Menu_LastState = 0;
unsigned int Menu_Sent = 0;          // 92: commands sent (so the page notices a new one)
byte Menu_RecList[10];               // 95: recordings found on the SD card
byte Menu_RecCount = 0;
int16_t Menu_KX, Menu_KY, Menu_KW, Menu_KH, Menu_KG;   // keypad position (for redrawing its number box)
byte Menu_ProgMode = 1;              // 91: 1 = loco CAB ID, 2 = accessory module ID
bool Menu_RangeWarn = false;         // 91: number out of range
unsigned long Menu_ProgEnd = 0;      // 91: when the 60 s wait ends
int Menu_ProgShown = -1;             // 91: seconds left, as shown

const unsigned long MENU_PROG_WAIT = 15000;   // 91: wait after the last programming command (no feedback from DCC-EX)
const unsigned long MENU_PROG_GAP  = 5000;    // 91: accessory: time for DCC-EX to finish one CV write before the next
const unsigned long MENU_PROG_POWER = 1500;   // 91: program track power on this long before the command
const unsigned long MENU_R8216_CHARGE = 30000;    // 91: R8216: time on TRACK B (as Main, powered) before programming (was 150 s)
const unsigned long MENU_R8216_STEP   = 300;      // 91: R8216: gap between the quick steps after the charge
const unsigned long MENU_R8216_RECHARGE = 30000; // 91: R8216: recharge on TRACK B (as Main) between writes
const unsigned long MENU_R8216_WRITE = 2000;     // 91: R8216: time for one write (retries off) before TRACK B goes back to Main
const byte MENU_PULSE_LONG = 5;                  // 91: R8247 "Long" point pulse: CV3-6 = 5 (0.5 s); "Short" = 1 (0.1 s, default)

const byte MOD_OTHER = 0, MOD_R8216 = 1, MOD_R8247 = 2;
byte Menu_ModType = MOD_OTHER;       // 91: which accessory module
const byte CH_SHORT = 0, CH_LONG = 1, CH_CONT = 2, CH_LEAVE = 3;
byte Menu_Chan[4];                   // 91: R8247 channels: point short / point long / light continuous / no change

// Acc 91: the CV writes to send, in order (cv 0 = loco address, <W n>)
struct ProgWrite { byte cv; byte val; };
ProgWrite Menu_PW[8];
byte Menu_PWn = 0;                   // writes in the list
byte Menu_PWi = 0;                   // next write to send
unsigned long Menu_ProgNext = 0;     // 91: when the next step is due
byte Menu_ProgStep = 0;              // 91: next step - see PS_... below
byte Menu_ProgChgShown = 0;          // 91: 0 = page shows "programming", 1+ = "charging" before write n
bool Menu_RetryOff = false;          // 91: <D ACK RETRY 0> sent - <D ACK RETRY 2> still to send (not used at present)
const byte PW_REG1 = 255;            // 91: Menu_PW cv for an R8216 register-mode write of register 1 (raw packets)
const byte MENU_R8216_TRIES = 1;     // 91: R8216: times the write is sent, recharging between (was 3; 1 on trial)
// Acc 91 programming steps:
//   all:        PS_WRITE sends the next write; MENU_PROG_GAP later the next one ... then PS_WAIT, <0 PROG>
//   R8216:      PS_B_MAIN <= B MAIN>, PS_B_POWER <1 B> then charge (first) or recharge (between writes),
//               PS_B_PROG <= B PROG>, PS_PROG_ON <1 PROG>, PS_WRITE (register mode, raw packets),
//               and after each write but the last, back to PS_B_MAIN
const byte PS_WRITE = 0, PS_WAIT = 1, PS_B_MAIN = 2, PS_B_POWER = 3, PS_B_PROG = 4, PS_PROG_ON = 5;

const char MN_L90[] PROGMEM = "90  Power";
const char MN_L91[] PROGMEM = "91  DCC Programming";
const char MN_L92[] PROGMEM = "92  Extended Point Setting";
const char MN_L93[] PROGMEM = "93  eMicroMimic";
const char MN_L94[] PROGMEM = "94  Reserved";
const char MN_L95[] PROGMEM = "95  Record / Playback";
const char MN_L96[] PROGMEM = "96  Assign CAB IDs";
const char MN_L97[] PROGMEM = "97  Choose roster";
const char MN_L98[] PROGMEM = "98  Light / Whistle";
const char MN_L99[] PROGMEM = "99  DCC functions";
const char * const MN_LABELS[10] PROGMEM = { MN_L90, MN_L91, MN_L92, MN_L93, MN_L94, MN_L95, MN_L96, MN_L97, MN_L98, MN_L99 };

// ================= Drawing helpers (landscape) =================

void Menu_Text(int16_t x, int16_t y, byte size, uint16_t colour, const __FlashStringHelper *t) {
  tft.setTextSize(size);
  tft.setTextColor(colour);
  tft.setCursor(x, y);
  tft.print(t);
}

void Menu_TextS(int16_t x, int16_t y, byte size, uint16_t colour, const char *t) {
  tft.setTextSize(size);
  tft.setTextColor(colour);
  tft.setCursor(x, y);
  tft.print(t);
}

// Text from flash, wrapped at spaces to lines of at most maxChars; '\n' starts a new line and a
// blank '\n\n' leaves a small gap. Returns the y below the last line.
int16_t Menu_Wrap(int16_t x, int16_t y, byte size, uint16_t colour, const __FlashStringHelper *t, byte maxChars) {
  const char *p = (const char *)t;
  char line[71];
  if (maxChars > 70) maxChars = 70;
  const int16_t lh = 8 * size + 2;
  tft.setTextSize(size);
  tft.setTextColor(colour);
  while (true) {
    // skip the spaces at the start of a line
    while (pgm_read_byte(p) == ' ') p++;
    char c = pgm_read_byte(p);
    if (c == 0) break;
    if (c == '\n') { y += lh / 2; p++; continue; }       // blank line: half a line of space
    // take as many whole words as fit
    byte n = 0, lastSpace = 0;
    const char *q = p;
    while (true) {
      c = pgm_read_byte(q);
      if (c == 0 || c == '\n') { lastSpace = n; break; }
      if (n == maxChars) { if (lastSpace == 0) lastSpace = n; break; }
      if (c == ' ') lastSpace = n;
      line[n++] = c;
      q++;
    }
    if (c == 0 || c == '\n') lastSpace = n;
    line[lastSpace] = 0;
    tft.setCursor(x, y);
    tft.print(line);
    y += lh;
    p += lastSpace;
    if (pgm_read_byte(p) == '\n') p++;
  }
  return y;
}

// the title at the top of every page: which accessory you're in
void Menu_Header(const __FlashStringHelper *title) {
  Menu_Text(24, 34, 2, MN_HEAD, title);
}
// two-line title for pages with the number pad on the right
void Menu_Header2(const __FlashStringHelper *acc, const __FlashStringHelper *name) {
  Menu_Text(30, 34, 2, MN_HEAD, acc);
  Menu_Text(30, 52, 2, MN_HEAD, name);
}

// a box with a 2 px black border and a size-2 label (from flash, or from RAM if fl is NULL)
void Menu_Box(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t fill, uint16_t textColour,
              const __FlashStringHelper *fl, const char *ram, bool centre) {
  tft.fillRect(x, y, w, h, fill);
  tft.drawRect(x, y, w, h, MN_TEXT);
  tft.drawRect(x + 1, y + 1, w - 2, h - 2, MN_TEXT);
  int16_t len = fl ? strlen_P((const char *)fl) : strlen(ram);
  int16_t tx = centre ? x + (w - len * 12) / 2 : x + 10;
  tft.setTextSize(2);
  tft.setTextColor(textColour);
  tft.setCursor(tx, y + (h - 14) / 2);
  if (fl) tft.print(fl); else tft.print(ram);
}

void Menu_Add(int16_t x, int16_t y, int16_t w, int16_t h, byte id) {
  if (Menu_Btns >= MENU_MAX_BTN) return;
  Menu_Btn[Menu_Btns].x = x; Menu_Btn[Menu_Btns].y = y;
  Menu_Btn[Menu_Btns].w = w; Menu_Btn[Menu_Btns].h = h;
  Menu_Btn[Menu_Btns].id = id;
  Menu_Btns++;
}

// a button with a label from flash
void Menu_Button(int16_t x, int16_t y, int16_t w, int16_t h, byte id, const __FlashStringHelper *label,
                 uint16_t fill, bool centre) {
  Menu_Box(x, y, w, h, fill, fill == MN_OFF ? MN_DIM : MN_TEXT, label, NULL, centre);
  if (fill != MN_OFF) Menu_Add(x, y, w, h, id);
}

// a button with a label in RAM
void Menu_ButtonS(int16_t x, int16_t y, int16_t w, int16_t h, byte id, const char *label,
                  uint16_t fill, bool centre) {
  Menu_Box(x, y, w, h, fill, fill == MN_OFF ? MN_DIM : MN_TEXT, NULL, label, centre);
  if (fill != MN_OFF) Menu_Add(x, y, w, h, id);
}

// a number in large text
void Menu_Big_Number(int16_t x, int16_t y, unsigned int n) {
  char buf[6];
  utoa(n, buf, 10);
  Menu_TextS(x, y, 3, MN_TEXT, buf);
}

unsigned int Acc_Block_First(unsigned int acc);

// Acc 91: what is being programmed, in large text - a loco number, or a module's block of 4 ("49 - 52").
// Returns the x just after it.
int16_t Menu_Big_Target(int16_t x, int16_t y) {
  char buf[14];
  if (Menu_ProgMode == 1) utoa(Menu_Entry, buf, 10);
  else {
    unsigned int first = Acc_Block_First(Menu_Entry);
    snprintf_P(buf, sizeof(buf), PSTR("%u - %u"), first, first + 3);
  }
  Menu_TextS(x, y, 3, MN_TEXT, buf);
  return x + strlen(buf) * 18;
}

// Acc 91: "Ch1 = 49  Ch2 = 50" / "Ch3 = 51  Ch4 = 52" (size 1, two lines); the number keyed in is in red
void Menu_Block_Channels(int16_t x, int16_t y) {
  unsigned int first = Acc_Block_First(Menu_Entry);
  for (byte c = 0; c < 4; c++) {
    char buf[16];
    snprintf_P(buf, sizeof(buf), PSTR("Ch%d = %u"), c + 1, first + c);
    Menu_TextS(x + (c % 2) * 90, y + (c / 2) * 12, 1, first + c == Menu_Entry ? MN_WARN : MN_TEXT, buf);
  }
}

// ================= Number pad =================
// 3 columns x 5 rows, laid out like a calculator (7 8 9 on top, as the Zero 1 keys):
// [ number number | X ] [7 8 9] [4 5 6] [1 2 3] [0 | <- | ->]  or  [0 | Select]

unsigned int Menu_Keypad_Value() {
  return Menu_Page == PG_92 ? Z1_Accessory92Number : Menu_Entry;
}

void Menu_Draw_Keypad_Box() {
  tft.setRotation(1);
  int16_t w = 2 * Menu_KW + Menu_KG;
  tft.fillRect(Menu_KX, Menu_KY, w, Menu_KH, 0xFFFF);
  tft.drawRect(Menu_KX, Menu_KY, w, Menu_KH, MN_TEXT);
  tft.drawRect(Menu_KX + 1, Menu_KY + 1, w - 2, Menu_KH - 2, MN_TEXT);
  unsigned int v = Menu_Keypad_Value();
  if (v) {
    char buf[6];
    utoa(v, buf, 10);
    byte size = Menu_KH >= 40 ? 3 : 2;
    tft.setTextSize(size);
    tft.setTextColor(MN_TEXT);
    tft.setCursor(Menu_KX + w - 8 - strlen(buf) * 6 * size, Menu_KY + (Menu_KH - 7 * size) / 2);
    tft.print(buf);
  }
  tft.setRotation(0);
}

void Menu_Keypad(int16_t x0, int16_t y0, int16_t cw, int16_t ch, int16_t g, bool selectMode) {
  Menu_KX = x0; Menu_KY = y0; Menu_KW = cw; Menu_KH = ch; Menu_KG = g;
  char lbl[2] = { 0, 0 };
  Menu_Draw_Keypad_Box();
  tft.setRotation(1);
  Menu_Button(x0 + 2 * (cw + g), y0, cw, ch, ID_X, F("X"), MN_EXIT, true);
  for (byte d = 1; d <= 9; d++) {
    byte r = 3 - (d - 1) / 3, c = (d - 1) % 3;          // 7 8 9 in row 1, 4 5 6 in row 2, 1 2 3 in row 3
    lbl[0] = '0' + d;
    Menu_ButtonS(x0 + c * (cw + g), y0 + r * (ch + g), cw, ch, d, lbl, MN_BTN, true);
  }
  int16_t yb = y0 + 4 * (ch + g);
  Menu_Button(x0, yb, cw, ch, 0, F("0"), MN_BTN, true);
  if (selectMode) {
    Menu_Button(x0 + cw + g, yb, 2 * cw + g, ch, ID_SELECT, F("Select"), MN_ON, true);
  }
  else {
    Menu_Button(x0 + cw + g, yb, cw, ch, ID_LEFT, F("<-"), MN_ON, true);
    Menu_Button(x0 + 2 * (cw + g), yb, cw, ch, ID_RIGHT, F("->"), MN_ON, true);
  }
}

// a digit / X on a keypad that edits Menu_Entry (Acc 91, 96), up to 'limit'
void Menu_Keypad_Entry(byte id, unsigned int limit) {
  if (id <= 9) {
    unsigned long next = (unsigned long)Menu_Entry * 10 + id;
    if (next <= limit) Menu_Entry = next;
  }
  else if (id == ID_X) Menu_Entry /= 10;
  Menu_Draw_Keypad_Box();
}

// ================= Loco strip (Acc 96 / 98 / 99) =================
// The bottom of the screen shows the 16 cab slots the way the loco screen does: CAB IDs at the
// bottom with the sloping red strip of slot numbers above them. It's drawn in portrait with the
// ZeroPanel.h line equation, so the rows slope exactly like the loco screen.
// Positions are portrait x (0 = bottom edge of the landscape screen).
//   Acc 96 / 99:  CAB IDs 0-58, red strip 60-128
//   Acc 98:       light bulbs 0-23, whistles 24-47, CAB IDs 50-102, red strip 104-150

bool Menu_Has_Strip() {
  return Menu_Page == PG_96 || Menu_Page == PG_98 || Menu_Page == PG_99;
}
int16_t Menu_ID_Px()   { return Menu_Page == PG_98 ? 50 : 4; }
int16_t Menu_Red_Px0() { return Menu_Page == PG_98 ? 104 : 60; }
int16_t Menu_Red_Px1() { return Menu_Page == PG_98 ? 150 : 128; }
const int16_t MN_BULB_PX = 12, MN_NOTE_PX = 36;

void Menu_Strip_ID(byte k) {
  tft.setRotation(0);
  tft.setFont();
  uint16_t bg = (Menu_Page == PG_96 && k == Menu_Sel) ? MN_EDIT : (Slot_Fn(k, 0) ? MN_ON : ZP_WHITE);
  int16_t px = Menu_ID_Px();
  tft.setTextSize(2);
  tft.setTextColor(MN_TEXT, bg);
  int16_t y = zpRowCentreY(k, px + 26) - 7;
  tft.setCursor(px + 2, y);
  tft.print("    ");
  tft.setCursor(px + 2, y);
  tft.print(Z1_LocoNo[k]);
}

void Menu_Strip_Icons(byte k) {
  tft.setRotation(1);
  // light bulb (F0): yellow while the light is on
  int16_t lx = zpRowCentreY(k, MN_BULB_PX), ly = ZP_W - 1 - MN_BULB_PX;
  tft.fillCircle(lx, ly - 2, 6, Slot_Fn(k, 0) ? MN_ON : 0xFFFF);
  tft.drawCircle(lx, ly - 2, 6, MN_TEXT);
  tft.fillRect(lx - 3, ly + 4, 7, 4, GRAY);
  // music note (F2): blue while the whistle is sounding
  lx = zpRowCentreY(k, MN_NOTE_PX); ly = ZP_W - 1 - MN_NOTE_PX;
  uint16_t c = Slot_Fn(k, 2) ? BLUE : MN_TEXT;
  tft.fillRect(lx - 7, ly - 9, 15, 19, ZP_WHITE);
  tft.fillCircle(lx - 2, ly + 5, 3, c);
  tft.fillRect(lx, ly - 7, 2, 12, c);
  tft.fillTriangle(lx + 1, ly - 7, lx + 7, ly - 3, lx + 1, ly - 2, c);
  tft.setRotation(0);
}

void Menu_Draw_Strip() {
  int16_t red0 = Menu_Red_Px0(), red1 = Menu_Red_Px1();
  tft.setRotation(0);
  for (int16_t px = 0; px <= red1; px++) {
    int16_t top = zpLineY(0, px), bot = zpLineY(ZP_LINES - 1, px);
    if (bot > ZP_H - 1) bot = ZP_H - 1;
    tft.drawFastVLine(px, top, bot - top + 1, px < red0 ? ZP_WHITE : ZP_RED);
  }
  for (uint8_t k = 0; k < ZP_LINES; k++) {
    int16_t y0 = zpLineY(k, 0), y1 = zpLineY(k, red1);
    tft.drawLine(0, y0, red1, y1, ZP_LINE);
    tft.drawLine(0, y0 + 1, red1, y1 + 1, ZP_LINE);
  }
  int16_t t = zpLineY(0, red1), b = zpLineY(ZP_LINES - 1, red1);
  tft.drawFastVLine(red1, t, b - t + 2, ZP_LINE);           // top edge of the strip
  tft.drawFastVLine(red1 - 1, t, b - t + 2, ZP_LINE);

  // slot numbers 1-16, sideways and stretched like the loco screen (ZeroPanel.h)
  const int16_t cx = (red0 + red1) / 2, yc = (ZP_W - 1) - cx;
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
    top *= ZP_NUM_STRETCH; bottom *= ZP_NUM_STRETCH;
    int16_t x = zpRowCentreY(r, cx) + 1 - total / 2;
    int16_t base = yc - (int16_t)lround((top + bottom) / 2);
    for (uint8_t d = 0; d < n; d++) {
      ZpGlyph g = zpGlyph(buf[d]);
      zpDrawTallGlyph(tft, buf[d], x, base);
      x += g.w + ZP_DIGIT_GAP;
    }
  }

  for (byte k = 1; k <= 16; k++) {
    Menu_Strip_ID(k);
    if (Menu_Page == PG_98) Menu_Strip_Icons(k);
  }
  tft.setRotation(0);
}

// which slot (row) and which part of the strip a landscape tap is on; false if not on the strip
bool Menu_Strip_Hit(int16_t x, int16_t y, byte &k, int16_t &px) {
  px = ZP_W - 1 - y;
  if (px < 0 || px > Menu_Red_Px1()) return false;
  k = 0;
  for (byte r = 1; r <= 16; r++) {
    if (x >= zpLineYf(r - 1, px) && x < zpLineYf(r, px)) { k = r; break; }
  }
  return k != 0;
}

// a loco's CAB ID or functions changed (k = 0: all slots; fn = -1: the CAB ID itself)
void Menu_Loco_Changed(byte k, int fn) {
  if (Z1_Screen != SCREEN_MENU) return;
  if (Menu_Has_Strip()) {
    for (byte s = 1; s <= 16; s++) {
      if (k && s != k) continue;
      Menu_Strip_ID(s);
      if (Menu_Page == PG_98) Menu_Strip_Icons(s);
    }
    tft.setRotation(0);
  }
  else if (Menu_Page == PG_99_FN && (k == 0 || k == Menu_FnSlot)) {
    tft.setRotation(1);
    if (fn >= 0 && fn <= 31) Menu_Fn_Button(fn, false);
    else for (byte f = 0; f < 32; f++) Menu_Fn_Button(f, false);
    tft.setRotation(0);
  }
}

// ================= F0-F31 matrix =================

void Menu_Fn_Button(byte fn, bool add) {
  const int16_t x0 = 24, y0 = 92, cw = 51, ch = 40, g = 3;
  int16_t x = x0 + (fn % 8) * (cw + g), y = y0 + (fn / 8) * (ch + 4);
  char lbl[4] = { 'F', 0, 0, 0 };
  itoa(fn, lbl + 1, 10);
  uint16_t fill = Slot_Fn(Menu_FnSlot, fn) ? MN_ON : MN_BTN;
  Menu_Box(x, y, cw, ch, fill, MN_TEXT, NULL, lbl, true);
  if (add) Menu_Add(x, y, cw, ch, ID_FN + fn);
}

// ================= SD card helpers =================

String Menu_Rec_Name(int n)    { return String(Z1_Recording_File + String(n, DEC) + ".txt"); }
String Menu_Roster_Name(int n) { return String(Z1_Settings_File + String(n, DEC) + ".txt"); }

// the recordings on the card (Z_DCCR<n>.TXT), lowest numbers first, up to 10
void Menu_Scan_Recordings() {
  Menu_RecCount = 0;
  if (!Z1_SDOK) return;
  File dir = SD.open("/");
  if (!dir) return;
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    const char *nm = entry.name();
    if (!entry.isDirectory() && strncasecmp_P(nm, PSTR("Z_DCCR"), 6) == 0 && isdigit(nm[6])) {
      int n = atoi(nm + 6);
      if (n >= 1 && n <= 99) {
        byte i = Menu_RecCount;
        while (i > 0 && Menu_RecList[i - 1] > n) {           // keep the list in order
          if (i < 10) Menu_RecList[i] = Menu_RecList[i - 1];
          i--;
        }
        if (i < 10) {
          Menu_RecList[i] = n;
          if (Menu_RecCount < 10) Menu_RecCount++;
        }
      }
    }
    entry.close();
  }
  dir.close();
}

// ================= Acc 91 programming commands =================

// The writes for the number on the keypad, into Menu_PW:
//   loco:          <W n> (DCC-EX writes the loco address CVs itself)
//   accessory:     the block ("Group Number") holding 'acc' is (acc - 1) / 4 + 1, held as
//                  Group = CV1 + 64 x CV9 (R8247 manual v2, Appendix 3): CV1 0-63, CV9 0-7.
//                  The R8247 refuses any write that would leave Group 0 (CV1 = 0 with CV9 = 0), so the order
//                  matters: writing CV9 first went wrong coming DOWN from a high group (from 253-256, CV9 1
//                  CV1 0, "CV9 = 0" is refused and the module ends up on 345-348 instead of 89-92). So three
//                  writes, each valid whatever the module held before:
//                  CV1 = new CV1 (or 1 if that's 0), CV9 = new CV9, CV1 = new CV1 again (CV1 last, as the manual)
//   R8247 first:   CV33 = 0 (Standard mode, always), then CV3-6 for each channel that is changed:
//                  1 = short pulse 0.1 s (default), MENU_PULSE_LONG = long pulse, 0 = continuous (lights)
void Menu_PW_Add(byte cv, byte val) {
  if (Menu_PWn < 8) { Menu_PW[Menu_PWn].cv = cv; Menu_PW[Menu_PWn].val = val; Menu_PWn++; }
}
void Menu_91_Build() {
  Menu_PWn = 0;
  Menu_PWi = 0;
  if (Menu_ProgMode == 1) { Menu_PW_Add(0, Menu_Entry); return; }
  if (Menu_ModType == MOD_R8247) {
    Menu_PW_Add(33, 0);
    for (byte c = 0; c < 4; c++)
      if (Menu_Chan[c] != CH_LEAVE)
        Menu_PW_Add(3 + c, Menu_Chan[c] == CH_SHORT ? 1 : Menu_Chan[c] == CH_LONG ? MENU_PULSE_LONG : 0);
  }
  unsigned int block = (Menu_Entry - 1) / 4 + 1;
  if (Menu_ModType == MOD_R8216) {           // R8216: Register mode only (Hornby Elite manual), group 1-63
    for (byte i = 0; i < MENU_R8216_TRIES; i++) Menu_PW_Add(PW_REG1, block);
    return;
  }
  byte cv1 = block & 63;
  Menu_PW_Add(1, cv1 ? cv1 : 1);
  Menu_PW_Add(9, block >> 6);
  Menu_PW_Add(1, cv1);
}

// send one write from the list
//   R8216: the Hornby Elite manual says it "must be programmed in Reg mode" - DCC-EX's <W> uses Direct
//   mode, which it ignores. DCC-EX has no Register mode, so the packets are sent raw on the program
//   track with <P 0 b1 b2> (hex bytes; DCC-EX adds the checksum and sends each one 4 times, with reset
//   packets in between commands). Physical register write (NMRA S-9.2.3): 0111 1RRR DDDDDDDD,
//   register 1 (RRR = 000) = the address (group number). Sent 3 times back to back = 12 write packets.
void Prog_Write(byte i) {
  if (Menu_PW[i].cv == PW_REG1) {
    for (byte r = 0; r < 3; r++) {
      DCC_Out.print("<P 0 78 ");
      DCC_Out.print(Menu_PW[i].val, HEX);
      DCC_Out.println(">");
    }
    return;
  }
  DCC_Out.print("<W ");
  if (Menu_PW[i].cv == 0) DCC_Out.print(Menu_Entry);
  else { DCC_Out.print(Menu_PW[i].cv); DCC_Out.print(" "); DCC_Out.print(Menu_PW[i].val); }
  DCC_Out.println(">");
}

// how long the whole sequence should take (for the countdown; the last wait is timed from the last write)
unsigned long Menu_91_Total() {
  if (Menu_ModType == MOD_R8216 && Menu_ProgMode == 2)
    return MENU_R8216_STEP + MENU_R8216_CHARGE + 2 * MENU_R8216_STEP +
           (unsigned long)(Menu_PWn - 1) * (MENU_R8216_WRITE + MENU_R8216_STEP + MENU_R8216_RECHARGE + 2 * MENU_R8216_STEP) +
           MENU_PROG_WAIT;
  return MENU_PROG_POWER + (unsigned long)(Menu_PWn - 1) * MENU_PROG_GAP + MENU_PROG_WAIT;
}

// first / last accessory number of the block of four that 'acc' is in
unsigned int Acc_Block_First(unsigned int acc) { return ((acc - 1) / 4) * 4 + 1; }

// program track power on / off (DCC-EX <1 PROG> / <0 PROG>) - only the program track, main stays off
void Prog_Track_Power(bool on) {
  DCC_Out.println(on ? "<1 PROG>" : "<0 PROG>");
  Z1_ProgPower = on ? 1 : 0;
  Draw_Track_Power();                        // status strip: TRACK POWER PROG while it's on
}

// R8216: TRACK B switched to Main for charging (status strip: TRACK B CHARGING)
void Prog_B_Main() {
  DCC_Out.println("<= B MAIN>");
  Z1_ProgPower = 2;
  Draw_Track_Power();
}

// R8216: DCC-EX retries back on (2 = its default)
void Prog_Retry_Restore() {
  if (Menu_RetryOff) { DCC_Out.println("<D ACK RETRY 2>"); Menu_RetryOff = false; }
}

// Acc 91 ended part way through programming: power off, and TRACK B back to Program if it was charging
void Prog_Abort() {
  Prog_Retry_Restore();
  if (Z1_ProgPower == 2) {
    DCC_Out.println("<0 B>");
    DCC_Out.println("<= B PROG>");
    Z1_ProgPower = 0;
    Draw_Track_Power();
  }
  else if (Z1_ProgPower) Prog_Track_Power(false);
}

// R8216 charging on TRACK B: 1 + the write it is charging for; 0 = not charging
byte Menu_91_Phase() { return Z1_ProgPower == 2 ? Menu_PWi + 1 : 0; }

// ================= Pages =================

void Menu_Page_Main() {
  for (byte i = 0; i < 10; i++) {
    byte n = 90 + i;
    bool active = (n == 90 && Z1_Accessory90Flag) || (n == 91 && Z1_Accessory91Flag) ||
                  (n == 92 && Z1_Accessory92Flag) || (n == 95 && Z1_Accessory95Flag) ||
                  (n == 96 && Z1_Accessory96Flag) || (n == 97 && Z1_Accessory97Flag) ||
                  (n == 98 && Z1_Accessory98Flag) || (n == 99 && Z1_Accessory99Flag);
    uint16_t fill = active ? MN_ON : MN_BTN;
    if (n == 94 || (n == 93 && !dccMapLoaded)) fill = MN_OFF;
    Menu_Button(60, 32 + i * 29, 360, 26, ID_MAIN + i,
                (const __FlashStringHelper *)pgm_read_ptr(&MN_LABELS[i]), fill, false);
  }
}

void Menu_Page_90() {
  Menu_Header(F("Acc 90  Power"));
  bool on = Z1_TrackPowerOn || Z1_PowerPending;
  Menu_Button(30, 60, 420, 34, ID_OPT + 1, F("1->  Power on"), on ? MN_ON : MN_BTN, false);
  Menu_Button(30, 100, 420, 34, ID_OPT + 2, F("2->  A: DCC Main,  B: DCC Program"), Z1_TrackType == 2 ? MN_ON : MN_BTN, false);
  Menu_Button(30, 140, 420, 34, ID_OPT + 3, F("3->  A: DCC Main,  B: DC CAB 999"), Z1_TrackType == 3 ? MN_ON : MN_BTN, false);
  Menu_Button(30, 180, 420, 34, ID_OPT + 4, F("4->  A: 12V DC 999, B: 12V DC 998"), Z1_TrackType == 4 ? MN_ON : MN_BTN, false);
  Menu_Text(30, 222, 1, MN_TEXT, F("Power on happens when you exit (as 90<- on the Zero 1)."));
  Menu_Text(30, 234, 1, MN_TEXT, F("2-4 set the track mode and turn track power off."));
  Menu_Button(30, 262, 200, 40, ID_EXIT, F("90<-  Exit"), MN_EXIT, false);
}

// ---- Acc 91 ----

void Menu_Page_91() {
  Menu_Header(F("Acc 91  DCC Programming"));
  Menu_Button(30, 70, 420, 40, ID_OPT + 1, F("1->  Program a loco CAB ID"), MN_BTN, false);
  Menu_Button(30, 120, 420, 40, ID_OPT + 2, F("2->  Program accessory module ID"), MN_BTN, false);
  Menu_Button(30, 262, 200, 40, ID_EXIT, F("91<-  Exit"), MN_EXIT, false);
}

void Menu_Page_91_Type() {
  Menu_Header(F("Acc 91  DCC Programming"));
  Menu_Text(30, 62, 2, MN_TEXT, F("Which accessory module is it?"));
  Menu_Button(30, 88, 420, 38, ID_OPT + 1, F("1->  Hornby R8216"), MN_BTN, false);
  Menu_Button(30, 132, 420, 38, ID_OPT + 2, F("2->  Hornby R8247"), MN_BTN, false);
  Menu_Button(30, 176, 420, 38, ID_OPT + 3, F("3->  Other module"), MN_BTN, false);
  Menu_Wrap(30, 222, 1, MN_TEXT, F("R8216: charged on TRACK B for 30 s before programming.\nR8247: you can also set each channel to a point (short or long pulse) or a light."), 70);
  Menu_Button(30, 262, 200, 40, ID_BACK, F("Back"), MN_EXIT, true);
}

// ---- Block of 4: which numbers the module will really use ----
void Menu_Page_91_Block() {
  Menu_Header(F("Acc 91  DCC Programming"));
  int16_t y = Menu_Wrap(30, 62, 2, MN_TEXT, F("DCC Accessory Modules use numbers in fixed blocks of 4 (1-4, 5-8, 9-12 ...)."), 35);
  char buf[40];
  unsigned int first = Acc_Block_First(Menu_Entry);
  snprintf_P(buf, sizeof(buf), PSTR("You chose %u, so this module"), Menu_Entry);
  Menu_TextS(30, y + 8, 2, MN_TEXT, buf);
  Menu_Text(30, y + 26, 2, MN_TEXT, F("will be programmed to:"));
  y += 52;
  Menu_Big_Target(30, y);
  y += 34;
  for (byte c = 0; c < 4; c++) {
    snprintf_P(buf, sizeof(buf), PSTR("Channel %d = %u"), c + 1, first + c);
    Menu_TextS(30 + (c % 2) * 210, y + (c / 2) * 20, 2, first + c == Menu_Entry ? MN_WARN : MN_TEXT, buf);
  }
  Menu_Button(80, 262, 140, 44, ID_OK, F("OK"), MN_BTN, true);
  Menu_Button(260, 262, 140, 44, ID_BACK, F("Change"), MN_EXIT, true);
}

// ---- R8247 setup: how each channel works ----
const int16_t MN_CX[4] = {82, 162, 242, 382};   // column x / width: Short, Long, Continuous, Leave
const int16_t MN_CW[4] = {76, 76, 136, 76};
const char MN_C0[] PROGMEM = "Short";
const char MN_C1[] PROGMEM = "Long";
const char MN_C2[] PROGMEM = "Continuous";
const char MN_C3[] PROGMEM = "Leave";
const char * const MN_CHAN[4] PROGMEM = { MN_C0, MN_C1, MN_C2, MN_C3 };
const char MN_S0[] PROGMEM = "Point - Short (default)";
const char MN_S1[] PROGMEM = "Point - Long";
const char MN_S2[] PROGMEM = "Light - Continuous";
const char MN_S3[] PROGMEM = "No change";
const char * const MN_CHAN_SUM[4] PROGMEM = { MN_S0, MN_S1, MN_S2, MN_S3 };

void Menu_Col_Head(byte col, int16_t y, const __FlashStringHelper *t) {
  int16_t len = strlen_P((const char *)t);
  Menu_Text(MN_CX[col] + (MN_CW[col] - len * 6) / 2, y, 1, MN_TEXT, t);
}

void Menu_Page_91_Setup() {
  Menu_Header(F("Acc 91  R8247 Setup"));
  unsigned int first = Acc_Block_First(Menu_Entry);
  Menu_Text(24, 62, 2, MN_TEXT, F("Choose how each channel works:"));
  Menu_Text(24, 86, 1, MN_TEXT, F("Accessory"));
  Menu_Col_Head(0, 86, F("Point"));
  Menu_Col_Head(1, 86, F("Point"));
  Menu_Col_Head(2, 86, F("Light"));
  Menu_Col_Head(3, 86, F("No change"));
  Menu_Col_Head(0, 96, F("(Default)"));
  for (byte c = 0; c < 4; c++) {
    int16_t y = 107 + c * 39;
    char buf[6];
    utoa(first + c, buf, 10);
    Menu_TextS(24, y + 10, 2, first + c == Menu_Entry ? MN_WARN : MN_TEXT, buf);   // the number keyed in: red
    for (byte i = 0; i < 4; i++)
      Menu_Button(MN_CX[i], y, MN_CW[i], 34, ID_LIST + c * 4 + i,
                  (const __FlashStringHelper *)pgm_read_ptr(&MN_CHAN[i]), Menu_Chan[c] == i ? MN_ON : MN_BTN, true);
  }
  Menu_Button(240, 272, 100, 38, ID_DONE, F("Next"), MN_BTN, true);
  Menu_Button(350, 272, 100, 38, ID_BACK, F("Back"), MN_EXIT, true);
}

void Menu_Page_91_Warn() {
  Menu_Header(F("Acc 91  DCC Programming"));
  int16_t y;
  if (Menu_ProgMode == 1)
    y = Menu_Wrap(30, 64, 2, MN_WARN, F("This function will re-program your DCC locomotive's ID. If not performed correctly this could cause damage to your locomotive DCC decoder."), 35);
  else
    y = Menu_Wrap(30, 64, 2, MN_WARN, F("This function will re-program your DCC Accessory Module's ID. If not performed correctly this could cause damage to your DCC Accessory Module."), 35);
  Menu_Text(30, y + 14, 2, MN_TEXT, F("Do you want to proceed?"));
  Menu_Button(80, 250, 140, 50, ID_YES, F("Yes"), MN_BTN, true);
  Menu_Button(260, 250, 140, 50, ID_NO, F("No"), MN_EXIT, true);
}

// compact: one line shorter, for the page with the most text
int16_t Menu_91_Track_Text(bool compact) {
  int16_t y = compact ? Menu_Wrap(30, 60, 2, MN_TEXT, F("Track power OFF and changed to:"), 35)
                      : Menu_Wrap(30, 60, 2, MN_TEXT, F("Track power has been turned OFF and changed to:"), 35);
  Menu_Text(54, y + 2, 2, MN_TEXT, F("Track A: Main   Track B: Program"));
  return y + (compact ? 20 : 26);
}

void Menu_Page_91_Clear() {
  Menu_Header(F("Acc 91  DCC Programming"));
  int16_t y = Menu_91_Track_Text(false);
  Menu_Wrap(30, y + 16, 2, MN_WARN, F("PLEASE REMOVE ALL LOCOMOTIVES FROM ALL TRACKS"), 35);
  Menu_Button(80, 262, 140, 44, ID_DONE, F("Done"), MN_BTN, true);
  Menu_Button(260, 262, 140, 44, ID_BACK, F("Back"), MN_EXIT, true);
}

void Menu_Page_91_Place() {
  Menu_Header(F("Acc 91  DCC Programming"));
  int16_t y = Menu_91_Track_Text(true);
  y = Menu_Wrap(30, y + 4, 2, MN_TEXT, F("Please ensure that TRACK B: PROGRAM TRACK has been cleaned."), 35);
  if (Menu_ProgMode == 1) {
    y = Menu_Wrap(30, y + 4, 2, MN_TEXT, F("Please now place the locomotive to be programmed on TRACK B: PROGRAM TRACK."), 35);
    Menu_Wrap(30, y + 4, 2, MN_TEXT, F("Make sure there is enough room for it to move forward as it is being programmed."), 35);
  }
  else {
    Menu_Wrap(30, y + 4, 2, MN_TEXT, F("Please now connect the DCC Accessory Module to be programmed to TRACK B: PROGRAM TRACK."), 35);
  }
  Menu_Button(80, 274, 140, 38, ID_DONE, F("Done"), MN_BTN, true);
  Menu_Button(260, 274, 140, 38, ID_BACK, F("Back"), MN_EXIT, true);
}

void Menu_Page_91_Number() {
  Menu_Header2(F("Acc 91"), F("DCC Programming"));
  int16_t y;
  if (Menu_ProgMode == 1)
    y = Menu_Wrap(30, 82, 2, MN_TEXT, F("Select the locomotive CAB number you wish to be programmed (1-99 only)"), 17);
  else if (Menu_ModType == MOD_R8216)
    y = Menu_Wrap(30, 82, 2, MN_TEXT, F("Select the DCC Accessory Module ID number you wish to be programmed (1-252 only)"), 17);
  else
    y = Menu_Wrap(30, 82, 2, MN_TEXT, F("Select the DCC Accessory Module ID number you wish to be programmed (1-2040 only)"), 17);
  if (Menu_RangeWarn)
    Menu_Wrap(30, y + 8, 2, MN_WARN, Menu_ProgMode == 1 ? F("Please enter 1-99") :
                                     Menu_ModType == MOD_R8216 ? F("Please enter 1-252") : F("Please enter 1-2040"), 17);
  Menu_Button(30, 262, 180, 40, ID_BACK, F("Back"), MN_EXIT, true);
  Menu_Keypad(250, 36, 66, 52, 4, true);
}

void Menu_Page_91_Confirm() {
  Menu_Header(F("Acc 91  DCC Programming"));
  int16_t y;
  if (Menu_ProgMode == 1)
    y = Menu_Wrap(30, 62, 2, MN_TEXT, F("Are you sure you want to program the locomotive on your program track to the DCC locomotive number of:"), 35);
  else
    y = Menu_Wrap(30, 62, 2, MN_TEXT, F("Are you sure you want to program the DCC Accessory Module connected to your program track to the DCC ID numbers:"), 35);
  int16_t xe = Menu_Big_Target(30, y + 6);
  if (Menu_ProgMode == 2) {
    char buf[48];
    unsigned int first = Acc_Block_First(Menu_Entry);
    if (Menu_ModType == MOD_R8247) {
      Menu_Text(xe + 20, y + 6, 1, MN_TEXT, F("Hornby R8247"));
      Menu_Text(xe + 20, y + 18, 1, MN_TEXT, F("Mode: Standard"));
      for (byte c = 0; c < 4; c++) {
        snprintf_P(buf, sizeof(buf), PSTR("%u: "), first + c);
        strcat_P(buf, (const char *)pgm_read_ptr(&MN_CHAN_SUM[Menu_Chan[c]]));
        Menu_TextS(30 + (c % 2) * 210, y + 34 + (c / 2) * 12, 1, MN_TEXT, buf);
      }
    }
    else {
      Menu_Block_Channels(xe + 20, y + 6);
      if (Menu_ModType == MOD_R8216)
        Menu_Wrap(30, y + 50, 1, MN_TEXT, F("R8216: charged on TRACK B for 30 s, then written in Register mode (as the Hornby Elite does)."), 70);
    }
  }
  Menu_Button(80, 250, 140, 50, ID_YES, F("Yes"), MN_BTN, true);
  Menu_Button(260, 250, 140, 50, ID_NO, F("No"), MN_EXIT, true);
}

void Menu_91_Countdown() {
  long left = (long)(Menu_ProgEnd - millis());
  int secs = left > 0 ? (left + 999) / 1000 : 0;
  Menu_ProgShown = secs;
  tft.setRotation(1);
  tft.fillRect(30, 250, 300, 30, MN_BG);
  char buf[24];
  if (secs > 60) snprintf_P(buf, sizeof(buf), PSTR("Please wait %d:%02d"), secs / 60, secs % 60);
  else snprintf_P(buf, sizeof(buf), PSTR("Please wait %d s"), secs);
  Menu_TextS(30, 254, 3, MN_TEXT, buf);
  tft.setRotation(0);
}

void Menu_Page_91_Prog() {
  Menu_Header(F("Acc 91  DCC Programming"));
  int16_t y;
  Menu_ProgChgShown = Menu_91_Phase();
  if (Menu_ProgChgShown == 1)
    y = Menu_Wrap(30, 62, 2, MN_TEXT, F("Charging the R8216 on TRACK B for 30 s, then programming it to ID numbers:"), 35);
  else if (Menu_ProgChgShown)
    y = Menu_Wrap(30, 62, 2, MN_TEXT, F("Recharging the R8216 on TRACK B for 30 s before the next write, programming it to ID numbers:"), 35);
  else if (Menu_ProgMode == 1)
    y = Menu_Wrap(30, 62, 2, MN_TEXT, F("Programming locomotive on TRACK B: Programming Track to locomotive number of:"), 35);
  else
    y = Menu_Wrap(30, 62, 2, MN_TEXT, F("Programming DCC Accessory Module on TRACK B: Programming Track to ID numbers:"), 35);
  int16_t xe = Menu_Big_Target(30, y + 6);
  if (!Menu_ProgChgShown) Menu_Text(xe + 18, y + 6, 3, MN_TEXT, F("NOW!"));
  Menu_Wrap(30, y + 46, 2, MN_WARN, Menu_ProgMode == 1 ? F("DO NOT REMOVE LOCOMOTIVE FROM TRACK!")
                                                       : F("DO NOT DISCONNECT DCC ACCESSORY MODULE FROM TRACK!"), 35);
  tft.setRotation(1);
  Menu_91_Countdown();
  tft.setRotation(1);
}

void Menu_Page_91_Done() {
  Menu_Header(F("Acc 91  DCC Programming"));
  int16_t y;
  if (Menu_ProgMode == 1) {
    y = Menu_Wrap(30, 62, 2, MN_TEXT, F("When the locomotive has stopped moving, it should have completed programming."), 35);
    y = Menu_Wrap(30, y + 8, 2, MN_TEXT, F("Take the locomotive off the program track and test it on TRACK A: MAIN under the new locomotive CAB number of"), 35);
  }
  else {
    y = Menu_Wrap(30, 62, 2, MN_TEXT, F("Your DCC Accessory Module should have completed programming."), 35);
    y = Menu_Wrap(30, y + 8, 2, MN_TEXT, F("Disconnect the DCC Accessory Module from the program track and test it on TRACK A: MAIN under the new DCC Accessory Module ID numbers"), 35);
  }
  int16_t xe = Menu_Big_Target(30, y + 4);
  unsigned int first = Acc_Block_First(Menu_Entry);
  if (Menu_ProgMode == 2) Menu_Block_Channels(xe + 20, y + 4);
  if (Menu_ProgMode == 2 && first <= 99 && first + 3 >= 90)   // Zero 1 keys 90-99 are ZeroDCC functions
    Menu_Wrap(30, y + 34, 1, MN_WARN, F("90-99 on the Zero 1 keys open ZeroDCC's own functions, so they can't switch this module: test it with Acc 92 (key 90 in, then -> or <-) or from the map."), 70);
  Menu_Button(160, 262, 160, 44, ID_OK, F("OK"), MN_BTN, true);
}

// ---- Acc 92 ----

void Menu_92_Info() {
  tft.setRotation(1);
  tft.fillRect(24, 164, 222, 64, MN_BG);
  if (Menu_LastAcc) {
    char buf[20];
    snprintf_P(buf, sizeof(buf), PSTR("Sent: %u %s"), Menu_LastAcc, Menu_LastState ? "<-" : "->");
    Menu_TextS(30, 172, 2, MN_TEXT, buf);
  }
  if (!Z1_TrackPowerOn) {
    Menu_Text(30, 202, 1, MN_WARN, F("Track power is off -"));
    Menu_Text(30, 212, 1, MN_WARN, F("nothing will be sent"));
  }
  tft.setRotation(0);
  Menu_Draw_Keypad_Box();
}

void Menu_Page_92() {
  Menu_Header2(F("Acc 92"), F("Extended Points"));
  Menu_Text(30, 88, 1, MN_TEXT, F("Enter the accessory number"));
  Menu_Text(30, 100, 1, MN_TEXT, F("(1-2040), then touch <- or ->."));
  Menu_Text(30, 112, 1, MN_TEXT, F("Touch an arrow again to keep"));
  Menu_Text(30, 124, 1, MN_TEXT, F("changing the same one."));
  Menu_Text(30, 136, 1, MN_TEXT, F("X deletes the last digit, or"));
  Menu_Text(30, 148, 1, MN_TEXT, F("the whole number once sent."));
  Menu_Button(30, 262, 180, 40, ID_EXIT, F("92<-  Exit"), MN_EXIT, false);
  Menu_Keypad(250, 36, 66, 52, 4, false);
  Menu_92_Info();
}

// ---- Acc 95 ----

void Menu_Page_95() {
  Menu_Header(F("Acc 95  Record / Playback"));
  char buf[28];
  if (Z1_Accessory95Flag == 2) {
    snprintf_P(buf, sizeof(buf), PSTR("Recording %d in progress"), Z1_RecordNumber);
    Menu_TextS(30, 62, 2, MN_WARN, buf);
  }
  else if (Z1_Accessory95Flag == 3) {
    snprintf_P(buf, sizeof(buf), PSTR("Playing recording %d"), Z1_PlaybackNumber);
    Menu_TextS(30, 62, 2, MN_WARN, buf);
  }
  Menu_Button(90, 92, 300, 46, ID_PLAY, F("Playback"), MN_BTN, true);
  Menu_Button(90, 172, 300, 46, ID_REC, F("Record"), MN_BTN, true);
  Menu_Button(30, 262, 200, 40, ID_EXIT, F("95<-  Exit"), MN_EXIT, false);
}

void Menu_Page_95_Play() {
  Menu_Header(F("Acc 95  Playback"));
  if (!Z1_SDOK) {
    Menu_Text(30, 80, 2, MN_WARN, F("SD card error - no recordings"));
  }
  else {
    Menu_Scan_Recordings();
    if (Menu_RecCount == 0) {
      Menu_Text(30, 80, 2, MN_TEXT, F("No recordings found"));
    }
    else {
      Menu_Text(30, 58, 2, MN_TEXT, F("Touch a recording to start playback"));
      char buf[16];
      for (byte i = 0; i < Menu_RecCount; i++) {
        snprintf_P(buf, sizeof(buf), PSTR("Recording %d"), Menu_RecList[i]);
        bool playing = Z1_Accessory95Flag == 3 && Z1_PlaybackNumber == Menu_RecList[i];
        Menu_ButtonS(30 + (i / 5) * 215, 84 + (i % 5) * 34, 205, 30, ID_LIST + i, buf, playing ? MN_ON : MN_BTN, false);
      }
    }
    Menu_Button(30, 268, 250, 40, ID_STOPPLAY, F("95<-  Stop Playback"),
                Z1_Accessory95Flag == 3 ? MN_EXIT : MN_OFF, false);
  }
  Menu_Button(300, 268, 150, 40, ID_BACK, F("Back"), MN_EXIT, true);
}

void Menu_Page_95_Rec() {
  Menu_Header(F("Acc 95  Record"));
  if (!Z1_SDOK) {
    Menu_Text(30, 80, 2, MN_WARN, F("SD card error - can't record"));
  }
  else {
    Menu_Text(30, 58, 2, MN_TEXT, F("Choose a recording"));
    char buf[16];
    for (byte i = 0; i < 10; i++) {
      int16_t x = 30 + (i / 5) * 215, y = 84 + (i % 5) * 36;
      snprintf_P(buf, sizeof(buf), PSTR("Recording %d"), i + 1);
      Menu_ButtonS(x, y, 205, 32, ID_LIST + i, buf, MN_BTN, false);
      if (SD.exists(Menu_Rec_Name(i + 1))) Menu_Text(x + 152, y + 13, 1, MN_DIM, F("exists"));
    }
  }
  Menu_Button(30, 268, 150, 40, ID_BACK, F("Back"), MN_EXIT, true);
}

void Menu_Page_95_Confirm() {
  Menu_Header(F("Acc 95  Record"));
  Menu_Text(30, 62, 3, MN_TEXT, F("Are you sure?"));
  char buf[32];
  bool exists = SD.exists(Menu_Rec_Name(Menu_Sel));
  snprintf_P(buf, sizeof(buf), exists ? PSTR("Recording %d already exists") : PSTR("Recording %d is available"), Menu_Sel);
  Menu_TextS(30, 104, 2, exists ? MN_WARN : MN_TEXT, buf);
  if (exists) Menu_Text(30, 128, 1, MN_TEXT, F("It will be replaced by the new recording."));
  if (Z1_Accessory95Flag == 3) Menu_Text(30, 142, 1, MN_TEXT, F("The recording playing now will be stopped."));
  Menu_Button(80, 190, 140, 56, ID_YES, F("Yes"), MN_BTN, true);
  Menu_Button(260, 190, 140, 56, ID_NO, F("No"), MN_EXIT, true);
}

void Menu_Page_95_Recording() {
  Menu_Header(F("Acc 95  Record"));
  char buf[28];
  snprintf_P(buf, sizeof(buf), PSTR("Recording %d in progress"), Z1_RecordNumber);
  Menu_TextS(30, 60, 2, MN_WARN, buf);
  Menu_Text(30, 92, 2, MN_TEXT, F("You are free to control via the"));
  Menu_Text(30, 112, 2, MN_TEXT, F("Zero 1 or other menus."));
  Menu_Text(30, 144, 2, MN_TEXT, F("Key 95<- on the Zero 1 to end"));
  Menu_Text(30, 164, 2, MN_TEXT, F("the recording."));
  Menu_Button(30, 230, 150, 50, ID_OK, F("OK"), MN_BTN, true);
  Menu_Button(210, 230, 240, 50, ID_ENDREC, F("End Recording"), MN_EXIT, true);
}

// ---- Acc 96 - 99 ----

void Menu_Page_96() {
  Menu_Header(F("Acc 96  Assign CAB IDs"));
  Menu_Text(24, 58, 2, MN_TEXT, F("Touch a Loco ID"));
  Menu_Text(24, 76, 2, MN_TEXT, F("to edit"));
  if (Menu_Sel) {
    char buf[12];
    snprintf_P(buf, sizeof(buf), PSTR("Slot %d:"), Menu_Sel);
    Menu_TextS(24, 102, 2, MN_TEXT, buf);
    Menu_Text(24, 122, 1, MN_TEXT, F("Enter the new CAB ID,"));
    Menu_Text(24, 132, 1, MN_TEXT, F("then touch Select"));
    Menu_Keypad(300, 32, 50, 28, 3, true);
  }
  Menu_Button(24, 148, 160, 36, ID_EXIT, F("96<- Exit"), MN_EXIT, false);
  Menu_Draw_Strip();
}

void Menu_Page_97() {
  Menu_Header(F("Acc 97  Choose roster"));
  if (!Z1_SDOK) {
    Menu_Text(30, 80, 2, MN_WARN, F("SD card error - no rosters"));
  }
  else {
    Menu_Text(30, 58, 2, MN_TEXT, F("Choose a roster:"));
    char buf[12];
    for (byte i = 0; i < 10; i++) {
      int16_t x = 30 + (i / 5) * 215, y = 84 + (i % 5) * 36;
      snprintf_P(buf, sizeof(buf), PSTR("Roster %d"), i + 1);
      Menu_ButtonS(x, y, 205, 32, ID_LIST + i, buf, Z1_Accessory97Bank == i + 1 ? MN_ON : MN_BTN, false);
      Menu_Text(x + 160, y + 13, 1, MN_DIM, SD.exists(Menu_Roster_Name(i + 1)) ? F("exists") : F("empty"));
    }
  }
  Menu_Button(30, 268, 200, 40, ID_EXIT, F("97<-  Exit"), MN_EXIT, false);
}

void Menu_Page_98() {
  Menu_Header(F("Acc 98  Light / Whistle"));
  Menu_Text(24, 58, 2, MN_TEXT, F("Touch the whistle or light icon"));
  Menu_Text(24, 76, 2, MN_TEXT, F("to switch it on a locomotive"));
  Menu_Button(24, 100, 160, 36, ID_EXIT, F("98<- Exit"), MN_EXIT, false);
  Menu_Draw_Strip();
}

void Menu_Page_99() {
  Menu_Header(F("Acc 99  DCC functions"));
  Menu_Text(24, 58, 2, MN_TEXT, F("Select a locomotive to control"));
  Menu_Text(24, 76, 2, MN_TEXT, F("its DCC functions"));
  Menu_Button(24, 148, 160, 36, ID_EXIT, F("99<- Exit"), MN_EXIT, false);
  Menu_Draw_Strip();
}

void Menu_Page_99_Fn() {
  Menu_Header(F("Acc 99  DCC functions"));
  Menu_Text(24, 54, 2, MN_TEXT, F("Choose a DCC function to turn on/off"));
  char buf[28];
  snprintf_P(buf, sizeof(buf), PSTR("Slot %d - CAB %d"), Menu_FnSlot, Z1_LocoNo[Menu_FnSlot]);
  Menu_TextS(24, 72, 2, MN_TEXT, buf);
  for (byte f = 0; f < 32; f++) Menu_Fn_Button(f, true);
  Menu_Button(24, 270, 150, 38, ID_BACK, F("Back"), MN_EXIT, true);
}

// ================= Page control =================

bool Menu_Page_Is_91() { return Menu_Page == PG_91 || (Menu_Page >= PG_91_WARN && Menu_Page <= PG_91_BLOCK); }

// a number that changes when something shown on the current page changes
long Menu_Signature() {
  switch (Menu_Page) {
    case PG_MAIN:
      return (long)Z1_Accessory90Flag | (long)Z1_Accessory91Flag << 1 | (long)(Z1_Accessory92Flag != 0) << 2 |
             (long)Z1_Accessory95Flag << 3 | (long)Z1_Accessory96Flag << 5 | (long)Z1_Accessory97Flag << 6 |
             (long)Z1_Accessory98Flag << 7 | (long)Z1_Accessory99Flag << 8;
    case PG_90:
      return Z1_TrackType | Z1_PowerPending << 3 | Z1_TrackPowerOn << 4;
    case PG_92:
      return (long)Z1_Accessory92Number | (long)Z1_Accessory92Flag << 12 | (long)Z1_TrackPowerOn << 14 | (long)Menu_Sent << 16;
    case PG_95:
    case PG_95_PLAY:
      return (long)Z1_Accessory95Flag << 8 | (Z1_Accessory95Flag == 3 ? Z1_PlaybackNumber : Z1_RecordNumber);
  }
  return 0;
}

void Menu_Draw_Page() {
  tft.setRotation(1);
  tft.setFont();
  tft.setTextWrap(false);
  tft.fillRect(0, MAP_STATUS_Y + 20, TOUCH_W, TOUCH_H - (MAP_STATUS_Y + 20), MN_BG);
  Menu_Btns = 0;
  switch (Menu_Page) {
    case PG_MAIN:         Menu_Page_Main(); break;
    case PG_90:           Menu_Page_90(); break;
    case PG_91:           Menu_Page_91(); break;
    case PG_91_WARN:      Menu_Page_91_Warn(); break;
    case PG_91_CLEAR:     Menu_Page_91_Clear(); break;
    case PG_91_PLACE:     Menu_Page_91_Place(); break;
    case PG_91_NUMBER:    Menu_Page_91_Number(); break;
    case PG_91_CONFIRM:   Menu_Page_91_Confirm(); break;
    case PG_91_PROG:      Menu_Page_91_Prog(); break;
    case PG_91_DONE:      Menu_Page_91_Done(); break;
    case PG_91_TYPE:      Menu_Page_91_Type(); break;
    case PG_91_SETUP:     Menu_Page_91_Setup(); break;
    case PG_91_BLOCK:     Menu_Page_91_Block(); break;
    case PG_92:           Menu_Page_92(); break;
    case PG_95:           Menu_Page_95(); break;
    case PG_95_PLAY:      Menu_Page_95_Play(); break;
    case PG_95_REC:       Menu_Page_95_Rec(); break;
    case PG_95_CONFIRM:   Menu_Page_95_Confirm(); break;
    case PG_95_RECORDING: Menu_Page_95_Recording(); break;
    case PG_96:           Menu_Page_96(); break;
    case PG_97:           Menu_Page_97(); break;
    case PG_98:           Menu_Page_98(); break;
    case PG_99:           Menu_Page_99(); break;
    case PG_99_FN:        Menu_Page_99_Fn(); break;
  }
  Menu_Watch = Menu_Signature();
  Draw_Screen_Arrows();                      // ends in portrait (rotation 0)
}

// the whole menu screen: background, status strip, arrows and the current page
void Menu_Show_Screen() {
  tft.setRotation(1);
  tft.fillScreen(MN_BG);
  showMapStatus();
  Menu_Draw_Page();
}

void Menu_Go(byte page) {
  Menu_Page = page;
  if (Z1_Screen == SCREEN_MENU) Menu_Draw_Page();
}

// Called from loop(): follow changes made with the Zero 1 keys (or anything else)
void Menu_Cycle() {

  // a function ended with its 9x<- key: its page has nothing left to do
  if ((Menu_Page == PG_90 && !Z1_Accessory90Flag) || (Menu_Page == PG_92 && !Z1_Accessory92Flag) ||
      (Menu_Page_Is_91() && !Z1_Accessory91Flag) ||
      (Menu_Page == PG_96 && !Z1_Accessory96Flag) || (Menu_Page == PG_97 && !Z1_Accessory97Flag) ||
      (Menu_Page == PG_98 && !Z1_Accessory98Flag) ||
      ((Menu_Page == PG_99 || Menu_Page == PG_99_FN) && !Z1_Accessory99Flag)) {
    if (Z1_ProgPower || Menu_RetryOff) Prog_Abort();   // Acc 91 ended part way through programming
    Menu_Sel = 0;
    Menu_Go(PG_MAIN);
    return;
  }
  if (Menu_Page == PG_95_RECORDING && Z1_Accessory95Flag != 2) {   // recording ended with 95<-
    Menu_Go(PG_95);
    return;
  }

  // Acc 91: (R8216: charge on TRACK B) -> program track power on -> the writes -> 15 s wait
  // -> program track power off. One step each time it is due (see PS_...)
  if (Menu_Page == PG_91_PROG) {
    if (Menu_ProgStep != PS_WAIT && (long)(millis() - Menu_ProgNext) >= 0) {
      bool r8216 = Menu_ModType == MOD_R8216 && Menu_ProgMode == 2;
      switch (Menu_ProgStep) {
        case PS_B_MAIN:                      // R8216: TRACK B to Main (DCC-EX turns it off) ...
          Prog_B_Main();
          Menu_ProgNext = millis() + MENU_R8216_STEP; Menu_ProgStep = PS_B_POWER; break;
        case PS_B_POWER:                     // ... and on: charge (first) or recharge (between writes)
          DCC_Out.println("<1 B>");
          Menu_ProgNext = millis() + (Menu_PWi == 0 ? MENU_R8216_CHARGE : MENU_R8216_RECHARGE);
          Menu_ProgStep = PS_B_PROG; break;
        case PS_B_PROG:                      // charged: TRACK B back to Program (DCC-EX turns it off)
          DCC_Out.println("<= B PROG>");
          Menu_ProgNext = millis() + MENU_R8216_STEP; Menu_ProgStep = PS_PROG_ON; break;
        case PS_PROG_ON:                     // and straight back on as the program track
          Prog_Track_Power(true);
          Menu_ProgNext = millis() + MENU_R8216_STEP; Menu_ProgStep = PS_WRITE; break;
        case PS_WRITE:
          Prog_Write(Menu_PWi++);
          if (Menu_PWi >= Menu_PWn) { Menu_ProgStep = PS_WAIT; Menu_ProgEnd = millis() + MENU_PROG_WAIT; }
          else if (r8216) { Menu_ProgNext = millis() + MENU_R8216_WRITE; Menu_ProgStep = PS_B_MAIN; }   // recharge
          else Menu_ProgNext = millis() + MENU_PROG_GAP;
          break;
      }
      if (Z1_Screen == SCREEN_MENU && Menu_91_Phase() != Menu_ProgChgShown) Menu_Draw_Page();   // charging <-> programming
    }
    if (Menu_ProgStep == PS_WAIT && (long)(millis() - Menu_ProgEnd) >= 0) {
      Prog_Retry_Restore();
      Prog_Track_Power(false);
      Menu_Go(PG_91_DONE);
      return;
    }
    if (Z1_Screen == SCREEN_MENU) {
      long left = (long)(Menu_ProgEnd - millis());
      if ((left + 999) / 1000 != Menu_ProgShown) Menu_91_Countdown();
    }
    return;
  }

  if (Z1_Screen != SCREEN_MENU) return;
  long sig = Menu_Signature();
  if (sig == Menu_Watch) return;
  Menu_Watch = sig;
  if (Menu_Page == PG_92) Menu_92_Info();    // just the number and the last command
  else Menu_Draw_Page();

}

// ================= Touch =================

void Menu_Touch(int16_t x, int16_t y) {

  // the loco strip at the bottom of the Acc 96 / 98 / 99 pages
  if (Menu_Has_Strip()) {
    byte k;
    int16_t px;
    if (Menu_Strip_Hit(x, y, k, px)) {
      if (Menu_Page == PG_96) {               // edit this slot's CAB ID
        Menu_Sel = k;
        Menu_Entry = 0;
        Menu_Draw_Page();
      }
      else if (Menu_Page == PG_98) {          // light bulb = F0 (as Acc 98 <-), music note = whistle (as 98 ->)
        if (px < 24)      Loco_Function(k, 0);
        else if (px < 48) Whistle(k);
      }
      else {                                  // Acc 99: this loco's functions (as choosing it with the key)
        Menu_FnSlot = k;
        LocoNumber = k;
        Menu_Go(PG_99_FN);
      }
      return;
    }
  }

  for (byte i = 0; i < Menu_Btns; i++) {
    MenuBtn &b = Menu_Btn[i];
    if (x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h) {
      Menu_Press(b.id);
      return;
    }
  }

}

void Menu_Press(byte id) {

  switch (Menu_Page) {

    case PG_MAIN: {
      byte n = 90 + (id - ID_MAIN);
      switch (n) {
        case 90: if (!Z1_Accessory90Flag) Acc_Start(90); Menu_Go(PG_90); break;
        case 91: if (!Z1_Accessory91Flag) Acc_Start(91); Menu_Go(PG_91); break;
        case 92: if (!Z1_Accessory92Flag) Acc_Start(92); Menu_Go(PG_92); break;
        case 93: Show_Screen(SCREEN_MAP); break;
        case 95:
          if (Z1_Accessory95Flag == 0) {    // as 95->: wait for a recording to be chosen
            Z1_Accessory95Flag = 1;
            digitalWrite(Z1_Relay_Pin, HIGH);
          }
          Menu_Go(PG_95);
          break;
        case 96: if (!Z1_Accessory96Flag) Acc_Start(96); Menu_Sel = 0; Menu_Go(PG_96); break;
        case 97: if (!Z1_Accessory97Flag) Acc_Start(97); Menu_Go(PG_97); break;
        case 98: if (!Z1_Accessory98Flag) Acc_Start(98); Menu_Go(PG_98); break;
        case 99: if (!Z1_Accessory99Flag) Acc_Start(99); Menu_Go(PG_99); break;
      }
      break;
    }

    case PG_90:
      if (id >= ID_OPT + 1 && id <= ID_OPT + 4) {
        AccessoryNo = id - ID_OPT;           // as keying 1-4 inside Acc 90
        Accessory90();
      }
      else if (id == ID_EXIT) { Acc_End(90); Menu_Go(PG_MAIN); }
      break;

    // ---- Acc 91 ----
    case PG_91:
      if (id == ID_OPT + 1) { Menu_ProgMode = 1; Menu_ModType = MOD_OTHER; Menu_Go(PG_91_WARN); }
      else if (id == ID_OPT + 2) { Menu_ProgMode = 2; Menu_Go(PG_91_TYPE); }   // ask which module first
      else if (id == ID_EXIT) { Acc_End(91); Menu_Go(PG_MAIN); }
      break;

    case PG_91_TYPE:
      if (id >= ID_OPT + 1 && id <= ID_OPT + 3) {
        Menu_ModType = id == ID_OPT + 1 ? MOD_R8216 : id == ID_OPT + 2 ? MOD_R8247 : MOD_OTHER;
        for (byte c = 0; c < 4; c++) Menu_Chan[c] = CH_LEAVE;   // R8247 setup starts on "Leave"
        Menu_Go(PG_91_WARN);
      }
      else if (id == ID_BACK) Menu_Go(PG_91);
      break;

    case PG_91_BLOCK:
      if (id == ID_OK) Menu_Go(Menu_ModType == MOD_R8247 ? PG_91_SETUP : PG_91_CONFIRM);
      else if (id == ID_BACK) { Menu_RangeWarn = false; Menu_Go(PG_91_NUMBER); }
      break;

    case PG_91_SETUP:
      if (id >= ID_LIST && id < ID_LIST + 16) {
        Menu_Chan[(id - ID_LIST) / 4] = (id - ID_LIST) % 4;
        Menu_Draw_Page();
      }
      else if (id == ID_DONE) Menu_Go(PG_91_CONFIRM);
      else if (id == ID_BACK) Menu_Go(PG_91_BLOCK);
      break;

    case PG_91_WARN:
      if (id == ID_YES) {
        Z1_PowerPending = false;
        Set_Track_Mode(2);                   // power off, A: Main, B: Program
        Menu_Go(PG_91_CLEAR);
      }
      else if (id == ID_NO) Menu_Go(PG_91);
      break;

    case PG_91_CLEAR:
      if (id == ID_DONE) Menu_Go(PG_91_PLACE);
      else if (id == ID_BACK) Menu_Go(PG_91);
      break;

    case PG_91_PLACE:
      if (id == ID_DONE) { Menu_Entry = 0; Menu_RangeWarn = false; Menu_Go(PG_91_NUMBER); }
      else if (id == ID_BACK) Menu_Go(PG_91);
      break;

    case PG_91_NUMBER:
      if (id <= 9 || id == ID_X) Menu_Keypad_Entry(id, 9999);
      else if (id == ID_SELECT) {
        unsigned int limit = Menu_ProgMode == 1 ? 99 : Menu_ModType == MOD_R8216 ? 252 : 2040;   // R8216: CV1 only
        if (Menu_Entry >= 1 && Menu_Entry <= limit)
          Menu_Go(Menu_ProgMode == 2 ? PG_91_BLOCK : PG_91_CONFIRM);   // modules: show the block of 4 first
        else { Menu_RangeWarn = true; Menu_Draw_Page(); }
      }
      else if (id == ID_BACK) Menu_Go(PG_91);
      break;

    case PG_91_CONFIRM:
      if (id == ID_YES) {
        Menu_91_Build();
        Menu_ProgEnd = millis() + Menu_91_Total();   // for the countdown; reset from the last write
        if (Menu_ProgMode == 2 && Menu_ModType == MOD_R8216) {   // R8216: TRACK B to Main and on, charge, ...
          Prog_B_Main();
          Menu_ProgNext = millis() + MENU_R8216_STEP;
          Menu_ProgStep = PS_B_POWER;
        }
        else {                               // program track on; the first write follows MENU_PROG_POWER later
          Prog_Track_Power(true);
          Menu_ProgNext = millis() + MENU_PROG_POWER;
          Menu_ProgStep = PS_WRITE;
        }
        Menu_Go(PG_91_PROG);
      }
      else if (id == ID_NO) Menu_Go(PG_91);
      break;

    case PG_91_DONE:
      if (id == ID_OK) Menu_Go(PG_91);
      break;

    // ---- Acc 92 ----
    case PG_92:
      if (id <= 9) {
        if (Z1_Accessory92Flag == 2) { Z1_Accessory92Number = 0; Z1_Accessory92Flag = 1; }   // new number after a send
        unsigned long next = (unsigned long)Z1_Accessory92Number * 10 + id;
        if (next <= 2040) Z1_Accessory92Number = next;
      }
      else if (id == ID_X) {
        if (Z1_Accessory92Flag == 2) { Z1_Accessory92Number = 0; Z1_Accessory92Flag = 1; }  // after a send: clear it all
        else Z1_Accessory92Number /= 10;                                                     // otherwise: last digit
      }
      else if ((id == ID_LEFT || id == ID_RIGHT) && Z1_Accessory92Number > 0) {
        Z1_Accessory92Flag = 2;              // command mode, as -> on the Zero 1
        Menu_LastAcc = Z1_Accessory92Number;
        Menu_LastState = id == ID_LEFT ? 1 : 0;
        Send_Accessory(Menu_LastAcc, Menu_LastState);
        Menu_Sent++;
      }
      else if (id == ID_EXIT) { Acc_End(92); Menu_LastAcc = 0; Menu_Go(PG_MAIN); }
      break;

    // ---- Acc 95 ----
    case PG_95:
      if (id == ID_PLAY) Menu_Go(PG_95_PLAY);
      else if (id == ID_REC) Menu_Go(Z1_Accessory95Flag == 2 ? PG_95_RECORDING : PG_95_REC);
      else if (id == ID_EXIT) {
        if (Z1_Accessory95Flag == 1) Accessory95_Cancel();   // recording / playback carry on if running
        Menu_Go(PG_MAIN);
      }
      break;

    case PG_95_PLAY:
      if (id >= ID_LIST && id < ID_LIST + Menu_RecCount) {
        if (Z1_Accessory95Flag == 2) Recording_Stop();
        if (Z1_Accessory95Flag == 3) Playback_Stop();
        Start_Playback(Menu_RecList[id - ID_LIST]);
      }
      else if (id == ID_STOPPLAY) { if (Z1_Accessory95Flag == 3) Playback_Stop(); }
      else if (id == ID_BACK) Menu_Go(PG_95);
      break;

    case PG_95_REC:
      if (id >= ID_LIST && id < ID_LIST + 10) { Menu_Sel = id - ID_LIST + 1; Menu_Go(PG_95_CONFIRM); }
      else if (id == ID_BACK) Menu_Go(PG_95);
      break;

    case PG_95_CONFIRM:
      if (id == ID_YES) {
        if (Z1_Accessory95Flag == 3) Playback_Stop();
        if (Z1_Accessory95Flag == 2) Recording_Stop();
        Start_Recording(Menu_Sel);
        Menu_Go(Z1_Accessory95Flag == 2 ? PG_95_RECORDING : PG_95_REC);
      }
      else if (id == ID_NO) Menu_Go(PG_95_REC);
      break;

    case PG_95_RECORDING:
      if (id == ID_OK) Menu_Go(PG_MAIN);
      else if (id == ID_ENDREC) { if (Z1_Accessory95Flag == 2) Recording_Stop(); Menu_Go(PG_95); }
      break;

    // ---- Acc 96 - 99 ----
    case PG_96:
      if ((id <= 9 || id == ID_X) && Menu_Sel) Menu_Keypad_Entry(id, 9999);
      else if (id == ID_SELECT && Menu_Sel && Menu_Entry > 0) {
        byte k = Menu_Sel;
        Menu_Sel = 0;
        Assign_Slot(k, Menu_Entry);          // as Acc 96: the old loco keeps running, roster saved
        Menu_Draw_Page();                    // number pad goes away
      }
      else if (id == ID_EXIT) { Acc_End(96); Menu_Sel = 0; Menu_Go(PG_MAIN); }
      break;

    case PG_97:
      if (id >= ID_LIST && id < ID_LIST + 10) {
        byte n = id - ID_LIST + 1;
        if (SD.exists(Menu_Roster_Name(n))) {
          AccessoryNo = n;                   // as keying n inside Acc 97: load it (ends Acc 97)
          Accessory97();
        }
        else {                               // empty: start it with the locos in use now
          Z1_Accessory97Bank = n;
          Save_Roster(n);
          Acc_End(97);
        }
        Menu_Go(PG_MAIN);
      }
      else if (id == ID_EXIT) { Acc_End(97); Menu_Go(PG_MAIN); }
      break;

    case PG_98:
      if (id == ID_EXIT) { Acc_End(98); Menu_Go(PG_MAIN); }
      break;

    case PG_99:
      if (id == ID_EXIT) { Acc_End(99); Menu_Go(PG_MAIN); }
      break;

    case PG_99_FN:
      if (id >= ID_FN && id < ID_FN + 32) Loco_Function(Menu_FnSlot, id - ID_FN);
      else if (id == ID_BACK) { LocoNumber = 0; Menu_Go(PG_99); }
      break;
  }

}
