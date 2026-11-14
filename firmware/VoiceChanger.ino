/*
  ============================================================
   ESP32 VOICE CHANGER — FINAL FIRMWARE  (rev 4)
  ============================================================
  Hardware : ESP32 DevKit V1, INMP441 I2S mic, MAX98357A I2S amp + speaker,
             SPI TFT (ST7789 or ST7735, picked on the screen itself), 4 buttons

  How it works:
    BLUE  (D27) : HOLD to record. It records ONLY while you hold it.
                  Let go = recording stops. It also stops when RAM is full.
    GREY  (D26) : play the recording with HIGH pitch  (chipmunk x1.6)
    WHITE (D22) : play the recording with LOW pitch   (monster  x0.65)
    BLACK (D23) : play the recording as ROBOT         (ring modulation 50 Hz)
    Press another effect button while it plays -> restarts with that effect.
    Press BLUE while it plays -> playback stops and a new recording starts.

  The mic is used ONLY while BLUE is held. The rest of the time its data is ignored.
  The screen shows: state, a "tape" of the recording (bars), time and frequency.

  FIRST START = SCREEN SETUP (because screens with the same pins differ):
    GREY  : try the next screen type
    WHITE : rotate
    BLUE  : save and start
    Pick the one where the white frame touches all 4 edges of the glass.
    To open this setup again later: hold GREY while you power on.

  SPEAKER TEST: hold WHITE while you power on. You must hear a clean 440 Hz tone.
    Clean tone  -> amp, speaker and power are good.
    Dirty tone  -> power or speaker wiring problem (see OUT_VOLUME below).

  Wiring:
    TFT  SCL->D17  SDA->D16  CS->D15  DC(DS)->D2  RST->D4  BL->3V3  VCC->VIN  GND->GND
    AMP  LRC->D21  BCLK->D18  DIN->D19  SD->D5  VIN->VIN(5V)  GND->GND  speaker on + / -
    MIC  SD->D34   WS->D33    SCK->D32  L/R->GND  VDD->3V3 (NOT 5V)  GND->GND
    BTN  one diagonal leg to the pin, the other diagonal leg to GND
         Blue D27, Grey D26, White D22, Black D23

  Arduino IDE:
    Board package : "esp32 by Espressif Systems" 3.x
    Libraries     : "Adafruit ST7735 and ST7789 Library" + "Adafruit GFX Library"
    Board         : ESP32 Dev Module
*/

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_ST7789.h>
#include <driver/i2s_std.h>
#include <esp_heap_caps.h>
#include <math.h>

// ======================= SETTINGS =======================
#define FORCE_PROFILE   -1        // -1 = choose on screen. 0..6 = force a line of PROFILES[] below
#define FORCE_ROTATION  0         // used only with FORCE_PROFILE (0..3)

#define SAMPLE_RATE     16000
#define MAX_SECONDS     8         // upper limit. Real limit = free RAM (about 3 s without PSRAM)

#define PITCH_HIGH      1.6f      // GREY
#define PITCH_LOW       0.65f     // WHITE
#define ROBOT_HZ        50.0f     // BLACK
#define OUT_VOLUME      0.40f     // 0.0 ... 1.0. Too high = the amp clips and USB power sags = dirty sound
#define NOISE_GATE      1         // 1 = make the quiet gaps between words quieter (less hiss)
#define TONE_CUT_HZ     3800.0f   // playback treble cut (less hiss). 7000 = almost off
#define TFT_SPI_HZ      27000000
// ========================================================

// ---------- Pins ----------
#define PIN_TFT_SCK   17
#define PIN_TFT_MOSI  16
#define PIN_TFT_CS    15
#define PIN_TFT_DC    2
#define PIN_TFT_RST   4

#define PIN_AMP_LRC   21
#define PIN_AMP_BCLK  18
#define PIN_AMP_DIN   19
#define PIN_AMP_SD    5

#define PIN_MIC_SD    34
#define PIN_MIC_WS    33
#define PIN_MIC_SCK   32

#define PIN_BTN_REC   27   // Blue
#define PIN_BTN_HIGH  26   // Grey
#define PIN_BTN_LOW   22   // White
#define PIN_BTN_ROBOT 23   // Black

#define CHUNK     256      // samples per block (16 ms)
#define DMA_BUFS  8        // 8 x 16 ms = 128 ms of I2S buffering
#define HIST      512      // samples kept for the frequency measurement

#ifndef I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG
#define I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG
#endif

// ======================= TYPES =======================
struct Profile {
  const char *name;
  uint8_t  driver;     // 89 = ST7789, 35 = ST7735
  uint16_t w, h;       // native size (ST7789)
  uint8_t  tab;        // ST7735 tab type
};

struct Button {
  uint8_t pin;
  bool stable;         // true = pressed (after debounce)
  bool raw;
  uint32_t t;
};

struct PitchShifter {
  float ratio, d, win;
};

enum Effect { FX_HIGH = 0, FX_LOW, FX_ROBOT };
enum Action { ACT_NONE, ACT_RECORD, ACT_HIGH, ACT_LOW, ACT_ROBOT };

// ======================= GLOBALS =======================
const Profile PROFILES[] = {
  { "ST7789 240x320", 89, 240, 320, 0 },               // 0  (2.0" / 2.4" / 2.8")
  { "ST7789 240x240", 89, 240, 240, 0 },               // 1  (1.3" / 1.54")
  { "ST7789 240x280", 89, 240, 280, 0 },               // 2  (1.69")
  { "ST7789 170x320", 89, 170, 320, 0 },               // 3  (1.9")
  { "ST7789 135x240", 89, 135, 240, 0 },               // 4  (1.14")
  { "ST7735 128x160 A", 35, 128, 160, INITR_BLACKTAB },// 5  (1.8")
  { "ST7735 128x160 B", 35, 128, 160, INITR_GREENTAB },// 6  (1.8", other batch)
};
const uint8_t NUM_PROFILES = sizeof(PROFILES) / sizeof(PROFILES[0]);

Adafruit_ST77xx *tft = nullptr;
Preferences prefs;

i2s_chan_handle_t rxChan = NULL;   // mic  = I2S port 0
i2s_chan_handle_t txChan = NULL;   // amp  = I2S port 1

int16_t *recBuf = nullptr;
uint32_t recMax = 0;               // capacity (samples)
uint32_t recLen = 0;               // length of last recording (samples)

int32_t micRaw[CHUNK * 2];         // stereo frames from the mic (L,R)
int16_t micBlock[CHUNK];           // cleaned mono block
int16_t outBlock[CHUNK];           // mono copy of what is played (for the screen)
int16_t outFrame[CHUNK * 2];       // stereo L,R to the amp
int16_t hist[HIST];                // last 512 samples (frequency measurement)
float   acBuf[HIST / 2];

bool  micRight = false;            // which I2S slot the mic really talks on (auto-detected)
bool  micLock  = true;             // slot is chosen once at start-up
float micEL = 0, micER = 0;        // activity of each slot
float dcX = 0, dcY = 0;            // DC blocker state

const char *fxName[] = { "CHIPMUNK", "MONSTER", "ROBOT" };
const char *fxHint[] = { "Pitch up x1.6", "Pitch down x0.65", "Ring mod 50 Hz" };
const uint16_t fxCol[] = { 0xEE2D, 0xF449, 0x7E9C };   // warm yellow, orange, cyan

Button bRec   = { PIN_BTN_REC,   false, false, 0 };
Button bHigh  = { PIN_BTN_HIGH,  false, false, 0 };
Button bLow   = { PIN_BTN_LOW,   false, false, 0 };
Button bRobot = { PIN_BTN_ROBOT, false, false, 0 };

// ---------- colours (RGB565) — same family as the project page ----------
#define C_BG       0x08C6   // deep navy
#define C_PANEL    0x1107   // panel navy
#define C_LINE     0x2A2D   // quiet lines
#define C_UPCOMING 0x4B72   // tape bars not played yet
#define C_SILK     0xF79C   // off-white text
#define C_DIM      0xA5B9   // secondary text
#define C_GOLD     0xDD28
#define C_READY    0x9F16   // mint
#define C_REC      0xFBCD   // coral red
#define C_WARN     0xF449   // orange

// ---------- screen layout (computed from the real screen size) ----------
int scrW, scrH, ts;                // ts = text scale (1 small screens, 2 big screens)
int topH, yBig, yHint, yTape, tapeH, yField, fieldH, fieldW, yChips, chipH, slotW;
int nBars, barPitch, barW, tapeX0, tapeMid, tapeHalf;
uint8_t env[100];                  // bar heights of the saved recording
char fieldTxt[2][16] = { "", "" };
bool micOk = true;

// ======================= BUTTONS =======================
// Returns true once, at the moment the button becomes pressed.
bool updateButton(Button &b) {
  bool r = (digitalRead(b.pin) == LOW);
  uint32_t now = millis();
  if (r != b.raw) { b.raw = r; b.t = now; }
  if ((now - b.t) > 25 && r != b.stable) {
    b.stable = r;
    return r;
  }
  return false;
}

Action pollButtons() {
  bool r = updateButton(bRec);
  bool h = updateButton(bHigh);
  bool l = updateButton(bLow);
  bool o = updateButton(bRobot);
  if (r) { Serial.println("BTN BLUE  (D27) pressed"); return ACT_RECORD; }
  if (h) { Serial.println("BTN GREY  (D26) pressed"); return ACT_HIGH; }
  if (l) { Serial.println("BTN WHITE (D22) pressed"); return ACT_LOW; }
  if (o) { Serial.println("BTN BLACK (D23) pressed"); return ACT_ROBOT; }
  return ACT_NONE;
}

// ======================= SCREEN =======================
void startDisplay(uint8_t p, uint8_t rot) {
  const Profile &pr = PROFILES[p];
  if (pr.driver == 89) {
    Adafruit_ST7789 *d = new Adafruit_ST7789(&SPI, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
    d->init(pr.w, pr.h);
    tft = d;
  } else {
    Adafruit_ST7735 *d = new Adafruit_ST7735(&SPI, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
    d->initR(pr.tab);
    tft = d;
  }
  tft->setSPISpeed(TFT_SPI_HZ);
  tft->setRotation(rot);
  tft->setTextWrap(false);
  tft->fillScreen(ST77XX_BLACK);
  Serial.printf("Screen: %s, rotation %d, %d x %d\n", pr.name, rot, tft->width(), tft->height());
}

void textAt(int x, int y, uint8_t size, uint16_t col, const char *s) {
  tft->setTextSize(size);
  tft->setTextColor(col);
  tft->setCursor(x, y);
  tft->print(s);
}

void computeLayout() {
  scrW = tft->width();
  scrH = tft->height();
  ts = (scrW >= 200 && scrH >= 200) ? 2 : 1;
  topH   = 8 * ts + 4;
  yBig   = topH + 6;
  yHint  = yBig + 16 * ts + 3;
  yTape  = yHint + 8 * ts + 5;
  chipH  = 8 * ts + 6;
  yChips = scrH - chipH - 3;
  fieldH = 8 * ts + 6;
  yField = yChips - fieldH - 5;
  tapeH  = yField - 5 - yTape;
  fieldW = (scrW - 12) / 2;
  slotW  = (scrW - 8) / 4;
  barW     = ts + 1;
  barPitch = barW + 1;
  nBars = (scrW - 16) / barPitch;
  if (nBars > 100) nBars = 100;
  tapeX0   = 4 + (scrW - 8 - (nBars * barPitch - 1)) / 2;
  tapeMid  = yTape + tapeH / 2;
  tapeHalf = tapeH / 2 - 4;
  if (tapeHalf < 2) tapeHalf = 2;
}

// ---------- the four buttons, drawn like the real ones ----------
void drawChips() {
  const char *lab[4]    = { "REC", "HI", "LO", "BOT" };
  const uint16_t fill[4] = { 0x2B7F, 0x8C71, 0xFFFF, 0x0000 };
  const uint16_t ink[4]  = { 0xFFFF, 0x0000, 0x0000, 0xFFFF };
  for (int i = 0; i < 4; i++) {
    int x = 4 + i * slotW + 2, w = slotW - 6;
    tft->fillRoundRect(x, yChips, w, chipH, chipH / 3, fill[i]);
    if (i == 3) tft->drawRoundRect(x, yChips, w, chipH, chipH / 3, C_DIM);
    int tw = strlen(lab[i]) * 6 * ts;
    textAt(x + (w - tw) / 2 + ts / 2, yChips + 3, ts, ink[i], lab[i]);
  }
}

// ring around the button that is in use (-1 = none)
void chipRing(int active, uint16_t col) {
  for (int i = 0; i < 4; i++) {
    int x = 4 + i * slotW;
    tft->drawRoundRect(x, yChips - 2, slotW - 2, chipH + 4, chipH / 3 + 2, i == active ? col : C_BG);
  }
}

void setField(int i, const char *s, uint16_t col) {
  if (strcmp(s, fieldTxt[i]) == 0) return;       // redraw only when the text changes
  strncpy(fieldTxt[i], s, sizeof(fieldTxt[i]) - 1);
  fieldTxt[i][sizeof(fieldTxt[i]) - 1] = 0;
  int x = 4 + i * (fieldW + 4);
  tft->fillRect(x + 3, yField + 2, fieldW - 6, fieldH - 4, C_PANEL);
  textAt(x + 5, yField + 3, ts, col, s);
}

void setHint(const char *s, uint16_t col) {
  tft->fillRect(0, yHint, scrW, 8 * ts, C_BG);
  textAt(4, yHint, ts, col, s);
}

// state mark at the right of the big word: 1 = ring, 2 = dot, 3 = play triangle
void drawGlyph(int type, uint16_t col, int textW) {
  int g = 14 * ts, gx = scrW - 4 - g, gy = yBig + ts;
  if (4 + textW + 6 > gx) return;                // no room next to a long word
  tft->fillRect(gx, gy, g, g, C_BG);
  int cx = gx + g / 2, cy = gy + g / 2;
  if (type == 1) { tft->drawCircle(cx, cy, 6 * ts, col); tft->drawCircle(cx, cy, 6 * ts - 1, col); tft->fillCircle(cx, cy, 2 * ts, col); }
  if (type == 2) tft->fillCircle(cx, cy, 6 * ts, col);
  if (type == 3) tft->fillTriangle(gx + 2 * ts, gy + ts, gx + 2 * ts, gy + 13 * ts, gx + 13 * ts, cy, col);
}

void setState(const char *big, uint16_t col, const char *hint, int glyph, int activeChip) {
  tft->fillRect(0, topH, scrW, 2, col);                    // accent line under the title
  tft->fillRect(0, yBig, scrW, 16 * ts, C_BG);
  textAt(4, yBig, 2 * ts, col, big);
  drawGlyph(glyph, col, strlen(big) * 12 * ts);
  setHint(hint, C_SILK);
  chipRing(activeChip, col);
  Serial.printf("[%s] %s\n", big, hint);
}

// ---------- the tape: one bar per slice of sound ----------
int barHeight(int peak, float ref) {
  float v = peak / ref;
  if (v > 1) v = 1;
  int h = (int)(sqrtf(v) * tapeHalf);
  return h < 1 ? 1 : h;
}

void tapeBar(int i, int h, uint16_t col, bool erase) {
  int x = tapeX0 + i * barPitch;
  if (erase) tft->fillRect(x, tapeMid - tapeHalf, barW, 2 * tapeHalf, C_PANEL);
  tft->fillRect(x, tapeMid - h, barW, 2 * h, col);
}

void tapeClear() {
  tft->fillRoundRect(4, yTape, scrW - 8, tapeH, 3 * ts, C_PANEL);
  for (int i = 0; i < nBars; i++) tapeBar(i, 1, C_LINE, false);   // empty slots
}

// measure the saved recording once: one bar height per slice
void buildOverview() {
  for (int i = 0; i < nBars; i++) {
    uint32_t a = (uint32_t)((uint64_t)i * recLen / nBars);
    uint32_t b = (uint32_t)((uint64_t)(i + 1) * recLen / nBars);
    int pk = 0;
    for (uint32_t k = a; k < b; k++) {
      int v = abs((int)recBuf[k]);
      if (v > pk) pk = v;
    }
    env[i] = (uint8_t)barHeight(pk, 30000.0f);
  }
}

void drawOverview(uint16_t col) {
  for (int i = 0; i < nBars; i++) tapeBar(i, env[i], col, true);
}

void drawFrame() {
  tft->fillScreen(C_BG);
  textAt(4, 2, ts, C_DIM, "VOICE CHANGER");
  for (int i = 0; i < 2; i++) {
    tft->fillRoundRect(4 + i * (fieldW + 4), yField, fieldW, fieldH, 2 * ts, C_PANEL);
    fieldTxt[i][0] = 0;
  }
  tapeClear();
  drawChips();
}

void showReady() {
  char s[16];
  if (!micOk)      setState("READY", C_READY, "No mic signal!", 1, -1);
  else if (recLen) setState("READY", C_READY, "Press GREY/WHT/BLK", 1, -1);
  else             setState("READY", C_READY, "Hold BLUE to record", 1, -1);
  if (recLen) {
    drawOverview(C_SILK);
    snprintf(s, sizeof(s), "LAST %.1fs", recLen / (float)SAMPLE_RATE);
  } else {
    tapeClear();
    snprintf(s, sizeof(s), "LAST --");
  }
  setField(0, s, C_SILK);
  snprintf(s, sizeof(s), "MAX %.1fs", recMax / (float)SAMPLE_RATE);
  setField(1, s, C_DIM);
}

void fatal(const char *big, const char *small) {
  setState(big, C_REC, small, 0, -1);
  while (true) delay(1000);
}

// ======================= SCREEN SETUP MODE =======================
void drawSetupScreen(uint8_t p, uint8_t rot) {
  int w = tft->width(), h = tft->height();
  tft->fillScreen(ST77XX_BLACK);
  tft->drawRect(0, 0, w, h, ST77XX_WHITE);
  tft->drawRect(1, 1, w - 2, h - 2, ST77XX_WHITE);
  tft->fillRect(3, 3, 8, 8, ST77XX_RED);
  tft->fillRect(w - 11, 3, 8, 8, ST77XX_GREEN);
  tft->fillRect(3, h - 11, 8, 8, ST77XX_BLUE);
  tft->fillRect(w - 11, h - 11, 8, 8, ST77XX_YELLOW);
  char r[20];
  snprintf(r, sizeof(r), "Rotation: %d", rot);
  int y = 14;
  textAt(6, y, 1, ST77XX_YELLOW, "SCREEN SETUP");      y += 10;
  textAt(6, y, 1, ST77XX_CYAN,   PROFILES[p].name);    y += 10;
  textAt(6, y, 1, ST77XX_CYAN,   r);                   y += 10;
  textAt(6, y, 1, 0xBDF7,        "GREY : next type");  y += 10;
  textAt(6, y, 1, ST77XX_WHITE,  "WHITE: rotate");     y += 10;
  textAt(6, y, 1, 0x5D1F,        "BLUE : save");       y += 10;
  textAt(6, y, 1, ST77XX_GREEN,  "Good = white frame");y += 10;
  textAt(6, y, 1, ST77XX_GREEN,  "on all 4 edges and");y += 10;
  textAt(6, y, 1, ST77XX_GREEN,  "4 colour corners");
}

void drawSetupButtons() {
  // four squares: they fill while a button is pressed (button wiring check)
  const Button *b[4] = { &bRec, &bHigh, &bLow, &bRobot };
  const uint16_t c[4] = { 0x5D1F, 0xBDF7, 0xFFFF, 0x7BEF };
  for (int i = 0; i < 4; i++) {
    int x = 6 + i * 14, y = 108;
    tft->fillRect(x, y, 10, 10, b[i]->stable ? c[i] : ST77XX_BLACK);
    tft->drawRect(x, y, 10, 10, c[i]);
  }
}

void screenSetup(uint8_t p, uint8_t rot) {
  Serial.println("SCREEN SETUP: GREY = next type, WHITE = rotate, BLUE = save");
  while (digitalRead(PIN_BTN_HIGH) == LOW) delay(10);   // wait until GREY is released
  startDisplay(p, rot);
  drawSetupScreen(p, rot);
  drawSetupButtons();
  uint8_t lastMask = 0;
  while (true) {
    bool b = updateButton(bRec);
    bool g = updateButton(bHigh);
    bool w = updateButton(bLow);
    updateButton(bRobot);
    if (g) { p = (p + 1) % NUM_PROFILES; startDisplay(p, rot); drawSetupScreen(p, rot); lastMask = 0xFF; }
    if (w) { rot = (rot + 1) & 3; tft->setRotation(rot); drawSetupScreen(p, rot); lastMask = 0xFF; }
    if (b) {
      prefs.putUChar("prof", p);
      prefs.putUChar("rot", rot);
      tft->fillScreen(ST77XX_BLACK);
      textAt(6, 14, 2, ST77XX_GREEN, "SAVED");
      Serial.printf("Saved screen type %d (%s), rotation %d\n", p, PROFILES[p].name, rot);
      delay(700);
      ESP.restart();
    }
    uint8_t mask = (bRec.stable ? 1 : 0) | (bHigh.stable ? 2 : 0) | (bLow.stable ? 4 : 0) | (bRobot.stable ? 8 : 0);
    if (mask != lastMask) { lastMask = mask; drawSetupButtons(); }
    delay(5);
  }
}

// ======================= AUDIO I/O =======================
void ampEnable(bool on) {
  digitalWrite(PIN_AMP_SD, on ? HIGH : LOW);   // HIGH = on (plays LEFT), LOW = shutdown
}

bool setupI2S() {
  // ---- mic: port 0, receive, 32-bit stereo (we pick the slot the mic uses) ----
  i2s_chan_config_t rc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  rc.dma_desc_num  = DMA_BUFS;
  rc.dma_frame_num = CHUNK;
  if (i2s_new_channel(&rc, NULL, &rxChan) != ESP_OK) return false;

  i2s_std_clk_config_t  rclk  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  i2s_std_slot_config_t rslot = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  i2s_std_config_t rs = {};
  rs.clk_cfg  = rclk;
  rs.slot_cfg = rslot;
  rs.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  rs.gpio_cfg.bclk = (gpio_num_t)PIN_MIC_SCK;
  rs.gpio_cfg.ws   = (gpio_num_t)PIN_MIC_WS;
  rs.gpio_cfg.dout = I2S_GPIO_UNUSED;
  rs.gpio_cfg.din  = (gpio_num_t)PIN_MIC_SD;
  if (i2s_channel_init_std_mode(rxChan, &rs) != ESP_OK) return false;
  if (i2s_channel_enable(rxChan) != ESP_OK) return false;

  // ---- amp: port 1, transmit, 16-bit stereo ----
  i2s_chan_config_t tc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  tc.dma_desc_num  = DMA_BUFS;
  tc.dma_frame_num = CHUNK;
  tc.auto_clear    = true;                     // silence when we send nothing
  if (i2s_new_channel(&tc, &txChan, NULL) != ESP_OK) return false;

  i2s_std_clk_config_t  tclk  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  i2s_std_slot_config_t tslot = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  i2s_std_config_t tcfg = {};
  tcfg.clk_cfg  = tclk;
  tcfg.slot_cfg = tslot;
  tcfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  tcfg.gpio_cfg.bclk = (gpio_num_t)PIN_AMP_BCLK;
  tcfg.gpio_cfg.ws   = (gpio_num_t)PIN_AMP_LRC;
  tcfg.gpio_cfg.dout = (gpio_num_t)PIN_AMP_DIN;
  tcfg.gpio_cfg.din  = I2S_GPIO_UNUSED;
  if (i2s_channel_init_std_mode(txChan, &tcfg) != ESP_OK) return false;
  if (i2s_channel_enable(txChan) != ESP_OK) return false;
  return true;
}

void writeOut() {
  size_t wr = 0;
  i2s_channel_write(txChan, outFrame, sizeof(outFrame), &wr, 1000);
}

void writeSilence(int blocks) {
  memset(outFrame, 0, sizeof(outFrame));
  for (int i = 0; i < blocks; i++) writeOut();
}

// Throw away old mic samples waiting in the I2S buffers.
void flushMic() {
  for (int i = 0; i < DMA_BUFS * 2; i++) {
    size_t got = 0;
    if (i2s_channel_read(rxChan, micRaw, sizeof(micRaw), &got, 0) != ESP_OK) break;
    if (got < sizeof(micRaw)) break;
  }
}

// Reads one block from the mic into micBlock[]. Returns the number of samples.
int readMicBlock(uint32_t timeoutMs) {
  size_t got = 0;
  i2s_channel_read(rxChan, micRaw, sizeof(micRaw), &got, timeoutMs);
  int n = got / (2 * sizeof(int32_t));
  if (n < 2) return 0;

  // which slot is the mic on? (L/R pin on GND = left, but ESP32 can swap them)
  float dL = 0, dR = 0;
  for (int i = 1; i < n; i++) {
    dL += fabsf((float)((micRaw[2 * i] >> 14) - (micRaw[2 * i - 2] >> 14)));
    dR += fabsf((float)((micRaw[2 * i + 1] >> 14) - (micRaw[2 * i - 1] >> 14)));
  }
  micEL = micEL * 0.9f + (dL / n) * 0.1f;
  micER = micER * 0.9f + (dR / n) * 0.1f;
  if (!micLock) {
    if (micRight && micEL > micER * 2.0f) micRight = false;
    else if (!micRight && micER > micEL * 2.0f) micRight = true;
  }

  int off = micRight ? 1 : 0;
  for (int i = 0; i < n; i++) {
    float x = (float)(micRaw[2 * i + off] >> 15);   // INMP441: 24-bit in a 32-bit slot (>>15 = headroom, loud voice does not clip)
    float y = x - dcX + 0.955f * dcY;                // high-pass ~115 Hz: removes DC, rumble and handling thumps
    dcX = x; dcY = y;
    if (y > 32767.0f)  y = 32767.0f;
    if (y < -32768.0f) y = -32768.0f;
    micBlock[i] = (int16_t)y;
  }
  return n;
}

bool micDead() { return (micRight ? micER : micEL) < 0.3f; }

void pushHist(const int16_t *b, int n) {
  if (n <= 0) return;
  if (n > HIST) n = HIST;
  memmove(hist, hist + n, (HIST - n) * sizeof(int16_t));
  memcpy(hist + HIST - n, b, n * sizeof(int16_t));
}

// Frequency (Hz) of the voice in 512 samples. 0 = too quiet / no clear pitch.
float detectPitch(const int16_t *x) {
  float e = 0;
  for (int i = 0; i < 256; i += 2) e += (float)x[i] * x[i];
  if (e / 128.0f < 150.0f * 150.0f) return 0;
  const int L0 = 27, L1 = 256;                 // 593 Hz ... 62 Hz
  float best = 0;
  for (int L = L0; L < L1; L++) {
    float s = 0;
    for (int i = 0; i < 256; i += 2) s += (float)x[i] * x[i + L];
    acBuf[L - L0] = s;
    if (s > best) best = s;
  }
  if (best < 0.3f * e) return 0;
  for (int L = L0 + 1; L < L1 - 1; L++) {      // first strong peak = real pitch
    float s = acBuf[L - L0];
    if (s > 0.85f * best && s >= acBuf[L - L0 - 1] && s >= acBuf[L - L0 + 1]) return SAMPLE_RATE / (float)L;
  }
  return 0;
}

int blockPeak(const int16_t *b, int n) {
  int p = 0;
  for (int i = 0; i < n; i++) {
    int a = abs((int)b[i]);
    if (a > p) p = a;
  }
  return p;
}

// Makes the quiet gaps between words quieter, so the hiss there is not heard.
// Works on the saved recording, in blocks of 16 ms, with smooth gain changes.
void noiseGate() {
  uint32_t blocks = recLen / CHUNK;
  if (blocks < 4) return;
  float floorRms = 1e9f, topRms = 0;
  for (uint32_t b = 0; b < blocks; b++) {          // quietest and loudest block
    float e = 0;
    for (uint32_t k = 0; k < CHUNK; k += 2) { float v = recBuf[b * CHUNK + k]; e += v * v; }
    float r = sqrtf(e / (CHUNK / 2));
    if (r < floorRms) floorRms = r;
    if (r > topRms) topRms = r;
  }
  float thr = floorRms * 2.5f;
  if (thr < topRms * 0.04f) thr = topRms * 0.04f;
  if (thr > topRms * 0.25f) return;                // noisy room: gate would chop words, skip it
  float gNow = 1.0f;
  for (uint32_t b = 0; b < blocks; b++) {
    float e = 0;
    for (uint32_t k = 0; k < CHUNK; k += 2) { float v = recBuf[b * CHUNK + k]; e += v * v; }
    float r = sqrtf(e / (CHUNK / 2));
    float target = (r > thr) ? 1.0f : 0.15f;
    float step = (target > gNow) ? 0.02f : 0.003f;  // open fast (3 ms), close slow (20 ms)
    for (uint32_t k = 0; k < CHUNK; k++) {
      gNow += (target - gNow) * step;
      recBuf[b * CHUNK + k] = (int16_t)(recBuf[b * CHUNK + k] * gNow);
    }
  }
}

// ======================= RECORD (hold BLUE) =======================
// The mic is read only inside this function, so only while BLUE is held.
void doRecord() {
  ampEnable(false);
  flushMic();                                // drop everything heard before the press
  recLen = 0;
  dcX = 0; dcY = 0;
  memset(hist, 0, sizeof(hist));
  setState("REC", C_REC, "Release BLUE = stop", 2, 0);
  tapeClear();
  setField(0, "MIC --", C_SILK);
  setField(1, "0.0s", C_DIM);

  bool full = false, dot = true;
  int slot = 0, slotPeak = 0;
  uint32_t lastT = 0, lastF = 0, lastDot = millis();
  char s[16];

  while (true) {
    updateButton(bRec);
    if (!bRec.stable) break;                 // button released -> stop recording

    int n = readMicBlock(100);
    if (n <= 0) continue;
    pushHist(micBlock, n);

    if (!full) {
      uint32_t room = recMax - recLen;
      uint32_t take = ((uint32_t)n < room) ? (uint32_t)n : room;
      memcpy(recBuf + recLen, micBlock, take * sizeof(int16_t));
      recLen += take;
      int pk = blockPeak(micBlock, (int)take);
      if (pk > slotPeak) slotPeak = pk;
      int sNow = (int)((uint64_t)recLen * nBars / recMax);
      while (slot < sNow && slot < nBars) {  // the tape fills as memory fills
        tapeBar(slot, barHeight(slotPeak, 3000.0f), C_REC, true);
        slot++;
        slotPeak = 0;
      }
      if (recLen >= recMax) {
        full = true;
        setHint("Memory full. Let go", C_WARN);
      }
    }

    // screen: one small job per block, so no sound is lost
    uint32_t now = millis();
    if (now - lastDot > 400) {
      lastDot = now;
      dot = !dot;
      drawGlyph(2, dot ? C_REC : C_BG, 3 * 12 * ts);
    } else if (now - lastT > 200) {
      lastT = now;
      snprintf(s, sizeof(s), "%.1fs", recLen / (float)SAMPLE_RATE);
      setField(1, s, C_DIM);
    } else if (now - lastF > 400) {
      lastF = now;
      float f = detectPitch(hist);
      if (f > 0) snprintf(s, sizeof(s), "MIC %dHz", (int)(f + 0.5f));
      else       snprintf(s, sizeof(s), "MIC --");
      setField(0, s, C_SILK);
    }
  }

  micOk = !micDead();

  // drop first 40 ms (button click + mic settling)
  uint32_t skip = SAMPLE_RATE * 40 / 1000;
  if (recLen > skip * 2) {
    memmove(recBuf, recBuf + skip, (recLen - skip) * sizeof(int16_t));
    recLen -= skip;
  }
  // drop last 60 ms (click of the button being released)
  uint32_t tail = SAMPLE_RATE * 60 / 1000;
  if (recLen > tail * 2) recLen -= tail;

  if (recLen < SAMPLE_RATE / 10) {           // shorter than 0.1 s
    recLen = 0;
    setState("TOO SHORT", C_WARN, "Hold BLUE longer", 0, -1);
    delay(900);
    showReady();
    return;
  }

  // auto volume: normalise to ~90 % (max gain x8)
  int32_t peak = 1;
  for (uint32_t i = 0; i < recLen; i++) {
    int32_t a = abs((int32_t)recBuf[i]);
    if (a > peak) peak = a;
  }
  float g = 29000.0f / peak;
  if (g > 16.0f) g = 16.0f;
  if (g > 1.0f) {
    for (uint32_t i = 0; i < recLen; i++) recBuf[i] = (int16_t)(recBuf[i] * g);
  }
#if NOISE_GATE
  noiseGate();
#endif
  Serial.printf("REC stop: %.2f s, peak %d, gain x%.1f, mic slot %s\n",
                recLen / (float)SAMPLE_RATE, (int)peak, g, micRight ? "RIGHT" : "LEFT");
  if (peak < 30) Serial.println("  Very quiet! Check the mic wires (SD D34, WS D33, SCK D32, VDD 3V3).");
  buildOverview();
  showReady();
}

// ======================= EFFECTS =======================
float sampleAt(float pos) {
  if (pos < 0) return 0;
  uint32_t i = (uint32_t)pos;
  if (i + 1 >= recLen) return 0;
  float f = pos - (float)i;
  return recBuf[i] + (recBuf[i + 1] - recBuf[i]) * f;
}

// Pitch shift with two cross-faded read heads. Keeps the same length.
float pitchNext(PitchShifter &ps, uint32_t n) {
  ps.d += 1.0f - ps.ratio;
  while (ps.d >= ps.win) ps.d -= ps.win;
  while (ps.d < 0)       ps.d += ps.win;
  float d2 = ps.d + ps.win * 0.5f;
  if (d2 >= ps.win) d2 -= ps.win;
  float w1 = sinf(PI * ps.d / ps.win); w1 *= w1;
  float w2 = sinf(PI * d2 / ps.win);   w2 *= w2;
  return sampleAt((float)n - ps.d) * w1 + sampleAt((float)n - d2) * w2;
}

// Plays the recording with an effect. Returns the next action if a button interrupts.
Action doPlay(Effect fx) {
  if (recLen == 0) {
    setState("NO AUDIO", C_WARN, "Hold BLUE to record", 0, -1);
    delay(900);
    showReady();
    return ACT_NONE;
  }

  setState(fxName[fx], fxCol[fx], fxHint[fx], 3, (int)fx + 1);
  drawOverview(C_UPCOMING);
  setField(0, "IN --", C_SILK);
  char f[16];
  if (fx == FX_ROBOT) snprintf(f, sizeof(f), "RING %dHz", (int)ROBOT_HZ);
  else                snprintf(f, sizeof(f), "OUT --");
  setField(1, f, fxCol[fx]);
  memset(hist, 0, sizeof(hist));

  PitchShifter ps = { 1.0f, 0, 800 };
  if (fx == FX_HIGH) { ps.ratio = PITCH_HIGH; ps.win = 800; }    // 50 ms window
  if (fx == FX_LOW)  { ps.ratio = PITCH_LOW;  ps.win = 1200; }   // 75 ms window
  const float ringStep = 2.0f * PI * ROBOT_HZ / SAMPLE_RATE;
  float ringPh = 0;
  const float lpA = 1.0f - expf(-2.0f * PI * TONE_CUT_HZ / SAMPLE_RATE);   // treble cut
  float lp = 0;

  writeSilence(DMA_BUFS);        // push out anything left from an interrupted playback
  ampEnable(true);

  Action next = ACT_NONE;
  uint32_t t0 = millis(), lastF = 0;
  int played = 0;

  for (uint32_t n0 = 0; n0 < recLen; n0 += CHUNK) {
    uint32_t cnt = recLen - n0;
    if (cnt > CHUNK) cnt = CHUNK;
    for (uint32_t k = 0; k < CHUNK; k++) {
      float y = 0;
      if (k < cnt) {
        uint32_t n = n0 + k;
        if (fx == FX_ROBOT) {
          y = recBuf[n] * sinf(ringPh) * 1.3f;
          ringPh += ringStep;
          if (ringPh > 2.0f * PI) ringPh -= 2.0f * PI;
        } else {
          y = pitchNext(ps, n);
        }
      }
      lp += (y - lp) * lpA;
      y = lp;
      if (y > 32767.0f)  y = 32767.0f;
      if (y < -32768.0f) y = -32768.0f;
      outBlock[k] = (int16_t)y;
      int16_t v = (int16_t)(y * OUT_VOLUME);
      outFrame[2 * k]     = v;   // left  (amp plays this, SD = HIGH)
      outFrame[2 * k + 1] = v;   // right (same, works in any amp mode)
    }
    writeOut();
    pushHist(outBlock, CHUNK);

    // playhead: bars already played take the effect colour
    int head = (int)((uint64_t)(n0 + cnt) * nBars / recLen);
    while (played < head && played < nBars) {
      tapeBar(played, env[played], fxCol[fx], false);
      played++;
    }

    uint32_t now = millis();
    if (now - t0 > 150 && now - lastF > 400) {
      lastF = now;
      float fin = (n0 + HIST <= recLen) ? detectPitch(recBuf + n0) : 0;
      if (fin > 0) snprintf(f, sizeof(f), "IN %dHz", (int)(fin + 0.5f));
      else         snprintf(f, sizeof(f), "IN --");
      setField(0, f, C_SILK);
      if (fx != FX_ROBOT) {
        float fout = detectPitch(hist);
        if (fout > 0) snprintf(f, sizeof(f), "OUT %dHz", (int)(fout + 0.5f));
        else          snprintf(f, sizeof(f), "OUT --");
        setField(1, f, fxCol[fx]);
      }
    }

    next = pollButtons();
    if (next != ACT_NONE) break;
  }

  if (next == ACT_NONE) {
    writeSilence(DMA_BUFS + 2);  // let the last sound leave the buffers, then mute
    ampEnable(false);
    showReady();
  } else {
    ampEnable(false);            // cut at once
  }
  return next;
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nESP32 Voice Changer rev 4");

  pinMode(PIN_BTN_REC,   INPUT_PULLUP);
  pinMode(PIN_BTN_HIGH,  INPUT_PULLUP);
  pinMode(PIN_BTN_LOW,   INPUT_PULLUP);
  pinMode(PIN_BTN_ROBOT, INPUT_PULLUP);
  pinMode(PIN_AMP_SD, OUTPUT);
  ampEnable(false);
  delay(50);

  // Screen: hardware SPI routed to D17 (SCK) and D16 (MOSI). No MISO.
  SPI.begin(PIN_TFT_SCK, -1, PIN_TFT_MOSI, PIN_TFT_CS);

  prefs.begin("voicechg", false);
  uint8_t prof = prefs.getUChar("prof", 255);
  uint8_t rot  = prefs.getUChar("rot", 0);
  if (FORCE_PROFILE >= 0) {
    prof = (uint8_t)FORCE_PROFILE;
    rot  = FORCE_ROTATION;
  } else if (prof >= NUM_PROFILES || digitalRead(PIN_BTN_HIGH) == LOW) {
    screenSetup(prof < NUM_PROFILES ? prof : 0, rot & 3);   // never returns (restarts)
  }
  if (prof >= NUM_PROFILES) prof = 0;

  startDisplay(prof, rot & 3);
  computeLayout();
  drawFrame();
  setState("STARTING", C_GOLD, "Audio...", 0, -1);

  if (!setupI2S()) {
    Serial.println("I2S start failed");
    fatal("I2S ERROR", "Audio start failed");
  }
  Serial.println("I2S0 mic OK, I2S1 amp OK");

  // one short listen at start-up: find the mic's I2S slot and check that it is alive.
  // Dead slot = flat line. Slot with wild values = floating wire, not the mic. Mic = the calm live one.
  for (int i = 0; i < 20; i++) readMicBlock(100);
  bool liveL = micEL > 0.3f, liveR = micER > 0.3f;
  if (liveL && liveR) micRight = (micER < micEL);
  else                micRight = (liveR && !liveL);
  micOk = !micDead();
  Serial.printf("Mic activity L=%.1f R=%.1f -> slot %s, %s\n", micEL, micER,
                micRight ? "RIGHT" : "LEFT", micOk ? "signal OK" : "NO SIGNAL (check mic wires)");

  // speaker test: hold WHITE at power-on -> clean 440 Hz tone for 2 s
  if (digitalRead(PIN_BTN_LOW) == LOW) {
    setState("SPK TEST", C_GOLD, "440 Hz tone, 2 s", 0, 2);
    ampEnable(true);
    float ph = 0;
    for (int b = 0; b < (SAMPLE_RATE * 2) / CHUNK; b++) {
      for (int k = 0; k < CHUNK; k++) {
        int16_t v = (int16_t)(12000.0f * OUT_VOLUME * sinf(ph));
        ph += 2.0f * PI * 440.0f / SAMPLE_RATE;
        if (ph > 2.0f * PI) ph -= 2.0f * PI;
        outFrame[2 * k] = v;
        outFrame[2 * k + 1] = v;
      }
      writeOut();
    }
    writeSilence(DMA_BUFS + 2);
    ampEnable(false);
    while (digitalRead(PIN_BTN_LOW) == LOW) delay(10);
  }

  // Record buffer: as big as RAM allows
  uint32_t want = (uint32_t)SAMPLE_RATE * MAX_SECONDS;
  if (psramFound()) {
    recBuf = (int16_t *)ps_malloc(want * sizeof(int16_t));
    if (recBuf) recMax = want;
  }
  if (!recBuf) {
    size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    size_t bytes = (largest > 30000) ? largest - 24000 : 0;   // leave RAM for the system
    if (bytes > want * sizeof(int16_t)) bytes = want * sizeof(int16_t);
    recBuf = (int16_t *)malloc(bytes);
    recMax = recBuf ? bytes / sizeof(int16_t) : 0;
  }
  Serial.printf("Max recording: %.2f s\n", recMax / (float)SAMPLE_RATE);
  if (!recBuf || recMax < SAMPLE_RATE / 2) fatal("RAM ERROR", "Buffer too small");

  showReady();
}

void loop() {
  Action a = pollButtons();
  while (a != ACT_NONE) {
    switch (a) {
      case ACT_RECORD: doRecord();           a = ACT_NONE; break;
      case ACT_HIGH:   a = doPlay(FX_HIGH);  break;
      case ACT_LOW:    a = doPlay(FX_LOW);   break;
      case ACT_ROBOT:  a = doPlay(FX_ROBOT); break;
      default:         a = ACT_NONE;         break;
    }
  }
  delay(5);    // idle: mic data is not read at all
}
