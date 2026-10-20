/*
============================================================
T-Camera Plus S3 Blob Counter… Serial Tuning Cheat Sheet
============================================================
By: HackMakeMod.com

WHAT YOU’RE TUNING
This sketch turns the camera image into a black/white “mask” and counts white blobs.
Pipeline:
  1) Compute mean brightness of the frame
  2) Threshold using: thr = mean - offset
  3) Clean up mask with erosion + dilation
  4) Label blobs and count those within minArea..maxArea

THE ONLY 3 KNOBS THAT MATTER
  offset  = contrast/sensitivity knob
            Higher offset = stricter threshold, less noise, might MISS parts
            Lower offset  = more sensitive, might count NOISE / merge blobs

  minArea = ignore tiny specks (dust, reflections, sensor junk)
            Raise minArea if you get phantom +1 blobs

  maxArea = ignore giant blobs (merged parts, big shadows)
            Lower maxArea if two touching parts get counted as one blob

BEST WAY TO TUNE (DO THIS IN ORDER)
  1) Press GPIO17 until MODE = MASK
     MASK shows exactly what the ESP32 is counting (white blobs on black).
  2) Tune offset first:
       - If blobs disappear or undercount… lower offset (try -5 steps)
       - If background noise becomes blobs… raise offset (try +5 steps)
  3) Tune minArea to kill specks without deleting real parts.
  4) Tune maxArea to prevent merged blobs from being counted.
  5) Press GPIO17 back to INFO mode to confirm count stability.
  6) Save settings (or leave autosave on).

GPIO17 BUTTON
Cycles display modes:
  INFO  -> count + stats
  CAMERA-> raw grayscale view
  MASK  -> processed black/white mask used for counting

SERIAL MONITOR NOTES
  - Set baud to 115200
  - Set line ending to: Newline

SERIAL COMMANDS (with expected ranges)

help
  - no args

show
  - no args

set offset <value>
  Range: 0 to 80
  Default: 40
  Typical tuning window: 20..55
  Notes: higher = stricter threshold (less noise, can miss parts)

set min <value>
  Range: 1 to 60000   (practically: 10..5000 at QVGA)
  Default: 45
  Typical tuning window: 30..200
  Notes: raise to ignore tiny specks and dust blobs

set max <value>
  Range: (min + 1) to 60000
  Default: 900
  Typical tuning window: 400..2000
  Notes: lower to reject merged blobs (two parts touching)

autosave on
autosave off
  Range: on|off
  Default: on
  Notes: if on, any "set ..." writes to flash immediately

save
  - no args
  Notes: writes current settings to flash (NVS)

defaults
  - no args
  Notes: loads defaults (offset=40, min=45, max=900, autosave=on) and saves them

ROI Size (Region of Interest)
  crop l <px>
  crop r <px>
  crop t <px>
  crop b <px>

TROUBLESHOOTING QUICK HITS
  Empty tray counts 1 or 2:
    offset up, or minArea up

  Clear parts but undercount:
    offset down

  Count flickers / jumps around:
    offset up (usually noise)

  Two parts touching becomes one blob:
    maxArea down (and sometimes offset up slightly)

  One part splits into two blobs:
    offset up a little, or minArea up

============================================================
*/

#include "esp_camera.h"
#include <Arduino_GFX_Library.h>
#include <Preferences.h>

// -------------------- Camera pins (T-Camera Plus S3) --------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// -------------------- Display pins --------------------
#define TFT_MOSI 35
#define TFT_SCLK 36
#define TFT_CS   34
#define TFT_DC   45
#define TFT_RST  33
#define TFT_BL   46

// -------------------- Button --------------------
#define BTN_MODE 17   // cycles INFO -> CAMERA -> MASK

Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, -1);
Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RST, 0, true, 240, 240);

// -------------------- Display constants --------------------
static const int DISP_W = 240;
static const int DISP_H = 240;

static const uint16_t COLOR_BLACK = 0x0000;
static const uint16_t COLOR_WHITE = 0xFFFF;
static const uint16_t COLOR_GREEN = 0x07E0;
static const uint16_t COLOR_RED   = 0xF800;

// -------------------- Persistent settings --------------------
Preferences prefs;

struct Settings {
  int threshOffset;   // adaptive threshold offset
  int minArea;
  int maxArea;
  bool autoSave;

  // ROI crop margins in DISPLAY PIXELS (what you see on screen)
  int cropL;
  int cropR;
  int cropT;
  int cropB;
};

Settings S;

// Your known-good defaults
const int DEF_THRESH_OFFSET = 40;
const int DEF_MIN_AREA      = 45;
const int DEF_MAX_AREA      = 900;
const bool DEF_AUTOSAVE     = true;

// ROI defaults (0 = no crop)
const int DEF_CROP_L = 0;
const int DEF_CROP_R = 0;
const int DEF_CROP_T = 0;
const int DEF_CROP_B = 0;

// -------------------- Modes --------------------
enum DisplayMode { MODE_INFO, MODE_CAMERA, MODE_MASK };
DisplayMode gMode = MODE_INFO;

// Button debounce
int lastBtnReading = HIGH;
int btnState = HIGH;
unsigned long lastDebounceMs = 0;
const unsigned long debounceDelayMs = 40;

// Serial input buffer
String cmdLine;

// -------------------- Helpers --------------------
static inline uint16_t grayTo565(uint8_t p)
{
  return ((p & 0xF8) << 8) | ((p & 0xFC) << 3) | (p >> 3);
}

static inline int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void clampSettings() {
  S.threshOffset = clampi(S.threshOffset, 0, 80);

  if (S.minArea < 1) S.minArea = 1;
  if (S.maxArea < S.minArea + 1) S.maxArea = S.minArea + 1;
  if (S.maxArea > 60000) S.maxArea = 60000;

  // Clamp crops so ROI always has at least 1 pixel in width/height
  S.cropL = clampi(S.cropL, 0, DISP_W - 1);
  S.cropR = clampi(S.cropR, 0, DISP_W - 1);
  S.cropT = clampi(S.cropT, 0, DISP_H - 1);
  S.cropB = clampi(S.cropB, 0, DISP_H - 1);

  if (S.cropL + S.cropR >= (DISP_W - 1)) {
    int over = (S.cropL + S.cropR) - (DISP_W - 2);
    // Reduce the larger side first
    if (S.cropL >= S.cropR) S.cropL = clampi(S.cropL - over, 0, DISP_W - 2);
    else S.cropR = clampi(S.cropR - over, 0, DISP_W - 2);
  }
  if (S.cropT + S.cropB >= (DISP_H - 1)) {
    int over = (S.cropT + S.cropB) - (DISP_H - 2);
    if (S.cropT >= S.cropB) S.cropT = clampi(S.cropT - over, 0, DISP_H - 2);
    else S.cropB = clampi(S.cropB - over, 0, DISP_H - 2);
  }
}

void loadSettings() {
  S.threshOffset = prefs.getInt("offset", DEF_THRESH_OFFSET);
  S.minArea      = prefs.getInt("minA",   DEF_MIN_AREA);
  S.maxArea      = prefs.getInt("maxA",   DEF_MAX_AREA);
  S.autoSave     = prefs.getBool("auto",  DEF_AUTOSAVE);

  S.cropL = prefs.getInt("cL", DEF_CROP_L);
  S.cropR = prefs.getInt("cR", DEF_CROP_R);
  S.cropT = prefs.getInt("cT", DEF_CROP_T);
  S.cropB = prefs.getInt("cB", DEF_CROP_B);

  clampSettings();
}

void saveSettings() {
  prefs.putInt("offset", S.threshOffset);
  prefs.putInt("minA",   S.minArea);
  prefs.putInt("maxA",   S.maxArea);
  prefs.putBool("auto",  S.autoSave);

  prefs.putInt("cL", S.cropL);
  prefs.putInt("cR", S.cropR);
  prefs.putInt("cT", S.cropT);
  prefs.putInt("cB", S.cropB);

  Serial.println("Saved.");
}

void setDefaults(bool saveNow) {
  S.threshOffset = DEF_THRESH_OFFSET;
  S.minArea = DEF_MIN_AREA;
  S.maxArea = DEF_MAX_AREA;
  S.autoSave = DEF_AUTOSAVE;

  S.cropL = DEF_CROP_L;
  S.cropR = DEF_CROP_R;
  S.cropT = DEF_CROP_T;
  S.cropB = DEF_CROP_B;

  clampSettings();
  Serial.println("Defaults loaded.");
  if (saveNow) saveSettings();
}

void showSettings() {
  Serial.println();
  Serial.println("Current settings:");
  Serial.printf("  offset   = %d\n", S.threshOffset);
  Serial.printf("  minArea  = %d\n", S.minArea);
  Serial.printf("  maxArea  = %d\n", S.maxArea);
  Serial.printf("  autosave = %s\n", S.autoSave ? "on" : "off");
  Serial.printf("  crop L/R/T/B = %d / %d / %d / %d\n", S.cropL, S.cropR, S.cropT, S.cropB);
  Serial.printf("  mode     = %s\n",
                (gMode == MODE_INFO) ? "info" : (gMode == MODE_CAMERA) ? "camera" : "mask");
  Serial.println();
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("  show");
  Serial.println("  ping   (or: ping in)");
  Serial.println("  set offset <0..80>");
  Serial.println("  set min <int>");
  Serial.println("  set max <int>");
  Serial.println("  crop l <px>   crop r <px>   crop t <px>   crop b <px>");
  Serial.println("  autosave on|off");
  Serial.println("  save");
  Serial.println("  defaults");
  Serial.println();
  Serial.println("GPIO17 cycles: INFO -> CAMERA -> MASK");
  Serial.println("Serial Monitor line ending: Newline");
  Serial.println();
}

// Draw ROI rectangle as thin red line on the display
void drawROIRect() {
  int x0 = S.cropL;
  int y0 = S.cropT;
  int x1 = (DISP_W - 1) - S.cropR;
  int y1 = (DISP_H - 1) - S.cropB;

  int w = x1 - x0 + 1;
  int h = y1 - y0 + 1;

  // Safety
  if (w <= 0 || h <= 0) return;

  gfx->drawRect(x0, y0, w, h, COLOR_RED);
}

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      cmdLine.trim();
      if (cmdLine.length() == 0) { cmdLine = ""; return; }

      String line = cmdLine;
      cmdLine = "";
      line.trim();
      String lower = line;
      lower.toLowerCase();

      if (lower == "help") { printHelp(); return; }
      if (lower == "show") { showSettings(); return; }
      if (lower == "save") { saveSettings(); return; }
      if (lower == "defaults") { setDefaults(true); return; }

      if (lower == "ping" || lower == "ping in") {
        Serial.println("pong");
        showSettings();
        return;
      }

      if (lower.startsWith("autosave ")) {
        if (lower.endsWith("on")) S.autoSave = true;
        else if (lower.endsWith("off")) S.autoSave = false;
        else { Serial.println("Use: autosave on|off"); return; }
        Serial.printf("autosave = %s\n", S.autoSave ? "on" : "off");
        if (S.autoSave) saveSettings();
        return;
      }

      if (lower.startsWith("set ")) {
        int p1 = lower.indexOf(' ');
        int p2 = lower.indexOf(' ', p1 + 1);
        if (p2 < 0) { Serial.println("Use: set offset|min|max <value>"); return; }

        String key = lower.substring(p1 + 1, p2);
        String valStr = lower.substring(p2 + 1);
        valStr.trim();
        int v = valStr.toInt();

        bool changed = false;

        if (key == "offset") {
          S.threshOffset = clampi(v, 0, 80);
          changed = true;
          Serial.printf("offset = %d\n", S.threshOffset);
        } else if (key == "min") {
          if (v < 1) v = 1;
          S.minArea = v;
          if (S.maxArea < S.minArea + 1) S.maxArea = S.minArea + 1;
          changed = true;
          Serial.printf("minArea = %d\n", S.minArea);
        } else if (key == "max") {
          if (v < S.minArea + 1) v = S.minArea + 1;
          if (v > 60000) v = 60000;
          S.maxArea = v;
          changed = true;
          Serial.printf("maxArea = %d\n", S.maxArea);
        } else {
          Serial.println("Keys: offset, min, max");
          return;
        }

        clampSettings();
        if (changed && S.autoSave) saveSettings();
        return;
      }

      // ROI crop commands: crop l|r|t|b <px>
      if (lower.startsWith("crop ")) {
        // Expect "crop <side> <value>"
        int p1 = lower.indexOf(' ');
        int p2 = lower.indexOf(' ', p1 + 1);
        if (p2 < 0) { Serial.println("Use: crop l|r|t|b <px>"); return; }

        String side = lower.substring(p1 + 1, p2);
        String valStr = lower.substring(p2 + 1);
        valStr.trim();
        int v = valStr.toInt();
        v = clampi(v, 0, 239);

        bool changed = true;

        if (side == "l") S.cropL = v;
        else if (side == "r") S.cropR = v;
        else if (side == "t") S.cropT = v;
        else if (side == "b") S.cropB = v;
        else { Serial.println("Sides: l r t b"); return; }

        clampSettings();

        Serial.printf("crop L/R/T/B = %d / %d / %d / %d\n", S.cropL, S.cropR, S.cropT, S.cropB);
        if (changed && S.autoSave) saveSettings();
        return;
      }

      Serial.println("Unknown command. Type: help");
      return;
    } else {
      if (cmdLine.length() < 160) cmdLine += c;
    }
  }
}

void handleModeButton() {
  int reading = digitalRead(BTN_MODE);

  if (reading != lastBtnReading) {
    lastDebounceMs = millis();
    lastBtnReading = reading;
  }

  if ((millis() - lastDebounceMs) > debounceDelayMs) {
    if (reading != btnState) {
      btnState = reading;
      if (btnState == LOW) {
        if (gMode == MODE_INFO) gMode = MODE_CAMERA;
        else if (gMode == MODE_CAMERA) gMode = MODE_MASK;
        else gMode = MODE_INFO;

        Serial.printf("Mode: %s\n",
          (gMode == MODE_INFO) ? "info" : (gMode == MODE_CAMERA) ? "camera" : "mask");

        gfx->fillScreen(COLOR_BLACK);
      }
    }
  }
}

// -------------------- Build processed mask (threshold + erosion + dilation) --------------------
// Allocates outMask (PSRAM), size width*height. Caller must free(outMask).
int buildProcessedMask(const uint8_t* img, int width, int height,
                       int &meanOut, int &thrOut, uint8_t* &outMask)
{
  int n = width * height;

  uint32_t sum = 0;
  for (int i = 0; i < n; i++) sum += img[i];
  int mean = sum / n;

  int thr = mean - S.threshOffset;
  if (thr < 20) thr = 20;
  if (thr > 230) thr = 230;

  meanOut = mean;
  thrOut = thr;

  uint8_t* binary = (uint8_t*)ps_malloc(n);
  if (!binary) return -1;

  for (int i = 0; i < n; i++) binary[i] = (img[i] < thr) ? 255 : 0;

  uint8_t* temp = (uint8_t*)ps_malloc(n);
  if (!temp) { free(binary); return -1; }

  // erosion 3x3
  memset(temp, 0, n);
  for (int y = 1; y < height - 1; y++) {
    for (int x = 1; x < width - 1; x++) {
      uint8_t minVal = 255;
      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          int idx = (y + dy) * width + (x + dx);
          if (binary[idx] < minVal) minVal = binary[idx];
        }
      }
      temp[y * width + x] = minVal;
    }
    yield();
  }

  // dilation 3x3
  memset(binary, 0, n);
  for (int y = 1; y < height - 1; y++) {
    for (int x = 1; x < width - 1; x++) {
      uint8_t maxVal = 0;
      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          int idx = (y + dy) * width + (x + dx);
          if (temp[idx] > maxVal) maxVal = temp[idx];
        }
      }
      binary[y * width + x] = maxVal;
    }
    yield();
  }

  free(temp);
  outMask = binary;
  return 0;
}

// -------------------- ROI extraction in DISPLAY coordinates --------------------
// The display shows a 240x240 center crop of the 320x240 camera frame, rotated 180 degrees.
// We extract ROI pixels (display coords) and map them back into the camera mask buffer.
int buildROIMaskFromFullMask(const uint8_t* fullMask, int camW, int camH,
                            uint8_t* &roiMask, int &roiW, int &roiH)
{
  // Center crop mapping (camera -> display pre-rotation)
  int xOff = (camW > DISP_W) ? ((camW - DISP_W) / 2) : 0; // usually 40
  int yOff = (camH > DISP_H) ? ((camH - DISP_H) / 2) : 0; // usually 0

  int dx0 = S.cropL;
  int dy0 = S.cropT;
  int dx1 = (DISP_W - 1) - S.cropR;
  int dy1 = (DISP_H - 1) - S.cropB;

  roiW = dx1 - dx0 + 1;
  roiH = dy1 - dy0 + 1;
  if (roiW <= 0 || roiH <= 0) return -1;

  roiMask = (uint8_t*)ps_malloc(roiW * roiH);
  if (!roiMask) return -1;

  // For each display pixel (dx,dy) inside ROI:
  // Convert to pre-rotation display coords (x,y) used when sampling from camera crop:
  // Our drawing uses dx = 239 - x, dy = 239 - y
  // So x = 239 - dx, y = 239 - dy
  for (int dy = dy0; dy <= dy1; dy++) {
    for (int dx = dx0; dx <= dx1; dx++) {

      int x = (DISP_W - 1) - dx;
      int y = (DISP_H - 1) - dy;

      int srcX = x + xOff;
      int srcY = y + yOff;

      uint8_t v = 0;
      if (srcX >= 0 && srcX < camW && srcY >= 0 && srcY < camH) {
        v = fullMask[srcY * camW + srcX];
      }

      int rx = dx - dx0;
      int ry = dy - dy0;
      roiMask[ry * roiW + rx] = v; // 0 or 255
    }
    yield();
  }

  return 0;
}

// -------------------- Count connected components inside ROI mask --------------------
int countFromROIMask(const uint8_t* mask, int width, int height) {
  // width/height here are ROI dimensions (max 240x240)
  const int MAX_LABELS = 2000;

  int n = width * height;

  uint16_t* labels = (uint16_t*)ps_calloc(n, sizeof(uint16_t));
  if (!labels) return -1;

  uint16_t* equivalence = (uint16_t*)malloc(MAX_LABELS * sizeof(uint16_t));
  if (!equivalence) { free(labels); return -1; }

  uint16_t* sizes = (uint16_t*)calloc(MAX_LABELS, sizeof(uint16_t));
  if (!sizes) { free(labels); free(equivalence); return -1; }

  for (int i = 0; i < MAX_LABELS; i++) equivalence[i] = i;

  uint16_t nextLabel = 1;

  // First pass (4-neighbor: left/top)
  for (int y = 1; y < height - 1; y++) {
    for (int x = 1; x < width - 1; x++) {
      int idx = y * width + x;

      if (mask[idx] == 255) {
        uint16_t left = labels[idx - 1];
        uint16_t top  = labels[idx - width];

        if (left == 0 && top == 0) {
          labels[idx] = nextLabel;
          nextLabel++;
          if (nextLabel >= MAX_LABELS) nextLabel = MAX_LABELS - 1;
        } else if (left != 0 && top == 0) {
          labels[idx] = left;
        } else if (left == 0 && top != 0) {
          labels[idx] = top;
        } else {
          uint16_t mn = (left < top) ? left : top;
          uint16_t mx = (left > top) ? left : top;
          labels[idx] = mn;
          if (mn != mx) equivalence[mx] = mn;
        }
      }
    }
    yield();
  }

  // Size accumulation
  for (int i = 0; i < n; i++) {
    uint16_t l = labels[i];
    if (l) {
      uint16_t root = l;
      while (equivalence[root] != root) root = equivalence[root];
      sizes[root]++;
    }
  }

  int count = 0;
  for (int i = 1; i < MAX_LABELS; i++) {
    if (sizes[i] >= S.minArea && sizes[i] <= S.maxArea) count++;
  }

  free(labels);
  free(equivalence);
  free(sizes);
  return count;
}

// -------------------- Drawing --------------------
void drawInfoScreen(int count, int meanVal, int thrVal)
{
  gfx->fillScreen(COLOR_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(8, 6);
  gfx->print("mean:");
  gfx->print(meanVal);
  gfx->print(" thr:");
  gfx->print(thrVal);

  gfx->setCursor(8, 28);
  gfx->print("off:");
  gfx->print(S.threshOffset);
  gfx->print(" min:");
  gfx->print(S.minArea);
  gfx->print(" max:");
  gfx->print(S.maxArea);

  gfx->setCursor(8, 50);
  gfx->print("crop LRTB:");
  gfx->print(S.cropL); gfx->print(",");
  gfx->print(S.cropR); gfx->print(",");
  gfx->print(S.cropT); gfx->print(",");
  gfx->print(S.cropB);

  gfx->setTextSize(10);
  gfx->setTextColor(COLOR_GREEN);

  int boxX = 20, boxY = 80, boxW = 200, boxH = 140;
  gfx->drawRect(boxX, boxY, boxW, boxH, COLOR_GREEN);

  gfx->setCursor(boxX + 60, boxY + 35);
  gfx->print(count);
}

void drawCameraScreen(camera_fb_t* fb)
{
  // Center crop 320x240 -> 240x240
  int xOff = ((int)fb->width  > DISP_W) ? (((int)fb->width  - DISP_W) / 2) : 0;  // 40
  int yOff = ((int)fb->height > DISP_H) ? (((int)fb->height - DISP_H) / 2) : 0;  // 0

  // Rotate 180 in software
  gfx->startWrite();
  for (int y = 0; y < DISP_H && (y + yOff) < (int)fb->height; y++) {
    int srcY = y + yOff;
    int rowBase = srcY * fb->width;

    for (int x = 0; x < DISP_W && (x + xOff) < (int)fb->width; x++) {
      int srcX = x + xOff;
      uint8_t pixel = fb->buf[rowBase + srcX];

      int dx = (DISP_W - 1) - x;
      int dy = (DISP_H - 1) - y;

      gfx->writePixel(dx, dy, grayTo565(pixel));
    }
    yield();
  }
  gfx->endWrite();

  drawROIRect();
}

void drawMaskScreen(const uint8_t* roiMask, int roiW, int roiH, int meanVal, int thrVal)
{
  // Draw ROI mask into the ROI rectangle area on-screen.
  // Everything outside ROI stays black.

  gfx->fillScreen(COLOR_BLACK);

  int dx0 = S.cropL;
  int dy0 = S.cropT;

  gfx->startWrite();
  for (int ry = 0; ry < roiH; ry++) {
    for (int rx = 0; rx < roiW; rx++) {
      uint8_t v = roiMask[ry * roiW + rx];
      gfx->writePixel(dx0 + rx, dy0 + ry, v ? COLOR_WHITE : COLOR_BLACK);
    }
    yield();
  }
  gfx->endWrite();

  drawROIRect();

  // Small status strip
  gfx->fillRect(0, 0, 240, 18, COLOR_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(4, 2);
  gfx->print("MASK m:");
  gfx->print(meanVal);
  gfx->print(" t:");
  gfx->print(thrVal);
}

// -------------------- Setup + loop --------------------
void setup() {
  Serial.begin(115200);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  pinMode(BTN_MODE, INPUT_PULLUP);

  prefs.begin("blobcam", false);
  loadSettings();

  printHelp();
  showSettings();

  gfx->begin();
  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(10, 10);
  gfx->println("Starting...");

  // Camera config
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_QVGA;        // 320x240
  config.pixel_format = PIXFORMAT_GRAYSCALE;   // 1 byte/pixel
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    gfx->setCursor(10, 30);
    gfx->println("Camera FAIL!");
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (1) delay(100);
  }

  gfx->fillScreen(COLOR_BLACK);
  gfx->setCursor(10, 10);
  gfx->println("Ready!");
  delay(200);
}

void loop() {
  handleSerial();
  handleModeButton();

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(50);
    return;
  }

  if (gMode == MODE_CAMERA) {
    drawCameraScreen(fb);
    esp_camera_fb_return(fb);
    delay(90);
    return;
  }

  // INFO and MASK use the processed mask, then crop it into ROI for counting and display.
  int meanVal = 0, thrVal = 0;
  uint8_t* fullMask = nullptr;

  int rc = buildProcessedMask(fb->buf, fb->width, fb->height, meanVal, thrVal, fullMask);
  if (rc != 0 || !fullMask) {
    Serial.println("Mask build failed (memory)");
    esp_camera_fb_return(fb);
    delay(120);
    return;
  }

  uint8_t* roiMask = nullptr;
  int roiW = 0, roiH = 0;

  int rc2 = buildROIMaskFromFullMask(fullMask, fb->width, fb->height, roiMask, roiW, roiH);
  free(fullMask);

  if (rc2 != 0 || !roiMask) {
    Serial.println("ROI mask build failed (memory or ROI too small)");
    esp_camera_fb_return(fb);
    delay(120);
    return;
  }

  int count = countFromROIMask(roiMask, roiW, roiH);

  if (gMode == MODE_MASK) {
    drawMaskScreen(roiMask, roiW, roiH, meanVal, thrVal);
  } else {
    drawInfoScreen(count, meanVal, thrVal);
  }

  Serial.printf("Count:%d mean=%d thr=%d offset=%d min=%d max=%d crop LRTB=%d,%d,%d,%d mode=%s\n",
    count, meanVal, thrVal, S.threshOffset, S.minArea, S.maxArea,
    S.cropL, S.cropR, S.cropT, S.cropB,
    (gMode == MODE_INFO) ? "info" : (gMode == MODE_CAMERA) ? "camera" : "mask");

  free(roiMask);

  esp_camera_fb_return(fb);
  delay(120);
}
