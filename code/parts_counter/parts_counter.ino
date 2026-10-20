#include "esp_camera.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== OLED =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SDA_PIN 14
#define SCL_PIN 13

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===== CAMERA PINS (AI Thinker ESP32-CAM) =====
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

//TUNABLE PARAMETERS
int thresholdOffset = 35;
int minArea = 40;
int maxArea = 3000;

int roi_x = 40, roi_y = 30, roi_w = 80, roi_h = 60;

// ===== RAM =====
#define MAX_STACK 800
uint8_t* bin = NULL;
int* stackBuf = NULL;

int clamp(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void showMessage(const char* msg, uint8_t textSize = 2) {
  display.clearDisplay();
  display.setTextSize(textSize);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(msg);
  display.display();
}

bool initDisplay() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(50000);
  delay(50);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) return true;
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) return true;
  return false;
}

// ===== ROI CALIBRATION =====
// Captures a frame, finds bounding box of the bright white square,
// stores it in roi_x/y/w/h for use throughout the session.
void calibrateROI() {
  showMessage("CAL ROI..", 2);

  // Flush a few frames so AEC settles before we sample
  for (int i = 0; i < 4; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(50);
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    showMessage("CAL FAIL", 2);
    delay(1000);
    return;   // keep fallback ROI
  }

  int w = fb->width;
  int h = fb->height;

  // Find the global mean so we can pick a brightness threshold adaptively
  long sum = 0;
  for (int i = 0; i < w * h; i++) sum += fb->buf[i];
  int mean = sum / (w * h);

  // Pixels must be noticeably brighter than average to count as "white box"
  uint8_t brightThr = clamp(mean + 40, 100, 240);

  // Walk every pixel and accumulate bounding box of bright region
  int x0 = w, y0 = h, x1 = 0, y1 = 0;

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      if (fb->buf[y * w + x] >= brightThr) {
        if (x < x0) x0 = x;
        if (x > x1) x1 = x;
        if (y < y0) y0 = y;
        if (y > y1) y1 = y;
      }
    }
  }

  esp_camera_fb_return(fb);

  if (x1 > x0 && y1 > y0) {
    // Small inset so ROI sits cleanly inside the white border
    roi_x = clamp(x0 + 2, 0, w - 1);
    roi_y = clamp(y0 + 2, 0, h - 1);
    roi_w = clamp(x1 - x0 - 4, 4, w - roi_x);
    roi_h = clamp(y1 - y0 - 4, 4, h - roi_y);
    showMessage("ROI OK", 2);
  } else {
    showMessage("ROI DEF", 2);   // nothing bright found, keep fallback
  }

  delay(700);
}

int floodFill(int start, int w, int h) {
  int size = 0;
  int top = 0;

  stackBuf[top++] = start;
  bin[start] = 0;

  while (top > 0) {
    int idx = stackBuf[--top];
    size++;

    int n1 = idx - 1;
    int n2 = idx + 1;
    int n3 = idx - w;
    int n4 = idx + w;

    if (n1 >= 0 && bin[n1] == 255 && top < MAX_STACK) {
      bin[n1] = 0;
      stackBuf[top++] = n1;
    }
    if (n2 < w * h && bin[n2] == 255 && top < MAX_STACK) {
      bin[n2] = 0;
      stackBuf[top++] = n2;
    }
    if (n3 >= 0 && bin[n3] == 255 && top < MAX_STACK) {
      bin[n3] = 0;
      stackBuf[top++] = n3;
    }
    if (n4 < w * h && bin[n4] == 255 && top < MAX_STACK) {
      bin[n4] = 0;
      stackBuf[top++] = n4;
    }
  }

  return size;
}

int countObjects(camera_fb_t* fb) {
  int w = fb->width;
  int h = fb->height;
  int n = w * h;

  if (!bin) bin = (uint8_t*)malloc(n);
  if (!stackBuf) stackBuf = (int*)malloc(MAX_STACK * sizeof(int));
  if (!bin || !stackBuf) return -1;

  long sum = 0;
  int countPix = 0;

  int x0 = clamp(roi_x, 0, w - 1);
  int y0 = clamp(roi_y, 0, h - 1);
  int x1 = clamp(roi_x + roi_w, 0, w);
  int y1 = clamp(roi_y + roi_h, 0, h);

  for (int y = y0; y < y1; y++) {
    for (int x = x0; x < x1; x++) {
      int i = y * w + x;
      sum += fb->buf[i];
      countPix++;
    }
  }

  if (countPix == 0) return 0;

  int mean = sum / countPix;
  int thr = clamp(mean - thresholdOffset, 20, 230);

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int i = y * w + x;
      if (x >= x0 && x < x1 && y >= y0 && y < y1 && fb->buf[i] < thr) {
        bin[i] = 255;
      } else {
        bin[i] = 0;
      }
    }
  }

  int count = 0;

  for (int y = y0; y < y1; y++) {
    for (int x = x0; x < x1; x++) {
      int i = y * w + x;
      if (bin[i] == 255) {
        int area = floodFill(i, w, h);
        if (area >= minArea && area <= maxArea) {
          count++;
        }
      }
    }
  }

  return count;
}

void setup() {
  delay(300);

  // 1) OLED first
  if (!initDisplay()) {
    while (1) delay(1000);
  }
  showMessage("OLED OK", 2);
  delay(700);

  // 2) Camera
  camera_config_t config = {};
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
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_QQVGA;
  config.fb_count     = 1;
  config.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&config) != ESP_OK) {
    showMessage("CAM FAIL", 2);
    while (1) delay(1000);
  }
  showMessage("CAM OK", 2);
  delay(700);

  // 3) Calibrate ROI from the lit white square — called ONCE, locked for session
  calibrateROI();

  showMessage("READY", 2);
  delay(700);
}

void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;

  int count = countObjects(fb);
  esp_camera_fb_return(fb);

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(3);
  display.setCursor(20, 20);
  display.print(count);
  display.display();

  delay(200);
}