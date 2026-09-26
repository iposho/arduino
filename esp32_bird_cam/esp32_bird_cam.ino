#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <time.h>
#include <esp_ota_ops.h>

#include "esp_camera.h"
#include "img_converters.h"
#include "FS.h"
#include "SD_MMC.h"

#include "secrets.h"
#include "firmware_info.h"
#include "../include/ota_mqtt.h"

// MQTT-топики (DEVICE_HOSTNAME из secrets.h):
//   devices/<hostname>/status       — online/offline (LWT)
//   devices/<hostname>/telemetry    — периодическая телеметрия
//   devices/<hostname>/command      — JSON-команды
//   devices/<hostname>/capabilities — retained JSON c описанием команд

// =====================
// AI-Thinker ESP32-CAM pins
// =====================
// LED вспышки (GPIO4). По умолчанию выкл; включается только командой led.
#define LED_FLASH_PIN 4

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

// =====================
// Intervals / limits
// =====================
// Кадр в RAM: днём раз в секунду (детектор птиц + /latest.jpg), ночью редко.
const unsigned long FRAME_INTERVAL_DAY_MS   = 1000UL;
const unsigned long FRAME_INTERVAL_NIGHT_MS = 30UL * 1000UL;
const unsigned long MQTT_TELEMETRY_INTERVAL = 10UL * 1000UL;
const unsigned long WIFI_RETRY_INTERVAL     = 30UL * 1000UL;
const unsigned long NTP_GMT_OFFSET_SEC      = 4UL * 3600UL;
const uint8_t JPEG_STREAM_QUALITY           = 12;  // аппаратный JPEG сенсора, SVGA ≈ 30–50 КБ
const uint8_t JPEG_SAVE_QUALITY             = 10;  // перекодирование снимка с меткой времени для SD

// =====================
// Детектор птиц (без нейронки): разница кадра 1/8 (100×75) с медленным фоном
// =====================
// Сетка детектора = кадр / 8: SVGA → 100×75, VGA (режим RGB565) → 80×60
const uint16_t MOTION_MAX_W = 100;
const uint16_t MOTION_MAX_H = 75;
// Аппаратный JPEG не дал кадра столько раз подряд — переходим на RGB565
const uint8_t  JPEG_FAIL_FALLBACK = 5;
const uint8_t  DAYLIGHT_LUMA_ON  = 45;             // средняя яркость: стало светло
const uint8_t  DAYLIGHT_LUMA_OFF = 30;             // стало темно (гистерезис)
const uint8_t  MOTION_PIXEL_DIFF = 28;             // порог изменения пикселя (0..255)
const uint16_t MOTION_TRIGGER_PERMILLE  = 20;      // ≥2% пикселей изменилось — движение
const uint16_t MOTION_LIGHTING_PERMILLE = 450;     // ≥45% — облако/экспозиция, сброс фона
const uint8_t  MOTION_CONFIRM_FRAMES    = 2;       // подряд, чтобы отсечь шум JPEG
const unsigned long BIRD_VISIT_GAP_MS   = 60UL * 1000UL;  // тишина дольше — следующий визит новый
const unsigned long BIRD_SAVE_INTERVAL_MS = 10UL * 1000UL; // на SD не чаще раза в 10 с

#define MQTT_BUFFER_SIZE  4096
#define FS_READ_MAX_BYTES 2800
#define FS_LS_MAX_ENTRIES 32

const uint16_t MAX_PHOTOS = 4800;
const char* PHOTOS_DIR = "/photos";
const char* PHOTO_INDEX_FILE = "/photo_index.dat";
const char* BIRD_LOG_FILE = "/bird_log.dat";
const uint8_t BIRD_LOG_SIZE = 24;  // последние снимки с птицами (id на SD + время)

// =====================
// State
// =====================
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebServer statusServer(80);

char topicStatus[64];
char topicTelemetry[64];
char topicCommand[64];
char topicCapabilities[64];
bool mqttTopicsReady = false;

const char *CAPABILITIES = R"CAP({
  "commands": [
    {
      "action": "led",
      "title": "Вспышка",
      "type": "toggle",
      "icon": "zap",
      "description": "LED вспышки камеры (GPIO4)"
    },
    {
      "action": "capture",
      "title": "Снимок",
      "type": "trigger",
      "icon": "camera",
      "description": "Сохранить текущий кадр на SD"
    },
    {
      "action": "reboot",
      "title": "Перезагрузка",
      "type": "trigger",
      "icon": "rotate-cw",
      "description": "ESP.restart()"
    }
  ],
  "metrics": [
    { "key": "ip", "label": "IP-адрес", "icon": "globe", "group": "Сеть", "dashboard": true, "order": 0 },
    { "key": "rssi", "label": "Сигнал Wi-Fi", "icon": "signal", "format": "rssi", "group": "Сеть", "dashboard": true, "order": 1 },
    { "key": "uptime", "label": "Аптайм", "icon": "clock", "format": "uptime", "group": "Система", "dashboard": true, "order": 2 },
    { "key": "heap", "keys": ["heap", "free_heap"], "label": "Свободная RAM", "icon": "memory", "format": "bytes", "group": "Система", "dashboard": true, "order": 3 },
    { "key": "sd_ready", "label": "SD-карта", "icon": "memory", "format": "boolean", "group": "Камера", "order": 10 },
    { "key": "sd_free_mb", "label": "Свободно на SD", "icon": "gauge", "format": "number", "unit": "МБ", "group": "Камера", "dashboard": true, "order": 11 },
    { "key": "bird_visits_today", "label": "Визиты птиц сегодня", "icon": "activity", "format": "number", "group": "Кормушка", "dashboard": true, "order": 5 },
    { "key": "bird_last", "label": "Последний визит", "icon": "clock", "format": "text", "group": "Кормушка", "dashboard": true, "order": 6 },
    { "key": "motion", "label": "Движение сейчас", "icon": "activity", "format": "boolean", "group": "Кормушка", "order": 7 },
    { "key": "daylight", "label": "Светло", "icon": "sun", "format": "boolean", "group": "Кормушка", "order": 8 },
    { "key": "last_capture_ok", "label": "Последний снимок", "icon": "camera", "format": "boolean", "group": "Камера", "order": 12 },
    { "key": "capture_errors", "label": "Ошибки съёмки", "icon": "activity", "format": "number", "group": "Камера", "order": 13 },
    { "key": "led", "label": "Вспышка", "icon": "zap", "format": "boolean", "group": "Камера", "order": 14 },
    { "key": "fw_version", "label": "Версия прошивки", "icon": "cpu", "group": "Система", "order": 20 }
  ],
  "dashboard": {
    "summary": ["bird_visits_today", "bird_last", "rssi", "uptime"],
    "max_items": 4
  }
})CAP";
bool webServerStarted = false;

bool cameraReady = false;
bool sdReady = false;
bool flashLedOn = false;
bool captureRequested = false;
bool lastCaptureOk = false;

uint16_t photoIndex = 0;
uint32_t captureErrors = 0;
uint32_t captureCount = 0;

char lastPhotoPath[48] = "";
char statusLine[64] = "Booting";
char sensorName[12] = "unknown";

// Последний кадр в PSRAM — отдаётся на /latest.jpg без SD.
uint8_t* frameBuf = nullptr;
size_t frameLen = 0;
size_t frameCap = 0;
unsigned long frameAtMs = 0;
uint32_t frameErrors = 0;

// Детектор
uint8_t* motionRgb = nullptr;   // MOTION_MAX_W*MOTION_MAX_H*2, RGB565
uint16_t motionW = MOTION_MAX_W;
uint16_t motionH = MOTION_MAX_H;
// Режим захвата. JPEG — аппаратный, SVGA. У некоторых OV3660 на AI-Thinker он
// сыплет cam_hal: FB-OVF и не отдаёт ни одного кадра — тогда RGB565 VGA
// с программным JPEG (так камера работала в прошивках ≤1.1.8).
bool camRgbMode = false;
uint8_t camFailStreak = 0;
uint8_t* motionGray = nullptr;  // текущий кадр
uint8_t* motionBg = nullptr;    // фон
bool motionBgReady = false;
bool daylight = true;
bool motionNow = false;
uint8_t motionStreak = 0;
uint8_t frameLuma = 0;
uint16_t motionPermille = 0;
uint32_t frameCount = 0;
unsigned long lastMotionMs = 0;
unsigned long lastBirdSaveMs = 0;
time_t birdLastAt = 0;
uint16_t birdVisitsToday = 0;
int birdVisitsYday = -1;

struct BirdShot {
  uint16_t photoId;
  uint32_t at;  // unix time
};
BirdShot birdLog[BIRD_LOG_SIZE];
uint8_t birdLogCount = 0;  // сколько записей заполнено
uint8_t birdLogHead = 0;   // куда писать следующую

unsigned long lastFrameTime = 0;
unsigned long lastMqttTelemetry = 0;
unsigned long lastWifiRetry = 0;

char otaUrl[256] = "";
bool otaPending = false;
int lastOtaProgress = -1;
bool littleFsReady = false;
bool otaInProgress = false;
char otaPhase[24] = "";
char otaFailedPhase[24] = "";
char otaErrorMsg[96] = "";
int otaProgressPct = -1;
size_t otaBytesDone = 0;
size_t otaBytesTotal = 0;
unsigned long otaStartedMs = 0;

// =====================
// Forward declarations
// =====================
void setStatus(const char* msg);
void setFlashLed(bool on);
bool initCamera();
bool initSdCard();
void restorePhotoIndex();
void persistPhotoIndex();
bool loadPhotoIndexFromFs();
bool captureAndSavePhoto();
bool grabFrame();
void analyzeFrame(unsigned long now);
void onBirdVisitStart();
void switchToRgbMode();
void loadBirdLog();
void pushBirdShot(uint16_t photoId, uint32_t at);
void handleBirdsJson();
bool encodePhotoJpeg(uint8_t** outBuf, size_t* outLen, bool* outAllocated);
void formatCaptureTimestamp(char* buf, size_t len);
void connectWiFi();
void syncTime();
void initMqttTopics();
void ensureMqtt();
void publishMqttTelemetry();
void publishOtaEvent(const char* phase, int progress = -1);
void setOtaProgress(const char* phase, int progress, size_t current = 0, size_t total = 0);
void queueOtaUpdate(const char* url);
void releaseResourcesForOta();
bool otaUpdateAvailable(size_t* slotSizeOut = nullptr);
void logPartitionInfo();
void performOtaUpdate(const char* url);
bool isFsPathValid(const char* path);
void publishFsTelemetry(JsonDocument& doc);
void publishFsError(const char* action, const char* path, const char* error);
void handleFsLs(const char* path);
void handleFsRead(const char* path);
void handleFsWrite(const char* path, const char* content);
void handleFsRm(const char* path);
void handleMqttCommand(char* topic, byte* payload, unsigned int length);
void startWebServer();
void ensureWebServer();
void handleStatusPage();
void handleOtaStatus();
void handleLatestPhoto();
void handlePhotoById();
String formatUptime(unsigned long ms);
String formatDateTime();
String htmlRow(const char* label, const String& value, const char* valueClass = "");
const char* otaPhaseLabel(const char* phase);
int otaPhaseOrder(const char* phase);
String htmlOtaStep(const char* label, int step, int currentStep, bool failed, int failedStep);

// =====================
// Helpers
// =====================
void setStatus(const char* msg) {
  strncpy(statusLine, msg, sizeof(statusLine) - 1);
  statusLine[sizeof(statusLine) - 1] = '\0';
  Serial.printf("[Status] %s\n", statusLine);
}

void setFlashLed(bool on) {
  flashLedOn = on;
  digitalWrite(LED_FLASH_PIN, on ? HIGH : LOW);
}

String formatUptime(unsigned long ms) {
  unsigned long sec = ms / 1000UL;
  return String(sec / 3600UL) + "ч " + String((sec % 3600UL) / 60UL) + "м";
}

String formatDateTime() {
  time_t now = time(nullptr);
  if (now < 100000) return "—";
  struct tm* t = localtime(&now);
  char buf[20];
  snprintf(buf, sizeof(buf), "%02d.%02d %02d:%02d",
           t->tm_mday, t->tm_mon + 1, t->tm_hour, t->tm_min);
  return String(buf);
}

void formatCaptureTimestamp(char* buf, size_t len) {
  time_t now = time(nullptr);
  if (now < 100000) {
    buf[0] = '\0';
    return;
  }
  struct tm* t = localtime(&now);
  strftime(buf, len, "%Y-%m-%d %H:%M:%S", t);
}

// 5x7, только символы для "YYYY-MM-DD HH:MM:SS"
static const char kTsFontChars[] = " 0123456789-:";
static const uint8_t kTsFont[][5] PROGMEM = {
  {0x00, 0x00, 0x00, 0x00, 0x00},
  {0x3E, 0x51, 0x49, 0x45, 0x3E},
  {0x00, 0x42, 0x7F, 0x40, 0x00},
  {0x42, 0x61, 0x51, 0x49, 0x46},
  {0x21, 0x41, 0x45, 0x4B, 0x31},
  {0x18, 0x14, 0x12, 0x7F, 0x10},
  {0x27, 0x45, 0x45, 0x45, 0x39},
  {0x3C, 0x4A, 0x49, 0x49, 0x30},
  {0x01, 0x71, 0x09, 0x05, 0x03},
  {0x36, 0x49, 0x49, 0x49, 0x36},
  {0x06, 0x49, 0x49, 0x29, 0x1E},
  {0x08, 0x08, 0x08, 0x08, 0x08},
  {0x00, 0x36, 0x36, 0x00, 0x00},
};

static void setPixel565(uint8_t* rgb, int width, int height, int x, int y, uint16_t color) {
  if (x < 0 || y < 0 || x >= width || y >= height) return;
  size_t idx = ((size_t)y * (size_t)width + (size_t)x) * 2;
  rgb[idx] = color & 0xFF;
  rgb[idx + 1] = color >> 8;
}

static void fillRect565(uint8_t* rgb, int width, int height,
                        int x, int y, int w, int h, uint16_t color) {
  for (int row = y; row < y + h; row++) {
    for (int col = x; col < x + w; col++) {
      setPixel565(rgb, width, height, col, row, color);
    }
  }
}

static const uint8_t* tsGlyphForChar(char c) {
  const char* hit = strchr(kTsFontChars, c);
  if (!hit) hit = kTsFontChars;
  return kTsFont[hit - kTsFontChars];
}

static void drawTsChar565(uint8_t* rgb, int width, int height,
                          int x, int y, char c, uint16_t fg, int scale) {
  const uint8_t* glyph = tsGlyphForChar(c);
  for (int col = 0; col < 5; col++) {
    uint8_t bits = pgm_read_byte(&glyph[col]);
    for (int row = 0; row < 7; row++) {
      if (!(bits & (1 << row))) continue;
      for (int sy = 0; sy < scale; sy++) {
        for (int sx = 0; sx < scale; sx++) {
          setPixel565(rgb, width, height, x + col * scale + sx, y + row * scale + sy, fg);
        }
      }
    }
  }
}

static void drawTimestamp565(uint8_t* rgb, int width, int height, const char* text) {
  const int scale = 2;
  const int margin = 10;
  const int charStep = 6 * scale;
  const int textW = (int)strlen(text) * charStep + 6;
  const int textH = 7 * scale + 6;
  const int x = margin;
  const int y = height - textH - margin;

  fillRect565(rgb, width, height, x - 3, y - 3, textW, textH, 0x0000);

  int cx = x;
  for (const char* p = text; *p; p++) {
    drawTsChar565(rgb, width, height, cx, y, *p, 0xFFFF, scale);
    cx += charStep;
  }
}

// Снимок для SD: JPEG кадра → RGB565 → метка времени → JPEG.
// Если метку поставить нельзя (нет времени/памяти) — пишем исходный JPEG как есть.
bool encodePhotoJpeg(uint8_t** outBuf, size_t* outLen, bool* outAllocated) {
  if (!frameBuf || frameLen == 0 || !outBuf || !outLen || !outAllocated) return false;
  *outAllocated = false;

  char ts[24];
  formatCaptureTimestamp(ts, sizeof(ts));
  sensor_t* s = esp_camera_sensor_get();
  if (ts[0] == '\0' || !s) {
    *outBuf = frameBuf;
    *outLen = frameLen;
    return true;
  }

  const uint16_t w = resolution[s->status.framesize].width;
  const uint16_t h = resolution[s->status.framesize].height;
  size_t rgbLen = (size_t)w * h * 2;
  uint8_t* rgb = (uint8_t*)ps_malloc(rgbLen);
  if (!rgb || !jpg2rgb565(frameBuf, frameLen, rgb, JPG_SCALE_NONE)) {
    if (rgb) free(rgb);
    *outBuf = frameBuf;
    *outLen = frameLen;
    return true;
  }

  // jpg2rgb565 отдаёт RGB565 в обратном порядке байт относительно того, что ждёт
  // fmt2jpg (порядок сенсора). Без перестановки снимок на SD выходит «кислотным»:
  // младшие биты зелёного попадают в старшие разряды — радужные полосы вместо градиентов.
  for (size_t i = 0; i + 1 < rgbLen; i += 2) {
    uint8_t t = rgb[i];
    rgb[i] = rgb[i + 1];
    rgb[i + 1] = t;
  }

  drawTimestamp565(rgb, w, h, ts);
  bool ok = fmt2jpg(rgb, rgbLen, w, h, PIXFORMAT_RGB565, JPEG_SAVE_QUALITY, outBuf, outLen);
  free(rgb);
  if (!ok) {
    *outBuf = frameBuf;
    *outLen = frameLen;
    return true;
  }
  *outAllocated = true;
  return true;
}

String photoPathForIndex(uint16_t index) {
  char path[48];
  snprintf(path, sizeof(path), "%s/%05u.jpg", PHOTOS_DIR, index % MAX_PHOTOS);
  return String(path);
}

uint64_t sdFreeBytes() {
  if (!sdReady) return 0;
  return SD_MMC.totalBytes() - SD_MMC.usedBytes();
}

// =====================
// Camera
// =====================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
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
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = JPEG_STREAM_QUALITY;
  if (camRgbMode) {
    config.frame_size = FRAMESIZE_VGA;
    config.pixel_format = PIXFORMAT_RGB565;
    config.fb_count = 1;
  } else {
    // Буферы драйвер выделяет под frame_size при init (JPEG ≈ w*h/5): init на UXGA
    // даёт запас, рабочее разрешение — SVGA ниже.
    config.frame_size = FRAMESIZE_UXGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.fb_count = 2;
  }

  if (!psramFound()) {
    Serial.println("[Camera] no PSRAM — AI-Thinker без PSRAM не поддерживается");
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.fb_count = 1;
  }

  // После мягкого ресета (OTA, RST, esptool) сенсор иногда остаётся в
  // неопределённом состоянии и не отвечает по SCCB — передёргиваем PWDN.
  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, HIGH);
  delay(100);
  digitalWrite(PWDN_GPIO_NUM, LOW);
  delay(100);

  esp_err_t err = ESP_FAIL;
  for (int attempt = 1; attempt <= 3; attempt++) {
    err = esp_camera_init(&config);
    if (err == ESP_OK) break;
    Serial.printf("[Camera] init failed: 0x%x (attempt %d/3)\n", err, attempt);
    esp_camera_deinit();  // без deinit повторный init не проходит
    digitalWrite(PWDN_GPIO_NUM, HIGH);
    delay(200);
    digitalWrite(PWDN_GPIO_NUM, LOW);
    delay(200);
  }
  if (err != ESP_OK) {
    cameraReady = false;
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    switch (s->id.PID) {
      case OV2640_PID: strcpy(sensorName, "OV2640"); break;
      case OV3660_PID: strcpy(sensorName, "OV3660"); break;
      case OV5640_PID: strcpy(sensorName, "OV5640"); break;
      default: snprintf(sensorName, sizeof(sensorName), "0x%04x", s->id.PID); break;
    }
    Serial.printf("[Camera] sensor %s, mode %s\n", sensorName, camRgbMode ? "rgb565 VGA" : "jpeg SVGA");
    if (!camRgbMode) s->set_framesize(s, FRAMESIZE_SVGA);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 1);
    if (s->id.PID == OV3660_PID) {
      // Как в CameraWebServer от Espressif: OV3660 по умолчанию тёмный и пересвеченный по цвету
      s->set_brightness(s, 1);
      s->set_saturation(s, -2);
    } else {
      s->set_brightness(s, 0);
      s->set_contrast(s, 0);
      s->set_saturation(s, 0);
    }
  }

  framesize_t fs = s ? (framesize_t)s->status.framesize : (camRgbMode ? FRAMESIZE_VGA : FRAMESIZE_SVGA);
  motionW = min<uint16_t>(resolution[fs].width / 8, MOTION_MAX_W);
  motionH = min<uint16_t>(resolution[fs].height / 8, MOTION_MAX_H);
  motionBgReady = false;
  camFailStreak = 0;

  cameraReady = true;
  Serial.println("[Camera] OK");
  return true;
}

// Аппаратный JPEG не работает на этом сенсоре — переинициализация в RGB565
void switchToRgbMode() {
  Serial.println("[Camera] JPEG mode gives no frames — switching to RGB565");
  esp_camera_deinit();
  cameraReady = false;
  camRgbMode = true;
  initCamera();
  publishMqttTelemetry();
}

// =====================
// SD card
// =====================
bool initSdCard() {
  if (!SD_MMC.setPins(14, 15, 2)) {
    Serial.println("[SD] setPins failed");
    sdReady = false;
    return false;
  }

  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[SD] mount failed");
    sdReady = false;
    return false;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("[SD] no card");
    sdReady = false;
    return false;
  }

  if (!SD_MMC.exists(PHOTOS_DIR)) {
    if (!SD_MMC.mkdir(PHOTOS_DIR)) {
      Serial.println("[SD] mkdir /photos failed");
      sdReady = false;
      return false;
    }
  }

  sdReady = true;
  Serial.printf("[SD] OK, free %llu MB\n", sdFreeBytes() / (1024ULL * 1024ULL));
  restorePhotoIndex();
  return true;
}

void persistPhotoIndex() {
  if (!littleFsReady) return;

  File f = LittleFS.open(PHOTO_INDEX_FILE, "w");
  if (!f) return;

  f.write((const uint8_t*)&photoIndex, sizeof(photoIndex));
  f.close();
}

bool loadPhotoIndexFromFs() {
  if (!littleFsReady || !LittleFS.exists(PHOTO_INDEX_FILE)) return false;

  File f = LittleFS.open(PHOTO_INDEX_FILE, "r");
  if (!f || f.size() < (int)sizeof(photoIndex)) {
    if (f) f.close();
    return false;
  }

  uint16_t saved = 0;
  if (f.read((uint8_t*)&saved, sizeof(saved)) != sizeof(saved)) {
    f.close();
    return false;
  }
  f.close();

  if (saved >= MAX_PHOTOS) return false;

  photoIndex = saved;
  uint16_t prev = (saved == 0) ? (uint16_t)(MAX_PHOTOS - 1) : (uint16_t)(saved - 1);
  String lastPath = photoPathForIndex(prev);
  if (SD_MMC.exists(lastPath)) {
    strncpy(lastPhotoPath, lastPath.c_str(), sizeof(lastPhotoPath) - 1);
    lastPhotoPath[sizeof(lastPhotoPath) - 1] = '\0';
    lastCaptureOk = true;
  }
  return true;
}

void restorePhotoIndex() {
  if (!sdReady) return;

  Serial.println("[SD] restoring photo index...");

  if (loadPhotoIndexFromFs()) {
    Serial.printf("[SD] photo index from LittleFS: %u\n", photoIndex);
    return;
  }

  Serial.println("[SD] scanning card (first boot, may take a minute)...");

  uint16_t maxIndex = 0;
  bool found = false;
  for (uint16_t i = 0; i < MAX_PHOTOS; i++) {
    if ((i % 200) == 0) {
      Serial.printf("[SD] scan %u/%u\n", i, MAX_PHOTOS);
      yield();
    }
    if (SD_MMC.exists(photoPathForIndex(i))) {
      if (i >= maxIndex) maxIndex = i;
      found = true;
    }
  }

  if (found) {
    photoIndex = (uint16_t)((maxIndex + 1) % MAX_PHOTOS);
    String lastPath = photoPathForIndex(maxIndex);
    strncpy(lastPhotoPath, lastPath.c_str(), sizeof(lastPhotoPath) - 1);
    lastPhotoPath[sizeof(lastPhotoPath) - 1] = '\0';
    lastCaptureOk = true;
  } else {
    photoIndex = 0;
  }

  persistPhotoIndex();
  Serial.printf("[SD] photo index restored to %u\n", photoIndex);
}

bool captureAndSavePhoto() {
  if (!cameraReady || !sdReady || !frameBuf || frameLen == 0) {
    lastCaptureOk = false;
    captureErrors++;
    return false;
  }

  String path = photoPathForIndex(photoIndex);
  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file) {
    Serial.printf("[Capture] open %s failed\n", path.c_str());
    lastCaptureOk = false;
    captureErrors++;
    return false;
  }

  uint8_t* jpegBuf = nullptr;
  size_t writeLen = 0;
  bool allocated = false;
  if (!encodePhotoJpeg(&jpegBuf, &writeLen, &allocated)) {
    Serial.println("[Capture] jpeg encode failed");
    file.close();
    lastCaptureOk = false;
    captureErrors++;
    return false;
  }

  size_t written = file.write(jpegBuf, writeLen);
  if (allocated) free(jpegBuf);
  file.close();

  if (written != writeLen) {
    Serial.println("[Capture] write incomplete");
    lastCaptureOk = false;
    captureErrors++;
    return false;
  }

  strncpy(lastPhotoPath, path.c_str(), sizeof(lastPhotoPath) - 1);
  lastPhotoPath[sizeof(lastPhotoPath) - 1] = '\0';

  photoIndex = (photoIndex + 1) % MAX_PHOTOS;
  captureCount++;
  lastCaptureOk = true;
  persistPhotoIndex();

  Serial.printf("[Capture] saved %s (%u bytes)\n", lastPhotoPath, (unsigned)written);
  setStatus("Photo saved");
  return true;
}

// =====================
// Кадр + детектор птиц
// =====================
static bool storeFrame(const uint8_t* jpg, size_t len) {
  if (len > frameCap) {
    size_t cap = len + 16 * 1024;
    uint8_t* buf = (uint8_t*)(frameBuf ? ps_realloc(frameBuf, cap) : ps_malloc(cap));
    if (!buf) return false;
    frameBuf = buf;
    frameCap = cap;
  }
  memcpy(frameBuf, jpg, len);
  frameLen = len;
  frameAtMs = millis();
  return true;
}

bool grabFrame() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb || fb->len == 0) {
    if (fb) esp_camera_fb_return(fb);
    frameErrors++;
    camFailStreak++;
    if (frameErrors % 10 == 1) {
      Serial.printf("[Frame] fb_get failed (errors=%lu)\n", (unsigned long)frameErrors);
    }
    if (!camRgbMode && camFailStreak >= JPEG_FAIL_FALLBACK) switchToRgbMode();
    return false;
  }
  camFailStreak = 0;

  if (fb->format != PIXFORMAT_JPEG) {
    uint8_t* jpg = nullptr;
    size_t jpgLen = 0;
    bool ok = frame2jpg(fb, JPEG_STREAM_QUALITY, &jpg, &jpgLen);
    esp_camera_fb_return(fb);
    ok = ok && storeFrame(jpg, jpgLen);
    if (jpg) free(jpg);
    if (!ok) frameErrors++;
    return ok;
  }

  bool ok = storeFrame(fb->buf, fb->len);
  esp_camera_fb_return(fb);
  if (!ok) frameErrors++;
  return ok;
}

static uint16_t countChangedPermille(const uint8_t* cur, const uint8_t* bg, size_t n) {
  size_t changed = 0;
  for (size_t i = 0; i < n; i++) {
    int d = (int)cur[i] - (int)bg[i];
    if (d > MOTION_PIXEL_DIFF || d < -MOTION_PIXEL_DIFF) changed++;
  }
  return (uint16_t)((changed * 1000UL) / n);
}

// Фон подтягивается к текущему кадру: быстро в покое, медленно при движении
// (сидящая долго птица постепенно «впитывается», и визит заканчивается).
static void blendBackground(uint8_t* bg, const uint8_t* cur, size_t n, uint8_t shift) {
  for (size_t i = 0; i < n; i++) {
    int d = (int)cur[i] - (int)bg[i];
    bg[i] = (uint8_t)((int)bg[i] + d / (1 << shift));
  }
}

void loadBirdLog() {
  if (!littleFsReady || !LittleFS.exists(BIRD_LOG_FILE)) return;
  File f = LittleFS.open(BIRD_LOG_FILE, "r");
  if (!f) return;
  uint8_t hdr[2];
  if (f.read(hdr, 2) == 2 && hdr[0] <= BIRD_LOG_SIZE && hdr[1] < BIRD_LOG_SIZE &&
      f.read((uint8_t*)birdLog, sizeof(birdLog)) == sizeof(birdLog)) {
    birdLogCount = hdr[0];
    birdLogHead = hdr[1];
  }
  f.close();

  if (birdLogCount > 0) {
    const BirdShot& last = birdLog[(birdLogHead + BIRD_LOG_SIZE - 1) % BIRD_LOG_SIZE];
    birdLastAt = (time_t)last.at;
  }
  Serial.printf("[Bird] log restored: %u shots\n", birdLogCount);
}

void pushBirdShot(uint16_t photoId, uint32_t at) {
  birdLog[birdLogHead] = { photoId, at };
  birdLogHead = (birdLogHead + 1) % BIRD_LOG_SIZE;
  if (birdLogCount < BIRD_LOG_SIZE) birdLogCount++;

  if (!littleFsReady) return;
  File f = LittleFS.open(BIRD_LOG_FILE, "w");
  if (!f) return;
  uint8_t hdr[2] = { birdLogCount, birdLogHead };
  f.write(hdr, 2);
  f.write((const uint8_t*)birdLog, sizeof(birdLog));
  f.close();
}

int latestBirdPhotoId() {
  if (birdLogCount == 0) return -1;
  return birdLog[(birdLogHead + BIRD_LOG_SIZE - 1) % BIRD_LOG_SIZE].photoId;
}

void resetBirdCounterIfNewDay() {
  time_t now = time(nullptr);
  if (now < 100000) return;
  struct tm* t = localtime(&now);
  if (t->tm_yday != birdVisitsYday) {
    birdVisitsYday = t->tm_yday;
    birdVisitsToday = 0;
  }
}

void onBirdVisitStart() {
  resetBirdCounterIfNewDay();
  birdVisitsToday++;
  Serial.printf("[Bird] visit #%u (changed %u‰)\n", birdVisitsToday, motionPermille);
  setStatus("Bird!");
}

void analyzeFrame(unsigned long now) {
  const size_t n = (size_t)motionW * motionH;
  if (!motionRgb) {
    const size_t maxN = (size_t)MOTION_MAX_W * MOTION_MAX_H;
    motionRgb = (uint8_t*)ps_malloc(maxN * 2);
    motionGray = (uint8_t*)ps_malloc(maxN);
    motionBg = (uint8_t*)ps_malloc(maxN);
    if (!motionRgb || !motionGray || !motionBg) {
      Serial.println("[Bird] no memory for detector");
      return;
    }
  }
  if (!jpg2rgb565(frameBuf, frameLen, motionRgb, JPG_SCALE_8X)) {
    frameErrors++;
    return;
  }

  // RGB565 big-endian (порядок байт декодера esp32-camera) → яркость.
  uint32_t lumaSum = 0;
  for (size_t i = 0; i < n; i++) {
    uint16_t c = ((uint16_t)motionRgb[i * 2] << 8) | motionRgb[i * 2 + 1];
    uint8_t r = (c >> 11) << 3;
    uint8_t g = ((c >> 5) & 0x3F) << 2;
    uint8_t b = (c & 0x1F) << 3;
    uint8_t y = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
    motionGray[i] = y;
    lumaSum += y;
  }
  frameLuma = (uint8_t)(lumaSum / n);

  // Диагностика в Serial: днём раз в минуту, ночью каждый кадр (раз в 30 с)
  frameCount++;
  if (!daylight || frameCount % 60 == 1) {
    Serial.printf("[Frame] #%lu %u B luma=%u motion=%u%% errors=%lu\n",
                  (unsigned long)frameCount, (unsigned)frameLen, frameLuma,
                  motionPermille / 10, (unsigned long)frameErrors);
  }

  bool wasDaylight = daylight;
  if (daylight && frameLuma < DAYLIGHT_LUMA_OFF) daylight = false;
  else if (!daylight && frameLuma > DAYLIGHT_LUMA_ON) daylight = true;

  if (daylight != wasDaylight) {
    Serial.printf("[Bird] %s (luma %u)\n", daylight ? "day" : "night", frameLuma);
    motionBgReady = false;
    motionStreak = 0;
    motionNow = false;
    publishMqttTelemetry();
  }

  if (!motionBgReady) {
    memcpy(motionBg, motionGray, n);
    motionBgReady = true;
    motionPermille = 0;
    return;
  }

  motionPermille = countChangedPermille(motionGray, motionBg, n);

  if (!daylight || motionPermille >= MOTION_LIGHTING_PERMILLE) {
    // Ночь или резкая смена освещения — это не птица, просто новый фон.
    memcpy(motionBg, motionGray, n);
    motionStreak = 0;
    motionNow = false;
    return;
  }

  bool moving = motionPermille >= MOTION_TRIGGER_PERMILLE;
  motionStreak = moving ? (motionStreak < 255 ? motionStreak + 1 : 255) : 0;
  blendBackground(motionBg, motionGray, n, moving ? 5 : 3);

  bool confirmed = motionStreak >= MOTION_CONFIRM_FRAMES;
  bool changed = confirmed != motionNow;
  motionNow = confirmed;
  if (!confirmed) {
    if (changed) publishMqttTelemetry();
    return;
  }

  bool newVisit = lastMotionMs == 0 || now - lastMotionMs >= BIRD_VISIT_GAP_MS;
  lastMotionMs = now;
  time_t wall = time(nullptr);
  if (wall >= 100000) birdLastAt = wall;

  if (newVisit) onBirdVisitStart();

  if (sdReady && (newVisit || now - lastBirdSaveMs >= BIRD_SAVE_INTERVAL_MS)) {
    lastBirdSaveMs = now;
    uint16_t id = photoIndex;
    if (captureAndSavePhoto() && birdLastAt > 0) {
      pushBirdShot(id, (uint32_t)birdLastAt);
    }
  }

  if (newVisit || changed) publishMqttTelemetry();
}

// =====================
// Wi-Fi / NTP
// =====================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] connecting");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK");
    Serial.printf("[WiFi] IP %s\n", WiFi.localIP().toString().c_str());
    setStatus("WiFi connected");
  } else {
    Serial.println(" FAILED");
    setStatus("WiFi failed");
  }
}

void syncTime() {
  configTime(NTP_GMT_OFFSET_SEC, 0, "pool.ntp.org", "time.google.com");

  Serial.print("[NTP] sync");
  int tries = 0;
  while (time(nullptr) < 100000 && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println(time(nullptr) >= 100000 ? " OK" : " timeout");
}

// =====================
// MQTT (шлюз esp32.kuzyak.in)
// =====================
void initMqttTopics() {
  if (mqttTopicsReady) return;

  snprintf(topicStatus, sizeof(topicStatus), "devices/%s/status", DEVICE_HOSTNAME);
  snprintf(topicTelemetry, sizeof(topicTelemetry), "devices/%s/telemetry", DEVICE_HOSTNAME);
  snprintf(topicCommand, sizeof(topicCommand), "devices/%s/command", DEVICE_HOSTNAME);
  snprintf(topicCapabilities, sizeof(topicCapabilities), "devices/%s/capabilities", DEVICE_HOSTNAME);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(handleMqttCommand);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  mqttTopicsReady = true;
}

bool isFsPathValid(const char* path) {
  if (!path || path[0] != '/') return false;
  if (strstr(path, "..") != nullptr) return false;
  return strlen(path) < 128;
}

void publishFsTelemetry(JsonDocument& doc) {
  if (!mqttClient.connected()) {
    Serial.println("[FS] mqtt not connected, skip publish");
    return;
  }

  char buf[MQTT_BUFFER_SIZE];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0 || n >= sizeof(buf)) {
    Serial.println("[FS] response too large for MQTT buffer");
    publishFsError("fs", nullptr, "response too large");
    return;
  }
  Serial.printf("[FS] >> %s (%u bytes)\n", topicTelemetry, n);
  bool ok = mqttClient.publish(topicTelemetry, buf, n);
  Serial.printf("[FS] publish %s\n", ok ? "OK" : "FAILED");
  mqttClient.loop();
}

void publishFsError(const char* action, const char* path, const char* error) {
  Serial.printf("[FS] error action=%s path=%s: %s\n",
                action ? action : "-", path ? path : "-", error);
  if (!mqttClient.connected()) return;

  StaticJsonDocument<256> doc;
  doc["fs_error"] = error;
  doc["fs_action"] = action;
  if (path) doc["fs_path"] = path;

  char buf[256];
  size_t n = serializeJson(doc, buf);
  mqttClient.publish(topicTelemetry, buf, n);
  mqttClient.loop();
}

void handleFsLs(const char* path) {
  if (!littleFsReady) {
    publishFsError("fs_ls", path, "filesystem not mounted");
    return;
  }
  if (!isFsPathValid(path)) {
    publishFsError("fs_ls", path, "invalid path");
    return;
  }

  File dir = LittleFS.open(path);
  if (!dir || !dir.isDirectory()) {
    publishFsError("fs_ls", path, dir ? "not a directory" : "open failed");
    if (dir) dir.close();
    return;
  }

  DynamicJsonDocument doc(4096);
  JsonArray arr = doc.createNestedArray("fs_ls");
  int count = 0;

  for (File entry = dir.openNextFile(); entry && count < FS_LS_MAX_ENTRIES; entry = dir.openNextFile()) {
    char fullPath[128];
    if (strcmp(path, "/") == 0) {
      snprintf(fullPath, sizeof(fullPath), "/%s", entry.name());
    } else {
      snprintf(fullPath, sizeof(fullPath), "%s/%s", path, entry.name());
    }

    JsonObject item = arr.createNestedObject();
    item["name"] = fullPath;
    item["size"] = entry.size();
    Serial.printf("[FS]   %s (%u)\n", fullPath, entry.size());
    entry.close();
    count++;
  }
  dir.close();

  if (count >= FS_LS_MAX_ENTRIES) {
    doc["fs_ls_truncated"] = true;
  }

  Serial.printf("[FS] ls %s -> %d entries\n", path, count);
  publishFsTelemetry(doc);
}

void handleFsRead(const char* path) {
  if (!littleFsReady) {
    publishFsError("fs_read", path, "filesystem not mounted");
    return;
  }
  if (!isFsPathValid(path)) {
    publishFsError("fs_read", path, "invalid path");
    return;
  }

  if (!LittleFS.exists(path)) {
    publishFsError("fs_read", path, "not found");
    return;
  }

  File f = LittleFS.open(path, "r");
  if (!f) {
    publishFsError("fs_read", path, "open failed");
    return;
  }
  if (f.isDirectory()) {
    f.close();
    publishFsError("fs_read", path, "is a directory");
    return;
  }

  size_t fileSize = f.size();
  if (fileSize > FS_READ_MAX_BYTES) {
    f.close();
    publishFsError("fs_read", path, "file too large");
    return;
  }

  String content;
  content.reserve(fileSize + 1);
  while (f.available()) {
    content += static_cast<char>(f.read());
  }
  f.close();

  DynamicJsonDocument doc(fileSize + 192);
  doc["fs_file"] = path;
  doc["content"] = content;
  Serial.printf("[FS] read %s (%u bytes)\n", path, fileSize);
  publishFsTelemetry(doc);
}

void handleFsWrite(const char* path, const char* content) {
  if (!littleFsReady) {
    publishFsError("fs_write", path, "filesystem not mounted");
    return;
  }
  if (!isFsPathValid(path)) {
    publishFsError("fs_write", path, "invalid path");
    return;
  }
  if (!content) content = "";

  File f = LittleFS.open(path, "w");
  if (!f) {
    publishFsError("fs_write", path, "open failed");
    return;
  }

  size_t written = f.print(content);
  f.close();

  StaticJsonDocument<192> doc;
  doc["fs_written"] = path;
  doc["bytes"] = written;
  Serial.printf("[FS] write %s (%u bytes)\n", path, written);
  publishFsTelemetry(doc);
}

void handleFsRm(const char* path) {
  if (!littleFsReady) {
    publishFsError("fs_rm", path, "filesystem not mounted");
    return;
  }
  if (!isFsPathValid(path)) {
    publishFsError("fs_rm", path, "invalid path");
    return;
  }

  if (!LittleFS.exists(path)) {
    publishFsError("fs_rm", path, "not found");
    return;
  }

  if (!LittleFS.remove(path)) {
    publishFsError("fs_rm", path, "remove failed");
    return;
  }

  StaticJsonDocument<128> doc;
  doc["fs_removed"] = path;
  Serial.printf("[FS] rm %s\n", path);
  publishFsTelemetry(doc);
}

void handleMqttCommand(char* topic, byte* payload, unsigned int length) {
  Serial.printf("[MQTT] << %s (%u): %.*s\n", topic, length, length, payload);

  DynamicJsonDocument doc(length + 64);
  if (deserializeJson(doc, payload, length)) {
    Serial.println("[MQTT] JSON parse error");
    return;
  }

  const char* action = doc["action"];
  if (!action) {
    Serial.println("[MQTT] no action field");
    return;
  }
  Serial.printf("[MQTT] action=%s\n", action);

  if (strcmp(action, "led") == 0) {
    if (doc["value"].is<bool>()) {
      setFlashLed(doc["value"]);
    } else if (doc["value"].is<int>()) {
      setFlashLed(doc["value"] != 0);
    } else {
      return;
    }
    Serial.printf("[MQTT] led %s\n", flashLedOn ? "on" : "off");
    publishMqttTelemetry();
    return;
  }

  if (strcmp(action, "reboot") == 0) {
    Serial.println("[MQTT] reboot");
    delay(300);
    ESP.restart();
    return;
  }

  if (strcmp(action, "capture") == 0) {
    Serial.println("[MQTT] capture");
    captureRequested = true;
    return;
  }

  if (strcmp(action, "ota") == 0) {
    const char* url = doc["url"];
    if (!url || url[0] == '\0') return;
    if (strncmp(url, "http://", 7) != 0 && strncmp(url, "https://", 8) != 0) {
      Serial.println("[MQTT] ota: invalid url scheme");
      return;
    }
    Serial.printf("[MQTT] ota url=%s\n", url);
    queueOtaUpdate(url);
    return;
  }

  if (strcmp(action, "fs_ls") == 0) {
    const char* path = doc["path"] | "/";
    handleFsLs(path);
    return;
  }

  if (strcmp(action, "fs_read") == 0) {
    const char* path = doc["path"];
    if (!path || path[0] == '\0') return;
    handleFsRead(path);
    return;
  }

  if (strcmp(action, "fs_write") == 0) {
    const char* path = doc["path"];
    const char* content = doc["content"];
    if (!path || path[0] == '\0') return;
    handleFsWrite(path, content);
    return;
  }

  if (strcmp(action, "fs_rm") == 0) {
    const char* path = doc["path"];
    if (!path || path[0] == '\0') return;
    handleFsRm(path);
    return;
  }

  Serial.printf("[MQTT] unknown action: %s\n", action);
}

void ensureMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;

  initMqttTopics();

  if (mqttClient.connected()) return;

  if (mqttClient.connect(DEVICE_HOSTNAME, MQTT_USER, MQTT_PASS,
                         topicStatus, 1, true, "{\"status\":\"offline\"}")) {
    mqttClient.publish(topicStatus, "{\"status\":\"online\"}", true);
    mqttClient.subscribe(topicCommand, 1);
    if (mqttClient.publish(topicCapabilities, CAPABILITIES, true)) {
      Serial.printf("[MQTT] capabilities -> %s\n", topicCapabilities);
    } else {
      Serial.printf("[MQTT] capabilities publish FAILED (%u bytes)\n", strlen(CAPABILITIES));
    }
    Serial.println("[MQTT] connected");
    Serial.printf("[MQTT] command  <- %s\n", topicCommand);
    Serial.printf("[MQTT] telemetry -> %s\n", topicTelemetry);
    setStatus("MQTT connected");
    publishMqttTelemetry();
  } else {
    Serial.printf("[MQTT] connect failed, rc=%d\n", mqttClient.state());
  }
}

void publishMqttTelemetry() {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<768> doc;
  doc["uptime"] = millis() / 1000UL;
  doc["heap"] = ESP.getFreeHeap();
  doc["camera_ready"] = cameraReady;
  doc["sd_ready"] = sdReady;
  doc["led"] = flashLedOn;
  doc["sd_free_mb"] = sdFreeBytes() / (1024ULL * 1024ULL);
  doc["photo_index"] = photoIndex;
  doc["capture_count"] = captureCount;
  doc["last_capture_ok"] = lastCaptureOk;
  doc["capture_errors"] = captureErrors;
  doc["sensor"] = sensorName;
  doc["cam_mode"] = camRgbMode ? "rgb565" : "jpeg";
  doc["daylight"] = daylight;
  doc["motion"] = motionNow;
  doc["luma"] = frameLuma;
  doc["frame_errors"] = frameErrors;
  resetBirdCounterIfNewDay();
  doc["bird_visits_today"] = birdVisitsToday;
  if (birdLastAt > 0) {
    doc["bird_last_at"] = (uint32_t)birdLastAt;
    char hhmm[8];
    strftime(hhmm, sizeof(hhmm), "%H:%M", localtime(&birdLastAt));
    doc["bird_last"] = hhmm;
    int birdPhoto = latestBirdPhotoId();
    if (birdPhoto >= 0) doc["bird_photo_id"] = birdPhoto;
  } else {
    doc["bird_last"] = "—";
  }
  addNetworkTelemetry(doc);
  addFirmwareTelemetry(doc);

  if (lastPhotoPath[0] != '\0') {
    doc["last_photo"] = lastPhotoPath;
  }

  if (otaInProgress) {
    doc["ota"] = otaPhase;
    doc["ota_label"] = otaPhaseLabel(otaPhase);
    if (otaProgressPct >= 0) doc["progress"] = otaProgressPct;
    if (otaBytesTotal > 0) {
      doc["ota_bytes"] = otaBytesDone;
      doc["ota_total"] = otaBytesTotal;
    }
    if (otaErrorMsg[0] != '\0') doc["ota_error"] = otaErrorMsg;
  }

  if (WiFi.status() == WL_CONNECTED) {
    char url[80];
    snprintf(url, sizeof(url), "http://%s/latest.jpg",
             WiFi.localIP().toString().c_str());
    doc["last_photo_url"] = url;
  }

  char buf[768];
  size_t n = serializeJson(doc, buf);
  mqttClient.publish(topicTelemetry, buf, n);
}

const char* otaPhaseLabel(const char* phase) {
  if (!phase || !phase[0]) return "—";
  if (strcmp(phase, "preparing") == 0) return "Подготовка";
  if (strcmp(phase, "connecting") == 0) return "Подключение";
  if (strcmp(phase, "downloading") == 0) return "Загрузка";
  if (strcmp(phase, "installing") == 0) return "Запись";
  if (strcmp(phase, "rebooting") == 0) return "Перезагрузка";
  if (strcmp(phase, "failed") == 0) return "Ошибка";
  return phase;
}

int otaPhaseOrder(const char* phase) {
  if (!phase) return 0;
  if (strcmp(phase, "preparing") == 0) return 0;
  if (strcmp(phase, "connecting") == 0) return 1;
  if (strcmp(phase, "downloading") == 0) return 2;
  if (strcmp(phase, "installing") == 0) return 3;
  if (strcmp(phase, "rebooting") == 0) return 4;
  if (strcmp(phase, "failed") == 0) return 2;
  return 0;
}

String htmlOtaStep(const char* label, int step, int currentStep, bool failed, int failedStep) {
  const char* cls = "pending";
  if (failed) {
    if (step == failedStep) cls = "bad";
    else if (step < failedStep) cls = "done";
  } else if (step < currentStep) {
    cls = "done";
  } else if (step == currentStep) {
    cls = "active";
  }

  String html;
  html.reserve(64);
  html += F("<span class=\"ota-step ");
  html += cls;
  html += F("\">");
  html += label;
  html += F("</span>");
  return html;
}

void publishOtaEvent(const char* phase, int progress) {
  ensureMqtt();

  StaticJsonDocument<512> doc;
  doc["ota"] = phase;
  doc["ota_label"] = otaPhaseLabel(phase);
  if (progress >= 0) {
    doc["progress"] = progress;
  } else if (lastOtaProgress >= 0) {
    doc["progress"] = lastOtaProgress;
  } else {
    doc["progress"] = 0;
  }
  if (otaBytesTotal > 0) {
    doc["ota_bytes"] = otaBytesDone;
    doc["ota_total"] = otaBytesTotal;
  }
  if (otaUrl[0] != '\0') doc["ota_url"] = otaUrl;
  if (otaErrorMsg[0] != '\0') doc["ota_error"] = otaErrorMsg;
  if (otaFailedPhase[0] != '\0') doc["ota_failed_at"] = otaFailedPhase;

  unsigned long elapsed = (otaStartedMs > 0) ? (millis() - otaStartedMs) : 0;
  if (elapsed > 0 && otaBytesDone > 0) {
    doc["ota_elapsed_ms"] = elapsed;
    doc["ota_speed_bps"] = (otaBytesDone * 1000UL) / elapsed;
    if (otaBytesTotal > otaBytesDone) {
      unsigned long speed = (otaBytesDone * 1000UL) / elapsed;
      if (speed > 0) {
        doc["ota_eta_sec"] = (otaBytesTotal - otaBytesDone) / speed;
      }
    }
  }

  char buf[512];
  size_t n = serializeJson(doc, buf);
  if (mqttClient.connected()) {
    mqttClient.publish(topicTelemetry, buf, n);
    mqttClient.loop();
  }
}

void setOtaProgress(const char* phase, int progress, size_t current, size_t total) {
  otaInProgress = true;
  strncpy(otaPhase, phase ? phase : "", sizeof(otaPhase) - 1);
  otaPhase[sizeof(otaPhase) - 1] = '\0';
  otaProgressPct = progress;
  otaBytesDone = current;
  otaBytesTotal = total;

  char status[64];
  if (progress >= 0 && total > 0) {
    snprintf(status, sizeof(status), "OTA %s %d%% (%u/%u KB)",
             otaPhaseLabel(otaPhase), progress, (unsigned)(current / 1024), (unsigned)(total / 1024));
  } else if (progress >= 0) {
    snprintf(status, sizeof(status), "OTA %s %d%%", otaPhaseLabel(otaPhase), progress);
  } else {
    snprintf(status, sizeof(status), "OTA %s", otaPhaseLabel(otaPhase));
  }
  setStatus(status);

  if (webServerStarted) {
    statusServer.handleClient();
  }
  mqttClient.loop();
}

void queueOtaUpdate(const char* url) {
  strncpy(otaUrl, url, sizeof(otaUrl) - 1);
  otaUrl[sizeof(otaUrl) - 1] = '\0';
  otaPending = true;
}

void releaseResourcesForOta() {
  if (cameraReady) {
    esp_camera_deinit();
    cameraReady = false;
    Serial.println("[OTA] camera deinit");
  }
}

bool otaUpdateAvailable(size_t* slotSizeOut) {
  const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
  // Huge APP: единственный слот app0 (ota_0), и «следующий» слот — это текущий.
  // Update.begin() тогда падает с «Partition Could Not be Found» уже после скачивания,
  // поэтому ловим это заранее и отдаём понятную ошибку про min_spiffs.
  if (!next || next == esp_ota_get_running_partition()) return false;
  if (slotSizeOut) *slotSizeOut = next->size;
  return true;
}

void logPartitionInfo() {
  size_t slotSize = 0;
  bool otaOk = otaUpdateAvailable(&slotSize);
  Serial.printf("[Partition] sketch=%u heap=%u ota=%s",
                ESP.getSketchSize(), ESP.getFreeHeap(), otaOk ? "yes" : "no");
  if (otaOk) {
    Serial.printf(" slot=%u bytes\n", (unsigned)slotSize);
  } else {
    Serial.println(" (flash USB: PartitionScheme=min_spiffs, not Huge APP)");
  }
}

void performOtaUpdate(const char* url) {
  Serial.printf("[OTA] starting: %s (heap=%u)\n", url, ESP.getFreeHeap());
  ensureWebServer();

  otaErrorMsg[0] = '\0';
  otaFailedPhase[0] = '\0';
  otaStartedMs = millis();
  lastOtaProgress = -1;

  setOtaProgress("preparing", 0, 0, 0);
  ota_mqtt::bind(mqttClient, topicTelemetry, ensureMqtt, lastOtaProgress);
  publishOtaEvent("preparing", 0);

  releaseResourcesForOta();

  size_t otaSlotSize = 0;
  if (!otaUpdateAvailable(&otaSlotSize)) {
    Serial.println("[OTA] no OTA slot (Huge APP — flash USB with min_spiffs)");
    strncpy(otaErrorMsg, "No OTA slot — flash USB: min_spiffs", sizeof(otaErrorMsg) - 1);
    otaErrorMsg[sizeof(otaErrorMsg) - 1] = '\0';
    strncpy(otaFailedPhase, "preparing", sizeof(otaFailedPhase) - 1);
    otaFailedPhase[sizeof(otaFailedPhase) - 1] = '\0';
    setOtaProgress("failed", lastOtaProgress >= 0 ? lastOtaProgress : 0, 0, 0);
    publishOtaEvent("failed", lastOtaProgress >= 0 ? lastOtaProgress : 0);
    setStatus("OTA failed: no partition");
    return;
  }
  Serial.printf("[OTA] OTA slot: %u bytes\n", (unsigned)otaSlotSize);

  WiFi.setSleep(WIFI_PS_NONE);
  Serial.printf("[OTA] heap after cleanup: %u\n", ESP.getFreeHeap());

  setOtaProgress("connecting", 0, 0, 0);
  publishOtaEvent("connecting", 0);

  httpUpdate.rebootOnUpdate(true);
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  httpUpdate.onStart([]() {
    Serial.println("[OTA] start");
    lastOtaProgress = -1;
    setOtaProgress("downloading", 0, 0, 0);
    publishOtaEvent("downloading", 0);
  });

  httpUpdate.onProgress([](size_t current, size_t total) {
    int pct = (total > 0) ? (int)((current * 100UL) / total) : 0;
    const char* phase = (pct >= 100) ? "installing" : "downloading";
    if (pct >= lastOtaProgress + 1 || pct == 100 || lastOtaProgress < 0) {
      lastOtaProgress = pct;
      Serial.printf("[OTA] %s %d%% (%u/%u)\n", phase, pct, (unsigned)current, (unsigned)total);
      setOtaProgress(phase, pct, current, total);
      publishOtaEvent(phase, pct);
    }
  });

  httpUpdate.onEnd([]() {
    Serial.println("[OTA] complete");
    setOtaProgress("rebooting", 100, otaBytesTotal, otaBytesTotal);
    publishOtaEvent("rebooting", 100);
  });

  httpUpdate.onError([](int error) {
    Serial.printf("[OTA] error %d: %s\n", error, httpUpdate.getLastErrorString().c_str());
    strncpy(otaFailedPhase, otaPhase, sizeof(otaFailedPhase) - 1);
    otaFailedPhase[sizeof(otaFailedPhase) - 1] = '\0';
    strncpy(otaErrorMsg, httpUpdate.getLastErrorString().c_str(), sizeof(otaErrorMsg) - 1);
    otaErrorMsg[sizeof(otaErrorMsg) - 1] = '\0';
    setOtaProgress("failed", lastOtaProgress >= 0 ? lastOtaProgress : 0, otaBytesDone, otaBytesTotal);
    publishOtaEvent("failed", lastOtaProgress >= 0 ? lastOtaProgress : 0);
  });

  t_httpUpdate_return ret;
  if (strncmp(url, "https://", 8) == 0) {
    WiFiClientSecure secureClient;
    secureClient.setInsecure();
    secureClient.setTimeout(30000);
    ret = httpUpdate.update(secureClient, url);
  } else {
    WiFiClient client;
    client.setTimeout(30000);
    ret = httpUpdate.update(client, url);
  }

  if (ret != HTTP_UPDATE_OK) {
    Serial.printf("[OTA] failed: %s\n", httpUpdate.getLastErrorString().c_str());
    if (otaFailedPhase[0] == '\0') {
      strncpy(otaFailedPhase, otaPhase, sizeof(otaFailedPhase) - 1);
      otaFailedPhase[sizeof(otaFailedPhase) - 1] = '\0';
    }
    strncpy(otaErrorMsg, httpUpdate.getLastErrorString().c_str(), sizeof(otaErrorMsg) - 1);
    otaErrorMsg[sizeof(otaErrorMsg) - 1] = '\0';
    setOtaProgress("failed", lastOtaProgress >= 0 ? lastOtaProgress : 0, otaBytesDone, otaBytesTotal);
    publishOtaEvent("failed", lastOtaProgress >= 0 ? lastOtaProgress : 0);
  }
}

// =====================
// HTTP
// =====================
String htmlRow(const char* label, const String& value, const char* valueClass) {
  String row = "<tr><td class=\"k\">";
  row += label;
  row += "</td><td";
  if (valueClass[0] != '\0') {
    row += " class=\"";
    row += valueClass;
    row += "\"";
  }
  row += ">";
  row += value;
  row += "</td></tr>";
  return row;
}

void sendJpegFile(const char* path) {
  if (!sdReady || path[0] == '\0' || !SD_MMC.exists(path)) {
    statusServer.send(404, "text/plain", "Photo not found");
    return;
  }

  File file = SD_MMC.open(path, FILE_READ);
  if (!file) {
    statusServer.send(500, "text/plain", "Open failed");
    return;
  }

  statusServer.sendHeader("Cache-Control", "public, max-age=3600");
  statusServer.streamFile(file, "image/jpeg");
  file.close();
}

// Живой кадр из RAM (обновляется раз в секунду днём). Шлюз кэширует его сам.
void handleLatestPhoto() {
  if (!frameBuf || frameLen == 0) {
    statusServer.send(503, "text/plain", "No frame yet");
    return;
  }
  statusServer.sendHeader("Cache-Control", "no-store");
  statusServer.sendHeader("X-Frame-Age-Ms", String(millis() - frameAtMs));
  statusServer.sendHeader("X-Daylight", daylight ? "1" : "0");
  statusServer.setContentLength(frameLen);
  statusServer.send(200, "image/jpeg", "");
  statusServer.client().write(frameBuf, frameLen);
}

void handlePhotoById() {
  if (!statusServer.hasArg("id")) {
    statusServer.send(400, "text/plain", "Missing id");
    return;
  }

  int id = statusServer.arg("id").toInt();
  if (id < 0 || id >= MAX_PHOTOS) {
    statusServer.send(400, "text/plain", "Invalid id");
    return;
  }

  String path = photoPathForIndex((uint16_t)id);
  sendJpegFile(path.c_str());
}

// Журнал снимков с птицами, новые первыми: [{"id":123,"at":1790000000}, ...]
// Сами кадры — /photo?id=<id> с SD.
void handleBirdsJson() {
  StaticJsonDocument<1536> doc;
  doc["visits_today"] = birdVisitsToday;
  doc["daylight"] = daylight;
  if (birdLastAt > 0) doc["last_at"] = (uint32_t)birdLastAt;
  JsonArray shots = doc.createNestedArray("shots");
  for (uint8_t i = 0; i < birdLogCount; i++) {
    const BirdShot& b = birdLog[(birdLogHead + BIRD_LOG_SIZE - 1 - i) % BIRD_LOG_SIZE];
    JsonObject o = shots.createNestedObject();
    o["id"] = b.photoId;
    o["at"] = b.at;
  }
  String out;
  serializeJson(doc, out);
  statusServer.sendHeader("Cache-Control", "no-store");
  statusServer.send(200, "application/json; charset=utf-8", out);
}

void handleOtaStatus() {
  StaticJsonDocument<512> doc;
  doc["in_progress"] = otaInProgress;
  doc["phase"] = otaPhase;
  doc["phase_label"] = otaPhaseLabel(otaPhase);
  if (otaProgressPct >= 0) doc["progress"] = otaProgressPct;
  if (otaBytesTotal > 0) {
    doc["bytes"] = otaBytesDone;
    doc["total"] = otaBytesTotal;
  }
  if (otaUrl[0] != '\0') doc["url"] = otaUrl;
  if (otaErrorMsg[0] != '\0') doc["error"] = otaErrorMsg;
  if (otaFailedPhase[0] != '\0') doc["failed_at"] = otaFailedPhase;

  unsigned long elapsed = (otaStartedMs > 0) ? (millis() - otaStartedMs) : 0;
  if (elapsed > 0) doc["elapsed_ms"] = elapsed;
  if (elapsed > 0 && otaBytesDone > 0) {
    unsigned long speed = (otaBytesDone * 1000UL) / elapsed;
    doc["speed_bps"] = speed;
    if (otaBytesTotal > otaBytesDone && speed > 0) {
      doc["eta_sec"] = (otaBytesTotal - otaBytesDone) / speed;
    }
  }

  String out;
  serializeJson(doc, out);
  statusServer.send(200, "application/json; charset=utf-8", out);
}

void handleStatusPage() {
  String html;
  html.reserve(6144);

  html += F("<!DOCTYPE html><html lang=\"ru\"><head><meta charset=\"utf-8\">"
            "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
  if (otaInProgress) {
    html += F("<meta http-equiv=\"refresh\" content=\"1\">");
  } else {
    html += F("<meta http-equiv=\"refresh\" content=\"10\">");
  }
  html += F("<title>");
  html += DEVICE_HOSTNAME;
  html += F("</title><style>"
            "body{font-family:system-ui,sans-serif;background:#0a0c12;color:#e8eaef;margin:0;padding:16px}"
            "h1{font-size:1.25rem;margin:0 0 4px}p.sub{color:#96a0b4;font-size:.85rem;margin:0 0 16px}"
            "section{background:#161a26;border-radius:10px;padding:12px 14px;margin-bottom:12px}"
            "h2{font-size:.75rem;text-transform:uppercase;letter-spacing:.06em;color:#7b8499;margin:0 0 10px}"
            "table{width:100%;border-collapse:collapse}td{padding:5px 0;border-bottom:1px solid #222836;font-size:.9rem}"
            "td.k{color:#96a0b4;width:44%}.ok{color:#3ecf8e}.bad{color:#ff5c6c}.warn{color:#ffb020}"
            "img.preview{width:100%;max-width:800px;border-radius:8px;background:#000}"
            ".ota-steps{display:flex;flex-wrap:wrap;gap:6px;margin:0 0 12px;font-size:.75rem}"
            ".ota-step{padding:4px 8px;border-radius:999px;background:#222836;color:#7b8499}"
            ".ota-step.done{background:#1a3d2e;color:#3ecf8e}"
            ".ota-step.active{background:#2a3f7a;color:#8eb4ff}"
            ".ota-step.bad{background:#4a1f28;color:#ff5c6c}"
            ".ota-bar{height:14px;background:#222836;border-radius:7px;overflow:hidden}"
            ".ota-fill{height:100%;background:linear-gradient(90deg,#2864ff,#3ecf8e);border-radius:7px;transition:width .3s}"
            ".ota-label{font-size:.85rem;color:#c8d0e0;margin:8px 0 0}"
            ".ota-pct{font-size:1.75rem;font-weight:600;margin:4px 0 8px}"
            ".ota-url{font-size:.75rem;color:#7b8499;word-break:break-all;margin-top:8px}"
            "</style></head><body><h1>");
  html += DEVICE_HOSTNAME;
  if (otaInProgress) {
    html += F("</h1><p class=\"sub\">ESP32-CAM · OTA обновление · автообновление 1 с · <a href=\"/ota.json\" style=\"color:#8eb4ff\">JSON</a></p>");
  } else {
    html += F("</h1><p class=\"sub\">ESP32-CAM · автообновление 10 с</p>");
  }

  if (otaInProgress) {
    bool otaFailed = strcmp(otaPhase, "failed") == 0;
    int currentStep = otaFailed ? otaPhaseOrder(otaFailedPhase) : otaPhaseOrder(otaPhase);
    int failedStep = otaFailed ? currentStep : -1;
    int pct = (otaProgressPct >= 0) ? otaProgressPct : 0;
    const char* phaseClass = otaFailed ? "bad" :
                             (strcmp(otaPhase, "rebooting") == 0) ? "ok" : "warn";

    html += F("<section><h2>OTA обновление</h2><div class=\"ota-steps\">");
    html += htmlOtaStep("Подготовка", 0, currentStep, otaFailed, failedStep);
    html += htmlOtaStep("Подключение", 1, currentStep, otaFailed, failedStep);
    html += htmlOtaStep("Загрузка", 2, currentStep, otaFailed, failedStep);
    html += htmlOtaStep("Запись", 3, currentStep, otaFailed, failedStep);
    html += htmlOtaStep("Перезагрузка", 4, currentStep, otaFailed, failedStep);
    html += F("</div><p class=\"ota-pct ");
    html += phaseClass;
    html += F("\">");
    if (otaProgressPct >= 0) {
      html += String(otaProgressPct);
      html += '%';
    } else {
      html += otaPhaseLabel(otaPhase);
    }
    html += F("</p><div class=\"ota-bar\"><div class=\"ota-fill\" style=\"width:");
    html += String(pct);
    html += F("%\"></div></div>");

    html += F("<table style=\"margin-top:12px\">");
    html += htmlRow("Этап", otaPhaseLabel(otaPhase), phaseClass);
    if (otaBytesTotal > 0) {
      html += htmlRow("Загружено",
                      String(otaBytesDone / 1024) + " / " + String(otaBytesTotal / 1024) + " KB (" +
                      String((otaBytesDone * 100UL) / otaBytesTotal) + "%)");
    }
    unsigned long elapsed = (otaStartedMs > 0) ? (millis() - otaStartedMs) : 0;
    if (elapsed > 1000 && otaBytesDone > 0) {
      unsigned long speed = (otaBytesDone * 1000UL) / elapsed;
      html += htmlRow("Скорость", String(speed / 1024) + " KB/s");
      if (otaBytesTotal > otaBytesDone && speed > 0) {
        html += htmlRow("Осталось", String((otaBytesTotal - otaBytesDone) / speed) + " с");
      }
      html += htmlRow("Прошло", String(elapsed / 1000) + " с");
    }
    if (otaFailed && otaFailedPhase[0] != '\0') {
      html += htmlRow("Сбой на этапе", otaPhaseLabel(otaFailedPhase), "bad");
    }
    if (otaErrorMsg[0] != '\0') {
      html += htmlRow("Ошибка", otaErrorMsg, "bad");
    }
    html += F("</table>");
    if (otaUrl[0] != '\0') {
      html += F("<p class=\"ota-url\">");
      html += otaUrl;
      html += F("</p>");
    }
    html += F("</section>");
  }

  html += F("<section><h2>Последний кадр</h2>");
  if (frameLen > 0) {
    html += F("<img class=\"preview\" src=\"/latest.jpg\" alt=\"latest frame\">");
  } else {
    html += F("<p class=\"sub\">Кадров пока нет</p>");
  }
  html += F("</section>");

  html += F("<section><h2>Сеть</h2><table>");
  bool wifiOk = WiFi.status() == WL_CONNECTED;
  html += htmlRow("Wi-Fi", wifiOk ? "Подключено" : "Нет связи", wifiOk ? "ok" : "bad");
  if (wifiOk) {
    html += htmlRow("IP", WiFi.localIP().toString());
    html += htmlRow("Hostname", WiFi.getHostname());
    html += htmlRow("RSSI", String(WiFi.RSSI()) + " dBm");
    html += htmlRow("SSID", WiFi.SSID());
  }
  html += htmlRow("Uptime", formatUptime(millis()));
  html += htmlRow("Время", formatDateTime());
  html += htmlRow("Статус", statusLine);
  html += htmlRow("MQTT", mqttClient.connected() ? "Подключено" : "Нет связи",
                  mqttClient.connected() ? "ok" : "bad");
  html += F("</table></section>");

  html += F("<section><h2>Камера</h2><table>");
  html += htmlRow("Камера", cameraReady ? "OK" : "Ошибка", cameraReady ? "ok" : "bad");
  html += htmlRow("SD-карта", sdReady ? "OK" : "Ошибка", sdReady ? "ok" : "bad");
  if (sdReady) {
    html += htmlRow("Свободно", String(sdFreeBytes() / (1024ULL * 1024ULL)) + " MB");
  }
  html += htmlRow("Сенсор", String(sensorName) + (camRgbMode ? " · RGB565 VGA" : " · JPEG SVGA"));
  html += htmlRow("Светло", daylight ? "Да" : "Нет (ночной режим)", daylight ? "ok" : "warn");
  html += htmlRow("Яркость", String(frameLuma));
  html += htmlRow("Движение", String(motionPermille) + "‰" + (motionNow ? " — птица" : ""),
                  motionNow ? "ok" : "");
  html += htmlRow("Визитов сегодня", String(birdVisitsToday));
  if (birdLastAt > 0) {
    char when[20];
    strftime(when, sizeof(when), "%d.%m %H:%M:%S", localtime(&birdLastAt));
    html += htmlRow("Последний визит", when);
  }
  html += htmlRow("Снимков на SD", String(captureCount));
  html += htmlRow("Ошибки", String(captureErrors),
                  captureErrors == 0 ? "ok" : "warn");
  html += htmlRow("Индекс", String(photoIndex) + "/" + String(MAX_PHOTOS));
  if (lastPhotoPath[0] != '\0') {
    html += htmlRow("Файл", lastPhotoPath);
  }
  html += htmlRow("Последний", lastCaptureOk ? "OK" : "Ошибка",
                  lastCaptureOk ? "ok" : "bad");
  html += F("</table></section>");

  html += F("<section><h2>Система</h2><table>");
  html += htmlRow("FW", FW_VERSION);
  html += htmlRow("Сборка", String(FW_BUILD_DATE) + " " + FW_BUILD_TIME);
  html += htmlRow("Heap", String(ESP.getFreeHeap()) + " B");
  html += htmlRow("Вспышка", flashLedOn ? "Вкл" : "Выкл");
  html += F("</table></section></body></html>");

  statusServer.send(200, "text/html; charset=utf-8", html);
}

void startWebServer() {
  if (webServerStarted) return;

  statusServer.on("/", handleStatusPage);
  statusServer.on("/ota.json", handleOtaStatus);
  statusServer.on("/latest.jpg", handleLatestPhoto);
  statusServer.on("/photo", handlePhotoById);
  statusServer.on("/birds.json", handleBirdsJson);
  statusServer.begin();
  webServerStarted = true;

  Serial.printf("[Web] http://%s/ (%s.local)\n",
                WiFi.localIP().toString().c_str(), DEVICE_HOSTNAME);
}

void ensureWebServer() {
  if (WiFi.status() == WL_CONNECTED && !webServerStarted) {
    startWebServer();
  }
}

// =====================
// Setup / loop
// =====================
void setup() {
  pinMode(LED_FLASH_PIN, OUTPUT);
  setFlashLed(false);

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32-CAM birdfeeder");
  logFirmwareInfo("esp32-bird-cam");
  logPartitionInfo();

  littleFsReady = LittleFS.begin(true, "/littlefs", 10, "spiffs");
  if (!littleFsReady) {
    littleFsReady = LittleFS.begin(true);
  }
  if (!littleFsReady) {
    Serial.println("[LittleFS] mount FAILED");
    setStatus("FS error");
  } else {
    Serial.printf("[LittleFS] mounted, used=%u total=%u\n",
                  LittleFS.usedBytes(), LittleFS.totalBytes());
  }

  loadBirdLog();

  setStatus("Init camera");
  if (!initCamera()) {
    setStatus("Camera error");
  }

  setStatus("Init SD");
  if (!initSdCard()) {
    setStatus("SD error");
  }

  connectWiFi();
  ensureWebServer();
  ensureMqtt();

  if (WiFi.status() == WL_CONNECTED) {
    syncTime();
  }

  if (cameraReady && grabFrame()) {
    lastFrameTime = millis();
    analyzeFrame(lastFrameTime);
  }
}

void loop() {
  if (otaPending) {
    otaPending = false;
    performOtaUpdate(otaUrl);
    return;
  }

  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiRetry >= WIFI_RETRY_INTERVAL) {
      lastWifiRetry = now;
      connectWiFi();
    }
  } else {
    ensureWebServer();
    ensureMqtt();
    mqttClient.loop();

    if (now - lastMqttTelemetry >= MQTT_TELEMETRY_INTERVAL) {
      lastMqttTelemetry = now;
      publishMqttTelemetry();
    }
  }

  statusServer.handleClient();

  if (!cameraReady && now - lastFrameTime >= FRAME_INTERVAL_NIGHT_MS) {
    lastFrameTime = now;
    initCamera();
  }

  unsigned long frameInterval = daylight ? FRAME_INTERVAL_DAY_MS : FRAME_INTERVAL_NIGHT_MS;
  if (cameraReady && (captureRequested || now - lastFrameTime >= frameInterval)) {
    lastFrameTime = now;
    if (grabFrame()) {
      analyzeFrame(now);
    }
  }

  if (captureRequested) {
    captureRequested = false;
    if (!sdReady) {
      initSdCard();
    }
    if (cameraReady && sdReady) {
      captureAndSavePhoto();
    }
  }

  delay(10);
}
