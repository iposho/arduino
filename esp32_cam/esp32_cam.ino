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
const unsigned long CAPTURE_INTERVAL_MS     = 15UL * 1000UL;
const unsigned long MQTT_TELEMETRY_INTERVAL = 10UL * 1000UL;
const unsigned long WIFI_RETRY_INTERVAL     = 30UL * 1000UL;
const unsigned long NTP_GMT_OFFSET_SEC      = 4UL * 3600UL;
const uint8_t JPEG_SAVE_QUALITY             = 8;

#define MQTT_BUFFER_SIZE  4096
#define FS_READ_MAX_BYTES 2800
#define FS_LS_MAX_ENTRIES 32

const uint16_t MAX_PHOTOS = 4800;
const char* PHOTOS_DIR = "/photos";
const char* PHOTO_INDEX_FILE = "/photo_index.dat";

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
      "description": "Сделать снимок сейчас"
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
    { "key": "last_capture_ok", "label": "Последний снимок", "icon": "camera", "format": "boolean", "group": "Камера", "dashboard": true, "order": 12 },
    { "key": "capture_errors", "label": "Ошибки съёмки", "icon": "activity", "format": "number", "group": "Камера", "order": 13 },
    { "key": "led", "label": "Вспышка", "icon": "zap", "format": "boolean", "group": "Камера", "order": 14 },
    { "key": "fw_version", "label": "Версия прошивки", "icon": "cpu", "group": "Система", "order": 20 }
  ],
  "dashboard": {
    "summary": ["last_capture_ok", "sd_free_mb", "rssi", "uptime"],
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

unsigned long lastCaptureTime = 0;
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
bool encodePhotoJpeg(camera_fb_t* fb, uint8_t** outBuf, size_t* outLen);
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

bool encodePhotoJpeg(camera_fb_t* fb, uint8_t** outBuf, size_t* outLen) {
  if (!fb || fb->format != PIXFORMAT_RGB565 || !outBuf || !outLen) return false;

  char ts[24];
  formatCaptureTimestamp(ts, sizeof(ts));
  if (ts[0] != '\0') {
    drawTimestamp565(fb->buf, fb->width, fb->height, ts);
  }

  return frame2jpg(fb, JPEG_SAVE_QUALITY, outBuf, outLen);
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
  config.frame_size = FRAMESIZE_HD;
  config.pixel_format = PIXFORMAT_RGB565;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = JPEG_SAVE_QUALITY;
  config.fb_count = 1;

  if (psramFound()) {
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[Camera] init failed: 0x%x\n", err);
    cameraReady = false;
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 1);
  }

  cameraReady = true;
  Serial.println("[Camera] OK");
  return true;
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
  if (!cameraReady || !sdReady) {
    lastCaptureOk = false;
    captureErrors++;
    return false;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[Capture] framebuffer failed");
    lastCaptureOk = false;
    captureErrors++;
    return false;
  }

  String path = photoPathForIndex(photoIndex);
  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file) {
    Serial.printf("[Capture] open %s failed\n", path.c_str());
    esp_camera_fb_return(fb);
    lastCaptureOk = false;
    captureErrors++;
    return false;
  }

  const uint8_t* writeBuf = nullptr;
  size_t writeLen = 0;
  uint8_t* jpegBuf = nullptr;

  if (!encodePhotoJpeg(fb, &jpegBuf, &writeLen)) {
    Serial.println("[Capture] jpeg encode failed");
    esp_camera_fb_return(fb);
    file.close();
    lastCaptureOk = false;
    captureErrors++;
    return false;
  }
  writeBuf = jpegBuf;

  size_t written = file.write(writeBuf, writeLen);
  if (jpegBuf) free(jpegBuf);
  file.close();
  esp_camera_fb_return(fb);

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
  } else {
    Serial.printf("[MQTT] connect failed, rc=%d\n", mqttClient.state());
  }
}

void publishMqttTelemetry() {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<512> doc;
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

  char buf[512];
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
  if (!next) return false;
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

  statusServer.sendHeader("Cache-Control", "no-cache");
  statusServer.streamFile(file, "image/jpeg");
  file.close();
}

void handleLatestPhoto() {
  sendJpegFile(lastPhotoPath);
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
  if (lastPhotoPath[0] != '\0') {
    html += F("<img class=\"preview\" src=\"/latest.jpg\" alt=\"latest photo\">");
  } else {
    html += F("<p class=\"sub\">Снимков пока нет</p>");
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
  html += htmlRow("Интервал", "15 с");
  html += htmlRow("Снимков", String(captureCount));
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
  Serial.println("ESP32-CAM photo station");
  logFirmwareInfo("esp32-cam");
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

  if (cameraReady && sdReady) {
    captureAndSavePhoto();
    lastCaptureTime = millis();
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

  bool shouldCapture = captureRequested;
  if (!shouldCapture && cameraReady && sdReady) {
    if (lastCaptureTime == 0 || now - lastCaptureTime >= CAPTURE_INTERVAL_MS) {
      shouldCapture = true;
    }
  }

  if (shouldCapture) {
    captureRequested = false;
    if (!cameraReady) {
      initCamera();
    }
    if (!sdReady) {
      initSdCard();
    }
    if (cameraReady && sdReady) {
      captureAndSavePhoto();
      lastCaptureTime = now;
    }
  }

  delay(10);
}
