#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <time.h>

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

#include "secrets.h"

// =====================
// AI-Thinker ESP32-CAM pins
// =====================
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

const uint16_t MAX_PHOTOS = 4800;
const char* PHOTOS_DIR = "/photos";

// =====================
// State
// =====================
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebServer statusServer(80);

char topicStatus[64];
char topicTelemetry[64];
char topicCommand[64];
bool mqttTopicsReady = false;
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

// =====================
// Forward declarations
// =====================
void setStatus(const char* msg);
void setFlashLed(bool on);
bool initCamera();
bool initSdCard();
void restorePhotoIndex();
bool captureAndSavePhoto();
void connectWiFi();
void syncTime();
void initMqttTopics();
void ensureMqtt();
void publishMqttTelemetry();
void handleMqttCommand(char* topic, byte* payload, unsigned int length);
void startWebServer();
void ensureWebServer();
void handleStatusPage();
void handleLatestPhoto();
void handlePhotoById();
String formatUptime(unsigned long ms);
String formatDateTime();
String htmlRow(const char* label, const String& value, const char* valueClass = "");

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
  config.frame_size = FRAMESIZE_SVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
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
    s->set_hmirror(s, 1);
    s->set_vflip(s, 1);    // кадр не вверх ногами
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

void restorePhotoIndex() {
  if (!sdReady) return;

  File root = SD_MMC.open(PHOTOS_DIR);
  if (!root || !root.isDirectory()) {
    if (root) root.close();
    photoIndex = 0;
    return;
  }

  uint16_t maxIndex = 0;
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      const char* name = file.name();
      const char* slash = strrchr(name, '/');
      const char* base = slash ? slash + 1 : name;
      unsigned int idx = 0;
      if (sscanf(base, "%5u.jpg", &idx) == 1 && idx > maxIndex) {
        maxIndex = (uint16_t)idx;
      }
    }
    file.close();
    file = root.openNextFile();
  }
  root.close();

  photoIndex = (maxIndex + 1) % MAX_PHOTOS;
  if (maxIndex < MAX_PHOTOS) {
    String lastPath = photoPathForIndex(maxIndex);
    if (SD_MMC.exists(lastPath)) {
      strncpy(lastPhotoPath, lastPath.c_str(), sizeof(lastPhotoPath) - 1);
      lastPhotoPath[sizeof(lastPhotoPath) - 1] = '\0';
      lastCaptureOk = true;
    }
  }
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

  size_t written = file.write(fb->buf, fb->len);
  file.close();
  esp_camera_fb_return(fb);

  if (written != fb->len) {
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

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(handleMqttCommand);
  mqttClient.setBufferSize(512);
  mqttTopicsReady = true;
}

void handleMqttCommand(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) return;

  const char* action = doc["action"];
  if (!action) return;

  if (strcmp(action, "led") == 0) {
    if (!doc["value"].is<bool>()) return;
    bool on = doc["value"];
    Serial.printf("[MQTT] led %s\n", on ? "on" : "off");
    setFlashLed(on);
    return;
  }

  if (strcmp(action, "reboot") == 0) {
    Serial.println("[MQTT] reboot");
    delay(300);
    ESP.restart();
  }

  if (strcmp(action, "capture") == 0) {
    Serial.println("[MQTT] capture");
    captureRequested = true;
  }
}

void ensureMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;

  initMqttTopics();

  if (mqttClient.connected()) return;

  if (mqttClient.connect(DEVICE_HOSTNAME, MQTT_USER, MQTT_PASS,
                         topicStatus, 1, true, "{\"status\":\"offline\"}")) {
    mqttClient.publish(topicStatus, "{\"status\":\"online\"}", true);
    mqttClient.subscribe(topicCommand, 1);
    Serial.println("[MQTT] connected");
    setStatus("MQTT connected");
  } else {
    Serial.printf("[MQTT] connect failed, rc=%d\n", mqttClient.state());
  }
}

void publishMqttTelemetry() {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<512> doc;
  doc["uptime"] = millis() / 1000UL;
  doc["rssi"] = WiFi.RSSI();
  doc["heap"] = ESP.getFreeHeap();
  doc["camera_ready"] = cameraReady;
  doc["sd_ready"] = sdReady;
  doc["led"] = flashLedOn;
  doc["sd_free_mb"] = sdFreeBytes() / (1024ULL * 1024ULL);
  doc["photo_index"] = photoIndex;
  doc["capture_count"] = captureCount;
  doc["last_capture_ok"] = lastCaptureOk;
  doc["capture_errors"] = captureErrors;

  if (lastPhotoPath[0] != '\0') {
    doc["last_photo"] = lastPhotoPath;
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

void handleStatusPage() {
  String html;
  html.reserve(4096);

  html += F("<!DOCTYPE html><html lang=\"ru\"><head><meta charset=\"utf-8\">"
            "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
            "<meta http-equiv=\"refresh\" content=\"10\">"
            "<title>");
  html += DEVICE_HOSTNAME;
  html += F("</title><style>"
            "body{font-family:system-ui,sans-serif;background:#0a0c12;color:#e8eaef;margin:0;padding:16px}"
            "h1{font-size:1.25rem;margin:0 0 4px}p.sub{color:#96a0b4;font-size:.85rem;margin:0 0 16px}"
            "section{background:#161a26;border-radius:10px;padding:12px 14px;margin-bottom:12px}"
            "h2{font-size:.75rem;text-transform:uppercase;letter-spacing:.06em;color:#7b8499;margin:0 0 10px}"
            "table{width:100%;border-collapse:collapse}td{padding:5px 0;border-bottom:1px solid #222836;font-size:.9rem}"
            "td.k{color:#96a0b4;width:44%}.ok{color:#3ecf8e}.bad{color:#ff5c6c}.warn{color:#ffb020}"
            "img.preview{width:100%;max-width:800px;border-radius:8px;background:#000}"
            "</style></head><body><h1>");
  html += DEVICE_HOSTNAME;
  html += F("</h1><p class=\"sub\">ESP32-CAM · автообновление 10 с</p>");

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
  html += htmlRow("Heap", String(ESP.getFreeHeap()) + " B");
  html += htmlRow("Вспышка", flashLedOn ? "Вкл" : "Выкл");
  html += F("</table></section></body></html>");

  statusServer.send(200, "text/html; charset=utf-8", html);
}

void startWebServer() {
  if (webServerStarted) return;

  statusServer.on("/", handleStatusPage);
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
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32-CAM photo station");

  pinMode(LED_FLASH_PIN, OUTPUT);
  setFlashLed(false);

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
