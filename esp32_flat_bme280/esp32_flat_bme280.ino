#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <time.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <ScioSense_ENS160.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// =====================
// Секреты — вынесены в secrets.h (.gitignore)
// =====================
#include "secrets.h"
#include "firmware_info.h"
#include "../include/ota_mqtt.h"

// MQTT-топики (DEVICE_HOSTNAME из secrets.h):
//   devices/<hostname>/status       — online/offline (LWT)
//   devices/<hostname>/telemetry    — периодическая телеметрия
//   devices/<hostname>/command      — JSON-команды
//   devices/<hostname>/capabilities — retained JSON c описанием команд

// =====================
// TFT SPI pins
// =====================
#define TFT_CS    5
#define TFT_DC    2
#define TFT_RST   4
#define TFT_SCLK  18
#define TFT_MOSI  23

// =====================
// BME280 I2C pins
// =====================
#define BME_SDA_PIN 21
#define BME_SCL_PIN 22
#define BME280_ADDRESS 0x76

// ENS160: ADDR high = 0x53 (как в тесте). Если не найден — ENS160_I2CADDR_0 (0x52).
#ifndef ENS160_I2C_ADDRESS
#define ENS160_I2C_ADDRESS ENS160_I2CADDR_1
#endif

// =====================
// Joystick pins
// =====================
#define JOY_X_PIN   34
#define JOY_Y_PIN   35
#define JOY_SW_PIN  32

// Тактовая кнопка: вкл/выкл вывеску «фламинго» по MQTT.
//   нога 1 → GPIO 33
//   нога 2 → GND
// Внутренний pull-up: отпущена = HIGH, нажата = LOW.
#define FLAMINGO_BTN_PIN  33

// Подробные логи кнопки в Serial Monitor (115200). Выключите после отладки.
#define FLAMINGO_BTN_DEBUG  1
#define FLAMINGO_BTN_LOG_MS 2000UL

#ifndef FLAMINGO_HOSTNAME
#define FLAMINGO_HOSTNAME "esp32-flamingo"
#endif

// Relay: flat пишет в свой namespace (ACL часто блокирует чужие command-топики).
// Топик строится в initMqttTopics(): devices/<DEVICE_HOSTNAME>/out/flamingo
#ifndef FLAMINGO_DIRECT_COMMAND
#define FLAMINGO_DIRECT_COMMAND 0
#endif

#define JOY_LEFT_THRESHOLD   1000
#define JOY_RIGHT_THRESHOLD  3000
#define JOY_DOWN_THRESHOLD   3000

// Встроенный LED на большинстве ESP32 DevKit (GPIO 2, active LOW). По умолчанию выкл;
// включается только командой led/pin_write (индикация загрузки — кратко, потом off).
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// =====================
// Intervals
// =====================
const unsigned long SEND_INTERVAL         = 10UL * 60UL * 1000UL;
const unsigned long FETCH_INTERVAL        = 10UL * 60UL * 1000UL;
const unsigned long SENSOR_CHECK_INTERVAL = 2000UL;
const unsigned long MQTT_TELEMETRY_INTERVAL = 10UL * 1000UL;

#define MQTT_BUFFER_SIZE  4096
#define FS_READ_MAX_BYTES 2800
#define FS_LS_MAX_ENTRIES 32

const unsigned long TIME_SHOW_MS = 9000UL;
const unsigned long INFO_SHOW_MS = 7000UL;

// Авто-переключение экрана при превышении (единожды до действия пользователя).
#define ALERT_ECO2_PPM        1000
#define ALERT_ECO2_CLEAR_PPM   950
#define ALERT_TVOC_PPB         500
#define ALERT_TVOC_CLEAR_PPB   450
#define ALERT_AQI_VALUE        100
#define ALERT_AQI_CLEAR         90

// =====================
// TFT
// =====================
Adafruit_ST7735 tft = Adafruit_ST7735(
  TFT_CS,
  TFT_DC,
  TFT_MOSI,
  TFT_SCLK,
  TFT_RST
);

// =====================
// BME280
// =====================
Adafruit_BME280 bme;
bool bmeReady = false;

// =====================
// AHT21 (на модуле ENS160 — компенсация T/RH для газового сенсора)
// =====================
Adafruit_AHTX0 aht;
bool ahtReady = false;

// =====================
// ENS160 (eCO2 / TVOC)
// =====================
ScioSense_ENS160 ens160(ENS160_I2C_ADDRESS);
bool ens160Ready = false;
bool ensDataValid = false;
uint16_t lastEco2 = 0;
uint16_t lastTvoc = 0;
uint8_t lastEnsAqi = 0;
float filteredEco2 = -999.0f;
float filteredTvoc = -999.0f;
const float ENS_EMA_ALPHA = 0.12f;
int lastDisplayedEco2 = -1;
int lastDisplayedTvoc = -1;

// =====================
// Screens
// =====================
enum Screen {
  SCREEN_HOME,
  SCREEN_OUTDOOR,
  SCREEN_INDOOR,
  SCREEN_AQI,
  SCREEN_COUNT
};

Screen currentScreen = SCREEN_HOME;

// =====================
// UI colors
// =====================
uint16_t COLOR_BG;
uint16_t COLOR_CARD;
uint16_t COLOR_TEXT;
uint16_t COLOR_MUTED;
uint16_t COLOR_BLUE;
uint16_t COLOR_GREEN;
uint16_t COLOR_ORANGE;
uint16_t COLOR_RED;
uint16_t COLOR_MAGENTA;
uint16_t COLOR_CYAN;
uint16_t COLOR_YELLOW;

// =====================
// Outdoor weather & PM data
// =====================
bool outDataValid = false;
float outTemp = 0;
float outHumidity = 0;
float outPressure = 0;
float outPm25 = 0;
float outPm10 = 0;

// =====================
// AQI Calculated
// =====================
bool aqiDataValid = false;
int aqiValue = 0;

// =====================
// Temporary screens
// =====================
bool showingTimeScreen = false;
unsigned long timeScreenStart = 0;

bool showingInfoScreen = false;
unsigned long infoScreenStart = 0;

bool alertLock = false;
bool indoorAlertEpisode = false;
bool aqiAlertEpisode = false;

bool boardLedOn = false;

// =====================
// Button state
// =====================
bool lastButtonState = HIGH;
unsigned long lastButtonTime = 0;
const unsigned long BUTTON_DEBOUNCE = 120;

bool lastFlamingoBtnState = HIGH;
unsigned long lastFlamingoBtnTime = 0;
unsigned long lastFlamingoBtnLog = 0;
const unsigned long FLAMINGO_BTN_DEBOUNCE = 200;
bool flamingoSignOn = false;
bool flamingoStateKnown = false;
bool flamingoTelemSeen = false;

// =====================
// Joystick state
// =====================
bool joyWasCentered = true;
unsigned long lastJoyTime = 0;
const unsigned long JOY_DEBOUNCE = 300;

// =====================
// Timers
// =====================
unsigned long lastSendTime = 0;
unsigned long lastFetchTime = 0;
unsigned long lastSensorCheckTime = 0;

// =====================
// Filtered & Last displayed home values
// =====================
float filteredTemp = -999.0;
float filteredHum  = -999.0;
float filteredPres = -999.0;
const float EMA_ALPHA = 0.1;

int lastDisplayedTemp = -999;
int lastDisplayedHumidity = -999;
int lastDisplayedPressure = -999;

// =====================
// Status
// =====================
char statusLine[40] = "Starting...";

// =====================
// MQTT (шлюз esp32.kuzyak.in)
// =====================
WiFiClient mqttNet;
PubSubClient mqttClient(mqttNet);

char topicStatus[64];
char topicTelemetry[64];
char topicCommand[64];
char topicCapabilities[64];
char topicFlamingoCommand[64];
char topicFlamingoTelemetry[64];
char topicFlamingoRelay[64];
bool mqttTopicsReady = false;

const char *CAPABILITIES = R"CAP({
  "commands": [
    {
      "action": "led",
      "title": "Светодиод",
      "type": "toggle",
      "icon": "lightbulb",
      "description": "Встроенный светодиод на GPIO (LED_BUILTIN)"
    },
    {
      "action": "status",
      "title": "System info",
      "type": "trigger",
      "icon": "info",
      "description": "Показать экран System info"
    },
    {
      "action": "refresh",
      "title": "Обновить улицу",
      "type": "trigger",
      "icon": "refresh-cw",
      "description": "Принудительно обновить данные с балкона"
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
    { "key": "temperature", "label": "Температура", "icon": "thermometer", "format": "temperature", "unit": "°C", "group": "Комната", "dashboard": true, "order": 10 },
    { "key": "humidity", "label": "Влажность", "icon": "droplets", "format": "percent", "group": "Комната", "dashboard": true, "order": 11 },
    { "key": "pressure", "label": "Давление", "icon": "gauge", "format": "number", "unit": "мм рт.ст.", "group": "Комната", "order": 12 },
    { "key": "eco2", "label": "eCO₂", "icon": "wind", "format": "number", "unit": "ppm", "group": "Комната", "dashboard": true, "order": 13 },
    { "key": "tvoc", "label": "TVOC", "icon": "wind", "format": "number", "unit": "ppb", "group": "Комната", "order": 14 },
    { "key": "aqi", "label": "AQI (балкон)", "icon": "activity", "format": "number", "group": "Воздух", "dashboard": true, "order": 15 },
    { "key": "out_temperature", "label": "Температура (балкон)", "icon": "thermometer", "format": "temperature", "unit": "°C", "group": "Улица", "order": 20 },
    { "key": "out_humidity", "label": "Влажность (балкон)", "icon": "droplets", "format": "percent", "group": "Улица", "order": 21 },
    { "key": "out_pressure", "label": "Давление (балкон)", "icon": "gauge", "format": "number", "unit": "мм рт.ст.", "group": "Улица", "order": 22 },
    { "key": "out_pm25", "label": "PM2.5 (балкон)", "icon": "activity", "format": "number", "unit": "µg/m³", "group": "Улица", "order": 23 },
    { "key": "out_pm10", "label": "PM10 (балкон)", "icon": "activity", "format": "number", "unit": "µg/m³", "group": "Улица", "order": 24 },
    { "key": "led", "label": "Светодиод", "icon": "lightbulb", "format": "boolean", "group": "Устройство", "order": 30 },
    { "key": "fw_version", "label": "Версия прошивки", "icon": "cpu", "group": "Система", "order": 40 }
  ],
  "dashboard": {
    "summary": ["temperature", "humidity", "eco2", "rssi"],
    "max_items": 4
  }
})CAP";
unsigned long lastMqttTelemetry = 0;

char otaUrl[256] = "";
bool otaPending = false;
int lastOtaProgress = -1;
bool littleFsReady = false;

// =====================
// Forward declarations
// =====================
const char* getAqiLevel(int value);
const char* getAqiAdvice(int value);
const char* getEco2Level(uint16_t ppm);
const char* getEco2Advice(uint16_t ppm);
const char* getEco2Verdict(uint16_t ppm);
uint16_t getEco2Color(uint16_t ppm);
void drawEco2Scale(int x, int y, int w, int h, uint16_t ppm);
void drawIndoorAirScreen();
void updateEns160IfNeeded();
void feedEns160EnvData();
void dismissScreenAlerts();
void checkScreenAlerts();
void drawCurrentScreen();
void drawTimeScreen();
void drawInfoScreen();
void drawStatusBar();
void setStatus(const char* status);
void initMqttTopics();
void handleMqttCommand(char* topic, byte* payload, unsigned int length);
void setBoardLed(bool on);
void showStatusScreen();
void ensureMqtt();
void publishMqttTelemetry();
void publishOtaEvent(const char* phase, int progress = -1);
void queueOtaUpdate(const char* url);
void performOtaUpdate(const char* url);
void fetchWeatherFromSupabase();
bool isGpioPinAllowed(uint8_t pin);
bool isPinOutputCapable(uint8_t pin);
bool applyPinMode(uint8_t pin, const char* mode);
void publishPinRead(uint8_t pin);
bool isFsPathValid(const char* path);
void publishFsTelemetry(JsonDocument& doc);
void publishFsError(const char* action, const char* path, const char* error);
void handleFsLs(const char* path);
void handleFsRead(const char* path);
void handleFsWrite(const char* path, const char* content);
void handleFsRm(const char* path);
void handleJoystick();
void handleButton();
void handleFlamingoButton();
void handleFlamingoTelemetry(byte* payload, unsigned int length);
void publishFlamingoSign(bool on);
void toggleFlamingoSign();
void updateHomeScreenIfNeeded();
int calculateEPA_AQI(float pm25, float pm10);

// =====================
// Helpers
// =====================
float hPaToMmHg(float hPa) {
  return hPa * 0.75006375541921;
}

void setupColors() {
  COLOR_BG      = tft.color565(8, 10, 16);
  COLOR_CARD    = tft.color565(22, 26, 36);
  COLOR_TEXT    = ST77XX_WHITE;
  COLOR_MUTED   = tft.color565(150, 158, 172);
  COLOR_BLUE    = tft.color565(40, 100, 255);
  COLOR_GREEN   = tft.color565(40, 190, 120);
  COLOR_ORANGE  = tft.color565(255, 145, 40);
  COLOR_RED     = tft.color565(230, 60, 70);
  COLOR_MAGENTA = tft.color565(190, 80, 255);
  COLOR_CYAN    = tft.color565(60, 200, 230);
  COLOR_YELLOW  = tft.color565(255, 210, 70);
}

void printDegreeC(int x, int y, uint16_t color) {
  tft.drawCircle(x, y + 3, 2, color);
  tft.setTextColor(color);
  tft.setTextSize(1);
  tft.setCursor(x + 6, y);
  tft.print("C");
}

void drawHeader(const char* title, uint16_t accentColor) {
  tft.fillRect(0, 0, tft.width(), 22, accentColor);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(7, 7);
  tft.print(title);

  tft.setCursor(tft.width() - 28, 7);
  tft.print((int)currentScreen + 1);
  tft.print("/");
  tft.print((int)SCREEN_COUNT);
}

void drawCard(int x, int y, int w, int h, const char* label, uint16_t borderColor) {
  tft.fillRoundRect(x, y, w, h, 5, COLOR_CARD);
  tft.drawRoundRect(x, y, w, h, 5, borderColor);

  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(x + 6, y + 5);
  tft.print(label);
}

void drawStatusBar() {
  int y = tft.height() - 14;

  tft.fillRect(0, y, tft.width(), 14, tft.color565(14, 16, 22));

  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(5, y + 3);
  tft.print(statusLine);
}

void setStatus(const char* status) {
  strncpy(statusLine, status, sizeof(statusLine) - 1);
  statusLine[sizeof(statusLine) - 1] = '\0';

  if (!showingTimeScreen && !showingInfoScreen) {
    drawStatusBar();
  }
}

void showMessage(const char* line1, const char* line2, const char* line3 = "") {
  tft.fillScreen(COLOR_BG);
  drawHeader("Flat Climate", COLOR_BLUE);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);
  tft.setCursor(8, 38);
  tft.print(line1);

  tft.setTextSize(1);
  tft.setCursor(8, 70);
  tft.print(line2);

  tft.setCursor(8, 88);
  tft.print(line3);

  drawStatusBar();
}

void formatTime(char* buffer, size_t size) {
  time_t now = time(nullptr);

  if (now < 100000) {
    snprintf(buffer, size, "--:--");
    return;
  }

  struct tm* t = localtime(&now);
  snprintf(buffer, size, "%02d:%02d", t->tm_hour, t->tm_min);
}

// =====================
// Wi-Fi / NTP
// =====================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");

  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    setStatus("WiFi connected");
  } else {
    Serial.println(" FAILED");
    setStatus("WiFi failed");
  }
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
  snprintf(topicFlamingoCommand, sizeof(topicFlamingoCommand),
           "devices/%s/command", FLAMINGO_HOSTNAME);
  snprintf(topicFlamingoTelemetry, sizeof(topicFlamingoTelemetry),
           "devices/%s/telemetry", FLAMINGO_HOSTNAME);
  snprintf(topicFlamingoRelay, sizeof(topicFlamingoRelay),
           "devices/%s/out/flamingo", DEVICE_HOSTNAME);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(handleMqttCommand);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  mqttTopicsReady = true;
}

void setBoardLed(bool on) {
  boardLedOn = on;
  digitalWrite(LED_BUILTIN, on ? LOW : HIGH);
}

void showStatusScreen() {
  showingInfoScreen = true;
  showingTimeScreen = false;
  infoScreenStart = millis();
  drawInfoScreen();
}

bool isGpioPinAllowed(uint8_t pin) {
  if (pin > 39) return false;
  if (pin >= 6 && pin <= 11) return false;
  if (pin == TFT_CS || pin == TFT_DC || pin == TFT_RST ||
      pin == TFT_SCLK || pin == TFT_MOSI ||
      pin == BME_SDA_PIN || pin == BME_SCL_PIN ||
      pin == JOY_X_PIN || pin == JOY_Y_PIN || pin == JOY_SW_PIN ||
      pin == FLAMINGO_BTN_PIN) {
    return false;
  }
  return true;
}

bool isPinOutputCapable(uint8_t pin) {
  if (pin >= 34 && pin <= 39) return false;
  return true;
}

bool applyPinMode(uint8_t pin, const char* mode) {
  if (!isGpioPinAllowed(pin) || !mode) return false;

  if (strcmp(mode, "OUTPUT") == 0) {
    if (!isPinOutputCapable(pin)) return false;
    pinMode(pin, OUTPUT);
    return true;
  }
  if (strcmp(mode, "INPUT") == 0) {
    pinMode(pin, INPUT);
    return true;
  }
  if (strcmp(mode, "INPUT_PULLUP") == 0) {
    pinMode(pin, INPUT_PULLUP);
    return true;
  }
  return false;
}

void publishPinRead(uint8_t pin) {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<128> doc;
  char keyDig[20];
  char keyAna[20];
  snprintf(keyDig, sizeof(keyDig), "pin_%d_digital", pin);
  snprintf(keyAna, sizeof(keyAna), "pin_%d_analog", pin);
  doc[keyDig] = digitalRead(pin);
  doc[keyAna] = analogRead(pin);

  char buf[128];
  size_t n = serializeJson(doc, buf);
  mqttClient.publish(topicTelemetry, buf, n);
  mqttClient.loop();
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
  if (strcmp(topic, topicFlamingoTelemetry) == 0) {
    handleFlamingoTelemetry(payload, length);
    return;
  }

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
      setBoardLed(doc["value"]);
    } else if (doc["value"].is<int>()) {
      setBoardLed(doc["value"] != 0);
    } else {
      return;
    }
    Serial.printf("[MQTT] led %s\n", boardLedOn ? "on" : "off");
    publishMqttTelemetry();
    return;
  }

  if (strcmp(action, "reboot") == 0) {
    Serial.println("[MQTT] reboot");
    delay(300);
    ESP.restart();
  }

  if (strcmp(action, "status") == 0) {
    Serial.println("[MQTT] status screen");
    showStatusScreen();
    return;
  }

  if (strcmp(action, "refresh") == 0) {
    Serial.println("[MQTT] refresh outdoor data");
    fetchWeatherFromSupabase();
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

  if (strcmp(action, "pin_mode") == 0) {
    if (!doc["pin"].is<int>()) return;
    const char* mode = doc["mode"];
    if (!mode) return;
    uint8_t pin = doc["pin"];
    if (!applyPinMode(pin, mode)) {
      Serial.printf("[MQTT] pin_mode rejected pin=%u mode=%s\n", pin, mode);
      return;
    }
    Serial.printf("[MQTT] pin_mode pin=%u mode=%s\n", pin, mode);
    return;
  }

  if (strcmp(action, "pin_write") == 0) {
    if (!doc["pin"].is<int>() || !doc["value"].is<int>()) return;
    uint8_t pin = doc["pin"];
    int value = doc["value"];
    if (!isGpioPinAllowed(pin)) {
      Serial.printf("[MQTT] pin_write rejected pin=%u\n", pin);
      return;
    }
    if (value <= 1) {
      if (!isPinOutputCapable(pin)) return;
      if (pin == LED_BUILTIN) {
        setBoardLed(value != 0);
        publishMqttTelemetry();
      } else {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, value ? HIGH : LOW);
      }
      Serial.printf("[MQTT] pin_write pin=%u value=%d\n", pin, value);
    } else {
      if (!isPinOutputCapable(pin)) return;
      pinMode(pin, OUTPUT);
      analogWrite(pin, constrain(value, 0, 255));
      Serial.printf("[MQTT] pin_write pin=%u pwm=%d\n", pin, constrain(value, 0, 255));
    }
    return;
  }

  if (strcmp(action, "pin_read") == 0) {
    if (!doc["pin"].is<int>()) return;
    uint8_t pin = doc["pin"];
    if (!isGpioPinAllowed(pin)) {
      Serial.printf("[MQTT] pin_read rejected pin=%u\n", pin);
      return;
    }
    Serial.printf("[MQTT] pin_read pin=%u\n", pin);
    publishPinRead(pin);
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
    mqttClient.subscribe(topicFlamingoTelemetry, 0);
    Serial.printf("[Flamingo] cmd topic  -> %s\n", topicFlamingoCommand);
    Serial.printf("[Flamingo] relay topic -> %s\n", topicFlamingoRelay);
    Serial.printf("[Flamingo] telem sub <- %s\n", topicFlamingoTelemetry);
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

  StaticJsonDocument<512> doc;
  doc["uptime"] = millis() / 1000UL;
  doc["heap"] = ESP.getFreeHeap();
  doc["bme_ready"] = bmeReady;
  doc["aht_ready"] = ahtReady;
  doc["ens160_ready"] = ens160Ready;
  doc["led"] = boardLedOn;
  doc["screen"] = (int)currentScreen + 1;
  doc["overlay"] = showingTimeScreen ? "time" : (showingInfoScreen ? "status" : "");
  addNetworkTelemetry(doc);
  addFirmwareTelemetry(doc);

  if (bmeReady) {
    float t = (filteredTemp > -900) ? filteredTemp : bme.readTemperature();
    float h = (filteredHum > -900)  ? filteredHum  : bme.readHumidity();
    float p = (filteredPres > -900) ? filteredPres : hPaToMmHg(bme.readPressure() / 100.0);
    doc["temperature"] = t;
    doc["humidity"] = h;
    doc["pressure"] = p;
  }

  if (ens160Ready && ensDataValid) {
    doc["eco2"] = lastEco2;
    doc["tvoc"] = lastTvoc;
    doc["indoor_aqi"] = lastEnsAqi;
  }

  if (outDataValid) {
    doc["out_temperature"] = outTemp;
    doc["out_humidity"] = outHumidity;
    doc["out_pressure"] = outPressure;
    doc["out_pm25"] = outPm25;
    doc["out_pm10"] = outPm10;
  }

  if (aqiDataValid) doc["aqi"] = aqiValue;

  char buf[512];
  size_t n = serializeJson(doc, buf);
  mqttClient.publish(topicTelemetry, buf, n);
}

void publishOtaEvent(const char* phase, int progress) {
  ota_mqtt::publish(phase, progress);
}

void queueOtaUpdate(const char* url) {
  strncpy(otaUrl, url, sizeof(otaUrl) - 1);
  otaUrl[sizeof(otaUrl) - 1] = '\0';
  otaPending = true;
}

void performOtaUpdate(const char* url) {
  Serial.printf("[OTA] starting: %s (heap=%u)\n", url, ESP.getFreeHeap());
  setStatus("OTA starting");
  showMessage("OTA UPDATE", "Downloading...", url);

  ota_mqtt::bind(mqttClient, topicTelemetry, ensureMqtt, lastOtaProgress);
  publishOtaEvent("starting", 0);
  lastOtaProgress = 0;

  WiFi.setSleep(WIFI_PS_NONE);
  Serial.printf("[OTA] heap before download: %u\n", ESP.getFreeHeap());

  t_httpUpdate_return ret = ota_mqtt::runUpdate(url);
  if (ret != HTTP_UPDATE_OK) {
    Serial.printf("[OTA] failed: %s\n", httpUpdate.getLastErrorString().c_str());
    int p = lastOtaProgress >= 0 ? lastOtaProgress : 0;
    publishOtaEvent("failed", p);
    setStatus("OTA failed");
    showMessage("OTA FAILED", httpUpdate.getLastErrorString().c_str(), "");
    delay(3000);
    drawCurrentScreen();
  }
}

void syncTime() {
  configTime(4 * 3600, 0, "pool.ntp.org", "time.google.com");

  Serial.print("NTP sync");
  int tries = 0;

  while (time(nullptr) < 100000 && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (time(nullptr) > 100000) {
    Serial.println(" OK");
    setStatus("Time synced");
  } else {
    Serial.println(" FAILED");
    setStatus("Time sync failed");
  }
}

// =====================
// AQI US EPA Standard Calculation
// =====================
int linearInterpolate(float concentration, float cLow, float cHigh, float iLow, float iHigh) {
  return (int)round(((iHigh - iLow) / (cHigh - cLow)) * (concentration - cLow) + iLow);
}

int calculatePM25_AQI(float pm25) {
  if (pm25 <= 12.0)  return linearInterpolate(pm25, 0.0, 12.0, 0, 50);
  if (pm25 <= 35.4)  return linearInterpolate(pm25, 12.1, 35.4, 51, 100);
  if (pm25 <= 55.4)  return linearInterpolate(pm25, 35.5, 55.4, 101, 150);
  if (pm25 <= 150.4) return linearInterpolate(pm25, 55.5, 150.4, 151, 200);
  if (pm25 <= 250.4) return linearInterpolate(pm25, 150.5, 250.4, 201, 300);
  if (pm25 <= 500.4) return linearInterpolate(pm25, 250.5, 500.4, 301, 500);
  return 500;
}

int calculatePM10_AQI(float pm10) {
  if (pm10 <= 54)    return linearInterpolate(pm10, 0, 54, 0, 50);
  if (pm10 <= 154)   return linearInterpolate(pm10, 55, 154, 51, 100);
  if (pm10 <= 254)   return linearInterpolate(pm10, 155, 254, 101, 150);
  if (pm10 <= 354)   return linearInterpolate(pm10, 255, 354, 151, 200);
  if (pm10 <= 424)   return linearInterpolate(pm10, 355, 424, 201, 300);
  if (pm10 <= 604)   return linearInterpolate(pm10, 425, 604, 301, 500);
  return 500;
}

int calculateEPA_AQI(float pm25, float pm10) {
  int aqiPm25 = calculatePM25_AQI(pm25);
  int aqiPm10 = calculatePM10_AQI(pm10);
  return max(aqiPm25, aqiPm10);
}

// =====================
// Supabase
// =====================
void sendToSupabase(float temperature, float humidity, float pressureMmHg) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) return;
  }

  HTTPClient http;
  String url = String(SUPABASE_URL) + "/flat_climate_metrics";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);
  http.addHeader("Prefer", "return=minimal");

  char json[256];
  if (ens160Ready && ensDataValid) {
    snprintf(
      json,
      sizeof(json),
      "{\"temperature\":%.1f,\"pressure\":%.1f,\"humidity\":%.1f,\"eco2\":%u,\"tvoc\":%u}",
      temperature,
      pressureMmHg,
      humidity,
      lastEco2,
      lastTvoc
    );
  } else {
    snprintf(
      json,
      sizeof(json),
      "{\"temperature\":%.1f,\"pressure\":%.1f,\"humidity\":%.1f}",
      temperature,
      pressureMmHg,
      humidity
    );
  }

  int code = http.POST(json);
  Serial.print("Supabase send: ");
  Serial.println(code);
  http.end();

  if (code >= 200 && code < 300) {
    char timeBuf[8];
    formatTime(timeBuf, sizeof(timeBuf));

    char status[40];
    snprintf(status, sizeof(status), "Indoor sent %s", timeBuf);
    setStatus(status);
  } else {
    setStatus("Indoor send failed");
  }
}

void fetchWeatherFromSupabase() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) return;
  }

  Serial.println("Starting Supabase GET request...");
  HTTPClient http;
  String url = String(SUPABASE_URL) +
    "/weather_logs?select=temperature,ds18_temperature,humidity,pressure,pm2_5,pm10_0&order=id.desc&limit=1";

  http.begin(url);
  http.addHeader("apikey", SUPABASE_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);

  int code = http.GET();
  Serial.print("HTTP Response Code: ");
  Serial.println(code);

  if (code == 200) {
    Serial.println("Request success. Streaming stream pointer...");
    WiFiClient* stream = http.getStreamPtr();

    Serial.println("Allocating DynamicJsonDocument...");
    DynamicJsonDocument doc(3072);
    
    Serial.println("Deserializing JSON stream...");
    DeserializationError err = deserializeJson(doc, *stream);

    if (!err && doc.is<JsonArray>() && doc.size() > 0) {
      Serial.println("JSON parse OK. Parsing fields...");
      JsonObject firstRow = doc[0];

      // Outdoor temp: предпочтительно DS18B20 (улица), иначе BME280
      float bmeTemp = firstRow["temperature"].as<float>();
      if (!firstRow["ds18_temperature"].isNull()) {
        outTemp = firstRow["ds18_temperature"].as<float>();
      } else {
        outTemp = bmeTemp;
      }
      outHumidity = firstRow["humidity"].as<float>();
      outPressure = firstRow["pressure"].as<float>();
      outPm25     = firstRow["pm2_5"].as<float>();
      outPm10     = firstRow["pm10_0"].as<float>();
      outDataValid = true;

      aqiValue = calculateEPA_AQI(outPm25, outPm10);
      aqiDataValid = true;

      Serial.printf(
        "Outdoor: %.1f C (BME %.1f), %.1f mmHg, %.1f %%, PM2.5: %.1f, PM10: %.1f -> Calc AQI: %d\n",
        outTemp, bmeTemp, outPressure, outHumidity, outPm25, outPm10, aqiValue
      );

      char timeBuf[8];
      formatTime(timeBuf, sizeof(timeBuf));

      char status[40];
      snprintf(status, sizeof(status), "Metrics updated %s", timeBuf);
      setStatus(status);

      checkScreenAlerts();

      if ((currentScreen == SCREEN_OUTDOOR || currentScreen == SCREEN_AQI || currentScreen == SCREEN_INDOOR)
          && !showingTimeScreen && !showingInfoScreen) {
        drawCurrentScreen();
      }
    } else {
      Serial.print("Weather parse error: ");
      Serial.println(err.c_str());
      setStatus("Outdoor parse failed");
    }
  } else {
    Serial.print("Weather fetch error: ");
    Serial.println(code);
    setStatus("Outdoor fetch failed");
  }

  http.end();
  Serial.println("HTTP Client execution finished.");
}

// =====================
// Input (Сквозное листание)
// =====================
void handleButton() {
  bool reading = digitalRead(JOY_SW_PIN);
  unsigned long now = millis();

  if (reading == LOW && lastButtonState == HIGH) {
    if (now - lastButtonTime > BUTTON_DEBOUNCE) {
      showingTimeScreen = true;
      showingInfoScreen = false;
      timeScreenStart = now;
      lastButtonTime = now;
      dismissScreenAlerts();

      Serial.println("Button -> time screen");
      drawTimeScreen();
    }
  }

  lastButtonState = reading;
}

void handleFlamingoTelemetry(byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) {
    Serial.println("[Flamingo] telemetry JSON parse error");
    return;
  }

  if (doc["sign"].is<bool>()) {
    flamingoSignOn = doc["sign"];
    flamingoStateKnown = true;
    flamingoTelemSeen = true;
    Serial.printf("[Flamingo] telemetry sign=%s (confirmed)\n", flamingoSignOn ? "ON" : "OFF");
  } else if (doc["sign"].is<int>()) {
    flamingoSignOn = doc["sign"] != 0;
    flamingoStateKnown = true;
    flamingoTelemSeen = true;
    Serial.printf("[Flamingo] telemetry sign=%s (confirmed)\n", flamingoSignOn ? "ON" : "OFF");
  } else {
    Serial.println("[Flamingo] telemetry without sign field");
  }
}

void publishFlamingoSign(bool on) {
  if (!mqttClient.connected()) {
    Serial.println("[Flamingo] MQTT offline, skip command");
    return;
  }

  StaticJsonDocument<64> doc;
  doc["action"] = "sign";
  doc["value"] = on;

  char buf[64];
  size_t n = serializeJson(doc, buf);
  Serial.printf("[Flamingo] publish relay=%s payload=%s\n", topicFlamingoRelay, buf);

  bool okRelay = mqttClient.publish(topicFlamingoRelay, buf, n);
  mqttClient.loop();

  bool okDirect = false;
#if FLAMINGO_DIRECT_COMMAND
  okDirect = mqttClient.publish(topicFlamingoCommand, buf, n);
  mqttClient.loop();
#endif

  if (okRelay || okDirect) {
    Serial.printf("[Flamingo] -> sent sign %s (relay=%s%s, ждём telemetry)\n",
                  on ? "ON" : "OFF",
                  okRelay ? "ok" : "fail",
#if FLAMINGO_DIRECT_COMMAND
                  okDirect ? ", direct=ok" : ", direct=fail"
#else
                  ""
#endif
                  );
  } else {
    Serial.printf("[Flamingo] publish FAILED mqtt_state=%d\n", mqttClient.state());
  }
}

void toggleFlamingoSign() {
  bool next = flamingoStateKnown ? !flamingoSignOn : true;
  Serial.printf("[Flamingo] toggle: known=%d current=%s -> next=%s\n",
                flamingoStateKnown,
                flamingoStateKnown ? (flamingoSignOn ? "ON" : "OFF") : "?",
                next ? "ON" : "OFF");
  publishFlamingoSign(next);
}

void handleFlamingoButton() {
  bool reading = digitalRead(FLAMINGO_BTN_PIN);
  unsigned long now = millis();

#if FLAMINGO_BTN_DEBUG
  if (now - lastFlamingoBtnLog >= FLAMINGO_BTN_LOG_MS) {
    lastFlamingoBtnLog = now;
    Serial.printf("[Btn33] GPIO%d raw=%s mqtt=%s sign=%s%s\n",
                  FLAMINGO_BTN_PIN,
                  reading ? "HIGH" : "LOW",
                  mqttClient.connected() ? "up" : "down",
                  flamingoStateKnown ? (flamingoSignOn ? "ON" : "OFF") : "?",
                  flamingoTelemSeen ? " (flamingo)" : " (no telem!)");
  }

  if (reading != lastFlamingoBtnState) {
    Serial.printf("[Btn33] edge %s -> %s\n",
                  lastFlamingoBtnState ? "HIGH" : "LOW",
                  reading ? "HIGH" : "LOW");
  }
#endif

  if (reading == LOW && lastFlamingoBtnState == HIGH) {
    unsigned long since = now - lastFlamingoBtnTime;
    if (since > FLAMINGO_BTN_DEBOUNCE) {
      lastFlamingoBtnTime = now;
      dismissScreenAlerts();
      Serial.println("[Btn33] PRESS accepted -> toggle flamingo");
      toggleFlamingoSign();
    }
#if FLAMINGO_BTN_DEBUG
    else {
      Serial.printf("[Btn33] PRESS ignored (debounce %lums < %lums)\n",
                    since, FLAMINGO_BTN_DEBOUNCE);
    }
#endif
  }

#if FLAMINGO_BTN_DEBUG
  if (reading == HIGH && lastFlamingoBtnState == LOW) {
    Serial.println("[Btn33] RELEASE");
  }
#endif

  lastFlamingoBtnState = reading;
}

void handleJoystick() {
  int x = analogRead(JOY_X_PIN);
  int y = analogRead(JOY_Y_PIN);

  unsigned long now = millis();

  bool centeredX = x > JOY_LEFT_THRESHOLD && x < JOY_RIGHT_THRESHOLD;
  bool centeredY = y > JOY_LEFT_THRESHOLD && y < JOY_RIGHT_THRESHOLD;

  if (centeredX && centeredY) {
    joyWasCentered = true;
    return;
  }

  if (!joyWasCentered) return;
  if (now - lastJoyTime < JOY_DEBOUNCE) return;

  bool screenChanged = false;

  if (y >= JOY_DOWN_THRESHOLD) {
    dismissScreenAlerts();
    showingInfoScreen = true;
    showingTimeScreen = false;
    infoScreenStart = now;

    Serial.println("DOWN -> info screen");
    drawInfoScreen();

    joyWasCentered = false;
    lastJoyTime = now;
    return;
  }

  // Реализация сквозного циклического переключения экранов
  if (x <= JOY_LEFT_THRESHOLD) {
    currentScreen = (Screen)(((int)currentScreen - 1 + SCREEN_COUNT) % SCREEN_COUNT);
    screenChanged = true;
    Serial.print("LEFT -> screen ");
    Serial.println((int)currentScreen);
  }

  if (x >= JOY_RIGHT_THRESHOLD) {
    currentScreen = (Screen)(((int)currentScreen + 1) % SCREEN_COUNT);
    screenChanged = true;
    Serial.print("RIGHT -> screen ");
    Serial.println((int)currentScreen);
  }

  joyWasCentered = false;
  lastJoyTime = now;

  if (screenChanged) {
    dismissScreenAlerts();
    showingTimeScreen = false;
    showingInfoScreen = false;
    drawCurrentScreen();
  }
}

void dismissScreenAlerts() {
  if (!alertLock) return;
  alertLock = false;
  Serial.println("[Alert] dismissed by user");
}

void checkScreenAlerts() {
  if (showingTimeScreen || showingInfoScreen) return;
  if (alertLock) return;

  int eco2 = (filteredEco2 >= 0) ? (int)round(filteredEco2) : -1;
  int tvoc = (filteredTvoc >= 0) ? (int)round(filteredTvoc) : -1;

  if (ensDataValid) {
    bool eco2Clear = eco2 < 0 || eco2 <= ALERT_ECO2_CLEAR_PPM;
    bool tvocClear = tvoc < 0 || tvoc <= ALERT_TVOC_CLEAR_PPB;
    if (eco2Clear && tvocClear) indoorAlertEpisode = false;
  }

  if (aqiDataValid && aqiValue <= ALERT_AQI_CLEAR) {
    aqiAlertEpisode = false;
  }

  bool eco2High = ensDataValid && eco2 > ALERT_ECO2_PPM;
  bool tvocHigh = ensDataValid && tvoc > ALERT_TVOC_PPB;
  bool indoorHigh = eco2High || tvocHigh;

  if (indoorHigh && !indoorAlertEpisode) {
    indoorAlertEpisode = true;
    currentScreen = SCREEN_INDOOR;
    showingTimeScreen = false;
    showingInfoScreen = false;
    lastDisplayedEco2 = -1;
    lastDisplayedTvoc = -1;
    drawCurrentScreen();
    alertLock = true;
    setStatus(eco2High ? "CO2 high!" : "TVOC high!");
    Serial.printf("[Alert] -> indoor screen (eCO2=%d tvoc=%d)\n", eco2, tvoc);
    return;
  }

  bool aqiHigh = aqiDataValid && aqiValue > ALERT_AQI_VALUE;
  if (aqiHigh && !aqiAlertEpisode) {
    aqiAlertEpisode = true;
    currentScreen = SCREEN_AQI;
    showingTimeScreen = false;
    showingInfoScreen = false;
    drawCurrentScreen();
    alertLock = true;
    setStatus("AQI high!");
    Serial.printf("[Alert] -> AQI screen (aqi=%d)\n", aqiValue);
  }
}

// =====================
// Drawing
// =====================
void drawClimateScreen(
  const char* title,
  float temperature,
  float pressureMmHg,
  float humidity,
  bool outdoor
) {
  tft.fillScreen(COLOR_BG);

  uint16_t accent = outdoor ? COLOR_ORANGE : COLOR_BLUE;
  drawHeader(title, accent);

  // Temperature card
  drawCard(6, 30, 72, 50, "TEMP", accent);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(3);
  tft.setCursor(14, 48);
  tft.print((int)round(temperature));

  printDegreeC(54, 50, COLOR_TEXT);

  // Humidity card
  drawCard(84, 30, 70, 50, "HUM", COLOR_CYAN);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(3);
  tft.setCursor(94, 48);
  tft.print((int)round(humidity));

  tft.setTextSize(2);
  tft.setCursor(132, 54);
  tft.print("%");

  // Pressure card
  drawCard(6, 86, 148, 28, "PRESSURE", COLOR_GREEN);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);
  tft.setCursor(80, 94);
  tft.print((int)round(pressureMmHg));

  tft.setTextColor(COLOR_MUTED);
  tft.setTextSize(1);
  tft.setCursor(122, 100);
  tft.print("mmHg");

  drawStatusBar();
}

const char* getAqiLevel(int value) {
  if (value <= 50)  return "Good";
  if (value <= 100) return "Moderate";
  if (value <= 150) return "Unhealthy S";
  if (value <= 200) return "Unhealthy";
  if (value <= 300) return "Very bad";
  return "Hazardous";
}

uint16_t getAqiColor(int value) {
  if (value <= 50)  return COLOR_GREEN;
  if (value <= 100) return COLOR_YELLOW;
  if (value <= 150) return COLOR_ORANGE;
  if (value <= 200) return COLOR_RED;
  if (value <= 300) return COLOR_MAGENTA;
  return COLOR_RED;
}

const char* getAqiAdvice(int value) {
  if (value <= 50)  return "Air is clean";
  if (value <= 100) return "Okay outside";
  if (value <= 150) return "Sensitive: care";
  if (value <= 200) return "Limit activity";
  if (value <= 300) return "Stay indoors";
  return "Avoid outdoor";
}

const char* getEco2Level(uint16_t ppm) {
  if (ppm <= 800)  return "Good";
  if (ppm <= 1000) return "Moderate";
  if (ppm <= 1500) return "Poor";
  return "Bad";
}

const char* getEco2Advice(uint16_t ppm) {
  if (ppm <= 800)  return "Fresh air";
  if (ppm <= 1000) return "Ventilate soon";
  if (ppm <= 1500) return "Open window";
  return "Air stale";
}

uint16_t getEco2Color(uint16_t ppm) {
  if (ppm <= 800)  return COLOR_GREEN;
  if (ppm <= 1000) return COLOR_YELLOW;
  if (ppm <= 1500) return COLOR_ORANGE;
  return COLOR_RED;
}

const char* getEco2Verdict(uint16_t ppm) {
  if (ppm <= 1000) return "NORM";
  return "BAD";
}

void drawEco2Scale(int x, int y, int w, int h, uint16_t ppm) {
  int segmentWidth = w / 4;

  uint16_t colors[4] = {
    COLOR_GREEN,
    COLOR_YELLOW,
    COLOR_ORANGE,
    COLOR_RED
  };

  for (int i = 0; i < 4; i++) {
    tft.fillRect(x + i * segmentWidth, y, segmentWidth - 1, h, colors[i]);
  }

  int markerX = x + map(constrain((int)ppm, 400, 2000), 400, 2000, 0, w - 4);
  tft.fillTriangle(
    markerX,
    y + h + 6,
    markerX + 4,
    y + h + 6,
    markerX + 2,
    y + h + 1,
    COLOR_TEXT
  );
}

void drawIndoorAirScreen() {
  tft.fillScreen(COLOR_BG);
  drawHeader("Indoor CO2", COLOR_CYAN);

  if (!ens160Ready) {
    drawCard(8, 36, 144, 64, "ENS160", COLOR_RED);
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(2);
    tft.setCursor(22, 58);
    tft.print("No sensor");
    tft.setTextSize(1);
    tft.setTextColor(COLOR_MUTED);
    tft.setCursor(18, 84);
    tft.print("SDA21 SCL22 0x53");
    drawStatusBar();
    return;
  }

  if (!ensDataValid || filteredEco2 < 0) {
    drawCard(8, 36, 144, 64, "ENS160", COLOR_CYAN);
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(2);
    tft.setCursor(18, 58);
    tft.print("Warming up");
    tft.setTextSize(1);
    tft.setTextColor(COLOR_MUTED);
    tft.setCursor(24, 84);
    tft.print("First read ~1 min");
    drawStatusBar();
    return;
  }

  uint16_t showEco2 = (uint16_t)round(filteredEco2);
  uint16_t showTvoc = (uint16_t)round(filteredTvoc);
  uint16_t eco2Color = getEco2Color(showEco2);
  const char* verdict = getEco2Verdict(showEco2);
  bool isNorm = (showEco2 <= 1000);

  drawCard(6, 30, 72, 50, "eCO2", eco2Color);
  tft.setTextColor(COLOR_TEXT);

  char eco2Buf[8];
  snprintf(eco2Buf, sizeof(eco2Buf), "%u", showEco2);
  uint8_t eco2Size = (showEco2 >= 1000) ? 2 : 3;

  tft.setTextSize(eco2Size);
  int16_t bx, by;
  uint16_t bw, bh;
  tft.getTextBounds(eco2Buf, 12, 48, &bx, &by, &bw, &bh);
  tft.setCursor(12, 48);
  tft.print(eco2Buf);

  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(12 + bw + 2, 48 + bh - 8);
  tft.print("ppm");

  drawCard(82, 30, 72, 50, verdict, eco2Color);
  tft.setTextColor(isNorm ? COLOR_GREEN : COLOR_RED);
  tft.setTextSize(isNorm ? 2 : 3);
  tft.setCursor(isNorm ? 96 : 94, isNorm ? 46 : 44);
  tft.print(verdict);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(1);
  tft.setCursor(88, 62);
  tft.print(getEco2Level(showEco2));

  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(8, 84);
  tft.print("TVOC");
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(36, 84);
  tft.print(showTvoc);
  tft.print(" ppb  ");
  tft.setTextColor(COLOR_MUTED);
  tft.print(getEco2Advice(showEco2));

  drawEco2Scale(8, 92, 144, 8, showEco2);

  tft.setTextColor(COLOR_MUTED);
  tft.setTextSize(1);
  tft.setCursor(8, 106);
  tft.print("400");

  tft.setCursor(68, 106);
  tft.print("1000");

  tft.setCursor(128, 106);
  tft.print("2000");

  drawStatusBar();
}

void feedEns160EnvData() {
  if (!ens160Ready) return;

  if (ahtReady) {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    ens160.set_envdata(temp.temperature, humidity.relative_humidity);
    return;
  }

  // Запасной вариант, если AHT не найден (датчик в другом месте — хуже для ENS160)
  if (bmeReady) {
    float t = (filteredTemp > -900) ? filteredTemp : bme.readTemperature();
    float h = (filteredHum > -900)  ? filteredHum  : bme.readHumidity();
    ens160.set_envdata(t, h);
  }
}

void updateEns160IfNeeded() {
  if (!ens160Ready) return;

  feedEns160EnvData();

  if (!ens160.available()) return;
  if (!ens160.measure(true)) return;

  lastEco2 = ens160.geteCO2();
  lastTvoc = ens160.getTVOC();
  lastEnsAqi = ens160.getAQI();
  ensDataValid = true;

  if (filteredEco2 < 0) {
    filteredEco2 = (float)lastEco2;
    filteredTvoc = (float)lastTvoc;
  } else {
    filteredEco2 = ((float)lastEco2 * ENS_EMA_ALPHA) + (filteredEco2 * (1.0f - ENS_EMA_ALPHA));
    filteredTvoc = ((float)lastTvoc * ENS_EMA_ALPHA) + (filteredTvoc * (1.0f - ENS_EMA_ALPHA));
  }

  if (currentScreen != SCREEN_INDOOR) return;
  if (showingTimeScreen || showingInfoScreen) return;

  int roundedEco2 = (int)round(filteredEco2);
  int roundedTvoc = (int)round(filteredTvoc);

  bool verdictChanged =
    lastDisplayedEco2 >= 0 &&
    ((lastDisplayedEco2 <= 1000) != (roundedEco2 <= 1000));
  bool eco2Changed =
    lastDisplayedEco2 < 0 || abs(roundedEco2 - lastDisplayedEco2) >= 10;
  bool tvocChanged =
    lastDisplayedTvoc < 0 || abs(roundedTvoc - lastDisplayedTvoc) >= 15;

  if (!eco2Changed && !tvocChanged && !verdictChanged) return;

  lastDisplayedEco2 = roundedEco2;
  lastDisplayedTvoc = roundedTvoc;
  drawIndoorAirScreen();

  checkScreenAlerts();
}

void drawAqiScale(int x, int y, int w, int h, int value) {
  int segmentWidth = w / 6;

  uint16_t colors[6] = {
    COLOR_GREEN,
    COLOR_YELLOW,
    COLOR_ORANGE,
    COLOR_RED,
    COLOR_MAGENTA,
    COLOR_RED
  };

  for (int i = 0; i < 6; i++) {
    tft.fillRect(x + i * segmentWidth, y, segmentWidth - 1, h, colors[i]);
  }

  int markerX = x + map(constrain(value, 0, 300), 0, 300, 0, w - 4);
  tft.fillTriangle(
    markerX,
    y + h + 6,
    markerX + 4,
    y + h + 6,
    markerX + 2,
    y + h + 1,
    COLOR_TEXT
  );
}

void drawAqiScreen() {
  tft.fillScreen(COLOR_BG);
  drawHeader("Calculated AQI", COLOR_MAGENTA);

  if (!aqiDataValid) {
    drawCard(8, 36, 144, 64, "AIR QUALITY", COLOR_MAGENTA);

    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(2);
    tft.setCursor(22, 58);
    tft.print("No Data");

    tft.setTextSize(1);
    tft.setTextColor(COLOR_MUTED);
    tft.setCursor(22, 84);
    tft.print("Waiting DB Sync");

    drawStatusBar();
    return;
  }

  uint16_t aqiColor = getAqiColor(aqiValue);

  drawCard(6, 30, 64, 50, "AQI", aqiColor);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(3);
  tft.setCursor(15, 48);
  tft.print(aqiValue);

  drawCard(76, 30, 78, 50, "LEVEL", aqiColor);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(1);
  tft.setCursor(84, 50);
  tft.print(getAqiLevel(aqiValue));

  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(84, 66);
  tft.print(getAqiAdvice(aqiValue));

  drawAqiScale(8, 92, 144, 8, aqiValue);

  tft.setTextColor(COLOR_MUTED);
  tft.setTextSize(1);
  tft.setCursor(8, 106);
  tft.print("0");

  tft.setCursor(70, 106);
  tft.print("150");

  tft.setCursor(136, 106);
  tft.print("300");

  drawStatusBar();
}

void drawInfoStatusCell(int x, int y, const char* label, bool ok) {
  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(x, y);
  tft.print(label);
  tft.fillCircle(x + 30, y + 4, 3, ok ? COLOR_GREEN : COLOR_RED);
}

void drawInfoScreen() {
  tft.fillScreen(COLOR_BG);
  drawHeader("System info", COLOR_GREEN);
  drawCard(6, 26, 148, 58, "STATUS", COLOR_GREEN);

  bool wifiOk = WiFi.status() == WL_CONNECTED;
  bool mqttOk = mqttClient.connected();

  drawInfoStatusCell(12, 40, "WiFi", wifiOk);
  drawInfoStatusCell(58, 40, "BME", bmeReady);
  drawInfoStatusCell(104, 40, "AHT", ahtReady);

  drawInfoStatusCell(12, 54, "ENS", ens160Ready);
  drawInfoStatusCell(58, 54, "OUT", outDataValid);
  drawInfoStatusCell(104, 54, "MQTT", mqttOk);

  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(12, 68);
  tft.print("IP");
  tft.setTextColor(wifiOk ? COLOR_TEXT : COLOR_RED);
  tft.setCursor(26, 68);
  tft.print(wifiOk ? WiFi.localIP().toString().c_str() : "---");

  drawCard(6, 88, 148, 24, "FIRMWARE", COLOR_CYAN);
  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(12, 98);
  tft.print("Ver");
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(30, 98);
  tft.print(FW_VERSION);

  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(12, 108);
  tft.print("Build");
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(40, 108);
  tft.print(FW_BUILD_DATE);

  drawStatusBar();
}

void drawTimeScreen() {
  tft.fillScreen(COLOR_BG);
  drawHeader("Time", COLOR_RED);

  time_t now = time(nullptr);

  if (now < 100000) {
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(2);
    tft.setCursor(22, 52);
    tft.print("NO TIME");
    tft.setTextSize(1);
    tft.setTextColor(COLOR_MUTED);
    tft.setCursor(28, 80);
    tft.print("NTP not synced");
    drawStatusBar();
    return;
  }

  struct tm* t = localtime(&now);

  char timeBuf[6];
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", t->tm_hour, t->tm_min);

  char dateBuf[12];
  snprintf(
    dateBuf,
    sizeof(dateBuf),
    "%02d.%02d.%04d",
    t->tm_mday,
    t->tm_mon + 1,
    t->tm_year + 1900
  );

  tft.fillRoundRect(18, 36, 124, 64, 5, COLOR_CARD);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(24, 41);
  tft.print("LOCAL TIME");

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(4);
  tft.setCursor(28, 56);
  tft.print(timeBuf);

  tft.setTextColor(COLOR_MUTED);
  tft.setTextSize(1);
  tft.setCursor(52, 104);
  tft.print(dateBuf);

  drawStatusBar();
}

void drawNoBmeScreen() {
  tft.fillScreen(COLOR_BG);
  drawHeader("BME280 error", COLOR_RED);
  drawCard(8, 34, 144, 70, "CHECK SENSOR", COLOR_RED);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(1);
  tft.setCursor(18, 56);
  tft.print("SDA -> GPIO 21");
  tft.setCursor(18, 74);
  tft.print("SCL -> GPIO 22");
  tft.setCursor(18, 92);
  tft.print("Try address 0x77");

  drawStatusBar();
}

void drawCurrentScreen() {
  if (!bmeReady && currentScreen == SCREEN_HOME) {
    drawNoBmeScreen();
    return;
  }

  if (currentScreen == SCREEN_HOME) {
    if (filteredTemp < -900) {
      filteredTemp = bme.readTemperature();
      filteredHum  = bme.readHumidity();
      filteredPres = hPaToMmHg(bme.readPressure() / 100.0);
    }

    lastDisplayedTemp     = (int)round(filteredTemp);
    lastDisplayedHumidity = (int)round(filteredHum);
    lastDisplayedPressure = (int)round(filteredPres);

    drawClimateScreen(
      "Home climate",
      filteredTemp,
      filteredPres,
      filteredHum,
      false
    );
  }
  else if (currentScreen == SCREEN_OUTDOOR) {
    if (outDataValid) {
      drawClimateScreen(
        "Outdoor",
        outTemp,
        outPressure,
        outHumidity,
        true
      );
    } else {
      tft.fillScreen(COLOR_BG);
      drawHeader("Outdoor", COLOR_ORANGE);
      drawCard(8, 38, 144, 56, "OUTDOOR WEATHER", COLOR_ORANGE);

      tft.setTextColor(COLOR_TEXT);
      tft.setTextSize(2);
      tft.setCursor(26, 60);
      tft.print("No data");

      tft.setTextColor(COLOR_MUTED);
      tft.setTextSize(1);
      tft.setCursor(24, 84);
      tft.print("Waiting Supabase");

      drawStatusBar();
    }
  }
  else if (currentScreen == SCREEN_INDOOR) {
    if (filteredEco2 >= 0) {
      lastDisplayedEco2 = (int)round(filteredEco2);
      lastDisplayedTvoc = (int)round(filteredTvoc);
    } else {
      lastDisplayedEco2 = -1;
      lastDisplayedTvoc = -1;
    }
    drawIndoorAirScreen();
  }
  else if (currentScreen == SCREEN_AQI) {
    drawAqiScreen();
  }
}

void updateHomeScreenIfNeeded() {
  if (!bmeReady) return;
  if (currentScreen != SCREEN_HOME) return;
  if (showingTimeScreen || showingInfoScreen) return;

  float rawTemp = bme.readTemperature();
  float rawHum  = bme.readHumidity();
  float rawPres = hPaToMmHg(bme.readPressure() / 100.0);

  if (filteredTemp < -900) {
    filteredTemp = rawTemp;
    filteredHum  = rawHum;
    filteredPres = rawPres;
  } else {
    filteredTemp = (rawTemp * EMA_ALPHA) + (filteredTemp * (1.0 - EMA_ALPHA));
    filteredHum  = (rawHum  * EMA_ALPHA) + (filteredHum  * (1.0 - EMA_ALPHA));
    filteredPres = (rawPres * EMA_ALPHA) + (filteredPres * (1.0 - EMA_ALPHA));
  }

  int roundedTemp     = (int)round(filteredTemp);
  int roundedHumidity = (int)round(filteredHum);
  int roundedPressure = (int)round(filteredPres);

  bool changed =
    roundedTemp     != lastDisplayedTemp ||
    roundedHumidity != lastDisplayedHumidity ||
    roundedPressure != lastDisplayedPressure;

  if (!changed) return;

  lastDisplayedTemp     = roundedTemp;
  lastDisplayedHumidity = roundedHumidity;
  lastDisplayedPressure = roundedPressure;

  drawClimateScreen(
    "Home climate",
    filteredTemp,
    filteredPres,
    filteredHum,
    false
  );
}

// =====================
// Setup / loop
// =====================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  setBoardLed(false);

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 TFT Climate Station");
  logFirmwareInfo("esp32-flat");

  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  pinMode(JOY_X_PIN, INPUT);
  pinMode(JOY_Y_PIN, INPUT);
  pinMode(FLAMINGO_BTN_PIN, INPUT_PULLUP);
  lastFlamingoBtnState = digitalRead(FLAMINGO_BTN_PIN);
  Serial.printf("[Btn33] init GPIO%d INPUT_PULLUP, startup=%s\n",
                FLAMINGO_BTN_PIN,
                lastFlamingoBtnState ? "HIGH (released)" : "LOW (pressed?)");

  analogReadResolution(12);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);

  setupColors();

  tft.fillScreen(COLOR_BG);
  setStatus("Booting");

  // Партиция в partitions.csv называется "spiffs" (историческое имя)
  littleFsReady = LittleFS.begin(true, "/littlefs", 10, "spiffs");
  if (!littleFsReady) {
    littleFsReady = LittleFS.begin(true);
  }
  if (!littleFsReady) {
    Serial.println("[LittleFS] mount FAILED");
    setStatus("FS error");
    showMessage("FS ERROR", "LittleFS mount", "failed");
    delay(1500);
  } else {
    Serial.printf("[LittleFS] mounted, used=%u total=%u\n",
                  LittleFS.usedBytes(), LittleFS.totalBytes());
  }

  showMessage("START", "TFT OK", "Starting BME280");
  delay(800);

  Wire.begin(BME_SDA_PIN, BME_SCL_PIN);
  bmeReady = bme.begin(BME280_ADDRESS, &Wire);

  if (!bmeReady) {
    Serial.println("BME280 not found at 0x76");
    setStatus("BME280 error");
    showMessage("BME ERROR", "Address 0x76", "Try 0x77");
    delay(1500);
  } else {
    Serial.println("BME280 OK");
    setStatus("BME280 ready");
    showMessage("BME280 OK", "Starting AHT21", "");
    delay(800);
  }

  ahtReady = aht.begin();
  if (!ahtReady) {
    Serial.println("AHT21 not found (ENS160 will use BME for T/RH)");
    setStatus("AHT21 missing");
    showMessage("AHT WARN", "Using BME T/RH", "for ENS160");
    delay(1200);
  } else {
    Serial.println("AHT21 OK");
    setStatus("AHT21 ready");
    showMessage("AHT21 OK", "Starting ENS160", "");
    delay(800);
  }

  ens160Ready = ens160.begin();
  if (!ens160Ready) {
    Serial.println("ENS160 not found");
    setStatus("ENS160 error");
    showMessage("ENS160 ERROR", "Check I2C addr", "0x52 or 0x53");
    delay(1500);
  } else {
    ens160.setMode(ENS160_OPMODE_STD);
    Serial.println("ENS160 OK");
    setStatus("ENS160 ready");
    showMessage("ENS160 OK", "Starting WiFi", "");
    delay(800);
  }

  connectWiFi();
  ensureMqtt();

  if (WiFi.status() == WL_CONNECTED) {
    showMessage("WiFi OK", WiFi.localIP().toString().c_str(), "Sync time");
    syncTime();
    delay(800);

    fetchWeatherFromSupabase();
  } else {
    showMessage("WiFi FAILED", "Offline mode", "");
    delay(1000);
  }

  drawCurrentScreen();
}

void loop() {
  if (otaPending) {
    otaPending = false;
    performOtaUpdate(otaUrl);
    return;
  }

  unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    ensureMqtt();
    mqttClient.loop();

    if (now - lastMqttTelemetry >= MQTT_TELEMETRY_INTERVAL) {
      lastMqttTelemetry = now;
      publishMqttTelemetry();
    }
  }

  handleJoystick();
  handleButton();
  handleFlamingoButton();

  if (showingTimeScreen) {
    if (now - timeScreenStart >= TIME_SHOW_MS) {
      showingTimeScreen = false;
      drawCurrentScreen();
    }
    delay(20);
    return;
  }

  if (showingInfoScreen) {
    if (now - infoScreenStart >= INFO_SHOW_MS) {
      showingInfoScreen = false;
      drawCurrentScreen();
    }
    delay(20);
    return;
  }

  if (now - lastSensorCheckTime >= SENSOR_CHECK_INTERVAL) {
    updateHomeScreenIfNeeded();
    updateEns160IfNeeded();
    checkScreenAlerts();
    lastSensorCheckTime = now;
  }

  if (bmeReady && (now - lastSendTime >= SEND_INTERVAL || lastSendTime == 0)) {
    float t = (filteredTemp > -900) ? filteredTemp : bme.readTemperature();
    float h = (filteredHum > -900)  ? filteredHum  : bme.readHumidity();
    float p = (filteredPres > -900) ? filteredPres : hPaToMmHg(bme.readPressure() / 100.0);

    sendToSupabase(t, h, p);
    lastSendTime = now;
  }

  if (now - lastFetchTime >= FETCH_INTERVAL || lastFetchTime == 0) {
    fetchWeatherFromSupabase();
    lastFetchTime = now;
  }

  delay(50);
}