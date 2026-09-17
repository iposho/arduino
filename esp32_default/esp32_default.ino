/*
 * ESP32 Default — универсальная прошивка для любой ESP32 DevKit.
 *
 * Прошейте по USB на новую плату: Wi-Fi, MQTT, телеметрия и стандартные команды
 * работают сразу. Потом OTA-обновите на целевую прошивку (flat, flamingo, …).
 *
 * Библиотеки (Arduino Library Manager): PubSubClient, ArduinoJson
 *
 * Скопируйте secrets.example.h → secrets.h и задайте Wi-Fi / MQTT / hostname.
 *
 * MQTT-топики (DEVICE_HOSTNAME из secrets.h):
 *   devices/<hostname>/status        — online/offline (LWT)
 *   devices/<hostname>/telemetry     — периодическая телеметрия
 *   devices/<hostname>/capabilities  — retained JSON с commands[], metrics[]
 *   devices/<hostname>/command       — входящие JSON-команды
 *
 * Команды: led, reboot, ota, status, pin_mode, pin_write, pin_read
 */
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

#include "secrets.h"
#include "firmware_info.h"
#include "../include/ota_mqtt.h"

// =====================
// Конфигурация
// =====================
#define DEVICE_LABEL "esp32-default"

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

const unsigned long MQTT_TELEMETRY_INTERVAL = 10UL * 1000UL;
const unsigned long WIFI_RETRY_INTERVAL     = 30UL * 1000UL;
const unsigned long WDT_TIMEOUT_SEC         = 30UL;

// =====================
// MQTT capabilities (retained)
// =====================
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
      "title": "Статус",
      "type": "trigger",
      "icon": "activity",
      "description": "Немедленно опубликовать телеметрию"
    },
    {
      "action": "reboot",
      "title": "Перезагрузка",
      "type": "trigger",
      "icon": "rotate-cw",
      "description": "ESP.restart()"
    },
    {
      "action": "ota",
      "title": "OTA-обновление",
      "type": "trigger",
      "icon": "download",
      "description": "JSON: {\"action\":\"ota\",\"url\":\"https://…/firmware.bin\"}"
    },
    {
      "action": "pin_write",
      "title": "Запись GPIO",
      "type": "trigger",
      "icon": "plug",
      "description": "JSON: {\"action\":\"pin_write\",\"pin\":N,\"value\":0|1|0-255}"
    },
    {
      "action": "pin_read",
      "title": "Чтение GPIO",
      "type": "trigger",
      "icon": "gauge",
      "description": "JSON: {\"action\":\"pin_read\",\"pin\":N} — ответ в telemetry"
    },
    {
      "action": "pin_mode",
      "title": "Режим GPIO",
      "type": "trigger",
      "icon": "settings",
      "description": "JSON: {\"action\":\"pin_mode\",\"pin\":N,\"mode\":\"OUTPUT|INPUT|INPUT_PULLUP\"}"
    }
  ],
  "metrics": [
    { "key": "ip", "label": "IP-адрес", "icon": "globe", "group": "Сеть", "dashboard": true, "order": 0 },
    { "key": "rssi", "label": "Сигнал Wi-Fi", "icon": "signal", "format": "rssi", "group": "Сеть", "dashboard": true, "order": 1 },
    { "key": "uptime", "label": "Аптайм", "icon": "clock", "format": "uptime", "group": "Система", "dashboard": true, "order": 2 },
    { "key": "heap", "keys": ["heap", "free_heap"], "label": "Свободная RAM", "icon": "memory", "format": "bytes", "group": "Система", "dashboard": true, "order": 3 },
    { "key": "led", "label": "Светодиод", "icon": "lightbulb", "format": "boolean", "group": "Устройство", "order": 10 },
    { "key": "status", "label": "Статус", "icon": "activity", "format": "text", "group": "Система", "order": 11 },
    { "key": "fw_version", "label": "Версия прошивки", "icon": "cpu", "group": "Система", "order": 20 }
  ],
  "dashboard": {
    "summary": ["ip", "rssi", "uptime", "heap"],
    "max_items": 4
  }
})CAP";

// =====================
// State
// =====================
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

char topicStatus[64];
char topicTelemetry[64];
char topicCommand[64];
char topicCapabilities[64];
bool mqttTopicsReady = false;

bool boardLedOn = false;
char statusLine[64] = "Booting";

unsigned long lastMqttTelemetry = 0;
unsigned long lastWifiRetry = 0;

char otaUrl[256] = "";
bool otaPending = false;
int lastOtaProgress = -1;

// =====================
// Forward declarations
// =====================
void setStatus(const char* msg);
void setBoardLed(bool on);
void blinkBootLed();
void initWatchdog();
void feedWatchdog();
void connectWiFi();
void initMqttTopics();
void ensureMqtt();
void publishMqttTelemetry();
void publishOtaEvent(const char* phase, int progress = -1);
void queueOtaUpdate(const char* url);
void performOtaUpdate(const char* url);
void handleMqttCommand(char* topic, byte* payload, unsigned int length);
bool handleCustomMqttAction(const char* action, JsonDocument& doc);
bool isGpioPinAllowed(uint8_t pin);
bool isPinOutputCapable(uint8_t pin);
bool applyPinMode(uint8_t pin, const char* mode);
void publishPinRead(uint8_t pin);
void handlePinWrite(uint8_t pin, int value);
void setupDevice();
void loopDevice();

// =====================
// Logging / status
// =====================
void setStatus(const char* msg) {
  strncpy(statusLine, msg, sizeof(statusLine) - 1);
  statusLine[sizeof(statusLine) - 1] = '\0';
  Serial.printf("[Status] %s\n", statusLine);
}

void setBoardLed(bool on) {
  boardLedOn = on;
  digitalWrite(LED_BUILTIN, on ? LOW : HIGH);
}

void blinkBootLed() {
  for (int i = 0; i < 3; i++) {
    setBoardLed(true);
    delay(100);
    setBoardLed(false);
    delay(100);
  }
}

void initWatchdog() {
  esp_task_wdt_config_t cfg = {
    .timeout_ms = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  esp_task_wdt_init(&cfg);
  esp_task_wdt_add(NULL);
}

void feedWatchdog() {
  esp_task_wdt_reset();
}

// =====================
// GPIO helpers
// =====================
bool isGpioPinAllowed(uint8_t pin) {
  if (pin > 39) return false;
  if (pin >= 6 && pin <= 11) return false;
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

void handlePinWrite(uint8_t pin, int value) {
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
    return;
  }

  if (!isPinOutputCapable(pin)) return;
  pinMode(pin, OUTPUT);
  analogWrite(pin, constrain(value, 0, 255));
  Serial.printf("[MQTT] pin_write pin=%u pwm=%d\n", pin, constrain(value, 0, 255));
}

// =====================
// Wi-Fi
// =====================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] connecting");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    feedWatchdog();
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf(" OK %s\n", WiFi.localIP().toString().c_str());
    setStatus("WiFi connected");
  } else {
    Serial.println(" FAILED");
    setStatus("WiFi failed");
  }
}

// =====================
// MQTT
// =====================
void initMqttTopics() {
  if (mqttTopicsReady) return;

  snprintf(topicStatus, sizeof(topicStatus), "devices/%s/status", DEVICE_HOSTNAME);
  snprintf(topicTelemetry, sizeof(topicTelemetry), "devices/%s/telemetry", DEVICE_HOSTNAME);
  snprintf(topicCommand, sizeof(topicCommand), "devices/%s/command", DEVICE_HOSTNAME);
  snprintf(topicCapabilities, sizeof(topicCapabilities), "devices/%s/capabilities", DEVICE_HOSTNAME);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(handleMqttCommand);
  mqttClient.setBufferSize(1024);
  mqttTopicsReady = true;
}

bool handleCustomMqttAction(const char* action, JsonDocument& doc) {
  (void)action;
  (void)doc;
  return false;
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

  if (handleCustomMqttAction(action, doc)) {
    return;
  }

  if (strcmp(action, "led") == 0) {
    if (doc["value"].is<bool>()) {
      setBoardLed(doc["value"]);
    } else if (doc["value"].is<int>()) {
      setBoardLed(doc["value"] != 0);
    }
    Serial.printf("[MQTT] led %s\n", boardLedOn ? "on" : "off");
    publishMqttTelemetry();
    return;
  }

  if (strcmp(action, "status") == 0) {
    Serial.println("[MQTT] status — publish telemetry");
    publishMqttTelemetry();
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
    handlePinWrite(doc["pin"], doc["value"]);
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

  if (strcmp(action, "reboot") == 0) {
    Serial.println("[MQTT] reboot");
    delay(300);
    ESP.restart();
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

  StaticJsonDocument<384> doc;
  doc["uptime"] = millis() / 1000UL;
  doc["heap"] = ESP.getFreeHeap();
  doc["free_heap"] = ESP.getFreeHeap();
  doc["led"] = boardLedOn;
  doc["status"] = statusLine;
  addNetworkTelemetry(doc);
  addFirmwareTelemetry(doc);

  char buf[384];
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

  ota_mqtt::bind(mqttClient, topicTelemetry, ensureMqtt, lastOtaProgress);
  publishOtaEvent("starting", 0);
  lastOtaProgress = 0;

  WiFi.setSleep(WIFI_PS_NONE);

  t_httpUpdate_return ret = ota_mqtt::runUpdate(url);
  if (ret != HTTP_UPDATE_OK) {
    Serial.printf("[OTA] failed: %s\n", httpUpdate.getLastErrorString().c_str());
    int p = lastOtaProgress >= 0 ? lastOtaProgress : 0;
    publishOtaEvent("failed", p);
    setStatus("OTA failed");
  }
}

// =====================
// Device hooks — расширяйте при форке под конкретное устройство
// =====================
void setupDevice() {}

void loopDevice() {}

// =====================
// Setup / loop
// =====================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  setBoardLed(false);

  Serial.begin(115200);
  delay(500);

  Serial.println();
  logFirmwareInfo(DEVICE_LABEL);
  setStatus("Booting");

  initWatchdog();
  blinkBootLed();

  setupDevice();
  connectWiFi();
  ensureMqtt();
}

void loop() {
  feedWatchdog();

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
    ensureMqtt();
    mqttClient.loop();

    if (now - lastMqttTelemetry >= MQTT_TELEMETRY_INTERVAL) {
      lastMqttTelemetry = now;
      publishMqttTelemetry();
    }
  }

  loopDevice();

  delay(50);
}
