#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>

#include "secrets.h"
#include "firmware_info.h"
#include "../include/ota_mqtt.h"

// MQTT-топики (DEVICE_HOSTNAME из secrets.h):
//   devices/<hostname>/status       — online/offline (LWT)
//   devices/<hostname>/telemetry    — периодическая телеметрия
//   devices/<hostname>/command      — JSON-команды
//   devices/<hostname>/capabilities — retained JSON c описанием команд
//
// Кнопка на esp32-flat не может писать в devices/esp32-flamingo/command (ACL брокера),
// поэтому flat дублирует команду в свой relay-топик — flamingo подписывается и сюда.
#ifndef FLAT_HOSTNAME
#define FLAT_HOSTNAME "esp32-flat"
#endif

#define MQTT_BUFFER_SIZE 4096

// =====================
// Пины
// =====================
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// Неоновая вывеска «фламинго» на 2 провода (+/−):
//   (+) красный  → GPIO 13
//   (−) чёрный   → GND
// Яркость — PWM на GPIO 13 (0 = выкл, 1–255 = диммирование).
#define SIGN_PIN           13
#define SIGN_DEFAULT_LEVEL 255

// Гирлянда на GPIO 33: (+) → GPIO 33, (−) → GND. HIGH = вкл.
#define GARLAND_PIN 33

// Стробоскоп на GPIO 25: (+) → GPIO 25, (−) → GND. HIGH = вкл.
#define STROBE_PIN  25

// Двухтактная кнопка (2 провода): GPIO 27 → кнопка → GND. INPUT_PULLUP.
//   одно нажатие — гирлянда вкл/выкл
//   двойное      — стробоскоп вкл/выкл
#define BUTTON_PIN          27
#define BUTTON_DEBOUNCE_MS  60UL    // антидребезг (не «удержание»!)
#define BUTTON_DOUBLE_MS    350UL   // окно двойного нажатия

// =====================

// =====================
// Intervals
// =====================
const unsigned long MQTT_TELEMETRY_INTERVAL = 10UL * 1000UL;
const unsigned long WIFI_RETRY_INTERVAL     = 30UL * 1000UL;

// =====================
// State
// =====================
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

char topicStatus[64];
char topicTelemetry[64];
char topicCommand[64];
char topicCapabilities[64];
char topicFlatRelay[64];
bool mqttTopicsReady = false;

const char *CAPABILITIES = R"CAP({
  "commands": [
    {
      "action": "sign",
      "title": "Фламинго",
      "type": "toggle",
      "icon": "sparkles",
      "description": "Вкл/выкл вывеску на GPIO13"
    },
    {
      "action": "brightness",
      "title": "Яркость",
      "type": "range",
      "icon": "sun",
      "min": 0,
      "max": 255,
      "description": "PWM 0–255 на GPIO13 (0 = выкл)"
    },
    {
      "action": "garland",
      "title": "Гирлянда",
      "type": "toggle",
      "icon": "tree-pine",
      "description": "Вкл/выкл гирлянду на GPIO33 (кнопка: одно нажатие)"
    },
    {
      "action": "strobe",
      "title": "Стробоскоп",
      "type": "toggle",
      "icon": "zap",
      "description": "Вкл/выкл стробоскоп на GPIO25 (кнопка: двойное нажатие)"
    },
    {
      "action": "led",
      "title": "Светодиод",
      "type": "toggle",
      "icon": "circle",
      "description": "Встроенный светодиод на GPIO (LED_BUILTIN)"
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
    { "key": "sign", "label": "Вывеска", "icon": "sparkles", "format": "boolean", "group": "Устройство", "dashboard": true, "order": 4 },
    { "key": "brightness", "label": "Яркость", "icon": "sun", "format": "number", "unit": "%", "group": "Устройство", "dashboard": true, "order": 5 },
    { "key": "garland", "label": "Гирлянда", "icon": "tree-pine", "format": "boolean", "group": "Устройство", "dashboard": true, "order": 6 },
    { "key": "strobe", "label": "Стробоскоп", "icon": "zap", "format": "boolean", "group": "Устройство", "dashboard": true, "order": 7 },
    { "key": "led", "label": "Светодиод", "icon": "circle", "format": "boolean", "group": "Устройство", "order": 10 },
    { "key": "fw_version", "label": "Версия прошивки", "icon": "cpu", "group": "Система", "order": 20 }
  ],
  "dashboard": {
    "summary": ["sign", "garland", "strobe", "brightness"],
    "max_items": 4
  }
})CAP";

bool boardLedOn = false;
bool signOn = false;
bool garlandOn = false;
bool strobeOn = false;
uint8_t signLevel = 0;

// Кнопка
bool btnLastRead = HIGH;
bool btnStable = HIGH;
unsigned long btnLastChangeMs = 0;
unsigned long btnLastPressMs = 0;   // время нажатия — для окна двойного клика (0 = нет)

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
void applySignOutput();
void setSignOn(bool on);
void setSignLevel(uint8_t level);
void setGarland(bool on);
void setStrobe(bool on);
void handleButton();
void checkButtonSingle();
void connectWiFi();
void initMqttTopics();
void ensureMqtt();
void publishMqttTelemetry();
void publishOtaEvent(const char* phase, int progress = -1);
void queueOtaUpdate(const char* url);
void performOtaUpdate(const char* url);
void handleMqttCommand(char* topic, byte* payload, unsigned int length);
bool isGpioPinAllowed(uint8_t pin);
bool isPinOutputCapable(uint8_t pin);
bool applyPinMode(uint8_t pin, const char* mode);
void publishPinRead(uint8_t pin);
void handlePinWrite(uint8_t pin, int value);
int parseBrightnessValue(JsonVariant value);

// =====================
// Helpers
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

void applySignOutput() {
  if (!signOn || signLevel == 0) {
    pinMode(SIGN_PIN, OUTPUT);
    digitalWrite(SIGN_PIN, LOW);
    Serial.println("[Sign] OFF (GPIO13 LOW)");
  } else {
    analogWrite(SIGN_PIN, signLevel);
    Serial.printf("[Sign] ON level=%u\n", signLevel);
  }
}

void setSignOn(bool on) {
  signOn = on;
  if (on && signLevel == 0) {
    signLevel = SIGN_DEFAULT_LEVEL;
  }
  applySignOutput();
}

void setSignLevel(uint8_t level) {
  signLevel = level;
  signOn = level > 0;
  applySignOutput();
}

void setGarland(bool on) {
  garlandOn = on;
  digitalWrite(GARLAND_PIN, on ? HIGH : LOW);
  Serial.printf("[Garland] %s\n", on ? "ON" : "OFF");
}

void setStrobe(bool on) {
  strobeOn = on;
  digitalWrite(STROBE_PIN, on ? HIGH : LOW);
  Serial.printf("[Strobe] %s\n", on ? "ON" : "OFF");
}

// =====================
// Кнопка: один клик — гирлянда, двойной — стробоскоп
// =====================
void handleButton() {
  bool reading = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  // Антидребезг: ждём, пока пин устоится
  if (reading != btnLastRead) {
    btnLastRead = reading;
    btnLastChangeMs = now;
    return;
  }
  if (now - btnLastChangeMs < BUTTON_DEBOUNCE_MS) return;
  if (reading == btnStable) return;   // состояния не изменилось

  btnStable = reading;

  // Реагируем только на НАЖАТИЕ
  if (btnStable == LOW) {
    if (btnLastPressMs != 0 && (now - btnLastPressMs) <= BUTTON_DOUBLE_MS) {
      // Второе нажатие в окне — двойной клик
      btnLastPressMs = 0;
      Serial.println("[Button] Double press → strobe");
      setStrobe(!strobeOn);
      publishMqttTelemetry();
    } else {
      btnLastPressMs = now;
      Serial.println("[Button] Press");
    }
  }
}

// Одиночное нажатие — срабатывает, если второе не пришло в окне
void checkButtonSingle() {
  if (btnLastPressMs != 0 && (millis() - btnLastPressMs > BUTTON_DOUBLE_MS)) {
    btnLastPressMs = 0;
    Serial.println("[Button] Single press → garland");
    setGarland(!garlandOn);
    publishMqttTelemetry();
  }
}

int parseBrightnessValue(JsonVariant value) {
  if (value.is<bool>()) {
    return value.as<bool>() ? SIGN_DEFAULT_LEVEL : 0;
  }
  if (value.is<int>()) {
    return constrain(value.as<int>(), 0, 255);
  }
  return -1;
}

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
    if (pin == SIGN_PIN) return true;
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
  if (pin == SIGN_PIN) {
    doc[keyDig] = signOn ? 1 : 0;
    doc[keyAna] = signOn ? signLevel : 0;
  } else {
    doc[keyDig] = digitalRead(pin);
    doc[keyAna] = analogRead(pin);
  }

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

  if (pin == SIGN_PIN) {
    if (value <= 1) {
      setSignOn(value != 0);
    } else {
      setSignLevel(constrain(value, 0, 255));
    }
    Serial.printf("[MQTT] pin_write pin=%u value=%d\n", pin, value);
    return;
  }

  if (pin == GARLAND_PIN) {
    if (value <= 1) {
      setGarland(value != 0);
    } else {
      setGarland(value > 0);
    }
    Serial.printf("[MQTT] pin_write pin=%u value=%d\n", pin, value);
    return;
  }

  if (pin == STROBE_PIN) {
    if (value <= 1) {
      setStrobe(value != 0);
    } else {
      setStrobe(value > 0);
    }
    Serial.printf("[MQTT] pin_write pin=%u value=%d\n", pin, value);
    return;
  }

  if (value <= 1) {
    if (!isPinOutputCapable(pin)) return;
    if (pin == LED_BUILTIN) {
      setBoardLed(value != 0);
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
// MQTT
// =====================
void initMqttTopics() {
  if (mqttTopicsReady) return;

  snprintf(topicStatus, sizeof(topicStatus), "devices/%s/status", DEVICE_HOSTNAME);
  snprintf(topicTelemetry, sizeof(topicTelemetry), "devices/%s/telemetry", DEVICE_HOSTNAME);
  snprintf(topicCommand, sizeof(topicCommand), "devices/%s/command", DEVICE_HOSTNAME);
  snprintf(topicCapabilities, sizeof(topicCapabilities), "devices/%s/capabilities", DEVICE_HOSTNAME);
  snprintf(topicFlatRelay, sizeof(topicFlatRelay),
           "devices/%s/out/flamingo", FLAT_HOSTNAME);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(handleMqttCommand);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  mqttTopicsReady = true;
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

  if (strcmp(action, "sign") == 0 ||
      strcmp(action, "light") == 0 ||
      strcmp(action, "relay") == 0) {
    if (doc["value"].is<bool>()) {
      setSignOn(doc["value"]);
    } else if (doc["value"].is<int>()) {
      setSignOn(doc["value"] != 0);
    }
    publishMqttTelemetry();
    return;
  }

  if (strcmp(action, "brightness") == 0) {
    int level = parseBrightnessValue(doc["value"]);
    if (level >= 0) {
      setSignLevel(static_cast<uint8_t>(level));
      publishMqttTelemetry();
    }
    return;
  }

  if (strcmp(action, "garland") == 0) {
    if (doc["value"].is<bool>()) {
      setGarland(doc["value"]);
    } else if (doc["value"].is<int>()) {
      setGarland(doc["value"] != 0);
    }
    publishMqttTelemetry();
    return;
  }

  if (strcmp(action, "strobe") == 0) {
    if (doc["value"].is<bool>()) {
      setStrobe(doc["value"]);
    } else if (doc["value"].is<int>()) {
      setStrobe(doc["value"] != 0);
    }
    publishMqttTelemetry();
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
    mqttClient.subscribe(topicFlatRelay, 1);
    if (mqttClient.publish(topicCapabilities, CAPABILITIES, true)) {
      Serial.printf("[MQTT] capabilities -> %s\n", topicCapabilities);
    } else {
      Serial.printf("[MQTT] capabilities publish FAILED (%u bytes)\n", strlen(CAPABILITIES));
    }
    Serial.println("[MQTT] connected");
    Serial.printf("[MQTT] command  <- %s\n", topicCommand);
    Serial.printf("[MQTT] relay    <- %s\n", topicFlatRelay);
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
  doc["sign"] = signOn;
  doc["brightness"] = signOn ? (signLevel * 100 + 127) / 255 : 0;
  doc["garland"] = garlandOn;
  doc["strobe"] = strobeOn;
  doc["led"] = boardLedOn;
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
// Setup / loop
// =====================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  setBoardLed(false);

  pinMode(SIGN_PIN, OUTPUT);
  pinMode(GARLAND_PIN, OUTPUT);
  pinMode(STROBE_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  setSignLevel(0);
  setGarland(false);
  setStrobe(false);

  btnLastRead = digitalRead(BUTTON_PIN);
  btnStable = btnLastRead;
  btnLastChangeMs = millis();
  btnLastPressMs = 0;

  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("ESP32 Flamingo Sign Controller");
  logFirmwareInfo("esp32-flamingo");

  connectWiFi();
  ensureMqtt();
}

void loop() {
  if (otaPending) {
    otaPending = false;
    performOtaUpdate(otaUrl);
    return;
  }

  handleButton();
  checkButtonSingle();

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

  delay(10);
}
