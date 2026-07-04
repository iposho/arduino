#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>

#include "secrets.h"
#include "firmware_info.h"

// MQTT-топики (DEVICE_HOSTNAME из secrets.h):
//   devices/<hostname>/status       — online/offline (LWT)
//   devices/<hostname>/telemetry    — периодическая телеметрия
//   devices/<hostname>/command      — JSON-команды
//   devices/<hostname>/capabilities — retained JSON c описанием команд

// =====================
// Пины
// =====================
// Встроенный LED DevKit (GPIO 2, active LOW)
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// LED-лампа на 2 провода (+/−), транзистор/резистор внутри корпуса.
// Подключение:
//   (+) красный  → GPIO 13  (можно через резистор 100–330 Ω, если ярко/греется)
//   (−) чёрный   → GND
// HIGH на GPIO 13 = вкл, LOW = выкл.
// Не подключайте к VIN — иначе всегда горит и GPIO не управляет.
#define LAMP_PIN         13
#define LAMP_ACTIVE_LOW  false

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
bool mqttTopicsReady = false;

const char *CAPABILITIES = R"CAP({
  "commands": [
    {
      "action": "light",
      "title": "Лампа GPIO13",
      "type": "toggle",
      "icon": "lightbulb",
      "description": "Вкл/выкл лампу на GPIO13 (+/−)"
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
  ]
})CAP";

bool boardLedOn = false;
bool lampOn = false;

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
void setLamp(bool on);
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

void setLamp(bool on) {
  lampOn = on;
  bool level = LAMP_ACTIVE_LOW ? !on : on;
  digitalWrite(LAMP_PIN, level ? HIGH : LOW);
  Serial.printf("[Lamp] %s\n", on ? "ON" : "OFF");
}

bool isGpioPinAllowed(uint8_t pin) {
  if (pin > 39) return false;
  if (pin >= 6 && pin <= 11) return false;  // flash
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
    if (pin == LAMP_PIN) {
      setLamp(value != 0);
    } else if (pin == LED_BUILTIN) {
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

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(handleMqttCommand);
  mqttClient.setBufferSize(1024);
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

  if (strcmp(action, "light") == 0 || strcmp(action, "lamp") == 0 || strcmp(action, "relay") == 0) {
    if (doc["value"].is<bool>()) {
      setLamp(doc["value"]);
    } else if (doc["value"].is<int>()) {
      setLamp(doc["value"] != 0);
    }
    return;
  }

  if (strcmp(action, "led") == 0) {
    if (doc["value"].is<bool>()) {
      setBoardLed(doc["value"]);
    } else if (doc["value"].is<int>()) {
      setBoardLed(doc["value"] != 0);
    }
    Serial.printf("[MQTT] led %s\n", boardLedOn ? "on" : "off");
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
  } else {
    Serial.printf("[MQTT] connect failed, rc=%d\n", mqttClient.state());
  }
}

void publishMqttTelemetry() {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<256> doc;
  doc["uptime"] = millis() / 1000UL;
  doc["rssi"] = WiFi.RSSI();
  doc["heap"] = ESP.getFreeHeap();
  doc["light"] = lampOn;
  doc["led"] = boardLedOn;
  addFirmwareTelemetry(doc);

  char buf[256];
  size_t n = serializeJson(doc, buf);
  mqttClient.publish(topicTelemetry, buf, n);
}

void publishOtaEvent(const char* phase, int progress) {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<128> doc;
  doc["ota"] = phase;
  if (progress >= 0) doc["progress"] = progress;

  char buf[128];
  size_t n = serializeJson(doc, buf);
  mqttClient.publish(topicTelemetry, buf, n);
  mqttClient.loop();
}

void queueOtaUpdate(const char* url) {
  strncpy(otaUrl, url, sizeof(otaUrl) - 1);
  otaUrl[sizeof(otaUrl) - 1] = '\0';
  otaPending = true;
}

void performOtaUpdate(const char* url) {
  Serial.printf("[OTA] starting: %s (heap=%u)\n", url, ESP.getFreeHeap());
  setStatus("OTA starting");
  publishOtaEvent("starting", 0);
  lastOtaProgress = 0;

  if (mqttClient.connected()) {
    mqttClient.disconnect();
    delay(200);
  }
  WiFi.setSleep(WIFI_PS_NONE);

  httpUpdate.rebootOnUpdate(true);
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  httpUpdate.onStart([]() {
    Serial.println("[OTA] start");
    publishOtaEvent("downloading", 0);
    lastOtaProgress = 0;
  });

  httpUpdate.onProgress([](size_t current, size_t total) {
    int pct = (total > 0) ? (int)((current * 100UL) / total) : 0;
    if (pct >= lastOtaProgress + 10 || pct == 100) {
      lastOtaProgress = pct;
      publishOtaEvent("downloading", pct);
      setStatus("OTA downloading");
    }
  });

  httpUpdate.onEnd([]() {
    Serial.println("[OTA] complete");
    publishOtaEvent("rebooting", 100);
    setStatus("OTA rebooting");
  });

  httpUpdate.onError([](int error) {
    Serial.printf("[OTA] error %d: %s\n", error, httpUpdate.getLastErrorString().c_str());
    publishOtaEvent("failed", -1);
    setStatus("OTA failed");
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
  }
}

// =====================
// Setup / loop
// =====================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("ESP32 Lamp Controller");
  logFirmwareInfo("esp32-lamp");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LAMP_PIN, OUTPUT);
  setBoardLed(false);
  setLamp(false);

  connectWiFi();
  ensureMqtt();
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
    ensureMqtt();
    mqttClient.loop();

    if (now - lastMqttTelemetry >= MQTT_TELEMETRY_INTERVAL) {
      lastMqttTelemetry = now;
      publishMqttTelemetry();
    }
  }

  delay(50);
}
