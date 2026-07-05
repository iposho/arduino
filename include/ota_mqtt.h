#pragma once

#include <PubSubClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

// Публикация OTA-прогресса в devices/<id>/telemetry.
// Формат: {"ota":"downloading","progress":40}
// ensure() переподключает MQTT, если сокет был закрыт перед HTTPS-загрузкой.

namespace ota_mqtt {

struct Bindings {
  PubSubClient* client = nullptr;
  const char* topic = nullptr;
  bool (*ensure)() = nullptr;
  void (*ensureVoid)() = nullptr;
  int* lastProgress = nullptr;
};

inline Bindings& bindings() {
  static Bindings b;
  return b;
}

inline void bind(PubSubClient& client, const char* topic,
                 bool (*ensure)(), int& lastProgress) {
  auto& b = bindings();
  b.client = &client;
  b.topic = topic;
  b.ensure = ensure;
  b.ensureVoid = nullptr;
  b.lastProgress = &lastProgress;
}

inline void bind(PubSubClient& client, const char* topic,
                 void (*ensure)(), int& lastProgress) {
  auto& b = bindings();
  b.client = &client;
  b.topic = topic;
  b.ensure = nullptr;
  b.ensureVoid = ensure;
  b.lastProgress = &lastProgress;
}

inline void reconnectIfNeeded() {
  auto& b = bindings();
  if (b.client && b.client->connected()) return;
  if (b.ensure) {
    b.ensure();
  } else if (b.ensureVoid) {
    b.ensureVoid();
  }
}

inline void publish(const char* phase, int progress) {
  reconnectIfNeeded();
  auto& b = bindings();
  if (!b.client || !b.topic || !b.client->connected()) return;

  StaticJsonDocument<160> doc;
  doc["ota"] = phase;

  if (progress >= 0) {
    doc["progress"] = progress;
  } else if (phase && strcmp(phase, "failed") == 0 &&
             b.lastProgress && *b.lastProgress >= 0) {
    doc["progress"] = *b.lastProgress;
  } else if (phase && strcmp(phase, "failed") != 0) {
    doc["progress"] = 0;
  }

  char buf[160];
  size_t n = serializeJson(doc, buf);
  b.client->publish(b.topic, buf, n);
  b.client->loop();
}

inline void onStart() {
  auto& b = bindings();
  if (b.lastProgress) *b.lastProgress = 0;
  publish("downloading", 0);
}

inline void onProgress(size_t current, size_t total) {
  auto& b = bindings();
  int pct = (total > 0) ? (int)((current * 100UL) / total) : 0;
  if (!b.lastProgress || pct >= *b.lastProgress + 1 || pct == 100 || *b.lastProgress < 0) {
    if (b.lastProgress) *b.lastProgress = pct;
    publish("downloading", pct);
  }
}

inline void onEnd() {
  auto& b = bindings();
  if (b.lastProgress) *b.lastProgress = 100;
  publish("rebooting", 100);
}

inline void onError(int error) {
  (void)error;
  auto& b = bindings();
  int p = (b.lastProgress && *b.lastProgress >= 0) ? *b.lastProgress : 0;
  publish("failed", p);
}

inline void attachHttpCallbacks() {
  httpUpdate.onStart([]() { onStart(); });
  httpUpdate.onProgress([](size_t current, size_t total) { onProgress(current, total); });
  httpUpdate.onEnd([]() { onEnd(); });
  httpUpdate.onError([](int error) { onError(error); });
}

inline t_httpUpdate_return runUpdate(const char* url) {
  httpUpdate.rebootOnUpdate(true);
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  attachHttpCallbacks();

  if (strncmp(url, "https://", 8) == 0) {
    WiFiClientSecure secureClient;
    secureClient.setInsecure();
    secureClient.setTimeout(30000);
    return httpUpdate.update(secureClient, url);
  }

  WiFiClient client;
  client.setTimeout(30000);
  return httpUpdate.update(client, url);
}

}  // namespace ota_mqtt
