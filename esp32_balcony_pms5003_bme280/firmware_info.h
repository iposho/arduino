#pragma once

#include <WiFi.h>
#include <ArduinoJson.h>

#ifndef FW_VERSION
#define FW_VERSION "1.2.0"
#endif

#define FW_BUILD_DATE __DATE__
#define FW_BUILD_TIME __TIME__

inline void logFirmwareInfo(const char* device) {
  Serial.printf("[FW] %s version=%s build=%s %s\n",
                device, FW_VERSION, FW_BUILD_DATE, FW_BUILD_TIME);
}

inline void addFirmwareTelemetry(JsonDocument& doc) {
  doc["fw_version"] = FW_VERSION;
  doc["fw_build"] = FW_BUILD_DATE " " FW_BUILD_TIME;
}

inline void addNetworkTelemetry(JsonDocument& doc) {
  if (WiFi.status() == WL_CONNECTED) {
    doc["ip"] = WiFi.localIP().toString();
    doc["wifi_ssid"] = WiFi.SSID();
    doc["rssi"] = WiFi.RSSI();
  } else {
    doc["ip"] = "";
    doc["wifi_ssid"] = "";
    doc["rssi"] = 0;
  }
}
