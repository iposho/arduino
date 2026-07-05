// secrets.h — скопируйте этот файл как secrets.h и заполните свои данные
#pragma once

#define WIFI_SSID        "your_wifi_ssid"
#define WIFI_PASS        "your_wifi_password"
#define DEVICE_HOSTNAME  "esp32-flamingo"   // имя в роутере + MQTT device id

// MQTT-шлюз esp32.kuzyak.in (Mosquitto на Raspberry Pi)
#define MQTT_HOST        "192.168.100.43"
#define MQTT_PORT        1883
#define MQTT_USER        "esp32"
#define MQTT_PASS        "change-me-esp32"

// Mosquitto ACL (пользователь esp32): flamingo должен иметь
//   read   devices/<FLAT_HOSTNAME>/out/flamingo   — relay от кнопки на flat (см. topicFlatRelay)
//   read   devices/<DEVICE_HOSTNAME>/command
//   write  devices/<DEVICE_HOSTNAME>/telemetry|status|capabilities
// FLAT_HOSTNAME — hostname flat в secrets.h (по умолчанию esp32-flat)
#ifndef FLAT_HOSTNAME
#define FLAT_HOSTNAME "esp32-flat"
#endif
