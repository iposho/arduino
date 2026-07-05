// secrets.h — скопируйте этот файл как secrets.h и заполните свои данные
#pragma once

#define WIFI_SSID        "your_wifi_ssid"
#define WIFI_PASS        "your_wifi_password"
#define DEVICE_HOSTNAME  "esp32-flat"      // имя в списке клиентов роутера + MQTT device id

// MQTT-шлюз esp32.kuzyak.in (Mosquitto на Raspberry Pi)
#define MQTT_HOST        "192.168.100.43"
#define MQTT_PORT        1883
#define MQTT_USER        "esp32"
#define MQTT_PASS        "change-me-esp32"

// MQTT id вывески «фламинго» (кнопка на flat шлёт команды сюда)
#ifndef FLAMINGO_HOSTNAME
#define FLAMINGO_HOSTNAME "esp32-flamingo"
#endif

// Mosquitto ACL (пользователь esp32): flat должен иметь
//   write  devices/<DEVICE_HOSTNAME>/out/flamingo   — relay для кнопки (см. topicFlamingoRelay)
//   write  devices/<FLAMINGO_HOSTNAME>/command      — прямой путь (часто запрещён ACL)
//   read   devices/<FLAMINGO_HOSTNAME>/telemetry    — подтверждение состояния
// Альтернатива без ACL: Node-RED bridge devices/+/out/flamingo → devices/esp32-flamingo/command

// Supabase REST API (чтение уличных данных + запись комнатных)
#define SUPABASE_URL     "https://your-project.supabase.co/rest/v1"
#define SUPABASE_KEY     "your-supabase-anon-key"
