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

#define SUPABASE_URL     "https://your-project.supabase.co/rest/v1"
#define SUPABASE_KEY     "your-supabase-anon-key"

// Токен бота и chat-id (для отправки алертов)
#define TELEGRAM_TOKEN   "your-telegram-bot-token"
#define TELEGRAM_CHAT_ID "your-telegram-chat-id"
