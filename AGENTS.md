# AGENTS.md — контекст для LLM

Домашние прошивки ESP32: климат, дисплей, камера, декоративная вывеска. Все устройства публикуют MQTT на `esp32.kuzyak.in`.

## Быстрые ссылки

| Что | Где |
|-----|-----|
| Версии и sha256 последних сборок | `include/firmware_manifest.json` |
| Общие поля телеметрии | `include/firmware_info.h` |
| OTA-прогресс в MQTT | `include/ota_mqtt.h` |
| Сборка бинарников | `scripts/build-ota.sh` |
| Правило версионирования | `.cursor/rules/firmware-versioning.mdc` |
| Человекочитаемая документация | `README.md` |

## Устройства (актуальные версии — смотри манифест)

| id | hostname | sketch | partition | Назначение |
|----|----------|--------|-----------|------------|
| default | `esp32-default` | `esp32_default` | default | Универсальная заготовка для новой платы |
| flat | `esp32-flat` | `esp32_flat_bme280` | default | Комнатный TFT + BME280/AHT/ENS160 |
| balcony | `esp32-balcony` | `esp32_balcony_pms5003_bme280` | default | Балкон: BME280 + PMS5003 + OLED |
| cam | `esp32-cam` | `esp32_cam` | **min_spiffs** | ESP32-CAM, фото на SD |
| flamingo | `esp32-flamingo` | `esp32_flamingo` | default | Вывеска PWM GPIO 13 + гирлянда GPIO 33 + стробоскоп GPIO 25 + кнопка GPIO 27 |

## MQTT

### Топики

| Топик | Retain | Назначение |
|-------|--------|------------|
| `devices/<hostname>/status` | да (LWT) | `{"status":"online"}` / `offline` |
| `devices/<hostname>/telemetry` | нет | Периодическая и событийная телеметрия |
| `devices/<hostname>/capabilities` | да | JSON: `commands[]`, `metrics[]`, `dashboard` |
| `devices/<hostname>/command` | — | Входящие команды `{"action":"...", "value":...}` |
| `devices/esp32-flat/out/flamingo` | нет | Relay flat → flamingo (ACL брокера) |

### Телеметрия — обязательные правила

1. **Период:** каждые 10 с (`MQTT_TELEMETRY_INTERVAL`).
2. **Сразу после MQTT-connect:** `publishMqttTelemetry()` в `ensureMqtt()` — сбрасывает устаревшее состояние в дашборде.
3. **Сразу после изменения toggle-состояния:** любая команда, меняющая поле из `metrics` (`led`, `light`, `sign`, `garland`, `brightness` и т.д.), должна вызывать `publishMqttTelemetry()` до `return`.
4. **Локальные переключатели** (кнопка flamingo на flat и на самом flamingo) — тоже публикуют телеметрию сразу.
5. Телеметрия **не retained** — дашборд хранит последнее значение у себя; при ребуте до первого пакета может показать старое состояние.

### Встроенный LED (GPIO 2)

- **По умолчанию выключен** (`boardLedOn = false`, `setBoardLed(false)` в `setup()`).
- Active **LOW** на большинстве DevKit: `digitalWrite(LED_BUILTIN, on ? LOW : HIGH)`.
- Исключение: **cam** — поле `led` в телеметрии = вспышка GPIO 4 (active HIGH).
- Краткое мигание при загрузке только в `esp32_default` (`blinkBootLed()`).

### Команда `status` — разная семантика

| Скетч | `{"action":"status"}` |
|-------|----------------------|
| default | Немедленно публикует телеметрию |
| flat, balcony | Показывает системный экран на дисплее |
| flamingo, cam | Команды нет в capabilities |

## Изменение прошивки

1. Читай `include/firmware_manifest.json` перед работой.
2. Меняешь `.ino` / логику → patch-bump `FW_VERSION` в `include/firmware_info.h` и `<sketch>/firmware_info.h`, обнови `"version"` в манифесте.
3. После релизной сборки: `./scripts/build-ota.sh <id>` → закоммить обновлённый `firmware_manifest.json` (`last_build` пишет скрипт).
4. `ota/` в `.gitignore` — бинарники локальные.

## Flamingo ↔ Flat

- Flat не может писать в `devices/esp32-flamingo/command` (ACL Mosquitto).
- Flat шлёт `{"action":"sign","value":bool}` в `devices/esp32-flat/out/flamingo`.
- Flamingo подписан на свой `command` и на relay-топик flat.
- Flat подписан на `devices/esp32-flamingo/telemetry` и синхронизирует `flamingoSignOn` перед toggle кнопкой.

## Типичные ошибки (не повторять)

- Добавить toggle-команду без немедленного `publishMqttTelemetry()`.
- Забыть `publishMqttTelemetry()` в `ensureMqtt()` после connect.
- Считать, что `led` включён по умолчанию — в коде всегда `false`; «включён» в UI = устаревшая телеметрия.
- Путать GPIO 33: кнопка на flat (вывеска), гирлянда на flamingo.
- Для cam использовать partition Huge APP или Default — нужен **min_spiffs**.
