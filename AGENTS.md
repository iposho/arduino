# AGENTS.md — контекст для LLM

Домашние прошивки ESP32: климат, дисплей, декоративная вывеска. Все устройства публикуют MQTT на `esp32.kuzyak.in`.

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
| birdcam | `esp32-bird-cam` | `esp32_bird_cam` | **min_spiffs** | ESP32-CAM у кормушки: кадр 1 fps в RAM (`/latest.jpg`), детектор движения, снимки визитов на SD (`/photo?id=`, `/birds.json`) |
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
- Исключение: **birdcam** — поле `led` в телеметрии = вспышка GPIO 4 (active HIGH).
- Краткое мигание при загрузке только в `esp32_default` (`blinkBootLed()`).

### Команда `status` — разная семантика

| Скетч | `{"action":"status"}` |
|-------|----------------------|
| default | Немедленно публикует телеметрию |
| flat, balcony | Показывает системный экран на дисплее |
| flamingo, birdcam | Команды нет в capabilities |

## birdcam (esp32-bird-cam)

- Команды цвета сенсора (`cam_ae`, `cam_saturation`, `cam_wb`, `cam_brightness`, `cam_contrast`, `cam_quality`, сброс `cam_reset`) — с дашборда, хранятся в LittleFS `/cam_tuning.dat` (сохранённые поля видны в телеметрии `cam_*`, а не в HTML статуса).
  - Каждая команда сохраняет настройку, применяет её и **сбрасывает фон детектора** — иначе пересвет выглядит как движение.
- Служебные `fs_ls` / `fs_read` / `fs_write` / `fs_rm` (`path`, `content`) для LittleFS (в `capabilities` не публикуются).
- Файлы состояния: `/cam_tuning.dat`, `/cam_rgb.flag`, `/bird_log.dat`, `/bird_visits.dat`, `/photo_index.dat`.
- Телеметрия дополнительно: `sd_photos` (число снимков на карте), `cam_mode` (`jpeg` / `rgb565`), `cam_*`, `reset_reason`, `crash` (выжимка coredump: `pc`, `cause`, `bt`). Адреса `bt` разворачиваются через `addr2line` и `.elf` этой сборки.
- Детектор — разница кадра 1/8 с медленным фоном; птицу определяет шлюз нейронкой. Снимки на SD — только по движению и `capture`, не чаще раза в 10 с.
- SD под кольцо меньше 4800 фото: при свободном месте < 4 МБ вытесняются самые старые `/photos/NNNNN.jpg`.
- OTA-сторож: нет новых байт > 60 с — ребут (старая прошивка остаётся в слоте). Во время скачивания не переподключать MQTT и не обслуживать веб-сервер (сканирование FAT и `/latest.jpg` рвут загрузку).

## Изменение прошивки

1. Читай `include/firmware_manifest.json` перед работой.
2. Меняешь `.ino` / логику → patch-bump `FW_VERSION` в `include/firmware_info.h` и `<sketch>/firmware_info.h`, обнови `"version"` в манифесте.
3. После релизной сборки: `./scripts/build-ota.sh <id>` → закоммить обновлённый `firmware_manifest.json` (`last_build` пишет скрипт).
4. `ota/` в `.gitignore` — бинарники локальные.

## flat: экран кормушки

- `SCREEN_BIRD` рисует последнюю подтверждённую птицу: `GET <GATEWAY_URL>/api/camera/birdfeeder/tft` (репозиторий шлюза, `lib/bird-tft.ts`) с `Authorization: Bearer <CAMERA_API_TOKEN>` — оба в `secrets.h`; `GATEWAY_URL` по умолчанию `http://<MQTT_HOST>:3000`.
- Ответ — сырой RGB565 little-endian 160×120, плата льёт его построчно в дисплей (JPEG на плате не декодируется). Подпись — из заголовков `X-Species` (латынь: в шрифте нет кириллицы) и `X-Shot-At`.
- Картинка в памяти не хранится: при каждом входе на экран качается заново, дальше раз в 30 с запрос с `If-None-Match` (304, пока птица та же).
- На этом экране `setStatus()` не рисует строку статуса — она легла бы поверх снимка.

## Flamingo ↔ Flat

- Flat не может писать в `devices/esp32-flamingo/command` (ACL Mosquitto).
- Flat шлёт `{"action":"sign","value":bool}` в `devices/esp32-flat/out/flamingo`.
- Flamingo подписан на свой `command` и на relay-топик flat.
- Flat подписан на `devices/esp32-flamingo/telemetry` и синхронизирует `flamingoSignOn` перед toggle кнопкой.

## Выведенные из эксплуатации

`esp32_cam` удалён из репозитория (2026-09). Плата ESP32-CAM теперь работает как `esp32-bird-cam` (кормушка); актуальная прошивка — `esp32_bird_cam` в этом репозитории, цель сборки — `birdcam`, partition — **min_spiffs**. Старый `esp32_cam` не восстанавливать. Ранее удалены `esp32_lamp`, `esp32_cam_stream`, `esp32_growbox`, `esp32_bedroom`; их `device_id` в шлюзе остаются в `deleted_devices`.

## Типичные ошибки (не повторять)

- Добавить toggle-команду без немедленного `publishMqttTelemetry()`.
- Забыть `publishMqttTelemetry()` в `ensureMqtt()` после connect.
- Считать, что `led` включён по умолчанию — в коде всегда `false`; «включён» в UI = устаревшая телеметрия.
- Путать GPIO 33: кнопка на flat (вывеска), гирлянда на flamingo.
- Для birdcam использовать partition Huge APP или Default — нужен **min_spiffs**.
- birdcam: сначала аппаратный JPEG SVGA (XGA на OV3660 не работает — FB-OVF, проверено в 1.3.1); если 5 кадров подряд не пришло (у OV3660 на AI-Thinker — `cam_hal: FB-OVF`), прошивка сама переходит на RGB565 VGA + программный JPEG (`cam_mode` в телеметрии). Детектор работает на кадре 1/8 (≤100×75, `MOTION_MAX_*`, буфер должен вмещать весь кадр / 8).
- birdcam: `capture` сохраняет текущий кадр на SD; периодической съёмки на SD больше нет — только визиты птиц (не чаще раза в 10 с).
- birdcam: `fmt2rgb888` (esp_jpeg) отдаёт пиксели R,G,B, а `fmt2jpg(PIXFORMAT_RGB888)` читает B,G,R — перед кодированием переставлять байты 0 и 2 (в 1.4.0 этого не было: синяя кормушка, персиковое небо).
- birdcam: розовые пересветы OV3660 (G клипуется раньше R/B) гасит `fixPinkHighlights565()` в RGB565-кадре до `frame2jpg`; на аппаратный JPEG не действует.
- birdcam: не проверять снимки через `SD_MMC.exists()` в цикле — поиск в FAT линейный по папке (на отсутствующем файле три прохода), при тысячах фото плата виснет на минуты. Наличие снимка — `photoExists()` (битовая карта, заполняется одним `readdir` при монтировании); после записи/удаления — `photoMark()`.
- birdcam: SD монтируется после Wi-Fi/MQTT — медленная карта не должна прятать плату из сети.
