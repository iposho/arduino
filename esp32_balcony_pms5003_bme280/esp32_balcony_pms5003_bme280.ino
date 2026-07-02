#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PMS.h>
#include <algorithm>
#include <cmath>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <stdint.h>
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =====================
// Секреты — вынесены в secrets.h (.gitignore)
// =====================
#include "secrets.h"

// =====================
// Константы железа
// =====================
#define PMS_RX 16
#define PMS_TX 17

// Пины для RGB Светофора
#define LED_R  25
#define LED_G  26
#define LED_Y  27   // Желтый (подписан B на плате, но реально Y)

#define BME280_ADDR    0x76
#define TEMP_MIN      -40.0f
#define TEMP_MAX       80.0f
#define HUM_MIN         0.0f
#define HUM_MAX       100.0f
#define PRES_MIN      600.0f   // mmHg
#define PRES_MAX      900.0f
#define PA_TO_MMHG      0.75006375f

#define PMS_TIMEOUT_MS   2000UL
#define PMS_MAX_ERRORS      3
#define WDT_TIMEOUT_SEC    30
#define HTTP_TIMEOUT_MS    8000UL

// NTP (UTC+4)
#define NTP_SERVER         "pool.ntp.org"
#define GMT_OFFSET_SEC     (4 * 3600)
#define DAYLIGHT_OFFSET    0

// OLED SSD1306 128x64 (отдельная I2C шина, пины 18/19)
#define OLED_ADDR       0x3C
#define OLED_SDA        18
#define OLED_SCL        19
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1

// Страницы дисплея
#define OLED_PAGE_CLIMATE   0   // Показания BME280 + расписание замеров
#define OLED_PAGE_DUST      1   // PM1/2.5/10 + расписание PMS
#define OLED_PAGE_CLOUD     2   // Отправка в Supabase
#define OLED_PAGE_HARDWARE  3   // Состояние датчиков и сети
#define OLED_PAGE_SYSTEM    4   // CPU, uptime, ребут
#define OLED_PAGE_COUNT     5
#define OLED_PAGE_MS       5000UL
#define OLED_STATUS_HOLD_MS 5000UL

// Встроенный LED на ESP32 DevKit (GPIO 2, active LOW) — отдельно от светофора R/Y/G
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// =====================
// Тайминги
// =====================
const unsigned long CLIMATE_SEND_INTERVAL = 5UL  * 60UL * 1000UL;  // 5 мин — отправка в базу
const unsigned long CLIMATE_TICK_INTERVAL = 30UL * 1000UL;         // 30 сек — замер в буфер
const unsigned long PMS_READ_INTERVAL     = 30UL * 60UL * 1000UL;  // 30 мин — цикл пыли
const unsigned long PMS_WAKEUP_TIME       = 30UL * 1000UL;         // 30 сек — прогрев лазера
const unsigned long PMS_SAMPLE_TIME       = 30UL * 1000UL;         // 30 сек — серия замеров
const unsigned long PMS_SAMPLE_DELAY      = 1000UL;                // 1 сек между чтениями
const unsigned long WIFI_TIMEOUT_REBOOT   = 10UL * 60UL * 1000UL;  // 10 минут без Wi-Fi -> ребут
const unsigned long MQTT_TELEMETRY_INTERVAL = 10UL * 1000UL;

// =====================
// PMS Accumulator — серия замеров со статистикой
// =====================
#define PMS_SAMPLE_MAX 30

struct PmsStats {
  float min;
  float max;
  float median;
  float mean;
  float stddev;
  int   count;
};

struct PmsAccumulator {
  uint16_t pm1[PMS_SAMPLE_MAX];
  uint16_t pm25[PMS_SAMPLE_MAX];
  uint16_t pm10[PMS_SAMPLE_MAX];
  int count;

  void reset() { count = 0; }

  void add(uint16_t p1, uint16_t p25, uint16_t p10) {
    if (count >= PMS_SAMPLE_MAX) return;
    pm1[count]  = p1;
    pm25[count] = p25;
    pm10[count] = p10;
    count++;
  }

  bool hasData() const { return count > 0; }

  PmsStats calcStats(uint16_t* arr, int n) {
    PmsStats s;
    s.count = n;

    uint16_t sorted[PMS_SAMPLE_MAX];
    for (int i = 0; i < n; i++) sorted[i] = arr[i];
    std::sort(sorted, sorted + n);

    s.min = (float)sorted[0];
    s.max = (float)sorted[n - 1];

    if (n % 2 != 0) {
      s.median = (float)sorted[n / 2];
    } else {
      s.median = ((float)sorted[n / 2 - 1] + (float)sorted[n / 2]) / 2.0f;
    }

    float sum = 0;
    for (int i = 0; i < n; i++) sum += (float)arr[i];
    s.mean = sum / (float)n;

    float sqSum = 0;
    for (int i = 0; i < n; i++) {
      float diff = (float)arr[i] - s.mean;
      sqSum += diff * diff;
    }
    s.stddev = sqrtf(sqSum / (float)n);

    return s;
  }

  PmsStats statsPm1()  { return calcStats(pm1,  count); }
  PmsStats statsPm25() { return calcStats(pm25, count); }
  PmsStats statsPm10() { return calcStats(pm10, count); }
};

// =====================
// Кольцевой лог (хранится в RAM)
// =====================
#define LOG_ENTRY_LEN    64
#define EVENT_LOG_SIZE   15
#define ERROR_LOG_SIZE   10
#define RTC_ERROR_LOG_SIZE  8

struct RingLog {
  char     entries[20][LOG_ENTRY_LEN];
  unsigned long timestamps[20];
  int      maxSize;
  int      head;
  int      count;

  void init(int size) {
    maxSize = size;
    head = 0;
    count = 0;
  }

  void add(const char* text) {
    strncpy(entries[head], text, LOG_ENTRY_LEN - 1);
    entries[head][LOG_ENTRY_LEN - 1] = '\0';
    timestamps[head] = millis() / 1000UL;
    head = (head + 1) % maxSize;
    if (count < maxSize) count++;
  }

  String format() {
    if (count == 0) return "  _пусто_\n";
    String result = "";
    int start = (head - count + maxSize) % maxSize;
    for (int i = 0; i < count; i++) {
      int idx = (start + i) % maxSize;
      unsigned long sec = timestamps[idx];
      char timeBuf[12];
      snprintf(timeBuf, sizeof(timeBuf), "%luh%02lum", sec / 3600, (sec % 3600) / 60);
      result += "  `" + String(timeBuf) + "` " + String(entries[idx]) + "\n";
    }
    return result;
  }
};

RingLog eventLog;
RingLog errorLog;

// =====================
// Объекты и состояние
// =====================
Adafruit_BME280 bme;
TwoWire oledWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &oledWire, OLED_RESET);
PMS pms(Serial2);
PMS::DATA data;

bool bmeReady   = false;
bool pmsIsAwake = false;
int  pmsErrorCount = 0;

unsigned long lastClimateSendTime  = 0;
unsigned long lastClimateTickTime  = 0;
unsigned long pmsWakeupTargetTime  = 0;
unsigned long pmsSamplingStartTime = 0;
unsigned long pmsLastSampleTime    = 0;
unsigned long lastWifiConnectedTime = 0;
bool pmsSampling = false;

PmsAccumulator pmsAcc;

PmsStats lastStatsPm1;
PmsStats lastStatsPm25;
PmsStats lastStatsPm10;
bool     hasPmsStats = false;

const int CLIMATE_BUFFER_SIZE = 10;
float tempBuffer[CLIMATE_BUFFER_SIZE];
float humBuffer [CLIMATE_BUFFER_SIZE];
float presBuffer[CLIMATE_BUFFER_SIZE];
int   climateBufferIndex = 0;

// Детекция залипших датчиков
float lastBmeTemp = -999.0f;
int   staleBmeCount = 0;
const int STALE_BME_THRESHOLD = 10;   // 10 x 30 сек = 5 мин одинаковых показаний

float lastPmsMedian = -1.0f;
int   stalePmsCount = 0;
const int STALE_PMS_THRESHOLD = 6;    // 6 x 30 мин = 3 часа одинаковых медиан

// Критический уровень PM2.5 — мигающий красный
bool pmsCriticalBlink = false;

// Причина последнего ребута
char rebootReasonShort[17] = "";

// Программные причины ребута (сохраняются в RTC между esp_restart)
enum RebootCause : int {
  REBOOT_NONE        = 0,
  REBOOT_WIFI_TIMEOUT = 1,
  REBOOT_DAILY       = 2,
  REBOOT_BME_STALE   = 3,
  REBOOT_PMS_STALE   = 4,
  REBOOT_TELEGRAM    = 5,
  REBOOT_MQTT        = 6,
};

#define REBOOT_RTC_MAGIC  0xEB00C001UL

RTC_DATA_ATTR static uint32_t rtcRebootMagic     = 0;
RTC_DATA_ATTR static int      rtcRebootCode      = REBOOT_NONE;
RTC_DATA_ATTR static uint32_t rtcRebootUptime    = 0;
RTC_DATA_ATTR static uint32_t rtcRebootHeap      = 0;
RTC_DATA_ATTR static int      rtcRebootStalePms  = 0;
RTC_DATA_ATTR static int      rtcRebootStaleBme  = 0;
RTC_DATA_ATTR static uint32_t rtcRebootMinHeap   = 0;
RTC_DATA_ATTR static uint32_t rtcLastLoopMs      = 0;

// Лог ошибок в RTC — переживает esp_restart и watchdog
RTC_DATA_ATTR static int      rtcErrorHead  = 0;
RTC_DATA_ATTR static int      rtcErrorCount = 0;
RTC_DATA_ATTR static char     rtcErrorEntries[RTC_ERROR_LOG_SIZE][LOG_ENTRY_LEN];
RTC_DATA_ATTR static uint32_t rtcErrorUptime[RTC_ERROR_LOG_SIZE];

// Минимальный heap за сессию (обновляется в loop)
uint32_t sessionMinHeap = UINT32_MAX;

// Ошибки Supabase
int  supabaseErrorCount = 0;
int  supabaseTotalErrors = 0;
const int SUPABASE_MAX_ERRORS = 3;

// NTP — время загрузки и метки последних апдейтов
char bootTimeStr[20]           = "---";
char lastClimateTimeStr[20]    = "---";
char lastPmsTimeStr[20]        = "---";
char lastCloudSendTimeStr[20]  = "---";
bool ntpSynced = false;

float lastClimateTemp = 0.0f;
float lastClimateHum  = 0.0f;
float lastClimatePres = 0.0f;
bool  hasClimateReading = false;

// Telegram-команды (удалённый ребут/статус)
unsigned long lastTelegramCheckTime = 0;
const unsigned long TELEGRAM_CHECK_INTERVAL = 60UL * 1000UL;
long telegramUpdateId = 0;
bool telegramOffsetInit = false;

WiFiClient mqttNet;
PubSubClient mqttClient(mqttNet);

char topicStatus[64];
char topicTelemetry[64];
char topicCommand[64];
bool mqttTopicsReady = false;
unsigned long lastMqttTelemetry = 0;

uint8_t oledPage = 0;
unsigned long lastOledUpdate = 0;
bool oledStatusHold = false;
unsigned long oledStatusHoldStart = 0;
bool boardLedOn = false;

// Forward declarations
void    connectToWiFi();
void    initMqttTopics();
void    handleMqttCommand(char* topic, byte* payload, unsigned int length);
void    setBoardLed(bool on);
void    showStatusScreen();
void    ensureMqtt();
void    publishMqttTelemetry();
void    sendDataToSupabase(float t, float h, float p);
void    sendTelegramMessage(String message);
bool    readClimate(float &t, float &h, float &p);
bool    isClimateValid(float t, float h, float p);
float   getMedian(float* array, int size);
String  pmsStatsJson(const char* prefix, PmsStats &s);
void    setLedColor(bool r, bool g, bool y);
void    updateTrafficLight();
void    updateOled(uint8_t page);
void    checkTelegramCommands();
void    updateTimeStr(char* buf, size_t len);
void    formatClockShort(char* buf, size_t len);
void    formatNextWallTime(unsigned long targetMs, unsigned long nowMs, char* buf, size_t len);
void    formatCountdown(unsigned long targetMs, unsigned long nowMs, char* buf, size_t len);
unsigned long nextPmsCompleteTime();
void    oledDrawHeader(uint8_t page, const char* title);
void    oledDrawLine(int y);
void    requestReboot(RebootCause cause);
String  resetReasonLabel(esp_reset_reason_t reason);
String  rebootCauseLabel(RebootCause cause);
String  formatUptime(unsigned long ms);
String  buildRebootDebugBlock(esp_reset_reason_t hwReason);
void    logError(const char* text);
void    rtcErrorPersist(const char* text, uint32_t uptimeMs);
String  formatRtcErrorLog();
void    setupHttpClient(HTTPClient &http);

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- Уличная метеостанция (Климат 5м / Пыль 30м) ---");

  eventLog.init(EVENT_LOG_SIZE);
  errorLog.init(ERROR_LOG_SIZE);

  esp_reset_reason_t rstReason = esp_reset_reason();
  if (rstReason != ESP_RST_SW) rtcRebootMagic = 0;
  sessionMinHeap = ESP.getFreeHeap();

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  setBoardLed(false);

  // Инициализация OLED SSD1306 128x64 (отдельная I2C шина на пинах 18/19)
  oledWire.begin(OLED_SDA, OLED_SCL);
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("System Booting..."));
    display.display();
    Serial.println("[OK] OLED SSD1306 инициализирован.");
  } else {
    Serial.println("[Ошибка] OLED SSD1306 не найден!");
  }
  setLedColor(false, false, true); // Желтый на старте (прогрев)

  memset(&lastStatsPm1,  0, sizeof(PmsStats));
  memset(&lastStatsPm25, 0, sizeof(PmsStats));
  memset(&lastStatsPm10, 0, sizeof(PmsStats));

  // --- Причина ребута (сохраняем для LCD) ---
  switch (rstReason) {
    case ESP_RST_UNKNOWN:    strncpy(rebootReasonShort, "UNK HW",      16); break;
    case ESP_RST_POWERON:    strncpy(rebootReasonShort, "POWER ON",    16); break;
    case ESP_RST_EXT:        strncpy(rebootReasonShort, "EXT PIN",     16); break;
    case ESP_RST_SW:         strncpy(rebootReasonShort, "SOFTWARE",    16); break;
    case ESP_RST_PANIC:      strncpy(rebootReasonShort, "PANIC!",      16); break;
    case ESP_RST_INT_WDT:    strncpy(rebootReasonShort, "INT WDT!",    16); break;
    case ESP_RST_TASK_WDT:   strncpy(rebootReasonShort, "TASK WDT!",   16); break;
    case ESP_RST_WDT:        strncpy(rebootReasonShort, "WDT!",        16); break;
    case ESP_RST_DEEPSLEEP:  strncpy(rebootReasonShort, "DEEP SLEEP",  16); break;
    case ESP_RST_BROWNOUT:   strncpy(rebootReasonShort, "BROWNOUT!",   16); break;
    case ESP_RST_SDIO:       strncpy(rebootReasonShort, "SDIO",        16); break;
    default:                 strncpy(rebootReasonShort, "OTHER",       16); break;
  }

  Serial.printf("[Reboot] HW reason: %s (code %d)\n",
                resetReasonLabel(rstReason).c_str(), (int)rstReason);
  if (rstReason == ESP_RST_SW && rtcRebootMagic == REBOOT_RTC_MAGIC) {
    Serial.printf("[Reboot] SW reason: %s, prev uptime: %s, heap: %lu, minHeap: %lu\n",
                  rebootCauseLabel((RebootCause)rtcRebootCode).c_str(),
                  formatUptime(rtcRebootUptime).c_str(),
                  (unsigned long)rtcRebootHeap,
                  (unsigned long)rtcRebootMinHeap);
    Serial.printf("[Reboot] stale PMS: %d, stale BME: %d\n",
                  rtcRebootStalePms, rtcRebootStaleBme);
  } else if (rstReason != ESP_RST_POWERON && rstReason != ESP_RST_SW) {
    char buf[LOG_ENTRY_LEN];
    snprintf(buf, sizeof(buf), "HW RESET: %s (#%d) @%s",
             resetReasonLabel(rstReason).c_str(), (int)rstReason,
             formatUptime(rtcLastLoopMs).c_str());
    logError(buf);
  }

  // --- BME280 ---
  bmeReady = bme.begin(BME280_ADDR, &Wire);
  Serial.println(bmeReady ? "[ОК] BME280 инициализирован." : "[Ошибка] BME280 не найден!");
  eventLog.add(bmeReady ? "BME280 OK" : "BME280 FAIL");
  if (!bmeReady) logError("BME280 не найден при старте");

  // --- PMS5003 ---
  Serial2.begin(9600, SERIAL_8N1, PMS_RX, PMS_TX);
  pms.passiveMode();
  pms.sleep();
  pmsIsAwake = false;
  Serial.println("[ОК] PMS5003 спит. Цикл: 30 мин.");

  // --- Wi-Fi ---
  connectToWiFi();
  int waitAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && waitAttempts < 10) {
    delay(1000);
    Serial.print("#");
    waitAttempts++;
  }

  // --- Честный стартовый прогрев PMS (30 сек) + серия замеров ---
  if (WiFi.status() == WL_CONNECTED) {
    eventLog.add("WiFi OK");
    lastWifiConnectedTime = millis();
    ensureMqtt();

    // NTP-синхронизация
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, NTP_SERVER);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 5000)) {
      ntpSynced = true;
      updateTimeStr(bootTimeStr, sizeof(bootTimeStr));
      Serial.printf("[NTP] Синхронизировано: %s\n", bootTimeStr);
      eventLog.add("NTP OK");
    } else {
      Serial.println("[NTP] Не удалось синхронизировать время");
      logError("NTP sync FAIL");
    }

    Serial.println("[Система] Старт полноценного прогрева PMS5003 для честного стартового замера...");
    pms.wakeUp();

    for (int i = 0; i < 30; i++) {
      delay(1000);
      if (i % 5 == 0) Serial.printf("[Прогрев] Осталось %d сек...\n", 30 - i);
    }

    // Серия замеров при старте
    Serial.println("[PMS5003] Сбор честной стартовой серии...");
    pmsAcc.reset();
    unsigned long sStart = millis();
    while (millis() - sStart < PMS_SAMPLE_TIME && pmsAcc.count < PMS_SAMPLE_MAX) {
      pms.requestRead();
      if (pms.readUntil(data, PMS_TIMEOUT_MS)) {
        pmsAcc.add(data.PM_AE_UG_1_0, data.PM_AE_UG_2_5, data.PM_AE_UG_10_0);
        Serial.printf("[PMS5003] #%d -> PM2.5: %d\n", pmsAcc.count, data.PM_AE_UG_2_5);
      }
      delay(PMS_SAMPLE_DELAY);
    }

    if (pmsAcc.hasData()) {
      lastStatsPm1  = pmsAcc.statsPm1();
      lastStatsPm25 = pmsAcc.statsPm25();
      lastStatsPm10 = pmsAcc.statsPm10();
      hasPmsStats   = true;
      pmsErrorCount = 0;
      if (ntpSynced) updateTimeStr(lastPmsTimeStr, sizeof(lastPmsTimeStr));
      Serial.printf("[PMS5003] Стартовая серия: n=%d, PM2.5 med=%.0f σ=%.1f\n",
                    lastStatsPm25.count, lastStatsPm25.median, lastStatsPm25.stddev);
      char buf[LOG_ENTRY_LEN];
      snprintf(buf, sizeof(buf), "PMS старт: n=%d PM2.5=%.0f", lastStatsPm25.count, lastStatsPm25.median);
      eventLog.add(buf);
      updateTrafficLight();
    } else {
      pmsErrorCount++;
      Serial.println("[Предупреждение] PMS5003: 0 чтений при старте.");
      logError("PMS5003: 0 чтений при старте");
      setLedColor(true, false, false);
    }
    pms.sleep();

    // Сброс RTC-метки — после формирования сообщения
    String rebootReasonStr = resetReasonLabel(rstReason);
    if (rstReason == ESP_RST_SW && rtcRebootMagic == REBOOT_RTC_MAGIC) {
      rebootReasonStr = "🔄 " + rebootCauseLabel((RebootCause)rtcRebootCode);
    } else if (rstReason == ESP_RST_WDT || rstReason == ESP_RST_TASK_WDT || rstReason == ESP_RST_INT_WDT) {
      rebootReasonStr = "🚨 WATCHDOG (" + resetReasonLabel(rstReason) + ")";
    } else if (rstReason == ESP_RST_PANIC) {
      rebootReasonStr = "💥 PANIC";
    } else if (rstReason == ESP_RST_BROWNOUT) {
      rebootReasonStr = "⚡ BROWNOUT";
    } else if (rstReason == ESP_RST_POWERON) {
      rebootReasonStr = "🔌 POWER ON";
    }

    String debugBlock = buildRebootDebugBlock(rstReason);
    rtcRebootMagic = 0;

    // Telegram Рапорт
    float sT = 0, sH = 0, sP = 0;
    bool cOk = readClimate(sT, sH, sP);

    String msg = "🤖 *МЕТЕОСТАНЦИЯ БАЛКОН — ЗАПУСК*\n";
    msg += "━━━━━━━━━━━━━━━━━━━━━\n\n";
    msg += "⚡ *Причина:* `" + rebootReasonStr + "`\n";
    msg += debugBlock;
    msg += "⏱ Uptime: `" + String(millis()) + " ms`\n";
    msg += "🌐 IP: `" + WiFi.localIP().toString() + "`\n";
    msg += "📶 RSSI: `" + String(WiFi.RSSI()) + " dBm`\n\n";

    msg += "📡 *Железо*\n";
    msg += (bmeReady ? "  ✅ BME280 — `OK`\n" : "  ❌ BME280 — `ОШИБКА`\n");
    msg += "  ✅ PMS5003 — `SLEEP` (цикл 30м)\n\n";

    msg += "📝 *Стартовые замеры*\n";
    if (cOk) {
      msg += "  🌡 `" + String(sT, 1) + " °C`\n";
      msg += "  💧 `" + String(sH, 0) + " %`\n";
      msg += "  📉 `" + String(sP, 1) + " mmHg`\n";
    } else {
      msg += "  ⚠️ BME280: нет данных\n";
    }
    if (hasPmsStats) {
      msg += "  💨 PM2.5: `" + String((int)lastStatsPm25.median) + " мкг/м³`";
      msg += "  (n=" + String(lastStatsPm25.count);
      msg += ", σ=" + String(lastStatsPm25.stddev, 1);
      msg += ", " + String((int)lastStatsPm25.min) + "–" + String((int)lastStatsPm25.max) + ")\n";
    }
    if (rtcErrorCount > 0) {
      msg += "\n🚨 *Ошибки прошлой сессии (RTC):*\n";
      msg += formatRtcErrorLog();
    }
    msg += "\n━━━━━━━━━━━━━━━━━━━━━";
    sendTelegramMessage(msg);
  }

  // Выравнивание таймеров
  unsigned long now         = millis();
  lastClimateTickTime       = now - CLIMATE_TICK_INTERVAL;
  lastClimateSendTime       = now;
  pmsWakeupTargetTime       = now + PMS_READ_INTERVAL - PMS_WAKEUP_TIME - PMS_SAMPLE_TIME;

  // Watchdog включаем только после долгого setup (PMS прогрев ~60 сек)
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms     = WDT_TIMEOUT_SEC * 1000UL,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wdtConfig);
  esp_task_wdt_add(NULL);

  Serial.println("[Система] Вход в рабочий цикл.");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // СБРОС СТОРОЖЕВОГО ТАЙМЕРА (Кормим собаку)
  esp_task_wdt_reset();

  unsigned long now = millis();
  rtcLastLoopMs = now;

  uint32_t heap = ESP.getFreeHeap();
  if (heap < sessionMinHeap) sessionMinHeap = heap;

  // Контроль отвала Wi-Fi
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
    // Если Wi-Fi лежит слишком долго — принудительный ребут
    if (now - lastWifiConnectedTime > WIFI_TIMEOUT_REBOOT) {
      Serial.println("[Критическая ошибка] Сеть недоступна более 10 минут. Ребут!");
      logError("WiFi таймаут 10м -> ребут");
      delay(500);
      requestReboot(REBOOT_WIFI_TIMEOUT);
    }
  } else {
    lastWifiConnectedTime = now; // Сбрасываем таймер ошибки сети
    ensureMqtt();
    mqttClient.loop();

    if (now - lastMqttTelemetry >= MQTT_TELEMETRY_INTERVAL) {
      lastMqttTelemetry = now;
      publishMqttTelemetry();
    }
  }

  // Профилактический суточный ребут (защита от утечек памяти)
  if (now > 24UL * 60UL * 60UL * 1000UL) {
    Serial.println("[Профилактика] Суточный перезапуск системы...");
    String msg = "🔄 *БАЛКОН — Суточный ребут*\n";
    msg += "━━━━━━━━━━━━━━━━━━━━━\n\n";
    msg += "🧠 Heap: `" + String(ESP.getFreeHeap()) + " bytes`\n";
    msg += "📶 RSSI: `" + String(WiFi.RSSI()) + " dBm`\n";
    msg += "🗄 Ошибок Supabase: `" + String(supabaseTotalErrors) + "`\n\n";
    msg += "━━━━━━━━━━━━━━━━━━━━━";
    sendTelegramMessage(msg);
    delay(1000);
    requestReboot(REBOOT_DAILY);
  }

  // ── 1. Климат в буфер каждые 30 сек ──
  if (now - lastClimateTickTime >= CLIMATE_TICK_INTERVAL) {
    lastClimateTickTime = now;
    if (climateBufferIndex < CLIMATE_BUFFER_SIZE) {
      float t, h, p;
      if (readClimate(t, h, p)) {
        tempBuffer[climateBufferIndex] = t;
        humBuffer [climateBufferIndex] = h;
        presBuffer[climateBufferIndex] = p;
        Serial.printf("[Буфер климата] %d/%d -> T:%.1f P:%.1f\n",
                      climateBufferIndex + 1, CLIMATE_BUFFER_SIZE, t, p);
        climateBufferIndex++;
        lastClimateTemp = t;
        lastClimateHum  = h;
        lastClimatePres = p;
        hasClimateReading = true;
        if (ntpSynced) updateTimeStr(lastClimateTimeStr, sizeof(lastClimateTimeStr));

        // Детекция залипшего BME280
        if (t == lastBmeTemp) {
          staleBmeCount++;
          if (staleBmeCount >= STALE_BME_THRESHOLD) {
            Serial.printf("[STALE] BME280 завис: %.2f°C x%d раз\n", t, staleBmeCount);
            char buf[LOG_ENTRY_LEN];
            snprintf(buf, sizeof(buf), "BME280 STALE %.2f°C x%d -> ребут", t, staleBmeCount);
            logError(buf);
            String msg = "🚨 *БАЛКОН — BME280 ЗАВИС*\n";
            msg += "━━━━━━━━━━━━━━━━━━━━━\n\n";
            msg += "🌡 Температура: `" + String(t, 2) + " °C`\n";
            msg += "🔁 Повторов подряд: `" + String(staleBmeCount) + "`\n\n";
            msg += "⚡ Перезагрузка...\n\n";
            msg += "━━━━━━━━━━━━━━━━━━━━━";
            sendTelegramMessage(msg);
            delay(1000);
            requestReboot(REBOOT_BME_STALE);
          }
        } else {
          staleBmeCount = 0;
          lastBmeTemp = t;
        }
      }
    }
  }

  // ── 2. Будим PMS за (прогрев + сбор) до дедлайна ──
  if (!pmsIsAwake && !pmsSampling && (long)(now - pmsWakeupTargetTime) >= 0) {
    pms.wakeUp();
    pmsIsAwake = true;
    Serial.println("[PMS5003] Пробуждение лазера. Прогрев 30 сек...");
  }

  // ── 3. После прогрева — старт серии ──
  if (pmsIsAwake && !pmsSampling && (now - pmsWakeupTargetTime >= PMS_WAKEUP_TIME)) {
    pmsSampling = true;
    pmsSamplingStartTime = now;
    pmsLastSampleTime = 0;
    pmsAcc.reset();
    Serial.println("[PMS5003] Старт серии замеров (30 сек)...");
  }

  // ── 4. Во время серии — неблокирующий сбор ──
  if (pmsSampling) {
    if (now - pmsSamplingStartTime < PMS_SAMPLE_TIME && pmsAcc.count < PMS_SAMPLE_MAX) {
      if (now - pmsLastSampleTime >= PMS_SAMPLE_DELAY) {
        pmsLastSampleTime = now;
        pms.requestRead();
        if (pms.readUntil(data, PMS_TIMEOUT_MS)) {
          pmsAcc.add(data.PM_AE_UG_1_0, data.PM_AE_UG_2_5, data.PM_AE_UG_10_0);
          if (pmsAcc.count % 5 == 0 || pmsAcc.count == 1) {
            Serial.printf("[PMS5003] #%d PM2.5:%d PM10:%d\n",
                          pmsAcc.count, data.PM_AE_UG_2_5, data.PM_AE_UG_10_0);
          }
        }
      }
    } else {
      Serial.printf("[PMS5003] Серия завершена: %d замеров.\n", pmsAcc.count);

      if (pmsAcc.hasData()) {
        lastStatsPm1  = pmsAcc.statsPm1();
        lastStatsPm25 = pmsAcc.statsPm25();
        lastStatsPm10 = pmsAcc.statsPm10();
        hasPmsStats   = true;
        pmsErrorCount = 0;
        if (ntpSynced) updateTimeStr(lastPmsTimeStr, sizeof(lastPmsTimeStr));

        // Детекция залипшего PMS5003
        if (lastStatsPm25.median == lastPmsMedian && lastPmsMedian >= 0) {
          stalePmsCount++;
          if (stalePmsCount >= STALE_PMS_THRESHOLD) {
            Serial.printf("[STALE] PMS5003 завис: PM2.5=%.0f x%d циклов\n",
                          lastPmsMedian, stalePmsCount);
            char buf[LOG_ENTRY_LEN];
            snprintf(buf, sizeof(buf), "PMS STALE PM2.5=%.0f x%d -> ребут", lastPmsMedian, stalePmsCount);
            logError(buf);
            String msg = "🚨 *БАЛКОН — PMS5003 ЗАВИС*\n";
            msg += "━━━━━━━━━━━━━━━━━━━━━\n\n";
            msg += "💨 PM2.5: `" + String((int)lastPmsMedian) + " мкг/м³`\n";
            msg += "🔁 Циклов подряд: `" + String(stalePmsCount) + "`\n\n";
            msg += "⚡ Перезагрузка...\n\n";
            msg += "━━━━━━━━━━━━━━━━━━━━━";
            sendTelegramMessage(msg);
            delay(1000);
            requestReboot(REBOOT_PMS_STALE);
          }
        } else {
          stalePmsCount = 0;
        }
        lastPmsMedian = lastStatsPm25.median;

        Serial.printf("[PMS5003] PM2.5: med=%.0f avg=%.1f σ=%.1f min=%.0f max=%.0f n=%d\n",
                      lastStatsPm25.median, lastStatsPm25.mean, lastStatsPm25.stddev,
                      lastStatsPm25.min, lastStatsPm25.max, lastStatsPm25.count);
        {
          char buf[LOG_ENTRY_LEN];
          snprintf(buf, sizeof(buf), "PMS OK n=%d PM2.5=%.0f", lastStatsPm25.count, lastStatsPm25.median);
          eventLog.add(buf);
        }

        updateTrafficLight();
      } else {
        pmsErrorCount++;
        Serial.printf("[PMS5003] 0 чтений! Ошибок подряд: %d\n", pmsErrorCount);
        char buf[LOG_ENTRY_LEN];
        snprintf(buf, sizeof(buf), "PMS 0 чтений (подряд: %d)", pmsErrorCount);
        logError(buf);
        setLedColor(true, false, false);
        if (pmsErrorCount >= PMS_MAX_ERRORS) {
          String alert = "🚨 *БАЛКОН — PMS5003 НЕ ОТВЕЧАЕТ*\n";
          alert += "━━━━━━━━━━━━━━━━━━━━━\n\n";
          alert += "🔁 Ошибок подряд: `" + String(pmsErrorCount) + "`\n";
          alert += "🔧 Проверьте питание и UART\n\n";
          alert += "━━━━━━━━━━━━━━━━━━━━━";
          sendTelegramMessage(alert);
          pmsErrorCount = 0;
        }
      }

      pms.sleep();
      pmsIsAwake  = false;
      pmsSampling = false;
      pmsWakeupTargetTime = now + PMS_READ_INTERVAL - PMS_WAKEUP_TIME - PMS_SAMPLE_TIME;
    }
  }

  // ── 5. Мигающие светодиоды ──
  if (pmsCriticalBlink) {
    bool blinkOn = (now / 300) % 2 == 0;
    setLedColor(blinkOn, false, false);       // Мигающий красный — опасный уровень
  } else if (pmsIsAwake || pmsSampling) {
    bool blinkOn = (now / 500) % 2 == 0;
    setLedColor(false, false, blinkOn);       // Мигающий жёлтый — прогрев/замеры
  }

  // ── 6. Обновление OLED (переключение страниц) ──
  if (oledStatusHold) {
    if (now - oledStatusHoldStart >= OLED_STATUS_HOLD_MS) {
      oledStatusHold = false;
      lastOledUpdate = now;
      updateOled(oledPage);
    }
  } else if (now - lastOledUpdate >= OLED_PAGE_MS) {
    lastOledUpdate = now;
    if (pmsErrorCount == 0 && bmeReady) {
      oledPage = (oledPage + 1) % OLED_PAGE_COUNT;
    }
    updateOled(oledPage);
  }

  // ── 7. Отправка в Supabase каждые 5 мин ──
  if (now - lastClimateSendTime >= CLIMATE_SEND_INTERVAL) {
    Serial.println("\n--- Отправка (5 мин) ---");

    float fT, fH, fP;
    if (climateBufferIndex > 0) {
      fT = getMedian(tempBuffer, climateBufferIndex);
      fH = getMedian(humBuffer,  climateBufferIndex);
      fP = getMedian(presBuffer, climateBufferIndex);
      climateBufferIndex = 0;
    } else {
      if (!readClimate(fT, fH, fP)) {
        Serial.println("[Supabase] Нет климата — пропуск.");
        lastClimateSendTime = millis();
        climateBufferIndex = 0;
        return;
      }
    }

    sendDataToSupabase(fT, fH, fP);
    lastClimateSendTime = millis();
  }

  // ── 8. Telegram-команды (удалённый ребут/статус) ──
  if (now - lastTelegramCheckTime >= TELEGRAM_CHECK_INTERVAL) {
    lastTelegramCheckTime = now;
    checkTelegramCommands();
  }

  delay(200);
}

// ============================================================
// Управление светодиодами (Исправлено под R, Y, G)
// ============================================================
void setLedColor(bool r, bool g, bool y) {
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_Y, y ? HIGH : LOW);
}

void updateTrafficLight() {
  if (!hasPmsStats) {
    setLedColor(false, false, true); // Желтый — идет прогрев / нет данных
    return;
  }

  float pm25 = lastStatsPm25.median;

  if (pm25 > 150.0f) {
    pmsCriticalBlink = true;          // Мигающий красный (Опасно)
  } else {
    pmsCriticalBlink = false;
    if (pm25 <= 12.0f) {
      setLedColor(false, true, false);  // Зеленый (Отлично)
    } else if (pm25 <= 35.0f) {
      setLedColor(false, false, true);  // Желтый — LED_Y
    } else {
      setLedColor(true, false, false);  // Красный (Плохо / Вредно)
    }
  }
}

// ============================================================
// BME280 helpers
// ============================================================
bool readClimate(float &t, float &h, float &p) {
  if (!bmeReady) return false;
  t = bme.readTemperature();
  h = bme.readHumidity();
  p = (bme.readPressure() / 100.0f) * PA_TO_MMHG;
  return isClimateValid(t, h, p);
}

bool isClimateValid(float t, float h, float p) {
  if (isnan(t) || isnan(h) || isnan(p))  return false;
  if (t < TEMP_MIN || t > TEMP_MAX)       return false;
  if (h < HUM_MIN  || h > HUM_MAX)        return false;
  if (p < PRES_MIN || p > PRES_MAX)       return false;
  return true;
}

// ============================================================
// Медиана (климатический буфер)
// ============================================================
float getMedian(float* array, int size) {
  float sorted[CLIMATE_BUFFER_SIZE];
  for (int i = 0; i < size; i++) sorted[i] = array[i];
  std::sort(sorted, sorted + size);
  if (size % 2 != 0) return sorted[size / 2];
  return (sorted[size / 2 - 1] + sorted[size / 2]) / 2.0f;
}

// ============================================================
// JSON-фрагмент статистики PMS
// ============================================================
String pmsStatsJson(const char* prefix, PmsStats &s) {
  String r = "";
  r += "\"" + String(prefix) + "\":" + String((int)s.median) + ",";
  r += "\"" + String(prefix) + "_min\":" + String((int)s.min) + ",";
  r += "\"" + String(prefix) + "_max\":" + String((int)s.max) + ",";
  r += "\"" + String(prefix) + "_median\":" + String((int)s.median) + ",";
  r += "\"" + String(prefix) + "_mean\":" + String(s.mean, 2) + ",";
  r += "\"" + String(prefix) + "_stddev\":" + String(s.stddev, 2) + ",";
  r += "\"" + String(prefix) + "_count\":" + String(s.count);
  return r;
}

// ============================================================
// Supabase POST
// ============================================================
void sendDataToSupabase(float t, float h, float p) {
  if (WiFi.status() != WL_CONNECTED) return;

  esp_task_wdt_reset();
  HTTPClient http;
  http.begin(SUPABASE_URL);
  setupHttpClient(http);
  http.addHeader("apikey",        SUPABASE_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);
  http.addHeader("Content-Type",  "application/json");
  http.addHeader("Prefer",        "return=minimal");

  String json = "{";
  json += "\"temperature\":"  + String(t, 2) + ",";
  json += "\"humidity\":"     + String(h, 2) + ",";
  json += "\"pressure\":"     + String(p, 2) + ",";

  if (hasPmsStats) {
    json += pmsStatsJson("pm1_0",  lastStatsPm1)  + ",";
    json += pmsStatsJson("pm2_5",  lastStatsPm25) + ",";
    json += pmsStatsJson("pm10_0", lastStatsPm10);
  } else {
    PmsStats nullStats = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0};
    json += pmsStatsJson("pm1_0",  nullStats)  + ",";
    json += pmsStatsJson("pm2_5",  nullStats) + ",";
    json += pmsStatsJson("pm10_0", nullStats);
  }
  json += "}";

  Serial.printf("[Supabase] T:%.1f H:%.0f P:%.1f PM2.5(med):%.0f (n:%d)\n",
                t, h, p,
                hasPmsStats ? lastStatsPm25.median : 0.0f,
                hasPmsStats ? lastStatsPm25.count  : 0);
  int code = http.POST(json);
  esp_task_wdt_reset();
  http.end();

  if (code >= 200 && code < 300) {
    Serial.printf("[Supabase] OK %d\n", code);
    if (supabaseErrorCount > 0) {
      Serial.printf("[Supabase] Восстановлено после %d ошибок подряд\n", supabaseErrorCount);
      char buf[LOG_ENTRY_LEN];
      snprintf(buf, sizeof(buf), "Supabase восст. после %d ошибок", supabaseErrorCount);
      eventLog.add(buf);
    }
    supabaseErrorCount = 0;
    eventLog.add("Supabase OK");
    if (ntpSynced) updateTimeStr(lastCloudSendTimeStr, sizeof(lastCloudSendTimeStr));
  } else {
    supabaseErrorCount++;
    supabaseTotalErrors++;
    Serial.printf("[Supabase] ОШИБКА %d (подряд: %d, всего: %d)\n",
                  code, supabaseErrorCount, supabaseTotalErrors);
    Serial.printf("[Supabase] Потерянные данные: T:%.2f H:%.2f P:%.2f PM2.5:%.0f\n",
                  t, h, p, hasPmsStats ? lastStatsPm25.median : 0.0f);
    char buf[LOG_ENTRY_LEN];
    snprintf(buf, sizeof(buf), "Supabase FAIL HTTP %d T:%.1f", code, t);
    logError(buf);

    if (supabaseErrorCount >= SUPABASE_MAX_ERRORS) {
      String alert = "🚨 *БАЛКОН — SUPABASE НЕ ОТВЕЧАЕТ*\n";
      alert += "━━━━━━━━━━━━━━━━━━━━━\n\n";
      alert += "🔁 Ошибок подряд: `" + String(supabaseErrorCount) + "`\n";
      alert += "🗄 Всего за сессию: `" + String(supabaseTotalErrors) + "`\n";
      alert += "📡 HTTP код: `" + String(code) + "`\n\n";
      alert += "📝 *Потерянные данные:*\n";
      alert += "  🌡 `" + String(t, 1) + " °C`\n";
      alert += "  💧 `" + String(h, 0) + " %`\n";
      alert += "  📉 `" + String(p, 1) + " mmHg`\n\n";
      alert += "━━━━━━━━━━━━━━━━━━━━━";
      sendTelegramMessage(alert);
      supabaseErrorCount = 0;
    }
  }
}

// ============================================================
// Wi-Fi
// ============================================================
void connectToWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("Wi-Fi... ");
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int att = 0;
  // Неблокирующее ожидание внутри функции, чтобы WDT не сработал при долгом подключении
  while (WiFi.status() != WL_CONNECTED && att < 20) {
    esp_task_wdt_reset();
    delay(500);
    Serial.print(".");
    att++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf(" OK! %s\n", WiFi.localIP().toString().c_str());
    eventLog.add("WiFi reconnect OK");
  } else {
    Serial.println(" FAIL");
    logError("WiFi reconnect FAIL");
  }
}

// ============================================================
// MQTT (шлюз esp32.kuzyak.in)
// ============================================================
void initMqttTopics() {
  if (mqttTopicsReady) return;

  snprintf(topicStatus, sizeof(topicStatus), "devices/%s/status", DEVICE_HOSTNAME);
  snprintf(topicTelemetry, sizeof(topicTelemetry), "devices/%s/telemetry", DEVICE_HOSTNAME);
  snprintf(topicCommand, sizeof(topicCommand), "devices/%s/command", DEVICE_HOSTNAME);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(handleMqttCommand);
  mqttClient.setBufferSize(512);
  mqttTopicsReady = true;
}

void setBoardLed(bool on) {
  boardLedOn = on;
  digitalWrite(LED_BUILTIN, on ? LOW : HIGH);
}

void showStatusScreen() {
  oledStatusHold = true;
  oledStatusHoldStart = millis();
  updateOled(OLED_PAGE_HARDWARE);
}

void handleMqttCommand(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) return;

  const char* action = doc["action"];
  if (!action) return;

  if (strcmp(action, "led") == 0) {
    if (!doc["value"].is<bool>()) return;
    bool on = doc["value"];
    Serial.printf("[MQTT] led %s\n", on ? "on" : "off");
    setBoardLed(on);
    return;
  }

  if (strcmp(action, "reboot") == 0) {
    Serial.println("[MQTT] reboot");
    delay(1000);
    requestReboot(REBOOT_MQTT);
    return;
  }

  if (strcmp(action, "status") == 0) {
    Serial.println("[MQTT] status screen");
    showStatusScreen();
  }
}

void ensureMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;

  initMqttTopics();

  if (mqttClient.connected()) return;

  esp_task_wdt_reset();
  if (mqttClient.connect(DEVICE_HOSTNAME, MQTT_USER, MQTT_PASS,
                         topicStatus, 1, true, "{\"status\":\"offline\"}")) {
    mqttClient.publish(topicStatus, "{\"status\":\"online\"}", true);
    mqttClient.subscribe(topicCommand, 1);
    Serial.println("[MQTT] connected");
    eventLog.add("MQTT OK");
  } else {
    Serial.printf("[MQTT] connect failed, rc=%d\n", mqttClient.state());
  }
}

void publishMqttTelemetry() {
  if (!mqttClient.connected()) return;

  esp_task_wdt_reset();

  StaticJsonDocument<512> doc;
  doc["uptime"] = millis() / 1000UL;
  doc["rssi"] = WiFi.RSSI();
  doc["heap"] = ESP.getFreeHeap();
  doc["min_heap"] = sessionMinHeap;
  doc["bme_ready"] = bmeReady;
  doc["led"] = boardLedOn;
  doc["oled_page"] = oledPage + 1;
  doc["overlay"] = oledStatusHold ? "status" : "";
  doc["pms_awake"] = pmsIsAwake;
  doc["pms_sampling"] = pmsSampling;
  doc["supabase_errors"] = supabaseTotalErrors;
  doc["stale_bme"] = staleBmeCount;
  doc["stale_pms"] = stalePmsCount;

  if (hasClimateReading) {
    doc["temperature"] = lastClimateTemp;
    doc["humidity"] = lastClimateHum;
    doc["pressure"] = lastClimatePres;
  } else {
    float t, h, p;
    if (readClimate(t, h, p)) {
      doc["temperature"] = t;
      doc["humidity"] = h;
      doc["pressure"] = p;
    }
  }

  if (hasPmsStats) {
    doc["pm1_median"] = (int)lastStatsPm1.median;
    doc["pm25_median"] = (int)lastStatsPm25.median;
    doc["pm10_median"] = (int)lastStatsPm10.median;
  }

  char buf[512];
  size_t n = serializeJson(doc, buf);
  mqttClient.publish(topicTelemetry, buf, n);
}

// ============================================================
// Telegram
// ============================================================
void sendTelegramMessage(String message) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Telegram] Нет Wi-Fi.");
    return;
  }
  esp_task_wdt_reset();
  message.replace("\\", "\\\\");
  message.replace("\"", "\\\"");

  HTTPClient http;
  String url = "https://api.telegram.org/bot" + String(TELEGRAM_TOKEN) + "/sendMessage";
  http.begin(url);
  setupHttpClient(http);
  http.addHeader("Content-Type", "application/json");

  String payload = "{\"chat_id\":\"" + String(TELEGRAM_CHAT_ID) +
                   "\",\"text\":\""  + message +
                   "\",\"parse_mode\":\"Markdown\"}";

  int code = http.POST(payload);
  esp_task_wdt_reset();
  if (code > 0) Serial.printf("[Telegram] %d\n", code);
  else {
    Serial.printf("[Telegram] ERR: %s\n", http.errorToString(code).c_str());
    logError("Telegram HTTP timeout/fail");
  }
  http.end();
}

// ============================================================
// Telegram — приём команд (/reboot, /status)
// ============================================================
void checkTelegramCommands() {
  if (WiFi.status() != WL_CONNECTED) return;

  esp_task_wdt_reset();
  HTTPClient http;

  // Первый вызов после загрузки: пропускаем старые сообщения,
  // чтобы старая команда /reboot не вызвала цикл перезагрузок
  if (!telegramOffsetInit) {
    String url = "https://api.telegram.org/bot" + String(TELEGRAM_CMD_TOKEN)
               + "/getUpdates?offset=-1&limit=1";
    http.begin(url);
    setupHttpClient(http);
    int code = http.GET();
    esp_task_wdt_reset();
    if (code == 200) {
      String payload = http.getString();
      int idx = payload.indexOf("\"update_id\":");
      if (idx >= 0) {
        telegramUpdateId = payload.substring(idx + 12).toInt() + 1;
      }
    }
    http.end();
    telegramOffsetInit = true;
    Serial.printf("[Telegram] Offset инициализирован: %ld\n", telegramUpdateId);
    return;
  }

  String url = "https://api.telegram.org/bot" + String(TELEGRAM_CMD_TOKEN)
             + "/getUpdates?offset=" + String(telegramUpdateId)
             + "&limit=5&timeout=1";
  http.begin(url);
  setupHttpClient(http);
  int code = http.GET();
  esp_task_wdt_reset();

  if (code != 200) {
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();

  // Сдвигаем offset на последний update_id + 1
  int lastIdx = payload.lastIndexOf("\"update_id\":");
  if (lastIdx >= 0) {
    long lastId = payload.substring(lastIdx + 12).toInt();
    telegramUpdateId = lastId + 1;
  }

  // Проверяем, что сообщение от нашего чата
  if (payload.indexOf(String(TELEGRAM_CHAT_ID)) < 0) return;

  if (payload.indexOf("/balcony_esp_rst") >= 0) {
    Serial.println("[Telegram] Получена команда /balcony_esp_rst");
    String msg = "🔄 *БАЛКОН — Удалённый ребут*\n";
    msg += "━━━━━━━━━━━━━━━━━━━━━\n\n";
    msg += "📩 Команда: `/balcony\\_esp\\_rst`\n";
    msg += "⚡ Перезагрузка...\n\n";
    msg += "━━━━━━━━━━━━━━━━━━━━━";
    sendTelegramMessage(msg);
    delay(1000);
    requestReboot(REBOOT_TELEGRAM);
  }

  if (payload.indexOf("/balcony_status") >= 0) {
    Serial.println("[Telegram] Получена команда /balcony_status");
    unsigned long upMin = millis() / 60000UL;
    String msg = "📊 *БАЛКОН — Статус*\n";
    msg += "━━━━━━━━━━━━━━━━━━━━━\n\n";

    msg += "⏱ Uptime: `" + String(upMin / 60) + "ч " + String(upMin % 60) + "м`\n";
    msg += "📶 RSSI: `" + String(WiFi.RSSI()) + " dBm`\n";
    msg += "🌐 IP: `" + WiFi.localIP().toString() + "`\n";
    msg += "🧠 Heap: `" + String(ESP.getFreeHeap()) + " bytes` (min: `" + String(sessionMinHeap) + "`)\n\n";

    msg += "📡 *Датчики*\n";
    msg += "  " + String(bmeReady ? "✅" : "❌") + " BME280 — stale `" + String(staleBmeCount) + "/" + String(STALE_BME_THRESHOLD) + "`\n";
    msg += "  " + String(pmsIsAwake ? "🔆" : "😴") + " PMS5003 " + String(pmsIsAwake ? "активен" : "спит");
    msg += " — stale `" + String(stalePmsCount) + "/" + String(STALE_PMS_THRESHOLD) + "`\n";
    msg += "  🗄 Supabase — ошибок: `" + String(supabaseTotalErrors) + "`\n\n";

    msg += "📝 *Показания*\n";
    float t, h, p;
    if (readClimate(t, h, p)) {
      msg += "  🌡 `" + String(t, 1) + " °C`\n";
      msg += "  💧 `" + String(h, 0) + " %`\n";
      msg += "  📉 `" + String(p, 1) + " mmHg`\n";
    }
    if (hasPmsStats) {
      msg += "  💨 PM2.5: `" + String((int)lastStatsPm25.median) + " мкг/м³`\n";
    }

    msg += "\n━━━━━━━━━━━━━━━━━━━━━";
    sendTelegramMessage(msg);
  }

  if (payload.indexOf("/balcony_logs") >= 0) {
    Serial.println("[Telegram] Получена команда /balcony_logs");
    String msg = "📋 *БАЛКОН — Лог событий*\n";
    msg += "━━━━━━━━━━━━━━━━━━━━━\n\n";
    msg += eventLog.format();
    msg += "\n━━━━━━━━━━━━━━━━━━━━━";
    sendTelegramMessage(msg);
  }

  if (payload.indexOf("/balcony_errors") >= 0) {
    Serial.println("[Telegram] Получена команда /balcony_errors");
    String msg = "🚨 *БАЛКОН — Лог ошибок*\n";
    msg += "━━━━━━━━━━━━━━━━━━━━━\n\n";
    msg += "*Текущая сессия:*\n";
    msg += errorLog.format();
    msg += "\n*Сохранено в RTC (переживает ребут):*\n";
    msg += formatRtcErrorLog();
    msg += "\n━━━━━━━━━━━━━━━━━━━━━";
    sendTelegramMessage(msg);
  }
}

// ============================================================
// NTP — форматирование времени
// ============================================================
void updateTimeStr(char* buf, size_t len) {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 0)) {
    strftime(buf, len, "%d.%m %H:%M", &timeinfo);
  }
}

void formatClockShort(char* buf, size_t len) {
  struct tm timeinfo;
  if (ntpSynced && getLocalTime(&timeinfo, 0)) {
    strftime(buf, len, "%H:%M", &timeinfo);
  } else {
    snprintf(buf, len, "--:--");
  }
}

void formatNextWallTime(unsigned long targetMs, unsigned long nowMs, char* buf, size_t len) {
  if (!ntpSynced) {
    snprintf(buf, len, "---");
    return;
  }

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 0)) {
    snprintf(buf, len, "---");
    return;
  }

  time_t nowEpoch = mktime(&timeinfo);
  long deltaSec = (long)((targetMs > nowMs) ? (targetMs - nowMs) / 1000UL : 0);
  time_t nextEpoch = nowEpoch + deltaSec;
  struct tm nextTm;
  localtime_r(&nextEpoch, &nextTm);
  strftime(buf, len, "%H:%M", &nextTm);
}

void formatCountdown(unsigned long targetMs, unsigned long nowMs, char* buf, size_t len) {
  if (targetMs <= nowMs) {
    snprintf(buf, len, "now");
    return;
  }

  unsigned long sec = (targetMs - nowMs) / 1000UL;
  if (sec >= 3600UL) {
    snprintf(buf, len, "%lum%02lus", sec / 60UL, sec % 60UL);
  } else {
    snprintf(buf, len, "%lum%02lus", sec / 60UL, sec % 60UL);
  }
}

unsigned long nextPmsCompleteTime() {
  if (pmsSampling) {
    return pmsSamplingStartTime + PMS_SAMPLE_TIME;
  }
  return pmsWakeupTargetTime + PMS_WAKEUP_TIME + PMS_SAMPLE_TIME;
}

void oledDrawLine(int y) {
  display.drawLine(0, y, 127, y, SSD1306_WHITE);
}

void oledDrawHeader(uint8_t page, const char* title) {
  char clockBuf[6];
  formatClockShort(clockBuf, sizeof(clockBuf));
  display.setCursor(0, 0);
  display.printf("[%d/%d] %-5s %s+4", page + 1, OLED_PAGE_COUNT, title, clockBuf);
  oledDrawLine(10);
}

// ============================================================
// OLED SSD1306 128x64 — статус системы (страницы)
// ============================================================
void updateOled(uint8_t page) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  unsigned long now = millis();

  if (pmsErrorCount > 0 || !bmeReady) {
    display.setTextSize(2);
    display.setCursor(22, 4);
    display.print(F("ALERT!"));
    display.setTextSize(1);
    display.drawLine(0, 24, 127, 24, SSD1306_WHITE);
    display.setCursor(0, 30);
    if (!bmeReady) {
      display.print(F("BME280 FAIL"));
    } else {
      display.printf("PMS ERR x%d", pmsErrorCount);
    }
    display.setCursor(0, 46);
    display.print(F("Check hardware!"));
    display.display();
    return;
  }

  switch (page) {
    case OLED_PAGE_CLIMATE: {
      oledDrawHeader(page, "KLIMA");

      display.setCursor(0, 14);
      if (hasClimateReading) {
        display.printf("T:%5.1fC  H:%3.0f%%", lastClimateTemp, lastClimateHum);
        display.setCursor(0, 24);
        display.printf("P:%6.1f mmHg", lastClimatePres);
      } else {
        display.print(F("No readings yet"));
      }

      oledDrawLine(34);

      display.setCursor(0, 38);
      display.print(F("Last: "));
      display.print(lastClimateTimeStr);

      char nextTime[6];
      char nextIn[12];
      formatNextWallTime(lastClimateTickTime + CLIMATE_TICK_INTERVAL, now, nextTime, sizeof(nextTime));
      formatCountdown(lastClimateTickTime + CLIMATE_TICK_INTERVAL, now, nextIn, sizeof(nextIn));
      display.setCursor(0, 50);
      display.printf("Next: %s (%s)", nextTime, nextIn);
      break;
    }

    case OLED_PAGE_DUST: {
      oledDrawHeader(page, "PYL");

      display.setCursor(0, 14);
      if (hasPmsStats) {
        display.printf("PM2.5:%4.0f  PM10:%4.0f",
                       lastStatsPm25.median, lastStatsPm10.median);
        display.setCursor(0, 24);
        display.printf("PM1:%5.0f  n=%d",
                       lastStatsPm1.median, lastStatsPm25.count);
      } else {
        display.print(F("No PMS data yet"));
      }

      oledDrawLine(34);

      display.setCursor(0, 38);
      display.print(F("Last: "));
      display.print(lastPmsTimeStr);

      if (pmsSampling) {
        display.setCursor(0, 50);
        display.print(F("Next: SAMPLING..."));
      } else if (pmsIsAwake) {
        display.setCursor(0, 50);
        display.print(F("Next: WARMUP..."));
      } else {
        char nextTime[6];
        char nextIn[12];
        unsigned long nextDone = nextPmsCompleteTime();
        formatNextWallTime(nextDone, now, nextTime, sizeof(nextTime));
        formatCountdown(nextDone, now, nextIn, sizeof(nextIn));
        display.setCursor(0, 50);
        display.printf("Next: %s (%s)", nextTime, nextIn);
      }
      break;
    }

    case OLED_PAGE_CLOUD: {
      oledDrawHeader(page, "CLOUD");

      display.setCursor(0, 14);
      display.print(F("Target: Supabase"));
      display.setCursor(0, 24);
      display.printf("Every %lum", CLIMATE_SEND_INTERVAL / 60000UL);

      oledDrawLine(34);

      display.setCursor(0, 38);
      display.print(F("Last: "));
      display.print(lastCloudSendTimeStr);

      char nextTime[6];
      char nextIn[12];
      formatNextWallTime(lastClimateSendTime + CLIMATE_SEND_INTERVAL, now, nextTime, sizeof(nextTime));
      formatCountdown(lastClimateSendTime + CLIMATE_SEND_INTERVAL, now, nextIn, sizeof(nextIn));
      display.setCursor(0, 50);
      display.printf("Next: %s (%s)", nextTime, nextIn);

      if (supabaseTotalErrors > 0) {
        display.setCursor(90, 14);
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.printf(" E:%d ", supabaseTotalErrors);
        display.setTextColor(SSD1306_WHITE);
      }
      break;
    }

    case OLED_PAGE_HARDWARE: {
      oledDrawHeader(page, "HW");

      display.setCursor(0, 14);
      display.print(F("BME280: "));
      display.print(bmeReady ? F("OK") : F("FAIL"));

      display.setCursor(0, 24);
      display.print(F("PMS:    "));
      if (pmsSampling) {
        display.print(F("SAMPLING"));
      } else if (pmsIsAwake) {
        display.print(F("WARMUP"));
      } else {
        display.print(F("SLEEP"));
      }

      display.setCursor(0, 34);
      display.print(F("WiFi:   "));
      if (WiFi.status() == WL_CONNECTED) {
        display.printf("OK %ddBm", WiFi.RSSI());
      } else {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.print(F(" DOWN "));
        display.setTextColor(SSD1306_WHITE);
      }

      oledDrawLine(44);

      display.setCursor(0, 48);
      if (WiFi.status() == WL_CONNECTED) {
        display.print(WiFi.localIP());
      } else {
        display.print(F("No network"));
      }

      display.setCursor(0, 58);
      if (supabaseTotalErrors == 0 && bmeReady && pmsErrorCount == 0) {
        display.print(F("ALL OK"));
      } else {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.printf(" ERR:%d ", supabaseTotalErrors + (bmeReady ? 0 : 1));
        display.setTextColor(SSD1306_WHITE);
      }
      break;
    }

    case OLED_PAGE_SYSTEM: {
      oledDrawHeader(page, "SYS");

      display.setCursor(0, 14);
      display.printf("CPU: %.1f C", temperatureRead());

      unsigned long upMin = now / 60000UL;
      display.setCursor(0, 24);
      display.printf("Up:  %luh %02lum", upMin / 60, upMin % 60);

      display.setCursor(0, 34);
      display.printf("Heap: %lu  min:%lu",
                     (unsigned long)ESP.getFreeHeap(),
                     (unsigned long)sessionMinHeap);

      oledDrawLine(44);

      display.setCursor(0, 48);
      display.print(F("Boot: "));
      display.print(bootTimeStr);

      display.setCursor(0, 58);
      display.printf("Rst: %s", rebootReasonShort);
      break;
    }
  }

  display.display();
}

// ============================================================
// HTTP helpers
// ============================================================
void setupHttpClient(HTTPClient &http) {
  http.setConnectTimeout(5000);
  http.setTimeout(HTTP_TIMEOUT_MS);
}

// ============================================================
// Диагностика ребутов и персистентный лог ошибок
// ============================================================
void rtcErrorPersist(const char* text, uint32_t uptimeMs) {
  strncpy(rtcErrorEntries[rtcErrorHead], text, LOG_ENTRY_LEN - 1);
  rtcErrorEntries[rtcErrorHead][LOG_ENTRY_LEN - 1] = '\0';
  rtcErrorUptime[rtcErrorHead] = uptimeMs;
  rtcErrorHead = (rtcErrorHead + 1) % RTC_ERROR_LOG_SIZE;
  if (rtcErrorCount < RTC_ERROR_LOG_SIZE) rtcErrorCount++;
}

void logError(const char* text) {
  errorLog.add(text);
  rtcErrorPersist(text, millis());
}

String formatRtcErrorLog() {
  if (rtcErrorCount == 0) return "  _пусто_\n";
  String result = "";
  int start = (rtcErrorHead - rtcErrorCount + RTC_ERROR_LOG_SIZE) % RTC_ERROR_LOG_SIZE;
  for (int i = 0; i < rtcErrorCount; i++) {
    int idx = (start + i) % RTC_ERROR_LOG_SIZE;
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "uptime %luh%02lum",
             rtcErrorUptime[idx] / 3600000UL,
             (rtcErrorUptime[idx] / 60000UL) % 60UL);
    result += "  `" + String(timeBuf) + "` " + String(rtcErrorEntries[idx]) + "\n";
  }
  return result;
}

String resetReasonLabel(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWER ON";
    case ESP_RST_EXT:       return "EXT PIN";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT WDT";
    case ESP_RST_TASK_WDT:  return "TASK WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEP SLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "OTHER(" + String((int)reason) + ")";
  }
}

String rebootCauseLabel(RebootCause cause) {
  switch (cause) {
    case REBOOT_WIFI_TIMEOUT: return "WiFi таймаут 10м";
    case REBOOT_DAILY:        return "Суточный ребут";
    case REBOOT_BME_STALE:    return "BME280 stale";
    case REBOOT_PMS_STALE:    return "PMS5003 stale";
    case REBOOT_TELEGRAM:     return "Telegram /balcony_esp_rst";
    case REBOOT_MQTT:         return "MQTT reboot";
    default:                  return "неизвестно";
  }
}

String formatUptime(unsigned long ms) {
  unsigned long sec = ms / 1000UL;
  return String(sec / 3600UL) + "ч " + String((sec % 3600UL) / 60UL) + "м " + String(sec % 60UL) + "с";
}

String buildRebootDebugBlock(esp_reset_reason_t hwReason) {
  String block = "🔍 *Диагностика:*\n";
  block += "  HW reset: `" + resetReasonLabel(hwReason) + " (#" + String((int)hwReason) + ")`\n";

  if (hwReason == ESP_RST_SW && rtcRebootMagic == REBOOT_RTC_MAGIC) {
    block += "  SW reset: `" + rebootCauseLabel((RebootCause)rtcRebootCode) + "`\n";
    block += "  Prev uptime: `" + formatUptime(rtcRebootUptime) + "`\n";
    block += "  Prev heap: `" + String(rtcRebootHeap) + "` (min: `" + String(rtcRebootMinHeap) + "`)\n";
    block += "  Prev stale PMS: `" + String(rtcRebootStalePms) + "/" + String(STALE_PMS_THRESHOLD) + "`\n";
    block += "  Prev stale BME: `" + String(rtcRebootStaleBme) + "/" + String(STALE_BME_THRESHOLD) + "`\n";
  } else if (hwReason != ESP_RST_POWERON && hwReason != ESP_RST_SW) {
    block += "  ⚠️ Неплановый ребут — см. лог ошибок RTC\n";
  }

  block += "  Heap now: `" + String(ESP.getFreeHeap()) + "`\n";
  block += "  Min heap (IDF): `" + String(esp_get_minimum_free_heap_size()) + "`\n\n";
  return block;
}

void requestReboot(RebootCause cause) {
  rtcRebootMagic    = REBOOT_RTC_MAGIC;
  rtcRebootCode     = cause;
  rtcRebootUptime   = millis();
  rtcRebootHeap     = ESP.getFreeHeap();
  rtcRebootMinHeap  = sessionMinHeap;
  rtcRebootStalePms = stalePmsCount;
  rtcRebootStaleBme = staleBmeCount;

  char buf[LOG_ENTRY_LEN];
  snprintf(buf, sizeof(buf), "REBOOT -> %s", rebootCauseLabel(cause).c_str());
  logError(buf);

  Serial.printf("[Reboot] Запрос: %s, uptime: %s, heap: %lu, minHeap: %lu\n",
                rebootCauseLabel(cause).c_str(),
                formatUptime(millis()).c_str(),
                (unsigned long)rtcRebootHeap,
                (unsigned long)rtcRebootMinHeap);
  delay(300);
  esp_restart();
}
