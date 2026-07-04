#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
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
#include "firmware_info.h"

// MQTT-топики (DEVICE_HOSTNAME из secrets.h):
//   devices/<hostname>/status       — online/offline (LWT)
//   devices/<hostname>/telemetry    — периодическая телеметрия
//   devices/<hostname>/command      — JSON-команды
//   devices/<hostname>/capabilities — retained JSON c описанием команд

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

#define MQTT_BUFFER_SIZE  4096
#define FS_READ_MAX_BYTES 2800
#define FS_LS_MAX_ENTRIES 32

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
bool bootPhase = true;

#define LED_LOADING_CYCLE_MS 400

// Причина последнего ребута
char rebootReasonShort[17] = "";

// Программные причины ребута (сохраняются в RTC между esp_restart)
enum RebootCause : int {
  REBOOT_NONE        = 0,
  REBOOT_WIFI_TIMEOUT = 1,
  REBOOT_DAILY       = 2,
  REBOOT_BME_STALE   = 3,
  REBOOT_PMS_STALE   = 4,
  REBOOT_MQTT        = 5,
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

WiFiClient mqttNet;
PubSubClient mqttClient(mqttNet);

char topicStatus[64];
char topicTelemetry[64];
char topicCommand[64];
char topicCapabilities[64];
bool mqttTopicsReady = false;

const char *CAPABILITIES = R"CAP({
  "commands": [
    {
      "action": "led",
      "title": "Светодиод",
      "type": "toggle",
      "icon": "lightbulb",
      "description": "Встроенный LED платы (GPIO 2)"
    },
    {
      "action": "status",
      "title": "HW",
      "type": "trigger",
      "icon": "monitor",
      "description": "Показать страницу HW на OLED"
    },
    {
      "action": "sync",
      "title": "Синхронизация времени",
      "type": "trigger",
      "icon": "clock",
      "description": "Синхронизация NTP"
    },
    {
      "action": "push",
      "title": "Отправить в Supabase",
      "type": "trigger",
      "icon": "cloud-upload",
      "description": "Принудительная отправка в Supabase"
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
unsigned long lastMqttTelemetry = 0;

char otaUrl[256] = "";
bool otaPending = false;
int lastOtaProgress = -1;
bool littleFsReady = false;

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
void    publishOtaEvent(const char* phase, int progress = -1);
void    queueOtaUpdate(const char* url);
void    performOtaUpdate(const char* url);
void    showOtaScreen(const char* line1, const char* line2);
bool    isFsPathValid(const char* path);
void    publishFsTelemetry(JsonDocument& doc);
void    publishFsError(const char* action, const char* path, const char* error);
void    handleFsLs(const char* path);
void    handleFsRead(const char* path);
void    handleFsWrite(const char* path, const char* content);
void    handleFsRm(const char* path);
bool    sendDataToSupabase(float t, float h, float p);
bool    pushClimateToSupabase();
bool    readClimate(float &t, float &h, float &p);
bool    isClimateValid(float t, float h, float p);
float   getMedian(float* array, int size);
String  pmsStatsJson(const char* prefix, PmsStats &s);
void    setLedColor(bool r, bool g, bool y);
void    updateLoadingLedCycle();
void    updateErrorLedBlink();
void    delayWithLoadingLed(unsigned long ms);
bool    isLoadingLedActive();
bool    isErrorLedActive();
void    refreshTrafficLed();
void    updateTrafficLight();
void    updateOled(uint8_t page);
bool    syncTime();
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

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_Y, OUTPUT);

  delayWithLoadingLed(1000);
  Serial.println("\n--- Уличная метеостанция (Климат 5м / Пыль 30м) ---");
  logFirmwareInfo("esp32-balcony");

  eventLog.init(EVENT_LOG_SIZE);
  errorLog.init(ERROR_LOG_SIZE);

  esp_reset_reason_t rstReason = esp_reset_reason();
  if (rstReason != ESP_RST_SW) rtcRebootMagic = 0;
  sessionMinHeap = ESP.getFreeHeap();

  pinMode(LED_BUILTIN, OUTPUT);
  setBoardLed(false);

  littleFsReady = LittleFS.begin(true, "/littlefs", 10, "spiffs");
  if (!littleFsReady) {
    littleFsReady = LittleFS.begin(true);
  }
  if (!littleFsReady) {
    Serial.println("[LittleFS] mount FAILED");
    eventLog.add("FS FAIL");
  } else {
    Serial.printf("[LittleFS] mounted, used=%u total=%u\n",
                  LittleFS.usedBytes(), LittleFS.totalBytes());
    eventLog.add("FS OK");
  }

  // Инициализация OLED SSD1306 128x64 (отдельная I2C шина на пинах 18/19)
  oledWire.begin(OLED_SDA, OLED_SCL);
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("System Booting..."));
    display.printf("FW %s\n", FW_VERSION);
    display.print(FW_BUILD_DATE);
    display.display();
    Serial.println("[OK] OLED SSD1306 инициализирован.");
  } else {
    Serial.println("[Ошибка] OLED SSD1306 не найден!");
  }
  setLedColor(false, false, false);
  refreshTrafficLed();

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
    delayWithLoadingLed(1000);
    Serial.print("#");
    waitAttempts++;
  }

  // --- Честный стартовый прогрев PMS (30 сек) + серия замеров ---
  if (WiFi.status() == WL_CONNECTED) {
    eventLog.add("WiFi OK");
    lastWifiConnectedTime = millis();
    ensureMqtt();

    syncTime();

    Serial.println("[Система] Старт полноценного прогрева PMS5003 для честного стартового замера...");
    pms.wakeUp();

    for (int i = 0; i < 30; i++) {
      delayWithLoadingLed(1000);
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
      delayWithLoadingLed(PMS_SAMPLE_DELAY);
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

    rtcRebootMagic = 0;

    Serial.println("[Старт] Рапорт загрузки:");
    Serial.printf("  Причина: %s\n", rebootReasonStr.c_str());
    Serial.print(buildRebootDebugBlock(rstReason));
    Serial.printf("  IP: %s, RSSI: %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    Serial.printf("  BME280: %s\n", bmeReady ? "OK" : "ОШИБКА");
    float sT = 0, sH = 0, sP = 0;
    if (readClimate(sT, sH, sP)) {
      Serial.printf("  Климат: %.1f C, %.0f %%, %.1f mmHg\n", sT, sH, sP);
    }
    if (hasPmsStats) {
      Serial.printf("  PM2.5: %.0f (n=%d)\n", lastStatsPm25.median, lastStatsPm25.count);
    }
    if (rtcErrorCount > 0) {
      Serial.println("  Ошибки прошлой сессии (RTC):");
      Serial.println(formatRtcErrorLog());
    }
    eventLog.add("BOOT OK");
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

  bootPhase = false;
  Serial.println("[Система] Вход в рабочий цикл.");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  if (otaPending) {
    otaPending = false;
    performOtaUpdate(otaUrl);
    return;
  }

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
    Serial.printf("[Профилактика] heap=%u, rssi=%d, supabase_errors=%d\n",
                  ESP.getFreeHeap(), WiFi.RSSI(), supabaseTotalErrors);
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
        if (pmsErrorCount >= PMS_MAX_ERRORS) {
          Serial.println("[PMS5003] Критическая серия ошибок — счётчик сброшен");
          pmsErrorCount = 0;
        }
      }

      pms.sleep();
      pmsIsAwake  = false;
      pmsSampling = false;
      pmsWakeupTargetTime = now + PMS_READ_INTERVAL - PMS_WAKEUP_TIME - PMS_SAMPLE_TIME;
    }
  }

  // ── 5. Индикация светодиодами ──
  refreshTrafficLed();

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
    pushClimateToSupabase();
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

bool isLoadingLedActive() {
  return bootPhase || !hasPmsStats || pmsIsAwake || pmsSampling;
}

bool isErrorLedActive() {
  return !bmeReady
      || pmsErrorCount > 0
      || WiFi.status() != WL_CONNECTED
      || supabaseErrorCount > 0;
}

void updateLoadingLedCycle() {
  switch ((millis() / LED_LOADING_CYCLE_MS) % 3) {
    case 0: setLedColor(true,  false, false); break;
    case 1: setLedColor(false, false, true);  break;
    default: setLedColor(false, true,  false); break;
  }
}

void updateErrorLedBlink() {
  static const uint16_t segments[] = {
    90, 210, 60, 340, 120, 160, 80, 290, 100, 240
  };
  const uint8_t segmentCount = sizeof(segments) / sizeof(segments[0]);
  unsigned long cycleMs = 0;
  for (uint8_t i = 0; i < segmentCount; i++) {
    cycleMs += segments[i];
  }

  unsigned long t = millis() % cycleMs;
  uint16_t acc = 0;
  bool on = false;
  for (uint8_t i = 0; i < segmentCount; i++) {
    acc += segments[i];
    if (t < acc) {
      on = (i % 2 == 0);
      break;
    }
  }
  setLedColor(false, false, on);
}

void refreshTrafficLed() {
  if (pmsCriticalBlink) {
    bool blinkOn = (millis() / 300) % 2 == 0;
    setLedColor(blinkOn, false, false);
  } else if (isErrorLedActive()) {
    updateErrorLedBlink();
  } else if (isLoadingLedActive()) {
    updateLoadingLedCycle();
  }
}

void delayWithLoadingLed(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    refreshTrafficLed();
    delay(50);
  }
}

void updateTrafficLight() {
  if (!hasPmsStats) {
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
bool pushClimateToSupabase() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Supabase] Нет Wi-Fi — пропуск.");
    return false;
  }

  float fT, fH, fP;
  if (climateBufferIndex > 0) {
    fT = getMedian(tempBuffer, climateBufferIndex);
    fH = getMedian(humBuffer,  climateBufferIndex);
    fP = getMedian(presBuffer, climateBufferIndex);
    climateBufferIndex = 0;
  } else if (!readClimate(fT, fH, fP)) {
    Serial.println("[Supabase] Нет климата — пропуск.");
    lastClimateSendTime = millis();
    climateBufferIndex = 0;
    return false;
  }

  bool ok = sendDataToSupabase(fT, fH, fP);
  lastClimateSendTime = millis();
  return ok;
}

bool sendDataToSupabase(float t, float h, float p) {
  if (WiFi.status() != WL_CONNECTED) return false;

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
    return true;
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
      Serial.printf("[Supabase] Критическая серия ошибок (%d), счётчик сброшен\n",
                    supabaseErrorCount);
      supabaseErrorCount = 0;
    }
  }
  return false;
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
    if (bootPhase) {
      delayWithLoadingLed(500);
    } else {
      delay(500);
    }
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
  snprintf(topicCapabilities, sizeof(topicCapabilities), "devices/%s/capabilities", DEVICE_HOSTNAME);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(handleMqttCommand);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
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

bool isFsPathValid(const char* path) {
  if (!path || path[0] != '/') return false;
  if (strstr(path, "..") != nullptr) return false;
  return strlen(path) < 128;
}

void publishFsTelemetry(JsonDocument& doc) {
  if (!mqttClient.connected()) {
    Serial.println("[FS] mqtt not connected, skip publish");
    return;
  }

  char buf[MQTT_BUFFER_SIZE];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0 || n >= sizeof(buf)) {
    Serial.println("[FS] response too large for MQTT buffer");
    publishFsError("fs", nullptr, "response too large");
    return;
  }
  Serial.printf("[FS] >> %s (%u bytes)\n", topicTelemetry, n);
  bool ok = mqttClient.publish(topicTelemetry, buf, n);
  Serial.printf("[FS] publish %s\n", ok ? "OK" : "FAILED");
  mqttClient.loop();
}

void publishFsError(const char* action, const char* path, const char* error) {
  Serial.printf("[FS] error action=%s path=%s: %s\n",
                action ? action : "-", path ? path : "-", error);
  if (!mqttClient.connected()) return;

  StaticJsonDocument<256> doc;
  doc["fs_error"] = error;
  doc["fs_action"] = action;
  if (path) doc["fs_path"] = path;

  char buf[256];
  size_t n = serializeJson(doc, buf);
  mqttClient.publish(topicTelemetry, buf, n);
  mqttClient.loop();
}

void handleFsLs(const char* path) {
  if (!littleFsReady) {
    publishFsError("fs_ls", path, "filesystem not mounted");
    return;
  }
  if (!isFsPathValid(path)) {
    publishFsError("fs_ls", path, "invalid path");
    return;
  }

  File dir = LittleFS.open(path);
  if (!dir || !dir.isDirectory()) {
    publishFsError("fs_ls", path, dir ? "not a directory" : "open failed");
    if (dir) dir.close();
    return;
  }

  DynamicJsonDocument doc(4096);
  JsonArray arr = doc.createNestedArray("fs_ls");
  int count = 0;

  for (File entry = dir.openNextFile(); entry && count < FS_LS_MAX_ENTRIES; entry = dir.openNextFile()) {
    char fullPath[128];
    if (strcmp(path, "/") == 0) {
      snprintf(fullPath, sizeof(fullPath), "/%s", entry.name());
    } else {
      snprintf(fullPath, sizeof(fullPath), "%s/%s", path, entry.name());
    }

    JsonObject item = arr.createNestedObject();
    item["name"] = fullPath;
    item["size"] = entry.size();
    Serial.printf("[FS]   %s (%u)\n", fullPath, entry.size());
    entry.close();
    count++;
  }
  dir.close();

  if (count >= FS_LS_MAX_ENTRIES) {
    doc["fs_ls_truncated"] = true;
  }

  Serial.printf("[FS] ls %s -> %d entries\n", path, count);
  publishFsTelemetry(doc);
}

void handleFsRead(const char* path) {
  if (!littleFsReady) {
    publishFsError("fs_read", path, "filesystem not mounted");
    return;
  }
  if (!isFsPathValid(path)) {
    publishFsError("fs_read", path, "invalid path");
    return;
  }

  if (!LittleFS.exists(path)) {
    publishFsError("fs_read", path, "not found");
    return;
  }

  File f = LittleFS.open(path, "r");
  if (!f) {
    publishFsError("fs_read", path, "open failed");
    return;
  }
  if (f.isDirectory()) {
    f.close();
    publishFsError("fs_read", path, "is a directory");
    return;
  }

  size_t fileSize = f.size();
  if (fileSize > FS_READ_MAX_BYTES) {
    f.close();
    publishFsError("fs_read", path, "file too large");
    return;
  }

  String content;
  content.reserve(fileSize + 1);
  while (f.available()) {
    content += static_cast<char>(f.read());
  }
  f.close();

  DynamicJsonDocument doc(fileSize + 192);
  doc["fs_file"] = path;
  doc["content"] = content;
  Serial.printf("[FS] read %s (%u bytes)\n", path, fileSize);
  publishFsTelemetry(doc);
}

void handleFsWrite(const char* path, const char* content) {
  if (!littleFsReady) {
    publishFsError("fs_write", path, "filesystem not mounted");
    return;
  }
  if (!isFsPathValid(path)) {
    publishFsError("fs_write", path, "invalid path");
    return;
  }
  if (!content) content = "";

  File f = LittleFS.open(path, "w");
  if (!f) {
    publishFsError("fs_write", path, "open failed");
    return;
  }

  size_t written = f.print(content);
  f.close();

  StaticJsonDocument<192> doc;
  doc["fs_written"] = path;
  doc["bytes"] = written;
  Serial.printf("[FS] write %s (%u bytes)\n", path, written);
  publishFsTelemetry(doc);
}

void handleFsRm(const char* path) {
  if (!littleFsReady) {
    publishFsError("fs_rm", path, "filesystem not mounted");
    return;
  }
  if (!isFsPathValid(path)) {
    publishFsError("fs_rm", path, "invalid path");
    return;
  }

  if (!LittleFS.exists(path)) {
    publishFsError("fs_rm", path, "not found");
    return;
  }

  if (!LittleFS.remove(path)) {
    publishFsError("fs_rm", path, "remove failed");
    return;
  }

  StaticJsonDocument<128> doc;
  doc["fs_removed"] = path;
  Serial.printf("[FS] rm %s\n", path);
  publishFsTelemetry(doc);
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

  if (strcmp(action, "led") == 0) {
    if (doc["value"].is<bool>()) {
      setBoardLed(doc["value"]);
    } else if (doc["value"].is<int>()) {
      setBoardLed(doc["value"] != 0);
    } else {
      return;
    }
    Serial.printf("[MQTT] led %s\n", boardLedOn ? "on" : "off");
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
    return;
  }

  if (strcmp(action, "sync") == 0) {
    Serial.println("[MQTT] sync time");
    syncTime();
    return;
  }

  if (strcmp(action, "push") == 0) {
    Serial.println("[MQTT] push to Supabase");
    pushClimateToSupabase();
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

  if (strcmp(action, "fs_ls") == 0) {
    const char* path = doc["path"] | "/";
    handleFsLs(path);
    return;
  }

  if (strcmp(action, "fs_read") == 0) {
    const char* path = doc["path"];
    if (!path || path[0] == '\0') return;
    handleFsRead(path);
    return;
  }

  if (strcmp(action, "fs_write") == 0) {
    const char* path = doc["path"];
    const char* content = doc["content"];
    if (!path || path[0] == '\0') return;
    handleFsWrite(path, content);
    return;
  }

  if (strcmp(action, "fs_rm") == 0) {
    const char* path = doc["path"];
    if (!path || path[0] == '\0') return;
    handleFsRm(path);
    return;
  }

  Serial.printf("[MQTT] unknown action: %s\n", action);
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
    if (mqttClient.publish(topicCapabilities, CAPABILITIES, true)) {
      Serial.printf("[MQTT] capabilities -> %s\n", topicCapabilities);
    } else {
      Serial.printf("[MQTT] capabilities publish FAILED (%u bytes)\n", strlen(CAPABILITIES));
    }
    Serial.println("[MQTT] connected");
    Serial.printf("[MQTT] command  <- %s\n", topicCommand);
    Serial.printf("[MQTT] telemetry -> %s\n", topicTelemetry);
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
  addFirmwareTelemetry(doc);

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

void showOtaScreen(const char* line1, const char* line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OTA UPDATE");
  display.println(line1);
  display.println(line2);
  display.display();
}

void publishOtaEvent(const char* phase, int progress) {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<192> doc;
  doc["ota"] = phase;
  if (progress >= 0) doc["progress"] = progress;

  char buf[192];
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
  Serial.printf("[OTA] starting: %s\n", url);
  showOtaScreen("Downloading...", url);
  publishOtaEvent("starting", 0);
  lastOtaProgress = 0;

  httpUpdate.rebootOnUpdate(true);
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  httpUpdate.onStart([]() {
    Serial.println("[OTA] start");
    esp_task_wdt_reset();
    publishOtaEvent("downloading", 0);
    lastOtaProgress = 0;
  });

  httpUpdate.onProgress([](size_t current, size_t total) {
    esp_task_wdt_reset();
    int pct = (total > 0) ? (int)((current * 100UL) / total) : 0;
    Serial.printf("[OTA] %d%%\n", pct);
    if (pct >= lastOtaProgress + 10 || pct == 100) {
      lastOtaProgress = pct;
      publishOtaEvent("downloading", pct);
      char line[16];
      snprintf(line, sizeof(line), "%d%%", pct);
      showOtaScreen("Downloading...", line);
    }
  });

  httpUpdate.onEnd([]() {
    Serial.println("[OTA] complete");
    esp_task_wdt_reset();
    publishOtaEvent("rebooting", 100);
    showOtaScreen("Complete", "Rebooting...");
  });

  httpUpdate.onError([](int error) {
    Serial.printf("[OTA] error %d: %s\n", error, httpUpdate.getLastErrorString().c_str());
    esp_task_wdt_reset();
    publishOtaEvent("failed", -1);
    showOtaScreen("FAILED", httpUpdate.getLastErrorString().c_str());
  });

  t_httpUpdate_return ret;
  if (strncmp(url, "https://", 8) == 0) {
    WiFiClientSecure secureClient;
    secureClient.setInsecure();
    ret = httpUpdate.update(secureClient, url);
  } else {
    WiFiClient client;
    ret = httpUpdate.update(client, url);
  }

  if (ret != HTTP_UPDATE_OK) {
    Serial.printf("[OTA] failed: %s\n", httpUpdate.getLastErrorString().c_str());
    showOtaScreen("FAILED", httpUpdate.getLastErrorString().c_str());
    eventLog.add("OTA FAIL");
    delay(3000);
    updateOled(oledPage);
  }
}

// ============================================================
// NTP — синхронизация и форматирование времени
// ============================================================
bool syncTime() {
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, NTP_SERVER);

  Serial.print("[NTP] sync");
  struct tm timeinfo;
  int tries = 0;
  while (!getLocalTime(&timeinfo, 500) && tries < 10) {
    esp_task_wdt_reset();
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (getLocalTime(&timeinfo, 0)) {
    ntpSynced = true;
    updateTimeStr(bootTimeStr, sizeof(bootTimeStr));
    Serial.printf(" OK (%s)\n", bootTimeStr);
    eventLog.add("NTP OK");
    return true;
  }

  Serial.println(" FAIL");
  logError("NTP sync FAIL");
  return false;
}

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
      display.printf("FW: %s", FW_VERSION);

      display.setCursor(0, 48);
      display.print(FW_BUILD_DATE);

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
