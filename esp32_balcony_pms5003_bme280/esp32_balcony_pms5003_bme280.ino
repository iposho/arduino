#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PMS.h>
#include <algorithm>
#include <cmath>
#include <esp_task_wdt.h>
#include <BitBang_LiquidCrystal_I2C.h>

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
#define WDT_TIMEOUT_SEC    15

// Дисплей 1602A (16x2 LCD, I2C-адаптер PCF8574, bitbang)
#define LCD_ADDR    0x27   // Часто 0x27 или 0x3F
#define LCD_SDA     18
#define LCD_SCL     19
#define LCD_COLS    16
#define LCD_ROWS    2

// Страницы дисплея
#define LCD_PAGE_SYSTEM    0   // WiFi + BME + PMS статус
#define LCD_PAGE_AQI       1   // PM2.5 + цвет LED
#define LCD_PAGE_TIMER     2   // Таймеры + буфер
#define LCD_PAGE_COUNT     3
#define LCD_PAGE_MS       3000UL // 3 сек на страницу

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
// Объекты и состояние
// =====================
Adafruit_BME280 bme;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS, LCD_SDA, LCD_SCL);
PMS pms(Serial2);
PMS::DATA data;

bool bmeReady   = false;
bool pmsIsAwake = false;
int  pmsErrorCount = 0;

unsigned long lastClimateSendTime  = 0;
unsigned long lastClimateTickTime  = 0;
unsigned long lastPmsReadTime      = 0;
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

// Forward declarations
void    connectToWiFi();
void    sendDataToSupabase(float t, float h, float p);
void    sendTelegramMessage(String message);
bool    readClimate(float &t, float &h, float &p);
bool    isClimateValid(float t, float h, float p);
float   getMedian(float* array, int size);
String  pmsStatsJson(const char* prefix, PmsStats &s);
void    setLedColor(bool r, bool g, bool y);
void    updateTrafficLight();
void    updateLcd(uint8_t page);

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- Уличная метеостанция (Климат 5м / Пыль 30м) ---");

  // Инициализация пинов светодиода
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_Y, OUTPUT);

  // Инициализация LCD 1602A (bitbang I2C)
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("System Booting.."));
  Serial.println("[OK] LCD 1602A инициализирован.");
  setLedColor(false, false, true); // Желтый на старте (прогрев)

  // Настройка Hardware Watchdog
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms    = WDT_TIMEOUT_SEC * 1000UL,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wdtConfig);
  esp_task_wdt_add(NULL);

  memset(&lastStatsPm1,  0, sizeof(PmsStats));
  memset(&lastStatsPm25, 0, sizeof(PmsStats));
  memset(&lastStatsPm10, 0, sizeof(PmsStats));

  // --- BME280 ---
  bmeReady = bme.begin(BME280_ADDR, &Wire);
  Serial.println(bmeReady ? "[ОК] BME280 инициализирован." : "[Ошибка] BME280 не найден!");

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
    lastWifiConnectedTime = millis();
    Serial.println("[Система] Старт полноценного прогрева PMS5003 для честного стартового замера...");
    pms.wakeUp();

    for (int i = 0; i < 30; i++) {
      delay(1000);
      esp_task_wdt_reset(); // Не даем WDT сбросить плату во время долгого setup
      if (i % 5 == 0) Serial.printf("[Прогрев] Осталось %d сек...\n", 30 - i);
    }

    // Серия замеров при старте
    Serial.println("[PMS5003] Сбор честной стартовой серии...");
    pmsAcc.reset();
    unsigned long sStart = millis();
    while (millis() - sStart < PMS_SAMPLE_TIME && pmsAcc.count < PMS_SAMPLE_MAX) {
      esp_task_wdt_reset();
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
      Serial.printf("[PMS5003] Стартовая серия: n=%d, PM2.5 med=%.0f σ=%.1f\n",
                    lastStatsPm25.count, lastStatsPm25.median, lastStatsPm25.stddev);
      updateTrafficLight();
    } else {
      pmsErrorCount++;
      Serial.println("[Предупреждение] PMS5003: 0 чтений при старте.");
      setLedColor(true, false, false);
    }
    pms.sleep();

    // Определение причины перезагрузки для Telegram
    esp_reset_reason_t reason = esp_reset_reason();
    String rebootReasonStr = "POWERON_RESET / Запуск";
    if (reason == ESP_RST_WDT) rebootReasonStr = "🚨 WATCHDOG_RESET (Зависание системы)";
    else if (reason == ESP_RST_SW) rebootReasonStr = "🔄 SOFTWARE_RESET (Отказ сети)";

    // Telegram Рапорт
    float sT = 0, sH = 0, sP = 0;
    bool cOk = readClimate(sT, sH, sP);

    String msg = "🤖 *СТАТУС: МЕТЕОСТАНЦИЯ БАЛКОН*\n";
    msg += "▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬\n";
    msg += "⚡ *Система:* `" + rebootReasonStr + "`\n";
    msg += "⏱ *Uptime:* `" + String(millis()) + " ms`\n";
    msg += "🌐 *IP:* `" + WiFi.localIP().toString() + "`\n";
    msg += "📶 *RSSI:* `" + String(WiFi.RSSI()) + " dBm`\n\n";
    msg += "📡 *Железо:*\n";
    msg += (bmeReady ? "✅ BME280: `OK`" : "❌ BME280: `ERROR`") + String("\n");
    msg += "✅ PMS5003: `SLEEP` (Цикл: 30м)\n\n";
    msg += "📝 *Первичные замеры (Честные):*\n";
    if (cOk) {
      msg += "🌡 Температура: *" + String(sT, 1) + " °C*\n";
      msg += "💧 Влажность: *" + String(sH, 0) + " %*\n";
      msg += "📉 Давление: *" + String(sP, 1) + " mmHg*\n";
    } else {
      msg += "⚠️ BME280: нет данных\n";
    }
    if (hasPmsStats) {
      msg += "💨 PM2.5: *" + String((int)lastStatsPm25.median) + " мкг/м³*";
      msg += " (n=" + String(lastStatsPm25.count);
      msg += " min=" + String((int)lastStatsPm25.min);
      msg += " max=" + String((int)lastStatsPm25.max);
      msg += " σ=" + String(lastStatsPm25.stddev, 1) + ")\n";
    }
    msg += "▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬";
    sendTelegramMessage(msg);
  }

  // Выравнивание таймеров
  unsigned long now         = millis();
  lastClimateTickTime       = now - CLIMATE_TICK_INTERVAL;
  lastClimateSendTime       = now;
  lastPmsReadTime           = now;
  pmsWakeupTargetTime       = now + PMS_READ_INTERVAL - PMS_WAKEUP_TIME - PMS_SAMPLE_TIME;

  Serial.println("[Система] Вход в рабочий цикл.");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // СБРОС СТОРОЖЕВОГО ТАЙМЕРА (Кормим собаку)
  esp_task_wdt_reset();

  unsigned long now = millis();

  // Контроль отвала Wi-Fi
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
    // Если Wi-Fi лежит слишком долго — принудительный ребут
    if (now - lastWifiConnectedTime > WIFI_TIMEOUT_REBOOT) {
      Serial.println("[Критическая ошибка] Сеть недоступна более 10 минут. Ребут!");
      delay(500);
      esp_restart();
    }
  } else {
    lastWifiConnectedTime = now; // Сбрасываем таймер ошибки сети
  }

  // Профилактический суточный ребут (защита от утечек памяти)
  if (now > 24UL * 60UL * 60UL * 1000UL) {
    Serial.println("[Профилактика] Суточный перезапуск системы...");
    delay(1000);
    esp_restart();
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
      }
    }
  }

  // ── 2. Будим PMS за (прогрев + сбор) до дедлайна ──
  if (!pmsIsAwake && !pmsSampling && (long)(now - pmsWakeupTargetTime) >= 0) {
    pms.wakeUp();
    pmsIsAwake = true;
    setLedColor(false, false, true); // Желтый — прогрев лазера
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

        Serial.printf("[PMS5003] PM2.5: med=%.0f avg=%.1f σ=%.1f min=%.0f max=%.0f n=%d\n",
                      lastStatsPm25.median, lastStatsPm25.mean, lastStatsPm25.stddev,
                      lastStatsPm25.min, lastStatsPm25.max, lastStatsPm25.count);

        updateTrafficLight();
      } else {
        pmsErrorCount++;
        Serial.printf("[PMS5003] 0 чтений! Ошибок подряд: %d\n", pmsErrorCount);
        setLedColor(true, false, false);
        if (pmsErrorCount >= PMS_MAX_ERRORS) {
          String alert = "⚠️ *PMS5003 НЕ ОТВЕЧАЕТ*\nОшибок подряд: *";
          alert += String(pmsErrorCount) + "*\nПроверьте питание и UART.";
          sendTelegramMessage(alert);
          pmsErrorCount = 0;
        }
      }

      pms.sleep();
      pmsIsAwake  = false;
      pmsSampling = false;
      lastPmsReadTime     = now;
      pmsWakeupTargetTime = now + PMS_READ_INTERVAL - PMS_WAKEUP_TIME - PMS_SAMPLE_TIME;
    }
  }

  // ── 5. Обновление LCD 1602A (переключение страниц) ──
  static unsigned long lastLcdUpdate  = 0;
  static uint8_t       lcdPage        = 0;
  if (now - lastLcdUpdate >= LCD_PAGE_MS) {
    lastLcdUpdate = now;
    if (pmsErrorCount == 0 && bmeReady) {
      lcdPage = (lcdPage + 1) % LCD_PAGE_COUNT;  // цикл страниц
    }
    updateLcd(lcdPage);
  }

  // ── 6. Отправка в Supabase каждые 5 мин ──
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

  if (pm25 <= 12.0f) {
    setLedColor(false, true, false);  // Зеленый (Отлично)
  } else if (pm25 <= 35.0f) {
    setLedColor(true, true, false);   // Желтый (R + G = Смешиваются в желтый)
  } else {
    setLedColor(true, false, false);  // Красный (Плохо / Вредно)
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

  HTTPClient http;
  http.begin(SUPABASE_URL);
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
  Serial.printf("[Supabase] HTTP %d\n", code);
  http.end();
}

// ============================================================
// Wi-Fi
// ============================================================
void connectToWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("Wi-Fi... ");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int att = 0;
  // Неблокирующее ожидание внутри функции, чтобы WDT не сработал при долгом подключении
  while (WiFi.status() != WL_CONNECTED && att < 20) {
    esp_task_wdt_reset();
    delay(500);
    Serial.print(".");
    att++;
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf(" OK! %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println(" FAIL");
}

// ============================================================
// Telegram
// ============================================================
void sendTelegramMessage(String message) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Telegram] Нет Wi-Fi.");
    return;
  }
  message.replace("\\", "\\\\");
  message.replace("\"", "\\\"");

  HTTPClient http;
  String url = "https://api.telegram.org/bot" + String(TELEGRAM_TOKEN) + "/sendMessage";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String payload = "{\"chat_id\":\"" + String(TELEGRAM_CHAT_ID) +
                   "\",\"text\":\""  + message +
                   "\",\"parse_mode\":\"Markdown\"}";

  int code = http.POST(payload);
  if (code > 0) Serial.printf("[Telegram] %d\n", code);
  else          Serial.printf("[Telegram] ERR: %s\n", http.errorToString(code).c_str());
  http.end();
}

// ============================================================
// LCD 1602A — статус системы (16x2, страницы)
// ============================================================
void updateLcd(uint8_t page) {
  lcd.clear();
  unsigned long now = millis();

  if (pmsErrorCount > 0 || !bmeReady) {
    // ═══ Аварийная страница ═══
    lcd.setCursor(0, 0);
    lcd.print(F("*** ALERT ***"));
    lcd.setCursor(0, 1);
    if (!bmeReady)      lcd.print(F("BME280 FAIL"));
    else {
      lcd.print(F("PMS ERR x"));
      lcd.print(pmsErrorCount);
    }
    return;
  }

  // Текущее название цвета LED
  const char* ledName;
  if (!hasPmsStats)      ledName = "YELLOW";
  else {
    float pm25 = lastStatsPm25.median;
    if (pm25 <= 12.0f)      ledName = "GREEN";
    else if (pm25 <= 35.0f) ledName = "YELLOW";
    else                    ledName = "RED";
  }

  switch (page) {
    // ════════════════════════════════════════════
    case LCD_PAGE_SYSTEM: {
      lcd.setCursor(0, 0);
      if (WiFi.status() == WL_CONNECTED) {
        lcd.print(F("WiFi:OK "));
        lcd.print(WiFi.RSSI());
        lcd.print(F("dBm"));
      } else {
        lcd.print(F("WiFi:DOWN    "));
      }

      lcd.setCursor(0, 1);
      lcd.print(F("BME:"));
      lcd.print(bmeReady ? "OK " : "ERR");
      lcd.print(F(" PMS:"));
      if (pmsSampling)      lcd.print(F("SAMP"));
      else if (pmsIsAwake)  lcd.print(F("WARM"));
      else                  lcd.print(F("SLP "));
      break;
    }

    // ════════════════════════════════════════════
    case LCD_PAGE_AQI: {
      lcd.setCursor(0, 0);
      lcd.print(F("PM2.5: "));
      if (hasPmsStats) {
        lcd.print((int)lastStatsPm25.median);
        lcd.print(F(" ug/m3"));
      } else {
        lcd.print(F("--- ug/m3"));
      }

      lcd.setCursor(0, 1);
      lcd.print(F("LED: "));
      lcd.print(ledName);
      break;
    }

    // ════════════════════════════════════════════
    case LCD_PAGE_TIMER: {
      lcd.setCursor(0, 0);
      if (!pmsIsAwake && !pmsSampling) {
        lcd.print(F("Next: "));
        if (now < pmsWakeupTargetTime) {
          unsigned long sec = (pmsWakeupTargetTime - now) / 1000UL;
          if (sec >= 60) { lcd.print(sec / 60); lcd.print('m'); lcd.print(' '); }
          lcd.print(sec % 60);
          lcd.print('s');
        } else {
          lcd.print(F("NOW     "));
        }
      } else if (pmsSampling) {
        lcd.print(F("> SAMPLING <"));
      } else {
        lcd.print(F("> WARMUP <"));
      }

      lcd.setCursor(0, 1);
      lcd.print(F("Up:"));
      {
        unsigned long up = millis() / 60000UL;
        lcd.print(up / 60);
        lcd.print('h');
        lcd.print(up % 60);
        lcd.print('m');
      }
      lcd.print(F(" Buf:"));
      lcd.print(climateBufferIndex);
      lcd.print('/');
      lcd.print(CLIMATE_BUFFER_SIZE);
      break;
    }
  }
}
