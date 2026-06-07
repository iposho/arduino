#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// =====================
// Секреты — вынесены в secrets.h (.gitignore)
// =====================
#include "secrets.h"

// =====================
// TFT SPI pins
// =====================
#define TFT_CS    5
#define TFT_DC    2
#define TFT_RST   4
#define TFT_SCLK  18
#define TFT_MOSI  23

// =====================
// BME280 I2C pins
// =====================
#define BME_SDA_PIN 21
#define BME_SCL_PIN 22
#define BME280_ADDRESS 0x76

// =====================
// Joystick pins
// =====================
#define JOY_X_PIN   34
#define JOY_Y_PIN   35
#define JOY_SW_PIN  32

#define JOY_LEFT_THRESHOLD   1000
#define JOY_RIGHT_THRESHOLD  3000
#define JOY_DOWN_THRESHOLD   3000

// =====================
// Intervals
// =====================
const unsigned long SEND_INTERVAL         = 10UL * 60UL * 1000UL;
const unsigned long FETCH_INTERVAL        = 10UL * 60UL * 1000UL;
const unsigned long SENSOR_CHECK_INTERVAL = 2000UL;

const unsigned long TIME_SHOW_MS = 5000UL;
const unsigned long INFO_SHOW_MS = 5000UL;

// =====================
// TFT
// =====================
Adafruit_ST7735 tft = Adafruit_ST7735(
  TFT_CS,
  TFT_DC,
  TFT_MOSI,
  TFT_SCLK,
  TFT_RST
);

// =====================
// BME280
// =====================
Adafruit_BME280 bme;
bool bmeReady = false;

// =====================
// Screens
// =====================
enum Screen {
  SCREEN_HOME,
  SCREEN_OUTDOOR,
  SCREEN_AQI,
  SCREEN_COUNT
};

Screen currentScreen = SCREEN_HOME;

// =====================
// UI colors
// =====================
uint16_t COLOR_BG;
uint16_t COLOR_CARD;
uint16_t COLOR_TEXT;
uint16_t COLOR_MUTED;
uint16_t COLOR_BLUE;
uint16_t COLOR_GREEN;
uint16_t COLOR_ORANGE;
uint16_t COLOR_RED;
uint16_t COLOR_MAGENTA;
uint16_t COLOR_CYAN;
uint16_t COLOR_YELLOW;

// =====================
// Outdoor weather & PM data
// =====================
bool outDataValid = false;
float outTemp = 0;
float outHumidity = 0;
float outPressure = 0;
float outPm25 = 0;
float outPm10 = 0;

// =====================
// AQI Calculated
// =====================
bool aqiDataValid = false;
int aqiValue = 0;

// =====================
// Temporary screens
// =====================
bool showingTimeScreen = false;
unsigned long timeScreenStart = 0;

bool showingInfoScreen = false;
unsigned long infoScreenStart = 0;

// =====================
// Button state
// =====================
bool lastButtonState = HIGH;
unsigned long lastButtonTime = 0;
const unsigned long BUTTON_DEBOUNCE = 120;

// =====================
// Joystick state
// =====================
bool joyWasCentered = true;
unsigned long lastJoyTime = 0;
const unsigned long JOY_DEBOUNCE = 300;

// =====================
// Timers
// =====================
unsigned long lastSendTime = 0;
unsigned long lastFetchTime = 0;
unsigned long lastSensorCheckTime = 0;

// =====================
// Filtered & Last displayed home values
// =====================
float filteredTemp = -999.0;
float filteredHum  = -999.0;
float filteredPres = -999.0;
const float EMA_ALPHA = 0.1;

int lastDisplayedTemp = -999;
int lastDisplayedHumidity = -999;
int lastDisplayedPressure = -999;

// =====================
// Status
// =====================
char statusLine[40] = "Starting...";

// =====================
// Forward declarations
// =====================
void drawCurrentScreen();
void drawTimeScreen();
void drawInfoScreen();
void drawStatusBar();
void setStatus(const char* status);
void handleJoystick();
void handleButton();
void updateHomeScreenIfNeeded();
int calculateEPA_AQI(float pm25, float pm10);

// =====================
// Helpers
// =====================
float hPaToMmHg(float hPa) {
  return hPa * 0.75006375541921;
}

void setupColors() {
  COLOR_BG      = tft.color565(8, 10, 16);
  COLOR_CARD    = tft.color565(22, 26, 36);
  COLOR_TEXT    = ST77XX_WHITE;
  COLOR_MUTED   = tft.color565(150, 158, 172);
  COLOR_BLUE    = tft.color565(40, 100, 255);
  COLOR_GREEN   = tft.color565(40, 190, 120);
  COLOR_ORANGE  = tft.color565(255, 145, 40);
  COLOR_RED     = tft.color565(230, 60, 70);
  COLOR_MAGENTA = tft.color565(190, 80, 255);
  COLOR_CYAN    = tft.color565(60, 200, 230);
  COLOR_YELLOW  = tft.color565(255, 210, 70);
}

void printDegreeC(int x, int y, uint16_t color) {
  tft.drawCircle(x, y + 3, 2, color);
  tft.setTextColor(color);
  tft.setTextSize(1);
  tft.setCursor(x + 6, y);
  tft.print("C");
}

void drawHeader(const char* title, uint16_t accentColor) {
  tft.fillRect(0, 0, tft.width(), 22, accentColor);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(7, 7);
  tft.print(title);

  tft.setCursor(tft.width() - 28, 7);
  tft.print((int)currentScreen + 1);
  tft.print("/");
  tft.print((int)SCREEN_COUNT);
}

void drawCard(int x, int y, int w, int h, const char* label, uint16_t borderColor) {
  tft.fillRoundRect(x, y, w, h, 5, COLOR_CARD);
  tft.drawRoundRect(x, y, w, h, 5, borderColor);

  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(x + 6, y + 5);
  tft.print(label);
}

void drawStatusBar() {
  int y = tft.height() - 14;

  tft.fillRect(0, y, tft.width(), 14, tft.color565(14, 16, 22));

  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(5, y + 3);
  tft.print(statusLine);
}

void setStatus(const char* status) {
  strncpy(statusLine, status, sizeof(statusLine) - 1);
  statusLine[sizeof(statusLine) - 1] = '\0';

  if (!showingTimeScreen && !showingInfoScreen) {
    drawStatusBar();
  }
}

void showMessage(const char* line1, const char* line2, const char* line3 = "") {
  tft.fillScreen(COLOR_BG);
  drawHeader("Flat Climate", COLOR_BLUE);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);
  tft.setCursor(8, 38);
  tft.print(line1);

  tft.setTextSize(1);
  tft.setCursor(8, 70);
  tft.print(line2);

  tft.setCursor(8, 88);
  tft.print(line3);

  drawStatusBar();
}

void formatTime(char* buffer, size_t size) {
  time_t now = time(nullptr);

  if (now < 100000) {
    snprintf(buffer, size, "--:--");
    return;
  }

  struct tm* t = localtime(&now);
  snprintf(buffer, size, "%02d:%02d", t->tm_hour, t->tm_min);
}

// =====================
// Wi-Fi / NTP
// =====================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");

  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    setStatus("WiFi connected");
  } else {
    Serial.println(" FAILED");
    setStatus("WiFi failed");
  }
}

void syncTime() {
  configTime(4 * 3600, 0, "pool.ntp.org", "time.google.com");

  Serial.print("NTP sync");
  int tries = 0;

  while (time(nullptr) < 100000 && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (time(nullptr) > 100000) {
    Serial.println(" OK");
    setStatus("Time synced");
  } else {
    Serial.println(" FAILED");
    setStatus("Time sync failed");
  }
}

// =====================
// AQI US EPA Standard Calculation
// =====================
int linearInterpolate(float concentration, float cLow, float cHigh, float iLow, float iHigh) {
  return (int)round(((iHigh - iLow) / (cHigh - cLow)) * (concentration - cLow) + iLow);
}

int calculatePM25_AQI(float pm25) {
  if (pm25 <= 12.0)  return linearInterpolate(pm25, 0.0, 12.0, 0, 50);
  if (pm25 <= 35.4)  return linearInterpolate(pm25, 12.1, 35.4, 51, 100);
  if (pm25 <= 55.4)  return linearInterpolate(pm25, 35.5, 55.4, 101, 150);
  if (pm25 <= 150.4) return linearInterpolate(pm25, 55.5, 150.4, 151, 200);
  if (pm25 <= 250.4) return linearInterpolate(pm25, 150.5, 250.4, 201, 300);
  if (pm25 <= 500.4) return linearInterpolate(pm25, 250.5, 500.4, 301, 500);
  return 500;
}

int calculatePM10_AQI(float pm10) {
  if (pm10 <= 54)    return linearInterpolate(pm10, 0, 54, 0, 50);
  if (pm10 <= 154)   return linearInterpolate(pm10, 55, 154, 51, 100);
  if (pm10 <= 254)   return linearInterpolate(pm10, 155, 254, 101, 150);
  if (pm10 <= 354)   return linearInterpolate(pm10, 255, 354, 151, 200);
  if (pm10 <= 424)   return linearInterpolate(pm10, 355, 424, 201, 300);
  if (pm10 <= 604)   return linearInterpolate(pm10, 425, 604, 301, 500);
  return 500;
}

int calculateEPA_AQI(float pm25, float pm10) {
  int aqiPm25 = calculatePM25_AQI(pm25);
  int aqiPm10 = calculatePM10_AQI(pm10);
  return max(aqiPm25, aqiPm10);
}

// =====================
// Supabase
// =====================
void sendToSupabase(float temperature, float humidity, float pressureMmHg) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) return;
  }

  HTTPClient http;
  String url = String(SUPABASE_URL) + "/flat_climate_metrics";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);
  http.addHeader("Prefer", "return=minimal");

  char json[160];
  snprintf(
    json,
    sizeof(json),
    "{\"temperature\":%.1f,\"pressure\":%.1f,\"humidity\":%.1f}",
    temperature,
    pressureMmHg,
    humidity
  );

  int code = http.POST(json);
  Serial.print("Supabase send: ");
  Serial.println(code);
  http.end();

  if (code >= 200 && code < 300) {
    char timeBuf[8];
    formatTime(timeBuf, sizeof(timeBuf));

    char status[40];
    snprintf(status, sizeof(status), "Indoor sent %s", timeBuf);
    setStatus(status);
  } else {
    setStatus("Indoor send failed");
  }
}

void fetchWeatherFromSupabase() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) return;
  }

  Serial.println("Starting Supabase GET request...");
  HTTPClient http;
  String url = String(SUPABASE_URL) +
    "/weather_logs?select=temperature,humidity,pressure,pm2_5,pm10_0&order=id.desc&limit=1";

  http.begin(url);
  http.addHeader("apikey", SUPABASE_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);

  int code = http.GET();
  Serial.print("HTTP Response Code: ");
  Serial.println(code);

  if (code == 200) {
    Serial.println("Request success. Streaming stream pointer...");
    WiFiClient* stream = http.getStreamPtr();

    Serial.println("Allocating DynamicJsonDocument...");
    DynamicJsonDocument doc(3072);
    
    Serial.println("Deserializing JSON stream...");
    DeserializationError err = deserializeJson(doc, *stream);

    if (!err && doc.is<JsonArray>() && doc.size() > 0) {
      Serial.println("JSON parse OK. Parsing fields...");
      JsonObject firstRow = doc[0];
      
      outTemp     = firstRow["temperature"].as<float>();
      outHumidity = firstRow["humidity"].as<float>();
      outPressure = firstRow["pressure"].as<float>();
      outPm25     = firstRow["pm2_5"].as<float>();
      outPm10     = firstRow["pm10_0"].as<float>();
      outDataValid = true;

      aqiValue = calculateEPA_AQI(outPm25, outPm10);
      aqiDataValid = true;

      Serial.printf(
        "Outdoor: %.1f C, %.1f mmHg, %.1f %%, PM2.5: %.1f, PM10: %.1f -> Calc AQI: %d\n",
        outTemp, outPressure, outHumidity, outPm25, outPm10, aqiValue
      );

      char timeBuf[8];
      formatTime(timeBuf, sizeof(timeBuf));

      char status[40];
      snprintf(status, sizeof(status), "Metrics updated %s", timeBuf);
      setStatus(status);

      if ((currentScreen == SCREEN_OUTDOOR || currentScreen == SCREEN_AQI) && !showingTimeScreen && !showingInfoScreen) {
        drawCurrentScreen();
      }
    } else {
      Serial.print("Weather parse error: ");
      Serial.println(err.c_str());
      setStatus("Outdoor parse failed");
    }
  } else {
    Serial.print("Weather fetch error: ");
    Serial.println(code);
    setStatus("Outdoor fetch failed");
  }

  http.end();
  Serial.println("HTTP Client execution finished.");
}

// =====================
// Input (Сквозное листание)
// =====================
void handleButton() {
  bool reading = digitalRead(JOY_SW_PIN);
  unsigned long now = millis();

  if (reading == LOW && lastButtonState == HIGH) {
    if (now - lastButtonTime > BUTTON_DEBOUNCE) {
      showingTimeScreen = true;
      showingInfoScreen = false;
      timeScreenStart = now;
      lastButtonTime = now;

      Serial.println("Button -> time screen");
      drawTimeScreen();
    }
  }

  lastButtonState = reading;
}

void handleJoystick() {
  int x = analogRead(JOY_X_PIN);
  int y = analogRead(JOY_Y_PIN);

  unsigned long now = millis();

  bool centeredX = x > JOY_LEFT_THRESHOLD && x < JOY_RIGHT_THRESHOLD;
  bool centeredY = y > JOY_LEFT_THRESHOLD && y < JOY_RIGHT_THRESHOLD;

  if (centeredX && centeredY) {
    joyWasCentered = true;
    return;
  }

  if (!joyWasCentered) return;
  if (now - lastJoyTime < JOY_DEBOUNCE) return;

  bool screenChanged = false;

  if (y >= JOY_DOWN_THRESHOLD) {
    showingInfoScreen = true;
    showingTimeScreen = false;
    infoScreenStart = now;

    Serial.println("DOWN -> info screen");
    drawInfoScreen();

    joyWasCentered = false;
    lastJoyTime = now;
    return;
  }

  // Реализация сквозного циклического переключения экранов
  if (x <= JOY_LEFT_THRESHOLD) {
    currentScreen = (Screen)(((int)currentScreen - 1 + SCREEN_COUNT) % SCREEN_COUNT);
    screenChanged = true;
    Serial.print("LEFT -> screen ");
    Serial.println((int)currentScreen);
  }

  if (x >= JOY_RIGHT_THRESHOLD) {
    currentScreen = (Screen)(((int)currentScreen + 1) % SCREEN_COUNT);
    screenChanged = true;
    Serial.print("RIGHT -> screen ");
    Serial.println((int)currentScreen);
  }

  joyWasCentered = false;
  lastJoyTime = now;

  if (screenChanged) {
    showingTimeScreen = false;
    showingInfoScreen = false;
    drawCurrentScreen();
  }
}

// =====================
// Drawing
// =====================
void drawClimateScreen(
  const char* title,
  float temperature,
  float pressureMmHg,
  float humidity,
  bool outdoor
) {
  tft.fillScreen(COLOR_BG);

  uint16_t accent = outdoor ? COLOR_ORANGE : COLOR_BLUE;
  drawHeader(title, accent);

  // Temperature card
  drawCard(6, 30, 72, 50, "TEMP", accent);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(3);
  tft.setCursor(14, 48);
  tft.print((int)round(temperature));

  printDegreeC(54, 50, COLOR_TEXT);

  // Humidity card
  drawCard(84, 30, 70, 50, "HUM", COLOR_CYAN);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(3);
  tft.setCursor(94, 48);
  tft.print((int)round(humidity));

  tft.setTextSize(2);
  tft.setCursor(132, 54);
  tft.print("%");

  // Pressure card
  drawCard(6, 86, 148, 28, "PRESSURE", COLOR_GREEN);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);
  tft.setCursor(80, 94);
  tft.print((int)round(pressureMmHg));

  tft.setTextColor(COLOR_MUTED);
  tft.setTextSize(1);
  tft.setCursor(122, 100);
  tft.print("mmHg");

  drawStatusBar();
}

const char* getAqiLevel(int value) {
  if (value <= 50)  return "Good";
  if (value <= 100) return "Moderate";
  if (value <= 150) return "Unhealthy S";
  if (value <= 200) return "Unhealthy";
  if (value <= 300) return "Very bad";
  return "Hazardous";
}

uint16_t getAqiColor(int value) {
  if (value <= 50)  return COLOR_GREEN;
  if (value <= 100) return COLOR_YELLOW;
  if (value <= 150) return COLOR_ORANGE;
  if (value <= 200) return COLOR_RED;
  if (value <= 300) return COLOR_MAGENTA;
  return COLOR_RED;
}

const char* getAqiAdvice(int value) {
  if (value <= 50)  return "Air is clean";
  if (value <= 100) return "Okay outside";
  if (value <= 150) return "Sensitive: care";
  if (value <= 200) return "Limit activity";
  if (value <= 300) return "Stay indoors";
  return "Avoid outdoor";
}

void drawAqiScale(int x, int y, int w, int h, int value) {
  int segmentWidth = w / 6;

  uint16_t colors[6] = {
    COLOR_GREEN,
    COLOR_YELLOW,
    COLOR_ORANGE,
    COLOR_RED,
    COLOR_MAGENTA,
    COLOR_RED
  };

  for (int i = 0; i < 6; i++) {
    tft.fillRect(x + i * segmentWidth, y, segmentWidth - 1, h, colors[i]);
  }

  int markerX = x + map(constrain(value, 0, 300), 0, 300, 0, w - 4);
  tft.fillTriangle(
    markerX,
    y + h + 6,
    markerX + 4,
    y + h + 6,
    markerX + 2,
    y + h + 1,
    COLOR_TEXT
  );
}

void drawAqiScreen() {
  tft.fillScreen(COLOR_BG);
  drawHeader("Calculated AQI", COLOR_MAGENTA);

  if (!aqiDataValid) {
    drawCard(8, 36, 144, 64, "AIR QUALITY", COLOR_MAGENTA);

    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(2);
    tft.setCursor(22, 58);
    tft.print("No Data");

    tft.setTextSize(1);
    tft.setTextColor(COLOR_MUTED);
    tft.setCursor(22, 84);
    tft.print("Waiting DB Sync");

    drawStatusBar();
    return;
  }

  uint16_t aqiColor = getAqiColor(aqiValue);

  drawCard(6, 30, 64, 50, "AQI", aqiColor);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(3);
  tft.setCursor(15, 48);
  tft.print(aqiValue);

  drawCard(76, 30, 78, 50, "LEVEL", aqiColor);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(1);
  tft.setCursor(84, 50);
  tft.print(getAqiLevel(aqiValue));

  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(84, 66);
  tft.print(getAqiAdvice(aqiValue));

  drawAqiScale(8, 92, 144, 8, aqiValue);

  tft.setTextColor(COLOR_MUTED);
  tft.setTextSize(1);
  tft.setCursor(8, 106);
  tft.print("0");

  tft.setCursor(70, 106);
  tft.print("150");

  tft.setCursor(136, 106);
  tft.print("300");

  drawStatusBar();
}

void drawInfoScreen() {
  tft.fillScreen(COLOR_BG);
  drawHeader("System info", COLOR_GREEN);
  drawCard(6, 30, 148, 84, "STATUS", COLOR_GREEN);

  tft.setTextSize(1);

  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(14, 48);
  tft.print("WiFi");
  tft.setTextColor(WiFi.status() == WL_CONNECTED ? COLOR_GREEN : COLOR_RED);
  tft.setCursor(70, 48);
  tft.print(WiFi.status() == WL_CONNECTED ? "connected" : "offline");

  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(14, 64);
  tft.print("BME280");
  tft.setTextColor(bmeReady ? COLOR_GREEN : COLOR_RED);
  tft.setCursor(70, 64);
  tft.print(bmeReady ? "ready" : "error");

  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(14, 80);
  tft.print("Outdoor");
  tft.setTextColor(outDataValid ? COLOR_GREEN : COLOR_RED);
  tft.setCursor(70, 80);
  tft.print(outDataValid ? "loaded" : "no data");

  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(14, 96);
  tft.print("AQI Calc");
  tft.setTextColor(aqiDataValid ? COLOR_GREEN : COLOR_RED);
  tft.setCursor(70, 96);
  tft.print(aqiDataValid ? "ready" : "error");

  drawStatusBar();
}

void drawTimeScreen() {
  tft.fillScreen(COLOR_BG);
  drawHeader("Time", COLOR_RED);

  time_t now = time(nullptr);

  if (now < 100000) {
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(2);
    tft.setCursor(22, 52);
    tft.print("NO TIME");
    tft.setTextSize(1);
    tft.setTextColor(COLOR_MUTED);
    tft.setCursor(28, 80);
    tft.print("NTP not synced");
    drawStatusBar();
    return;
  }

  struct tm* t = localtime(&now);

  char timeBuf[6];
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", t->tm_hour, t->tm_min);

  char dateBuf[12];
  snprintf(
    dateBuf,
    sizeof(dateBuf),
    "%02d.%02d.%04d",
    t->tm_mday,
    t->tm_mon + 1,
    t->tm_year + 1900
  );

  tft.fillRoundRect(18, 36, 124, 64, 5, COLOR_CARD);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED);
  tft.setCursor(24, 41);
  tft.print("LOCAL TIME");

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(4);
  tft.setCursor(28, 56);
  tft.print(timeBuf);

  tft.setTextColor(COLOR_MUTED);
  tft.setTextSize(1);
  tft.setCursor(52, 104);
  tft.print(dateBuf);

  drawStatusBar();
}

void drawNoBmeScreen() {
  tft.fillScreen(COLOR_BG);
  drawHeader("BME280 error", COLOR_RED);
  drawCard(8, 34, 144, 70, "CHECK SENSOR", COLOR_RED);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(1);
  tft.setCursor(18, 56);
  tft.print("SDA -> GPIO 21");
  tft.setCursor(18, 74);
  tft.print("SCL -> GPIO 22");
  tft.setCursor(18, 92);
  tft.print("Try address 0x77");

  drawStatusBar();
}

void drawCurrentScreen() {
  if (!bmeReady && currentScreen == SCREEN_HOME) {
    drawNoBmeScreen();
    return;
  }

  if (currentScreen == SCREEN_HOME) {
    if (filteredTemp < -900) {
      filteredTemp = bme.readTemperature();
      filteredHum  = bme.readHumidity();
      filteredPres = hPaToMmHg(bme.readPressure() / 100.0);
    }

    lastDisplayedTemp     = (int)round(filteredTemp);
    lastDisplayedHumidity = (int)round(filteredHum);
    lastDisplayedPressure = (int)round(filteredPres);

    drawClimateScreen(
      "Home climate",
      filteredTemp,
      filteredPres,
      filteredHum,
      false
    );
  }
  else if (currentScreen == SCREEN_OUTDOOR) {
    if (outDataValid) {
      drawClimateScreen(
        "Outdoor",
        outTemp,
        outPressure,
        outHumidity,
        true
      );
    } else {
      tft.fillScreen(COLOR_BG);
      drawHeader("Outdoor", COLOR_ORANGE);
      drawCard(8, 38, 144, 56, "OUTDOOR WEATHER", COLOR_ORANGE);

      tft.setTextColor(COLOR_TEXT);
      tft.setTextSize(2);
      tft.setCursor(26, 60);
      tft.print("No data");

      tft.setTextColor(COLOR_MUTED);
      tft.setTextSize(1);
      tft.setCursor(24, 84);
      tft.print("Waiting Supabase");

      drawStatusBar();
    }
  }
  else if (currentScreen == SCREEN_AQI) {
    drawAqiScreen();
  }
}

void updateHomeScreenIfNeeded() {
  if (!bmeReady) return;
  if (currentScreen != SCREEN_HOME) return;
  if (showingTimeScreen || showingInfoScreen) return;

  float rawTemp = bme.readTemperature();
  float rawHum  = bme.readHumidity();
  float rawPres = hPaToMmHg(bme.readPressure() / 100.0);

  if (filteredTemp < -900) {
    filteredTemp = rawTemp;
    filteredHum  = rawHum;
    filteredPres = rawPres;
  } else {
    filteredTemp = (rawTemp * EMA_ALPHA) + (filteredTemp * (1.0 - EMA_ALPHA));
    filteredHum  = (rawHum  * EMA_ALPHA) + (filteredHum  * (1.0 - EMA_ALPHA));
    filteredPres = (rawPres * EMA_ALPHA) + (filteredPres * (1.0 - EMA_ALPHA));
  }

  int roundedTemp     = (int)round(filteredTemp);
  int roundedHumidity = (int)round(filteredHum);
  int roundedPressure = (int)round(filteredPres);

  bool changed =
    roundedTemp     != lastDisplayedTemp ||
    roundedHumidity != lastDisplayedHumidity ||
    roundedPressure != lastDisplayedPressure;

  if (!changed) return;

  lastDisplayedTemp     = roundedTemp;
  lastDisplayedHumidity = roundedHumidity;
  lastDisplayedPressure = roundedPressure;

  drawClimateScreen(
    "Home climate",
    filteredTemp,
    filteredPres,
    filteredHum,
    false
  );
}

// =====================
// Setup / loop
// =====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 TFT Climate Station");

  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  pinMode(JOY_X_PIN, INPUT);
  pinMode(JOY_Y_PIN, INPUT);

  analogReadResolution(12);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);

  setupColors();

  tft.fillScreen(COLOR_BG);
  setStatus("Booting");

  showMessage("START", "TFT OK", "Starting BME280");
  delay(800);

  Wire.begin(BME_SDA_PIN, BME_SCL_PIN);
  bmeReady = bme.begin(BME280_ADDRESS, &Wire);

  if (!bmeReady) {
    Serial.println("BME280 not found at 0x76");
    setStatus("BME280 error");
    showMessage("BME ERROR", "Address 0x76", "Try 0x77");
    delay(1500);
  } else {
    Serial.println("BME280 OK");
    setStatus("BME280 ready");
    showMessage("BME280 OK", "Starting WiFi", "");
    delay(800);
  }

  connectWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    showMessage("WiFi OK", WiFi.localIP().toString().c_str(), "Sync time");
    syncTime();
    delay(800);

    fetchWeatherFromSupabase();
  } else {
    showMessage("WiFi FAILED", "Offline mode", "");
    delay(1000);
  }

  drawCurrentScreen();
}

void loop() {
  unsigned long now = millis();

  handleJoystick();
  handleButton();

  if (showingTimeScreen) {
    if (now - timeScreenStart >= TIME_SHOW_MS) {
      showingTimeScreen = false;
      drawCurrentScreen();
    }
    delay(20);
    return;
  }

  if (showingInfoScreen) {
    if (now - infoScreenStart >= INFO_SHOW_MS) {
      showingInfoScreen = false;
      drawCurrentScreen();
    }
    delay(20);
    return;
  }

  if (now - lastSensorCheckTime >= SENSOR_CHECK_INTERVAL) {
    updateHomeScreenIfNeeded();
    lastSensorCheckTime = now;
  }

  if (bmeReady && (now - lastSendTime >= SEND_INTERVAL || lastSendTime == 0)) {
    float t = (filteredTemp > -900) ? filteredTemp : bme.readTemperature();
    float h = (filteredHum > -900)  ? filteredHum  : bme.readHumidity();
    float p = (filteredPres > -900) ? filteredPres : hPaToMmHg(bme.readPressure() / 100.0);

    sendToSupabase(t, h, p);
    lastSendTime = now;
  }

  if (now - lastFetchTime >= FETCH_INTERVAL || lastFetchTime == 0) {
    fetchWeatherFromSupabase();
    lastFetchTime = now;
  }

  delay(50);
}