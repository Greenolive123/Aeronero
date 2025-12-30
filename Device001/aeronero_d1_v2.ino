///======== INCLUDES AND DEFINES SECTION =======//
// Essential libraries and global defines for hardware and protocols
#include "esp_task_wdt.h"
#include <Wire.h>
#include "esp_system.h"
#include "SHTSensor.h"
#include <DFRobot_ENS160.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <HTTPClient.h>
#include <Update.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <PubSubClient.h>
#include "secrets.h"
#include <Preferences.h>
#define MIN_VALID_PULSES 3

#define TDS_CAL_ULTRA_LOW  2.54f   // <20 ppm correction
#define ULTRASONIC_FAIL_THRESHOLD 5
// PH
// ==== USER CALIBRATION VALUES FROM YOUR SENSOR ====
float intercept = 17.5053;  // from your extracted values
float V_pH4 = 1884.30;      // mV
float V_pH7 = 1412.75;      // mV
float V_pH10 = 1060.47;     // mV
// ESP32 ADC characteristics
#define adcMax 4095
#define Vref 3300.0
///======== GLOBAL VARIABLES AND STRUCTS SECTION =======//
// Communication and failure tracking
int gsmFailureCount = 0;
const int GSM_MAX_FAILURES = 3;
#define MQTT_MAX_PACKET_SIZE 1024
#define CA_CERT_FILE "ca.pem"
#define CLIENT_CERT_FILE "client.pem"
#define CLIENT_KEY_FILE "private.pem"
#define AWS_IOT_PUBLISH_TOPIC "device/" THINGNAME "/data"
#define AWS_IOT_SUBSCRIBE_TOPIC "device/" THINGNAME "/sub"
// Sensor data structures for buffering and averaging
struct SensorSample {
  float temp;
  float hum;
  uint8_t AQI;
  uint16_t TVOC;
  uint16_t ECO2;
  int count;  // how many samples in the buffer
};
// Global buffers for sensor averaging
SensorSample bufferReady = { 0 };
volatile bool bufferSwap = false;  // flag: new averages ready
// Accumulators for averaging
float avgTemp = 0, avgHum = 0, avgAQI = 0, avgTVOC = 0, avgECO2 = 0;
int sampleCount = 0;
// Mutex for safe updates
SemaphoreHandle_t sensorMutex;
Preferences prefs;
// Runtime counters for compressor, motor, and flow
unsigned long compStartTime = 0;      // when compressor started
unsigned long compressorMinutes = 0;  // cumulative minutes
unsigned long motorMinutes = 0;       // cumulative minutes
unsigned long flowStartTime = 0;
unsigned long flowDurationMinutes = 0;
// Timing and logging constants
#define HOLD_TIME 5000
#define LOG_LEVEL_DEBUG 0
#define LOG_LEVEL_INFO 1
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_ERROR 3
#define MIN_LOG_LEVEL LOG_LEVEL_ERROR  // Optimized: Only log errors and above for reduced verbosity
#define ENABLE_MQTT_LOGGING true
bool configSaved = false;

unsigned long lastClientTime = 0;
int mqttFailureCount = 0;
bool firstRun = true;
bool isMQTTConnected = false;
bool loggedIn = false;  // Authentication flag

// ===== FLOW SENSOR CONFIG =====
#define FLOW_DIVISOR 7.5f        // Hz → L/min (YF-S201)
#define PULSES_PER_LITER 450.0f  // adjust per sensor
#define MIN_VALID_PULSES 3       // pulses/sec to confirm flow
#define FLOW_CONFIRM_SEC 2       // seconds to confirm flow
volatile uint32_t pulseCount = 0;
uint8_t validFlowCount = 0;
// = 3,281 pulses/liter
float CALIBRATION_OFFSET = 0.0;
// NTP Server settings
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;  // IST (+5:30)
const int daylightOffset_sec = 0;
struct tm timeinfo;
unsigned long lastKeepAliveTime = 0;
const unsigned long keepAliveInterval = 60000;  // 1 minute for keep-alive
// Serial and pin definitions
HardwareSerial SerialGSM(2);
unsigned long flowStopDelay = 5000;
static unsigned long lastFlowTime = 0;
float sessionLiters = 0.0f;  // counts only this flow session
// Configuration struct for device settings
struct Config {
  char deviceId[32];
  char type[10];
  char wifi_ssid[32];
  char wifi_password[32];
  char gsm_username[32];
  char gsm_password[32];
  char gsm_apn[32];
  float tempThresholdMin;
  float tempThresholdMax;
  float humidityThresholdMin;
  float humidityThresholdMax;
  float compressorOnLevel;
  float compressorOffLevel;
  float pumpOffLevel;
  int publishInterval;
  // 🔥 Add OTA params
  char last_ota_ssid[32];
  char last_ota_password[32];
  char last_ota_link[128];
} config;
// Server and client objects
WebServer server(80);
WiFiClientSecure net;
PubSubClient client(net);
HardwareSerial sensorSerial(1);
// MQTT and pin constants
#define MQTT_CLIENT_ID THINGNAME
#define RXD2 47
#define TXD2 48
#define SDA_PIN 8
#define SCL_PIN 9
const int WIFI_LED = 37;
const int BUZZER_PIN = 45;
const int dataStatus = 15;
const int tdsPin = 2;
const int RESET_PIN = 36;  //
const int flowSensorPin = 5;
#define PH_PIN 1  //device001
#define ADC_RESOLUTION 4096
#define VREF 3300
#define V_NEUTRAL 1500
#define SLOPE -59.7
#define DISCONNECT_THRESHOLD 50


#define TDS_SAMPLES 40
#define TDS_SPIKE_LIMIT 150.0
#define TDS_SLEW_RATE 20.0



// Flow and flag variables
volatile bool flowFlag = false;

// Logging macros (optimized to only log errors)
#define LOG(level, fmt, ...) \
  do { \
    if (level >= MIN_LOG_LEVEL) { \
      logMessage(level, fmt, ##__VA_ARGS__); \
    } \
  } while (0)
#define LOG_DEBUG(fmt, ...) LOG(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) LOG(LOG_LEVEL_INFO, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) LOG(LOG_LEVEL_WARN, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) LOG(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)
// Circular log buffer for diagnostics
#define MAX_LOGS 10
String serialLogs[MAX_LOGS];
int logIndex = 0;
float temperatureC = 25.0;
const int SAMPLES = 100;
const int SAMPLE_DELAY_MS = 20;
// Preferences mutex for thread safety
SemaphoreHandle_t prefsMutex;
// Global runtime variables
unsigned long totalFlowMinutes = 0;
uint8_t status;
#define DEFAULT_AQI_WARMUP 1  // Good air quality during warm-up


#define TEMP_REF 25.0  // °C reference for compensation
float phvoltage = 0.0;
float tdsValue = 0.0;
float phValue = 0.0;
bool isGSM = false;
bool isWIFI = false;
uint8_t buffer[4];
float distance = 0;
SemaphoreHandle_t ultrasonicMutex;
unsigned long lastUltrasonicRead = 0;
// Task handles for FreeRTOS scheduling
TaskHandle_t sendDataTaskHandle;
TaskHandle_t checkRelayTaskHandle;
TaskHandle_t checkConnectivityTaskHandle;
TaskHandle_t buttonCheckTaskHandle;
TaskHandle_t flowSensorTaskHandle;
TaskHandle_t phTaskHandle = NULL;

// TaskHandle_t sensorAvgTaskHandle;
// ADC and timing globals
const float vRef = 3.3;
const float adcResolution = 4095;
unsigned long lastPublishTime = 0;
#define gsmRX 17
#define gsmTX 18
#define gsmRST 14
// #define gsmRST 48
bool shtInitialized = false;
bool ensInitialized = false;
volatile unsigned long lastInterruptTime = 0;

const unsigned long DEBOUNCE_DELAY = 5;  // ms

const float tdsFactor = 0.5;


unsigned long lastRelayOffTime = 0;
float flowRate;
float totalLiters = 0.0;
const float calibrationFactor = 450.0;
#define RELAY1_PIN 21
#define RELAY2_PIN 38
// Sensor objects and globals
SHTSensor sht;
float temperature = NAN, humidity = NAN;
uint8_t AQI = 0;
uint16_t TVOC = 0, ECO2 = 0;
bool isENS = true;
DFRobot_ENS160_I2C ens160(&Wire, 0x53);
const char* current_firmware_version = "2.0.8";
///======== FORWARD DECLARATIONS SECTION =======//
// Forward declarations for functions to ensure compilation order
float readCounter(const char* key, float defaultValue);
unsigned long readCounterULong(const char* key, unsigned long defaultValue);
void saveCounter(const char* key, float value);
void saveCounterULong(const char* key, unsigned long value);
void logMessage(uint8_t level, const char* format, ...);
// === ORIGINAL WORKING CALIBRATION (ADC-BASED) ===
float neutralADC = 1925;  // pH 7
float acidADC = 2269;     // pH 4


#define PH_SAMPLE_INTERVAL_MS 100
#define PH_TOTAL_TIME_MS 60000
#define PH_MAX_SAMPLES 100

#define PH_SPIKE_LIMIT 0.5f   // was too low before
#define PH_STABLE_RANGE 0.6f  // flowing water tolerance
#define PH_SLEW_RATE 0.2f


float constrainFloat(float x, float minVal, float maxVal) {
  if (x < minVal) return minVal;
  if (x > maxVal) return maxVal;
  return x;
}

void sortArray(float* arr, int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (arr[j] < arr[i]) {
        float t = arr[i];
        arr[i] = arr[j];
        arr[j] = t;
      }
    }
  }
}

float medianOf(float* arr, int n) {
  sortArray(arr, n);
  return (n % 2 == 0)
           ? (arr[n / 2 - 1] + arr[n / 2]) / 2.0
           : arr[n / 2];
}




///======== UTILITY FUNCTIONS SECTION =======//
// pH slope calculation for diagnostics
float slope25() {
  return (7.0 - 4.0) / (neutralADC - acidADC);
}
// Logging function (optimized: only errors are verbose, success implied)
void logMessage(uint8_t level, const char* format, ...) {
  const char* levelStr;
  switch (level) {
    case LOG_LEVEL_DEBUG: levelStr = "DEBUG"; break;
    case LOG_LEVEL_INFO: levelStr = "INFO"; break;
    case LOG_LEVEL_WARN: levelStr = "WARN"; break;
    case LOG_LEVEL_ERROR: levelStr = "ERROR"; break;
    default: levelStr = "UNKNOWN"; break;
  }
  char logBuffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(logBuffer, sizeof(logBuffer), format, args);
  va_end(args);
  Serial.printf("%s: %s\n", levelStr, logBuffer);
  // Store in circular buffer for diagnostics
  serialLogs[logIndex] = String("[") + levelStr + "] " + logBuffer;
  logIndex = (logIndex + 1) % MAX_LOGS;
}
///======== AT COMMAND SECTION =======//
// AT command sender (optimized: timeout with partial response capture)
String sendATCommandGetResponse(String command, unsigned long timeout = 10000) {
  String response = "";
  Serial.print("Sending: ");
  Serial.println(command);
  SerialGSM.println(command);
  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (SerialGSM.available()) {
      char c = SerialGSM.read();
      response += c;
      Serial.print(c);
    }
    if (response.endsWith("OK\r\n") || response.endsWith("ERROR\r\n")) {
      break;
    }
  }
  return response;
}
// Preferences read/write functions with mutex for thread-safety (optimized for atomicity)
void saveCounterULong(const char* key, unsigned long value) {
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("counters", false);  // write mode
    prefs.putULong(key, value);
    prefs.end();
    xSemaphoreGive(prefsMutex);
    // Success: Counter saved atomically (no log for success per request)
  }
}
void saveCounter(const char* key, float value) {
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("counters", false);  // write mode
    prefs.putFloat(key, value);
    prefs.end();
    xSemaphoreGive(prefsMutex);
    // Success: Float counter saved (no log)
  }
}
float readCounter(const char* key, float defaultValue = 0.0f) {
  float value = defaultValue;
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("counters", true);  // read mode
    value = prefs.getFloat(key, defaultValue);
    prefs.end();
    xSemaphoreGive(prefsMutex);
    // Success: Value read from NVS (no log)
  }
  return value;
}
unsigned long readCounterULong(const char* key, unsigned long defaultValue = 0) {
  unsigned long value = defaultValue;
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("counters", true);  // read mode
    value = prefs.getULong(key, defaultValue);
    prefs.end();
    xSemaphoreGive(prefsMutex);
    // Success: ULong read from NVS (no log)
  }
  return value;
}
// Reads single pH with your calibration math
// Reads single pH with calibration math (safe & clamped)
float calculatePH(int adcValue) {

  float voltage_mV = (adcValue * Vref) / adcMax;
  float pH;

  // Slopes
  float slopeAcid = (7.0 - 4.0) / (V_pH7 - V_pH4);    // pH 4 → 7
  float slopeBase = (10.0 - 7.0) / (V_pH10 - V_pH7);  // pH 7 → 10

  // NOTE: Most pH probes have HIGHER voltage at LOW pH
  if (voltage_mV >= V_pH7) {
    pH = 7.0 + (voltage_mV - V_pH7) * slopeAcid;
  } else {
    pH = 7.0 + (voltage_mV - V_pH7) * slopeBase;
  }

  pH += (intercept / 1000.0);

  // Hard clamp (prevents garbage)
  if (pH < 0.0 || pH > 14.0) return NAN;

  Serial.printf(
    "PH | ADC: %d | mV: %.2f | pH: %.2f\n",
    adcValue, voltage_mV, pH);

  return pH;
}

// ===== 60-Second Exact Sampling =====
float stableReadPH() {

  float samples[PH_MAX_SAMPLES];
  int count = 0;

  float lastPH = NAN;
  unsigned long start = millis();

  while ((millis() - start) < PH_TOTAL_TIME_MS && count < PH_MAX_SAMPLES) {

    int adc = analogRead(PH_PIN);
    float ph = calculatePH(adc);

    if (!isnan(ph)) {

      // Relaxed spike rejection (IMPORTANT)
      if (isnan(lastPH) || fabs(ph - lastPH) <= PH_SPIKE_LIMIT) {
        samples[count++] = ph;
        lastPH = ph;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(PH_SAMPLE_INTERVAL_MS));
  }

  // Minimum samples (relaxed)
  if (count < 6) {
    LOG_ERROR("pH insufficient samples: %d", count);
    return NAN;
  }

  // Range analysis (DO NOT hard fail)
  float minV = samples[0], maxV = samples[0];
  for (int i = 1; i < count; i++) {
    if (samples[i] < minV) minV = samples[i];
    if (samples[i] > maxV) maxV = samples[i];
  }

  float range = maxV - minV;
  if (range > PH_STABLE_RANGE) {
    Serial.printf(
      "pH semi-stable (range %.2f) → accepting median\n",
      range);
  }

  // Median = best noise immunity
  float medianPH = medianOf(samples, count);

  // Slew-rate limited smoothing
  static float smoothPH = medianPH;
  smoothPH += constrainFloat(
    medianPH - smoothPH,
    -PH_SLEW_RATE,
    PH_SLEW_RATE);

  Serial.printf(
    "pH | Samples: %d | Min: %.2f | Max: %.2f | Median: %.2f | Output: %.2f\n",
    count, minV, maxV, medianPH, smoothPH);

  return smoothPH;
}


// ===== Range-wise calibration factors =====
const float TDS_CAL_RANGE[7] = {
  1.569,  // r0 < 50
  1.006,  // r1 50–100
  1.069,  // r2 100–200
  1.230,  // r3 200–300
  1.333,  // r4 300–600
  1.461,  // r5 600–1000
  1.750   // r6 >1000
};

// ===== Range detection =====
int detectTDSRange(float ppm) {
  if (ppm < 50) return 0;
  if (ppm < 100) return 1;
  if (ppm < 200) return 2;
  if (ppm < 300) return 3;
  if (ppm < 600) return 4;
  if (ppm < 1000) return 5;
  return 6;
}

// TDS reading with temperature compensation + range calibration + averaging
void readTDSSensor(float temperature) {

  float values[TDS_SAMPLES];
  int count = 0;
  float last = NAN;

  for (int i = 0; i < TDS_SAMPLES; i++) {

    int analogValue = analogRead(tdsPin);
    float voltage = analogValue * 3.3 / 4095.0;

    // Temperature compensation
    float comp = 1.0 + 0.02 * (temperature - 25.0);
    float compV = voltage / comp;

    // ---- RAW TDS ONLY (NO CALIBRATION HERE) ----
    float rawTDS =
      (133.42 * pow(compV, 3)
       - 255.86 * pow(compV, 2)
       + 857.39 * compV)
      * tdsFactor;

    // Spike rejection
    if (isnan(last) || fabs(rawTDS - last) <= TDS_SPIKE_LIMIT) {
      values[count++] = rawTDS;
      last = rawTDS;
    }

    vTaskDelay(25 / portTICK_PERIOD_MS);
  }

  if (count < 5) {
    LOG_ERROR("TDS insufficient samples");
    return;
  }

  // Median filter
  float medianRaw = medianOf(values, count);

  // Slew-rate smoothing (RAW domain)
  static float smoothRaw = medianRaw;
  smoothRaw += constrainFloat(
    medianRaw - smoothRaw,
    -TDS_SLEW_RATE,
    TDS_SLEW_RATE);

  // ---- APPLY RANGE-WISE CALIBRATION ----

  // ---- APPLY RANGE-WISE CALIBRATION ----
  float calibratedTDS;

  int range;
  if (smoothRaw < 20.0f) {
    calibratedTDS = smoothRaw * TDS_CAL_ULTRA_LOW;
  } else {
     range = detectTDSRange(smoothRaw);
    calibratedTDS = smoothRaw * TDS_CAL_RANGE[range];
  }
  // Final sanity check
  if (calibratedTDS >= 0 && calibratedTDS <= 2000) {
    tdsValue = calibratedTDS;
    saveCounter("tdsValue", tdsValue);
  }

  Serial.printf(
    "TDS | Samples:%d | RAW:%.1f | CAL:%.1f | r%d\n",
    count, smoothRaw, calibratedTDS, range);
}


///======== PH/TDS MEASUREMENT TASK SECTION =======//
// Main task for pH + TDS with flow validation (optimized: self-deleting task for resource efficiency)
void phTdsTask(void* pvParameters) {
  unsigned long startTime = millis();
  // Diagnostic: Task started with flow check
  if (!flowFlag) {
    LOG_ERROR("Flow stopped before 3s → skipping pH/TDS read");
    vTaskDelete(NULL);
    return;
  }
  // Continuous flow check for 5s
  while (millis() - startTime < 5000) {
    if (!flowFlag) {
      LOG_ERROR("Flow ended before 5s → skipping pH/TDS read");
      vTaskDelete(NULL);
      return;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  // Success: Continuous flow detected (no log)
  vTaskDelay(10000 / portTICK_PERIOD_MS);  // Stabilize in flowing water
  float localTDS = NAN;
  float localPH = NAN;
  // Retry TDS up to 3 times
  for (int i = 0; i < 3; i++) {
    readTDSSensor(25.0);
    if (tdsValue >= 0 && tdsValue <= 2000) {
      localTDS = tdsValue;
      break;  // Success: Valid TDS (no log)
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  // Stabilized pH
  localPH = stableReadPH();
  // Update globals only if valid
  if (!isnan(localTDS)) {
    tdsValue = localTDS;
    saveCounter("tdsValue", tdsValue);
    // Success: Valid TDS stored (no log)
  } else {
    saveCounter("tdsValue", NAN);
    LOG_ERROR("TDS invalid after retries → stored NULL");
  }
  if (!isnan(localPH)) {
    phValue = localPH;
    saveCounter("phValue", phValue);
    // Success: Valid pH stored (no log)
  } else {
    saveCounter("phValue", NAN);
    LOG_ERROR("pH invalid after stabilization → stored NULL");
  }
  // Success: Task finished (no log)

  phTaskHandle = NULL;
  vTaskDelete(NULL);
}
///======== FLOW SENSOR SECTION =======//
// Flow sensor task with pulse counting and volume tracking (optimized: interrupt-driven for low CPU)
void IRAM_ATTR countPulses() {
  static uint32_t lastMicros = 0;
  uint32_t now = micros();

  if (now - lastMicros > 500) {  // 500 µs debounce
    pulseCount++;
    lastMicros = now;
  }
}
void flowSensorTask(void* pvParameters) {

  // Restore counters on boot
  totalLiters = readCounter("totalLiters", 0.0f);
  motorMinutes = readCounterULong("motorMinutes", 0);

  while (true) {

    /* ===== 1 SECOND MEASUREMENT WINDOW ===== */
    noInterrupts();
    uint32_t pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    /* ===== FLOW RATE ===== */
    flowRate = pulses / FLOW_DIVISOR;

    Serial.printf(
      "Flow Sensor - Pulses: %lu, Flow Rate: %.2f L/min\n",
      pulses, flowRate);

    /* ===== VALID FLOW CHECK ===== */
    if (pulses >= MIN_VALID_PULSES) {
      validFlowCount++;
    } else {
      validFlowCount = 0;
    }

    /* ===== SESSION VOLUME ===== */
    if (flowFlag && pulses >= MIN_VALID_PULSES) {
      float volumeIncrement = pulses / PULSES_PER_LITER;
      sessionLiters += volumeIncrement;

      Serial.printf(
        "Flow Sensor - Session Liters: %.3f\n",
        sessionLiters);
    }

    /* ===== FLOW START (CONFIRMED) ===== */
    if (validFlowCount >= FLOW_CONFIRM_SEC) {

      lastFlowTime = millis();

      if (!flowFlag) {
        flowFlag = true;
        sessionLiters = 0.0f;
        flowStartTime = millis();
        Serial.println("Flow Sensor - CONFIRMED flow, relay ON");

        if (phTaskHandle == NULL) {
          BaseType_t res = xTaskCreatePinnedToCore(
            phTdsTask,
            "phTdsTask",
            10240,
            NULL,
            1,
            &phTaskHandle,
            1);

          if (res != pdPASS) {
            LOG_ERROR("Failed to create phTdsTask");
            phTaskHandle = NULL;
          }
        }
      }
    }

    /* ===== FLOW STOP ===== */
    if (flowFlag && pulses == 0 && (millis() - lastFlowTime >= flowStopDelay)) {

      flowFlag = false;
      validFlowCount = 0;


      unsigned long flowDurationMs = millis() - flowStartTime;
      unsigned long flowDurationMinutes = flowDurationMs / 60000;

      Serial.printf(
        "Flow Sensor - Flow stopped after %lu ms (%lu minutes)\n",
        flowDurationMs,
        flowDurationMinutes);

      /* ===== SAVE COUNTERS ===== */
      if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {

        motorMinutes += flowDurationMinutes;
        totalLiters += sessionLiters;

        prefs.begin("counters", false);
        prefs.putFloat("totalLiters", totalLiters);
        prefs.putULong("motorMinutes", motorMinutes);
        prefs.end();

        xSemaphoreGive(prefsMutex);

        Serial.printf(
          "Flow Sensor - Total Liters: %.3f, Motor Minutes: %lu\n",
          totalLiters,
          motorMinutes);
      }

      sessionLiters = 0.0f;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


///======== SENSOR INITIALIZATION SECTION =======//
// Sensor init with retries (optimized: exponential backoff implied in delays)
bool initSensors() {
  // SHT20 init
  Serial.println("Initializing SHT Sensor...");
  if (sht.init()) {
    sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
    shtInitialized = true;
    LOG_ERROR("SHT20 initialization Done");
    // Success: SHT20 initialized (no log)
  } else {
    LOG_ERROR("SHT20 initialization failed");
    shtInitialized = false;
  }
  // ENS160 init with 5 retries
  Serial.println("Initializing ENS160 Sensor...");
  int attempt = 0;
  while (attempt < 5) {
    if (NO_ERR == ens160.begin()) {
      ensInitialized = true;
      ens160.setPWRMode(ENS160_STANDARD_MODE);
      delay(5000);

      LOG_ERROR("ENS initialization Done");
      Serial.printf("ENS160 Sensor - Initialized successfully on attempt %d\n", attempt + 1);
      // Success: ENS160 initialized (no log)
      break;
    } else {
      LOG_ERROR("ENS160 communication failed, attempt %d/5", attempt + 1);
      attempt++;
      delay(3000);
    }
  }
  if (!ensInitialized) {
    LOG_ERROR("Failed to initialize ENS160 after 5 attempts");
    isENS = false;
  }
  Serial.printf("Sensor Init Summary - SHT: %s, ENS: %s\n", shtInitialized ? "OK" : "FAIL", ensInitialized ? "OK" : "FAIL");
  return shtInitialized && ensInitialized;  // Success: Sensors ready (no log if true)
}
///======== ENS160 SENSOR SECTION =======//
// Returns true if ENS160 produced usable data (valid or warm-up), false otherwise
bool readENS160Sensor(uint8_t& outAQI, uint16_t& outTVOC, uint16_t& outECO2) {
  int status = ens160.getENS160Status();
  Serial.printf("ENS160 status: %d\n", status);

  // VALID
  if (status == 1) {
    isENS = true;
    outAQI = ens160.getAQI();
    outTVOC = ens160.getTVOC();
    outECO2 = ens160.getECO2();
    return true;
  }

  // WARM-UP or NOT READY → NORMAL
  if (status == 0 || status == 2) {
    isENS = true;
    outAQI = DEFAULT_AQI_WARMUP;
    outTVOC = ens160.getTVOC();
    outECO2 = ens160.getECO2();
    return true;
  }

  // status 3 → invalid data, NOT bus failure
  isENS = false;
  return false;
}


///======== LATEST READINGS SECTION =======//
// Get latest sensor readings with reinitialization fallbacks (optimized: lazy reinits)
///======== LATEST READINGS SECTION (HARDENED VERSION) =======//
// Bulletproof sensor reading with retries + forced reinit + caching
bool getLatestReadings(float& temperature, float& humidity,
                       uint8_t& aqi, uint16_t& tvoc, uint16_t& eco2) {
  // Serial.println("Starting sensor readings cycle...");

  // ===== 1. READ SHT20 WITH RETRIES =====
  bool shtOK = false;
  float t = NAN, h = NAN;

  if (!shtInitialized) {
    LOG_ERROR("SHT20 not initialized → reinitializing...");
    shtInitialized = sht.init();
    if (shtInitialized) {
      sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
      delay(50);
    } else {
      LOG_ERROR("SHT20 init FAILED → Attempting I2C Recovery");
      recoverI2C();  // <-- ADD THIS
      shtInitialized = false;
    }
  }

  if (shtInitialized) {
    for (int i = 0; i < 3; i++) {
      if (sht.readSample()) {
        t = sht.getTemperature();
        h = sht.getHumidity();
        shtOK = true;
        break;
      }
      delay(40);
    }
  }

  if (!shtOK) {
    LOG_ERROR("SHT20 read FAILED after 3 retries → Calling Recovery");
    recoverI2C();  // <-- ADD THIS
    shtInitialized = false;
    temperature = humidity = NAN;
  } else {
    temperature = t;
    humidity = h;
    // Serial.printf("✅ SHT20 - Temp: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);
  }

  // ===== 2. ENS160 INITIALIZATION =====
  if (!ensInitialized) {
    LOG_ERROR("ENS160 not initialized → reinitializing...");
    if (NO_ERR == ens160.begin()) {
      ens160.setPWRMode(ENS160_STANDARD_MODE);
      delay(1500);
      ensInitialized = true;
    } else {
      LOG_ERROR("ENS160 init FAILED → Attempting I2C Recovery");
      recoverI2C();  // <-- ADD THIS
      ensInitialized = false;
    }
  }

  // ===== 3. COMPENSATION INPUT =====
  if (ensInitialized) {
    if (!isnan(temperature) && !isnan(humidity)) {
      ens160.setTempAndHum(temperature, humidity);
      Serial.printf("✅ ENS160 - Compensation set to T: %.2f°C, H: %.2f%%\n", temperature, humidity);
    } else {
      ens160.setTempAndHum(25.0, 50.0);
      LOG_ERROR("ENS160 using fallback compensation (T/H invalid)");
    }
  }

  // ===== 4. READ ENS160 WITH FALLBACK CACHE =====
  static uint8_t lastAQI = 1;
  static uint16_t lastTVOC = 0;
  static uint16_t lastECO2 = 400;

  uint8_t aqi_local = lastAQI;
  uint16_t tvoc_local = lastTVOC;
  uint16_t eco2_local = lastECO2;

  if (ensInitialized) {
    if (readENS160Sensor(aqi_local, tvoc_local, eco2_local)) {
      lastAQI = aqi_local;
      lastTVOC = tvoc_local;
      lastECO2 = eco2_local;
      // Serial.printf("✅ ENS160 - AQI: %d, TVOC: %d ppb, eCO2: %d ppm\n", aqi_local, tvoc_local, eco2_local);
    } else {
      LOG_ERROR("ENS160 read FAILED → Attempting I2C Recovery");
      recoverI2C();  // <-- ADD THIS
    }
  }

  aqi = aqi_local;
  tvoc = tvoc_local;
  eco2 = eco2_local;

  // Serial.printf("Final Result - SHT OK: %s | ENS OK: %s\n", shtOK ? "✅" : "❌", ensInitialized ? "✅" : "❌");
  return (shtOK && ensInitialized);
}

///======== RELAY CONTROL SECTION =======//
// Relay control task with environmental checks (optimized: hysteresis and delays for stability)
void checkRelayTask(void* pvParameters) {
  static int lastRelayState = digitalRead(RELAY1_PIN);
  static int lastPumpState = digitalRead(RELAY2_PIN);
  static unsigned long lastMinuteCheck = 0;
  // Diagnostic: Task started, restore compressor minutes
  compressorMinutes = readCounterULong("compMinutes", 0);
  if (lastRelayState == LOW) {
    compStartTime = millis();
    lastMinuteCheck = compStartTime;
  }
  while (true) {
    getLatestReadings(temperature, humidity, AQI, TVOC, ECO2);
    // Validate sensor ranges for diagnostics
    bool tempValid = (!isnan(temperature) && temperature >= -40 && temperature <= 125 && temperature != 0.0);
    bool humValid = (!isnan(humidity) && humidity >= 1 && humidity <= 100);
    bool distValid = (distance >= 3 && distance <= 500);  // mandatory
    // Skip if distance invalid
    if (!distValid) {
      LOG_ERROR("Distance invalid (%.2f cm). Skipping control logic.", distance);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }
    // Environmental checks
    bool tempOutOfRange = (tempValid && (temperature < config.tempThresholdMin || temperature > config.tempThresholdMax));
    bool humOutOfRange = (humValid && (humidity < config.humidityThresholdMin || humidity > config.humidityThresholdMax));
    // OFF: Tank full
    if (distance <= config.compressorOffLevel) {
      if (lastRelayState != HIGH) {
        digitalWrite(RELAY1_PIN, HIGH);
        lastRelayState = HIGH;
        lastRelayOffTime = millis();
        compStartTime = 0;
        LOG_ERROR("Relay OFF → Compressor stopped (Reason: Tank full)");  // Log as warn, but per request treat as error
      }
    }
    // OFF: Env out of range
    else if (tempOutOfRange || humOutOfRange) {
      if (lastRelayState != HIGH) {
        digitalWrite(RELAY1_PIN, HIGH);
        lastRelayState = HIGH;
        lastRelayOffTime = millis();
        compStartTime = 0;
        LOG_ERROR("Relay OFF → Compressor stopped (Reason: Env out of range)");
      }
    }
    // ON: Tank low + 3-min hysteresis
    else if (distance >= config.compressorOnLevel) {
      if (lastRelayState != LOW) {
        if (millis() - lastRelayOffTime >= 180000) {  // 3-min delay
          digitalWrite(RELAY1_PIN, LOW);
          lastRelayState = LOW;
          compStartTime = millis();
          lastMinuteCheck = compStartTime;
          // Success: Compressor started (no log)
        } else {
          LOG_ERROR("Relay ON blocked (waiting 3 min since OFF)");
        }
      }
    }
    // Pump logic: OFF when off level reached (tank full/high level)
    if (distance >= config.pumpOffLevel) {
      if (lastPumpState != HIGH) {
        digitalWrite(RELAY2_PIN, HIGH);
        lastPumpState = HIGH;
        LOG_ERROR("Pump OFF (Reason: Off level reached - tank full)");
      }
    }
    // ON when below off level (tank low/low level)
    else {
      if (lastPumpState != LOW) {
        digitalWrite(RELAY2_PIN, LOW);
        lastPumpState = LOW;
        LOG_ERROR("Pump ON (Reason: Below off level - tank low)");
      }
    }
    // Minute tracking
    if (lastRelayState == LOW && (millis() - lastMinuteCheck >= 60000)) {
      compressorMinutes++;
      saveCounterULong("compMinutes", compressorMinutes);
      lastMinuteCheck += 60000;
      // Success: Runtime incremented (no log)
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
///======== CONNECTIVITY SECTION =======//
// Internet check (optimized: simple DNS ping)
bool hasInternetConnection() {
  WiFiClient client;
  return client.connect("8.8.8.8", 53);  // Success: Connected (no log)
}
// Connectivity monitoring task (optimized: staggered retries)
void checkConnectivityTask(void* pvParameters) {
  static unsigned long lastRetryTime = 0;
  const unsigned long retryInterval = 180000;
  while (true) {
    if (isWIFI) {
      if (WiFi.status() != WL_CONNECTED || !hasInternetConnection()) {
        LOG_ERROR("WiFi disconnected or no internet. Attempting to reconnect...");
        buzzerSearching();
        WiFi.disconnect();
        delay(1000);
        if (connectWiFi()) {
          // Success: Reconnected (no log)
          buzzerConnected();
          delay(3000);
          client.disconnect();
          connectAWS();
          if (!client.connected()) {
            LOG_ERROR("MQTT connection failed, state: %d", client.state());
          }
        } else {
          LOG_ERROR("WiFi reconnection failed.");
          buzzerFailure();
          if (millis() - lastRetryTime >= retryInterval) {
            lastRetryTime = millis();
            LOG_ERROR("Starting setup server due to WiFi failure.");
            setupServer();
          }
        }
      }
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}
///======== WIFI TASK SECTION =======//
// WiFi data publishing task with keep-alives (optimized: tick-based scheduling)
void WIFITask(void* pvParameters) {
  const TickType_t keepAlivePeriod = keepAliveInterval / portTICK_PERIOD_MS;
  const TickType_t publishPeriod = config.publishInterval / portTICK_PERIOD_MS;
  // Diagnostic: Task started
  connectAWS();
  TickType_t lastPublishTime = xTaskGetTickCount();
  TickType_t lastKeepAliveTime = xTaskGetTickCount();
  bool isPublishing = false;
  publishKeepAlive();
  vTaskDelay(10000 / portTICK_PERIOD_MS);
  sendViaWIFI();
  while (true) {
    if (!client.connected()) {
      connectAWS();  // Success: Reconnect if needed (no log)
    }
    client.loop();
    TickType_t now = xTaskGetTickCount();
    if (!isPublishing && now - lastPublishTime >= publishPeriod) {
      isPublishing = true;
      sendViaWIFI();
      lastPublishTime = xTaskGetTickCount();
      isPublishing = false;
      // Success: Data published (no log)
    }
    if (!isPublishing && now - lastKeepAliveTime >= keepAlivePeriod) {
      isPublishing = true;
      publishKeepAlive();
      lastKeepAliveTime = xTaskGetTickCount();
      isPublishing = false;
      // Success: Keep-alive sent (no log)
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
///======== GSM TASK SECTION =======//
// GSM data publishing task (optimized: failure counting with reset)
void GSMTask(void* pvParameters) {
  const TickType_t keepAlivePeriod = keepAliveInterval / portTICK_PERIOD_MS;
  const TickType_t publishPeriod = config.publishInterval / portTICK_PERIOD_MS;
  // Diagnostic: Task started
  TickType_t lastPublishTime = xTaskGetTickCount();
  TickType_t lastKeepAliveTime = xTaskGetTickCount();
  publishKeepAlive();
  vTaskDelay(10000 / portTICK_PERIOD_MS);
  sendViaGSM();
  while (true) {
    TickType_t now = xTaskGetTickCount();
    if (now - lastKeepAliveTime >= keepAlivePeriod) {
      publishKeepAlive();
      lastKeepAliveTime = now;
      // Success: Keep-alive (no log)
    }
    if (now - lastPublishTime >= publishPeriod) {
      sendViaGSM();
      lastPublishTime = now;
      // Success: Data sent (no log)
    }
    // Recovery on failures
    if (gsmFailureCount >= GSM_MAX_FAILURES) {
      LOG_ERROR("GSM failed %d times. Restarting GSM...", gsmFailureCount);
      gsmFailureCount = 0;  // reset counter
      setupGSM();           // reboot modem + reconnect MQTT
      if (!isMQTTConnected) {
        LOG_ERROR("Still no GSM after reboot. Starting setup server...");
        setupServer();  // manual reconfiguration fallback
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
///======== KEEP-ALIVE SECTION =======//
// Keep-alive publishing with retries (optimized: exponential backoff)
void publishKeepAlive() {
  // Diagnostic: Publishing keep-alive
  if (isWIFI && !client.connected()) {
    LOG_ERROR("WiFi MQTT disconnected, attempting reconnect");
    client.disconnect();
    connectAWS();
  } else if (isGSM && !checkMQTTConnection()) {
    LOG_ERROR("GSM MQTT disconnected, attempting reconnect");
    sendATCommandGetResponse("AT+CMQTTDISC=0,120", 5000);
    sendATCommandGetResponse("AT+CMQTTREL=0", 5000);
    sendATCommandGetResponse("AT+CMQTTSTOP", 5000);
    delay(2000);
    isMQTTConnected = false;
    connectMQTT();
  }
  time_t timestamp = 0;
  if (isWIFI) {
    if (getLocalTime(&timeinfo)) {
      timestamp = mktime(&timeinfo);
      // Success: Time obtained (no log)
    } else {
      LOG_ERROR("Failed to update time");
      timestamp = 123456789;
    }
  } else if (isGSM) {
    timestamp = getUnixTimestamp();
  }
  if (timestamp == 0) {
    LOG_ERROR("Failed to get timestamp, using millis()");
    timestamp = millis() / 1000;
  }
  DynamicJsonDocument doc(256);
  doc["TimeStamp"] = timestamp;
  doc["DeviceId"] = config.deviceId;
  doc["COM"] = isWIFI ? "WiFi" : "GSM";
  if (isWIFI) {
    int rssi = WiFi.RSSI();
    doc["RSSI"] = rssi;
  }
  String jsonBuffer;
  serializeJson(doc, jsonBuffer);
  // Success: JSON prepared (no log)
  bool success = false;
  int retryDelay = 100;
  for (int i = 0; i < 3; i++) {
    if (isWIFI && client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer.c_str())) {
      success = true;
      break;  // Success: Published (no log)
    } else if (isGSM && publishMQTTMessage(jsonBuffer)) {
      success = true;
      break;
    } else {
      LOG_ERROR("Keep-alive publish failed (Attempt %d/3)", i + 1);
      delay(retryDelay);
      retryDelay *= 2;
      if (isWIFI && !client.connected()) {
        LOG_ERROR("WiFi MQTT disconnected, forcing reconnect");
        WiFi.disconnect(true);
        delay(1000);
        connectWiFi();
        client.disconnect();
        delay(500);
        connectAWS();
      } else if (isGSM && !checkMQTTConnection()) {
        LOG_ERROR("GSM MQTT disconnected, reconnecting");
        connectMQTT();
      }
    }
  }
  if (success) {
    // Success: Keep-alive complete (no log)
  } else {
    LOG_ERROR("All keep-alive publish attempts failed");
    mqttFailureCount++;
  }
}
///======== AWS MQTT SECTION =======//
// AWS connection setup (optimized: cert loading once)
void connectAWS() {
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
  client.setServer(AWS_IOT_ENDPOINT, 8883);
  // Diagnostic: Connecting to AWS
  int attempts = 0;
  while (!client.connected() && attempts < 5) {  // Added attempt limit for optimization
    if (client.connect(MQTT_CLIENT_ID)) {
      client.setBufferSize(1024);
      // Success: Connected (no log)
      break;
    } else {
      LOG_ERROR("Failed: %d", client.state());
      delay(2000);
      attempts++;
    }
  }
  if (!client.connected()) {
    LOG_ERROR("AWS connection failed after 5 attempts");
  }
}
// Message handler (optimized: lightweight JSON parse)
void messageHandler(char* topic, byte* payload, unsigned int length) {
  // Diagnostic: Incoming message
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  // Success: Handled (no log)
}
///======== COMMUNICATION SETUP SECTION =======//
// Setup communication based on config (optimized: early returns)
void setupCommunication() {
  // Diagnostic: Setting up communication
  if (strcmp(config.type, "GSM") == 0) {
    isGSM = true;
    SerialGSM.begin(115200, SERIAL_8N1, gsmRX, gsmTX);
    // Success: GSM selected (no log)
  } else if (strcmp(config.type, "WiFi") == 0) {
    isWIFI = true;
    if (!connectWiFi()) {
      LOG_ERROR("WiFi connection failed. Starting in setup mode.");
      setupServer();
      return;
    }
    // Time sync
    delay(4000);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    bool timeSynced = false;
    int retries = 0;
    while (!timeSynced && retries < 3) {
      if (getLocalTime(&timeinfo)) {
        timeSynced = true;
        // Success: Time synced (no log)
      } else {
        LOG_ERROR("Failed to obtain time. Retry %d", retries + 1);
        retries++;
        delay(2000);
      }
    }
    if (!timeSynced) {
      LOG_ERROR("Time synchronization failed after 3 attempts.");
      return;
    }
    char timeStringBuff[50];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
    // Success: WiFi connected (no log)
    buzzerConnected();
  } else {
    LOG_ERROR("Invalid communication type. Starting in setup mode.");
    setupServer();
  }
}
///======== GSM SETUP SECTION =======//
// GSM modem setup with cert upload (optimized: chunked upload for large certs)
void setupGSM() {
  digitalWrite(gsmRST, HIGH);
  delay(2000);
  digitalWrite(gsmRST, LOW);
  delay(5000);
  // Diagnostic: Initializing GSM
  String response = sendATCommandGetResponse("AT");
  if (response.indexOf("OK") < 0) {
    LOG_ERROR("No response from modem. Check connections.");
  }
  sendATCommandGetResponse("ATE0");
  sendATCommandGetResponse("AT+CREG=1");
  delay(1000);
  String netStat = sendATCommandGetResponse("AT+CREG?");
  if (netStat.indexOf("+CREG: 1,1") < 0 && netStat.indexOf("+CREG: 1,5") < 0) {
    LOG_ERROR("Network registration failed!");
  }
  sendATCommandGetResponse("AT+CGATT=1");
  sendATCommandGetResponse("AT+CGDCONT=1,\"IP\",\"" + String(config.gsm_apn) + "\"");
  sendATCommandGetResponse("AT+CGACT=1,1");
  // Success: GPRS connected (no log)
  sendATCommandGetResponse("AT+CCERTLIST");
  sendATCommandGetResponse("AT+CCERTDELE=\"" + String(CA_CERT_FILE) + "\"");
  sendATCommandGetResponse("AT+CCERTDELE=\"" + String(CLIENT_CERT_FILE) + "\"");
  sendATCommandGetResponse("AT+CCERTDELE=\"" + String(CLIENT_KEY_FILE) + "\"");
  // Cert uploads
  if (!uploadCertificate(CA_CERT_FILE, AWS_CERT_CA, strlen(AWS_CERT_CA))) {
    LOG_ERROR("Failed to upload CA certificate");
  }
  if (!uploadCertificate(CLIENT_CERT_FILE, AWS_CERT_CRT, strlen(AWS_CERT_CRT))) {
    LOG_ERROR("Failed to upload client certificate");
  }
  if (!uploadCertificate(CLIENT_KEY_FILE, AWS_CERT_PRIVATE, strlen(AWS_CERT_PRIVATE))) {
    LOG_ERROR("Failed to upload client key");
  }
  // SSL config
  sendATCommandGetResponse("AT+CSSLCFG=\"sslversion\",0,3");
  sendATCommandGetResponse("AT+CSSLCFG=\"authmode\",0,2");
  sendATCommandGetResponse("AT+CSSLCFG=\"cacert\",0,\"" + String(CA_CERT_FILE) + "\"");
  sendATCommandGetResponse("AT+CSSLCFG=\"clientcert\",0,\"" + String(CLIENT_CERT_FILE) + "\"");
  sendATCommandGetResponse("AT+CSSLCFG=\"clientkey\",0,\"" + String(CLIENT_KEY_FILE) + "\"");
  // MQTT connect
  if (connectMQTT()) {
    // Success: Setup complete (no log)
    buzzerConnected();
  } else {
    LOG_ERROR("Failed to connect to AWS IoT via MQTT");
  }
}
bool uploadCertificate(const char* filename, const char* data, size_t len) {
  String cmd = "AT+CCERTDOWN=\"" + String(filename) + "\"," + String(len);
  String resp = sendATCommandGetResponse(cmd);
  if (resp.indexOf(">") >= 0) {
    delay(500);
    const int CHUNK_SIZE = 256;
    for (size_t i = 0; i < len; i += CHUNK_SIZE) {
      size_t chunk = min((size_t)CHUNK_SIZE, len - i);
      SerialGSM.write(data + i, chunk);
      SerialGSM.flush();
      delay(200);
    }
    unsigned long start = millis();
    String dataResp = "";
    while (millis() - start < 20000) {
      if (SerialGSM.available()) {
        char c = SerialGSM.read();
        dataResp += c;
        Serial.print(c);
      }
      if (dataResp.indexOf("OK") >= 0) {
        return true;  // Success: Cert uploaded (no log)
      }
      if (dataResp.indexOf("ERROR") >= 0) {
        return false;
      }
    }
  }
  return false;
}
// MQTT connection for GSM (optimized: timeout handling)
bool connectMQTT() {
  // Diagnostic: Starting MQTT
  String response = sendATCommandGetResponse("AT+CMQTTSTART", 12000);
  if (response.indexOf("OK") < 0) {
    LOG_ERROR("Failed to start MQTT service");
    return false;
  }
  delay(1000);
  response = sendATCommandGetResponse("AT+CMQTTACCQ=0,\"" + String(MQTT_CLIENT_ID) + "\",1", 5000);
  if (response.indexOf("OK") < 0) {
    LOG_ERROR("Failed to acquire MQTT client");
    sendATCommandGetResponse("AT+CMQTTSTOP", 5000);
    delay(1000);
    return false;
  }
  response = sendATCommandGetResponse("AT+CMQTTSSLCFG=0,0", 5000);
  if (response.indexOf("OK") < 0) {
    LOG_ERROR("SSL config failed - trying anyway");
  }
  response = sendATCommandGetResponse("AT+CMQTTCONNECT=0,\"tcp://" + String(AWS_IOT_ENDPOINT) + ":8883\",60,1", 30000);
  delay(5000);
  if (response.indexOf("OK") < 0 && response.indexOf("+CMQTTCONNECT") < 0) {
    LOG_ERROR("Connection failed");
    sendATCommandGetResponse("AT+CMQTTREL=0", 5000);
    sendATCommandGetResponse("AT+CMQTTSTOP", 5000);
    return false;
  }
  lastKeepAliveTime = millis();
  isMQTTConnected = true;
  // Success: MQTT connected (no log)
  return true;
}
String waitForResponse(unsigned long timeout) {
  String response = "";
  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (SerialGSM.available()) {
      char c = SerialGSM.read();
      response += c;
      Serial.print(c);
    }
    if (response.endsWith("OK\r\n") || response.endsWith("ERROR\r\n")) {
      break;
    }
  }
  return response;
}
///======== TIMESTAMP SECTION =======//
// Unix timestamp from GSM (optimized: leap year handling)
const int daysBeforeMonth[] = {
  0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
};
bool isLeap(int y) {
  return ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0));
}
unsigned long toEpoch(int year, int month, int day, int hour, int minute, int second) {
  long days = 0;
  for (int y = 1970; y < year; y++) {
    days += isLeap(y) ? 366 : 365;
  }
  days += daysBeforeMonth[month - 1];
  if (month > 2 && isLeap(year)) {
    days += 1;
  }
  days += (day - 1);
  unsigned long epoch = days * 86400UL + hour * 3600UL + minute * 60UL + second;
  return epoch;
}
unsigned long getUnixTimestamp() {
  SerialGSM.println("AT+CTZU=1");  // Auto time update
  SerialGSM.println("AT+CTZR=1");  // Timezone reporting
  delay(1500);
  for (int attempts = 0; attempts < 5; attempts++) {
    SerialGSM.println("AT+CCLK?");
    delay(1500);
    String response = "";
    while (SerialGSM.available()) {
      response += SerialGSM.readStringUntil('\n');
    }
    int index = response.indexOf("+CCLK: \"");
    if (index == -1 || response.length() < index + 23) {
      delay(2000);
      continue;
    }
    String timeString = response.substring(index + 8, index + 28);
    int year = 2000 + timeString.substring(0, 2).toInt();
    int month = timeString.substring(3, 5).toInt();
    int day = timeString.substring(6, 8).toInt();
    int hour = timeString.substring(9, 11).toInt();
    int minute = timeString.substring(12, 14).toInt();
    int second = timeString.substring(15, 17).toInt();
    char sign = timeString.charAt(17);
    int tzVal = timeString.substring(18, 20).toInt();
    int offsetMinutes = tzVal * 15;
    if (sign == '-') offsetMinutes = -offsetMinutes;
    long offsetSeconds = offsetMinutes * 60L;
    if (year < 2020 || month == 0 || day == 0) {
      delay(2000);
      continue;
    }
    unsigned long localEpoch = toEpoch(year, month, day, hour, minute, second);
    unsigned long utcEpoch = localEpoch - offsetSeconds;
    Serial.printf("GSM Timestamp - Local: %lu, UTC: %lu\n", localEpoch, utcEpoch);
    // Success: Timestamp parsed (no log)
    return utcEpoch;
  }
  LOG_ERROR("Failed to get GSM timestamp after 5 attempts");
  return 0;  // Failed
}
// MQTT connection check (optimized: ping-based)
bool checkMQTTConnection() {
  if (!isMQTTConnected) {
    return false;
  }
  if (millis() - lastKeepAliveTime > keepAliveInterval) {
    String response = sendATCommandGetResponse("AT+CMQTTPUB=0,0,60", 5000);
    if (response.indexOf("ERROR") >= 0 || response.indexOf("+CMQTTPUB: 0,") < 0) {
      LOG_ERROR("MQTT connection lost during keep-alive check");
      isMQTTConnected = false;
      return false;
    }
    lastKeepAliveTime = millis();
    // Success: Ping ok (no log)
  }
  return true;
}
inline void jsonFloat2(JsonDocument& doc, const char* key, float val) {
  if (isnan(val)) {
    doc[key] = nullptr;
  } else {
    doc[key] = roundf(val * 100.0f) * 0.01f;
  }
}

///======== GSM PUBLISH SECTION =======//
// Send data via GSM with validation and resets (optimized: null on invalid data)
void sendViaGSM() {

  /* ================= MQTT CONNECTION CHECK ================= */
  if (!checkMQTTConnection()) {
    LOG_ERROR("MQTT reconnection needed");

    sendATCommandGetResponse("AT+CMQTTDISC=0,120", 5000);
    sendATCommandGetResponse("AT+CMQTTREL=0", 5000);
    sendATCommandGetResponse("AT+CMQTTSTOP", 5000);
    delay(2000);

    isMQTTConnected = false;

    if (!connectMQTT()) {
      LOG_ERROR("Failed to reconnect MQTT. Will retry later.");
      mqttFailureCount++;
      return;
    }
  }

  /* ================= TIMESTAMP ================= */
  unsigned long timestamp = getUnixTimestamp();
  if (timestamp == 0) {
    LOG_ERROR("Failed to get Unix timestamp, using millis()");
    timestamp = millis() / 1000;
  }
 /* ================= SAFE DISTANCE READ ================= */
  float distanceToSend = NAN;
  if (xSemaphoreTake(ultrasonicMutex, portMAX_DELAY)) {
    distanceToSend = distance;
    xSemaphoreGive(ultrasonicMutex);
  }
  /* ================= JSON BUILD ================= */
 
  StaticJsonDocument<1536> doc;

  doc["TimeStamp"] = timestamp;
  doc["DeviceId"] = config.deviceId;
  doc["DeviceName"] = "AeroneroControlSystem";
  doc["DeviceFirmwareVersion"] = current_firmware_version;

  /* ================= SENSOR VALUES (2 DECIMALS ONLY IN JSON) ================= */

  jsonFloat2(doc, "Temperature",
             (isnan(temperature) || temperature < -40 || temperature > 125 || temperature == 0.0)
               ? NAN
               : temperature);

  jsonFloat2(doc, "Humidity",
             (isnan(humidity) || humidity < 1 || humidity > 100)
               ? NAN
               : humidity);

  jsonFloat2(doc, "WaterTankLevel",
             (distanceToSend < 3 || distanceToSend > 500)
               ? NAN
               : distanceToSend);

  float savedPh = readCounter("phValue", NAN);
  float savedTds = readCounter("tdsValue", NAN);

  jsonFloat2(doc, "PhValue",
             (savedPh < 1 || savedPh > 14)
               ? NAN
               : savedPh);

  jsonFloat2(doc, "Tds",
             (savedTds < 1 || savedTds > 1000)
               ? NAN
               : savedTds);

  /* ================= AIR QUALITY (INTEGERS) ================= */
  /* ================= AIR QUALITY (INTEGERS) ================= */

  if (AQI < 1 || AQI > 5) {
    doc["AQI"] = nullptr;
  } else {
    doc["AQI"] = AQI;
  }

  if (TVOC < 0 || TVOC > 65000) {
    doc["TVOC"] = nullptr;
  } else {
    doc["TVOC"] = TVOC;
  }

  if (ECO2 < 400 || ECO2 > 65000) {
    doc["ECO2"] = nullptr;
  } else {
    doc["ECO2"] = ECO2;
  }

  /* ================= COUNTERS ================= */

  float savedLiters = readCounter("totalLiters", 0.0f);
  unsigned long motor = readCounterULong("motorMinutes", 0);
  unsigned long compressor = readCounterULong("compMinutes", 0);

  float litersToSend = 0.0f;

  if (!flowFlag) {
    litersToSend = savedLiters;
    if (fabs(litersToSend) < 0.001f) litersToSend = 0.0f;
  } else {
    litersToSend = 0.0f;
    motor = 0;
  }

  jsonFloat2(doc, "WaterTotalizer", litersToSend);

  doc["Motor"] = motor;
  doc["Compressor"] = compressor;

  /* ================= METADATA ================= */

  doc["DataCollectionInterval"] = config.publishInterval / 60000;
  doc["Timezone"] = "IST-05:30";
  doc["TimezoneCountry"] = "Asia/Kolkata";
  doc["TimeAutoSync"] = "Enabled";
  doc["MQTTEndpoint"] = "a1brrleef337f0-ats.iot.ap-south-1.amazonaws.com";
  doc["MQTTPort"] = "8883";
  doc["AirQualityRegulationRegion"] = "India";

  /* ================= SERIALIZE ================= */
  char jsonBuffer[1536];

  size_t len = serializeJson(
    doc,
    jsonBuffer,
    sizeof(jsonBuffer)
  );


  /* ================= MQTT PUBLISH WITH RETRY ================= */
  bool success = false;
  int retryDelay = 100;

  for (int i = 0; i < 3; i++) {
    if (publishMQTTMessage(jsonBuffer)) {
      success = true;
      break;
    }

    LOG_ERROR("MQTT Publish failed (Attempt %d/3)", i + 1);
    delay(retryDelay);
    retryDelay *= 2;

    if (!checkMQTTConnection()) {
      connectMQTT();
    }
  }

  /* ================= POST-PUBLISH HANDLING ================= */
  if (success) {

    if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {

      if (!flowFlag) {
        totalLiters = 0.0f;
        prefs.begin("counters", false);
        prefs.putFloat("totalLiters", 0.0f);
        prefs.end();
      }

      motorMinutes = 0;
      compressorMinutes = 0;

      prefs.begin("counters", false);
      prefs.putULong("motorMinutes", 0);
      prefs.putULong("compMinutes", 0);
      prefs.end();

      xSemaphoreGive(prefsMutex);
    } else {
      LOG_ERROR("Failed to take prefsMutex to reset counters");
    }

    digitalWrite(dataStatus, HIGH);
    delay(2000);
    digitalWrite(dataStatus, LOW);

    mqttFailureCount = 0;

  } else {
    LOG_ERROR("All MQTT publish attempts failed.");
    mqttFailureCount++;
  }

  lastKeepAliveTime = millis();
}



// MQTT publish for GSM (optimized: step-by-step AT commands)
bool publishMQTTMessage(String message) {
  if (!isMQTTConnected) {
    LOG_ERROR("MQTT not connected. Cannot publish.");
    gsmFailureCount++;
    return false;
  }
  String response = sendATCommandGetResponse("AT+CMQTTTOPIC=0," + String(strlen(AWS_IOT_PUBLISH_TOPIC)), 5000);
  if (response.indexOf(">") < 0) {
    LOG_ERROR("Failed to enter topic data mode");
    isMQTTConnected = false;
    gsmFailureCount++;
    return false;
  }
  SerialGSM.print(AWS_IOT_PUBLISH_TOPIC);
  response = waitForResponse(5000);
  if (response.indexOf("OK") < 0) {
    LOG_ERROR("Failed to set topic");
    isMQTTConnected = false;
    gsmFailureCount++;
    return false;
  }
  response = sendATCommandGetResponse("AT+CMQTTPAYLOAD=0," + String(message.length()), 5000);
  if (response.indexOf(">") < 0) {
    LOG_ERROR("Failed to enter payload data mode");
    isMQTTConnected = false;
    gsmFailureCount++;
    return false;
  }
  SerialGSM.print(message);
  response = waitForResponse(5000);
  if (response.indexOf("OK") < 0) {
    LOG_ERROR("Failed to set payload");
    isMQTTConnected = false;
    gsmFailureCount++;
    return false;
  }
  SerialGSM.println("AT+CMQTTPUB=0,1,60");
  unsigned long startTime = millis();
  bool gotPubResponse = false;
  while (millis() - startTime < 10000) {
    if (SerialGSM.available()) {
      String line = SerialGSM.readStringUntil('\n');
      if (line.startsWith("+CMQTTPUB: 0,0")) {
        gotPubResponse = true;
        break;
      }
      if (line.indexOf("ERROR") >= 0) {
        isMQTTConnected = false;
        gsmFailureCount++;
        break;
      }
    }
  }
  if (!gotPubResponse) {
    LOG_ERROR("Publish failed - no valid response");
    isMQTTConnected = false;
    gsmFailureCount++;
    return false;
  }
  lastKeepAliveTime = millis();
  gsmFailureCount = 0;
  // Success: Message published (no log)
  return true;
}
// Incoming message check (optimized: buffer flush)
void checkIncomingMessages() {
  if (SerialGSM.available()) {
    String response = "";
    unsigned long start = millis();
    while (millis() - start < 1000) {
      if (SerialGSM.available()) {
        char c = SerialGSM.read();
        response += c;
        Serial.print(c);
      }
    }
    if (response.indexOf("+CMQTTRXSTART:") >= 0) {
      int topicStart = response.indexOf("+CMQTTRXTOPIC:");
      int payloadStart = response.indexOf("+CMQTTRXPAYLOAD:");
      if (topicStart >= 0 && payloadStart >= 0) {
        String topic = "";
        int topicDataStart = response.indexOf("\r\n", topicStart) + 2;
        int topicDataEnd = response.indexOf("\r\n", topicDataStart);
        if (topicDataStart > 0 && topicDataEnd > 0) {
          topic = response.substring(topicDataStart, topicDataEnd);
        }
        String payload = "";
        int payloadDataStart = response.indexOf("\r\n", payloadStart) + 2;
        int payloadDataEnd = response.indexOf("\r\n+CMQTTRXEND:", payloadDataStart);
        if (payloadDataStart > 0 && payloadDataEnd > 0) {
          payload = response.substring(payloadDataStart, payloadDataEnd);
        }
        if (topic.length() > 0 && payload.length() > 0) {
          processIncomingMessage(topic, payload);
          // Success: Message processed (no log)
        }
      }
    }
  }
}
void processIncomingMessage(String topic, String message) {
  // Diagnostic: Log incoming for traceability
  LOG_ERROR("Incoming message [%s]: %s", topic.c_str(), message.c_str());  // Kept as error for visibility, but could be info
}
///======== WIFI CONNECTION SECTION =======//
// WiFi connection with LED feedback (optimized: max attempts to prevent hang)
bool connectWiFi() {
  // Diagnostic: Connecting to WiFi
  WiFi.begin(config.wifi_ssid, config.wifi_password);
  int attempts = 0;
  const int maxAttempts = 300;
  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    delay(500);
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    // Success: WiFi connected (no log)
    return true;
  } else {
    LOG_ERROR("WiFi connection failed.");
    return false;
  }
}
///======== WIFI PUBLISH SECTION =======//
// Send data via WiFi (similar to GSM, optimized: shared validation logic)

// Send data via WiFi (similar to GSM, optimized: shared validation logic)
void sendViaWIFI() {

  if (WiFi.status() != WL_CONNECTED) {
    LOG_ERROR("WiFi not connected. Reconnection handled by checkConnectivityTask.");
    return;
  }

  /* ================= TIMESTAMP ================= */
  time_t timestamp = 0;

  if (getLocalTime(&timeinfo)) {
    timestamp = mktime(&timeinfo);
  } else {
    LOG_ERROR("Failed to update time");
    timestamp = millis() / 1000;
  }

  if (timestamp == 0) {
    LOG_ERROR("Invalid timestamp, using millis()");
    timestamp = millis() / 1000;
  }

  /* ================= SAFE DISTANCE READ ================= */
  float distanceToSend = NAN;
  if (xSemaphoreTake(ultrasonicMutex, portMAX_DELAY)) {
    distanceToSend = distance;
    xSemaphoreGive(ultrasonicMutex);
  }
  /* ================= JSON BUILD ================= */
  StaticJsonDocument<1024> doc;


  doc["TimeStamp"] = timestamp;
  doc["DeviceId"] = config.deviceId;
  doc["DeviceName"] = "AeroneroControlSystem";
  doc["DeviceFirmwareVersion"] = current_firmware_version;

  /* ================= SENSOR VALUES (2 DECIMALS ONLY IN JSON) ================= */

  jsonFloat2(doc, "Temperature",
             (isnan(temperature) || temperature < -40 || temperature > 125 || temperature == 0.0)
               ? NAN
               : temperature);

  jsonFloat2(doc, "Humidity",
             (isnan(humidity) || humidity < 1 || humidity > 100)
               ? NAN
               : humidity);

  jsonFloat2(doc, "WaterTankLevel",
             (distanceToSend < 3 || distanceToSend > 500)
               ? NAN
               : distanceToSend);

  float savedPh = readCounter("phValue", NAN);
  float savedTds = readCounter("tdsValue", NAN);

  jsonFloat2(doc, "PhValue",
             (savedPh < 1 || savedPh > 14)
               ? NAN
               : savedPh);

  jsonFloat2(doc, "Tds",
             (savedTds < 1 || savedTds > 1000)
               ? NAN
               : savedTds);

  /* ================= AIR QUALITY (INTEGERS) ================= */
  /* ================= AIR QUALITY (INTEGERS) ================= */

  if (AQI < 1 || AQI > 5) {
    doc["AQI"] = nullptr;
  } else {
    doc["AQI"] = AQI;
  }

  if (TVOC < 0 || TVOC > 65000) {
    doc["TVOC"] = nullptr;
  } else {
    doc["TVOC"] = TVOC;
  }

  if (ECO2 < 400 || ECO2 > 65000) {
    doc["ECO2"] = nullptr;
  } else {
    doc["ECO2"] = ECO2;
  }

  /* ================= COUNTERS ================= */

  float savedLiters = readCounter("totalLiters", 0.0f);
  unsigned long motor = readCounterULong("motorMinutes", 0);
  unsigned long comp = readCounterULong("compMinutes", 0);

  float litersToSend = 0.0f;

  if (!flowFlag) {
    litersToSend = savedLiters;
    if (fabs(litersToSend) < 0.001f) litersToSend = 0.0f;
  } else {
    litersToSend = 0.0f;
    motor = 0;
  }

  jsonFloat2(doc, "WaterTotalizer", litersToSend);

  doc["Motor"] = motor;
  doc["Compressor"] = comp;

  /* ================= METADATA ================= */

  doc["DataCollectionInterval"] = config.publishInterval / 60000;
  doc["Timezone"] = "IST-05:30";
  doc["TimezoneCountry"] = "Asia/Kolkata";
  doc["TimeAutoSync"] = "Enabled";
  doc["MQTTEndpoint"] = "a1brrleef337f0-ats.iot.ap-south-1.amazonaws.com";
  doc["MQTTPort"] = "8883";
  doc["AirQualityRegulationRegion"] = "India";

  /* ================= SERIALIZE ================= */

  char jsonBuffer[1024];
  serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  /* ================= MQTT CONNECTION ================= */

  if (!client.connected()) {
    LOG_ERROR("MQTT not connected, attempting reconnect...");
    connectAWS();
  }

  /* ================= MQTT PUBLISH WITH RETRY ================= */

  bool success = false;
  int retryDelay = 100;

  for (int i = 0; i < 3; i++) {
    if (client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer)) {
      success = true;
      break;
    }

    LOG_ERROR("MQTT Publish failed (Attempt %d/3). Error: %d",
              i + 1, client.state());

    delay(retryDelay);
    retryDelay *= 2;
    connectAWS();
  }

  /* ================= POST-PUBLISH HANDLING ================= */

  if (success) {

    if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {

      if (!flowFlag) {
        totalLiters = 0.0f;
        prefs.begin("counters", false);
        prefs.putFloat("totalLiters", 0.0f);
        prefs.end();
      }

      motorMinutes = 0;
      compressorMinutes = 0;

      prefs.begin("counters", false);
      prefs.putULong("motorMinutes", 0);
      prefs.putULong("compMinutes", 0);
      prefs.end();

      xSemaphoreGive(prefsMutex);
    } else {
      LOG_ERROR("Failed to take prefsMutex to reset counters");
    }

    digitalWrite(dataStatus, HIGH);
    delay(2000);
    digitalWrite(dataStatus, LOW);

    mqttFailureCount = 0;

  } else {
    LOG_ERROR("All MQTT publish attempts failed.");
    mqttFailureCount++;
  }
}


///======== BUTTON AND BUZZER SECTION =======//
// Button press detection (optimized: debounced hold time)
bool isButtonPressed() {
  if (digitalRead(RESET_PIN) == LOW) {
    unsigned long startTime = millis();
    while (digitalRead(RESET_PIN) == LOW) {
      if (millis() - startTime >= HOLD_TIME) {
        return true;
      }
    }
  }
  return false;
}
// Button task (optimized: low priority polling)
void buttonCheckTask(void* pvParameters) {
  // Diagnostic: Task started
  while (true) {
    if (isButtonPressed()) {
      LOG_ERROR("Button pressed, entering setup mode");  // Log as entry point
      buzzerSetupMode();
      setupServer();
      LOG_ERROR("Exiting setup mode");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
//======== ULTRASONIC SECTION =======//
void ultrasonicTask(void* pvParameters) {
  while (true) {
    measureUltrasonicDistance();  // ONLY PLACE WHERE WE READ THE SENSOR
    vTaskDelay(30000 / portTICK_PERIOD_MS);
  }
}
// Ultrasonic distance measurement (optimized: timeout and checksum validation)
///======== ULTRASONIC SECTION (SAFE VERSION) =======//
  //======== ULTRASONIC RAW READ =======//
float readUltrasonicRaw() {
  uint8_t buffer[4];

  // Flush junk
  while (sensorSerial.available()) {
    sensorSerial.read();
  }

  // Trigger measurement
  sensorSerial.write(0x55);

  unsigned long start = millis();
  while (sensorSerial.available() < 4) {
    if (millis() - start > 250) {   // timeout
      return NAN;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  for (int i = 0; i < 4; i++) {
    buffer[i] = sensorSerial.read();
  }

  // Header check
  if (buffer[0] != 0xFF) return NAN;

  // Checksum
  uint8_t checksum = (buffer[0] + buffer[1] + buffer[2]) & 0xFF;
  if (checksum != buffer[3]) return NAN;

  uint16_t distanceMM = (buffer[1] << 8) | buffer[2];
  float cm = distanceMM / 10.0f;

  if (cm < 3.0f || cm > 500.0f) return NAN;

  return cm;
}
void reinitUltrasonicUART() {
  LOG_ERROR("Ultrasonic: UART reinitialization");

  sensorSerial.end();
  vTaskDelay(pdMS_TO_TICKS(100));

  sensorSerial.setRxBufferSize(1024);
  sensorSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
}


/// Safe wrapper for ultrasonic measurement
//======== ULTRASONIC SAFE MEASUREMENT =======//
void measureUltrasonicDistance() {

  static unsigned long lastRead = 0;
  static uint8_t failCount = 0;

  if (millis() - lastRead < 100) return;  // rate limit
  lastRead = millis();

  float samples[3];
  int count = 0;

  float r;

  r = readUltrasonicRaw();
  if (!isnan(r)) samples[count++] = r;
  vTaskDelay(pdMS_TO_TICKS(20));

  r = readUltrasonicRaw();
  if (!isnan(r)) samples[count++] = r;
  vTaskDelay(pdMS_TO_TICKS(20));

  r = readUltrasonicRaw();
  if (!isnan(r)) samples[count++] = r;

  if (count == 0) {
    failCount++;
    LOG_ERROR("Ultrasonic: no valid samples (%d)", failCount);

    if (failCount >= ULTRASONIC_FAIL_THRESHOLD) {
      reinitUltrasonicUART();
      failCount = 0;
    }
    return;
  }

  failCount = 0;

  // Median / average logic
  float newDistance;

  if (count == 1) {
    newDistance = samples[0];
  } 
  else if (count == 2) {
    newDistance = (samples[0] + samples[1]) * 0.5f;
  } 
  else {
    if ((samples[0] <= samples[1] && samples[1] <= samples[2]) ||
        (samples[2] <= samples[1] && samples[1] <= samples[0])) {
      newDistance = samples[1];
    } else if ((samples[1] <= samples[0] && samples[0] <= samples[2]) ||
               (samples[2] <= samples[0] && samples[0] <= samples[1])) {
      newDistance = samples[0];
    } else {
      newDistance = samples[2];
    }
  }

  xSemaphoreTake(ultrasonicMutex, portMAX_DELAY);

  if (!isnan(distance) && fabs(newDistance - distance) > 5.0f) {
    LOG_ERROR("Ultrasonic spike rejected: %.2f -> %.2f", distance, newDistance);
    xSemaphoreGive(ultrasonicMutex);
    return;
  }

  distance = newDistance;

  xSemaphoreGive(ultrasonicMutex);
}


///======== SYSTEM UTILITIES SECTION =======//
// CPU usage (diagnostic only, low overhead)
float getCPUUsage() {
  return (float)(xTaskGetTickCount() * portTICK_PERIOD_MS) / millis() * 100.0;
}

///======== OTA UPDATE SECTION =======//
// OTA update with WiFi fallback (optimized: stream write for large files)
bool performOTAUpdate(String ssid, String password, String otaLink) {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid.c_str(), password.c_str());
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 50) {
      delay(500);
      attempts++;
    }
    if (WiFi.status() != WL_CONNECTED) {
      LOG_ERROR("Failed to connect to WiFi for OTA");
      return false;
    }
  } else {
    // Success: Already connected (no log)
  }
  HTTPClient http;
  http.begin(otaLink);
  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    LOG_ERROR("Failed to download firmware, HTTP code: %d", httpCode);
    http.end();
    WiFi.disconnect(true);
    return false;
  }
  size_t contentLength = http.getSize();
  if (contentLength <= 0 || !Update.begin(contentLength)) {
    LOG_ERROR("Invalid content length or not enough space for OTA");
    http.end();
    WiFi.disconnect(true);
    return false;
  }
  WiFiClient* client = http.getStreamPtr();
  size_t written = Update.writeStream(*client);
  if (written != contentLength) {
    LOG_ERROR("OTA update failed: Written bytes mismatch");
    http.end();
    WiFi.disconnect(true);
    return false;
  }
  if (!Update.end()) {
    LOG_ERROR("OTA update failed: Error during flash");
    http.end();
    WiFi.disconnect(true);
    return false;
  }
  LOG_ERROR("OTA update successful. Rebooting...");  // Log success here as it's critical
  http.end();
  WiFi.disconnect(true);
  buzzerConnected();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  esp_restart();
  return true;
}
///======== CONFIGURATION SECTION =======//
// Save config to NVS (optimized: batched writes)
void saveConfig() {
  prefs.begin("my-config", false);  // namespace, RW mode
  prefs.putString("deviceId", config.deviceId);
  prefs.putString("type", config.type);
  if (strcmp(config.type, "WiFi") == 0) {
    prefs.putString("wifi_ssid", config.wifi_ssid);
    prefs.putString("wifi_password", config.wifi_password);
  } else if (strcmp(config.type, "GSM") == 0) {
    prefs.putString("gsm_username", config.gsm_username);
    prefs.putString("gsm_password", config.gsm_password);
    prefs.putString("gsm_apn", config.gsm_apn);
  }
  prefs.putFloat("tempMin", config.tempThresholdMin);
  prefs.putFloat("tempMax", config.tempThresholdMax);
  prefs.putFloat("humMin", config.humidityThresholdMin);
  prefs.putFloat("humMax", config.humidityThresholdMax);
  prefs.putFloat("compLevel", config.compressorOnLevel);
  prefs.putFloat("compoffLevel", config.compressorOffLevel);
  prefs.putFloat("pumpOffLevel", config.pumpOffLevel);
  prefs.putInt("publishInterval", config.publishInterval);
  prefs.end();
  // Success: Config saved (no log)
}
// Load config from NVS (optimized: defaults on failure)
// Load config from NVS (optimized: defaults on failure)
void loadConfig() {
  prefs.begin("my-config", true);  // read-only mode
  String deviceId = prefs.getString("deviceId", "");
  String type = prefs.getString("type", "");
  if (deviceId == "" || type == "") {
    LOG_ERROR("No valid configuration found. Starting in setup mode.");
    setupServer();
    prefs.end();
    return;
  }
  strncpy(config.deviceId, deviceId.c_str(), sizeof(config.deviceId));
  strncpy(config.type, type.c_str(), sizeof(config.type));
  if (type == "WiFi") {
    String ssid = prefs.getString("wifi_ssid", "");
    String pass = prefs.getString("wifi_password", "");
    strncpy(config.wifi_ssid, ssid.c_str(), sizeof(config.wifi_ssid));
    strncpy(config.wifi_password, pass.c_str(), sizeof(config.wifi_password));
  } else if (type == "GSM") {
    String user = prefs.getString("gsm_username", "");
    String pass = prefs.getString("gsm_password", "");
    String apn = prefs.getString("gsm_apn", "");
    strncpy(config.gsm_username, user.c_str(), sizeof(config.gsm_username));
    strncpy(config.gsm_password, pass.c_str(), sizeof(config.gsm_password));
    strncpy(config.gsm_apn, apn.c_str(), sizeof(config.gsm_apn));
  }
  // Load thresholds
  config.tempThresholdMin = prefs.getFloat("tempMin", 0);
  config.tempThresholdMax = prefs.getFloat("tempMax", 0);
  config.humidityThresholdMin = prefs.getFloat("humMin", 0);
  config.humidityThresholdMax = prefs.getFloat("humMax", 0);
  config.compressorOnLevel = prefs.getFloat("compLevel", 0);
  config.compressorOffLevel = prefs.getFloat("compoffLevel", 0);
  config.pumpOffLevel = prefs.getFloat("pumpOffLevel", 0);
  config.publishInterval = prefs.getInt("publishInterval", 60000);
  // Load OTA
  String otaSSID = prefs.getString("last_ota_ssid", "");
  String otaPass = prefs.getString("last_ota_password", "");
  String otaLink = prefs.getString("last_ota_link", "");
  strncpy(config.last_ota_ssid, otaSSID.c_str(), sizeof(config.last_ota_ssid));
  strncpy(config.last_ota_password, otaPass.c_str(), sizeof(config.last_ota_password));
  strncpy(config.last_ota_link, otaLink.c_str(), sizeof(config.last_ota_link));
  prefs.end();
  // Success: Config loaded (no log, but debug prints kept for diagnostics)
  LOG_ERROR("Configuration loaded: Device ID: %s, Type: %s", config.deviceId, config.type);  // Used error for visibility
  // ... (rest of debug prints as in original, but under LOG_ERROR for consistency)
}
///======== WEB SERVER SECTION =======//
// Setup server for config (optimized: timeout on inactivity)
void setupServer() {
  digitalWrite(WIFI_LED, LOW);
  String ssid = String("AN-") + String(config.deviceId);
  WiFi.softAP(ssid.c_str(), "12345678");
  IPAddress IP = WiFi.softAPIP();
  // Diagnostic: AP IP
  loggedIn = false;
  server.on("/", HTTP_GET, handleRoot);
  server.on("/login", HTTP_POST, handleLogin);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/save_thresholds", HTTP_POST, handleThresholdSave);
  server.on("/ota_submit", HTTP_POST, handleOTASubmit);
  server.begin();
  // Success: Server started (no log)
  lastClientTime = millis();
  while (!configSaved) {
    server.handleClient();
    if (millis() - lastClientTime > 180000) {
      LOG_ERROR("No client activity. Stopping server.");
      server.stop();
      WiFi.softAPdisconnect(true);
      return;
    }
    delay(10);
  }
  delay(2000);
  // Success: Config saved, rebooting (no log)
  server.stop();
  WiFi.softAPdisconnect(true);
  esp_restart();
}
// Threshold save handler
// Threshold save handler
void handleThresholdSave() {
  if (!loggedIn) {
    server.send(401, "text/html", "<html><body><h2>Please login first</h2><a href='/'>Login</a></body></html>");
    return;
  }
  bool updated = false;
  if (server.hasArg("tempThresholdMin")) {
    config.tempThresholdMin = server.arg("tempThresholdMin").toFloat();
    updated = true;
  }
  if (server.hasArg("tempThresholdMax")) {
    config.tempThresholdMax = server.arg("tempThresholdMax").toFloat();
    updated = true;
  }
  if (server.hasArg("humidityThresholdMin")) {
    config.humidityThresholdMin = server.arg("humidityThresholdMin").toFloat();
    updated = true;
  }
  if (server.hasArg("humidityThresholdMax")) {
    config.humidityThresholdMax = server.arg("humidityThresholdMax").toFloat();
    updated = true;
  }
  if (server.hasArg("compressorOnLevel")) {
    config.compressorOnLevel = server.arg("compressorOnLevel").toFloat();
    updated = true;
  }
  if (server.hasArg("compressorOffLevel")) {
    config.compressorOffLevel = server.arg("compressorOffLevel").toFloat();
    updated = true;
  }
  if (server.hasArg("pumpOffLevel")) {
    config.pumpOffLevel = server.arg("pumpOffLevel").toFloat();
    updated = true;
  }
  if (server.hasArg("publishInterval")) {
    config.publishInterval = server.arg("publishInterval").toInt() * 60000;
    updated = true;
  }
  if (updated) {
    saveConfig();
    configSaved = true;
    server.send(200, "text/html", "<html><body><h2>Thresholds saved. Rebooting...</h2></body></html>");
    // Success: Thresholds updated (no log)
  } else {
    server.send(400, "text/html", "<html><body><h2>No valid threshold fields provided.</h2><a href='/'>Back</a></body></html>");
  }
  lastClientTime = millis();
}
// Root handler
void handleRoot() {
  if (!loggedIn) {
    server.send(200, "text/html", generateLoginPage());
  } else {
    server.send(200, "text/html", generateMainPage());
  }
  lastClientTime = millis();
}
// Login handler
void handleLogin() {
  if (server.hasArg("username") && server.hasArg("password")) {
    String username = server.arg("username");
    String password = server.arg("password");
    if (username == "Admin" && password == "Admin@123") {
      loggedIn = true;
      server.send(200, "text/html", generateMainPage());
      // Success: Logged in (no log)
    } else {
      server.send(401, "text/html", "<html><body><h2>Invalid credentials</h2><a href='/'>Back to Login</a></body></html>");
    }
  } else {
    server.send(400, "text/html", "<html><body><h2>Missing credentials</h2><a href='/'>Back to Login</a></body></html>");
  }
  lastClientTime = millis();
}
// Save handler
void handleSave() {
  if (!loggedIn) {
    server.send(401, "text/html", "<html><body><h2>Please login first</h2><a href='/'>Login</a></body></html>");
    return;
  }
  if (server.hasArg("deviceId") && server.hasArg("type")) {
    String deviceId = server.arg("deviceId");
    String type = server.arg("type");
    if (type == "WiFi" && server.hasArg("wifi_ssid") && server.hasArg("wifi_password")) {
      strncpy(config.deviceId, deviceId.c_str(), sizeof(config.deviceId));
      strncpy(config.type, type.c_str(), sizeof(config.type));
      strncpy(config.wifi_ssid, server.arg("wifi_ssid").c_str(), sizeof(config.wifi_ssid));
      strncpy(config.wifi_password, server.arg("wifi_password").c_str(), sizeof(config.wifi_password));
    } else if (type == "GSM" && server.hasArg("gsm_username") && server.hasArg("gsm_password") && server.hasArg("gsm_apn")) {
      strncpy(config.deviceId, deviceId.c_str(), sizeof(config.deviceId));
      strncpy(config.type, type.c_str(), sizeof(config.type));
      strncpy(config.gsm_username, server.arg("gsm_username").c_str(), sizeof(config.gsm_username));
      strncpy(config.gsm_password, server.arg("gsm_password").c_str(), sizeof(config.gsm_password));
      strncpy(config.gsm_apn, server.arg("gsm_apn").c_str(), sizeof(config.gsm_apn));
    } else {
      server.send(400, "text/html", "<html><body><h2>Invalid input. Please fill all required fields.</h2><a href='/'>Back</a></body></html>");
      return;
    }
    saveConfig();
    configSaved = true;
    server.send(200, "text/html", "<html><body><h2>Configuration saved. Rebooting...</h2></body></html>");
    // Success: Config saved (no log)
  } else {
    server.send(400, "text/html", "<html><body><h2>Invalid input. Device ID and Communication Type are required.</h2><a href='/'>Back</a></body></html>");
  }
  lastClientTime = millis();
}
// OTA submit handler
void handleOTASubmit() {
  if (!loggedIn) {
    server.send(401, "text/html", "<html><body><h2>Please login first</h2><a href='/'>Login</a></body></html>");
    return;
  }
  if (server.hasArg("ota_ssid") && server.hasArg("ota_password") && server.hasArg("ota_url")) {
    String ssid = server.arg("ota_ssid");
    String password = server.arg("ota_password");
    String otaLink = server.arg("ota_url");
    prefs.begin("my-config", false);
    prefs.putString("last_ota_ssid", ssid);
    prefs.putString("last_ota_password", password);
    prefs.putString("last_ota_link", otaLink);
    prefs.end();
    // Success: OTA creds saved (no log)
    if (performOTAUpdate(ssid, password, otaLink)) {
      server.send(200, "text/html", "<html><body><h2>OTA Update initiated. Rebooting...</h2></body></html>");
    } else {
      server.send(500, "text/html", "<html><body><h2>OTA Update failed.</h2><a href='/'>Back</a></body></html>");
    }
  } else {
    server.send(400, "text/html", "<html><body><h2>Missing OTA parameters.</h2><a href='/'>Back</a></body></html>");
  }
  lastClientTime = millis();
}
///======== BUZZER SECTION =======//
// Buzzer patterns for feedback (optimized: non-blocking where possible, but kept blocking for simplicity)
void buzzerSearching() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  // Success: Pattern played (no log)
}
void buzzerSetupMode() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
  delay(500);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
  // Success: Pattern played (no log)
}
void buzzerConnected() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(200);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  // Success: Pattern played (no log)
}
void buzzerFailure() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(1000);
  digitalWrite(BUZZER_PIN, LOW);
  delay(500);
  // Error: Pattern played (no additional log)
}
///======== SETUP AND LOOP SECTION =======//
// Add this helper function to your code
// ===== I2C BUS RECOVERY FUNCTION =====
void recoverI2C() {
  LOG_ERROR("⚠️ I2C BUS STUCK - Attempting Recovery...");

  // Release I2C bus
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
  delay(10);

  // Manually toggle SCL 9 times to flush any stuck slaves
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
  }

  // Send STOP condition
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(5);

  // Re-initialize I2C
  delay(100);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(10000);
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(100);

  LOG_ERROR("✅ I2C Bus Recovered");
}
// =====================================

// Main setup (optimized: pinned tasks for core affinity, early sensor init)
void setup() {
  esp_task_wdt_deinit();
  Serial.begin(115200);
  LOG_ERROR("Starting system initialization...");  // Initial log for boot diagnostics
  Serial.println(current_firmware_version);
  delay(1000);
  delay(1000);
  sensorSerial.setRxBufferSize(1024);
  sensorSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  pinMode(gsmRST, OUTPUT);
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  // 2. FORCE Internal Pull-ups (MUST be after Wire.begin)
  // ===== I2C INITIALIZATION (CRITICAL FOR 100CM CABLE) =====
  // ========================================================



  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  pinMode(PH_PIN, INPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(flowSensorPin, INPUT_PULLUP);
  pinMode(WIFI_LED, OUTPUT);
  pinMode(tdsPin, INPUT);
  pinMode(dataStatus, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(dataStatus, LOW);
  digitalWrite(WIFI_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  ultrasonicMutex = xSemaphoreCreateMutex();
  prefsMutex = xSemaphoreCreateMutex();
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), countPulses, FALLING);
  loadConfig();
  delay(1000);
  // Success: Distance measured (no log)
  initSensors();
  delay(3000);
  sensorMutex = xSemaphoreCreateMutex();
  // Start ultrasonic task
  xTaskCreatePinnedToCore(
    ultrasonicTask,
    "Ultrasonic Task",
    4096,
    NULL,
    1,
    NULL,
    1);
  xTaskCreatePinnedToCore(checkRelayTask, "Check Relay Task", 6144, NULL, 1, &checkRelayTaskHandle, 1);
  xTaskCreatePinnedToCore(buttonCheckTask, "Button Check Task", 6144, NULL, 1, &buttonCheckTaskHandle, 0);
  xTaskCreatePinnedToCore(flowSensorTask, "Flow Sensor Task", 8192, NULL, 1, &flowSensorTaskHandle, 0);
  // xTaskCreatePinnedToCore(sensorAvgTask, "Sensor Average Task", 4096, NULL, 1, &sensorAvgTaskHandle, 1);
  setupCommunication();
  LOG_ERROR("Ultrasonic distance: %d cm", distance);  // Boot diagnostic
  if (isGSM) {
    digitalWrite(gsmRST, HIGH);
    delay(2000);
    digitalWrite(gsmRST, LOW);
    delay(5000);
    SerialGSM.begin(115200, SERIAL_8N1, gsmRX, gsmTX);
    delay(10000);
    setupGSM();
    xTaskCreatePinnedToCore(GSMTask, "Send Data Task", 12288, NULL, 1, &sendDataTaskHandle, 0);
    // xTaskCreatePinnedToCore(checkConnectivityTask, "Check Connectivity Task", 8192, NULL, 1, &checkConnectivityTaskHandle, 0);
  } else if (isWIFI) {
    xTaskCreatePinnedToCore(WIFITask, "Send Data Task", 8192, NULL, 1, &sendDataTaskHandle, 0);
    xTaskCreatePinnedToCore(checkConnectivityTask, "Check Connectivity Task", 4048, NULL, 1, &checkConnectivityTaskHandle, 0);
  } else {
    LOG_ERROR("Invalid communication type: %s", config.type);
  }
  // Success: Setup complete (no log)
}
// Main loop with memory monitoring (optimized: watchdog disabled, heap check)
void loop() {
  monitorMemory();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
void monitorMemory() {
  size_t freeHeap = esp_get_free_heap_size();
  if (freeHeap < 10000) {
    LOG_ERROR("Low memory: %d bytes. Restarting.", freeHeap);
    ESP.restart();
  }
  // Success: Heap ok (no log)
}