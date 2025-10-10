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


int gsmFailureCount = 0;
const int GSM_MAX_FAILURES = 3;

#define MQTT_MAX_PACKET_SIZE 1024

#define CA_CERT_FILE "ca.pem"
#define CLIENT_CERT_FILE "client.pem"
#define CLIENT_KEY_FILE "private.pem" 
#define AWS_IOT_PUBLISH_TOPIC "device/" THINGNAME "/data"
#define AWS_IOT_SUBSCRIBE_TOPIC "device/" THINGNAME "/sub"
struct SensorSample {
  float temp;
  float hum;
  uint8_t AQI;
  uint16_t TVOC;
  uint16_t ECO2;
  int count;   // how many samples in the buffer
};

// Global buffers
SensorSample bufferReady  = {0};
volatile bool bufferSwap = false;  // flag: new averages ready

// Accumulators for averaging
float avgTemp = 0, avgHum = 0, avgAQI = 0, avgTVOC = 0, avgECO2 = 0;
int sampleCount = 0;

// Mutex for safe updates
SemaphoreHandle_t sensorMutex;


Preferences prefs;

unsigned long compStartTime = 0;   // when compressor started
unsigned long compressorMinutes = 0;  // cumulative minutes

unsigned long motorMinutes = 0;  // cumulative minutes


unsigned long flowStartTime = 0;
unsigned long flowDurationMinutes = 0;


#define HOLD_TIME 5000
#define LOG_LEVEL_DEBUG 0
#define LOG_LEVEL_INFO 1
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_ERROR 3
#define MIN_LOG_LEVEL LOG_LEVEL_DEBUG
#define ENABLE_MQTT_LOGGING true

bool configSaved = false;
unsigned long lastClientTime = 0;
int mqttFailureCount = 0;
bool firstRun = true;
bool isMQTTConnected = false;
bool loggedIn = false;  // Authentication flag

// NTP Server settings
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;  // IST (+5:30)
const int daylightOffset_sec = 0;
struct tm timeinfo;
unsigned long lastKeepAliveTime = 0;
const unsigned long keepAliveInterval = 60000;  // 1 minute for keep-alive
HardwareSerial SerialGSM(2);
unsigned long flowStopDelay = 5000;
static unsigned long lastFlowTime = 0;

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
  int   publishInterval;

  // 🔥 Add OTA params
  char last_ota_ssid[32];
  char last_ota_password[32];
  char last_ota_link[128];
} config;


WebServer server(80);
WiFiClientSecure net;
PubSubClient client(net);
HardwareSerial sensorSerial(1);

#define MQTT_CLIENT_ID THINGNAME
#define RXD2 47
#define TXD2 48
#define SDA_PIN 8
#define SCL_PIN 9
const int WIFI_LED = 37;
const int BUZZER_PIN = 45;
const int dataStatus = 15;
const int tdsPin = 2;
const int RESET_PIN = 36;
const int flowSensorPin = 5;
#define PH_PIN 4   //device002
#define ADC_RESOLUTION 4096
#define VREF 3300
#define V_NEUTRAL 1500
#define SLOPE -59.7
#define DISCONNECT_THRESHOLD 50
bool flowFlag = false;

// Logging
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

// Circular log buffer
#define MAX_LOGS 10
String serialLogs[MAX_LOGS];
int logIndex = 0;
// ---- forward declarations (no defaults here) ----
float readCounter(const char* key, float defaultValue);
unsigned long readCounterULong(const char* key, unsigned long defaultValue);
void saveCounter(const char* key, float value);
void saveCounterULong(const char* key, unsigned long value);

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


// Store in circular buffer
  serialLogs[logIndex] = String("[") + levelStr + "] " + logBuffer;


  logIndex = (logIndex + 1) % MAX_LOGS;
}

float neutralADC = 1925;
float acidADC = 2269;
float temperatureC = 25.0;
const int SAMPLES = 100;
const int SAMPLE_DELAY_MS = 20;

float slope25() {
    // slope = (pH7 - pH4) / (ADC@7 - ADC@4)
    return (7.0 - 4.0) / (neutralADC - acidADC);
}

SemaphoreHandle_t prefsMutex;

// Global variables
unsigned long totalFlowMinutes = 0;

uint8_t status;

float phvoltage = 0.0;
float tdsValue = 0.0;
float phValue = 0.0;
bool isGSM = false;
bool isWIFI = false;
uint8_t buffer[4];
uint16_t distance = 0;
TaskHandle_t sendDataTaskHandle;
TaskHandle_t checkRelayTaskHandle;
TaskHandle_t checkConnectivityTaskHandle;
TaskHandle_t buttonCheckTaskHandle;
TaskHandle_t flowSensorTaskHandle;
TaskHandle_t phTaskHandle;
// TaskHandle_t sensorAvgTaskHandle;
const float vRef = 3.3;
const float adcResolution = 4095;
unsigned long lastPublishTime = 0;
#define gsmRX 17
#define gsmTX 18
#define gsmRST 14
// #define gsmRST 48
bool shtInitialized = false;
bool ensInitialized = false;
const float tdsFactor = 0.5;
unsigned long lastRelayOffTime = 0;
volatile uint32_t pulseCount = 0;
float flowRate;
float totalLiters = 0.0;
const float calibrationFactor = 450.0;
#define RELAY1_PIN 21
#define RELAY2_PIN 38
SHTSensor sht;
  float temperature = NAN, humidity = NAN;

    uint8_t AQI = 0;
    uint16_t TVOC = 0, ECO2 = 0;
bool isENS = true;
DFRobot_ENS160_I2C ens160(&Wire, 0x53);
const char* current_firmware_version = "1.4.1";

void IRAM_ATTR countPulses() {
  pulseCount++;
}

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


void setup() {
  esp_task_wdt_deinit();


  Serial.begin(115200);
  LOG_INFO("Starting system initialization...");
  Serial.println(current_firmware_version);
  delay(1000);
    delay(1000);
  sensorSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(gsmRST, OUTPUT);
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
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
  

prefsMutex = xSemaphoreCreateMutex();
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), countPulses, RISING);
loadConfig();
  delay(1000);
  
  measureUltrasonicDistance();
  LOG_INFO("Initializing ENS160 Sensor...");
  initSensors();
  delay(3000);

  sensorMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(checkRelayTask, "Check Relay Task", 6144, NULL, 1, &checkRelayTaskHandle, 1);
  xTaskCreatePinnedToCore(buttonCheckTask, "Button Check Task", 6144, NULL, 1, &buttonCheckTaskHandle, 0);
  xTaskCreatePinnedToCore(flowSensorTask, "Flow Sensor Task", 4096, NULL, 1, &flowSensorTaskHandle, 0);

  // xTaskCreatePinnedToCore(sensorAvgTask, "Sensor Average Task", 4096, NULL, 1, &sensorAvgTaskHandle, 1);
  setupCommunication();
  measureUltrasonicDistance();
  LOG_DEBUG("Ultrasonic distance: %d cm", distance);
  if (isGSM) {
    digitalWrite(gsmRST, HIGH);
    delay(2000);
    digitalWrite(gsmRST, LOW);
    delay(5000);
    SerialGSM.begin(115200, SERIAL_8N1, gsmRX, gsmTX);
    delay(10000);
    LOG_INFO("Setting up GSM...");
    setupGSM();
    xTaskCreatePinnedToCore(GSMTask, "Send Data Task", 12288, NULL, 1, &sendDataTaskHandle, 0);
    // xTaskCreatePinnedToCore(checkConnectivityTask, "Check Connectivity Task", 8192, NULL, 1, &checkConnectivityTaskHandle, 0);
  } else if (isWIFI) {
    LOG_INFO("Setting up WiFi...");
    xTaskCreatePinnedToCore(WIFITask, "Send Data Task", 8192, NULL, 1, &sendDataTaskHandle, 0);
    xTaskCreatePinnedToCore(checkConnectivityTask, "Check Connectivity Task", 2024, NULL, 1, &checkConnectivityTaskHandle, 0);
  } else {
    LOG_ERROR("Invalid communication type: %s", config.type);
  }
}


bool initSensors() {
  LOG_INFO("Initializing SHT20 Sensor...");
  if (sht.init()) {
    LOG_INFO("SHT20 initialization successful");
    sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
    shtInitialized = true;
  } else {
    LOG_ERROR("SHT20 initialization failed");
    shtInitialized = false;
  }

  LOG_INFO("Initializing ENS160 Sensor...");
  int attempt = 0;
  while (attempt < 5) {
    if (NO_ERR == ens160.begin()) {
      LOG_INFO("ENS160 initialization successful!");
      ensInitialized = true;
      break;
    } else {
      LOG_WARN("ENS160 communication failed, attempt %d/5", attempt + 1);
      attempt++;
      delay(3000);
    }
  }

  if (!ensInitialized) {
    LOG_ERROR("Failed to initialize ENS160 after 5 attempts");
    isENS = false;
  } else {
    ens160.setPWRMode(ENS160_STANDARD_MODE);
    delay(5000);
  }

  return shtInitialized && ensInitialized;
}
void loop() {
  monitorMemory();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void monitorMemory() {
  size_t freeHeap = esp_get_free_heap_size();
  if (freeHeap < 10000) {
    LOG_WARN("Low memory: %d bytes. Restarting.", freeHeap);
    ESP.restart();
  }
}

void buttonCheckTask(void* pvParameters) {
  LOG_INFO("Button Check Task started");
  while (true) {
    if (isButtonPressed()) {
      LOG_INFO("Button pressed, entering setup mode");
      buzzerSetupMode();
      setupServer();
      LOG_INFO("Exiting setup mode");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


// ------------------- pH / TDS TASK -------------------
// ------------------- pH / TDS TASK -------------------
void phTdsTask(void* pvParameters) {
    unsigned long startTime = millis();
    LOG_INFO("🧪 pH/TDS Task started at %lu ms", startTime);

    if (!flowFlag) {
        LOG_WARN("⚠️ Flow stopped before 3s → skipping pH/TDS read");
        vTaskDelete(NULL);
        return;
    }

    LOG_DEBUG("Checking continuous flow for 5 seconds...");
    while (millis() - startTime < 5000) {
        if (!flowFlag) {
            LOG_WARN("⚠️ Flow ended before 5s → skipping pH/TDS read");
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    LOG_INFO("✅ Continuous flow detected ≥5s → waiting 10s for stabilization...");
    vTaskDelay(10000 / portTICK_PERIOD_MS);  // let sensor stabilize in flowing water

    float localTDS = NAN;
    float localPH = NAN;

    // --- Retry TDS ---
    for (int i = 0; i < 3; i++) {
        readTDSSensor(25.0);
        LOG_DEBUG("TDS attempt %d → Raw: %.2f", i + 1, tdsValue);
        if (tdsValue >= 0 && tdsValue <= 2000) {
            localTDS = tdsValue;
            break;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    // --- Stabilized pH Reading ---
    localPH = stableReadPH();

    // --- Update globals only if valid ---
    if (!isnan(localTDS)) {
        tdsValue = localTDS;
        saveCounter("tdsValue", tdsValue);
        LOG_INFO("💧 Valid TDS = %.2f ppm", tdsValue);
    } else {
        saveCounter("tdsValue", NAN);
        LOG_WARN("❌ TDS invalid after retries → stored NULL");
    }

    if (!isnan(localPH)) {
        phValue = localPH;
        saveCounter("phValue", phValue);
        LOG_INFO("🧪 Valid pH = %.2f", phValue);
    } else {
        saveCounter("phValue", NAN);
        LOG_WARN("❌ pH invalid after stabilization → stored NULL");
    }

    LOG_INFO("✅ pH/TDS Task finished → deleting itself");
    vTaskDelete(NULL);
}


// ------------------- STABLE pH READING LOGIC -------------------
float stableReadPH() {
    LOG_INFO("🔬 Starting stabilized pH reading...");

    float lastPH = 0.0, currentPH = 0.0;
    int stableCount = 0;
    int maxAttempts = 15;  // max ~15 seconds
    int attempt = 0;

    while (attempt < maxAttempts && stableCount < 3) {
        readPHSensor();
        currentPH = phValue;

        LOG_DEBUG("pH[%d] = %.2f", attempt + 1, currentPH);

        if (attempt > 0 && fabs(currentPH - lastPH) < 0.05) {
            stableCount++;
        } else {
            stableCount = 0;  // reset if unstable
        }

        lastPH = currentPH;
        attempt++;

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    if (stableCount >= 3 && currentPH > 0 && currentPH <= 14) {
        LOG_INFO("✅ pH stabilized at %.2f after %d samples", currentPH, attempt);
        return currentPH;
    } else {
        LOG_WARN("❌ pH failed to stabilize within %d attempts", attempt);
        return NAN;
    }
}


void flowSensorTask(void* pvParameters) {
  static unsigned long lastSensorReadTime = 0;
  LOG_INFO("Flow Sensor Task started");

  // Restore counters on boot
  totalLiters   = readCounter("totalLiters", 0.0f);
  motorMinutes  = readCounterULong("motorMinutes", 0);

  while (true) {
    noInterrupts();
    uint32_t currentPulseCount = pulseCount;
    pulseCount = 0;
    interrupts();

    flowRate = ((float)currentPulseCount / calibrationFactor) * 60.0;
    float totalLiters1 = (float)currentPulseCount / calibrationFactor;
    totalLiters += totalLiters1 / 10;

    if (flowRate > 0) {
      lastFlowTime = millis();

      if (!flowFlag) {
        flowFlag = true;
        digitalWrite(RELAY2_PIN, HIGH);
        flowStartTime = millis();
        LOG_INFO("💧 Water flow started, Relay 2 ON");

        // Start pH/TDS read task
        xTaskCreatePinnedToCore(
          phTdsTask,
          "phTdsTask",
          4096,
          NULL,
          1,
          &phTaskHandle,
          1
        );
      }
    } 
    else if (flowFlag && (millis() - lastFlowTime >= flowStopDelay)) {
      flowFlag = false;
      digitalWrite(RELAY2_PIN, LOW);

      unsigned long flowDurationMs = millis() - flowStartTime;
      unsigned long flowDurationMinutes = flowDurationMs / 60000;
      motorMinutes += flowDurationMinutes;

      saveCounter("totalLiters", totalLiters);
      saveCounterULong("motorMinutes", motorMinutes);

      LOG_INFO("💧 Water flow stopped, Relay 2 OFF");
      LOG_INFO("📊 Total liters saved: %.2f | Flow duration: %lu min | Cumulative motor minutes: %lu",
                totalLiters, flowDurationMinutes, motorMinutes);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void checkRelayTask(void* pvParameters) {
  static int lastRelayState = digitalRead(RELAY1_PIN);
  static unsigned long lastMinuteCheck = 0;
  LOG_INFO("Check Relay Task started");

  compressorMinutes = readCounterULong("compMinutes", 0);

  if (lastRelayState == LOW) {
    compStartTime = millis();
    lastMinuteCheck = compStartTime;
  }

  while (true) {
    measureUltrasonicDistance();
    getLatestReadings(temperature, humidity, AQI, TVOC, ECO2);

    // --- Validate sensor ranges ---
    bool tempValid = (!isnan(temperature) && temperature >= -40 && temperature <= 125 && temperature != 0.0);
    bool humValid  = (!isnan(humidity)    && humidity >= 1   && humidity <= 100);
    bool distValid = (distance > 0 && distance < 500);

    bool tempOk = tempValid &&
                  (temperature >= config.tempThresholdMin && temperature <= config.tempThresholdMax);
    bool humOk  = humValid &&
                  (humidity   >= config.humidityThresholdMin && humidity   <= config.humidityThresholdMax);

    // --- OFF conditions (tank full → always allowed) ---
    if (distance <= config.compressorOffLevel) {
      if (lastRelayState != HIGH) {
        digitalWrite(RELAY1_PIN, HIGH);
        lastRelayState = HIGH;
        lastRelayOffTime = millis();
        compStartTime = 0;
        LOG_WARN("Relay OFF → Compressor stopped (Reason: Tank full)");
      }
    }

    // --- ON / OFF control (only when valid sensors) ---
    else if (tempValid && humValid && distValid) {
      // --- OFF for bad environmental conditions ---
      if (!tempOk || !humOk) {
        if (lastRelayState != HIGH) {
          digitalWrite(RELAY1_PIN, HIGH);
          lastRelayState = HIGH;
          lastRelayOffTime = millis();
          compStartTime = 0;
          LOG_WARN("Relay OFF → Compressor stopped (Reason: Env out of range)");
        }
      }
      // --- ON when all ok ---
      else if (distance >= config.compressorOnLevel && tempOk && humOk) {
        if (lastRelayState != LOW) {
          if (millis() - lastRelayOffTime >= 180000) {
            digitalWrite(RELAY1_PIN, LOW);
            lastRelayState = LOW;
            compStartTime = millis();
            lastMinuteCheck = compStartTime;
            LOG_INFO("Relay ON → Compressor started");
          } else {
            LOG_DEBUG("Relay ON blocked (waiting 3 min since OFF)");
          }
        }
      }
    } else {
      // --- Invalid sensor → do not turn ON, but allow OFF already handled above ---
      LOG_WARN("Sensor invalid, holding current state (T=%.2f, H=%.2f, D=%d)",
               temperature, humidity, distance);
    }

    // --- Per-minute runtime tracking ---
    if (lastRelayState == LOW && (millis() - lastMinuteCheck >= 60000)) {
      compressorMinutes++;
      saveCounterULong("compMinutes", compressorMinutes);
      lastMinuteCheck += 60000;
      LOG_DEBUG("Compressor runtime incremented: %lu minutes", compressorMinutes);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}




void saveCounter(const char* key, float value) {
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("counters", false);  // write mode
    prefs.putFloat(key, value);
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }
}

void saveCounterULong(const char* key, unsigned long value) {
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("counters", false);  // write mode
    prefs.putULong(key, value);
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }
}

float readCounter(const char* key, float defaultValue = 0.0f) {
  float value = defaultValue;
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("counters", true);   // read mode
    value = prefs.getFloat(key, defaultValue);
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }
  return value;
}

unsigned long readCounterULong(const char* key, unsigned long defaultValue = 0) {
  unsigned long value = defaultValue;
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("counters", true);   // read mode
    value = prefs.getULong(key, defaultValue);
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }
  return value;
}
bool getLatestReadings(float &temperature, float &humidity,
                       uint8_t &aqi, uint16_t &tvoc, uint16_t &eco2) {

  // --- Check SHT20 initialization ---
  if (!shtInitialized) {
    LOG_WARN("SHT20 not initialized. Attempting reinitialization...");
    shtInitialized = sht.init();
    if (shtInitialized) {
      LOG_INFO("SHT20 reinitialized successfully.");
      sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
    } else {
      LOG_ERROR("SHT20 reinitialization failed.");
    }
  }

  // --- Read SHT20 sensor ---
  if (shtInitialized && sht.readSample()) {
    temperature = sht.getTemperature();
    humidity    = sht.getHumidity();
  } else {
    temperature = humidity = NAN;
    LOG_WARN("Failed to read from SHT20 sensor.");
  }

  // --- Check ENS160 initialization ---
  if (!ensInitialized) {
    LOG_WARN("ENS160 not initialized. Attempting reinitialization...");
    if (NO_ERR == ens160.begin()) {
      LOG_INFO("ENS160 reinitialized successfully!");
      ens160.setPWRMode(ENS160_STANDARD_MODE);
      ens160.setTempAndHum(25.0, 50.0);  // Initial default
      ensInitialized = true;
      delay(2000);
    } else {
      LOG_ERROR("ENS160 reinitialization failed.");
      ensInitialized = false;
    }
  }

  // --- Send T/H to ENS160 ---
  if (ensInitialized) {
    if (!isnan(temperature) && !isnan(humidity)) {
      ens160.setTempAndHum(temperature, humidity);
      LOG_INFO("Sent T=%.2f°C, H=%.2f%% to ENS160", temperature, humidity);
    } else {
      ens160.setTempAndHum(25.0, 50.0);
      LOG_WARN("Using default T/H values for ENS160 compensation");
    }
  }

  // --- Read ENS160 data ---
  static uint8_t lastAQI = 1;
  static uint16_t lastTVOC = 0, lastECO2 = 400;

  uint8_t localAQI = lastAQI;
  uint16_t localTVOC = lastTVOC, localECO2 = lastECO2;

  if (ensInitialized) {
    readENS160Sensor(localAQI, localTVOC, localECO2);
    lastAQI = localAQI;
    lastTVOC = localTVOC;
    lastECO2 = localECO2;
  }

  aqi  = localAQI;
  tvoc = localTVOC;
  eco2 = localECO2;

  return (shtInitialized && ensInitialized);
}


// --- ENS160 robust read function ---
void readENS160Sensor(uint8_t &outAQI, uint16_t &outTVOC, uint16_t &outECO2) {
  static unsigned long lastRetry = 0;
  int status = ens160.getENS160Status();
  LOG_INFO("ENS160 Status: %d", status);

  switch (status) {
    case 1: // Valid data
      isENS = true;
      outAQI  = ens160.getAQI();
      outTVOC = ens160.getTVOC();
      outECO2 = ens160.getECO2();
      LOG_INFO("ENS160 Data OK → AQI: %d, TVOC: %d ppb, eCO2: %d ppm",
               outAQI, outTVOC, outECO2);
      break;

    case 2: // Warm-up
      isENS = true; // Sensor is running, data may still be good
      outAQI  = ens160.getAQI();
      outTVOC = ens160.getTVOC();
      outECO2 = ens160.getECO2();
      LOG_INFO("ENS160 warming up → AQI: %d, TVOC: %d, eCO2: %d",
               outAQI, outTVOC, outECO2);
      break;

    case 0: // Not initialized
    case 3: // Invalid data
    default:
      isENS = false;
      if (millis() - lastRetry > 5000) {
        LOG_WARN("ENS160 invalid/uninitialized. Attempting reinit...");
        ens160.begin();
        ens160.setPWRMode(ENS160_STANDARD_MODE);
        ens160.setTempAndHum(25.0, 50.0);
        lastRetry = millis();
        LOG_INFO("ENS160 reinitialized.");
      }
      break;
  }
}




// // -------------------- Get Latest Averages --------------------
// bool getLatestReadings(float &temperature, float &humidity,
//                        uint8_t &aqi, uint16_t &tvoc, uint16_t &eco2) {
//   if (bufferSwap && bufferReady.count > 0) {
//     if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
//       temperature = bufferReady.temp;
//       humidity    = bufferReady.hum;
//       aqi         = bufferReady.AQI;
//       tvoc        = bufferReady.TVOC;
//       eco2        = bufferReady.ECO2;
//       bufferSwap  = false;  // consume data
//       xSemaphoreGive(sensorMutex);
//     }
//     return true;
//   }
//   return false;
// }

bool hasInternetConnection() {
  WiFiClient client;
  return client.connect("8.8.8.8", 53);
}

void checkConnectivityTask(void* pvParameters) {
  static unsigned long lastRetryTime = 0;
  const unsigned long retryInterval = 180000;
  while (true) {
    if (isWIFI) {
      if (WiFi.status() != WL_CONNECTED || !hasInternetConnection()) {
        LOG_WARN("WiFi disconnected or no internet. Attempting to reconnect...");
        buzzerSearching();
        WiFi.disconnect();
        delay(1000);
        if (connectWiFi()) {
          LOG_INFO("WiFi reconnected.");
          buzzerConnected();
          delay(3000);
          client.disconnect();
          connectAWS();
          if (!client.connected()) {
            Serial.print("MQTT connection failed, state: ");
            Serial.println(client.state());
          }
        } else {
          LOG_ERROR("WiFi reconnection failed.");
          buzzerFailure();
          if (millis() - lastRetryTime >= retryInterval) {
            lastRetryTime = millis();
            LOG_INFO("Starting setup server due to WiFi failure.");
            setupServer();
          }
        }
      }
    } 
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void WIFITask(void* pvParameters) {
  const TickType_t keepAlivePeriod = keepAliveInterval / portTICK_PERIOD_MS;
  const TickType_t publishPeriod = config.publishInterval / portTICK_PERIOD_MS;
  LOG_INFO("WiFi Task started");
  connectAWS();
  TickType_t lastPublishTime = xTaskGetTickCount();
  TickType_t lastKeepAliveTime = xTaskGetTickCount();
  bool isPublishing = false;
  publishKeepAlive();
  vTaskDelay(10000 / portTICK_PERIOD_MS);
  sendViaWIFI();
  while (true) {
    if (!client.connected()) {
      connectAWS();
    }
    client.loop();
    TickType_t now = xTaskGetTickCount();
    if (!isPublishing && now - lastPublishTime >= publishPeriod) {
      isPublishing = true;
      LOG_INFO("Triggering data publish via WiFi");
      sendViaWIFI();
      lastPublishTime = xTaskGetTickCount();
      isPublishing = false;
    }
    if (!isPublishing && now - lastKeepAliveTime >= keepAlivePeriod) {
      isPublishing = true;
      LOG_DEBUG("Triggering keep-alive publish");
      publishKeepAlive();
      lastKeepAliveTime = xTaskGetTickCount();
      isPublishing = false;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void GSMTask(void* pvParameters) {
  const TickType_t keepAlivePeriod = keepAliveInterval / portTICK_PERIOD_MS;
  const TickType_t publishPeriod = config.publishInterval / portTICK_PERIOD_MS;
  // const TickType_t publishPeriod = 180000 / portTICK_PERIOD_MS;
  LOG_INFO("GSM Task started");
  TickType_t lastPublishTime = xTaskGetTickCount();
  TickType_t lastKeepAliveTime = xTaskGetTickCount();
  publishKeepAlive();
  vTaskDelay(10000 / portTICK_PERIOD_MS);
  sendViaGSM();
  while (true) {
    TickType_t now = xTaskGetTickCount();
    if (now - lastKeepAliveTime >= keepAlivePeriod) {
      LOG_DEBUG("Triggering keep-alive publish");
      publishKeepAlive();
      lastKeepAliveTime = now;
    }
    if (now - lastPublishTime >= publishPeriod) {
      LOG_INFO("Triggering data publish via GSM");
      sendViaGSM();
      lastPublishTime = now;
    }
     // 🔹 Recovery logic
    if (gsmFailureCount >= GSM_MAX_FAILURES) {
      LOG_ERROR("GSM failed %d times. Restarting GSM...", gsmFailureCount);

      gsmFailureCount = 0;   // reset counter
      setupGSM();            // reboot modem + reconnect MQTT

      if (!isMQTTConnected) {
        LOG_ERROR("Still no GSM after reboot. Starting setup server...");
        setupServer();  // ⬅️ manual reconfiguration fallback
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}




void publishKeepAlive() {
  LOG_DEBUG("Publishing keep-alive message");
  if (isWIFI && !client.connected()) {
    LOG_WARN("WiFi MQTT disconnected, attempting reconnect");
    client.disconnect();
    connectAWS();
  } else if (isGSM && !checkMQTTConnection()) {
    LOG_WARN("GSM MQTT disconnected, attempting reconnect");
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
      Serial.print("Current UTC timestamp: ");
      Serial.println(timestamp);
    } else {
      Serial.println("Failed to update time");
      timestamp = 123456789;
    }
  } else if (isGSM) {
    timestamp = getUnixTimestamp();
  }
  if (timestamp == 0) {
    LOG_WARN("Failed to get timestamp, using millis()");
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
  Serial.println("Payload:");
  Serial.println(jsonBuffer);
  bool success = false;
  int retryDelay = 100;
  for (int i = 0; i < 3; i++) {
    if (isWIFI && client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer.c_str())) {
      success = true;
      break;
    } else if (isGSM && publishMQTTMessage(jsonBuffer)) {
      success = true;
      break;
    } else {
      LOG_WARN("Keep-alive publish failed (Attempt %d/3)", i + 1);
      delay(retryDelay);
      retryDelay *= 2;
      if (isWIFI && !client.connected()) {
        LOG_WARN("WiFi MQTT disconnected, forcing reconnect");
        WiFi.disconnect(true);
        delay(1000);
        connectWiFi();
        client.disconnect();
        delay(500);
        connectAWS();
      } else if (isGSM && !checkMQTTConnection()) {
        LOG_WARN("GSM MQTT disconnected, reconnecting");
        connectMQTT();
      }
    }
  }
  if (success) {
    LOG_INFO("Keep-alive published successfully");
  } else {
    LOG_ERROR("All keep-alive publish attempts failed");
    mqttFailureCount++;
  }
}

void connectAWS() {
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
  client.setServer(AWS_IOT_ENDPOINT, 8883);
  Serial.println("Connecting to AWS IoT");
  int attempts = 0;
  while (!client.connected()) {
    Serial.print("Connecting to AWS IoT...");
    if (client.connect(MQTT_CLIENT_ID)) {
      Serial.println("Connected to AWS IoT");
      client.setBufferSize(1024);
    } else {
      Serial.print("Failed: ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("Incoming: ");
  Serial.println(topic);
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}

void setupCommunication() {
  Serial.println("Setting up communication...");

  if (strcmp(config.type, "GSM") == 0) {
    Serial.println("Communication type: GSM");
    isGSM = true;

    SerialGSM.begin(115200, SERIAL_8N1, gsmRX, gsmTX);

  } else if (strcmp(config.type, "WiFi") == 0) {
    Serial.println("Communication type: WiFi");
    isWIFI = true;

    if (!connectWiFi()) {
      Serial.println("WiFi connection failed. Starting in setup mode.");
      setupServer();
      return;
    }

    // If WiFi connected, sync time
    delay(4000);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    bool timeSynced = false;
    int retries = 0;
    while (!timeSynced && retries < 3) {
      if (getLocalTime(&timeinfo)) {
        timeSynced = true;
      } else {
        Serial.printf("Failed to obtain time. Retry %d\n", retries + 1);
        retries++;
        delay(2000);
      }
    }

    if (!timeSynced) {
      Serial.println("Time synchronization failed after 3 attempts.");
      return;
    }

    char timeStringBuff[50];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Serial.print("Current time: ");
    Serial.println(timeStringBuff);

    Serial.println("✅ Connected to WiFi successfully.");
    buzzerConnected();

  } else {
    Serial.println("❌ Invalid communication type. Starting in setup mode.");
    setupServer();
  }
}

void setupGSM() {
    digitalWrite(gsmRST, HIGH);
        delay(2000);
        digitalWrite(gsmRST, LOW);
        delay(5000);
  Serial.println("Initializing GSM...");
  String response = sendATCommandGetResponse("AT");
  if (response.indexOf("OK") < 0) {
    Serial.println("No response from modem. Check connections.");
  }
  sendATCommandGetResponse("ATE0");
  sendATCommandGetResponse("AT+CREG=1");
  delay(1000);
  Serial.println("\nConnecting to network...");
  String netStat = sendATCommandGetResponse("AT+CREG?");
  if (netStat.indexOf("+CREG: 1,1") < 0 && netStat.indexOf("+CREG: 1,5") < 0) {
    Serial.println("Network registration failed!");
  }
  sendATCommandGetResponse("AT+CGATT=1");
  sendATCommandGetResponse("AT+CGDCONT=1,\"IP\",\"" + String(config.gsm_apn) + "\"");
  sendATCommandGetResponse("AT+CGACT=1,1");
  Serial.println("Connected to GPRS.");
  sendATCommandGetResponse("AT+CCERTLIST");
  sendATCommandGetResponse("AT+CCERTDELE=\"" + String(CA_CERT_FILE) + "\"");
  sendATCommandGetResponse("AT+CCERTDELE=\"" + String(CLIENT_CERT_FILE) + "\"");
  sendATCommandGetResponse("AT+CCERTDELE=\"" + String(CLIENT_KEY_FILE) + "\"");
  Serial.println("Uploading certificates...");
  if (!uploadCertificate(CA_CERT_FILE, AWS_CERT_CA, strlen(AWS_CERT_CA))) {
    Serial.println("Failed to upload CA certificate");
  }
  if (!uploadCertificate(CLIENT_CERT_FILE, AWS_CERT_CRT, strlen(AWS_CERT_CRT))) {
    Serial.println("Failed to upload client certificate");
  }
  if (!uploadCertificate(CLIENT_KEY_FILE, AWS_CERT_PRIVATE, strlen(AWS_CERT_PRIVATE))) {
    Serial.println("Failed to upload client key");
  }
  Serial.println("Configuring SSL...");
  sendATCommandGetResponse("AT+CSSLCFG=\"sslversion\",0,3");
  sendATCommandGetResponse("AT+CSSLCFG=\"authmode\",0,2");
  sendATCommandGetResponse("AT+CSSLCFG=\"cacert\",0,\"" + String(CA_CERT_FILE) + "\"");
  sendATCommandGetResponse("AT+CSSLCFG=\"clientcert\",0,\"" + String(CLIENT_CERT_FILE) + "\"");
  sendATCommandGetResponse("AT+CSSLCFG=\"clientkey\",0,\"" + String(CLIENT_KEY_FILE) + "\"");
  Serial.println("Connecting to AWS IoT via MQTT...");
  if (connectMQTT()) {
    Serial.println("Setup complete.");
    buzzerConnected();
  } else {
    Serial.println("Failed to connect to AWS IoT via MQTT");
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
        return true;
      }
      if (dataResp.indexOf("ERROR") >= 0) {
        return false;
      }
    }
  }
  return false;
}

bool connectMQTT() {
  Serial.println("Starting MQTT service...");
  String response = sendATCommandGetResponse("AT+CMQTTSTART", 12000);
  if (response.indexOf("OK") < 0) {
    Serial.println("Failed to start MQTT service");
    return false;
  }
  delay(1000);
  response = sendATCommandGetResponse("AT+CMQTTACCQ=0,\"" + String(MQTT_CLIENT_ID) + "\",1", 5000);
  if (response.indexOf("OK") < 0) {
    Serial.println("Failed to acquire MQTT client");
    sendATCommandGetResponse("AT+CMQTTSTOP", 5000);
    delay(1000);
    return false;
  }
  response = sendATCommandGetResponse("AT+CMQTTSSLCFG=0,0", 5000);
  if (response.indexOf("OK") < 0) {
    Serial.println("SSL config failed - trying anyway");
  }
  Serial.println("Connecting to AWS IoT...");
  response = sendATCommandGetResponse("AT+CMQTTCONNECT=0,\"tcp://" + String(AWS_IOT_ENDPOINT) + ":8883\",60,1", 30000);
  delay(5000);
  if (response.indexOf("OK") < 0 && response.indexOf("+CMQTTCONNECT") < 0) {
    Serial.println("Connection failed");
    sendATCommandGetResponse("AT+CMQTTREL=0", 5000);
    sendATCommandGetResponse("AT+CMQTTSTOP", 5000);
    return false;
  }
  lastKeepAliveTime = millis();
  isMQTTConnected = true;
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
// Days before each month in a non-leap year
const int daysBeforeMonth[] = {
  0,   // Jan
  31,  // Feb
  59,  // Mar
  90,  // Apr
  120, // May
  151, // Jun
  181, // Jul
  212, // Aug
  243, // Sep
  273, // Oct
  304, // Nov
  334  // Dec
};

// Check leap year
bool isLeap(int y) {
  return ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0));
}

// Convert date-time to Unix epoch (ignores TZ correction here)
unsigned long toEpoch(int year, int month, int day, int hour, int minute, int second) {
  long days = 0;

  // Days from complete years
  for (int y = 1970; y < year; y++) {
    days += isLeap(y) ? 366 : 365;
  }

  // Days from complete months
  days += daysBeforeMonth[month - 1];
  if (month > 2 && isLeap(year)) {
    days += 1;
  }

  // Days of current month
  days += (day - 1);

  // Convert to seconds
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

    Serial.println("GSM Time Response: " + response);

    int index = response.indexOf("+CCLK: \"");
    if (index == -1 || response.length() < index + 22) {
      delay(2000);
      continue;
    }

    String timeString = response.substring(index + 8, index + 27);
    Serial.println("Parsed Time String: " + timeString);

    int year   = 2000 + timeString.substring(0, 2).toInt();
    int month  = timeString.substring(3, 5).toInt();
    int day    = timeString.substring(6, 8).toInt();
    int hour   = timeString.substring(9, 11).toInt();
    int minute = timeString.substring(12, 14).toInt();
    int second = timeString.substring(15, 17).toInt();

    // Parse timezone offset (+ or - in quarters of an hour)
    char sign = timeString.charAt(17);
    int tzVal = timeString.substring(18, 20).toInt();
    int offsetMinutes = tzVal * 15;
    if (sign == '-') offsetMinutes = -offsetMinutes;
    long offsetSeconds = offsetMinutes * 60L;

    if (year < 2020 || month == 0 || day == 0) {
      delay(2000);
      continue;
    }

    // Convert local time → epoch
    unsigned long localEpoch = toEpoch(year, month, day, hour, minute, second);

    // Adjust to UTC
    unsigned long utcEpoch = localEpoch - offsetSeconds;

    // Debug prints
    Serial.printf("Parsed Local: %04d-%02d-%02d %02d:%02d:%02d (offset %c%d)\n",
                  year, month, day, hour, minute, second, sign, tzVal);
    Serial.print("Epoch (UTC): ");
    Serial.println(utcEpoch);

    return utcEpoch;
  }

  return 0; // Failed
}

bool checkMQTTConnection() {
  if (!isMQTTConnected) {
    return false;
  }
  if (millis() - lastKeepAliveTime > keepAliveInterval) {
    String response = sendATCommandGetResponse("AT+CMQTTPUB=0,0,60", 5000);
    if (response.indexOf("ERROR") >= 0 || response.indexOf("+CMQTTPUB: 0,") < 0) {
      Serial.println("MQTT connection lost during keep-alive check");
      isMQTTConnected = false;
      return false;
    }
    lastKeepAliveTime = millis();
    Serial.println("Keep-alive ping successful");
  }
  return true;
}
void sendViaGSM() {
  if (!checkMQTTConnection()) {
    LOG_WARN("MQTT reconnection needed");
    sendATCommandGetResponse("AT+CMQTTDISC=0,120", 5000);
    sendATCommandGetResponse("AT+CMQTTREL=0", 5000);
    sendATCommandGetResponse("AT+CMQTTSTOP", 5000);
    delay(2000);
    isMQTTConnected = false;
    if (!connectMQTT()) {
      LOG_ERROR("Failed to reconnect MQTT. Will retry later.");
      mqttFailureCount++;
    }
  }

  measureUltrasonicDistance();
  unsigned long timestamp = getUnixTimestamp();
  if (timestamp == 0) {
    LOG_WARN("Failed to get Unix timestamp, using millis()");
    timestamp = millis() / 1000;
  }

  DynamicJsonDocument doc(1024);

  doc["TimeStamp"] = timestamp;
  doc["DeviceId"] = config.deviceId;
  doc["DeviceName"] = "AeroneroControlSystem";
  doc["DeviceFirmwareVersion"] = current_firmware_version;

  // Pull the latest averages
  if (!getLatestReadings(temperature, humidity, AQI, TVOC, ECO2)) {


  }

  // ✅ Range validations
  if (isnan(temperature) || temperature < -40 || temperature > 125 || temperature == 0.0) doc["Temperature"] = nullptr;
  else doc["Temperature"] = temperature;

  if (isnan(humidity) || humidity < 1 || humidity > 100) doc["Humidity"] = nullptr;
  else doc["Humidity"] = humidity;

  if (distance < 3 || distance > 500) doc["WaterTankLevel"] = nullptr;
  else doc["WaterTankLevel"] = distance;
   // ✅ Always load from prefs
    float savedPh  = readCounter("phValue", NAN);
    float savedTds = readCounter("tdsValue", NAN);

  if (savedPh < 1 || savedPh > 14) doc["PhValue"] = nullptr;
  else doc["PhValue"] = savedPh;

  if (savedTds < 1 || savedTds > 1000) doc["Tds"] = nullptr;
  else doc["Tds"] = savedTds;

  if (AQI < 1 || AQI > 5) doc["AQI"] = nullptr;
  else doc["AQI"] = AQI;

  if (TVOC < 0 || TVOC > 65000) doc["TVOC"] = nullptr;
  else doc["TVOC"] = TVOC;

  if (ECO2 < 400 || ECO2 > 65000) doc["ECO2"] = nullptr;
  else doc["ECO2"] = ECO2;

  // ✅ Use generic read functions
  float savedLiters = readCounter("totalLiters", 0.0f);
  unsigned long motor = readCounterULong("motorMinutes", 0);
  unsigned long compressor = readCounterULong("compMinutes", 0);

  float litersToSend = roundf(savedLiters * 1000) / 1000.0f;
  if (fabs(litersToSend) < 0.001) litersToSend = 0.0f;

  doc["WaterTotalizer"] = litersToSend;
  doc["Compressor"] = compressor;
  doc["Motor"] = motor;
  doc["DataCollectionInterval"] = config.publishInterval/60000;
  doc["Timezone"] = "IST-05:30";
  doc["TimezoneCountry"] = "Asia/Kolkata";
  doc["TimeAutoSync"] = "Enabled";
  doc["MQTTEndpoint"] = "a1brrleef337f0-ats.iot.ap-south-1.amazonaws.com";
  doc["MQTTPort"] = "8883";
  doc["AirQualityRegulationRegion"] = "India";
  String jsonBuffer;
  serializeJson(doc, jsonBuffer);

  bool success = false;
  int retryDelay = 100;

  for (int i = 0; i < 3; i++) {
    if (publishMQTTMessage(jsonBuffer)) {
      success = true;
      break;
    } else {
      LOG_WARN("MQTT Publish failed (Attempt %d/3)", i + 1);
      delay(retryDelay);
      retryDelay *= 2;
      if (!checkMQTTConnection()) {
        connectMQTT();
      }
    }
  }

  if (success) {
    LOG_INFO("MQTT Publish Success: %s", jsonBuffer.c_str());


    mqttFailureCount = 0;

    // ✅ Reset counters with generic save functions
    saveCounter("totalLiters", 0.0f);
    saveCounterULong("motorMinutes", 0);
    saveCounterULong("compMinutes", 0);
    motorMinutes = 0;
compressorMinutes = 0;

  } else {
    LOG_ERROR("All MQTT publish attempts failed.");
    mqttFailureCount++;
  }

  lastKeepAliveTime = millis();
}


bool publishMQTTMessage(String message) {
  if (!isMQTTConnected) {
    Serial.println("MQTT not connected. Cannot publish.");
    gsmFailureCount++;
    return false;
  }

  String response = sendATCommandGetResponse("AT+CMQTTTOPIC=0," + String(strlen(AWS_IOT_PUBLISH_TOPIC)), 5000);
  if (response.indexOf(">") < 0) {
    Serial.println("Failed to enter topic data mode");
    isMQTTConnected = false;
    gsmFailureCount++;
    return false;
  }

  SerialGSM.print(AWS_IOT_PUBLISH_TOPIC);
  response = waitForResponse(5000);
  if (response.indexOf("OK") < 0) {
    Serial.println("Failed to set topic");
    isMQTTConnected = false;
    gsmFailureCount++;
    return false;
  }

  response = sendATCommandGetResponse("AT+CMQTTPAYLOAD=0," + String(message.length()), 5000);
  if (response.indexOf(">") < 0) {
    Serial.println("Failed to enter payload data mode");
    isMQTTConnected = false;
    gsmFailureCount++;
    return false;
  }

  SerialGSM.print(message);
  response = waitForResponse(5000);
  if (response.indexOf("OK") < 0) {
    Serial.println("Failed to set payload");
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
    Serial.println("Publish failed - no valid response");
    isMQTTConnected = false;
    gsmFailureCount++;
    return false;
  }

  lastKeepAliveTime = millis();
  gsmFailureCount = 0;  // ✅ reset after success
  Serial.println("Message published successfully!");
  return true;
}

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
        }
      }
    }
  }
}

void processIncomingMessage(String topic, String message) {
  Serial.print("Incoming message [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);
}

bool connectWiFi() {
  Serial.println("Connecting to WiFi...");
  Serial.printf("SSID: %s\n", config.wifi_ssid);

  WiFi.begin(config.wifi_ssid, config.wifi_password);

  int attempts = 0;
  const int maxAttempts = 300;
  const int delayTime = 250;

  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    Serial.printf("Attempting WiFi connection... Attempt %d\n", attempts + 1);

    digitalWrite(WIFI_LED, HIGH);
    delay(delayTime);
    digitalWrite(WIFI_LED, LOW);
    delay(delayTime);

    if (WiFi.status() == WL_CONNECTED) break;
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(WIFI_LED, HIGH);
    Serial.println("✅ WiFi connected successfully.");
    return true;
  } else {
    Serial.println("❌ WiFi connection failed.");
    digitalWrite(WIFI_LED, LOW);
    return false;
  }
}
void sendViaWIFI() {
  if (WiFi.status() == WL_CONNECTED) {
    time_t timestamp = 0;
    if (getLocalTime(&timeinfo)) {
      timestamp = mktime(&timeinfo);
      Serial.print("Current UTC timestamp: ");
      Serial.println(timestamp);
    } else {
      Serial.println("Failed to update time");
      timestamp = 123456789;
    }

    if (timestamp == 0) {
      LOG_WARN("Failed to get timestamp, using millis()");
      timestamp = millis() / 1000;
    }

   // Pull the latest averages
  if (!getLatestReadings(temperature, humidity, AQI, TVOC, ECO2)) {
  LOG_WARN("No averages ready yet.");

  }


    DynamicJsonDocument doc(1024);
    doc["TimeStamp"] = timestamp;
    doc["DeviceId"] = config.deviceId;
      doc["DeviceName"] = "AeroneroControlSystem";
    doc["DeviceFirmwareVersion"] = current_firmware_version;

    // ✅ Range validations
    if (isnan(temperature) || temperature < -40 || temperature > 125 || temperature == 0.0) doc["Temperature"] = nullptr;
    else doc["Temperature"] = temperature;

    if (isnan(humidity) || humidity < 1 || humidity > 100) doc["Humidity"] = nullptr;
    else doc["Humidity"] = humidity;

    if (distance < 3 || distance > 500) doc["WaterTankLevel"] = nullptr;
    else doc["WaterTankLevel"] = distance;

    // ✅ Always load from prefs
    float savedPh  = readCounter("phValue", NAN);
    float savedTds = readCounter("tdsValue", NAN);

    if (isnan(savedPh) || savedPh < 1 || savedPh > 14) doc["PhValue"] = nullptr;
    else doc["PhValue"] = savedPh;

    if (isnan(savedTds) || savedTds < 1 || savedTds > 1000) doc["Tds"] = nullptr;
    else doc["Tds"] = savedTds;

    if (AQI < 1 || AQI > 5) doc["AQI"] = nullptr;
    else doc["AQI"] = AQI;

    if (TVOC < 0 || TVOC > 65000) doc["TVOC"] = nullptr;
    else doc["TVOC"] = TVOC;

    if (ECO2 < 400 || ECO2 > 65000) doc["ECO2"] = nullptr;
    else doc["ECO2"] = ECO2;

    // ✅ Use your generic read functions
    float savedLiters = readCounter("totalLiters", 0.0f);
    unsigned long motor = readCounterULong("motorMinutes", 0);
    unsigned long comp  = readCounterULong("compMinutes", 0);

    float litersToSend = roundf(savedLiters * 1000) / 1000.0f;
    if (fabs(litersToSend) < 0.001) litersToSend = 0.0f;
    doc["WaterTotalizer"] = litersToSend;

    doc["Compressor"] = comp;
    doc["Motor"] = motor;
    doc["DataCollectionInterval"] = config.publishInterval/60000;
    doc["Timezone"] = "IST-05:30";
    doc["TimezoneCountry"] = "Asia/Kolkata";
    doc["TimeAutoSync"] = "Enabled";
    doc["MQTTEndpoint"] = "a1brrleef337f0-ats.iot.ap-south-1.amazonaws.com";
    doc["MQTTPort"] = "8883";
    doc["AirQualityRegulationRegion"] = "India";
    char jsonBuffer[1024];
    serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

    if (!client.connected()) {
      LOG_WARN("MQTT not connected, attempting to reconnect...");
      connectAWS();
    }

    bool success = false;
    int retryDelay = 100;

    for (int i = 0; i < 3; i++) {
      if (client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer)) {
        LOG_INFO("MQTT Publish success: %s", jsonBuffer);
        success = true;
        break;
      } else {
        LOG_WARN("MQTT Publish failed (Attempt %d/3). Error Code: %d", i + 1, client.state());
        delay(retryDelay);
        retryDelay *= 2;
        connectAWS();
      }
    }

    if (success) {
      // ✅ Reset using your generic save functions
      saveCounter("totalLiters", 0.0f);
      saveCounterULong("motorMinutes", 0);
      saveCounterULong("compMinutes", 0);
      motorMinutes = 0;
      compressorMinutes = 0;
      digitalWrite(dataStatus, HIGH);
      delay(2000);
      digitalWrite(dataStatus, LOW);

      mqttFailureCount = 0;
    } else {
      LOG_ERROR("All MQTT publish attempts failed.");
      mqttFailureCount++;
    }
  } else {
    LOG_WARN("WiFi not connected. Reconnection handled by checkConnectivityTask.");
  }
}



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

int medianOfArray(int arr[], int n) {
    int temp[n];
    memcpy(temp, arr, n * sizeof(int));
    for (int i = 0; i < n - 1; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (temp[i] > temp[j]) {
                int t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }
    return temp[n / 2];  // Return median value
}
// ------------------- TDS Sensor -------------------
void readTDSSensor(float temperature) {
  int analogValue = analogRead(tdsPin);
  float voltage = analogValue * 3.3 / 4095.0;
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensationVoltage = voltage / compensationCoefficient;
  tdsValue = (133.42 * pow(compensationVoltage, 3)
              - 255.86 * pow(compensationVoltage, 2)
              + 857.39 * compensationVoltage)
             * tdsFactor;

  Serial.printf("TDS Value: %.2f ppm\n", tdsValue);

  if (tdsValue >= 0 && tdsValue <= 2000) {
    prefs.putFloat("tdsValue", tdsValue);
    Serial.println("TDS Value saved to Preferences");
  } else {
    Serial.println("Invalid TDS Value, not saved");
  }
}



// ------------------- pH SENSOR READ FUNCTION -------------------
void readPHSensor() {
    int rawSamples[SAMPLES];

    // Collect multiple samples
    for (int i = 0; i < SAMPLES; ++i) {
        rawSamples[i] = analogRead(PH_PIN);
        delay(SAMPLE_DELAY_MS);
    }

    // Find median
    int med = medianOfArray(rawSamples, SAMPLES);

    // Remove outliers (±37 counts ≈ ±30 mV)
    long sum = 0;
    int count = 0;
    for (int i = 0; i < SAMPLES; ++i) {
        if (abs(rawSamples[i] - med) <= 37) {
            sum += rawSamples[i];
            ++count;
        }
    }

    float avgADC = (count > 0) ? (float)sum / count : (float)med;

    // Convert ADC to voltage
    float avgV = avgADC * 3.3 / 4095.0;
    float mv = avgV * 1000.0;
    float rel_mV = (avgADC - neutralADC) * (3.3 / 4095.0) * 1000.0;

    // Apply temperature-compensated slope
    float s25 = slope25();
    float sT = s25 * ((temperatureC + 273.15) / (25.0 + 273.15));
    phValue = sT * (avgADC - neutralADC) + 7.0;

    // Force invalid readings to 0.0
    if (phValue < 0.0 || phValue > 14.0) {
        phValue = 0.0;
    }

    LOG_DEBUG("Raw ADC: %.2f → pH: %.2f", avgADC, phValue);
}


void measureUltrasonicDistance() {
  sensorSerial.write(0x55);
  delay(200);
  if (sensorSerial.available() >= 4) {
    for (int i = 0; i < 4; i++) {
      buffer[i] = sensorSerial.read();
    }
    if (buffer[0] == 0xFF) {
      uint8_t checksum = (buffer[0] + buffer[1] + buffer[2]) & 0xFF;
      if (checksum == buffer[3]) {
        uint16_t distance_mm = (buffer[1] << 8) | buffer[2];
        distance = distance_mm / 10;
        Serial.print("Ultrasonic Sensor Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
      } else {
        Serial.println("Ultrasonic Sensor: Checksum error.");
      }
    } else {
      Serial.println("Ultrasonic Sensor: Invalid frame header.");
    }
  }
}

void readENS160Sensor() {
  status = ens160.getENS160Status();
  if (status == 1) {
    isENS = true;
  }
  if (isENS) {
    AQI = ens160.getAQI();
    TVOC = ens160.getTVOC();
    ECO2 = ens160.getECO2();
  } else {
    AQI = 0;
    TVOC = 0;
    ECO2 = 0;
  }
  if (status == 0) {
    ens160.begin();
    ens160.setPWRMode(ENS160_STANDARD_MODE);
  }
}

float getCPUUsage() {
  return (float)(xTaskGetTickCount() * portTICK_PERIOD_MS) / millis() * 100.0;
}
String getInternalTemperature() {
  float temp = temperatureRead();  // Read temperature as a float
  return String(temp, 2);          // Convert to String with 2 decimal places
}


bool performOTAUpdate(String ssid, String password, String otaLink) {
   if (WiFi.status() != WL_CONNECTED) {
    LOG_INFO("Connecting to WiFi: %s", ssid.c_str());
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
    LOG_INFO("Already connected to WiFi: %s", WiFi.SSID().c_str());
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
  LOG_INFO("OTA update successful. Rebooting...");
  http.end();
  WiFi.disconnect(true);
  buzzerConnected();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  esp_restart();
  return true;
}
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
  prefs.putInt("publishInterval", config.publishInterval);

  prefs.end();
  Serial.println("Configuration saved to Preferences.");
}

// Load configuration from Preferences
void loadConfig() {
  prefs.begin("my-config", true);  // read-only mode

  String deviceId = prefs.getString("deviceId", "");
  String type     = prefs.getString("type", "");

  if (deviceId == "" || type == "") {
    Serial.println("No valid configuration found. Starting in setup mode.");
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
    String apn  = prefs.getString("gsm_apn", "");
    strncpy(config.gsm_username, user.c_str(), sizeof(config.gsm_username));
    strncpy(config.gsm_password, pass.c_str(), sizeof(config.gsm_password));
    strncpy(config.gsm_apn, apn.c_str(), sizeof(config.gsm_apn));
  }

  // Load thresholds
  config.tempThresholdMin   = prefs.getFloat("tempMin", 0);
  config.tempThresholdMax   = prefs.getFloat("tempMax", 0);
  config.humidityThresholdMin = prefs.getFloat("humMin", 0);
  config.humidityThresholdMax = prefs.getFloat("humMax", 0);
  config.compressorOnLevel  = prefs.getFloat("compLevel", 0);
  
  config.compressorOffLevel  = prefs.getFloat("compoffLevel", 0);
  config.publishInterval    = prefs.getInt("publishInterval", 60000);

  // Load OTA credentials
  String otaSSID   = prefs.getString("last_ota_ssid", "");
  String otaPass   = prefs.getString("last_ota_password", "");
  String otaLink   = prefs.getString("last_ota_link", "");

  strncpy(config.last_ota_ssid, otaSSID.c_str(), sizeof(config.last_ota_ssid));
  strncpy(config.last_ota_password, otaPass.c_str(), sizeof(config.last_ota_password));
  strncpy(config.last_ota_link, otaLink.c_str(), sizeof(config.last_ota_link));

  prefs.end();

  // Debug prints
  Serial.println("Configuration loaded:");
  Serial.print("Device ID: "); Serial.println(config.deviceId);
  Serial.print("Communication Type: "); Serial.println(config.type);
  if (strcmp(config.type, "WiFi") == 0) {
    Serial.print("WiFi SSID: "); Serial.println(config.wifi_ssid);
  } else if (strcmp(config.type, "GSM") == 0) {
    Serial.print("GSM APN: "); Serial.println(config.gsm_apn);
  }
  Serial.print("tempMin: "); Serial.println(config.tempThresholdMin);
  Serial.print("tempMax: "); Serial.println(config.tempThresholdMax);
  Serial.print("humMin: "); Serial.println(config.humidityThresholdMin);
  Serial.print("humMax: "); Serial.println(config.humidityThresholdMax);
  Serial.print("compLevel: "); Serial.println(config.compressorOnLevel);
  
  Serial.print("compofffLevel: "); Serial.println(config.compressorOffLevel);
  Serial.print("publishInterval: "); Serial.println(config.publishInterval);

  // OTA creds
  Serial.print("Last OTA SSID: "); Serial.println(config.last_ota_ssid);
  Serial.print("Last OTA Password: "); Serial.println(config.last_ota_password);
  Serial.print("Last OTA Link: "); Serial.println(config.last_ota_link);
}


void setupServer() {
  digitalWrite(WIFI_LED, LOW);
   // Build SSID with prefix and deviceId
   String ssid = String("AN-") + String(config.deviceId);


  // Start soft AP with SSID and password
  WiFi.softAP(ssid.c_str(), "12345678");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  loggedIn = false;

  server.on("/", HTTP_GET, handleRoot);
  server.on("/login", HTTP_POST, handleLogin);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/save_thresholds", HTTP_POST, handleThresholdSave); // New endpoint
  server.on("/ota_submit", HTTP_POST, handleOTASubmit);
  server.begin();
  Serial.println("Local server started.");
  lastClientTime = millis();

  while (!configSaved) {
    server.handleClient();
    if (millis() - lastClientTime > 180000) {
      Serial.println("No client activity. Stopping server.");
      server.stop();
      WiFi.softAPdisconnect(true);
      digitalWrite(WIFI_LED, LOW);
      return;
    }
    delay(10);
  }
  delay(2000);
  Serial.println("Configuration saved. Rebooting...");
  server.stop();
  WiFi.softAPdisconnect(true);
  digitalWrite(WIFI_LED, LOW);
  esp_restart();
}

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
  if (server.hasArg("publishInterval")) {
    config.publishInterval = server.arg("publishInterval").toInt() * 60000; // Convert minutes to ms
    updated = true;
  }

  if (updated) {
    saveConfig();
    configSaved = true;
    server.send(200, "text/html", "<html><body><h2>Thresholds saved. Rebooting...</h2></body></html>");
  } else {
    server.send(400, "text/html", "<html><body><h2>No valid threshold fields provided.</h2><a href='/'>Back</a></body></html>");
  }
  lastClientTime = millis();
}

void handleRoot() {
  if (!loggedIn) {
    server.send(200, "text/html", generateLoginPage());
  } else {
    server.send(200, "text/html", generateMainPage());
  }
  lastClientTime = millis();
}

void handleLogin() {
  if (server.hasArg("username") && server.hasArg("password")) {
    String username = server.arg("username");
    String password = server.arg("password");
    if (username == "Admin" && password == "Admin@123") {
      loggedIn = true;
      server.send(200, "text/html", generateMainPage());
    } else {
      server.send(401, "text/html", "<html><body><h2>Invalid credentials</h2><a href='/'>Back to Login</a></body></html>");
    }
  } else {
    server.send(400, "text/html", "<html><body><h2>Missing credentials</h2><a href='/'>Back to Login</a></body></html>");
  }
  lastClientTime = millis();
}

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
  } else {
    server.send(400, "text/html", "<html><body><h2>Invalid input. Device ID and Communication Type are required.</h2><a href='/'>Back</a></body></html>");
  }
  lastClientTime = millis();
}



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
    server.send(200, "text/html", "<html><body><h2>OTA Update initiated. Rebooting...</h2></body></html>");
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


//===========================buzzer section ==================================================//
void buzzerSearching() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
}

void buzzerSetupMode() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
  delay(500);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
}

void buzzerConnected() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(200);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
}

void buzzerFailure() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(1000);
  digitalWrite(BUZZER_PIN, LOW);
  delay(500);
}
