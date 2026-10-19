#include "dht11.h"
#include "vehicle_config.h"
#include "sensor_Data.h"

// ===== DISABLED: WiFi + MQTT (not needed for real-time sensor data) =====
// #include <WiFi.h>
// #include <PubSubClient.h>

// const char* WIFI_SSID = "H3-908-5G";
// const char* WIFI_PASS = "66668888";
// const char* MQTT_BROKER = "192.168.1.100";
// =========================================================================

// ===== Định danh thiết bị (đưa vào JSON) =====
// VEHICLE_ID is now dynamic from gVehicleConfig.getDeviceId()
// static const char* DEVICE_ID  = "ESP32-AMMO-001";
// static const char* COMPARTMENT= "MAIN_BAY";

// ===== DISABLED: Threshold alerts + WiFi/MQTT (not needed) =====
// static const float AMMO_TEMP_HIGH_C  = 30.0f;
// static const float AMMO_TEMP_CLEAR_C = 29.0f;
// static const float AMMO_HUM_HIGH_P   = 80.0f;
// static const float AMMO_HUM_CLEAR_P  = 75.0f;
// static const int WARMUP_SAMPLES = 2;
// static const unsigned long ALERT_MIN_INTERVAL_MS = 500UL;
// static const uint8_t TELEMETRY_PRINT_EVERY_N = 1;
// static int telemetryPrintCounter = 0;
// static bool thermalAlertActive  = false;
// static bool humidityAlertActive = false;
// static unsigned long lastAlertMs = 0;
// static int sampleCount = 0;
// static WiFiClient espClient;
// static PubSubClient mqttClient(espClient);
// =========================================================================

// The project defines a global `DHTModule dht` in main.cpp. Use that instance here.
extern DHTModule dht;

// ===== DISABLED: WiFi/MQTT functions =====
// void wifiConnectIfNeeded() { ... }
// void mqttConnectIfNeeded() { ... }
// void publishIfConnected() { ... }
// void printTelemetryHuman() { ... }
// void printTelemetryJSON() { ... }
// void maybePrintAlerts() { ... }
// =========================================================================

// Implement DHTModule methods (ESP32-safe bit-banged driver)
DHTModule::DHTModule(uint8_t pin, uint8_t type)
  : _pin(pin), _last_temp(-999.0f), _last_hum(-999.0f), _last_read_time(0) {
  (void)type;
}

void DHTModule::begin() {
  pinMode(_pin, INPUT_PULLUP);
}

bool DHTModule::read(float &temp, float &hum) {
  uint32_t now = millis();
  // Minimum 1.5s interval between DHT11 hardware queries
  if (now - _last_read_time < 1500 && _last_temp != -999.0f) {
    temp = _last_temp;
    hum = _last_hum;
    return true;
  }

  uint8_t data[5] = {0, 0, 0, 0, 0};

  // 1. Send start signal: pull LOW for 20ms
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20);

  // 2. Critical section for handshake + 40-bit read (~4ms total)
  // This prevents FreeRTOS task preemption (like TaskTamper/GPS) from breaking microsecond timings
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);

  // Release pin to INPUT_PULLUP
  pinMode(_pin, INPUT_PULLUP);
  delayMicroseconds(30);

  // Helper with strict timeout
  auto waitState = [this](int state, uint32_t timeout_us) -> uint32_t {
    uint32_t start = micros();
    while (digitalRead(_pin) == state) {
      if ((uint32_t)(micros() - start) > timeout_us) return 0;
    }
    return (uint32_t)(micros() - start);
  };

  // Handshake:
  // Sensor drops LOW for 80us, then pulls HIGH for 80us
  if (waitState(HIGH, 100) == 0) {
    portEXIT_CRITICAL(&mux);
    return false; // Pin never went LOW
  }
  if (waitState(LOW, 120) == 0) {
    portEXIT_CRITICAL(&mux);
    return false; // Pin stuck LOW
  }
  if (waitState(HIGH, 120) == 0) {
    portEXIT_CRITICAL(&mux);
    return false; // Pin stuck HIGH
  }

  // 3. Read 40 bits
  bool ok = true;
  for (int i = 0; i < 40; i++) {
    // 50us LOW
    if (waitState(LOW, 80) == 0) {
      ok = false;
      break;
    }

    // HIGH pulse (26-28us = '0', 70us = '1')
    uint32_t high_dur = waitState(HIGH, 100);
    if (high_dur == 0) {
      ok = false;
      break;
    }

    data[i / 8] <<= 1;
    if (high_dur > 40) {
      data[i / 8] |= 1;
    }
  }

  portEXIT_CRITICAL(&mux);

  if (!ok) return false;

  // 4. Verify checksum
  uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
  if (data[4] != checksum) {
    return false;
  }

  hum = (float)data[0] + (float)data[1] * 0.1f;
  temp = (float)data[2] + (float)data[3] * 0.1f;

  if (temp < -20.0f || temp > 80.0f || hum < 5.0f || hum > 100.0f) {
    return false;
  }

  _last_temp = temp;
  _last_hum = hum;
  _last_read_time = millis();
  return true;
}

float DHTModule::readTemperature() {
  float t = -999.0f, h = -999.0f;
  if (read(t, h)) return t;
  return -999.0f;
}

float DHTModule::readHumidity() {
  float t = -999.0f, h = -999.0f;
  if (read(t, h)) return h;
  return -999.0f;
}

// ===== Task đọc DHT11 =====
void TaskDHT11(void *pvParameters) {
  (void)pvParameters;
  vTaskDelay(pdMS_TO_TICKS(2000)); // Warm-up sensor

  for (;;) {
    float t = -999.0f;
    float h = -999.0f;
    bool ok = dht.read(t, h);

    if (!ok) {
      Serial.println("[DHT11-ERROR] Sensor read failed (no valid data)");
      t = -999.0f;
      h = -999.0f;
    }

    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      sensorData.temp = t;
      sensorData.hum = h;
      xSemaphoreGive(sensorDataMutex);
    } else {
      Serial.println("[DHT11] Failed to lock sensorDataMutex");
    }

    vTaskDelay(pdMS_TO_TICKS(3000)); // Read every 3s
  }
}

// Helper to create the task from other modules (matching header declaration)
void startDhtTask(unsigned long stackSize, UBaseType_t priority) {
  xTaskCreatePinnedToCore(TaskDHT11, "DHT11 Sensor", stackSize, NULL, priority, NULL, 1);
}