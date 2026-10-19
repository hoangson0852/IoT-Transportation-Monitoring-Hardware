#include <Arduino.h>
#include "security.h"

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <esp_system.h>   // For chip ID functions

#include "gps.h"
#include "dht11.h"
#include "adxl345.h"
#include "tamper.h"
#include "battery.h"
#include "vehicle_config.h"
#include "sensor_Data.h"

#include "frame_codec.h"
#include "local_memory.h"
#include <esp_task_wdt.h>

// WDT timeout in seconds
#define WDT_TIMEOUT 10

uint16_t global_session_id = 0;

// ===== Pins / Config =====
#define DHTPIN      14
#define DHTTYPE     DHT11
#define LED_PIN     2
#define TAMPER_PIN  27
#define BUZZER_PIN  13
#define BAT_ADC_PIN 34     

#ifndef VEHICLE_DEVICE_ID
  #define VEHICLE_DEVICE_ID "VX"
#endif

// GPS: UART2 RX=16, TX=17 @ 9600
GPSNeo6M  gps(16, 17, 9600);
DHTModule dht(DHTPIN, DHTTYPE);
ADXLModule adxl; // ADXL345
TamperModule tamper(TAMPER_PIN, BUZZER_PIN, LED_PIN);
BatteryModule battery(BAT_ADC_PIN); 

// --- LoRa UART (RA-08H TX connected here) on UART1 ---
#define LORA_RX   25     // ESP32 RX1  <= TX of RA-08H
#define LORA_TX   26     // ESP32 TX1  => RX of RA-08H
#define LORA_BAUD 115200 // match pingpong_tx UART baud

HardwareSerial LORA_SER(1); 

volatile uint32_t g_send_interval_ms = 2000; 
volatile bool g_tamper_alert = false;        
static TaskHandle_t xTamperTaskHandle = NULL;
static TaskHandle_t xLoraSendTaskHandle = NULL;

// Hardware ISR for Tamper switch SW2 (GPIO 27)
void IRAM_ATTR tamperISR() {
  g_tamper_alert = true;
  // Kích hoạt còi và đèn LED cảnh báo phần cứng tức thì tại chỗ (<0.15ms)
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (xTamperTaskHandle != NULL) {
    vTaskNotifyGiveFromISR(xTamperTaskHandle, &xHigherPriorityTaskWoken);
  }
  if (xLoraSendTaskHandle != NULL) {
    vTaskNotifyGiveFromISR(xLoraSendTaskHandle, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

#define FORCE_NODE_ID 1  // Set to 1 for Transport-1, 2 for Transport-2, etc. | 0 = auto-increment from EEPROM

void detectOrGenerateNodeId(HardwareSerial &lora_uart) {
  (void)lora_uart; 
  
  if (FORCE_NODE_ID > 0 && FORCE_NODE_ID <= 100) {
    Serial.printf("[SYNC] FORCE_NODE_ID enabled - Using Transport-%d (override mode)\r\n", FORCE_NODE_ID);
    gVehicleConfig.setDeviceIdFromNodeId(FORCE_NODE_ID);
    return;
  }
  
  Serial.println("[SYNC] Auto-assigning node_id from EEPROM counter...");
  
  const uint8_t MAGIC_BYTE = 0xAA;
  uint8_t magic = EEPROM.read(1);
  
  if (magic != MAGIC_BYTE) {
    Serial.println("[SYNC] Old EEPROM detected - resetting counter to 0");
    EEPROM.write(0, 0);
    EEPROM.write(1, MAGIC_BYTE);
    EEPROM.commit();
  }
  
  uint8_t counter = EEPROM.read(0);
  
  if (counter == 0 || counter == 0xFF) {
    counter = 1;
  }
  
  uint8_t node_id = counter;
  
  // Increment counter for next device
  counter++;
  if (counter > 100) counter = 1;  // Wrap around after 100
  
  // Save incremented counter for next device
  EEPROM.write(0, counter);
  EEPROM.write(1, MAGIC_BYTE);  // Keep magic byte
  EEPROM.commit();
  
  Serial.printf("[SYNC] Assigned node_id = %u (Transport-%u)\r\n", node_id, node_id);
  Serial.printf("[SYNC] Next device will get node_id = %u\r\n", counter);
  
  gVehicleConfig.setDeviceIdFromNodeId(node_id);
}

static uint32_t getVehicleLoraSendDelayMs() {
  uint8_t veh_num = gVehicleConfig.getVehicleNumber();
  const uint32_t slot_ms = 250;
  static const uint8_t slot_order[8] = { 0, 4, 2, 6, 1, 5, 3, 7 };

  if (veh_num == 0) {
    return 0;
  }

  uint8_t slot_index = (veh_num - 1) % 8;
  return slot_order[slot_index] * slot_ms;
}

// --- FreeRTOS Task: GPS Reader ---
void TaskGPS(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(100);

  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    gps.read();

    if (gps.updated()) {
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sensorData.lat = gps.latitude();
        sensorData.lng = gps.longitude();
        sensorData.sats = gps.satellites();
        sensorData.speed = gps.gpsObject().speed.kmph();
        xSemaphoreGive(sensorDataMutex);
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void TaskTamperMonitor(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    // Chờ tín hiệu đánh thức tức thì từ Hardware ISR (<0.15ms) hoặc thăm dò định kỳ 100ms
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

    // SW2 reads HIGH if box is opened (switch released)
    bool open_state = tamper.isTriggered();

    if (open_state) {
      if (!g_tamper_alert) {
        g_tamper_alert = true;
        Serial.println("[TAMPER ALERT] BOX OPENED (Hardware ISR triggered)!");
      }
      tamper.checkTamper(); // Set tamper_detected = true
      tamper.updateAlarm(true);
    } else {
      g_tamper_alert = false;
      tamper.resetTamper(); // Clear tamper state when closed
      tamper.updateAlarm(false);
    }
  }
}

void TaskLoraSend(void *pv) {
  delay(100);
  while (LORA_SER.available()) LORA_SER.read();

  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint32_t send_delay = getVehicleLoraSendDelayMs();
  if (send_delay > 0) {
    Serial.printf("[SYNC] Slot offset: %lums\r\n", (unsigned long)send_delay);
    vTaskDelay(pdMS_TO_TICKS(send_delay));
    xLastWakeTime = xTaskGetTickCount();
  }

  const TickType_t xInterval = pdMS_TO_TICKS(g_send_interval_ms);
  static uint16_t packet_seq = 0;

  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    SensorData localData = {}; 

    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      localData = sensorData;   // snapshot toàn bộ struct
      xSemaphoreGive(sensorDataMutex);
    }

    // extract ra biến local
    float temp = localData.temp;
    float hum = localData.hum;
    float accel_g = localData.accel;
    double lat = localData.lat;
    double lng = localData.lng;
    uint32_t sats = localData.sats;

    if (isnan(temp) || temp < -100 || temp > 150) {
      Serial.println("[DHT11-ERROR] Sensor read failed (no valid data)");
      temp = -999.0f;
    }
    if (isnan(hum) || hum < 0 || hum > 100) {
      hum = -999.0f;
    }
    if (accel_g < -900.0f) {
        Serial.println("[ADXL345-ERROR] Sensor data unavailable");
        accel_g = -999.0f;
    }

    // Read battery voltage in millivolts (e.g. 3850 mV)
    uint16_t battery_mv = battery.readVoltagemV();
    // Check tamper state from SW2
    bool is_tamper = tamper.getTamperState();

    uint32_t ts = millis();

    // Fill the binary frame
    LoRaFrame frame;
    memset(&frame, 0, sizeof(frame));

    frame.header.magic[0] = 0x25;
    frame.header.magic[1] = 0x03;
    frame.header.device_id = gVehicleConfig.getVehicleNumber();
    frame.header.vehicle_id = gVehicleConfig.getVehicleNumber();
    frame.header.session_id = global_session_id;
    frame.header.packet_id = ++packet_seq;
    frame.header.timestamp = ts;
    frame.header.type = 1; // Telemetry
    
    // Set Flags bitmask
    frame.header.flags = 0;
    if (is_tamper) {
      frame.header.flags |= (1 << 0);
    }
    if (lat != 0.0 && lng != 0.0) {
      frame.header.flags |= (1 << 1); // GPS fix flag
    }
    if (accel_g > 1.2f || accel_g < -1.2f) { // simple vibration threshold
      frame.header.flags |= (1 << 2);
    }
    if (battery_mv < 3400) { // Low battery threshold (3.4V for 3.7V Li-ion battery)
      frame.header.flags |= (1 << 3);
    }

    // Fill Payload
    frame.payload.temperature = (int16_t)round(temp * 10.0f);
    frame.payload.humidity = (int16_t)round(hum * 10.0f);
    frame.payload.accel_mag = (int16_t)round(accel_g * 100.0f);
    frame.payload.light_level = battery_mv; // Send battery voltage instead of light level
    frame.payload.latitude = (int32_t)round(lat * 1e6);
    frame.payload.longitude = (int32_t)round(lng * 1e6);

    // Encrypt payload in-place (AES-128 16 bytes)
    encryptBlockInPlace((uint8_t*)&frame.payload);

    // Compute Auth Tag (2 bytes) over Header + encrypted Payload
    frame.footer.auth_tag = computeFrameAuthTag((const uint8_t*)&frame.header, sizeof(frame.header), (const uint8_t*)&frame.payload, sizeof(frame.payload));

    // Compute CRC (2 bytes) over Header + Payload + Auth Tag (which is 30 bytes)
    frame.footer.crc = calculateCRC16((const uint8_t*)&frame, sizeof(frame) - sizeof(frame.footer.crc));

    // Convert the 32-byte frame to a 64-character Hex string
    char hex_payload[100];
    encode_frame_to_hex(frame, hex_payload);

    Serial.printf("[SENSOR] Temp=%.1fC, Hum=%.1f%%, Accel=%.2fg, Battery=%umV (%u%%), Tamper=%d\r\n", 
                  temp, hum, accel_g, battery_mv, battery.getPercent(), is_tamper ? 1 : 0);
    Serial.printf("[GPS] Lat=%.6f, Lng=%.6f\r\n", lat, lng);
    char cipher_hex[33];
    for (int i = 0; i < 16; i++) {
        sprintf(cipher_hex + i * 2, "%02x", ((uint8_t*)&frame.payload)[i]);
    }
    cipher_hex[32] = '\0';
    Serial.printf("[AES] Encrypted Payload block (16B): %s\r\n", cipher_hex);
    Serial.printf("[AUTH] Calculated Auth Tag: 0x%04X, CRC: 0x%04X\r\n", frame.footer.auth_tag, frame.footer.crc);

    // Send the hex-encoded frame via UART to RA-08H module
    LORA_SER.println(hex_payload); 
    LORA_SER.flush();

    Serial.printf("[ESP32->LORA] Frame sent (Seq=%u). Sent to Gateway!\r\n", packet_seq);

    // Chế độ Unconfirmed Uplink (Không bắt buộc chờ ACK) giúp hệ thống gửi tin cậy và không bị kẹt timeout
    #define ENABLE_ACK_WAIT false
    bool ack_received = true;

    if (ENABLE_ACK_WAIT) {
      int retry_count = 0;
      ack_received = false;
      while (!ack_received && retry_count < 3) {
        uint32_t t0 = millis();
        String rx_buf = "";
        while (millis() - t0 < 2000) {
          esp_task_wdt_reset();
          while (LORA_SER.available()) {
            char c = (char)LORA_SER.read();
            rx_buf += c;
            Serial.print(c);
          }
          
          int start_idx = rx_buf.indexOf("2503");
          if (start_idx >= 0 && rx_buf.length() >= start_idx + 68) {
             String frame_hex = rx_buf.substring(start_idx, start_idx + 68);
             LoRaFrame ack_frame;
             if (hex_string_to_bytes(frame_hex, (uint8_t*)&ack_frame, sizeof(ack_frame))) {
                 if (ack_frame.header.type == 0xFF && ack_frame.header.packet_id == packet_seq) {
                     ack_received = true;
                     Serial.println("[LORA] ACK Received successfully!");
                     break;
                 }
             }
             rx_buf = rx_buf.substring(start_idx + 68);
          }
          vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (!ack_received) {
          retry_count++;
          Serial.printf("[LORA] ACK timeout, retrying %d/3...\n", retry_count);
          vTaskDelay(pdMS_TO_TICKS(500 + random(0, 1000)));
        }
      }
    }

    if (!ack_received) {
      Serial.println("[ERROR] Failed to send LoRa frame after 3 retries.");
    }

    // Nếu có sự kiện cảnh báo khẩn cấp, đánh thức phát ngay (preemptive alert); nếu bình thường chờ theo chu kỳ
    uint32_t wait_ms = g_tamper_alert ? 500 : g_send_interval_ms;
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_ms));
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,    // Bitmask of all cores
      .trigger_panic = true,
  };
  esp_task_wdt_init(&wdt_config);
#else
  esp_task_wdt_init(WDT_TIMEOUT, true); // Initialize Watchdog with WDT_TIMEOUT seconds timeout, panic = true
#endif
  esp_task_wdt_add(NULL); // Add setup/loop task to WDT
  
  EEPROM.begin(512); 
  global_session_id = (uint16_t)increment_and_get_boot_counter();
  Serial.printf("[INIT] Boot counter / Session ID: %u\r\n", global_session_id);
  
  Serial.println("[INIT] Initializing modules...");

  sensorDataMutex = xSemaphoreCreateMutex();
  if (sensorDataMutex == NULL) {
    Serial.println("[ERROR] Failed to create sensorDataMutex!");
  }

  if (sensorDataMutex != NULL) {
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      sensorData.temp = -999.0f;
      sensorData.hum = -999.0f;
      sensorData.accel = -999.0f;
      sensorData.shock_detected = false;
      sensorData.is_moving = false;
      xSemaphoreGive(sensorDataMutex);
    }
  }

  gVehicleConfig.begin();
  Serial.printf("[INIT] Vehicle ID (default): %s\r\n", gVehicleConfig.getDeviceId());

  Wire.begin();
  Wire.setTimeOut(50); // Prevent I2C hardware bus lock if sensor not attached

  dht.begin();
  if (!adxl.begin()) {
    Serial.println("[WARN] ADXL345 not found (check wiring)");
  }
  
  Serial.println("[INIT] Starting LoRa UART...");
  LORA_SER.begin(LORA_BAUD, SERIAL_8N1, LORA_RX, LORA_TX);
  delay(100);
  while (LORA_SER.available()) LORA_SER.read(); 
  
  Serial.println("[INIT] Detecting/generating node_id...");
  detectOrGenerateNodeId(LORA_SER);
  Serial.printf("[INIT] Final Vehicle ID: %s\r\n", gVehicleConfig.getDeviceId());
  
  tamper.begin();
  battery.begin();
  Serial.println("[INIT] Tamper (SW2/Buzzer/LED) and Battery ADC initialized");

  // Gắn ngắt cứng phần cứng (Hardware ISR) cho công tắc SW2 trên GPIO 27 (đánh thức tức thì <0.15ms)
  attachInterrupt(digitalPinToInterrupt(TAMPER_PIN), tamperISR, CHANGE);
  Serial.println("[INIT] Hardware ISR attached to TAMPER_PIN (GPIO 27)");

  // Tamper & GPS & LoRa tasks pinned to Core 1
  xTaskCreatePinnedToCore(TaskTamperMonitor, "TamperMon", 2048, NULL, 2, &xTamperTaskHandle, 1);

  // NOTE: Bo comment 2 dong duoi khi da cam day du cam bien DHT11 va ADXL345 vao mach
  startDhtTask(2048, 1);        
  startAdxlTelemetry(4096, 1);  

  xTaskCreatePinnedToCore(TaskLoraSend, "LoraSend", 4096, NULL, 1, &xLoraSendTaskHandle, 1);
  xTaskCreatePinnedToCore(TaskGPS,      "TaskGPS",  4096, NULL, 1, NULL, 1);

  gps.begin();
  
  Serial.printf("\r\n[INIT] All systems initialized\r\n");
  Serial.printf("[INIT] Starting patrol mode...\r\n");
  Serial.printf("\r\n");
}

void loop() {
  esp_task_wdt_reset();
  vTaskDelay(pdMS_TO_TICKS(1000));
}
// #include <Arduino.h>
// // Khai báo thư viện GPS, ADXL345, LoRa Ra-08H

// // Chu kỳ ngủ sâu = 58 giây (tính theo micro-giây)
// #define TIME_TO_SLEEP 58 
// #define uS_TO_S_FACTOR 1000000ULL 

// void setup() {
//   Serial.begin(115200);
//   delay(500); // Chờ nguồn ổn định

//   Serial.println("\n==============================================");
//   Serial.println("[KICH BAN 2] ESP32 THUC DAY TU DEEP SLEEP!");
  
//   // 1. Đọc nhanh dữ liệu từ các cảm biến (khoảng 1.5s)
//   Serial.println("[1] Doc du lieu GPS, ADXL345, LDR...");
//   // readSensors();
//   delay(1500); // Giả lập thời gian thu thập dữ liệu GPS fix
  
//   // 2. Phát gói tin qua LoRa Ra-08H (khoảng 0.5s)
//   Serial.println("[2] Gui goi tin LoRa qua Ra-08H...");
//   // sendLoRaPacket();
//   delay(500); // Giả lập phát LoRa 0.5s
//   Serial.println("[2] Gui LoRa thanh cong!");

//   // 3. Chuẩn bị đưa vi điều khiển vào Deep Sleep 58 giây
//   Serial.println("[3] Chuan bi vao Deep Sleep 58 giay...");
//   Serial.flush(); // Xóa sạch bộ đệm Serial trước khi ngủ
  
//   // Cài đặt ngắt hẹn giờ thức dậy sau 58 giây
//   esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  
//   // Bắt đầu vào Deep Sleep (Mạch sẽ tắt CPU, giảm dòng về 11-13mA)
//   esp_deep_sleep_start();
// }

// void loop() {
//   // Trong Kịch bản 2, hàm loop() KHÔNG BAO GIỜ được chạy
//   // Vì sau khi setup() kết thúc, chip lập tức đi ngủ và khi thức dậy sẽ khởi động lại từ setup()
// }

