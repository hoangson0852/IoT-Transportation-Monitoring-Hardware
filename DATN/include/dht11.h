#ifndef DHT11_MODULE_H
#define DHT11_MODULE_H

#include <Arduino.h>

#ifndef DHT11
  #define DHT11 11
#endif
#ifndef DHT22
  #define DHT22 22
#endif

class DHTModule {
public:
  DHTModule(uint8_t pin, uint8_t type = 11);
  void begin();
  bool read(float &temp, float &hum);
  float readTemperature();
  float readHumidity();
private:
  uint8_t _pin;
  float _last_temp;
  float _last_hum;
  uint32_t _last_read_time;
};

// Helper to start the DHT FreeRTOS task from other files
void startDhtTask(unsigned long stackSize, UBaseType_t priority);

#endif // DHT11_MODULE_H