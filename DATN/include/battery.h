#ifndef BATTERY_MODULE_H
#define BATTERY_MODULE_H

#include <Arduino.h>

class BatteryModule {
public:
    BatteryModule(uint8_t pin = 34);
    
    void begin();
    
    // Read raw ADC value (0 - 4095)
    uint16_t readRaw();
    
    // Read battery voltage in millivolts (mV, e.g. 4200 for 4.2V)
    uint16_t readVoltagemV();
    
    // Calculate battery percentage (0 - 100%)
    uint8_t getPercent();

private:
    uint8_t adc_pin;
    
    // ADC parameters
    static const uint8_t SAMPLE_SIZE = 10;
    
    // Voltage conversion parameters calibrated with multimeter (3.80V actual vs 3.084V uncalibrated)
    // Multiplier = 1.6117 * (3800 / 3084) = 1.9858f
    const float ADC_TO_MV = 1.9858f;
};

#endif // BATTERY_MODULE_H
