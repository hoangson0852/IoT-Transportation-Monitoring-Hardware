#include "battery.h"

BatteryModule::BatteryModule(uint8_t pin) : adc_pin(pin) {}

void BatteryModule::begin() {
    pinMode(adc_pin, INPUT);
}

uint16_t BatteryModule::readRaw() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < SAMPLE_SIZE; i++) {
        sum += analogRead(adc_pin);
        delay(2);
    }
    return sum / SAMPLE_SIZE;
}

uint16_t BatteryModule::readVoltagemV() {
    uint16_t raw = readRaw();
    return (uint16_t)(raw * ADC_TO_MV);
}

uint8_t BatteryModule::getPercent() {
    uint16_t mv = readVoltagemV();
    if (mv >= 4200) return 100;
    if (mv <= 3300) return 0;
    
    // Đường cong xả chuẩn 6 mốc thực nghiệm cho cell pin 18650 Li-ion 3.7V khớp chuẩn Chương 5
    // 100% (>= 4.20V), 80% (3.95V - 4.20V), 60% (3.80V - 3.95V), 40% (3.70V - 3.80V), 20% (3.60V - 3.70V), 0% (<= 3.30V)
    if (mv >= 3950) {
        return 80 + (uint8_t)((mv - 3950) * 20 / 250);
    } else if (mv >= 3800) {
        return 60 + (uint8_t)((mv - 3800) * 20 / 150);
    } else if (mv >= 3700) {
        return 40 + (uint8_t)((mv - 3700) * 20 / 100);
    } else if (mv >= 3600) {
        return 20 + (uint8_t)((mv - 3600) * 20 / 100);
    } else {
        return (uint8_t)((mv - 3300) * 20 / 300);
    }
}


