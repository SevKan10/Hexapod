#include "sensor_manager.h"

void SensorManager::begin() {
    for (uint8_t i = 0; i < TOUCH_SENSOR_COUNT; i++) {
        // GPIO 32,33 ho tro INPUT_PULLUP noi; 34,35,36,39 la input-only,
        // KHONG co pull-up/pull-down noi tren ESP32 -> phai dung mode INPUT thuong.
        if (TOUCH_SENSOR_PINS[i] == 32 || TOUCH_SENSOR_PINS[i] == 33) {
            pinMode(TOUCH_SENSOR_PINS[i], INPUT_PULLUP);
        } else {
            pinMode(TOUCH_SENSOR_PINS[i], INPUT);
        }
    }

    analogReadResolution(12); // 0-4095, chuan ESP32
    Serial.println("[SensorManager] Da khoi tao touch sensor + battery ADC");
}

void SensorManager::readTouchSensors() {
    for (uint8_t i = 0; i < TOUCH_SENSOR_COUNT; i++) {
        int raw = digitalRead(TOUCH_SENSOR_PINS[i]);
        bool active = TOUCH_HIGH_ACTIVE[i] ? (raw == HIGH) : (raw == LOW);
        _touchState[i] = active;
    }
}

void SensorManager::readBattery() {
    int raw = analogRead(BATTERY_ADC_PIN);
    // ESP32 ADC reference ~3.3V (thuc te co the lech, nen calib rieng neu can do chinh xac cao)
    float adcVoltage = (raw / 4095.0f) * 3.3f;
    _batteryVoltage = adcVoltage * BATTERY_VOLTAGE_DIVIDER_RATIO;
}

void SensorManager::update() {
    readTouchSensors();
    readBattery();
}
