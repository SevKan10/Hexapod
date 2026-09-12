#pragma once
#include <Arduino.h>
#include "config.h"

// ============================================================
// SensorManager — Quan ly 6 touch sensor va bao dong pin (battery)
// ============================================================
class SensorManager {
public:
    void begin();
    void update(); // Goi dinh ky (khong can moi loop, co the vai lan/giay)

    bool isTouching(uint8_t legIndex) const {
        return legIndex < 6 ? _touchState[legIndex] : false;
    }
    void getAllTouch(bool out[6]) const {
        for (int i = 0; i < 6; i++) out[i] = _touchState[i];
    }

    float getBatteryVoltage() const { return _batteryVoltage; }
    bool  isBatteryWarning() const  { return _batteryVoltage > 0.5f && _batteryVoltage < BATTERY_WARN_V; }
    bool  isBatteryShutoff() const  { return _batteryVoltage > 0.5f && _batteryVoltage < BATTERY_SHUTOFF_V; }

private:
    bool  _touchState[6] = {false};
    float _batteryVoltage = 0.0f;

    void readTouchSensors();
    void readBattery();
};
