#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

class ServoController {
public:
    bool begin();
    uint16_t angleToPwmCount(uint8_t servoIndex, float angleDeg);
    void setAngle(uint8_t servoIndex, float angleDeg);
    float getCurrentAngle(uint8_t servoIndex) const;   // <-- thêm dòng này
    void centerAll();
    void disableServo(uint8_t servoIndex);
    void disableAll();

private:
    Adafruit_PWMServoDriver _pca[2] = {
        Adafruit_PWMServoDriver(PCA9685_ADDR_1),
        Adafruit_PWMServoDriver(PCA9685_ADDR_2)
    };
    float _currentAngle[18] = {0};
    bool _ready = false;
};