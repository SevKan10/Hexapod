#include "servo_controller.h"
float ServoController::getCurrentAngle(uint8_t servoIndex) const {
    if (servoIndex >= 18) return 0.0f;
    return _currentAngle[servoIndex];
}
bool ServoController::begin() {
    // KHONG goi Wire.begin() o day nua.
    // I2C duoc init 1 lan duy nhat trong setup() cua .ino
    // Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    _pca[0].begin();
    delay(1);
    _pca[0].setPWMFreq(PCA9685_FREQ);
    delay(1);

    _pca[1].begin();
    delay(1);
    _pca[1].setPWMFreq(PCA9685_FREQ);
    delay(1);

    delay(50);

    for (uint8_t i = 0; i < 18; i++) {
        _currentAngle[i] = 0.0f;
        if ((i % 6) == 0) delay(1);
    }

    _ready = true;
    Serial.println("[ServoController] 2x PCA9685 (0x40, 0x41) da san sang");
    return true;
}

uint16_t ServoController::angleToPwmCount(uint8_t servoIndex, float angleDeg) {
    const ServoCal& cal = SERVO_CAL[servoIndex];

    float adjusted = angleDeg + (float)cal.trim;
    if (cal.inverted) adjusted = -adjusted;

    adjusted = constrain(adjusted, -90.0f, 90.0f);

    float physicalAngle = 90.0f + adjusted;
    physicalAngle = constrain(physicalAngle, 0.0f, 180.0f);

    long count = map((long)physicalAngle, 0, 180,
                     SERVO_PWM_MIN_COUNT, SERVO_PWM_MAX_COUNT);

    return (uint16_t)constrain(
        count,
        (long)SERVO_PWM_MIN_COUNT,
        (long)SERVO_PWM_MAX_COUNT
    );
}

void ServoController::setAngle(uint8_t servoIndex, float angleDeg) {
    if (!_ready || servoIndex >= 18) return;

    const ServoCal& cal = SERVO_CAL[servoIndex];
    uint16_t count = angleToPwmCount(servoIndex, angleDeg);

    _pca[cal.board].setPWM(cal.channel, 0, count);
    _currentAngle[servoIndex] = constrain(angleDeg, -90.0f, 90.0f);
}

void ServoController::centerAll() {
    for (uint8_t i = 0; i < 18; i++) {
        setAngle(i, 0.0f);
        delay(2); // watchdog-safe + giam dot bien dong dien servo
    }
    Serial.println("[ServoController] Tat ca servo ve vi tri center");
}

void ServoController::disableServo(uint8_t servoIndex) {
    if (!_ready || servoIndex >= 18) return;
    const ServoCal& cal = SERVO_CAL[servoIndex];
    _pca[cal.board].setPWM(cal.channel, 0, 0);
}

void ServoController::disableAll() {
    for (uint8_t i = 0; i < 18; i++) {
        disableServo(i);
        if ((i % 6) == 0) delay(1);
    }
    Serial.println("[ServoController] Tat ca servo da disable (nghi luc)");
}