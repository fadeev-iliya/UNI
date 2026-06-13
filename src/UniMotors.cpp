#include "UniMotors.h"

UniMotors::UniMotors() {}

void UniMotors::begin(uint8_t pinLA, uint8_t pinLB, uint8_t pinRA, uint8_t pinRB) {
    _pinLA = pinLA;
    _pinLB = pinLB;
    _pinRA = pinRA;
    _pinRB = pinRB;

    // Setup LEDC channels
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(_pinLA, _pwmFreq, _pwmResolution);
    ledcAttach(_pinLB, _pwmFreq, _pwmResolution);
    ledcAttach(_pinRA, _pwmFreq, _pwmResolution);
    ledcAttach(_pinRB, _pwmFreq, _pwmResolution);
#else
    ledcSetup(_chanLA, _pwmFreq, _pwmResolution);
    ledcSetup(_chanLB, _pwmFreq, _pwmResolution);
    ledcSetup(_chanRA, _pwmFreq, _pwmResolution);
    ledcSetup(_chanRB, _pwmFreq, _pwmResolution);

    // Attach pins to channels
    ledcAttachPin(_pinLA, _chanLA);
    ledcAttachPin(_pinLB, _chanLB);
    ledcAttachPin(_pinRA, _chanRA);
    ledcAttachPin(_pinRB, _chanRB);
#endif

    // Initialize to stop
    setPower(0, 0);
}

void UniMotors::setLeftPower(float power) {
    power = constrain(power, -100.0f, 100.0f);
    uint32_t duty = (uint32_t)(abs(power) * 10.23f); // 0-1023 range

    if (power > 0) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(_pinLA, duty);
        ledcWrite(_pinLB, 0);
#else
        ledcWrite(_chanLA, duty);
        ledcWrite(_chanLB, 0);
#endif
    } else if (power < 0) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(_pinLA, 0);
        ledcWrite(_pinLB, duty);
#else
        ledcWrite(_chanLA, 0);
        ledcWrite(_chanLB, duty);
#endif
    } else {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(_pinLA, 0);
        ledcWrite(_pinLB, 0);
#else
        ledcWrite(_chanLA, 0);
        ledcWrite(_chanLB, 0);
#endif
    }
}

void UniMotors::setRightPower(float power) {
    power = constrain(power, -100.0f, 100.0f);
    uint32_t duty = (uint32_t)(abs(power) * 10.23f); // 0-1023 range

    if (power > 0) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(_pinRA, 0);
        ledcWrite(_pinRB, duty);
#else
        ledcWrite(_chanRA, 0);
        ledcWrite(_chanRB, duty);
#endif
    } else if (power < 0) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(_pinRA, duty);
        ledcWrite(_pinRB, 0);
#else
        ledcWrite(_chanRA, duty);
        ledcWrite(_chanRB, 0);
#endif
    } else {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(_pinRA, 0);
        ledcWrite(_pinRB, 0);
#else
        ledcWrite(_chanRA, 0);
        ledcWrite(_chanRB, 0);
#endif
    }
}

void UniMotors::setPower(float leftPower, float rightPower) {
    setLeftPower(leftPower);
    setRightPower(rightPower);
}

void UniMotors::brakeLeft() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(_pinLA, 1023);
    ledcWrite(_pinLB, 1023);
#else
    ledcWrite(_chanLA, 1023);
    ledcWrite(_chanLB, 1023);
#endif
}

void UniMotors::brakeRight() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(_pinRA, 1023);
    ledcWrite(_pinRB, 1023);
#else
    ledcWrite(_chanRA, 1023);
    ledcWrite(_chanRB, 1023);
#endif
}

void UniMotors::brakeBoth() {
    brakeLeft();
    brakeRight();
}
