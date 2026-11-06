#include "UniMotors.h"

UniMotors::UniMotors() {}

void UniMotors::begin(uint8_t pinLA, uint8_t pinLB, uint8_t pinRA, uint8_t pinRB, bool slowDecay) {
    _pinLA = pinLA;
    _pinLB = pinLB;
    _pinRA = pinRA;
    _pinRB = pinRB;
    _slowDecay = slowDecay;

    // Setup PWM frequency and resolution
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // Core 3.x: settings are per pin and only apply after the first analogWrite()
    const uint8_t pins[] = {_pinLA, _pinLB, _pinRA, _pinRB};
    for (uint8_t pin : pins) {
        analogWrite(pin, 0);
        analogWriteResolution(pin, _pwmResolution);
        analogWriteFrequency(pin, _pwmFreq);
    }
#else
    // Core 2.x: settings are global and must be set before the first analogWrite()
    analogWriteResolution(_pwmResolution);
    analogWriteFrequency(_pwmFreq);
#endif

    // Initialize to stop
    setPower(0, 0);
}

// Все записи в ШИМ моторов идут через эту функцию: она следит, чтобы предыдущая
// запись в тот же вход драйвера успела вступить в силу
void UniMotors::writePwm(uint8_t pin, uint32_t duty) {
    uint8_t slot = (pin == _pinLA) ? 0 : (pin == _pinLB) ? 1 : (pin == _pinRA) ? 2 : 3;
    uint32_t sinceLast = micros() - _lastWriteUs[slot];
    if (sinceLast < _minWriteGapUs) delayMicroseconds(_minWriteGapUs - sinceLast);
    analogWrite(pin, duty);
    _lastWriteUs[slot] = micros();
}

// pinDrive - вход драйвера, задающий направление вращения, pinIdle - второй вход
void UniMotors::drive(uint8_t pinDrive, uint8_t pinIdle, uint32_t duty) {
    if (_slowDecay && duty > 0) {
        // "Ход/тормоз": в паузах ШИМ обмотка замкнута, ток в ней сохраняется.
        // Быстрому драйверу (DRV8837) это дает тягу, пропорциональную мощности:
        // в режиме "ход/выбег" он в каждой паузе сбрасывает ток обратно в питание
        writePwm(pinDrive, _pwmMax);
        writePwm(pinIdle, _pwmMax - duty);
    } else {
        // "Ход/выбег": в паузах ШИМ выходы драйвера отключены
        writePwm(pinIdle, 0);
        writePwm(pinDrive, duty);
    }
}

void UniMotors::setLeftPower(float power) {
    power = constrain(power, -100.0f, 100.0f);
    uint32_t duty = (uint32_t)(abs(power) * 10.23f); // 0-1023 range

    if (power > 0) {
        drive(_pinLA, _pinLB, duty);
    } else if (power < 0) {
        drive(_pinLB, _pinLA, duty);
    } else {
        writePwm(_pinLA, 0);
        writePwm(_pinLB, 0);
    }
}

void UniMotors::setRightPower(float power) {
    power = constrain(power, -100.0f, 100.0f);
    uint32_t duty = (uint32_t)(abs(power) * 10.23f); // 0-1023 range

    if (power > 0) {
        drive(_pinRB, _pinRA, duty);
    } else if (power < 0) {
        drive(_pinRA, _pinRB, duty);
    } else {
        writePwm(_pinRA, 0);
        writePwm(_pinRB, 0);
    }
}

void UniMotors::setPower(float leftPower, float rightPower) {
    setLeftPower(leftPower);
    setRightPower(rightPower);
}

void UniMotors::brakeLeft() {
    writePwm(_pinLA, _pwmMax);
    writePwm(_pinLB, _pwmMax);
}

void UniMotors::brakeRight() {
    writePwm(_pinRA, _pwmMax);
    writePwm(_pinRB, _pwmMax);
}

void UniMotors::brakeBoth() {
    brakeLeft();
    brakeRight();
}
