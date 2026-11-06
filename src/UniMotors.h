#ifndef UNIMOTORS_H
#define UNIMOTORS_H

#include <Arduino.h>

class UniMotors {
private:
    uint8_t _pinLA;
    uint8_t _pinLB;
    uint8_t _pinRA;
    uint8_t _pinRB;

    const uint32_t _pwmFreq = 20000; // 20 kHz for silent operation
    const uint8_t _pwmResolution = 10; // 10-bit (0-1023) for smooth PID control
    const uint32_t _pwmMax = 1023;

    // Режим ШИМ: false - "ход/выбег" (второй вход драйвера в нуле),
    // true - "ход/тормоз" (активный вход в единице, на втором инверсный ШИМ)
    bool _slowDecay = false;

    // Новое значение ШИМ вступает в силу только на границе периода. Вторая запись
    // в тот же канал, пришедшая раньше, может потеряться - и вход драйвера
    // останется в прежнем состоянии (например, на 100% после тормоза). Поэтому
    // между записями в один канал выдерживается пауза в несколько периодов
    static const uint32_t _minWriteGapUs = 200;
    uint32_t _lastWriteUs[4] = {0, 0, 0, 0};

    void writePwm(uint8_t pin, uint32_t duty);
    void drive(uint8_t pinDrive, uint8_t pinIdle, uint32_t duty);

public:
    UniMotors();

    // Инициализация пинов
    void begin(uint8_t pinLA, uint8_t pinLB, uint8_t pinRA, uint8_t pinRB, bool slowDecay = false);

    // Управление моторами
    void setLeftPower(float power);
    void setRightPower(float power);
    void setPower(float leftPower, float rightPower);

    // Экстренное торможение
    void brakeLeft();
    void brakeRight();
    void brakeBoth();
};

#endif
