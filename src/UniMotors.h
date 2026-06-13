#ifndef UNIMOTORS_H
#define UNIMOTORS_H

#include <Arduino.h>

class UniMotors {
private:
    uint8_t _pinLA;
    uint8_t _pinLB;
    uint8_t _pinRA;
    uint8_t _pinRB;

    // LEDC channels (ESP32 specific)
    const uint8_t _chanLA = 0;
    const uint8_t _chanLB = 1;
    const uint8_t _chanRA = 2;
    const uint8_t _chanRB = 3;

    const uint32_t _pwmFreq = 20000; // 20 kHz for silent operation
    const uint8_t _pwmResolution = 10; // 10-bit (0-1023) for smooth PID control

public:
    UniMotors();
    
    // Инициализация пинов
    void begin(uint8_t pinLA, uint8_t pinLB, uint8_t pinRA, uint8_t pinRB);
    
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
