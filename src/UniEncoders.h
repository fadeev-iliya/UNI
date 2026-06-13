#ifndef UNIENCODERS_H
#define UNIENCODERS_H

#include <Arduino.h>
#include "driver/pcnt.h"

class UniEncoders {
private:
    uint8_t _pinL_Int;
    uint8_t _pinL_Dir;
    uint8_t _pinR_Int;
    uint8_t _pinR_Dir;

    pcnt_unit_t _unitL = PCNT_UNIT_0;
    pcnt_unit_t _unitR = PCNT_UNIT_1;

    long _lEncAccumulator = 0;
    long _rEncAccumulator = 0;

    int16_t _lastPcntL = 0;
    int16_t _lastPcntR = 0;

    // update() вызывается с обоих ядер (контур управления и пользовательский API) -
    // без блокировки read-modify-write аккумуляторов портит счетчики
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

    void updateUnsafe();

public:
    UniEncoders();
    
    // Инициализация пинов
    void begin(uint8_t pinL_Int, uint8_t pinL_Dir, uint8_t pinR_Int, uint8_t pinR_Dir);
    
    // Вызывать периодически (например, раз в 10-50 мс) для предотвращения переполнения 16-битного счетчика
    void update();
    
    // Получение счетчиков
    long getLeftTicks();
    long getRightTicks();
    
    // Сброс
    void resetLeft();
    void resetRight();
    void resetBoth();
};

#endif
