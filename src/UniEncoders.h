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

    // Аккумуляторы в отсчетах счетчика: в квадратурном режиме их вчетверо
    // больше, чем тиков (тик - единица ticksPerRev в UniConfig)
    long _lEncAccumulator = 0;
    long _rEncAccumulator = 0;

    int16_t _lastPcntL = 0;
    int16_t _lastPcntR = 0;

    uint8_t _countsPerTick = 1;

    // Скачок счетчика, невозможный физически (сбой чтения или переполнения),
    // в позицию не попадает; здесь считается, сколько таких было
    uint32_t _glitches = 0;

    // update() вызывается с обоих ядер (контур управления и пользовательский API) -
    // без блокировки read-modify-write аккумуляторов портит счетчики
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

    void configUnit(pcnt_unit_t unit, uint8_t pinInt, uint8_t pinDir, bool inverted, bool quadrature);
    void updateUnsafe();

public:
    UniEncoders();

    // Инициализация пинов. quadrature = true - счет по всем фронтам обоих каналов
    // (x4): вчетверо выше разрешение, дрожание колеса на фронте не дает ложных тиков
    void begin(uint8_t pinL_Int, uint8_t pinL_Dir, uint8_t pinR_Int, uint8_t pinR_Dir, bool quadrature = false);

    // Вызывать периодически (например, раз в 10-50 мс) для предотвращения переполнения 16-битного счетчика
    void update();

    // Оба счетчика одним снимком, в отсчетах (тики * countsPerTick)
    void read(long& leftCount, long& rightCount);

    // Отсчетов счетчика на один тик: 4 в квадратурном режиме, иначе 1
    uint8_t countsPerTick() const { return _countsPerTick; }

    // Сколько раз отброшен невозможный скачок счетчика (в норме 0)
    uint32_t glitches() const { return _glitches; }

    // Получение счетчиков в тиках (единицы ticksPerRev, не зависят от режима счета)
    long getLeftTicks();
    long getRightTicks();

    // Сброс
    void resetLeft();
    void resetRight();
    void resetBoth();
};

#endif
