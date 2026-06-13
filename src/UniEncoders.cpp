#include "UniEncoders.h"

UniEncoders::UniEncoders() {}

void UniEncoders::begin(uint8_t pinL_Int, uint8_t pinL_Dir, uint8_t pinR_Int, uint8_t pinR_Dir) {
    _pinL_Int = pinL_Int;
    _pinL_Dir = pinL_Dir;
    _pinR_Int = pinR_Int;
    _pinR_Dir = pinR_Dir;

    // Настройка PCNT для левого энкодера
    // Старая логика: if(!digitalRead(DIR)) enc++ else enc-- на RISING
    // Значит:
    // Когда DIR=LOW, RISING -> Count UP (PCNT_MODE_KEEP)
    // Когда DIR=HIGH, RISING -> Count DOWN (PCNT_MODE_REVERSE)
    pcnt_config_t pcnt_config_l = {
        .pulse_gpio_num = _pinL_Int,
        .ctrl_gpio_num = _pinL_Dir,
        .lctrl_mode = PCNT_MODE_KEEP,      // Когда DIR LOW, сохраняем направление pos_mode
        .hctrl_mode = PCNT_MODE_REVERSE,   // Когда DIR HIGH, инвертируем направление
        .pos_mode = PCNT_COUNT_INC,        // Считать по переднему фронту (RISING)
        .neg_mode = PCNT_COUNT_DIS,        // Игнорировать задний фронт
        .counter_h_lim = 32767,
        .counter_l_lim = -32767,
        .unit = _unitL,
        .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&pcnt_config_l);
    pcnt_set_filter_value(_unitL, 100); // Глитч-фильтр
    pcnt_filter_enable(_unitL);
    pcnt_counter_pause(_unitL);
    pcnt_counter_clear(_unitL);
    pcnt_counter_resume(_unitL);

    // Настройка PCNT для правого энкодера
    // Старая логика правого была инвертирована относительно левого:
    // if(!digitalRead(DIR)) enc-- else enc++ на RISING
    // То есть: LOW -> DOWN (REVERSE), HIGH -> UP (KEEP)
    pcnt_config_t pcnt_config_r = {
        .pulse_gpio_num = _pinR_Int,
        .ctrl_gpio_num = _pinR_Dir,
        .lctrl_mode = PCNT_MODE_REVERSE,   // Когда DIR LOW, инвертируем (Down)
        .hctrl_mode = PCNT_MODE_KEEP,      // Когда DIR HIGH, сохраняем (Up)
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 32767,
        .counter_l_lim = -32767,
        .unit = _unitR,
        .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&pcnt_config_r);
    pcnt_set_filter_value(_unitR, 100);
    pcnt_filter_enable(_unitR);
    pcnt_counter_pause(_unitR);
    pcnt_counter_clear(_unitR);
    pcnt_counter_resume(_unitR);

    _lEncAccumulator = 0;
    _rEncAccumulator = 0;
    _lastPcntL = 0;
    _lastPcntR = 0;
}

void UniEncoders::updateUnsafe() {
    int16_t currentL, currentR;
    pcnt_get_counter_value(_unitL, &currentL);
    pcnt_get_counter_value(_unitR, &currentR);

    // Рассчитываем разницу в int32_t
    int32_t diffL = (int32_t)currentL - (int32_t)_lastPcntL;
    int32_t diffR = (int32_t)currentR - (int32_t)_lastPcntR;

    // ESP32 PCNT сбрасывается в 0 при достижении h_lim или l_lim (+/-32767),
    // само значение лимита счетчик не показывает - полный оборот равен 32767 шагам
    if (diffL < -16384) diffL += 32767;
    else if (diffL > 16384) diffL -= 32767;

    if (diffR < -16384) diffR += 32767;
    else if (diffR > 16384) diffR -= 32767;

    _lEncAccumulator += diffL;
    _rEncAccumulator += diffR;

    _lastPcntL = currentL;
    _lastPcntR = currentR;
}

void UniEncoders::update() {
    portENTER_CRITICAL(&_mux);
    updateUnsafe();
    portEXIT_CRITICAL(&_mux);
}

long UniEncoders::getLeftTicks() {
    portENTER_CRITICAL(&_mux);
    updateUnsafe();
    long v = _lEncAccumulator;
    portEXIT_CRITICAL(&_mux);
    return v;
}

long UniEncoders::getRightTicks() {
    portENTER_CRITICAL(&_mux);
    updateUnsafe();
    long v = _rEncAccumulator;
    portEXIT_CRITICAL(&_mux);
    return v;
}

void UniEncoders::resetLeft() {
    portENTER_CRITICAL(&_mux);
    pcnt_counter_pause(_unitL);
    pcnt_counter_clear(_unitL);
    _lEncAccumulator = 0;
    _lastPcntL = 0;
    pcnt_counter_resume(_unitL);
    portEXIT_CRITICAL(&_mux);
}

void UniEncoders::resetRight() {
    portENTER_CRITICAL(&_mux);
    pcnt_counter_pause(_unitR);
    pcnt_counter_clear(_unitR);
    _rEncAccumulator = 0;
    _lastPcntR = 0;
    pcnt_counter_resume(_unitR);
    portEXIT_CRITICAL(&_mux);
}

void UniEncoders::resetBoth() {
    resetLeft();
    resetRight();
}
