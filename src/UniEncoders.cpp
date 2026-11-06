#include "UniEncoders.h"

UniEncoders::UniEncoders() {}

// Счетчик одного колеса. Канал Int считает импульсы, уровень Dir задает знак:
// у левого колеса LOW на Dir при фронте Int - счет вверх, у правого (inverted) -
// вниз. В квадратурном режиме считаются оба фронта Int и оба фронта Dir:
// полный цикл энкодера дает четыре отсчета вместо одного.
void UniEncoders::configUnit(pcnt_unit_t unit, uint8_t pinInt, uint8_t pinDir, bool inverted, bool quadrature) {
    pcnt_config_t ch0 = {};
    ch0.pulse_gpio_num = pinInt;
    ch0.ctrl_gpio_num = pinDir;
    ch0.lctrl_mode = inverted ? PCNT_MODE_REVERSE : PCNT_MODE_KEEP;
    ch0.hctrl_mode = inverted ? PCNT_MODE_KEEP : PCNT_MODE_REVERSE;
    ch0.pos_mode = PCNT_COUNT_INC;
    // Задний фронт считается в обратную сторону: при том же направлении вращения
    // уровень Dir на нем противоположный, и знак снова получается верным
    ch0.neg_mode = quadrature ? PCNT_COUNT_DEC : PCNT_COUNT_DIS;
    ch0.counter_h_lim = 32767;
    ch0.counter_l_lim = -32767;
    ch0.unit = unit;
    ch0.channel = PCNT_CHANNEL_0;
    pcnt_unit_config(&ch0);

    if (quadrature) {
        // Второй канал того же счетчика: импульсы Dir, знак по уровню Int
        pcnt_config_t ch1 = ch0;
        ch1.pulse_gpio_num = pinDir;
        ch1.ctrl_gpio_num = pinInt;
        ch1.lctrl_mode = inverted ? PCNT_MODE_KEEP : PCNT_MODE_REVERSE;
        ch1.hctrl_mode = inverted ? PCNT_MODE_REVERSE : PCNT_MODE_KEEP;
        ch1.channel = PCNT_CHANNEL_1;
        pcnt_unit_config(&ch1);
    }

    pcnt_set_filter_value(unit, 100); // Глитч-фильтр
    pcnt_filter_enable(unit);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void UniEncoders::begin(uint8_t pinL_Int, uint8_t pinL_Dir, uint8_t pinR_Int, uint8_t pinR_Dir, bool quadrature) {
    _pinL_Int = pinL_Int;
    _pinL_Dir = pinL_Dir;
    _pinR_Int = pinR_Int;
    _pinR_Dir = pinR_Dir;
    _countsPerTick = quadrature ? 4 : 1;

    // Правый энкодер стоит зеркально, поэтому считает в обратную сторону
    configUnit(_unitL, _pinL_Int, _pinL_Dir, false, quadrature);
    configUnit(_unitR, _pinR_Int, _pinR_Dir, true, quadrature);

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

    // Между двумя чтениями колесо не может пройти столько (это секунда на полной
    // скорости, а читаем мы каждые несколько миллисекунд). Такой скачок - сбой:
    // если пустить его в позицию, привод увидит ошибку в метры и рванет исправлять
    const int32_t maxStep = 8000;
    if (diffL > maxStep || diffL < -maxStep) { diffL = 0; _glitches++; }
    if (diffR > maxStep || diffR < -maxStep) { diffR = 0; _glitches++; }

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

void UniEncoders::read(long& leftCount, long& rightCount) {
    portENTER_CRITICAL(&_mux);
    updateUnsafe();
    leftCount = _lEncAccumulator;
    rightCount = _rEncAccumulator;
    portEXIT_CRITICAL(&_mux);
}

long UniEncoders::getLeftTicks() {
    portENTER_CRITICAL(&_mux);
    updateUnsafe();
    long v = _lEncAccumulator;
    portEXIT_CRITICAL(&_mux);
    return v / _countsPerTick;
}

long UniEncoders::getRightTicks() {
    portENTER_CRITICAL(&_mux);
    updateUnsafe();
    long v = _rEncAccumulator;
    portEXIT_CRITICAL(&_mux);
    return v / _countsPerTick;
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
