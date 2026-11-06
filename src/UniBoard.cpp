#include "UniBoard.h"
#include <Wire.h>

using namespace uni_detail;

static const UniBoardPins BOARD_V1 = {
  2, 4, 27, 26,       // моторы: левый A/B, правый A/B
  18, 19, 35, 34,     // энкодеры: левый Int/Dir, правый Int/Dir
  25, 33,             // светодиод, батарея
  16, 17,             // UART RX/TX (те же линии, что порты P6/P5)
  {12, 13, 14, 15, 17, 16, 32, 23}
};

static const UniBoardPins BOARD_V2 = {
  16, 17, 23, 2,      // LM_D1, LM_D2, RM_D1, RM_D2
  18, 19, 35, 34,     // LM_C1, LM_C2, RM_C1, RM_C2
  5, 36,              // BUILTIN, BAT_CTRL (SENSOR_VP)
  14, 27,             // UART RX/TX: как и на V1, выведен на порты P6/P5
  {32, 33, 25, 26, 27, 14, 13, 4} // P1_POGO..P8_POGO
};

// ============ Калибровка движения ============
// Зависит от драйверов моторов: на плате V1 стоят TA6586, на V2 - DRV8837

static const float MAX_TICKS_PER_SEC_V1 = 1500.0f;
static const float MAX_TICKS_PER_SEC_V2 = 1500.0f;

// Напряжение батареи, при котором измерена скорость выше (оценка)
static const float BATTERY_REF_VOLTAGE_V1 = 3.9f;
static const float BATTERY_REF_VOLTAGE_V2 = 3.9f;

static TuningConfig tuningV1() {
  TuningConfig t;
  t.minPower = 15;
  t.pGain = 0.5f;
  t.iGain = 0.8f;   // постоянная интегрирования pGain / iGain около 0.6 с
  t.dGain = 0.05f;

  t.rotateTolDeg = 0.2f;
  t.rotateAccel = 1300.0f;
  t.rotateMinSpeed = 40.0f;

  t.moveTolMM = 0.5f;
  t.moveAccel = 1800.0f;
  t.moveMinSpeed = 60.0f;

  t.mainAxisGain = 0.8f;
  t.crossAxisGain = 2.0f;
  t.iLimit = 30.0f;
  t.brakeGain = 0.2f;
  t.leashMM = 8.0f;
  t.crossLeashMM = 6.0f;
  t.rampUpFactor = 1.15f;
  t.maxDecel = 6000.0f;
  t.powerSlew = 1000.0f;
  t.accelFF = 0.03f;

  // Измерено на роботе с платой V1 приемочным тестом: выбег 31 мс * v - 0.8 мм
  // на прямых и 25 мс * v - 0.55 мм в поворотах
  t.moveBrakeLeadMs = 31.0f;
  t.moveBrakeDropMM = 0.8f;
  t.rotateBrakeLeadMs = 25.0f;
  t.rotateBrakeDropMM = 0.55f;
  t.brakeTimeMs = 50.0f;
  t.stallTimeMs = 800.0f;
  return t;
}

// Пока повторяет калибровку V1: значения для DRV8837 еще не подобраны
static TuningConfig tuningV2() {
  return tuningV1();
}

UniBoardVersion UniBoard::_version = UNI_BOARD_AUTO;

void UniBoard::set(UniBoardVersion version) {
  _version = version;
}

bool UniBoard::isSet() {
  return _version != UNI_BOARD_AUTO;
}

UniBoardVersion UniBoard::detect(uint8_t sda, uint8_t scl) {
  Wire.begin(sda, scl);

  // Содержимое EEPROM не важно: микросхема подтверждает свой адрес даже пустой.
  // Повторы нужны, потому что во время внутреннего цикла записи (до 5 мс) она молчит
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(UNI_EEPROM_ADDR);
    if (Wire.endTransmission() == 0) return UNI_BOARD_V2;
    delay(6);
  }
  return UNI_BOARD_V1;
}

UniBoardVersion UniBoard::version() {
  if (_version == UNI_BOARD_AUTO) _version = detect();
  return _version;
}

const char* UniBoard::name() {
  return (version() == UNI_BOARD_V2) ? "V2" : "V1";
}

const UniBoardPins& UniBoard::pins() {
  return (version() == UNI_BOARD_V2) ? BOARD_V2 : BOARD_V1;
}

TuningConfig UniBoard::tuning() {
  return (version() == UNI_BOARD_V2) ? tuningV2() : tuningV1();
}

float UniBoard::maxTicksPerSec() {
  return (version() == UNI_BOARD_V2) ? MAX_TICKS_PER_SEC_V2 : MAX_TICKS_PER_SEC_V1;
}

float UniBoard::batteryRefVoltage() {
  return (version() == UNI_BOARD_V2) ? BATTERY_REF_VOLTAGE_V2 : BATTERY_REF_VOLTAGE_V1;
}

UniMotorDecay UniBoard::motorDecay() {
  return (version() == UNI_BOARD_V2) ? UNI_DECAY_SLOW : UNI_DECAY_FAST;
}

uint8_t UniBoard::port(uint8_t index) {
  if (index < 1 || index > UNI_PORT_COUNT) return UNI_PIN_AUTO;

  // Глобальные переменные (const int LINE = P2;) инициализируются до запуска
  // планировщика, когда опрашивать I2C еще нельзя. Отдаем логический номер
  // порта - в GPIO его переведет gpio() при первом обращении к порту
  if (_version == UNI_BOARD_AUTO && xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    return UNI_PORT_BASE + index;
  }
  return pins().ports[index - 1];
}

int UniBoard::gpio(int port) {
  if (port > UNI_PORT_BASE && port <= UNI_PORT_BASE + UNI_PORT_COUNT) {
    return pins().ports[port - UNI_PORT_BASE - 1];
  }
  return port;
}
