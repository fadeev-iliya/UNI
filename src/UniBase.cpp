#include "UniBase.h"

using namespace uni_detail;

// ============ Параметры контура управления ============

static const uint32_t CONTROL_PERIOD_MS = 5;   // период контура управления (200 Гц)
static const uint32_t FINISH_PERIOD_MS = 1;    // то же на финише маневра: точнее момент тормоза
static const float FINISH_ZONE_S = 0.03f;      // финиш начинается за это время до включения тормоза
static const float FINISH_ZONE_MM = 0.5f;      // и не ближе этого расстояния до него
static const float SPEED_FILTER_TAU = 0.02f;   // сглаживание измеренной скорости колес (с)
static const uint32_t STILL_TIME_MS = 30;      // столько колеса не двигались - робот стоит
static const uint32_t BRAKE_MAX_MS = 300;      // дольше остановку под тормозом не ждем
static const uint32_t WHEEL_BRAKE_MS = 30;     // тормоз одного колеса в stopLeft()/stopRight()
static const float STEER_FREEZE_MM = 40.0f;    // ближе к точке moveTo курс не пересчитывается
static const float HOLD_DEADBAND_MM = 0.3f;    // зона нечувствительности holdPosition()
static const float CHAIN_MAX_ALONG_MM = 3.0f;  // больший остаток по ходу следующей команде не передается
static const float NUDGE_START = 0.6f;         // holdPosition() возвращает колесо, начиная с этой доли minPower,
static const float NUDGE_RAMP = 80.0f;         // и наращивает мощность (%/с), пока колесо стоит
static const float NUDGE_RAMP_MAX = 25.0f;
static const float BRAKE_LEARN = 0.5f;         // шаг подстройки модели выбега под тормозом
static const uint8_t BRAKE_FAST_STOPS = 2;     // столько первых остановок учитываются полным шагом
static const float BRAKE_SPEED_SCALE = 60.0f;  // масштаб скорости в подстройке (мм/с)
static const uint8_t RELEASE_BURST = 20;       // циклов повторного обнуления моторов после остановки
static const float BATTERY_COMP_MIN = 0.85f;   // пределы поправки мощности на заряд батареи
static const float BATTERY_COMP_MAX = 1.25f;

static const float DEG2RAD = 0.01745329252f;
static const float RAD2DEG = 57.2957795131f;
static const float TWO_PI_F = 6.28318530718f;
static const float PI_F = 3.14159265359f;

// Больший остаток курса следующей команде не передается (робота сдвинули рукой)
static const float CHAIN_MAX_HEADING = 5.0f * DEG2RAD;

// Угол в диапазоне -PI..PI
static float wrapPi(float a) {
  a = fmodf(a + PI_F, TWO_PI_F);
  if (a < 0) a += TWO_PI_F;
  return a - PI_F;
}

static UniConfig configForBoard(UniBoardVersion board) {
  UniConfig config;
  config.board = board;
  return config;
}

// Параметры, оставленные как UNI_TUNING_AUTO, берутся из калибровки платы board
static void applyBoardTuning(TuningConfig& tuning, const TuningConfig& board) {

  if (tuning.minPower < 0) tuning.minPower = board.minPower;
  if (tuning.pGain < 0) tuning.pGain = board.pGain;
  if (tuning.iGain < 0) tuning.iGain = board.iGain;
  if (tuning.dGain < 0) tuning.dGain = board.dGain;

  if (tuning.rotateTolDeg < 0) tuning.rotateTolDeg = board.rotateTolDeg;
  if (tuning.rotateAccel < 0) tuning.rotateAccel = board.rotateAccel;
  if (tuning.rotateMinSpeed < 0) tuning.rotateMinSpeed = board.rotateMinSpeed;

  if (tuning.moveTolMM < 0) tuning.moveTolMM = board.moveTolMM;
  if (tuning.moveAccel < 0) tuning.moveAccel = board.moveAccel;
  if (tuning.moveMinSpeed < 0) tuning.moveMinSpeed = board.moveMinSpeed;

  if (tuning.mainAxisGain < 0) tuning.mainAxisGain = board.mainAxisGain;
  if (tuning.crossAxisGain < 0) tuning.crossAxisGain = board.crossAxisGain;
  if (tuning.iLimit < 0) tuning.iLimit = board.iLimit;
  if (tuning.brakeGain < 0) tuning.brakeGain = board.brakeGain;
  if (tuning.leashMM < 0) tuning.leashMM = board.leashMM;
  if (tuning.crossLeashMM < 0) tuning.crossLeashMM = board.crossLeashMM;
  if (tuning.rampUpFactor < 0) tuning.rampUpFactor = board.rampUpFactor;
  if (tuning.maxDecel < 0) tuning.maxDecel = board.maxDecel;
  if (tuning.powerSlew < 0) tuning.powerSlew = board.powerSlew;
  if (tuning.accelFF < 0) tuning.accelFF = board.accelFF;

  if (tuning.moveBrakeLeadMs < 0) tuning.moveBrakeLeadMs = board.moveBrakeLeadMs;
  if (tuning.moveBrakeDropMM < 0) tuning.moveBrakeDropMM = board.moveBrakeDropMM;
  if (tuning.rotateBrakeLeadMs < 0) tuning.rotateBrakeLeadMs = board.rotateBrakeLeadMs;
  if (tuning.rotateBrakeDropMM < 0) tuning.rotateBrakeDropMM = board.rotateBrakeDropMM;
  if (tuning.brakeTimeMs < 0) tuning.brakeTimeMs = board.brakeTimeMs;
  if (tuning.stallTimeMs < 0) tuning.stallTimeMs = board.stallTimeMs;
}

UniBase::UniBase(const char* robotName, UniBoardVersion board)
  : UniBase(robotName, configForBoard(board)) {}

UniBase::UniBase(UniBoardVersion board)
  : UniBase("UNI Robot", configForBoard(board)) {}

UniBase::UniBase(const char* robotName, UniConfig config)
  : _cfg(config), _display(nullptr), _displayInitialized(false), _customDisplayMode(false),
    _displayDirty(false), _displayPageMode(false),
    _ledBlinking(false), _ledBlinkInterval(0),
    _batteryPercent(-1), _batteryValid(false), _batteryVoltage(0.0f), _ffComp(1.0f),
    _controlTaskHandle(nullptr), _serviceTaskHandle(nullptr), _cmdQueue(nullptr),
    _xPos(0.0f), _yPos(0.0f), _heading(0.0f), _headingRaw(0.0), _headingOffset(0.0),
    _angleZero(0.0f), _totalDistance(0.0f), _distAcc(0.0f), _odomReset(false),
    _tuningVersion(1), _tuningApplied(0),
    _nowMs(0), _cntL(0), _cntR(0), _velL(0.0f), _velR(0.0f),
    _stillRefL(0), _stillRefR(0), _lastMotionMs(0), _stillCounts(1),
    _state(CS_IDLE), _nearFinish(false), _fastLoop(false), _iBase(0.0f), _iTurn(0.0f), _lastPowerL(0.0f), _lastPowerR(0.0f),
    _afterBrake(AB_RELEASE), _brakeStartMs(0), _brakeResult(UNI_MOVE_OK),
    _nudgeRampL(0.0f), _nudgeRampR(0.0f), _holdL(0), _holdR(0),
    _rawBrakeL(false), _rawBrakeR(false), _rawBrakeUntilL(0), _rawBrakeUntilR(0),
    _brakeModelSet(false), _releaseBurst(0),
    _rawMode(false), _idleAssertMs(0),
    _moveStartMs(0),
    _chainValid(false), _chainHeading(0.0f), _chainAlongValid(false), _chainAlong(0.0f),
    _goalX(0.0f), _goalY(0.0f), _goalPower(0.0f),
    _curFinite(false), _curSeq(0), _stopSeq(0),
    _cmdSeq(0), _busySeq(0), _activeFinite(false), _resultSeq(0), _result(UNI_MOVE_OK),
    _appliedSeq(0), _stopDoneSeq(0), _lastErrMM(0.0f), _lastErrDeg(0.0f),
    _ctrlSerial(nullptr), _ctrlInitialized(false),
    _ctrlBuffer(nullptr), _decodeBuffer(nullptr), _ctrlBufferIndex(0),
    _begun(false)
{
  // Явно заданная версия платы должна быть известна и UniDev, и портам P1..P8
  // еще до begin()
  if (_cfg.board != UNI_BOARD_AUTO) UniBoard::set(_cfg.board);

  _ctrlBuffer = new uint8_t[_cfg.uartBufferSize];
  _decodeBuffer = new uint8_t[_cfg.uartBufferSize];

  strncpy(_robotName, robotName, sizeof(_robotName) - 1);
  _robotName[sizeof(_robotName) - 1] = '\0';

  strncpy(_currentCommand, "Idle", sizeof(_currentCommand) - 1);
  _currentCommand[sizeof(_currentCommand) - 1] = '\0';

  _customDisplayName[0] = '\0';
  _customDisplayText[0] = '\0';
  _displayPage[0] = '\0';

  for (uint8_t i = 0; i < 4; i++) {
    _brakeK[i] = 0.0f;
    _brakeC[i] = 0.0f;
    _brakeSeed[i] = 0.0f;
  }
  _brakeStops[0] = _brakeStops[1] = 0;
  _work = UniMoveReport();
  _report = UniMoveReport();

  // Внутри все расчеты идут в миллиметрах. Раздельные тики на оборот для левого
  // и правого колеса учитываются везде: и в одометрии, и в регуляторах
  float countsPerTick = _cfg.encoderQuad ? 4.0f : 1.0f;
  _mmPerTickL = (PI_F * _cfg.wheelDiameterMM) / _cfg.ticksPerRevLeft;
  _mmPerTickR = (PI_F * _cfg.wheelDiameterMM) / _cfg.ticksPerRevRight;
  _mmPerCountL = _mmPerTickL / countsPerTick;
  _mmPerCountR = _mmPerTickR / countsPerTick;
  _mmPerTick = (_mmPerTickL + _mmPerTickR) * 0.5f;
  _halfTrack = _cfg.trackLengthMM * 0.5f;
  _vmaxL = _vmaxR = _vmaxAvg = 1.0f; // настоящие значения - в begin(), когда известна плата
  _stillCounts = _cfg.encoderQuad ? 2 : 1;

  _moveSemaphore = xSemaphoreCreateBinary();
  _submitMutex = xSemaphoreCreateMutex();
  _cmdQueue = xQueueCreate(16, sizeof(MotionCmd));

  _display = new Adafruit_SSD1306(_cfg.screenWidth, _cfg.screenHeight, &Wire, _cfg.oledReset);
}

UniBase::~UniBase() {
  if (_controlTaskHandle) vTaskDelete(_controlTaskHandle);
  if (_serviceTaskHandle) vTaskDelete(_serviceTaskHandle);
  if (_display) delete _display;
  if (_moveSemaphore) vSemaphoreDelete(_moveSemaphore);
  if (_submitMutex) vSemaphoreDelete(_submitMutex);
  if (_cmdQueue) vQueueDelete(_cmdQueue);
  if (_ctrlBuffer) delete[] _ctrlBuffer;
  if (_decodeBuffer) delete[] _decodeBuffer;
  if (_ctrlSerial) delete _ctrlSerial;
}

// ============ Initialization ============

void UniBase::begin(const char* robotName) {
  // Имя можно задать здесь, в конструкторе или нигде (останется стандартное)
  if (robotName && robotName[0] != '\0') {
    strncpy(_robotName, robotName, sizeof(_robotName) - 1);
    _robotName[sizeof(_robotName) - 1] = '\0';
  }

  if (_begun) return;
  _begun = true; // ставим до инициализации: методы внутри begin() сами зовут ensureBegun

  Serial.begin(115200);

  resolveBoard();

  #ifdef DEBUG_MODE
  Serial.println("\n\nUNI Platform - Industrial Refactoring");
  Serial.print("Robot name: ");
  Serial.println(_robotName);
  Serial.print("Board: ");
  Serial.println(UniBoard::name());
  #endif

  _motors.begin(_cfg.leftMotorA, _cfg.leftMotorB, _cfg.rightMotorA, _cfg.rightMotorB,
                _cfg.motorDecay == UNI_DECAY_SLOW);
  _encoders.begin(_cfg.leftEncInt, _cfg.leftEncDir, _cfg.rightEncInt, _cfg.rightEncDir,
                  _cfg.encoderQuad);
  _encoders.read(_cntL, _cntR);
  _stillRefL = _cntL;
  _stillRefR = _cntR;

  initOLED();
  _uartLastRxTime = millis();

  pinMode(_cfg.ledPin, OUTPUT);
  digitalWrite(_cfg.ledPin, LOW);
  pinMode(_cfg.batteryPin, INPUT);
  analogReadResolution(12);
  updateBattery();

  refreshTuning();
  initTasks();

  #ifdef DEBUG_MODE
  Serial.println("Robot ready");
  #endif

  setCommandName("Ready");
  // Дисплей отрисует служебная задача в своем цикле: рисовать отсюда нельзя,
  // одновременные кадры из двух задач ломают шину I2C на самом старте
}

void UniBase::begin(const char* robotName, UniBoardVersion board) {
  if (!_begun) _cfg.board = board;
  begin(robotName);
}

void UniBase::begin(UniBoardVersion board) {
  begin(nullptr, board);
}

void UniBase::ensureBegun() {
  if (!_begun) begin();
}

// Версия платы: заданная явно либо определенная по EEPROM. Пины, оставленные
// в конфигурации как UNI_PIN_AUTO, берутся из профиля этой платы
void UniBase::resolveBoard() {
  if (_cfg.board != UNI_BOARD_AUTO) {
    UniBoard::set(_cfg.board);
  } else if (!UniBoard::isSet()) {
    UniBoard::set(UniBoard::detect(_cfg.oledSda, _cfg.oledScl));
  }
  _cfg.board = UniBoard::version();

  const UniBoardPins& pins = UniBoard::pins();
  if (_cfg.leftMotorA == UNI_PIN_AUTO) _cfg.leftMotorA = pins.leftMotorA;
  if (_cfg.leftMotorB == UNI_PIN_AUTO) _cfg.leftMotorB = pins.leftMotorB;
  if (_cfg.rightMotorA == UNI_PIN_AUTO) _cfg.rightMotorA = pins.rightMotorA;
  if (_cfg.rightMotorB == UNI_PIN_AUTO) _cfg.rightMotorB = pins.rightMotorB;
  if (_cfg.motorDecay == UNI_DECAY_AUTO) _cfg.motorDecay = UniBoard::motorDecay();

  if (_cfg.leftEncInt == UNI_PIN_AUTO) _cfg.leftEncInt = pins.leftEncInt;
  if (_cfg.leftEncDir == UNI_PIN_AUTO) _cfg.leftEncDir = pins.leftEncDir;
  if (_cfg.rightEncInt == UNI_PIN_AUTO) _cfg.rightEncInt = pins.rightEncInt;
  if (_cfg.rightEncDir == UNI_PIN_AUTO) _cfg.rightEncDir = pins.rightEncDir;

  if (_cfg.ledPin == UNI_PIN_AUTO) _cfg.ledPin = pins.ledPin;
  if (_cfg.batteryPin == UNI_PIN_AUTO) _cfg.batteryPin = pins.batteryPin;

  if (_cfg.uartRxPin == UNI_PIN_AUTO) _cfg.uartRxPin = pins.uartRxPin;
  if (_cfg.uartTxPin == UNI_PIN_AUTO) _cfg.uartTxPin = pins.uartTxPin;

  // Нулевая скорость недопустима (на нее делят), поэтому 0 тоже значит "из калибровки"
  if (_cfg.maxTicksPerSec <= 0) _cfg.maxTicksPerSec = UniBoard::maxTicksPerSec();
  if (_cfg.batteryRefVoltage < 0) _cfg.batteryRefVoltage = UniBoard::batteryRefVoltage();
  applyBoardTuning(_tuning, UniBoard::tuning());

  _vmaxL = _cfg.maxTicksPerSec * _mmPerTickL;
  _vmaxR = _cfg.maxTicksPerSec * _mmPerTickR;
  _vmaxAvg = (_vmaxL + _vmaxR) * 0.5f;
}

void UniBase::setTuning(const TuningConfig& tuning) {
  // До begin() версия платы неизвестна: параметры UNI_TUNING_AUTO заменит
  // resolveBoard(). После - подставляем сразу, причем в копию: контур управления
  // забирает настройки целиком и значения-заглушки ему попадать не должны
  TuningConfig resolved = tuning;
  if (_begun) applyBoardTuning(resolved, UniBoard::tuning());
  portENTER_CRITICAL(&_cmdMux);
  _tuning = resolved;
  _tuningVersion = _tuningVersion + 1;
  portEXIT_CRITICAL(&_cmdMux);
}

TuningConfig UniBase::getTuning() {
  ensureBegun();
  portENTER_CRITICAL(&_cmdMux);
  TuningConfig copy = _tuning;
  portEXIT_CRITICAL(&_cmdMux);
  return copy;
}

void UniBase::initOLED() {
  Wire.begin(_cfg.oledSda, _cfg.oledScl);
  if(!_display->begin(SSD1306_SWITCHCAPVCC, _cfg.screenAddress)) {
    Serial.println(F("SSD1306 allocation failed - continuing without display"));
    _displayInitialized = false;
    return;
  }
  _displayInitialized = true;
  _display->clearDisplay();
  _display->setTextSize(1);
  _display->setTextColor(SSD1306_WHITE);
  _display->setCursor(0, 0);
  _display->println(F("UNI"));
  _display->println(F("Initializing..."));
  _display->display();
  delay(100);
}

// Две задачи на ядре 0. Контур управления работает со строгим периодом и
// приоритетом выше служебной задачи: кадр дисплея по I2C занимает десятки
// миллисекунд и не должен задерживать одометрию и регуляторы
void UniBase::initTasks() {
  xTaskCreatePinnedToCore(controlTask, "UniControl", 6144, this, 10, &_controlTaskHandle, 0);
  xTaskCreatePinnedToCore(serviceTask, "UniService", 8192, this, 1, &_serviceTaskHandle, 0);
  delay(10);
}

// ============ FreeRTOS Tasks ============

void UniBase::controlTask(void* pvParameters) {
  UniBase* self = static_cast<UniBase*>(pvParameters);
  if (!self) vTaskDelete(NULL);

  TickType_t wake = xTaskGetTickCount();
  uint32_t prevMicros = micros();

  for (;;) {
    // На последних миллиметрах маневра и под тормозом контур работает впятеро
    // чаще: за обычный период робот успевает проехать заметную долю миллиметра
    uint32_t periodMs = self->_fastLoop ? FINISH_PERIOD_MS : CONTROL_PERIOD_MS;
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(periodMs));

    // Период задан планировщиком, но шаг считаем по часам: задержку из-за
    // более приоритетных задач (Wi-Fi, Bluetooth) интеграторы учтут
    uint32_t nowMicros = micros();
    float dt = (uint32_t)(nowMicros - prevMicros) * 1e-6f;
    prevMicros = nowMicros;
    self->controlStep(constrain(dt, 0.0005f, 0.05f));
  }
}

void UniBase::serviceTask(void* pvParameters) {
  UniBase* self = static_cast<UniBase*>(pvParameters);
  if (!self) vTaskDelete(NULL);

  uint32_t lastDisplayUpdate = 0;
  const uint32_t displayUpdateInterval = 100;
  uint32_t lastLedToggle = 0;
  uint32_t lastBatteryUpdate = 0;
  const uint32_t batteryUpdateInterval = 500;
  bool ledState = false;

  for (;;) {
    uint32_t currentMillis = millis();
    if (self->_ledBlinking && self->_ledBlinkInterval > 0) {
      if (currentMillis - lastLedToggle >= (uint32_t)self->_ledBlinkInterval) {
        lastLedToggle = currentMillis;
        ledState = !ledState;
        digitalWrite(self->_cfg.ledPin, ledState ? HIGH : LOW);
      }
    }

    if (currentMillis - lastBatteryUpdate >= batteryUpdateInterval) {
      lastBatteryUpdate = currentMillis;
      self->updateBattery();
    }

    if (self->_ctrlInitialized && self->_ctrlSerial->available() > 0) {
      self->ctrlReceiveUART();
    }

    if (self->_displayDirty || currentMillis - lastDisplayUpdate >= displayUpdateInterval) {
      lastDisplayUpdate = currentMillis;
      self->_displayDirty = false;
      self->updateDisplay();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============ Контур управления ============

void UniBase::refreshTuning() {
  portENTER_CRITICAL(&_cmdMux);
  _t = _tuning;
  _tuningApplied = _tuningVersion;
  portEXIT_CRITICAL(&_cmdMux);

  // Нулевые и отрицательные значения там, где на них делят, недопустимы
  _t.rampUpFactor = fmaxf(_t.rampUpFactor, 0.1f);
  _t.powerSlew = fmaxf(_t.powerSlew, 10.0f);
  _t.leashMM = fmaxf(_t.leashMM, 0.5f);
  _t.crossLeashMM = fmaxf(_t.crossLeashMM, 0.5f);
  _t.minPower = constrain(_t.minPower, 0, 90);

  // Настройки заданы в тиках, расчеты идут в миллиметрах
  _d.kP = _t.pGain / _mmPerTick;
  _d.kI = _t.iGain / _mmPerTick;
  _d.kD = _t.dGain / _mmPerTick;
  _d.accMove = fmaxf(_t.moveAccel * _mmPerTick, 1.0f);
  _d.accSpin = fmaxf(_t.rotateAccel * _mmPerTick, 1.0f);
  _d.vMinMove = fmaxf(_t.moveMinSpeed * _mmPerTick, 0.5f);
  _d.vMinSpin = fmaxf(_t.rotateMinSpeed * _mmPerTick, 0.5f);
  _d.maxDecel = fmaxf(_t.maxDecel * _mmPerTick, 1.0f);

  // Модель выбега под тормозом стартует с калибровки и дальше подстраивается сама.
  // Заново она задается, только когда эти настройки изменили
  const float seed[4] = {_t.moveBrakeLeadMs, _t.moveBrakeDropMM, _t.rotateBrakeLeadMs, _t.rotateBrakeDropMM};
  bool seedChanged = !_brakeModelSet;
  for (uint8_t i = 0; i < 4; i++) {
    if (seed[i] != _brakeSeed[i]) seedChanged = true;
  }
  if (seedChanged) {
    for (uint8_t i = 0; i < 4; i++) _brakeSeed[i] = seed[i];
    for (uint8_t w = 0; w < 2; w++) {
      _brakeK[w] = seed[0] * 0.001f;
      _brakeC[w] = seed[1];
      _brakeK[2 + w] = seed[2] * 0.001f;
      _brakeC[2 + w] = seed[3];
    }
    _brakeStops[0] = _brakeStops[1] = 0;
    _brakeModelSet = true;
  }

  // Допуск не может быть тоньше разрешения энкодера
  float resolution = fmaxf(_mmPerCountL, _mmPerCountR);
  _d.tolBase = fmaxf(_t.moveTolMM, resolution);
  _d.tolTurn = fmaxf(_t.rotateTolDeg * DEG2RAD * _halfTrack, resolution);
}

void UniBase::controlStep(float dt) {
  _nowMs = millis();
  if (_tuningVersion != _tuningApplied) refreshTuning();

  long cntL, cntR;
  _encoders.read(cntL, cntR);
  float dL = (cntL - _cntL) * _mmPerCountL;
  float dR = (cntR - _cntR) * _mmPerCountR;
  _cntL = cntL;
  _cntR = cntR;
  updateOdometry(dL, dR);

  float alpha = dt / (SPEED_FILTER_TAU + dt);
  _velL += alpha * (dL / dt - _velL);
  _velR += alpha * (dR / dt - _velR);

  // Детектор остановки: дрожание в один отсчет движением не считается
  if (labs(cntL - _stillRefL) >= _stillCounts || labs(cntR - _stillRefR) >= _stillCounts) {
    _stillRefL = cntL;
    _stillRefR = cntR;
    _lastMotionMs = _nowMs;
  } else if (_nowMs - _lastMotionMs >= STILL_TIME_MS) {
    // Колеса стоят: сглаженная скорость еще "помнит" движение, обнуляем ее,
    // иначе следующая команда примет этот хвост за настоящую скорость
    _velL = 0.0f;
    _velR = 0.0f;
  }

  // Команды применяются только здесь, между шагами регулятора: пользовательские
  // задачи состояние привода не трогают, поэтому смена команды всегда целостна
  MotionCmd cmd;
  while (xQueueReceive(_cmdQueue, &cmd, 0) == pdTRUE) applyCommand(cmd);

  switch (_state) {
    case CS_VELOCITY: stepVelocity(dt); break;
    case CS_PATH:     stepPath(dt); break;
    case CS_HOLD:     stepHold(dt); break;
    case CS_BRAKE:    stepBrake(); break;
    default:          stepIdle(); break;
  }

  _fastLoop = (_state == CS_BRAKE || (_state == CS_PATH && _nearFinish));
}

void UniBase::updateOdometry(float dL, float dR) {
  // Курс считается прямо по счетчикам, а не суммированием приращений:
  // ошибки округления не накапливаются
  double headingRaw = ((double)_cntL * _mmPerCountL - (double)_cntR * _mmPerCountR) / _cfg.trackLengthMM;
  float dCenter = (dL + dR) * 0.5f;

  // Вход в критическую секцию: защищаем от Data Tearing
  portENTER_CRITICAL(&_odomMux);

  float heading = (float)(headingRaw + _headingOffset);
  float midHeading = (_heading + heading) * 0.5f;
  _xPos += dCenter * cosf(midHeading);
  _yPos += dCenter * sinf(midHeading);
  _heading = heading;
  _headingRaw = headingRaw;

  // Путь - это ход центра робота: поворот на месте его не увеличивает,
  // дрожание колес на месте гасится в накопителе
  _distAcc += dCenter;
  if (fabsf(_distAcc) >= 0.5f) {
    _totalDistance += fabsf(_distAcc);
    _distAcc = 0.0f;
  }

  portEXIT_CRITICAL(&_odomMux);
}

void UniBase::readPose(float& x, float& y, float& heading) {
  portENTER_CRITICAL(&_odomMux);
  x = _xPos;
  y = _yPos;
  heading = _heading;
  portEXIT_CRITICAL(&_odomMux);
}

// ---- Следящий привод ----

// Мощность, нужная мотору для скорости v и ускорения a - по модели мотора.
// Регулятору остается исправлять только ошибку этой модели
float UniBase::feedforward(float v, float a, float vmax) {
  float span = 100.0f - _t.minPower;
  float u = a * _t.accelFF * span / vmax;
  if (fabsf(v) > 0.02f) u += copysignf((float)_t.minPower, v) + v / vmax * span;
  return u * _ffComp; // на севшей батарее той же скорости нужна большая мощность
}

// Сколько колесо проедет под тормозом со скорости v. На роботе выбег не
// пропорционален скорости: с малой скорости колесо встает почти сразу, поэтому
// модель - прямая со смещением. wheel < 0 - среднее по двум колесам
float UniBase::brakeLead(float v, bool spin, int8_t wheel) {
  uint8_t mode = spin ? 2 : 0;
  float k, drop;
  if (wheel < 0) {
    k = (_brakeK[mode] + _brakeK[mode + 1]) * 0.5f;
    drop = (_brakeC[mode] + _brakeC[mode + 1]) * 0.5f;
  } else {
    k = _brakeK[mode + wheel];
    drop = _brakeC[mode + wheel];
  }
  return fmaxf(0.0f, k * fmaxf(v, 0.0f) - drop);
}

// Подстройка обоих параметров модели колеса по одной остановке: шаг в сторону
// измеренной точки (скорость, выбег). Возвращает false, если точка не годится
bool UniBase::learnBrake(float v, float coast, bool spin, uint8_t wheel) {
  if (v < 10.0f) return false; // на такой скорости выбега практически нет
  uint8_t i = (spin ? 2 : 0) + wheel;
  // Первые остановки принимаются целиком: модель сразу попадает в измеренную точку
  float step = (_brakeStops[spin ? 1 : 0] < BRAKE_FAST_STOPS) ? 1.0f : BRAKE_LEARN;
  float x = v / BRAKE_SPEED_SCALE;
  float error = coast - (_brakeK[i] * v - _brakeC[i]);
  float norm = x * x + 1.0f;
  float k = _brakeK[i] * BRAKE_SPEED_SCALE + step * error * x / norm;
  float drop = _brakeC[i] - step * error / norm;
  _brakeK[i] = constrain(k / BRAKE_SPEED_SCALE, 0.0f, 0.15f);
  _brakeC[i] = constrain(drop, 0.0f, 5.0f);
  return true;
}

// eBase, eTurn - ошибки положения по ходу и по курсу (мм хода колеса),
// vL/vR и aL/aR - заданные скорости и ускорения колес
void UniBase::servoOutput(float eBase, float eTurn, float vL, float vR, float aL, float aR, bool spin, float dt) {
  // Ведущая ось мягче, поперечная жестче: на прямой робот держит курс,
  // в повороте - центр вращения
  float gBase = spin ? _t.crossAxisGain : _t.mainAxisGain;
  float gTurn = spin ? _t.mainAxisGain : _t.crossAxisGain;

  // Ошибка поперечной оси ограничивается, но не забывается: после помехи
  // робот вернется на заданный курс
  if (spin) eBase = constrain(eBase, -_t.crossLeashMM, _t.crossLeashMM);
  else eTurn = constrain(eTurn, -_t.crossLeashMM, _t.crossLeashMM);

  float vBaseRef = (vL + vR) * 0.5f;
  float vTurnRef = (vL - vR) * 0.5f;
  float vBaseAct = (_velL + _velR) * 0.5f;
  float vTurnAct = (_velL - _velR) * 0.5f;

  // Мягкое торможение: когда робот опережает задание, регулятор не давит назад
  // в полную силу - замедление получается без рывка
  float eBaseP = eBase;
  if (eBase * vBaseRef < 0) eBaseP *= _t.brakeGain;

  // D работает по ошибке скорости (заданная минус измеренная), а не по
  // производной квантованной ошибки положения
  float uBase = gBase * _d.kP * eBaseP + _iBase + _d.kD * (vBaseRef - vBaseAct);
  float uTurn = gTurn * _d.kP * eTurn + _iTurn + _d.kD * (vTurnRef - vTurnAct);

  float uL = feedforward(vL, aL, _vmaxL) + uBase + uTurn;
  float uR = feedforward(vR, aR, _vmaxR) + uBase - uTurn;

  // Интеграл ведущей оси не копится на разгоне (отставание там динамическое,
  // а не постоянное) и считается по той же ослабленной ошибке, что и P
  float eBaseI = eBaseP;
  float eTurnI = eTurn;
  float accNominal = spin ? _d.accSpin : _d.accMove;
  if (spin) {
    float aTurn = (aL - aR) * 0.5f;
    if (aTurn * vTurnRef > 0 && fabsf(aTurn) > 0.5f * accNominal) eTurnI = 0.0f;
  } else {
    float aBase = (aL + aR) * 0.5f;
    if (aBase * vBaseRef > 0 && fabsf(aBase) > 0.5f * accNominal) eBaseI = 0.0f;
  }

  // При насыщении интеграл может только разряжаться
  bool saturated = fmaxf(fabsf(uL), fabsf(uR)) > 100.0f;
  if (!saturated || eBaseI * _iBase < 0) {
    _iBase = constrain(_iBase + gBase * _d.kI * eBaseI * dt, -_t.iLimit, _t.iLimit);
  }
  if (!saturated || eTurnI * _iTurn < 0) {
    _iTurn = constrain(_iTurn + gTurn * _d.kI * eTurnI * dt, -_t.iLimit, _t.iLimit);
  }

  // Когда мощности не хватает, жертвуем ведущей осью, а поперечную сохраняем:
  // на прямой робот поедет медленнее, но не уйдет с курса
  float common = (uL + uR) * 0.5f;
  float diff = (uL - uR) * 0.5f;
  if (spin) {
    common = constrain(common, -100.0f, 100.0f);
    float room = 100.0f - fabsf(common);
    diff = constrain(diff, -room, room);
  } else {
    diff = constrain(diff, -100.0f, 100.0f);
    float room = 100.0f - fabsf(diff);
    common = constrain(common, -room, room);
  }
  uL = common + diff;
  uR = common - diff;

  float maxStep = _t.powerSlew * dt;
  _lastPowerL += constrain(uL - _lastPowerL, -maxStep, maxStep);
  _lastPowerR += constrain(uR - _lastPowerR, -maxStep, maxStep);
  _motors.setPower(_lastPowerL, _lastPowerR);
}

// Доводка одного колеса малым ходом. Внутри зоны - тормоз (inside = true), вне
// ее - мощность чуть ниже страгивания, которая растет, пока колесо не тронется
float UniBase::nudgePower(float err, float vel, float deadband, float gain, float& ramp, float dt, bool& inside) {
  // Тормозим с упреждением на выбег
  float lead = (err * vel > 0) ? brakeLead(fabsf(vel), false, -1) : 0.0f;
  if (fabsf(err) <= deadband + lead) {
    inside = true;
    ramp = 0.0f;
    return 0.0f;
  }

  inside = false;
  if (fabsf(vel) < 4.0f) ramp = fminf(ramp + NUDGE_RAMP * dt, NUDGE_RAMP_MAX);
  float u = (_t.minPower * NUDGE_START + ramp) * _ffComp + gain * fabsf(err);
  u = fminf(u, 100.0f);
  return (err > 0) ? u : -u;
}

void UniBase::driveOrBrake(float uL, bool brakeL, float uR, bool brakeR) {
  if (brakeL) { _motors.brakeLeft(); _lastPowerL = 0.0f; }
  else { _motors.setLeftPower(uL); _lastPowerL = uL; }

  if (brakeR) { _motors.brakeRight(); _lastPowerR = 0.0f; }
  else { _motors.setRightPower(uR); _lastPowerR = uR; }
}

void UniBase::releaseMotors() {
  _motors.setPower(0, 0);
  _lastPowerL = 0.0f;
  _lastPowerR = 0.0f;
  // Команда "моторы выключены" повторяется еще несколько циклов: одна потерянная
  // запись в ШИМ не должна оставить робота едущим
  _releaseBurst = RELEASE_BURST;
  _rawMode = false;
}

// Скорости колес, с которых новая команда подхватывает движение без рывка
void UniBase::currentRefVel(float& vL, float& vR) {
  if (_state == CS_PATH) {
    vL = _p.vRef * _p.cL;
    vR = _p.vRef * _p.cR;
  } else if (_state == CS_VELOCITY) {
    vL = _v.refL;
    vR = _v.refR;
  } else if (_state == CS_HOLD) {
    vL = 0.0f;
    vR = 0.0f;
  } else {
    // Робот едет на motors() или катится: берем измеренную скорость
    vL = (fabsf(_velL) > 8.0f) ? _velL : 0.0f;
    vR = (fabsf(_velR) > 8.0f) ? _velR : 0.0f;
  }
}

// ---- Состояния ----

void UniBase::stepIdle() {
  // Сторож покоя. Команда не выполняется и прямой мощности motors() нет - значит,
  // моторы обязаны молчать. Выключение повторяется сразу после остановки и
  // всякий раз, когда колеса в покое почему-то едут
  if (!_rawMode && !_rawBrakeL && !_rawBrakeR) {
    bool moving = fabsf(_velL) > 20.0f || fabsf(_velR) > 20.0f;
    if (_releaseBurst > 0 || (moving && (_nowMs - _idleAssertMs) >= 20)) {
      if (_releaseBurst > 0) _releaseBurst--;
      _idleAssertMs = _nowMs;
      _motors.setPower(0, 0);
    }
  }
  if (_rawBrakeL && (int32_t)(_nowMs - _rawBrakeUntilL) >= 0) {
    _rawBrakeL = false;
    _motors.setLeftPower(0);
    _lastPowerL = 0.0f;
    _stopDoneSeq = _stopSeq;
  }
  if (_rawBrakeR && (int32_t)(_nowMs - _rawBrakeUntilR) >= 0) {
    _rawBrakeR = false;
    _motors.setRightPower(0);
    _lastPowerR = 0.0f;
    _stopDoneSeq = _stopSeq;
  }
}

void UniBase::startBrake(AfterBrake after) {
  _rawMode = false;
  _motors.brakeBoth();
  _lastPowerL = 0.0f;
  _lastPowerR = 0.0f;
  _state = CS_BRAKE;
  _afterBrake = after;
  _brakeStartMs = _nowMs;
  _stillRefL = _cntL;
  _stillRefR = _cntR;
  _lastMotionMs = _nowMs;
}

// Тормоз держится до фактической остановки колес, а не фиксированное время
void UniBase::stepBrake() {
  uint32_t elapsed = _nowMs - _brakeStartMs;
  bool still = (_nowMs - _lastMotionMs) >= STILL_TIME_MS;
  uint32_t minMs = (_afterBrake == AB_RELEASE) ? (uint32_t)_t.brakeTimeMs : STILL_TIME_MS;
  if (!((still && elapsed >= minMs) || elapsed >= BRAKE_MAX_MS)) return;

  switch (_afterBrake) {
    case AB_VERIFY:
      verifyPath();
      break;
    case AB_FINISH:
      releaseMotors();
      _state = CS_IDLE;
      reportFinish(_brakeResult);
      break;
    default:
      releaseMotors();
      _state = CS_IDLE;
      _stopDoneSeq = _stopSeq;
      break;
  }
}

void UniBase::enterVelocity() {
  float vL, vR;
  currentRefVel(vL, vR);
  _v = Velocity();
  _v.startL = _cntL;
  _v.startR = _cntR;
  _v.refL = vL;
  _v.refR = vR;
  _v.spin = (vL * vR < 0);
  _v.decel = _v.spin ? _d.accSpin : _d.accMove;
  _iBase = 0.0f;
  _iTurn = 0.0f;
  _rawMode = false;
  _state = CS_VELOCITY;
}

void UniBase::stepVelocity(float dt) {
  Velocity& v = _v;

  // moveTime: замедление начинается заранее, чтобы к концу времени робот стоял
  if (v.timed && !v.stopping) {
    float stopTime = fmaxf(fabsf(v.refL), fabsf(v.refR)) / v.decel;
    if ((int32_t)(v.endMs - _nowMs) <= (int32_t)(stopTime * 1000.0f)) {
      v.stopping = true;
      v.cmdL = 0.0f;
      v.cmdR = 0.0f;
    }
  }

  // Заданные скорости обоих колес меняются согласованно: вектор скорости идет
  // к цели по прямой, поэтому соотношение колес (радиус дуги) держится и на разгоне
  float accUp = (v.spin ? _d.accSpin : _d.accMove) * _t.rampUpFactor;
  float dL = v.cmdL - v.refL;
  float dR = v.cmdR - v.refR;
  bool speedingUp = (v.cmdL * v.cmdL + v.cmdR * v.cmdR > v.refL * v.refL + v.refR * v.refR)
                    && (v.cmdL * v.refL + v.cmdR * v.refR >= 0);
  float limit = speedingUp ? accUp : (v.stopping ? v.decel : _d.maxDecel);
  float step = limit * dt;
  float biggest = fmaxf(fabsf(dL), fabsf(dR));
  float prevL = v.refL;
  float prevR = v.refR;
  if (biggest <= step) {
    v.refL = v.cmdL;
    v.refR = v.cmdR;
  } else {
    float k = step / biggest;
    v.refL += dL * k;
    v.refR += dR * k;
  }
  float aL = (v.refL - prevL) / dt;
  float aR = (v.refR - prevR) / dt;

  if (v.stopping && v.refL == 0.0f && v.refR == 0.0f) {
    if (v.timed) {
      _brakeResult = UNI_MOVE_OK;
      startBrake(AB_FINISH);
    } else {
      // Плавная остановка закончена: моторы отпущены
      releaseMotors();
      _state = CS_IDLE;
      _stopDoneSeq = _stopSeq;
    }
    return;
  }

  // Начало отсчета каждый цикл переносится в текущее положение колес: команда
  // может длиться сколько угодно, а числа остаются малыми и точными
  v.posRefL += v.refL * dt - (_cntL - v.startL) * _mmPerCountL;
  v.posRefR += v.refR * dt - (_cntR - v.startR) * _mmPerCountR;
  v.startL = _cntL;
  v.startR = _cntR;
  float errL = v.posRefL;
  float errR = v.posRefR;
  float eBase = (errL + errR) * 0.5f;
  float eTurn = (errL - errR) * 0.5f;

  // Поводки: у команды нет конечной точки, поэтому "долг" не копится - задание,
  // убежавшее от робота дальше поводка, подтягивается обратно
  float baseLeash = v.spin ? _t.crossLeashMM : _t.leashMM;
  float turnLeash = v.spin ? _t.leashMM : _t.crossLeashMM;
  if (fabsf(eBase) > baseLeash) {
    float excess = eBase - copysignf(baseLeash, eBase);
    v.posRefL -= excess;
    v.posRefR -= excess;
    eBase -= excess;
  }
  if (fabsf(eTurn) > turnLeash) {
    float excess = eTurn - copysignf(turnLeash, eTurn);
    v.posRefL -= excess;
    v.posRefR += excess;
    eTurn -= excess;
  }

  servoOutput(eBase, eTurn, v.refL, v.refR, aL, aR, v.spin, dt);
}

void UniBase::stepHold(float dt) {
  float errL = (_holdL - _cntL) * _mmPerCountL;
  float errR = (_holdR - _cntR) * _mmPerCountR;
  float gain = _d.kP * _t.crossAxisGain; // чем дальше сдвинули, тем сильнее возврат
  bool insideL, insideR;
  float uL = nudgePower(errL, _velL, HOLD_DEADBAND_MM, gain, _nudgeRampL, dt, insideL);
  float uR = nudgePower(errR, _velR, HOLD_DEADBAND_MM, gain, _nudgeRampR, dt, insideR);
  driveOrBrake(uL, insideL, uR, insideR);
}

// ---- Маневр с известной геометрией ----

// Что измерено по текущему пути: прогресс и остатки до цели по ходу и по курсу
void UniBase::pathMeasure(float& sAct, float& finBase, float& finTurn) {
  Path& p = _p;
  if (p.steer) {
    float x, y, heading;
    readPose(x, y, heading);
    // Остаток - проекция вектора до точки на заданный курс
    float rem = (p.goalX - x) * cosf(p.headingRef) + (p.goalY - y) * sinf(p.headingRef);
    sAct = p.S - rem;
    finBase = rem;
    finTurn = (p.headingRef - heading) * _halfTrack;
    return;
  }

  float posL = (_cntL - p.startL) * _mmPerCountL;
  float posR = (_cntR - p.startR) * _mmPerCountR;
  sAct = ((posL - p.oL) * p.cL + (posR - p.oR) * p.cR) / (p.cL * p.cL + p.cR * p.cR);
  float finL = p.oL + p.S * p.cL - posL;
  float finR = p.oR + p.S * p.cR - posR;
  finBase = (finL + finR) * 0.5f;
  finTurn = (finL - finR) * 0.5f;
}

void UniBase::startPath(Path& p, float power) {
  float vL0, vR0;
  currentRefVel(vL0, vR0);

  // Пределы скорости и ускорения относятся к самому быстрому колесу: на дуге
  // внешнее колесо не получит задание выше 100%
  float maxC = fmaxf(fabsf(p.cL), fabsf(p.cR));
  float norm = p.cL * p.cL + p.cR * p.cR;
  p.vMin = p.spin ? _d.vMinSpin : _d.vMinMove;
  float vTop = fmaxf(_vmaxAvg / maxC, p.vMin);
  p.vCruise = constrain(power * 0.01f * _vmaxAvg, p.vMin, vTop);
  p.accel = (p.spin ? _d.accSpin : _d.accMove) / maxC;

  p.startL = _cntL;
  p.startR = _cntR;
  p.sRef = p.steer ? 0.0f : -(p.oL * p.cL + p.oR * p.cR) / norm;
  p.vRef = constrain((vL0 * p.cL + vR0 * p.cR) / norm, 0.0f, p.vCruise);

  // Таймаут безопасности: 3x от расчетного времени движения + запас
  p.deadlineMs = _nowMs + 1500 + (uint32_t)(p.S / p.vCruise * 3000.0f);
  p.stallMarkS = -1e9f;
  p.stallMarkMs = _nowMs;
  p.crossMarkMs = _nowMs;

  _p = p;
  _rawMode = false;
  _nearFinish = false;
  _iBase = 0.0f;
  _iTurn = 0.0f;
  _state = CS_PATH;
}

void UniBase::stepPath(float dt) {
  Path& p = _p;

  // moveTo: курс на точку пересчитывается по одометрии, пока точка не слишком
  // близко (вплотную направление на нее меняется скачками)
  if (p.steer && !p.frozen) {
    float x, y, heading;
    readPose(x, y, heading);
    float dx = p.goalX - x;
    float dy = p.goalY - y;
    if (dx * dx + dy * dy > STEER_FREEZE_MM * STEER_FREEZE_MM) {
      p.headingRef = heading + wrapPi(atan2f(dy, dx) - heading);
    } else {
      p.frozen = true;
    }
  }

  float sAct, finBase, finTurn;
  pathMeasure(sAct, finBase, finTurn);
  float rem = p.S - sAct;
  float vAct = (_velL * p.cL + _velR * p.cR) / (p.cL * p.cL + p.cR * p.cR);

  if ((int32_t)(_nowMs - p.deadlineMs) > 0) {
    failPath(UNI_MOVE_TIMEOUT);
    return;
  }

  // Тормоз включается с упреждением на выбег: к остановке робот докатится до цели
  float lead = brakeLead(vAct, p.spin, -1);
  _nearFinish = (rem <= lead + fmaxf(vAct, 0.0f) * FINISH_ZONE_S + FINISH_ZONE_MM);

  if (rem <= lead) {
    // Оба колеса тормозятся одновременно: пока одно стоит, а второе едет,
    // робот разворачивается вокруг стоящего. Для подстройки модели выбега
    // запоминаем, где и с какой скоростью шло каждое колесо
    p.brakeS = sAct;
    p.brakeV = vAct;
    const float pos[2] = {(_cntL - p.startL) * _mmPerCountL, (_cntR - p.startR) * _mmPerCountR};
    const float vel[2] = {_velL, _velR};
    const float share[2] = {p.cL, p.cR};
    for (uint8_t w = 0; w < 2; w++) {
      float dir = (share[w] >= 0) ? 1.0f : -1.0f;
      p.wheelBraked[w] = (share[w] != 0.0f); // колесо, которое в маневре стоит, модель не учит
      p.wheelBrakePos[w] = pos[w];
      p.wheelBrakeV[w] = vel[w] * dir;
    }
    startBrake(AB_VERIFY);
    return;
  }

  // Внешний контур: измеренный остаток пути -> заданная скорость. Остаток
  // пересчитывается каждый цикл, поэтому инерция и проскальзывание закрываются сами
  float vCmd = sqrtf(2.0f * p.accel * (rem - lead));
  vCmd = constrain(vCmd, p.vMin, p.vCruise);

  float vPrev = p.vRef;
  if (vCmd > p.vRef) p.vRef = fminf(p.vRef + p.accel * _t.rampUpFactor * dt, vCmd);
  else p.vRef = fmaxf(p.vRef - _d.maxDecel * dt, vCmd);
  float aRef = (p.vRef - vPrev) / dt;

  // Поводок: задание не убегает от робота (и робот от задания) дальше поводка.
  // Задание при этом остается на траектории - курс не теряется
  p.sRef += p.vRef * dt;
  float lag = p.sRef - sAct;
  if (lag > _t.leashMM) p.sRef = sAct + _t.leashMM;
  else if (lag < -_t.leashMM) p.sRef = sAct - _t.leashMM;

  float eBase, eTurn;
  if (p.steer) {
    eBase = p.sRef - sAct;
    eTurn = (p.headingRef - _heading) * _halfTrack;
  } else {
    float errL = p.oL + p.sRef * p.cL - (_cntL - p.startL) * _mmPerCountL;
    float errR = p.oR + p.sRef * p.cR - (_cntR - p.startR) * _mmPerCountR;
    eBase = (errL + errR) * 0.5f;
    eTurn = (errL - errR) * 0.5f;
  }

  // Упор. По ходу: привод давит (задание натянуло поводок), а прогресса нет.
  // Поперек: ошибка курса (или центра в повороте) стоит на пределе - одно
  // колесо заблокировано
  if (_t.stallTimeMs > 0) {
    uint32_t stallMs = (uint32_t)_t.stallTimeMs;
    if (lag < 0.8f * _t.leashMM || sAct > p.stallMarkS + 0.5f) {
      p.stallMarkS = sAct;
      p.stallMarkMs = _nowMs;
    }
    float eCross = p.spin ? eBase : eTurn;
    if (fabsf(eCross) < 0.9f * _t.crossLeashMM) p.crossMarkMs = _nowMs;
    if ((_nowMs - p.stallMarkMs) > stallMs || (_nowMs - p.crossMarkMs) > stallMs) {
      failPath(UNI_MOVE_STALLED);
      return;
    }
  }

  // Оба колеса получают одно и то же задание прогресса, умноженное на свою долю:
  // соотношение их скоростей постоянно на разгоне, круизе и торможении
  servoOutput(eBase, eTurn, p.vRef * p.cL, p.vRef * p.cR, aRef * p.cL, aRef * p.cR, p.spin, dt);
}

void UniBase::failPath(UniMoveResult result) {
  _chainValid = false;
  _brakeResult = result;
  startBrake(AB_FINISH);
}

// Робот остановился после тормоза: смотрим, куда он попал на самом деле
void UniBase::verifyPath() {
  Path& p = _p;
  float sAct, finBase, finTurn;
  pathMeasure(sAct, finBase, finTurn);

  // Выбег под тормозом зависит от батареи, покрытия и нагрузки, поэтому
  // упреждение подстраивается по каждой остановке
  _work.brakeSpeed = p.brakeV;
  _work.coastMM = sAct - p.brakeS;
  uint8_t mode = p.spin ? 2 : 0;
  _work.brakeLeadMs = (uint16_t)((_brakeK[mode] + _brakeK[mode + 1]) * 500.0f);
  _work.brakeDropMM = (_brakeC[mode] + _brakeC[mode + 1]) * 0.5f;

  // Каждая остановка уточняет модель выбега того колеса, которое тормозило
  bool learned = false;
  const float pos[2] = {(_cntL - p.startL) * _mmPerCountL, (_cntR - p.startR) * _mmPerCountR};
  const float share[2] = {p.cL, p.cR};
  for (uint8_t w = 0; w < 2; w++) {
    if (!p.wheelBraked[w]) continue;
    float dir = (share[w] >= 0) ? 1.0f : -1.0f;
    learned |= learnBrake(p.wheelBrakeV[w], (pos[w] - p.wheelBrakePos[w]) * dir, p.spin, w);
  }
  if (learned && _brakeStops[p.spin ? 1 : 0] < 255) _brakeStops[p.spin ? 1 : 0]++;

  // Остаток после остановки не исправляется: робот не дергается в конце маневра.
  // Остаток уходит в отчет, а цепочка целей передает его следующей команде
  pathDone();
}

void UniBase::pathDone() {
  Path& p = _p;
  float sAct, finBase, finTurn;
  pathMeasure(sAct, finBase, finTurn);
  _lastErrMM = finBase;
  _lastErrDeg = finTurn / _halfTrack * RAD2DEG;

  // Следующая команда отсчитывает цель от цели этой, а не от места остановки
  _chainHeading = p.steer ? p.headingRef : p.headingTarget;
  _chainValid = true;
  _chainAlongValid = p.straight;
  _chainAlong = p.straight ? finBase : 0.0f;

  if (p.next == NEXT_MOVETO_DRIVE) {
    beginMoveToDrive(); // довернулись на курс к цели - едем прямой участок
    return;
  }

  releaseMotors();
  _state = CS_IDLE;
  reportFinish(UNI_MOVE_OK);
}

bool UniBase::chainUsable() {
  return _cfg.chainTargets && _chainValid && fabsf(_chainHeading - _heading) <= CHAIN_MAX_HEADING;
}

void UniBase::beginDist(float power, float dist) {
  float h0 = _heading;
  if (chainUsable()) {
    h0 = _chainHeading;
    if (_chainAlongValid && fabsf(_chainAlong) <= CHAIN_MAX_ALONG_MM) dist += _chainAlong;
  }

  if (fabsf(dist) <= _d.tolBase) {
    // Вместе с остатком прошлой прямой ехать уже некуда
    _chainHeading = h0;
    _chainValid = true;
    _chainAlong = dist;
    _chainAlongValid = true;
    _lastErrMM = dist;
    _lastErrDeg = (h0 - _heading) * RAD2DEG;
    finishImmediately(UNI_MOVE_OK);
    return;
  }

  // Остаток курса прошлого маневра становится стартовой ошибкой поперечной оси:
  // привод исправит его на первых сантиметрах пути
  float dir = (dist > 0) ? 1.0f : -1.0f;
  float off = (h0 - _heading) * _halfTrack;
  Path p;
  p.cL = dir;
  p.cR = dir;
  p.S = fabsf(dist);
  p.oL = off;
  p.oR = -off;
  p.straight = true;
  p.headingTarget = h0;
  startPath(p, power);
}

void UniBase::beginRotate(float power, float angleDeg, bool absolute) {
  float rad = angleDeg * DEG2RAD;
  float target;
  if (absolute) {
    // Кратчайший доворот к курсу в системе getAngle()
    target = _heading + wrapPi((_angleZero + rad) - _heading);
  } else {
    target = (chainUsable() ? _chainHeading : _heading) + rad;
  }
  beginSpin(target, power, NEXT_NONE);
}

void UniBase::beginSpin(float target, float power, NextPhase next) {
  float delta = target - _heading;
  float arc = fabsf(delta) * _halfTrack;

  if (arc <= _d.tolTurn) {
    // Уже на курсе: запоминаем цель, остаток уйдет следующей команде
    _chainHeading = target;
    _chainValid = true;
    _chainAlongValid = false;
    if (next == NEXT_MOVETO_DRIVE) {
      beginMoveToDrive();
      return;
    }
    _lastErrMM = 0.0f;
    _lastErrDeg = delta * RAD2DEG;
    finishImmediately(UNI_MOVE_OK);
    return;
  }

  float side = (delta > 0) ? 1.0f : -1.0f;
  Path p;
  p.cL = side;
  p.cR = -side;
  p.S = arc;
  p.spin = true;
  p.headingTarget = target;
  p.next = next;
  startPath(p, power);
}

// k - кривизна: колеса проходят (1 + k) и (1 - k) на единицу пути центра
void UniBase::beginArc(float power, float k, float length, float dir) {
  float h0 = chainUsable() ? _chainHeading : _heading;
  float off = (h0 - _heading) * _halfTrack;
  Path p;
  p.cL = dir * (1.0f + k);
  p.cR = dir * (1.0f - k);
  p.S = length;
  p.oL = off;
  p.oR = -off;
  p.spin = (p.cL * p.cR < 0); // внутреннее колесо едет назад: это уже скорее поворот
  p.headingTarget = h0 + dir * 2.0f * k * length / _cfg.trackLengthMM;
  startPath(p, power);
}

void UniBase::beginMoveTo(float power, float x, float y) {
  _goalX = x;
  _goalY = y;
  _goalPower = power;

  float curX, curY, heading;
  readPose(curX, curY, heading);
  float dx = x - curX;
  float dy = y - curY;
  if (sqrtf(dx * dx + dy * dy) <= _d.tolBase) {
    _lastErrMM = 0.0f;
    _lastErrDeg = 0.0f;
    finishImmediately(UNI_MOVE_OK);
    return;
  }

  // Фаза 1: кратчайший доворот на курс к цели. Фаза 2 (прямая до точки)
  // начинается после завершения поворота
  beginSpin(heading + wrapPi(atan2f(dy, dx) - heading), power, NEXT_MOVETO_DRIVE);
}

void UniBase::beginMoveToDrive() {
  float curX, curY, heading;
  readPose(curX, curY, heading);
  float dx = _goalX - curX;
  float dy = _goalY - curY;
  float dist = sqrtf(dx * dx + dy * dy);

  if (dist <= _d.tolBase) {
    _lastErrMM = dist;
    _lastErrDeg = 0.0f;
    finishImmediately(UNI_MOVE_OK);
    return;
  }

  Path p;
  p.S = dist;
  p.steer = true;
  p.goalX = _goalX;
  p.goalY = _goalY;
  p.headingRef = heading + wrapPi(atan2f(dy, dx) - heading);
  p.headingTarget = p.headingRef;
  startPath(p, _goalPower);
}

// ---- Команды ----

void UniBase::beginFinite(const MotionCmd& cmd) {
  _curSeq = cmd.seq;
  _curFinite = true;
  _work = UniMoveReport();
  _moveStartMs = _nowMs;
  portENTER_CRITICAL(&_cmdMux);
  // Назад номер не двигаем: более новая команда может уже ждать в очереди
  if ((int32_t)(cmd.seq - _busySeq) >= 0) {
    _busySeq = cmd.seq;
    _activeFinite = true;
  }
  portEXIT_CRITICAL(&_cmdMux);
}

void UniBase::reportFinish(UniMoveResult result) {
  if (!_curFinite) return;
  _curFinite = false;

  _work.result = result;
  _work.errorMM = _lastErrMM;
  _work.errorDeg = _lastErrDeg;
  _work.timeMs = _nowMs - _moveStartMs;

  portENTER_CRITICAL(&_cmdMux);
  _result = result;
  _resultSeq = _curSeq;
  _report = _work;
  if (_busySeq == _curSeq) _activeFinite = false;
  portEXIT_CRITICAL(&_cmdMux);

  if (result != UNI_MOVE_INTERRUPTED) setCommandName("Ready");
  xSemaphoreGive(_moveSemaphore);
}

// Команда закончилась, не начавшись (цель уже достигнута). Если робот при этом
// едет по прошлой команде, сначала останавливаем его
void UniBase::finishImmediately(UniMoveResult result) {
  if (_state == CS_IDLE || _state == CS_HOLD) {
    reportFinish(result);
  } else {
    _brakeResult = result;
    startBrake(AB_FINISH);
  }
}

void UniBase::applyStop(const MotionCmd& cmd) {
  if (_curFinite) {
    _chainValid = false;
    reportFinish(UNI_MOVE_INTERRUPTED);
  }
  _stopSeq = cmd.seq;

  if (cmd.mask != 3) {
    // Одно колесо: привод снимается, второе колесо сохраняет свою мощность
    if (_state != CS_IDLE) {
      _chainValid = false;
      _rawMode = true; // второе колесо едет дальше на своей мощности без привода
    }
    _state = CS_IDLE;
    _releaseBurst = 0; // второе колесо должно сохранить свою мощность
    bool left = (cmd.mask & 1);
    if (cmd.stopType == HARD) {
      if (left) { _motors.brakeLeft(); _lastPowerL = 0.0f; _rawBrakeL = true; _rawBrakeUntilL = _nowMs + WHEEL_BRAKE_MS; }
      else { _motors.brakeRight(); _lastPowerR = 0.0f; _rawBrakeR = true; _rawBrakeUntilR = _nowMs + WHEEL_BRAKE_MS; }
    } else {
      if (left) { _motors.setLeftPower(0); _lastPowerL = 0.0f; _rawBrakeL = false; }
      else { _motors.setRightPower(0); _lastPowerR = 0.0f; _rawBrakeR = false; }
      _stopDoneSeq = cmd.seq;
    }
    return;
  }

  _rawBrakeL = false;
  _rawBrakeR = false;
  if (_state == CS_VELOCITY || _state == CS_PATH) _chainValid = false;

  if (cmd.stopType == HARD) {
    startBrake(AB_RELEASE);
    return;
  }

  // SOFT: замедление по профилю следящим приводом, затем моторы отпускаются
  if (_state == CS_PATH) enterVelocity(); // подхватывает текущие скорости колес
  if (_state == CS_VELOCITY) {
    _v.cmdL = 0.0f;
    _v.cmdR = 0.0f;
    _v.timed = false;
    _v.stopping = true;
    _v.decel = _v.spin ? _d.accSpin : _d.accMove;
  } else {
    releaseMotors();
    _state = CS_IDLE;
    _stopDoneSeq = cmd.seq;
  }
}

void UniBase::applyCommand(const MotionCmd& cmd) {
  _appliedSeq = cmd.seq;
  if (_odomReset) {
    _odomReset = false;
    _chainValid = false; // поза задана заново: остатки прошлых маневров не к чему привязать
  }

  if (cmd.type == MC_STOP) {
    applyStop(cmd);
    return;
  }

  // Новая команда вытесняет текущую
  if (_curFinite) reportFinish(UNI_MOVE_INTERRUPTED);

  switch (cmd.type) {
    case MC_RAW:
      // Прямая мощность без стабилизации. Колесо вне маски сохраняет свою мощность
      _chainValid = false;
      _state = CS_IDLE;
      _releaseBurst = 0;
      _rawMode = true;
      if (cmd.mask & 1) {
        _rawBrakeL = false;
        _lastPowerL = constrain(cmd.a, -100.0f, 100.0f);
        _motors.setLeftPower(_lastPowerL);
      }
      if (cmd.mask & 2) {
        _rawBrakeR = false;
        _lastPowerR = constrain(cmd.b, -100.0f, 100.0f);
        _motors.setRightPower(_lastPowerR);
      }
      return;

    case MC_VELOCITY:
    case MC_TIMED:
      _chainValid = false;
      _rawBrakeL = false;
      _rawBrakeR = false;
      if (cmd.type == MC_TIMED) beginFinite(cmd);
      // Привод, который уже держит скорости, не сбрасывается: меняется только задание
      if (_state != CS_VELOCITY) enterVelocity();
      _v.cmdL = cmd.a;
      _v.cmdR = cmd.b;
      _v.stopping = false;
      _v.timed = (cmd.type == MC_TIMED);
      if (cmd.a != 0.0f || cmd.b != 0.0f) _v.spin = (cmd.a * cmd.b < 0);
      _v.decel = _v.spin ? _d.accSpin : _d.accMove;
      if (_v.timed) _v.endMs = _nowMs + (uint32_t)cmd.c;
      return;

    case MC_HOLD:
      _rawBrakeL = false;
      _rawBrakeR = false;
      _rawMode = false;
      _state = CS_HOLD;
      _holdL = _cntL;
      _holdR = _cntR;
      _nudgeRampL = 0.0f;
      _nudgeRampR = 0.0f;
      return;

    default:
      break;
  }

  _rawBrakeL = false;
  _rawBrakeR = false;
  beginFinite(cmd);
  switch (cmd.type) {
    case MC_DIST:      beginDist(cmd.power, cmd.a); break;
    case MC_ROTATE:    beginRotate(cmd.power, cmd.a, false); break;
    case MC_ROTATE_TO: beginRotate(cmd.power, cmd.a, true); break;
    case MC_ARC:       beginArc(cmd.power, cmd.a, cmd.b, cmd.c); break;
    case MC_MOVE_TO:   beginMoveTo(cmd.power, cmd.a, cmd.b); break;
    default:           reportFinish(UNI_MOVE_INTERRUPTED); break;
  }
}

// ============ Постановка команд ============

// Команда уходит в очередь контура управления. Возвращает ее номер (0 - не принята)
uint32_t UniBase::submit(MotionCmd& cmd, bool finite) {
  // Номер и место в очереди выдаются под одним замком: порядок номеров
  // совпадает с порядком выполнения, даже если команды шлют две задачи
  xSemaphoreTake(_submitMutex, portMAX_DELAY);

  portENTER_CRITICAL(&_cmdMux);
  if (++_cmdSeq == 0) _cmdSeq = 1;
  cmd.seq = _cmdSeq;
  _activeFinite = finite;
  if (finite) _busySeq = cmd.seq;
  portEXIT_CRITICAL(&_cmdMux);

  bool queued = (xQueueSend(_cmdQueue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE);
  if (!queued && finite) {
    portENTER_CRITICAL(&_cmdMux);
    if (_busySeq == cmd.seq) _activeFinite = false;
    portEXIT_CRITICAL(&_cmdMux);
  }

  xSemaphoreGive(_submitMutex);
  return queued ? cmd.seq : 0;
}

// Ждет завершения конечной команды. true - цель достигнута
bool UniBase::waitFinite(uint32_t seq) {
  if (seq == 0) return true; // движения не требовалось

  for (;;) {
    portENTER_CRITICAL(&_cmdMux);
    bool done = !(_activeFinite && _busySeq == seq);
    portEXIT_CRITICAL(&_cmdMux);
    if (done) break;
    xSemaphoreTake(_moveSemaphore, pdMS_TO_TICKS(10));
  }

  portENTER_CRITICAL(&_cmdMux);
  bool ok = (_resultSeq == seq && _result == UNI_MOVE_OK);
  portEXIT_CRITICAL(&_cmdMux);
  return ok;
}

void UniBase::waitStop(uint32_t seq, uint32_t timeoutMs) {
  if (seq == 0) return;
  uint32_t start = millis();
  // Ждем, пока остановка завершится или ее вытеснит более новая команда
  while (_stopDoneSeq != seq && (int32_t)(_appliedSeq - seq) <= 0) {
    if (millis() - start >= timeoutMs) break;
    vTaskDelay(1);
  }
}

// ============ Display ============
void UniBase::setCommandName(const char* name) {
  portENTER_CRITICAL(&_dispMux);
  strncpy(_currentCommand, name, sizeof(_currentCommand)-1);
  _currentCommand[sizeof(_currentCommand)-1] = '\0';
  portEXIT_CRITICAL(&_dispMux);
}

void UniBase::updateDisplay() {
  if (!_displayInitialized) return;

  // Снимаем копии строк под блокировкой: их пишут с другого ядра
  char cmdBuf[sizeof(_currentCommand)];
  char nameBuf[sizeof(_customDisplayName)];
  char textBuf[sizeof(_customDisplayText)];
  char pageBuf[sizeof(_displayPage)];
  portENTER_CRITICAL(&_dispMux);
  bool customMode = _customDisplayMode;
  bool pageMode = _displayPageMode;
  if (pageMode) memcpy(pageBuf, _displayPage, sizeof(pageBuf));
  memcpy(cmdBuf, _currentCommand, sizeof(cmdBuf));
  memcpy(nameBuf, _customDisplayName, sizeof(nameBuf));
  memcpy(textBuf, _customDisplayText, sizeof(textBuf));
  portEXIT_CRITICAL(&_dispMux);
  cmdBuf[sizeof(cmdBuf)-1] = '\0';
  nameBuf[sizeof(nameBuf)-1] = '\0';
  textBuf[sizeof(textBuf)-1] = '\0';

  _display->clearDisplay();

  if (pageMode) {
    // Мелкий шрифт: 8 строк по 21 символу, перенос по '\n'
    pageBuf[sizeof(pageBuf) - 1] = '\0';
    _display->setTextSize(1);
    _display->setTextWrap(false);
    int row = 0;
    const char* line = pageBuf;
    while (*line && row < 8) {
      const char* end = strchr(line, '\n');
      int len = end ? (int)(end - line) : (int)strlen(line);
      _display->setCursor(0, row * 8);
      for (int i = 0; i < len && i < 21; i++) _display->write(line[i]);
      if (!end) break;
      line = end + 1;
      row++;
    }
  } else if (customMode) {
    _display->setTextSize(2);
    int16_t x1, y1; uint16_t w, h;

    if (strlen(nameBuf) > 0) {
      _display->getTextBounds(nameBuf, 0, 0, &x1, &y1, &w, &h);
      _display->setCursor((_cfg.screenWidth - w) / 2, 0);
      _display->print(nameBuf);

      _display->getTextBounds(textBuf, 0, 0, &x1, &y1, &w, &h);
      _display->setCursor((_cfg.screenWidth - w) / 2, (_cfg.screenHeight + 10 - h) / 2);
      _display->print(textBuf);
    } else {
      _display->getTextBounds(textBuf, 0, 0, &x1, &y1, &w, &h);
      _display->setCursor((_cfg.screenWidth - w) / 2, (_cfg.screenHeight - h) / 2);
      _display->print(textBuf);
    }
  } else {
    _display->setTextSize(2);
    int16_t x1, y1; uint16_t w, h;
    _display->getTextBounds(_robotName, 0, 0, &x1, &y1, &w, &h);
    _display->setCursor((_cfg.screenWidth - w) / 2, 0);
    _display->print(_robotName);

    float dispX, dispY, dispT;
    readPose(dispX, dispY, dispT);
    dispT = wrapPi(dispT);

    _display->setTextSize(1);
    _display->setCursor(0, 18); _display->print("X: "); _display->print(dispX, 0); _display->print(" mm");
    _display->setCursor(70, 18); _display->print("Y: "); _display->print(dispY, 0); _display->print(" mm");
    _display->setCursor(0, 30); _display->print("Angle: "); _display->print(dispT * RAD2DEG, 0); _display->print((char)247);
    _display->setCursor(0, 42); _display->print("L: "); _display->print(_encoders.getLeftTicks());
    _display->setCursor(70, 42); _display->print("R: "); _display->print(_encoders.getRightTicks());
    _display->setCursor(0, 55); _display->print(cmdBuf);

    if (_batteryValid && _batteryPercent >= 0) {
      drawBatteryIcon(104, 53, _batteryPercent);
      _display->setTextSize(1);
      char batBuf[8];
      snprintf(batBuf, sizeof(batBuf), "%d%%", _batteryPercent);
      _display->getTextBounds(batBuf, 0, 0, &x1, &y1, &w, &h);
      _display->setCursor(102 - w, 55);
      _display->print(batBuf);
    }
  }
  _display->display();
}

void UniBase::drawBatteryIcon(int x, int y, int percent) {
  const int batteryWidth = 20;
  const int batteryHeight = 10;
  const int tipWidth = 2;
  const int tipHeight = 6;
  _display->drawRect(x, y, batteryWidth, batteryHeight, SSD1306_WHITE);
  _display->fillRect(x + batteryWidth, y + (batteryHeight - tipHeight) / 2, tipWidth, tipHeight, SSD1306_WHITE);
  int fillWidth = ((batteryWidth - 4) * percent) / 100;
  if (fillWidth > 0) {
    _display->fillRect(x + 2, y + 2, fillWidth, batteryHeight - 4, SSD1306_WHITE);
  }
}

// ============ Async API ============

bool UniBase::isMoving() {
  return _activeFinite;
}

uint32_t UniBase::startDist(int power, int millimeters) {
  ensureBegun();
  if (power == 0 || millimeters == 0) return 0;

  setCommandName("moveDist");
  // Назад едем при отрицательной мощности или отрицательной дистанции
  bool backward = (power < 0) || (millimeters < 0);
  MotionCmd cmd = {};
  cmd.type = MC_DIST;
  cmd.power = constrain(abs(power), 1, 100);
  cmd.a = backward ? -(float)abs(millimeters) : (float)abs(millimeters);
  return submit(cmd, true);
}

// Скорости колес в % от максимальной; время в мс
uint32_t UniBase::startTime(float powerLeft, float powerRight, int milliseconds) {
  if (milliseconds <= 0) return 0;

  // Соотношение колес сохраняется: если одно задание выше 100%, уменьшаются оба
  float biggest = fmaxf(fabsf(powerLeft), fabsf(powerRight));
  if (biggest > 100.0f) {
    powerLeft *= 100.0f / biggest;
    powerRight *= 100.0f / biggest;
  }

  MotionCmd cmd = {};
  cmd.type = MC_TIMED;
  cmd.a = powerLeft * 0.01f * _vmaxL;
  cmd.b = powerRight * 0.01f * _vmaxR;
  cmd.c = (float)milliseconds;
  return submit(cmd, true);
}

// Дуга: k - кривизна (доля разницы хода колес), длина по центру робота
uint32_t UniBase::startArc(int power, float k, float lengthMM, bool backward) {
  MotionCmd cmd = {};
  cmd.type = MC_ARC;
  cmd.power = constrain(abs(power), 1, 100);
  cmd.a = k;
  cmd.b = lengthMM;
  cmd.c = backward ? -1.0f : 1.0f;
  return submit(cmd, true);
}

uint32_t UniBase::startRotate(int power, float angleDeg, bool absolute) {
  MotionCmd cmd = {};
  cmd.type = absolute ? MC_ROTATE_TO : MC_ROTATE;
  cmd.power = constrain(abs(power), 1, 100);
  cmd.a = angleDeg;
  return submit(cmd, true);
}

uint32_t UniBase::startMoveTo(int power, float x, float y) {
  MotionCmd cmd = {};
  cmd.type = MC_MOVE_TO;
  cmd.power = constrain(abs(power), 1, 100);
  cmd.a = x;
  cmd.b = y;
  return submit(cmd, true);
}

void UniBase::moveDistAsync(int power, int millimeters) {
  startDist(power, millimeters);
}

void UniBase::moveTimeAsync(int power, int milliseconds) {
  ensureBegun();
  setCommandName("moveTime");
  startTime(power, power, milliseconds);
}

void UniBase::moveArcDistAsync(int power, int angle, int millimeters) {
  ensureBegun();
  if (power == 0 || millimeters == 0) return;

  setCommandName("moveArcD");
  // angle - соотношение колес в модели (1 +/- angle/90)
  bool backward = (power < 0) || (millimeters < 0);
  startArc(power, angle / 90.0f, (float)abs(millimeters), backward);
}

void UniBase::moveArcRadiusAsync(int power, float radiusMM, float angleDeg) {
  ensureBegun();
  // Радиус меньше половины колеи дугой не реализуется - используйте rotate()
  if (power == 0 || angleDeg == 0 || radiusMM < _halfTrack) return;

  setCommandName("moveArcR");
  // Соотношение скоростей колес из геометрии дуги: vL/vR = (R + d/2)/(R - d/2)
  float k = _halfTrack / radiusMM;
  if (angleDeg < 0) k = -k;
  float arcMM = fabsf(angleDeg) * DEG2RAD * radiusMM; // длина дуги по центру
  startArc(power, k, arcMM, power < 0);
}

void UniBase::moveArcTimeAsync(int power, int angle, int milliseconds) {
  ensureBegun();
  setCommandName("moveArcT");
  startTime(power * (1.0f + angle / 90.0f), power * (1.0f - angle / 90.0f), milliseconds);
}

void UniBase::rotateAsync(int power, int angle) {
  ensureBegun();
  if (angle == 0 || power == 0) return;

  setCommandName("rotate");
  startRotate(power, (float)angle, false);
}

void UniBase::rotateToAsync(int power, float angleDeg) {
  ensureBegun();
  if (power == 0) return;

  setCommandName("rotateTo");
  startRotate(power, angleDeg, true);
}

void UniBase::moveToAsync(int power, float x, float y) {
  ensureBegun();
  if (power == 0) return;

  setCommandName("moveTo");
  startMoveTo(power, x, y);
}

// ============ Blocking API wrappers ============

bool UniBase::moveDist(int power, int millimeters) {
  return waitFinite(startDist(power, millimeters));
}

bool UniBase::moveTime(int power, int milliseconds) {
  ensureBegun();
  setCommandName("moveTime");
  return waitFinite(startTime(power, power, milliseconds));
}

bool UniBase::moveArcDist(int power, int angle, int millimeters) {
  ensureBegun();
  if (power == 0 || millimeters == 0) return true;

  setCommandName("moveArcD");
  bool backward = (power < 0) || (millimeters < 0);
  return waitFinite(startArc(power, angle / 90.0f, (float)abs(millimeters), backward));
}

bool UniBase::moveArcTime(int power, int angle, int milliseconds) {
  ensureBegun();
  setCommandName("moveArcT");
  return waitFinite(startTime(power * (1.0f + angle / 90.0f), power * (1.0f - angle / 90.0f), milliseconds));
}

bool UniBase::rotate(int power, int angle) {
  ensureBegun();
  if (angle == 0 || power == 0) return true;

  setCommandName("rotate");
  return waitFinite(startRotate(power, (float)angle, false));
}

bool UniBase::rotateTo(int power, float angleDeg) {
  ensureBegun();
  if (power == 0) return true;

  setCommandName("rotateTo");
  return waitFinite(startRotate(power, angleDeg, true));
}

bool UniBase::moveTo(int power, float x, float y) {
  ensureBegun();
  if (power == 0) return true;

  setCommandName("moveTo");
  return waitFinite(startMoveTo(power, x, y));
}

bool UniBase::moveArcRadius(int power, float radiusMM, float angleDeg) {
  ensureBegun();
  if (power == 0 || angleDeg == 0 || radiusMM < _halfTrack) return true;

  setCommandName("moveArcR");
  float k = _halfTrack / radiusMM;
  if (angleDeg < 0) k = -k;
  float arcMM = fabsf(angleDeg) * DEG2RAD * radiusMM;
  return waitFinite(startArc(power, k, arcMM, power < 0));
}

bool UniBase::waitMove(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (isMoving()) {
    if (timeoutMs > 0 && millis() - start >= timeoutMs) return false;
    delay(5);
  }
  return true;
}

UniMoveResult UniBase::getMoveResult() {
  portENTER_CRITICAL(&_cmdMux);
  UniMoveResult result = _result;
  portEXIT_CRITICAL(&_cmdMux);
  return result;
}

float UniBase::getMoveErrorMM() { return _lastErrMM; }
float UniBase::getMoveErrorDeg() { return _lastErrDeg; }

UniMoveReport UniBase::getMoveReport() {
  portENTER_CRITICAL(&_cmdMux);
  UniMoveReport report = _report;
  portEXIT_CRITICAL(&_cmdMux);
  return report;
}

// ============ Motor basics & Stop ============

void UniBase::motors(int powerLeft, int powerRight) {
  ensureBegun();
  setCommandName("motors");
  MotionCmd cmd = {};
  cmd.type = MC_RAW;
  cmd.mask = 3;
  cmd.a = powerLeft;
  cmd.b = powerRight;
  submit(cmd, false);
}

void UniBase::motorLeft(int power) {
  ensureBegun();
  setCommandName("motorL");
  MotionCmd cmd = {};
  cmd.type = MC_RAW;
  cmd.mask = 1;
  cmd.a = power;
  submit(cmd, false);
}

void UniBase::motorRight(int power) {
  ensureBegun();
  setCommandName("motorR");
  MotionCmd cmd = {};
  cmd.type = MC_RAW;
  cmd.mask = 2;
  cmd.b = power;
  submit(cmd, false);
}

// Скорости колес в % от максимальной
void UniBase::submitVelocity(float powerLeft, float powerRight) {
  // Соотношение колес сохраняется: если одно задание выше 100%, уменьшаются оба
  float biggest = fmaxf(fabsf(powerLeft), fabsf(powerRight));
  if (biggest > 100.0f) {
    powerLeft *= 100.0f / biggest;
    powerRight *= 100.0f / biggest;
  }

  MotionCmd cmd = {};
  cmd.type = MC_VELOCITY;
  cmd.a = powerLeft * 0.01f * _vmaxL;
  cmd.b = powerRight * 0.01f * _vmaxR;
  submit(cmd, false);
}

void UniBase::motorsArc(int power, float angle) {
  ensureBegun();
  setCommandName("motorsArc");
  submitVelocity(power * (1.0f + angle / 90.0f), power * (1.0f - angle / 90.0f));
}

void UniBase::motorsSync(int powerLeft, int powerRight) {
  ensureBegun();
  setCommandName("motorsSync");
  submitVelocity(powerLeft, powerRight);
}

void UniBase::motorsSpeed(float leftMMperSec, float rightMMperSec) {
  ensureBegun();
  setCommandName("motorsSpeed");
  submitVelocity(leftMMperSec / _vmaxL * 100.0f, rightMMperSec / _vmaxR * 100.0f);
}

void UniBase::holdPosition() {
  ensureBegun();
  setCommandName("hold");
  // Привод активно удерживает текущую позицию колес
  MotionCmd cmd = {};
  cmd.type = MC_HOLD;
  submit(cmd, false);
}

void UniBase::stop(int stopType) {
  ensureBegun();
  setCommandName("stop");
  MotionCmd cmd = {};
  cmd.type = MC_STOP;
  cmd.mask = 3;
  cmd.stopType = (stopType == HARD) ? HARD : SOFT;
  uint32_t seq = submit(cmd, false);
  // HARD возвращается, когда робот уже стоит: следующая команда стартует с места.
  // SOFT не ждет: замедление идет в фоне
  if (stopType == HARD) waitStop(seq, BRAKE_MAX_MS + 100);
}

void UniBase::stopLeft(int stopType) {
  ensureBegun();
  setCommandName("stopL");
  MotionCmd cmd = {};
  cmd.type = MC_STOP;
  cmd.mask = 1;
  cmd.stopType = (stopType == HARD) ? HARD : SOFT;
  uint32_t seq = submit(cmd, false);
  if (stopType == HARD) waitStop(seq, WHEEL_BRAKE_MS + 50);
}

void UniBase::stopRight(int stopType) {
  ensureBegun();
  setCommandName("stopR");
  MotionCmd cmd = {};
  cmd.type = MC_STOP;
  cmd.mask = 2;
  cmd.stopType = (stopType == HARD) ? HARD : SOFT;
  uint32_t seq = submit(cmd, false);
  if (stopType == HARD) waitStop(seq, WHEEL_BRAKE_MS + 50);
}

// ============ Speed ============

float UniBase::getMaxSpeed() {
  ensureBegun();
  return _vmaxAvg;
}

int UniBase::speedToPower(float mmPerSec) {
  ensureBegun();
  return constrain((int)lroundf(mmPerSec / _vmaxAvg * 100.0f), -100, 100);
}

int UniBase::turnRateToPower(float degPerSec) {
  ensureBegun();
  // При повороте на месте колеса идут по окружности радиусом в половину колеи
  float wheelSpeed = degPerSec * DEG2RAD * _halfTrack;
  return constrain((int)lroundf(wheelSpeed / _vmaxAvg * 100.0f), -100, 100);
}

float UniBase::getSpeed() {
  ensureBegun();
  return (_velL + _velR) * 0.5f;
}

float UniBase::getTurnRate() {
  ensureBegun();
  return (_velL - _velR) / _cfg.trackLengthMM * RAD2DEG;
}

// ============ Odometry ============

void UniBase::resetDistance() {
    ensureBegun();
    portENTER_CRITICAL(&_odomMux);
    _totalDistance = 0.0f;
    _distAcc = 0.0f;
    portEXIT_CRITICAL(&_odomMux);
}
float UniBase::getDistance() {
    ensureBegun();
    portENTER_CRITICAL(&_odomMux);
    float v = _totalDistance + fabsf(_distAcc);
    portEXIT_CRITICAL(&_odomMux);
    return v;
}
void UniBase::resetAngle() {
    ensureBegun();
    portENTER_CRITICAL(&_odomMux);
    _angleZero = _heading;
    portEXIT_CRITICAL(&_odomMux);
}
float UniBase::getAngle() {
    ensureBegun();
    portENTER_CRITICAL(&_odomMux);
    float v = (_heading - _angleZero) * RAD2DEG;
    portEXIT_CRITICAL(&_odomMux);
    return v;
}
void UniBase::setPosition(float x, float y, float angleDeg) {
    ensureBegun();
    float a = angleDeg * DEG2RAD;
    portENTER_CRITICAL(&_odomMux);
    _xPos = x;
    _yPos = y;
    _headingOffset = (double)a - _headingRaw;
    _heading = a;
    _angleZero = 0.0f;
    _odomReset = true;
    portEXIT_CRITICAL(&_odomMux);
}

OdometryData UniBase::getOdometry() {
    ensureBegun();
    OdometryData data;
    readPose(data.x, data.y, data.angle);
    data.angle = wrapPi(data.angle) * RAD2DEG;
    return data;
}
float UniBase::getAbsX() {
    ensureBegun();
    portENTER_CRITICAL(&_odomMux);
    float v = _xPos;
    portEXIT_CRITICAL(&_odomMux);
    return v;
}
float UniBase::getAbsY() {
    ensureBegun();
    portENTER_CRITICAL(&_odomMux);
    float v = _yPos;
    portEXIT_CRITICAL(&_odomMux);
    return v;
}
float UniBase::getAbsAngle() {
    ensureBegun();
    portENTER_CRITICAL(&_odomMux);
    float v = _heading;
    portEXIT_CRITICAL(&_odomMux);
    return wrapPi(v) * RAD2DEG;
}
long UniBase::getLeftTicks() { ensureBegun(); return _encoders.getLeftTicks(); }
long UniBase::getRightTicks() { ensureBegun(); return _encoders.getRightTicks(); }
uint32_t UniBase::getEncoderGlitches() { ensureBegun(); return _encoders.glitches(); }

// ============ Display ============

void UniBase::displayPrint(const char* text) {
    ensureBegun();
    portENTER_CRITICAL(&_dispMux);
    strncpy(_customDisplayText, text, sizeof(_customDisplayText) - 1);
    _customDisplayText[sizeof(_customDisplayText) - 1] = '\0';
    _customDisplayName[0] = '\0';
    _customDisplayMode = true;
    _displayPageMode = false;
    portEXIT_CRITICAL(&_dispMux);
}
void UniBase::displayPrint(int value) { char buf[16]; snprintf(buf, sizeof(buf), "%d", value); displayPrint(buf); }
void UniBase::displayPrint(long value) { char buf[16]; snprintf(buf, sizeof(buf), "%ld", value); displayPrint(buf); }
void UniBase::displayPrint(float value) { char buf[16]; snprintf(buf, sizeof(buf), "%.2f", value); displayPrint(buf); }
void UniBase::displayPrint(double value) { char buf[16]; snprintf(buf, sizeof(buf), "%.4f", value); displayPrint(buf); }
void UniBase::displayPrint(bool value) { displayPrint(value ? "true" : "false"); }

void UniBase::displayPrint(const char* name, const char* value) {
    ensureBegun();
    portENTER_CRITICAL(&_dispMux);
    strncpy(_customDisplayName, name, sizeof(_customDisplayName) - 1);
    _customDisplayName[sizeof(_customDisplayName) - 1] = '\0';
    strncpy(_customDisplayText, value, sizeof(_customDisplayText) - 1);
    _customDisplayText[sizeof(_customDisplayText) - 1] = '\0';
    _customDisplayMode = true;
    _displayPageMode = false;
    portEXIT_CRITICAL(&_dispMux);
}
void UniBase::displayPrint(const char* name, int value) { char buf[16]; snprintf(buf, sizeof(buf), "%d", value); displayPrint(name, buf); }
void UniBase::displayPrint(const char* name, long value) { char buf[16]; snprintf(buf, sizeof(buf), "%ld", value); displayPrint(name, buf); }
void UniBase::displayPrint(const char* name, float value) { char buf[16]; snprintf(buf, sizeof(buf), "%.2f", value); displayPrint(name, buf); }
void UniBase::displayPrint(const char* name, double value) { char buf[16]; snprintf(buf, sizeof(buf), "%.4f", value); displayPrint(name, buf); }
void UniBase::displayPrint(const char* name, bool value) { displayPrint(name, value ? "true" : "false"); }

void UniBase::displayText(const char* text) {
    ensureBegun();
    portENTER_CRITICAL(&_dispMux);
    strncpy(_displayPage, text, sizeof(_displayPage) - 1);
    _displayPage[sizeof(_displayPage) - 1] = '\0';
    _displayPageMode = true;
    portEXIT_CRITICAL(&_dispMux);
    _displayDirty = true;
}

void UniBase::displayClear() {
    ensureBegun();
    portENTER_CRITICAL(&_dispMux);
    _customDisplayMode = false;
    _displayPageMode = false;
    _customDisplayText[0] = '\0';
    _customDisplayName[0] = '\0';
    portEXIT_CRITICAL(&_dispMux);
    // Рисует только служебная задача: кадр отсюда столкнулся бы с ее кадром на шине I2C
    _displayDirty = true;
}

// ============ Utility ============

void UniBase::printOdometry() {
  ensureBegun();
  OdometryData odom = getOdometry();
  Serial.printf("encL: %ld \tencR: %ld \tX: %.2f \tY: %.2f \tTheta: %.2f\n",
    _encoders.getLeftTicks(), _encoders.getRightTicks(), odom.x, odom.y, odom.angle);
}

void UniBase::blinkLED(int interval) {
  ensureBegun();
  if (interval > 0) { _ledBlinkInterval = interval; _ledBlinking = true; }
  else { _ledBlinking = false; _ledBlinkInterval = 0; digitalWrite(_cfg.ledPin, LOW); }
}

float UniBase::readBatteryVoltage() {
  return (analogReadMilliVolts(_cfg.batteryPin) / 1000.0f) * 2.0f;
}

int UniBase::percentFromVoltage(float batteryVoltage) {
  static const float lipoTable[][2] = {
    {3.20,0}, {3.50,8}, {3.70,22}, {3.80,40}, {3.96,72}, {4.20,100}
  };

  float percent = 0;
  if (batteryVoltage >= 4.20) percent = 100.0;
  else if (batteryVoltage <= 3.20) percent = 0.0;
  else {
    for (int i = 1; i < 6; i++) {
      if (batteryVoltage <= lipoTable[i][0]) {
        float v0 = lipoTable[i-1][0], p0 = lipoTable[i-1][1];
        float v1 = lipoTable[i][0], p1 = lipoTable[i][1];
        percent = p0 + (p1 - p0) * (batteryVoltage - v0) / (v1 - v0);
        break;
      }
    }
  }
  return constrain((int)(percent + 0.5), 0, 100);
}

// Вызывается служебной задачей: сглаженное напряжение, процент заряда для экрана
// и поправка мощности моторов на заряд
void UniBase::updateBattery() {
  float voltage = readBatteryVoltage();
  if (voltage < 2.5f || voltage > 4.5f) {
    _batteryValid = false;
    _batteryPercent = -1;
    _batteryVoltage = 0.0f;
    _ffComp = 1.0f;
    return;
  }

  // Под нагрузкой напряжение проседает и шумит, поэтому берется сглаженное
  float filtered = _batteryVoltage;
  filtered = (filtered <= 0.0f) ? voltage : filtered + 0.2f * (voltage - filtered);
  _batteryVoltage = filtered;
  _batteryPercent = percentFromVoltage(filtered);
  _batteryValid = true;

  // Калибровка скорости снята при batteryRefVoltage: на севшей батарее
  // той же скорости нужна пропорционально большая мощность
  float comp = 1.0f;
  if (_cfg.batteryRefVoltage > 0) {
    comp = constrain(_cfg.batteryRefVoltage / filtered, BATTERY_COMP_MIN, BATTERY_COMP_MAX);
  }
  _ffComp = comp;
}

int UniBase::getBatteryPower() {
  ensureBegun();
  float batteryVoltage = readBatteryVoltage();
  if (batteryVoltage < 2.5 || batteryVoltage > 4.5) return -1;
  return percentFromVoltage(batteryVoltage);
}

float UniBase::getBatteryVoltage() {
  ensureBegun();
  return _batteryVoltage;
}

// ============ UART Control Protocol (COBS + CRC8) ============

void UniBase::UniBaseControl() {
  ensureBegun();
  if (_ctrlInitialized) return;
  _ctrlSerial = new HardwareSerial(2);
  _ctrlSerial->begin(_cfg.uartBaudRate, SERIAL_8N1, _cfg.uartRxPin, _cfg.uartTxPin);
  _ctrlBufferIndex = 0;
  _ctrlInitialized = true;
}

void UniBase::sendPacket(const uint8_t* payload, uint8_t len) {
    uint8_t crc = UniProtocol::crc8(payload, len);
    uint8_t fullPayload[32];
    memcpy(fullPayload, payload, len);
    fullPayload[len] = crc;

    uint8_t encoded[35];
    size_t encLen = UniProtocol::cobsEncode(fullPayload, len + 1, encoded);

    _ctrlSerial->write(UBC_SYNC_BYTE);
    _ctrlSerial->write(encoded, encLen);
    _ctrlSerial->write(UBC_SYNC_BYTE);
}

void UniBase::ctrlSendFloat(float val) { sendPacket((uint8_t*)&val, 4); }
void UniBase::ctrlSendLong(long val) { sendPacket((uint8_t*)&val, 4); }

void UniBase::ctrlReceiveUART() {
  while (_ctrlSerial->available() > 0) {
    uint8_t b = _ctrlSerial->read();

    if (b == UBC_SYNC_BYTE) {
        if (_ctrlBufferIndex > 0) {
            size_t decLen = UniProtocol::cobsDecode(_ctrlBuffer, _ctrlBufferIndex, _decodeBuffer);
            _ctrlBufferIndex = 0;

            if (decLen >= 2) {
                uint8_t rxCrc = _decodeBuffer[decLen - 1];
                uint8_t calcCrc = UniProtocol::crc8(_decodeBuffer, decLen - 1);
                if (rxCrc == calcCrc) {
                    uartDispatchCommand(_decodeBuffer, decLen - 1);
                }
            }
        }
    } else {
        if (_ctrlBufferIndex < _cfg.uartBufferSize) {
            _ctrlBuffer[_ctrlBufferIndex++] = b;
        } else {
            _ctrlBufferIndex = 0;
        }
    }
  }
}

// Минимальная длина пакета (включая байт команды); переменная часть
// команд дисплея проверяется дополнительно в их обработчиках
static uint8_t uartRequiredLen(uint8_t cmd) {
  switch (cmd) {
    case CMD_MOTORS:              return 3;
    case CMD_STOP:                return 2;
    case CMD_MOVE_DIST:           return 4;
    case CMD_ROTATE:              return 4;
    case CMD_MOTORS_ARC:          return 4;
    case CMD_MOTORS_SYNC:         return 3;
    case CMD_ROTATE_TO:           return 4;
    case CMD_MOVE_TO:             return 6;
    case CMD_MOVE_ARC_RADIUS:     return 6;
    case CMD_SET_POSITION:        return 7;
    case CMD_MOVE_TIME:           return 4;
    case CMD_MOVE_ARC_TIME:       return 6;
    case CMD_MOTOR_LEFT:          return 2;
    case CMD_MOTOR_RIGHT:         return 2;
    case CMD_STOP_LEFT:           return 2;
    case CMD_STOP_RIGHT:          return 2;
    case CMD_DISPLAY_PRINT:       return 2;
    case CMD_BLINK_LED:           return 3;
    case CMD_MOVE_ARC_DIST:       return 6;
    case CMD_DISPLAY_PRINT_NAMED: return 3;
    default:                      return 1;
  }
}

void UniBase::uartDispatchCommand(uint8_t* data, uint8_t len) {
  uint8_t cmd = data[0];
  if (len < uartRequiredLen(cmd)) return; // битый/обрезанный пакет
  switch (cmd) {
    case CMD_MOTORS: motors((int8_t)data[1], (int8_t)data[2]); break;
    case CMD_STOP: stop(data[1]); break;
    case CMD_MOVE_DIST: moveDistAsync((int8_t)data[1], (int)((uint16_t)data[2] << 8 | data[3])); break;
    case CMD_ROTATE: rotateAsync((int8_t)data[1], (int16_t)((data[2] << 8) | data[3])); break;
    case CMD_GET_DISTANCE: ctrlSendFloat(getDistance()); break;
    case CMD_GET_ANGLE: ctrlSendFloat(getAngle()); break;
    case CMD_GET_ODOMETRY: {
      OdometryData od = getOdometry();
      uint8_t buf[12];
      memcpy(buf, &od.x, 4); memcpy(buf+4, &od.y, 4); memcpy(buf+8, &od.angle, 4);
      sendPacket(buf, 12);
      break;
    }
    case CMD_MOTORS_ARC: motorsArc((int8_t)data[1], (float)((int16_t)((data[2] << 8) | data[3]))); break;
    case CMD_MOTORS_SYNC: motorsSync((int8_t)data[1], (int8_t)data[2]); break;
    case CMD_ROTATE_TO: rotateToAsync((int8_t)data[1], (float)(int16_t)((data[2] << 8) | data[3])); break;
    case CMD_MOVE_TO: moveToAsync((int8_t)data[1], (float)(int16_t)((data[2] << 8) | data[3]), (float)(int16_t)((data[4] << 8) | data[5])); break;
    case CMD_MOVE_ARC_RADIUS: moveArcRadiusAsync((int8_t)data[1], (float)(uint16_t)((data[2] << 8) | data[3]), (float)(int16_t)((data[4] << 8) | data[5])); break;
    case CMD_SET_POSITION: setPosition((float)(int16_t)((data[1] << 8) | data[2]), (float)(int16_t)((data[3] << 8) | data[4]), (float)(int16_t)((data[5] << 8) | data[6])); break;
    case CMD_HOLD_POSITION: holdPosition(); break;
    case CMD_MOVE_TIME: moveTimeAsync((int8_t)data[1], (int)((uint16_t)data[2] << 8 | data[3])); break;
    case CMD_MOVE_ARC_TIME: moveArcTimeAsync((int8_t)data[1], (int16_t)((data[2] << 8) | data[3]), (int)((uint16_t)data[4] << 8 | data[5])); break;
    case CMD_MOTOR_LEFT: motorLeft((int8_t)data[1]); break;
    case CMD_MOTOR_RIGHT: motorRight((int8_t)data[1]); break;
    case CMD_STOP_LEFT: stopLeft(data[1]); break;
    case CMD_STOP_RIGHT: stopRight(data[1]); break;
    case CMD_RESET_DIST: resetDistance(); break;
    case CMD_RESET_ANGLE: resetAngle(); break;
    case CMD_GET_ABS_X: ctrlSendFloat(getAbsX()); break;
    case CMD_GET_ABS_Y: ctrlSendFloat(getAbsY()); break;
    case CMD_GET_ABS_ANGLE: ctrlSendFloat(getAbsAngle()); break;
    case CMD_GET_L_TICKS: ctrlSendLong(getLeftTicks()); break;
    case CMD_GET_R_TICKS: ctrlSendLong(getRightTicks()); break;
    case CMD_DISPLAY_PRINT: {
      char buf[21];
      uint8_t cLen = min(data[1], (uint8_t)20);
      if (cLen > len - 2) cLen = len - 2; // не читаем за концом пакета
      for (uint8_t i = 0; i < cLen; i++) buf[i] = (char)data[2 + i];
      buf[cLen] = '\0';
      displayPrint(buf);
      break;
    }
    case CMD_DISPLAY_CLEAR: displayClear(); break;
    case CMD_PRINT_ODOM: printOdometry(); break;
    case CMD_BLINK_LED: blinkLED(((uint16_t)data[1] << 8) | data[2]); break;
    case CMD_GET_BATTERY: {
      int batt = getBatteryPower();
      uint8_t val = (batt < 0) ? 0xFF : (uint8_t)batt;
      sendPacket(&val, 1);
      break;
    }
    case CMD_MOVE_ARC_DIST: moveArcDistAsync((int8_t)data[1], (int16_t)((data[2] << 8) | data[3]), (int)((uint16_t)data[4] << 8 | data[5])); break;
    case CMD_DISPLAY_PRINT_NAMED: {
      char nameBuf[12];
      char valBuf[12];
      uint8_t nLen = min(data[1], (uint8_t)10);
      if (2 + nLen + 1 > len) break; // имя + байт длины значения не помещаются
      uint8_t idx = 2;
      for (uint8_t i = 0; i < nLen; i++) nameBuf[i] = (char)data[idx++];
      nameBuf[nLen] = '\0';

      uint8_t vLen = min(data[idx++], (uint8_t)10);
      if (idx + vLen > len) break; // значение не помещается
      for (uint8_t i = 0; i < vLen; i++) valBuf[i] = (char)data[idx++];
      valBuf[vLen] = '\0';

      displayPrint(nameBuf, valBuf);
      break;
    }
    case CMD_IS_MOVING: {
      uint8_t ans = isMoving() ? 1 : 0;
      sendPacket(&ans, 1);
      break;
    }
  }
}
