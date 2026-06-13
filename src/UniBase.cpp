#include "UniBase.h"

UniBase::UniBase(const char* robotName, UniConfig config)
  : _cfg(config), _display(nullptr), _displayInitialized(false), _customDisplayMode(false),
    _ledBlinking(false), _ledBlinkInterval(0),
    _batteryPercent(-1), _batteryValid(false),
    _xPos(0.0), _yPos(0.0), _theta(0.0), _totalDistance(0.0), _totalAngle(0.0),
    _secondCore(nullptr),
    _moveState(STATE_IDLE), _targetTheta(0.0),
    _rotateCruiseVel(0.0),
    _encMoveActive(false), _encMovePowerL(0), _encMovePowerR(0),
    _encMoveStartL(0), _encMoveStartR(0),
    _encPrevTime(0), _targetPosL(0), _targetPosR(0), _currentTargetVelL(0), _currentTargetVelR(0), _encPrevErrL(0), _encPrevErrR(0), _IL(0), _IR(0), _lastLeftPower(0), _lastRightPower(0),
    _ctrlSerial(nullptr), _ctrlInitialized(false),
    _ctrlBuffer(nullptr), _decodeBuffer(nullptr), _ctrlBufferIndex(0),
    _begun(false)
{
  _ctrlBuffer = new uint8_t[_cfg.uartBufferSize];
  _decodeBuffer = new uint8_t[_cfg.uartBufferSize];
  
  strncpy(_robotName, robotName, sizeof(_robotName) - 1);
  _robotName[sizeof(_robotName) - 1] = '\0';
  
  strncpy(_currentCommand, "Idle", sizeof(_currentCommand) - 1);
  _currentCommand[sizeof(_currentCommand) - 1] = '\0';
  
  _customDisplayName[0] = '\0';
  _customDisplayText[0] = '\0';

  _distPerTickLeft = (PI * _cfg.wheelDiameterMM) / _cfg.ticksPerRevLeft;
  _distPerTickRight = (PI * _cfg.wheelDiameterMM) / _cfg.ticksPerRevRight;
  
  _moveSemaphore = xSemaphoreCreateBinary();

  _display = new Adafruit_SSD1306(_cfg.screenWidth, _cfg.screenHeight, &Wire, _cfg.oledReset);
}

UniBase::~UniBase() {
  if (_secondCore) vTaskDelete(_secondCore);
  if (_display) delete _display;
  if (_moveSemaphore) vSemaphoreDelete(_moveSemaphore);
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

  #ifdef DEBUG_MODE
  Serial.println("\n\nUNI Platform - Industrial Refactoring");
  Serial.print("Robot name: ");
  Serial.println(_robotName);
  #endif

  _motors.begin(_cfg.leftMotorA, _cfg.leftMotorB, _cfg.rightMotorA, _cfg.rightMotorB);
  _encoders.begin(_cfg.leftEncInt, _cfg.leftEncDir, _cfg.rightEncInt, _cfg.rightEncDir);
  
  initOLED();
  _uartLastRxTime = millis();
  
  pinMode(_cfg.ledPin, OUTPUT);
  digitalWrite(_cfg.ledPin, LOW);
  pinMode(_cfg.batteryPin, INPUT);
  analogReadResolution(12);
  
  initSecondCore();
  getBatteryPower();
  
  #ifdef DEBUG_MODE
  Serial.println("Robot ready");
  #endif
  
  setCommandName("Ready");
  // Дисплей отрисует второе ядро в своем цикле: рисовать отсюда нельзя,
  // одновременные кадры с двух ядер ломают шину I2C на самом старте
}

void UniBase::ensureBegun() {
  if (!_begun) begin();
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

void UniBase::initSecondCore() {
  // Передаем this как pvParameters
  xTaskCreatePinnedToCore(
    secondCoreLoop,
    "Odometry",
    10000,
    this,
    1,
    &_secondCore,
    0);
  delay(10);
}

// ============ Internal methods ============

void UniBase::updateOdometry() {
  _encoders.update(); 
  
  long lEncI = _encoders.getLeftTicks();
  long rEncI = _encoders.getRightTicks();
  
  float SL = (lEncI - _lEncOdomOld) * _distPerTickLeft;
  float SR = (rEncI - _rEncOdomOld) * _distPerTickRight;
  
  _lEncOdomOld = lEncI;
  _rEncOdomOld = rEncI;
  
  float deltaDistance = (abs(SL) + abs(SR)) / 2.0;
  float deltaAngle = (SL - SR) / _cfg.trackLengthMM;
  
  // Вход в критическую секцию: защищаем от Data Tearing
  portENTER_CRITICAL(&_odomMux);
  
  _totalDistance += deltaDistance;
  _totalAngle += deltaAngle;
  
  _xPos += ((SR + SL) / 2.0) * cos(_theta + ((SL - SR) / (2.0 * _cfg.trackLengthMM)));
  _yPos += ((SR + SL) / 2.0) * sin(_theta + ((SL - SR) / (2.0 * _cfg.trackLengthMM)));
  _theta += deltaAngle;
  
  if (_theta > PI) _theta -= 2 * PI;
  else if (_theta < -PI) _theta += 2 * PI;
  
  portEXIT_CRITICAL(&_odomMux);
}

void UniBase::encMove(float pL, float pR, long startLEnc, long startREnc) {
  long funcLEnc = _encoders.getLeftTicks() - startLEnc;
  long funcREnc = _encoders.getRightTicks() - startREnc;
  
  unsigned long nowTime = micros();
  double dt = (nowTime - _encPrevTime) / 1000000.0;
  if (dt <= 0.0) dt = 0.0001;
  _encPrevTime = nowTime;
  
  float rawTargetVelL = (pL / 100.0f) * _cfg.maxTicksPerSec;
  float rawTargetVelR = (pR / 100.0f) * _cfg.maxTicksPerSec;
  
  bool isRotation = (pL * pR < 0);

  // Лимит разгона чуть выше замедления внешнего профиля, чтобы сервопривод
  // успевал отслеживать тормозную кривую без накопления отставания
  float accL_limit = isRotation ? _tuning.rotateAccel * 1.15f : _tuning.moveAccel * 1.15f;
  float accR_limit = accL_limit;

  // Adaptive physics: smooth the primary axis, rigidify the secondary axis
  float accL = (abs(rawTargetVelL) > abs(_currentTargetVelL) && rawTargetVelL * _currentTargetVelL >= 0) ? accL_limit : 6000.0f;
  float accR = (abs(rawTargetVelR) > abs(_currentTargetVelR) && rawTargetVelR * _currentTargetVelR >= 0) ? accR_limit : 6000.0f;
  
  accL *= dt;
  accR *= dt;
  
  if (_currentTargetVelL < rawTargetVelL) _currentTargetVelL = min(_currentTargetVelL + accL, rawTargetVelL);
  else if (_currentTargetVelL > rawTargetVelL) _currentTargetVelL = max(_currentTargetVelL - accL, rawTargetVelL);
  
  if (_currentTargetVelR < rawTargetVelR) _currentTargetVelR = min(_currentTargetVelR + accR, rawTargetVelR);
  else if (_currentTargetVelR > rawTargetVelR) _currentTargetVelR = max(_currentTargetVelR - accR, rawTargetVelR);
  
  _targetPosL += _currentTargetVelL * dt;
  _targetPosR += _currentTargetVelR * dt;
  
  float errL = _targetPosL - funcLEnc;
  float errR = _targetPosR - funcREnc;
  
  // Kinematic Decomposition
  float eBase = (errL + errR) / 2.0f;
  float eTurn = (errL - errR) / 2.0f;
  
  // Короткие поводки: позицию ведет внешний контур по одометрии, внутреннему
  // длинная память не нужна. Большой поводок копил "долг" на длинных дистанциях
  // (когда реальная скорость ниже заданной) и развязывал его в конце рывком
  float baseLeash = isRotation ? 30.0f : 40.0f;
  float turnLeash = isRotation ? 40.0f : 30.0f;
  
  // Anti-windup / Catch-up prevention for Translation (Base)
  float excessBase = 0;
  if (eBase > baseLeash) excessBase = eBase - baseLeash;
  else if (eBase < -baseLeash) excessBase = eBase + baseLeash;
  
  if (excessBase != 0) {
      _targetPosL -= excessBase;
      _targetPosR -= excessBase;
      errL -= excessBase;
      errR -= excessBase;
      eBase -= excessBase;
  }
  
  // Anti-windup for Rotation (Turn)
  float excessTurn = 0;
  if (eTurn > turnLeash) excessTurn = eTurn - turnLeash;
  else if (eTurn < -turnLeash) excessTurn = eTurn + turnLeash;
  
  if (excessTurn != 0) {
      _targetPosL -= excessTurn;
      _targetPosR += excessTurn;
      errL -= excessTurn;
      errR += excessTurn;
      eTurn -= excessTurn;
  }
  
  // Soften base braking
  float activeBaseErr = eBase;
  float expectedBaseVel = (_currentTargetVelL + _currentTargetVelR) / 2.0f;
  if (eBase * expectedBaseVel < 0) activeBaseErr *= 0.2f;
  
  // Base Translation PID
  float P_Base = _tuning.pGain * (isRotation ? 2.0f : 0.8f);
  float I_Base = _tuning.iGain * (isRotation ? 2.0f : 5.0f);
  
  float PL_Base = P_Base * activeBaseErr;
  _IL = _IL + I_Base * eBase * dt;
  _IL = constrain(_IL, -30.0f, 30.0f);
  
  float errBaseDiff = (eBase - _encPrevErrL) / dt;
  float DL_Base = (_tuning.dGain + 0.05f) * errBaseDiff;
  float U_Base = PL_Base + _IL + DL_Base;
  
  // Turn Rotation PID (Cross-Coupling)
  float P_Turn = _tuning.pGain * (isRotation ? 0.8f : 2.0f);
  float I_Turn = _tuning.iGain * (isRotation ? 5.0f : 2.0f);
  
  float PL_Turn = P_Turn * eTurn;
  _IR = _IR + I_Turn * eTurn * dt;
  _IR = constrain(_IR, -30.0f, 30.0f);
  
  float errTurnDiff = (eTurn - _encPrevErrR) / dt;
  float DL_Turn = (_tuning.dGain + 0.05f) * errTurnDiff;
  float U_Turn = PL_Turn + _IR + DL_Turn;
  
  _encPrevErrL = eBase;
  _encPrevErrR = eTurn;
  
  float currentPowerL = 0;
  if (abs(_currentTargetVelL) > 0.1f) {
      float signL = (_currentTargetVelL > 0) ? 1.0f : -1.0f;
      float velRatioL = abs(_currentTargetVelL) / _cfg.maxTicksPerSec;
      currentPowerL = signL * (_tuning.minPower + velRatioL * (100.0f - _tuning.minPower));
  }
  
  float currentPowerR = 0;
    if (abs(_currentTargetVelR) > 0.1f) {
        float signR = (_currentTargetVelR > 0) ? 1.0f : -1.0f;
        float velRatioR = abs(_currentTargetVelR) / _cfg.maxTicksPerSec;
        currentPowerR = signR * (_tuning.minPower + velRatioR * (100.0f - _tuning.minPower));
    }
    
    float targetLeftPower = currentPowerL + U_Base + U_Turn;
    float targetRightPower = currentPowerR + U_Base - U_Turn;
    
    const float maxPowerChange = 10.0;
    float leftPowerChange = constrain(targetLeftPower - _lastLeftPower, -maxPowerChange, maxPowerChange);
    float rightPowerChange = constrain(targetRightPower - _lastRightPower, -maxPowerChange, maxPowerChange);
    
    _lastLeftPower = constrain(_lastLeftPower + leftPowerChange, -100.0, 100.0);
    _lastRightPower = constrain(_lastRightPower + rightPowerChange, -100.0, 100.0);
    
    _motors.setPower(_lastLeftPower, _lastRightPower);
}

void UniBase::startEncMoveOnSecondCore(float powerL, float powerR) {
  _cmdSeq++; // новая команда: устаревший stop() не должен гасить ее своим хвостом
  _encMovePowerL = powerL;
  _encMovePowerR = powerR;
  _encMoveStartL = _encoders.getLeftTicks();
  _encMoveStartR = _encoders.getRightTicks();
  _encPrevTime = micros();
  _targetPosL = 0;
  _targetPosR = 0;
  _currentTargetVelL = 0;
  _currentTargetVelR = 0;
  _IL = 0;
  _IR = 0;
  _encPrevErrL = 0;
  _encPrevErrR = 0;
  _lastLeftPower = 0;
  _lastRightPower = 0;
  _encMoveActive = true;
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
  portENTER_CRITICAL(&_dispMux);
  bool customMode = _customDisplayMode;
  memcpy(cmdBuf, _currentCommand, sizeof(cmdBuf));
  memcpy(nameBuf, _customDisplayName, sizeof(nameBuf));
  memcpy(textBuf, _customDisplayText, sizeof(textBuf));
  portEXIT_CRITICAL(&_dispMux);
  cmdBuf[sizeof(cmdBuf)-1] = '\0';
  nameBuf[sizeof(nameBuf)-1] = '\0';
  textBuf[sizeof(textBuf)-1] = '\0';

  _display->clearDisplay();

  if (customMode) {
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

    portENTER_CRITICAL(&_odomMux);
    float dispX = _xPos, dispY = _yPos, dispT = _theta;
    portEXIT_CRITICAL(&_odomMux);

    _display->setTextSize(1);
    _display->setCursor(0, 18); _display->print("X: "); _display->print(dispX, 0); _display->print(" mm");
    _display->setCursor(70, 18); _display->print("Y: "); _display->print(dispY, 0); _display->print(" mm");
    _display->setCursor(0, 30); _display->print("Angle: "); _display->print(dispT * 180 / PI, 0); _display->print((char)247);
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

// ============ ASYNC STATE MACHINE ============

void UniBase::processAsyncMovement() {
  if (_moveState == STATE_IDLE) return;

  switch (_moveState) {
    case STATE_MOVE_DIST:
    case STATE_MOVE_ARC_DIST:
    {
      // Внешний контур: остаток дистанции по энкодерам -> заданная скорость.
      // Тот же замкнутый профиль, что и у поворота: ошибка пересчитывается
      // каждый цикл, открытых компенсаций торможения нет.
      float curL = _encoders.getLeftTicks() - _targetStartLEnc;
      float curR = _encoders.getRightTicks() - _targetStartREnc;
      float cur = (curL + curR) / 2.0f; // знаковая дистанция центра (тики)
      float remTicks = (float)_targetDistTicks * _asyncDirection - cur;
      float absRem = abs(remTicks);
      float tolTicks = (_tuning.moveTolMM / (_cfg.wheelDiameterMM * PI)) * _cfg.ticksPerRevLeft;

      // Завершаем при входе в допуск ИЛИ при пересечении цели: назад не сдаем,
      // перелет фиксируется тормозом вместо реверсного "доезда"
      bool crossed = (remTicks * (float)_asyncDirection) < 0;
      if (absRem <= tolTicks || crossed || millis() > _targetEndTime) {
          setCommandName("Ready");
          stop(HARD); // семафор отдается внутри stop() последним действием
          return;
      }

      // Профиль целится в ближний край допуска: скорость обнуляется
      // ровно на входе в зону цели, тормоз гасит остаток инерции
      float aimErr = absRem - tolTicks * 0.5f;
      float v = sqrtf(2.0f * _tuning.moveAccel * aimErr);
      v = min(v, (_asyncAbsPower / 100.0f) * _cfg.maxTicksPerSec);
      v = max(v, _tuning.moveMinSpeed);

      float p = (v / _cfg.maxTicksPerSec) * 100.0f * ((remTicks > 0) ? 1.0f : -1.0f);
      if (_moveState == STATE_MOVE_ARC_DIST) {
          _encMovePowerL = p * (1.0f + _targetAngleDeg / 90.0f);
          _encMovePowerR = p * (1.0f - _targetAngleDeg / 90.0f);
      } else {
          _encMovePowerL = p;
          _encMovePowerR = p;
      }
      break;
    }

    case STATE_MOVE_TIME:
    case STATE_MOVE_ARC_TIME:
      if (millis() >= _targetEndTime) {
          setCommandName("Ready");
          stop(HARD);
      }
      break;

    case STATE_ROTATE_PROFILE:
    case STATE_MOVETO_TURN:
    {
      // Внешний контур: ошибка по углу одометрии -> заданная скорость колес.
      // Ошибка пересчитывается каждый цикл, поэтому инерция и проскальзывание
      // закрываются автоматически, без компенсаций разомкнутого контура.
      portENTER_CRITICAL(&_odomMux);
      float errRad = _targetTheta - _totalAngle;
      portEXIT_CRITICAL(&_odomMux);

      float halfTrack = _cfg.trackLengthMM / 2.0f;
      float errTicks = errRad * halfTrack / _distPerTickLeft; // остаток дуги левого колеса (тики)
      float tolTicks = (_tuning.rotateTolDeg * PI / 180.0f) * halfTrack / _distPerTickLeft;
      float absErr = abs(errTicks);

      // Завершение с первого захода: вошли в допуск — чёткий тормоз, без доводки
      if (absErr <= tolTicks || millis() > _targetEndTime) {
          if (_moveState == STATE_MOVETO_TURN && millis() <= _targetEndTime) {
              beginMoveToDrive(); // довернулись на курс к цели — едем прямой участок
          } else {
              setCommandName("Ready");
              stop(HARD);
          }
          break;
      }

      // Профиль целится в ближний край допуска, а не в его центр: скорость
      // обнуляется ровно на входе в зону цели — без перелёта и обратной доводки
      float aimErr = absErr - tolTicks * 0.5f;
      float v = sqrtf(2.0f * _tuning.rotateAccel * aimErr);
      v = min(v, (float)_rotateCruiseVel);
      v = max(v, _tuning.rotateMinSpeed);

      float p = (v / _cfg.maxTicksPerSec) * 100.0f;
      float dir = (errTicks > 0) ? 1.0f : -1.0f;
      _encMovePowerL = p * dir;
      _encMovePowerR = -p * dir;
      break;
    }

    default:
      break;
  }
}

// ============ FreeRTOS Task ============

void UniBase::secondCoreLoop(void* pvParameters) {
  UniBase* self = static_cast<UniBase*>(pvParameters);
  if (!self) vTaskDelete(NULL);

  delay(10);
  unsigned long lastDisplayUpdate = 0;
  const unsigned long displayUpdateInterval = 100;
  unsigned long lastLedToggle = 0;
  unsigned long lastBatteryUpdate = 0;
  const unsigned long batteryUpdateInterval = 2000;
  bool ledState = false;
  
  for (;;) {
    self->updateOdometry();
    self->processAsyncMovement();
    
    if (self->_encMoveActive) {
      self->encMove(self->_encMovePowerL, self->_encMovePowerR, 
                    self->_encMoveStartL, self->_encMoveStartR);
    }
    
    unsigned long currentMillis = millis();
    if (self->_ledBlinking && self->_ledBlinkInterval > 0) {
      if (currentMillis - lastLedToggle >= self->_ledBlinkInterval) {
        lastLedToggle = currentMillis;
        ledState = !ledState;
        digitalWrite(self->_cfg.ledPin, ledState ? HIGH : LOW);
      }
    }
    
    if (currentMillis - lastBatteryUpdate >= batteryUpdateInterval) {
      lastBatteryUpdate = currentMillis;
      self->getBatteryPower();
    }
    
    if (self->_ctrlInitialized && self->_ctrlSerial->available() > 0) {
      self->ctrlReceiveUART();
    }
    
    if (currentMillis - lastDisplayUpdate >= displayUpdateInterval) {
      lastDisplayUpdate = currentMillis;
      self->updateDisplay();
    }
    
    delay(10);
  }
}

// ============ Async API ============

bool UniBase::isMoving() {
  return (_moveState != STATE_IDLE);
}

void UniBase::moveDistAsync(int power, int millimeters) {
  ensureBegun();
  xSemaphoreTake(_moveSemaphore, 0);

  if (power == 0 || millimeters == 0) {
    xSemaphoreGive(_moveSemaphore);
    return;
  }

  // Глушим предыдущий автомат до перенастройки сервопривода,
  // иначе он может сработать посреди установки новых целей
  _moveState = STATE_IDLE;

  // Сервопривод стартует с нулевой скоростью: разгон, круиз и торможение
  // целиком задает внешний профиль в processAsyncMovement
  startEncMoveOnSecondCore(0, 0);

  portENTER_CRITICAL(&_odomMux);
  setCommandName("moveDist");
  _asyncAbsPower = constrain(abs(power), _tuning.minPower, 100);
  _asyncDirection = (power > 0) ? 1 : -1;
  _targetDistTicks = (abs(millimeters) / (_cfg.wheelDiameterMM * PI)) * _cfg.ticksPerRevRight;

  _targetStartLEnc = _encoders.getLeftTicks();
  _targetStartREnc = _encoders.getRightTicks();

  // Таймаут безопасности: 3x от расчетного времени движения + запас
  float cruise = (_asyncAbsPower / 100.0f) * _cfg.maxTicksPerSec;
  _targetEndTime = millis() + 1500 + (unsigned long)(((float)_targetDistTicks / cruise) * 3000.0f);

  _moveState = STATE_MOVE_DIST;
  portEXIT_CRITICAL(&_odomMux);
}

void UniBase::moveTimeAsync(int power, int milliseconds) {
  ensureBegun();
  xSemaphoreTake(_moveSemaphore, 0);
  _moveState = STATE_IDLE;

  portENTER_CRITICAL(&_odomMux);
  setCommandName("moveTime");
  _targetEndTime = millis() + milliseconds;
  _moveState = STATE_MOVE_TIME;
  portEXIT_CRITICAL(&_odomMux);
  
  startEncMoveOnSecondCore(power, power);
}

// Настройка фазы дуги по дистанции: angleParam - соотношение колес в модели (1 +/- a/90)
void UniBase::startArcDistPhase(int power, float angleParam, float millimeters) {
  _moveState = STATE_IDLE;
  startEncMoveOnSecondCore(0, 0);

  portENTER_CRITICAL(&_odomMux);
  _targetDistTicks = (fabs(millimeters) / (_cfg.wheelDiameterMM * PI)) * _cfg.ticksPerRevRight;
  _targetStartLEnc = _encoders.getLeftTicks();
  _targetStartREnc = _encoders.getRightTicks();
  _targetAngleDeg = angleParam;
  _asyncAbsPower = constrain(abs(power), _tuning.minPower, 100);
  _asyncDirection = (power > 0) ? 1 : -1;

  float cruise = (_asyncAbsPower / 100.0f) * _cfg.maxTicksPerSec;
  _targetEndTime = millis() + 1500 + (unsigned long)(((float)_targetDistTicks / cruise) * 3000.0f);

  _moveState = STATE_MOVE_ARC_DIST;
  portEXIT_CRITICAL(&_odomMux);
}

void UniBase::moveArcDistAsync(int power, int angle, int millimeters) {
  ensureBegun();
  xSemaphoreTake(_moveSemaphore, 0);

  if (power == 0 || millimeters == 0) {
    xSemaphoreGive(_moveSemaphore);
    return;
  }

  setCommandName("moveArcD");
  startArcDistPhase(power, (float)angle, (float)millimeters);
}

void UniBase::moveArcRadiusAsync(int power, float radiusMM, float angleDeg) {
  ensureBegun();
  xSemaphoreTake(_moveSemaphore, 0);

  float halfTrack = _cfg.trackLengthMM / 2.0f;
  if (power == 0 || angleDeg == 0 || radiusMM < halfTrack) {
    // Радиус меньше половины колеи дугой не реализуется - используйте rotate()
    xSemaphoreGive(_moveSemaphore);
    return;
  }

  // Соотношение скоростей колес из геометрии дуги: vL/vR = (R + d/2)/(R - d/2)
  float angleParam = 90.0f * halfTrack / radiusMM;
  if (angleDeg < 0) angleParam = -angleParam;
  float arcMM = fabs(angleDeg) * (PI / 180.0f) * radiusMM; // длина дуги по центру

  setCommandName("moveArcR");
  startArcDistPhase(power, angleParam, arcMM);
}

void UniBase::moveArcTimeAsync(int power, int angle, int milliseconds) {
  ensureBegun();
  xSemaphoreTake(_moveSemaphore, 0);
  _moveState = STATE_IDLE;

  portENTER_CRITICAL(&_odomMux);
  setCommandName("moveArcT");
  float leftPower = power * (1.0 + angle / 90.0);
  float rightPower = power * (1.0 - angle / 90.0);
  _targetEndTime = millis() + milliseconds;
  _moveState = STATE_MOVE_ARC_TIME;
  portEXIT_CRITICAL(&_odomMux);
  
  startEncMoveOnSecondCore(leftPower, rightPower);
}

// Настройка поворотной фазы: цель - абсолютный угол одометрии (текущий + deltaDeg),
// ошибка измеряется и закрывается замкнутым контуром до конца
void UniBase::startRotatePhase(int power, float deltaDeg, MoveState state) {
  _moveState = STATE_IDLE;

  // Запускаем внутренний тиковый сервопривод с нулевой скоростью:
  // внешний контур по углу будет задавать скорость в processAsyncMovement
  startEncMoveOnSecondCore(0, 0);

  portENTER_CRITICAL(&_odomMux);
  _targetTheta = _totalAngle + deltaDeg * PI / 180.0f;

  _asyncAbsPower = constrain(abs(power), _tuning.minPower, 100);
  _rotateCruiseVel = (_asyncAbsPower / 100.0f) * _cfg.maxTicksPerSec;

  _targetStartLEnc = _encoders.getLeftTicks();
  _targetStartREnc = _encoders.getRightTicks();

  // Таймаут безопасности: 3x от расчетного времени поворота + запас
  float arcMM = (fabs(deltaDeg) / 360.0f) * (PI * _cfg.trackLengthMM);
  float arcTicks = (arcMM / (PI * _cfg.wheelDiameterMM)) * _cfg.ticksPerRevLeft;
  _targetEndTime = millis() + 1500 + (unsigned long)((arcTicks / _rotateCruiseVel) * 3000.0f);

  _moveState = state;
  portEXIT_CRITICAL(&_odomMux);
}

void UniBase::rotateAsync(int power, int angle) {
  ensureBegun();
  xSemaphoreTake(_moveSemaphore, 0);

  if (angle == 0 || power == 0) {
    xSemaphoreGive(_moveSemaphore);
    return;
  }

  setCommandName("rotate");
  startRotatePhase(power, (float)angle, STATE_ROTATE_PROFILE);
}

void UniBase::rotateToAsync(int power, float angleDeg) {
  ensureBegun();
  xSemaphoreTake(_moveSemaphore, 0);

  portENTER_CRITICAL(&_odomMux);
  float curDeg = _totalAngle * 180.0f / PI;
  portEXIT_CRITICAL(&_odomMux);

  // Кратчайший доворот к абсолютному курсу
  float delta = angleDeg - curDeg;
  while (delta > 180.0f) delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;

  if (power == 0 || fabs(delta) <= _tuning.rotateTolDeg) {
    xSemaphoreGive(_moveSemaphore);
    return;
  }

  setCommandName("rotateTo");
  startRotatePhase(power, delta, STATE_ROTATE_PROFILE);
}

void UniBase::moveToAsync(int power, float x, float y) {
  ensureBegun();
  xSemaphoreTake(_moveSemaphore, 0);

  portENTER_CRITICAL(&_odomMux);
  float dx = x - _xPos;
  float dy = y - _yPos;
  float curThetaDeg = _theta * 180.0f / PI;
  portEXIT_CRITICAL(&_odomMux);

  float dist = sqrtf(dx * dx + dy * dy);
  if (power == 0 || dist <= _tuning.moveTolMM) {
    xSemaphoreGive(_moveSemaphore);
    return;
  }

  _moveToX = x;
  _moveToY = y;

  // Фаза 1: кратчайший доворот на курс к цели. Фаза 2 (прямая до точки)
  // настраивается автоматом состояний после завершения поворота
  float delta = atan2f(dy, dx) * 180.0f / PI - curThetaDeg;
  while (delta > 180.0f) delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;

  setCommandName("moveTo");
  startRotatePhase(power, delta, STATE_MOVETO_TURN);
}

// Переход от поворотной фазы moveTo к прямой (вызывается автоматом на втором ядре)
void UniBase::beginMoveToDrive() {
  // Гасим остаток вращения перед прямой
  _encMoveActive = false;
  _motors.brakeBoth();
  delay(30);
  _motors.setPower(0, 0);

  portENTER_CRITICAL(&_odomMux);
  float dx = _moveToX - _xPos;
  float dy = _moveToY - _yPos;
  portEXIT_CRITICAL(&_odomMux);
  float dist = sqrtf(dx * dx + dy * dy);

  if (dist <= _tuning.moveTolMM) {
    setCommandName("Ready");
    stop(SOFT);
    return;
  }

  startEncMoveOnSecondCore(0, 0);

  portENTER_CRITICAL(&_odomMux);
  _targetDistTicks = (dist / (_cfg.wheelDiameterMM * PI)) * _cfg.ticksPerRevRight;
  _asyncDirection = 1;
  _targetStartLEnc = _encoders.getLeftTicks();
  _targetStartREnc = _encoders.getRightTicks();
  float cruise = (_asyncAbsPower / 100.0f) * _cfg.maxTicksPerSec;
  _targetEndTime = millis() + 1500 + (unsigned long)(((float)_targetDistTicks / cruise) * 3000.0f);
  _moveState = STATE_MOVE_DIST;
  portEXIT_CRITICAL(&_odomMux);
}

// ============ Blocking API wrappers ============

void UniBase::moveDist(int power, int millimeters) {
  moveDistAsync(power, millimeters);
  xSemaphoreTake(_moveSemaphore, portMAX_DELAY);
}
void UniBase::moveTime(int power, int milliseconds) {
  moveTimeAsync(power, milliseconds);
  xSemaphoreTake(_moveSemaphore, portMAX_DELAY);
}
void UniBase::moveArcDist(int power, int angle, int millimeters) {
  moveArcDistAsync(power, angle, millimeters);
  xSemaphoreTake(_moveSemaphore, portMAX_DELAY);
}
void UniBase::moveArcTime(int power, int angle, int milliseconds) {
  moveArcTimeAsync(power, angle, milliseconds);
  xSemaphoreTake(_moveSemaphore, portMAX_DELAY);
}
void UniBase::rotate(int power, int angle) {
  rotateAsync(power, angle);
  xSemaphoreTake(_moveSemaphore, portMAX_DELAY);
}
void UniBase::rotateTo(int power, float angleDeg) {
  rotateToAsync(power, angleDeg);
  xSemaphoreTake(_moveSemaphore, portMAX_DELAY);
}
void UniBase::moveTo(int power, float x, float y) {
  moveToAsync(power, x, y);
  xSemaphoreTake(_moveSemaphore, portMAX_DELAY);
}
void UniBase::moveArcRadius(int power, float radiusMM, float angleDeg) {
  moveArcRadiusAsync(power, radiusMM, angleDeg);
  xSemaphoreTake(_moveSemaphore, portMAX_DELAY);
}

bool UniBase::waitMove(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (isMoving()) {
    if (timeoutMs > 0 && millis() - start >= timeoutMs) return false;
    delay(5);
  }
  return true;
}

// ============ Motor basics & Stop ============

void UniBase::motors(int powerLeft, int powerRight) {
  ensureBegun();
  _cmdSeq++;
  _moveState = STATE_IDLE; _encMoveActive = false;
  setCommandName("motors");
  _motors.setPower(powerLeft, powerRight);
}
void UniBase::motorLeft(int power) {
  ensureBegun();
  _cmdSeq++;
  _moveState = STATE_IDLE; _encMoveActive = false;
  setCommandName("motorL");
  _motors.setLeftPower(power);
}
void UniBase::motorRight(int power) {
  ensureBegun();
  _cmdSeq++;
  _moveState = STATE_IDLE; _encMoveActive = false;
  setCommandName("motorR");
  _motors.setRightPower(power);
}
void UniBase::motorsArc(int power, float angle) {
  ensureBegun();
  _moveState = STATE_IDLE; _encMoveActive = false;
  setCommandName("motorsArc");
  startEncMoveOnSecondCore(power * (1.0 + angle / 90.0), power * (1.0 - angle / 90.0));
}

void UniBase::motorsSync(int powerLeft, int powerRight) {
  ensureBegun();
  _moveState = STATE_IDLE; _encMoveActive = false;
  setCommandName("motorsSync");
  startEncMoveOnSecondCore(constrain(powerLeft, -100, 100), constrain(powerRight, -100, 100));
}

void UniBase::holdPosition() {
  ensureBegun();
  _moveState = STATE_IDLE;
  setCommandName("hold");
  // Сервопривод с нулевой скоростью активно удерживает текущую позицию
  startEncMoveOnSecondCore(0, 0);
}

void UniBase::stop(int stopType) {
  ensureBegun();
  uint32_t seq = ++_cmdSeq;
  setCommandName("stop");
  _moveState = STATE_IDLE; _encMoveActive = false;
  if (stopType == HARD) {
    _motors.brakeBoth();
    delay(50);
  }
  // Если во время торможения пришла новая команда движения, не гасим ее:
  // хвост этого stop() больше не имеет права трогать моторы и семафор
  if (seq != _cmdSeq) return;
  _motors.setPower(0, 0);
  _lastLeftPower = 0; _lastRightPower = 0;
  // Семафор отдаем последним: блокирующие команды должны проснуться только
  // когда остановка полностью завершена, иначе следующая команда гонится со stop()
  xSemaphoreGive(_moveSemaphore);
}
void UniBase::stopLeft(int stopType) {
  ensureBegun();
  uint32_t seq = ++_cmdSeq;
  setCommandName("stopL");
  _moveState = STATE_IDLE; _encMoveActive = false;
  if (stopType == HARD) { _motors.brakeLeft(); delay(30); }
  if (seq != _cmdSeq) return; // новая команда пришла во время торможения
  _motors.setLeftPower(0); _lastLeftPower = 0;
}
void UniBase::stopRight(int stopType) {
  ensureBegun();
  uint32_t seq = ++_cmdSeq;
  setCommandName("stopR");
  _moveState = STATE_IDLE; _encMoveActive = false;
  if (stopType == HARD) { _motors.brakeRight(); delay(30); }
  if (seq != _cmdSeq) return; // новая команда пришла во время торможения
  _motors.setRightPower(0); _lastRightPower = 0;
}

// ============ Odometry ============

void UniBase::resetDistance() { 
    ensureBegun();
    portENTER_CRITICAL(&_odomMux);
    _totalDistance = 0.0; 
    portEXIT_CRITICAL(&_odomMux);
}
float UniBase::getDistance() { 
    ensureBegun();
    portENTER_CRITICAL(&_odomMux);
    float v = _totalDistance; 
    portEXIT_CRITICAL(&_odomMux);
    return v;
}
void UniBase::resetAngle() { 
    ensureBegun();
    portENTER_CRITICAL(&_odomMux);
    _totalAngle = 0.0; 
    portEXIT_CRITICAL(&_odomMux);
}
float UniBase::getAngle() { 
    ensureBegun();
    portENTER_CRITICAL(&_odomMux);
    float v = _totalAngle * 180 / PI; 
    portEXIT_CRITICAL(&_odomMux);
    return v;
}
void UniBase::setPosition(float x, float y, float angleDeg) {
    ensureBegun();
    float a = angleDeg * PI / 180.0f;
    portENTER_CRITICAL(&_odomMux);
    _xPos = x;
    _yPos = y;
    _totalAngle = a;
    _theta = a;
    while (_theta > PI) _theta -= 2 * PI;
    while (_theta < -PI) _theta += 2 * PI;
    portEXIT_CRITICAL(&_odomMux);
}

OdometryData UniBase::getOdometry() {
    ensureBegun();
    OdometryData data;
    portENTER_CRITICAL(&_odomMux);
    data.x = _xPos;
    data.y = _yPos;
    data.angle = _theta * 180 / PI;
    portEXIT_CRITICAL(&_odomMux);
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
    float v = _theta * 180 / PI; 
    portEXIT_CRITICAL(&_odomMux);
    return v;
}
long UniBase::getLeftTicks() { ensureBegun(); return _encoders.getLeftTicks(); }
long UniBase::getRightTicks() { ensureBegun(); return _encoders.getRightTicks(); }

// ============ Display ============

void UniBase::displayPrint(const char* text) {
    ensureBegun();
    portENTER_CRITICAL(&_dispMux);
    strncpy(_customDisplayText, text, sizeof(_customDisplayText) - 1);
    _customDisplayText[sizeof(_customDisplayText) - 1] = '\0';
    _customDisplayName[0] = '\0';
    _customDisplayMode = true;
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
    portEXIT_CRITICAL(&_dispMux);
}
void UniBase::displayPrint(const char* name, int value) { char buf[16]; snprintf(buf, sizeof(buf), "%d", value); displayPrint(name, buf); }
void UniBase::displayPrint(const char* name, long value) { char buf[16]; snprintf(buf, sizeof(buf), "%ld", value); displayPrint(name, buf); }
void UniBase::displayPrint(const char* name, float value) { char buf[16]; snprintf(buf, sizeof(buf), "%.2f", value); displayPrint(name, buf); }
void UniBase::displayPrint(const char* name, double value) { char buf[16]; snprintf(buf, sizeof(buf), "%.4f", value); displayPrint(name, buf); }
void UniBase::displayPrint(const char* name, bool value) { displayPrint(name, value ? "true" : "false"); }

void UniBase::displayClear() {
    ensureBegun();
    portENTER_CRITICAL(&_dispMux);
    _customDisplayMode = false;
    _customDisplayText[0] = '\0';
    _customDisplayName[0] = '\0';
    portEXIT_CRITICAL(&_dispMux);
    updateDisplay();
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

int UniBase::getBatteryPower() {
  ensureBegun();
  float batteryVoltage = (analogReadMilliVolts(_cfg.batteryPin) / 1000.0) * 2.0;
  if (batteryVoltage < 2.5 || batteryVoltage > 4.5) { _batteryValid = false; _batteryPercent = -1; return -1; }
  
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
  _batteryPercent = constrain((int)(percent + 0.5), 0, 100);
  _batteryValid = true;
  return _batteryPercent;
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
