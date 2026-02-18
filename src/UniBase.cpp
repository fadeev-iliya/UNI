#include "UniBase.h"
#include "UniDev.h"

UniBase* UniBase::_instance = nullptr;

UniBase::UniBase(String robotName)
  : _display(nullptr), _displayInitialized(false), _customDisplayMode(false),
    _ledBlinking(false), _ledBlinkInterval(0),
    _batteryPercent(-1), _batteryValid(false),
    _lEnc(0), _rEnc(0), _lEncOld(0), _rEncOld(0), _encPrevTime(0), _encPrevErr(0), _I(0),
    _xPos(0.0), _yPos(0.0), _theta(0.0), _totalDistance(0.0), _totalAngle(0.0),
    _targetTheta(0.0), _useTargetTheta(true),
    _currentCommand("Idle"), _robotName(robotName),
    _encMoveActive(false), _encMovePowerL(0), _encMovePowerR(0),
    _encMoveStartL(0), _encMoveStartR(0),
    _lastLeftPower(0), _lastRightPower(0),
    _distPerTickLeft((PI * WHEEL_DIAMETER_MM) / TICKS_PER_REV_LEFT),
    _distPerTickRight((PI * WHEEL_DIAMETER_MM) / TICKS_PER_REV_RIGHT),
    _ctrlSerial(nullptr), _ctrlInitialized(false),
    _ctrlBufferIndex(0),
    _cmdQueue(nullptr), _cmdTaskHandle(nullptr),
    _abortCommand(false)
{
  _instance = this;
  _display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
}

UniBase::~UniBase() {
  if (_display) delete _display;
}

// ============ Initialization ============

void UniBase::begin() {
  Serial.begin(115200);
  
  #ifdef DEBUG_MODE
  Serial.println("\n\nUNI Platform - UniBase & UniDev");
  Serial.print("Robot name: ");
  Serial.println(_robotName);
  #endif

  initMotors();
  initEncoders();
  initOLED();
  _uartLastRxTime = millis();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  pinMode(BATTERY_PIN, INPUT);
  analogReadResolution(12);
  
  initSecondCore();
  getBatteryPower();
  
  UniDev::staticInit();
  
  #ifdef DEBUG_MODE
  Serial.println("All initializations done properly");
  Serial.println("Robot ready");
  #endif
  
  _currentCommand = "Ready";
  updateDisplay();
}

void UniBase::begin(String robotName) {
  _robotName = robotName;
  begin();
}

void UniBase::initMotors() {
  #ifdef DEBUG_MODE
  Serial.println("Motors initialization");
  #endif
  
  pinMode(LEFT_MOTOR_PIN_A, OUTPUT);
  pinMode(LEFT_MOTOR_PIN_B, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_A, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_B, OUTPUT);
  
  analogWrite(LEFT_MOTOR_PIN_A, 0);
  analogWrite(LEFT_MOTOR_PIN_B, 0);
  analogWrite(RIGHT_MOTOR_PIN_A, 0);
  analogWrite(RIGHT_MOTOR_PIN_B, 0);
  
}

void UniBase::initEncoders() {
  #ifdef DEBUG_MODE
  Serial.println("Encoders initialization");
  #endif
  
  pinMode(LEFT_ENC_INTERRUPT, INPUT);
  pinMode(LEFT_ENC_DIRECTION, INPUT);
  pinMode(RIGHT_ENC_INTERRUPT, INPUT);
  pinMode(RIGHT_ENC_DIRECTION, INPUT);
  
  attachInterrupt(LEFT_ENC_INTERRUPT, leftEncISR, RISING);
  attachInterrupt(RIGHT_ENC_INTERRUPT, rightEncISR, RISING);
}

void UniBase::initOLED() {
  Wire.begin(OLED_SDA, OLED_SCL);
  
  if(!_display->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
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
  #ifdef DEBUG_MODE
  Serial.println("Second core initialization");
  #endif
  
  xTaskCreatePinnedToCore(
    secondCoreLoop,
    "Odometry",
    10000,
    NULL,
    1,
    &_secondCore,
    0);
  delay(10);
}

// ============ Internal methods ============

void UniBase::leftMotor(int power) {
  power = constrain(power, -100, 100);
  
  if (power > 0) {
    analogWrite(LEFT_MOTOR_PIN_A, map(abs(power), 0, 100, 0, 255));
    analogWrite(LEFT_MOTOR_PIN_B, 0);
  }
  else if (power < 0) {
    analogWrite(LEFT_MOTOR_PIN_A, 0);
    analogWrite(LEFT_MOTOR_PIN_B, map(abs(power), 0, 100, 0, 255));
  }
  else {
    digitalWrite(LEFT_MOTOR_PIN_A, 0);
    digitalWrite(LEFT_MOTOR_PIN_B, 0);
  }
}

void UniBase::rightMotor(int power) {
  power = constrain(power, -100, 100);
  
  if (power < 0) {
    analogWrite(RIGHT_MOTOR_PIN_A, map(abs(power), 0, 100, 0, 255));
    analogWrite(RIGHT_MOTOR_PIN_B, 0);
  }
  else if (power > 0) {
    analogWrite(RIGHT_MOTOR_PIN_A, 0);
    analogWrite(RIGHT_MOTOR_PIN_B, map(abs(power), 0, 100, 0, 255));
  }
  else {
    digitalWrite(RIGHT_MOTOR_PIN_A, 0);
    digitalWrite(RIGHT_MOTOR_PIN_B, 0);
  }
}

void UniBase::updateOdometry() {
  noInterrupts();
  long lEncI = _lEnc;
  long rEncI = _rEnc;
  interrupts();
  
  float SL = (lEncI - _lEncOld) * _distPerTickLeft;
  float SR = (rEncI - _rEncOld) * _distPerTickRight;
  
  _lEncOld = lEncI;
  _rEncOld = rEncI;
  
  float deltaDistance = (abs(SL) + abs(SR)) / 2;
  _totalDistance += deltaDistance;
  
  float deltaAngle = (SL - SR) / TRACK_LENGTH_MM;
  _totalAngle += deltaAngle;
  
  _xPos += ((SR + SL) / 2) * cos(_theta + ((SL - SR) / (2 * TRACK_LENGTH_MM)));
  _yPos += ((SR + SL) / 2) * sin(_theta + ((SL - SR) / (2 * TRACK_LENGTH_MM)));
  _theta += deltaAngle;
  
  if (_theta > PI)
    _theta -= 2 * PI;
  else if (_theta < -PI)
    _theta += 2 * PI;
}

void UniBase::updateDisplay() {
  if (!_displayInitialized) {
    return;
  }
  
  _display->clearDisplay();
  
  if (_customDisplayMode) {
    // Show name at top if set, otherwise no title
    if (_customDisplayName.length() > 0) {
      _display->setTextSize(2);
      String displayName = _customDisplayName;
      if (displayName.length() > 10) displayName = displayName.substring(0, 10);
      int16_t x1, y1;
      uint16_t w, h;
      _display->getTextBounds(displayName.c_str(), 0, 0, &x1, &y1, &w, &h);
      _display->setCursor((SCREEN_WIDTH - w) / 2, 0);
      _display->print(displayName);
      
      // Value centered below
      _display->setTextSize(2);
      String displayText = _customDisplayText;
      if (displayText.length() > 10) displayText = displayText.substring(0, 10);
      _display->getTextBounds(displayText.c_str(), 0, 0, &x1, &y1, &w, &h);
      _display->setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT + 10 - h) / 2);
      _display->print(displayText);
    } else {
      // No name — center value on screen
      _display->setTextSize(2);
      String displayText = _customDisplayText;
      if (displayText.length() > 10) displayText = displayText.substring(0, 10);
      int16_t x1, y1;
      uint16_t w, h;
      _display->getTextBounds(displayText.c_str(), 0, 0, &x1, &y1, &w, &h);
      _display->setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 2);
      _display->print(displayText);
    }
  } else {
    _display->setTextSize(2);
    
    String displayName = _robotName;
    if (displayName.length() > 10) {
      displayName = displayName.substring(0, 10);
    }
    
    int16_t x1, y1;
    uint16_t w, h;
    _display->getTextBounds(displayName.c_str(), 0, 0, &x1, &y1, &w, &h);
    _display->setCursor((SCREEN_WIDTH - w) / 2, 0);
    _display->print(displayName);

    _display->setTextSize(1);
    
    _display->setCursor(0, 18);
    _display->print("X: ");
    _display->print(_xPos, 0);
    _display->print(" mm");
    
    _display->setCursor(70, 18);
    _display->print("Y: ");
    _display->print(_yPos, 0);
    _display->print(" mm");
    
    _display->setCursor(0, 30);
    _display->print("Angle: ");
    _display->print(_theta * 180 / PI, 0);
    _display->print((char)247);
    
    
    _display->setCursor(0, 42);
    _display->print("L: ");
    _display->print(_lEnc);
    
    _display->setCursor(70, 42);
    _display->print("R: ");
    _display->print(_rEnc);
    
    _display->setCursor(0, 56);
    _display->print(_currentCommand);
    
    if (_batteryValid && _batteryPercent >= 0) {
      drawBatteryIcon(104, 54, _batteryPercent);
      
      // Show percentage number next to icon
      _display->setTextSize(1);
      String batText = String(_batteryPercent) + "%";
      int16_t bx1, by1;
      uint16_t bw, bh;
      _display->getTextBounds(batText.c_str(), 0, 0, &bx1, &by1, &bw, &bh);
      _display->setCursor(102 - bw, 56);
      _display->print(batText);
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

void UniBase::encMove(int pL, int pR) {
  if (pL != 0 && pR != 0) {
    float ratio = (float)pR / (float)pL;
    float err = (_lEnc * ratio - _rEnc);
    
    leftMotor(pL + err * (pL / abs(pL)) * (pR / abs(pR)));
    rightMotor(pR - err * (pL / abs(pL)) * (pR / abs(pR)));
  }
  else {
    leftMotor(pL);
    rightMotor(pR);
  }
}

void UniBase::encMove(int pL, int pR, long startLEnc, long startREnc) {
  long funcLEnc = _lEnc - startLEnc;
  long funcREnc = _rEnc - startREnc;
  
  if (pL != 0 && pR != 0) {
    float ratio = (float)pR / (float)pL;
    float err = (funcLEnc * ratio - funcREnc);
    
    unsigned long nowTime = micros();
    double dt = (nowTime - _encPrevTime) / 1000000.0;
    if (dt == 0) {
      dt = 0.0001;
    }
    
    float P = 0.5 * err;
    _I = _I + 0.005 * err * dt;
    float D = (0.05 * (err - _encPrevErr)) / dt;
    
    float U = P + _I + D;
    
    _encPrevErr = err;
    _encPrevTime = nowTime;
    
    int targetLeftPower = pL - U * (pL / abs(pL)) * (pR / abs(pR));
    int targetRightPower = pR + U;
    
    const int maxPowerChange = 10;
    
    int leftPowerChange = targetLeftPower - _lastLeftPower;
    int rightPowerChange = targetRightPower - _lastRightPower;
    
    leftPowerChange = constrain(leftPowerChange, -maxPowerChange, maxPowerChange);
    rightPowerChange = constrain(rightPowerChange, -maxPowerChange, maxPowerChange);
    
    _lastLeftPower += leftPowerChange;
    _lastRightPower += rightPowerChange;
    
    leftMotor(_lastLeftPower);
    rightMotor(_lastRightPower);
  }
  else {
    _lastLeftPower = pL;
    _lastRightPower = pR;
    leftMotor(pL);
    rightMotor(pR);
  }
}

void UniBase::startEncMoveOnSecondCore(int powerL, int powerR) {
  _encMovePowerL = powerL;
  _encMovePowerR = powerR;
  _encMoveStartL = _lEnc;
  _encMoveStartR = _rEnc;
  _encPrevTime = micros();
  _I = 0;
  _encPrevErr = 0;
  _lastLeftPower = 0;
  _lastRightPower = 0;
  _encMoveActive = true;
}

// ============ Interrupt handlers ============
void IRAM_ATTR UniBase::leftEncISR() {
  if (_instance) {
    if (!digitalRead(LEFT_ENC_DIRECTION))
      _instance->_lEnc++;
    else
      _instance->_lEnc--;
  }
}

void IRAM_ATTR UniBase::rightEncISR() {
  if (_instance) {
    if (!digitalRead(RIGHT_ENC_DIRECTION))
      _instance->_rEnc--;
    else
      _instance->_rEnc++;
  }
}

void UniBase::secondCoreLoop(void* pvParameters) {
  #ifdef DEBUG_MODE
  Serial.print("Odometry running on core ");
  Serial.println(xPortGetCoreID());
  #endif
  delay(10);
  
  unsigned long lastDisplayUpdate = 0;
  const unsigned long displayUpdateInterval = 100;
  
  unsigned long lastLedToggle = 0;
  unsigned long lastBatteryUpdate = 0;
  const unsigned long batteryUpdateInterval = 2000;
  bool ledState = false;
  
  for (;;) {
    if (_instance) {
      _instance->updateOdometry();
      
      if (_instance->_encMoveActive) {
        _instance->encMove(_instance->_encMovePowerL, _instance->_encMovePowerR, 
                          _instance->_encMoveStartL, _instance->_encMoveStartR);
      }
      
      unsigned long currentMillis = millis();
      
      if (_instance->_ledBlinking && _instance->_ledBlinkInterval > 0) {
        if (currentMillis - lastLedToggle >= _instance->_ledBlinkInterval) {
          lastLedToggle = currentMillis;
          ledState = !ledState;
          digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        }
      }
      
      if (currentMillis - lastBatteryUpdate >= batteryUpdateInterval) {
        lastBatteryUpdate = currentMillis;
        _instance->getBatteryPower();
      }
      
      // UART control: read and dispatch commands on Core 1
      if (_instance->_ctrlInitialized && _instance->_ctrlSerial->available() > 0) {
        _instance->ctrlReceiveUART();
      }
      
      if (currentMillis - lastDisplayUpdate >= displayUpdateInterval) {
        lastDisplayUpdate = currentMillis;
        _instance->updateDisplay();
      }
    }
    delay(10);
  }
}

// ============ Public API ============

void UniBase::motors(int powerLeft, int powerRight) {
  _encMoveActive = false;
  _useTargetTheta = false;
  _currentCommand = "motors";
  leftMotor(powerLeft);
  rightMotor(powerRight);
}

void UniBase::motorLeft(int power) {
  _encMoveActive = false;
  _useTargetTheta = false;
  _currentCommand = "motorL";
  leftMotor(power);
}

void UniBase::motorRight(int power) {
  _encMoveActive = false;
  _useTargetTheta = false;
  _currentCommand = "motorR";
  rightMotor(power);
}

void UniBase::motorsArc(int power, float angle) {
  _encMoveActive = false;
  _currentCommand = "motorsArc";
  if (angle != 0) {
    _useTargetTheta = false;
  }
  int leftPower = power * (1.0 + angle / 90.0);
  int rightPower = power * (1.0 - angle / 90.0);
  
  startEncMoveOnSecondCore(leftPower, rightPower);
}

void UniBase::moveDist(int power, int millimeters) {
  _currentCommand = "moveDist";
  power = constrain(power, -100, 100);

  long distTicks = (abs(millimeters) / (WHEEL_DIAMETER_MM * PI)) * TICKS_PER_REV_RIGHT;

  #ifdef DEBUG_MODE
  Serial.print("Ticks: ");
  Serial.println(distTicks);
  #endif
  
  long startLEnc = _lEnc;
  long startREnc = _rEnc;
  
  // Deceleration parameters
  const float decelDistMM = 35.0;  // Start slowing down 50mm before destination
  long decelTicks = (decelDistMM / (WHEEL_DIAMETER_MM * PI)) * TICKS_PER_REV_RIGHT;
  const int minPower = 15;         // Minimum driving power (overcomes friction)
  int absPower = abs(power);
  int direction = (power > 0) ? 1 : -1;
  
  // Compensate for distance traveled during final stop(HARD) braking
  // Overshoot peaks at mid-power (max inertia + weaker EM braking), zero at full power
  float stopCompMM = 2.0 * ((float)absPower * (100.0 - (float)absPower)) / 2500.0;
  long stopCompTicks = (stopCompMM / (WHEEL_DIAMETER_MM * PI)) * TICKS_PER_REV_RIGHT;
  distTicks = max(0L, distTicks - stopCompTicks);
  
  // If total distance is shorter than 2× decel zone, shrink decel zone
  if (distTicks < decelTicks * 2) {
    decelTicks = distTicks / 2;
  }
  
  // If requested power is already at or below minimum, no deceleration needed
  if (absPower <= minPower) {
    decelTicks = 0;
  }
  
  // Speed measurement variables
  long prevAvgTicks = 0;
  unsigned long prevSpeedTime = micros();
  float measuredTickSpeed = 0.0;   // Current measured speed (ticks/sec)
  float peakTickSpeed = 0.0;       // Peak speed at full power (ticks/sec)
  
  startEncMoveOnSecondCore(power, power);
  
  while (true) {
    if (_abortCommand) { _abortCommand = false; stop(HARD); return; }
    long currentDistance = (abs(_lEnc - startLEnc) + abs(_rEnc - startREnc)) / 2;
    if (currentDistance >= distTicks) break;
    
    long remainingTicks = distTicks - currentDistance;
    
    // === Measure tick speed (both in cruise and decel phases) ===
    unsigned long nowMicros = micros();
    unsigned long dtMicros = nowMicros - prevSpeedTime;
    
    if (dtMicros >= 2000) {  // Sample every 2ms
      long dTicks = currentDistance - prevAvgTicks;
      measuredTickSpeed = (float)abs(dTicks) / ((float)dtMicros / 1000000.0);  // ticks/sec
      
      if (measuredTickSpeed > peakTickSpeed) {
        peakTickSpeed = measuredTickSpeed;
      }
      
      prevAvgTicks = currentDistance;
      prevSpeedTime = nowMicros;
    }
    
    if (remainingTicks <= decelTicks && decelTicks > 0 && peakTickSpeed > 0) {
      // === Gradual deceleration zone with active braking ===
      
      // Target speed ramps linearly from peakTickSpeed down to a creep speed
      float ratio = (float)remainingTicks / (float)decelTicks;  // 1.0 → 0.0
      float creepSpeed = peakTickSpeed * ((float)minPower / (float)absPower);
      float targetSpeed = creepSpeed + (peakTickSpeed - creepSpeed) * ratio;
      
      // Braking tolerance: lenient at start of decel, tight near destination
      float brakeTolerance = 1.0 + 0.10 * ratio;  // 1.10 at entry → 1.0 at destination
      
      if (measuredTickSpeed > targetSpeed * brakeTolerance) {
        // TOO FAST — actively brake by shorting motor pins (electromagnetic braking)
        _encMoveActive = false;
        analogWrite(LEFT_MOTOR_PIN_A, 255);
        analogWrite(LEFT_MOTOR_PIN_B, 255);
        analogWrite(RIGHT_MOTOR_PIN_A, 255);
        analogWrite(RIGHT_MOTOR_PIN_B, 255);
      } else {
        // Speed is at or below target — apply proportional driving power
        int targetPower = minPower + (int)((absPower - minPower) * ratio);
        targetPower = constrain(targetPower, minPower, absPower);
        int newPower = targetPower * direction;
        
        if (!_encMoveActive) {
          // Re-engage PID motor control after braking
          startEncMoveOnSecondCore(newPower, newPower);
        } else {
          _encMovePowerL = newPower;
          _encMovePowerR = newPower;
        }
      }

      #ifdef DEBUG_MODE
      static unsigned long lastDebugPrint = 0;
      if (millis() - lastDebugPrint > 50) {
        Serial.print("Decel: remain=");
        Serial.print(remainingTicks);
        Serial.print(" speed=");
        Serial.print(measuredTickSpeed, 0);
        Serial.print(" target=");
        Serial.print(targetSpeed, 0);
        Serial.print(" braking=");
        Serial.println(measuredTickSpeed > targetSpeed * 1.05 ? "YES" : "no");
        lastDebugPrint = millis();
      }
      #endif
    }
    
    delay(1);
  }
  
  stop(HARD);
  _currentCommand = "Ready";
}

void UniBase::moveTime(int power, int milliseconds) {
  _currentCommand = "moveTime";
  startEncMoveOnSecondCore(power, power);
  unsigned long endTime = millis() + milliseconds;
  while (millis() < endTime) {
    if (_abortCommand) { _abortCommand = false; stop(HARD); return; }
    delay(1);
  }
  stop(HARD);
  _currentCommand = "Ready";
}

void UniBase::moveArcDist(int power, int angle, int millimeters) {
  _currentCommand = "moveArcD";
  if (angle != 0) {
    _useTargetTheta = false;
  }
  int leftPower = power * (1.0 + angle / 90.0);
  int rightPower = power * (1.0 - angle / 90.0);
  
  long distTicks = (abs(millimeters) / (WHEEL_DIAMETER_MM * PI)) * TICKS_PER_REV_RIGHT;
  long startLEnc = _lEnc;
  long startREnc = _rEnc;
  
  startEncMoveOnSecondCore(leftPower, rightPower);
  
  while ((abs(_lEnc - startLEnc) + abs(_rEnc - startREnc)) / 2 < distTicks) {
    if (_abortCommand) { _abortCommand = false; stop(HARD); return; }
    delay(1);
  }
  
  stop(HARD);
  _currentCommand = "Ready";
}

void UniBase::moveArcTime(int power, int angle, int milliseconds) {
  _currentCommand = "moveArcT";
  if (angle != 0) {
    _useTargetTheta = false;
  }
  int leftPower = power * (1.0 + angle / 90.0);
  int rightPower = power * (1.0 - angle / 90.0);
  
  startEncMoveOnSecondCore(leftPower, rightPower);
  
  unsigned long endTime = millis() + milliseconds;
  while (millis() < endTime) {
    if (_abortCommand) { _abortCommand = false; stop(HARD); return; }
    delay(1);
  }
  
  stop(HARD);
  _currentCommand = "Ready";
}

void UniBase::rotate(int power, int angle) {
  _currentCommand = "rotate";
  
  if (!_useTargetTheta) {
    _targetTheta = _theta;
    _useTargetTheta = true;
  }
  
  _targetTheta += angle * PI / 180.0;
  
  while (_targetTheta > PI) _targetTheta -= 2 * PI;
  while (_targetTheta < -PI) _targetTheta += 2 * PI;
  
  float targetAngleDeg = _targetTheta * 180 / PI;
  
  int absPower = abs(power);
  int leftPow, rightPow;
  if (angle < 0) {
    leftPow = -absPower;
    rightPow = absPower;
  } else {
    leftPow = absPower;
    rightPow = -absPower;
  }
  
  // ============================================================
  // PHASE 1: Fast rotation with tick-based deceleration
  // Intentionally stops a few degrees early to guarantee no overshoot
  // ============================================================
  
  const float decelAngleDeg = 30.0;
  const int minPower = 15;
  const float earlyStopDeg = 3.0;  // Stop 3° early, Phase 2 will correct
  
  long startLEnc = _lEnc;
  long startREnc = _rEnc;
  long prevAvgTicks = 0;
  unsigned long prevSpeedTime = micros();
  float measuredTickSpeed = 0.0;
  float peakTickSpeed = 0.0;
  
  startEncMoveOnSecondCore(leftPow, rightPow);
  
  while (true) {
    if (_abortCommand) { _abortCommand = false; stop(HARD); return; }
    float currentAngle = _theta * 180 / PI;
    float angleDiff = targetAngleDeg - currentAngle;
    
    while (angleDiff > 180) angleDiff -= 360;
    while (angleDiff < -180) angleDiff += 360;
    
    float remainingAngle = abs(angleDiff);
    
    // Check if we've passed the target or are close enough for Phase 2
    if ((angle > 0 && angleDiff < 0) || (angle < 0 && angleDiff > 0)) {
      break;
    }
    if (remainingAngle < earlyStopDeg) {
      break;
    }
    
    // Measure tick speed (real-time via encoder interrupts)
    long currentTicks = (abs(_lEnc - startLEnc) + abs(_rEnc - startREnc)) / 2;
    unsigned long nowMicros = micros();
    unsigned long dtMicros = nowMicros - prevSpeedTime;
    
    if (dtMicros >= 2000) {
      long dTicks = currentTicks - prevAvgTicks;
      measuredTickSpeed = (float)abs(dTicks) / ((float)dtMicros / 1000000.0);
      
      if (measuredTickSpeed > peakTickSpeed) {
        peakTickSpeed = measuredTickSpeed;
      }
      
      prevAvgTicks = currentTicks;
      prevSpeedTime = nowMicros;
    }
    
    if (remainingAngle <= decelAngleDeg && peakTickSpeed > 0) {
      float ratio = remainingAngle / decelAngleDeg;
      float creepSpeed = peakTickSpeed * ((float)minPower / (float)absPower);
      float targetTickSpeed = creepSpeed + (peakTickSpeed - creepSpeed) * ratio;
      float brakeTolerance = 1.0 + 0.10 * ratio;
      
      if (measuredTickSpeed > targetTickSpeed * brakeTolerance) {
        _encMoveActive = false;
        analogWrite(LEFT_MOTOR_PIN_A, 255);
        analogWrite(LEFT_MOTOR_PIN_B, 255);
        analogWrite(RIGHT_MOTOR_PIN_A, 255);
        analogWrite(RIGHT_MOTOR_PIN_B, 255);
      } else {
        int targetPower = minPower + (int)((absPower - minPower) * ratio);
        targetPower = constrain(targetPower, minPower, absPower);
        
        int fL, fR;
        if (angle < 0) { fL = -targetPower; fR = targetPower; }
        else { fL = targetPower; fR = -targetPower; }
        
        if (!_encMoveActive) {
          startEncMoveOnSecondCore(fL, fR);
        } else {
          _encMovePowerL = fL;
          _encMovePowerR = fR;
        }
      }
    }
    
    delay(1);
  }
  
  stop(HARD);
  delay(30);  // Let robot settle and _theta update
  
  // ============================================================
  // PHASE 2: Precision correction loop
  // Use _theta feedback to nudge the robot to the exact target angle
  // ============================================================
  
  const int corrPower = 12;          // Very low power for fine corrections
  const float angleTolerance = 0.5;  // Acceptable error in degrees
  const int maxAttempts = 5;         // Max correction attempts
  
  for (int attempt = 0; attempt < maxAttempts; attempt++) {
    // Measure current error
    float currentAngle = _theta * 180 / PI;
    float angleDiff = targetAngleDeg - currentAngle;
    
    while (angleDiff > 180) angleDiff -= 360;
    while (angleDiff < -180) angleDiff += 360;
    
    #ifdef DEBUG_MODE
    Serial.print("Correction #");
    Serial.print(attempt);
    Serial.print(": error=");
    Serial.print(angleDiff, 2);
    Serial.println(" deg");
    #endif
    
    // If within tolerance, we're done
    if (abs(angleDiff) < angleTolerance) {
      break;
    }
    
    // Determine correction direction
    int cL, cR;
    int activeCorrPower = corrPower;  // Start at base correction power
    if (angleDiff > 0) {
      cL = activeCorrPower;
      cR = -activeCorrPower;
    } else {
      cL = -activeCorrPower;
      cR = activeCorrPower;
    }
    
    // Drive at correction power until we reach target or pass it
    startEncMoveOnSecondCore(cL, cR);
    
    float corrStartDiff = angleDiff;
    long stallCheckTicks = (abs(_lEnc) + abs(_rEnc));
    unsigned long stallCheckTime = millis();
    
    while (true) {
      if (_abortCommand) { _abortCommand = false; stop(HARD); return; }
      float curAng = _theta * 180 / PI;
      float diff = targetAngleDeg - curAng;
      
      while (diff > 180) diff -= 360;
      while (diff < -180) diff += 360;
      
      // Stop if within tight tolerance or if we've passed the target
      if (abs(diff) < angleTolerance * 0.5) {
        break;
      }
      if ((corrStartDiff > 0 && diff <= 0) || (corrStartDiff < 0 && diff >= 0)) {
        break;  // Crossed target
      }
      
      // Stall detection: if ticks haven't changed for 50ms, increase power
      long currentStallTicks = (abs(_lEnc) + abs(_rEnc));
      if (currentStallTicks != stallCheckTicks) {
        // Movement detected, reset stall check
        stallCheckTicks = currentStallTicks;
        stallCheckTime = millis();
      } else if (millis() - stallCheckTime > 50) {
        // Stalled — increase power
        activeCorrPower = min(activeCorrPower + 5, 40);
        if (diff > 0) {
          cL = activeCorrPower;
          cR = -activeCorrPower;
        } else {
          cL = -activeCorrPower;
          cR = activeCorrPower;
        }
        startEncMoveOnSecondCore(cL, cR);
        stallCheckTime = millis();
        
        #ifdef DEBUG_MODE
        Serial.print("Stall detected, power up to ");
        Serial.println(activeCorrPower);
        #endif
      }
      
      delay(1);
    }
    
    stop(HARD);
    delay(30);  // Settle
  }
  _currentCommand = "Ready";
}

void UniBase::stop(int stopType) {
  _currentCommand = "stop";
  _encMoveActive = false;
  _useTargetTheta = false;
  
  if (stopType == HARD) {
    // Фаза 1: Резкое торможение
    const byte stopPower = 255;
    for (int i = 0; i < 50; i++) {
      analogWrite(LEFT_MOTOR_PIN_A, stopPower);
      analogWrite(LEFT_MOTOR_PIN_B, stopPower);
      analogWrite(RIGHT_MOTOR_PIN_A, stopPower);
      analogWrite(RIGHT_MOTOR_PIN_B, stopPower);
      delay(1);
    }
  }
  
  leftMotor(0);
  rightMotor(0);
  
  _lastLeftPower = 0;
  _lastRightPower = 0;
}

void UniBase::stopLeft(int stopType) {
  _currentCommand = "stopL";
  _encMoveActive = false;
  _useTargetTheta = false;
  
  if (stopType == HARD) {
    const byte stopPower = 255;
    
    // Фаза 1: Резкое торможение
    for (int i = 0; i < 30; i++) {
      analogWrite(LEFT_MOTOR_PIN_A, stopPower);
      analogWrite(LEFT_MOTOR_PIN_B, stopPower);
      delay(1);
    }
  }
  
  leftMotor(0);
  
  _lastLeftPower = 0;
}

void UniBase::stopRight(int stopType) {
  _currentCommand = "stopR";
  _encMoveActive = false;
  _useTargetTheta = false;
  
  if (stopType == HARD) {
    const byte stopPower = 255;
    
    // Фаза 1: Резкое торможение
    for (int i = 0; i < 30; i++) {
      analogWrite(RIGHT_MOTOR_PIN_A, stopPower);
      analogWrite(RIGHT_MOTOR_PIN_B, stopPower);
      delay(1);
    }
  }
  
  rightMotor(0);
  
  _lastRightPower = 0;
}

// ============ Odometry ============

void UniBase::resetDistance() {
  noInterrupts();
  _totalDistance = 0.0;
  interrupts();
}

float UniBase::getDistance() {
  float dist;
  noInterrupts();
  dist = _totalDistance;
  interrupts();
  return dist;
}

void UniBase::resetAngle() {
  noInterrupts();
  _totalAngle = 0.0;
  interrupts();
}

float UniBase::getAngle() {
  float angle;
  noInterrupts();
  angle = _totalAngle * 180 / PI;
  interrupts();
  return angle;
}

OdometryData UniBase::getOdometry() {
  OdometryData data;
  noInterrupts();
  data.x = _xPos;
  data.y = _yPos;
  data.angle = _theta * 180 / PI;
  interrupts();
  return data;
}

float UniBase::getAbsX() {
  return _xPos;
}

float UniBase::getAbsY() {
  return _yPos;
}

float UniBase::getAbsAngle() {
  return _theta * 180 / PI;
}

long UniBase::getLeftTicks() {
  return _lEnc;
}

long UniBase::getRightTicks() {
  return _rEnc;
}

// ============ Display ============

void UniBase::displayPrint(String text) {
  _customDisplayMode = true;
  _customDisplayText = text;
  _customDisplayName = "";
}

void UniBase::displayPrint(const char* text) {
  displayPrint(String(text));
}

void UniBase::displayPrint(int value) {
  displayPrint(String(value));
}

void UniBase::displayPrint(long value) {
  displayPrint(String(value));
}

void UniBase::displayPrint(float value) {
  displayPrint(String(value, 2));
}

void UniBase::displayPrint(double value) {
  displayPrint(String(value, 4));
}

void UniBase::displayPrint(bool value) {
  displayPrint(value ? "true" : "false");
}

// Named displayPrint overloads
void UniBase::displayPrint(const char* name, String value) {
  _customDisplayMode = true;
  _customDisplayName = String(name);
  _customDisplayText = value;
}

void UniBase::displayPrint(const char* name, const char* value) {
  displayPrint(name, String(value));
}

void UniBase::displayPrint(const char* name, int value) {
  displayPrint(name, String(value));
}

void UniBase::displayPrint(const char* name, long value) {
  displayPrint(name, String(value));
}

void UniBase::displayPrint(const char* name, float value) {
  displayPrint(name, String(value, 2));
}

void UniBase::displayPrint(const char* name, double value) {
  displayPrint(name, String(value, 4));
}

void UniBase::displayPrint(const char* name, bool value) {
  displayPrint(name, value ? "true" : "false");
}

void UniBase::displayClear() {
  _customDisplayMode = false;
  _customDisplayText = "";
  _customDisplayName = "";
  updateDisplay();
}

// ============ Utility ============

void UniBase::printOdometry() {
  Serial.print("encL: ");
  Serial.print(_lEnc);
  Serial.print(" \tencR: ");
  Serial.print(_rEnc);
  Serial.print(" \tX: ");
  Serial.print(_xPos);
  Serial.print(" \tY: ");
  Serial.print(_yPos);
  Serial.print(" \tTheta: ");
  Serial.println(_theta * 180 / PI);
}

void UniBase::blinkLED(int interval) {
  if (interval > 0) {
    _ledBlinkInterval = interval;
    _ledBlinking = true;
  } else {
    _ledBlinking = false;
    _ledBlinkInterval = 0;
    digitalWrite(LED_PIN, LOW);
  }
}

int UniBase::getBatteryPower() {
  // Use analogReadMilliVolts for accurate voltage (uses ESP32 internal calibration)
  float measuredVoltage = analogReadMilliVolts(BATTERY_PIN) / 1000.0;
  float batteryVoltage = measuredVoltage * 2.0;
  
  #ifdef DEBUG_MODE
    Serial.print("Battery voltage: ");
    Serial.println(batteryVoltage);
  #endif
  
  if (batteryVoltage < 2.5 || batteryVoltage > 4.5) {
    _batteryValid = false;
    _batteryPercent = -1;
    return -1;
  }
  
  // LiPo discharge lookup table (voltage → percentage)
  // Based on typical 1S LiPo discharge curve, 3.20V = 0%, 4.20V = 100%
  static const float lipoTable[][2] = {
    {3.20,   0}, {3.30,   2}, {3.40,   5}, {3.50,   8},
    {3.55,  10}, {3.60,  13}, {3.65,  17}, {3.70,  22},
    {3.73,  27}, {3.75,  30}, {3.77,  34}, {3.79,  38},
    {3.80,  40}, {3.82,  45}, {3.84,  50}, {3.86,  55},
    {3.88,  60}, {3.92,  65}, {3.96,  72}, {4.00,  78},
    {4.05,  84}, {4.10,  90}, {4.15,  95}, {4.20, 100}
  };
  static const int tableSize = sizeof(lipoTable) / sizeof(lipoTable[0]);
  
  float percent = 0;
  float v = batteryVoltage;
  
  if (v >= 4.20) {
    percent = 100.0;
  } else if (v <= 3.20) {
    percent = 0.0;
  } else {
    // Linear interpolation between table points
    for (int i = 1; i < tableSize; i++) {
      if (v <= lipoTable[i][0]) {
        float v0 = lipoTable[i-1][0], p0 = lipoTable[i-1][1];
        float v1 = lipoTable[i][0],   p1 = lipoTable[i][1];
        percent = p0 + (p1 - p0) * (v - v0) / (v1 - v0);
        break;
      }
    }
  }
  
  int percentInt = (int)(percent + 0.5);
  percentInt = constrain(percentInt, 0, 100);
  
  _batteryPercent = percentInt;
  _batteryValid = true;
  
  return percentInt;
}

// ============ UART Control Mode ============

void UniBase::UniBaseControl() {
  if (_ctrlInitialized) return;  // Already initialized
  
  _ctrlSerial = new HardwareSerial(2);
  _ctrlSerial->begin(CTRL_UART_BAUD_RATE, SERIAL_8N1, CTRL_UART_RX_PIN, CTRL_UART_TX_PIN);
  _ctrlBufferIndex = 0;
  _abortCommand = false;
  
  // Create command queue (depth 1 — latest command wins)
  _cmdQueue = xQueueCreate(1, sizeof(UartCmd));
  
  // Create command executor task on Core 0
  xTaskCreatePinnedToCore(
    uartCommandTaskFunc,  // task function
    "uartCmd",            // name
    4096,                 // stack size
    this,                 // parameter
    1,                    // priority
    &_cmdTaskHandle,      // handle
    0                     // Core 0
  );
  
  _ctrlInitialized = true;
}

// --- UART Receiver (non-blocking, byte-by-byte) ---
void UniBase::ctrlReceiveUART() {
  while (_ctrlSerial->available() > 0) {
    uint8_t b = _ctrlSerial->read();
    
    // Timeout: if partial packet stalled > 100ms, reset buffer
    if (_ctrlBufferIndex > 0 && (millis() - _uartLastRxTime > 100)) {
      _ctrlBufferIndex = 0;
    }
    _uartLastRxTime = millis();
    
    if (_ctrlBufferIndex == 0) {
      // Expect a valid command byte (0x01 to 0x1C+)
      if (b >= 0x01 && b <= 0x20) {
        _ctrlBuffer[_ctrlBufferIndex++] = b;
      }
      continue;
    }
    
    if (_ctrlBufferIndex < CTRL_UART_BUFFER_SIZE - 1) {
      _ctrlBuffer[_ctrlBufferIndex++] = b;
    } else {
      _ctrlBufferIndex = 0;
      continue;
    }
    
    // Determine expected length
    uint8_t cmd = _ctrlBuffer[0];
    uint8_t expectedLength = 0;
    
    switch (cmd) {
      case 0x01: expectedLength = 3; break;  // motors
      case 0x02: expectedLength = 2; break;  // stop
      case 0x03: expectedLength = 4; break;  // moveDist
      case 0x04: expectedLength = 4; break;  // rotate
      case 0x05: expectedLength = 1; break;  // getDistance
      case 0x06: expectedLength = 1; break;  // getAngle
      case 0x07: expectedLength = 1; break;  // getOdometry
      case 0x08: expectedLength = 4; break;  // motorsArc
      case 0x09: expectedLength = 4; break;  // moveTime
      case 0x0A: expectedLength = 6; break;  // moveArcTime
      case 0x0B: expectedLength = 2; break;  // motorLeft
      case 0x0C: expectedLength = 2; break;  // motorRight
      case 0x0D: expectedLength = 2; break;  // stopLeft
      case 0x0E: expectedLength = 2; break;  // stopRight
      case 0x0F: expectedLength = 1; break;  // resetDistance
      case 0x10: expectedLength = 1; break;  // resetAngle
      case 0x11: expectedLength = 1; break;  // getAbsX
      case 0x12: expectedLength = 1; break;  // getAbsY
      case 0x13: expectedLength = 1; break;  // getAbsAngle
      case 0x14: expectedLength = 1; break;  // getLeftTicks
      case 0x15: expectedLength = 1; break;  // getRightTicks
      case 0x16: {  // displayPrint (variable length: cmd + len + chars)
        if (_ctrlBufferIndex >= 2) {
          expectedLength = 2 + _ctrlBuffer[1];  // cmd + len byte + N chars
        } else {
          continue;  // wait for length byte
        }
        break;
      }
      case 0x17: expectedLength = 1; break;  // displayClear
      case 0x18: expectedLength = 1; break;  // printOdometry
      case 0x19: expectedLength = 3; break;  // blinkLED
      case 0x1A: expectedLength = 1; break;  // getBatteryPower
      case 0x1B: expectedLength = 6; break;  // moveArcDist
      case 0x1C: { // displayPrintNamed (cmd + nLen + name + vLen + val)
        // Need CMD + NameLen
        if (_ctrlBufferIndex < 2) {
          continue; 
        }
        uint8_t nameLen = _ctrlBuffer[1];
        
        // Need CMD + NameLen + Name + ValLen
        if (_ctrlBufferIndex < 2 + nameLen + 1) {
          continue;
        }
        uint8_t valLen = _ctrlBuffer[2 + nameLen];
        expectedLength = 1 + 1 + nameLen + 1 + valLen;
        break;
      }
      default:
        _ctrlBufferIndex = 0;
        continue;
    }
    
    if (_ctrlBufferIndex >= expectedLength) {
      uartDispatchCommand(_ctrlBuffer, expectedLength);
      _ctrlBufferIndex = 0;
    }
  }
}

// --- Command Dispatcher ---
// Called from Core 1. Immediate commands execute here.
// Blocking commands are queued to the executor task on Core 0.
void UniBase::uartDispatchCommand(uint8_t* data, uint8_t len) {
  uint8_t cmd = data[0];
  
  // ---- DATA commands (getters) — run anytime, no interruption ----
  switch (cmd) {
    case 0x05: { // getDistance
      ctrlSendFloat(getDistance());
      return;
    }
    case 0x06: { // getAngle
      ctrlSendFloat(getAngle());
      return;
    }
    case 0x07: { // getOdometry (3 floats: x, y, angle)
      OdometryData od = getOdometry();
      ctrlSendFloat(od.x);
      ctrlSendFloat(od.y);
      ctrlSendFloat(od.angle);
      return;
    }
    case 0x0F: { // resetDistance
      resetDistance();
      return;
    }
    case 0x10: { // resetAngle
      resetAngle();
      return;
    }
    case 0x11: { // getAbsX
      ctrlSendFloat(getAbsX());
      return;
    }
    case 0x12: { // getAbsY
      ctrlSendFloat(getAbsY());
      return;
    }
    case 0x13: { // getAbsAngle
      ctrlSendFloat(getAbsAngle());
      return;
    }
    case 0x14: { // getLeftTicks
      ctrlSendLong(getLeftTicks());
      return;
    }
    case 0x15: { // getRightTicks
      ctrlSendLong(getRightTicks());
      return;
    }
    case 0x18: { // printOdometry
      printOdometry();
      return;
    }
    case 0x1A: { // getBatteryPower
      int batt = getBatteryPower();
      uint8_t val = (batt < 0) ? 0xFF : (uint8_t)batt;
      _ctrlSerial->write(val);
      return;
    }
  }
  
  // ---- DIRECT commands — execute immediately, abort any blocking command ----
  switch (cmd) {
    case 0x01: { // motors(powerLeft, powerRight)
      _abortCommand = true;
      int8_t pL = (int8_t)(data[1] - 128);
      int8_t pR = (int8_t)(data[2] - 128);
      motors(pL, pR);
      return;
    }
    case 0x02: { // stop(stopType)
      _abortCommand = true;
      uint8_t stopType = data[1];
      stop(stopType);
      return;
    }
    case 0x08: { // motorsArc(power, angle)
      _abortCommand = true;
      int8_t power = (int8_t)(data[1] - 128);
      int16_t angle = (int16_t)(((int8_t)data[2] << 8) | data[3]);
      motorsArc(power, (float)angle);
      return;
    }
    case 0x0B: { // motorLeft(power)
      _abortCommand = true;
      int8_t power = (int8_t)(data[1] - 128);
      motorLeft(power);
      return;
    }
    case 0x0C: { // motorRight(power)
      _abortCommand = true;
      int8_t power = (int8_t)(data[1] - 128);
      motorRight(power);
      return;
    }
    case 0x0D: _abortCommand = true; stopLeft(data[1]); return;
    case 0x0E: _abortCommand = true; stopRight(data[1]); return;
    case 0x16: { // displayPrint(string)
      uint8_t strLen = data[1];
      String text = "";
      for (uint8_t i = 0; i < strLen; i++) text += (char)data[2 + i];
      displayPrint(text);
      return;
    }
    case 0x17: displayClear(); return;
    case 0x1C: { // displayPrint(name, value)
      uint8_t nLen = data[1];
      String name = "";
      for (uint8_t i = 0; i < nLen; i++) name += (char)data[2 + i];
      
      uint8_t vLen = data[2 + nLen];
      String value = "";
      for (uint8_t i = 0; i < vLen; i++) value += (char)data[3 + nLen + i];
      
      displayPrint(name.c_str(), value);
      return;
    }
    case 0x19: {
      uint16_t interval = ((uint16_t)data[1] << 8) | data[2];
      blinkLED((int)interval);
      return;
    }
  }
  
  // ---- BLOCKING commands — abort current, queue to executor task ----
  UartCmd cmd_struct;
  cmd_struct.type = cmd;
  cmd_struct.p1 = 0; cmd_struct.p2 = 0; cmd_struct.p3 = 0;
  
  // Abort any running blocking command
  _abortCommand = true;
  
  switch (cmd) {
    case 0x03: { // moveDist(power, millimeters)
      cmd_struct.p1 = (int8_t)(data[1] - 128);
      cmd_struct.p2 = (int)((uint16_t)data[2] << 8 | data[3]);
      break;
    }
    case 0x04: { // rotate(power, angle)
      cmd_struct.p1 = (int8_t)(data[1] - 128);
      cmd_struct.p2 = (int16_t)(((int8_t)data[2] << 8) | data[3]);
      break;
    }
    case 0x09: { // moveTime(power, milliseconds)
      cmd_struct.p1 = (int8_t)(data[1] - 128);
      cmd_struct.p2 = (int)((uint16_t)data[2] << 8 | data[3]);
      break;
    }
    case 0x0A: { // moveArcTime(power, arcAngle, milliseconds)
      cmd_struct.p1 = (int8_t)(data[1] - 128);
      cmd_struct.p2 = (int16_t)(((int8_t)data[2] << 8) | data[3]);
      cmd_struct.p3 = (int)((uint16_t)data[4] << 8 | data[5]);
      break;
    }
    case 0x1B: { // moveArcDist(power, arcAngle, millimeters)
      cmd_struct.p1 = (int8_t)(data[1] - 128);
      cmd_struct.p2 = (int16_t)(((int8_t)data[2] << 8) | data[3]);
      cmd_struct.p3 = (int)((uint16_t)data[4] << 8 | data[5]);
      break;
    }
    default: return;  // Unknown command
  }
  
  // Overwrite queue (if a command is waiting, replace it)
  xQueueOverwrite(_cmdQueue, &cmd_struct);
}

// --- Command Executor Task (runs on Core 0) ---
// Waits for blocking commands and calls native functions directly.
void UniBase::uartCommandTaskFunc(void* param) {
  UniBase* self = (UniBase*)param;
  UartCmd cmd;
  
  for (;;) {
    // Block until a command arrives
    if (xQueueReceive(self->_cmdQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      self->_abortCommand = false;
      
      switch (cmd.type) {
        case 0x03: self->moveDist(cmd.p1, cmd.p2); break;
        case 0x04: self->rotate(cmd.p1, cmd.p2); break;
        case 0x09: self->moveTime(cmd.p1, cmd.p2); break;
        case 0x0A: self->moveArcTime(cmd.p1, cmd.p2, cmd.p3); break;
        case 0x1B: self->moveArcDist(cmd.p1, cmd.p2, cmd.p3); break;
      }
      
      self->_currentCommand = "Ready";
    }
  }
}



    






      


      


    








  



// --- Send float response over UART ---
void UniBase::ctrlSendFloat(float val) {
  uint8_t* bytes = (uint8_t*)&val;
  _ctrlSerial->write(bytes, 4);
}

// --- Send long response over UART ---
void UniBase::ctrlSendLong(long val) {
  uint8_t* bytes = (uint8_t*)&val;
  _ctrlSerial->write(bytes, 4);
}
