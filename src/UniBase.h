#ifndef UNIBASE_H
#define UNIBASE_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#include "UniMotors.h"
#include "UniEncoders.h"
#include "UniProtocol.h"

// Debug mode - раскомментируйте для вывода отладочных сообщений
// #define DEBUG_MODE

// ============ Конфигурация платформы ============

struct UniConfig {
    // Пины моторов
    uint8_t leftMotorA = 2;
    uint8_t leftMotorB = 4;
    uint8_t rightMotorA = 27;
    uint8_t rightMotorB = 26;

    // Пины энкодеров
    uint8_t leftEncInt = 18;
    uint8_t leftEncDir = 19;
    uint8_t rightEncInt = 35;
    uint8_t rightEncDir = 34;

    // Системные пины
    uint8_t ledPin = 25;
    uint8_t batteryPin = 33;
    uint8_t sdaPin = 21;
    uint8_t sclPin = 22;
    uint8_t oledSda = 21;
    uint8_t oledScl = 22;
    int8_t oledReset = -1;
    uint8_t screenWidth = 128;
    uint8_t screenHeight = 64;
    uint8_t screenAddress = 0x3C;

    // Кинематика
    float wheelDiameterMM = 45.0f;
    float trackLengthMM = 106.0f;
    float ticksPerRevLeft = 690.0f;
    float ticksPerRevRight = 690.0f;
    float maxTicksPerSec = 1500.0f;

    // UART Управление (Связь с Nano)
    long uartBaudRate = 38400;
    uint8_t uartRxPin = 16;
    uint8_t uartTxPin = 17;
    uint16_t uartBufferSize = 128;
};

// ============ Конфигурация физики и ПИД ============

struct TuningConfig {
    int minPower = 15;            // Минимальная мощность страгивания
    float pGain = 0.5;            // Коэффициент P для прямолинейного движения
    float iGain = 0.005;          // Коэффициент I
    float dGain = 0.0;            // Коэффициент D

    // Точный поворот (замкнутый контур по одометрии)
    float rotateTolDeg = 1.0f;      // Допуск завершения поворота (град)
    float rotateAccel = 1300.0f;    // Замедление профиля поворота (тики/с^2)
    float rotateMinSpeed = 40.0f;   // Минимальная скорость подхода к цели (тики/с)

    // Прямолинейное движение (замкнутый профиль по энкодерам)
    float moveTolMM = 1.0f;         // Допуск завершения движения (мм)
    float moveAccel = 1800.0f;      // Замедление профиля движения (тики/с^2)
    float moveMinSpeed = 60.0f;     // Минимальная скорость подхода к цели (тики/с)
};

// ============ Внутренние константы ============

enum StopType {
  SOFT = 0,
  HARD = 1
};

struct OdometryData {
  float x;
  float y;
  float angle;
};

class UniBase {
private:
  UniConfig _cfg;
  TuningConfig _tuning;

  // OLED Display
  Adafruit_SSD1306* _display;
  bool _displayInitialized;
  bool _customDisplayMode;
  char _customDisplayName[16];
  char _customDisplayText[16];
  
  // Peripherals state
  bool _ledBlinking;
  int _ledBlinkInterval;
  int _batteryPercent;
  bool _batteryValid;
  
  // RTOS Handles and Synchronization
  TaskHandle_t _secondCore;
  portMUX_TYPE _odomMux = portMUX_INITIALIZER_UNLOCKED;
  portMUX_TYPE _dispMux = portMUX_INITIALIZER_UNLOCKED; // защита строк дисплея (пишутся с обоих ядер)
  SemaphoreHandle_t _moveSemaphore;
  
  // Submodules
  UniMotors _motors;
  UniEncoders _encoders;
  
  // Odometry variables (Protected by _odomMux)
  float _xPos;
  float _yPos;
  float _theta;
  float _totalDistance;
  float _totalAngle;
  float _distPerTickLeft;
  float _distPerTickRight;
  long _lEncOdomOld = 0;
  long _rEncOdomOld = 0;
  
  // State Machine for Async movements
  enum MoveState {
    STATE_IDLE,
    STATE_MOVE_DIST,
    STATE_MOVE_TIME,
    STATE_MOVE_ARC_DIST,
    STATE_MOVE_ARC_TIME,
    STATE_ROTATE_PROFILE,
    STATE_MOVETO_TURN
  };
  volatile MoveState _moveState;
  
  // Async movement targets and parameters
  volatile long _targetDistTicks;
  volatile long _targetStartLEnc;
  volatile long _targetStartREnc;
  volatile unsigned long _targetEndTime;
  volatile float _targetAngleDeg;
  volatile float _targetTheta;
  volatile float _rotateCruiseVel;
  volatile float _asyncAbsPower;
  volatile int _asyncDirection;
  volatile float _moveToX;
  volatile float _moveToY;
  volatile uint32_t _cmdSeq = 0; // счетчик команд: защита от хвоста устаревшего stop()

  // Control variables
  char _currentCommand[16];
  char _robotName[16];
  
  // PID variables for straight line (EncMove)
  volatile bool _encMoveActive;
  volatile float _encMovePowerL;
  volatile float _encMovePowerR;
  volatile long _encMoveStartL;
  volatile long _encMoveStartR;
  volatile long _encPrevTime;
  volatile float _targetPosL;
  volatile float _targetPosR;
  volatile float _currentTargetVelL;
  volatile float _currentTargetVelR;
  volatile float _encPrevErrL;
  volatile float _encPrevErrR;
  volatile float _IL;
  volatile float _IR;
  volatile float _lastLeftPower;
  volatile float _lastRightPower;
  
  // UART control variables
  HardwareSerial* _ctrlSerial;
  bool _ctrlInitialized;
  uint8_t* _ctrlBuffer;
  uint8_t* _decodeBuffer;
  uint16_t _ctrlBufferIndex;
  unsigned long _uartLastRxTime;
  
  // Lazy init
  bool _begun;
  void ensureBegun();

  // Internal methods
  void initOLED();
  void initSecondCore();
  
  void updateOdometry();
  void updateDisplay();
  void drawBatteryIcon(int x, int y, int percent);
  void setCommandName(const char* name);

  void encMove(float pL, float pR, long startLEnc, long startREnc);
  
  void startEncMoveOnSecondCore(float powerL, float powerR);

  // Общие настройщики фаз движения
  void startRotatePhase(int power, float deltaDeg, MoveState state);
  void startArcDistPhase(int power, float angleParam, float millimeters);
  void beginMoveToDrive();

  // State machine logic
  void processAsyncMovement();

  // RTOS Task
  static void secondCoreLoop(void* pvParameters);

  // UART control private methods
  void ctrlReceiveUART();
  void uartDispatchCommand(uint8_t* data, uint8_t len);
  void ctrlSendFloat(float val);
  void ctrlSendLong(long val);
  void sendPacket(const uint8_t* payload, uint8_t len);

public:
  /**
   * @brief Конструктор платформы
   * @param robotName Имя робота (будет выведено на экран), макс 15 символов
   * @param config Структура с конфигурацией железа (опционально)
   */
  UniBase(const char* robotName = "UNI Robot", UniConfig config = UniConfig());
  ~UniBase();

  /**
   * @brief Инициализация периферии (опционально). Если не вызвать,
   * выполнится автоматически при первой команде.
   * @param robotName Имя робота (опционально, переопределяет имя из конструктора)
   */
  void begin(const char* robotName = nullptr);
  
  // Конфигурация ПИД и физики на лету
  void setTuning(const TuningConfig& tuning) { _tuning = tuning; }
  TuningConfig getTuning() const { return _tuning; }

  // ---- Control logic ----
  /**
   * @brief Инициализировать прослушивание UART с Arduino Nano
   */
  void UniBaseControl();
  
  // ---- Motor Basics ----
  void motors(int powerLeft, int powerRight);
  void motorLeft(int power);
  void motorRight(int power);
  void motorsArc(int power, float angle);

  /**
   * @brief Езда с раздельными скоростями колес и выравниванием по энкодерам.
   * Работает асинхронно до вызова stop(): соотношение скоростей колес
   * удерживается сервоприводом, в отличие от motors() без стабилизации.
   * @param powerLeft Скорость левого колеса (-100..100)
   * @param powerRight Скорость правого колеса (-100..100)
   */
  void motorsSync(int powerLeft, int powerRight);

  // ---- Blocking Movement ----
  void moveDist(int power, int millimeters);
  void moveTime(int power, int milliseconds);
  void moveArcDist(int power, int angle, int millimeters);
  void moveArcTime(int power, int angle, int milliseconds);
  void rotate(int power, int angle);

  /**
   * @brief Поворот к абсолютному курсу одометрии кратчайшим путем.
   * В отличие от rotate() съедает накопленную ошибку предыдущих маневров.
   * @param angleDeg Целевой курс в градусах (в системе getAngle()/setPosition())
   */
  void rotateTo(int power, float angleDeg);

  /**
   * @brief Поехать в точку одометрии: доворот на курс к цели, затем прямая.
   * @param x, y Целевая точка (мм, в системе getOdometry()/setPosition())
   */
  void moveTo(int power, float x, float y);

  /**
   * @brief Дуга с заданной геометрией: радиус и угол.
   * @param radiusMM Радиус дуги по центру робота (не меньше половины колеи)
   * @param angleDeg Угол дуги; знак задает сторону поворота (как у rotate)
   */
  void moveArcRadius(int power, float radiusMM, float angleDeg);

  // ---- Async Movement ----
  void moveDistAsync(int power, int millimeters);
  void moveTimeAsync(int power, int milliseconds);
  void moveArcDistAsync(int power, int angle, int millimeters);
  void moveArcTimeAsync(int power, int angle, int milliseconds);
  void rotateAsync(int power, int angle);
  void rotateToAsync(int power, float angleDeg);
  void moveToAsync(int power, float x, float y);
  void moveArcRadiusAsync(int power, float radiusMM, float angleDeg);

  /**
   * @brief Возвращает true, если робот сейчас выполняет команду движения
   */
  bool isMoving();

  /**
   * @brief Ожидание завершения асинхронной команды движения
   * @param timeoutMs Максимальное время ожидания в мс (0 = ждать бесконечно)
   * @return true - движение завершено, false - вышли по таймауту
   */
  bool waitMove(unsigned long timeoutMs = 0);

  /**
   * @brief Активное удержание текущей позиции сервоприводом
   * (робот сопротивляется сдвигу). Отменяется stop() или любым движением.
   */
  void holdPosition();
  
  // ---- Stop ----
  void stop(int stopType = HARD);
  void stopLeft(int stopType = HARD);
  void stopRight(int stopType = HARD);

  // ---- Odometry ----
  void resetDistance();
  float getDistance();
  
  void resetAngle();
  float getAngle();
  
  OdometryData getOdometry();

  /**
   * @brief Установить позу одометрии (например, стартовую клетку поля)
   */
  void setPosition(float x, float y, float angleDeg);

  float getAbsX();
  float getAbsY();
  float getAbsAngle();
  
  long getLeftTicks();
  long getRightTicks();

  void printOdometry();

  // ---- Peripherals ----
  void blinkLED(int interval);
  int getBatteryPower();

  // ---- Display ----
  void displayPrint(const char* text);
  void displayPrint(int value);
  void displayPrint(long value);
  void displayPrint(float value);
  void displayPrint(double value);
  void displayPrint(bool value);

  void displayPrint(const char* name, const char* value);
  void displayPrint(const char* name, int value);
  void displayPrint(const char* name, long value);
  void displayPrint(const char* name, float value);
  void displayPrint(const char* name, double value);
  void displayPrint(const char* name, bool value);

  void displayClear();
};

#endif
