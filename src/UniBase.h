#ifndef UNIBASE_H
#define UNIBASE_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#include "UniBoard.h"
#include "UniMotors.h"
#include "UniEncoders.h"
#include "UniProtocol.h"

// Debug mode - раскомментируйте для вывода отладочных сообщений
// #define DEBUG_MODE

// ============ Конфигурация платформы ============
// Внутренняя структура библиотеки: пины, геометрия и калибровка зашиты в нее

namespace uni_detail {

struct UniConfig {
    // Версия платы: UNI_BOARD_AUTO - определить по наличию EEPROM на I2C
    UniBoardVersion board = UNI_BOARD_AUTO;

    // Пины ниже зависят от версии платы: UNI_PIN_AUTO - взять из ее профиля
    // (см. UniBoard.cpp), любое другое значение переопределяет профиль

    // Пины моторов
    uint8_t leftMotorA = UNI_PIN_AUTO;
    uint8_t leftMotorB = UNI_PIN_AUTO;
    uint8_t rightMotorA = UNI_PIN_AUTO;
    uint8_t rightMotorB = UNI_PIN_AUTO;

    // Режим ШИМ моторов: UNI_DECAY_AUTO - по драйверам платы
    UniMotorDecay motorDecay = UNI_DECAY_AUTO;

    // Пины энкодеров
    uint8_t leftEncInt = UNI_PIN_AUTO;
    uint8_t leftEncDir = UNI_PIN_AUTO;
    uint8_t rightEncInt = UNI_PIN_AUTO;
    uint8_t rightEncDir = UNI_PIN_AUTO;

    // Счет энкодера: true - по всем фронтам обоих каналов (вчетверо выше
    // разрешение одометрии), false - прежний, по одному фронту. На тики
    // (ticksPerRev, getLeftTicks(), maxTicksPerSec) режим не влияет
    bool encoderQuad = true;

    // Системные пины
    uint8_t ledPin = UNI_PIN_AUTO;
    uint8_t batteryPin = UNI_PIN_AUTO;
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
    float trackLengthMM = 106.6f;
    float ticksPerRevLeft = 690.0f;
    float ticksPerRevRight = 690.0f;
    float maxTicksPerSec = UNI_TUNING_AUTO; // скорость при 100% мощности: из калибровки платы

    // Напряжение батареи, при котором измерена maxTicksPerSec (В): по нему мощность
    // пересчитывается под текущий заряд. UNI_TUNING_AUTO - из калибровки платы,
    // 0 - не учитывать заряд
    float batteryRefVoltage = UNI_TUNING_AUTO;

    // Цепочка целей: следующая команда отсчитывает цель от цели предыдущей,
    // а не от места фактической остановки - остатки не накапливаются
    bool chainTargets = true;

    // UART Управление (Связь с Nano)
    long uartBaudRate = 38400;
    uint8_t uartRxPin = UNI_PIN_AUTO;
    uint8_t uartTxPin = UNI_PIN_AUTO;
    uint16_t uartBufferSize = 128;
};

} // namespace uni_detail

// Конфигурация физики и ПИД (TuningConfig) объявлена в UniBoard.h:
// ее значения по умолчанию зависят от версии платы

// ============ Внутренние константы ============

enum StopType {
  SOFT = 0,
  HARD = 1
};

namespace uni_detail {

// Чем закончилась последняя команда движения
enum UniMoveResult : uint8_t {
  UNI_MOVE_OK = 0,          // цель достигнута
  UNI_MOVE_TIMEOUT = 1,     // истек таймаут безопасности
  UNI_MOVE_STALLED = 2,     // робот уперся: колеса перестали двигаться
  UNI_MOVE_INTERRUPTED = 3  // прервана другой командой или stop()
};

} // namespace uni_detail

struct OdometryData {
  float x;
  float y;
  float angle;
};

namespace uni_detail {

// Как прошел последний маневр (для настройки и диагностики)
struct UniMoveReport {
  UniMoveResult result;   // чем закончился
  float errorMM;          // остаток по ходу после остановки (мм)
  float errorDeg;         // остаток по курсу после остановки (град)
  float brakeSpeed;       // скорость в момент включения тормоза (мм/с)
  float coastMM;          // выбег под тормозом (мм)
  uint16_t brakeLeadMs;   // модель выбега в этом маневре: наклон (мс)...
  float brakeDropMM;      // ...и вычитаемое (мм): выбег = brakeLeadMs / 1000 * v - brakeDropMM
  uint32_t timeMs;        // длительность маневра (мс)
};

} // namespace uni_detail

class UniBase {
private:
  // Внутренние типы библиотеки (см. uni_detail)
  typedef uni_detail::UniConfig UniConfig;
  typedef uni_detail::TuningConfig TuningConfig;
  typedef uni_detail::UniMoveResult UniMoveResult;
  typedef uni_detail::UniMoveReport UniMoveReport;

  UniConfig _cfg;
  TuningConfig _tuning;

  // OLED Display
  Adafruit_SSD1306* _display;
  bool _displayInitialized;
  bool _customDisplayMode;
  volatile bool _displayDirty; // экран нужно перерисовать вне очереди
  char _customDisplayName[16];
  char _customDisplayText[16];
  bool _displayPageMode;       // на экране многострочный текст displayText()
  char _displayPage[8 * 22];   // до 8 строк по 21 символу

  // Peripherals state
  bool _ledBlinking;
  int _ledBlinkInterval;
  int _batteryPercent;
  bool _batteryValid;
  volatile float _batteryVoltage; // сглаженное напряжение батареи (В), 0 - нет данных
  volatile float _ffComp;         // поправка мощности на заряд батареи

  // RTOS Handles and Synchronization
  TaskHandle_t _controlTaskHandle; // контур управления: одометрия и моторы, строгий период
  TaskHandle_t _serviceTaskHandle; // дисплей, батарея, светодиод, UART
  QueueHandle_t _cmdQueue;         // команды движения: применяет их только контур управления
  portMUX_TYPE _odomMux = portMUX_INITIALIZER_UNLOCKED;
  portMUX_TYPE _dispMux = portMUX_INITIALIZER_UNLOCKED; // защита строк дисплея (пишутся с обоих ядер)
  portMUX_TYPE _cmdMux = portMUX_INITIALIZER_UNLOCKED;  // номера команд, результат, настройки
  SemaphoreHandle_t _moveSemaphore;
  SemaphoreHandle_t _submitMutex;  // команды встают в очередь строго по порядку номеров

  // Submodules
  UniMotors _motors;
  UniEncoders _encoders;

  // ---- Геометрия (постоянна после конструктора) ----
  float _mmPerTickL, _mmPerTickR;   // мм на тик колеса
  float _mmPerCountL, _mmPerCountR; // мм на отсчет счетчика энкодера
  float _mmPerTick;                 // среднее: перевод настроек из тиков в мм
  float _halfTrack;
  float _vmaxL, _vmaxR, _vmaxAvg;   // скорость колес при 100% мощности (мм/с)

  // ---- Одометрия (защищена _odomMux) ----
  float _xPos;
  float _yPos;
  float _heading;          // непрерывный курс (рад), без приведения к +-180
  double _headingRaw;      // курс по счетчикам энкодеров, без смещения
  double _headingOffset;   // смещение, заданное setPosition()
  float _angleZero;        // нулевая точка getAngle()/rotateTo()
  float _totalDistance;
  float _distAcc;          // дрожание на месте в пройденный путь не попадает
  volatile bool _odomReset; // поза задана заново: остатки прошлых маневров недействительны

  // ---- Контур управления: все поля ниже меняет только задача управления ----

  enum CmdType : uint8_t {
    MC_RAW, MC_VELOCITY, MC_TIMED, MC_HOLD, MC_STOP,
    MC_DIST, MC_ROTATE, MC_ROTATE_TO, MC_ARC, MC_MOVE_TO
  };
  struct MotionCmd {
    uint32_t seq;
    CmdType type;
    uint8_t mask;     // MC_RAW и MC_STOP: 1 - левое колесо, 2 - правое
    uint8_t stopType;
    float power;      // модуль скорости, % от максимума
    float a, b, c;
  };

  enum CtrlState : uint8_t { CS_IDLE, CS_VELOCITY, CS_PATH, CS_HOLD, CS_BRAKE };
  enum AfterBrake : uint8_t { AB_RELEASE, AB_VERIFY, AB_FINISH };
  enum NextPhase : uint8_t { NEXT_NONE, NEXT_MOVETO_DRIVE };

  // Маневр с известной геометрией. Путь задан долей хода каждого колеса на
  // единицу прогресса: прямая (1, 1), поворот на месте (1, -1), дуга (1+k, 1-k)
  struct Path {
    float cL = 1, cR = 1;
    float S = 0;             // длина пути в единицах прогресса (мм)
    float oL = 0, oR = 0;    // стартовое смещение колес: подхват остатка курса
    long startL = 0, startR = 0;
    float sRef = 0;          // прогресс задания
    float vRef = 0;          // скорость задания (мм/с)
    float vCruise = 0, vMin = 0, accel = 0;
    bool spin = false;       // ведущая ось - поворот
    bool straight = false;   // прямая: остаток по ходу передается следующей прямой
    bool steer = false;      // прямая moveTo: подруливание на точку по одометрии
    bool frozen = false;     // курс на точку больше не пересчитывается
    float goalX = 0, goalY = 0;
    float headingRef = 0;    // курс на точку (рад, непрерывный)
    float headingTarget = 0; // курс, которым маневр должен закончиться
    NextPhase next = NEXT_NONE;
    uint32_t deadlineMs = 0;
    float stallMarkS = 0;
    uint32_t stallMarkMs = 0;  // когда робот последний раз продвинулся по пути
    uint32_t crossMarkMs = 0;  // когда поперечная ошибка последний раз была не на пределе
    float brakeS = 0, brakeV = 0; // прогресс и скорость в момент включения тормоза
    bool wheelBraked[2] = {false, false};
    float wheelBrakePos[2] = {0, 0}; // положение и скорость колеса в момент его торможения
    float wheelBrakeV[2] = {0, 0};
  };

  // Езда с заданными скоростями колес (без конечной точки)
  struct Velocity {
    long startL = 0, startR = 0;
    float posRefL = 0, posRefR = 0;
    float refL = 0, refR = 0; // текущее задание скорости (мм/с)
    float cmdL = 0, cmdR = 0; // к чему оно плавно идет
    bool spin = false;
    bool timed = false;
    bool stopping = false;
    uint32_t endMs = 0;
    float decel = 0;
  };

  // Настройки, приведенные к миллиметрам
  struct Derived {
    float kP = 0, kI = 0, kD = 0;         // % на мм, % на мм*с, % на мм/с
    float accMove = 0, accSpin = 0;       // мм/с^2
    float vMinMove = 0, vMinSpin = 0;     // мм/с
    float maxDecel = 0;                   // мм/с^2
    float tolBase = 0, tolTurn = 0;       // маневр короче этого уже считается выполненным (мм хода колеса)
  };

  TuningConfig _t;                 // копия настроек для контура управления
  Derived _d;
  volatile uint32_t _tuningVersion;
  uint32_t _tuningApplied;

  uint32_t _nowMs;
  long _cntL, _cntR;               // счетчики энкодеров на текущем шаге
  float _velL, _velR;              // измеренные скорости колес (мм/с), сглаженные
  long _stillRefL, _stillRefR;
  uint32_t _lastMotionMs;          // когда колеса двигались последний раз
  uint8_t _stillCounts;

  CtrlState _state;
  bool _nearFinish;                // маневр подходит к цели
  bool _fastLoop;                  // следующий шаг контура - через 1 мс, а не через 5
  Path _p;
  Velocity _v;
  float _iBase, _iTurn;            // интегральные составляющие
  float _lastPowerL, _lastPowerR;  // мощность, выданная на моторы
  AfterBrake _afterBrake;
  uint32_t _brakeStartMs;
  UniMoveResult _brakeResult;
  float _nudgeRampL, _nudgeRampR;
  long _holdL, _holdR;
  bool _rawBrakeL, _rawBrakeR;     // одно колесо под тормозом после stopLeft()/stopRight()
  uint32_t _rawBrakeUntilL, _rawBrakeUntilR;
  // Модель выбега под тормозом: выбег = K * v - C. Своя у каждого колеса и режима:
  // индекс = режим (0 - прямые и дуги, 1 - поворот на месте) * 2 + колесо.
  // Стартует с калибровки и подстраивается после каждой остановки
  float _brakeK[4];
  float _brakeC[4];
  uint8_t _brakeStops[2];          // сколько остановок уже учтено в каждом режиме
  float _brakeSeed[4];             // настройки, с которых модель стартовала
  bool _brakeModelSet;

  uint8_t _releaseBurst;           // столько циклов после остановки моторы повторно обнуляются
  bool _rawMode;                   // моторами управляет motors(): библиотека выходы не трогает
  uint32_t _idleAssertMs;          // когда сторож покоя последний раз выключал моторы

  // Отчет о текущем маневре: собирает контур, публикует при завершении
  UniMoveReport _work;
  UniMoveReport _report;           // защищен _cmdMux
  uint32_t _moveStartMs;

  // Цепочка целей
  bool _chainValid;
  float _chainHeading;
  bool _chainAlongValid;
  float _chainAlong;

  float _goalX, _goalY, _goalPower; // цель moveTo

  // Текущая конечная команда
  bool _curFinite;
  uint32_t _curSeq;
  uint32_t _stopSeq;

  // ---- Обмен с пользовательскими задачами ----
  uint32_t _cmdSeq;                 // защищен _cmdMux
  uint32_t _busySeq;                // номер последней конечной команды
  volatile bool _activeFinite;      // конечная команда еще выполняется
  uint32_t _resultSeq;
  UniMoveResult _result;
  volatile uint32_t _appliedSeq;    // последняя команда, принятая контуром
  volatile uint32_t _stopDoneSeq;   // последний завершенный stop()
  volatile float _lastErrMM, _lastErrDeg;

  // Control variables
  char _currentCommand[16];
  char _robotName[16];

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
  void resolveBoard();
  void initOLED();
  void initTasks();

  void updateDisplay();
  void drawBatteryIcon(int x, int y, int percent);
  void setCommandName(const char* name);
  float readBatteryVoltage();
  int percentFromVoltage(float voltage);
  void updateBattery();

  // RTOS Tasks
  static void controlTask(void* pvParameters);
  static void serviceTask(void* pvParameters);

  // Контур управления
  void controlStep(float dt);
  void refreshTuning();
  void updateOdometry(float dL, float dR);
  void readPose(float& x, float& y, float& heading);

  void applyCommand(const MotionCmd& cmd);
  void applyStop(const MotionCmd& cmd);
  void beginFinite(const MotionCmd& cmd);
  void reportFinish(UniMoveResult result);
  void finishImmediately(UniMoveResult result);

  void stepIdle();
  void stepVelocity(float dt);
  void stepPath(float dt);
  void stepHold(float dt);
  void stepBrake();

  float feedforward(float v, float a, float vmax);
  float brakeLead(float v, bool spin, int8_t wheel);
  bool learnBrake(float v, float coast, bool spin, uint8_t wheel);
  void servoOutput(float eBase, float eTurn, float vL, float vR, float aL, float aR, bool spin, float dt);
  float nudgePower(float err, float vel, float deadband, float gain, float& ramp, float dt, bool& inside);
  void driveOrBrake(float uL, bool brakeL, float uR, bool brakeR);
  void releaseMotors();
  void currentRefVel(float& vL, float& vR);

  void startBrake(AfterBrake after);
  void enterVelocity();
  void startPath(Path& p, float power);
  void pathMeasure(float& sAct, float& finBase, float& finTurn);
  void failPath(UniMoveResult result);
  void verifyPath();
  void pathDone();
  bool chainUsable();

  void beginDist(float power, float dist);
  void beginRotate(float power, float angleDeg, bool absolute);
  void beginSpin(float target, float power, NextPhase next);
  void beginArc(float power, float k, float length, float dir);
  void beginMoveTo(float power, float x, float y);
  void beginMoveToDrive();

  // Постановка команд (из пользовательских задач)
  uint32_t submit(MotionCmd& cmd, bool finite);
  bool waitFinite(uint32_t seq);
  void waitStop(uint32_t seq, uint32_t timeoutMs);
  void submitVelocity(float powerLeft, float powerRight);
  uint32_t startDist(int power, int millimeters);
  uint32_t startTime(float powerLeft, float powerRight, int milliseconds);
  uint32_t startArc(int power, float k, float lengthMM, bool backward);
  uint32_t startRotate(int power, float angleDeg, bool absolute);
  uint32_t startMoveTo(int power, float x, float y);

  // UART control private methods
  void ctrlReceiveUART();
  void uartDispatchCommand(uint8_t* data, uint8_t len);
  void ctrlSendFloat(float val);
  void ctrlSendLong(long val);
  void sendPacket(const uint8_t* payload, uint8_t len);

  // Основной конструктор: публичные передают сюда конфигурацию платформы
  UniBase(const char* robotName, UniConfig config);

  // ---- Внутренние: настройка и диагностика движения ----
  /**
   * @brief Задать параметры регуляторов. Параметры, оставленные как
   * UNI_TUNING_AUTO, берутся из калибровки платы - она своя у каждой версии.
   */
  void setTuning(const TuningConfig& tuning);

  /**
   * @brief Действующие параметры регуляторов (с учетом калибровки платы)
   */
  TuningConfig getTuning();

  /**
   * @brief Чем закончилась последняя команда движения: UNI_MOVE_OK,
   * UNI_MOVE_TIMEOUT, UNI_MOVE_STALLED или UNI_MOVE_INTERRUPTED
   */
  UniMoveResult getMoveResult();

  /**
   * @brief Остаток последнего маневра по ходу (мм): сколько робот не доехал
   * до цели (отрицательный - переехал)
   */
  float getMoveErrorMM();

  /**
   * @brief Остаток последнего маневра по курсу (град)
   */
  float getMoveErrorDeg();

  /**
   * @brief Подробности последнего маневра: остаток после остановки, скорость
   * и выбег при торможении, длительность. Для настройки и диагностики.
   */
  UniMoveReport getMoveReport();

  /**
   * @brief Скорость колеса при 100% мощности (мм/с)
   */
  float getMaxSpeed();

  /**
   * @brief Сколько раз отброшен невозможный скачок счетчика энкодера.
   * В норме 0; ненулевое значение говорит о сбоях чтения энкодера.
   */
  uint32_t getEncoderGlitches();

public:
  /**
   * @brief Конструктор платформы
   * @param robotName Имя робота (будет выведено на экран), макс 15 символов
   * @param board Версия платы (опционально): UNI_BOARD_V1 или UNI_BOARD_V2.
   * По умолчанию определяется автоматически
   */
  UniBase(const char* robotName = "UNI Robot", UniBoardVersion board = UNI_BOARD_AUTO);
  UniBase(UniBoardVersion board);
  ~UniBase();

  /**
   * @brief Инициализация периферии (опционально). Если не вызвать,
   * выполнится автоматически при первой команде.
   * @param robotName Имя робота (опционально, переопределяет имя из конструктора)
   */
  void begin(const char* robotName = nullptr);

  /**
   * @brief Инициализация с явным указанием версии платы. Без него версия
   * определяется автоматически: на плате V2 на шине I2C отвечает EEPROM.
   * @param board Версия платы: UNI_BOARD_V1 или UNI_BOARD_V2
   */
  void begin(const char* robotName, UniBoardVersion board);
  void begin(UniBoardVersion board);


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
   * Работает асинхронно до вызова stop(): скорости колес удерживаются
   * следящим приводом, в отличие от motors() без стабилизации. Команду можно
   * повторять сколько угодно часто: задание меняется плавно, без сброса привода.
   * @param powerLeft Скорость левого колеса (-100..100, % от максимальной)
   * @param powerRight Скорость правого колеса (-100..100)
   */
  void motorsSync(int powerLeft, int powerRight);

  /**
   * @brief То же, что motorsSync(), но скорости колес заданы в мм/с.
   * Предел - скорость колеса на 100% мощности.
   */
  void motorsSpeed(float leftMMperSec, float rightMMperSec);

  // ---- Blocking Movement ----
  // Возвращают true, если цель достигнута, и false, если робот уперся,
  // не уложился в таймаут или команду прервали

  /**
   * @brief Проехать прямо. Назад - при отрицательной мощности или
   * отрицательной дистанции.
   */
  bool moveDist(int power, int millimeters);
  bool moveTime(int power, int milliseconds);
  bool moveArcDist(int power, int angle, int millimeters);
  bool moveArcTime(int power, int angle, int milliseconds);
  bool rotate(int power, int angle);

  /**
   * @brief Поворот к абсолютному курсу одометрии кратчайшим путем.
   * @param angleDeg Целевой курс в градусах (в системе getAngle()/setPosition())
   */
  bool rotateTo(int power, float angleDeg);

  /**
   * @brief Поехать в точку одометрии: доворот на курс к цели, затем прямая
   * с подруливанием на точку.
   * @param x, y Целевая точка (мм, в системе getOdometry()/setPosition())
   */
  bool moveTo(int power, float x, float y);

  /**
   * @brief Дуга с заданной геометрией: радиус и угол.
   * @param radiusMM Радиус дуги по центру робота (не меньше половины колеи)
   * @param angleDeg Угол дуги; знак задает сторону поворота (как у rotate)
   */
  bool moveArcRadius(int power, float radiusMM, float angleDeg);

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
  /**
   * @brief Остановка. HARD - тормоз до полной остановки (вызов ждет ее),
   * SOFT - плавное замедление по профилю (вызов возвращается сразу).
   */
  void stop(int stopType = HARD);
  void stopLeft(int stopType = HARD);
  void stopRight(int stopType = HARD);

  // ---- Speed ----

  /**
   * @brief Перевести скорость в мм/с в значение power для команд движения
   */
  int speedToPower(float mmPerSec);

  /**
   * @brief Перевести скорость поворота на месте в град/с в значение power
   * для rotate() и rotateTo()
   */
  int turnRateToPower(float degPerSec);

  /**
   * @brief Измеренная скорость центра робота (мм/с, назад - отрицательная)
   */
  float getSpeed();

  /**
   * @brief Измеренная скорость поворота (град/с)
   */
  float getTurnRate();

  // ---- Odometry ----
  void resetDistance();

  /**
   * @brief Путь, пройденный центром робота (мм). Повороты на месте
   * путь не увеличивают.
   */
  float getDistance();

  /**
   * @brief Сделать текущий курс нулевым для getAngle() и rotateTo().
   * Координаты и getAbsAngle() не меняются.
   */
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

  /**
   * @brief Напряжение батареи (В), сглаженное. 0 - измерить не удалось
   */
  float getBatteryVoltage();

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

  /**
   * @brief Многострочный текст мелким шрифтом: до 8 строк по 21 символу,
   * строки разделяются символом '\n'. Удобно для отладки без Serial Monitor.
   */
  void displayText(const char* text);

  void displayClear();
};

#endif
