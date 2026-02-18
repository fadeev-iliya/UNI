#ifndef UNIBASE_H
#define UNIBASE_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// Debug mode - раскомментируйте для вывода отладочных сообщений
#define DEBUG_MODE

// Stop types
#define SMOOTH 0
#define HARD 1

// Motor pins
#define LEFT_MOTOR_PIN_A GPIO_NUM_2
#define LEFT_MOTOR_PIN_B GPIO_NUM_4
#define RIGHT_MOTOR_PIN_A GPIO_NUM_27
#define RIGHT_MOTOR_PIN_B GPIO_NUM_26

// Encoder pins
#define LEFT_ENC_INTERRUPT GPIO_NUM_18
#define LEFT_ENC_DIRECTION GPIO_NUM_19
#define RIGHT_ENC_INTERRUPT GPIO_NUM_35
#define RIGHT_ENC_DIRECTION GPIO_NUM_34

// OLED pins
#define OLED_SDA GPIO_NUM_21
#define OLED_SCL GPIO_NUM_22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// LED and Battery pins
#define LED_PIN GPIO_NUM_25
#define BATTERY_PIN GPIO_NUM_33

// Robot parameters
#define TRACK_LENGTH_MM 104
#define WHEEL_DIAMETER_MM 45
#define TICKS_PER_REV_LEFT 690
#define TICKS_PER_REV_RIGHT 690
#ifndef PI
#define PI 3.141592653589793
#endif

struct OdometryData {
  float x;
  float y;
  float angle;
};


// UART pins for control mode
#define CTRL_UART_RX_PIN 17
#define CTRL_UART_TX_PIN 16
#define CTRL_UART_BUFFER_SIZE 40
#define CTRL_UART_BAUD_RATE 9600

/// @brief Класс управления UniBase
class UniBase {
private:
  // Display
  Adafruit_SSD1306* _display;
  bool _displayInitialized;
  bool _customDisplayMode;
  String _customDisplayText;
  String _customDisplayName;
  
  // LED
  volatile bool _ledBlinking;
  volatile int _ledBlinkInterval;
  
  // Battery
  int _batteryPercent;
  bool _batteryValid;
  
  // Task handle for second core
  TaskHandle_t _secondCore;
  
  // Encoder variables
  volatile long _lEnc;
  volatile long _rEnc;
  volatile long _lEncOld;
  volatile long _rEncOld;
  volatile long _encPrevTime;
  volatile float _encPrevErr;
  volatile float _I;
  
  // Odometry variables  
  volatile float _xPos;
  volatile float _yPos;
  volatile float _theta;
  
  // Tracking variables
  volatile float _totalDistance;
  volatile float _totalAngle;
  
  // Target angle for accumulated rotation control
  volatile float _targetTheta;
  volatile bool _useTargetTheta;
  
  // Control variables
  String _currentCommand;
  String _robotName;
  
  // EncMove on second core variables
  volatile bool _encMoveActive;
  volatile int _encMovePowerL;
  volatile int _encMovePowerR;
  volatile long _encMoveStartL;  
  volatile long _encMoveStartR;
  
  // Rate limiting variables для плавного изменения мощности
  volatile int _lastLeftPower;
  volatile int _lastRightPower;
  
  // Constants
  const float _distPerTickLeft;
  const float _distPerTickRight;
  
  // Private methods
  void initMotors();
  void initEncoders();
  void initOLED();
  void initSecondCore();
  
  void leftMotor(int power);
  void rightMotor(int power);
  void updateOdometry();
  void updateDisplay();
  void drawBatteryIcon(int x, int y, int percent);

  void encMove(int pL, int pR);
  void encMove(int pL, int pR, long startLEnc, long startREnc);
  
  void startEncMoveOnSecondCore(int powerL, int powerR);
  
  static void leftEncISR();
  static void rightEncISR();
  static void secondCoreLoop(void* pvParameters);

  static UniBase* _instance;

  // ============ UART Control Mode ============
  HardwareSerial* _ctrlSerial;
  bool _ctrlInitialized;
  
  // UART receive buffer
  uint8_t _ctrlBuffer[CTRL_UART_BUFFER_SIZE];
  uint8_t _ctrlBufferIndex;
  uint32_t _uartLastRxTime;
  
  // FreeRTOS command dispatcher
  struct UartCmd { uint8_t type; int p1; int p2; int p3; };
  QueueHandle_t _cmdQueue;
  TaskHandle_t _cmdTaskHandle;
  volatile bool _abortCommand;
  
  // UART control private methods
  void ctrlReceiveUART();
  void uartDispatchCommand(uint8_t* data, uint8_t len);
  void ctrlSendFloat(float val);
  void ctrlSendLong(long val);
  static void uartCommandTaskFunc(void* param);

public:
  /**
   * @brief Конструктор класса управления роботом UniBase
   * @param robotName Имя робота (по умолчанию "UNI").
   */
  UniBase(String robotName = "UNI");
  
  /**
   * @brief Деструктор класса.
   */
  ~UniBase();
  
  /**
   * @brief Инициализация всех компонентов робота.
   */
  void begin();
  
  /**
   * @brief Инициализация всех компонентов робота с заданием имени.
   * @param robotName Имя робота
   */
  void begin(String robotName);
  
  // Motor control
  /**
   * @brief Управление моторами напрямую.
   * @param powerLeft Мощность левого мотора (-100...100)
   * @param powerRight Мощность правого мотора (-100...100)
   */
  void motors(int powerLeft, int powerRight);
  
  /**
   * @brief Управление левым мотором.
   * @param power Мощность (-100...100)
   */
  void motorLeft(int power);
  
  /**
   * @brief Управление правым мотором.
   * @param power Мощность (-100...100)
   */
  void motorRight(int power);
  
  /**
   * @brief Движение по дуге с выравниванием.
   * @param power Мощность моторов (-100...100)
   * @param angle Угол дуги (-90...90)
   */
  void motorsArc(int power, float angle);
  
  // Movement
  /**
   * @brief Проехать заданное расстояние.
   * @param power Мощность моторов (-100...100)
   * @param millimeters Расстояние в миллиметрах
   */
  void moveDist(int power, int millimeters);
  
  /**
   * @brief Движение в течение заданного времени.
   * @param power Мощность моторов (-100...100)
   * @param milliseconds Время в миллисекундах
   */
  void moveTime(int power, int milliseconds);
  
  /**
   * @brief Движение по дуге на заданное расстояние.
   * @param power Мощность моторов (-100...100)
   * @param angle Угол дуги (-90...90)
   * @param millimeters Расстояние в миллиметрах
   */
  void moveArcDist(int power, int angle, int millimeters);
  
  /**
   * @brief Движение по дуге в течение заданного времени.
   * @param power Мощность моторов (-100...100)
   * @param angle Угол дуги (-90...90)
   * @param milliseconds Время в миллисекундах
   */
  void moveArcTime(int power, int angle, int milliseconds);
  
  /**
   * @brief Поворот на заданный угол.
   * @param power Мощность моторов (0...100)
   * @param angle Угол в градусах (+ по часовой, - против)
   */
  void rotate(int power, int angle);
  
  // Stop
  /**
   * @brief Остановка робота.
   * @param stopType SMOOTH = плавная, HARD = жёсткая
   */
  void stop(int stopType);
  
  /**
   * @brief Остановка левого мотора.
   * @param stopType SMOOTH = плавная, HARD = жёсткая
   */
  void stopLeft(int stopType);
  
  /**
   * @brief Остановка правого мотора.
   * @param stopType SMOOTH = плавная, HARD = жёсткая
   */
  void stopRight(int stopType);
  
  // Odometry
  /**
   * @brief Сбросить пройденное расстояние.
   */
  void resetDistance();
  
  /**
   * @brief Получить пройденное расстояние.
   * @return Расстояние в миллиметрах
   */
  float getDistance();
  
  /**
   * @brief Сбросить угол поворота.
   */
  void resetAngle();
  
  /**
   * @brief Получить пройденный угол.
   * @return Угол в градусах
   */
  float getAngle();
  
  /**
   * @brief Получить полную одометрию (x, y, angle).
   * @return OdometryData структура с данными
   */
  OdometryData getOdometry();
  
  /**
   * @brief Получить абсолютную координату X.
   * @return Координата X в миллиметрах
   */
  float getAbsX();
  
  /**
   * @brief Получить абсолютную координату Y.
   * @return Координата Y в миллиметрах
   */
  float getAbsY();
  
  /**
   * @brief Получить абсолютный угол поворота.
   * @return Угол в градусах
   */
  float getAbsAngle();
  
  /**
   * @brief Получить значение тиков левого энкодера.
   * @return Количество тиков
   */
  long getLeftTicks();
  
  /**
   * @brief Получить значение тиков правого энкодера.
   * @return Количество тиков
   */
  long getRightTicks();
  
  // Display
  /**
   * @brief Вывести текст на дисплей.
   * @param text Текст для отображения
   */
  void displayPrint(String text);
  
  /**
   * @brief Вывести C-строку на дисплей.
   * @param text C-строка для отображения
   */
  void displayPrint(const char* text);
  
  /**
   * @brief Вывести целое число на дисплей.
   * @param value Число для отображения
   */
  void displayPrint(int value);
  
  /**
   * @brief Вывести длинное целое число на дисплей.
   * @param value Число для отображения
   */
  void displayPrint(long value);
  
  /**
   * @brief Вывести число с плавающей точкой на дисплей (2 знака после запятой).
   * @param value Число для отображения
   */
  void displayPrint(float value);
  
  /**
   * @brief Вывести число с плавающей точкой двойной точности на дисплей (4 знака после запятой).
   * @param value Число для отображения
   */
  void displayPrint(double value);
  
  /**
   * @brief Вывести булевое значение на дисплей.
   * @param value true или false
   */
  void displayPrint(bool value);
  
  /**
   * @brief Вывести именованное значение на дисплей.
   * @param name Название (вверху дисплея)
   * @param value Значение для отображения
   */
  void displayPrint(const char* name, const char* value);
  void displayPrint(const char* name, int value);
  void displayPrint(const char* name, long value);
  void displayPrint(const char* name, float value);
  void displayPrint(const char* name, double value);
  void displayPrint(const char* name, bool value);
  void displayPrint(const char* name, String value);
  
  /**
   * @brief Очистить дисплей и вернуть стандартный режим.
   */
  void displayClear();
  
  // Utility
  /**
   * @brief Вывести одометрию в Serial.
   */
  void printOdometry();
  
  // LED
  /**
   * @brief Запустить мигание светодиода.
   * @param interval Интервал мигания в миллисекундах (0 = выключить мигание)
   */
  void blinkLED(int interval);
  
  // Battery
  /**
   * @brief Получить уровень заряда батареи.
   * @return Процент заряда (0-100), -1 если данные недоступны
   */
  int getBatteryPower();

  /**
   * @brief Активировать режим управления по UART.
   * Вызывайте в loop(). Читает команды UART2 и выполняет их.
   */
  void UniBaseControl();

};

#endif

