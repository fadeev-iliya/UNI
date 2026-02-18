#ifndef UNIDEV_H
#define UNIDEV_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Debug mode - раскомментируйте для вывода отладочных сообщений
// #define DEBUG_MODE

// Порты UniDev
#define P1 12  // GPIO_NUM_12 - Traffic light red
#define P2 13  // GPIO_NUM_13 - Traffic light yellow
#define P3 14  // GPIO_NUM_14 - Traffic light green / side ultrasonic trig
#define P4 15  // GPIO_NUM_15 - Side ultrasonic echo
#define P5 17  // GPIO_NUM_17 - Button
#define P6 16  // GPIO_NUM_16 - Front ultrasonic trig
#define P7 32  // GPIO_NUM_32 - Front ultrasonic echo
#define P8 23  // GPIO_NUM_23 - NeoPixel ring

#define NEOPIXEL_COUNT 24

enum TrafficLightColor {
  TRAFFIC_OFF,
  TRAFFIC_RED,
  TRAFFIC_YELLOW,
  TRAFFIC_GREEN
};

/// @brief Класс работы с UniDev
class UniDev {
private:
  static bool _initialized;
  static Adafruit_NeoPixel* _staticNeoPixels;
  static uint8_t _pinModes[40];
  
  void ensureInitialized();
  void ensurePinMode(uint8_t pin, uint8_t mode);
  
public:
  /**
   * @brief Конструктор класса UniDev.
   */
  UniDev();
  
  /**
   * @brief Статическая инициализация (вызывается из UniBase::begin()).
   */
  static void staticInit();
  
  /**
   * @brief Явная инициализация (опционально).
   */
  void begin();
  
  // Датчики
  /**
   * @brief Измерение расстояния ультразвуком.
   * @param trig Порт trigger
   * @param echo Порт echo
   * @return Расстояние в миллиметрах
   */
  int ultraSonic(int trig, int echo);
  
  /**
   * @brief Чтение датчика линии.
   * @param port Порт датчика
   * @return Аналоговое значение (0-4095)
   */
  int lineSensor(int port);
  
  /**
   * @brief Чтение цифрового датчика.
   * @param port Порт датчика
   * @return 0 или 1
   */
  int digitalSensor(int port);
  
  /**
   * @brief Чтение аналогового датчика.
   * @param port Порт датчика
   * @return Значение (0-4095)
   */
  int analogSensor(int port);
  
  /**
   * @brief Получить текущий режим пина.
   * @param pin Номер пина
   * @return Режим пина: OUTPUT, INPUT, INPUT_PULLUP, INPUT_PULLDOWN или -1 при ошибке
   */
  int getPinMode(uint8_t pin);
  
  // Кнопка
  /**
   * @brief Ждать нажатия кнопки.
   * @param port Порт кнопки
   */
  void waitButton(int port);
  
  /**
   * @brief Получить состояние кнопки.
   * @param port Порт кнопки
   * @return true если нажата
   */
  bool getButtonState(int port);
  
  // Servo
  /**
   * @brief Установить угол сервопривода.
   * @param port Порт сервопривода
   * @param angle Угол (0-180)
   */
  void servo(int port, int angle);
  
  // NeoPixel базовые функции
  /**
   * @brief Установить цвет одного пикселя.
   * @param index Индекс (0-23)
   * @param r Красный (0-255)
   * @param g Зелёный (0-255)
   * @param b Синий (0-255)
   */
  void pixel(int index, int r, int g, int b);
  
  /**
   * @brief Установить цвет всех пикселей.
   * @param r Красный
   * @param g Зелёный
   * @param b Синий
   */
  void pixelsAll(int r, int g, int b);
  
  /**
   * @brief Очистить все пиксели.
   */
  void pixelsClear();
  
  /**
   * @brief Отобразить изменения.
   */
  void pixelsShow();
  
  /**
   * @brief Установить яркость.
   * @param brightness Яркость (0-100)
   */
  void pixelsBrightness(int brightness);
  
  // NeoPixel эффекты
  /**
   * @brief Эффект радуги.
   * @param speed Процент скорости (0 - самое медленное, 100 - самое быстрое)
   * @param duration Общее время выполнения (мс)
   */
  void pixelsRainbow(int speed, int duration);
  
  /**
   * @brief Бегущий огонь.
   * @param r Красный
   * @param g Зелёный
   * @param b Синий
   * @param duration Общее время выполнения (мс)
   */
  void pixelsRunning(int r, int g, int b, int duration);
  
  /**
   * @brief Эффект дыхания.
   * @param r Красный
   * @param g Зелёный
   * @param b Синий
   * @param duration Общее время выполнения (мс)
   */
  void pixelsBreathing(int r, int g, int b, int duration);
  
  /**
   * @brief Заполнение кольца.
   * @param r Красный
   * @param g Зелёный
   * @param b Синий
   * @param duration Общее время выполнения (мс)
   */
  void pixelsFill(int r, int g, int b, int duration);
  
  /**
   * @brief Искры.
   * @param r Красный
   * @param g Зелёный
   * @param b Синий
   * @param duration Общее время выполнения (мс)
   * @param count Количество искр
   */
  void pixelsSparkle(int r, int g, int b, int duration, int count);
  
  /**
   * @brief Вращающийся сегмент.
   * @param r Красный
   * @param g Зелёный
   * @param b Синий
   * @param duration Общее время выполнения (мс)
   * @param segmentLength Длина сегмента в светодиодах (1-22)
   * @param rotations Количество оборотов
   */
  void pixelsRotating(int r, int g, int b, int duration, int segmentLength, int rotations);
  
  /**
   * @brief Спиннер.
   * @param r Красный
   * @param g Зелёный
   * @param b Синий
   * @param duration Общее время выполнения (мс)
   */
  void pixelsSpinner(int r, int g, int b, int duration);
  
  // Светофор
  /**
   * @brief Установить цвет светофора.
   * @param color Цвет светофора
   */
  void setTrafficLight(TrafficLightColor color);
  
  /**
   * @brief Последовательность светофора.
   */
  void trafficLightSequence();
};

#endif

