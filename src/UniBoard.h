#ifndef UNIBOARD_H
#define UNIBOARD_H

#include <Arduino.h>

// ============ Версии платы ============

enum UniBoardVersion : uint8_t {
  UNI_BOARD_AUTO = 0, // определить автоматически по наличию EEPROM на шине I2C
  UNI_BOARD_V1   = 1, // плата без EEPROM
  UNI_BOARD_V2   = 2  // плата с EEPROM M24C02 и pogo-портами
};

// EEPROM M24C02 (E0-E2 на GND) стоит только на плате V2
#define UNI_EEPROM_ADDR 0x50

#define UNI_PORT_COUNT 8

// Логические номера портов (UNI_PORT_BASE + 1..8). P1..P8 возвращают их,
// пока версия платы еще не может быть определена
#define UNI_PORT_BASE 100

// ============ Внутренние типы библиотеки ============
// Настройки платформы и регуляторов зашиты в библиотеку: в скетчах они не видны,
// поэтому вынесены в отдельное пространство имен

namespace uni_detail {

// Значение пина в UniConfig: "взять из профиля платы"
constexpr uint8_t UNI_PIN_AUTO = 0xFF;

// Значение параметра движения: "взять из калибровки платы"
constexpr int UNI_TUNING_AUTO = -1;

// Режим ШИМ моторов. Зависит от драйверов: медленные TA6586 (плата V1)
// откалиброваны на "ход/выбег", быстрым DRV8837 (плата V2) нужен "ход/тормоз"
enum UniMotorDecay : uint8_t {
  UNI_DECAY_AUTO = 0, // по версии платы
  UNI_DECAY_FAST = 1, // "ход/выбег": в паузах ШИМ выходы драйвера отключены
  UNI_DECAY_SLOW = 2  // "ход/тормоз": в паузах ШИМ обмотка мотора замкнута
};

// ============ Конфигурация физики и ПИД ============

// Калибровка зависит от драйверов моторов, поэтому у каждой версии платы она
// своя (см. UniBoard.cpp). Параметры, оставленные как UNI_TUNING_AUTO, берутся
// из калибровки платы, любое другое значение ее переопределяет
// Тик здесь и ниже - единица ticksPerRev из UniConfig (от режима счета
// энкодера не зависит). Внутри библиотека считает в миллиметрах.
struct TuningConfig {
    int minPower = UNI_TUNING_AUTO;         // Минимальная мощность страгивания (%)
    float pGain = UNI_TUNING_AUTO;          // P: % мощности на тик ошибки положения
    float iGain = UNI_TUNING_AUTO;          // I: % мощности в секунду на тик ошибки
    float dGain = UNI_TUNING_AUTO;          // D: % мощности на тик/с ошибки скорости

    // Точный поворот (замкнутый контур по одометрии)
    float rotateTolDeg = UNI_TUNING_AUTO;   // Поворот на меньший угол считается уже выполненным (град)
    float rotateAccel = UNI_TUNING_AUTO;    // Замедление профиля поворота (тики/с^2)
    float rotateMinSpeed = UNI_TUNING_AUTO; // Минимальная скорость подхода к цели (тики/с)

    // Прямолинейное движение (замкнутый профиль по энкодерам)
    float moveTolMM = UNI_TUNING_AUTO;      // Движение на меньшее расстояние считается уже выполненным (мм)
    float moveAccel = UNI_TUNING_AUTO;      // Замедление профиля движения (тики/с^2)
    float moveMinSpeed = UNI_TUNING_AUTO;   // Минимальная скорость подхода к цели (тики/с)

    // Следящий привод. Ведущая ось - вдоль пути (ход на прямой, угол в повороте),
    // поперечная - то, что нужно удержать (курс на прямой, центр в повороте)
    float mainAxisGain = UNI_TUNING_AUTO;   // Множитель P и I по ведущей оси
    float crossAxisGain = UNI_TUNING_AUTO;  // Множитель P и I по поперечной оси
    float iLimit = UNI_TUNING_AUTO;         // Предел интегральной составляющей (% мощности)
    float brakeGain = UNI_TUNING_AUTO;      // Доля P, когда робот опережает задание (0..1)
    float leashMM = UNI_TUNING_AUTO;        // Поводок: предел отставания от задания по ведущей оси (мм)
    float crossLeashMM = UNI_TUNING_AUTO;   // Предел ошибки по поперечной оси (мм)
    float rampUpFactor = UNI_TUNING_AUTO;   // Разгон быстрее замедления профиля во столько раз
    float maxDecel = UNI_TUNING_AUTO;       // Предел сброса заданной скорости (тики/с^2)
    float powerSlew = UNI_TUNING_AUTO;      // Предел скорости изменения мощности (%/с)
    float accelFF = UNI_TUNING_AUTO;        // Постоянная времени мотора для упреждения по ускорению (с)

    // Финиш маневра. Выбег колеса под тормозом со скорости v (мм/с) считается как
    // BrakeLeadMs / 1000 * v - BrakeDropMM: на эту величину раньше включается тормоз.
    // Это стартовые значения, дальше робот уточняет их сам по каждой остановке
    float moveBrakeLeadMs = UNI_TUNING_AUTO;   // Прямые и дуги: наклон (мс)
    float moveBrakeDropMM = UNI_TUNING_AUTO;   // Прямые и дуги: вычитаемое (мм)
    float rotateBrakeLeadMs = UNI_TUNING_AUTO; // Повороты на месте: наклон (мс)
    float rotateBrakeDropMM = UNI_TUNING_AUTO; // Повороты на месте: вычитаемое (мм)
    float brakeTimeMs = UNI_TUNING_AUTO;    // Минимальное время тормоза при stop(HARD) (мс)
    float stallTimeMs = UNI_TUNING_AUTO;    // Колеса стоят дольше - робот уперся (мс), 0 - не проверять
};

} // namespace uni_detail

// Пины, которые отличаются между версиями платы
struct UniBoardPins {
  uint8_t leftMotorA;
  uint8_t leftMotorB;
  uint8_t rightMotorA;
  uint8_t rightMotorB;

  uint8_t leftEncInt;
  uint8_t leftEncDir;
  uint8_t rightEncInt;
  uint8_t rightEncDir;

  uint8_t ledPin;
  uint8_t batteryPin;

  uint8_t uartRxPin;
  uint8_t uartTxPin;

  uint8_t ports[UNI_PORT_COUNT]; // GPIO портов P1..P8
};

/// @brief Версия платы UNI и ее распиновка (общая для UniBase и UniDev)
class UniBoard {
  friend class UniBase;
  friend class UniDev;

private:
  static UniBoardVersion _version;

  // ---- Только для классов библиотеки ----

  /**
   * @brief Версия платы. Если она не задана вручную, при первом вызове
   * определяется по EEPROM и запоминается.
   */
  static UniBoardVersion version();

  /**
   * @brief Опросить шину I2C: отвечает EEPROM - плата V2, иначе V1.
   * Результат не запоминается, заданную версию не меняет.
   */
  static UniBoardVersion detect(uint8_t sda = 21, uint8_t scl = 22);

  /**
   * @brief Калибровка движения текущей версии платы (все параметры заданы).
   */
  static uni_detail::TuningConfig tuning();

  /**
   * @brief Скорость колеса при 100% мощности на текущей версии платы (тики/с).
   */
  static float maxTicksPerSec();

  /**
   * @brief Напряжение батареи, при котором снята калибровка скорости (В).
   * По нему мощность моторов пересчитывается под текущий заряд.
   */
  static float batteryRefVoltage();

  /**
   * @brief Режим ШИМ моторов для драйверов текущей версии платы.
   */
  static uni_detail::UniMotorDecay motorDecay();

public:
  /**
   * @brief Задать версию платы вручную (отключает автоопределение).
   * Вызывать до begin() и до первого обращения к портам P1..P8.
   * @param version UNI_BOARD_V1, UNI_BOARD_V2 или UNI_BOARD_AUTO (вернуть автоопределение)
   */
  static void set(UniBoardVersion version);


  /**
   * @brief Возвращает true, если версия уже задана или определена.
   */
  static bool isSet();


  /**
   * @brief Имя версии платы: "V1" или "V2".
   */
  static const char* name();

  /**
   * @brief Распиновка текущей версии платы.
   */
  static const UniBoardPins& pins();


  /**
   * @brief GPIO порта по его номеру (на это раскрываются P1..P8).
   * @param index Номер порта 1..8
   */
  static uint8_t port(uint8_t index);

  /**
   * @brief Привести порт к номеру GPIO: логические номера портов
   * переводятся по распиновке платы, обычные номера GPIO не меняются.
   */
  static int gpio(int port);
};

#endif
