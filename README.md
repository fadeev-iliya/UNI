# UNI Library

Библиотека для управления роботом UniBase на базе **ESP32**.

## Документация
 -> [Перейти к документации](https://fadeev-iliya.github.io/UNI)

> **Как обновлять документацию:** разделы API сгруппированы по классам и совпадают
> с секциями `public:` в заголовочных файлах. Добавили метод — найдите нужную
> таблицу ниже и допишите строку. Новый пример — добавьте строку в раздел Примеры.

---

## Содержание
- [Установка](#установка)
- [Быстрый старт](#быстрый-старт)
- [Конфигурация](#конфигурация)
- [UniBase — управление платформой](#unibase--управление-платформой)
- [UniDev — внешние модули](#unidev--внешние-модули)
- [Примеры](#примеры)

---

## Установка

1. Скопируйте папку `UNI` в директорию `libraries` вашего Arduino IDE.
2. Перезапустите Arduino IDE.
3. В скетче подключите: `#include <UNI.h>`

**Зависимости** (установить через Library Manager):
- `Adafruit GFX Library`
- `Adafruit SSD1306`
- `Adafruit NeoPixel`

---

## Быстрый старт

```cpp
#include <UNI.h>

UniBase robot("UNI");   // имя на дисплее
UniDev  module;         // внешние модули

void setup() {
  robot.begin();        // инициализация (опционально — запустится сама при первой команде)

  robot.moveDist(50, 500);   // вперёд 500 мм на мощности 50%
  robot.rotate(50, 90);      // поворот 90° вправо
  robot.stop();
}

void loop() { }
```

---

## Конфигурация

Геометрию и пины платформы можно переопределить через `UniConfig`,
параметры регуляторов — через `TuningConfig`.

```cpp
UniConfig cfg;
cfg.trackLengthMM  = 106.0f;   // ширина колеи (мм) — для калибровки поворота
cfg.wheelDiameterMM = 45.0f;   // диаметр колеса (мм)
cfg.ticksPerRevLeft  = 690.0f; // тиков энкодера на оборот, левое колесо
cfg.ticksPerRevRight = 690.0f; // тиков энкодера на оборот, правое колесо

UniBase robot("UNI", cfg);
```

| Поле `UniConfig`    | По умолчанию | Описание |
|---------------------|-------------|----------|
| `leftMotorA/B`      | 2, 4        | Пины левого мотора |
| `rightMotorA/B`     | 27, 26      | Пины правого мотора |
| `leftEncInt/Dir`    | 18, 19      | Пины левого энкодера |
| `rightEncInt/Dir`   | 35, 34      | Пины правого энкодера |
| `ledPin`            | 25          | Пин индикаторного светодиода |
| `batteryPin`        | 33          | Пин АЦП батареи |
| `wheelDiameterMM`   | 45.0        | Диаметр колеса (мм) |
| `trackLengthMM`     | 106.0       | Колея (мм) |
| `ticksPerRevLeft`   | 690.0       | Тиков/оборот, левый энкодер |
| `ticksPerRevRight`  | 690.0       | Тиков/оборот, правый энкодер |
| `uartBaudRate`      | 38400       | Скорость UART (для UniBaseControl) |
| `uartRxPin/TxPin`   | 16, 17      | Пины UART (для UniBaseControl) |

```cpp
// Настройка регуляторов на лету
TuningConfig tuning = robot.getTuning();
tuning.rotateTolDeg = 0.5f;    // точнее повороты
tuning.moveAccel    = 1500.0f; // мягче торможение
robot.setTuning(tuning);
```

| Поле `TuningConfig`  | По умолчанию | Описание |
|----------------------|-------------|----------|
| `minPower`           | 15          | Минимальная мощность страгивания |
| `pGain / iGain / dGain` | 0.5 / 0.005 / 0.0 | ПИД прямолинейного движения |
| `rotateTolDeg`       | 1.0         | Допуск поворота (°) |
| `rotateAccel`        | 1300.0      | Замедление профиля поворота (тики/с²) |
| `rotateMinSpeed`     | 40.0        | Минимальная скорость подхода к углу (тики/с) |
| `moveTolMM`          | 1.0         | Допуск движения (мм) |
| `moveAccel`          | 1800.0      | Замедление профиля движения (тики/с²) |
| `moveMinSpeed`       | 60.0        | Минимальная скорость подхода к цели (тики/с) |

> **Подсказки по настройке:**
> - Робот перелетает точку → уменьшите `moveAccel`
> - Физический угол не совпадает с одометрией → калибруйте `trackLengthMM`, не ПИД

---

## UniBase — управление платформой

### Инициализация

```cpp
UniBase robot;                   // имя "UNI Robot" по умолчанию
UniBase robot("MyRobot");        // своё имя на дисплее
UniBase robot("MyRobot", cfg);   // с кастомной конфигурацией

robot.begin();                   // явная инициализация (опционально)
```

### Управление моторами

| Метод | Описание |
|-------|----------|
| `motors(left, right)` | Прямое управление (−100…100) |
| `motorLeft(power)` | Только левый мотор |
| `motorRight(power)` | Только правый мотор |
| `motorsArc(power, angle)` | Дуга по соотношению скоростей (−90…90°) |
| `motorsSync(left, right)` | Движение с удержанием соотношения скоростей по энкодерам |

### Блокирующие команды движения

Функция возвращает управление только после завершения движения.

| Метод | Описание |
|-------|----------|
| `moveDist(power, mm)` | Проехать дистанцию в миллиметрах |
| `moveTime(power, ms)` | Ехать заданное время |
| `moveArcDist(power, angle, mm)` | Дуга на дистанцию (angle: −90…90°) |
| `moveArcTime(power, angle, ms)` | Дуга на время |
| `moveArcRadius(power, radiusMM, angleDeg)` | Дуга с заданным радиусом и углом |
| `rotate(power, angle)` | Поворот на угол (+ по часовой) |
| `rotateTo(power, angleDeg)` | Поворот к абсолютному курсу одометрии (кратчайший путь) |
| `moveTo(power, x, y)` | Доехать до точки одометрии: доворот + прямая |

### Асинхронные команды движения

Запускают движение и **сразу возвращают управление** — используйте `isMoving()` / `waitMove()`.

| Метод | Описание |
|-------|----------|
| `moveDistAsync(power, mm)` | |
| `moveTimeAsync(power, ms)` | |
| `moveArcDistAsync(power, angle, mm)` | |
| `moveArcTimeAsync(power, angle, ms)` | |
| `moveArcRadiusAsync(power, radiusMM, angleDeg)` | |
| `rotateAsync(power, angle)` | |
| `rotateToAsync(power, angleDeg)` | |
| `moveToAsync(power, x, y)` | |
| `isMoving()` | `true`, если робот выполняет команду |
| `waitMove(timeoutMs = 0)` | Ждать конца движения; `false` — таймаут |
| `holdPosition()` | Удерживать текущую позицию (сервопривод) |

```cpp
// Пример: делать что-то, пока робот едет
robot.moveDistAsync(50, 800);
while (robot.isMoving()) {
  robot.displayPrint("Dist", robot.getDistance());
  delay(50);
}

// Страховка от застревания
robot.moveDistAsync(50, 1000);
if (!robot.waitMove(5000)) {
  robot.stop(HARD);
}
```

### Остановка

| Метод | Описание |
|-------|----------|
| `stop(stopType = HARD)` | Остановить оба мотора |
| `stopLeft(stopType)` | Остановить левый мотор |
| `stopRight(stopType)` | Остановить правый мотор |

`stopType`: `SOFT` (выбег) или `HARD` (торможение).

### Одометрия

| Метод | Описание |
|-------|----------|
| `getDistance()` | Пройденное расстояние (мм) с момента сброса |
| `resetDistance()` | Сбросить счётчик расстояния |
| `getAngle()` | Накопленный угол (°) с момента сброса |
| `resetAngle()` | Сбросить счётчик угла |
| `getOdometry()` | `OdometryData {x, y, angle}` — позиция в системе координат |
| `getAbsX()` | Координата X (мм) |
| `getAbsY()` | Координата Y (мм) |
| `getAbsAngle()` | Абсолютный курс (°) |
| `setPosition(x, y, angle)` | Задать позу одометрии (например, стартовую клетку) |
| `getLeftTicks()` | Тики левого энкодера |
| `getRightTicks()` | Тики правого энкодера |
| `printOdometry()` | Вывести X, Y, угол в Serial |

```cpp
// Привязка к полю
robot.setPosition(0, 0, 0);
robot.moveTo(50, 400, 0);
robot.moveTo(50, 400, 400);
robot.rotateTo(50, 0);  // вернуть исходный курс
```

### Дисплей

```cpp
robot.displayPrint("Hello");         // строка
robot.displayPrint(42);              // число (int, long, float, double, bool)
robot.displayPrint("Speed", 75);     // имя + значение (имя вверху, значение крупно)
robot.displayClear();                // вернуть стандартный режим (имя + батарея)
```

### Прочее

| Метод | Описание |
|-------|----------|
| `blinkLED(interval)` | Мигать светодиодом с интервалом мс (0 = выключить) |
| `getBatteryPower()` | Заряд батареи (0–100%) |
| `UniBaseControl()` | Включить приём команд от Arduino Nano по UART |

---

## UniDev — внешние модули

```cpp
UniDev module;
```

### Порты

| Константа | GPIO | Назначение по умолчанию |
|-----------|------|-------------------------|
| `P1` | 12 | Светофор: красный |
| `P2` | 13 | Светофор: жёлтый / датчик линии |
| `P3` | 14 | Светофор: зелёный / trig бокового ультразвука |
| `P4` | 15 | Echo бокового ультразвука |
| `P5` | 17 | Кнопка |
| `P6` | 16 | Trig переднего ультразвука |
| `P7` | 32 | Echo переднего ультразвука |
| `P8` | 23 | NeoPixel кольцо (24 LED) |

### Датчики

| Метод | Возвращает | Описание |
|-------|-----------|----------|
| `ultraSonic(trig, echo)` | мм | Расстояние ультразвуком |
| `lineSensor(port)` | 0–4095 | Аналоговое значение датчика линии |
| `analogSensor(port)` | 0–4095 | Аналоговый вход |
| `digitalSensor(port)` | 0 / 1 | Цифровой вход |
| `getPinMode(pin)` | режим | Текущий режим пина |

```cpp
int dist = module.ultraSonic(P6, P7);   // передний ультразвук
int line = module.lineSensor(P2);
```

### Кнопка

```cpp
module.waitButton(P5);              // ждать нажатия
bool pressed = module.getButtonState(P5);
```

### Сервопривод

```cpp
module.servo(P1, 90);   // порт, угол 0–180°
```

### NeoPixel (24 LED, порт P8)

**Базовые:**

| Метод | Описание |
|-------|----------|
| `pixel(index, r, g, b)` | Один пиксель (0–23) |
| `pixelsAll(r, g, b)` | Все пиксели |
| `pixelsClear()` | Погасить все |
| `pixelsShow()` | Применить изменения |
| `pixelsBrightness(0–100)` | Яркость |

**Эффекты (блокирующие):**

| Метод | Параметры | Описание |
|-------|-----------|----------|
| `pixelsRainbow(speed, duration)` | скорость 0–100, мс | Радуга |
| `pixelsRunning(r,g,b, duration)` | RGB, мс | Бегущий огонь |
| `pixelsBreathing(r,g,b, duration)` | RGB, мс | Дыхание |
| `pixelsFill(r,g,b, duration)` | RGB, мс | Заполнение кольца |
| `pixelsSparkle(r,g,b, duration, count)` | RGB, мс, число искр | Искры |
| `pixelsRotating(r,g,b, duration, segLen, rotations)` | RGB, мс, длина сегмента, обороты | Вращающийся сегмент |
| `pixelsSpinner(r,g,b, duration)` | RGB, мс | Спиннер |

### Светофор (P1–P3)

```cpp
module.setTrafficLight(TRAFFIC_RED);
module.setTrafficLight(TRAFFIC_YELLOW);
module.setTrafficLight(TRAFFIC_GREEN);
module.setTrafficLight(TRAFFIC_OFF);
module.trafficLightSequence();  // красный → жёлтый → зелёный
```

---

## Примеры

Открыть в Arduino IDE: `Файл → Примеры → UNI`

| Пример | Описание |
|--------|----------|
| **Start** | Минимальный шаблон |
| **BasicMovement** | Движение по квадрату |
| **Arcs** | Дуги: по углу, по радиусу |
| **PrecisionTurns** | `rotateTo` — поворот к абсолютному курсу |
| **DriveToPoint** | `moveTo` — езда по точкам одометрии |
| **AsyncMovement** | Асинхронные команды, `isMoving`, `waitMove` |
| **ObstacleStop** | Остановка по ультразвуковому датчику |
| **ManualControl** | Управление через Serial |
| **Tuning** | Настройка `UniConfig` и `TuningConfig` |
| **LineSensor** | Чтение датчика линии |
| **DistanceSensors** | Ультразвуковые датчики |
| **UniBaseControl_Start** | Пример для связки ESP32 ↔ Arduino Nano |
