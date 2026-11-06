# UNI Library

Библиотека для управления роботом UniBase

## Документация
 -> [Перейти к документации](https://fadeev-iliya.github.io/UNI)

## Возможности
- **Движение**: Управление моторами, энкодеры, PID-регуляторы.
- **Одометрия**: Отслеживание координат X, Y и угла поворота.
- **Дисплей**: Вывод текста и данных на OLED дисплей.
- **UniDev**: Управление модулями.

## Использование
Подключите библиотеку и создайте экземпляр класса `UniBase`:

```cpp
#include <UNI.h>

UniBase robot;
UniDev module;

void setup() {
  robot.begin("UNI");
}

void loop() {
  
}

```
## Версии платы
Библиотека работает с двумя версиями платы: `UNI_BOARD_V1` и `UNI_BOARD_V2` (плата с EEPROM и pogo-портами). Версия определяется автоматически: на плате V2 на шине I2C отвечает EEPROM (адрес `0x50`), поэтому один и тот же скетч работает на обеих платах.

Версию можно задать и вручную:

```cpp
UniBase robot("UNI", UNI_BOARD_V2);   // в конструкторе

void setup() {
  robot.begin("UNI", UNI_BOARD_V2);   // или при инициализации
}
```

Порты `P1`–`P8` подставляют GPIO той платы, на которой запущена программа. Узнать версию в скетче: `UniBoard::name()` (вернет "V1" или "V2").

## Работа с модулями (UniDev)
Для работы с дополнительными модулями используется класс `UniDev`. Пример считывания данных с датчика линии:

```cpp
#include <UNI.h>

UniBase robot;
UniDev module;

void setup() {
  robot.begin();
}

void loop() {
  int sensorValue = module.lineSensor(P2);
  
  Serial.print("Line sensor: ");
  Serial.println(sensorValue);
}
```


## Примеры
Примеры использования находятся в меню `Файл -> Примеры -> UNI`.
- **Start**: Минимальная программа: создать робота и начать писать свой код.
- **ManualControl**: Прямое управление моторами: motors, motorsSync, motorsArc, holdPosition.
- **Arcs**: Дуги: moveArcDist и moveArcRadius.
- **PrecisionTurns**: Точные повороты: rotate и rotateTo.
- **DriveToPoint**: Езда по точкам одометрии: setPosition и moveTo.
- **AsyncMovement**: Асинхронные команды движения: isMoving и waitMove.
- **ObstacleStop**: Асинхронное движение с остановкой перед препятствием.
- **LineSensor**: Считывание значения с датчика линии.
- **DistanceSensors**: Считывание значений с ультразвуковых датчиков расстояния.
- **ServoControl**: Сервопривод: углы, подключение и отключение.
- **UniBaseControl_Start**: Прошивка платформы для управления с Arduino Nano по UART.
