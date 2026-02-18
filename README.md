# UNI Library

Библиотека для управления роботом UniBase

## Возможности
- **Движение**: Управление моторами, энкодеры, PID-регуляторы.
- **Одометрия**: Отслеживание координат X, Y и угла поворота.
- **Дисплей**: Вывод текста и данных на OLED дисплей.
- **UART Control**: Режим внешнего управления (например, с Arduino Nano).

## Установка
1. Скопируйте папку `UNI` в директорию `libraries` вашего Arduino IDE.
2. Перезапустите Arduino IDE.

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

## Режим внешнего управления (UART Control)
Для управления роботом с внешнего контроллера (например, Arduino Nano) используйте пример `UniBaseControl_Start`.

```cpp
#include <UNI.h>

UniBase robot;

void setup() {
  robot.begin("UNI");
}

void loop() {
  robot.UniBaseControl();
}

```
В этом режиме UniBase слушает команды на портах UART2 (RX=16, TX=17) и выполняет их.

## Примеры
Примеры использования находятся в меню `Файл -> Примеры -> UNI`.
- **Start**: Базовый пример.
- **UniBaseControl_Start**: Запуск режима внешнего управления.
- **LineSensor**: Считывание значения с датчика линии.
- **DistanceSensor**: Считывание значений с датчиков расстояния.