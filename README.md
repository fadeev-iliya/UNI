# UNI Library

Библиотека для управления роботом UniBase

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
- **Start**: Базовый пример.
- **LineSensor**: Считывание значения с датчика линии.
- **DistanceSensor**: Считывание значений с датчиков расстояния.
