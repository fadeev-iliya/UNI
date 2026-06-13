/*
  LineSensor — чтение аналогового датчика линии.

  lineSensor(pin) возвращает значение 0..4095: над светлым полом
  значение одно, над черной линией - другое. Прокатите робота
  рукой над линией и подберите порог между ними.

  Подключение: датчик линии на порту P2.

  Откройте Serial Monitor (115200 бод). Значение также выводится
  на экран робота.
*/

#include <UNI.h>

UniBase robot;
UniDev module;

const int THRESHOLD = 2000; // порог светлое/темное - подберите под свой пол

void setup() {
  robot.begin("Line");
}

void loop() {
  int value = module.lineSensor(P2);
  bool onLine = (value > THRESHOLD);

  Serial.print("Line sensor: ");
  Serial.print(value);
  Serial.println(onLine ? "  [LINE]" : "");

  robot.displayPrint(onLine ? "LINE" : "floor", value);

  delay(100);
}
