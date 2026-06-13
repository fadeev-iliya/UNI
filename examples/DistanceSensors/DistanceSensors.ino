/*
  DistanceSensors — чтение ультразвуковых датчиков расстояния.

  ultraSonic(trigPin, echoPin) возвращает расстояние в миллиметрах
  (0 - если эхо не пришло). Датчиков может быть несколько, каждый
  на своей паре портов.

  Подключение: передний датчик на P6 (trig) и P7 (echo),
  боковой на P3 (trig) и P4 (echo).

  Откройте Serial Monitor (115200 бод) и поднесите руку к датчикам.
*/

#include <UNI.h>

UniBase robot;
UniDev module;

void setup() {
  robot.begin("Dist"); // заодно запускает Serial на 115200
}

void loop() {
  int front = module.ultraSonic(P6, P7);
  int side = module.ultraSonic(P3, P4);

  Serial.print("Front: ");
  Serial.print(front);
  Serial.print(" mm, Side: ");
  Serial.print(side);
  Serial.println(" mm");

  // Ближайшее расстояние удобно видеть и на экране робота
  robot.displayPrint("Front", front);

  delay(100); // ультразвуку нужна пауза между измерениями
}
