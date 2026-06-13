/*
  ManualControl — прямое управление моторами.

  motors(l, r)     — подать мощность на моторы без стабилизации.
  motorsSync(l, r) — то же, но соотношение скоростей колес
                     удерживается по энкодерам: робот не уводит
                     в сторону, даже если одно колесо нагружено.
  motorsArc(p, a)  — езда по дуге с выравниванием (мощность + угол).
  holdPosition()   — активное удержание места: попробуйте сдвинуть
                     робота рукой, он будет сопротивляться.

  Все эти команды работают, пока не вызван stop().
*/

#include <UNI.h>

UniBase robot("UNI");

void setup() {
  robot.begin();

  // Прямо без стабилизации (может уводить в сторону)
  robot.displayPrint("motors");
  robot.motors(40, 40);
  delay(1500);
  robot.stop(HARD);
  delay(500);

  // Прямо со стабилизацией по энкодерам
  robot.displayPrint("motorsSync");
  robot.motorsSync(40, 40);
  delay(1500);

  // Дуга: правое колесо вдвое медленнее левого
  robot.motorsSync(50, 25);
  delay(1500);

  // Задний ход со стабилизацией
  robot.motorsSync(-40, -40);
  delay(1500);
  robot.stop(HARD);
  delay(500);

  // Дуга через мощность и угол
  robot.displayPrint("motorsArc");
  robot.motorsArc(40, 20);
  delay(2000);
  robot.stop(SOFT); // мягкая остановка без блокировки колес

  // Активное удержание позиции (10 секунд)
  robot.displayPrint("HOLD");
  robot.holdPosition();
  delay(10000);
  robot.stop(SOFT);
  robot.displayPrint("Done");
}

void loop() {
}
