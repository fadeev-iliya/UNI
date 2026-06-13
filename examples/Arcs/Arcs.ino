/*
  Arcs — дуги: moveArcDist и moveArcRadius.

  moveArcDist(power, angle, mm) — дуга через соотношение скоростей
  колес (angle от -90 до 90), дистанция по центру робота.

  moveArcRadius(power, radiusMM, angleDeg) — дуга с настоящей
  геометрией: радиус по центру робота и угол сектора. Знак угла
  задает сторону поворота. Удобно планировать траектории.
*/

#include <UNI.h>

UniBase robot("UNI");

void setup() {
  robot.begin();

  // Квадрат со скругленными углами:
  // прямые участки + четверти круга радиусом 100 мм
  robot.displayPrint("Round sq");
  delay(1000);
  for (int i = 0; i < 4; i++) {
    robot.moveDist(50, 250);
    robot.moveArcRadius(50, 100, 90); // четверть круга R = 100 мм
  }
  delay(1000);

  // Змейка через соотношение скоростей колес
  robot.displayPrint("Slalom");
  delay(1000);
  for (int i = 0; i < 3; i++) {
    robot.moveArcDist(50, 30, 250);   // дуга вправо
    robot.moveArcDist(50, -30, 250);  // дуга влево
  }

  robot.displayPrint("Arcs", "DONE");
}

void loop() {
}
