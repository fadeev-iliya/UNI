/*
  DriveToPoint — езда по точкам одометрии.

  setPosition(x, y, angle) привязывает одометрию к полю
  (например, к стартовой клетке), а moveTo(power, x, y)
  сам доворачивает на курс к цели и едет к ней по прямой.

  Маршрут задается простым списком точек.
*/

#include <UNI.h>

UniBase robot("UNI");

// Маршрут: массив точек {x, y} в миллиметрах
const float route[][2] = {
  {400, 0},
  {400, 400},
  {0, 400},
  {0, 0},     // возврат на старт
};
const int routeLen = sizeof(route) / sizeof(route[0]);

void setup() {
  robot.begin();

  // Робот стоит в начале координат и смотрит вдоль оси X
  robot.setPosition(0, 0, 0);

  for (int i = 0; i < routeLen; i++) {
    robot.displayPrint("Point", i + 1);
    robot.moveTo(50, route[i][0], route[i][1]);
    delay(300);
  }

  robot.rotateTo(50, 0); // в конце вернуть исходный курс
  robot.displayPrint("Route", "DONE");
}

void loop() {
  robot.printOdometry(); // следим за позицией в Serial Monitor
  delay(1000);
}
