/*
  ObstacleStop — асинхронное движение с реакцией на препятствие.

  Робот едет вперед и постоянно следит за ультразвуковым датчиком.
  Препятствие ближе 150 мм — остановка и ожидание; препятствие
  убрали — продолжает движение к цели.

  Подключение: ультразвуковой датчик на портах P6 (trig) и P7 (echo).
*/

#include <UNI.h>

UniBase robot("UNI");
UniDev module;

const int OBSTACLE_MM = 150;   // дистанция остановки
const int TARGET_MM   = 1500;  // сколько всего проехать

void setup() {
  robot.begin();
  robot.resetDistance();
}

void loop() {
  // Цель достигнута - стоим
  if (robot.getDistance() >= TARGET_MM) {
    robot.displayPrint("DONE");
    return;
  }

  int dist = module.ultraSonic(P6, P7);
  bool blocked = (dist > 0 && dist < OBSTACLE_MM);

  if (blocked) {
    if (robot.isMoving()) {
      robot.stop(HARD);
      robot.displayPrint("Obstacle", dist);
    }
  } else {
    if (!robot.isMoving()) {
      // Едем остаток пути асинхронно, продолжая опрашивать датчик
      int remaining = TARGET_MM - (int)robot.getDistance();
      robot.moveDistAsync(50, remaining);
      robot.displayClear();
    }
  }

  delay(50);
}
