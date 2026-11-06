#include <UNI.h>

UniBase robot("UNI");
UniDev module;

const int OBSTACLE_MM = 150;   // дистанция остановки
const int TARGET_MM   = 1500;  // сколько всего проехать
const int FINISH_MM   = 5;     // ближе этого к цели - доехали

void setup() {
  robot.begin();
  robot.resetDistance();
}

void loop() {
  // Цель достигнута - стоим. Движение завершается с небольшим остатком
  // до цели, поэтому сравниваем его с запасом, а не с нулем:
  // иначе робот бесконечно "доезжал" бы последний миллиметр
  int remaining = TARGET_MM - (int)robot.getDistance();
  if (remaining <= FINISH_MM) {
    robot.displayPrint("DONE");
    delay(500);
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
      robot.moveDistAsync(50, remaining);
      robot.displayClear();
    }
  }

  delay(50);
}
