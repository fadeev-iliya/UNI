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
