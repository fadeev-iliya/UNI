#include <UNI.h>

UniBase robot("UNI");

void setup() {
  robot.begin();

  // Квадрат на относительных поворотах
  robot.displayPrint("rotate");
  delay(1000);
  for (int i = 0; i < 4; i++) {
    robot.moveDist(50, 300);
    robot.rotate(50, 90);
  }
  robot.displayPrint("Angle", robot.getAngle()); // накопленный курс
  delay(3000);

  // Квадрат на абсолютных поворотах
  robot.displayPrint("rotateTo");
  delay(1000);
  robot.setPosition(0, 0, 0); // обнуляем одометрию перед заездом
  for (int i = 0; i < 4; i++) {
    robot.moveDist(50, 300);
    robot.rotateTo(50, (i + 1) * 90); // курсы 90, 180, 270, 360
  }
  robot.displayPrint("Angle", robot.getAngle());
}

void loop() {
}
