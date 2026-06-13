/*
  PrecisionTurns — точные повороты: rotate против rotateTo.

  rotate(power, angle)   — поворот НА угол относительно текущего.
                           Ошибка каждого поворота добавляется к следующему.
  rotateTo(power, angle) — поворот К абсолютному курсу одометрии.
                           Ошибка предыдущих маневров съедается.

  Робот проезжает два квадрата. После первого (rotate) сравните
  курс на экране с нулем; после второго (rotateTo) он вернется
  к нулю точнее: ошибки не накапливались.
*/

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
