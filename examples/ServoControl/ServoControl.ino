#include <UNI.h>

UniBase robot;
UniDev module;

void setup() {
  robot.begin("Servo");
}

void loop() {
  // Рабочий цикл: серво включено и держит углы
  robot.displayPrint("Servo", "ON");
  module.servo(P3, 0);
  delay(1000);
  module.servo(P3, 90);
  delay(1000);
  module.servo(P3, 180);
  delay(1000);

  // Отключаем: привод обмякает, механизм мягко опускается.
  // Попробуйте повернуть качалку рукой - сопротивления нет
  robot.displayPrint("Servo", "OFF");
  module.servoDetach(P3);
  delay(3000);

  // Следующий servo() в начале цикла снова включит привод
}
