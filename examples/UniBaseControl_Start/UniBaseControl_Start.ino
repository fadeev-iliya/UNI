#include <UNI.h>

UniBase robot;

void setup() {
  robot.begin("UNI");
  robot.UniBaseControl(); // включаем прослушивание команд
}

void loop() {
  // Прием команд, движение и экран работают на втором ядре.
  // loop() свободен - сюда можно добавить свою логику.
}
