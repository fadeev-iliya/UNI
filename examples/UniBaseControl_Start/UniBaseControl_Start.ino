#include <UNI.h>

UniBase robot;

void setup() {
  robot.begin("UNI");
}

void loop() {
  robot.UniBaseControl();
}
