#include <UNI.h>

UniBase robot;


void setup() {
  robot.begin("TestBot");
}

void loop() {
  robot.UniBaseControl();
}
