#include <UNI.h>

UniBase robot;
UniDev module;

const int THRESHOLD = 2000; // порог светлое/темное - подберите под свой пол

void setup() {
  robot.begin("Line");
}

void loop() {
  int value = module.lineSensor(P2);
  bool onLine = (value > THRESHOLD);

  Serial.print("Line sensor: ");
  Serial.print(value);
  Serial.println(onLine ? "  [LINE]" : "");

  robot.displayPrint(onLine ? "LINE" : "floor", value);

  delay(100);
}
