#include <UNI.h>

UniBase robot;
UniDev module;

void setup() {
  robot.begin();
}

void loop() {
  // Считываем значение с датчика линии на пине P2
  int sensorValue = module.lineSensor(P2);
  
  // Выводим значение в монитор порта
  Serial.print("Line sensor: ");
  Serial.println(sensorValue);
}

