#include <UNI.h>

UniBase robot;
UniDev module;

void setup() {
  robot.begin("Dist"); // заодно запускает Serial на 115200
}

void loop() {
  int front = module.ultraSonic(P6, P7);
  int side = module.ultraSonic(P3, P4);

  Serial.print("Front: ");
  Serial.print(front);
  Serial.print(" mm, Side: ");
  Serial.print(side);
  Serial.println(" mm");

  // Ближайшее расстояние удобно видеть и на экране робота
  robot.displayPrint("Front", front);

  delay(100); // ультразвуку нужна пауза между измерениями
}
