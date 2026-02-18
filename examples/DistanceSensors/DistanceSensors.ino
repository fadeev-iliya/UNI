#include <UNI.h>

UniBase robot;
UniDev module;

void setup() {
  Serial.begin(115200);
  robot.begin("Dist");
}

void loop() {
  // Получаем значения с датчиков
  int front = module.ultraSonic(P6, P7);
  int side = module.ultraSonic(P3, P4);
  
  // Выводим в Serial Monitor
  Serial.print("Front: ");
  Serial.print(front);
  Serial.print(" mm, Side: ");
  Serial.print(side);
  Serial.println(" mm");
} 