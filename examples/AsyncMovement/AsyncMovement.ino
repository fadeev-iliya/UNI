/*
  AsyncMovement — асинхронные команды движения.

  Команды с суффиксом Async запускают движение и сразу возвращают
  управление: пока робот едет, программа может заниматься другим
  (мигать светом, опрашивать датчики, считать).

  isMoving()        — проверить, едет ли робот.
  waitMove()        — дождаться конца движения.
  waitMove(timeout) — ждать не дольше timeout мс; вернет false,
                      если робот не доехал (например, застрял).
*/

#include <UNI.h>

UniBase robot("UNI");

void setup() {
  robot.begin();
  robot.blinkLED(0);

  // 1. Делаем полезную работу, пока робот едет
  robot.moveDistAsync(50, 800);
  while (robot.isMoving()) {
    robot.displayPrint("Dist", robot.getDistance()); // живой прогресс на экране
    delay(100);
  }
  robot.displayClear();
  delay(500);

  // 2. Запустить и просто дождаться
  robot.rotateAsync(50, 180);
  robot.blinkLED(100);   // мигаем во время поворота
  robot.waitMove();
  robot.blinkLED(0);
  delay(500);

  // 3. Ожидание с таймаутом - страховка от застревания
  robot.moveDistAsync(50, 800);
  if (!robot.waitMove(5000)) {  // не доехал за 5 секунд?
    robot.stop(HARD);
    robot.displayPrint("STUCK!");
    return;
  }
  delay(500);

  // 4. Прервать движение по собственному условию
  robot.moveDistAsync(30, 2000);     // далекая цель...
  delay(1500);                       // ...но через 1.5 секунды
  robot.stop(HARD);                  // передумали
  robot.displayPrint("Async", "DONE");
}

void loop() {
}
