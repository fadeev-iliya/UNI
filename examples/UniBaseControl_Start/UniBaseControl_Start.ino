#include <UNI.h>

// Create UniBase instance
UniBase robot("UNI");

void setup() {
  // Initialize robot components
  robot.begin();
  
  // Optional: Display message or blink LED to indicate readiness
  robot.displayPrint("UART CTRL", "READY");
  delay(1000);
}

void loop() {
  // Activate UART Control Mode
  // This function listens to UART2 (Pins 16/17) and executes commands from the controller (e.g. Nano)
  // It handles all motor control, movement, and sensors automatically.
  robot.UniBaseControl();
}
