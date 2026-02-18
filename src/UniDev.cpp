#include "UniDev.h"

bool UniDev::_initialized = false;
Adafruit_NeoPixel* UniDev::_staticNeoPixels = nullptr;
uint8_t UniDev::_pinModes[40] = {0};

UniDev::UniDev() {}

void UniDev::staticInit() {
  if (_initialized) return;
  
  #ifdef DEBUG_MODE
  Serial.println("UniDev static initialization");
  #endif
  
  _staticNeoPixels = new Adafruit_NeoPixel(NEOPIXEL_COUNT, P8, NEO_GRB + NEO_KHZ800);
  _staticNeoPixels->begin();
  _staticNeoPixels->clear();
  _staticNeoPixels->show();
  _staticNeoPixels->setBrightness(100);
  
  pinMode(P3, OUTPUT);
  _pinModes[P3] = OUTPUT;
  pinMode(P4, INPUT);
  _pinModes[P4] = INPUT;
  pinMode(P6, OUTPUT);
  _pinModes[P6] = OUTPUT;
  pinMode(P7, INPUT);
  _pinModes[P7] = INPUT;
  digitalWrite(P3, LOW);
  digitalWrite(P6, LOW);
  
  pinMode(P1, OUTPUT);
  _pinModes[P1] = OUTPUT;
  pinMode(P2, OUTPUT);
  _pinModes[P2] = OUTPUT;
  digitalWrite(P1, LOW);
  digitalWrite(P2, LOW);
  digitalWrite(P3, LOW);
  
  _initialized = true;
  
  #ifdef DEBUG_MODE
  Serial.println("UniDev initialized");
  #endif
}

void UniDev::begin() {
  staticInit();
}

void UniDev::ensureInitialized() {
  if (!_initialized) {
    staticInit();
  }
}

void UniDev::ensurePinMode(uint8_t pin, uint8_t mode) {
  if (pin >= 40) return;
  
  if (_pinModes[pin] != mode) {
    pinMode(pin, mode);
    _pinModes[pin] = mode;
  }
}

// ============ Sensors ============

int UniDev::ultraSonic(int trig, int echo) {
  ensureInitialized();
  ensurePinMode(trig, OUTPUT);
  ensurePinMode(echo, INPUT);
  
  delayMicroseconds(100);
  
  for (int i = 0; i < 3; i++) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    
    long duration = pulseIn(echo, HIGH, 23529);
    
    if (duration > 0) {
      int distance = (duration * 0.34) / 2;
      
      if (distance >= 20 && distance <= 4000) {
        return distance; 
      }
    }
    
    if (i < 2) {
      delayMicroseconds(100);
    }
  }
  
  return 0;
}

int UniDev::lineSensor(int port) {
  ensureInitialized();
  ensurePinMode(port, INPUT);
  
  return analogRead(port);
}

int UniDev::digitalSensor(int port) {
  ensureInitialized();
  ensurePinMode(port, INPUT);

  return digitalRead(port);
}

int UniDev::analogSensor(int port) {
  ensureInitialized();
  ensurePinMode(port, INPUT);
  
  return analogRead(port);
}

int UniDev::getPinMode(uint8_t pin) {
  if (pin >= 40) return -1;
  
  return _pinModes[pin];
}

// ============ Button ============

void UniDev::waitButton(int port) {
  ensureInitialized();
  ensurePinMode(port, INPUT_PULLUP);
  
  #ifdef DEBUG_MODE
  Serial.println("Waiting for button press...");
  #endif
  
  while (digitalRead(port)) {
    delay(10);
  }
  
  while (!digitalRead(port)) {
    delay(10);
  }
  
  #ifdef DEBUG_MODE
  Serial.println("Button pressed!");
  #endif
}

bool UniDev::getButtonState(int port) {
  ensureInitialized();
  ensurePinMode(port, INPUT_PULLUP);
  return !digitalRead(port);
}

// ============ Servo ============

void UniDev::servo(int port, int angle) {
  ensureInitialized();
  ensurePinMode(port, OUTPUT);
  
  angle = constrain(angle, 0, 180);
  // Не сделано 
}

// ============ NeoPixel ============

void UniDev::pixel(int index, int r, int g, int b) {
  ensureInitialized();
  if (_staticNeoPixels && index >= 0 && index < NEOPIXEL_COUNT) {
    _staticNeoPixels->setPixelColor(index, _staticNeoPixels->Color(r, g, b));
    _staticNeoPixels->show();
  }
}

void UniDev::pixelsAll(int r, int g, int b) {
  ensureInitialized();
  if (_staticNeoPixels) {
    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
      _staticNeoPixels->setPixelColor(i, _staticNeoPixels->Color(r, g, b));
    }
    _staticNeoPixels->show();
  }
}

void UniDev::pixelsClear() {
  ensureInitialized();
  if (_staticNeoPixels) {
    _staticNeoPixels->clear();
    _staticNeoPixels->show();
  }
}

void UniDev::pixelsShow() {
  ensureInitialized();
  if (_staticNeoPixels) {
    _staticNeoPixels->show();
  }
}

void UniDev::pixelsBrightness(int brightness) {
  ensureInitialized();
  if (_staticNeoPixels) {
    _staticNeoPixels->setBrightness(brightness);
    _staticNeoPixels->show();
  }
}

// ============ NeoPixel Effects ============

void UniDev::pixelsRainbow(int speed, int duration) {
  ensureInitialized();
  if (!_staticNeoPixels) return;
  
  speed = constrain(speed, 0, 100);
  int delayTime = 50 - (speed * 49 / 100);
  
  unsigned long startTime = millis();
  long firstPixelHue = 0;
  
  while (millis() - startTime < duration) {
    for(int i = 0; i < NEOPIXEL_COUNT; i++) {
      int pixelHue = firstPixelHue + (i * 65536L / NEOPIXEL_COUNT);
      _staticNeoPixels->setPixelColor(i, _staticNeoPixels->gamma32(_staticNeoPixels->ColorHSV(pixelHue)));
    }
    _staticNeoPixels->show();
    delay(delayTime);
    
    firstPixelHue += 256;
    if (firstPixelHue >= 5 * 65536) firstPixelHue = 0;
  }
}

void UniDev::pixelsRunning(int r, int g, int b, int duration) {
  ensureInitialized();
  if (!_staticNeoPixels) return;
  
  int delayTime = duration / NEOPIXEL_COUNT;
  
  for(int i = 0; i < NEOPIXEL_COUNT; i++) {
    _staticNeoPixels->clear();
    _staticNeoPixels->setPixelColor(i, _staticNeoPixels->Color(r, g, b));
    _staticNeoPixels->show();
    delay(delayTime);
  }
}

void UniDev::pixelsBreathing(int r, int g, int b, int duration) {
  ensureInitialized();
  if (!_staticNeoPixels) return;
  
  int totalSteps = 104;
  int delayTime = duration / totalSteps;
  
  for(int brightness = 0; brightness <= 255; brightness += 5) {
    for(int i = 0; i < NEOPIXEL_COUNT; i++) {
      _staticNeoPixels->setPixelColor(i, _staticNeoPixels->Color(
        (r * brightness) / 255,
        (g * brightness) / 255,
        (b * brightness) / 255
      ));
    }
    _staticNeoPixels->show();
    delay(delayTime);
  }
  
  for(int brightness = 255; brightness >= 0; brightness -= 5) {
    for(int i = 0; i < NEOPIXEL_COUNT; i++) {
      _staticNeoPixels->setPixelColor(i, _staticNeoPixels->Color(
        (r * brightness) / 255,
        (g * brightness) / 255,
        (b * brightness) / 255
      ));
    }
    _staticNeoPixels->show();
    delay(delayTime);
  }
}

void UniDev::pixelsFill(int r, int g, int b, int duration) {
  ensureInitialized();
  if (!_staticNeoPixels) return;
  
  _staticNeoPixels->clear();
  
  int totalSteps = NEOPIXEL_COUNT * 2;
  int delayTime = duration / totalSteps;
  
  for(int i = 0; i < NEOPIXEL_COUNT; i++) {
    _staticNeoPixels->setPixelColor(i, _staticNeoPixels->Color(r, g, b));
    _staticNeoPixels->show();
    delay(delayTime);
  }
  
  for(int i = 0; i < NEOPIXEL_COUNT; i++) {
    _staticNeoPixels->setPixelColor(i, _staticNeoPixels->Color(0, 0, 0));
    _staticNeoPixels->show();
    delay(delayTime);
  }
}

void UniDev::pixelsSparkle(int r, int g, int b, int duration, int count) {
  ensureInitialized();
  if (!_staticNeoPixels) return;
  
  _staticNeoPixels->clear();
  
  int delayTime = duration / count;
  
  for(int j = 0; j < count; j++) {
    int randomPixel = random(NEOPIXEL_COUNT);
    _staticNeoPixels->setPixelColor(randomPixel, _staticNeoPixels->Color(r, g, b));
    _staticNeoPixels->show();
    delay(delayTime);
    _staticNeoPixels->setPixelColor(randomPixel, _staticNeoPixels->Color(0, 0, 0));
  }
  
  _staticNeoPixels->show();
}

void UniDev::pixelsRotating(int r, int g, int b, int duration, int segmentLength, int rotations) {
  ensureInitialized();
  if (!_staticNeoPixels) return;
  
  int totalSteps = rotations * NEOPIXEL_COUNT;
  int delayTime = duration / totalSteps;
  
  for(int rot = 0; rot < rotations; rot++) {
    for(int startPos = 0; startPos < NEOPIXEL_COUNT; startPos++) {
      _staticNeoPixels->clear();
      
      for(int i = 0; i < segmentLength; i++) {
        int pixelPos = (startPos + i) % NEOPIXEL_COUNT;
        _staticNeoPixels->setPixelColor(pixelPos, _staticNeoPixels->Color(r, g, b));
      }
      
      _staticNeoPixels->show();
      delay(delayTime);
    }
  }
}

void UniDev::pixelsSpinner(int r, int g, int b, int duration) {
  ensureInitialized();
  if (!_staticNeoPixels) return;
  
  int delayTime = duration / NEOPIXEL_COUNT;
  
  for(int i = 0; i < NEOPIXEL_COUNT; i++) {
    _staticNeoPixels->clear();
    _staticNeoPixels->setPixelColor(i, _staticNeoPixels->Color(r, g, b));
    _staticNeoPixels->show();
    delay(delayTime);
  }
}

// ============ Traffic Light ============

void UniDev::setTrafficLight(TrafficLightColor color) {
  ensureInitialized();
  ensurePinMode(P1, OUTPUT);
  ensurePinMode(P2, OUTPUT);
  ensurePinMode(P3, OUTPUT);
  
  digitalWrite(P1, LOW);
  digitalWrite(P2, LOW);
  digitalWrite(P3, LOW);
  
  switch (color) {
    case TRAFFIC_RED:
      digitalWrite(P1, HIGH);
      break;
    case TRAFFIC_YELLOW:
      digitalWrite(P2, HIGH);
      break;
    case TRAFFIC_GREEN:
      digitalWrite(P3, HIGH);
      break;
    case TRAFFIC_OFF:
    default:
      break;
  }
}

void UniDev::trafficLightSequence() {
  ensureInitialized();
  
  setTrafficLight(TRAFFIC_RED);
  delay(1000);
  setTrafficLight(TRAFFIC_YELLOW);
  delay(1000);
  setTrafficLight(TRAFFIC_GREEN);
  delay(1000);
  setTrafficLight(TRAFFIC_OFF);
}

