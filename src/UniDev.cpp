#include "UniDev.h"

bool UniDev::_initialized = false;
Adafruit_NeoPixel* UniDev::_staticNeoPixels = nullptr;
uint8_t UniDev::_pinModes[40] = {0};
int8_t UniDev::_servoChannels[40] = {-1}; // Initializes first element, others 0
uint8_t UniDev::_nextFreeChannel = 4; // 0-3 reserved for UniMotors

UniDev::UniDev() {}

void UniDev::staticInit() {
  if (_initialized) return;
  
  #ifdef DEBUG_MODE
  Serial.println("UniDev static initialization");
  #endif
  
  for (int i = 0; i < 40; i++) {
    _servoChannels[i] = -1;
  }
  
  _staticNeoPixels = new Adafruit_NeoPixel(NEOPIXEL_COUNT, P8, NEO_GRB + NEO_KHZ800);
  _staticNeoPixels->begin();
  _staticNeoPixels->clear();
  _staticNeoPixels->show();
  _staticNeoPixels->setBrightness(100);
  
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

// На ESP32 АЦП имеют только пины ADC1 (32-39) и ADC2 (0,2,4,12-15,25-27).
// GPIO 16/17 (порты P5/P6) к АЦП не подключены - analogRead вернет мусор.
static bool isAdcCapable(int pin) {
  if (pin >= 32 && pin <= 39) return true; // ADC1
  switch (pin) {                            // ADC2
    case 0: case 2: case 4:
    case 12: case 13: case 14: case 15:
    case 25: case 26: case 27:
      return true;
    default:
      return false;
  }
}

static void warnNoAdc(int port) {
  Serial.print(F("[UniDev] WARNING: GPIO "));
  Serial.print(port);
  Serial.println(F(" has no ADC - analog read will be invalid. Use P1-P4 or P7."));
}

int UniDev::lineSensor(int port) {
  ensureInitialized();
  ensurePinMode(port, INPUT);

  if (!isAdcCapable(port)) warnNoAdc(port);
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

  if (!isAdcCapable(port)) warnNoAdc(port);
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

void UniDev::servoAttach(int port) {
  ensureInitialized();
  if (port < 0 || port >= 40) return;
  ensurePinMode(port, OUTPUT);

  // Закрепляем за портом LEDC-канал один раз. servoDetach канал не
  // освобождает, поэтому циклы attach/detach каналы не расходуют
  if (_servoChannels[port] == -1) {
    if (_nextFreeChannel > 15) return; // ESP32 has max 16 channels (0-15)
    _servoChannels[port] = _nextFreeChannel++;
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(port, 50, 14); // 50 Hz, 14-bit resolution
#else
    ledcSetup(_servoChannels[port], 50, 14); // 50 Hz, 14-bit resolution
    ledcAttachPin(port, _servoChannels[port]);
#endif
  }
}

void UniDev::servo(int port, int angle) {
  servoAttach(port);
  if (port < 0 || port >= 40 || _servoChannels[port] == -1) return;

  angle = constrain(angle, 0, 180);

  // 14-bit resolution means values 0-16383.
  // 50 Hz = 20 ms period.
  // Servo pulses are 0.5 ms to 2.5 ms.
  // 0.5 ms = 0.5 / 20 * 16384 = 410
  // 2.5 ms = 2.5 / 20 * 16384 = 2048
  uint32_t duty = map(angle, 0, 180, 410, 2048);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(port, duty);
#else
  ledcWrite(_servoChannels[port], duty);
#endif
}

void UniDev::servoDetach(int port) {
  ensureInitialized();
  if (port < 0 || port >= 40) return;
  if (_servoChannels[port] == -1) return; // не был подключен

  // Нулевая скважность = нет импульсов: привод расслабляется и не держит
  // нагрузку. Канал остается закрепленным - servo() мгновенно оживит его
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(port, 0);
#else
  ledcWrite(_servoChannels[port], 0);
#endif
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

