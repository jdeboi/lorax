#define NUM_PADS 1       
#define CONDUCTIVE_PIN 2
#define trigPin 13
#define echoPin 12
#define NEO_PIN 7
#define NUM_NEOPIXELS 144

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_NEOPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

#define SERIAL9600

boolean pressed = false;
int movingAverage = 0;
float pressThreshold = 13;
float releaseThreshold = 3.6;
float movingAverageFactor = 1;

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  strip.begin();
  initializeArduino();
  delay(100);
}

void loop() {
  checkDistance();
  updateMovingAverage();
  updateInputState();
}


//////////////////////////
// INITIALIZE ARDUINO
//////////////////////////
void initializeArduino() {
  Serial.begin(9600);  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  /* Set up input pins 
   DEactivate the internal pull-ups, since we're using external resistors */
  pinMode(CONDUCTIVE_PIN, INPUT);
  digitalWrite(CONDUCTIVE_PIN, LOW);
}


///////////////////////////
// UPDATE INPUT STATES
///////////////////////////
void updateInputState() {
  if (movingAverage > pressThreshold) {  // input becomes pressed
    pressed = true;
    rainbowCycle(20);
    Serial.print("On: ");
    Serial.println(movingAverage);
  }
  
  else if (movingAverage < releaseThreshold) {
    pressed = false;
    Serial.print("off: ");
    Serial.println(movingAverage);
  }
}



void checkDistance() {
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  
  if (distance >= 200 || distance <= 0){
    Serial.println("Out of range");
    setColor(0,0,0);
  }
  else {
    Serial.print(distance);
    Serial.println(" cm");
    setColor(Wheel(map(distance, 0, 200, 0, 255)));
  }
  delay(100);
}

void setColor(int R, int G, int B) {
  for(int i = 0; i < NUM_NEOPIXELS; i++) {
    strip.setPixelColor(i, strip.Color(R, G, B));
  }
  strip.show();
  delay(50);
}

void setColor(uint32_t c) {
  for(int i = 0; i < NUM_NEOPIXELS; i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
  delay(50);
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


///////////////////////////
// CAPACITIVE SENSORS
///////////////////////////
uint8_t readCapacitivePin(int pinToMeasure) {
  // Variables used to translate from Arduino to AVR pin naming
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.
  byte bitmask;
  port = portOutputRegister(digitalPinToPort(pinToMeasure));
  ddr = portModeRegister(digitalPinToPort(pinToMeasure));
  bitmask = digitalPinToBitMask(pinToMeasure);
  pin = portInputRegister(digitalPinToPort(pinToMeasure));
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  delay(1);
  // Prevent the timer IRQ from disturbing our measurement
  noInterrupts();
  // Make the pin an input with the internal pull-up on
  *ddr &= ~(bitmask);
  *port |= bitmask;

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;
       if (*pin & bitmask) { cycles =  0;}
  else if (*pin & bitmask) { cycles =  1;}
  else if (*pin & bitmask) { cycles =  2;}
  else if (*pin & bitmask) { cycles =  3;}
  else if (*pin & bitmask) { cycles =  4;}
  else if (*pin & bitmask) { cycles =  5;}
  else if (*pin & bitmask) { cycles =  6;}
  else if (*pin & bitmask) { cycles =  7;}
  else if (*pin & bitmask) { cycles =  8;}
  else if (*pin & bitmask) { cycles =  9;}
  else if (*pin & bitmask) { cycles = 10;}
  else if (*pin & bitmask) { cycles = 11;}
  else if (*pin & bitmask) { cycles = 12;}
  else if (*pin & bitmask) { cycles = 13;}
  else if (*pin & bitmask) { cycles = 14;}
  else if (*pin & bitmask) { cycles = 15;}
  else if (*pin & bitmask) { cycles = 16;}

  // End of timing-critical section
  interrupts();

  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  *port &= ~(bitmask);
  *ddr  |= bitmask;

  return cycles;
}

void updateMovingAverage() {
  int cycles = readCapacitivePin(CONDUCTIVE_PIN);
  int mave = movingAverage;
  movingAverage = mave * (1.0 - movingAverageFactor) + cycles * movingAverageFactor;
}
