////////////////////////
// DEFINE THESE CONSTANTS////
////////////////////////
#define NUM_STRIPS 4       
#define SERIAL9600
#define NEO_PIN 7
#define NUM_NEOPIXELS 144

int checkPin = 2;
float pressThreshold[NUM_STRIPS] = {13, 13, 13, 13}; //, 10, 17.5, 10, 12};
float releaseThreshold[NUM_STRIPS] = {3.6, 3.6, 3.6, 3.6};//, 3.6, 3.6, 3.6, 3.6};
float movingAverageFactor = 1;
const int outputK = 13;

/////////////////////////
// MAKEY MAKEY STRUCT ///
/////////////////////////
typedef struct {
  byte pinNumber;
  int keyCode;
  int timePressed;
  
  float pressThreshold;
  float releaseThreshold;
  
  float movingAverage;
  boolean pressed;
  boolean prevPressed;
} 
MakeyMakeyInput;
MakeyMakeyInput inputs[NUM_STRIPS];

int pinNumbers[NUM_STRIPS] = { 
  2,    
  3,    
  4,    
  5      
  //6,     
  //7,     
  //8,     
  //9     
};


/////////////////////////
// NEOPIXELS ////////////
/////////////////////////
// http://learn.adafruit.com/adafruit-neopixel-uberguide/neomatrix-library
// set the Neopixel pin to 0 - D0
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_NEOPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

///////////////////////////
// FUNCTIONS //////////////
///////////////////////////
void initializeArduino();
void initializeInputs();
void updateMovingAverage();
void updateInputStates();
void updateOutLED();

//////////////////////
// SETUP /////////////
//////////////////////
void setup() 
{
  strip.begin();
  initializeArduino();
  initializeInputs();
  delay(100);
  
}

////////////////////
// MAIN LOOP ///////
////////////////////
void loop() 
{ 
  updateMovingAverage();
  updateInputStates();
  updateOutLED();
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
  for (int i=0; i<NUM_STRIPS; i++)
  {
    pinMode(pinNumbers[i], INPUT);
    digitalWrite(pinNumbers[i], LOW);
  }
  
  pinMode(outputK, OUTPUT);
  digitalWrite(outputK, LOW);
}

///////////////////////////
// INITIALIZE INPUTS
///////////////////////////
void initializeInputs() {

  for (int i=0; i<NUM_STRIPS; i++) {
    inputs[i].pinNumber = pinNumbers[i];
    inputs[i].movingAverage = 0;
    
    inputs[i].pressThreshold = pressThreshold[i];
    inputs[i].releaseThreshold = releaseThreshold[i];
    
    inputs[i].pressed = false;
    inputs[i].prevPressed = false;

#ifdef DEBUG
    Serial.println(i);
#endif

  }
}


///////////////////////////
// UPDATE INPUT STATES
///////////////////////////
void updateInputStates() {
  for (int i=0; i<NUM_STRIPS; i++) {
      if (inputs[i].movingAverage > inputs[i].pressThreshold) {  // input becomes pressed
        inputs[i].pressed = true;
        if(i == checkPin) {
          Serial.print(checkPin);
          Serial.print(" is on and is ");
          Serial.println(inputs[i].movingAverage);
        }
      }
      else if (inputs[i].movingAverage < inputs[i].releaseThreshold) {
        inputs[i].pressed = false;
        if(i == checkPin) {
          Serial.print(checkPin);
          Serial.print(" is off and is ");
          Serial.println(inputs[i].movingAverage);
        }
      }
  }
}


void setColor(int R, int G, int B) {
  for(int i = 0; i < NUM_NEOPIXELS; i++) {
    strip.setPixelColor(i, strip.Color(R, G, B));
  }
  strip.show();
  delay(50);
}

///////////////////////////
// UPDATE OUT LED
///////////////////////////
void updateOutLED() {
  boolean keyPressed = 0;
  for (int i=0; i<NUM_STRIPS; i++) {
    if (inputs[i].pressed) {
        keyPressed = i+1;
        
#ifdef DEBUG
        Serial.print("Key ");
        Serial.print(i);
        Serial.println(" pressed");
#endif
    }
  }

  if (keyPressed == 1) {
    digitalWrite(outputK, HIGH);
    setColor(0,155,100);
  }
  else if (keyPressed == 2) {
    digitalWrite(outputK, HIGH);
    setColor(155,0,100);
  }
  else if (keyPressed == 3) {
    digitalWrite(outputK, HIGH);
    setColor(55,70,200);
  }
  else if (keyPressed == 4) {
    digitalWrite(outputK, HIGH);
    setColor(255,100,100);
  }
  else {       
    digitalWrite(outputK, LOW);
    setColor(0,0,0);
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
  for(int i = 0; i < NUM_STRIPS; i++) {
    int cycles = readCapacitivePin(pinNumbers[i]);
    int mave = inputs[i].movingAverage;
    inputs[i].movingAverage = mave * (1.0 - movingAverageFactor) + cycles * movingAverageFactor;
  }
}

