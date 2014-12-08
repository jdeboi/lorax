/*
 ************************************************
 ****** SIMPLE CONDUCTIVE ARDUINO MONOME ********
 ************************************************
 MAKER
 Jenna deBoisblanc
 http://jdeboi.com
 start date: October 2014
 
 DESCRIPTION
 My objective for this project was to build a monome- http://monome.org/ -
 (basically a programmable array of backlit buttons that's used
 to compose electronic music or mix video) using tools and processes 
 that were so simple, a third grader could build it.
 
 This particular monome - currently setup as a 4x4 monome - can be made 
 with copper tape, jumpers, an Arduino Uno, and the monomeVisual 
 Processing sketch. It relies on capacitive sensors.
 
 CREDIT
 Special thanks to Amanda Ghassaei, one of my best friends, for introducing
 me to monomes and the Maker Movement.
 
 Code shout-outs:
 MaKey MaKey FIRMWARE v1.4.1
 by: Eric Rosenbaum, Jay Silver, and Jim Lindblom http://makeymakey.com
 Capacitive sensor code: http://playground.arduino.cc/Code/CapacitiveSensor
*/

/////////////////////////
// DEBUG DEFINITIONS ////               
/////////////////////////
//#define DEBUG
//#define DEBUG2 
//#define DEBUG3 
//#define DEBUG_TIMING
//#define DEBUG_MONOME

////////////////////////
// DEFINED CONSTANTS////
////////////////////////
#define NUM_STRIPS 8       
#define SERIAL9600
#include "settings.h"

/////////////////////////
// MAKEY MAKEY STRUCT ///
/////////////////////////
typedef struct {
  byte pinNumber;
  int keyCode;
  int timePressed;
  int color;
  
  float pressThreshold;
  float releaseThreshold;
  
  float movingAverage;
  boolean pressed;
  boolean prevPressed;
} 
MakeyMakeyInput;
MakeyMakeyInput inputs[NUM_STRIPS];

///////////////////////////////////
// VARIABLES //////////////////////
///////////////////////////////////
float movingAverageFactor = 1;
byte inByte;

float pressThreshold[NUM_STRIPS] = {13, 13, 13, 13, 10, 17.5, 10, 12};
float releaseThreshold[NUM_STRIPS] = {3.6, 3.6, 3.6, 3.6, 3.6, 3.6, 3.6, 3.6};
int triggerThresh = 200;
boolean inputChanged;

int pinNumbers[NUM_STRIPS] = { 
  2,    
  3,    
  4,    
  5,      
  6,     
  7,     
  8,     
  9     
};

//Switches 1-8, pink, red, orange, yellow, green, blue, indigo, purple... 
int stripColors[NUM_STRIPS] = {
  0xF69CFB, // pink
  0xFF0004, // red
  0xFFA600, // orange
  0xFFFF00, // yellow
  0x09FF00, // green
  0x00DDFF, // blue
  0x0D00FF, // indigo
  0xAA00FF // purple 
};

// LED that indicates when key is pressed
const int outputK = 13;
byte ledCycleCounter = 0;

// timing
int loopTime = 0;
int prevTime = 0;
int loopCounter = 0;

/////////////////////////
// NEOPIXELS ////////////
/////////////////////////
// http://learn.adafruit.com/adafruit-neopixel-uberguide/neomatrix-library
// set the Neopixel pin to 0 - D0
#define NEO_PIN 12
#define NUM_NEOPIXELS 144
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_NEOPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

///////////////////////////
// FUNCTIONS //////////////
///////////////////////////
void initializeArduino();
void initializeInputs();
void updateMovingAverage();
void updateInputStates();
void addDelay();
void updateOutLED();

//////////////////////
// SETUP /////////////
//////////////////////
void setup() 
{
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


#ifdef DEBUG
  delay(4000); // allow us time to reprogram in case things are freaking out
#endif
}

///////////////////////////
// INITIALIZE INPUTS
///////////////////////////
void initializeInputs() {

  for (int i=0; i<NUM_STRIPS; i++) {
    inputs[i].pinNumber = pinNumbers[i];
    inputs[i].keyCode = keyCodes[i];
    inputs[i].movingAverage = 0;
    inputs[i].color = stripColors[i];
    
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
  inputChanged = false;
  for (int i=0; i<NUM_STRIPS; i++) {
    inputs[i].prevPressed = inputs[i].pressed; // store previous pressed state (only used for mouse buttons)
    if (inputs[i].pressed) {
      if (inputs[i].movingAverage < inputs[i].releaseThreshold) {  
        inputChanged = true;
        inputs[i].pressed = false;
        updateLorax();
      }
    } 
    else if (!inputs[i].pressed) {
      if (inputs[i].movingAverage > inputs[i].pressThreshold) {  // input becomes pressed
        inputChanged = true;
        inputs[i].pressed = true; 
        updateLorax(); 
      }
    }
  }
#ifdef DEBUG3
  if (inputChanged) {
    Serial.println("change");
  }
#endif
}


///////////////////////////
// UPDATE LORAX
///////////////////////////
void updateLorax() {
  for(int i=0; i< NUM_STRIPS; i++) {
    if(inputs[i].pressed){
      byte passVal = i;
      //Serial.write(passVal);
      Serial.print(i);
      Serial.println(" should be on");
      updateNeopixelColor(inputs[i].color);
      break;
    }
    else {
      byte passVal = i+NUM_STRIPS;
      //Serial.write(passVal);
      updateNeopixelColor(0);
      Serial.print(i);
      Serial.println(" should be off");
    }
  }
}

  

void updateNeopixelColor(int color) {
  for(int i = 0; i < NUM_NEOPIXELS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

///////////////////////////
// UPDATE OUT LED
///////////////////////////
void updateOutLED() {
  boolean keyPressed = 0;
  for (int i=0; i<NUM_STRIPS; i++) {
    if (inputs[i].pressed) {
        keyPressed = 1;
#ifdef DEBUG
        Serial.print("Key ");
        Serial.print(i);
        Serial.println(" pressed");
#endif
    }
  }

  if (keyPressed) {
    digitalWrite(outputK, HIGH);
  }
  else {       
    digitalWrite(outputK, LOW);
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

