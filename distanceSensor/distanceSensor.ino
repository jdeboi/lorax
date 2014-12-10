#define trigPin 13
#define echoPin 12
#define led 11
#define led2 10

#define NEO_PIN 7
#define NUM_NEOPIXELS 144
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_NEOPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);


void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(led, OUTPUT);
  pinMode(led2, OUTPUT);
  strip.begin();
}

void loop() {
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  if (distance < 4) {  // This is where the LED On/Off happens
    digitalWrite(led,HIGH); // When the Red condition is met, the Green LED should turn off
  digitalWrite(led2,LOW);
}
  else {
    digitalWrite(led,LOW);
    digitalWrite(led2,HIGH);
  }
  if (distance >= 200 || distance <= 0){
    Serial.println("Out of range");
    setColor(0,0,0);
  }
  else {
    Serial.print(distance);
    Serial.println(" cm");
    setColor(Wheel(map(distance, 0, 255, 0, 200)));
  }
  delay(500);
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
