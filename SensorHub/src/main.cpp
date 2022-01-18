#include <Arduino.h>
#include <Wire.h>

void requestEvent() {
  Wire.write("Hi There! "); 
}

void loop() {
  delay(100);
}

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
}
