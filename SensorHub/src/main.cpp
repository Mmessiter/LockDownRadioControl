#include <Arduino.h>
#include <Wire.h>

void requestEvent() {
  Wire.write("Hi There! "); 
}

void loop() {
  char a;
  while (Serial1.available()){
    a = Serial1.read();
    Serial.print (a);
  }
}

void setup() {
  Serial1.begin(9600);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
}
