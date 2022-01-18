#include <Arduino.h>
#include <Wire.h>
int led = LED_BUILTIN;

void loop()
{
  Serial.print("Received over I2C: ");

  digitalWrite(led, HIGH);  // briefly flash the LED
  Wire.requestFrom(8, 9);   // request 6 bytes from slave device #8

  while(Wire.available()) { // slave may send less than requested
    char c = Wire.read();   // receive a byte as character
    Serial.print(c);        // print the character
  }
  Serial.println();
  digitalWrite(led, LOW);
  delay(500);
}

void setup()
{
  pinMode(led, OUTPUT);
  Wire.begin();             // join i2c bus (address optional for master)
  Serial.begin(9600);       // start serial for output
}
