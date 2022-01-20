#include <Arduino.h>
#include <Wire.h>
int led = LED_BUILTIN;



void GetI2CData(){
  Serial.print("Received over I2C: ");
  Wire.requestFrom(8, 9);  
  while(Wire.available()) { 
    char c = Wire.read();  
    Serial.print(c);   
  }
  Serial.println();
}

void SendI2CData(){
  char request[4] = "LAT";
  Wire.beginTransmission(8);   
  Wire.write(request);
  Wire.endTransmission();   
}

void loop()
{
  SendI2CData();
  delay(250);
  GetI2CData();
}
void setup()
{
  Wire.begin();  
}
