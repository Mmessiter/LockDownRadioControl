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

  Wire.beginTransmission(8);   
  Wire.write("This is a test message ...  ");
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
