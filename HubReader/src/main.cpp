#define GPSI2CHUB 8
#include <Arduino.h>
#include <Wire.h>
int led = LED_BUILTIN;


// ***************************************************************************************************************************************************
// The GPS HUB is asked for 11 bytes of data over I2C. The first 3 are the ID (LAT, LNG, etc...)
// The next 8 are the value - as a double.

void GetI2CData(){
   #define GPSI2CBYTES 11
   int j = 0;
   char RdataID[3];
   double RdataIn;
   union {double Val64;uint8_t Val8[8];} Rdata;

  Wire.requestFrom(GPSI2CHUB, GPSI2CBYTES);  // Ask hub for data
  while(Wire.available()) {                  // Listen to HUB
      if (j < 3){
             RdataID[j]      = Wire.read();  // This gets the three-char data id (eg LAT)
      }else{
             Rdata.Val8[j-3] = Wire.read();  //  This gets the 64 bit value for that data ID
      }
      ++j;
  }
  RdataIn = Rdata.Val64;
  Serial.println(RdataID);
  Serial.println(RdataIn,8);
}
// ***************************************************************************************************************************************************
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
