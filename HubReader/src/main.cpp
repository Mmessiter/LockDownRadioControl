#define GPSI2CHUB 8
#include <Arduino.h>
#include <Wire.h>
int led = LED_BUILTIN;


// ***************************************************************************************************************************************************
// The GPS HUB is asked for 11 bytes of data over I2C. The first IDLEN bytes are the ID (LAT, LNG, etc...)
// The next 8 bytes are the value (as a double).

void GetI2CData(){
  #define IDLEN 3
  #define GPSI2CBYTES IDLEN + 8

   char RdataID[IDLEN];
   double RdataIn;
   union { double Val64; uint8_t Val8[8]; } Rdata;  // 'union' allows access to every byte

  Wire.requestFrom(GPSI2CHUB, GPSI2CBYTES);         // Ask hub for data
  for (int j = 0; j < GPSI2CBYTES; ++j ){
    if (Wire.available()) {                         // Listen to HUB
      if (j < 3){
             RdataID[j]          = Wire.read();     // This gets the three-char data id (eg LAT)
      }else{
             Rdata.Val8[j-IDLEN] = Wire.read();     // This gets the 64 bit value for that data ID
      }
    }
  }
  RdataIn = Rdata.Val64;
  Serial.println(RdataID);
  Serial.println(RdataIn,18);
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
