#define GPSI2CHUB 8
#include <Arduino.h>
#include <Wire.h>
int led = LED_BUILTIN;
int count = 0;


// ***************************************************************************************************************************************************
// Here the GPS HUB is asked for 11 bytes of data over I2C. 
// The first IDLEN bytes are the ID (LAT, LNG, etc...)
// The next 8 bytes are the value (as a double).
// The ID changes with each call

void GetI2CData(){
  #define IDLEN 3
  #define GPSI2CBYTES IDLEN + 8
  char RdataID[IDLEN+1];
  double RdataIn;
  union { double Val64; uint8_t Val8[8]; } Rdata;   // 'union' allows access to every byte
  Wire.requestFrom(GPSI2CHUB, GPSI2CBYTES);         // Ask hub for data
  for (int j = 0; j < GPSI2CBYTES; ++j ){
    if (Wire.available()) {                         // Listen to HUB
      if (j < IDLEN){
             RdataID[j]          = Wire.read();     // This gets the three-char data id (eg LAT)
      }else{
             Rdata.Val8[j-IDLEN] = Wire.read();     // This gets the 64 bit value for that data ID
      }
    }
  }
  RdataID[3] = 0;                                   // To terminate the string.
  RdataIn = Rdata.Val64;
  Serial.print(RdataID);
  Serial.print(" = ");
  Serial.println(RdataIn,8);
}
// ***************************************************************************************************************************************************
void  SendDataToI2C(char m[]){
  Wire.beginTransmission(GPSI2CHUB);   
  Wire.write(m);
  Wire.endTransmission();   
}
void loop()
{
  char MRK[4] = "MRK";
  char MAY[4] = "MAY";

  if (count > 20){
    SendDataToI2C(MRK);
    }
  delay(750);
  GetI2CData();
  ++count;
}
void setup()
{
  Wire.begin();  
}
