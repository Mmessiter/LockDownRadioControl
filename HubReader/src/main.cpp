#define GPSI2CHUB 8
#include <Arduino.h>
#include <Wire.h>
int led = LED_BUILTIN;
int count = 0;


// ***************************************************************************************************************************************************
// Here the GPS HUB is asked for 11 bytes of data over I2C. 
// The first IDLEN (=3) bytes are the ID (LAT, LNG, etc...)
// The next 8 bytes are the value (as a double).
// The ID changes with each call

void GetI2CData(){
  #define IDLEN 3
  #define GPSI2CBYTES IDLEN + 8
  char  LAT[IDLEN+1]    = "LAT";  // Latitude
  char  LON[IDLEN+1]    = "LON";  // Longitude
  char  FIX[IDLEN+1]    = "FIX";  // Fix ?
  char  SAT[IDLEN+1]    = "SAT";  // How many satellites
  char  ALT[IDLEN+1]    = "ALT";  // Altitude
  char  SPD[IDLEN+1]    = "SPD";  // Speed
  char  COR[IDLEN+1]    = "COR";  // Course
  char  CTO[IDLEN+1]    = "CTO";  // Course to
  char  DTO[IDLEN+1]    = "DTO";  // Distance to
  char  HRS[IDLEN+1]    = "HRS";  // GMT Hours  
  char  MNS[IDLEN+1]    = "MNS";  // GMT Minutes  
  char  SEC[IDLEN+1]    = "SEC";  // GMT Seconds


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
  RdataID[3] = 0;                                   // To terminate the ID string.
  RdataIn = Rdata.Val64;                            // To re-assemble the 64 BIT data to a double


  if (strncmp(FIX,RdataID,3) == 0) {     
      if (RdataIn == 1) {
        Serial.println ("GOT FIX!"); 
        return;
      }
  }

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
