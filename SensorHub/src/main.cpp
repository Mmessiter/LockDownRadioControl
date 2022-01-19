
//***********************************************************************************************************
//************************************* SENSOR HUB CODE *****************************************************
//***********************************************************************************************************
#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#define I2CADDRESS  8
#define GPSBAUDRATE 9600
#define GPSDEVICE Serial1

TinyGPSPlus gps;  

//************************************* SEND DATA INTERRUPT HANDLER ******************************************

void SendEvent() {
      Wire.write("Hi There! "); 
}
//************************************* RECEIVE DATA INTERRUPT HANDLER ***************************************

void ReceiveEvent(int q) {
  while(Wire.available()) 
  {
    char c = Wire.read(); 
    Serial.print(c);        
  }      
  Serial.println("");    
  Serial.println(gps.location.lat());
  Serial.println(gps.location.lng());
  Serial.println(gps.satellites.value());
}
//*************************************** READ GPS DEVICE ***************************************************
 void ReadGps() {
   int c = 0;
    while (GPSDEVICE.available()){
      ++c; 
      delay(4);
     gps.encode(GPSDEVICE.read());
   }
   if (c) {
     Serial.print ("bytes encoded: ");
     Serial.println (c);
    }
   
 }
//*************************************** MAIN LOOP **********************************************************
void loop() {
    ReadGps();
}
//**************************************** SETUP *************************************************************
void setup() {
  GPSDEVICE.begin(GPSBAUDRATE);
  Wire.begin(I2CADDRESS);             
  Wire.onRequest(SendEvent);    
  Wire.onReceive(ReceiveEvent);
}
