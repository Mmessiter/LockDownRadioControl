
//***********************************************************************************************************
//************************************* SENSOR HUB CODE *****************************************************
//***********************************************************************************************************
#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#define I2CADDRESS  8
#define GPSBAUDRATE 9600
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
}
//*************************************** READ GPS DEVICE ***************************************************
 void ReadGps() {
    while (Serial1.available()){
    gps.encode(Serial1.read());
   }
 }
//*************************************** MAIN LOOP **********************************************************
void loop() {
    ReadGps();
}
//**************************************** SETUP *************************************************************
void setup() {
  Serial1.begin(GPSBAUDRATE);
  Wire.begin(I2CADDRESS);             
  Wire.onRequest(SendEvent);    
  Wire.onReceive(ReceiveEvent);
}
