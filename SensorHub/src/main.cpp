
//***********************************************************************************************************
//************************************* SENSOR HUB CODE *****************************************************
//***********************************************************************************************************
#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#define I2CADDRESS  8
#define GPSBAUDRATE 9600
#define GPSDEVICE Serial1
 int t = 0;
TinyGPSPlus gps;  

//************************************* SEND DATA INTERRUPT HANDLER ******************************************

void SendEvent() {
      Wire.write("Hi There! "); 
}
//************************************* RECEIVE DATA INTERRUPT HANDLER ***************************************

void ReceiveEvent(int q) {
  while(Wire.available()) 
  {
    Wire.read(); 
   // char c = Wire.read(); 
   // Serial.print(c);        
  }      
 
}
//*************************************** READ GPS DEVICE ***************************************************
 void ReadGps() {
   
    while (GPSDEVICE.available()){
     gps.encode(GPSDEVICE.read());
   }

 }
//*************************************** MAIN LOOP **********************************************************
void loop() {
 
  ReadGps();
  if ((millis() - t) > 5000) {
      t = millis();
      Serial.println("");  
      Serial.print ("  Latitude: ");
      Serial.println(gps.location.lat(),8);
      Serial.print (" Longitude: ");
      Serial.println(gps.location.lng(),8);
      Serial.print ("Satellites: ");
      Serial.println(gps.satellites.value());
      Serial.print ("  Altitude: ");
      Serial.println(gps.altitude.feet());

  }

}
//**************************************** SETUP *************************************************************
void setup() {
  GPSDEVICE.begin(GPSBAUDRATE);
  Wire.begin(I2CADDRESS);             
  Wire.onRequest(SendEvent);    
  Wire.onReceive(ReceiveEvent);
}
