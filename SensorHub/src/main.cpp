
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
float   GPSLatitude;
float   GPSLongitude;
uint8_t GPSSatellites;
float   GPSSpeed;
uint8_t Hours;
uint8_t Mins;
uint8_t Secs;

//************************************* SEND DATA INTERRUPT HANDLER ******************************************


// Here Send data response:  (LAT + float etc ...

void SendEvent() {
      Wire.write("Hi There! "); 
}
//************************************* RECEIVE DATA INTERRUPT HANDLER ***************************************

// Here receive request for data:  (SAT,LAT,LON,SPE,MIN,TIM,FAR,MRK,BER)

void ReceiveEvent(int q) {
  
  while(Wire.available()) 
  {
    Wire.read(); // one byte at a time
   
        
  }      
 
}
//*************************************** READ GPS DEVICE ***************************************************
 void ReadGps() {
   
    while (GPSDEVICE.available()){
     gps.encode(GPSDEVICE.read());  // send data to library for processing
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
      Serial.println(gps.altitude.meters());
      Serial.print (" Speed MPH: ");
      Serial.println(gps.speed.mph());
      Serial.print ("      Time: ");
      Serial.print(gps.time.hour());
      Serial.print(".");
      Serial.print(gps.time.minute());
      Serial.print(".");
      Serial.print(gps.time.second());
      Serial.println("");


  }

}
//**************************************** SETUP *************************************************************
void setup() {
  GPSDEVICE.begin(GPSBAUDRATE);
  Wire.begin(I2CADDRESS);             
  Wire.onRequest(SendEvent);    
  Wire.onReceive(ReceiveEvent);
}
