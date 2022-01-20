
//***********************************************************************************************************
//************************************* SENSOR HUB CODE *****************************************************
//***********************************************************************************************************
#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#define I2CADDRESS  8
#define GPSBAUDRATE 9600
#define GPSDEVICE Serial1
#define DEBUG
#define DEBUGTIMER 1000

int DebugTimer = 0;
TinyGPSPlus gps;  
float   GPSLatitude;
float   GPSLongitude;
float   GPSAltitude;
uint8_t GPSSatellites;
float   GPSSpeed;
uint8_t GPSHours;
uint8_t GPSMins;
uint8_t GPSSecs;
uint8_t GPSDay;
uint8_t GPSMonth;
uint8_t GPSYear;

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
   if (gps.sentencesWithFix()) {
      GPSLatitude   = gps.location.lat();
      GPSLongitude  = gps.location.lng();
      GPSSatellites = gps.satellites.value(); 
      GPSAltitude   = gps.altitude.feet();
      GPSSpeed      = gps.speed.mph();
      GPSHours      = gps.time.hour();   
      GPSMins       = gps.time.minute(); 
      GPSSecs       = gps.time.second(); 
      GPSDay        = gps.date.day();
      GPSMonth      = gps.date.month();
      GPSYear       = gps.date.year();
   }
 }

void ShowGPS(){
  if ((millis() - DebugTimer) > DEBUGTIMER) {
      DebugTimer = millis();
      Serial.println("");
      Serial.print (" Satellites: ");
      Serial.println(GPSSatellites);    
      Serial.print ("   Latitude: ");
      Serial.println(GPSLatitude,8);
      Serial.print ("  Longitude: ");
      Serial.println(GPSLongitude,8);
      Serial.print ("Altitude ft: ");
      Serial.println(GPSAltitude);
      Serial.print ("  Speed MPH: ");
      Serial.println(GPSSpeed);
      Serial.print ("       Time: ");
      Serial.print (GPSHours);
      Serial.print (".");
      Serial.print (GPSMins);
      Serial.print (".");
      Serial.println (GPSSecs);
      Serial.print ("       Date: ");
      Serial.print (GPSDay);
      Serial.print (":");
      Serial.print (GPSMonth);
      Serial.print (":");
      Serial.print (GPSYear);
      Serial.println ("");
  }
}


//*************************************** MAIN LOOP **********************************************************
void loop() {
 
   ReadGps();


#ifdef DEBUG
    ShowGPS();
#endif

}
//**************************************** SETUP *************************************************************
void setup() {
  GPSDEVICE.begin(GPSBAUDRATE);
  Wire.begin(I2CADDRESS);             
  Wire.onRequest(SendEvent);    
  Wire.onReceive(ReceiveEvent);
}
