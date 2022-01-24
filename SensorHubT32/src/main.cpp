//***********************************************************************************************************
//************************************* SENSOR HUB CODE *****************************************************
//***********************************************************************************************************
#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>

#define I2CADDRESS  8       // Address of this I2C slave
#define GPSBAUDRATE 9600    // Didn't work any faster
#define GPSDEVICE Serial1   // GPS is connected to Serial1
#define DEBUG               // Local console debug only
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
uint8_t GPSFix;
const   char *  GPSLibVersion[20];
float   GPSCourse;
double  GPSDistanceTo;
double  GPSCourseTo; 
static  const double MAYSLANE_LAT = 51.638963994850364;  // Mays Lane
static  const double MAYSLANE_LON = -0.22926821753992477;
float   DestinationLat = MAYSLANE_LAT;
float   DestinationLng = MAYSLANE_LON;
char    PMTK_API_SET_FIX_CTL_1HZ[]       =  "$PMTK300,1000,0,0,0,0*1C";     // < 1 Hz
char    PMTK_API_SET_FIX_CTL_5HZ[]       =  "$PMTK300,200,0,0,0,0*2F" ;    // < 5 Hz
char    PMTK_SET_NMEA_OUTPUT_ALLDATA[]   =  "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"; ///< turn on ALL THE DATA
char    PMTK_SET_NMEA_OUTPUT_RMCGGAGSA[] =  "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"; ///< turn on GPRMC, GPGGA and GPGSA
char    PMTK_SET_BAUD_9600[]             =  "$PMTK251,9600*17";     // <   9600 bps     
char    PGCMD_NOANTENNA[]                =  "$PGCMD,33,0*6D" ;      // < don't show antenna status messages
uint8_t ParameterNumber                  = 0;


//************************************* SEND DATA INTERRUPT HANDLER ******************************************
// 
void SendDataToReceiver() {
  #define IDLEN 3
  #define GPSI2CBYTES IDLEN + 8
  #define MAXPARAMS 11
  uint8_t i;
  char RdataID[IDLEN+1] = "NUL";
  char  LAT[IDLEN+1]    = "LAT";
  char  LON[IDLEN+1]    = "LON";
  char  FIX[IDLEN+1]    = "FIX";
  char  SAT[IDLEN+1]    = "SAT";
  char  ALT[IDLEN+1]    = "ALT";
  char  SPD[IDLEN+1]    = "SPD";
  char  COR[IDLEN+1]    = "COR";
  char  CTO[IDLEN+1]    = "CTO";
  char  DTO[IDLEN+1]    = "DTO";
  char  HRS[IDLEN+1]    = "HRS";
  char  MNS[IDLEN+1]    = "MNS";
  char  SEC[IDLEN+1]    = "SEC";
  double RdataOut = 42;
  union { double   Val64;uint8_t Val8[8]; } Rdata;
  switch (ParameterNumber) {
      case  0:
        RdataOut = GPSLatitude;     // Latitiude
        strcpy (RdataID,LAT);
        break;
      case  1:
        RdataOut = GPSLongitude;   // Longitude
        strcpy (RdataID,LON);
        break;
      case  2:
        RdataOut = GPSFix;          // Fix
        strcpy (RdataID,FIX);
        break;
      case  3:
        RdataOut = GPSAltitude;     // Altitude
        strcpy (RdataID,ALT);
        break;
      case  4:
        RdataOut = GPSSpeed;        // Speed
        strcpy (RdataID,SPD);
        break;
      case  5:
        RdataOut = GPSCourse;       // Course
        strcpy (RdataID,COR);
        break;
      case  6:
        RdataOut = GPSCourseTo;     // Course to
        strcpy (RdataID,CTO);
        break;
      case  7:
        RdataOut = GPSDistanceTo;   // Distance to
        strcpy (RdataID,DTO);
        break;
      case  8:
        RdataOut = GPSHours;        // Hours
        strcpy (RdataID,HRS);
        break;
      case  9:
        RdataOut = GPSMins;        // Hours
        strcpy (RdataID,MNS);
        break;
      case  10:
        RdataOut = GPSSecs;        // Secs
        strcpy (RdataID,SEC);
        break;
      case  11:
        RdataOut = GPSSatellites;  // Sats
        strcpy (RdataID,SAT);
        break;
    
  }
   Rdata.Val64 = RdataOut;
   
   for (i = 0; i < IDLEN; ++i) {
      Wire.write(RdataID[i]); 
   }
    for (i = 0; i < 8; ++i) {
       Wire.write(Rdata.Val8[i]); 
    }

    ParameterNumber++;
    if (ParameterNumber > MAXPARAMS) ParameterNumber = 0; 
}
//************************************* RECEIVE DATA INTERRUPT HANDLER ***************************************

void ReceiveEvent(int q) {  
char MRK[] = "MRK";
char MAY[] = "MAY";
char RCV[4];
    for (uint8_t i = 0; i < 3; ++i) {
        if (Wire.available()) {
            RCV[i]= Wire.read();    // Get one byte at a time
        }
    } 
  RCV[3] = 0; // Add a terminator

  if (strcmp(MRK,RCV) == 0) {     // Match first 3 chars with MRK?
     DestinationLat = GPSLatitude;   // Mark current location
     DestinationLng = GPSLongitude;  // Mark current location
     return;
  }

 if (strcmp(MAY,RCV) == 0) {      // Match first 3 chars with MAY?
     DestinationLat = MAYSLANE_LAT;  // Mark MAYS LANE location
     DestinationLng = MAYSLANE_LON;  // Mark MAYS LANE location
     return;
  }

}
//*************************************** READ GPS DEVICE ***************************************************
 void ReadGps() {

   char a;
    while (GPSDEVICE.available()){
      a = GPSDEVICE.read();
      gps.encode(a);  // Simply send every byte library 
   }
      GPSFix         = gps.location.isValid();
      GPSLatitude    = gps.location.lat();
      GPSLongitude   = gps.location.lng();
      GPSSatellites  = gps.satellites.value(); 
      GPSAltitude    = gps.altitude.feet();
      GPSSpeed       = gps.speed.mph();
      GPSHours       = gps.time.hour();   
      GPSMins        = gps.time.minute(); 
      GPSSecs        = gps.time.second(); 
      GPSDay         = gps.date.day();
      GPSMonth       = gps.date.month();
      GPSYear        = gps.date.year();
      *GPSLibVersion = gps.libraryVersion();
      GPSCourse      = gps.course.deg();
      GPSDistanceTo  = gps.distanceBetween(GPSLatitude,GPSLongitude,DestinationLat,DestinationLng);
      GPSCourseTo    = gps.courseTo(GPSLatitude,GPSLongitude,DestinationLat,DestinationLng);
 }
// *********************************************** DEBUG DATA ***********************************************
void ShowGPS(){
  if ((millis() - DebugTimer) > DEBUGTIMER) {
      DebugTimer = millis();
      Serial.print ("        Fix: ");
      if (GPSFix) {Serial.println ("Yes");}else {Serial.println ("No");} 
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
      Serial.print ("     Course: ");
      Serial.println(GPSCourse);
      Serial.print ("  Course To: ");
      Serial.println(GPSCourseTo);
      Serial.print ("Distance To: ");
      Serial.println(GPSDistanceTo/1000,6);
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
      Serial.println (GPSYear+1792);  // ?????????
      Serial.print ("Lib version: ");
      Serial.println (*GPSLibVersion);
      Serial.println ("-------------------------");
  }
}

//*************************************** SendToGPS  **********************************************************
void SendToGPS(char Cmd[80]){
char a = 0;
uint8_t i = 0;
  while (i < strlen(Cmd)) {
     a = char (Cmd[i]) ;
     if (!a) return;
     GPSDEVICE.write(a);
     ++i;
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
  Wire.onRequest(SendDataToReceiver);    
  Wire.onReceive(ReceiveEvent);
  delay (100);
  SendToGPS(PGCMD_NOANTENNA);                     // These setup commands are for Adafruit Ulimate GPS only
  delay (100);
  SendToGPS(PMTK_API_SET_FIX_CTL_1HZ);            // These setup commands are for Adafruit Ulimate GPS only
  delay (100);
  SendToGPS(PMTK_SET_NMEA_OUTPUT_RMCGGAGSA);       // These setup commands are for Adafruit Ulimate GPS only
  delay (100);
}
