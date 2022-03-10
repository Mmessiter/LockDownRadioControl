//***********************************************************************************************************
//************************************* SENSOR HUB CODE FOR TEENSY 3.2....   ********************************
// // Malcolm Messiter 2022
//                                   Version 1.3 Feb 7 2022
//                                   Uses Wire1 in Slave Mode ... This works. Don't use Teensy LC. IT DOESN'T!!
//***********************************************************************************************************
#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <Adafruit_INA219.h> 
#include <Adafruit_BMP280.h>
#define I2CADDRESS  8       // Address of this I2C slave
#define GPSBAUDRATE 9600    // Didn't work any faster
#define GPSDEVICE Serial1   // GPS is connected to Serial1
// #define DEBUG             // Local console debug only
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
float   GPSDistanceTo;
float   GPSCourseTo; 
static  const double MAYSLANE_LAT = 51.638963994850364;       // Mays Lane location
static  const double MAYSLANE_LON = -0.22926821753992477;     // Mays Lane location
float   DestinationLat = MAYSLANE_LAT;
float   DestinationLng = MAYSLANE_LON;
char    PMTK_API_SET_FIX_CTL_1HZ[]       =  "$PMTK300,1000,0,0,0,0*1C";     // < 1 Hz
char    PMTK_API_SET_FIX_CTL_5HZ[]       =  "$PMTK300,200,0,0,0,0*2F" ;    // < 5 Hz
char    PMTK_SET_NMEA_OUTPUT_ALLDATA[]   =  "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"; ///< turn on ALL THE DATA
char    PMTK_SET_NMEA_OUTPUT_RMCGGAGSA[] =  "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"; ///< turn on GPRMC, GPGGA and GPGSA
char    PMTK_SET_BAUD_9600[]             =  "$PMTK251,9600*17";     // <   9600 bps     
char    PGCMD_NOANTENNA[]                =  "$PGCMD,33,0*6D" ;      // < don't show antenna status messages
uint8_t ParameterNumber                  = 0;
bool    FOUND_BMP280 = false;                   //  BMP280 sensor connected ?
bool    FOUND_INA219 = false;                   //  Volts from INA219 ?
Adafruit_INA219 ina219;
Adafruit_BMP280 bmp280;
uint32_t LoopTimer;
float    BaroTemperature ;
float    BaroAltitude ;
float    INA219Volts;
uint16_t Qnh = 0;
volatile bool ReceiveRequestFlag = false;

//************************************* RECEIVE DATA INTERRUPT HANDLER ***************************************
void ReceiveEventInterrupt(int q) { 
     ReceiveRequestFlag = true;  // Set the flag and return immediately
}
// ***********************************************************************************************************
void SendDataToReceiver() {
  #define IDLEN 3
  #define GPSI2CBYTES IDLEN + 4
  #define MAXPARAMS 17
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
  char  BLT[IDLEN+1]    = "BLT";
  char  TMP[IDLEN+1]    = "TMP";
  char  VLT[IDLEN+1]    = "VLT";
  char  DAY[IDLEN+1]    = "DAY";
  char  MTH[IDLEN+1]    = "MTH";
  char  YER[IDLEN+1]    = "YER";

  float RdataOut = 42;
  union { float   Val32;uint8_t Val8[4]; } Rdata;

  switch (ParameterNumber) {
      case  0:
        RdataOut =  GPSLatitude;    // Latitiude
        strcpy (RdataID,LAT);
        break;
      case  1:
        RdataOut = GPSLongitude;   // Longitude
        strcpy (RdataID,LON);
        break;
      case  2:
        RdataOut = (float) GPSFix;  // Fix
        strcpy (RdataID,FIX);
        break;
      case  3:
        RdataOut =  GPSAltitude;    // Altitude
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
        RdataOut =  GPSCourseTo;     // Course to
        strcpy (RdataID,CTO);
        break;
      case  7:
        RdataOut =  GPSDistanceTo;   // Distance to
        strcpy (RdataID,DTO);
        break;
      case  8:
        RdataOut = (float) GPSHours;       // Hours
        strcpy (RdataID,HRS);
        break;
      case  9:
        RdataOut = (float) GPSMins;        // Mins
        strcpy (RdataID,MNS);
        break;
      case  10:
        RdataOut = (float) GPSSecs;        // Secs
        strcpy (RdataID,SEC);
        break;
      case  11:
        RdataOut =  (float) GPSSatellites;  // Sats
        strcpy (RdataID,SAT);
        break;
      case  12:
        RdataOut = BaroAltitude;            // ALT FROM BMP280
        strcpy (RdataID,BLT);
        break;
      case  13:
        RdataOut =  BaroTemperature;        // TEMPERATURE FROM BMP280
        strcpy (RdataID,TMP);
        break;
      case  14:
        RdataOut =  INA219Volts;             // VOLTAGE FROM INA219
        strcpy (RdataID,VLT);
        break;
      case  15:
        RdataOut =  GPSDay;                  // DATE
        strcpy (RdataID,DAY);
        break;
      case  16:
        RdataOut =  GPSMonth;                // Month
        strcpy (RdataID,MTH);
        break;
      case  17:
        RdataOut =  GPSYear;                  // YEAR
        strcpy (RdataID,YER);
        break;
      default:
        break;
  }
   Rdata.Val32 = RdataOut;
   for (i = 0; i < IDLEN; ++i) {
      Wire1.write(RdataID[i]); 
   }
    for (i = 0; i < 4; ++i) {
       Wire1.write(Rdata.Val8[i]); 
    }
    ParameterNumber++;
    if (ParameterNumber > MAXPARAMS) ParameterNumber = 0; 
}
// *******************************************************************************************************

void DoTheRequest(){
char MRK[] = "MRK";
char MAY[] = "MAY";
char QNH[] = "QNH";
char RCV[10];

union {uint16_t Val16; uint8_t Val8[2];} Uqnh;  
    
    ReceiveRequestFlag = false;
    
    for (uint8_t i = 0; i < 6; ++i) {
        if (Wire1.available()) {
            RCV[i]= Wire1.read();    // Get one byte at a time
        }
    } 
  RCV[3] = 0; // Add a terminator
  if (strcmp(MRK,RCV) == 0) {        // Match first 3 chars with MRK?
     DestinationLat = GPSLatitude;   // Mark current location
     DestinationLng = GPSLongitude;  // Mark current location
     return;
  }
 if (strcmp(MAY,RCV) == 0) {      // Match first 3 chars with MAY?
      DestinationLat = MAYSLANE_LAT;  // Mark MAYS LANE location
      DestinationLng = MAYSLANE_LON;  // Mark MAYS LANE location
      return;
  }
  if (strcmp(QNH,RCV) == 0) {      // Match first 3 chars with QNH?
      Uqnh.Val8[0] = RCV[4];       // 16 bit value for "Qnh" sent in bytes 4 and 5
      Uqnh.Val8[1] = RCV[5]; 
      Qnh = Uqnh.Val16;
      return;
  }
}

//*************************************** READ GPS DEVICE ***************************************************
 void ReadGps() {
   char a;
    while (GPSDEVICE.available()){
      a = GPSDEVICE.read();
      gps.encode(a);  // Simply send every byte to the library 
   }
      GPSFix           = gps.location.isValid();
      if (GPSFix) {
        GPSLatitude    = (float)   gps.location.lat();
        GPSLongitude   = (float)   gps.location.lng();
        GPSSatellites  = (uint8_t) gps.satellites.value(); 
        GPSAltitude    = (float)   gps.altitude.feet();
        GPSSpeed       = (float)   gps.speed.mph();
        GPSHours       = (uint8_t) gps.time.hour();   
        GPSMins        = (uint8_t) gps.time.minute(); 
        GPSSecs        = (uint8_t) gps.time.second(); 
        GPSDay         = (uint8_t) gps.date.day();
        GPSMonth       = (uint8_t) gps.date.month();
        GPSYear        = (uint8_t) gps.date.year();
        *GPSLibVersion = gps.libraryVersion();
        GPSCourse      = (float) gps.course.deg();
        GPSDistanceTo  = (float) gps.distanceBetween(GPSLatitude,GPSLongitude,DestinationLat,DestinationLng);
        GPSCourseTo    = (float) gps.courseTo(GPSLatitude,GPSLongitude,DestinationLat,DestinationLng);
      }
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
      Serial.print ("B. Altitude: ");
      Serial.println (BaroAltitude);
      Serial.print  ("B. Temp.   : ");
      Serial.println (BaroTemperature);
      Serial.print  ("Voltage:   : ");
      Serial.println (INA219Volts);
      Serial.println ("-------------------------");
  }
}
// ***********************************************************************************************************
void ReadOtherSensors(){
    if (FOUND_BMP280) {
        BaroTemperature = bmp280.readTemperature();
        BaroAltitude    = (bmp280.readAltitude(Qnh)) * 3.28084; 
    }
    if (FOUND_INA219) INA219Volts = ina219.getBusVoltage_V();
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

 if (ReceiveRequestFlag) DoTheRequest();

 if ((millis()-LoopTimer) > 10)        // 100x per second is enough
  {
    LoopTimer = millis();
    ReadGps();
    ReadOtherSensors();
#ifdef DEBUG
    ShowGPS();
#endif
  }
}
// **************************************************************************************
void InitBMP280()
{
    bmp280.begin(0x76);
    bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                       Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                       Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                       Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                       Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}
/***********************************************************************************************************/
void ScanI2c()
{
    delay(200); // allow time to wake things up ... BUT NOT EXCESSIVELY - MUST BOOT BEFORE RECEIVER LOOKS FOR!
    for (uint8_t i = 1; i < 127; ++i) {
        Wire.beginTransmission(i);
        delay(10);
        if (Wire.endTransmission() == 0) {
             if (i == 0x40) {
                FOUND_INA219 = true;
#ifdef DEBUG
                Serial.println("INA219 voltage meter detected!");
#endif
              }
             if (i == 0x76) {
                FOUND_BMP280 = true;
#ifdef DEBUG
                Serial.println("BMP280 barometer detected!");
#endif
              }
         // Serial.print(i, HEX);
         // Serial.println("   "); // in case some new device shows up  
        }
    }
}
//**************************************** SETUP *************************************************************
void setup() {
  Wire1.begin(I2CADDRESS);                        // Wire1 MUST boot up BEFORE the receiver in order to be found by it.
  Wire1.onRequest(SendDataToReceiver);    
  Wire1.onReceive(ReceiveEventInterrupt);
  GPSDEVICE.begin(GPSBAUDRATE);
  Wire.begin();                                   // Wire used to read locally attached i2c sensors
  ScanI2c();
  if (FOUND_BMP280) InitBMP280();
  if (FOUND_INA219) ina219.begin();
  delay (100);
  SendToGPS(PGCMD_NOANTENNA);                     // These setup commands are for Adafruit Ulimate GPS only
  delay (100);
  SendToGPS(PMTK_API_SET_FIX_CTL_1HZ);           
  delay (100);
  SendToGPS(PMTK_SET_NMEA_OUTPUT_RMCGGAGSA);      
  delay (100);
}
