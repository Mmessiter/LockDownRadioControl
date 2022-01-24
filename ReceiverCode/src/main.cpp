/** @file ReceiverCode/src/main.cpp
 * @page RXCODE RecieverCode
 *
 * @section rxFeatures Features List
 * - WORKS ON TEENSY 4.0
 * - Detects and uses INA219 to read volts
 * - Detects and uses BMP280 pressure sensor for altitude
 * - Binding implemented
 * - SBUS implemented
 * - Failsafe implemented (after two seconds)
 * - RESOLUTION INCREASED TO 12 BITS
 * - Channels increased to 16, 9 PWM outputs.  SBUS can handle all.
 * - Exponential implemented (at TX end)
 *
 * @section rxpinout TEENSY 4.0 PINS
 * | pin number(s) | purpose |
 * |---------------|---------|
 * | 0...8 | PWM SERVOS Channels 1 - 9 |
 * | 9     | SPI CE1  (FOR RADIO1) |
 * | 10    | SPI CSN1 (FOR RADIO1)  |
 * | 11    | SPI MOSI (FOR BOTH RADIOS)  |
 * | 12    | SPI MISO (FOR BOTH RADIOS)  |
 * | 13    | SPI SCK  (FOR BOTH RADIOS) |
 * | 14    | SBUS output (TX3) |
 * | 15    | Don't use. SBUS driver takes it (RX3) |
 * | 16    | RX4 for GPS | (Still to be implemented ...)
 * | 17    | TX4 for GPS |
 * | 18    | I2C SDA (FOR I2C) |
 * | 19    | I2C SCK (FOR I2C) |
 * | 20    | SPI CSN2 (FOR RADIO2)  |
 * | 21    | SPI CE2 (FOR RADIO2) |
 * | 22    | IRQ1 (FOR RADIO1) |
 * | 23    | IRQ2 (FOR RADIO2) |
 *
 * @see ReceiverCode/src/main.cpp
 */

// ************************************************** Receiver code **************************************************

#define RECEIVE_TIMEOUT 25 // 25 milliseconds seems an optimal value
#define CHANNELSUSED    16
#define SERVOSUSED      9  // All 16 are available via SBUS
#define SBUSRATE        10 // SBUS frame every 10 milliseconds
#define SBUSPORT        Serial3
#define RECONNECTGAP    20 // Send no data to servos for 20 ms after a reconnect (10 was not quite enough)
#define MINMICROS       500
#define MAXMICROS       2500
#define LED_PIN         LED_BUILTIN
#define RANGEMAX        2047 // = Frsky at 150 %
#define RANGEMIN        0

#include <Servo.h>
#include <EEPROM.h>
#include <SBUS.h>
#include <Adafruit_INA219.h> // new library used here±
#include <Adafruit_BMP280.h>
#include "utilities/radio.h"

bool            USE_BMP280 = false;                   //  BMP280 sensor connected ?
bool            USE_INA219 = false;                   //  Volts from INA219 ?
bool            USE_AdafruitUltimateGps = false;      //  GPS (Adafruit Ultimate GPS) ?
Adafruit_INA219 ina219;
Adafruit_BMP280 bmp280;
Servo           MCMServo[SERVOSUSED];
uint8_t         PWMPins[SERVOSUSED] = {0, 1, 2, 3, 4, 5, 6, 7, 8}; // 9 PWMs, remaining 7 via sbus
SBUS            MySbus(SBUSPORT);
float           PacketStartTime;
float           temperature280, pressure, altitude;
uint8_t         BindNow        = 0;     /** indicates that the receiver should start the binding/pairing process */
bool            BoundFlag      = false; /** indicates if receiver paired with transmitter */
int             BindOKTimer    = 0;
bool            ServosAttached = false;
uint16_t        SbusChannels[CHANNELSUSED + 8]; // a few spare
int             SBUSTimer = 0;
bool            FailSafeChannel[17];
bool            FailSafeDataLoaded = false;
bool            ReInit             = false;
uint8_t         FS_byte1           = 0; // All 16 failsafe channel flags are in these two bytes
uint8_t         FS_byte2           = 0;
uint32_t        ReconnectedMoment;
float           BaroAltitude;
float           BaroTemperature;
float           INA219Volts;
bool            Radio1Exists = false;
bool            Radio2Exists = false;
uint32_t        SensorTime   = 0;
uint16_t        Qnh          = 0;    //Pressure at sea level here and now (defined by TX option)
uint8_t         SatellitesGPS; 
double          LatitudeGPS;
double          LongitudeGPS;
double          SpeedGPS;
double          AngleGPS;
bool            GpsFix = false;
double          AltitudeGPS;
double          DistanceGPS;
double          CourseToGPS;
uint8_t         HoursGPS;
uint8_t         MinsGPS;
uint8_t         SecsGPS;


uint16_t        CompressedData[COMPRESSEDWORDS]; // 30 bytes -> 40 bytes when uncompressed
      

/************************************************************************************************************/
// This function returns distance (in meters) between two GPS coordinates (in degrees)
// it was essentially cribbed from the internet, then tested and adjusted a little. 

//FASTRUN double HowFar(double latitude_new, double longitude_new, double latitude_old, double longitude_old) {
//        double  RadiusOfTheEarth = 6372797.56085;                 // Meters by the way
//        double  DegreesToRadians = 3.14159265358979323846 / 180;
//        double  lat_new = latitude_old * DegreesToRadians;
//        double  lat_old = latitude_new * DegreesToRadians;
//        double  lat_diff = (latitude_new-latitude_old) *  DegreesToRadians;
//        double  lng_diff = (longitude_new-longitude_old) *  DegreesToRadians;
//        double  a = sin(lat_diff/2) * sin(lat_diff/2) + cos(lat_new) * cos(lat_old) *  sin(lng_diff/2) * sin(lng_diff/2);
//        double  c = 2 * atan2(sqrt(a), sqrt(1-a));
//        double  distance = RadiusOfTheEarth * c;
//        return  distance; 
//   }

void LoadFailSafeData()
{
    uint8_t  FS_Offset = 10;
    uint16_t s[CHANNELSUSED];

    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        s[i] = map(EEPROM.read(i + FS_Offset), 0, 180, MINMICROS, MAXMICROS); // load failsafe values and simulate better resloution
    }
    FS_Offset += CHANNELSUSED;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        if (EEPROM.read(i + FS_Offset)) {
            ReceivedData[i] = s[i];
        }
    }
    FailSafeDataLoaded = true;
#ifdef DB_FAILSAFE
    Serial.println("Fail safe settings are loaded!");
#endif
}

/************************************************************************************************************/

/** Map servo channels' data from ReceivedData buffer into SbusChannels buffer */
void MapToSBUS()
{
    if (Connected) {
        for (int j = 0; j < CHANNELSUSED; ++j) {
            SbusChannels[j] = map(ReceivedData[j], MINMICROS, MAXMICROS, RANGEMIN, RANGEMAX);
        }
    }
}

/************************************************************************************************************/

void MoveServos()
{
    if (!Connected) {
        return;
    }                               // avoid sending rubbish
    MySbus.write(&SbusChannels[0]); // Send SBUS data
    for (int j = 0; j < SERVOSUSED; ++j) {
        if (PreviousData[j] != ReceivedData[j]) { // if same as last time, don't bother sending again.
            MCMServo[j].writeMicroseconds(ReceivedData[j]);
            PreviousData[j] = ReceivedData[j];
        }
    }
}

/************************************************************************************************************/

/** Execute FailSafe data from EEPROM. */
void FailSafe()
{
    if (BoundFlag) {
        LoadFailSafeData();
        Connected = true; // to force sending this data!
        MapToSBUS();
        MoveServos();
        Connected = false; // I lied earlier - we're not really connected.
    }
}

/************************************************************************************************************/

/**
 * Print out some debugging information about the channel hopping implementation
 * @param freq The next frequency to be used.
 */
void ShowHopDurationEtc()
{
    float OnePacketTime = (millis() - PacketStartTime) / PacketNumber;
    Serial.print("Hop duration: ");
    Serial.print((millis() - PacketStartTime) / 1000);
    Serial.print("s  Packets per hop: ");
    Serial.print(PacketNumber);
    Serial.print("  Average Time per packet: ");
    Serial.print(OnePacketTime);
    Serial.print("ms  Next channel: ");
    Serial.print(FHSS_Channels[NextChannelNumber]);
    Serial.print(BoundFlag ? " Bound!" : " NOT Bound");
    Serial.print("  Radio: ");
    Serial.println(ThisRadio);
    PacketStartTime = millis();
}

/************************************************************************************************************/

void ClearAckPayload()
{
    AckPayload.Byte1 = 0;
    AckPayload.Byte2 = 0;
    AckPayload.Byte3 = 0;
    AckPayload.Byte4 = 0;
    AckPayload.Byte5 = 0;
    AckPayload.Purpose |= 0x80;
}
/************************************************************************************************************/

void UseReceivedData(){
        Decompress(ReceivedData, CompressedData, UNCOMPRESSEDWORDS);   // Decompress only the most recent data
        MapToSBUS();
        ClearAckPayload();
        CurrentRadio->flush_rx();
        LastConnectionMoment = millis();
        if (HopNow) {                   // this flag gets set in LoadAckPayload();
            FailSafeDataLoaded = false; // Ack payload instructed to Hop at next opportunity...
            HopToNextFrequency();       // So hop now
            HopNow = false;             // and clear the flag.
        }
}
/************************************************************************************************************/

bool ReadData()
{
    Connected = false;
    while (CurrentRadio->available()) {                                // Get all, but use only the latest
        LoadAckPayload();
        Connected = true;
        CurrentRadio->flush_tx();                                      // This maybe avoids a lockup that happens when the FIFO gets full.**************
        CurrentRadio->writeAckPayload(1, &AckPayload, AckPayloadSize); // Send telemetry (actual length plus 0)
        CurrentRadio->read(&CompressedData, sizeof(CompressedData));   // Get Data  
    }
    if (Connected) UseReceivedData();  
    return Connected;
}

/************************************************************************************************************/

void AttachServos()
{
    if (!ServosAttached) {
        for (uint8_t i = 0; i < SERVOSUSED; ++i) {
            MCMServo[i].attach(PWMPins[i]);
        }
        ServosAttached = true;
    }
    MySbus.begin();
}

/************************************************************************************************************/

void BindModel()
{
    ThisPipe = NewPipe;
    if (SaveNewBind) {
        for (uint8_t i = 0; i < 8; ++i) {
            EEPROM.update(i, ReceivedData[i]);
        }
    }
    CurrentRadio->stopListening();
    SetNewPipe();
    BoundFlag   = true;
    BindNow     = 0;
    SaveNewBind = false;
    AttachServos(); // AND START SBUS!!!
#ifdef DB_BIND
    Serial.println("BINDING NOW");
#endif
}

/************************************************************************************************************/

void RebuildFlags(bool* f, uint16_t tb)
{ // Pass arraypointer and the two bytes to be decoded
    for (uint8_t i = 0; i < 16; ++i) {
        f[15 - i] = false;                 // false is default
        if (tb & 1 << i) f[15 - i] = true; // sets true if bit was on
    }
}

/************************************************************************************************************/

/**
 * extra parameters can be sent using the last four bytes in every data packet.
 * the parameter sent is defined by the packet number ... which goes only upto about 5!
 */
void CheckParams()
{ uint16_t TwoBytes = 0;      
    PacketNumber = ReceivedData[CHANNELSUSED];     
    switch (PacketNumber) {     
        case 0:
             BindNow = ReceivedData[CHANNELSUSED + 2];
             FailSafeSave = bool(ReceivedData[CHANNELSUSED + 1]);
                if (FailSafeSave) {
                TwoBytes = uint16_t(FS_byte2) + uint16_t(FS_byte1 << 8);
                RebuildFlags(FailSafeChannel, TwoBytes);
            }
            break;
        case 1:
            FS_byte1 = ReceivedData[CHANNELSUSED + 1]; // These 2 bytes are 16 failsafe flags
            FS_byte2 = ReceivedData[CHANNELSUSED + 2]; // These 2 bytes are 16 failsafe flags
            break;
        case 2:
              Qnh = (ReceivedData[CHANNELSUSED + 1]) << 8; // 16 bits sent as two bytes for pressure here at sea level
              Qnh += ReceivedData[CHANNELSUSED + 2];
        default:
            break;
    }
    return;
}
/************************************************************************************************************/
#ifdef DB_SENSORS
void Sensors_Status()
{
    if (USE_AdafruitUltimateGps){
        Serial.print ("GPS FIX?  ");
        if (GpsFix) {
            Serial.println ("YES");
        } else {
            Serial.println ("No");
        }
        if (GpsFix){
            Serial.print ("GPS Satellites: ");
            Serial.println (SatellitesGPS);
            Serial.print ("GPS Latitude: ");
            Serial.println (LatitudeGPS,14);
            Serial.print ("GPS Longitude: ");
            Serial.println (LongitudeGPS,14);
            Serial.print ("GPS Speed (MPH): ");
            Serial.println (SpeedGPS);
            Serial.print ("GPS Angle: ");
            Serial.println (AngleGPS);
            Serial.print ("GPS distance: ");
            Serial.println (DistanceGPS);
            Serial.print ("GPS course to: ");
            Serial.println (CourseToGPS);
            Serial.print ("GPS Altitude: ");
            Serial.println (AltitudeGPS);
            Serial.print ("Hours GPS: ");
            Serial.println (HoursGPS);
            Serial.print ("Minutes GPS: ");
            Serial.println (MinsGPS);
            Serial.print ("Seconds GPS: ");
            Serial.println (SecsGPS);
            }
    }
    if (USE_INA219) {
        Serial.print("     Volts=");
        Serial.print(INA219Volts);
    }
    if (USE_BMP280) {
        Serial.print("  Altitude=");
        Serial.print(BaroAltitude);
        Serial.print(" Temp=");
        Serial.print(BaroTemperature);
    }
    Serial.println(" ");
}
#endif // defined DB_SENSORS



// ***************************************************************************************************************************************************
// Here the GPS HUB is asked for 11 bytes of data over I2C. 
// The first IDLEN (=3) bytes are the ID (LAT, LNG, etc...)
// The next 8 bytes are the value (as a double).
// The ID changes with each call

FASTRUN void ReadTheNewGPSHub(){
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

  if (strcmp(FIX,RdataID) == 0) {     
      if (int(RdataIn) == 1) {
       GpsFix = true;
       return;
      }
  }
  if (strcmp(LAT,RdataID) == 0) {     
     LatitudeGPS =  RdataIn;
     return;
  }
  if (strcmp(LON,RdataID) == 0) {     
     LongitudeGPS =  RdataIn;
     return;
  }
  if (strcmp(SPD,RdataID) == 0) {     
     SpeedGPS =  RdataIn;
     return;
  }
  if (strcmp(COR,RdataID) == 0) {     
     AngleGPS =  RdataIn;
     return;
  }
  if (strcmp(ALT,RdataID) == 0) {     
     AltitudeGPS =  RdataIn;
     return;
  }
  if (strcmp(DTO,RdataID) == 0) {     
     DistanceGPS =  RdataIn;
     return;
  }
   if (strcmp(SAT,RdataID) == 0) {     
     SatellitesGPS =  uint8_t(RdataIn);
     return;
  }
  if (strcmp(CTO,RdataID) == 0) {     
     CourseToGPS =  uint8_t(RdataIn);
     return;
  }
  if (strcmp(HRS,RdataID) == 0) {     
     HoursGPS =  uint8_t(RdataIn);
     return;
  }
  if (strcmp(MNS,RdataID) == 0) {     
     MinsGPS =  uint8_t(RdataIn);
     return;
  }
  if (strcmp(SEC,RdataID) == 0) {     
     SecsGPS =  uint8_t(RdataIn);
     return;
  }
}
// ***************************************************************************************************************************************************

void  SendToTheNewGPSHub(char m[]){
  Wire.beginTransmission(GPSI2CHUB);   
  Wire.write(m);
  Wire.endTransmission();   
}

/************************************************************************************************************/
FASTRUN void DoSensors()
{      
    if ((millis() - SensorTime) < 1000) return;              // no need to measure too often
        SensorTime = millis();
    
    if (USE_AdafruitUltimateGps) {
            ReadTheNewGPSHub();                             // Sensor now has its own MCU   
    }
    if (USE_BMP280) {
        if (BoundFlag && Connected) {
            BaroTemperature = bmp280.readTemperature();
            BaroAltitude    = bmp280.readAltitude(Qnh) * 3.28084; 
            if (BaroAltitude < 0) BaroAltitude = 0;
        }
    }
    if (USE_INA219) {
        if (BoundFlag && Connected) { 
            INA219Volts = ina219.getBusVoltage_V();
        }
        
    }

#ifdef DB_SENSORS
    Sensors_Status(); // does nothing if DB_SENSORS is not defined
    Serial.println (millis()-SensorTime);
#endif

}
/************************************************************************************************************/

FASTRUN void ReceiveData()
{ 
    if (millis() - LastConnectionMoment < 1 ) { // If we have still loads of time, read the sensors. But not the GPS.
       DoSensors();                             // This must take less than 24 ms to avoid a timeout, or < 7 ms when all's going well.
    }

    Connected = false;                          // Assume not connected until we know better  
    
    if (CurrentRadio->available()) {
        Connected = true;                        // ... ok we now know better ! 
    }
    if (!Connected) {
        if (millis() - LastConnectionMoment >= RECEIVE_TIMEOUT) { // Has transmitter died? 
        Reconnect();                                              // Try to reconnect.
        }
    }
    if (ReadData()) {
        CheckParams();
    }
}
/************************************************************************************************************/
void ScanI2c()
{
    delay(500); // allow time to wake things up
    for (uint8_t i = 1; i < 127; ++i) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            
            if (i == 8) {
                USE_AdafruitUltimateGps = true;
#ifdef DB_SENSORS
                Serial.println("Adafruit Ultimate GPS detected!");
#endif
              }
             if (i == 0x40) {
                USE_INA219 = true;
#ifdef DB_SENSORS
                Serial.println("INA219 voltage meter detected!");
#endif
            }
            else if (i == 0x76) {
                USE_BMP280 = true;
#ifdef DB_SENSORS
                Serial.println("BMP280 barometer detected!");
            }
            else {
                Serial.print(i, HEX);
                Serial.print("   "); // in case some new device shows up
#endif
            }
        }
    }
}
/************************************************************************************************************/
/** Initialize the BMP280 sensor */

void InitBMP280()
{
    bmp280.begin(0x76);
    bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                       Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                       Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                       Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                       Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    
}

/************************************************************************************************************/
void SaveFailSafeData()
{
    // FailSafe data occupies EEPROM from offset 10 to 26
    uint8_t FS_Offset = 10;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        EEPROM.update(i + FS_Offset, (map(ReceivedData[i], MINMICROS, MAXMICROS, 0, 180))); // save servo positions lower res: 8 bits
    }
    FS_Offset += CHANNELSUSED;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        EEPROM.update(i + FS_Offset, FailSafeChannel[i]); // save flags
    }
#ifdef DB_FAILSAFE
    Serial.println("Fail safe settings are saved!");
#endif
    FailSafeSave = false;
}
/************************************************************************************************************/
void DoBinding()
{
    GetNewPipe();
#ifdef DB_BIND
    Serial.print("NewPipe: ");
    Serial.println((int)NewPipe, HEX);

    Serial.print("OldPipe: ");
    Serial.println((int)OldPipe, HEX);
#endif
    if (OldPipe == NewPipe) {
        SaveNewBind = false;

#ifdef DB_BIND
        Serial.println(millis() - BindOKTimer);
#endif
        if (BindOKTimer == 0) {
            BindOKTimer = millis();
        }
        else if ((millis() - BindOKTimer) > 400) { // allow .4 of a second for the TX to bind
            BindNow = 1;
        }
    }
    if (BindNow == 1 && !BoundFlag) {
#ifdef DB_BIND
        Serial.print("Binding to: ");
        Serial.println((int)NewPipe, HEX);
#endif
        BindModel();
    }
}



/************************************************************************************************************/
// SETUP
/************************************************************************************************************/
void setup()
{
 pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    Wire.begin();
    delay(2000); // Needed ! - possibly for stabilising capacitors.
    ScanI2c();   // see what's connected
#ifdef SECOND_TRANSCEIVER
    CurrentRadio = &Radio2;
    if (InitCurrentRadio()) Radio2Exists = true;
#endif
    CurrentRadio = &Radio1;
    if (InitCurrentRadio()) Radio1Exists = true;
    ThisRadio = 1;
    if (USE_INA219)               ina219.begin();
    if (USE_BMP280)               InitBMP280();
   // if (USE_AdafruitUltimateGps)  AdafruitUltimateGpsInit();
    GetOldPipe();
    digitalWrite(LED_PIN, LOW);
}
/************************************************************************************************************/
// LOOP
/************************************************************************************************************/
void loop()
{
    ReceiveData();
    if (BoundFlag && Connected) {
        if (millis() - SBUSTimer >= SBUSRATE) {                  // SBUS rate is also good enough for servo rate
            SBUSTimer = millis();                                // timer starts before send starts....
            if ((millis() - ReconnectedMoment) > RECONNECTGAP) { // Don't send data for 10 ms after reconnect
                MoveServos();
            }
        }
        if (FailSafeSave) SaveFailSafeData();
    }
    else {
        DoBinding();
    }
}