
// ************************************************** Receiver code **************************************************

/** @file ReceiverCode/src/main.cpp
 * // Malcolm Messiter 2022
 * @page RXCODE RecieverCode
 *
 * @section rxFeatures Features List
 * - WORKS ON TEENSY 4.0
 * - Detects and uses INA219 to read volts (NOW IN SENSOR HUB as well)
 * - Detects and uses BMP280 pressure sensor for altitude (NOW IN SENSOR HUB)
 * - Binding implemented
 * - SBUS implemented
 * - Failsafe implemented (after two seconds)
 * - RESOLUTION INCREASED TO 12 BITS
 * - Channels increased to 16, 9 PWM outputs.  SBUS can handle all.
 * - Exponential implemented (at TX end)
 * - Sensor Hub added with GPS and more sensors
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
 * | 16    | SPARE
 * | 17    | SPARE
 * | 18    | I2C SDA (FOR I2C) |
 * | 19    | I2C SCK (FOR I2C) |
 * | 20    | SPI CSN2 (FOR RADIO2)  |
 * | 21    | SPI CE2 (FOR RADIO2) |
 * | 22    | IRQ1 (FOR RADIO1) |
 * | 23    | IRQ2 (FOR RADIO2) |
 *
 * @see ReceiverCode/src/main.cpp
 */
#include "utilities/radio.h"
Adafruit_INA219 ina219;
bool            SENSOR_HUB_CONNECTED = false;      //  GPS (Adafruit Ultimate GPS) ?
Servo           MCMServo[SERVOSUSED];
uint8_t         PWMPins[SERVOSUSED] = {0, 1, 2, 3, 4, 5, 6, 7, 8}; // 9 PWMs, remaining 7 via sbus
SBUS            MySbus(SBUSPORT);
float           PacketStartTime;
uint8_t         BindNow        = 0;     /** indicates that the receiver should start the binding/pairing process */
bool            BoundFlag      = false; /** indicates if receiver paired with transmitter */
int             BindOKTimer    = 0;
bool            ServosAttached = false;
uint16_t        SbusChannels[CHANNELSUSED + 1]; // Just one spare
uint32_t        SBUSTimer = 0;
bool            FailSafeChannel[17];
bool            FailSafeDataLoaded = false;
bool            ReInit             = false;
uint8_t         FS_byte1           = 0; // All 16 failsafe channel flags are in these two bytes
uint8_t         FS_byte2           = 0;
uint32_t        ReconnectedMoment;
uint16_t        BaroAltitude;
float           BaroTemperature;
float           INA219Volts = 0;
uint32_t        SensorTime       = 0;
uint32_t        SensorHubAccessed    = 0;
uint16_t        Qnh              = 0;    //Pressure at sea level here and now (defined at TX)
uint16_t        OldQnh           = 0;
uint8_t         SatellitesGPS; 
float           LatitudeGPS;
float           LongitudeGPS;
float           SpeedGPS;
float           AngleGPS;
bool            GpsFix = false;
float           AltitudeGPS;
float           DistanceGPS;
float           CourseToGPS;
uint8_t         DayGPS;
uint8_t         MonthGPS;
uint8_t         YearGPS;
uint8_t         HoursGPS;
uint8_t         MinsGPS;
uint8_t         SecsGPS;
uint16_t        CompressedData[COMPRESSEDWORDS]; // 30 bytes -> 40 bytes when uncompressed
bool            SensorHubDead = false;
uint32_t        BootupMoment  = 0;
bool            QNHSent       = false;
bool            FirstLostPacket = true;

/************************************************************************************************************/

void LoadFailSafeData()
{
    uint8_t  FS_Offset = 10;
    uint16_t s[CHANNELSUSED];

    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        s[i] = map(EEPROM.read(i + FS_Offset), 0, 180, MINMICROS, MAXMICROS); // load failsafe values and simulate better resolution
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
            SbusChannels[j] = static_cast<uint16_t> (map(ReceivedData[j], MINMICROS, MAXMICROS, RANGEMIN, RANGEMAX));
        }
    }
}

/************************************************************************************************************/

void MoveServos()
{
    if (!Connected) return;                         // Avoid sending rubbish
    MySbus.write(SbusChannels);                     // Send SBUS data
    for (int j = 0; j < SERVOSUSED; ++j) {
        if (PreviousData[j] != ReceivedData[j]) {   // if same as last time, don't send again.
            MCMServo[j].writeMicroseconds(ReceivedData[j]);
            PreviousData[j] = ReceivedData[j];
        }
    }
}

/************************************************************************************************************/

/** Execute FailSafe data from EEPROM. */
void FailSafe(){
    if (BoundFlag) {
        LoadFailSafeData();
        Connected = true; // to force sending this data!
        MapToSBUS();
        MoveServos();
        Connected = false; // I lied earlier - we're not really connected.
    }
    FailSafeSent = true;                                        // Once is enough
    SbusRepeats = 0;                                            // Reset this count for next connection
    SetUKFrequencies();                                         // In case this had been changed
}

#ifdef DB_FHSS
/************************************************************************************************************/
/*
 * Print out some FHSS information about the channel hopping implementation
 */
void ShowHopDurationEtc() 
{
    PacketNumber+=2; // ?
    float freq = 2.4 + (float) NextChannel/1000;
    uint8_t OnePacketTime = (millis() - PacketStartTime) / PacketNumber;
    Serial.print("Hop duration: ");
    Serial.print(int (millis() - PacketStartTime));
    Serial.print("ms.  Packets per hop: ");
    Serial.print(PacketNumber);
    Serial.print("  Average Time per packet: ");
    Serial.print(OnePacketTime);
    Serial.print("ms.  Next frequency: ");
    Serial.print(freq);
    Serial.print(BoundFlag ? " Bound!" : " NOT Bound");
    Serial.print("  Radio: ");
    Serial.print(ThisRadio);
    Serial.println("");
    PacketStartTime = millis();
}
#endif

/************************************************************************************************************/
void UseReceivedData(){
        Decompress(ReceivedData, CompressedData, UNCOMPRESSEDWORDS);   // Decompress only the most recent data
        MapToSBUS();                        // Get SBUS data ready 
        LastPacketArrivalTime = millis();   // Note the arrival time
        if (HopNow) {                       // This flag gets set in LoadAckPayload();
            HopToNextChannel();             // Ack payload instructed us to Hop at next opportunity. So hop now ...
            HopNow = false;                 // ... and clear the flag,
            HopStart = millis();            // ... and start the timer.
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
        CurrentRadio->writeAckPayload(1, &AckPayload, AckPayloadSize); // Send telemetry
        CurrentRadio->read(&CompressedData, sizeof(CompressedData));   // Get Data  
        CurrentRadio->flush_rx();                                      // Flush FIFO to avoid a lock up
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
    MySbus.begin();   // AND START SBUS!!!
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
    delayMicroseconds(250);
    SetNewPipe();
    BoundFlag   = true;
    BindNow     = 0;
    SaveNewBind = false;
    AttachServos(); // AND START SBUS!!!
#ifdef DB_BIND
    Serial.println("BINDING NOW");
#endif
}
// ***************************************************************************************************************************************************
void  SendToSensorHub(char m[]){
  Wire.beginTransmission(SENSOR_HUB_I2C_ADDRESS);   
  Wire.write(m);
  Wire.endTransmission(true);   
}
// ***************************************************************************************************************************************************

void MarkHere(){
        char MRK[4] = "MRK";
        SendToSensorHub(MRK);  // Mark this GPS location  
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
void SendQnhToSensorHub(){ 
  union {uint16_t Val16; uint8_t Val8[2];} Uqnh;  
        char QNH[] = "QNH=??";
        Uqnh.Val16 = Qnh;
        QNH[4] = Uqnh.Val8[0];    
        QNH[5] = Uqnh.Val8[1];  
        SendToSensorHub(QNH);   
}
/************************************************************************************************************/

// This allows a new array of pseudo-random channel numbers to be used. 
// "FHSSChPointer" and "FrequencyCount" simply need to be set appropriately.

void SetTestFrequencies(){
   
    FHSSChPointer  = FHSS_Channels1; 
    FrequencyCount = FREQUENCYSCOUNT1;
}
/************************************************************************************************************/

// This allows a new array of pseudo-random channel numbers to be used. 
// "FHSSChPointer" and "FrequencyCount" simply need to be set appropriately.

void SetUKFrequencies(){
    
    FHSSChPointer  = FHSS_Channels;     
    FrequencyCount = FREQUENCYSCOUNT;
}
/************************************************************************************************************/
/**
 * extra parameters can be sent using the last four bytes in every data packet.
 * the parameter sent is defined by the packet number ... which goes only upto about 5
 * 
 * Note: If extra parameters are needed, the "HOPTIME" duration can be increased.
 * It's 50ms right now, which gives about 7 packets between hops.
 */
void ReadExtraParameters()
{ uint16_t TwoBytes = 0; 
  uint8_t SwapWaveBand;
       
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
                if (OldQnh != Qnh) SendQnhToSensorHub();
                OldQnh = Qnh; // Send new one once only
                break;
         case 3: 
                if  ((ReceivedData[CHANNELSUSED + 2]) == 255) {    // Mark this location
                    MarkHere();
                    ReceivedData[CHANNELSUSED + 2] = 0;             // ... Once only
                }
                break;
         case 4:
               SwapWaveBand =  ReceivedData[CHANNELSUSED + 2] ;
               if (SwapWaveBand > 0 ) {
                    if (SwapWaveBand == 1) SetUKFrequencies();
                    if (SwapWaveBand == 2) SetTestFrequencies();
               } 
        default:
                break;
    }
    return;
}
// ***************************************************************************************************************************************************
// Here the GPS HUB is asked for 7 bytes of data over I2C. 
// The first IDLEN (=3) bytes are the ID (LAT, LNG, etc...)
// The next 4 bytes are the value (as a float).
// The ID changes with each call

FASTRUN void ReadTheSensorHub(){

  #define IDLEN 3
  #define GPSI2CBYTES IDLEN + 4   // = 7 (only floats now)
  
  char  FIX[IDLEN+1]    = "FIX";  // GPS Fix 
  char  SAT[IDLEN+1]    = "SAT";  // How many satellites
  char  LAT[IDLEN+1]    = "LAT";  // Latitude
  char  LON[IDLEN+1]    = "LON";  // Longitude
  char  ALT[IDLEN+1]    = "ALT";  // GPS Altitude
  char  SPD[IDLEN+1]    = "SPD";  // Speed
  char  COR[IDLEN+1]    = "COR";  // Course
  char  CTO[IDLEN+1]    = "CTO";  // Course to Mark
  char  DTO[IDLEN+1]    = "DTO";  // Distance to Mark
  char  HRS[IDLEN+1]    = "HRS";  // GMT Hours  
  char  MNS[IDLEN+1]    = "MNS";  // GMT Minutes  
  char  SEC[IDLEN+1]    = "SEC";  // GMT Seconds
  char  BLT[IDLEN+1]    = "BLT";  // Altitiude from BMP280
  char  TMP[IDLEN+1]    = "TMP";  // Temperature from BMP280
  char  VLT[IDLEN+1]    = "VLT";  // Volts from INA219
  char  DAY[IDLEN+1]    = "DAY";  // DAY
  char  MTH[IDLEN+1]    = "MTH";  // MONTH
  char  YER[IDLEN+1]    = "YER";  // YEAR
  char  RdataID[IDLEN+1];
  float RdataIn;
  union {float Val32; uint8_t Val8[4];} Rdata;      // 'union' allows access to every byte

  Wire.requestFrom(SENSOR_HUB_I2C_ADDRESS, GPSI2CBYTES);         // Ask hub for data
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
  RdataIn = Rdata.Val32;                            // To re-assemble the 64 BIT data to a double

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
     CourseToGPS =  RdataIn;
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
  if (strcmp(BLT,RdataID) == 0) {     
     BaroAltitude =  RdataIn;
     return;
  }
  if (strcmp(TMP,RdataID) == 0) {     
     BaroTemperature =  RdataIn;
     return;
  }
  if (strcmp(VLT,RdataID) == 0) {     
      if (!INA219_CONNECTED) INA219Volts = RdataIn; // if there's a locally connected sensor, use it.
     return;
  }
  if (strcmp(DAY,RdataID) == 0) {     
     DayGPS = uint8_t(RdataIn);
     return;
  }
   if (strcmp(MTH,RdataID) == 0) {     
     MonthGPS = uint8_t(RdataIn);
     return;
  }
   if (strcmp(YER,RdataID) == 0) {     
    YearGPS = uint8_t(RdataIn);
    return;
  }
}

// ******************************************************************************************************************************************************************
void SensorHubHasFailed(){       // If the I2C bus gets its knickers in a twist, it can lock up the reciever, so DON'T call it until landed and reset.
#define Failed  42
     LatitudeGPS     = Failed; 
     LongitudeGPS    = Failed;
     SpeedGPS        = Failed;
     AngleGPS        = Failed;    
     AltitudeGPS     = Failed;
     DistanceGPS     = Failed;   
     SatellitesGPS   = Failed;
     CourseToGPS     = Failed;
     HoursGPS        = Failed;
     MinsGPS         = Failed;
     SecsGPS         = Failed;
     BaroAltitude    = Failed;
     BaroTemperature = Failed;   
     INA219Volts     = 0;
     GpsFix          = 0;
     SensorHubDead   = true;    // This flag inhibits further attempts to call the hub, which might save a model.
}

// ******************************************************************************************************************************************************************
FASTRUN void TryNextChannel(){
        ++NextChannelNumber;                      // Move up the channels' array
        if (NextChannelNumber >= FrequencyCount)  NextChannelNumber = 1; // If needed, wrap the channels' array pointer
        NextChannel =  * (FHSSChPointer + NextChannelNumber);
        HopToNextChannel();
        FirstLostPacket = false;
}
// ******************************************************************************************************************************************************************
FASTRUN void ReceiveData(){ 
    uint32_t TimeTest; 
    if (Connected) {
      if ((millis() - SensorHubAccessed) > 10){                                         //  Reading Sensor hub 100 x per second should be enough
        if (millis() - LastPacketArrivalTime < 1 ) {                                    //  If, and only if, we have still absolutely loads of time, do stuff now while waiting ...           
             SensorHubAccessed = millis();                                              //  Note the moment of last attempted read. 
             if (!SensorHubDead){                                                       //  Better check it hasn't died.
                TimeTest =  millis();                                                   //  Time the I2C calls. If too long, don't repeat it ... save the model.                                                       
                if (SENSOR_HUB_CONNECTED)       ReadTheSensorHub();                     //  Sensor now has its own MCU. Calls return in far less that 6 ms unless it lost I2C synch  
                if (INA219_CONNECTED)           INA219Volts = ina219.getBusVoltage_V(); //  Get RX LIPO volts if connected separately (as will be needed on 'planes with no GPS fitted.)  
                if ((millis() - BootupMoment) > 5000) {
                        if ((millis() - TimeTest) > 6)  SensorHubHasFailed();           //  If sensor hub and/or INA219 fails, don't bother calling either again (It normally returns within 2 ms.    
                }
            } 
         }
      }
    }
    if (ReadData()) {
       // if (!FirstLostPacket) Serial.println ("Hooray!!");                           // Proves it worked!! 
        ReadExtraParameters();                                                         // Check the extra parameters
        FirstLostPacket = true;                                                        // it will be when one is lost!
    } else {        
        if (millis() - SBUSTimer >= SBUSRATE) {                                        // No new packet yet - but maybe it's time to dispatch the last?
            if (BoundFlag && (millis() > 10000)) {
                if (Connected) {
                    KeepSbusHappy();                                                    // if it's time - send a SBUS packet. It might be new data.
                    -- SbusRepeats;                                                     // It's not really a "repeat".
                }
            }                                          
        }                                                                                                                                           
    if (millis() - LastPacketArrivalTime >= RECEIVE_TIMEOUT) {                      
        if (FirstLostPacket) {
                 TryNextChannel();
            } else {
                 Reconnect();
            }                                                                           // Try to reconnect.
        } 
    }
}
/************************************************************************************************************/

// Discover what was connected on I2C

void ScanI2c(){
     
    for (uint8_t i = 1; i < 127; ++i) {
        Wire.beginTransmission(i);
       
        if (Wire.endTransmission() == 0) {
            if (i == SENSOR_HUB_I2C_ADDRESS) {
                SENSOR_HUB_CONNECTED = true;
#ifdef DB_SENSORS
                Serial.println("Sensor Hub with Adafruit Ultimate GPS etc. detected!");
#endif
            }
            if (i == 0x40) {
                INA219_CONNECTED = true;
#ifdef DB_SENSORS
                Serial.println("INA219 voltage meter detected!");
#endif
            }
        }
    }
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
    pinMode(pinCSN1, OUTPUT);
    pinMode(pinCSN2, OUTPUT);
    pinMode(pinCE1,  OUTPUT);
    pinMode(pinCE2,  OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    delay(2500);                           // Needed so that the Sensor hub can boot first and be detected
    CurrentRadio = &Radio1;
    digitalWrite(pinCSN2,CSN_OFF);
    digitalWrite(pinCE2, CE_OFF);
    digitalWrite(pinCSN1,CSN_ON);
    digitalWrite(pinCE1, CE_ON);
    delay(4);
    InitCurrentRadio();
    ThisRadio = 1;
    Wire.begin();
    delay(20); 
    ScanI2c();                             // Detect what's connected
    if (INA219_CONNECTED) ina219.begin();
#ifdef SECOND_TRANSCEIVER
    CurrentRadio = &Radio2;
    digitalWrite(pinCSN1,CSN_OFF);
    digitalWrite(pinCE1, CE_OFF);
    digitalWrite(pinCSN2,CSN_ON);
    digitalWrite(pinCE2, CE_ON);
    delay(4);
    InitCurrentRadio();
    ThisRadio = 2;
#endif
    GetOldPipe();
    BootupMoment = millis();
    SetUKFrequencies();
    digitalWrite(LED_PIN, LOW);
    ReconnectedMoment = millis();
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
            if ((millis() - ReconnectedMoment) > RECONNECTGAP) { // Don't send data for 25 ms after reconnect
                MoveServos();
            }
        }
        if (FailSafeSave) SaveFailSafeData();
    }
    else {
        DoBinding();
    }
}