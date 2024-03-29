
// ************************************************** Receiver code **************************************************

/** @file ReceiverCode/src/main.cpp
 * // Malcolm Messiter 2020 - 2023
 * @page RXCODE RecieverCode
 *
 * @section rx Features List
 * - WORKS ON TEENSY 4.0
 * - Detects and uses INA219 to read volts (NOW IN SENSOR HUB as well)
 * - Detects and uses BMP280 pressure sensor for altitude (NOW IN SENSOR HUB)
 * - Binding implemented
 * - SBUS implemented
 * - PPM Implemented on the same pin as SBUS (Serial 3 / Pin 14)
 * - Failsafe implemented (after two seconds)
 * - RESOLUTION INCREASED TO 12 BITS
 * - Channels increased to 16. 9 PWM outputs.  SBUS can handle all. PPM Does 8
 * - Exponential implemented (at TX end)
 * - Sensor Hub added with GPS and more sensors
 * - Supports one or two tranceivers (nRF24L01+)
 *
 * 
 * @section rxpinout TEENSY 4.0 PINS
 * | pin number(s) | purpose |
 * |---------------|---------|
 * | 0...8 | PWM SERVOS Channels 1 - 9 |  (Channels 10 - 16 available via SBUS)  (TODO:the 8 PWM outputs on PCB could be expanded to 11.)
 * | 9     | SPI CE1  (FOR RADIO1) |
 * | 10    | SPI CSN1 (FOR RADIO1)  |
 * | 11    | SPI MOSI (FOR BOTH RADIOS)  |
 * | 12    | SPI MISO (FOR BOTH RADIOS)  |
 * | 13    | SPI SCK  (FOR BOTH RADIOS) |
 * | 14    | SBUS *OR PPM* output (Serial TX3) |
 * | 15    | Don't use if using SBUS. The driver takes it (RX3) |
 * | 16    | RED LED  - This LED is ON when connected, OFF when disconnected and blinking when binding
 * | 17 << | BIND PLUG (held LOW means plug is in)
 * | 18    | I2C SDA (FOR I2C) | *** --- >> BLUE WIRE   = 18 !! << --- ***
 * | 19    | I2C SCK (FOR I2C) | *** --- >> YELLOW WIRE = 19 !! << --- ***
 * | 20    | SPI CSN2 (FOR RADIO2)  |
 * | 21    | SPI CE2 (FOR RADIO2) |
 * | 22    | Spare (CH10 ? ... or CE1)
 * | 23    | Spare (CH11 ? ... or CSN1)
 *
 * @see ReceiverCode/src/main.cpp
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Adafruit_INA219.h>
#include <stdint.h>
#include <EEPROM.h>
#include <Wire.h>

#include <SBUS.h>
#include <PulsePosition.h>
#include <Watchdog_t4.h>
#include "utilities/common.h"
#include "utilities/radio.h"
#include "utilities/pid.h"

void DelayMillis(uint16_t ms) // This replaces any delay() calls
{
    uint32_t tt = millis();
    while (millis() - tt < ms) {
#ifdef USE_STABILISATION
      if (MPU6050Connected) DoStabilsation();
#endif

    }
}   
/************************************************************************************************************/

void LoadFailSafeData()
{
    uint8_t  FS_Offset = FS_EEPROM_OFFSET;
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

void KickTheDog()
{
    if (millis() - LastDogKick >= KICKRATE) {
        TeensyWatchDog.feed();
        LastDogKick = millis();
    }
}


/************************************************************************************************************/

bool CheckCrazyValues()
{ // might come when binding
    for (int i = 0; i < 7; ++i) {
        if ((ReceivedData[i] < MINMICROS) || (ReceivedData[i] > MAXMICROS)) return false;
    }
    return true;
}

/************************************************************************************************************/
// Function to get PWM value needed for given pulse length in microseconds 

int GetPWMValue(int frequency, int length) {return float(length / (1000000.00 / frequency)) * SERVO_RESOLUTION;} // *** >> DONT EDIT THIS LINE!! << ***

/************************************************************************************************************/
void MoveServos()
{
    if (!CheckCrazyValues()||(millis() < 7000)) {
        TurnLedOff();
        for (int j = 0; j < SERVOSUSED; ++j) PreviousData[j] = 0; // Force a send when data is good again
        return;
    }
    else {
        TurnLedOn();
    }

    if (UseSBUS)
    {
        MySbus.write(SbusChannels); // Send SBUS data
    }
    else
    { // not SBUS = PPM
        for (int j = 0; j < PPMChannelCount; ++j) {
            PPMOutput.write(PPMChannelOrder[j], map(ReceivedData[j], MINMICROS, MAXMICROS, 1000, 2000));
        }
    }
    for (int j = 0; j < SERVOSUSED; ++j) {
        if (PreviousData[j] != ReceivedData[j]) { // if same as last time, don't send again.
            int S = ReceivedData[j];
            if (ServoCentrePulse[j] < 1000) {
                S = map(S, MINMICROS, MAXMICROS, ServoCentrePulse[j] - EXTRAAT760, ServoCentrePulse[j] + EXTRAAT760);   // these lines allow for the fact that some servos don't like 760 - 2240
            }else{
                S = map(S, MINMICROS, MAXMICROS, ServoCentrePulse[j] - EXTRAAT1500, ServoCentrePulse[j] + EXTRAAT1500);  // these lines allow for the fact that some servos don't like 760 - 2240
            }
            analogWrite(PWMPins[j],GetPWMValue(ServoFrequency[j], S));
            PreviousData[j] = ReceivedData[j];
        }
    }
}

/************************************************************************************************************/

/** Execute FailSafe data from EEPROM. */
void FailSafe()
{
    if (BoundFlag)
    {
        LoadFailSafeData();
        Connected = true; // to force sending this data!
        MapToSBUS();
        MoveServos();
        Connected = false; // I lied earlier - we're not really connected.
    }
    FailSafeSent = true; // Once is enough
    FailedSafe   = true;
    TurnLedOff();
    MacAddressSentCounter = 0;
}


/************************************************************************************************************/
void SetServoFrequency()
{
     analogWriteResolution(SERVO_RES_BITS);  // 12 Bits for 4096 steps
    for (uint8_t i = 0; i < SERVOSUSED; ++i)
    { 
        analogWriteFrequency(PWMPins[i], ServoFrequency[i]); 
    }

}

/************************************************************************************************************/
void AttachServos()
{
    SetServoFrequency();
    if (UseSBUS) {
        MySbus.begin(); // AND START SBUS
    }
    else {
        PPMOutput.begin(PPMPORT); // Or PPM on same pin
    }
}

/************************************************************************************************************/

void TurnLedOn()
{
    if (!LedIsOn) {
        digitalWrite(LED_RED, HIGH);
        LedIsOn = true;
    }
}

/************************************************************************************************************/

void TurnLedOff()
{
    if (LedIsOn) {
        digitalWrite(LED_RED, LOW);
        LedIsOn = false;
    }
}

/************************************************************************************************************/
// This function binds the model using the TX supplied Pipe instead of the default one.
// If not already saved, this saves it to the eeprom too for next time.

void BindModel()
{
    CurrentRadio->stopListening();
    delayMicroseconds(250);
    BoundFlag    = true;
    ModelMatched = true;
   
    if (Blinking) {
        
        SetNewPipe(); // change to bound pipe <<< ***************************************
       
#ifdef DB_BIND
        Serial.println("SAVING RECEIVED PIPE:");
#endif

        for (uint8_t i = 0; i < 5; ++i) {
            EEPROM.update(i + BIND_EEPROM_OFFSET, TheReceivedPipe[i]);

#ifdef DB_BIND
            Serial.print(TheReceivedPipe[i], HEX);
            Serial.print(" ");
#endif
        }

#ifdef DB_BIND
        Serial.println("");
        Serial.println("TX PIPE SAVED");
#endif
    }
    Blinking = false;

    if (FirstConnection) {
        AttachServos(); // AND START SBUS / PPM
        FirstConnection = false;
    }
    SaveNewBind = false;


#ifdef DB_BIND
        Serial.println("");
        Serial.println("DONE BINDING");
#endif

}

/************************************************************************************************************/
void ReadSavedPipe() // read only 6 bytes
{
    for (uint8_t i = 0; i < 5; ++i) {
        TheReceivedPipe[i] = EEPROM.read(i + BIND_EEPROM_OFFSET); // uses first 5 bytes only.
    }
    TheReceivedPipe[5] = 0;
}

// ***************************************************************************************************************************************************
void SendToSensorHub(char m[])
{
    Wire.beginTransmission(SENSOR_HUB_I2C_ADDRESS);
    Wire.write(m);
    Wire.endTransmission(true);
}
// ***************************************************************************************************************************************************

void MarkHere()
{
    char MRK[4] = "MRK";
    SendToSensorHub(MRK); // Mark this GPS location
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
void SendQnhToSensorHub()
{
    union
    {
        uint16_t Val16;
        uint8_t  Val8[2];
    } Uqnh;
    char QNH[] = "QNH=??";
    Uqnh.Val16 = Qnh;
    QNH[4]     = Uqnh.Val8[0];
    QNH[5]     = Uqnh.Val8[1];
    SendToSensorHub(QNH);
}

/************************************************************************************************************/

template<typename any>
void Look(const any& value) // this is a template function that can print anything but cannot be used to change anything
{
    Serial.println(value);
}

/************************************************************************************************************/
template<typename any>
void Look1(const any& value) // this is a template function that can print anything but cannot be used to change anything
{
    Serial.print(value);
}


// ***************************************************************************************************************************************************
// Here the GPS (Sensor) HUB is asked for 7 bytes of data over I2C.
// The first IDLEN (=3) bytes are the ID (LAT, LNG, etc...)
// The next 4 bytes are the value (as a float).
// The ID changes with each call

FASTRUN void ReadTheSensorHub()
{

#define IDLEN       3
#define GPSI2CBYTES IDLEN + 4 // = 7 (only floats now)

    char  FIX[IDLEN + 1] = "FIX"; // GPS Fix
    char  SAT[IDLEN + 1] = "SAT"; // How many satellites
    char  LAT[IDLEN + 1] = "LAT"; // Latitude
    char  LON[IDLEN + 1] = "LON"; // Longitude
    char  ALT[IDLEN + 1] = "ALT"; // GPS Altitude
    char  SPD[IDLEN + 1] = "SPD"; // Speed
    char  COR[IDLEN + 1] = "COR"; // Course
    char  CTO[IDLEN + 1] = "CTO"; // Course to Mark
    char  DTO[IDLEN + 1] = "DTO"; // Distance to Mark
    char  HRS[IDLEN + 1] = "HRS"; // GMT Hours
    char  MNS[IDLEN + 1] = "MNS"; // GMT Minutes
    char  SEC[IDLEN + 1] = "SEC"; // GMT Seconds
    char  BLT[IDLEN + 1] = "BLT"; // Altitiude from BMP280
    char  TMP[IDLEN + 1] = "TMP"; // Temperature from BMP280
    char  VLT[IDLEN + 1] = "VLT"; // Volts from INA219
    char  DAY[IDLEN + 1] = "DAY"; // DAY
    char  MTH[IDLEN + 1] = "MTH"; // MONTH
    char  YER[IDLEN + 1] = "YER"; // YEAR
    char  RdataID[IDLEN + 1];
    float RdataIn;
    union
    {
        float   Val32;
        uint8_t Val8[4];
    } Rdata; // 'union' allows access to every byte

    Wire.requestFrom(SENSOR_HUB_I2C_ADDRESS, GPSI2CBYTES); // Ask hub for data
    for (int j = 0; j < GPSI2CBYTES; ++j) {
        if (Wire.available()) { // Listen to HUB
            if (j < IDLEN) {
                RdataID[j] = Wire.read(); // This gets the three-char data id (eg LAT)
            }
            else {
                Rdata.Val8[j - IDLEN] = Wire.read(); // This gets the 64 bit value for that data ID
            }
        }
    }
    RdataID[3] = 0;           // To terminate the ID string.
    RdataIn    = Rdata.Val32; // To re-assemble the 64 BIT data to a double

    if (strcmp(FIX, RdataID) == 0) {
        if (int(RdataIn) == 1) {
            GpsFix = true;
            return;
        }
    }
    if (strcmp(LAT, RdataID) == 0) {
        LatitudeGPS = RdataIn;
        return;
    }
    if (strcmp(LON, RdataID) == 0) {
        LongitudeGPS = RdataIn;
        return;
    }
    if (strcmp(SPD, RdataID) == 0) {
        SpeedGPS = RdataIn;
        return;
    }
    if (strcmp(COR, RdataID) == 0) {
        AngleGPS = RdataIn;
        return;
    }
    if (strcmp(ALT, RdataID) == 0) {
        AltitudeGPS = RdataIn;
        return;
    }
    if (strcmp(DTO, RdataID) == 0) {
        DistanceGPS = RdataIn;
        return;
    }
    if (strcmp(SAT, RdataID) == 0) {
        SatellitesGPS = uint8_t(RdataIn);
        return;
    }
    if (strcmp(CTO, RdataID) == 0) {
        CourseToGPS = RdataIn;
        return;
    }
    if (strcmp(HRS, RdataID) == 0) {
        HoursGPS = uint8_t(RdataIn);
        return;
    }
    if (strcmp(MNS, RdataID) == 0) {
        MinsGPS = uint8_t(RdataIn);
        return;
    }
    if (strcmp(SEC, RdataID) == 0) {
        SecsGPS = uint8_t(RdataIn);
        return;
    }
    if (strcmp(BLT, RdataID) == 0) {
        BaroAltitude = RdataIn;
        return;
    }
    if (strcmp(TMP, RdataID) == 0) {
        BaroTemperature = RdataIn;
        return;
    }
    if (strcmp(VLT, RdataID) == 0) {
        if (!INA219Connected) INA219Volts = RdataIn; // if there's a locally connected sensor, use it.
        return;
    }
    if (strcmp(DAY, RdataID) == 0) {
        DayGPS = uint8_t(RdataIn);
        return;
    }
    if (strcmp(MTH, RdataID) == 0) {
        MonthGPS = uint8_t(RdataIn);
        return;
    }
    if (strcmp(YER, RdataID) == 0) {
        YearGPS = uint8_t(RdataIn);
        return;
    }
}

// ******************************************************************************************************************************************************************
void SensorHubHasFailed()
{ // If the I2C bus gets its knickers in a twist, it can lock up the receiver, so DON'T call it until landed and reset.
#define Failed 42
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
    SensorHubDead   = true; // This flag inhibits further attempts to call the hub, which might save a model.
}


// Discover what was connected on I2C

FLASHMEM void ScanI2c()
{
      
    for (uint8_t i = 1; i < 127; ++i) {
        Wire.beginTransmission(i);

        if (Wire.endTransmission() == 0) {
            if (i == SENSOR_HUB_I2C_ADDRESS) {
                SensorHubConnected = true;
#ifdef DB_SENSORS
                Serial.println("Sensor Hub with Adafruit Ultimate GPS etc. detected");
#endif
            }
            if (i == 0x40) {
                INA219Connected = true;
#ifdef DB_SENSORS
                delay(3000);
                Serial.println("INA219 voltage meter detected!");
#endif
            }
            if (i == 0x68) {
                MPU6050Connected = true;
    #ifdef DB_SENSORS
                delay(3000);
                Serial.println("MPU 6050 detected");  
    #endif // DB_SENSORS
            }



        }
    }
}
/************************************************************************************************************/
void SaveFailSafeData()
{
    // FailSafe data occupies EEPROM from offset FS_EEPROM_OFFSET
    uint8_t FS_Offset = FS_EEPROM_OFFSET;

    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        EEPROM.update(i + FS_Offset, (map(ReceivedData[i], MINMICROS, MAXMICROS, 0, 180))); // save servo positions lower res: 8 bits
        DelayMillis(1);
    }
    FS_Offset += CHANNELSUSED;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        EEPROM.update(i + FS_Offset, FailSafeChannel[i]); // save flags
        DelayMillis(1);
    }
#ifdef DB_FAILSAFE
    Serial.println("Fail safe settings are saved!");
#endif
}

/************************************************************************************************************/

void WatchDogCallBack()
{
    // Serial.println("RESETTING ...");
}
/************************************************************************************************************/

void teensyMAC(uint8_t* mac)
{ // GET UNIQUE TEENSY 4.0 ID
    for (uint8_t by = 0; by < 2; by++) mac[by] = (HW_OCOTP_MAC1 >> ((1 - by) * 8)) & 0xFF;
    for (uint8_t by = 0; by < 4; by++) mac[by + 2] = (HW_OCOTP_MAC0 >> ((3 - by) * 8)) & 0xFF;
}

/************************************************************************************************************/

void ReadBindPlug()
{
    uint32_t tt = millis();
    PipePointer = DefaultPipe;
    CopyCurrentPipe(DefaultPipe, PIPENUMBER);
    if (!digitalRead(BINDPLUG_PIN)) { // Bind Plug needed to bind!
        Blinking = true;              // Blinking = binding to new TX
#ifdef DB_BIND
        Serial.println("Bind plug detected.");
#endif
    }
    else {
        Blinking    = false; // Already bound
        PipePointer = TheReceivedPipe;
        CopyCurrentPipe(TheReceivedPipe, BOUNDPIPENUMBER);
        BoundFlag   = true;
        SaveNewBind = false;
        while (millis() - tt < 500) ReceiveData();
        BindModel(); // TODO check this...
    }
}
/************************************************************************************************************/
// SETUP
/************************************************************************************************************/
FLASHMEM void setup()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(pinCSN1, OUTPUT);
#ifdef SECOND_TRANSCEIVER
    pinMode(pinCSN2, OUTPUT);
    pinMode(pinCE2, OUTPUT);
#endif
    pinMode(pinCE1, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(BINDPLUG_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, HIGH);
    TurnLedOff();
    if (digitalRead(BINDPLUG_PIN)) {
        delay(2500); // Needed so that the Sensor hub can boot first and be detected (no bind plug)
    }
    else {
        delay(200);
    }
    
    Wire.begin();
    delay(20);
    ScanI2c(); // Detect what's connected


#ifdef USE_STABILISATION
    if (MPU6050Connected) InitialiseTheMPU6050();
#endif


    if (INA219Connected) ina219.begin();
    teensyMAC(MacAddress);
    PipePointer = DefaultPipe;
    CopyCurrentPipe(DefaultPipe, PIPENUMBER);
    CurrentRadio = &Radio1;
    if (digitalRead(BINDPLUG_PIN)) { // ie no bind plug, so initialise to bound pipe
        GetOldPipe();
    }

#ifdef SECOND_TRANSCEIVER
    digitalWrite(pinCSN2, CSN_OFF);
    digitalWrite(pinCE2, CE_OFF);
#endif
    digitalWrite(pinCSN1, CSN_ON);
    digitalWrite(pinCE1, CE_ON);
    delay(4);
    InitCurrentRadio();
    ThisRadio = 1;

#ifdef SECOND_TRANSCEIVER
    CurrentRadio = &Radio2;
    digitalWrite(pinCSN1, CSN_OFF);
    digitalWrite(pinCE1, CE_OFF);
    digitalWrite(pinCSN2, CSN_ON);
    digitalWrite(pinCE2, CE_ON);
    delay(4);
    InitCurrentRadio();
    ThisRadio = 2;
#endif


    WatchDogConfig.window   = WATCHDOGMAXRATE; //  = MINIMUM RATE in milli seconds, (32ms to 522.232s) must be MUCH smaller than timeout
    WatchDogConfig.timeout  = WATCHDOGTIMEOUT; //  = MAX TIMEOUT in milli seconds, (32ms to 522.232s)
    WatchDogConfig.callback = WatchDogCallBack;
    TeensyWatchDog.begin(WatchDogConfig);
    KickTheDog();
    ReadBindPlug();
    digitalWrite(LED_PIN, LOW);
}

/************************************************************************************************************/

void BlinkLed()
{
    uint16_t Blinkrate = 222;
    if (MPU6050Connected)   Blinkrate = 666; // Blink rate is reduced if MPU6050 is connected
    if ((millis() - BlinkTimer) >= Blinkrate) {
        BlinkTimer = millis(); if (BlinkValue ^= 1) TurnLedOn(); else TurnLedOff();
    }
}

/************************************************************************************************************/
// LOOP
/************************************************************************************************************/

void loop()
{
#ifdef USE_STABILISATION
    if (MPU6050Connected)  DoStabilsation();
#endif

    KickTheDog();
    ReceiveData();
    if (Blinking) BlinkLed();
    if (BoundFlag && Connected && ModelMatched) { // Only move servos if everything is good
        if (millis() - SBUSTimer >= SBUSRATE) {   // SBUSRATE rate is also good enough for servo rate
            SBUSTimer = millis();                 // timer starts before send starts....
            MoveServos();                         // Actually do something useful at last
        }
    }
    else {
    if (!BoundFlag)  {
            GetNewPipe();
        }
    }
}