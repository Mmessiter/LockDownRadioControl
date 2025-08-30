/** @file ReceiverCode/src/utilities/1Definitions.h */
// Malcolm Messiter 2020 - 2025 ...
#ifndef _SRC_UTILITIES_1DEFINITIONS_H
#define _SRC_UTILITIES_1DEFINITIONS_H

#include <Arduino.h>
#include <Adafruit_DPS310.h>
#include <EEPROM.h>

#define RXVERSION_MAJOR 2
#define RXVERSION_MINOR 5
#define RXVERSION_MINIMUS 4
#define RXVERSION_EXTRA 'D' // 30 August 2025
#define HOPTIME 15          // gives about 48Hz FHSS, 47 gives 20Hz FHSS
#define RECEIVE_TIMEOUT 6   // 6 at 200 Hz  can be 3 or 4 at 500 Hz

// **************************************************************************

//  #define DB_FHSS
//  #define DB_SENSORS
//  #define DB_BIND
//  #define DB_FAILSAFE
//  #define DB_RXTIMERS

// >>>>>>>>>>>>>>>>>******* DON'T FORGET TO SET THIS LOT !!! ******* <<<<<<<<<<<<<<<<<<<<< **** <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//#define USE_NEXUS // if DEFINED then USE_PWM must be UNDEFINED ( = commented out) and USE_SBUS must be DEFINED ( = uncommented) to use the Nexus transceiver

#define SECOND_TRANSCEIVER // must be UNDEFINED ( = commented out) if using ONE transceiver but DEFINED if using TWO transceivers!
 #define USE_11PWM_OUTPUTS  // must be UNDEFINED ( = commented out) if NOT using all 11 PWM outputs (i.e. older rxs with only 8 outputs) but DEFINED if using all 11 PWM outputs!
#define USE_SBUS

#ifndef USE_NEXUS
#define USE_PWM
#endif // USE_NEXUS

// **************************************************************************
#define NEXUS_SERIAL_TELEMETRY Serial1
#define SERVO_RES_BITS 12
#define SERVO_RESOLUTION 4096

#define EXTRAAT1500 1000
#define MINMICROS 500 // normal servos
#define MAXMICROS 2500

#define EXTRAAT760 350 // high frequency servos
#define MAXAT760 760 + EXTRAAT760
#define MINAT760 760 - EXTRAAT760

// **************************************************************************
//                            WATCHDOG PARAMETERS                           *
//***************************************************************************

#define WATCHDOGTIMEOUT 3500 // 2 Seconds before reboot (250ms -> 3500 ms)
#define KICKRATE 500         // Kick twice a second (must be between WATCHDOGMAXRATE and WATCHDOGTIMEOUT)
#define WATCHDOGMAXRATE 250  // 250 ms secs between kicks is max rate allowed
// **************************************************************************
//                            EEPROM PARAMETERS                           *
//***************************************************************************

#define BIND_EEPROM_OFFSET 0                          // use 8 bytes from here (in fact 5 bytes only, but we reserve 8 bytes for future use)
#define FAILSAFE_EEPROM_OFFSET BIND_EEPROM_OFFSET + 8 // use 32 bytes from here. Not 16 as had been rumoured ...

// ********************* >>> Reconnect params <<< ***************************************

#define LISTEN_PERIOD 11       //  was 14 (How many ms to listen for TX in Reconnect())
#define STOPLISTENINGDELAY 100 // was 30 microseconds to wait after stopListening() in Reconnect()

// *************************************************************************************

#define DATARATE RF24_250KBPS // RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
#define PIPENUMBER 1
#define BOUNDPIPENUMBER 1
#define MAX_TELEMETERY_ITEMS 20 // Max number of telemetry items to send... 1 per packet
#define CHANNELSUSED 16         // Number of channels used
#define RECEIVEBUFFERSIZE 20

struct CD
{
    uint16_t ChannelBitMask = 0;
    uint16_t CompressedData[10]; // 20 bytes ... far too big
};
CD DataReceived;

uint16_t SizeOfDataReceived = sizeof(DataReceived);

struct CD2
{
    uint16_t ID = 0;
    uint16_t word[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // The 0th word isn't used, yet ...
};

CD2 Parameters;
uint8_t SizeOfParameters = sizeof(Parameters);

#define FREQUENCYSCOUNT 83 // uses 83 different channels

#ifdef USE_11PWM_OUTPUTS
#define SERVOSUSED 11
#else
#define SERVOSUSED 9 // But all 16 are available via SBUS
#endif

#define SBUSRATE 10      // SBUS frame every 10 milliseconds
#define SBUSPORT Serial3 // = 14
#define SBUSPIN 14       //
#define MINMICROS 500
#define MAXMICROS 2500
#define LED_PIN LED_BUILTIN
#define LED_RED 16
#define BINDPLUG_PIN 17
#define RANGEMAX 2047 // = Frsky at 150 %
#define RANGEMIN 0

#ifdef USE_11PWM_OUTPUTS
#define pinCE1 22  // NRF1
#define pinCSN1 23 // NRF1
#else
#define pinCE1 9   // NRF1
#define pinCSN1 10 // NRF1
#endif

#define pinCSN2 20            // NRF2
#define pinCE2 21             // NRF2
#define FAILSAFE_TIMEOUT 1500 //
#define CSN_ON LOW
#define CSN_OFF HIGH
#define CE_ON HIGH
#define CE_OFF LOW

#define PIPES_TO_COMPARE 8

// **************************************************************************
//                             Parameters' IDs                              *
// **************************************************************************

// Parameter ID definitions.
#define FAILSAFE_SETTINGS 1
#define QNH_SETTING 2
#define GPS_MARK_LOCATION 3
#define SERVO_FREQUENCIES 6
#define SERVO_PULSE_WIDTHS 7
#define GEAR_RATIO 8
#define PARAMETERS_MAX_ID 12 // Max types of parameters packet to send  ... will increase.

// ****************************************************************************************************************************************

RF24 Radio1(pinCE1, pinCSN1);
RF24 Radio2(pinCE2, pinCSN2);
RF24 *CurrentRadio = &Radio1;

bool Connected = false;
bool HopNow = false;
uint8_t ThisRadio = 1;
uint8_t SavedPipeAddress[8];
uint8_t NextChannelNumber = 0;
uint8_t NextChannel;
uint8_t ReconnectIndex = 0;
uint8_t PacketNumber;
uint16_t RawDataIn[RECEIVEBUFFERSIZE + 1];    //  21 x 16 BIT words // lots of spare space
uint16_t ReceivedData[RECEIVEBUFFERSIZE + 1]; //  21 x 16 BIT words// lots of spare space//
uint16_t Interations = 0;
uint32_t HopStart;
uint64_t NewPipeMaybe = 0;
uint64_t PreviousNewPipes[PIPES_TO_COMPARE];
uint8_t PreviousNewPipesIndex = 0;
bool FailSafeSent = true;
uint32_t RX1TotalTime = 0;
uint32_t RX2TotalTime = 0;
uint32_t RadioSwaps = 0;
uint32_t LastPacketArrivalTime = 0;
bool INA219Connected = false;  //  Volts from INA219 ?
bool MPU6050Connected = false; //  Accelerometer and Gyro from MPU6050 ?
uint8_t ReconnectChannel = 0;
int32_t RateOfClimb = 0;
uint32_t RotorRPM = 0;
float Ratio = 0;
bool ForceCalibration = false; // Force calibration of MPU6050

char ParaNames[12][30] = {
    "FailSafe positions",  // 1
    "QNH",                 // 2
    "Mark Location",       // 3
    "PID Values",          // 4
    "Kalman Values",       // 5
    "Servo Frequencies",   // 6
    "Servo Pulse Widths",  // 7
    "Alpha & beta Values", // 8
    "Booleans",            // 9
    "Re-calibtrate MPU",   // 10
    "Tail PID Values"      // 11
};
/** AckPayload Stucture for data returned to transmitter. */
struct Payload
{
    /**
     * This first byte "Purpose" defines what all the other bytes mean, AND ...
     * the highest BIT of Purpose means ** HOP TO NEXT CHANNEL A.S.A.P. (IF ON) **
     * the lower 7 BITs then define the meaning of the remainder of the ackpayload bytes
     **/
    uint8_t Purpose = 0; // 0  Purpose
    uint8_t Byte1 = 0;   // 1
    uint8_t Byte2 = 0;   // 2
    uint8_t Byte3 = 0;   // 3
    uint8_t Byte4 = 0;   // 4
    uint8_t Byte5 = 0;   // 5
};
Payload AckPayload;
uint8_t AckPayloadSize = sizeof(AckPayload); // Size for later externs if needed etc. (=6)

// ************************************************************************************************************/

uint8_t FHSS_Recovery_Channels[3] = {15, 71, 82};                                                                               // three possible channels used for Recovery
uint8_t FHSS_Channels[83] = {51, 28, 24, 61, 64, 55, 66, 19, 76, 21, 59, 67, 15, 71, 82, 32, 49, 69, 13, 2, 34, 47, 20, 16, 72, // These are good for UK
                             35, 57, 45, 29, 75, 3, 41, 62, 11, 9, 77, 37, 8, 31, 36, 18, 17, 50, 78, 73, 30, 79, 6, 23, 40,
                             54, 12, 80, 53, 22, 1, 74, 39, 58, 63, 70, 52, 42, 25, 43, 26, 14, 38, 48, 68, 33, 27, 60, 44, 46,
                             56, 7, 81, 5, 65, 4, 10, 1};
uint8_t *FHSSChPointer = FHSS_Channels; // Pointer for FHSS channels' array
bool PipeSeen = false;

uint16_t ServoCentrePulse[11] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // 11 channels for servo centre pulse
uint16_t ServoFrequency[11] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};                         // 11 channels for servo frequency

/************************************************************************************************************/

void HopToNextChannel();
void DelayMillis(uint16_t ms);
void ReadExtraParameters();
FASTRUN void Reconnect();
void LoadLongerAckPayload();
void Decompress(uint16_t *uncompressed_buf, uint16_t *compressed_buf, uint8_t uncompressed_size);
void RebuildFlags(bool *f, uint16_t tb);
void MarkHere();
void KickTheDog();
void BindModel();
void LoadSavedPipeFromEEPROM(); // read only 6 bytes
void MoveServos();
void BlinkLed();
void FailSafe();
void TurnLedOff();
void TurnLedOn();
void SaveFailSafeDataToEEPROM();
void IncChannelNumber();
#ifdef USE_PWM
void SetServoFrequency();
#endif

float MetersToFeet(float Meters);
void GetRXVolts();
#ifdef USE_SBUS
void SendSBUSData();
#endif
bool CheckForCrazyValues();
void ReadGPS();
FASTRUN void ReceiveData();
void CopyToCurrentPipe(uint8_t *p, uint8_t pn);
void SetNewPipe();
void UnbindModel();
void AttachServos();
void LoadFailSafeDataFromEEPROM();
void SaveFailSafeDataToEEPROM();
void SavePipeToEEPROM();

/************************************************************************************************************/
// For numeric types (int, float, double, etc.)
template <typename T>
void Look(const T &value, int format)
{
    Serial.println(value, format);
}

template <typename T>
void Look1(const T &value, int format)
{
    Serial.print(value, format);
}

// Fallback for types where a format doesn't apply (e.g., String, const char*)
template <typename T>
void Look(const T &value)
{
    Serial.println(value);
}

template <typename T>
void Look1(const T &value)
{
    Serial.print(value);
}

Adafruit_INA219 ina219;
Adafruit_BMP280 bmp;
Adafruit_DPS310 dps310;

#ifdef USE_PWM
//           Channels: 1  2 [3][4] 5  6 {7} 8  9 {10} 11 (Channels 3+4 & 7+10 must have same frquency)
uint8_t PWMPins[11] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}; // if only SERVOSUSED = 9 then last two are ignored
#endif

#ifdef USE_SBUS
SBUS MySbus(SBUSPORT); // SBUS
#endif

bool BoundFlag = false;                  /** indicates if receiver paired with transmitter */
uint16_t SbusChannels[CHANNELSUSED + 1]; // Just one spare
bool FailSafeChannel[17];
bool FailSafeDataLoaded = false;
uint8_t FS_byte1 = 0; // All 16 failsafe channel flags are in these two bytes
uint8_t FS_byte2 = 0;
uint32_t ReconnectedMoment;
float BaroAltitude;
float BaroTemperature;
float INA219Volts = 0;
uint32_t SensorTime = 0;
uint32_t SensorHubAccessed = 0;
float Qnh = 1079.00; // Pressure at sea level here and now (defined at TX)
uint8_t SatellitesGPS;
float LatitudeGPS;
float LongitudeGPS;
float SpeedGPS;
float AngleGPS;
bool GpsFix = false;
float AltitudeGPS;
float DistanceGPS;
float CourseToGPS;
uint8_t DayGPS;
uint8_t MonthGPS;
uint8_t YearGPS;
uint8_t HoursGPS;
uint8_t MinsGPS;
uint8_t SecsGPS;
float StoredLatitudeGPS = 51.922291;  // haverfordwest!
float StoredLongitudeGPS = -5.213110; // haverfordwest!
bool GPS_Connected = false;
bool BMP280Connected = false;
uint32_t NewConnectionMoment = 0;
bool QNHSent = false;
uint8_t MacAddress[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t TheReceivedPipe[6];
uint8_t TheSavedPipe[6];
uint8_t TheCurrentPipe[6];
bool FirstConnection = true;
bool FailedSafe = true; // Starting up as the same as after failsafe
bool NewData = false;
uint16_t pcount = 0; // how many pipes so far received from TX
bool Blinking = false;
uint8_t BlinkValue = 1;
uint32_t BlinkTimer = 0;
uint8_t MacAddressSentCounter = 0;
WDT_T4<WDT3> TeensyWatchDog;
WDT_timings_t WatchDogConfig;
uint32_t LastDogKick = 0;
bool LedIsOn = false;
uint8_t *PipePointer;
uint8_t Pipnum = PIPENUMBER;
uint8_t DefaultPipe[6] = {0x23, 0x94, 0x3e, 0xbe, 0xb7, 0x00};
uint8_t CurrentPipe[6];
uint8_t Randomized_Recovery_Channels_Counter = 0;
uint32_t HopMoment = 0;
float X_GyroOffset = 0;
float Y_GyroOffset = 0;
float Z_GyroOffset = 0;
bool GyroOffsetsSet = false;
uint16_t BMP280Address = 0x76; // BMP280 I2C address
uint32_t SuccessfulPackets = 0;
uint32_t ConnectMoment = 0;
int16_t LongAcknowledgementsCounter = 0;
int16_t LongAcknowledgementsMinimum = 200;
bool DPS310Connected = false;
uint16_t DPS310Address = 0x76; // DPS310 I2C address
bool BindPlugInserted = false; // Bind plug inserted or not
uint8_t ReceiveTimeout = RECEIVE_TIMEOUT;

#endif // defined (_SRC_UTILITIES_1DEFINITIONS_H)
