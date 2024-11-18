/** @file ReceiverCode/src/utilities/common.h */
// Malcolm Messiter 2020 - 2024
#ifndef _SRC_UTILITIES_COMMON_H
#define _SRC_UTILITIES_COMMON_H

#include <Arduino.h>
#include <RF24.h>
#include <PulsePosition.h>
#include <Adafruit_INA219.h>
//#include <MPU6050_tockn.h>

#define RXVERSION_MAJOR   2
#define RXVERSION_MINOR   4
#define RXVERSION_MINIMUS 7 // 18 Novemeber 2024
#define RXVERSION_EXTRA   'V' // now with GPS 


#define HOPTIME           17 // 47     //  17 gives 50Hz FHSS, 47 gives 20Hz FHSS
#define RECEIVE_TIMEOUT   8  // 5 milliseconds is 'perfect' time between packets    


// **************************************************************************

// #define DB_FHSS
//  #define DB_SENSORS
 // #define DB_BIND
//  #define DB_FAILSAFE
//  #define DB_RXTIMERS

//  #define USE_STABILISATION           1

// >>>>>>>>>>>>>>>>               ******* DON'T FORGET TO SET THESE TWO !!! ******* <<<<<<<<<<<<<<<<<<<<< **** <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

          #define SECOND_TRANSCEIVER         // must be undefined if not using two transceivers
     //  #define USE_11PWM_OUTPUTS           // must be undefined if not using all 11 PWM outputs  

// >>>>>>>>>>>>>>>>               ******* DON'T FORGET TO SET THESE TWO !!! ******* <<<<<<<<<<<<<<<<<<<<< **** <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        #define SERVO_RES_BITS           12
        #define SERVO_RESOLUTION         4096    

        #define EXTRAAT1500  1000                          
        #define MINMICROS    500                        // normal servos
        #define MAXMICROS    2500
        
        #define EXTRAAT760   350                        // high frequency servos
        #define MAXAT760     760 + EXTRAAT760
        #define MINAT760     760 - EXTRAAT760
        #define MAXPARAMETERS          7                // Max number of parameters types to expect        
// **************************************************************************
//                            WATCHDOG PARAMETERS                           *
//***************************************************************************

#define WATCHDOGTIMEOUT 2000 // 2 Seconds before reboot (32ms -> 500 seconds)
#define KICKRATE        500  // Kick twice a second (must be between WATCHDOGMAXRATE and WATCHDOGTIMEOUT)
#define WATCHDOGMAXRATE 250  // 250 ms secs between kicks is max rate allowed

//**************************************************************************************************************************

#define SENSOR_HUB_I2C_ADDRESS 8

// ********************* >>> Reconnect params <<< ***************************************

#define LISTEN_PERIOD           14   //  was 14 (How many ms to listen for TX in Reconnect())
#define STOPLISTENINGDELAY      30   //  microseconds to wait after stopListening() in Reconnect()

// *************************************************************************************

#define DATARATE RF24_250KBPS // RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
#define PIPENUMBER       1
#define BOUNDPIPENUMBER  1
     
#define CHANNELSUSED       16                        
#define RECEIVEBUFFERSIZE  20        

struct  CD{
    uint16_t      ChannelBitMask = 0;                   
    uint16_t      CompressedData[10]; // 40 bytes ... far too big
};
CD DataReceived;

uint16_t SizeOfDataReceived = sizeof(DataReceived);

struct CD2{
    uint16_t        ID    = 0;          
    uint16_t        word[12] = {0,0,0,0,0,0,0,0,0,0,0,0}; // The 0th word isn't used, yet ...
};  

CD2 Parameters;
uint8_t         SizeOfParameters = sizeof(Parameters);


#define FREQUENCYSCOUNT  83                             // uses 83 different channels

#ifdef USE_11PWM_OUTPUTS 
    #define SERVOSUSED       11
#else
    #define SERVOSUSED       9                          // But all 16 are available via SBUS
#endif

#define SBUSRATE         10                             // SBUS frame every 10 milliseconds
#define SBUSPORT         Serial3                        // = 14
#define SBUSPIN          14                             // same as PPM pin
#define PPMPORT          14                             // same as SBUS
#define RECONNECTGAP     25                             // Send no data to servos for 25 ms after a reconnect (10 was not quite enough)
#define MINMICROS        500
#define MAXMICROS        2500
#define LED_PIN          LED_BUILTIN
#define LED_RED          16
#define BINDPLUG_PIN     17
#define RANGEMAX         2047                        // = Frsky at 150 %
#define RANGEMIN         0

#ifdef USE_11PWM_OUTPUTS
    #define pinCE1           22                          // NRF1
    #define pinCSN1          23                          // NRF1 
#else
    #define pinCE1           9                           // NRF1
    #define pinCSN1          10                          // NRF1
#endif

#define pinCSN2          20                          // NRF2
#define pinCE2           21                          // NRF2
#define FAILSAFE_TIMEOUT 2000                        // two seconds until failsafe
#define CSN_ON             LOW
#define CSN_OFF            HIGH
#define CE_ON              HIGH
#define CE_OFF             LOW
#define BIND_EEPROM_OFFSET 0                      // use 8 bytes from here
#define FS_EEPROM_OFFSET   BIND_EEPROM_OFFSET + 8 // use 16 bytes from here
#define PIPES_TO_COMPARE   8


// ****************************************************************************************************************************************

RF24     Radio1(pinCE1, pinCSN1);
RF24     Radio2(pinCE2, pinCSN2);
RF24*    CurrentRadio = &Radio1;

bool     Connected    = false;
bool     SaveNewBind  = true;
bool     HopNow       = false;
uint8_t  ThisRadio    = 1;
uint8_t  SavedPipeAddress[8];
uint8_t  NextChannelNumber = 0;
uint8_t  NextChannel;
uint8_t  ReconnectIndex = 0;
uint8_t  PacketNumber;
uint16_t RawDataIn    [RECEIVEBUFFERSIZE+1]; //  21 x 16 BIT words
uint16_t ReceivedData [RECEIVEBUFFERSIZE+1]; //  21 x 16 BIT words
uint16_t PreviousData [RECEIVEBUFFERSIZE+1]; //** Previously received data (used for servos. Hence not sent if unchanged) */
uint16_t Interations = 0;
uint32_t HopStart;
uint64_t NewPipeMaybe = 0;
uint64_t PreviousNewPipes[PIPES_TO_COMPARE];
uint8_t  PreviousNewPipesIndex  = 0;
bool     FailSafeSent           = true;
uint16_t SbusRepeats            = 0;
uint32_t RX1TotalTime           = 0;
uint32_t RX2TotalTime           = 0;
uint32_t RadioSwaps             = 0;
uint32_t  LastPacketArrivalTime = 0;
bool      INA219Connected       = false; //  Volts from INA219 ?
bool      MPU6050Connected      = false; //  Accelerometer and Gyro from MPU6050 ?
uint8_t   ReconnectChannel      = 0;

uint8_t   FHSS_Recovery_Channels[3]           = {15, 71, 82};                       // three possible channels used for Recovery
uint8_t   FHSS_Channels[83] = {51, 28, 24, 61, 64, 55, 66, 19, 76, 21, 59, 67, 15, 71, 82, 32, 49, 69, 13, 2, 34, 47, 20, 16, 72, // These are good for UK
                             35, 57, 45, 29, 75, 3, 41, 62, 11, 9, 77, 37, 8, 31, 36, 18, 17, 50, 78, 73, 30, 79, 6, 23, 40,
                             54, 12, 80, 53, 22, 1, 74, 39, 58, 63, 70, 52, 42, 25, 43, 26, 14, 38, 48, 68, 33, 27, 60, 44, 46,
                             56, 7, 81, 5, 65, 4, 10, 1};
uint8_t*  FHSSChPointer = FHSS_Channels; // Pointer for FHSS channels' array
bool    PipeSeen        = false;

uint16_t  ServoCentrePulse[11]     = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // 11 channels for servo centre pulse
uint16_t  ServoFrequency[11]       = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};                       // 11 channels for servo frequency


char ParaNames[7][30] = {"(FailSafeChannels)", "(Qnh)", "(GPSMarkHere)","(ServoCentre & Frequency)", "(SBUS/PPM)","(Servo frequencies)","(Servo centre pulses)"};

// Kalman data ********************************************************************************************************************
                                                                 
float RateRoll = 0, RatePitch = 0, RateYaw = 0;
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;
float AccX = 0, AccY = 0, AccZ = 0;
float AngleRoll = 0, AnglePitch = 0;
float CurrentRollAngle = 0;
float KalmanUncertaintyAngleRoll = 4;
float CurrentPitchAngle = 0;
float KalmanUncertaintyAnglePitch = 4;
float Kalman1DOutput[] = {0, 0};

/************************************************************************************************************/

void HopToNextChannel();
void DoStabilsation();
void DelayMillis(uint16_t ms);
void PIDEntryPoint();
void DoStabilsation();
void UseExtraParameters();
FASTRUN void ReadTheSensorHub();
void SensorHubHasFailed();
FASTRUN void Reconnect();
void LoadAckPayload();
void Decompress(uint16_t* uncompressed_buf, uint16_t* compressed_buf, uint8_t uncompressed_size);
void KeepSbusHappy();
void RebuildFlags(bool* f, uint16_t tb);
void SendQnhToSensorHub();
void MarkHere();
void UseDefaultRecoveryChannels();
void UseRandomizedRecoveryChannels();
void KickTheDog();
void SetUKFrequencies();
void BindModel();
void ReadSavedPipe(); // read only 6 bytes
void MoveServos();  
void BlinkLed();
void FailSafe();
void TurnLedOff();
void TurnLedOn();
void SaveFailSafeData();
void LoadShortAckPayload();
void IncChannelNumber();
void SetServoFrequency();

template<typename any>
void Look(const any& value);

template<typename any>
void Look1(const any& value);

Adafruit_INA219     ina219;

//                         Channels: 1  2 [3][4] 5  6 {7} 8  9 {10} 11 (Channels 3+4 & 7+10 must have same frquency)
uint8_t               PWMPins[11] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,  10}; //  if only SERVOSUSED = 9 then last two are ignored


SBUS                MySbus(SBUSPORT);                                  // SBUS
PulsePositionOutput PPMOutput;                                         // PPM

bool          BoundFlag      = false; /** indicates if receiver paired with transmitter */
uint16_t      SbusChannels[CHANNELSUSED + 1]; // Just one spare
uint32_t      SBUSTimer = 0;
bool          FailSafeChannel[17];
bool          FailSafeDataLoaded = false;
uint8_t       FS_byte1           = 0; // All 16 failsafe channel flags are in these two bytes
uint8_t       FS_byte2           = 0;
uint32_t      ReconnectedMoment;
uint16_t      BaroAltitude;
float         BaroTemperature;
float         INA219Volts       = 0;
uint32_t      SensorTime        = 0;
uint32_t      SensorHubAccessed = 0;
uint16_t      Qnh               = 0; // Pressure at sea level here and now (defined at TX)
uint16_t      OldQnh            = 0;


uint8_t       SatellitesGPS;
float         LatitudeGPS;
float         LongitudeGPS;
float         SpeedGPS;
float         AngleGPS;
bool          GpsFix = false;
float         AltitudeGPS;
float         DistanceGPS;
float         CourseToGPS;
uint8_t       DayGPS;
uint8_t       MonthGPS;
uint8_t       YearGPS;
uint8_t       HoursGPS;
uint8_t       MinsGPS;
uint8_t       SecsGPS;
float         StoredLatitudeGPS   = 51.922291; // haverfordwest!
float         StoredLongitudeGPS  = -5.213110; // haverfordwest!
bool          GPS_Connected = false;

bool          SensorHubDead       = false;
uint32_t      NewConnectionMoment = 0;
bool          QNHSent             = false;
uint8_t       MacAddress[9]       = {0, 0, 0, 0, 0, 0, 0, 0, 0};
bool          ModelMatched        = false;
uint8_t       TheReceivedPipe[6];
uint8_t       TheCurrentPipe[6];
bool          FirstConnection = true;
bool          FailedSafe      = true; // Starting up as the same as after failsafe
uint8_t       PPMChannelOrder[CHANNELSUSED] = {2, 3, 1, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
uint8_t       PPMChannelCount               = 8;
bool          UseSBUS                       = true;
bool          NewData                       = false;
uint16_t      pcount                        = 0; // how many pipes so far received from TX
bool          Blinking                      = false;
uint8_t       BlinkValue                    = 1;
uint32_t      BlinkTimer                    = 0;
uint8_t       MacAddressSentCounter         = 0;
WDT_T4<WDT3>  TeensyWatchDog;
WDT_timings_t WatchDogConfig;
uint32_t      LastDogKick = 0;
bool          LedIsOn     = false;
uint8_t*      PipePointer;
uint8_t       Pipnum         = PIPENUMBER;
uint8_t       DefaultPipe[6] = {0x23, 0x94, 0x3e, 0xbe, 0xb7, 0x00};
uint8_t       CurrentPipe[6];
uint8_t       Randomized_Recovery_Channels_Counter = 0;
uint32_t      HopMoment                           = 0;
float         X_GyroOffset = 0;
float         Y_GyroOffset = 0; 
float         Z_GyroOffset = 0; 
bool          GyroOffsetsSet = false;


#endif // defined (_SRC_UTILITIES_COMMON_H)
