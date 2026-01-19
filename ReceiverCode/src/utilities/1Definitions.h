/** @file ReceiverCode/src/utilities/1Definitions.h */
// Malcolm Messiter 2020 - 2026 

// NOTE: This header contains *definitions* of globals (storage), not just declarations.
// Project must remain single translation unit (only main.cpp compiled).
// If additional .cpp files are added they must not include this header, only extern declarations.

#ifndef _SRC_UTILITIES_1DEFINITIONS_H
#define _SRC_UTILITIES_1DEFINITIONS_H

#include <Arduino.h>
#include <Adafruit_DPS310.h>
#include <EEPROM.h>

#define RXVERSION_MAJOR 2
#define RXVERSION_MINOR 5
#define RXVERSION_MINIMUS 6
#define RXVERSION_EXTRA 'A' // 5th January 2026
#define HOPTIME 8           // gives about 100Hz FHSS

// **************************************************************************
// These debug options can be enabled or disabled as needed.
// Don't leave any enabled in normal use as they slow things down.

//  #define DB_FHSS
//  #define DB_SENSORS
//  #define DB_BIND
//  #define DB_FAILSAFE
//  #define DB_RXTIMERS

// >>>>>>>>>>>>>>>>>********************************************************************
// These options can be enabled or disabled as needed.

#define USE_BOTTOM_SOLDER_PADS_FOR_SERIAL6 // Uncomment this line to use Serial6 on the bottom solder pads for MSP communication with Nexus Rotorflight22 etc.
#define USE_SBUS                           // Enable SBUS output
#define USE_PWM                            // Enable PWM output

// **************************************************************************

#ifdef USE_BOTTOM_SOLDER_PADS_FOR_SERIAL6
#define MSP_UART Serial6
#else
#define MSP_UART Serial1
#endif
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

#define LISTEN_PERIOD 14       // milliseconds to listen for packets on the 3 recovery channels
#define STOPLISTENINGDELAY 100 // microseconds to wait after stopListening() in Reconnect()

// *************************************************************************************

#define DATARATE RF24_250KBPS // RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
#define PIPENUMBER 1
#define BOUNDPIPENUMBER 1
#define CHANNELSUSED 16 // Number of channels used
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

#define FREQUENCYSCOUNT 83 // uses 83 different channels (0 to 82)
#define SBUSRATE 10        // SBUS frame every 10 milliseconds
#define SBUSPORT Serial3   // = 14
#define SBUSPIN 14         ///
#define LED_PIN LED_BUILTIN
#define LED_RED 16
#define BINDPLUG_PIN 17
#define RANGEMAX 2047 // = Frsky at 150 %
#define RANGEMIN 0

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
#define SEND_PID_VALUES 9
#define GET_FIRST_6_PID_VALUES 10
#define GET_SECOND_6_PID_VALUES 11
#define SEND_RATES_VALUES 12
#define GET_FIRST_7_RATES_VALUES 13           // Command to update first 6 RATES values to RX
#define GET_SECOND_6_RATES_VALUES 14          // Command to update second 6 RATES values to RX
#define SEND_RATES_ADVANCED_VALUES 15         // Command to send RATES ADVANCED values to TX
#define GET_RATES_ADVANCED_VALUES_SECOND_8 16 // Command to update second 8 RATES ADVANCED values to RX
#define GET_RATES_ADVANCED_VALUES_FIRST_7 17  // Command to update first 7 RATES ADVANCED values to RX
#define SEND_PID_ADVANCED_VALUES 18           // Command to send PID ADVANCED values to TX
#define GET_FIRST_9_ADVANCED_PID_VALUES 19    // Command to update first 9 Advanced PID values to RX
#define GET_SECOND_9_ADVANCED_PID_VALUES 20   // Command to update second 9 Advanced PID values to RX
#define GET_THIRD_8_ADVANCED_PID_VALUES 21    // Command to update third 8 Advanced PID values to RX ... 26 total

#define PARAMETERS_MAX_ID 21 // Max types of parameters packet to send  ... might increase.

// **************************************************************************
//                             Rotorflight Definitions                      *
// **************************************************************************
#define SEND_NO_RF 0
#define SEND_PID_RF 1
#define SEND_RATES_RF 2
#define SEND_RATES_ADVANCED_RF 3
#define SEND_PID_ADVANCED_RF 4

// ****************************************************************************************************************************************
#define PIN_CE1 22   // NRF1 for new rxs with 11 pwm outputs
#define PIN_CSN1 23  // NRF1 for new rxs with 11 pwm outputs
#define PINA_CE1 9   // NRF1 for old rxs with only 8 pwm outputs
#define PINA_CSN1 10 // NRF1 for old rxs with only 8 pwm outputs
#define PIN_CSN2 20  // NRF2 // always same if exists
#define PIN_CE2 21   // NRF2 // always same if exists
// ****************************************************************************************************************************************

RF24 Radio1(PIN_CE1, PIN_CSN1);    // for new rxs with 11 pwm outputs
RF24 Radio1a(PINA_CE1, PINA_CSN1); // for old rxs with only 8 pwm outputs
RF24 Radio2(PIN_CE2, PIN_CSN2);    // NRF2 always same

bool Use_eleven_PWM_Outputs = false;
bool Use_Second_Transceiver = false;

uint8_t V_Pin_Ce1; // using variables now to allow for both old and new pin assignments
uint8_t V_Pin_Csn1;
uint8_t V_Pin_Ce2;
uint8_t V_Pin_Csn2;

RF24 *CurrentRadio = nullptr;

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
bool FailSafeSent = true;
uint32_t RX1TotalTime = 0;
uint32_t RX2TotalTime = 0;
uint32_t RadioSwaps = 0;
uint32_t LastPacketArrivalTime = 0;
bool INA219Connected = false;  //  Volts from INA219 ?
bool MPU6050Connected = false; //  Accelerometer and Gyro from MPU6050 ?
uint8_t ReconnectChannel = 0;
int32_t RateOfClimb = 0;
uint32_t RotorRPM = 0xffff;
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

#define PAYLOADSIZE 6 // 6 bytes only
struct Payload
{
    uint8_t Ack_Payload_byte[PAYLOADSIZE];
};
Payload AckPayload;

uint8_t AckPayloadSize = sizeof(AckPayload); // Size for later externs if needed etc.

// ************************************************************************************************************/

uint8_t FHSS_Recovery_Channels[3] = {15, 71, 82};                                                                               // three possible channels used for Recovery
uint8_t FHSS_Channels[83] = {51, 28, 24, 61, 64, 55, 66, 19, 76, 21, 59, 67, 15, 71, 82, 32, 49, 69, 13, 2, 34, 47, 20, 16, 72, // These are good for UK
                             35, 57, 45, 29, 75, 3, 41, 62, 11, 9, 77, 37, 8, 31, 36, 18, 17, 50, 78, 73, 30, 79, 6, 23, 40,
                             54, 12, 80, 53, 22, 1, 74, 39, 58, 63, 70, 52, 42, 25, 43, 26, 14, 38, 48, 68, 33, 27, 60, 44, 46,
                             56, 7, 81, 5, 65, 4, 10, 0};
uint8_t *FHSSChPointer = FHSS_Channels;                                                             // Pointer for FHSS channels' array
uint16_t ServoCentrePulse[11] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // 11 channels for servo centre pulse
uint16_t ServoFrequency[11] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};                         // 11 channels for servo frequency

/************************************************************************************************************/

void HopToNextChannel();
void DelayMillis(uint16_t ms);
void ReadExtraParameters();
FASTRUN void Reconnect();
void LoadAckPayload();
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
void StartSBUSandSERVOS();
void LoadFailSafeDataFromEEPROM();
void SaveFailSafeDataToEEPROM();
void SavePipeToEEPROM();
void DetectRotorFlightAtBoot();
void RequestFromMSP(uint8_t command);

inline bool Parse_MSP_Motor_Telemetry(const uint8_t *data, uint8_t n);
void PointToRadio1();
void PointToRadio2();
void DebugPIDValues(char const *msg);
inline void WritePIDsToNexusAndSave(const uint16_t pid[12]);
inline bool Parse_MSP_RC_TUNING(const uint8_t *data, uint8_t n);
inline void WriteRatesToNexusAndSave();
inline bool Parse_MSP_PID_PROFILE(const uint8_t *data, uint8_t n);
inline void WritePIDAdvancedToNexusAndSave();

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
uint8_t PWMPins[11] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}; // if  Servos_Used = 9 then last two are ignored
#endif

#ifdef USE_SBUS
SBUS MySbus(SBUSPORT); // SBUS
#endif

uint16_t SbusChannels[CHANNELSUSED + 1]; // Just one spare
bool FailSafeChannel[17];
bool FailSafeDataLoaded = false;
uint8_t FS_byte1 = 0; // All 16 failsafe channel flags are in these two bytes
uint8_t FS_byte2 = 0;
uint32_t ReconnectedMoment;
float BaroAltitude;
float BaroTemperature;
float RXModelVolts = 0;
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
uint32_t HopMoment = 0;
uint16_t BMP280Address = 0x76; // BMP280 I2C address
uint32_t SuccessfulPackets = 0;
uint32_t ConnectMoment = 0;
int16_t LongAcknowledgementsCounter = 0;
int16_t LongAcknowledgementsMinimum = 200;
bool DPS310Connected = false;
uint16_t DPS310Address = 0x76; // DPS310 I2C address
bool BindPlugInserted = false; // Bind plug inserted or not
uint8_t ReceiveTimeout = 10;   // this gets adjusted later

uint8_t Servos_Used = 9; // default to 9 servos used
uint8_t Receiver_Type = 0;
// *************************************************************************************************************************
// Rotorflight API and MSP protocol version variables
// *************************************************************************************************************************
bool Rotorflight22Detected = false;
float PackVoltage;
float Battery_Amps = 0;
float Battery_mAh = 0;
float ESC_Temp_C = 0.0f; // ESC temperature in degrees C

uint16_t api100 = 0; // API version as integer 12.08 -> 1208
uint8_t SendRotorFlightParametresNow = 0;
uint32_t Started_Sending_PIDs = 0;
uint32_t Started_Sending_RATEs = 0;
uint32_t Started_Sending_RATES_ADVANCED = 0;
uint32_t Started_Sending_PID_ADVANCED = 0;

uint16_t PID_Roll_P;
uint16_t PID_Roll_I;
uint16_t PID_Roll_D;
uint16_t PID_Roll_FF;
uint16_t PID_Pitch_P;
uint16_t PID_Pitch_I;
uint16_t PID_Pitch_D;
uint16_t PID_Pitch_FF;
uint16_t PID_Yaw_P;
uint16_t PID_Yaw_I;
uint16_t PID_Yaw_D;
uint16_t PID_Yaw_FF;
uint16_t All_PIDs[12];

uint16_t PID_Send_Duration = 1000;
uint16_t RATES_Send_Duration = 1000;
uint16_t RATES_ADVANCED_Send_Duration = 1000;
uint16_t PID_ADVANCED_Send_Duration = 1000;

char RF_RateTypes[6][15] = {
    "None",
    "Betatflight",
    "Raceflight",
    "KISS",
    "Actual",
    "QuickRates"};

uint8_t Rates_Type;

uint8_t Roll_Centre_Rate, Roll_Expo, Roll_Max_Rate;
uint8_t Pitch_Centre_Rate, Pitch_Expo, Pitch_Max_Rate;
uint8_t Yaw_Centre_Rate, Yaw_Expo, Yaw_Max_Rate;
uint8_t Collective_Centre_Rate, Collective_Expo, Collective_Max_Rate;
uint8_t Roll_Response_Time, Pitch_Response_Time, Yaw_Response_Time, Collective_Response_Time;
uint16_t Roll_Accel_Limit, Pitch_Accel_Limit, Yaw_Accel_Limit, Collective_Accel_Limit;
uint8_t Roll_Setpoint_Boost_Gain, Roll_Setpoint_Boost_Cutoff, Pitch_Setpoint_Boost_Gain, Pitch_Setpoint_Boost_Cutoff;
uint8_t Yaw_Setpoint_Boost_Gain, Yaw_Setpoint_Boost_Cutoff, Collective_Setpoint_Boost_Gain, Collective_Setpoint_Boost_Cutoff;
uint8_t Yaw_Dynamic_Ceiling_Gain, Yaw_Dynamic_Deadband_Gain, Yaw_Dynamic_Deadband_Filter;

#define MAX_RATES_BYTES 13
#define MAX_RATES_ADVANCED_BYTES 15
#define MAX_PID_ADVANCED_BYTES 43
#define MAX_PID_SEND_PAYLOAD_BYTES 26

uint8_t RatesBytes[MAX_RATES_BYTES];                         // 13 bytes to store rates for ack payload
uint8_t RatesBytesAdvanced[MAX_RATES_ADVANCED_BYTES];        // 15 bytes to store advanced rates for ack payload
uint8_t PID_Advanced_Bytes[MAX_PID_SEND_PAYLOAD_BYTES];      // PID Advanced data for Ackpayload
uint8_t Original_PID_Advanced_Bytes[MAX_PID_ADVANCED_BYTES]; // to detect changes

uint8_t Piro_Compensation6 = 0;
uint8_t Ground_Error_Decay1 = 0;
uint8_t Cutoff_Roll17 = 0;
uint8_t Cutoff_Pitch18 = 0;
uint8_t Cutoff_Yaw19 = 0;
uint8_t Error_Limit_Roll7 = 0;
uint8_t Error_Limit_Pitch8 = 0;
uint8_t Error_Limit_Yaw9 = 0;
uint8_t HSI_Offset_Limit_Roll36 = 0;
uint8_t HSI_Offset_Limit_Pitch37 = 0;
uint8_t HSI_Offset_Bandwidth_Roll10 = 0;
uint8_t HSI_Offset_Bandwidth_Pitch11 = 0;
uint8_t HSI_Offset_Bandwidth_Yaw12 = 0;
uint8_t Roll_D_Term_Cutoff13 = 0;
uint8_t Pitch_D_Term_Cutoff14 = 0;
uint8_t Yaw_D_Term_Cutoff15 = 0;
uint8_t Roll_B_Term_Cutoff38 = 0;
uint8_t Pitch_B_Term_Cutoff39 = 0;
uint8_t Yaw_B_Term_Cutoff40 = 0;
uint8_t CW_Yaw_Stop_Gain20 = 0;
uint8_t CCW_Yaw_Stop_Gain21 = 0;
uint8_t Yaw_Precomp_Cutoff22 = 0;
uint8_t Cyclic_FF_Gain23 = 0;
uint8_t Collective_FF_Gain24 = 0;
uint8_t Inertia_Precomp_Gain41 = 0;
uint8_t Inertia_Precomp_Cutoff42 = 0;

bool BoundFlag = false; /** indicates if receiver paired with transmitter */

#endif // defined (_SRC_UTILITIES_1DEFINITIONS_H)
