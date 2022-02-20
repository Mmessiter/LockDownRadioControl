/** @file TransmitterCode/src/main.cpp
 *
 * @page TransmitterCode
 * @section txFeatures Features list
 * - Works on Teensy 4.1
 * - 16 channels
 * - 12 BIT servo resolution (11 BIT via SBUS)
 * - 32 Mixes
 * - 4 Flight modes, or 3 plus autorotation
 * - User defined Channel names
 * - 64 editable 5-point curves (16 channels x 4 flight modes) Plus expo
 * - FailSafe on any channel(s)
 * - 2.4 GHz RF scan
 * - Motor Timer
 * - Lossless data compression.
 * - Trims on screen saved per flight mode and model.
 * - Screen timeout.
 * - FFHS fast recovery on lost packet
 * - Use 32 GIG SD card for model memories
 * - Binding - uses Mac address as unique pipe address.
 * - Four User definable three position switches
 * - Input sources definable
 * - Model memories export and import
 * - Model memory files alphabetically sorted
 * - Timer goes on and off with motor to keep track of motor use.
 * - Model memories can be sent between transmitters by RF link.
 * - DS1307 RTC added
 * - MAC address now used as unique TX ID for pipe and binding.
 * - Failsafe channel flags compressed to two bytes
 * - Exponential added
 * - GPS support added using another MCU and Adafruit Ultimate GPS
 *
 * @section txPinout Teensy 4.1 Pins
 * | Teensy 4.1 Pins | Connections |
 * |-----------------|-------------|
 * | GND       | GND |
 * | Vin       | + 5.0 VDC |
 * | 0  (RX1)  | Nextion  (TX) |
 * | 1  (TX2)  | Nextion  (RX) |
 * | 2  LED    | RED |
 * | 3  LED    | GREEN |
 * | 4  LED    | BLUE |
 * | 5  POLOLU | 2808 ALL POWER OFF SIGNAL (When high) |
 * | 6  (!! SPARE !!)
 * | 7  (RX2)  | SBUS IN    ------> BUDDY BOX SYSTEM (still under developement) |
 * | 8  (TX2)  | SBUS OUT   ------> BUDDY BOX SYSTEM (still under developement) |
 * | 9  (CE)   | nRF24l01 (CE) |
 * | 10 (CS)   | nRF24l01 (CSN) |
 * | 11 (MOSI) | nRF24l01 (MOSI) |
 * | 12 (MISO) | nRF24l01 (MISO) |
 * | 13 (SCK)  | nRF24l01 (SCK) |
 * | 14 (A0)   | Joystick POT CH1 |
 * | 15 (A1)   | Joystick POT CH2 |
 * | 16 (A2)   | Joystick POT CH3 |
 * | 17 (A3)   | Joystick POT CH4 |
 * | 18        | I2C bus  SDA |
 * | 19        | I2C bus  SCL |
 * | 20 (A6)   | POT KNOB CH5 |
 * | 21 (A7)   | POT KNOB CH6 |
 * | 22 (A8)   | POT KNOB CH7 |
 * | 23 (A9)   | POT KNOB CH8 |
 * | 24 (!! SPARE !!)
 * | 25        | Switch 1 |
 * | 26        | Switch 1 |
 * | 27        | Switch 2 |
 * | 28        | Switch 2 |
 * | 29        | Switch 3 |
 * | 30        | Switch 3 |
 * | 31        | Switch 4 |
 * | 32        | Switch 4 |
 * | 33 (!! SPARE !!)
 * | 34 (!! SPARE !! RX8)
 * | 35 (!! SPARE !! TX8)
 * | 36 (!! SPARE !!)
 * | 37 (!! SPARE !!)
 * | 38 (!! SPARE !!)
 * | 39 (!! SPARE !!)
 * | 40 (!! SPARE !!)
 * | 41 (!! SPARE !!)
 * @see TransmitterCode/src/main.cpp
 */
// ************************************************** TRANSMITTER CODE **************************************************

#define CHANNELSUSED       16                  // 16 Channels
#define MAXMIXES           32                  // 32 mixes
#define TICKSPERMINUTE     60000               // millis() += 60000 per minute
#define PROPOCHANNELS      8                   // Only 4 have knobs / 2 sticks (= 4 hall sensors)
#define FLIGHTMODESWITCH   4                   // Default MODE switch
#define AUTOSWITCH         1                   // Default AUTO switch
#define DEFAULTPIPEADDRESS 0xBABE1E5420LL      // Pipe address for startup - any value but MUST match RX
#define LOWBATTERY         40                  // percent for warning
#define CE_PIN             9                   // for SPI to nRF24L01
#define CSN_PIN            10                  // for SPI to nRF24L01
#define INACTIVITYTIMEOUT  10                  // Default time after which to switch off
#define INACTIVITYMINIMUM  5 * TICKSPERMINUTE  // Inactivity timeout minimum is 5 minutes
#define INACTIVITYMAXIMUM  30 * TICKSPERMINUTE // Inactivity timeout maximum is 30 minutes
#define DS1307_ADDRESS     0x68

// CurrentMode values (=WHETHER TO SEND DATA)

#define NORMAL          0 // Normal for transmit as usual
#define CALIBRATELIMITS 1 // Calibrate limits
#define CENTRESTICKS    2 // Calibrate Centres
#define SCANWAVEBAND    3 // Scan waveband
#define SENDNOTHING     4 // Transmission off

// VALUES FOR MAX SERVO RESOLUTION

#define MINMICROS       500
#define MAXMICROS       2500
#define HALFMICROSRANGE (MAXMICROS - MINMICROS) / 2 //  = 500
#define MIDMICROS       MINMICROS + HALFMICROSRANGE

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <TeensyID.h>
#include <EEPROM.h>
#include <InterpolationLib.h>
#include <SBUS.h>
#include "Hardware/RadioFunctions.h"

#ifdef USE_WATCHDOG
    #include <Watchdog_t4.h>
#endif

RF24 Radio1(CE_PIN, CSN_PIN);

#define Nextion         Serial1 // Nextion is connected to Serial1
#define Black           0
#define Blue            31
#define Brown           48192
#define Green           2016
#define Yellow          65504
#define Red             63488
#define Gray            33840
#define SkyBlue         2047
#define Purple          39070
#define Orange          64512
#define White           65535
#define FlightModesUsed 4
#define M_Enabled       0 // Offsets for Mixes array
#define M_FlightMode    1
#define M_MasterChannel 2
#define M_SlaveChannel  3
#define M_Reversed      4
#define M_Percent       5
#define M_R1            6
#define M_R2            7
#define FrontView       0
#define SticksView      1
#define GraphView       2
#define MixesView       3
#define FhssView        4
#define ModelsView      5
#define CalibrateView   6
#define MainSetupView   7
#define GainsView       8
#define DataView        9
#define Trim_View       10
#define Mode_View       11
#define Switches_View   12
#define One_Switch_View 13
#define Help_View       14
#define Options_View    15
#define Inputs_View     16
#define FailSafe_View   17

#define UNCOMPRESSEDWORDS 20                        // DATA TO SEND = 40  bytes
#define COMPRESSEDWORDS   UNCOMPRESSEDWORDS * 3 / 4 // COMPRESSED DATA SENT = 30  bytes

#define Switch0       32 // SWITCHES' PIN NUMBERS ...
#define Switch1       31
#define Switch2       30
#define Switch3       29
#define Switch4       28
#define Switch5       27
#define Switch6       26
#define Switch7       25
#define REDLED        2 // COLOURED LEDS' PIN NUMBERS ...
#define GREENLED      3
#define BLUELED       4
#define POWER_OFF_PIN 5 //should be 5

// SDCARD MODEL MEMORY CONSTANTS

#define RENEWDATA  8787 // Change these to rewrite all
#define TXSIZE     250  // SD space reserved for transmitter
#define MODELSIZE  1600 // SD space reserved for each model
#define MAXFILELEN 1021 // MAX SIZE FOR HELP FILE

#ifdef USE_WATCHDOG
WDT_T4<WDT3>  TeensyWatchDog;
WDT_timings_t WatchDogConfig;
#endif

SBUS     MySbus(SBUSPORT);
uint16_t SbusChannels[CHANNELSUSED + 2]; // a few spare
uint32_t SBUSTimer = 0;

uint8_t Mixes[MAXMIXES + 1][CHANNELSUSED + 1];                // Channel mixes' 2D array store
int     Trims[FlightModesUsed + 1][CHANNELSUSED + 1];         // Trims to store
uint8_t TrimsReversed[FlightModesUsed + 1][CHANNELSUSED + 1]; // Trim directions to store
uint8_t Exponential[FlightModesUsed + 1][CHANNELSUSED + 1];   // Exponential
uint8_t InterpolationTypes[FlightModesUsed + 1][CHANNELSUSED + 1];

uint8_t       LastMixNumber      = 1;
uint8_t       MixNumber          = 0;
uint8_t       CurrentView        = FrontView;
uint8_t       SavedCurrentView   = FrontView;
const uint8_t CharsMax           = 120;                // 80;
const uint8_t MaxDataTransferred = UNCOMPRESSEDWORDS;  // = 40 bytes     A few extra bytes sent after channels' values
uint64_t      DefaultPipe        = DEFAULTPIPEADDRESS; //          Default Radio pipe address
uint64_t      NewPipe            = 0xBABE1E5420LL;     //             New Radio pipe address for binding will come from MAC address
char          TextIn[CharsMax];
char          WordsIn[CharsMax];
unsigned int  i;
unsigned int  PacketsPerSecond = 0;
unsigned int  LostPackets      = 0;
uint8_t       PacketNumber     = 0;
uint8_t       GPSMarkHere      = 0;

// ************************************* AckPayload structure ******************************************************
/**
     * This first byte "Purpose" defines what all the other bytes mean, AND ...
     * the highest BIT of Purpose means ** HOP TO NEXT CHANNEL A.S.A.P. (IF ON) **
     * the lower 7 BITs then define the meaning of the remainder of the ackpayload bytes
     * If Purpose = 1 then ...
     * 
     * AckPayload.Byte2           =  ThisRadio;            // Radio in current use  Byte1 and Byte2 are free
     * AckPayload.Byte3           =  RXVERSION_MAJOR;
     * AckPayload.Byte4           =  RXVERSION_MINOR;
     * AckPayload.Byte5           =  RXVERSION_MINIMUS;
     *
     *  If Purpose = 2 then ...
     **/


struct Payload
{
    uint8_t Purpose; // Defines meaning of the remainder
                     // Highest BIT of Purpose means HOP NOW! IF ON
    uint8_t Byte1;   // 
    uint8_t Byte2;   // 
    uint8_t Byte3;   // 
    uint8_t Byte4;   // 
    uint8_t Byte5;   // 
};
Payload AckPayload;

uint8_t AckPayloadSize = sizeof(AckPayload); // i.e. 6

// *****************************************************************************************************************

uint16_t SendBuffer[MaxDataTransferred];     //    Data to send to rx (16 words)
uint16_t ShownBuffer[MaxDataTransferred];    //    Data shown before
uint16_t LastBuffer[CHANNELSUSED + 1];       //    Used to spot any change
uint16_t PreMixBuffer[CHANNELSUSED + 1];     //    Data collected from sticks
uint8_t  MaxDegrees[5][CHANNELSUSED + 1];    //    Max degrees (180?)
uint8_t  MidHiDegrees[5][CHANNELSUSED + 1];  //    MidHi degrees (135?)
uint8_t  CentreDegrees[5][CHANNELSUSED + 1]; //    Degrees (90)
uint8_t  MidLowDegrees[5][CHANNELSUSED + 1]; //    MidLow Degrees (45?)
uint8_t  MinDegrees[5][CHANNELSUSED + 1];    //    Max Degrees (0?)
uint8_t  FlightMode         = 1;
uint8_t  PreviousFlightMode = 1;
int      ChannelMax[CHANNELSUSED + 1];    //    output of pots at max
int      ChannelMidHi[CHANNELSUSED + 1];  //    output of pots at MidHi
int      ChannelCentre[CHANNELSUSED + 1]; //    output of pots at Centre
int      ChannelMidLow[CHANNELSUSED + 1]; //    output of pots at MidLow
int      ChannelMin[CHANNELSUSED + 1];    //    output of pots at min
int      ChanneltoSet     = 0;
bool     Connected        = false;
uint8_t  ShowCommsCounter = 0;

double PointsCount = 5; // This for displaying curves only
double xPoints[5];
double yPoints[5];
double xPoint = 0;
double yPoint = 0;

int           BoxOffset = 35;
int           BoxSize   = 395;
int           BoxBottom;
int           BoxTop;
int           BoxLeft;
int           BoxRight;
int           ClickX;
int           ClickY;
bool          CalibratedYet                = false;
int           AnalogueInput[PROPOCHANNELS] = {A0, A1, A2, A3, A6, A7, A8, A9}; // PROPO Channels for transmission
uint8_t       CurrentMode                  = NORMAL;
uint8_t       AllChannels[127]; /// for scanning
uint8_t       NoCarrier[127];
uint8_t       ScanStart   = 1;
uint8_t       ScanEnd     = 125;
uint32_t      TimerMillis = 0;
uint32_t      LastSeconds = 0;
uint32_t      Secs        = 0;
uint32_t      PausedSecs  = 0;
uint32_t      Mins        = 0;
uint32_t      Hours       = 0;
uint8_t       NameCount     = 0;
char          ModelName[30] = "Undefined";
uint8_t       ModelNumber   = 1;
uint8_t       ModelDefined  = 0;
uint16_t      MemoryForTransmtter  = 0;   // SD space for transmitter parameters
uint16_t      OneModelMemory       = 0;   // SD space for every model's parameters
uint16_t      SDCardAddress        = 0;   // Address on SD card (offset from zero)

char     FrontView_Hours[]           = "Hours";
char     FrontView_Mins[]            = "Mins";
char     FrontView_Secs[]            = "Secs";
char     page_FrontView[]            = "page FrontView";
char     page_FhssView[]             = "page FhssView";
char     FhssView_Rlow[]             = "FHSSLow";
char     FhssView_Rhigh[]            = "FHSSHigh";
char     BindScreenBox[]             = "BindStatus";
char     NextionSleepTime[]          = "thsp=";
char     NextionWakeOnTouch[]        = "thup=1";
char     NextionSleepNow[]           = "sleep=1";
char     NextionWakeUp[]             = "sleep=0";
char     ScreenViewTimeout[]         = "Sto";
char     NoSleeping[]                = "thsp=0";
int      ScreenTimeout               = 120; // Screen has two minute timeout by default
char     HtextCMD[]                  = "click HelpText,0";
int      LastLinePosition            = 0;
uint8_t  RXCellCount                 = 2;
bool     JustHoppedFlag              = true;
bool     LostContactFlag             = true;
uint8_t  RecentPacketsLost           = 0;
uint32_t TotalledRecentPacketsLost   = 0;
long int RecoveryTimer               = 0;
bool     ReconnectingFlag            = true;
int      ReconnectTime               = 0;
uint32_t GapSum                      = 0;
uint32_t GapLongest                  = 0;
uint32_t GapStart                    = 0;
uint32_t ThisGap                     = 0;
uint32_t GapAverage                  = 0;
uint32_t GapCount                    = 0;
uint32_t GapShortest                 = 0;
char     CalibrateNow[]              = "touch_j";
char     ModelVolts[8]               = " ";
float    GPSLatitude                 = 0;  
float    GPSLongitude                = 0;
float    GPSMarkLatitude             = 0;  
float    GPSMarkLongitude            = 0;
float    GPSAngle                    = 0;
bool     GpsFix                      = 0;
uint8_t  GPSSatellites               = 0;
uint16_t GPSSpeed                    = 0;
uint16_t GPSMaxSpeed                 = 0;

uint8_t  GPSHours                    = 0;
uint8_t  GPSMins                     = 0;
uint8_t  GPSSecs                     = 0;

uint8_t  GPSDay                      = 0;
uint8_t  GPSMonth                    = 0;
uint8_t  GPSYear                     = 0;


float    GPSAltitude                 = 0;
float    GPSMaxAltitude              = 0;
float    GPSGroundAltitude           = 0;
float    GPSDistanceTo               = 0;
float    GPSCourseTo                 = 0;
float    GPSMaxDistance              = 0;
float    RXModelVolts                = 0;
int      RXModelAltitude             = 0;
int      RXMAXModelAltitude          = 0;
int      GroundModelAltitude         = 0;
float    RXModelTemperature          = 0;
char     ModelTemperature[8]         = " ";
char     ModelAltitude[8]            = " ";
char     MaxAltitude[8]              = " ";
float    MaxAlt                      = 0;
char     ReceiverVersionNumber[8]    = " ";
char     TransmitterVersionNumber[8] = " ";
char     deletedmodel[]              = "Deleted";

File ModelsFileNumber;

Adafruit_INA219 ina219;

const int chipSelect = BUILTIN_SDCARD;
char      SingleModelFile[80];
bool      SingleModelFlag = false;
char      ModelsFile[]    = "models.dat";
bool      ModelsFileOpen  = false;
bool      USE_INA219      = false;
uint8_t   BindingNow      = 0;
int       BindingTimer    = 0;
bool      BoundFlag       = false;
int       PipeTimeout     = 0;
bool      Switch[8];
uint8_t   SwitchNumber[8] = {Switch0, Switch1, Switch2, Switch3, Switch4, Switch5, Switch6, Switch7};

char TrimView_ch1[] = "ch1";
char TrimView_ch2[] = "ch2";
char TrimView_ch3[] = "ch3";
char TrimView_ch4[] = "ch4";
char TrimView_n1[]  = "n1";
char TrimView_n2[]  = "n2";
char TrimView_n3[]  = "n3";
char TrimView_n4[]  = "n4";
char TrimView_r1[]  = "r1";
char TrimView_r2[]  = "r2";
char TrimView_r3[]  = "r3";
char TrimView_r4[]  = "r4";

uint8_t FMSwitch   = FLIGHTMODESWITCH;
uint8_t AutoSwitch = AUTOSWITCH;

uint8_t Channel9Switch  = 0;
uint8_t Channel10Switch = 0;
uint8_t Channel11Switch = 0;
uint8_t Channel12Switch = 0;

uint8_t Channel9SwitchValue  = 0;
uint8_t Channel10SwitchValue = 0;
uint8_t Channel11SwitchValue = 0;
uint8_t Channel12SwitchValue = 0;

bool Switch1Reversed = false;
bool Switch2Reversed = false;
bool Switch3Reversed = false;
bool Switch4Reversed = false;

int      StartLocation       = 0;
bool     ValueSent           = false;
int      SwitchEditNumber    = 0; // number of switch being edited
uint32_t ShowServoTimer      = 0;
bool     LastFourOnly        = false;
uint8_t  InPutStick[17]      = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}; //
uint8_t  ExportedFileCounter = 0;
char     TheFilesList[100][14];
int      FileNumberInView     = 0;
bool     FileError            = false;
int      RangeTestStart       = 0;
int      RangeTestGoodPackets = 0;
int      RangeTestLostPackets = 0;
float    success              = 0; // percent of packets that  succeed
uint8_t  SaveFlightMode       = 0;
bool     FailSafeChannel[CHANNELSUSED];
bool     SaveFailSafeNow                = false;
uint32_t FailSafeTimer;
char     ChannelNames[CHANNELSUSED][11] = {{"Aileron"}, {"Elevator"}, {"Throttle"}, {"Rudder"}, {"Gear"}, {"AUX1"}, {"AUX2"}, {"AUX3"}, {"AUX4"}, {"AUX5"}, {"AUX6"}, {"AUX7"}, {"AUX8"}, {"AUX9"}, {"AUX10"}, {"AUX11"}};

bool VoltsDetected = false;
bool TXWarningFlag = false;
bool RXWarningFlag = false;

uint32_t TxOnTime      = 0;
uint32_t TxPace        = 0;
bool     ModelDetected = false;
uint16_t CompressedData[COMPRESSEDWORDS]; // = 20
uint8_t  SizeOfCompressedData;
uint32_t Inactivity_Timeout = 10 * TICKSPERMINUTE;
uint32_t Inactivity_Start   = 0;

tmElements_t tm;
char         TxName[32]   = {"Curtis Youngblood"};
int          LastTimeRead = 0;
int          LastShowTime = 0;
int          LastDogKick  = 0;
uint8_t      MacAddress[6];
char         DateTime[] = "DateTime";

int      XtouchPlace = 0; // Clicked X
int      YtouchPlace = 0; // Clicked Y
uint8_t  zero        = 0x00;
bool     BindButton  = false;

uint8_t  PreviousChannelNumber = 0;
uint8_t  NextChannelNumber     = 0;
bool     InhibitNameCheck      = false;

// changing these four valiables controls LED blink and speed

bool     LedIsBlinking = false;
float    BlinkHertz    = 1;
uint32_t BlinkTimer    = 0;
uint8_t  BlinkOnPhase  = 1;
bool     LedWasGreen   = false;
char     ThisRadio[4]  = "0 ";
uint8_t NextChannel   = 0;
bool    DoSbusSendOnly  = false;
bool    BuddyMaster     = false;
bool    SlaveHasControl = false;
uint16_t Qnh            = 1009;               // pressure at sea level here
uint16_t ModelNumberOffset = 0;
uint32_t ModelNameTimeCheck = 0;
uint16_t LastModelLoaded    = 0;

uint8_t * FHSSChPointer;                                                              // pointer for channels array (first five only used for reconnect)

uint8_t FHSS_Channels1[42] = {93,111,107,103,106,97,108,102,118,104,101,109,98,      // TEST array
113,124,115,91,96,85,117,89,99,114,87,112,
86,94,92,119,120,100,121,123,95,122,105,84,116,90,110,88};

uint8_t FHSS_Channels[83] = {51,28,24,61,64,55,66,19,76,21,59,67,15,71,82,32,49,69,13,2,34,47,20,16,72,  // UK array
35,57,45,29,75,3,41,62,11,9,77,37,8,31,36,18,17,50,78,73,30,79,6,23,40,
54,12,80,53,22,1,74,39,58,63,70,52,42,25,43,26,14,38,48,68,33,27,60,44,46,
56,7,81,5,65,4,10};

uint8_t  Gsecond;  // = tm.Second; // 0-59
uint8_t  Gminute;  // = tm.Minute; // 0-59
uint8_t  Ghour;    // = tm.Hour;   // 0-23
uint8_t  GweekDay; // = tm.Wday;   // 1-7
uint8_t  GmonthDay;// = tm.Day;    // 1-31
uint8_t  Gmonth;   // = tm.Month;  // 1-12
uint8_t  Gyear;    // = tm.Year;   // 0-99
bool     GPSTimeSynched  =   false;
int      DeltaGMT        = 0;
uint32_t SwapWaveBandTimer = 0;
uint8_t  UkRulesCounter = 0;
bool     UkRules = true;
uint8_t  SwapWaveBand = 0;  
uint16_t TrimFactor   = 2;   // How much to multiply trim by


/************************************************************************************************************/
// This function returns distance (in MILES) between two GPS coordinates (in degrees)
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
//        double  distance = (RadiusOfTheEarth * c ) ; //* 3.28084) / 1760;
//        return  distance; // in MILES now
//   }

/******************* DeltaGMT is a user defined representation of time zone. It should never exceed 24. Not on this planet. **********/
void FixDeltaGMTSign(){
    if (DeltaGMT < -24) DeltaGMT = 0;  // Undefined value?f
    if (DeltaGMT > 24){                // This fixes the sign bit if negative !!!! (There's surely a better way !!!)
        DeltaGMT ^= 0xffff;            // toggle every bit! :-)
        ++DeltaGMT;                    // Add one
        DeltaGMT =- DeltaGMT;          // it's definately meant to be negative!
    }
}
/************************************************************************************************************/
// This function reads data from BUDDY (Slave) BUT uses it ONLY WHILE the channel 12 switch is in the ON position ( > 1000)

void GetSlaveChannelValues()
{
    bool failSafeM; // These flags not used, yet.
    bool lostFrameM;
    SlaveHasControl = false;
    if (SendBuffer[11] > 1000)
    { // MASTER'S CHANNEL 12 (500 - 2500) used here as switch.
        if (MySbus.read(&SbusChannels[0], &failSafeM, &lostFrameM))
        {
            SBUSTimer = millis();       // RESET timeout when data comes in
        }                               // Even if there's no new data, re-use old data
        if (millis() - SBUSTimer < 500) // Ignore data more than 500ms old
        {
            SlaveHasControl = true;
            for (int j = 0; j < CHANNELSUSED; ++j) // While slave has control, his stick data replaces all ours
            {
                SendBuffer[j] = map(SbusChannels[j], RANGEMIN, RANGEMAX, MINMICROS, MAXMICROS); // Put re-mapped data where we use it.
            }
        }
    }
}

/************************************************************************************************************/

/** Map servo channels' data from SendBuffer into SbusChannels buffer */
// This funtion is used by the BUDDY slave to send it's controls out down a wire using SBUS
void MapToSBUS()
{
    if (millis() - SBUSTimer >= SBUSRATE)
    {
        SBUSTimer = millis();
        for (int j = 0; j < CHANNELSUSED; ++j)
        {
            SbusChannels[j] = map(SendBuffer[j], MINMICROS, MAXMICROS, RANGEMIN, RANGEMAX);
        }
        MySbus.write(&SbusChannels[0]);
    }
}
/************************************************************************************************************/

/*********************************************************************************************************************************/

uint8_t decToBcd(uint8_t val)
{
    return ((val / 10 * 16) + (val % 10));
}

/*********************************************************************************************************************************/

uint8_t bcdToDec(uint8_t val)
{
    return ((val / 16 * 10) + (val % 16));
}

/*********************************************************************************************************************************/

void SetTheRTC(){
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(zero);                               // Stop the oscillator
    Wire.write(decToBcd(Gsecond));
    Wire.write(decToBcd(Gminute));
    Wire.write(decToBcd(Ghour));
    Wire.write(decToBcd(GweekDay));
    Wire.write(decToBcd(GmonthDay));
    Wire.write(decToBcd(Gmonth));
    Wire.write(decToBcd(Gyear));
    Wire.write(zero);                               //  Re-start it
    Wire.endTransmission();
}

/*********************************************************************************************************************************/
void ReadTheRTC(){
    uint8_t second   = tm.Second; // 0-59
    uint8_t minute   = tm.Minute; // 0-59
    uint8_t hour     = tm.Hour;   // 0-23
    uint8_t weekDay  = tm.Wday;   // 1-7
    uint8_t monthDay = tm.Day;    // 1-31
    uint8_t month    = tm.Month;  // 1-12
    uint8_t year     = tm.Year;   // 0-99
    Gsecond          = second; 
    Gminute          = minute;  
    Ghour            = hour;   
    GweekDay         = weekDay;
    GmonthDay        = monthDay;
    Gmonth           = month;  
    Gyear            = year-30;  // ???
}
/*********************************************************************************************************************************/
void SynchRTCwithGPSTime(){            // This function corrects the time and the date.    
    if (!GPSTimeSynched){              
        GPSTimeSynched = true;
        Gsecond   = GPSSecs;
        Gminute   = GPSMins;
        Ghour     = GPSHours;
        GmonthDay = GPSDay;
        Gmonth    = GPSMonth;
        Gyear     = GPSYear + 1744; // ????
        SetTheRTC();
    }
}
/*********************************************************************************************************************************/

void AdjustDateTime(uint8_t MinChange, uint8_t HourChange, uint8_t YearChange, uint8_t MonthChange, uint8_t DateChange) 
{
   ReadTheRTC();
    Gminute += MinChange;
    if (Gminute > 59) {
        Gminute = 0;
        if (Ghour < 23) {
            ++Ghour;
        }
    }
    if (Gminute < 1) {
        Gminute = 0;
        if (Ghour > 0) {
            --Ghour;
        }
    }
    Ghour += HourChange;
    if (Ghour < 0) Ghour = 0;
    if (Ghour > 23) Ghour = 23;
    Gyear += YearChange;
    if (Gyear < 0) Gyear = 0;
    if (Gyear > 99) Gyear = 99;
    Gmonth += MonthChange;
    if (Gmonth < 1) Gmonth = 1;
    if (Gmonth > 12) Gmonth = 12;
    GmonthDay += DateChange;
    if (GmonthDay < 1) GmonthDay = 1;
    if (GmonthDay > 31) GmonthDay = 31;
    SetTheRTC();
}
/*********************************************************************************************************************************/

void IncMinute()
{
    uint8_t c = 1;
    if (RTC.read(tm)) {
        AdjustDateTime(c, zero, zero, zero, zero);
    }
}

/*********************************************************************************************************************************/

void DecMinute()
{
    uint8_t c = -1;
    if (RTC.read(tm)) {
        AdjustDateTime(c, zero, zero, zero, zero);
    }
}

/*********************************************************************************************************************************/

void IncHour()
{
    uint8_t c = 1;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, c, zero, zero, zero);
    }
}

/*********************************************************************************************************************************/

void DecHour()
{
    uint8_t c = -1;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, c, zero, zero, zero);
    }
}

/*********************************************************************************************************************************/

void IncYear()
{
    uint8_t c = 1;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, c, zero, zero);
    }
}

/*********************************************************************************************************************************/

void DecYear()
{
    uint8_t c = -1;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, c, zero, zero);
    }
}

/*********************************************************************************************************************************/

void IncMonth()
{
    uint8_t c = 1;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, zero, c, zero);
    }
}

/*********************************************************************************************************************************/

void DecMonth()
{
    uint8_t c = -1;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, zero, c, zero);
    }
}

/*********************************************************************************************************************************/

void IncDate()
{
    uint8_t c = 1;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, zero, zero, c);
    }
}

/*********************************************************************************************************************************/

void DecDate()
{
    uint8_t c = -1;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, zero, zero, c);
    }
}

/*********************************************************************************************************************************/

bool getTime(const char* str)
{
    int Hour, Min, Sec;
    if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
    tm.Hour   = Hour;
    tm.Minute = Min;
    tm.Second = Sec;
    return true;
}

/*********************************************************************************************************************************/

bool getDate(const char* str)
{
    char        Month[12];
    int         Day, Year;
    uint8_t     monthIndex;
    const char* monthName[12] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
    for (monthIndex = 0; monthIndex < 12; ++monthIndex) {
        if (strcmp(Month, monthName[monthIndex]) == 0) break;
    }
    if (monthIndex >= 12) return false;
    tm.Day   = Day;
    tm.Month = monthIndex + 1;
    tm.Year  = CalendarYrToTm(Year);
    return true;
}

/*********************************************************************************************************************************/

void KickTheDog()
{
#ifdef USE_WATCHDOG
    if (millis() - LastDogKick >= KICKRATE) {
        LastDogKick = millis();
        TeensyWatchDog.feed();
    }
#endif
}

/*********************************************************************************************************************************/

void Reboot()
{
#ifdef USE_WATCHDOG
    for (i = 0; i < 30; ++i) {
        TeensyWatchDog.feed();
    } // Dog will explode when overfed
#endif
}

/*********************************************************************************************************************************/

void SendCommand(char* tbox)
{
    Nextion.print(tbox);

    // Serial.print (" Nextion TEST COMMAND: ");  // For wierd session
    // Serial.println(tbox);

    for (i = 0; i < 3; ++i) {
        Nextion.write(0xff);
    } // Send end of Input message
}

/*********************************************************************************************************************************/

void SendText(char* tbox, char* NewWord)
{
    char txt[]   = ".txt=\"";
    char quote[] = "\"";
    char CB[100];
    char TooLong[] = "Too long!";

    if (strlen(NewWord) > 90) {
        strcpy(NewWord, TooLong);
    }
    strcpy(CB, tbox);
    strcat(CB, txt);
    strcat(CB, NewWord);
    strcat(CB, quote);
    SendCommand(CB);
}

/*********************************************************************************************************************************/

void SendText1(char* tbox, char* NewWord)
{
    char txt[]   = ".txt=\"";
    char quote[] = "\"";
    char CB[1300];
    char TooLong[] = "Too long!";

    if (strlen(NewWord) > 1250) {
        strcpy(NewWord, TooLong);
    }
    strcpy(CB, tbox);
    strcat(CB, txt);
    strcat(CB, NewWord);
    strcat(CB, quote);
    SendCommand(CB);
}

/*********************************************************************************************************************************/

// This function converts an "int" to a string, AND then adds a comma, a dot, or nothing at the end
// It dates for a time when I didn't know about standard library functions! But it works just fine, so it says in.

char* Str(char* s, int n, int comma) // comma = 0 for nothing, 1 for a comma, 2 for a dot.
{
    int  r, i, m, flag;
    char cma[] = ",";
    char dot[] = ".";

    flag = 0;
    i    = 0;
    m    = 1000000000;
    if (n < 0) {
        s[0] = '-';
        i    = 1;
        n    = -n;
    }
    if (n == 0) {
        s[0] = 48;
        s[1] = 0;

        if (comma == 1) {
            strcat(s, cma);
        }
        if (comma == 2) {
            strcat(s, dot);
        }
        return s;
    }
    while (m >= 1) {
        r = n / m;
        if (r > 0) {
            flag = 1;
        } //  first digit
        if (flag == 1) {
            s[i] = 48 + r;
            ++i;
            s[i] = 0;
        }
        n -= (r * m);
        m /= 10;
    }
    if (comma == 1) {
        strcat(s, cma);
    }
    if (comma == 2) {
        strcat(s, dot);
    }
    return s;
}

/*********************************************************************************************************************************/

bool MayBeAddZero(uint8_t nn)
{
    if (nn >= 0 && nn < 10) {
        return true;
    }
    return false;
}

/*********************************************************************************************************************************/

void ReadTime()
{
    static char month[12][15]     = {"January", "February", "March", "April", "May", "June", "July", "August", "Sept", "October", "November", "December"};
    static char ShortMonth[12][7] = {"Jan. ", "Feb. ", "Mar. ", "Apr. ", "May  ", "June ", "July ", "Aug. ", "Sept ", "Oct. ", "Nov. ", "Dec. "};

    char NB[10];
    char TimeString[50];
    char Space[] = " ";
    char colon[]  = ":";
    char colon1[] = ".";
    char zero[]  = "0";

    char Owner[] = "Owner";

    FixDeltaGMTSign();  // TODO Fix the time and date when it goes past midnight in either direction owing to time zone

    if (CurrentView == FrontView || CurrentView == Options_View) {
        if (RTC.read(tm)) {
            strcpy(TimeString, Str(NB, tm.Day, 0));
            if (CurrentView == Options_View)
            {
                if ((tm.Day) < 10) {
                    strcat(TimeString, Space); // to align better the rest of the data
                    strcat(TimeString, Space);
                }
            }
            strcat(TimeString, Space);
            if (CurrentView == Options_View)
            {
                strcat(TimeString, ShortMonth[tm.Month - 1]);
            }
            else
            {
                strcat(TimeString, month[tm.Month - 1]);
            }
            strcat(TimeString, Space);
            strcat(TimeString, (Str(NB, tmYearToCalendar(tm.Year), 0)));
            strcat(TimeString, Space);
            if (MayBeAddZero(tm.Hour)) strcat(TimeString, zero);
            strcat(TimeString, Str(NB, tm.Hour+DeltaGMT, 0));
            
            if (UkRules) strcat(TimeString, colon);
            if (!UkRules) strcat(TimeString, colon1);

            if (MayBeAddZero(tm.Minute)) strcat(TimeString, zero);
            strcat(TimeString, Str(NB, tm.Minute, 0));

             if (UkRules)strcat(TimeString, colon);
            if (!UkRules) strcat(TimeString, colon1);
            if (MayBeAddZero(tm.Second)) strcat(TimeString, zero);
            strcat(TimeString, Str(NB, tm.Second, 0));
            SendText(DateTime, TimeString);
            SendText(Owner, TxName);
        }
    }
}

/*********************************************************************************************************************************/

void StartInactvityTimeout()
{
    Inactivity_Start = millis();
}

/*********************************************************************************************************************************/

uint8_t GetBrightness()
{
    if (LedIsBlinking) {
        if ((millis() - BlinkTimer) > (500 / BlinkHertz)) {
            BlinkOnPhase ^= 1;
            BlinkTimer = millis();
        }
    }
    else {
        BlinkOnPhase = 1;
    }
    if (BlinkOnPhase) {
        return 32;
    }
    else {
        return 0;
    }
}

/*********************************************************************************************************************************/

void RedLedOn()
{
    LedWasGreen = false;
    analogWrite(GREENLED, 0);
    analogWrite(BLUELED, 0);
    analogWrite(REDLED, GetBrightness()); // Brightness is a function of blinking
}

void MakeBindButtonInvisible()
{
    char bbiv[] = "vis bind,0";
    if (BindButton) {
        SendCommand(bbiv);
        BindButton = false;
    }
}

/*********************************************************************************************************************************/

void GreenLedOn()
{
    if (!LedWasGreen) {
        LedWasGreen = true;
        analogWrite(BLUELED, 0);
        analogWrite(REDLED, 0);
        analogWrite(GREENLED, GetBrightness()); // Brightness is a function maybe blinking
        LastShowTime = 0;
        MakeBindButtonInvisible();
    }
}

/*********************************************************************************************************************************/

void BlueLedOn()
{
    LedWasGreen = false;
    analogWrite(REDLED, 0);
    analogWrite(GREENLED, 0);
    analogWrite(BLUELED, GetBrightness()); // Brightness is a function of blinking
}

/*********************************************************************************************************************************/

uint16_t mp(uint8_t lowres)
{
    return map(lowres, 0, 180, MINMICROS, MAXMICROS);
} // This returns the 5 curve-points at the higher resolution

/*********************************************************************************************************************************/

void ClearText()
{
    for (i = 0; i < CharsMax; ++i) {
        WordsIn[i] = char(0);
        TextIn[i]  = 0;
    }
    i = 0;
}

/*********************************************************************************************************************************/
// Nextion functions
/*********************************************************************************************************************************/

void EndSend()
{
    for (int pp = 0; pp < 3; ++pp) Nextion.write(0xff); // Send end of Input message
    delay(65);                                          // ** A DELAY ** (>=50 ms) is needed if an answer might come!
}

/*********************************************************************************************************************************/

void SendValue(char* nbox, int value)
{
    char Val[] = ".val=";
    char CB[100];
    char NB[25];
    strcpy(CB, nbox);
    strcat(CB, Val);
    strcat(CB, Str(NB, value, 0));
    SendCommand(CB);
    ValueSent = true;
}

/*********************************************************************************************************************************/

void GetTextIn()
{
    ClearText();
    if (Nextion.available()) {
        delay(10);
        while (Nextion.available()) {
            TextIn[i] = uint8_t(Nextion.read());
            if (TextIn[i] == '$') TextIn[i] = 0; 
            if (i < CharsMax) ++i;
        }
    }
}

/*********************************************************************************************************************************/

int GetValue(char* nbox)
{
    double ValueIn = 0;
    char   GET[]   = "get ";
    char   VAL[]   = ".val";
    char   CB[100];
    strcpy(CB, GET);
    strcat(CB, nbox);
    strcat(CB, VAL);
    Nextion.print(CB);
    EndSend();
    GetTextIn();
    if (TextIn[0] == 'q') {
        ValueIn = TextIn[1]; // Collect and build 32 bit value from 4 bytes
        ValueIn += (TextIn[2] << 8);
        ValueIn += (TextIn[3] << 16);  
        ValueIn += (TextIn[4] << 24);
    }
    return ValueIn;
}

/*********************************************************************************************************************************/

bool GetButtonPress()
{
    bool ButtonPressed = false;
    char a;
    i = 0;
    if (Nextion.available()) {
        ButtonPressed = true;
        delay(10);
        while (Nextion.available()) {
            a = char(Nextion.read());
            if (a > 31 && a < 128) {
                WordsIn[i]     = a;
                if (WordsIn[i] == '$') WordsIn[i] = 0; 
                WordsIn[i + 1] = char(0);
            }
            if (i < CharsMax) ++i;
        }
    }
    return ButtonPressed;
}

/*********************************************************************************************************************************/

void SendValue1(char* nbox, int value)
{
    char CB[100];
    char NB[25];
    strcpy(CB, nbox);
    strcat(CB, Str(NB, value, 0));
    SendCommand(CB);
}

/*********************************************************************************************************************************/

uint8_t um(uint16_t bv) // convert to lower resolution
{
    return (map(bv, MINMICROS, MAXMICROS, 0, 100)); // lower res is enough on display
}

/*********************************************************************************************************************************/

void CheckTimer()
{
    if (FlightMode < 4 && !LostContactFlag) {
        Secs  = ((millis() - TimerMillis) / 1000) + PausedSecs;
        Hours = Secs / 3600;
        Secs %= 3600;
        Mins = Secs / 60;
        Secs %= 60;
    }
    if (CurrentView == FrontView) {
        if (LastSeconds != Secs) {
            SendValue(FrontView_Secs, Secs);
            SendValue(FrontView_Mins, Mins);
            SendValue(FrontView_Hours, Hours);
            LastSeconds = Secs;
        }
    }
}

/*********************************************************************************************************************************/

void ShowServoPos()
{
    char SticksView_Ch1[]    = "Ch1";
    char SticksView_Ch2[]    = "Ch2";
    char SticksView_Ch3[]    = "Ch3";
    char SticksView_Ch4[]    = "Ch4";
    char SticksView_Ch5[]    = "Ch5";
    char SticksView_Ch6[]    = "Ch6";
    char SticksView_Ch7[]    = "Ch7";
    char SticksView_Ch8[]    = "Ch8";
    char SticksView_Ch9[]    = "Ch9";
    char SticksView_Ch10[]   = "Ch10";
    char SticksView_Ch11[]   = "Ch11";
    char SticksView_Ch12[]   = "Ch12";
    char SticksView_Ch13[]   = "Ch13";
    char SticksView_Ch14[]   = "Ch14";
    char SticksView_Ch15[]   = "Ch15";
    char SticksView_Ch16[]   = "Ch16";
    char CalibrateView_Ch1[] = "Ch1";
    char CalibrateView_Ch2[] = "Ch2";
    char CalibrateView_Ch3[] = "Ch3";
    char CalibrateView_Ch4[] = "Ch4";
    char CalibrateView_Ch5[] = "Ch5";
    char CalibrateView_Ch6[] = "Ch6";
    char CalibrateView_Ch7[] = "Ch7";
    char CalibrateView_Ch8[] = "Ch8";
    char ChannelInput[]      = "Input";
    char ChannelOutput[]     = "Output";

    int l             = 0;
    int l1            = 0;
    int LeastDistance = 2; // if the change is small, don't re-display anything - to reduce flashing.

    if (CurrentView == CalibrateView) {
        if (abs(SendBuffer[0] - ShownBuffer[0]) > LeastDistance) {
            SendValue(CalibrateView_Ch1, um(SendBuffer[0]));
            ShownBuffer[0] = SendBuffer[0];
        }
        if (abs(SendBuffer[1] - ShownBuffer[1]) > LeastDistance) {
            SendValue(CalibrateView_Ch2, um(SendBuffer[1]));
            ShownBuffer[1] = SendBuffer[1];
        }
        if (abs(SendBuffer[2] - ShownBuffer[2]) > LeastDistance) {
            SendValue(CalibrateView_Ch3, um(SendBuffer[2]));
            ShownBuffer[2] = SendBuffer[2];
        }
        if (abs(SendBuffer[3] - ShownBuffer[3]) > LeastDistance) {
            SendValue(CalibrateView_Ch4, um(SendBuffer[3]));
            ShownBuffer[3] = SendBuffer[3];
        }
        if (abs(SendBuffer[4] - ShownBuffer[4]) > LeastDistance) {
            SendValue(CalibrateView_Ch5, um(SendBuffer[4]));
            ShownBuffer[4] = SendBuffer[4];
        }
        if (abs(SendBuffer[5] - ShownBuffer[5]) > LeastDistance) {
            SendValue(CalibrateView_Ch6, um(SendBuffer[5]));
            ShownBuffer[5] = SendBuffer[5];
        }
        if (abs(SendBuffer[6] - ShownBuffer[6]) > LeastDistance) {
            SendValue(CalibrateView_Ch7, um(SendBuffer[6]));
            ShownBuffer[6] = SendBuffer[6];
        }
        if (abs(SendBuffer[7] - ShownBuffer[7]) > LeastDistance) {
            SendValue(CalibrateView_Ch8, um(SendBuffer[7]));
            ShownBuffer[7] = SendBuffer[7];
        }
    }
    if (CurrentView == SticksView) {
        if (abs(SendBuffer[0] - ShownBuffer[0]) > LeastDistance) {
            SendValue(SticksView_Ch1, um(SendBuffer[0]));
            ShownBuffer[0] = SendBuffer[0];
        }
        if (abs(SendBuffer[1] - ShownBuffer[1]) > LeastDistance) {
            SendValue(SticksView_Ch2, um(SendBuffer[1]));
            ShownBuffer[1] = SendBuffer[1];
        }
        if (abs(SendBuffer[2] - ShownBuffer[2]) > LeastDistance) {
            SendValue(SticksView_Ch3, um(SendBuffer[2]));
            ShownBuffer[2] = SendBuffer[2];
        }
        if (abs(SendBuffer[3] - ShownBuffer[3]) > LeastDistance) {
            SendValue(SticksView_Ch4, um(SendBuffer[3]));
            ShownBuffer[3] = SendBuffer[3];
        }
        if (abs(SendBuffer[4] - ShownBuffer[4]) > LeastDistance) {
            SendValue(SticksView_Ch5, um(SendBuffer[4]));
            ShownBuffer[4] = SendBuffer[4];
        }
        if (abs(SendBuffer[5] - ShownBuffer[5]) > LeastDistance) {
            SendValue(SticksView_Ch6, um(SendBuffer[5]));
            ShownBuffer[5] = SendBuffer[5];
        }
        if (abs(SendBuffer[6] - ShownBuffer[6]) > LeastDistance) {
            SendValue(SticksView_Ch7, um(SendBuffer[6]));
            ShownBuffer[6] = SendBuffer[6];
        }
        if (abs(SendBuffer[7] - ShownBuffer[7]) > LeastDistance) {
            SendValue(SticksView_Ch8, um(SendBuffer[7]));
            ShownBuffer[7] = SendBuffer[7];
        }
        if (abs(SendBuffer[8] - ShownBuffer[8]) > LeastDistance) {
            SendValue(SticksView_Ch9, um(SendBuffer[8]));
            ShownBuffer[8] = SendBuffer[8];
        }
        if (abs(SendBuffer[9] - ShownBuffer[9]) > LeastDistance) {
            SendValue(SticksView_Ch10, um(SendBuffer[9]));
            ShownBuffer[9] = SendBuffer[9];
        }
        if (abs(SendBuffer[10] - ShownBuffer[10]) > LeastDistance) {
            SendValue(SticksView_Ch11, um(SendBuffer[10]));
            ShownBuffer[10] = SendBuffer[10];
        }
        if (abs(SendBuffer[11] - ShownBuffer[11]) > LeastDistance) {
            SendValue(SticksView_Ch12, um(SendBuffer[11]));
            ShownBuffer[11] = SendBuffer[11];
        }
        if (abs(SendBuffer[12] - ShownBuffer[12]) > LeastDistance) {
            SendValue(SticksView_Ch13, um(SendBuffer[12]));
            ShownBuffer[12] = SendBuffer[12];
        }
        if (abs(SendBuffer[13] - ShownBuffer[13]) > LeastDistance) {
            SendValue(SticksView_Ch14, um(SendBuffer[13]));
            ShownBuffer[13] = SendBuffer[13];
        }
        if (abs(SendBuffer[14] - ShownBuffer[14]) > LeastDistance) {
            SendValue(SticksView_Ch15, um(SendBuffer[14]));
            ShownBuffer[14] = SendBuffer[14];
        }
        if (abs(SendBuffer[15] - ShownBuffer[15]) > LeastDistance) {
            SendValue(SticksView_Ch16, um(SendBuffer[15]));
            ShownBuffer[15] = SendBuffer[15];
        }
    }
    if (CurrentView == GraphView) { // Display current stick and output values
        if (ChanneltoSet <= 8) {
            l  = (InPutStick[ChanneltoSet - 1]);
            l1 = analogRead(AnalogueInput[l]);
            if (l1 <= ChannelCentre[l]) {
                SendValue(ChannelInput, map(l1, ChannelCentre[l], ChannelMin[l], 0, -100));
            }
            else {
                SendValue(ChannelInput, map(l1, ChannelCentre[l], ChannelMax[l], 0, 100));
            }
            SendValue(ChannelOutput, map(SendBuffer[ChanneltoSet - 1], MINMICROS, MAXMICROS, -100, 100));
        }
    }
    else {
        SendValue(ChannelInput, 0);
        SendValue(ChannelOutput, 0);
    }
}

/*********************************************************************************************************************************/

/** @brief SHOW COMMS */
FASTRUN void ShowComms()
{
    if (Nextion.available()) return; // was a button pressed?
    bool  ShowNow                = false;
    char  na[]                   = "";
    char  FrontView_Connected[]  = "Connected";
    char  FrontView_AckPayload[] = "AckPayload";
    char  FrontView_RXBV[]       = "RXBV";
    char  FrontView_TXBV[]       = "TXBV";
    char  Not_Connected[]        = "Not connected";
    char  Msg_Connected[]        = "** Connected! **";
    char  Msg_CnctdBuddyMast[]   = "Connected BUDDY MASTER";
    char  Msg_CnctdBuddySlave[]  = "Connected BUDDY SLAVE";
    char  MsgBuddying[]          = "Buddy";
    char  DataView_pps[]         = "pps";       // These are all label names in the Nextion data screen. They are best kept short.
    char  DataView_lps[]         = "lps";
    char  DataView_Alt[]         = "alt";
    char  DataView_Temp[]        = "Temp";
    char  DataView_MaxAlt[]      = "MaxAlt";
    char  DataView_txv[]         = "txv";
    char  DataView_rxv[]         = "rxv";
    char  DataView_Ls[]          = "Ls";
    char  DataView_Ts[]          = "Ts";
    char  DataView_Rx[]          = "rx";
    char  DataView_Sg[]          = "Sg";
    char  DataView_Ag[]          = "Ag";
    char  DataView_Gc[]          = "Gc";
    char  WarnNow[]              = "vis Warning,1";
    char  WarnOff[]              = "vis Warning,0";
    char  TXVolts[]              = "t21";
    float Volts                  = 0;
    char  Vbuf[16];
    char  pc[]      = "%";
    char  v[]       = "V, ";
    float txv       = 0;
    int   txpc      = 0;
    char  LiFe2s[]  = " 2s LiFe,  ";
    char  LiPo2s[]  = " 2s LiPo,  ";
    char  LiPo3s[]  = " 3s LiPo,  ";
    char  LiPo4s[]  = " 4s LiPo,  ";
    char  LiPo5s[]  = " 5s LiPo,  ";
    char  LiPo6s[]  = " 6s LiPo,  ";
    char  PerCell[] = "V/C";
    char  RXBattInfo[65];
    char  RXBattNA[] = "(No data)";
    char  RXBattNV[] = "    ";
    char  TXBattInfo[65];
    float ReadVolts           = 0;
    float VoltsPerCell        = 0;
    char  BindButtonVisible[] = "vis bind,1";
    char  Fix[]               = "Fix";   // These are all label names in the Nextion data screen. They are best kept short.
    char  Lon[]               = "Lon";
    char  Lat[]               = "Lat";
    char  Bear[]              = "Bear";
    char  Dist[]              = "Dist";
    char  Sped[]              = "Sped";
    char  yes[]               = "Yes";
    char  no[]                = "No";
    char  ALT[]               = "ALT";
    char  MALT[]              = "MALT";
    char  MxS[]               = "MxS";
    char  Mxd[]               = "Mxd";
    char  BTo[]               = "BTo";
    char  Sat[]               = "Sat";

    if (CurrentView == FrontView || CurrentView == DataView) {
        if (millis() - LastShowTime > ShowCommsDelay) { 
            ShowNow = true;
        }
    }
    if (ShowNow)
    {
        LastShowTime = millis();
        if (USE_INA219) {
            txv  = (ina219.getBusVoltage_V()) * 100;
            txpc = map(txv, 512, 670, 0, 100); // LiFePo4 Battery 2.6 ->3.5  volts per cell
            if (txpc < LOWBATTERY) TXWarningFlag = true;
            if (txpc > 100) txpc = 100; // avoid showing > 100% !
            dtostrf(txpc, 0, 0, Vbuf);
            strcat(Vbuf, pc);
            if (CurrentView == FrontView) SendText(TXVolts, Vbuf);
            txv /= 100;
            snprintf(Vbuf, 5, "%f", txv); // float to string...
            strcpy(TXBattInfo, Vbuf);
            strcat(TXBattInfo, v);
            txv /= 2;
            strcat(TXBattInfo, LiFe2s);
            dtostrf(txv, 2, 2, Vbuf);
            strcat(TXBattInfo, Vbuf);
            strcat(TXBattInfo, PerCell);
            if (CurrentView == FrontView) SendText(FrontView_TXBV, TXBattInfo);
            if (CurrentView == DataView) SendText(DataView_txv, TransmitterVersionNumber); // TX Version Number
                                                                                           // if (CurrentView == DataView) SendText(DataView_txv, Vbuf);
        }
        if (!LostContactFlag)
        {
            if ((CurrentView == FrontView)) {
                if (!BoundFlag) {
                    SendCommand(BindButtonVisible);
                    BindButton = true;
                }
                else {
                    if (BoundFlag)
                    {
                        if (!BuddyMaster)
                        {
                            SendText(FrontView_Connected, Msg_Connected);
                        }
                        else
                        {
                            if (!SlaveHasControl)
                            {
                                SendText(FrontView_Connected, Msg_CnctdBuddyMast);
                            }
                            else
                            {
                                SendText(FrontView_Connected, Msg_CnctdBuddySlave);
                            }
                        }
                        GreenLedOn();
                        StartInactvityTimeout();
                    }
                }
            }
            if (CurrentView == DataView) {
                SendValue(DataView_pps,   PacketsPerSecond);
                SendValue(DataView_lps,   LostPackets);
                SendText(DataView_Alt,    ModelAltitude);
                SendText(DataView_MaxAlt, MaxAltitude);
                SendText(DataView_Temp,   ModelTemperature);
                SendText(DataView_Rx,     ThisRadio);
                SendText(DataView_rxv,    ReceiverVersionNumber);
                SendValue(DataView_Ls,    GapLongest);
                SendValue(DataView_Ts,    GapSum);
                SendValue(DataView_Sg,    GapShortest);
                SendValue(DataView_Ag,    GapAverage);
                SendValue(DataView_Gc,    GapCount);
                if (GpsFix){                               // if no fix, then leave display as before 
                    SendText(Fix, yes);
                } else {
                    SendText(Fix, no);
                }                   
                snprintf(Vbuf, 3,"%d", GPSSatellites);
                SendText(Sat,Vbuf);
                snprintf(Vbuf, 10,"%f", GPSLongitude);
                SendText(Lon,Vbuf);
                snprintf(Vbuf, 10,"%f", GPSLatitude);
                SendText(Lat,Vbuf);
                snprintf(Vbuf, 7,"%d",  int(GPSAngle));
                SendText(Bear,Vbuf);
                snprintf(Vbuf, 6,"%d", (int) GPSDistanceTo);
                SendText(Dist,Vbuf);
                snprintf(Vbuf, 4,"%d",  (int) GPSSpeed);
                SendText(Sped,Vbuf);
                snprintf(Vbuf, 4,"%d",  (int) GPSMaxSpeed);
                SendText(MxS,Vbuf);   
                snprintf(Vbuf, 4,"%d",  (int) GPSAltitude);
                SendText(ALT,Vbuf);   
                snprintf(Vbuf, 4,"%d",  (int) GPSMaxAltitude);
                SendText(MALT,Vbuf);   
                snprintf(Vbuf, 4,"%d",  (int) GPSCourseTo);
                SendText(BTo,Vbuf);   
                snprintf(Vbuf, 6,"%d",  (int) GPSMaxDistance);
                SendText(Mxd,Vbuf);   
                
            }
            ReadVolts = RXModelVolts * 10;
            // 6s Max 25.2 -> 20.4
            // 5s Max 21.0 -> 17.0
            // 4s Max 16.8 -> 13.6
            // 3s Max 12.6 -> 10.2
            // 2s Max 8.4  -> 6.8
            if (RXCellCount == 6) Volts = map(ReadVolts, 204, 252, 0, 100); // works for 6s batteries (*10)
            if (RXCellCount == 5) Volts = map(ReadVolts, 170, 210, 0, 100); // works for 5s batteries (*10)
            if (RXCellCount == 4) Volts = map(ReadVolts, 136, 168, 0, 100); // works for 4s batteries (*10)
            if (RXCellCount == 3) Volts = map(ReadVolts, 102, 126, 0, 100); // works for 3s batteries (*10)
            if (RXCellCount == 2) Volts = map(ReadVolts, 68, 84, 0, 100);   // works for 2s batteries (*10)

            if (VoltsDetected) {
                Volts = constrain(Volts, 0, 100);
                if (Volts <= LOWBATTERY && Volts > 0) {
                    RXWarningFlag = true;
                }
                else {
                    RXWarningFlag = false;
                }
            }
            if (VoltsDetected) {
                dtostrf(Volts, 0, 0, Vbuf);
                strcat(Vbuf, pc);
                if (BoundFlag && CurrentView == FrontView) SendText(FrontView_AckPayload, Vbuf);

                strcpy(RXBattInfo, ModelVolts);

                strcat(RXBattInfo, v);
                if (RXCellCount == 6) strcat(RXBattInfo, LiPo6s);
                if (RXCellCount == 5) strcat(RXBattInfo, LiPo5s);
                if (RXCellCount == 4) strcat(RXBattInfo, LiPo4s);
                if (RXCellCount == 3) strcat(RXBattInfo, LiPo3s);
                if (RXCellCount == 2) strcat(RXBattInfo, LiPo2s);
                VoltsPerCell = (ReadVolts / RXCellCount) / 10;
                dtostrf(VoltsPerCell, 2, 2, Vbuf);
                strcat(RXBattInfo, Vbuf);
                strcat(RXBattInfo, PerCell);
                if (BoundFlag && CurrentView == FrontView) SendText(FrontView_RXBV, RXBattInfo);
            }
            if (!VoltsDetected) {
                if (BoundFlag && CurrentView == FrontView) SendText(FrontView_RXBV, RXBattNA);
                if (BoundFlag && CurrentView == FrontView) SendText(FrontView_AckPayload, RXBattNV);
            }
        }
        else {
            if (BoundFlag) {
                if (CurrentView == DataView) {
                    PacketsPerSecond = 0;
                    SendValue(DataView_pps, PacketsPerSecond);
                    SendValue(DataView_lps, LostPackets);
                }

                if (CurrentView == FrontView) {
                    SendText(FrontView_Connected, Not_Connected);
                    SendText(FrontView_RXBV, na); // data not available
                    SendText(FrontView_AckPayload, na);
                }
            }
            else // i.e. contact is lost
            {
                if (CurrentView == FrontView)
                {
                    if (DoSbusSendOnly) {
                        SendText(FrontView_Connected, MsgBuddying);
                    }
                    else {
                        SendText(FrontView_Connected, Not_Connected);
                    }
                }
            }
        }
    }
    if (CurrentView == FrontView) {
        if (RXWarningFlag || TXWarningFlag) {
            SendCommand(WarnNow);
        }
        if (!RXWarningFlag) {
            if (!TXWarningFlag) {
                SendCommand(WarnOff);
            }
        }
    }
} // end ShowComms()

/*********************************************************************************************************************************/

void FailedPacket()
{
    int secondsRemaining;
    if (GapStart == 0) {
        GapStart = millis(); // To keep track of gaps' length
    }
    ++RecentPacketsLost;
    ++TotalledRecentPacketsLost; // this is to keep track of events when receiver is off
    if (RecentPacketsLost > LOSTCONTACTCUTOFF) {
        LostContactFlag   = true;
        RecentPacketsLost = 0;
        if ((millis() - GapStart) > RED_LED_ON_TIME) // there's no need to blink red for every single lost packet. Only after 1/2 second of no connection.
        {
            RedLedOn();
        }
    }
    ++LostPackets;
    secondsRemaining = (Inactivity_Timeout / 1000) - (millis() - Inactivity_Start) / 1000;
    if (secondsRemaining <= 0) digitalWrite(POWER_OFF_PIN, HIGH); // INACTIVITY POWER OFF
}

/*********************************************************************************************************************************/

/** Send 13 joined together char arrays to Nextion */
void SendCharArray(char* ch0, char* ch1, char* ch2, char* ch3, char* ch4, char* ch5, char* ch6, char* ch7, char* ch8, char* ch9, char* ch10, char* ch11, char* ch12)
{
    strcpy(ch0, ch1);
    strcat(ch0, ch2);
    strcat(ch0, ch3);
    strcat(ch0, ch4);
    strcat(ch0, ch5);
    strcat(ch0, ch6);
    strcat(ch0, ch7);
    strcat(ch0, ch8);
    strcat(ch0, ch9);
    strcat(ch0, ch10);
    strcat(ch0, ch11);
    strcat(ch0, ch12);
    SendCommand(ch0);
}

/*********************************************************************************************************************************/

#define xx1 90 // was 75
#define yy1 90 // Needed below... Edit xx1,yy1 to move box

void DrawFhssBox()
{
    int  x1          = xx1;
    int  y1          = yy1;
    int  x2          = x1 + (128 * 5);
    int  y2          = y1 + 255;
    int  xd          = 20; // half of 40 which is character width
    int  xd1         = 24;
    char STR126[]    = "\"127\"";
    char STR126GHZ[] = "\"2.527\"";
    char STR96[]     = "\"96\"";
    char STR96GHZ[]  = "\"2.496\"";
    char STR64[]     = "\"64\"";
    char STR64GHZ[]  = "\"2.464\"";
    char STR32[]     = "\"32\"";
    char STR32GHZ[]  = "\"2.432\"";
    char STR1[]      = "\"0\"";
    char STR1GHZ[]   = "\"2.400\"";
    char GHZ[]       = "\"GHz:\"";
    char CH[]        = "\"Ch:\"";
    char CB[150]; // COMMAND BUFFER
    char draw[] = "draw ";
    char xstr[] = "xstr ";
    char fyll[] = "fill ";
    char NB[12]; // Number Buffers
    char NB1[12];
    char NB2[12];
    char NB3[12];
    char NB4[12];
    char NB5[12];
    char NB6[12];
    char NB7[12];
    char NB8[12];
    char NA[1]    = ""; // blank one
    char whyte[]  = "WHITE";
    char whyte1[] = "WHITE,"; // ... with a comma

    SendCharArray(CB, draw, Str(NB1, x1, 1), Str(NB2, y1, 1), Str(NB3, x2, 1), Str(NB4, y2, 1), whyte, NA, NA, NA, NA, NA, NA);
    SendCharArray(CB, xstr, Str(NB, x1 - xd, 1), Str(NB1, y2 + 4, 1), Str(NB2, 40, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), whyte1, Str(NB5, 214, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR1);
    SendCharArray(CB, xstr, Str(NB, 30, 1), Str(NB1, y2 + 4, 1), Str(NB2, 40, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), whyte1, Str(NB5, 214, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), CH);
    SendCharArray(CB, xstr, Str(NB, 16, 1), Str(NB1, y2 + 25, 1), Str(NB2, 50, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), whyte1, Str(NB5, 214, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), GHZ);
    SendCharArray(CB, xstr, Str(NB, x1 - xd1, 1), Str(NB1, y2 + 25, 1), Str(NB2, 60, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), whyte1, Str(NB5, 214, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR1GHZ);
    SendCharArray(CB, xstr, Str(NB, x1 + ((x2 - x1) / 4) - xd, 1), Str(NB1, y2 + 4, 1), Str(NB2, 40, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), whyte1, Str(NB5, 214, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR32);
    SendCharArray(CB, xstr, Str(NB, (x1 + ((x2 - x1) / 4) - xd1), 1), Str(NB1, y2 + 25, 1), Str(NB2, 60, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), whyte1, Str(NB5, 214, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR32GHZ);
    SendCharArray(CB, xstr, Str(NB, (x1 + ((x2 - x1) / 2) - xd), 1), Str(NB1, y2 + 4, 1), Str(NB2, 40, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), whyte1, Str(NB5, 214, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR64);
    SendCharArray(CB, xstr, Str(NB, (x1 + ((x2 - x1) / 2) - xd1), 1), Str(NB1, y2 + 25, 1), Str(NB2, 60, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), whyte1, Str(NB5, 214, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR64GHZ);
    SendCharArray(CB, xstr, Str(NB, (x1 + (((x2 - x1) / 4) * 3) - xd), 1), Str(NB1, y2 + 4, 1), Str(NB2, 40, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), whyte1, Str(NB5, 214, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR96);
    SendCharArray(CB, xstr, Str(NB, (x1 + (((x2 - x1) / 4) * 3) - xd1), 1), Str(NB1, y2 + 25, 1), Str(NB2, 60, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), whyte1, Str(NB5, 214, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR96GHZ);
    SendCharArray(CB, xstr, Str(NB, (x2 - xd), 1), Str(NB1, y2 + 4, 1), Str(NB2, 40, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), whyte1, Str(NB5, 214, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR126);
    SendCharArray(CB, xstr, Str(NB, (x2 - xd1), 1), Str(NB1, y2 + 25, 1), Str(NB2, 60, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), whyte1, Str(NB5, 214, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR126GHZ);
    SendCharArray(CB, fyll, Str(NB, (x1 + 1), 1), Str(NB1, (y1 + 1), 1), Str(NB2, ((128 * 5) - 2), 1), Str(NB3, 254, 1), Str(NB4, 214, 0), NA, NA, NA, NA, NA, NA);
}

/*********************************************************************************************************************************/

void SendMixValues()
{

    char MixesView_Enabled[]       = "MixesView.Enabled";
    char MixesView_FlightMode[]    = "MixesView.FlightMode";
    char MixesView_MasterChannel[] = "MixesView.MasterChannel";
    char MixesView_SlaveChannel[]  = "MixesView.SlaveChannel";
    char MixesView_Reversed[]      = "MixesView.Reversed";
    char MixesView_Percent[]       = "MixesView.Percent";
    char MixesView_h0[]            = "MixesView.h0"; // =the slider control
    char MixesView_chM[]           = "MixesView.chM";
    char MixesView_chS[]           = "MixesView.chS";

    SendText(MixesView_chM, ChannelNames[Mixes[MixNumber][M_MasterChannel] - 1]);
    SendText(MixesView_chS, ChannelNames[Mixes[MixNumber][M_SlaveChannel] - 1]);
    SendValue(MixesView_Enabled, Mixes[MixNumber][M_Enabled]);
    SendValue(MixesView_FlightMode, Mixes[MixNumber][M_FlightMode]);
    if (Mixes[MixNumber][M_MasterChannel] == 0) Mixes[MixNumber][M_MasterChannel] = 1;
    SendValue(MixesView_MasterChannel, Mixes[MixNumber][M_MasterChannel]);
    if (Mixes[MixNumber][M_SlaveChannel] == 0) Mixes[MixNumber][M_SlaveChannel] = 1;
    SendValue(MixesView_SlaveChannel, Mixes[MixNumber][M_SlaveChannel]);
    SendValue(MixesView_Reversed, Mixes[MixNumber][M_Reversed]);
    if (Mixes[MixNumber][M_Percent] == 0) Mixes[MixNumber][M_Percent] = 100;
    if (Mixes[MixNumber][M_SlaveChannel] == Mixes[MixNumber][M_MasterChannel]) {
        Mixes[MixNumber][M_SlaveChannel]++;
        SendValue(MixesView_SlaveChannel, Mixes[MixNumber][M_SlaveChannel]);
    }
    SendValue(MixesView_Percent, Mixes[MixNumber][M_Percent]);
    SendValue(MixesView_h0, Mixes[MixNumber][M_Percent]);
}

/*********************************************************************************************************************************/

int GetNextNumber(int p1, char text1[CharsMax])
{
    char text2[CharsMax];
    int  j = 0;
    i      = p1 - 1;
    j      = 0;
    while (isDigit(text1[i]) && i < CharsMax) {
        text2[j] = text1[i];
        ++i;
        ++j;
        text2[j] = 0;
    }
    i = j; // = strlen only simpler
    if (i == 3) {
        j = (text2[0] - 48) * 100;
        j += (text2[1] - 48) * 10;
        j += (text2[2] - 48);
    }
    if (i == 2) {
        j = (text2[0] - 48) * 10;
        j += (text2[1] - 48);
    }
    if (i == 1) j = (text2[0] - 48);
    return j;
}

/*********************************************************************************************************************************/

short unsigned int GetStickInput(uint8_t l)
{
    // This bit needs to be optimised but it works well enough.
    short unsigned int k = 0;

    if (l == 8) {
        if (Channel9SwitchValue == 0) k = mp(MinDegrees[FlightMode][8]);
        if (Channel9SwitchValue == 90) k = mp(CentreDegrees[FlightMode][8]);
        if (Channel9SwitchValue == 180) k = mp(MaxDegrees[FlightMode][8]);
    }

    if (l == 9) {
        if (Channel10SwitchValue == 0) k = mp(MinDegrees[FlightMode][9]);
        if (Channel10SwitchValue == 90) k = mp(CentreDegrees[FlightMode][9]);
        if (Channel10SwitchValue == 180) k = mp(MaxDegrees[FlightMode][9]);
    }

    if (l == 10) {
        if (Channel11SwitchValue == 0) k = mp(MinDegrees[FlightMode][10]);
        if (Channel11SwitchValue == 90) k = mp(CentreDegrees[FlightMode][10]);
        if (Channel11SwitchValue == 180) k = mp(MaxDegrees[FlightMode][10]);
    }

    if (l == 11) {
        if (Channel12SwitchValue == 0) k = mp(MinDegrees[FlightMode][11]);
        if (Channel12SwitchValue == 90) k = mp(CentreDegrees[FlightMode][11]);
        if (Channel12SwitchValue == 180) k = mp(MaxDegrees[FlightMode][11]);
    }
    return k;
}

/*********************************************************************************************************************************/
// MIXES
/*********************************************************************************************************************************/

void DoMixes()
{
    int m, c, p, mindeg, maxdeg, TheSum, Result;
    for (m = 1; m <= MAXMIXES; ++m) {
        if (Mixes[m][M_FlightMode] == FlightMode || Mixes[m][M_FlightMode] == 0) {
            if (Mixes[m][M_Enabled] == 1) {
                for (c = 0; c < CHANNELSUSED; ++c) {
                    if ((Mixes[m][M_MasterChannel] - 1) == c) {
                        p = map(PreMixBuffer[c], MINMICROS, MAXMICROS, -HALFMICROSRANGE, HALFMICROSRANGE);
                        p = p * Mixes[m][M_Percent] / 50; // *****  50, not 100, because mix can now go right to 200% *****
                        if (Mixes[m][M_Reversed] == 1) p = -p;
                        TheSum = SendBuffer[(Mixes[m][M_SlaveChannel]) - 1] + p;
                        mindeg = mp(MinDegrees[FlightMode][(Mixes[m][M_SlaveChannel]) - 1]);
                        maxdeg = mp(MaxDegrees[FlightMode][(Mixes[m][M_SlaveChannel]) - 1]);
                        if (mindeg > maxdeg) {
                            Result = constrain(TheSum, maxdeg, mindeg);
                        }
                        else {
                            Result = constrain(TheSum, mindeg, maxdeg);
                        }
                        SendBuffer[(Mixes[m][M_SlaveChannel]) - 1] = Result;
                    }
                }
            }
        }
    }
}

/*********************************************************************************************************************************/

float MapExp(float xx, float Xxmin, float Xxmax, float Yymin, float Yymax, float Expo)
{
    Expo  = map(Expo, -100, 100, 0, 1);
    xx    = pow(xx * xx, Expo);
    Xxmin = pow(Xxmin * Xxmin, Expo);
    Xxmax = pow(Xxmax * Xxmax, Expo);
    return map(xx, Xxmin, Xxmax, Yymin, Yymax);
}

/*********************************************************************************************************************************/

/** @brief GET NEW SERVO POSITIONS */
void get_new_channels_values()
{
    
    uint16_t k = 0, l = 0, m = 0, n = 0, TrimAmount;

    for (n = 0; n < CHANNELSUSED; ++n) {
        l = InPutStick[n]; // input sticks knobs & switches are now mapped by user
        if (l <= 7)
        {
            m = analogRead(AnalogueInput[l]); // Get values from sticks' pots
        }

        if (l > 7)
        {                         // Switch ?
            k = GetStickInput(l); // Four 3 postion switches
        }
        else {                                            // Map the eight analogue inputs
            if (InterpolationTypes[FlightMode][n] == 0) { // Linear
                if (m >= ChannelMidHi[l]) k = map(m, ChannelMidHi[l], ChannelMax[l], mp(MidHiDegrees[FlightMode][n]), mp(MaxDegrees[FlightMode][n]));
                if (m >= ChannelCentre[l] && m <= (ChannelMidHi[l])) k = map(m, ChannelCentre[l], ChannelMidHi[l], mp(CentreDegrees[FlightMode][n]), mp(MidHiDegrees[FlightMode][n]));
                if (m >= ChannelMidLow[l] && m <= ChannelCentre[l]) k = map(m, ChannelMidLow[l], ChannelCentre[l], mp(MidLowDegrees[FlightMode][n]), mp(CentreDegrees[FlightMode][n]));
                if (m <= ChannelMidLow[l]) k = map(m, ChannelMin[l], ChannelMidLow[l], mp(MinDegrees[FlightMode][n]), mp(MidLowDegrees[FlightMode][n]));
            }

            if (InterpolationTypes[FlightMode][n] == 1) { // CatmullSpline (!)
                xPoints[0] = ChannelMin[l];
                xPoints[1] = ChannelMidLow[l];
                xPoints[2] = ChannelCentre[l];
                xPoints[3] = ChannelMidHi[l];
                xPoints[4] = ChannelMax[l];
                yPoints[4] = mp(MaxDegrees[FlightMode][n]);
                yPoints[3] = mp(MidHiDegrees[FlightMode][n]);
                yPoints[2] = mp(CentreDegrees[FlightMode][n]);
                yPoints[1] = mp(MidLowDegrees[FlightMode][n]);
                yPoints[0] = mp(MinDegrees[FlightMode][n]);
                k          = Interpolation::CatmullSpline(xPoints, yPoints, PointsCount, m);
            }
            if (InterpolationTypes[FlightMode][n] == 2) { // EXPONENTIAL (!!)
                if (m >= ChannelCentre[l]) {
                    k = MapExp(m - ChannelCentre[l], 0, ChannelMax[l] - ChannelCentre[l], 0, mp(MaxDegrees[FlightMode][n]) - mp(CentreDegrees[FlightMode][n]), Exponential[FlightMode][n]) + mp(CentreDegrees[FlightMode][n]);
                }
                if (m < ChannelCentre[l]) {
                    k = MapExp(ChannelCentre[l] - m, 0, ChannelCentre[l] - ChannelMin[l], mp(CentreDegrees[FlightMode][n]) - mp(MinDegrees[FlightMode][n]), 0, Exponential[FlightMode][n]) + mp(MinDegrees[FlightMode][n]);
                }
            }
        }
        if (n < 4) {
            TrimAmount = (Trims[FlightMode][n] - 80) * TrimFactor;  // TRIMS on lower four channels (80 is mid point !! (range 40 - 120))
            if (!TrimsReversed[FlightMode][n]) {
                k += TrimAmount; 
            }
            else {
                k -= TrimAmount;
            }
        }

        if (!CalibratedYet) k = map(m, 0, 1024, MINMICROS, MAXMICROS); // Crude servos until calibrated
        PreMixBuffer[n] = constrain(k, MINMICROS, MAXMICROS);
        k               = 1500;
        SendBuffer[n]   = PreMixBuffer[n];
    }
    if (CurrentMode == NORMAL) DoMixes(); // not while calibrating
}

/*********************************************************************************************************************************/

void CalibrateSticks()
{
    int j;
    for (i = 0; i < PROPOCHANNELS; ++i)
    {
        j = analogRead(AnalogueInput[i]);
        if (ChannelMax[i] < j) ChannelMax[i] = j;
        if (ChannelMin[i] > j) ChannelMin[i] = j;
    }
    get_new_channels_values();
    ShowServoPos();
    SendCommand(NoSleeping); // no screen timeout while calibrating
}

/*********************************************************************************************************************************/

void CentreMaxMins()
{
    for (i = 0; i < CHANNELSUSED; ++i)
    {
        ChannelMax[i]    = 512; // halfway up
        ChannelMin[i]    = 512; // halfway down
        ChannelCentre[i] = 512;
    }
}

/*********************************************************************************************************************************/

void UpdateTrimView()
{
    SendValue(TrimView_ch1, (Trims[FlightMode][0]));
    SendValue(TrimView_ch4, (Trims[FlightMode][1]));
    SendValue(TrimView_ch2, (Trims[FlightMode][2]));
    SendValue(TrimView_ch3, (Trims[FlightMode][3]));

    SendValue(TrimView_n1, (Trims[FlightMode][0] - 80));
    SendValue(TrimView_n4, (Trims[FlightMode][1] - 80));
    SendValue(TrimView_n2, (Trims[FlightMode][2] - 80));
    SendValue(TrimView_n3, (Trims[FlightMode][3] - 80));

    SendValue(TrimView_r1, TrimsReversed[FlightMode][0]);
    SendValue(TrimView_r4, TrimsReversed[FlightMode][1]);
    SendValue(TrimView_r2, TrimsReversed[FlightMode][2]);
    SendValue(TrimView_r3, TrimsReversed[FlightMode][3]);
}

/*********************************************************************************************************************************/

void ScanI2c()
{
    int ii;
    USE_INA219 = false;
    for (ii = 1; ii < 127; ++ii) {
        Wire.beginTransmission(ii);
        if (Wire.endTransmission() == 0) {
#ifdef DB_SENSORS
            Serial.print(ii, HEX); // in case new one shows up
            Serial.print("   ");
            if (ii == 0x40) Serial.println("INA219 voltage meter detected!");
#endif
            if (ii == 0x40) {
                USE_INA219 = true;
            }
        }
    }
}

/*********************************************************************************************************************************/

void CloseModelsFile()
{
    if (ModelsFileOpen) {
        ModelsFileNumber.close();
        ModelsFileOpen = false;
    }
}

/*********************************************************************************************************************************/

void OpenModelsFile()
{
    if (SingleModelFlag) {
        ModelsFileNumber = SD.open(SingleModelFile, FILE_WRITE);
    }
    else {
        ModelsFileNumber = SD.open(ModelsFile, FILE_WRITE);
    }
    if (ModelsFileNumber == 0) FileError = true;
    ModelsFileOpen = true;
}

/*********************************************************************************************************************************/

void SDUpdateInt(int p_address, int p_value)
{
    ModelsFileNumber.seek(p_address);
    ModelsFileNumber.write(uint8_t(p_value));
    ModelsFileNumber.write(uint8_t(p_value >> 8));
}

/*********************************************************************************************************************************/

void SDUpdateByte(int p_address, uint8_t p_value)
{
    ModelsFileNumber.seek(p_address);
    ModelsFileNumber.write(uint8_t(p_value));
}

/*********************************************************************************************************************************/

int SDReadInt(int p_address)
{
    ModelsFileNumber.seek(p_address);
    int r = ModelsFileNumber.read();
    r += ModelsFileNumber.read() << 8;
    return r;
}
/*********************************************************************************************************************************/

uint8_t SDReadByte(int p_address)
{
    ModelsFileNumber.seek(p_address);
    uint8_t r = ModelsFileNumber.read();
    return r;
}

/*********************************************************************************************************************************/

void UpdateModelsNameEveryWhere()
{ // ... and flight mode ... and trim settings etc...
    char fm1[]                  = "Flight mode 1";
    char fm2[]                  = "Flight mode 2";
    char fm3[]                  = "Flight mode 3";
    char fm4[]                  = "Flight mode 4";
    char FrontView_ModelName[]  = "ModelName";
    char SticksView_ModelName[] = "ModelName";
    char MixesView_ModelName[]  = "ModelName";
    char ModelsView_ModelName[] = "ModelName";
    char GraphView_ModelName[]  = "ModelName";
    char GraphView_Channel[]    = "Channel";
    char TrimView_ModelName[]   = "ModelName";
    char TrimView_FlightMode[]  = "t1";
    char GraphView_fmode[]      = "fmode";
    char SticksView_t1[]        = "t1";
    char FrontView_fm1[]        = "fm1";
    char FrontView_fm2[]        = "fm2";
    char FrontView_fm3[]        = "fm3";
    char FrontView_fm4[]        = "fm4";
    char NoName[17];
    char Ch[] = "Channel ";
    char Nbuf[7];

    switch (CurrentView) {
        case FrontView:
            SendText(FrontView_ModelName, ModelName);
            break;
        case SticksView:
            SendText(SticksView_ModelName, ModelName);
            break;
        case MixesView:
            SendText(MixesView_ModelName, ModelName);
            break;
        case GraphView:
            SendText(GraphView_ModelName, ModelName);
            if (strlen(ChannelNames[ChanneltoSet - 1]) < 2) { // if no name, just show the channel number
                strcpy(NoName, Ch);
                SendText(GraphView_Channel, strcat(NoName, Str(Nbuf, ChanneltoSet, 0)));
            }
            else {
                SendText(GraphView_Channel, ChannelNames[ChanneltoSet - 1]);
            }
            break;
        case ModelsView:
            SendText(ModelsView_ModelName, ModelName);
            break;
        case Trim_View:
            SendText(TrimView_ModelName, ModelName);
            UpdateTrimView();
            break;
        default:
            break;
    }

    if (FlightMode == 1) {
        if (CurrentView == SticksView) SendText(SticksView_t1, fm1);
        if (CurrentView == GraphView) SendText(GraphView_fmode, fm1);
        if (CurrentView == Trim_View) {
            SendText(TrimView_FlightMode, fm1);
            UpdateTrimView();
        }
        if (CurrentView == FrontView) SendValue(FrontView_fm1, 1);
    }
    if (FlightMode == 2) {
        if (CurrentView == SticksView) SendText(SticksView_t1, fm2);
        if (CurrentView == GraphView) SendText(GraphView_fmode, fm2);
        if (CurrentView == Trim_View) {
            SendText(TrimView_FlightMode, fm2);
            UpdateTrimView();
        }
        if (CurrentView == FrontView) SendValue(FrontView_fm2, 1);
    }
    if (FlightMode == 3) {
        if (CurrentView == SticksView) SendText(SticksView_t1, fm3);
        if (CurrentView == GraphView) SendText(GraphView_fmode, fm3);
        if (CurrentView == Trim_View) {
            SendText(TrimView_FlightMode, fm3);
            UpdateTrimView();
        }
        if (CurrentView == FrontView) SendValue(FrontView_fm3, 1);
    }
    if (FlightMode == 4) {
        if (CurrentView == SticksView) SendText(SticksView_t1, fm4);
        if (CurrentView == GraphView) SendText(GraphView_fmode, fm4);
        if (CurrentView == Trim_View) {
            SendText(TrimView_FlightMode, fm4);
            UpdateTrimView();
        }
        if (CurrentView == FrontView) SendValue(FrontView_fm4, 1);
    }
}

/*********************************************************************************************************************************/

void InitSwitches()
{
    for (i = 0; i < 8; ++i) {
        pinMode(SwitchNumber[i], INPUT_PULLUP);
    }
}

/*********************************************************************************************************************************/

/** @brief STICKS CALIBRATION */
void InitMaxMin()
{
    for (i = 0; i < CHANNELSUSED; ++i) {
        ChannelMax[i]    = 1024;
        ChannelMidHi[i]  = 512 + 256;
        ChannelCentre[i] = 512;
        ChannelMidLow[i] = 256;
        ChannelMin[i]    = 0;
    }
}

/*********************************************************************************************************************************/

void CentreTrims()
{
    for (int j = 0; j <= FlightModesUsed; ++j) {
        for (i = 0; i < CHANNELSUSED; ++i) {
            Trims[j][i] = 80;
        }
    }
}

/*********************************************************************************************************************************/

void InitCentreDegrees()
{
    int j;
    for (j = 1; j <= 4; ++j) {
        for (i = 0; i < CHANNELSUSED; ++i) {
            MaxDegrees[j][i]    = 180; //  180 degrees
            MidHiDegrees[j][i]  = 135;
            CentreDegrees[j][i] = 90; //  90 degrees
            MidLowDegrees[j][i] = 45;
            MinDegrees[j][i]    = 0; //  0 degrees
        }
    }
}

/*********************************************************************************************************************************/

/** @brief Get centre as 90 degrees */
void ChannelCentres()
{
    for (i = 0; i < 8; ++i) {
        ChannelCentre[i] = analogRead(AnalogueInput[i]);
        ChannelMidHi[i]  = ChannelCentre[i] + ((ChannelMax[i] - ChannelCentre[i]) / 2);
        ChannelMidLow[i] = ChannelMin[i] + ((ChannelCentre[i] - ChannelMin[i]) / 2);
    }
    get_new_channels_values();
    ShowServoPos();
}

/*********************************************************************************************************************************/

void UpdateButtonLabels()
{
    char InPutStick_c1[]  = "c1";
    char InPutStick_c2[]  = "c2";
    char InPutStick_c3[]  = "c3";
    char InPutStick_c4[]  = "c4";
    char InPutStick_c5[]  = "c5";
    char InPutStick_c6[]  = "c6";
    char InPutStick_c7[]  = "c7";
    char InPutStick_c8[]  = "c8";
    char InPutStick_c9[]  = "c9";
    char InPutStick_c10[] = "c10";
    char InPutStick_c11[] = "c11";
    char InPutStick_c12[] = "c12";
    char InPutStick_c13[] = "c13";
    char InPutStick_c14[] = "c14";
    char InPutStick_c15[] = "c15";
    char InPutStick_c16[] = "c16";

    char fsch1[]   = "ch1";
    char fsch2[]   = "ch2";
    char fsch3[]   = "ch3";
    char fsch4[]   = "ch4";
    char fsch5[]   = "ch5";
    char fsch6[]   = "ch6";
    char fsch7[]   = "ch7";
    char fsch8[]   = "ch8";
    char fsch9[]   = "ch9";
    char fsch10[]  = "ch10";
    char fsch11[]  = "ch11";
    char fsch12[]  = "ch12";
    char fsch13[]  = "ch13";
    char fsch14[]  = "ch14";
    char fsch15[]  = "ch15";
    char fsch16[]  = "ch16";
    char arrowrh[] = " >";
    char arrowlh[] = "< ";
    char BoxOffsetLabel[20];
    char SticksViewButton1[]  = "Sch1";
    char SticksViewButton2[]  = "Sch2";
    char SticksViewButton3[]  = "Sch3";
    char SticksViewButton4[]  = "Sch4";
    char SticksViewButton5[]  = "Sch5";
    char SticksViewButton6[]  = "Sch6";
    char SticksViewButton7[]  = "Sch7";
    char SticksViewButton8[]  = "Sch8";
    char SticksViewButton9[]  = "Sch9";
    char SticksViewButton10[] = "Sch10";
    char SticksViewButton11[] = "Sch11";
    char SticksViewButton12[] = "Sch12";
    char SticksViewButton13[] = "Sch13";
    char SticksViewButton14[] = "Sch14";
    char SticksViewButton15[] = "Sch15";
    char SticksViewButton16[] = "Sch16";
    char one[]                = " (1)";
    char two[]                = "(2) ";
    char three[]              = " (3)";
    char four[]               = "(4) ";
    char five[]               = " (5)";
    char six[]                = "(6) ";
    char seven[]              = " (7)";
    char eight[]              = "(8) ";
    char nine[]               = " (9)";
    char ten[]                = "(10) ";
    char eleven[]             = " (11)";
    char twelve[]             = "(12) ";
    char thirteen[]           = " (13)";
    char fourteen[]           = "(14) ";
    char fifteen[]            = " (15)";
    char sixteen[]            = "(16) ";
    if (CurrentView == SticksView) {
        strcpy(BoxOffsetLabel, ChannelNames[0]);
        strcat(BoxOffsetLabel, one);
        strcat(BoxOffsetLabel, arrowrh);
        SendText(SticksViewButton1, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, arrowlh);
        strcat(BoxOffsetLabel, two);
        strcat(BoxOffsetLabel, ChannelNames[1]);
        SendText(SticksViewButton2, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, ChannelNames[2]);
        strcat(BoxOffsetLabel, three);
        strcat(BoxOffsetLabel, arrowrh);
        SendText(SticksViewButton3, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, arrowlh);
        strcat(BoxOffsetLabel, four);
        strcat(BoxOffsetLabel, ChannelNames[3]);
        SendText(SticksViewButton4, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, ChannelNames[4]);
        strcat(BoxOffsetLabel, five);
        strcat(BoxOffsetLabel, arrowrh);
        SendText(SticksViewButton5, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, arrowlh);
        strcat(BoxOffsetLabel, six);
        strcat(BoxOffsetLabel, ChannelNames[5]);
        SendText(SticksViewButton6, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, ChannelNames[6]);
        strcat(BoxOffsetLabel, seven);
        strcat(BoxOffsetLabel, arrowrh);
        SendText(SticksViewButton7, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, arrowlh);
        strcat(BoxOffsetLabel, eight);
        strcat(BoxOffsetLabel, ChannelNames[7]);
        SendText(SticksViewButton8, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, ChannelNames[8]);
        strcat(BoxOffsetLabel, nine);
        strcat(BoxOffsetLabel, arrowrh);
        SendText(SticksViewButton9, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, arrowlh);
        strcat(BoxOffsetLabel, ten);
        strcat(BoxOffsetLabel, ChannelNames[9]);
        SendText(SticksViewButton10, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, ChannelNames[10]);
        strcat(BoxOffsetLabel, eleven);
        strcat(BoxOffsetLabel, arrowrh);
        SendText(SticksViewButton11, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, arrowlh);
        strcat(BoxOffsetLabel, twelve);
        strcat(BoxOffsetLabel, ChannelNames[11]);
        SendText(SticksViewButton12, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, ChannelNames[12]);
        strcat(BoxOffsetLabel, thirteen);
        strcat(BoxOffsetLabel, arrowrh);
        SendText(SticksViewButton13, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, arrowlh);
        strcat(BoxOffsetLabel, fourteen);
        strcat(BoxOffsetLabel, ChannelNames[13]);
        SendText(SticksViewButton14, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, ChannelNames[14]);
        strcat(BoxOffsetLabel, fifteen);
        strcat(BoxOffsetLabel, arrowrh);
        SendText(SticksViewButton15, BoxOffsetLabel);
        strcpy(BoxOffsetLabel, arrowlh);
        strcat(BoxOffsetLabel, sixteen);
        strcat(BoxOffsetLabel, ChannelNames[15]);
        SendText(SticksViewButton16, BoxOffsetLabel);
    }
    if (CurrentView == Inputs_View || CurrentView == FailSafe_View) {
        SendText(fsch1, ChannelNames[0]);
        SendText(fsch2, ChannelNames[1]);
        SendText(fsch3, ChannelNames[2]);
        SendText(fsch4, ChannelNames[3]);
        SendText(fsch5, ChannelNames[4]);
        SendText(fsch6, ChannelNames[5]);
        SendText(fsch7, ChannelNames[6]);
        SendText(fsch8, ChannelNames[7]);
        SendText(fsch9, ChannelNames[8]);
        SendText(fsch10, ChannelNames[9]);
        SendText(fsch11, ChannelNames[10]);
        SendText(fsch12, ChannelNames[11]);
        SendText(fsch13, ChannelNames[12]);
        SendText(fsch14, ChannelNames[13]);
        SendText(fsch15, ChannelNames[14]);
        SendText(fsch16, ChannelNames[15]);
        SendValue(InPutStick_c1, InPutStick[0] + 1);
        SendValue(InPutStick_c2, InPutStick[1] + 1);
        SendValue(InPutStick_c3, InPutStick[2] + 1);
        SendValue(InPutStick_c4, InPutStick[3] + 1);
        SendValue(InPutStick_c5, InPutStick[4] + 1);
        SendValue(InPutStick_c6, InPutStick[5] + 1);
        SendValue(InPutStick_c7, InPutStick[6] + 1);
        SendValue(InPutStick_c8, InPutStick[7] + 1);
        SendValue(InPutStick_c9, InPutStick[8] + 1);
        SendValue(InPutStick_c10, InPutStick[9] + 1);
        SendValue(InPutStick_c11, InPutStick[10] + 1);
        SendValue(InPutStick_c12, InPutStick[11] + 1);
        SendValue(InPutStick_c13, InPutStick[12] + 1);
        SendValue(InPutStick_c14, InPutStick[13] + 1);
        SendValue(InPutStick_c15, InPutStick[14] + 1);
        SendValue(InPutStick_c16, InPutStick[15] + 1);
    }
}

/*********************************************************************************************************************************/

bool ReadOneModel(uint8_t Mnum)
{
    unsigned int j;
    ModelDetected = true;
    if (!ModelsFileOpen) OpenModelsFile();
    SDCardAddress = TXSIZE;                    //  spare bytes for TX stuff
    SDCardAddress += ((Mnum - 1) * MODELSIZE); //  spare bytes for Model params
    StartLocation = SDCardAddress;
    ModelDefined  = SDReadByte(SDCardAddress); // this variable is redundant now   could be re-used
    ++SDCardAddress;
    for (j = 0; j < 30; ++j) {
        ModelName[j] = SDReadByte(SDCardAddress);
        ++SDCardAddress;
    }

    for (i = 0; i < CHANNELSUSED; ++i) {
        for (j = 1; j <= 4; ++j) {
            MaxDegrees[j][i] = SDReadByte(SDCardAddress);
            ++SDCardAddress;
            MidHiDegrees[j][i] = SDReadByte(SDCardAddress);
            ++SDCardAddress;
            CentreDegrees[j][i] = SDReadByte(SDCardAddress);
            ++SDCardAddress;
            MidLowDegrees[j][i] = SDReadByte(SDCardAddress);
            ++SDCardAddress;
            MinDegrees[j][i] = SDReadByte(SDCardAddress);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < MAXMIXES; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            Mixes[j][i] = SDReadByte(SDCardAddress); // Read mixes
            ++SDCardAddress;
        }
    }

    for (j = 0; j < FlightModesUsed + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            Trims[j][i] = SDReadByte(SDCardAddress);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < FlightModesUsed + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            TrimsReversed[j][i] = SDReadByte(SDCardAddress);
            ++SDCardAddress;
        }
    }
    RXCellCount = SDReadByte(SDCardAddress);
    ++SDCardAddress;
    SDCardAddress += 30; // 30 Spare Bytes here (PID stuff gone)
    ++SDCardAddress;     // another Spare byte
    for (i = 0; i < CHANNELSUSED; ++i) {
        InPutStick[i] = SDReadByte(SDCardAddress);
        if (InPutStick[i] > 16) InPutStick[i] = i; // reset if nothing was saved!
        ++SDCardAddress;
    }

    // **************************

    FMSwitch = SDReadByte(SDCardAddress);
    ++SDCardAddress;
    AutoSwitch = SDReadByte(SDCardAddress);
    ++SDCardAddress;
    Channel9Switch = SDReadByte(SDCardAddress);
    ++SDCardAddress;
    Channel10Switch = SDReadByte(SDCardAddress);
    ++SDCardAddress;
    Channel11Switch = SDReadByte(SDCardAddress);
    ++SDCardAddress;
    Channel12Switch = SDReadByte(SDCardAddress);
    ++SDCardAddress;
    Switch1Reversed = bool(SDReadByte(SDCardAddress));
    ++SDCardAddress;
    Switch2Reversed = bool(SDReadByte(SDCardAddress));
    ++SDCardAddress;
    Switch3Reversed = bool(SDReadByte(SDCardAddress));
    ++SDCardAddress;
    Switch4Reversed = bool(SDReadByte(SDCardAddress));
    ++SDCardAddress;
    for (i = 0; i < CHANNELSUSED; ++i) {
        FailSafeChannel[i] = bool(SDReadByte(SDCardAddress));
        if (int(FailSafeChannel[i]) > 1) FailSafeChannel[i] = 0;
        ++SDCardAddress;
    }
    for (i = 0; i < CHANNELSUSED; ++i) {
        for (j = 0; j < 10; ++j) {
            ChannelNames[i][j] = SDReadByte(SDCardAddress);
            ++SDCardAddress;
        }
    }

    for (j = 0; j < FlightModesUsed + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            Exponential[j][i] = SDReadByte(SDCardAddress);
            if (Exponential[j][i] > 200 || Exponential[j][i] < 0) {
                Exponential[j][i] = 20;
            }
            ++SDCardAddress;
        }
    }
    for (j = 0; j < FlightModesUsed + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            InterpolationTypes[j][i] = SDReadByte(SDCardAddress);
            if (InterpolationTypes[j][i] < 0 || InterpolationTypes[j][i] > 2) {
                InterpolationTypes[j][i] = 2;
            }
            ++SDCardAddress;
        }
    }

    // **************************************

    OneModelMemory = SDCardAddress - StartLocation;

#ifdef DB_NEXTION
    Serial.print(MemoryForTransmtter);
    Serial.println(" bytes were  used for TX data.");
    Serial.print(TXSIZE);
    Serial.println(" bytes had been reserved.");
    Serial.print("So ");
    Serial.print(TXSIZE - MemoryForTransmtter);
    Serial.println(" spare bytes still remain for TX params.");
    Serial.println(" ");
    Serial.print("Loaded model number: ");
    Serial.println(ModelNumber);
    Serial.print("Model name: ");
    Serial.println(ModelName);
    Serial.println(" ");
    Serial.print(OneModelMemory);
    Serial.println(" bytes used per model.");
    Serial.print(MODELSIZE - OneModelMemory);
    Serial.println(" spare bytes per model.");
    Serial.print(MODELSIZE);
    Serial.println(" bytes reserved per model.)");
#endif // defined DB_NEXTION
    UpdateButtonLabels();
    return true;
}

/*********************************************************************************************************************************/

bool LoadAllParameters()
{
    int p;
    int j = 0;
    if (!ModelsFileOpen) OpenModelsFile();
    SDCardAddress = 0;
    p    = SDReadInt(SDCardAddress);
    if (p == RENEWDATA) {
        SDCardAddress += 2;
        for (i = 0; i < CHANNELSUSED; ++i) {
            ChannelMin[i] = SDReadInt(SDCardAddress);
            SDCardAddress += 2;
            ChannelMidLow[i] = SDReadInt(SDCardAddress);
            SDCardAddress += 2;
            ChannelCentre[i] = SDReadInt(SDCardAddress);
            SDCardAddress += 2;
            ChannelMidHi[i] = SDReadInt(SDCardAddress);
            SDCardAddress += 2;
            ChannelMax[i] = SDReadInt(SDCardAddress);
            SDCardAddress += 2;
        }
        DoSbusSendOnly = SDReadByte(SDCardAddress);
        ++SDCardAddress;
        BuddyMaster = SDReadByte(SDCardAddress);
        ++SDCardAddress;
        ModelNumber = SDReadByte(SDCardAddress);
        ModelNumberOffset = SDCardAddress;         // remember offset!
        ++SDCardAddress;
        ScreenTimeout = SDReadInt(SDCardAddress);
        ++SDCardAddress;
        ++SDCardAddress;
        Inactivity_Timeout = SDReadByte(SDCardAddress) * TICKSPERMINUTE;

        ++SDCardAddress;
        for (j = 0; j < 30; ++j) {
            TxName[j] = SDReadByte(SDCardAddress);
            ++SDCardAddress;
        }
        Qnh = SDReadInt(SDCardAddress);
        ++SDCardAddress;
        ++SDCardAddress;
        DeltaGMT = SDReadInt(SDCardAddress);
        ++SDCardAddress;
        ++SDCardAddress;
        MemoryForTransmtter = SDCardAddress;
        ReadOneModel(ModelNumber);
        return true;
    }
    else {
        return false;
    }
}

/*********************************************************************************************************************************/

void Force_ReDisplay()
{
    for (int i = 0; i < CHANNELSUSED; ++i) ShownBuffer[i] = 242; // to force a re-show of servo positions
}

/*********************************************************************************************************************************/

void ButtonRed(char* but)
{
    char lbut[60];
    char red[] = ".pco=RED";
    strcpy(lbut, but);
    strcat(lbut, red);
    SendCommand(lbut);
}

/*********************************************************************************************************************************/

void ButtonWhite(char* but)
{
    char lbut[60];
    char wight[] = ".pco=WHITE";
    strcpy(lbut, but);
    strcat(lbut, wight);
    SendCommand(lbut);
}

/*********************************************************************************************************************************/

#ifdef USE_WATCHDOG
void WatchDogCallBack()
{
    // Serial.println("RESETTING ...");
}
#endif

/*********************************************************************************************************************************/

void SetDS1307ToCompilerTime()
{
    if (getDate(__DATE__) && getTime(__TIME__)) {
        RTC.write(tm);
    } // only useful when connected to compiler
}

/************************************************************************************************************/

void GetTXVersionNumber()
{
    char    nbuf[5];
    uint8_t Txv1 = TXVERSION_MAJOR;
    uint8_t Txv2 = TXVERSION_MINOR;
    uint8_t Txv3 = TXVERSION_MINIMUS;
    Str(TransmitterVersionNumber, Txv1, 2);
    Str(nbuf, Txv2, 2);
    strcat(TransmitterVersionNumber, nbuf);
    Str(nbuf, Txv3, 0);
    strcat(TransmitterVersionNumber, nbuf);
}
/************************************************************************************************************/
void SetUKFrequencies(){
            FHSSChPointer = FHSS_Channels; 
            UkRules = true;     
}
/************************************************************************************************************/
void SetTestFrequencies(){
            FHSSChPointer = FHSS_Channels1; 
            UkRules = false;     
}
/*********************************************************************************************************************************/
// SETUP
/*********************************************************************************************************************************/
void setup()
{
    char FrontView_Connected[] = "FrontView.Connected";
    char Initialising[]        = "Initialising ... ";
    char OptionsViewTXname[] = "OptionsView.TxName";
    Nextion.begin(921600);   // BAUD rate also set in display code THIS IS THE MAX (was 115200)
    //115200
    //230400
    //250000
    //256000
    //512000
    //921600
    teensyMAC(MacAddress); // Get MAC address and use it as pipe address
    NewPipe = (uint64_t)MacAddress[0] << 40;
    NewPipe += (uint64_t)MacAddress[1] << 32;
    NewPipe += (uint64_t)MacAddress[2] << 24;
    NewPipe += (uint64_t)MacAddress[3] << 16;
    NewPipe += (uint64_t)MacAddress[4] << 8;
    NewPipe += (uint64_t)MacAddress[5];

    pinMode(REDLED, OUTPUT);
    pinMode(GREENLED, OUTPUT);
    pinMode(BLUELED, OUTPUT);
    pinMode(POWER_OFF_PIN, OUTPUT);
    BlueLedOn();
    Serial.begin(115200);
    Wire.begin();
    ScanI2c();
    if (USE_INA219) ina219.begin();
    SD.begin(chipSelect);
    InitSwitches();
    SendCommand(NextionWakeUp);
    InitMaxMin();        // in case not yet calibrated
    InitCentreDegrees(); // In case not yet calibrated
    CentreTrims();
    CalibratedYet = LoadAllParameters(); // If they exist, read saved SD card settings.
    InitRadio(DefaultPipe);
    SendCommand(page_FrontView); // Let's start at the beginning. Why not?
    SendText(FrontView_Connected, Initialising);
    SendValue1(NextionSleepTime, ScreenTimeout); // Setup Screen timeout (No .val needed)
    SendCommand(NextionWakeOnTouch);             // Wake on touch
    SendValue(FrontView_Hours, 0);
    SendValue(FrontView_Mins, 0);
    SendValue(FrontView_Secs, 0);

    //  ***************************************************************************************
     // SetDS1307ToCompilerTime();    //  **   Uncomment this line to set DS1307 clock to compiler's (Computer's) time.        **
    //  **   BUT then re-comment it!! Otherwise it will reset to same time on every boot up! **
    //  ***************************************************************************************
    RecoveryTimer = millis();
    BoundFlag     = false;
    TxOnTime      = millis();
    UpdateModelsNameEveryWhere();
    SendText(OptionsViewTXname, TxName);
#ifdef USE_WATCHDOG
    WatchDogConfig.window   = WATCHDOGMAXRATE; //  = MINIMUM RATE in milli seconds, (32ms to 522.232s) must be MUCH smaller than timeout
    WatchDogConfig.timeout  = WATCHDOGTIMEOUT; //  = MAX TIMEOUT in milli seconds, (32ms to 522.232s)
    WatchDogConfig.callback = WatchDogCallBack;
    TeensyWatchDog.begin(WatchDogConfig);
    LastDogKick = millis(); // needed? - yes!
#endif
    StartInactvityTimeout();
    SizeOfCompressedData = sizeof(CompressedData);
    GetTXVersionNumber();
    MySbus.begin();
    SetUKFrequencies(); 
}
/*********************************************************************************************************************************/

void GetStatistics()
{
    PacketsPerSecond     = RangeTestGoodPackets;
    RangeTestGoodPackets = 0;
}

/*********************************************************************************************************************************/

/** @returns position of text1 within text2 or 0 if not found */
int InStrng(char text1[CharsMax], char text2[CharsMax])
{
    unsigned int j;
    unsigned int flag;
    for (j = 0; j < strlen(text2); ++j) {
        flag = 0;
        for (i = 0; i < strlen(text1); ++i) {
            if (text1[i] != text2[i + j]) flag = 1;
        }
        if (flag == 0) return j + 1;
    }
    return 0;
}

/*********************************************************************************************************************************/

void SaveTXStuff()
{
    int  rd  = 0;
    bool EON = false;
    int  j   = 0;

    if (!ModelsFileOpen) OpenModelsFile();
    rd            = RENEWDATA;
    SDCardAddress          = 0;
    CalibratedYet = true;
    SDUpdateInt(SDCardAddress, rd); // xxxx in first two bytes = calibration done! *** CHANGE THIS NUMBER IF FORMAT IS NEW!! ****
    SDCardAddress += 2;
    for (i = 0; i < CHANNELSUSED; ++i) {
        SDUpdateInt(SDCardAddress, ChannelMin[i]); // Stick min output of pot
        SDCardAddress += 2;
        SDUpdateInt(SDCardAddress, ChannelMidLow[i]); //
        SDCardAddress += 2;
        SDUpdateInt(SDCardAddress, ChannelCentre[i]); // Stick Centre output of pot
        SDCardAddress += 2;
        SDUpdateInt(SDCardAddress, ChannelMidHi[i]); //
        SDCardAddress += 2;
        SDUpdateInt(SDCardAddress, ChannelMax[i]); // Stick max output of pot
        SDCardAddress += 2;
    }
    SDUpdateByte(SDCardAddress, DoSbusSendOnly);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, BuddyMaster);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, ModelNumber);
    ModelNumberOffset = SDCardAddress;                // remember offset!
    ++SDCardAddress;
    SDUpdateInt(SDCardAddress, ScreenTimeout);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, (Inactivity_Timeout / TICKSPERMINUTE));
    ++SDCardAddress;
    for (j = 0; j < 30; ++j) {
        if (EON) TxName[j] = 0;
        SDUpdateByte(SDCardAddress, TxName[j]);
        if (TxName[j] == 0) EON = true;
        ++SDCardAddress;
    }
    SDUpdateInt(SDCardAddress,Qnh);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdateInt(SDCardAddress,DeltaGMT);
    ++SDCardAddress;
    ++SDCardAddress;
    CloseModelsFile();
}

/*********************************************************************************************************************************/

/** MODEL Specific */
void SaveOneModel(int mnum)
{
    bool EndOfName = false;

    if (!ModelsFileOpen) OpenModelsFile();
    unsigned int j;
    SDCardAddress = TXSIZE;                  //  spare bytes for TX stuff
    SDCardAddress += (mnum - 1) * MODELSIZE; //  spare bytes for Model params
    StartLocation = SDCardAddress;
    ModelDefined  = 42;
    SDUpdateByte(SDCardAddress, ModelDefined);
    ++SDCardAddress;
    for (j = 0; j < 30; ++j) {
        if (EndOfName) ModelName[j] = 0;
        SDUpdateByte(SDCardAddress, ModelName[j]);
        if (ModelName[j] == 0) EndOfName = true;
        ++SDCardAddress;
    }
    for (i = 0; i < CHANNELSUSED; ++i) {
        for (j = 1; j <= 4; ++j) {
            SDUpdateByte(SDCardAddress, MaxDegrees[j][i]); // Max requested in degrees (180)
            ++SDCardAddress;
            SDUpdateByte(SDCardAddress, MidHiDegrees[j][i]); // MidHi requested in degrees (135)
            ++SDCardAddress;
            SDUpdateByte(SDCardAddress, CentreDegrees[j][i]); // Centre requested in degrees (90)
            ++SDCardAddress;
            SDUpdateByte(SDCardAddress, MidLowDegrees[j][i]); // MidLo requested in degrees (45)
            ++SDCardAddress;
            SDUpdateByte(SDCardAddress, MinDegrees[j][i]); // Min requested in degrees (0)
            ++SDCardAddress;
        }
    }
    for (j = 0; j < MAXMIXES; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            SDUpdateByte(SDCardAddress, Mixes[j][i]); // Save mixes
            ++SDCardAddress;
        }
    }
    for (j = 0; j < FlightModesUsed + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            SDUpdateByte(SDCardAddress, Trims[j][i]);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < FlightModesUsed + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            SDUpdateByte(SDCardAddress, TrimsReversed[j][i]);
            ++SDCardAddress;
        }
    }
    SDUpdateByte(SDCardAddress, RXCellCount);
    ++SDCardAddress;

    SDCardAddress += 30; // 30 Spare Bytes here (PID stuff gone)

    ++SDCardAddress; // another Spare

    for (i = 0; i < CHANNELSUSED; ++i) {
        SDUpdateByte(SDCardAddress, InPutStick[i]);
        ++SDCardAddress;
    }

    SDUpdateByte(SDCardAddress, FMSwitch);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, AutoSwitch);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, Channel9Switch);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, Channel10Switch);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, Channel11Switch);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, Channel12Switch);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, Switch1Reversed);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, Switch2Reversed);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, Switch3Reversed);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, Switch4Reversed);
    ++SDCardAddress;
    for (i = 0; i < CHANNELSUSED; ++i) {
        SDUpdateByte(SDCardAddress, FailSafeChannel[i]);
        ++SDCardAddress;
    }
    for (i = 0; i < CHANNELSUSED; ++i) {
        for (j = 0; j < 10; ++j) {
            SDUpdateByte(SDCardAddress, ChannelNames[i][j]);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < FlightModesUsed + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            SDUpdateByte(SDCardAddress, Exponential[j][i]);
            ++SDCardAddress;
        }
    }

    for (j = 0; j < FlightModesUsed + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            SDUpdateByte(SDCardAddress, InterpolationTypes[j][i]);
            ++SDCardAddress;
        }
    }
    // **********************

    OneModelMemory = SDCardAddress - StartLocation;
#ifdef DB_NEXTION
    Serial.print("Saved model: ");
    Serial.println(ModelName);
    Serial.println(" ");
    Serial.print(OneModelMemory);
    Serial.println(" bytes used per model.");
    Serial.print(MODELSIZE - OneModelMemory);
    Serial.println(" spare bytes per model.");
    Serial.print(MODELSIZE);
    Serial.println(" bytes reserved per model.)");
    Serial.println(" ");
#endif // defined DB_NEXTION
    CloseModelsFile();
}
/*********************************************************************************************************************************/
void ReadHelpFile(char* fname, char* htext)
{
    char errormsg[] = "Help file was not found.";
    File fnumber;
    char a[] = " ";

    htext[0] = 0;
    i        = 0;
    fnumber  = SD.open(fname, FILE_READ);
    if (fnumber) {
        while (fnumber.available() && i < MAXFILELEN) {
            a[0] = fnumber.read();
            if (a[0] != 34) {
                strcat(htext, a);
                ++i;
            }
        }
    }
    else {
        strcpy(htext, errormsg);
    }
    fnumber.close();
}

/*********************************************************************************************************************************/
void SendHelp()
{
    char HelpView[] = "HelpView.HelpText";
    char HelpFile[20];
    char HelpText[MAXFILELEN + 10]; // MAX = 1200
    i = 9;
    int j = 0;
    while (WordsIn[i] != 0 && j < 19) {
        HelpFile[j] = WordsIn[i];
        ++i;
        ++j;
        HelpFile[j] = 0;
    }
    ReadHelpFile(HelpFile, HelpText);
    SendText1(HelpView, HelpText);
}
/*********************************************************************************************************************************/
/** @brief Discover which channel to setup */
int GetChannel()
{
    for (i = 0; i < sizeof(WordsIn); ++i) {
        if (isdigit(WordsIn[i])) break;
    }
    return atoi(&WordsIn[i]);
}

/*********************************************************************************************************************************/

void UpdateSwitchesDisplay()
{
    char SwitchesView_sw1[] = "sw1";
    char SwitchesView_sw2[] = "sw2";
    char SwitchesView_sw3[] = "sw3";
    char SwitchesView_sw4[] = "sw4";
    char NotUsed[]          = "Not used";
    char FlightModes123[]   = "Flight modes 1 2 3";
    char Auto[]             = "Auto (FM 4)";
    char Channel_9[]        = "Channel 9";
    char Channel_10[]       = "Channel 10";
    char Channel_11[]       = "Channel 11";
    char Channel_12[]       = "Channel 12";

    SendText(SwitchesView_sw1, NotUsed);
    if (AutoSwitch == 1) SendText(SwitchesView_sw1, Auto);
    if (FMSwitch == 1) SendText(SwitchesView_sw1, FlightModes123);
    if (Channel9Switch == 1) SendText(SwitchesView_sw1, Channel_9);
    if (Channel10Switch == 1) SendText(SwitchesView_sw1, Channel_10);
    if (Channel11Switch == 1) SendText(SwitchesView_sw1, Channel_11);
    if (Channel12Switch == 1) SendText(SwitchesView_sw1, Channel_12);

    SendText(SwitchesView_sw2, NotUsed);
    if (AutoSwitch == 2) SendText(SwitchesView_sw2, Auto);
    if (FMSwitch == 2) SendText(SwitchesView_sw2, FlightModes123);
    if (Channel9Switch == 2) SendText(SwitchesView_sw2, Channel_9);
    if (Channel10Switch == 2) SendText(SwitchesView_sw2, Channel_10);
    if (Channel11Switch == 2) SendText(SwitchesView_sw2, Channel_11);
    if (Channel12Switch == 2) SendText(SwitchesView_sw2, Channel_12);

    SendText(SwitchesView_sw3, NotUsed);
    if (AutoSwitch == 3) SendText(SwitchesView_sw3, Auto);
    if (FMSwitch == 3) SendText(SwitchesView_sw3, FlightModes123);
    if (Channel9Switch == 3) SendText(SwitchesView_sw3, Channel_9);
    if (Channel10Switch == 3) SendText(SwitchesView_sw3, Channel_10);
    if (Channel11Switch == 3) SendText(SwitchesView_sw3, Channel_11);
    if (Channel12Switch == 3) SendText(SwitchesView_sw3, Channel_12);

    SendText(SwitchesView_sw4, NotUsed);
    if (AutoSwitch == 4) SendText(SwitchesView_sw4, Auto);
    if (FMSwitch == 4) SendText(SwitchesView_sw4, FlightModes123);
    if (Channel9Switch == 4) SendText(SwitchesView_sw4, Channel_9);
    if (Channel10Switch == 4) SendText(SwitchesView_sw4, Channel_10);
    if (Channel11Switch == 4) SendText(SwitchesView_sw4, Channel_11);
    if (Channel12Switch == 4) SendText(SwitchesView_sw4, Channel_12);
}

/*********************************************************************************************************************************/
uint8_t CheckRange_0_16(uint8_t v)
{
    if (v > 16) v = 16;
    if (v < 0) v = 0;
    return v;
}
/*********************************************************************************************************************************/

void DoNewChannelName(int ch, int k)
{
    int j                   = 0;
    ChannelNames[ch - 1][0] = 32;
    ChannelNames[ch - 1][1] = 0; // remove old name
    while (uint8_t(WordsIn[k]) > 0) {
        ChannelNames[ch - 1][j] = WordsIn[k];
        ++j;
        k++;
        ChannelNames[ch - 1][j] = 0;
    }
    SaveOneModel(ModelNumber);
}

/*********************************************************************************************************************************/

/** Bubble sort */
void SortDirectory()
{
    int  f      = 0;
    bool flag   = true;
    int  Scount = 0;
    char BoxOffset[18];
    while (flag && Scount < 10000) {
        flag = false;
        for (f = 0; f < ExportedFileCounter - 1; f++) {
            if (strcmp(TheFilesList[f], TheFilesList[f + 1]) > 0) {
                strcpy(BoxOffset, TheFilesList[f]);
                strcpy(TheFilesList[f], TheFilesList[f + 1]);
                strcpy(TheFilesList[f + 1], BoxOffset);
                flag = true;
                Scount++;
            }
        }
    }
}

/*********************************************************************************************************************************/

void BuildDirectory()
{
    char MOD[] = ".MOD";
    char Entry1[20];
    char fn[18];
    File dir;
    dir                 = SD.open("/");
    ExportedFileCounter = 0;
    while (true) {
        File entry = dir.openNextFile();
        if (!entry || ExportedFileCounter > 18) break;
        strcpy(Entry1, entry.name());
        if (InStrng(MOD, Entry1) > 0) {
            strcpy(fn, entry.name());
            for (i = 0; i < 12; ++i) {
                TheFilesList[ExportedFileCounter][i] = fn[i];
            }
            ExportedFileCounter++;
        }
        entry.close();
    }
    SortDirectory();
}

/*********************************************************************************************************************************/

/** @brief updates display in textbox */
void ShowFileNumber()
{
    char ModelsView_filename[] = "filename";
    char newfname[17];
    if (FileNumberInView >= ExportedFileCounter) FileNumberInView = 0;
    if (FileNumberInView < 0) FileNumberInView = ExportedFileCounter - 1;
    for (i = 0; i < 12; ++i) {
        newfname[i]     = TheFilesList[FileNumberInView][i];
        newfname[i + 1] = 0;
        if (newfname[i] <= 32 || newfname[i] > 127) break;
    }
    SendText(ModelsView_filename, newfname);
}

/*********************************************************************************************************************************/

void ShowFileErrorMsg()
{
    char ErrorOn[]  = "vis error,1";
    char ErrorOff[] = "vis error,0";
    for (int pp = 0; pp < 3; pp++) {
        SendCommand(ErrorOn);
        delay(200);
        SendCommand(ErrorOff);
        delay(100);
    }
    FileError = false;
}

/*********************************************************************************************************************************/

void SaveAllParameters()
{
    if (!ModelsFileOpen) OpenModelsFile();
    SaveTXStuff();
    MemoryForTransmtter = SDCardAddress - 2;
    SaveOneModel(ModelNumber);
#ifdef DB_NEXTION
    Serial.println(" ");
    Serial.print(MemoryForTransmtter);
    Serial.println(" bytes written to SD CARD FOR TX.");
    Serial.print(TXSIZE);
    Serial.println(" bytes reserved for TX.");
    Serial.print(TXSIZE - MemoryForTransmtter);
    Serial.println(" Spare bytes still for any new TX params.");
    Serial.print("Saved model: ");
    Serial.print(ModelNumber);
    Serial.print(" (");
    Serial.print(ModelName);
    Serial.println(")");
#endif // defined DB_NEXTION
}

/*********************************************************************************************************************************/

void ShowDirectory()
{
    char    filelistbuf[1024];
    char    nul[]          = "";
    char    crlf[]         = {13, 10, 0};
    char    space[]        = {' ', 0};
    char    fileviewlist[] = "FilesView.list";
    char    t[2]           = "P";
    uint8_t n              = 0;
    uint8_t nlp            = 0;
    strcpy(filelistbuf, nul);
    for (i = 0; i < ExportedFileCounter; ++i) {
        nlp = 13;
        for (int z = 0; z < 12; z++) {
            t[0] = TheFilesList[i][z];
            if (t[0] == 0) break;
            nlp--;
            strcat(filelistbuf, t);
        }
        n++;
        if (n > 2) {
            strcat(filelistbuf, crlf);
            n = 0;
        }
        else
        {
            for (int q = 0; q < nlp; q++) {
                strcat(filelistbuf, space);
            }
        }
    }
    SendText1(fileviewlist, filelistbuf);
}

/*********************************************************************************************************************************/

void SetDefaultValues()
{
    int  j;
    char ProgressStart[]                       = "vis Progress,1";
    char ProgressEnd[]                         = "vis Progress,0";
    char Progress[]                            = "Progress";
    char defaultName[]                         = "(Deleted)";
    char DefaultChannelNames[CHANNELSUSED][11] = {{"Aileron"}, {"Elevator"}, {"Throttle"}, {"Rudder"}, {"Ch 5"}, {"Ch 6"}, {"Ch 7"}, {"Ch 8"}, {"Ch 9"}, {"Ch 10"}, {"Ch 11"}, {"Ch 12"}, {"Ch 13"}, {"Ch 14"}, {"Ch 15"}, {"Ch 16"}};
    SendCommand(ProgressStart);
    SendValue(Progress, 5);
    delay(10);
    strcpy(ModelName, defaultName);

    for (i = 0; i < CHANNELSUSED; ++i) {
        for (j = 1; j <= 4; ++j) {
            if (i == 1) {
                MaxDegrees[j][i]    = 30; // Elevator goes the other way by default
                MidHiDegrees[j][i]  = 60;
                CentreDegrees[j][i] = 90;
                MidLowDegrees[j][i] = 120;
                MinDegrees[j][i]    = 150;
            }
            else {
                MaxDegrees[j][i]    = 150;
                MidHiDegrees[j][i]  = 120;
                CentreDegrees[j][i] = 90;
                MidLowDegrees[j][i] = 60;
                MinDegrees[j][i]    = 30;
            }
        }
    }
    SendValue(Progress, 15);
    delay(10);
    for (j = 0; j < MAXMIXES; ++j) {
        for (i = 0; i < CHANNELSUSED; ++i) {
            Mixes[j][i] = 0;
        }
    }
    SendValue(Progress, 25);
    delay(10);

    for (j = 0; j < FlightModesUsed + 1; ++j) { // must have fudged this somewhere.... 5?!
        for (i = 0; i < CHANNELSUSED; ++i) {
            Trims[j][i]         = 80; // MIDPOINT is 80 !
            TrimsReversed[j][i] = 0;
        }
    }
    RXCellCount = 3;

    SendValue(Progress, 45);
    delay(10);
    for (i = 0; i < CHANNELSUSED; ++i) {
        InPutStick[i] = i;
    }
    FMSwitch        = 4;
    AutoSwitch      = 1;
    Channel9Switch  = 2;
    Channel10Switch = 3;
    Channel11Switch = 0;
    Channel12Switch = 0;
    Switch1Reversed = false;
    Switch2Reversed = false;
    Switch3Reversed = false;
    Switch4Reversed = false;
    SendValue(Progress, 65);
    delay(10);
    for (i = 0; i < CHANNELSUSED; ++i) {
        FailSafeChannel[i] = false;
    }
    SendValue(Progress, 75);
    delay(10);
    for (i = 0; i < CHANNELSUSED; ++i) {
        for (j = 0; j < 10; ++j) {
            ChannelNames[i][j] = DefaultChannelNames[i][j];
            ++SDCardAddress;
        }
    }

    for (j = 0; j < FlightModesUsed + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            Exponential[j][i] = 0; // 20% expo = default
            ++SDCardAddress;
        }
    }
    for (j = 0; j < FlightModesUsed + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            InterpolationTypes[j][i] = 2; // Expo is default
            ++SDCardAddress;
        }
    }

    SendValue(Progress, 95);
    delay(10);
    SaveOneModel(ModelNumber);
    UpdateModelsNameEveryWhere();
    SendValue(Progress, 100);
    delay(100);
    SendCommand(ProgressEnd);
}

/*********************************************************************************************************************************/

void ClearBox()
{
    char fillcmd[] = "fill 30,30,380,365,214";
    SendCommand(fillcmd);
}

/*********************************************************************************************************************************/

/**
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param color
 */
void DrawLine(int x1, int y1, int x2, int y2, int c)
{
    char line[] = "line ";
    char nb[12];
    char cb[50];
    char comma[] = ",";
    strcpy(cb, line);
    strcat(cb, Str(nb, x1, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, y1, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, x2, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, y2, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, c, 0));
    SendCommand(cb);
}

/*********************************************************************************************************************************/

/**
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param color
 */
void DrawBox(int x1, int y1, int x2, int y2, int c)
{
    char line[] = "draw ";
    char nb[12];
    char cb[50];
    char comma[] = ",";
    strcpy(cb, line);
    strcat(cb, Str(nb, x1, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, y1, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, x2, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, y2, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, c, 0));
    SendCommand(cb);
}

/*********************************************************************************************************************************/

/** @brief Uses servo degrees to position dots */
void GetDotPositions()
{
    int p      = 0;
    BoxOffset  = 35;
    BoxLeft    = BoxOffset;
    BoxTop     = BoxOffset;
    BoxRight   = BoxLeft + 395;
    BoxBottom  = BoxRight;
    xPoints[0] = BoxLeft;
    xPoints[4] = BoxRight - BoxOffset;

    p          = map(MinDegrees[FlightMode][ChanneltoSet - 1], 0, 180, BoxSize, BoxOffset);
    yPoints[0] = constrain(p, 39, 391);
    p          = map(MidLowDegrees[FlightMode][ChanneltoSet - 1], 0, 180, BoxSize, BoxOffset);
    yPoints[1] = constrain(p, 39, 391);
    xPoints[1] = BoxOffset + 90;
    xPoints[2] = BoxOffset + 180;
    p          = map(CentreDegrees[FlightMode][ChanneltoSet - 1], 0, 180, BoxSize, BoxOffset);
    yPoints[2] = constrain(p, 39, 391);
    xPoints[3] = BoxOffset + 270;
    p          = map(MidHiDegrees[FlightMode][ChanneltoSet - 1], 0, 180, BoxSize, BoxOffset);
    yPoints[3] = constrain(p, 39, 391);
    p          = map(MaxDegrees[FlightMode][ChanneltoSet - 1], 0, 180, BoxSize, BoxOffset);
    yPoints[4] = constrain(p, 39, 391);
}

/*********************************************************************************************************************************/

int DegsToPercent(int degs)
{
    return map(degs, 0, 180, -100, 100);
}

/*********************************************************************************************************************************/

void DrawDot(int xx, int yy, int rad, int colr)
{
    char cirs[] = "cirs ";
    char nb[12];
    char cb[50];
    char comma[] = ",";
    strcpy(cb, cirs);
    strcat(cb, Str(nb, xx, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, yy, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, rad, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, colr, 0));
    SendCommand(cb);
}

/*********************************************************************************************************************************/

void updateInterpolationTypes()
{
    char ExpR[]     = "Exp";
    char Smooth[]   = "Smooth";
    char Lines[]    = "Lines";
    char Expon[]    = "Expo";       // number display
    char Ex1[]      = "Ex1";        // slider
    char t3on[]     = "vis t3,1";   // expo value lable
    char b13on[]    = "vis b13,1";  // expo -
    char b12on[]    = "vis b12,1";  // expo +
    char ExponOn[]  = "vis Expo,1"; // number display
    char Ex1On[]    = "vis Ex1,1";  // slider
    char t3off[]    = "vis t3,0";   // expo value lable
    char b13off[]   = "vis b13,0";  // expo -
    char b12off[]   = "vis b12,0";  // expo +
    char ExponOff[] = "vis Expo,0"; // number display
    char Ex1Off[]   = "vis Ex1,0";  // slider

    switch (InterpolationTypes[FlightMode][ChanneltoSet - 1]) {
        case 0:
            SendValue(Lines, 1);
            SendValue(Smooth, 0);
            SendValue(ExpR, 0);
            SendCommand(t3off);
            SendCommand(b13off);
            SendCommand(b12off);
            SendCommand(ExponOff);
            SendCommand(Ex1Off);
            break;
        case 1:
            SendValue(Lines, 0);
            SendValue(Smooth, 1);
            SendValue(ExpR, 0);
            SendCommand(t3off);
            SendCommand(b13off);
            SendCommand(b12off);
            SendCommand(ExponOff);
            SendCommand(Ex1Off);
            break;
        case 2:
            SendValue(Lines, 0);
            SendValue(Smooth, 0);
            SendValue(ExpR, 1);
            SendCommand(t3on);
            SendCommand(b13on);
            SendCommand(b12on);
            SendCommand(ExponOn);
            SendCommand(Ex1On);
            SendValue(Ex1, Exponential[FlightMode][ChanneltoSet - 1]);
            SendValue(Expon, Exponential[FlightMode][ChanneltoSet - 1]);
            break;
        default:
            break;
    }
}

/*********************************************************************************************************************************/

void DisplayCurve()
{
    int  p       = 0;
    char Gn1[]   = "n1";
    char Gn2[]   = "n2";
    char Gn3[]   = "n3";
    char Gn4[]   = "n4";
    char Gn5[]   = "n5";
    char b3on[]  = "vis b3,1";
    char b4on[]  = "vis b4,1";
    char b7on[]  = "vis b7,1";
    char b8on[]  = "vis b8,1";
    char n2on[]  = "vis n2,1";
    char n4on[]  = "vis n4,1";
    char b3off[] = "vis b3,0";
    char b4off[] = "vis b4,0";
    char b7off[] = "vis b7,0";
    char b8off[] = "vis b8,0";
    char n2off[] = "vis n2,0";
    char n4off[] = "vis n4,0";

    int   Step = 8;
    float HalfXRange;
    float TopHalfYRange;
    float BottomHalfYRange;
    int   xDot1;
    int   yDot1;
    int   xDot2     = 0;
    int   yDot2     = 0;
    int   DotSize   = 2;
    int   DotColour = Yellow;
    ClearBox();
    p                                           = constrain(MinDegrees[FlightMode][ChanneltoSet - 1], 0, 180);
    MinDegrees[FlightMode][ChanneltoSet - 1]    = p;
    p                                           = constrain(MidLowDegrees[FlightMode][ChanneltoSet - 1], 0, 180);
    MidLowDegrees[FlightMode][ChanneltoSet - 1] = p;
    p                                           = constrain(CentreDegrees[FlightMode][ChanneltoSet - 1], 0, 180);
    CentreDegrees[FlightMode][ChanneltoSet - 1] = p;
    p                                           = constrain(MidHiDegrees[FlightMode][ChanneltoSet - 1], 0, 180);
    MidHiDegrees[FlightMode][ChanneltoSet - 1]  = p;
    p                                           = constrain(MaxDegrees[FlightMode][ChanneltoSet - 1], 0, 180);
    MaxDegrees[FlightMode][ChanneltoSet - 1]    = p;
    GetDotPositions();
    SendValue(Gn1, DegsToPercent(MinDegrees[FlightMode][ChanneltoSet - 1])); // put numbers at top row
    SendValue(Gn2, DegsToPercent(MidLowDegrees[FlightMode][ChanneltoSet - 1]));
    SendValue(Gn3, DegsToPercent(CentreDegrees[FlightMode][ChanneltoSet - 1]));
    SendValue(Gn4, DegsToPercent(MidHiDegrees[FlightMode][ChanneltoSet - 1]));
    SendValue(Gn5, DegsToPercent(MaxDegrees[FlightMode][ChanneltoSet - 1]));
    DrawBox(BoxLeft, BoxTop, BoxRight - BoxLeft, BoxBottom - BoxTop, Green);

    xDot1 = xPoints[0];
    yDot1 = ((BoxBottom - BoxTop) / 2) + 20; // ?
    xDot2 = BoxRight - BoxOffset;
    yDot2 = yDot1;
    DrawLine(xDot1, yDot1, xDot2, yDot1, Red);

    xDot1 = xPoints[2];
    yDot1 = BoxTop;
    xDot2 = xDot1;
    yDot2 = BoxBottom - BoxOffset;
    DrawLine(xDot1, yDot1, xDot2, yDot2, Red);

    if (InterpolationTypes[FlightMode][ChanneltoSet - 1] == 0) {
        SendCommand(b3on);
        SendCommand(b4on);
        SendCommand(b7on);
        SendCommand(b8on);
        SendCommand(n2on);
        SendCommand(n4on);
        DrawLine(xPoints[0], yPoints[0], xPoints[1], yPoints[1], White); // this adds the straight version of the curve
        DrawLine(xPoints[1], yPoints[1], xPoints[2], yPoints[2], White);
        DrawLine(xPoints[2], yPoints[2], xPoints[3], yPoints[3], White);
        DrawLine(xPoints[3], yPoints[3], xPoints[4], yPoints[4], White);
    }
    delay(250);
    if (InterpolationTypes[FlightMode][ChanneltoSet - 1] == 1) {
        SendCommand(b3on);
        SendCommand(b4on);
        SendCommand(b7on);
        SendCommand(b8on);
        SendCommand(n2on);
        SendCommand(n4on);
        yDot2 = 0;
        for (xPoint = xPoints[0]; xPoint <= xPoints[4]; xPoint += Step) {
            if (Step > xPoints[4] - xPoint) {
                Step = xPoints[4] - xPoint;
            }
            if (Step < 1) Step = 1;
            yPoint = Interpolation::CatmullSpline(xPoints, yPoints, PointsCount, xPoint);
            xDot1  = xPoint;
            yDot1  = yPoint;
            if (yDot2 == 0) {
                xDot2 = xDot1;
                yDot2 = yDot1;
            }
            DrawLine(xDot1, yDot1, xDot2, yDot2, White);
            xDot2 = xDot1;
            yDot2 = yDot1;
        }
    }

    if (InterpolationTypes[FlightMode][ChanneltoSet - 1] == 2) { //EXPO  ************************************************************************************************
#define APPROXIMATION 8                                          // This is for the approximation of the screen curve

        SendCommand(b3off);
        SendCommand(b4off);
        SendCommand(b7off);
        SendCommand(b8off);
        SendCommand(n2off);
        SendCommand(n4off);
        HalfXRange       = xPoints[4] - xPoints[2];
        TopHalfYRange    = yPoints[4] - yPoints[2];
        BottomHalfYRange = yPoints[2] - yPoints[0];
        yDot2            = 0;
        Step             = APPROXIMATION;                        // This is the approximation of the screen curve
        for (xPoint = 0; xPoint <= HalfXRange; xPoint += Step) { // Simulate a curve with many short lines to speed it up
            yPoint = MapExp(HalfXRange - xPoint, HalfXRange, 0, 0, BottomHalfYRange, Exponential[FlightMode][ChanneltoSet - 1]);
            if (Step > HalfXRange - xPoint) {
                Step = HalfXRange - xPoint;
            }
            if (Step < 1) Step = 1;
            yDot1 = yPoint + yPoints[0];
            xDot1 = xPoint + xPoints[0];
            if (yDot2 == 0) {
                xDot2 = xDot1;
                yDot2 = yDot1;
            }
            DrawLine(xDot1, yDot1, xDot2, yDot2, White); // Draw short line from this point to previous point
            xDot2 = xDot1;
            yDot2 = yDot1;
        }
        Step  = APPROXIMATION;
        yDot2 = 0;
        for (xPoint = HalfXRange; xPoint >= 0; xPoint -= Step) { // Simulate a curve with many short lines to speed it up
            yPoint = MapExp(xPoint, 0, HalfXRange, 0, TopHalfYRange, Exponential[FlightMode][ChanneltoSet - 1]);
            if (Step > xPoint) {
                Step = xPoint;
            }
            if (Step < 1) Step = 1;
            yDot1 = yPoint + yPoints[2];
            xDot1 = xPoint + xPoints[2];
            if (yDot2 == 0) {
                xDot2 = xDot1;
                yDot2 = yDot1;
            }
            DrawLine(xDot1, yDot1, xDot2, yDot2, White); // Draw short line from this point to previous point
            xDot2 = xDot1;
            yDot2 = yDot1;
        }
        DrawDot(xPoints[0], yPoints[0], DotSize, DotColour); // This adds 3 dots
        DrawDot(xPoints[2], yPoints[2], DotSize, DotColour);
        DrawDot(xPoints[4], yPoints[4], DotSize, DotColour);
    }

    if (InterpolationTypes[FlightMode][ChanneltoSet - 1] != 2) {
        DrawDot(xPoints[0], yPoints[0], DotSize, DotColour); // This adds 5 dots
        DrawDot(xPoints[1], yPoints[1], DotSize, DotColour);
        DrawDot(xPoints[2], yPoints[2], DotSize, DotColour);
        DrawDot(xPoints[3], yPoints[3], DotSize, DotColour);
        DrawDot(xPoints[4], yPoints[4], DotSize, DotColour);
    }
    updateInterpolationTypes();
}

/*********************************************************************************************************************************/

void BindNow()
{

#ifdef DB_BIND
    Serial.println("Binding");
#endif
    BindingNow    = 1;
    ModelDetected = true; // just to stop auto model switching
}

/*********************************************************************************************************************************/

int GetDifference(int YtouchPlace, int oldy)
{
    int dd;
    dd = (YtouchPlace - oldy) / 2;
    if (dd < 0) dd = -dd;
    return dd;
}
/*********************************************************************************************************************************/

/** @brief moves point very close to where user hit screen */
void MovePoint()
{
    int rjump = 0;
    GetDotPositions();                               // current
    if (XtouchPlace > BoxRight - BoxOffset) return;  // out of range
    if (XtouchPlace < BoxOffset) return;             // out of range
    if (YtouchPlace < BoxTop) return;                // out of range
    if (YtouchPlace > BoxBottom - BoxOffset) return; // out of range

    if (XtouchPlace < BoxOffset + xPoints[0]) { // do leftmost point  ?
        rjump = GetDifference(YtouchPlace, yPoints[0]);
        if (YtouchPlace > yPoints[0]) {
            if (MinDegrees[FlightMode][ChanneltoSet - 1] >= rjump) MinDegrees[FlightMode][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (MinDegrees[FlightMode][ChanneltoSet - 1] <= (180 - rjump)) MinDegrees[FlightMode][ChanneltoSet - 1] += rjump;
        }
    }

    if (XtouchPlace > xPoints[1] - BoxOffset && XtouchPlace < xPoints[1] + BoxOffset) { // do next point  ?
        if (InterpolationTypes[FlightMode][ChanneltoSet - 1] == 2) return;              //  expo = ignore this area
        rjump = GetDifference(YtouchPlace, yPoints[1]);
        if (YtouchPlace > yPoints[1]) {
            if (MidLowDegrees[FlightMode][ChanneltoSet - 1] >= rjump) MidLowDegrees[FlightMode][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (MidLowDegrees[FlightMode][ChanneltoSet - 1] <= 180 - rjump) MidLowDegrees[FlightMode][ChanneltoSet - 1] += rjump;
        }
    }

    if (XtouchPlace > xPoints[2] - BoxOffset && XtouchPlace < xPoints[2] + BoxOffset) { // do next point  ?
        rjump = GetDifference(YtouchPlace, yPoints[2]);
        if (YtouchPlace > yPoints[2]) {
            if (CentreDegrees[FlightMode][ChanneltoSet - 1] >= rjump) CentreDegrees[FlightMode][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (CentreDegrees[FlightMode][ChanneltoSet - 1] <= 180 - rjump) CentreDegrees[FlightMode][ChanneltoSet - 1] += rjump;
        }
    }

    if (XtouchPlace > xPoints[3] - BoxOffset && XtouchPlace < xPoints[3] + BoxOffset) { // do next point  ?
        if (InterpolationTypes[FlightMode][ChanneltoSet - 1] == 2) return;              //  expo = ignore this area
        rjump = GetDifference(YtouchPlace, yPoints[3]);
        if (YtouchPlace > yPoints[3]) {
            if (MidHiDegrees[FlightMode][ChanneltoSet - 1] >= rjump) MidHiDegrees[FlightMode][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (MidHiDegrees[FlightMode][ChanneltoSet - 1] <= 180 - rjump) MidHiDegrees[FlightMode][ChanneltoSet - 1] += rjump;
        }
    }
    if (XtouchPlace > xPoints[3] + BoxOffset) // do hi point  ?
    {
        rjump = GetDifference(YtouchPlace, yPoints[4]);
        if (YtouchPlace > yPoints[4]) {
            if (MaxDegrees[FlightMode][ChanneltoSet - 1] > rjump) MaxDegrees[FlightMode][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (MaxDegrees[FlightMode][ChanneltoSet - 1] <= 180 - rjump) MaxDegrees[FlightMode][ChanneltoSet - 1] += rjump;
        }
    }
    DisplayCurve();
}

/*********************************************************************************************************************************/
// SEND AND RECEIVE A MODEL FILE
/*********************************************************************************************************************************/

#define FILEPIPEADDRESS 0xFEFEFEFEFELL // Unique pipe address for FILE EXCHANGE
#define BUFFERSIZE      28             // + 4 = 32
#define FILEDATARATE    RF24_250KBPS
#define FILEPALEVEL     RF24_PA_MAX
#define FILECHANNEL     123
#define FILETIMEOUT     30

/*********************************************************************************************************************************/

/** @brief RECEIVE A MODEL FILE */
void ReceiveModelFile()
{
    uint64_t RXPipe;
    uint32_t RXTimer = 0;

    char          ModelsView_filename[] = "filename";
    char          ProgressStart[]       = "vis Progress,1";
    char          ProgressEnd[]         = "vis Progress,0";
    char          Progress[]            = "Progress";
    char          Fbuffer[BUFFERSIZE + 8]; // spare space
    uint8_t       Fack      = 1;           // just a token byte
    char          Waiting[] = "Waiting ";
    char          WaitTime[6];
    char          WaitMsg[17];
    char          ThreeDots[]    = "...";
    char          Receiving[]    = "Receiving ...";
    char          TimeoutMsg[]   = "TIMEOUT";
    char          Success[]      = "* Success! *";
    unsigned long Fsize          = 0;
    unsigned long Fposition      = 0;
    float         SecondsElapsed = 0;
    uint8_t       p              = 5;
#ifdef DB_MODEL_EXCHANGE
    uint8_t PacketNumber = 0;
    Serial.println("Receiving model ...");
    Serial.println(Waiting);
#endif
    SendText(ModelsView_filename, Waiting);
    RXPipe = FILEPIPEADDRESS;
    Radio1.setRetries(15, 15);
    Radio1.setChannel(FILECHANNEL);
    Radio1.flush_tx();
    Radio1.openReadingPipe(1, RXPipe);
    Radio1.startListening();
    RXTimer = millis();           // Start timer
    while (!Radio1.available()) { // Await the sender....
        delay(5);
        KickTheDog(); // Watchdog
        if ((millis() - RXTimer) / 1000 >= FILETIMEOUT) {
#ifdef DB_MODEL_EXCHANGE
            Serial.println("Timeout");
#endif
            SendText(ModelsView_filename, TimeoutMsg);
            SetThePipe(DefaultPipe);
            Radio1.setCRCLength(RF24_CRC_8);
            return; // Give up waiting
        }
        else {
            SecondsElapsed = (millis() - RXTimer) / 1000;
            if (SecondsElapsed == (int)SecondsElapsed) { // whole number of seconds?
                strcpy(WaitMsg, Waiting);
                strcat(WaitMsg, Str(WaitTime, (FILETIMEOUT - SecondsElapsed), 0));
                strcat(WaitMsg, ThreeDots);
                SendText(ModelsView_filename, WaitMsg); // Show user how long remains to wait
            }
        }
    } // *First* packet must have arrived!
    SendCommand(ProgressStart);
    SendValue(Progress, p);
    SendText(ModelsView_filename, Receiving);
    Radio1.writeAckPayload(1, &Fack, sizeof(Fack)); // Ack first packet
    Radio1.read(&Fbuffer, BUFFERSIZE + 4);          //  Read it was 12
    strcpy(SingleModelFile, Fbuffer);               // Get filename
    Fsize = Fbuffer[BUFFERSIZE];
    Fsize += Fbuffer[BUFFERSIZE + 1] << 8;
    Fsize += Fbuffer[BUFFERSIZE + 2] << 16;
    Fsize += Fbuffer[BUFFERSIZE + 3] << 24; // Get file size
#ifdef DB_MODEL_EXCHANGE
    Serial.println("CONNECTED!");
    Serial.print("FileName=");
    Serial.println(SingleModelFile);
    Serial.print("File size = ");
    Serial.println(Fsize);
#endif
    Fposition        = 0;
    ModelsFileNumber = SD.open(SingleModelFile, FILE_WRITE);                    // Open file to receive
    RXTimer          = millis();                                                // zero timeout
    while ((Fposition < Fsize) && (millis() - RXTimer) / 1000 <= FILETIMEOUT) { //  (Fposition<Fsize) ********************
        KickTheDog();                                                           // Watchdog
        if (Radio1.available()) {
            Radio1.flush_tx();
            Radio1.writeAckPayload(1, &Fack, sizeof(Fack));
            Radio1.read(&Fbuffer, BUFFERSIZE + 4);
            ModelsFileNumber.seek(Fposition);            // Move filepointer
            ModelsFileNumber.write(Fbuffer, BUFFERSIZE); // Write part of file
            Radio1.flush_rx();
            Fposition += BUFFERSIZE;
            p = ((float)Fposition / (float)Fsize) * 100;
            SendValue(Progress, p);
            delay(5);
#ifdef DB_MODEL_EXCHANGE
            PacketNumber = Fbuffer[25];
            Serial.print("PacketNumber: ");
            Serial.println(PacketNumber);
            Serial.print("Local checksum: ");
            Serial.print(LCheckSum);
            Serial.print("  Remote checksum: ");
            Serial.println(RCheckSum);
#endif
        }
    }
    SendValue(Progress, 100);
    ModelsFileNumber.close();
    BuildDirectory();
    SendText(ModelsView_filename, Success);
    delay(2000);
    SendText(ModelsView_filename, SingleModelFile);
    Radio1.setRetries(RETRYCOUNT, RETRYWAIT);
    // **************************************** Below Here the new model is imported for immediate use
    SingleModelFlag = true;
    CloseModelsFile();
    ReadOneModel(1);
    SingleModelFlag = false;
    CloseModelsFile();
    SaveAllParameters();
    CloseModelsFile();
    UpdateModelsNameEveryWhere();
    SetThePipe(DefaultPipe);
    Radio1.setCRCLength(RF24_CRC_8);
    SendCommand(ProgressEnd);
}

/*********************************************************************************************************************************/

/** @brief SEND A MODEL FILE */
void SendModelFile()
{
    char          ProgressStart[] = "vis Progress,1";
    char          ProgressEnd[]   = "vis Progress,0";
    char          Progress[]      = "Progress";
    uint64_t      TXPipe;
    uint8_t       Fack      = 1;
    unsigned long Fsize     = 0;
    unsigned long Fposition = 0;
    char          Fbuffer[BUFFERSIZE + 8]; // spare space
    uint8_t       PacketNumber = 0;
    int           p            = 5;
    SendCommand(ProgressStart);
    SendValue(Progress, p);
    delay(10);
#ifdef DB_MODEL_EXCHANGE
    Serial.print("Sending model: ");
    Serial.println(SingleModelFile);
#endif
    TXPipe           = FILEPIPEADDRESS;
    ModelsFileNumber = SD.open(SingleModelFile, O_READ); // Open file for reading
    Fsize            = ModelsFileNumber.size();          // Get file size
#ifdef DB_MODEL_EXCHANGE
    Serial.print("File Size: ");
    Serial.print(Fsize);
    Serial.println(" bytes.");
#endif
    Radio1.setChannel(FILECHANNEL);
    Radio1.setPALevel(FILEPALEVEL);
    Radio1.setRetries(15, 15);
    Radio1.openWritingPipe(TXPipe);
    Radio1.stopListening();
    delay(4);
    while (Fposition < Fsize) {
        KickTheDog(); // Watchdog
        p = ((float)Fposition / (float)Fsize) * 100;
        SendValue(Progress, p);
        PacketNumber++;
        if (PacketNumber == 1) {
            strcpy(Fbuffer, SingleModelFile); // Filename in first packet
            Fbuffer[BUFFERSIZE]     = Fsize;
            Fbuffer[BUFFERSIZE + 1] = Fsize >> 8;
            Fbuffer[BUFFERSIZE + 2] = Fsize >> 16;
            Fbuffer[BUFFERSIZE + 3] = Fsize >> 24; // SEND FILE SIZE (four bytes)
        }
        else {

            ModelsFileNumber.seek(Fposition);           // Move filepointer
            ModelsFileNumber.read(Fbuffer, BUFFERSIZE); // Read part of file
            Fposition += BUFFERSIZE;
        }
        Radio1.flush_tx();
        Radio1.flush_rx();
        if (Radio1.write(&Fbuffer, BUFFERSIZE + 4)) {
            delay(25); // allow time for receive and write
            if (Radio1.isAckPayloadAvailable()) {
                Radio1.read(&Fack, sizeof(Fack));
            }
        }
        else {
            if (PacketNumber == 2) { // error - no connection
                SetThePipe(DefaultPipe);
                SendCommand(ProgressEnd);
                return;
            }
        }
#ifdef DB_MODEL_EXCHANGE
        Serial.println(PacketNumber);
#endif
    }
    ModelsFileNumber.close();
#ifdef DB_MODEL_EXCHANGE
    Serial.println("ALL SENT.");
#endif
    SendValue(Progress, 100);
    delay(100);
    SetThePipe(DefaultPipe);
    Radio1.setRetries(RETRYCOUNT, RETRYWAIT);
    Radio1.setCRCLength(RF24_CRC_8);
    SendCommand(ProgressEnd);
}

/*********************************************************************************************************************************/

void ShowFlightMode()
{
    char FMPress1[]  = "click fm1,1";
    char FMPress2[]  = "click fm2,1";
    char FMPress3[]  = "click fm3,1";
    char FMPress4[]  = "click fm4,1";
    char FMPress10[] = "click fm1,0";
    char FMPress20[] = "click fm2,0";
    char FMPress30[] = "click fm3,0";
    char FMPress40[] = "click fm4,0";
    switch (FlightMode) {
        case 1:
            SendCommand(FMPress1);
            SendCommand(FMPress10);
            break;
        case 2:
            SendCommand(FMPress2);
            SendCommand(FMPress20);
            break;
        case 3:
            SendCommand(FMPress3);
            SendCommand(FMPress30);
            break;
        case 4:
            SendCommand(FMPress4);
            SendCommand(FMPress40);
            break;
        default:
            break;
    }
}

/*********************************************************************************************************************************/

void updateOneSwitchView()
{
    char OneSwitchView_r0[]    = "r0";     // Not used
    char OneSwitchView_r1[]    = "r1";     // Flight modes
    char OneSwitchView_r2[]    = "r2";     // Auto
    char OneSwitchView_r3[]    = "r3";     // Ch9
    char OneSwitchView_r4[]    = "r4";     // Ch10
    char OneSwitchView_r5[]    = "r5";     // Ch11
    char OneSwitchView_r6[]    = "r6";     // Ch12
    char OneSwitchViewc_revd[] = "c_revd"; // Reversed
    char SwNum[]               = "Sw";

    if (SwitchEditNumber == 1) {
        ValueSent = false; // If no setting, = Not Used
        if (FMSwitch == 1) SendValue(OneSwitchView_r1, 1);
        if (AutoSwitch == 1) SendValue(OneSwitchView_r2, 1);
        if (Channel9Switch == 1) SendValue(OneSwitchView_r3, 1);
        if (Channel10Switch == 1) SendValue(OneSwitchView_r4, 1);
        if (Channel11Switch == 1) SendValue(OneSwitchView_r5, 1);
        if (Channel12Switch == 1) SendValue(OneSwitchView_r6, 1);
        if (!ValueSent) SendValue(OneSwitchView_r0, 1); // nothing yet, so not used
        if (Switch1Reversed) SendValue(OneSwitchViewc_revd, 1);
    }
    if (SwitchEditNumber == 2) {
        ValueSent = false;
        if (FMSwitch == 2) SendValue(OneSwitchView_r1, 1);
        if (AutoSwitch == 2) SendValue(OneSwitchView_r2, 1);
        if (Channel9Switch == 2) SendValue(OneSwitchView_r3, 1);
        if (Channel10Switch == 2) SendValue(OneSwitchView_r4, 1);
        if (Channel11Switch == 2) SendValue(OneSwitchView_r5, 1);
        if (Channel12Switch == 2) SendValue(OneSwitchView_r6, 1);
        if (!ValueSent) SendValue(OneSwitchView_r0, 1); // nothing yet, so not used
        if (Switch2Reversed) SendValue(OneSwitchViewc_revd, 1);
    }
    if (SwitchEditNumber == 3) {
        ValueSent = false;
        if (FMSwitch == 3) SendValue(OneSwitchView_r1, 1);
        if (AutoSwitch == 3) SendValue(OneSwitchView_r2, 1);
        if (Channel9Switch == 3) SendValue(OneSwitchView_r3, 1);
        if (Channel10Switch == 3) SendValue(OneSwitchView_r4, 1);
        if (Channel11Switch == 3) SendValue(OneSwitchView_r5, 1);
        if (Channel12Switch == 3) SendValue(OneSwitchView_r6, 1);
        if (!ValueSent) SendValue(OneSwitchView_r0, 1); // nothing yet, so not used
        if (Switch3Reversed) SendValue(OneSwitchViewc_revd, 1);
    }
    if (SwitchEditNumber == 4) {
        ValueSent = false;
        if (FMSwitch == 4) SendValue(OneSwitchView_r1, 1);
        if (AutoSwitch == 4) SendValue(OneSwitchView_r2, 1);
        if (Channel9Switch == 4) SendValue(OneSwitchView_r3, 1);
        if (Channel10Switch == 4) SendValue(OneSwitchView_r4, 1);
        if (Channel11Switch == 4) SendValue(OneSwitchView_r5, 1);
        if (Channel12Switch == 4) SendValue(OneSwitchView_r6, 1);
        if (!ValueSent) SendValue(OneSwitchView_r0, 1); // nothing yet, so not used
        if (Switch4Reversed) SendValue(OneSwitchViewc_revd, 1);
    }
    SendValue(SwNum, SwitchEditNumber); // show switch number
}

/*********************************************************************************************************************************/

/**
 * BUTTON WAS PRESSED (DEAL WITH INPUT FROM NEXTION DISPLAY)
 *
 * This function is 1000+ lines long
 */
void Button_was_pressed()
{
    char OneSwitchView_r1[]        = "r1";     // Flight modes
    char OneSwitchView_r2[]        = "r2";     // Auto
    char OneSwitchView_r3[]        = "r3";     // Ch9
    char OneSwitchView_r4[]        = "r4";     // Ch10
    char OneSwitchView_r5[]        = "r5";     // Ch11
    char OneSwitchView_r6[]        = "r6";     // Ch12
    char OneSwitchViewc_revd[]     = "c_revd"; // Reversed
    char Write[]                   = "Write";
    char Setup[]                   = "Setup";
    char ClickX[]                  = "ClickX";
    char ClickY[]                  = "ClickY";
    char Reset[]                   = "Reset";
    char Reverse[]                 = "Reverse";
    char yy1up[]                   = "yy1up";
    char yy1down[]                 = "yy1down";
    char yy2up[]                   = "yy2up";
    char yy2down[]                 = "yy2down";
    char midlowyup[]               = "midlowyup";
    char midlowydown[]             = "midlowydown";
    char midyup[]                  = "midyup";
    char midydown[]                = "midydown";
    char midhiyup[]                = "midhiyup";
    char midhiydown[]              = "midhiydown";
    char Front_View[]              = "FrontView";
    char Sticks_View[]             = "SticksView";
    char Graph_View[]              = "GraphView";
    char Mixes_View[]              = "MixesView";
    char SetupView[]               = "MainSetup";
    char Scan_End[]                = "ScanEnd";
    char DataEnd[]                 = "DataEnd";
    char SetupViewFM[]             = "SetupViewFM:";
    char ModelNMSave[]             = "ModelNMSave";
    char Data_View[]               = "DataView";
    char Calibrate_View[]          = "CalibrateView";
    char Trim[]                    = "Trim";
    char TrimView[]                = "TrimView";
    char TR1[]                     = "TR1";
    char TR2[]                     = "TR2";
    char TR3[]                     = "TR3";
    char TR4[]                     = "TR4";
    char TRIMS50[]                 = "TRIMS50";
    char RTRIM[]                   = "RTRIM";
    char MIXES_VIEW[]              = "MIXESVIEW"; // first call
    char Fhss_View[]               = "FhssView";
    char bind[]                    = "Bind";
    char FM1[]                     = "FM 1";
    char FM2[]                     = "FM 2";
    char FM3[]                     = "FM 3";
    char FM4[]                     = "FM 4";
    char ReScan[]                  = "ReScan";
    char LoadModel[]               = "LoadModel";
    char Models_View[]             = "ModelsView";
    char Delete[]                  = "Delete";
    int  j                         = 0;
    int  p                         = 0;
    char MixesView_MixNumber[]     = "MixNumber";
    char ModelsView_ModelNumber[]  = "ModelNumber";
    char page_SticksView[]         = "page SticksView";
    char page_GraphView[]          = "page GraphView";
    char MixesView_Enabled[]       = "Enabled";
    char MixesView_FlightMode[]    = "FlightMode";
    char MixesView_MasterChannel[] = "MasterChannel";
    char MixesView_SlaveChannel[]  = "SlaveChannel";
    char MixesView_Reversed[]      = "Reversed";
    char MixesView_Percent[]       = "Percent";
    char page_SetupView[]          = "page SetupView";
    char page_FrontView[]          = "page FrontView";
    char GoSetupView[]             = "GoSetupView";
    char GoFrontView[]             = "GoFrontView";
    char SvT11[]                   = "t11";
    char CMsg1[]                   = "Move all controls\r\nto their full extent several times,\r\nthen press the button again.";
    char SvB0[]                    = "b0";
    char CMsg2[]                   = "Wiggle, then press!";
    char Cmsg3[]                   = "Please CENTRE all controls,\r\nWait a moment,\r\nthen press again...";
    char Cmsg4[]                   = "CENTRE ALL!";
    char Cmsg5[]                   = "To calibrate TX sticks,\r\npress the button below\r\nthen follow instructions here... ";
    char Cmsg6[]                   = "Calibrate TX sticks";
    char CaliNextion[]             = "CaliNextion";
    char TypeView[]                = "TypeView";
    char CopyToAllFlightModes[]    = "callfm";
    char RXBAT[]                   = "RXBAT";
    char r2s[]                     = "r2s";
    char r3s[]                     = "r3s";
    char r4s[]                     = "r4s";
    char r5s[]                     = "r5s";
    char r6s[]                     = "r6s";
    char SwitchesView[]            = "SwitchesView";
    char SwitchesView1[]           = "SwitchesView1";
    char OneSwitchView[]           = "OneSwitchView";
    char PageOneSwitchView[]       = "page OneSwitchView";
    char PageSwitchView[]          = "page SwitchesView";
    char InputsView[]              = "InputsView";
    char InputsDone[]              = "InputsDone";
    char InPutStick_c1[]           = "c1";
    char InPutStick_c2[]           = "c2";
    char InPutStick_c3[]           = "c3";
    char InPutStick_c4[]           = "c4";
    char InPutStick_c5[]           = "c5";
    char InPutStick_c6[]           = "c6";
    char InPutStick_c7[]           = "c7";
    char InPutStick_c8[]           = "c8";
    char InPutStick_c9[]           = "c9";
    char InPutStick_c10[]          = "c10";
    char InPutStick_c11[]          = "c11";
    char InPutStick_c12[]          = "c12";
    char InPutStick_c13[]          = "c13";
    char InPutStick_c14[]          = "c14";
    char InPutStick_c15[]          = "c15";
    char InPutStick_c16[]          = "c16";
    char Export[]                  = "Export";
    char Import[]                  = "Import";
    char ListFiles[]               = "ListFiles";
    char Nextfile[]                = "Nextfile";
    char Prevfile[]                = "Prevfile";
    char DelFile[]                 = "DelFile";
    char ModExt[]                  = ".MOD";
    char FailSAVE[]                = "FailSAVE";
    char FailSafe[]                = "FailSafe";
    char fs1[]                     = "fs1";
    char fs2[]                     = "fs2";
    char fs3[]                     = "fs3";
    char fs4[]                     = "fs4";
    char fs5[]                     = "fs5";
    char fs6[]                     = "fs6";
    char fs7[]                     = "fs7";
    char fs8[]                     = "fs8";
    char fs9[]                     = "fs9";
    char fs10[]                    = "fs10";
    char fs11[]                    = "fs11";
    char fs12[]                    = "fs12";
    char fs13[]                    = "fs13";
    char fs14[]                    = "fs14";
    char fs15[]                    = "fs15";
    char fs16[]                    = "fs16";
    char CH1NAME[]                 = "CH1NAME=";
    char CH2NAME[]                 = "CH2NAME=";
    char CH3NAME[]                 = "CH3NAME=";
    char CH4NAME[]                 = "CH4NAME=";
    char CH5NAME[]                 = "CH5NAME=";
    char CH6NAME[]                 = "CH6NAME=";
    char CH7NAME[]                 = "CH7NAME=";
    char CH8NAME[]                 = "CH8NAME=";
    char CH9NAME[]                 = "CH9NAME=";
    char CH10NAME[]                = "CH10NAME=";
    char CH11NAME[]                = "CH11NAME=";
    char CH12NAME[]                = "CH12NAME=";
    char CH13NAME[]                = "CH13NAME=";
    char CH14NAME[]                = "CH14NAME=";
    char CH15NAME[]                = "CH15NAME=";
    char CH16NAME[]                = "CH16NAME=";
    char MixesView_chM[]           = "chM";
    char MixesView_chS[]           = "chS";
    char ProgressStart[]           = "vis Progress,1";
    char ProgressEnd[]             = "vis Progress,0";
    char Progress[]                = "Progress";
    char HelpView[]                = "HelpView";
    char ModelsView_filename[]     = "filename";
    char ReceiveModel[]            = "ReceiveModel";
    char SendModel[]               = "SendModel";
    char PowerOff[]                = "PowerOff";
    char OffNow[]                  = "OffNow"; // force power off
    char StillConnected[]          = "vis StillConnected,1";
    char NotStillConnected[]       = "vis StillConnected,0";
    char OptionsView[]             = "OptionsView";
    char OptionsViewS[]            = "OptionsViewS";
    char Pto[]                     = "Pto";
    char Tx_Name[]                 = "TxName";
    char Exrite[]                  = "Exrite";
    char ExpR[]                    = "Exp";
    char Smooth[]                  = "Smooth";
    char Lines[]                   = "Lines";
    char Expon[]                   = "Expo"; // number display
    char GOTO[]                    = "GOTO:";
    char WhichPage[]               = "page                            "; // excessive spaces for page name
    char AddMinute[]               = "IncMinute";
    char Dec_Minute[]              = "DecMinute";
    char Dec_Hour[]                = "DecHour";
    char Inc_Hour[]                = "IncHour";
    char Inc_Year[]                = "IncYear";
    char Dec_Year[]                = "DecYear";
    char Inc_Date[]                = "IncDate";
    char Dec_Date[]                = "DecDate";
    char Inc_Month[]               = "IncMonth";
    char Dec_Month[]               = "DecMonth";
    char pDataView[]               = "page DataView";
    char pSwitchesView[]           = "page SwitchesView";
    char pInputsView[]             = "page InputsView";
    char pOptionsViewS[]           = "page OptionsView";
    char pModelsView[]             = "page ModelsView";
    char pTrimView[]               = "page TrimView";
    char pMixesView[]              = "page MixesView";
    char pTypeView[]               = "page TypeView";
    char pCalibrateView[]          = "page CalibrateView";
    char pFailSafe[]               = "page FailSafeView";
    char DataView_Clear[]          = "Clear";
    char BuddyM[]                  = "BuddyM";
    char BuddyP[]                  = "BuddyP";
    char OptionsEnd[]              = "OptionsEnd";
    char QNH[]                     = "Qnh";
    char Mark[]                    = "Mark";
    char dGMT[]                    = "dGMT";
    char UKRULES[]                 = "UKRULES";
    char Htext0[]                  = "HELP";
    char Htext1[]                  = "Help";
    char b17[]                     = "b17";


    if (strlen(WordsIn) > 0) {
        StartInactvityTimeout();
#ifdef DB_NEXTION
        Serial.print("From Nextion: ");
        Serial.println(WordsIn);
#endif

        if (InStrng(SetupView, WordsIn) > 0) { // default goto setup screen
            ClearText();
            SaveAllParameters();
            SendCommand(page_SetupView);
            CurrentMode = NORMAL;
            CurrentView = MainSetupView;
            ClearText();
            return;
        }
        
        if (InStrng(Mark, WordsIn) > 0) {           
            GPSMarkHere = 255;                // Mark this location
            ClearText();
            return;
        }
        if (InStrng(UKRULES, WordsIn) > 0) { // UK Offcom regulations? 
            ++ UkRulesCounter;
            if (UkRulesCounter == 1) SwapWaveBandTimer = millis();
            if (UkRulesCounter == 3 ) {
                if ((millis() - SwapWaveBandTimer) < 5000){   // pressed three times in under 5 seconds?!
                    if (!UkRules){
                        SwapWaveBand  = 1;
                        UkRules = true;
                        SendText(b17,Htext1);
                    }else{
                        SwapWaveBand  = 2;
                        UkRules = false;
                        SendText(b17,Htext0);
                    }  
                }
                UkRulesCounter = 0 ;
            }
        ClearText();
        return;
        }
        
        if (InStrng(OptionsEnd, WordsIn) > 0) { // Options screen end
            DoSbusSendOnly = GetValue(BuddyP);  // Pupil, wired
            BuddyMaster    = GetValue(BuddyM);  // Master, either.
            Qnh            = GetValue(QNH);
            DeltaGMT       = GetValue(dGMT);
            FixDeltaGMTSign();
            if (DoSbusSendOnly)
            {
                Connected       = false;
                LostContactFlag = true;
                BlueLedOn();
            }
            ClearText();
            SaveAllParameters();
            SendCommand(page_SetupView);
            CurrentMode = NORMAL;
            CurrentView = MainSetupView;
            return;
        }

        if (InStrng(DataEnd, WordsIn) > 0) { //  goto setup screen from Data screen
            CurrentView = MainSetupView;
            ClearText();
            CurrentMode = NORMAL;
            return;
        }
        if (InStrng(DataView_Clear, WordsIn) > 0) { //  goto setup screen from Data screen 
            LostPackets        = 0;
            GapShortest        = 0;
            GapLongest         = 0;
            GapSum             = 0;
            GapAverage         = 0;
            GapCount           = 0;
            RXMAXModelAltitude = 0;
            GPSMaxAltitude     = 0;
            GPSMaxDistance     = 0;
            GPSMaxSpeed        = 0;
            if (!GroundModelAltitude) {
                GroundModelAltitude = RXModelAltitude;}
            else {
                GroundModelAltitude = 0 ; }
            if (!GPSGroundAltitude){
                GPSGroundAltitude = GPSAltitude;}
            else{
                GPSGroundAltitude = 0; } 
            ClearText();
            return;
        }
        if (InStrng(GoFrontView, WordsIn) > 0) {
            CurrentView = FrontView;
            SendCommand(page_FrontView);
            UpdateModelsNameEveryWhere();
            ShowFlightMode();
            LastShowTime = 0; // this is to make redisplay sooner (in ShowComms())
            LastTimeRead = 0;
            ClearText();
            return;
        }

        if (InStrng(Scan_End, WordsIn) > 0) { //  goto setup screen from Scan screen
            CurrentView = MainSetupView;
            ClearText();
            SendCommand(page_SetupView);
            DoScanEnd();
            return;
        }

        if (InStrng(HelpView, WordsIn) > 0) {
            SavedCurrentView = CurrentView;
            CurrentView      = Help_View;
            SendHelp();
            ClearText();
            return;
        }
        if (InStrng(Dec_Minute, WordsIn) > 0) {
            DecMinute();
            ClearText();
            return;
        }
        if (InStrng(AddMinute, WordsIn) > 0) {
            IncMinute();
            ClearText();
            return;
        }

        if (InStrng(Dec_Hour, WordsIn) > 0) {
            DecHour();
            ClearText();
            return;
        }
        if (InStrng(Inc_Hour, WordsIn) > 0) {
            IncHour();
            ClearText();
            return;
        }

        if (InStrng(Dec_Year, WordsIn) > 0) {
            DecYear();
            ClearText();
            return;
        }
        if (InStrng(Inc_Year, WordsIn) > 0) {
            IncYear();
            ClearText();
            return;
        }

        if (InStrng(Dec_Date, WordsIn) > 0) {
            DecDate();
            ClearText();
            return;
        }
        if (InStrng(Inc_Date, WordsIn) > 0) {
            IncDate();
            ClearText();
            return;
        }

        if (InStrng(Dec_Month, WordsIn) > 0) {
            DecMonth();
            ClearText();
            return;
        }
        if (InStrng(Inc_Month, WordsIn) > 0) {
            IncMonth();
            ClearText();
            return;
        }

        if (InStrng(OptionsViewS, WordsIn) > 0) { 
            FixDeltaGMTSign();
            SendCommand(pOptionsViewS);
            SendValue(ScreenViewTimeout, ScreenTimeout);
            SendValue(BuddyM, BuddyMaster);
            SendValue(BuddyP, DoSbusSendOnly);
            SendValue(Pto, (Inactivity_Timeout / TICKSPERMINUTE));
            SendText(Tx_Name, TxName);
            SendValue(QNH,Qnh);
            SendValue(dGMT,DeltaGMT);
            CurrentView = Options_View;
            CurrentMode = NORMAL;
            ClearText();
            return;
        }
        if (InStrng(GOTO, WordsIn) > 0) {
            i = 5;
            while (uint8_t(WordsIn[i]) > 0 && i < 30) {
                WhichPage[i] = WordsIn[i];
                ++i;
                WhichPage[i] = 0;
            } // Get page name to which to return
            SendCommand(WhichPage);
            CurrentView = SavedCurrentView;
            if (CurrentView == GraphView) {
                DisplayCurve();
                SendValue(CopyToAllFlightModes, 0);
            }
            if (CurrentView == Switches_View) {
                UpdateSwitchesDisplay();
            }
            if (CurrentView == One_Switch_View) {
                updateOneSwitchView();
            }
            if (CurrentView == ModelsView) {
                SendValue(ModelsView_ModelNumber, ModelNumber);
            }
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(Exrite, WordsIn) > 0) { //  *******************
            if (GetValue(ExpR)) {
                InterpolationTypes[FlightMode][ChanneltoSet - 1] = 2;
            }
            if (GetValue(Smooth)) {
                InterpolationTypes[FlightMode][ChanneltoSet - 1] = 1;
            }
            if (GetValue(Lines)) {
                InterpolationTypes[FlightMode][ChanneltoSet - 1] = 0;
            }
            Exponential[FlightMode][ChanneltoSet - 1] = (GetValue(Expon));
            ClearText();
            DisplayCurve();
            return;
        }

        p = InStrng(OptionsView, WordsIn); 
        if (p > 0) {
            i = strlen(OptionsView);
            j = 0;
            while (uint8_t(WordsIn[i]) > 0 && i < 100) {
                TxName[j] = WordsIn[i];
                ++j;
                ++i;
                TxName[j] = 0;
            }
            ScreenTimeout      = GetValue(ScreenViewTimeout);
            Inactivity_Timeout = GetValue(Pto) * TICKSPERMINUTE;
            if (Inactivity_Timeout < INACTIVITYMINIMUM) Inactivity_Timeout = INACTIVITYMINIMUM;
            if (Inactivity_Timeout > INACTIVITYMAXIMUM) Inactivity_Timeout = INACTIVITYMAXIMUM;
            SendText(Tx_Name, TxName);
            CurrentView = Options_View;
            CurrentMode = NORMAL;
            SaveTXStuff();
            ClearText();
            return;
        }

        if (InStrng(ReceiveModel, WordsIn) > 0) {
            i = strlen(ReceiveModel);
            j = 0;
            while (uint8_t(WordsIn[i] && i < 100) > 0) {
                SingleModelFile[j] = WordsIn[i];
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            } // got local name but won't use it.....
            ReceiveModelFile();
            ClearText();
            return;
        }
        if (InStrng(PowerOff, WordsIn) > 0) {
            if (!LostContactFlag && BoundFlag) {
                SendCommand(StillConnected);
                delay(750); // 3/4 second
                SendCommand(NotStillConnected);
            }
            else {
                digitalWrite(POWER_OFF_PIN, HIGH);
            }
            return;
        }

        if (InStrng(OffNow, WordsIn) > 0) {
            digitalWrite(POWER_OFF_PIN, HIGH); // force OFF in Options View
            ClearText();
            return;
        }

        if (InStrng(SendModel, WordsIn) > 0) {
            i = strlen(SendModel);
            j = 0;
            while (uint8_t(WordsIn[i]) > 0) {
                SingleModelFile[j] = WordsIn[i];
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            }
            SendModelFile();
            ClearText();
            return;
        }
        if (InStrng(FailSAVE, WordsIn) > 0) {
            SendCommand(ProgressStart);
            FailSafeChannel[0] = GetValue(fs1);
            SendValue(Progress, 5);
            FailSafeChannel[1] = GetValue(fs2);
            FailSafeChannel[2] = GetValue(fs3);
            SendValue(Progress, 25);
            FailSafeChannel[3] = GetValue(fs4);
            FailSafeChannel[4] = GetValue(fs5);
            FailSafeChannel[5] = GetValue(fs6);
            SendValue(Progress, 50);
            FailSafeChannel[6] = GetValue(fs7);
            FailSafeChannel[7] = GetValue(fs8);
            FailSafeChannel[8] = GetValue(fs9);
            FailSafeChannel[9] = GetValue(fs10);
            SendValue(Progress, 75);
            FailSafeChannel[10] = GetValue(fs11);
            FailSafeChannel[11] = GetValue(fs12);
            FailSafeChannel[12] = GetValue(fs13);
            SendValue(Progress, 100);
            FailSafeChannel[13] = GetValue(fs14);
            FailSafeChannel[14] = GetValue(fs15);
            FailSafeChannel[15] = GetValue(fs16);
            SaveOneModel(ModelNumber);
          //  SendCommand(ProgressEnd);
            ClearText();
            FailSafeTimer= millis();
            SaveFailSafeNow = true;
            return;
        }
        if (InStrng(FailSafe, WordsIn) > 0) {
            SendCommand(pFailSafe);
            SendValue(fs1, FailSafeChannel[0]);
            SendValue(fs2, FailSafeChannel[1]);
            SendValue(fs3, FailSafeChannel[2]);
            SendValue(fs4, FailSafeChannel[3]);
            SendValue(fs5, FailSafeChannel[4]);
            SendValue(fs6, FailSafeChannel[5]);
            SendValue(fs7, FailSafeChannel[6]);
            SendValue(fs8, FailSafeChannel[7]);
            SendValue(fs9, FailSafeChannel[8]);
            SendValue(fs10, FailSafeChannel[9]);
            SendValue(fs11, FailSafeChannel[10]);
            SendValue(fs12, FailSafeChannel[11]);
            SendValue(fs13, FailSafeChannel[12]);
            SendValue(fs14, FailSafeChannel[13]);
            SendValue(fs15, FailSafeChannel[14]);
            SendValue(fs16, FailSafeChannel[15]);
            CurrentView = FailSafe_View;
            UpdateButtonLabels();
            ClearText();
            return;
        }

        if (InStrng(OneSwitchView, WordsIn) > 0) {
            SwitchEditNumber = GetChannel(); // which switch?
            CurrentView      = One_Switch_View;
            SendCommand(PageOneSwitchView); // edit one switch - could be 1-4
            updateOneSwitchView();
            ClearText();
            return;
        }

        if (InStrng(SwitchesView1, WordsIn) > 0) { //  read switch values from screen (could be 1-4)
            if (GetValue(OneSwitchView_r1)) {
                FMSwitch = SwitchEditNumber;
            }
            else {
                if (FMSwitch == SwitchEditNumber) FMSwitch = 0;
            }
            if (GetValue(OneSwitchView_r2)) {
                AutoSwitch = SwitchEditNumber;
            }
            else {
                if (AutoSwitch == SwitchEditNumber) AutoSwitch = 0;
            }
            if (GetValue(OneSwitchView_r3)) {
                Channel9Switch = SwitchEditNumber;
            }
            else {
                if (Channel9Switch == SwitchEditNumber) Channel9Switch = 0;
            }
            if (GetValue(OneSwitchView_r4)) {
                Channel10Switch = SwitchEditNumber;
            }
            else {
                if (Channel10Switch == SwitchEditNumber) Channel10Switch = 0;
            }
            if (GetValue(OneSwitchView_r5)) {
                Channel11Switch = SwitchEditNumber;
            }
            else {
                if (Channel11Switch == SwitchEditNumber) Channel11Switch = 0;
            }
            if (GetValue(OneSwitchView_r6)) {
                Channel12Switch = SwitchEditNumber;
            }
            else {
                if (Channel12Switch == SwitchEditNumber) Channel12Switch = 0;
            }

            if (SwitchEditNumber == 1) {
                if (GetValue(OneSwitchViewc_revd)) {
                    Switch1Reversed = true;
                }
                else {
                    Switch1Reversed = false;
                }
            }
            if (SwitchEditNumber == 2) {
                if (GetValue(OneSwitchViewc_revd)) {
                    Switch2Reversed = true;
                }
                else {
                    Switch2Reversed = false;
                }
            }
            if (SwitchEditNumber == 3) {
                if (GetValue(OneSwitchViewc_revd)) {
                    Switch3Reversed = true;
                }
                else {
                    Switch3Reversed = false;
                }
            }
            if (SwitchEditNumber == 4) {
                if (GetValue(OneSwitchViewc_revd)) {
                    Switch4Reversed = true;
                }
                else {
                    Switch4Reversed = false;
                }
            }
            SaveOneModel(ModelNumber);
            SendCommand(PageSwitchView); // change to all switches screen
            UpdateSwitchesDisplay();     // update its info
            ClearText();
            return;
        }
        if (InStrng(InputsView, WordsIn) > 0) {
            SendCommand(pInputsView);
            CurrentView = Inputs_View;
            UpdateButtonLabels();
            ClearText();
            return;
        }

        if (InStrng(InputsDone, WordsIn) > 0) {
            SendCommand(ProgressStart);
            InPutStick[0] = CheckRange_0_16(GetValue(InPutStick_c1)) - 1;
            SendValue(Progress, 5);
            InPutStick[1] = CheckRange_0_16(GetValue(InPutStick_c2)) - 1;
            SendValue(Progress, 15);
            InPutStick[2] = CheckRange_0_16(GetValue(InPutStick_c3)) - 1;
            SendValue(Progress, 25);
            InPutStick[3] = CheckRange_0_16(GetValue(InPutStick_c4)) - 1;
            SendValue(Progress, 35);
            InPutStick[4] = CheckRange_0_16(GetValue(InPutStick_c5)) - 1;
            SendValue(Progress, 45);
            InPutStick[5] = CheckRange_0_16(GetValue(InPutStick_c6)) - 1;
            SendValue(Progress, 55);
            InPutStick[6] = CheckRange_0_16(GetValue(InPutStick_c7)) - 1;
            SendValue(Progress, 65);
            InPutStick[7] = CheckRange_0_16(GetValue(InPutStick_c8)) - 1;
            SendValue(Progress, 75);
            InPutStick[8] = CheckRange_0_16(GetValue(InPutStick_c9)) - 1;
            SendValue(Progress, 80);
            InPutStick[9] = CheckRange_0_16(GetValue(InPutStick_c10)) - 1;
            SendValue(Progress, 85);
            InPutStick[10] = CheckRange_0_16(GetValue(InPutStick_c11)) - 1;
            SendValue(Progress, 86);
            InPutStick[11] = CheckRange_0_16(GetValue(InPutStick_c12)) - 1;
            SendValue(Progress, 87);
            InPutStick[12] = CheckRange_0_16(GetValue(InPutStick_c13)) - 1;
            SendValue(Progress, 88);
            InPutStick[13] = CheckRange_0_16(GetValue(InPutStick_c14)) - 1;
            SendValue(Progress, 89);
            InPutStick[14] = CheckRange_0_16(GetValue(InPutStick_c15)) - 1;
            SendValue(Progress, 95);
            InPutStick[15] = CheckRange_0_16(GetValue(InPutStick_c16)) - 1;
            SendValue(Progress, 99);
            SaveOneModel(ModelNumber);
            SendValue(Progress, 100);
            CurrentMode = NORMAL;
            SendCommand(ProgressEnd);
            UpdateButtonLabels();
            SendCommand(page_SetupView);
            ClearText();
            return;
        }
        if (InStrng(CH1NAME, WordsIn) > 0) {
            p = InStrng(CH1NAME, WordsIn);
            i = p + 7;
            DoNewChannelName(1, i);
            ClearText();
            return;
        }
        if (InStrng(CH2NAME, WordsIn) > 0) {
            p = InStrng(CH2NAME, WordsIn);
            i = p + 7;
            DoNewChannelName(2, i);
            ClearText();
            return;
        }
        if (InStrng(CH3NAME, WordsIn) > 0) {
            p = InStrng(CH3NAME, WordsIn);
            i = p + 7;
            DoNewChannelName(3, i);
            ClearText();
            return;
        }
        if (InStrng(CH4NAME, WordsIn) > 0) {
            p = InStrng(CH4NAME, WordsIn);
            i = p + 7;
            DoNewChannelName(4, i);
            ClearText();
            return;
        }
        if (InStrng(CH5NAME, WordsIn) > 0) {
            p = InStrng(CH5NAME, WordsIn);
            i = p + 7;
            DoNewChannelName(5, i);
            ClearText();
            return;
        }
        if (InStrng(CH6NAME, WordsIn) > 0) {
            p = InStrng(CH6NAME, WordsIn);
            i = p + 7;
            DoNewChannelName(6, i);
            ClearText();
            return;
        }
        if (InStrng(CH7NAME, WordsIn) > 0) {
            p = InStrng(CH7NAME, WordsIn);
            i = p + 7;
            DoNewChannelName(7, i);
            ClearText();
            return;
        }
        if (InStrng(CH8NAME, WordsIn) > 0) {
            p = InStrng(CH8NAME, WordsIn);
            i = p + 7;
            DoNewChannelName(8, i);
            ClearText();
            return;
        }
        if (InStrng(CH9NAME, WordsIn) > 0) {
            p = InStrng(CH9NAME, WordsIn);
            i = p + 7;
            DoNewChannelName(9, i);
            ClearText();
            return;
        }
        if (InStrng(CH10NAME, WordsIn) > 0) {
            p = InStrng(CH10NAME, WordsIn);
            i = p + 8;
            DoNewChannelName(10, i);
            ClearText();
            return;
        }
        if (InStrng(CH11NAME, WordsIn) > 0) {
            p = InStrng(CH11NAME, WordsIn);
            i = p + 8;
            DoNewChannelName(11, i);
            ClearText();
            return;
        }
        if (InStrng(CH12NAME, WordsIn) > 0) {
            p = InStrng(CH12NAME, WordsIn);
            i = p + 8;
            DoNewChannelName(12, i);
            ClearText();
            return;
        }
        if (InStrng(CH13NAME, WordsIn) > 0) {
            p = InStrng(CH13NAME, WordsIn);
            i = p + 8;
            DoNewChannelName(13, i);
            ClearText();
            return;
        }
        if (InStrng(CH14NAME, WordsIn) > 0) {
            p = InStrng(CH14NAME, WordsIn);
            i = p + 8;
            DoNewChannelName(14, i);
            ClearText();
            return;
        }
        if (InStrng(CH15NAME, WordsIn) > 0) {
            p = InStrng(CH15NAME, WordsIn);
            i = p + 8;
            DoNewChannelName(15, i);
            ClearText();
            return;
        }
        if (InStrng(CH16NAME, WordsIn) > 0) {
            p = InStrng(CH16NAME, WordsIn);
            i = p + 8;
            DoNewChannelName(16, i);
            ClearText();
            return;
        }

        if (InStrng(DelFile, WordsIn) > 0) { // Delete a file
            j = 0;
            p = InStrng(DelFile, WordsIn);
            i = p + 6;
            while (uint8_t(WordsIn[i]) > 0) {
                SingleModelFile[j] = WordsIn[i];
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            }
            SD.remove(SingleModelFile);
            BuildDirectory();
            FileNumberInView--;
            ShowFileNumber();
            CloseModelsFile();
            ClearText();
            return;
        }

        if (InStrng(Nextfile, WordsIn)) { // show next file
            FileNumberInView++;
            ShowFileNumber();
            CloseModelsFile();
            ClearText();
            return;
        }

        if (InStrng(Prevfile, WordsIn)) { // show prev file
            FileNumberInView--;
            ShowFileNumber();
            CloseModelsFile();
            ClearText();
            return;
        }

        if (InStrng(SwitchesView, WordsIn)) {
            SendCommand(pSwitchesView);
            UpdateSwitchesDisplay(); // display saved values
            CurrentView = Switches_View;
            ClearText();
            return;
        }

        if (InStrng(Calibrate_View, WordsIn)) {
            SendCommand(pCalibrateView);
            Force_ReDisplay();
            CurrentView = CalibrateView;
            ClearText();
            return;
        }

        p = InStrng(Export, WordsIn);
        if (p > 0) {
            j = 0;
            i = p + 5;
            while ((WordsIn[i]) > 0) {
                if (WordsIn[i] >= 97 && WordsIn[i] <= 122) {
                    WordsIn[i] &= ~0x20;
                } // upper case only
                if (WordsIn[i] <= 32 || WordsIn[i] > 127) break;
                SingleModelFile[j] = WordsIn[i];
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            }

            if ((InStrng(ModExt, SingleModelFile) == 0) && (strlen(SingleModelFile) <= 8)) {
                strcat(SingleModelFile, ModExt);
            }
            if ((strlen(SingleModelFile) <= 12) && (InStrng(ModExt, SingleModelFile) > 0))
            {
                SendText(ModelsView_filename, SingleModelFile);
                SendCommand(ProgressStart);
                delay(20);
                SendValue(Progress, 10);
                delay(20);
                CloseModelsFile();
                for (uint8_t WriteTwice = 1; WriteTwice <= 2; ++WriteTwice) { // if a new file, write twice seems to be needed!!
                    SingleModelFlag = true;
                    OpenModelsFile();
                    SendValue(Progress, 25);
                    delay(10);
                    SaveOneModel(1);
                    SendValue(Progress, 50);
                    delay(10);
                    CloseModelsFile();
                }
                SingleModelFlag = false;
                SendValue(Progress, 75);
                delay(10);
                BuildDirectory();
                SendValue(Progress, 100);
                delay(10);
                SendCommand(ProgressEnd);
            }
            else {
                FileError = true;
            }

            if (FileError) ShowFileErrorMsg();
            ClearText();
            return;
        }

        p = InStrng(Import, WordsIn);
        if (p > 0) {
            SendCommand(ProgressStart);
            delay(10);
            SendValue(Progress, 5);
            delay(10);
            j = 0;
            i = p + 5;
            while (WordsIn[i] > 0) {
                if (WordsIn[i] >= 97 && WordsIn[i] <= 122) {
                    WordsIn[i] &= ~0x20;
                }
                SingleModelFile[j] = WordsIn[i];
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            }
            if (InStrng(ModExt, SingleModelFile) == 0) strcat(SingleModelFile, ModExt);
            SingleModelFlag = true;
            SendValue(Progress, 10);
            delay(10);
            CloseModelsFile();
            ReadOneModel(1);
            SendValue(Progress, 50);
            delay(10);
            SingleModelFlag = false;
            CloseModelsFile();
            SendValue(Progress, 75);
            delay(10);
            SaveAllParameters();
            CloseModelsFile();
            UpdateModelsNameEveryWhere();
            SendValue(Progress, 100);
            delay(10);
            SendCommand(ProgressEnd);
            if (FileError) ShowFileErrorMsg();
            ClearText();
            return;
        }

        if (InStrng(ListFiles, WordsIn) > 0) {
            ShowDirectory(); //
            ClearText();
            return;
        }

        if (InStrng(CaliNextion, WordsIn) > 0) {
            SendCommand(CalibrateNow);
            ClearText();
            return;
        }

        if (InStrng(SetupViewFM, WordsIn) > 0) { // New model name occurs at offset 12 in WordsIn
            i = 0;
            while (WordsIn[i + 12] > 0) {
                ModelName[i]     = WordsIn[i + 12];
                ModelName[i + 1] = 0;
                ++i;
            } // copy new name
            SaveOneModel(ModelNumber);
            ClearText();
            SendCommand(page_SetupView);
            CurrentMode = NORMAL; // Send data again
            CurrentView = MainSetupView;
            ClearText();
            return;
        }

        if (InStrng(ModelNMSave, WordsIn) > 0) { // edit modelname  
            InhibitNameCheck = true;
            i = 0;
            while (WordsIn[i + 12] > 0) {
                ModelName[i] = WordsIn[i + 12];  // copy new user supplied name
                ModelName[i + 1] = 0;
                ++i;
            } 
            ModelNumber = GetValue(ModelsView_ModelNumber);   
            Serial.println (ModelNumber);
            SaveOneModel(ModelNumber);
            ClearText();
            delay (1500);                        // allow time for SD write to happen
            InhibitNameCheck = false;
            return;
        }

        if (InStrng(GoSetupView, WordsIn) > 0) {
            ClearText();
            CurrentView = MainSetupView;
            SendCommand(page_SetupView);
            ClearText();
            return;
        }

        if (InStrng(TypeView, WordsIn) > 0) {
            SendCommand(pTypeView);
            SendValue(r2s, 0); // Zero all RX batt cell count
            SendValue(r3s, 0);
            SendValue(r4s, 0);
            SendValue(r5s, 0);
            SendValue(r6s, 0);
            if (RXCellCount == 2) SendValue(r2s, 1); // Then update RX batt cell count
            if (RXCellCount == 3) SendValue(r3s, 1);
            if (RXCellCount == 4) SendValue(r4s, 1);
            if (RXCellCount == 5) SendValue(r5s, 1);
            if (RXCellCount == 6) SendValue(r6s, 1);
            ClearText();
            return;
        }

        if (InStrng(RXBAT, WordsIn) > 0) { // UPdate RX batt cell count
            if (GetValue(r2s) == 1) RXCellCount = 2;
            if (GetValue(r3s) == 1) RXCellCount = 3;
            if (GetValue(r4s) == 1) RXCellCount = 4;
            if (GetValue(r5s) == 1) RXCellCount = 5;
            if (GetValue(r6s) == 1) RXCellCount = 6;
            SaveOneModel(ModelNumber);
            ClearText();
            return;
        }

        if (InStrng(TrimView, WordsIn) > 0) { // TrimView just appeared, so update it.
            SendCommand(pTrimView);
            CurrentView = Trim_View;
            UpdateModelsNameEveryWhere(); // also updates trimview
            if (!UkRules){
                    SendText(b17,Htext0);
                }else{
                    SendText(b17,Htext1);
                } 
            ClearText();
            return;
        }

        if (InStrng(RTRIM, WordsIn) > 0) {
            TrimsReversed[FlightMode][0] = GetValue(TrimView_r1); // Fudged as I'd numbered differently!
            TrimsReversed[FlightMode][1] = GetValue(TrimView_r4);
            TrimsReversed[FlightMode][2] = GetValue(TrimView_r2);
            TrimsReversed[FlightMode][3] = GetValue(TrimView_r3);
            ClearText();
            return;
        }

        if (InStrng(TRIMS50, WordsIn) > 0) {
            for (i = 0; i < 4; ++i) {
                Trims[FlightMode][i] = 80; // Mid value is 80
            }
        }
        if (InStrng(TR1, WordsIn) > 0) { //  TR1->0
            Trims[FlightMode][0] = WordsIn[3];
        }
        if (InStrng(TR4, WordsIn) > 0) { // TR4 ->1
            Trims[FlightMode][1] = WordsIn[3];
        }
        if (InStrng(TR2, WordsIn) > 0) { // TR2 ->2
            Trims[FlightMode][2] = WordsIn[3];
        }
        if (InStrng(TR3, WordsIn) > 0) { // TR3 ->3
            Trims[FlightMode][3] = WordsIn[3];
        }

        if (InStrng(Trim, WordsIn) > 0) {
            SaveOneModel(ModelNumber); // save trims to SDcard
        }
        if (InStrng(Models_View, WordsIn) > 0) {
            SendCommand(pModelsView);
            ReadOneModel(ModelNumber);

           // Serial.println ("HERE?????????????");

            CurrentView = ModelsView;
            UpdateModelsNameEveryWhere();
            SendValue(ModelsView_ModelNumber, ModelNumber);
            BuildDirectory(); // of SD card
            ShowFileNumber();
        }
        if (InStrng(LoadModel, WordsIn) > 0) {
            ModelNumber = GetValue(ModelsView_ModelNumber);
            if (ModelNumber >= 99) {
                ModelNumber = 1;
                SendValue(ModelsView_ModelNumber, ModelNumber);
            }
            if (!ModelsFileOpen) OpenModelsFile();
            SDUpdateByte(ModelNumberOffset, ModelNumber);  // the offset was grabbed when loading file 
            CloseModelsFile();
            ClearText(); 
        }

        if (InStrng(Delete, WordsIn) > 0) {
            ModelNumber = GetValue(ModelsView_ModelNumber);
            SetDefaultValues();
            SaveOneModel(ModelNumber);
            ClearText();
            return;
        }

        if (InStrng(Write, WordsIn) > 0) { //  write new data to SD
            p = GetValue(CopyToAllFlightModes);
            if (p == 1) {
                for (p = 1; p <= 4; p++) {
                    if (p != FlightMode) {
                        MinDegrees[p][ChanneltoSet - 1]         = MinDegrees[FlightMode][ChanneltoSet - 1];
                        MidLowDegrees[p][ChanneltoSet - 1]      = MidLowDegrees[FlightMode][ChanneltoSet - 1];
                        CentreDegrees[p][ChanneltoSet - 1]      = CentreDegrees[FlightMode][ChanneltoSet - 1];
                        MidHiDegrees[p][ChanneltoSet - 1]       = MidHiDegrees[FlightMode][ChanneltoSet - 1];
                        MaxDegrees[p][ChanneltoSet - 1]         = MaxDegrees[FlightMode][ChanneltoSet - 1];
                        InterpolationTypes[p][ChanneltoSet - 1] = InterpolationTypes[FlightMode][ChanneltoSet - 1];
                        Exponential[p][ChanneltoSet - 1]        = Exponential[FlightMode][ChanneltoSet - 1];
                    }
                }
            }
            ChanneltoSet = 0;
            SaveOneModel(ModelNumber);
            Force_ReDisplay();
            SendCommand(page_SticksView); // Set to SticksView
            CurrentView = SticksView;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(Setup, WordsIn) > 0) { // Which channel to setup ... Goes to GraphView
            ChanneltoSet = GetChannel();
            ClearText();
            SendCommand(page_GraphView); // Set to GraphView
            CurrentView = GraphView;
            DisplayCurve(); // redisplay curve
            updateInterpolationTypes();
            UpdateModelsNameEveryWhere();
            SendValue(CopyToAllFlightModes, 0);
            ClearText();
            return;
        }

        p = (InStrng(Front_View, WordsIn)); //which screen is in view?
        if (p > 0) {
            CurrentView = FrontView;
            ClearText();
            PreviousFlightMode = 250; // sure to be different
            CurrentMode        = NORMAL;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        p = (InStrng(Sticks_View, WordsIn));
        if (p > 0) {
            SendCommand(page_SticksView);
            Force_ReDisplay();
            CurrentView = SticksView;
            SendCommand(page_SticksView); // Set to SticksView
            UpdateModelsNameEveryWhere();
            UpdateButtonLabels();
            ClearText();
            return;
        }

        if (InStrng(Fhss_View, WordsIn))
            if (!BoundFlag) { // no scan while connected!!!
                {
                    SendCommand(page_FhssView);
                    DrawFhssBox();
                    DoScanInit();
                    CurrentMode = SCANWAVEBAND;
                    BlueLedOn();
                    ClearText();
                }
                return;
            }

        if (InStrng(ReScan, WordsIn))
        {
            DrawFhssBox();
            DoScanInit();
            ClearText();
            return;
        }

        p = (InStrng(MIXES_VIEW, WordsIn)); //
        if (p > 0) {
            SendCommand(pMixesView);
            CurrentView = MixesView;
            UpdateModelsNameEveryWhere();
            if (MixNumber == 0) MixNumber = 1;
            LastMixNumber = 33;                        // just to be differernt
            SendValue(MixesView_MixNumber, MixNumber); // New load of mix window
            ClearText();
            return;
        }

        p = (InStrng(Mixes_View, WordsIn)); // Get New Mixes!
        if (p > 0) {
            CurrentView = MixesView;
            UpdateModelsNameEveryWhere();
            MixNumber = GetValue(MixesView_MixNumber);
            if (LastMixNumber != MixNumber) { // Did it change?
                LastMixNumber = MixNumber;
                SendMixValues();
            }
            else {
                Mixes[MixNumber][M_Enabled]       = GetValue(MixesView_Enabled);
                Mixes[MixNumber][M_FlightMode]    = GetValue(MixesView_FlightMode);
                Mixes[MixNumber][M_MasterChannel] = GetValue(MixesView_MasterChannel);
                Mixes[MixNumber][M_SlaveChannel]  = GetValue(MixesView_SlaveChannel);
                Mixes[MixNumber][M_Reversed]      = GetValue(MixesView_Reversed);
                Mixes[MixNumber][M_Percent]       = GetValue(MixesView_Percent);
                SendText(MixesView_chM, ChannelNames[Mixes[MixNumber][M_MasterChannel] - 1]);
                SendText(MixesView_chS, ChannelNames[Mixes[MixNumber][M_SlaveChannel] - 1]);
            }
            ClearText();
            return;
        }

        if (InStrng(Graph_View, WordsIn))
        {
            CurrentView = GraphView;
            ClearText();
            return;
        }

        if (InStrng(Data_View, WordsIn))
        {
            CurrentMode  = NORMAL;
            CurrentView  = DataView;
            LastShowTime = 0;
            SendCommand(pDataView);
            ClearText();
            return;
        }

        if (InStrng(bind, WordsIn))
        {
            BindNow();
            ClearText();
            return;
        }

        if (InStrng(FM1, WordsIn))
        {
            FlightMode         = 1;
            PreviousFlightMode = 1;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(FM2, WordsIn))
            if (p > 0) {
                FlightMode         = 2;
                PreviousFlightMode = 2;
                UpdateModelsNameEveryWhere();
                ClearText();
                return;
            }

        if (InStrng(FM3, WordsIn))
        {
            FlightMode         = 3;
            PreviousFlightMode = 3;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(FM4, WordsIn))
        {
            FlightMode         = 4;
            PreviousFlightMode = 4;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }
        if (InStrng(midyup, WordsIn)) // midy up?
        {
            CentreDegrees[FlightMode][ChanneltoSet - 1]++;
            DisplayCurve();
            ClearText();
            return;
        }

        if (InStrng(midydown, WordsIn)) // midy down?
        {
            CentreDegrees[FlightMode][ChanneltoSet - 1]--;
            DisplayCurve();
            ClearText();
            return;
        }

        if (InStrng(midhiyup, WordsIn)) // midhiy up?
        {
            MidHiDegrees[FlightMode][ChanneltoSet - 1]++;
            DisplayCurve();
            ClearText();
            return;
        }

        if (InStrng(midhiydown, WordsIn)) // midhiy down?
        {
            MidHiDegrees[FlightMode][ChanneltoSet - 1]--;
            DisplayCurve();
            ClearText();
            return;
        }

        if (InStrng(midlowyup, WordsIn)) // midlowy up?
        {
            MidLowDegrees[FlightMode][ChanneltoSet - 1]++;
            DisplayCurve();
            ClearText();
            return;
        }

        if (InStrng(midlowydown, WordsIn)) // midlowy down?
        {
            MidLowDegrees[FlightMode][ChanneltoSet - 1]--;
            DisplayCurve();
            ClearText();
            return;
        }

        if (InStrng(yy1up, WordsIn)) // yy1 up?
        {
            MaxDegrees[FlightMode][ChanneltoSet - 1]++;
            DisplayCurve();
            ClearText();
            return;
        }

        if (InStrng(yy1down, WordsIn)) // yy1 down?
        {
            MaxDegrees[FlightMode][ChanneltoSet - 1]--;
            DisplayCurve();
            ClearText();
            return;
        }

        if (InStrng(yy2up, WordsIn)) // yy1 up?
        {
            MinDegrees[FlightMode][ChanneltoSet - 1]++;
            DisplayCurve();
            ClearText();
            return;
        }

        if (InStrng(yy2down, WordsIn)) // yy1 down?
        {
            MinDegrees[FlightMode][ChanneltoSet - 1]--;
            DisplayCurve();
            ClearText();
            return;
        }

        if (InStrng(Reset, WordsIn)) // RESET?
        {
            MinDegrees[FlightMode][ChanneltoSet - 1]         = 30;
            MidLowDegrees[FlightMode][ChanneltoSet - 1]      = 60;
            CentreDegrees[FlightMode][ChanneltoSet - 1]      = 90;
            MidHiDegrees[FlightMode][ChanneltoSet - 1]       = 120;
            MaxDegrees[FlightMode][ChanneltoSet - 1]         = 150;
            Exponential[FlightMode][ChanneltoSet - 1]        = 0;
            InterpolationTypes[FlightMode][ChanneltoSet - 1] = 2; // expo = default
            DisplayCurve();
            ClearText();
            return;
        }

        if (InStrng(Reverse, WordsIn)) // REVERSE?
        {
            p                                           = MinDegrees[FlightMode][ChanneltoSet - 1];
            MinDegrees[FlightMode][ChanneltoSet - 1]    = 180 - p;
            p                                           = MidLowDegrees[FlightMode][ChanneltoSet - 1];
            MidLowDegrees[FlightMode][ChanneltoSet - 1] = 180 - p;
            p                                           = CentreDegrees[FlightMode][ChanneltoSet - 1];
            CentreDegrees[FlightMode][ChanneltoSet - 1] = 180 - p;
            p                                           = MidHiDegrees[FlightMode][ChanneltoSet - 1];
            MidHiDegrees[FlightMode][ChanneltoSet - 1]  = 180 - p;
            p                                           = MaxDegrees[FlightMode][ChanneltoSet - 1];
            MaxDegrees[FlightMode][ChanneltoSet - 1]    = 180 - p;
            DisplayCurve();
            ClearText();
            return;
        }
        p = (InStrng(ClickX, WordsIn)); // Clicked to move point?
        if (p > 0) {
            XtouchPlace = GetNextNumber(p + 7, WordsIn);
            // This drops through to get Y as well before moving the point
        }
        p = (InStrng(ClickY, WordsIn)); // Clicked to move point?
        if (p > 0) {
            YtouchPlace = GetNextNumber(p + 7, WordsIn);
            MovePoint();
            ClearText();
            return;
        }
        if (CurrentMode == 0) {
            if (strcmp(WordsIn, "Calibrate1") == 0) {
                CurrentMode = CALIBRATELIMITS;
                CentreMaxMins();
                SendText1(SvT11, CMsg1);
                SendText(SvB0, CMsg2);
                ClearText();
               // Serial.println ("ZERO");
               // Serial.println (CurrentMode);
                return;
            }
        }

        if (CurrentMode == 1) {
            if (strcmp(WordsIn, "Calibrate1") == 0) {
                CurrentMode = CENTRESTICKS;
                SendText(SvT11, Cmsg3);
                SendText(SvB0, Cmsg4);
                ClearText();
                //Serial.println ("ONE");
                //Serial.println (CurrentMode);
                return;
            }
        }
        if (CurrentMode == 2) {
            if (strcmp(WordsIn, "Calibrate1") == 0) {
                CurrentMode = NORMAL;
                SaveAllParameters();
                SendText(SvT11, Cmsg5);
                SendText(SvB0, Cmsg6);
                SendValue1(NextionSleepTime, ScreenTimeout); // Re enable timeout
                ClearText();
                return;
            }
        }
    }
    ClearText(); // Let's have cleared text for next one!
} // end Button_was_pressed()

/************************************************************************************************************/

uint16_t MakeTwobytes(bool* f)
{                    // Pass arraypointer. Returns the two bytes
    uint16_t tb = 0; // all false is default
    for (i = 0; i < 16; ++i) {
        if (f[15 - i] == true) {
            tb |= 1 << (i);
        } // sets a bit if true
    }
    return tb;
}

/************************************************************************************************************/

void LoadPacketData()
{
    uint16_t Twobytes = 0; // Extra data can be send using the last four bytes of each data packet. These are defined by the packet number
    uint8_t  FS_Byte1;
    uint8_t  FS_Byte2;
    char ProgressEnd[]                         = "vis Progress,0";
    SendBuffer[CHANNELSUSED] = PacketNumber;  
    Twobytes = MakeTwobytes(FailSafeChannel); // 16 bool values compressed to 16 bits
    FS_Byte1 = uint8_t(Twobytes >> 8);        // sent as two bytes
    FS_Byte2 = uint8_t(Twobytes & 0x00FF);
    SendBuffer[CHANNELSUSED + 1] = 0;
    SendBuffer[CHANNELSUSED + 2] = 0;
    switch (PacketNumber) {
        case 0:  
            SendBuffer[CHANNELSUSED + 2] = BindingNow;
            if (BindingNow == 1) {
                BindingTimer = millis(); // start a timer
                BindingNow   = 2;
            }
            if (((millis() - FailSafeTimer) > 1500) && SaveFailSafeNow) {
                    SendBuffer[CHANNELSUSED + 1] = SaveFailSafeNow; // FailSafeSaveMoment
                    SaveFailSafeNow    = false;                     // once should do it.
                    SendCommand(ProgressEnd);
            }
            break;
        case 1:
             SendBuffer[CHANNELSUSED + 1] = FS_Byte2;      // these are failsafe flags
             SendBuffer[CHANNELSUSED + 2] = FS_Byte1;      // these are failsafe flags
            break;
        case 2: 
            SendBuffer[CHANNELSUSED + 1] = Qnh >> 8;       // (HiByte)   Qnh is current atmospheric pressure at sea level here (an aviation term)
            SendBuffer[CHANNELSUSED + 2] = Qnh & 0x00ff;   // (LowByte)  Qnh is current atmospheric pressure at sea level here (an aviation term)
            break;
        case 3: 
            if (GPSMarkHere) {
                SendBuffer[CHANNELSUSED + 1] = 0;
                SendBuffer[CHANNELSUSED + 2] = GPSMarkHere;
                GPSMarkHere = 0;
            }
            break;
        case 4: 
            SendBuffer[CHANNELSUSED + 1] = 0;
            SendBuffer[CHANNELSUSED + 2] = SwapWaveBand;   // This feature allows the quiet switching between 2.400-2.4830 and 2.4840-2.525 (press HELP three times)
            if (SwapWaveBand == 2) SetTestFrequencies();
            if (SwapWaveBand == 1) SetUKFrequencies();
            SwapWaveBand = 0;
            break;
        default:
            break;
    }
}

/************************************************************************************************************/

void ReadFMSwitch(bool sw1, bool sw2, bool rev)
{
    if (sw1 == false && sw2 == false) FlightMode = 2;
    if (rev) {
        if (sw1) FlightMode = 1;
        if (sw2) FlightMode = 3;
    }
    else {
        if (sw1) FlightMode = 3;
        if (sw2) FlightMode = 1;
    }
}

/************************************************************************************************************/

uint8_t ReadCHSwitch(bool sw1, bool sw2, bool rev)
{
    uint8_t ttmp = 90;
    if (sw1 == false && sw2 == false) ttmp = 90;
    if (rev) {
        if (sw1) ttmp = 0;
        if (sw2) ttmp = 180;
    }
    else {
        if (sw1) ttmp = 180;
        if (sw2) ttmp = 0;
    }
    return ttmp;
}

/************************************************************************************************************/

uint8_t CheckSwitch(uint8_t swt)
{
    uint8_t rtv = 90;
    if (swt == 1) rtv = ReadCHSwitch(Switch[7], Switch[6], Switch1Reversed);
    if (swt == 2) rtv = ReadCHSwitch(Switch[5], Switch[4], Switch2Reversed);
    if (swt == 3) rtv = ReadCHSwitch(Switch[0], Switch[1], Switch3Reversed);
    if (swt == 4) rtv = ReadCHSwitch(Switch[2], Switch[3], Switch4Reversed);
    return rtv;
}
/************************************************************************************************************/

void GetFlightMode()
{ //  and AUTO and other switchy things ...

    if (FMSwitch == 4) ReadFMSwitch(Switch[2], Switch[3], Switch4Reversed);
    if (FMSwitch == 3) ReadFMSwitch(Switch[0], Switch[1], Switch3Reversed);
    if (FMSwitch == 2) ReadFMSwitch(Switch[4], Switch[5], Switch2Reversed);
    if (FMSwitch == 1) ReadFMSwitch(Switch[6], Switch[7], Switch1Reversed);
    if (AutoSwitch == 1 && Switch[6] == Switch1Reversed) FlightMode = 4; // Flight mode 4 (Auto) overrides modes 1,2,3.
    if (AutoSwitch == 2 && Switch[4] == Switch2Reversed) FlightMode = 4;
    if (AutoSwitch == 3 && Switch[1] == Switch3Reversed) FlightMode = 4;
    if (AutoSwitch == 4 && Switch[3] == Switch4Reversed) FlightMode = 4;

    Channel9SwitchValue  = CheckSwitch(Channel9Switch);
    Channel10SwitchValue = CheckSwitch(Channel10Switch);
    Channel11SwitchValue = CheckSwitch(Channel11Switch);
    Channel12SwitchValue = CheckSwitch(Channel12Switch);

    if (FlightMode != PreviousFlightMode) {
        SendCommand(NextionWakeUp);    // wake screen up if flight mode changes
        LastSeconds = 0;               // Just to force redisplay of timer
        if (PreviousFlightMode == 4) { // Start or restart timer when auto goes off
            TimerMillis = millis();
        }

        if (FlightMode == 4) {                                // Pause timer when auto on
            PausedSecs = Secs + (Mins * 60) + (Hours * 3600); // Remember how long so far
        }

        CheckTimer(); // update timer

        if (CurrentView == FrontView) {
            ShowFlightMode();
        }
        UpdateModelsNameEveryWhere();
        if (CurrentView == GraphView) DisplayCurve();
    }
    PreviousFlightMode = FlightMode;
}

/************************************************************************************************************/

void ReadSwitches()
{
    for (i = 0; i < 8; ++i) {
        if (!digitalRead(SwitchNumber[i])) {
            Switch[i] = true;
        }
        else {
            Switch[i] = false;
        }
#ifdef DB_SWITCHES
        Serial.print("Switch ");
        Serial.print(i);
        Serial.print(" = ");
        Serial.println(Switch[i]);
#endif
    }
    GetFlightMode();
}

/************************************************************************************************************/

void GetRXVersionNumber()
{
    char nbuf[5];
    Str(nbuf,AckPayload.Byte1, 0);
    strcpy(ThisRadio, nbuf);
    Str(ReceiverVersionNumber, AckPayload.Byte2, 2);
    Str(nbuf, AckPayload.Byte3, 2);
    strcat(ReceiverVersionNumber, nbuf);
    Str(nbuf, AckPayload.Byte4, 0);
    strcat(ReceiverVersionNumber, nbuf);
}

/************************************************************************************************************/

float GetFromAckPayload(){
    union  {float Val32;uint8_t Val8[4];} ThisUnion;
    ThisUnion.Val8[0] = AckPayload.Byte1;
    ThisUnion.Val8[1] = AckPayload.Byte2;
    ThisUnion.Val8[2] = AckPayload.Byte3;
    ThisUnion.Val8[3] = AckPayload.Byte4;
    return ThisUnion.Val32;
}
/************************************************************************************************************/
void GetTimeFromAckPayload(){
    GPSSecs  = AckPayload.Byte1;    
    GPSMins  = AckPayload.Byte2;   
    GPSHours = AckPayload.Byte3;
}
/************************************************************************************************************/
void GetDateFromAckPayload(){
    GPSDay    = AckPayload.Byte1;    
    GPSMonth  = AckPayload.Byte2;   
    GPSYear   = AckPayload.Byte3;
}
/************************************************************************************************************/
void GetAltitude()
{
    RXModelAltitude = int(GetFromAckPayload()) - GroundModelAltitude;
    if (RXMAXModelAltitude < RXModelAltitude) RXMAXModelAltitude = RXModelAltitude;
    snprintf(MaxAltitude, 5, "%d", RXMAXModelAltitude);
    snprintf(ModelAltitude, 5, "%d", RXModelAltitude);
}
/************************************************************************************************************/
void GetTemperature()
{
    RXModelTemperature       = GetFromAckPayload();
    snprintf(ModelTemperature, 5, "%f", RXModelTemperature);
}
/************************************************************************************************************/
void ParseAckPayload()
{
    if (AckPayload.Purpose & 0x80)                                       // Hi bit is now the **HOP NOW!!** flag
    {
        NextChannelNumber   =  AckPayload.Byte5;                         // This is just the array pointer or offset  
        NextChannel       =  * (FHSSChPointer + NextChannelNumber);      // The actual channel number pointed to.
        HopToNextChannel();
        AckPayload.Purpose &= 0x7f;
    }    
        switch (AckPayload.Purpose) // Only look at the low 7 BITS
        {
            case 0:
                GetRXVersionNumber();
                break;
            case 1:
                RXModelVolts = GetFromAckPayload();
                if (RXModelVolts > 0) {
                    VoltsDetected = true;
                    snprintf(ModelVolts, 5, "%f", RXModelVolts);
                }
                break;
            case 2:
                GetAltitude();
                break;
            case 3:
                GetTemperature();
                break;
            case 4:
                GPSLatitude = GetFromAckPayload(); 
                break;
            case 5:
                GPSLongitude = GetFromAckPayload(); 
                break;
            case 6:
                GPSAngle     = GetFromAckPayload();
                break;
            case 7:
                GPSSpeed     = GetFromAckPayload(); 
                if (GPSMaxSpeed < GPSSpeed) GPSMaxSpeed = GPSSpeed;
                break;
            case 8:
                GpsFix       =  GetFromAckPayload();
                break;
            case 9:
                GPSAltitude  = GetFromAckPayload() - GPSGroundAltitude;
                if (GPSAltitude < 0) GPSAltitude = 0;
                if (GPSMaxAltitude < GPSAltitude) GPSMaxAltitude = GPSAltitude;
                break;
            case 10:
                 GPSDistanceTo = GetFromAckPayload();
                 if (GPSMaxDistance < GPSDistanceTo) GPSMaxDistance = GPSDistanceTo;
                 break;
            case 11:
                 GPSCourseTo = GetFromAckPayload();  
                 break;
            case 12:
                 GPSSatellites = (uint8_t) GetFromAckPayload();
                 break;
            case 13:
                 GetDateFromAckPayload();
                 break;
            case 14:
                 GetTimeFromAckPayload();
                 ReadTheRTC();
                 if (GPSDay   != GmonthDay) GPSTimeSynched = false;
                 if (GPSMonth != GPSMonth)  GPSTimeSynched = false;
                 if (GPSMins  != Gminute)   GPSTimeSynched = false;
                 if (GPSHours != Ghour)     GPSTimeSynched = false;
                 if (GPSSecs  != Gsecond)   GPSTimeSynched = false;
                 if (GpsFix)  SynchRTCwithGPSTime();
                 break;
            default:
                break;
    }
}
/************************************************************************************************************/
void CheckGapsLength()
{
    if (GapStart > 0) { // when reconnected, how long was connection lost?
        ++GapCount;
        ThisGap = (millis() - GapStart); // AND in fact RX sends no data for 20 ms after reconnection
        if (!GapShortest) GapShortest = ThisGap;
        if (ThisGap > GapLongest) {
            GapLongest = ThisGap;
        }

        if (ThisGap < GapShortest) {
            GapShortest = ThisGap;
        }
        GapSum += ThisGap;
        GapStart   = 0;
        GapAverage = GapSum / GapCount;
#ifdef DB_GAPS
        Serial.print("GapCount: ");
        Serial.println(GapCount);
        Serial.print("GapAverage: ");
        Serial.println(GapAverage);
        Serial.print("GapShortest: ");
        Serial.println(GapShortest);
        Serial.print("GapLongest: ");
        Serial.println(GapLongest);
        Serial.println(" ");
#endif
    }
}
/************************************************************************************************************/
void CheckModelName(){                        // In ModelsView, this function checks correct name is displayed.
char ModelsView_ModelNumber[]  = "ModelNumber"; 
    if ((millis()-ModelNameTimeCheck) > 500) {  
        ModelNameTimeCheck  = millis();
        if (!InhibitNameCheck){               // if name is being edited, do not check it.
            ModelNumber = GetValue(ModelsView_ModelNumber);
            if (LastModelLoaded != ModelNumber) {
                if (ModelNumber >= 1) {      // Don't use number zero
                     ReadOneModel(ModelNumber);
                     LastModelLoaded = ModelNumber;
                     UpdateModelsNameEveryWhere();  
                }
            }
        }
    }
}
/************************************************************************************************************/
// LOOP
/************************************************************************************************************/
void loop()
{
    KickTheDog();                    // Watchdog
    if (GetButtonPress()) {
        Button_was_pressed();        // Deal with button
    }
    if (CurrentView == ModelsView){ 
        CheckModelName();            // In ModelsView, this function checks correct name is displayed.
    }
    if (millis() - LastTimeRead >= 1000) {
        ReadTime();                  // Do the clock
        LastTimeRead = millis();
    }
    if (millis() - RangeTestStart >= 1000) {
        GetStatistics();             // Do stats
        RangeTestStart = millis();
    }
    if ((millis() - ShowServoTimer >= 100) && (CurrentView != FrontView)) {
        ShowServoPos();              // Show servos positions
        ShowServoTimer = millis();
    }
    if ((millis() - TxOnTime) > 2000) { // Transmit nothing for first 2 seconds

        switch (CurrentMode) {
            case 0:
                SendData();
                if (!LedWasGreen) ShowComms();   // Show when not connected
                break;
            case 1:
                CalibrateSticks();
                break;
            case 2:
                ChannelCentres();
                break;
            case 3:
                ScanAllChannels();
                break;
            default:
                break; // CurrentMode >= 4 for no action at all.
        }
    }



    if (BindingNow == 2 && (millis() - BindingTimer) > 100) {
        if (!BoundFlag) {
#ifdef DB_BIND
            Serial.println("Binding now");
#endif
            SetThePipe(NewPipe);
            BindingNow  = 0;
            BoundFlag   = true;
            LostPackets = 0;
            GapShortest = 0;
            GapLongest  = 0;
            GapSum      = 0;
            GapAverage  = 0;
            GapCount    = 0;
            GreenLedOn();
            MakeBindButtonInvisible();
        }
    }
} // end loop()
