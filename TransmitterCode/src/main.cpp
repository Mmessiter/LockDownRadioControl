/** @file TransmitterCode/src/main.cpp
 * // Malcolm Messiter 2022
 *
 * @page TransmitterCode
 * @section LockDown Radio Control Features list, so far:
 * - Uses Teensy 4.1 MCU (at 600 Mhz) with nRF24L01+ transceiver
 * - (Ebyte's ML01DP5 recommended for TX, two ML01SP4s for RX.)
 * - 16 channels
 * - 12 BIT servo resolution (11 BIT via SBUS)
 * - 32 Mixes
 * - 4 Flight modes (AKA Banks), or 3 plus autorotation
 * - User defined Channel names
 * - 2.4 Ghz FHSS ISM band licence free in UK and most other countries.
 * - 2.5 Km range (approx.)
 * - Telemetry including GPS, volts, temperature & barometric pressure, using custom I2C sensor hub
 * - 64 editable 5-point curves (16 channels x 4 flight modes): straight, smoothed or exponential.
 * - FailSafe on any or all channel(s)
 * - 2.4 GHz RF scan
 * - Motor Timer
 * - Lossless data compression.
 * - Trims saved per bank and per model.
 * - Screen timeout to save battery.
 * - FHSS with very fast connect and reconnect
 * - Uses 32 GIG SD card for model memories and help files (TODO: LOG FILES)
 * - Binding without bind plug - uses unique Mac address as pipe address.
 * - Four User definable three position switches
 * - Channels 5,6,7 & 8 can be switches or knobs.
 * - Input sources definable - any stick, switch or knob can be mapped to any function.
 * - Model memories export and import to backup files on SD card
 * - Model memory files alphabetically sorted
 * - Timer goes on and off with motor to keep track of motor use.
 * - DS1307 Real time clock auto synched to exact GPS time.
 * - Model memory sharing between transmitters - wirelessly.
 * - Sub-trim
 * - Servo reverse
 * - Macros - for snap rolls, heli rescue, etc.
 * - Hardware digital trims with accellerating repeat
 * - Capacitive touch screen GUI
 * - Hardware bug in nRF24L01+ fixed. (FIFO buffers crash the chip when they're full for > 4ms. So these are not used.)
 * - Screen colours definable
 * - Data screen gives all possible telemetry  
 * - Log files implemented - and help file system with unlimited file length
 *  
 *
 * @section txPinout Teensy 4.1 Pins
 * | Teensy 4.1 Pins | Connections |
 * |-----------------|-------------|
 * | GND        | GND |
 * | Vin        | + 5.0 VDC |
 * | 0  (RX1)   | NEXTION  (TX) |
 * | 1  (TX2)   | NEXTION  (RX) |
 * | 2  LED     | RED |
 * | 3  LED     | GREEN |
 * | 4  LED     | BLUE |
 * | 5  POLOLU  | 2808 ALL POWER OFF SIGNAL (When high) |
 * | 6  (!! SPARE !!)
 * | 7  (RX2)   | SBUS IN    ------> BUDDY BOX SYSTEM |
 * | 8  (TX2)   | SBUS OUT   ------> BUDDY BOX SYSTEM |
 * | 9  (CE)    | nRF24l01 (CE) |
 * | 10 (CS)    | nRF24l01 (CSN) |
 * | 11 (MOSI)  | nRF24l01 (MOSI) |
 * | 12 (MISO)  | nRF24l01 (MISO) |
 * | 13 (SCK)   | nRF24l01 (SCK) |
 * | 14 (A0)    | Joystick POT CH1 |
 * | 15 (A1)    | Joystick POT CH2 |
 * | 16 (A2)    | Joystick POT CH3 |
 * | 17 (A3)    | Joystick POT CH4 |
 * | 18         | I2C bus  SDA |
 * | 19         | I2C bus  SCL |
 * | 20 (A6)    | POT KNOB CH5 |
 * | 21 (A7)    | POT KNOB CH6 |
 * | 22 (A8)    | POT KNOB CH7 |
 * | 23 (A9)    | POT KNOB CH8 |
 * | 24 (!! SPARE !!)
 * | 25         | Switch 1 |
 * | 26         | Switch 1 |
 * | 27         | Switch 2 |
 * | 28         | Switch 2 |
 * | 29         | Switch 3 |
 * | 30         | Switch 3 |
 * | 31         | Switch 4 |
 * | 32         | Switch 4 |
 * | 33 (!! SPARE !!)
 * | 34         |TRIM (CH1a)|
 * | 35         |TRIM (CH1b)|
 * | 36         |TRIM (CH2a)|
 * | 37         |TRIM (CH2b)|
 * | 38         |TRIM (CH3a)|
 * | 39         |TRIM (CH3b)|
 * | 40         |TRIM (CH4a)|
 * | 41         |TRIM (CH4b)|
 * | 53 NOT USED
 * @see TransmitterCode/src/main.cpp
 */
// ************************************************** TRANSMITTER CODE **************************************************

#include "Hardware/RadioFunctions.h"                           // This file contains many definitions and further includes

RF24 Radio1(CE_PIN, CSN_PIN);
WDT_T4<WDT3>  TeensyWatchDog;
WDT_timings_t WatchDogConfig;
SBUS     MySbus(SBUSPORT);
uint16_t SbusChannels[CHANNELSUSED + 2]; // a few spare
uint32_t SBUSTimer = 0;
uint8_t Mixes[MAXMIXES + 1][CHANNELSUSED + 1];                // Channel mixes' 2D array store
int     Trims[FLIGHTMODESUSED + 1][CHANNELSUSED + 1];         // Trims to store
uint8_t TrimsReversed[FLIGHTMODESUSED + 1][CHANNELSUSED + 1]; // Trim directions to store
uint8_t Exponential[FLIGHTMODESUSED + 1][CHANNELSUSED + 1];   // Exponential
uint8_t InterpolationTypes[FLIGHTMODESUSED + 1][CHANNELSUSED + 1];

uint8_t       LastMixNumber      = 1;
uint8_t       MixNumber          = 0;
uint8_t       CurrentView        = FRONTVIEW;
uint8_t       SavedCurrentView   = FRONTVIEW;
uint64_t      DefaultPipe        = DEFAULTPIPEADDRESS; //          Default Radio pipe address
uint64_t      NewPipe            = 0xBABE1E5420LL;     //          New Radio pipe address for binding comes from MAC address
char          TextIn[CHARSMAX+2];  // spare space
uint16_t      PacketsPerSecond = 0;
uint16_t      LostPackets      = 0;
uint8_t       PacketNumber     = 0;
uint8_t       GPSMarkHere      = 0;
uint8_t       PreviousTrim     = 255;   
uint32_t      TrimTimer        = 0;  
uint16_t      TrimRepeatSpeed  = 600;   
uint16_t      DefaultTrimRepeatSpeed  = 600;   

// ************************************* AckPayload structure ******************************************************
/**
     * This first byte "Purpose" defines what all the other bytes mean, AND ...
     * the highest BIT of Purpose means ** HOP TO NEXT CHANNEL A.S.A.P. (IF ON) **
     * the lower 7 BITs then define the meaning of the remainder of the ackpayload bytes
     * If Purpose == 1 then ...
     * 
     * AckPayload.Byte2           =  ThisRadio;              // RX Transceiver in current use 
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

uint8_t AckPayloadSize = sizeof(AckPayload);   // i.e. 6

// *****************************************************************************************************************

uint16_t SendBuffer[UNCOMPRESSEDWORDS];      //    Data to send to rx (16 words)
uint16_t ShownBuffer[UNCOMPRESSEDWORDS];     //    Data shown before
uint16_t LastBuffer[CHANNELSUSED + 1];       //    Used to spot any change
uint16_t PreMixBuffer[CHANNELSUSED + 1];     //    Data collected from sticks
uint8_t  MaxDegrees[5][CHANNELSUSED + 1];    //    Max degrees (180)
uint8_t  MidHiDegrees[5][CHANNELSUSED + 1];  //    MidHi degrees (135)
uint8_t  CentreDegrees[5][CHANNELSUSED + 1]; //    Middle degrees (90)
uint8_t  MidLowDegrees[5][CHANNELSUSED + 1]; //    MidLow Degrees (45)
uint8_t  MinDegrees[5][CHANNELSUSED + 1];    //    Min Degrees (0)
uint8_t  SubTrims[CHANNELSUSED + 1];         //    Subtrims
uint8_t  SubTrimToEdit = 0;
uint8_t  FlightMode         = 1;
uint8_t  PreviousFlightMode = 1;
uint16_t ChannelMax[CHANNELSUSED + 1];       //    output of pots at max
uint16_t ChannelMidHi[CHANNELSUSED + 1];     //    output of pots at MidHi
uint16_t ChannelCentre[CHANNELSUSED + 1];    //    output of pots at Centre
uint16_t ChannelMidLow[CHANNELSUSED + 1];    //    output of pots at MidLow
uint16_t ChannelMin[CHANNELSUSED + 1];       //    output of pots at min
uint16_t ChanneltoSet     = 0;
bool     Connected        = false;
uint8_t  ShowCommsCounter = 0;

double PointsCount = 5; // This for displaying curves only
double xPoints[5];
double yPoints[5];
double xPoint = 0;
double yPoint = 0;

uint16_t      BoxBottom;
uint16_t      BoxTop;
uint16_t      BoxLeft;
uint16_t      BoxRight;
uint16_t      ClickX;
uint16_t      ClickY;
bool          CalibratedYet                = false;
uint16_t      AnalogueInput[PROPOCHANNELS] = {A0, A1, A2, A3, A6, A7, A8, A9}; // PROPO Channels for transmission
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
uint8_t       ModelNumber   = 1;
uint8_t       ModelDefined  = 0;
uint16_t      MemoryForTransmtter  = 0;   // SD space for transmitter parameters
uint16_t      OneModelMemory       = 0;   // SD space for every model's parameters
uint16_t      SDCardAddress        = 0;   // Address on SD card (offset from zero)

                                                                      

uint8_t FHSS_Channels1[42] = {93,111,107,103,106,97,108,102,118,                                        // TEST array 
104,101,109,98,113,124,115,91,96,85,117,89,99,114,87,112,
86,94,92,119,120,100,121,123,95,122,105,84,116,90,110,88};

uint8_t FHSS_Channels[83] = {51,28,24,61,64,55,66,19,76,21,59,67,15,71,82,32,49,69,13,2,34,47,20,16,72,  // UK array
35,57,45,29,75,3,41,62,11,9,77,37,8,31,36,18,17,50,78,73,30,79,6,23,40,
54,12,80,53,22,1,74,39,58,63,70,52,42,25,43,26,14,38,48,68,33,27,60,44,46,
56,7,81,5,65,4,10}; /// test comment 

uint8_t * FHSSChPointer;                                                                                 // pointer for channels array (three only used for reconnect)

char        page_FrontView[]            = "page FrontView";
char        page_FhssView[]             = "page FhssView";
char        FrontView_Hours[]           = "Hours";
char        FrontView_Mins[]            = "Mins";
char        FrontView_Secs[]            = "Secs";
char        StartBackGround[]           = "click Background,0";
char        ModelsFile[]                = "models.dat";
uint8_t     SwitchNumber[8]             = {SWITCH0, SWITCH1, SWITCH2, SWITCH3, SWITCH4, SWITCH5, SWITCH6, SWITCH7};
uint8_t     TrimNumber[8]               = {TRIM1A, TRIM1B, TRIM2A, TRIM2B, TRIM3A, TRIM3B, TRIM4A, TRIM4B};
const int   chipSelect                  = BUILTIN_SDCARD;
char        DateTime[]                  = "DateTime";
char        ScreenViewTimeout[]         = "Sto";                  // needed for display info
char        NoSleeping[]                = "thsp=0";
char     ModelName[30];
uint16_t ScreenTimeout               = 120;                     // Screen has two minute timeout by default
int      LastLinePosition            = 0;
uint8_t  RXCellCount                 = 2;
bool     JustHoppedFlag              = true;
bool     LostContactFlag             = true;
uint8_t  RecentPacketsLost           = 0;
uint32_t TotalledRecentPacketsLost   = 0;
uint32_t GapSum                      = 0;
uint32_t GapLongest                  = 0;
uint32_t GapStart                    = 0;
uint32_t ThisGap                     = 0;
uint32_t GapAverage                  = 0;
uint32_t GapCount                    = 0;
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
File     ModelsFileNumber;

Adafruit_INA219 ina219;

char      SingleModelFile[80];
bool      SingleModelFlag = false;

bool      ModelsFileOpen  = false;
bool      USE_INA219      = false;
uint8_t   BindingNow      = 0;
int       BindingTimer    = 0;
bool      BoundFlag       = false;
int       PipeTimeout     = 0;
bool      Switch[8];
bool      TrimSwitch[8];


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
bool SWITCH1Reversed = false;
bool SWITCH2Reversed = false;
bool SWITCH3Reversed = false;
bool SWITCH4Reversed = false;
uint16_t StartLocation       = 0;
bool     ValueSent           = false;
uint8_t  SwitchEditNumber    = 0; // number of switch being edited
uint32_t ShowServoTimer      = 0;
bool     LastFourOnly        = false;
uint8_t  InPutStick[17]      = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}; //
uint8_t  ExportedFileCounter = 0;
char     TheFilesList[100][14];
uint16_t FileNumberInView     = 0;
bool     FileError            = false;
uint32_t RangeTestStart       = 0;
uint16_t RangeTestGoodPackets = 0;
uint16_t RangeTestLostPackets = 0;
uint8_t  SaveFlightMode       = 0;
bool     FailSafeChannel[CHANNELSUSED];
bool     SaveFailSafeNow                = false;
uint32_t FailSafeTimer;
char     ChannelNames[CHANNELSUSED][11] = {{"Aileron"}, {"Elevator"}, {"Throttle"}, {"Rudder"}, {"Gear"}, {"AUX1"}, {"AUX2"}, {"AUX3"}, {"AUX4"}, {"AUX5"}, {"AUX6"}, {"AUX7"}, {"AUX8"}, {"AUX9"}, {"AUX10"}, {"AUX11"}};

uint32_t TxOnTime      = 0;
uint32_t TxPace        = 0;
uint16_t CompressedData[COMPRESSEDWORDS]; // = 20
uint8_t  SizeOfCompressedData;
uint32_t Inactivity_Timeout = INACTIVITYTIMEOUT ;
uint32_t Inactivity_Start   = 0;

tmElements_t tm;
char         TxName[32]   = {"No name was found!"};
uint32_t     LastTimeRead = 0;
uint32_t     LastShowTime = 0;
uint32_t     LastDogKick  = 0;
uint8_t      MacAddress[6];


uint16_t     XtouchPlace = 0; // Clicked X
uint16_t     YtouchPlace = 0; // Clicked Y

bool     BindButton  = false;

uint8_t  PreviousChannelNumber = 0;
uint8_t  NextChannelNumber     = 0;
bool     InhibitNameCheck      = false;

// changing these four valiables controls LED blink and speed

bool     LedIsBlinking = false;
float    BlinkHertz    = 2;
uint32_t BlinkTimer    = 0;
uint8_t  BlinkOnPhase  = 1;
bool     LedWasGreen   = false;
char     ThisRadio[4]  = "0 ";
uint8_t  LastRadio     = 0;
uint8_t  NextChannel   = 0;
bool     DoSbusSendOnly  = false;
bool     BuddyMaster     = false;
uint8_t  BuddyTriggerChannel = 12;
bool     SlaveHasControl = false;
uint16_t Qnh            = 1009;               // pressure at sea level here
uint32_t ModelNameTimeCheck = 0;
uint16_t LastModelLoaded    = 0;
uint8_t  MinimumGap = 75;
uint8_t  RecentStartLine = 0;
char     RecentTextFile[20];
bool     LogRXSwaps =  false;
bool     ThereIsMoreToSee = false;
bool     UseLog = false;


uint8_t  Gsecond;  // = tm.Second; // 0-59
uint8_t  Gminute;  // = tm.Minute; // 0-59
uint8_t  Ghour;    // = tm.Hour;   // 0-23
uint8_t  GweekDay; // = tm.Wday;   // 1-7
uint8_t  GmonthDay;// = tm.Day;    // 1-31
uint8_t  Gmonth;   // = tm.Month;  // 1-12
uint8_t  Gyear;    // = tm.Year;   // 0-99
bool     GPSTimeSynched  =   false;
short int      DeltaGMT           = 0;
uint32_t SwapWaveBandTimer  = 0;
uint8_t  UkRulesCounter     = 0;
bool     UkRules            = true;
bool     PreviousUkRules    = false;
uint8_t  SwapWaveBand       = 0;  
uint16_t TrimFactor         = 2;   // How much to multiply trim by
uint8_t  DateFix            = 0;
bool     b5isGrey           = false;
uint16_t BackGroundColour   = 214;
uint16_t ForeGroundColour   = 65535;
uint16_t HighlightColour    = Yellow;
uint16_t SpecialColour      = Red;
bool     Reconnected        = false;
uint8_t  LowBattery         = LOWBATTERY;
uint16_t SbusRepeats        = 0;
uint16_t SavedSbusRepeats   = 0;
bool     RXVoltsDetected      = false;
uint8_t  SticksMode         = 2;
uint16_t RadioSwaps         = 0 ;
uint16_t RX1TotalTime       = 0 ;
uint16_t RX2TotalTime       = 0 ;
uint16_t SavedRadioSwaps    = 0;
uint16_t SavedRX1TotalTime  = 0;
uint16_t SavedRX2TotalTime  = 0;
uint8_t  AudioVolume        = 90;
uint32_t WarningTimer       = 0;
uint32_t ScreenTimeTimer    = 0;
bool     ScreenIsOff        = false;
uint8_t  Brightness         =  100;
bool     ButtonClicks       = true;
bool     PlayFanfare        = true;
bool     TrimClicks         = true;
bool     SpeakingClock      = true;
bool     ClockSpoken        = false;
bool     AnnounceBanks      = true;
bool     AnnounceConnected  = true;
bool     CopyTrimsToAll     = true;

uint8_t  MacrosBuffer[MAXMACROS][BYTESPERMACRO];        // macros' buffer
uint32_t MacroStartTime[MAXMACROS];
uint32_t MacroStopTime[MAXMACROS];
uint8_t  PreviousMacroNumber = 1;
bool     UseMacros = false;
uint16_t ReversedChannelBITS = 0; // 16 BIT for 16 Channels
uint16_t SavedLineX          = 12345;
bool     FirstConnection     = true;
File     LogFileNumber;
bool     LogFileOpen         =  false;
//
// ********************************************************************************************************************************** 

uint8_t Ascii(char c){
 return (uint8_t) c;
}
// **************************************************************** Play a sound from RAM *********************************************
void PlaySound(uint16_t TheSound){    // Plays a sound identified by a number

char Sound[20];
char SoundPrefix[] = "play 0,";
char SoundPostfix[] = "0";
char NB[6];
    Str(NB,TheSound,1);
    strcpy (Sound,SoundPrefix);
    strcat (Sound,NB);
    strcat (Sound,SoundPostfix);
    SendCommand(Sound);
}
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
// This function reads data from BUDDY (Slave) BUT uses it ONLY WHILE the channel BUDDTRIGGERCHANNEL switch is in the ON position ( > 1000)

void GetSlaveChannelValues()
{
    bool failSafeM; // These flags not used, yet.
    bool lostFrameM;
    SlaveHasControl = false;
    if (SendBuffer[BuddyTriggerChannel-1] > 1000)
    { // MASTER'S CHANNEL 'BuddyTriggerChannel' (500 - 2500) used here as switch.
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
/**************************** Clear Macros if junk was loaded from SD ********************************************************************************/
void CheckMacrosBuffer(){

bool junk = false;
for (uint8_t i = 0; i < MAXMACROS; ++i){
                if (MacrosBuffer[i][MACROTRIGGERCHANNEL] > 16)   junk = true;
                if (MacrosBuffer[i][MACROMOVECHANNEL] > 16)      junk = true; 
                if (MacrosBuffer[i][MACROMOVETOPOSITION] > 180)  junk = true; 
                if (MacrosBuffer[i][MACROTRIGGERCHANNEL] > 0)    UseMacros = true;
}
 if (junk == false) return;
 UseMacros = false;
 for (uint8_t j = 0; j < BYTESPERMACRO; ++j){
    for (uint8_t i = 0; i < MAXMACROS; ++i){
                MacrosBuffer[i][j] = 0;
    } 
  }
}
/************************************************************************************************************/
FLASHMEM void ResetSubTrims(){
    for (int i = 0;i<16;++i){
            SubTrims[i]= 127;
    }
}
/************************************************************************************************************/
/** Map servo channels' data from SendBuffer into SbusChannels buffer */
// This funtion is used by the BUDDY slave to send it's controls out down a wire using SBUS

FASTRUN void MapToSBUS()
{
    if (millis() - SBUSTimer >= SBUSRATE)
    {
        SBUSTimer = millis();
        for (int j = 0; j < CHANNELSUSED; ++j)
        {
            SbusChannels[j] = static_cast<uint16_t> (map(SendBuffer[j], MINMICROS, MAXMICROS, RANGEMIN, RANGEMAX)); 
        }
        MySbus.write(SbusChannels);        
    }
}
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

FLASHMEM void SetTheRTC(){
    uint8_t  zero        = 0x00;
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
        Gyear     = GPSYear + 1744;     // ????
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
      uint8_t  zero = 0x00;
      uint8_t  c    = 1;
    if (RTC.read(tm)) {
        AdjustDateTime(c, zero, zero, zero, zero);
    }
}

/*********************************************************************************************************************************/

void DecMinute()
{
     uint8_t  zero = 0x00;
     uint8_t c = -1;
    if (RTC.read(tm)) {
        AdjustDateTime(c, zero, zero, zero, zero);
    }
}

/*********************************************************************************************************************************/

void IncHour()
{
    uint8_t c = 1;
    uint8_t  zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, c, zero, zero, zero);
    }
}

/*********************************************************************************************************************************/

void DecHour()
{
    uint8_t c = -1;
    uint8_t  zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, c, zero, zero, zero);
    }
}

/*********************************************************************************************************************************/

void IncYear()
{
    uint8_t c = 1;
    uint8_t  zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, c, zero, zero);
    }
}

/*********************************************************************************************************************************/

void DecYear()
{
    uint8_t c = -1;
    uint8_t  zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, c, zero, zero);
    }
}

/*********************************************************************************************************************************/

void IncMonth()
{
    uint8_t c = 1;
    uint8_t  zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, zero, c, zero);
    }
}

/*********************************************************************************************************************************/

void DecMonth()
{
    uint8_t c = -1;
    uint8_t  zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, zero, c, zero);
    }
}

/*********************************************************************************************************************************/

void IncDate()
{
    uint8_t c = 1;
    uint8_t  zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, zero, zero, c);
    }
}

/*********************************************************************************************************************************/

void DecDate()
{
    uint8_t c = -1;
    uint8_t  zero = 0x00;
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

    if (millis() - LastDogKick >= KICKRATE) {
        LastDogKick = millis();
        TeensyWatchDog.feed();
    }

}

/*********************************************************************************************************************************/

void Reboot()
{

    for (int i = 0; i < 30; ++i) {
        TeensyWatchDog.feed();
    } // Dog will explode when overfed

}

/*********************************************************************************************************************************/

void SetAudioVolume(uint16_t v){   // sets audio volume v (0-100)
    char vol[]="volume=";
    char cmd[20];
    char nb[6];
    strcpy(cmd,vol);
    Str(nb,v,0);
    strcat (cmd,nb);
    SendCommand(cmd);
}

/*********************************************************************************************************************************/

// This function converts an int to a char[] array, then adds a comma, a dot, or nothing at the end.
// It builds the char[] array at a pointer (*s) Where there MUST be enough space for all characters plus a zero terminator.
// It dates for a very early time when I didn't know about standard library functions! 
// But it works just fine, so it says in.

FASTRUN char* Str(char* s, int n, int comma) // comma = 0 for nothing, 1 for a comma, 2 for a dot.
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
    uint8_t DisplayedHour;
    FixDeltaGMTSign();  
    if (CurrentView == FRONTVIEW || CurrentView == OPTIONS_VIEW) {
        if (RTC.read(tm)) { 
            strcpy(TimeString, Str(NB, tm.Day + DateFix, 0));
            if (CurrentView == OPTIONS_VIEW)
            {
                if ((tm.Day) < 10) {
                    strcat(TimeString, Space); // to align better the rest of the data
                    strcat(TimeString, Space);
                }
            }
            strcat(TimeString, Space);
            if (CurrentView == OPTIONS_VIEW)
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
            DisplayedHour = tm.Hour+DeltaGMT;
            DateFix = 0;
            if (DisplayedHour > 24) {
                DisplayedHour -= 24;
                DateFix = 1;
                }
            if (DisplayedHour < 0) {
                DisplayedHour += 24;
                DateFix = -1;
            }
            if (MayBeAddZero(DisplayedHour)) strcat(TimeString, zero);
            strcat(TimeString, Str(NB, DisplayedHour, 0));
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
            UpdateModelsNameEveryWhere();
            if (CurrentView == FRONTVIEW) ShowFlightMode();
        }
    }
}

/*********************************************************************************************************************************/

void StartInactvityTimeout()
{
    Inactivity_Start = millis();
}

/*********************************************************************************************************************************/

void MakeBindButtonInvisible()
{
  if (CurrentView == FRONTVIEW){
    char bbiv[] = "vis bind,0";
    if (BindButton) {
        SendCommand(bbiv);
        BindButton = false;
    }
   }
}

/*********************************************************************************************************************************/

uint8_t GetLEDBrightness()
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
        return 150; // 0 - 254
    }
    else {
        return 0;
    }
}

/*********************************************************************************************************************************/

void RedLedOn()
{
    if (LedWasGreen){
        if (UseLog) LogDisConnection();
        if (AnnounceConnected) PlaySound(DISCONNECTEDMSG);
        RXVoltsDetected = false;
        LedWasGreen = false;
    }
    
    FirstConnection = true;
    analogWrite(GREENLED, 0);
    analogWrite(BLUELED, 0);
    analogWrite(REDLED, GetLEDBrightness());     // Brightness is a function of maybe blinking
}

/*********************************************************************************************************************************/

void GreenLedOn()
{
    if (!LedWasGreen || LedIsBlinking) {         // no need to repeat unless it is blinking
        LedWasGreen = true;
        if (FirstConnection) {                   // Zero data on first connection
            ZeroDataScreen(); 
            FirstConnection = false; 
            if (AnnounceConnected) PlaySound(CONNECTEDMSG);
            if (UseLog){ 
                LogConnection();
            }
        }
        analogWrite(BLUELED, 0);
        analogWrite(REDLED, 0); 
        analogWrite(GREENLED, GetLEDBrightness()); // Brightness is a function of maybe blinking
        MakeBindButtonInvisible();
        Reconnected=false;
    }
}

/*********************************************************************************************************************************/

void BlueLedOn()
{
    LedWasGreen = false;
    analogWrite(REDLED, 0);
    analogWrite(GREENLED, 0);
    analogWrite(BLUELED, GetLEDBrightness()); // Brightness is a function of maybe blinking
}

/*********************************************************************************************************************************/
uint8_t IntoLowerRes(uint16_t HiRes)                        // convert to lower resolution for screen display
{
    return (map(HiRes, MINMICROS, MAXMICROS, 0, 100));      
}
/*********************************************************************************************************************************/
uint16_t IntoHigherRes(uint8_t LowRes)                      // This returns the main curve-points at the higher resolution for Servo output
{
    return map(LowRes, 0, 180, MINMICROS, MAXMICROS);
} 
/*********************************************************************************************************************************/
void ClearText()
{
    for (int i = 0; i < CHARSMAX; ++i) {
        TextIn[i] = 0;
    }
}
/*********************************************************************************************************************************/
//                        NEXTION functions
/*********************************************************************************************************************************/
void GetReturnCode(){  // currently absorbed but ignored.
    while (NEXTION.available()){
            NEXTION.read();
     }
}
/*********************************************************************************************************************************/
void SendCommand(char* tbox)
{
    NEXTION.print(tbox);
    for (int i = 0; i < 3; ++i) {
        NEXTION.write(0xff);
    } 
    GetReturnCode();
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
    GetReturnCode();
}
/*********************************************************************************************************************************/
void SendText1(char* tbox, char* NewWord)
{
    char txt[]   = ".txt=\"";
    char quote[] = "\"";
    char CB[MAXFILELEN+10];
    char TooLong[] = "Too long!";

    if (strlen(NewWord) > MAXFILELEN) {
        strcpy(NewWord, TooLong);
    }
    strcpy(CB, tbox);
    strcat(CB, txt);
    strcat(CB, NewWord);
    strcat(CB, quote);
    SendCommand(CB);
    GetReturnCode();
}
/*********************************************************************************************************************************/
void EndSend()
{
    for (u_int8_t pp = 0; pp < 3; ++pp) {NEXTION.write(0xff);} // Send end of Input message // 
    delay(55);                                            // ** A DELAY ** (>=50 ms) was needed if an answer might come! (!! Shorter with Intelligent dislay)
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
    GetReturnCode();
}
/*********************************************************************************************************************************/
void SendOtherValue(char* nbox, int value)
{
    char Val[] = "=";
    char CB[100];
    char NB[25];
    strcpy(CB, nbox);
    strcat(CB, Val);
    strcat(CB, Str(NB, value, 0));
    SendCommand(CB);
    ValueSent = true;
    GetReturnCode();
}
/*********************************************************************************************************************************/
void GetTextIn()
{
    int j = 0;
    delayMicroseconds(20);
    if (NEXTION.available()) {
        while (NEXTION.available()) {
            TextIn[j] = uint8_t(NEXTION.read());
            if (TextIn[j] == '$') TextIn[j] = 0; 
            if (j < CHARSMAX) ++j;
            delayMicroseconds(20);
        }
    }
}
/*********************************************************************************************************************************/
uint32_t GetValue(char* nbox) 
{
    uint32_t ValueIn = 0;
    char   GET[]   = "get ";
    char   VAL[]   = ".val";
    char   CB[100];
    
    strcpy(CB, GET);
    strcat(CB, nbox);
    strcat(CB, VAL);
    NEXTION.print(CB);
    EndSend(); 
    GetTextIn();
    if (TextIn[0] == 'q') {
        ValueIn = TextIn[1]; // Collect and build 32 bit value from 4 bytes
        ValueIn += (TextIn[2] << 8);
        ValueIn += (TextIn[3] << 16);  
        ValueIn += (TextIn[4] << 24);
    } else{
        ValueIn = 65535;    // = THERE WAS AN ERROR !
    }
    return ValueIn;
}

/*********************************************************************************************************************************/

uint32_t GetValueSafer(char* nbox) // This function calls the function above until it returns no error
{
    int i               = 0;
    uint32_t   ValueIn  = GetValue(nbox);

    while (ValueIn == 65535 && i < 25){ // if error read again!
        delay (50);
        ValueIn = GetValue(nbox);
        ++i;
    }
    return ValueIn;
}

// ***************************************************************************************************************
// This function gets Nextion textbox Text into a char array pointed to by * TheText. There better be room!
// It returns the length of array
uint16_t GetText(char* TextBoxName, char* TheText)
{
    char   get[]   = "get ";
    char   _txt[]   = ".txt";
    char   CB[100];
    uint8_t j = 0;
    strcpy(CB, get);
    strcat(CB, TextBoxName);
    strcat(CB, _txt);
    NEXTION.print(CB);
    EndSend();
    GetTextIn();
    if (TextIn[0] == 'p') {
      while (TextIn[j+1] < 0xFF){
        TheText[j] = TextIn[j+1];
        ++j;
      }
       TheText[j] = 0;
    }
    return  strlen(TheText);
}
/*********************************************************************************************************************************/
int GetOtherValue(char* nbox)  // don't add .val as other thingy is already there ...
{
    double ValueIn = 0;
    char   GET[]   = "get ";
    char   CB[100];
    strcpy(CB, GET);
    strcat(CB, nbox);
    NEXTION.print(CB);
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
    uint8_t a   = 0;
    int i       = 0;
    delayMicroseconds(20);                // 20 seems best so far value here
    bool ButtonPressed = false;
    if (NEXTION.available()) {
        ButtonPressed = true;
        while (NEXTION.available()) {
            a = NEXTION.read();
          //  if (a > 127) Serial.println (a);   // to detect errors 
            if (a > 31 && a < 254) {
                TextIn[i]     = a;
                if (TextIn[i] == '$') TextIn[i] = 0; 
                TextIn[i + 1] = 0;
            }
            if (i < CHARSMAX-1) ++i; 
            delayMicroseconds(20);      // 20 seems best so far value here
        }
    }
   if(!(strlen(TextIn))) ButtonPressed = false;
   if (ButtonPressed && ButtonClicks) PlaySound(CLICKONE);
   return ButtonPressed;
}

/*********************************************************************************************************************************/
//             END OF NEXTION FUNCTIONS
/*********************************************************************************************************************************/

FASTRUN void CheckTimer()
{
   
    if (FlightMode < 4 && !LostContactFlag) {
        Secs  = ((millis() - TimerMillis) / 1000) + PausedSecs;
        Hours = Secs / 3600;
        Secs %= 3600;
        Mins = Secs / 60;
        Secs %= 60;
    }
    if (LastSeconds != Secs) {
        ClockSpoken = false;
        if (CurrentView == FRONTVIEW) {
            SendValue(FrontView_Secs, Secs);
            SendValue(FrontView_Mins, Mins);
            SendValue(FrontView_Hours, Hours);
        }
        LastSeconds = Secs;       
    }
    if (!Secs && SpeakingClock && !ClockSpoken){   
        ClockSpoken = true;
        switch (Mins) {
        case 1:
            PlaySound(ONEMINUTE);
            break;
        case 2:
            PlaySound(TWOMINUTES);
            break;
        case 3:
            PlaySound(THREEMINUTES);
            break;
        case 4:
            PlaySound(FOURMINUTES);
            break;
        case 5:
            PlaySound(FIVEMINUTES); 
            break;
        case 6:
            PlaySound(SIXMINUTES); 
            break;
        case 7:
            PlaySound(SEVENMINUTES); 
            break;
        case 8:
            PlaySound(EIGHTMINUTES); 
            break;
        case 9:
            PlaySound(NINEMINUTES); 
            break;
        case 10:
            PlaySound(TENMINUTES); 
            break;
        default:
            break;   
        }  
    }
}
/*********************************************************************************************************************************/

FASTRUN void ShowServoPos()
{
    if ((Connected) || (CurrentView == GRAPHVIEW))  { // (this is needed! :-)
        if (millis() - ShowServoTimer <= 60) return;    
        ShowServoTimer = millis();
    }
    char Ch_Lables[16][5]    =  {"Ch1","Ch2","Ch3","Ch4","Ch5","Ch6","Ch7","Ch8","Ch9","Ch10","Ch11","Ch12","Ch13","Ch14","Ch15","Ch16"};
    char ChannelInput[]      =  "Input";
    char ChannelOutput[]     =  "Output"; 
    uint16_t StickPosition   =  54321;
    int l             = 0;
    int l1            = 0;
    int LeastDistance = 2;          // if the change is very small, don't re-display anything - to reduce flashing.
  
    if ((CurrentView == STICKSVIEW) || (CurrentView == FRONTVIEW) || (CurrentView == CALIBRATEVIEW)){ 
        for (int i = 0; i < 8; ++i){
            if (SendBuffer[i] != ShownBuffer[i]) {
                SendValue(Ch_Lables[i], IntoLowerRes(SendBuffer[i]));
                ShownBuffer[i] = SendBuffer[i];
            }
        }
    }
    if ((CurrentView == STICKSVIEW) || (CurrentView == FRONTVIEW) ){ 
        for (int i = 8; i < 16; ++i){
            if (SendBuffer[i] != ShownBuffer[i]){
                SendValue(Ch_Lables[i], IntoLowerRes(SendBuffer[i]));
                ShownBuffer[i] = SendBuffer[i];
            }
        }
    }
    
    if (CurrentView == GRAPHVIEW) { 
#define fixitx 35
#define BarWidth 3
        if (ChanneltoSet <= 8) {
            l  = (InPutStick[ChanneltoSet - 1]);
            l1 = analogRead(AnalogueInput[l]);
            if (ReversedChannelBITS & 1 << (ChanneltoSet-1)){  // reversed??
                    if (l1 <= ChannelCentre[l]){
                        l1 = map(l1,ChannelMin[l],ChannelCentre[l],ChannelMax[l],ChannelCentre[l]);
                    }else{
                        l1 = map(l1,ChannelCentre[l],ChannelMax[l],ChannelCentre[l],ChannelMin[l]);
                    }
            }
            if (l1 <= ChannelCentre[l]) {
                SendValue(ChannelInput, map(l1, ChannelCentre[l], ChannelMin[l], 0, -100));
                StickPosition = map(l1, ChannelMin[l], ChannelCentre[l],BoxLeft-0,BoxLeft+(((BoxRight-fixitx)-BoxLeft)/2));
                if (abs(StickPosition - SavedLineX) > LeastDistance) {
                    DisplayCurve();
                    FillBox(StickPosition-1,BoxTop+4,BarWidth,(BoxBottom-42)-BoxTop, HighlightColour);
                    SavedLineX = StickPosition;
                }
            }
            else {
                SendValue(ChannelInput, map(l1, ChannelCentre[l], ChannelMax[l], 0, 100));
                StickPosition = map(l1, ChannelCentre[l], ChannelMax[l],BoxLeft+(((BoxRight-fixitx)-BoxLeft)/2),BoxRight-fixitx);
                if (abs(StickPosition - SavedLineX) > LeastDistance) {
                    DisplayCurve();   
                    FillBox(StickPosition-1,BoxTop+4,BarWidth,(BoxBottom-42)-BoxTop, HighlightColour);
                    SavedLineX = StickPosition;
                }
            }
            if (Connected){
                SendValue(ChannelOutput, map(SendBuffer[ChanneltoSet - 1], MINMICROS, MAXMICROS, -100, 100));
            }else{
                SendValue(ChannelOutput,0);   // because when not connected nothing is sent
            }
        }else{
            SendValue(ChannelInput, 0);
            SendValue(ChannelOutput, 0);
        }
    }
}

/*********************************************************************************************************************************/
FASTRUN bool CheckTXVolts(){
    char  DataView_txv[]         = "txv";
    char  JTX[]                  = "JTX";
    char  FrontView_TXBV[]       = "TXBV";
    bool  TXWarningFlag          = false;
    float txpc,txv;            
    char  Vbuf[16];
    char  TXBattInfo[65];
    char  pc[] = "%";
        if (USE_INA219) {
            txv  = (ina219.getBusVoltage_V()) * 100;
            txpc = map(txv, 3.2 * 200, 3.33 * 200, 0, 100); // LiFePo4 Battery 3.1 ->3.35  volts per cell
            if (txpc < LowBattery) TXWarningFlag = true;
            txpc = constrain(txpc, 0, 100);
            strcpy(TXBattInfo, Str(Vbuf,txpc,0));
            strcat(TXBattInfo, pc);
            if (CurrentView == FRONTVIEW) {SendValue(JTX,txpc); SendText(FrontView_TXBV, TXBattInfo);}
            if (CurrentView == DATAVIEW)  SendText(DataView_txv, TransmitterVersionNumber); 
        }
return TXWarningFlag;
}
/*********************************************************************************************************************************/

FASTRUN bool CheckRXVolts(){
    float Volts                  = 0;
    float ReadVolts              = 0;
    char  JRX[]                  = "JRX";
    bool  RXWarningFlag          = false;
    char  Vbuf[16];
    char  RXBattInfo[65];
    float VoltsPerCell              = 0;
    char  FrontView_RXBV[]          = "RXBV";
    char  RXPC[]                    = "RXPC";
    char  PerCell[]                 = " per cell)";
    char  RXBattNA[]                = "(No data from RX)";
    char  v[]                       = "V  (";
    char pc[]                       = "%";
    char spaces[]                   = "  "; 
            ReadVolts = RXModelVolts * 100;
            Volts = map(ReadVolts,  3.4f * RXCellCount * 100 ,  4.2f * RXCellCount * 100, 0, 100);  
            if (RXVoltsDetected) {
                Volts = constrain(Volts, 0, 100);
                if (Volts <= LowBattery && Volts > 0)  RXWarningFlag = true;
                if (BoundFlag && CurrentView == FRONTVIEW) SendValue(JRX, Volts);
                strcat(Str(Vbuf,Volts,0),pc);
                SendText(RXPC,Vbuf);
                strcpy(RXBattInfo, ModelVolts);
                strcat(RXBattInfo, v);
                VoltsPerCell = (ReadVolts / RXCellCount) / 100;
                dtostrf(VoltsPerCell, 2, 2, Vbuf);
                strcat(RXBattInfo, Vbuf);
                strcat(RXBattInfo, PerCell);
                if (BoundFlag && CurrentView == FRONTVIEW) SendText(FrontView_RXBV, RXBattInfo);
            }else{
                if (BoundFlag && CurrentView == FRONTVIEW) {
                    SendText(FrontView_RXBV, RXBattNA);
                    SendValue(JRX, 0);
                    SendText(RXPC,spaces);
                }
            }
            return RXWarningFlag;
}
/*********************************************************************************************************************************/
  void CheckScreenTime(){
  char ScreenOff[] = "dim=10";
  if ((millis() - ScreenTimeTimer) > ScreenTimeout * 1000) {
      SendCommand(ScreenOff);
      ScreenTimeTimer = millis();
      ScreenIsOff = true;
  }
 }
/*********************************************************************************************************************************/

/** @brief SHOW COMMS */

// This displays many telemetry data onto the current screen

FASTRUN void ShowComms()
{
    if (NEXTION.available()) return; // was a button pressed?
    char  WarnNow[]              = "vis Warning,1";
    char  WarnOff[]              = "vis Warning,0";
    bool  ShowNow                = false;
    char  na[]                   = "";
    char  FrontView_Connected[]  = "Connected";
    char  FrontView_AckPayload[] = "AckPayload";
    char  FrontView_RXBV[]       = "RXBV";
    char  Not_Connected[]        = "Not connected";
    char  Msg_Connected[]        = "** Connected! **";
    char  Msg_CnctdBuddyMast[]   = "* BUDDY MASTER! *";
    char  Msg_CnctdBuddySlave[]  = "* BUDDY SLAVE! *";
    char  MsgBuddying[]          = "Buddy";
    char  DataView_pps[]         = "pps";       // These are label names in the NEXTION data screen. They are best kept short.
    char  DataView_lps[]         = "lps";
    char  DataView_Alt[]         = "alt";
    char  DataView_Temp[]        = "Temp";
    char  DataView_MaxAlt[]      = "MaxAlt";
    char  DataView_rxv[]         = "rxv";
    char  DataView_Ls[]          = "Ls";
    char  DataView_Ts[]          = "Ts";
    char  DataView_Rx[]          = "rx";
    char  DataView_Sg[]          = "Sg";
    char  DataView_Ag[]          = "Ag";
    char  DataView_Gc[]          = "Gc";
    char  Vbuf[16];
    char  BindButtonVisible[] = "vis bind,1";
    char  Fix[]               = "Fix";        // These are label names in the NEXTION data screen. They are best kept short.
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
    char  Sbs[]               = "Sbus";


if (millis() - LastShowTime > SHOWCOMMSDELAY) { 
    ShowNow = true;
    LastShowTime = millis();
}
if (ShowNow){
    if (CurrentView == FRONTVIEW || CurrentView == DATAVIEW) {
        if (!LostContactFlag)
        {
            if ((CurrentView == FRONTVIEW)) {
                if (!BoundFlag) {
                    SendCommand(BindButtonVisible); 
                    BindButton = true;
                }
                else {
                    if (BoundFlag)
                    {
                        if (!BuddyMaster)
                        {
                            if (!Reconnected){
                                MakeBindButtonInvisible();
                                SendText(FrontView_Connected, Msg_Connected);
                                Reconnected = true;
                            }
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
            if (CurrentView == DATAVIEW) {
                SendValue(DataView_pps,   PacketsPerSecond);
                SendValue(DataView_lps,   LostPackets);
                SendText(DataView_Alt,    ModelAltitude);
                SendText(DataView_MaxAlt, MaxAltitude);
                SendText(DataView_Temp,   ModelTemperature);
                SendText(DataView_Rx,     ThisRadio);
                SendText(DataView_rxv,    ReceiverVersionNumber);
                SendValue(DataView_Ls,    GapLongest);
                SendValue(DataView_Ts,    RadioSwaps - SavedRadioSwaps);
                SendValue(DataView_Sg,    RX1TotalTime - SavedRX1TotalTime);
                SendValue(DataView_Ag,    GapAverage);
                SendValue(DataView_Gc,    RX2TotalTime - SavedRX2TotalTime);
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
                snprintf(Vbuf, 6,"%d",  (int) SbusRepeats - SavedSbusRepeats);
                SendText(Sbs,Vbuf);   
            }
        }
        else {
            if (BoundFlag) {
                if (CurrentView == DATAVIEW) {
                    PacketsPerSecond = 0;
                    SendValue(DataView_pps, PacketsPerSecond);
                    SendValue(DataView_lps, LostPackets);
                }
                if (CurrentView == FRONTVIEW) {
                    SendText(FrontView_Connected, Not_Connected);
                    SendText(FrontView_RXBV, na); // data not available
                    SendText(FrontView_AckPayload, na);
                }
            }
            else // i.e. contact is lost
            {
                if (CurrentView == FRONTVIEW)
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
 CheckScreenTime(); 
 if (CheckTXVolts() || CheckRXVolts()) {        // Note: If TX Battery is low, then CheckRXVolts() is not even called.
        LedIsBlinking = true; 
        if((millis() - WarningTimer) > 10000) {
            WarningTimer = millis();
            PlaySound(BATTERYISLOW); // issue audible warning every 10 seconds
        }
        if (CurrentView == FRONTVIEW) SendCommand(WarnNow);         
    }else{
        if (LedIsBlinking && (CurrentView == FRONTVIEW)) SendCommand(WarnOff);
        LedIsBlinking = false; 
    }
}
} // end ShowComms()

/************************************************************************************************************/
void  ReEnableScanButton(){ 
    char b5NOTGreyed[]= "b5.pco=";
    char nb[15];
    char cmd[30];
     if (CurrentView == MAINSETUPVIEW){
         if (b5isGrey){
            Str(nb,ForeGroundColour,0);
            strcpy(cmd,b5NOTGreyed);
            strcat(cmd,nb);
            SendCommand(cmd);
            b5isGrey = false;
        }   
    }
}


/*********************************************************************************************************************************/

/** Send 13 joined together char arrays to NEXTION */
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

int GetNextNumber(int p1, char text1[CHARSMAX])
{
    char text2[CHARSMAX];
    int  j = 0;
    int  i = p1 - 1;
    while (isDigit(text1[i]) && i < CHARSMAX) {
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
 FASTRUN uint16_t GetStickInput(uint8_t l)
{
   uint16_t k = 0;
   switch (l) {
    case 8:
        if (Channel9SwitchValue == 0) k = IntoHigherRes(MinDegrees[FlightMode][8]);
        if (Channel9SwitchValue == 90) k = IntoHigherRes(CentreDegrees[FlightMode][8]);
        if (Channel9SwitchValue == 180) k = IntoHigherRes(MaxDegrees[FlightMode][8]);
        break;
    case 9:
        if (Channel10SwitchValue == 0) k = IntoHigherRes(MinDegrees[FlightMode][9]);
        if (Channel10SwitchValue == 90) k = IntoHigherRes(CentreDegrees[FlightMode][9]);
        if (Channel10SwitchValue == 180) k = IntoHigherRes(MaxDegrees[FlightMode][9]);
        break;
    case 10:
        if (Channel11SwitchValue == 0) k = IntoHigherRes(MinDegrees[FlightMode][10]);
        if (Channel11SwitchValue == 90) k = IntoHigherRes(CentreDegrees[FlightMode][10]);
        if (Channel11SwitchValue == 180) k = IntoHigherRes(MaxDegrees[FlightMode][10]);
        break;
    case 11:
        if (Channel12SwitchValue == 0) k = IntoHigherRes(MinDegrees[FlightMode][11]);
        if (Channel12SwitchValue == 90) k = IntoHigherRes(CentreDegrees[FlightMode][11]);
        if (Channel12SwitchValue == 180) k = IntoHigherRes(MaxDegrees[FlightMode][11]);
        break;
    default:
           k = IntoHigherRes(CentreDegrees[FlightMode][15]); // channels 13,14,15,16 are simply centred
    }
    return k;
}
/*********************************************************************************************************************************/
// MIXES  (Channel mixes)
/*********************************************************************************************************************************/
FASTRUN void DoMixes()
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
                        TheSum = SendBuffer[(Mixes[m][M_SlaveChannel]) - 1] + p;  // THIS IS THE MIX!
                        mindeg = IntoHigherRes(MinDegrees[FlightMode][(Mixes[m][M_SlaveChannel]) - 1]);
                        maxdeg = IntoHigherRes(MaxDegrees[FlightMode][(Mixes[m][M_SlaveChannel]) - 1]);
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
//                  My new version of the the traditional "map()" function -- but here with exponential added.
/*********************************************************************************************************************************/
 
FASTRUN float MapWithExponential(float xx, float Xxmin, float Xxmax, float Yymin, float Yymax, float Expo)
{
    Expo  = map(Expo, -100, 100, -0.25, 0.75);
    xx    = pow(xx * xx, Expo);
    Xxmin = pow(Xxmin * Xxmin, Expo);
    Xxmax = pow(Xxmax * Xxmax, Expo);
    return map(xx, Xxmin, Xxmax, Yymin, Yymax);
}


/******************************************** CHANNEL REVERSE FUNCTION **********************************************************/

FASTRUN void DoReverseSense(){
  for (uint8_t i = 0; i < 16; i++) {
    if (ReversedChannelBITS & 1 << i){                                                              // Is this channel reversed?
            PreMixBuffer[i] = map(SendBuffer[i],MINMICROS,MAXMICROS,MAXMICROS,MINMICROS);           // Yes so reverse the channel
            SendBuffer[i] = PreMixBuffer[i];
    }
  }
}
/*********************************************************************************************************************************/

/** @brief GET NEW SERVO POSITIONS */
FASTRUN void GetNewChannelValues()
{
    uint16_t k = 0, l = 0, m = 0, n = 0, TrimAmount;
    // key: -
    // m = input value
    // l = input channel
    // n = output channel
    // k = interim output result
    // TrimAmount = TrimAmount :-)

    for (n = 0; n < CHANNELSUSED; ++n) {
        l = InPutStick[n];                                      // input sticks knobs & switches are now mapped by user
        if (l <= 7)
        {
            m = analogRead(AnalogueInput[l]);                   // Get values from sticks' pots
        }

        if (l > 7)
        {                                                       // Switch ?
            k = GetStickInput(l);                               // Four 3 postion switches
        }
        else {                                                  // Map the eight analogue inputs
            if (InterpolationTypes[FlightMode][n] == STRAIGHTLINES) {       
                if (m >= ChannelMidHi[l]) k = map(m, ChannelMidHi[l], ChannelMax[l], IntoHigherRes(MidHiDegrees[FlightMode][n]), IntoHigherRes(MaxDegrees[FlightMode][n]));
                if (m >= ChannelCentre[l] && m <= (ChannelMidHi[l])) k = map(m, ChannelCentre[l], ChannelMidHi[l], IntoHigherRes(CentreDegrees[FlightMode][n]), IntoHigherRes(MidHiDegrees[FlightMode][n]));
                if (m >= ChannelMidLow[l] && m <= ChannelCentre[l]) k = map(m, ChannelMidLow[l], ChannelCentre[l], IntoHigherRes(MidLowDegrees[FlightMode][n]), IntoHigherRes(CentreDegrees[FlightMode][n]));
                if (m <= ChannelMidLow[l]) k = map(m, ChannelMin[l], ChannelMidLow[l], IntoHigherRes(MinDegrees[FlightMode][n]), IntoHigherRes(MidLowDegrees[FlightMode][n]));
            }

            if (InterpolationTypes[FlightMode][n] == SMOOTHEDCURVES) {           // CatmullSpline (!)
                xPoints[0] = ChannelMin[l];
                xPoints[1] = ChannelMidLow[l];
                xPoints[2] = ChannelCentre[l];
                xPoints[3] = ChannelMidHi[l];
                xPoints[4] = ChannelMax[l];
                yPoints[4] = IntoHigherRes(MaxDegrees[FlightMode][n]);
                yPoints[3] = IntoHigherRes(MidHiDegrees[FlightMode][n]);
                yPoints[2] = IntoHigherRes(CentreDegrees[FlightMode][n]);
                yPoints[1] = IntoHigherRes(MidLowDegrees[FlightMode][n]);
                yPoints[0] = IntoHigherRes(MinDegrees[FlightMode][n]);
                k          = Interpolation::CatmullSpline(xPoints, yPoints, PointsCount, m);
            }
            if (InterpolationTypes[FlightMode][n] == EXPONENTIALCURVES) {               // EXPONENTIAL (!!)
                if (m >= ChannelCentre[l]) {
                    k = MapWithExponential(m - ChannelCentre[l], 0, ChannelMax[l] - ChannelCentre[l], 0, IntoHigherRes(MaxDegrees[FlightMode][n]) - IntoHigherRes(CentreDegrees[FlightMode][n]), Exponential[FlightMode][n]) + IntoHigherRes(CentreDegrees[FlightMode][n]);
                }
                if (m < ChannelCentre[l]) {
                    k = MapWithExponential(ChannelCentre[l] - m, 0, ChannelCentre[l] - ChannelMin[l], IntoHigherRes(CentreDegrees[FlightMode][n]) - IntoHigherRes(MinDegrees[FlightMode][n]), 0, Exponential[FlightMode][n]) + IntoHigherRes(MinDegrees[FlightMode][n]);
                }
            }
        }
        k += (SubTrims[n]-127) * (TrimFactor/2);                        //  ADD SUBTRIM (just to output channel, ignoring any mapped input channel) (Range 0 - 127 - 254)
        if (l < 4) {
            TrimAmount = (Trims[FlightMode][l] - 80) * TrimFactor;      // TRIMS on lower four channels (80 is mid point !! (range 40 - 80 - 120)) 
            if (!TrimsReversed[FlightMode][l]) {
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
    if (CurrentMode == NORMAL) {
        DoReverseSense();
        DoMixes(); 
    }                                 // not while calibrating

}

/*********************************************************************************************************************************/

void ReduceLimits(){                              // Get things setup for sticks calibration
 for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        ChannelMax[i] = 512;
        ChannelMin[i] = 512;
    }
    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        MaxDegrees[FlightMode][i]       = 180;
        CentreDegrees[FlightMode][i]    = 90;
        MinDegrees[FlightMode][i]       = 0;
    }
}
/*********************************************************************************************************************************/
void CalibrateSticks()   // This discovers end of travel place for sticks etc. 
{
    uint16_t p;
    for (uint8_t i = 0; i < PROPOCHANNELS; ++i)
    {
       p = analogRead(AnalogueInput[i]);
       if (ChannelMax[i] < p)   ChannelMax[i] = p;
       if (ChannelMin[i] > p)   ChannelMin[i] = p;
    }
    GetNewChannelValues();
}
/*********************************************************************************************************************************/
/** @brief Get centre as 90 degrees */
void ChannelCentres()
{ 
    for (int i = 0; i < PROPOCHANNELS; ++i) {
        ChannelCentre[i] = analogRead(AnalogueInput[i]);
        ChannelMidHi[i]  = ChannelCentre[i] + ((ChannelMax[i] - ChannelCentre[i]) / 2);
        ChannelMidLow[i] = ChannelMin[i] + ((ChannelCentre[i] - ChannelMin[i]) / 2);
    }
    GetNewChannelValues();
    CalibrateEdgeSwitches();        // These are now calibrated too in case some are reversed.
}
/*********************************************************************************************************************************/
void UpdateTrimView()
{
    char Mode1[]        = "Mode1";
    char Mode2[]        = "Mode2";
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
    
    SendValue(TrimView_ch1, (Trims[FlightMode][0]));
    SendValue(TrimView_ch4, (Trims[FlightMode][1]));
    SendValue(TrimView_ch2, (Trims[FlightMode][2]));
    SendValue(TrimView_ch3, (Trims[FlightMode][3]));

    if (CurrentView == TRIM_VIEW) 
    {
        SendValue(TrimView_n1, (Trims[FlightMode][0] - 80));
        SendValue(TrimView_n4, (Trims[FlightMode][1] - 80));
        SendValue(TrimView_n2, (Trims[FlightMode][2] - 80));
        SendValue(TrimView_n3, (Trims[FlightMode][3] - 80));

        SendValue(TrimView_r1, TrimsReversed[FlightMode][0]);
        SendValue(TrimView_r4, TrimsReversed[FlightMode][1]);
        SendValue(TrimView_r2, TrimsReversed[FlightMode][2]);
        SendValue(TrimView_r3, TrimsReversed[FlightMode][3]);
        if (SticksMode == 2) {
                SendValue(Mode2,1);
                SendValue(Mode1,0);}
            else {
                SendValue(Mode1,1);
                SendValue(Mode2,0);
                }
        }
}
/*********************************************************************************************************************************/

FLASHMEM void ScanI2c()
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

void OpenModelsFile(){
    if (SingleModelFlag) {
        ModelsFileNumber = SD.open(SingleModelFile, FILE_WRITE);
    }
    else {
        ModelsFileNumber = SD.open(ModelsFile, FILE_WRITE);
    }
    if (ModelsFileNumber == 0) {
        FileError = true;
    }else{
        ModelsFileOpen = true;
        }
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
    char fm1[]                  = "Bank 1";
    char fm2[]                  = "Bank 2";
    char fm3[]                  = "Bank 3";
    char fm4[]                  = "Bank 4";
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
    char NoName[17];
    char Ch[] = "Channel ";
    char Nbuf[7];

    switch (CurrentView) {
        case FRONTVIEW:
            SendText(FrontView_ModelName, ModelName);
            UpdateTrimView();
            break;
        case STICKSVIEW:
            SendText(SticksView_ModelName, ModelName);
            break;
        case MIXESVIEW:
            SendText(MixesView_ModelName, ModelName);
            break;
        case GRAPHVIEW:
            SendText(GraphView_ModelName, ModelName);
            if (strlen(ChannelNames[ChanneltoSet - 1]) < 2) { // if no name, just show the channel number
                strcpy(NoName, Ch);
                SendText(GraphView_Channel, strcat(NoName, Str(Nbuf, ChanneltoSet, 0)));
            }
            else {
                SendText(GraphView_Channel, ChannelNames[ChanneltoSet - 1]);
            }
            break;
        case MODELSVIEW:
            SendText(ModelsView_ModelName, ModelName);
            break;
        case TRIM_VIEW:
            SendText(TrimView_ModelName, ModelName);
            UpdateTrimView();
            break;
        default:
            break;
    }

    if (FlightMode == 1) {
        if (CurrentView == STICKSVIEW) SendText(SticksView_t1, fm1);
        if (CurrentView == GRAPHVIEW) SendText(GraphView_fmode, fm1);
        if (CurrentView == TRIM_VIEW) {
            SendText(TrimView_FlightMode, fm1);
            UpdateTrimView();
        }
    }
    if (FlightMode == 2) {
        if (CurrentView == STICKSVIEW) SendText(SticksView_t1, fm2);
        if (CurrentView == GRAPHVIEW) SendText(GraphView_fmode, fm2);
        if (CurrentView == TRIM_VIEW) {
            SendText(TrimView_FlightMode, fm2);
            UpdateTrimView();
        }
    }
    if (FlightMode == 3) {
        if (CurrentView == STICKSVIEW) SendText(SticksView_t1, fm3);
        if (CurrentView == GRAPHVIEW) SendText(GraphView_fmode, fm3);
        if (CurrentView == TRIM_VIEW) {
            SendText(TrimView_FlightMode, fm3);
            UpdateTrimView();
        }
    }
    if (FlightMode == 4) {
        if (CurrentView == STICKSVIEW) SendText(SticksView_t1, fm4);
        if (CurrentView == GRAPHVIEW) SendText(GraphView_fmode, fm4);
        if (CurrentView == TRIM_VIEW) {
            SendText(TrimView_FlightMode, fm4);
            UpdateTrimView();
        }
    }
}

/*********************************************************************************************************************************/

FLASHMEM void InitSwitchesAndTrims()
{
    for (int i = 0; i < 8; ++i) {
        pinMode(SwitchNumber[i], INPUT_PULLUP);
        pinMode(TrimNumber[i], INPUT_PULLUP);
    }
}

/*********************************************************************************************************************************/

/** @brief STICKS CALIBRATION */
FLASHMEM void InitMaxMin()
{
    for (int i = 0; i < CHANNELSUSED; ++i) {
        ChannelMax[i]    = 1024;
        ChannelMidHi[i]  = 512 + 256;
        ChannelCentre[i] = 512;
        ChannelMidLow[i] = 256;
        ChannelMin[i]    = 0;
    }
}

/*********************************************************************************************************************************/

FLASHMEM void CentreTrims()
{
    for (int j = 0; j <= FLIGHTMODESUSED; ++j) {
        for (int i = 0; i < CHANNELSUSED; ++i) {
            Trims[j][i] = 80;
        }
    }
}

/*********************************************************************************************************************************/

FLASHMEM void InitCentreDegrees()
{
  
    for (int j = 1; j <= 4; ++j) {
        for (int i = 0; i < CHANNELSUSED; ++i) {
            MaxDegrees[j][i]    = 180; //  180 degrees
            MidHiDegrees[j][i]  = 135;
            CentreDegrees[j][i] = 90; //  90 degrees
            MidLowDegrees[j][i] = 45;
            MinDegrees[j][i]    = 0; //  0 degrees
        }
    }
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
    if (CurrentView == STICKSVIEW) {
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
    if (CurrentView == INPUTS_VIEW || CurrentView == FAILSAFE_VIEW || CurrentView == REVERSEVIEW) {
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
    uint16_t j;
    uint16_t i;
    char NoModelYet[]  = "Model ";
    char nb[5];

    Str(nb,ModelNumber,0);
    strcpy(ModelName,NoModelYet);
    strcat(ModelName, nb);


    if (!ModelsFileOpen) OpenModelsFile();
    if (!ModelsFileOpen) return false;
    SDCardAddress = TXSIZE;                    //  spare bytes for TX stuff
    SDCardAddress += ((Mnum - 1) * MODELSIZE); //  spare bytes for Model params
    StartLocation = SDCardAddress;
    ModelDefined  = SDReadByte(SDCardAddress); 
    ++SDCardAddress;
    if (ModelDefined != 42) return false;
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

    for (j = 0; j < FLIGHTMODESUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            Trims[j][i] = SDReadByte(SDCardAddress);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < FLIGHTMODESUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            TrimsReversed[j][i] = SDReadByte(SDCardAddress);
            ++SDCardAddress;
        }
    }
    RXCellCount = SDReadByte(SDCardAddress);
    ++SDCardAddress;
    TrimFactor = SDReadInt(SDCardAddress);  
    if (TrimFactor < 1) TrimFactor = 1;
    if (TrimFactor > 10) TrimFactor = 10;
    ++SDCardAddress;
    ++SDCardAddress;
    LowBattery = SDReadByte(SDCardAddress);  
    if (LowBattery>100) LowBattery = LOWBATTERY;
    if (LowBattery<10) LowBattery = LOWBATTERY;
    ++SDCardAddress;
    CopyTrimsToAll = SDReadByte(SDCardAddress); 
    ++SDCardAddress;
    
    for (i = 0; i < CHANNELSUSED; ++i) {
        SubTrims[i] = SDReadByte(SDCardAddress);   
        if ((SubTrims[i] < 10) ||  (SubTrims[i] > 244))  SubTrims[i] = 127; // centre if undefined or zero
        ++SDCardAddress;                
    }
    ReversedChannelBITS = SDReadInt(SDCardAddress);
    ++SDCardAddress; 
    ++SDCardAddress; 
   
    SDCardAddress += 9; // 9 Spare Bytes here (PID stuff gone) *****************************
    
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
    SWITCH1Reversed = bool(SDReadByte(SDCardAddress));
    ++SDCardAddress;
    SWITCH2Reversed = bool(SDReadByte(SDCardAddress));
    ++SDCardAddress;
    SWITCH3Reversed = bool(SDReadByte(SDCardAddress));
    ++SDCardAddress;
    SWITCH4Reversed = bool(SDReadByte(SDCardAddress));
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

    for (j = 0; j < FLIGHTMODESUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            Exponential[j][i] = SDReadByte(SDCardAddress);
            if (Exponential[j][i] >= 201 || Exponential[j][i] == 0 ) {
                Exponential[j][i] = DEFAULT_EXPO;
            }
            ++SDCardAddress;
        }
    }
    for (j = 0; j < FLIGHTMODESUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            InterpolationTypes[j][i] = SDReadByte(SDCardAddress);
            if (InterpolationTypes[j][i] < 0 || InterpolationTypes[j][i] > 2) {
                InterpolationTypes[j][i] = EXPONENTIALCURVES;
            }
            ++SDCardAddress;
        }
    }
    for ( j = 0; j < BYTESPERMACRO; ++j){
        for ( i = 0; i < MAXMACROS; ++i){
                MacrosBuffer[i][j] = SDReadByte(SDCardAddress);
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
    CheckMacrosBuffer();
    return true;
}

/*********************************************************************************************************************************/

bool LoadAllParameters()
{
    int p;
    int j = 0;
    int i = 0;
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
        ++SDCardAddress;
        ScreenTimeout = SDReadInt(SDCardAddress);
        ++SDCardAddress;
        ++SDCardAddress;
        Inactivity_Timeout = SDReadByte(SDCardAddress) * TICKSPERMINUTE;
        if (Inactivity_Timeout < INACTIVITYMINIMUM) Inactivity_Timeout = INACTIVITYMINIMUM;
        if (Inactivity_Timeout > INACTIVITYMAXIMUM) Inactivity_Timeout = INACTIVITYMAXIMUM;
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
        BackGroundColour=SDReadInt(SDCardAddress);
       if(BackGroundColour == 0) BackGroundColour = 214;
        ++SDCardAddress;
        ++SDCardAddress;
        ForeGroundColour=SDReadInt(SDCardAddress);
        if(ForeGroundColour == 0) ForeGroundColour = 65535;
       ++SDCardAddress;
       ++SDCardAddress;
       SpecialColour=SDReadInt(SDCardAddress);
       if(SpecialColour == 0) SpecialColour = Red;
       ++SDCardAddress;
       ++SDCardAddress;
       HighlightColour=SDReadInt(SDCardAddress);
      if(HighlightColour == 0) HighlightColour = Yellow;
       ++SDCardAddress;
       ++SDCardAddress;
        SticksMode=SDReadByte(SDCardAddress);
        ++SDCardAddress;
        AudioVolume=SDReadByte(SDCardAddress);
         ++SDCardAddress;
        Brightness = SDReadByte(SDCardAddress);
        if (Brightness < 10) Brightness = 10;
         ++SDCardAddress;
        PlayFanfare = SDReadByte(SDCardAddress);
        ++SDCardAddress;
        TrimClicks = SDReadByte(SDCardAddress);
        ++SDCardAddress;
        ButtonClicks = SDReadByte(SDCardAddress);
        ++SDCardAddress;
        SpeakingClock = SDReadByte(SDCardAddress);
        ++SDCardAddress;
        AnnounceBanks= SDReadByte(SDCardAddress);
        ++SDCardAddress;
        for (i = 0; i < 8; ++i){
            j = SDReadByte(SDCardAddress);
            if ((j >= SWITCH7) && (j <= SWITCH0))  {SwitchNumber[i] = j;} 
            ++SDCardAddress;
        }
        BuddyTriggerChannel= SDReadByte(SDCardAddress);
        ++SDCardAddress;
        MinimumGap = SDReadByte(SDCardAddress);
        ++SDCardAddress;
        LogRXSwaps = SDReadByte(SDCardAddress);
        ++SDCardAddress;
        UseLog = SDReadByte(SDCardAddress);
        ++SDCardAddress;
        AnnounceConnected = SDReadByte(SDCardAddress);
         ++SDCardAddress;
        
        MemoryForTransmtter = SDCardAddress;
        ReadOneModel(ModelNumber); // this
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

void WatchDogCallBack()
{
    // Serial.println("RESETTING ...");
}

/*********************************************************************************************************************************/

FLASHMEM void SetDS1307ToCompilerTime()
{
    if (getDate(__DATE__) && getTime(__TIME__)) {
        RTC.write(tm);
    } // only useful when connected to compiler
}

/************************************************************************************************************/

FLASHMEM void GetTXVersionNumber()
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
FASTRUN void SetUKFrequencies(){ 
            FHSSChPointer = FHSS_Channels; 
            UkRules = true;        
}
/************************************************************************************************************/
FASTRUN void SetTestFrequencies(){
            FHSSChPointer = FHSS_Channels1; 
            UkRules = false;     
}
/************************************************************************************************************/
FASTRUN void CreateTimeStamp(char *  DateAndTime){
    char NB[10];
    char zero[]     = "0";
    char Colon[]    = "."; // a dot!
    char null[] = "";
  
    if (RTC.read(tm)) { 
            strcpy(DateAndTime,null);
            if (MayBeAddZero(tm.Hour)) strcat(DateAndTime,zero);
            strcat(DateAndTime, Str(NB, tm.Hour,0));
            strcat(DateAndTime, Colon);
            if (MayBeAddZero(tm.Minute)) strcat(DateAndTime,zero);
            strcat(DateAndTime, Str(NB, tm.Minute , 0));
            strcat(DateAndTime, Colon);
            if (MayBeAddZero(tm.Second)) strcat(DateAndTime,zero);
            strcat(DateAndTime, Str(NB, tm.Second , 0));
    }
}

/************************************************************************************************************/
FASTRUN void CreateTimeDateStamp(char *  DateAndTime){
    char NB[10];
    char zero[]     = "0";
    char Dash[]     = "-";
    char Colon[]    = "."; // a dot!
    char Space[]    = " ";
  
    if (RTC.read(tm)) { 
            if (MayBeAddZero(tm.Day)) strcat(DateAndTime,zero);
            strcpy(DateAndTime, Str(NB, tm.Day , 0));
            strcat(DateAndTime, Dash);
            if (MayBeAddZero(tm.Month)) strcat(DateAndTime,zero);
            strcat(DateAndTime, Str(NB, tm.Month , 0));
            strcat(DateAndTime, Dash);
            strcat(DateAndTime, (Str(NB, tmYearToCalendar(tm.Year), 0)));
            strcat(DateAndTime, Space);
            if (MayBeAddZero(tm.Hour)) strcat(DateAndTime,zero);
            strcat(DateAndTime, Str(NB, tm.Hour,0));
            strcat(DateAndTime, Colon);
            if (MayBeAddZero(tm.Minute)) strcat(DateAndTime,zero);
            strcat(DateAndTime, Str(NB, tm.Minute , 0));
            strcat(DateAndTime, Colon);
            if (MayBeAddZero(tm.Second)) strcat(DateAndTime,zero);
            strcat(DateAndTime, Str(NB, tm.Second , 0));
    }
}
/************************************************************************************************************/
FASTRUN void MakeLogFileName(char * LogFileName){
    char NB[10];
    char Ext[]      = ".LOG";
    if (RTC.read(tm)) { 
        strcpy(LogFileName, Str(NB,tm.Day,0));
        strcat(LogFileName, Str(NB,tm.Month,0));
        strcat(LogFileName, Ext);
    }
}
/************************************************************************************************************/
FASTRUN void OpenLogFileW(char * LogFileName){
    if (!LogFileOpen){
        LogFileNumber = SD.open(LogFileName, FILE_WRITE);
        LogFileOpen = true;
    }
}
// ************************************************************************
FASTRUN void CheckLogFileIsOpen(){
     char LogFileName[20];
     if (!LogFileOpen){
        MakeLogFileName(LogFileName);                   // Create a "today" filename
        OpenLogFileW(LogFileName);                      // Open file for writing
     }
}

/************************************************************************************************************/
FASTRUN void DeleteLogFile(char * LogFileName){
    SD.remove(LogFileName);
}
/************************************************************************************************************/
FASTRUN void DeleteLogFile1(){
    char LogFileName[20];
    char LogTeXt[] = "LogText";     
    char BlankText[] = " ";     
    CloseLogFile();
    MakeLogFileName(LogFileName);  
    DeleteLogFile(LogFileName);   
    SendText1(LogTeXt, BlankText);  
    if (!UseLog) return;
    if (LedWasGreen) {
        RecentStartLine = 0;
        ShowLogFile(RecentStartLine);
    }
}

/************************************************************************************************************/
FASTRUN void OpenLogFileR(char * LogFileName){
    if (!LogFileOpen){
        LogFileNumber = SD.open(LogFileName, FILE_READ);
        LogFileOpen = true;
    }
}
/************************************************************************************************************/
FASTRUN void CloseLogFile(){
    if (LogFileOpen) LogFileNumber.close();
    LogFileOpen = false;
}
/************************************************************************************************************/
FASTRUN void WriteToLogFile(char * SomeData, uint16_t len){
    LogFileNumber.write(SomeData, len);
}
// ************************************************************************
FASTRUN void LogFilePreamble(){
    char dbuf[12];
    char Divider[] = " - ";
    CheckLogFileIsOpen();
    CreateTimeStamp(dbuf);                           // Put time stamp into buffer
    WriteToLogFile(dbuf,9);                          // Add time stamp
    WriteToLogFile(Divider,sizeof (Divider));           
}

// ************************************************************************
FASTRUN void LogText(char * TheText, uint16_t len){
    char crlf[]  = {'|',13,10,0};
    LogFilePreamble();
    WriteToLogFile(TheText,len);
    WriteToLogFile(crlf, sizeof(crlf));
    CloseLogFile();
}
// ************************************************************************
FASTRUN void LogMinGap(){
        char TheText[] = "Minimum logged gap (ms): ";
        char buf[50] = " ";
        char NB[8];
        Str(NB,MinimumGap,0);
        strcpy (buf,TheText);
        strcat (buf,NB);
        LogText(buf, sizeof (buf));
}
// ************************************************************************
FASTRUN void LogConnection(){
        char TheText[] = "Connected to ";
        char buf[40] = " ";
        strcpy (buf,TheText);
        strcat (buf,ModelName);
        LogText(buf, sizeof (buf));
        LogMinGap();
}
// ************************************************************************
FASTRUN void LogDisConnection(){ 
char TheText[] = "Disconnected from ";
        char buf[40] = " ";
        strcpy (buf,TheText);
        strcat (buf,ModelName);
        LogText(buf, sizeof (buf));
}
// ************************************************************************
FASTRUN void LogNewFlightMode(){
    char Ltext[] = "Bank: ";
    char NB[5];
    char thetext[10];
    Str(NB,FlightMode,0);
    strcpy(thetext,Ltext);
    strcat(thetext, NB);
    LogText(thetext, 7);
}

// ************************************************************************

FASTRUN void LogUKRules(){
    char Rtext[] = "Using 2.400 Ghz - 2.483 Ghz";
    char Ttext[] = "Using 2.484 Ghz - 2.525 Ghz";
    if (UkRules) {
        LogText(Rtext, strlen(Rtext));
    } else{
        LogText(Ttext, strlen(Ttext));
    }
}

// ************************************************************************

FASTRUN void LogThisRX(){
    char Ltext[] = "RX: ";
    char thetext[10];
    strcpy (thetext,Ltext);
    strcat (thetext,ThisRadio);
    LogText(thetext, 5);
  
}

// ************************************************************************
FASTRUN void LogLowBattery(){ // Not yet implemented
    char TheText[]= "Low battery";
    LogText(TheText, strlen(TheText));
}
// ************************************************************************

FASTRUN void LogThisGap(){
    char Ltext[] = "Gap: ";
    char NB[5];
    char thetext[10];
    if (ThisGap > 1000) return;
    Str(NB,ThisGap,0);
    strcpy(thetext,Ltext);
    strcat(thetext, NB);
    LogText(thetext, 8);
}
// ************************************************************************

FASTRUN void LogThisLongGap(){    // here is logged a Gap that exceeds one second - probably because rx was turned off
    ThisGap = (millis() - GapStart); 
    char Ltext[] = "Long Gap: ";
    char NB[5];
    char thetext[20];
    Str(NB,ThisGap,0);
    strcpy(thetext,Ltext);
    strcat(thetext, NB);
    LogText(thetext, strlen(Ltext)+4);
}

// ************************************************************************

FASTRUN void LogPowerOn(){    
    char Ltext[] = "Power ON";
    LogText(Ltext, strlen(Ltext));
}

// ************************************************************************

FASTRUN void LogPowerOff(){    
    char Ltext[] = "Power OFF";
    LogText(Ltext, strlen(Ltext));
}

// ************************************************************************

FASTRUN void LogThisModel(){    
    char Ltext[] = "Model loaded: ";
    char thetext[55];
    strcpy(thetext,Ltext);
    strcat(thetext, ModelName);
    LogText(thetext, strlen(Ltext)+strlen(ModelName));
}
// ************************************************************************

void ShowLogFile(uint8_t StartLine){ 
    char TheText[MAXFILELEN + 10];      // MAX = 5K or so
    char LogFileName[20];
    char LogTeXt[] = "LogText";     
    CloseLogFile();
    MakeLogFileName(LogFileName);        // Create "today" filename
    ReadTextFile(LogFileName,TheText,StartLine,MAXLINES);  // Then load text
    SendText1(LogTeXt, TheText);         // Then send it
}
/*********************************************************************************************************************************/
// SETUP
/*********************************************************************************************************************************/
FLASHMEM void setup()
{
    char FrontView_BackGround[]    = "FrontView.BackGround";
    char FrontView_ForeGround[]    = "FrontView.ForeGround";
    char FrontView_Special[]       = "FrontView.Special";
    char FrontView_Highlight[]     = "FrontView.Highlight";
    pinMode(REDLED, OUTPUT);
    pinMode(GREENLED, OUTPUT);
    pinMode(BLUELED, OUTPUT);
    pinMode(POWER_OFF_PIN, OUTPUT);
    BlueLedOn();
    NEXTION.begin(921600);      // BAUD rate also set in display code THIS IS THE MAX (was 115200)  
    InitMaxMin();               // in case not yet calibrated
    InitCentreDegrees();        // In case not yet calibrated
    ResetSubTrims();
    CentreTrims();
    WatchDogConfig.window   = WATCHDOGMAXRATE; //  = MINIMUM RATE in milli seconds, (32ms to 522.232s) must be MUCH smaller than timeout
    WatchDogConfig.timeout  = WATCHDOGTIMEOUT; //  = MAX TIMEOUT in milli seconds, (32ms to 522.232s)
    WatchDogConfig.callback = WatchDogCallBack;
    TeensyWatchDog.begin(WatchDogConfig);
    LastDogKick = millis();        // needed? - yes!
    delay (WARMUPDELAY);
    if (!SD.begin(chipSelect)){    // MUST return true or all is lost! (todo: create error page)
       delay (WARMUPDELAY);
       SD.begin(chipSelect);       // a second attempt for iffy sd cards ?!
    }                                
    CalibratedYet = LoadAllParameters();                  // If they exist, read saved SD card settings.   
    if (!CalibratedYet) {Procrastinate(250); CalibratedYet = LoadAllParameters(); }   
    teensyMAC(MacAddress);                                // Get MAC address and use it as pipe address
    NewPipe  = (uint64_t)MacAddress[0] << 40;
    NewPipe += (uint64_t)MacAddress[1] << 32;
    NewPipe += (uint64_t)MacAddress[2] << 24;
    NewPipe += (uint64_t)MacAddress[3] << 16;
    NewPipe += (uint64_t)MacAddress[4] << 8;
    NewPipe += (uint64_t)MacAddress[5];
    Wire.begin();
    ScanI2c();
    if (USE_INA219) ina219.begin();
    InitSwitchesAndTrims();
    InitRadio(DefaultPipe);
    Procrastinate(WARMUPDELAY);                           // Allow Nextion time to warm up
    SendValue(FrontView_BackGround,BackGroundColour);     // Get colours ready
    SendValue(FrontView_ForeGround,ForeGroundColour);
    SendValue(FrontView_Special,SpecialColour);
    SendValue(FrontView_Highlight,HighlightColour);
    SendCommand(page_FrontView);
    SetAudioVolume(AudioVolume);
    if (PlayFanfare){
        PlaySound(THEFANFARE);
        Procrastinate(4000);        // Fanafare takes about 4 seconds
        }
    SendValue(FrontView_Hours, 0);
    SendValue(FrontView_Mins, 0);
    SendValue(FrontView_Secs, 0);
    //  ***************************************************************************************
    //  SetDS1307ToCompilerTime();    //  **   Uncomment this line to set DS1307 clock to compiler's (Computer's) time.        **
    //  **   BUT then re-comment it!! Otherwise it will reset to same time on every boot up! **
    //  ***************************************************************************************
    BoundFlag     = false;
    TxOnTime      = millis();
    StartInactvityTimeout();
    SizeOfCompressedData = sizeof(CompressedData);
    GetTXVersionNumber();
    MySbus.begin();
    SetUKFrequencies(); 
    ScreenTimeTimer = millis();
    RestoreBrightness();
    if (UseLog) {
            LogPowerOn();
            LogThisModel();
    }
}
/*********************************************************************************************************************************/

void GetStatistics()
{
    PacketsPerSecond     = RangeTestGoodPackets;
    RangeTestGoodPackets = 0;
}

/*********************************************************************************************************************************/
/** @returns position of text1 within text2 or 0 if not found */
int InStrng(char * text1, char * text2)
{
    bool flag;
    for (uint16_t j = 0; j < strlen(text2); ++j) {
        flag = false;
        for (uint16_t i = 0; i < strlen(text1); ++i) {
            if (text1[i] != text2[i + j]) {
                flag = true; break;
            }
        }
        if (!flag) return j + 1;
    }
    return 0;
}

/*********************************************************************************************************************************/

void SaveTXStuff()
{
    int  rd  = 0;
    bool EON = false;
    int  j   = 0;
    int  i   = 0;
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
     SDUpdateInt(SDCardAddress,BackGroundColour);
    ++SDCardAddress;
    ++SDCardAddress;
     SDUpdateInt(SDCardAddress,ForeGroundColour);
    ++SDCardAddress;
    ++SDCardAddress;
     SDUpdateInt(SDCardAddress,SpecialColour);
    ++SDCardAddress;
    ++SDCardAddress;
     SDUpdateInt(SDCardAddress,HighlightColour);
    ++SDCardAddress;
    ++SDCardAddress;
     SDUpdateByte(SDCardAddress,SticksMode);
    ++SDCardAddress;
     SDUpdateByte(SDCardAddress,AudioVolume);
    ++SDCardAddress;
     SDUpdateByte(SDCardAddress,Brightness);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress,PlayFanfare);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress,TrimClicks);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress,ButtonClicks);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress,SpeakingClock);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress,AnnounceBanks);
    ++SDCardAddress;
    for (i = 0; i < 8; ++i){
        SDUpdateByte(SDCardAddress,SwitchNumber[i]);
        ++SDCardAddress;
    }
    SDUpdateByte(SDCardAddress,BuddyTriggerChannel);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress,MinimumGap);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress,LogRXSwaps);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress,UseLog);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress,AnnounceConnected); 
    ++SDCardAddress;
    CloseModelsFile();
}

/*********************************************************************************************************************************/

/** MODEL Specific */
void SaveOneModel(uint16_t mnum)
{
    uint16_t j;
    uint16_t i;
    bool EndOfName = false;
    if ((mnum < 1) || (mnum > 99))  return;  // There is no model zero!
    if (!ModelsFileOpen) OpenModelsFile();
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
    for (j = 0; j < FLIGHTMODESUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            SDUpdateByte(SDCardAddress, Trims[j][i]);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < FLIGHTMODESUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            SDUpdateByte(SDCardAddress, TrimsReversed[j][i]);
            ++SDCardAddress;
        }
    }
    SDUpdateByte(SDCardAddress, RXCellCount);
    ++SDCardAddress;
    SDUpdateInt(SDCardAddress, TrimFactor); 
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, LowBattery); 
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, CopyTrimsToAll); 
    ++SDCardAddress;
    for (i = 0; i < CHANNELSUSED; ++i) {
            SDUpdateByte(SDCardAddress,SubTrims[i]);   
            ++SDCardAddress;                
        }
    SDUpdateInt (SDCardAddress,ReversedChannelBITS);
    ++SDCardAddress; 
    ++SDCardAddress; 


    SDCardAddress += 9; // *********************** 9 spare here remaining  **********************


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
    SDUpdateByte(SDCardAddress, SWITCH1Reversed);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, SWITCH2Reversed);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, SWITCH3Reversed);
    ++SDCardAddress;
    SDUpdateByte(SDCardAddress, SWITCH4Reversed);
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
    for (j = 0; j < FLIGHTMODESUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            SDUpdateByte(SDCardAddress, Exponential[j][i]);
            ++SDCardAddress;
        }
    }

    for (j = 0; j < FLIGHTMODESUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            SDUpdateByte(SDCardAddress, InterpolationTypes[j][i]);
            ++SDCardAddress;
        }
    }

    for ( j = 0; j < BYTESPERMACRO; ++j){
        for ( i = 0; i < MAXMACROS; ++i){
                SDUpdateByte(SDCardAddress,MacrosBuffer[i][j]);
                ++SDCardAddress;
        } 
    }
   
     
      


    // ********************** Add more 

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
void BuildDirectory(){
    char MOD[] = ".MOD";
    char Entry1[20];
    char fn[18];
    int i = 0;
    File dir  = SD.open("/");
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
uint16_t WordWrap(char * htext){
char crlf[]= {13,10,0};
char temp1[MAXFILELEN] = "";
char a[]= " ";
uint16_t len = 0;
uint16_t i,j;

        for (i = strlen(htext)-1; i > 1; --i){   
            if ((htext[i] == ' ') || (htext[i] == '-')) break;   // 'i' now has last space pointer
        }
        for (j = 0; j < i; ++j){                                 // get text to i ...
            a[0] = htext[j];
            strcat(temp1,a);
        }
        strcat(temp1, crlf);                                     // add crlf ...
        for (j = i + 1; j < strlen(htext); ++j){                 // then last word on next line by-passing the space
            a[0] = htext[j];
            strcat(temp1,a);
            ++ len;                                              // length onto next line
        }
        strcpy (htext,temp1);
        return len;
}
/*********************************************************************************************************************************/
// This function now scrolls by loading only part of the text file
// This allows very long files to be handled ok.

void ReadTextFile(char* fname, char* htext, uint8_t StartLineNumber, uint8_t MaxLines){
    #define MAXWIDTH 68
    char errormsg[] = "File not found! -> ";
    uint16_t LineCounter        = 0;
    uint16_t StopLineNumber     = 0;
    uint16_t i                  = 0;
    uint8_t  Column             = 0;
    File     fnumber;

    char crlf[]         = {13,10,0};
    char a[]            = " ";
    char dots[]         = "(There's more below ...) ";
    char dots1[]        = "(There's more above ...) ";
    char slash[]        =  "/";
    char OpenBracket[]  = "( ";
    char CloseBracket[] = " )";
    char SearchFile[30];

        StopLineNumber = StartLineNumber + MaxLines;
        strcpy (SearchFile,slash);
        strcat (SearchFile,fname);
        strcpy (htext,OpenBracket);
        strcat (htext,SearchFile);
        strcat (htext,CloseBracket);
        strcat (htext, crlf);
        strcat (htext, crlf);
        if (StartLineNumber > 1) {
            strcat (htext, crlf);
            strcat (htext,dots1);
            strcat (htext, crlf);
        }
        
        ThereIsMoreToSee = false;
        fnumber  = SD.open(SearchFile, FILE_READ); 
        if (fnumber) {
            while (fnumber.available() && i < (MAXFILELEN-10)) {
                a[0] = fnumber.read();                             //  Read in one byte at a time.
                 if (a[0] == '|') {                                //  New Line character = '|'   
                    if ((LineCounter >= StartLineNumber) && (LineCounter <= StopLineNumber)){
                        strcat(htext, crlf);
                        ++i;
                        ++i;
                    }
                    Column = 0;
                    ++LineCounter;         
                    a[0] = 34; // a '34'  ( " ) is ignored.
                }
                if (Column >= MAXWIDTH) {
                    Column = WordWrap(htext);
                    ++LineCounter;
                }       
                if ((a[0] == 13) || (a[0] == 10)) a[0] = 34;      // Ignore CrLfs
                if (a[0] != 34) {
                    if ((LineCounter >= StartLineNumber) && (LineCounter <= StopLineNumber)){
                        strcat(htext, a);
                        ++ Column;
                        ++ i;
                    }
                }
                if (LineCounter > StopLineNumber){
                    strcat (htext, crlf);
                    strcat (htext,dots);
                    ThereIsMoreToSee = true;
                    break;
                } 
            }
        }
        else 
        {
            strcpy(htext, errormsg);
            strcat(htext,fname);
        }
    fnumber.close();
}

/*********************************************************************************************************************************/
void ScrollHelpFile(){                 // redisplays help file to scroll it ... maybe from top of file
  char HelpText[MAXFILELEN + 10];      // MAX = 3K or so
  char HelpView[] = "HelpText";
    ReadTextFile(RecentTextFile,HelpText,RecentStartLine,MAXLINES);    // Then load help text
    SendText1(HelpView, HelpText);                  // Then send it
}
/*********************************************************************************************************************************/
void SendHelp(){  // load new help file
    char hcmd[] = "page HelpView";
    char HelpFile[20];
    int i = 9;
    int j = 0;
    SendCommand(hcmd);                   // first load the right screen
    while (TextIn[i] != 0 && j < 19) {
        HelpFile[j] = TextIn[i];
        ++i;
        ++j;
        HelpFile[j] = 0;
    }
    strcpy(RecentTextFile,HelpFile);
    RecentStartLine = 0;
    ScrollHelpFile();
}
/*********************************************************************************************************************************/
/** @brief Discover which channel to setup */
int GetChannel()
{
    unsigned int i ;
    for (i = 0; i < sizeof(TextIn); ++i) {
        if (isdigit(TextIn[i])) break;
    }
    return atoi(&TextIn[i]);
}
/*********************************************************************************************************************************/

void UpdateSwitchesDisplay()
{
    char SwitchesView_sw1[] = "sw1";
    char SwitchesView_sw2[] = "sw2";
    char SwitchesView_sw3[] = "sw3";
    char SwitchesView_sw4[] = "sw4";
    char NotUsed[]          = "Not used";
    char FlightModes123[]   = "Banks 1 - 2 - 3";
    char Auto[]             = "Auto (Bank 4)";
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
    while (uint8_t(TextIn[k]) > 0) {
        ChannelNames[ch - 1][j] = TextIn[k];
        ++j;
        ++k;
        ChannelNames[ch - 1][j] = 0;
    }
    SaveOneModel(ModelNumber);
}

/*********************************************************************************************************************************/

/** @brief updates display in textbox */
void ShowFileNumber()
{
    char ModelsView_filename[] = "filename";
    char newfname[17];
    if (FileNumberInView >= ExportedFileCounter) FileNumberInView = 0;
    if (FileNumberInView < 0) FileNumberInView = ExportedFileCounter - 1;
    for (int i = 0; i < 12; ++i) {
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
    for (int pp = 0; pp < 3; ++pp) {
        SendCommand(ErrorOn);
        Procrastinate(200);
        SendCommand(ErrorOff);
        Procrastinate(100);
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
    for (int i = 0; i < ExportedFileCounter; ++i) {
        nlp = 13;
        for (int z = 0; z < 12; ++z) {
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
            for (int q = 0; q < nlp; ++q) {
                strcat(filelistbuf, space);
            }
        }
    }
    SendText1(fileviewlist, filelistbuf);
}

/*********************************************************************************************************************************/

void SetDefaultValues()
{
    uint16_t  j;
    uint16_t  i;
    char ProgressStart[]                       = "vis Progress,1";
    char ProgressEnd[]                         = "vis Progress,0";
    char Progress[]                            = "Progress";
   
    char DefaultChannelNames[CHANNELSUSED][11] = {{"Aileron"}, {"Elevator"}, {"Throttle"}, {"Rudder"}, {"Ch 5"}, {"Ch 6"}, {"Ch 7"}, {"Ch 8"}, {"Ch 9"}, {"Ch 10"}, {"Ch 11"}, {"Ch 12"}, {"Ch 13"}, {"Ch 14"}, {"Ch 15"}, {"Ch 16"}};
    SendCommand(ProgressStart);
    SendValue(Progress, 5);
    Procrastinate(10);

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
    Procrastinate(10);
    for (j = 0; j < MAXMIXES; ++j) {
        for (i = 0; i < CHANNELSUSED; ++i) {
            Mixes[j][i] = 0;
        }
    }
    SendValue(Progress, 25);
    Procrastinate(10);

    for (j = 0; j < FLIGHTMODESUSED + 1; ++j) { // must have fudged this somewhere.... 5?!
        for (i = 0; i < CHANNELSUSED; ++i) {
            Trims[j][i]         = 80; // MIDPOINT is 80 !
            TrimsReversed[j][i] = 0;
        }
    }
    RXCellCount = 3;

    SendValue(Progress, 45);
    Procrastinate(10);
    for (i = 0; i < CHANNELSUSED; ++i) {
        InPutStick[i] = i;
    }
    FMSwitch        = 4;
    AutoSwitch      = 1;
    Channel9Switch  = 2;
    Channel10Switch = 3;
    Channel11Switch = 0;
    Channel12Switch = 0;
    SWITCH1Reversed = false;
    SWITCH2Reversed = false;
    SWITCH3Reversed = false;
    SWITCH4Reversed = false;
    SendValue(Progress, 65);
    Procrastinate(10);
    for (i = 0; i < CHANNELSUSED; ++i) {
        FailSafeChannel[i] = false;
    }
    SendValue(Progress, 75);
    Procrastinate(10);
    for (i = 0; i < CHANNELSUSED; ++i) {
        for (j = 0; j < 10; ++j) {
            ChannelNames[i][j] = DefaultChannelNames[i][j];
        }
    }

    for (j = 0; j < FLIGHTMODESUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            Exponential[j][i] = DEFAULT_EXPO;     // 0% (50) expo = default
        }
    }
    for (j = 0; j < FLIGHTMODESUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            InterpolationTypes[j][i] = EXPONENTIALCURVES;        // Expo is default
        }
    }

    for (i = 0; i < CHANNELSUSED + 1; ++i) {
            SubTrims[i] = 127;                    // centre (0 - 254)
        }
    for (j = 0; j < BYTESPERMACRO; ++j){
        for (i = 0; i < MAXMACROS; ++i){
                MacrosBuffer[i][j] = 0;
        } 
    }
    SendValue(Progress, 95);
    Procrastinate(10);
    SaveOneModel(ModelNumber);
    SendValue(Progress, 100);
    ReversedChannelBITS = 0;                        // No channel reversed
    SDCardAddress = TXSIZE;                         //  spare bytes for TX stuff
    SDCardAddress += (ModelNumber - 1) * MODELSIZE; //  spare bytes for Model params
    StartLocation = SDCardAddress;
    ModelDefined  = 0;
    OpenModelsFile();
    SDUpdateByte(SDCardAddress, ModelDefined);      // mark this model as undefined
    CloseModelsFile();
    ReadOneModel(ModelNumber);  // not this
    UpdateModelsNameEveryWhere();
    Procrastinate(100);
    SendCommand(ProgressEnd);
}

/*********************************************************************************************************************************/

void ClearBox()
{
    char nb[10];
    char cmd[50];
    char fillcmd[] = "fill 30,30,380,365,";
    strcpy(cmd,fillcmd);
    Str(nb,BackGroundColour,0);
    strcat(cmd,nb);
    SendCommand(cmd);
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
void FillBox(int x1, int y1, int w, int h, int c)
{
    char line[] = "fill ";
    char nb[12];
    char cb[50];
    char comma[] = ",";
    strcpy(cb, line);
    strcat(cb, Str(nb, x1, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, y1, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, w, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, h, 0));
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
FASTRUN void DrawBox(int x1, int y1, int x2, int y2, int c)
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
FASTRUN void GetDotPositions()
{
    int p      = 0;
    BoxLeft    = BOXOFFSET;
    BoxTop     = BOXOFFSET;
    BoxRight   = BOXOFFSET + BOXSIZE;
    BoxBottom  = BoxRight;
    xPoints[0] = BoxLeft;
    xPoints[4] = BoxRight - BOXOFFSET;
    p          = map(MinDegrees[FlightMode][ChanneltoSet - 1], 0, 180, BOXSIZE, BOXOFFSET);
    yPoints[0] = constrain(p, 39, 391);
    p          = map(MidLowDegrees[FlightMode][ChanneltoSet - 1], 0, 180, BOXSIZE, BOXOFFSET);
    yPoints[1] = constrain(p, 39, 391);
    xPoints[1] = BOXOFFSET + 90;
    xPoints[2] = BOXOFFSET + 180;
    p          = map(CentreDegrees[FlightMode][ChanneltoSet - 1], 0, 180, BOXSIZE, BOXOFFSET);
    yPoints[2] = constrain(p, 39, 391);
    xPoints[3] = BOXOFFSET + 270;
    p          = map(MidHiDegrees[FlightMode][ChanneltoSet - 1], 0, 180, BOXSIZE, BOXOFFSET);
    yPoints[3] = constrain(p, 39, 391);
    p          = map(MaxDegrees[FlightMode][ChanneltoSet - 1], 0, 180, BOXSIZE, BOXOFFSET);
    yPoints[4] = constrain(p, 39, 391);
}

/*********************************************************************************************************************************/

int DegsToPercent(int degs)
{
    return map(degs, 0, 180, -100, 100);
}

/*********************************************************************************************************************************/

FASTRUN void DrawDot(int xx, int yy, int rad, int colr)
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

FASTRUN void updateInterpolationTypes()
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
        case STRAIGHTLINES:
            SendValue(Lines, 1);
            SendValue(Smooth, 0);
            SendValue(ExpR, 0);
            SendCommand(t3off);
            SendCommand(b13off);
            SendCommand(b12off);
            SendCommand(ExponOff);
            SendCommand(Ex1Off);
            break;
        case SMOOTHEDCURVES:
            SendValue(Lines, 0);
            SendValue(Smooth, 1);
            SendValue(ExpR, 0);
            SendCommand(t3off);
            SendCommand(b13off);
            SendCommand(b12off);
            SendCommand(ExponOff);
            SendCommand(Ex1Off);
            break;
        case EXPONENTIALCURVES:
            SendValue(Lines, 0);
            SendValue(Smooth, 0);
            SendValue(ExpR, 1);
            SendCommand(t3on);
            SendCommand(b13on);
            SendCommand(b12on);
            SendCommand(ExponOn);
            SendCommand(Ex1On);
            SendValue (Ex1,     (Exponential[FlightMode][ChanneltoSet - 1]));           // The slider 
            SendValue (Expon,   (Exponential[FlightMode][ChanneltoSet - 1])-50);        // the number
            break;
        default:
            break;
    }
}

/*********************************************************************************************************************************/

FASTRUN void DisplayCurve()
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
    int   DotColour = HighlightColour; 
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
    DrawBox(BoxLeft, BoxTop, BoxRight - BoxLeft, BoxBottom - BoxTop, HighlightColour);

    xDot1 = xPoints[0];
    yDot1 = ((BoxBottom - BoxTop) / 2) + 20; // ?
    xDot2 = BoxRight - BOXOFFSET;
    yDot2 = yDot1;
    DrawLine(xDot1, yDot1, xDot2, yDot1, SpecialColour);

    xDot1 = xPoints[2];
    yDot1 = BoxTop;
    xDot2 = xDot1;
    yDot2 = BoxBottom - BOXOFFSET;
    DrawLine(xDot1, yDot1, xDot2, yDot2, SpecialColour);

    if (InterpolationTypes[FlightMode][ChanneltoSet - 1] == STRAIGHTLINES) {                // Linear
        SendCommand(b3on);
        SendCommand(b4on);
        SendCommand(b7on);
        SendCommand(b8on);
        SendCommand(n2on);
        SendCommand(n4on);
        DrawLine(xPoints[0], yPoints[0], xPoints[1], yPoints[1], ForeGroundColour); // this adds the straight version of the curve
        DrawLine(xPoints[1], yPoints[1], xPoints[2], yPoints[2], ForeGroundColour);
        DrawLine(xPoints[2], yPoints[2], xPoints[3], yPoints[3], ForeGroundColour);
        DrawLine(xPoints[3], yPoints[3], xPoints[4], yPoints[4], ForeGroundColour);
    }
      //  delay(250);
    if (InterpolationTypes[FlightMode][ChanneltoSet - 1] == SMOOTHEDCURVES) {                // CatmullSpline
       
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
            DrawLine(xDot1, yDot1, xDot2, yDot2, ForeGroundColour);
            xDot2 = xDot1;
            yDot2 = yDot1;
        }
    }

    if (InterpolationTypes[FlightMode][ChanneltoSet - 1] == EXPONENTIALCURVES) { //EXPO  ************************************************************************************************
#define APPROXIMATION 7                                         // This is for the approximation of the screen curve

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
            yPoint = MapWithExponential(HalfXRange - xPoint, HalfXRange, 0, 0, BottomHalfYRange, Exponential[FlightMode][ChanneltoSet - 1]);
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
            DrawLine(xDot1, yDot1, xDot2, yDot2, ForeGroundColour); // Draw short line from this point to previous point
            xDot2 = xDot1;
            yDot2 = yDot1;
        }
        Step  = APPROXIMATION;
        yDot2 = 0;
        for (xPoint = HalfXRange; xPoint >= 0; xPoint -= Step) { // Simulate a curve with many short lines to speed it up
            yPoint = MapWithExponential(xPoint, 0, HalfXRange, 0, TopHalfYRange, Exponential[FlightMode][ChanneltoSet - 1]);
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
            DrawLine(xDot1, yDot1, xDot2, yDot2, ForeGroundColour); // Draw short line from this point to previous point
            xDot2 = xDot1;
            yDot2 = yDot1;
        }
        DrawDot(xPoints[0], yPoints[0], DotSize, DotColour); // This adds 3 dots
        DrawDot(xPoints[2], yPoints[2], DotSize, DotColour);
        DrawDot(xPoints[4], yPoints[4], DotSize, DotColour);
    }
    if (InterpolationTypes[FlightMode][ChanneltoSet - 1] != EXPONENTIALCURVES) {
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
    if (XtouchPlace > BoxRight - BOXOFFSET) return;  // out of range
    if (XtouchPlace < BOXOFFSET) return;             // out of range
    if (YtouchPlace < BoxTop) return;                // out of range
    if (YtouchPlace > BoxBottom - BOXOFFSET) return; // out of range

    if (XtouchPlace < BOXOFFSET+ xPoints[0]) { // do leftmost point  ?
        rjump = GetDifference(YtouchPlace, yPoints[0]);
        if (YtouchPlace > yPoints[0]) {
            if (MinDegrees[FlightMode][ChanneltoSet - 1] >= rjump) MinDegrees[FlightMode][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (MinDegrees[FlightMode][ChanneltoSet - 1] <= (180 - rjump)) MinDegrees[FlightMode][ChanneltoSet - 1] += rjump;
        }
    }

    if (XtouchPlace > xPoints[1] - BOXOFFSET && XtouchPlace < xPoints[1] + BOXOFFSET) { // do next point  ?
        if (InterpolationTypes[FlightMode][ChanneltoSet - 1] == EXPONENTIALCURVES) return;              //  expo = ignore this area
        rjump = GetDifference(YtouchPlace, yPoints[1]);
        if (YtouchPlace > yPoints[1]) {
            if (MidLowDegrees[FlightMode][ChanneltoSet - 1] >= rjump) MidLowDegrees[FlightMode][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (MidLowDegrees[FlightMode][ChanneltoSet - 1] <= 180 - rjump) MidLowDegrees[FlightMode][ChanneltoSet - 1] += rjump;
        }
    }

    if (XtouchPlace > xPoints[2] - BOXOFFSET && XtouchPlace < xPoints[2] + BOXOFFSET) { // do next point  ?
        rjump = GetDifference(YtouchPlace, yPoints[2]);
        if (YtouchPlace > yPoints[2]) {
            if (CentreDegrees[FlightMode][ChanneltoSet - 1] >= rjump) CentreDegrees[FlightMode][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (CentreDegrees[FlightMode][ChanneltoSet - 1] <= 180 - rjump) CentreDegrees[FlightMode][ChanneltoSet - 1] += rjump;
        }
    }

    if (XtouchPlace > xPoints[3] - BOXOFFSET && XtouchPlace < xPoints[3] + BOXOFFSET) { // do next point  ?
        if (InterpolationTypes[FlightMode][ChanneltoSet - 1] == EXPONENTIALCURVES) return;              //  expo = ignore this area
        rjump = GetDifference(YtouchPlace, yPoints[3]);
        if (YtouchPlace > yPoints[3]) {
            if (MidHiDegrees[FlightMode][ChanneltoSet - 1] >= rjump) MidHiDegrees[FlightMode][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (MidHiDegrees[FlightMode][ChanneltoSet - 1] <= 180 - rjump) MidHiDegrees[FlightMode][ChanneltoSet - 1] += rjump;
        }
    }
    if (XtouchPlace > xPoints[3] + BOXOFFSET) // do hi point  ?
    {
        rjump = GetDifference(YtouchPlace, yPoints[4]);
        if (YtouchPlace > yPoints[4]) {
            if (MaxDegrees[FlightMode][ChanneltoSet - 1] > rjump) MaxDegrees[FlightMode][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (MaxDegrees[FlightMode][ChanneltoSet - 1] <= 180 - rjump) MaxDegrees[FlightMode][ChanneltoSet - 1] += rjump;
        }
    }
}


/*********************************************************************************************************************************/

void NormaliseTheRadio() {
              SetThePipe(DefaultPipe); 
              Radio1.setCRCLength(RF24_CRC_8);
              Radio1.setRetries(RETRYCOUNT, RETRYWAIT);
}


/*********************************************************************************************************************************/
void ShowFileProgress(char * Msg){
char t1[] = "t1";
    SendText(t1,Msg);
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
    uint64_t      RXPipe;
    uint32_t      RXTimer = 0;
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
    char          nb1[20];
    char          Received[] = "Received ";
    char          of[]   = " of ";
    char          msg[50];  
    char          bytes[] = " bytes.";
   
#ifdef DB_MODEL_EXCHANGE
    uint8_t PacketNumber = 0;
    Serial.println("Receiving model ...");
    Serial.println(Waiting);
#endif
    BlueLedOn();
    SendText(ModelsView_filename, Waiting);
    RXPipe = FILEPIPEADDRESS;
    Radio1.setRetries(15, 15);  
    Radio1.setChannel(FILECHANNEL);
    Radio1.flush_tx();
    Radio1.openReadingPipe(1, RXPipe);
    Radio1.startListening();
    RXTimer = millis();           // Start timer
    while (!Radio1.available()) { // Await the sender....
          Procrastinate(5);
          if (GetButtonPress()){
              NormaliseTheRadio();
              RedLedOn();
              ButtonWasPressed();
              return;
           }
        KickTheDog(); // Watchdog
        if ((millis() - RXTimer) / 1000 >= FILETIMEOUT) {
#ifdef DB_MODEL_EXCHANGE
            Serial.println("Timeout");
#endif
            SendText(ModelsView_filename, TimeoutMsg);
            NormaliseTheRadio();
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
    Radio1.writeAckPayload(1, &Fack, sizeof(Fack)); //  Ack first packet
    Radio1.read(&Fbuffer, BUFFERSIZE + 4);          //  Read it was 12
    strcpy(SingleModelFile, Fbuffer);               //  Get filename
    Fsize = Fbuffer[BUFFERSIZE];
    Fsize += Fbuffer[BUFFERSIZE + 1] << 8;
    Fsize += Fbuffer[BUFFERSIZE + 2] << 16;
    Fsize += Fbuffer[BUFFERSIZE + 3] << 24;         //  Get file size
#ifdef DB_MODEL_EXCHANGE
    Serial.println("CONNECTED!");
    Serial.print("FileName=");
    Serial.println(SingleModelFile);
    Serial.print("File size = ");
    Serial.println(Fsize);
#endif
    Fposition        = 0;
    ModelsFileNumber = SD.open(SingleModelFile, FILE_WRITE);                    //  Open file to receive
    RXTimer          = millis();                                                //  zero timeout
    while ((Fposition < Fsize) && (millis() - RXTimer) / 1000 <= FILETIMEOUT) { //  (Fposition<Fsize) ********************
        KickTheDog();                                                           //  Watchdog
        if (Radio1.available()) {
            Radio1.flush_tx();
            Radio1.writeAckPayload(1, &Fack, sizeof(Fack));
            Radio1.read(&Fbuffer, BUFFERSIZE + 4);
            ModelsFileNumber.seek(Fposition);            // Move filepointer IS NEEDED!!
            ModelsFileNumber.write(Fbuffer, BUFFERSIZE); // Write part of file
            Radio1.flush_rx();
            Fposition += BUFFERSIZE;
            if (Fposition > Fsize) Fposition = Fsize;
            p = ((float)Fposition / (float)Fsize) * 100;
            SendValue(Progress, p);
            strcpy (msg,Received);
            strcat(msg, Str(nb1,Fposition,0));
            strcat(msg, of);
            strcat(msg, Str(nb1,Fsize,0));
            ShowFileProgress(msg);
#ifdef DB_MODEL_EXCHANGE
            PacketNumber = Fbuffer[25];
            Serial.print("PacketNumber: ");
            Serial.println(PacketNumber);
#endif
        }
    }
    SendValue(Progress, 100);
    ModelsFileNumber.close();
    BuildDirectory();
    SendText(ModelsView_filename, Success);
    Procrastinate(750);
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
    NormaliseTheRadio();
    SendCommand(ProgressEnd);
    RedLedOn();
    strcpy (msg,Received);
    strcat(msg, Str(nb1,Fsize,0));  
    strcat(msg,bytes);     
    ShowFileProgress(msg);
    PlaySound(BEEPCOMPLETE);
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
    char          nb1[20];
    char          Sent[] = "Sent ";
    char          of[]   = " of ";
    char          msg[50];  
    char          bytes[] = " bytes.";
    uint32_t      SentMoment = 0;
  
    BlueLedOn();
    SendCommand(ProgressStart);
    SendValue(Progress, p);
    Procrastinate(10);
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
    Procrastinate(4);
    while (Fposition < Fsize) {
        KickTheDog(); // Watchdog
        p = ((float)Fposition / (float)Fsize) * 100;
        strcpy (msg,Sent);
        strcat(msg, Str(nb1,Fposition,0));
        strcat(msg, of);
        strcat(msg, Str(nb1,Fsize,0));
        ShowFileProgress(msg);
        SendValue(Progress, p);
        PacketNumber++;
        if (PacketNumber == 1) {
            strcpy(Fbuffer, SingleModelFile);      // Filename in first packet
            Fbuffer[BUFFERSIZE]     = Fsize;
            Fbuffer[BUFFERSIZE + 1] = Fsize >> 8;
            Fbuffer[BUFFERSIZE + 2] = Fsize >> 16;
            Fbuffer[BUFFERSIZE + 3] = Fsize >> 24; // SEND FILE SIZE (four bytes)
        }
        else {
            ModelsFileNumber.seek(Fposition);           // Move filepointer
            ModelsFileNumber.read(Fbuffer, BUFFERSIZE); // Read part of file
            Fposition += BUFFERSIZE;
            if (Fposition > Fsize) Fposition = Fsize;
        }
       while ((millis()-SentMoment) < PACEMAKER * 2) {   
                Radio1.flush_tx();
                Radio1.flush_rx();
       }
        if (Radio1.write(&Fbuffer, BUFFERSIZE + 4)) {
            SentMoment = millis();
            Procrastinate(1);     
            if (Radio1.isAckPayloadAvailable()) {
                Radio1.read(&Fack, sizeof(Fack));
                Serial.println ("ACK received");
                Serial.println (Fposition);
            }else{
                Serial.println ("NO ACK received");
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
    Procrastinate(750);
    NormaliseTheRadio();
    SendCommand(ProgressEnd);
    RedLedOn();
    strcpy (msg,Sent);
    strcat (msg, Str(nb1,Fsize,0));
    strcat (msg,bytes);
    ShowFileProgress(msg);
    PlaySound(BEEPCOMPLETE);
}

/*********************************************************************************************************************************/

void SoundFlightMode()
{
    switch (FlightMode) {
        case 1:
            PlaySound(BANKONE);
            break;
        case 2:
            PlaySound(BANKTWO);
            break;
        case 3:
             PlaySound(BANKTHREE);
            break;
        case 4:
            PlaySound(BANKFOUR);
            break;
        default:
            break;
    }
    ScreenTimeTimer = millis();  // reset screen counter
    if (ScreenIsOff) {
       RestoreBrightness();
       ScreenIsOff = false;
    }

}
/*********************************************************************************************************************************/

void ShowFlightMode() 
{
    char FMPress1[]  = "click fm1,1";
    char FMPress2[]  = "click fm2,1";
    char FMPress3[]  = "click fm3,1";
    char FMPress4[]  = "click fm4,1";

    switch (FlightMode) {
        case 1:
            SendCommand(FMPress1);
            break;
        case 2:
            SendCommand(FMPress2);
            break;
        case 3:
            SendCommand(FMPress3);
            break;
        case 4:
            SendCommand(FMPress4);
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
        if (SWITCH1Reversed) SendValue(OneSwitchViewc_revd, 1);
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
        if (SWITCH2Reversed) SendValue(OneSwitchViewc_revd, 1);
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
        if (SWITCH3Reversed) SendValue(OneSwitchViewc_revd, 1);
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
        if (SWITCH4Reversed) SendValue(OneSwitchViewc_revd, 1);
    }
    SendValue(SwNum, SwitchEditNumber); // show switch number
}

/*********************************************************************************************************************************/

void RestoreBrightness(){
    char cmd[20];
    char dim[] = "dim=";
    char nb[10];
    if (Brightness < 10) Brightness = 10;
    strcpy(cmd,dim);
    Str(nb,Brightness,0);
    strcat(cmd,nb);
    SendCommand(cmd);
    ScreenTimeTimer = millis();  // reset screen counter
}
/*********************************************************************************************************************************/

void ZeroDataScreen(){             // ZERO Those parameters that are zeroable
            LostPackets        = 0;
            GapLongest         = 0;
            GapSum             = 0;
            GapAverage         = 0;
            GapCount           = 0;
            GapStart           = 0;
            RXMAXModelAltitude = 0;
            GPSMaxAltitude     = 0;
            ThisGap            = 0;
            GPSMaxDistance     = 0;
            GPSMaxSpeed        = 0;
            SavedRadioSwaps    = RadioSwaps;       // Cannot easily zero these, so do a subtraction
            SavedRX1TotalTime  = RX1TotalTime;
            SavedRX2TotalTime  = RX2TotalTime;
            SavedSbusRepeats   = SbusRepeats;
}
/***************************************************** ShowChannelName ****************************************************************************/

void ShowChannelName(){
    char MoveToChannel[]    =   "Mch";
    char MacrosView_chM[]   =   "chM";
    uint8_t ch = GetValue(MoveToChannel);
    if (ch > 0) -- ch;                 // no zero
    SendText(MacrosView_chM,ChannelNames[ch]);
}
/***************************************************** Populate Macros View ****************************************************************************/

void PopulateMacrosView(){ 
    char MacroNumber[]      =   "Mno";
    char TriggerChannel[]   =   "Tch";
    char MoveToChannel[]    =   "Mch";
    char MoveToPosition[]   =   "Pos";
    char Delay[]            =   "Del";
    char Duration[]         =   "Dur";
    uint8_t n               =   PreviousMacroNumber;
   
    if (n < 8) {                                                                        // Read previous values before moveing to next  
        MacrosBuffer[n][MACROTRIGGERCHANNEL]    = GetValue(TriggerChannel);
        MacrosBuffer[n][MACROMOVECHANNEL]       = GetValue(MoveToChannel);
        MacrosBuffer[n][MACROMOVETOPOSITION]    = GetValue(MoveToPosition);
        MacrosBuffer[n][MACROSTARTTIME]         = GetValue(Delay);
        MacrosBuffer[n][MACRODURATION]          = GetValue(Duration);
    }
    n = GetValue(MacroNumber)-1;
    SendValue(TriggerChannel,MacrosBuffer[n][MACROTRIGGERCHANNEL]);
    SendValue(MoveToChannel,MacrosBuffer[n][MACROMOVECHANNEL]);
    SendValue(MoveToPosition,MacrosBuffer[n][MACROMOVETOPOSITION]);
    SendValue(Delay,MacrosBuffer[n][MACROSTARTTIME]);
    SendValue(Duration,MacrosBuffer[n][MACRODURATION]);
    ShowChannelName();
    PreviousMacroNumber = n;
}

/*********************************************************************************************************************************/

void ExitMacrosView(){
    char pSetupView[]       = "page SetupView";
    char MacroNumber[]      =   "Mno";
    char TriggerChannel[]   =   "Tch";
    char MoveToChannel[]    =   "Mch";
    char MoveToPosition[]   =   "Pos";
    char Delay[]            =   "Del";
    char Duration[]         =   "Dur";
    uint8_t n = GetValue(MacroNumber)-1;
    MacrosBuffer[n][MACROTRIGGERCHANNEL]    = GetValue(TriggerChannel);
    MacrosBuffer[n][MACROMOVECHANNEL]       = GetValue(MoveToChannel);
    MacrosBuffer[n][MACROMOVETOPOSITION]    = GetValue(MoveToPosition);
    MacrosBuffer[n][MACROSTARTTIME]         = GetValue(Delay);
    MacrosBuffer[n][MACRODURATION]          = GetValue(Duration);
    UseMacros = true;
    SaveOneModel(ModelNumber);
    b5isGrey = false;
    SendCommand (pSetupView);
    CurrentView =  MAINSETUPVIEW;
}

/*********************************************************************************************************************************/

void  EndReverseView(){ // channel reverse flags are 16 individual BITs in var 'ReversedChannelBITS'
        char fs[16][5]                 = {"fs1","fs2","fs3","fs4","fs5","fs6","fs7","fs8","fs9","fs10","fs11","fs12","fs13","fs14","fs15","fs16"};
        uint8_t i;    
        char pSetupView[]              = "page SetupView";
        char ProgressStart[]           = "vis Progress,1";
        char ProgressEnd[]             = "vis Progress,0";
        char Progress[]                = "Progress";
        SendCommand(ProgressStart);
        ReversedChannelBITS = 0;
        for (i = 0; i < 16 ; ++i){
             SendValue(Progress, (i * (100/16)));
             if (GetValue(fs[i])) ReversedChannelBITS |= 1 << i; // set a BIT 
        }
        CurrentView = MAINSETUPVIEW;
        SaveOneModel(ModelNumber);
        SendCommand(ProgressEnd);
        b5isGrey = false;
        SendCommand(pSetupView);
}

/*********************************************************************************************************************************/

void  StartReverseView(){  // channel reverse flags are 16 individual BITs in ReversedChannelBITS
    char pReverseView[] = "page ReverseView";
    char fs[16][5]      = {"fs1","fs2","fs3","fs4","fs5","fs6","fs7","fs8","fs9","fs10","fs11","fs12","fs13","fs14","fs15","fs16"};
    uint8_t i;    
    char Progress[]                = "Progress";
    char ProgressStart[]           = "vis Progress,1";
    char ProgressEnd[]             = "vis Progress,0";
    CurrentView = REVERSEVIEW;
    SendCommand(pReverseView);
    UpdateButtonLabels();
    SendCommand(ProgressStart);
        for (i = 0; i < 16 ; ++i){
             SendValue(Progress, (i *(100/16)));
            if (ReversedChannelBITS & 1 << i){  // is BIT set?? 
                SendValue(fs[i],1);
            }else{
                SendValue(fs[i],0);
            }
        }
    SendCommand(ProgressEnd);
}

/*********************************************************************************************************************************/

void StartBuddyView(){
    char BuddyM[]                  = "BuddyM";
    char BuddyP[]                  = "BuddyP"; 
    char n0[]                      = "n0";
    char pBuddyView[]  = "page BuddyView";
    SendCommand(pBuddyView);
    CurrentView = BUDDYVIEW;
    delay (200);
    if (BuddyTriggerChannel > 16) BuddyTriggerChannel = 16;
    if (BuddyTriggerChannel < 1) BuddyTriggerChannel = 12;
    SendValue(BuddyM, BuddyMaster);
    SendValue(BuddyP, DoSbusSendOnly);
    SendValue(n0,BuddyTriggerChannel);
}

/*********************************************************************************************************************************/

void  EndBuddyView(){
    char BuddyM[]                  = "BuddyM";
    char BuddyP[]                  = "BuddyP";
    char n0[]                      = "n0";

    char pSetupView[] = "page SetupView";
    DoSbusSendOnly      = GetValue(BuddyP);  // Pupil, wired
    BuddyMaster         = GetValue(BuddyM);  // Master, either.
    BuddyTriggerChannel = GetValue(n0);
    if (BuddyTriggerChannel > 16) BuddyTriggerChannel = 16;
    if (BuddyTriggerChannel < 1) BuddyTriggerChannel = 12;

    SaveAllParameters();
    b5isGrey = false;
    SendCommand(pSetupView);
    CurrentView = MAINSETUPVIEW;
}
/*********************************************************************************************************************************/

FASTRUN void  DoNumberedCommands(uint8_t nc){ // These gradually are replacing word-invoked commands for speed and economy
  
    char pModelsView[]  = "page ModelsView";
    char mn[]           = "ModelNumber";
    char pMacrosView[]  = "page MacrosView";
    
#ifdef DB_NEXTION
    Serial.print ("Command number: -> ");
    Serial.println (nc);
#endif
    
    b5isGrey = false;

    switch(nc){
          case 1:                      // Previous file (modelsview)
            FileNumberInView--;
            ShowFileNumber();
            CloseModelsFile();
            break;
        case 2:                        // Next file (modelsview)
            FileNumberInView++;
            ShowFileNumber();
            CloseModelsFile();
            break;
        case 3:                    
            ModelNameTimeCheck = 0 ;
            break;
        case 4:                              // goto models view   
            SendCommand(pModelsView);
            CurrentView = MODELSVIEW;
            UpdateModelsNameEveryWhere();
            BuildDirectory();                 // of SD card
            ShowFileNumber();
            SendValue(mn,ModelNumber);
            break;
        case 5:
            PreviousMacroNumber = 200;        // i.e. no usable number
            SendCommand(pMacrosView);         // Display MacroView
            CurrentView = MACROS_VIEW;
            Procrastinate(200);               // allow enough time for screen to display
            PopulateMacrosView();
            break;
        case 6:
            PopulateMacrosView();             // Macros view number has moved to new macro
            break;
        case 7:
            ExitMacrosView();
            break;
        case 8:
            ShowChannelName();
            break;
        case 9:
            StartReverseView();
            break;
        case 10:
            EndReverseView();
            break;
        case 11:
            StartBuddyView();
            break;
        case 12:
            EndBuddyView();
            break;

        default:
           break;
    }
    ClearText();
}


/*********************************************************************************************************************************/

FASTRUN void DisplayCurveAndServoPos(){
            DisplayCurve();
            SavedLineX = 0;  
            ShowServoPos(); 
            ClearText();
}

/*********************************************************************************************************************************/

/**
 * BUTTON WAS PRESSED (DEAL WITH INPUT FROM NEXTION DISPLAY)
 *
 */
FASTRUN void ButtonWasPressed()
{
  if (TextIn[0] & 128 ){                                // first byte hi bit indicates a numbered command
          DoNumberedCommands(TextIn[0] & 127);          // send number with the high bit off.
          return;                                       // skip the rest!
  }         

    int i = 0;
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
    char Data_View[]               = "DataView";
    char CalibrateView[]           = "CalibrateView";
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
    char page_AudioView[]          = "page AudioView";
    char page_ColoursView[]        = "page ColoursView";
    char GoSetupView[]             = "GoSetupView";
    char ColoursView[]             = "ColoursView";
    char GoFrontView[]             = "GoFrontView";
    char SvT11[]                   = "t11";
    char CMsg1[]                   = "Move all controls to their full\r\nextent several times,\r\nthen press Next.";
    char SvB0[]                    = "b0";
    char CMsg2[]                   = "Next ...";
    char Cmsg3[]                   = "Centre all channels,\r\nPush edge switches fully back,\r\nthen press Finish.";
    char Cmsg4[]                   = "Finish";
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
    char PageFilesView[]           = "page FilesView";
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
    char StillConnectedBox[]       = "StillConnected";
    char StillConnectedMsg[]       = "Disconnect RX!";
    char NotStillConnected[]       = "vis StillConnected,0";
    char OptionsViewS[]            = "OptionsViewS";
    char Pto[]                     = "Pto";
    char Tx_Name[]                 = "TxName";
    char Exrite[]                  = "Exrite";
    char ExpR[]                    = "Exp";
    char Smooth[]                  = "Smooth";
    char Lines[]                   = "Lines";
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
    char pTrimView[]               = "page TrimView";
    char pMixesView[]              = "page MixesView";
    char pTypeView[]               = "page TypeView";
    char pCalibrateView[]          = "page CalibrateView";
    char pFailSafe[]               = "page FailSafeView";
    char pSubTrimView[]            = "page SubTrimView";
    char pLogView[]                = "page LogView"; 
    char DataView_Clear[]          = "Clear";
    char DataView_AltZero[]        = "AltZero";
    char LogVIEW[]                 = "LogVIEW";
    char LogEND[]                  = "LogEND";
    char RefrLOG[]                 = "RefrLOG";
    char DownLOG[]                 = "DownLOG";
    char UpLOG[]                   = "UpLOG";
    char OptionsEnd[]              = "OptionsEnd";
    char QNH[]                     = "Qnh";
    char Mark[]                    = "Mark";
    char dGMT[]                    = "dGMT";
    char UKRULES[]                 = "UKRULES";
    char Htext0[]                  = "HELP";
    char Htext1[]                  = "Help";
    char b17[]                     = "b17";
    char trf[]                     = "trf"; // Trim factor 
    char Bwn[]                     = "Bwn"; 
    char SetupCol[]                = "SetupCol";
    char b0_bco[]                  = "b0.bco";
    char b0_pco[]                  = "b0.pco";
    char High_pco[]                = "High.pco";
    char Fm_pco[]                  = "Fm.pco";
    char FrontView_BackGround[]    = "FrontView.BackGround";
    char FrontView_ForeGround[]    = "FrontView.ForeGround";
    char FrontView_Special[]       = "FrontView.Special";
    char FrontView_Highlight[]     = "FrontView.Highlight";
    char Mode1[]                   = "Mode1";
    char Mode2[]                   = "Mode2";
    char TrimView_r1[]             = "r1";
    char TrimView_r2[]             = "r2";
    char TrimView_r3[]             = "r3";
    char TrimView_r4[]             = "r4";
    char Md1[]                     = "Md1";
    char Md2[]                     = "Md2";
    char SetupAud[]                = "SetupAud";
    char n0[]                      = "n0";
    char Ex1[]                     = "Ex1";
    char Expo[]                    = "Expo";
    char AudioView[]               = "AudioView";
    char n1[]                      = "n1";
    char h0[]                      = "h0";
    char c0[]                      = "c0";
    char sw0[]                     = "sw0";
    char c1[]                      = "c1";
    char c2[]                      = "c2";
    char c3[]                      = "c3";
    char c4[]                      = "c4";
    char c5[]                      = "c5";
    char StEND[]                   = "StEND";
    char STgo[]                    = "STgo";
    char StCH[]                    = "StCH";
    char s0[]                      = "s0";
    char t2[]                      = "t2";
    char StEDIT[]                  = "StEDIT";     
    char DelLOG[]                  = "DelLOG";       
    

     ScreenTimeTimer = millis();  // reset screen timeout counter
     if (ScreenIsOff) {
         RestoreBrightness();
         ScreenIsOff = false;
         ClearText();
         return;
     }

    if (strlen(TextIn) > 0) {
        StartInactvityTimeout();
 #ifdef DB_NEXTION
        Serial.print("Command word: -> ");
        Serial.println(TextIn);
 #endif
            
 if (InStrng(UpLOG, TextIn)){  
        if (RecentStartLine > 0 && (CurrentView == LOGVIEW))  {
            RecentStartLine-=1;
            ShowLogFile(RecentStartLine); 
        }
        if (RecentStartLine > 0 && (CurrentView == HELP_VIEW))  {
            RecentStartLine-=1;
            ScrollHelpFile(); 
        }
        ClearText();
        return;        
    }   
          
    if (InStrng(DownLOG, TextIn)){  
        if (ThereIsMoreToSee && (CurrentView == LOGVIEW)) {
            RecentStartLine+=1;
            ShowLogFile(RecentStartLine); 
        }

        if (ThereIsMoreToSee && (CurrentView == HELP_VIEW)) {
            RecentStartLine+=1;
            ScrollHelpFile(); 
        }

        ClearText();
        return;        
    }   

    if (InStrng(RefrLOG, TextIn)){   // refresh log screen
        if (UseLog){
            RecentStartLine = 0;
            ShowLogFile(RecentStartLine); 
            ClearText();
        }
        return;        
    }   

    if (InStrng(LogEND, TextIn)){ // close log screen 
            CurrentMode  = NORMAL;
            CurrentView  = DATAVIEW;
            LastShowTime = 0;
            MinimumGap = GetValue(n0);
            LogRXSwaps = GetValue(c0);
            UseLog     = GetValue(sw0);
            SaveTXStuff();
            SendCommand(pDataView);
            ClearText();
            return;
    }   
    if (InStrng(DelLOG, TextIn)){  // delete log and start new one
            DeleteLogFile1();
            ClearText();
            return;
    }   

    if (InStrng(LogVIEW, TextIn)){  // Start log screen
            SendCommand(pLogView);
            CurrentView = LOGVIEW;
            SendValue(n0,MinimumGap);
            SendValue(c0,LogRXSwaps);
            SendValue(sw0,UseLog);   
            if (UseLog){
                RecentStartLine = 0;
                ShowLogFile(RecentStartLine);
            }        
            ClearText();
            return;
    }

    if (InStrng(SetupViewFM, TextIn) > 0) {                // New model name occurs at offset 12 in TextIn
            i = 0;
            while (TextIn[i + 12] > 0) {
                ModelName[i]     = TextIn[i + 12];     // copy new name
                ModelName[i + 1] = 0;
                ++i;
            } 
            SaveAllParameters();
            SendCommand(page_SetupView);
            CurrentMode = NORMAL; // Send data again
            CurrentView = MAINSETUPVIEW;
            ModelNameTimeCheck = 0;
            b5isGrey = false;
            ClearText();
            return;
        }
          if (InStrng(STgo, TextIn)) {                  // Subtrim view start
            SendCommand(pSubTrimView);
            SubTrimToEdit = 0;
            CurrentView =  SUBTRIMVIEW;
            SendText(t2,ChannelNames[SubTrimToEdit]);  // 
            SendValue(n0,SubTrims[SubTrimToEdit]-127);
            SendValue(h0,SubTrims[SubTrimToEdit]);
            ClearText();
            return;
        }

        if (InStrng(StEND, TextIn)) {                  // Subtrim view exit
            SaveOneModel(ModelNumber);
            CurrentView = MAINSETUPVIEW;
            b5isGrey = false;
            SendCommand(page_SetupView);
            ModelNameTimeCheck = 0;
            ClearText();
            return;
        }
        if (InStrng(StCH, TextIn)) {                   // select sub trim channel
            SubTrimToEdit = GetValue(s0);
            SendText(t2,ChannelNames[SubTrimToEdit]); 
            SendValue(n0,SubTrims[SubTrimToEdit]-127);
            SendValue(h0,SubTrims[SubTrimToEdit]);
            ClearText();
            return;
        }

        if (InStrng(StEDIT, TextIn)) {                  // edit sub trim value
            SubTrims[SubTrimToEdit] = GetValue(n0)+127; // 127 is mid point in 8 bit value 0 - 254
            ClearText();
            return;
        }

        if (InStrng(Delete, TextIn) > 0) { 
            ModelNumber = GetValue(ModelsView_ModelNumber);
            SetDefaultValues();
            SaveOneModel(ModelNumber);
            ClearText();
            return;
        }


        if (InStrng(AudioView, TextIn) > 0) {  // Display screen with audio options
            CurrentMode = NORMAL;
            CurrentView = AUDIOVIEW;
            SendCommand(page_AudioView);
            SendValue(n0,AudioVolume);
            SendValue(Ex1,AudioVolume);
            SendValue(n1,Brightness);
            SendValue(h0,Brightness);
            SendValue(c0,PlayFanfare);
            SendValue(c1,TrimClicks);
            SendValue(c2,ButtonClicks);
            SendValue(c3,SpeakingClock);   
            SendValue(c4,AnnounceBanks);  
            SendValue(c5,AnnounceConnected);
            SetAudioVolume(AudioVolume); 
            RestoreBrightness();
            ClearText();
            return;
        }
        if (InStrng(SetupAud, TextIn) > 0) {  // Exit from screen with audio options
            CurrentMode = NORMAL;
            CurrentView = MAINSETUPVIEW;
            AudioVolume   = GetValue(n0);
            if (AudioVolume < 5) AudioVolume = 5;
            Brightness    = GetValue(n1);
            PlayFanfare   = GetValue(c0);
            TrimClicks    = GetValue(c1);
            ButtonClicks  = GetValue(c2);
            SpeakingClock = GetValue(c3);
            AnnounceBanks = GetValue(c4);
            AnnounceConnected = GetValue(c5);
            RestoreBrightness();
            SetAudioVolume(AudioVolume);
            SendCommand(page_SetupView);
            ModelNameTimeCheck = 0;
            SaveTXStuff();
            b5isGrey = false;
            ClearText();
            return;
        }

        if (InStrng(SetupView, TextIn) > 0) { //  goto main setup screen
            ClearText();
            SaveAllParameters();
            SendCommand(page_SetupView);
            ModelNameTimeCheck = 0;
            CurrentMode = NORMAL;
            CurrentView = MAINSETUPVIEW;
            b5isGrey = false; 
            ClearText();
            return;
        }
        if (InStrng(Md1, TextIn) > 0) {           // Mode 1 for trims
            SticksMode = 1;
            ClearText();
            return;
        }  
        if (InStrng(Md2, TextIn) > 0) {           // Mode 2 for trims
            SticksMode = 2;
            ClearText();
            return;
        }
        if (InStrng(Mark, TextIn) > 0) {           
            GPSMarkHere = 255;                // Mark this location
            GPSMaxDistance = 0;
            ClearText();
            return;
        }
        if (InStrng(UKRULES, TextIn) > 0) { // UK Offcom regulations? 
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
        if (InStrng(OptionsEnd, TextIn) > 0) { // Exit from Options screen
            SendCommand(ProgressStart);
            i = strlen(OptionsEnd);
            j = 0;
            while (uint8_t(TextIn[i]) > 0 && i < 100) {
                TxName[j] = TextIn[i];
                ++j;
                ++i;
                TxName[j] = 0;
            }
            SendValue(Progress,35);
            Qnh = (uint16_t)      GetValueSafer(QNH); // error protected version
            DeltaGMT            = GetValueSafer(dGMT);
            SendValue(Progress,45);
            TrimFactor          = GetValueSafer(trf);
            LowBattery          = GetValueSafer(Bwn);
            ScreenTimeout       = GetValueSafer(ScreenViewTimeout);
            CopyTrimsToAll      = GetValueSafer(c0);
            SendValue(Progress,100);
            Inactivity_Timeout  = GetValueSafer(Pto) * TICKSPERMINUTE;
            if (Inactivity_Timeout < INACTIVITYMINIMUM) Inactivity_Timeout = INACTIVITYMINIMUM;
            if (Inactivity_Timeout > INACTIVITYMAXIMUM) Inactivity_Timeout = INACTIVITYMAXIMUM;
            FixDeltaGMTSign();
            if (DoSbusSendOnly)
            {
                Connected       = false;
                LostContactFlag = true;
                BlueLedOn();
            }
            SaveAllParameters();
            SendCommand(page_SetupView);
            ModelNameTimeCheck = 0;
            CurrentMode = NORMAL;
            CurrentView = MAINSETUPVIEW;
            b5isGrey = false;
            SendCommand(ProgressEnd);
            ClearText();
            return;
        }

        if (InStrng(DataEnd, TextIn) > 0) { //  Exit from Data screen
            CurrentView = MAINSETUPVIEW;
            b5isGrey = false;
            CurrentMode = NORMAL;
            ClearText();
            return;
        }
        if (InStrng(DataView_Clear, TextIn) > 0) { //  Clear Data screen
            ZeroDataScreen();
            ClearText();
            return;
        }
         if (InStrng(DataView_AltZero, TextIn) > 0) { //  Set zero altitude on data screen
            if (!GroundModelAltitude) {
                GroundModelAltitude = RXModelAltitude;}
            else {
                GroundModelAltitude = 0 ; }
            if (!GPSGroundAltitude){
                GPSGroundAltitude = GPSAltitude;}
            else{
                 GPSGroundAltitude = 0; }
            GPSMaxAltitude     = 0;
            RXMAXModelAltitude = 0;
            ClearText();
            return;
        }
        if (InStrng(GoFrontView, TextIn) > 0) { // GOTO frontview 
            CurrentView = FRONTVIEW;
            SendCommand(page_FrontView);
            UpdateModelsNameEveryWhere();
            ShowFlightMode();
            LastShowTime = 0;     // this is to make redisplay sooner (in ShowComms())
            LastTimeRead = 0;
            Reconnected = false;  // this is to make '** Connected! **' redisplay (in ShowComms())
            LastSeconds = 0;      // This forces redisplay of timer...
            Force_ReDisplay();
            CheckTimer();
            ClearText();
            return;
        }

        if (InStrng(Scan_End, TextIn) > 0) { //  goto setup screen from Scan screen
            CurrentView = MAINSETUPVIEW;
            b5isGrey = false;
            SendCommand(page_SetupView);
            ModelNameTimeCheck = 0;
            DoScanEnd();
            ClearText();
            return;
        }

        if (InStrng(HelpView, TextIn) > 0) { // Display Help screen(s)
            SavedCurrentView = CurrentView;
            CurrentView      = HELP_VIEW;
            SendHelp();
            ClearText();
            return;
        }
        if (InStrng(Dec_Minute, TextIn) > 0) {
            DecMinute();
            ClearText();
            return;
        }
        if (InStrng(AddMinute, TextIn) > 0) {
            IncMinute();
            ClearText();
            return;
        }

        if (InStrng(Dec_Hour, TextIn) > 0) {
            DecHour();
            ClearText();
            return;
        }
        if (InStrng(Inc_Hour, TextIn) > 0) {
            IncHour();
            ClearText();
            return;
        }

        if (InStrng(Dec_Year, TextIn) > 0) {
            DecYear();
            ClearText();
            return;
        }
        if (InStrng(Inc_Year, TextIn) > 0) {
            IncYear();
            ClearText();
            return;
        }

        if (InStrng(Dec_Date, TextIn) > 0) {
            DecDate();
            ClearText();
            return;
        }
        if (InStrng(Inc_Date, TextIn) > 0) {
            IncDate();
            ClearText();
            return;
        }

        if (InStrng(Dec_Month, TextIn) > 0) {
            DecMonth();
            ClearText();
            return;
        }
        if (InStrng(Inc_Month, TextIn) > 0) {
            IncMonth();
            ClearText();
            return;
        }

        if (InStrng(OptionsViewS, TextIn) > 0) { 
            FixDeltaGMTSign();
            SendCommand(pOptionsViewS);
            SendValue(ScreenViewTimeout, ScreenTimeout);
         
            SendValue(Pto, (Inactivity_Timeout / TICKSPERMINUTE));
            SendText(Tx_Name, TxName);
            SendValue(QNH,Qnh);
            SendValue(dGMT,DeltaGMT);
            SendValue(trf,TrimFactor); 
            SendValue(Bwn,LowBattery);
            SendValue(c0,CopyTrimsToAll);
            CurrentView = OPTIONS_VIEW;
            CurrentMode = NORMAL;
            ClearText();
            return;
        }
        if (InStrng(GOTO, TextIn) > 0) {                     // Return from Help screen returns here to relevent config screen 
            i = 5;
            while (uint8_t(TextIn[i]) > 0 && i < 30) {
                WhichPage[i] = TextIn[i];
                ++i;
                WhichPage[i] = 0;
            } // Get page name to which to return
            SendCommand(WhichPage);                         // this sends nextion back to last screen
            CurrentView = SavedCurrentView; 
   
            if (CurrentView == GRAPHVIEW) {
                DisplayCurveAndServoPos();
                SendValue(CopyToAllFlightModes, 0);
            }
            if (CurrentView == SWITCHES_VIEW) {
                UpdateSwitchesDisplay();
            }
            if (CurrentView == ONE_SWITCH_VIEW) {
                updateOneSwitchView();
            }
            if (CurrentView == MODELSVIEW) {
                SendValue(ModelsView_ModelNumber, ModelNumber);
            }
            if (CurrentView == MIXESVIEW) {
                if (MixNumber == 0) MixNumber = 1;
                LastMixNumber = 33;                        // just to be differernt
                SendValue(MixesView_MixNumber, MixNumber); // New load of mix window
                SendMixValues();
            }
            if (CurrentView == FILESVIEW) { 
                ShowDirectory();  
            }
            if (CurrentView == MACROS_VIEW){ 
               // Do nothing!
            }
             if (CurrentView == CALIBRATEVIEW){
                Force_ReDisplay();
                ShowServoPos(); 
             }
             if (CurrentView == REVERSEVIEW){
                 StartReverseView();
             }
             if ((CurrentView == STICKSVIEW) || (CurrentView == FRONTVIEW)){
                Force_ReDisplay();
                ShowServoPos(); 
                if (CurrentView == FRONTVIEW){
                    SendValue(FrontView_Secs, Secs);
                    SendValue(FrontView_Mins, Mins);
                    SendValue(FrontView_Hours, Hours);  
                }
             }
             if (CurrentView == LOGVIEW){ 
                SendCommand(pLogView);
                CurrentView = LOGVIEW;
                RecentStartLine = 0;
                if (UseLog) ShowLogFile(RecentStartLine);
             }
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(Exrite, TextIn) > 0) { //  *******************
            if (GetValue(ExpR)) {
                InterpolationTypes[FlightMode][ChanneltoSet - 1] = EXPONENTIALCURVES;
            }
            if (GetValue(Smooth)) {
                InterpolationTypes[FlightMode][ChanneltoSet - 1] = SMOOTHEDCURVES;
            }
            if (GetValue(Lines)) {
                InterpolationTypes[FlightMode][ChanneltoSet - 1] = STRAIGHTLINES;
            }
            Exponential[FlightMode][ChanneltoSet - 1] = GetValue(Expo)+50; // Note: Getting this value from slider was not reliable (could not return 36!)
            ClearText();
            DisplayCurveAndServoPos();
            return;
        }
        if (InStrng(ReceiveModel, TextIn) > 0) {
            i = strlen(ReceiveModel);
            j = 0;
            while (uint8_t(TextIn[i] && i < 100) > 0) {
                SingleModelFile[j] = TextIn[i];
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            } // got local name but won't use it.....
            ReceiveModelFile();
            ClearText();
            return;
        }
        if (InStrng(PowerOff, TextIn) > 0) {
            if (!LostContactFlag && BoundFlag) {
                SendText(StillConnectedBox,StillConnectedMsg);
                SendCommand(StillConnected);
                Procrastinate(750); // 3/4 second
                SendCommand(NotStillConnected);
            }
            else {
                if (UseLog) LogPowerOff();
                SaveAllParameters();
                digitalWrite(POWER_OFF_PIN, HIGH);
            }
            ClearText();
            return;
        }

        if (InStrng(OffNow, TextIn) > 0) {
           if (UseLog) LogPowerOff();

            digitalWrite(POWER_OFF_PIN, HIGH); // force OFF in Options View
            ClearText();
            return;
        }

        if (InStrng(SendModel, TextIn) > 0) {
            i = strlen(SendModel);
            j = 0;
            while (uint8_t(TextIn[i]) > 0) {
                SingleModelFile[j] = TextIn[i];
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            }
            SendModelFile();
            ClearText();
            return;
        }
        if (InStrng(FailSAVE, TextIn) > 0) {
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
            FailSafeTimer= millis();
            SaveFailSafeNow = true;
            ClearText();
            return;
        }
        if (InStrng(FailSafe, TextIn) > 0) {
            SendCommand(pFailSafe);
            CurrentView = FAILSAFE_VIEW;
            UpdateButtonLabels();
            ClearText();
            return;
        }

        if (InStrng(OneSwitchView, TextIn) > 0) {
            SwitchEditNumber = GetChannel(); // which switch?
            CurrentView      = ONE_SWITCH_VIEW;
            SendCommand(PageOneSwitchView); // edit one switch - could be 1-4
            updateOneSwitchView();
            ClearText();
            return;
        }

        if (InStrng(SwitchesView1, TextIn) > 0) { //  read switch values from screen (could be 1-4)
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
                    SWITCH1Reversed = true;
                }
                else {
                    SWITCH1Reversed = false;
                }
            }
            if (SwitchEditNumber == 2) {
                if (GetValue(OneSwitchViewc_revd)) {
                    SWITCH2Reversed = true;
                }
                else {
                    SWITCH2Reversed = false;
                }
            }
            if (SwitchEditNumber == 3) {
                if (GetValue(OneSwitchViewc_revd)) {
                    SWITCH3Reversed = true;
                }
                else {
                    SWITCH3Reversed = false;
                }
            }
            if (SwitchEditNumber == 4) {
                if (GetValue(OneSwitchViewc_revd)) {
                    SWITCH4Reversed = true;
                }
                else {
                    SWITCH4Reversed = false;
                }
            }
            SaveOneModel(ModelNumber);
            SendCommand(PageSwitchView); // change to all switches screen
            UpdateSwitchesDisplay();     // update its info
            ClearText();
            return;
        }
        if (InStrng(InputsView, TextIn) > 0) {
            SendCommand(pInputsView);
            CurrentView = INPUTS_VIEW;
            UpdateButtonLabels();
            ClearText();
            return;
        }

        if (InStrng(InputsDone, TextIn) > 0) {
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
            ModelNameTimeCheck = 0;
            ClearText();
            return;
        }
        if (InStrng(CH1NAME, TextIn) > 0) {
            p = InStrng(CH1NAME, TextIn);
            i = p + 7;
            DoNewChannelName(1, i);
            ClearText();
            return;
        }
        if (InStrng(CH2NAME, TextIn) > 0) {
            p = InStrng(CH2NAME, TextIn);
            i = p + 7;
            DoNewChannelName(2, i);
            ClearText();
            return;
        }
        if (InStrng(CH3NAME, TextIn) > 0) {
            p = InStrng(CH3NAME, TextIn);
            i = p + 7;
            DoNewChannelName(3, i);
            ClearText();
            return;
        }
        if (InStrng(CH4NAME, TextIn) > 0) {
            p = InStrng(CH4NAME, TextIn);
            i = p + 7;
            DoNewChannelName(4, i);
            ClearText();
            return;
        }
        if (InStrng(CH5NAME, TextIn) > 0) {
            p = InStrng(CH5NAME, TextIn);
            i = p + 7;
            DoNewChannelName(5, i);
            ClearText();
            return;
        }
        if (InStrng(CH6NAME, TextIn) > 0) {
            p = InStrng(CH6NAME, TextIn);
            i = p + 7;
            DoNewChannelName(6, i);
            ClearText();
            return;
        }
        if (InStrng(CH7NAME, TextIn) > 0) {
            p = InStrng(CH7NAME, TextIn);
            i = p + 7;
            DoNewChannelName(7, i);
            ClearText();
            return;
        }
        if (InStrng(CH8NAME, TextIn) > 0) {
            p = InStrng(CH8NAME, TextIn);
            i = p + 7;
            DoNewChannelName(8, i);
            ClearText();
            return;
        }
        if (InStrng(CH9NAME, TextIn) > 0) {
            p = InStrng(CH9NAME, TextIn);
            i = p + 7;
            DoNewChannelName(9, i);
            ClearText();
            return;
        }
        if (InStrng(CH10NAME, TextIn) > 0) {
            p = InStrng(CH10NAME, TextIn);
            i = p + 8;
            DoNewChannelName(10, i);
            ClearText();
            return;
        }
        if (InStrng(CH11NAME, TextIn) > 0) {
            p = InStrng(CH11NAME, TextIn);
            i = p + 8;
            DoNewChannelName(11, i);
            ClearText();
            return;
        }
        if (InStrng(CH12NAME, TextIn) > 0) {
            p = InStrng(CH12NAME, TextIn);
            i = p + 8;
            DoNewChannelName(12, i);
            ClearText();
            return;
        }
        if (InStrng(CH13NAME, TextIn) > 0) {
            p = InStrng(CH13NAME, TextIn);
            i = p + 8;
            DoNewChannelName(13, i);
            ClearText();
            return;
        }
        if (InStrng(CH14NAME, TextIn) > 0) {
            p = InStrng(CH14NAME, TextIn);
            i = p + 8;
            DoNewChannelName(14, i);
            ClearText();
            return;
        }
        if (InStrng(CH15NAME, TextIn) > 0) {
            p = InStrng(CH15NAME, TextIn);
            i = p + 8;
            DoNewChannelName(15, i);
            ClearText();
            return;
        }
        if (InStrng(CH16NAME, TextIn) > 0) {
            p = InStrng(CH16NAME, TextIn);
            i = p + 8;
            DoNewChannelName(16, i);
            ClearText();
            return;
        }

        if (InStrng(DelFile, TextIn) > 0) { // Delete a file
            j = 0;
            p = InStrng(DelFile, TextIn);
            i = p + 6;
            while (uint8_t(TextIn[i]) > 0) {
                SingleModelFile[j] = TextIn[i];
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            }
            SD.remove(SingleModelFile);
            BuildDirectory();
            -- FileNumberInView;
            ShowFileNumber();
            CloseModelsFile();
            ClearText();
            return;
        }


        if (InStrng(SwitchesView, TextIn)) {
            SendCommand(pSwitchesView);
            UpdateSwitchesDisplay(); // display saved values
            CurrentView = SWITCHES_VIEW;
            ClearText();
            return;
        }

        if (InStrng(CalibrateView, TextIn)) { 
            SendCommand(pCalibrateView);
            Force_ReDisplay();
            CurrentView = CALIBRATEVIEW;
            ClearText();
            return;
        }

        p = InStrng(Export, TextIn);
        if (p > 0) {
            j = 0;
            i = p + 5;
            while ((TextIn[i] > 0) && (i < 18)) {
                if (TextIn[i] >= 97 && TextIn[i] <= 122) {
                    TextIn[i] &= ~0x20; // upper case only
                } 
                SingleModelFile[j] = TextIn[i];
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
                Procrastinate(20);
                SendValue(Progress, 10);
                Procrastinate(20);
                CloseModelsFile();
                for (uint8_t WriteTwice = 1; WriteTwice <= 2; ++WriteTwice) { // if a new file, write twice seems to be needed!!
                    SingleModelFlag = true;
                    OpenModelsFile();
                    SendValue(Progress, 25);
                    Procrastinate(10);
                    SaveOneModel(1);
                    SendValue(Progress, 50);
                    Procrastinate(10);
                    CloseModelsFile();
                }
                SingleModelFlag = false;
                SendValue(Progress, 75);
                Procrastinate(10);
                BuildDirectory();
                SendValue(Progress, 100);
                Procrastinate(10);
                SendCommand(ProgressEnd);
            }
            else {
                FileError = true;
            }
            if (FileError) ShowFileErrorMsg();
            ClearText();
            return;
        }
        p = InStrng(Import, TextIn); 
        if (p > 0) {
            SendCommand(ProgressStart);
            Procrastinate(10);
            SendValue(Progress, 5);
            Procrastinate(10);
            j = 0;
            i = p + 5;
            while (TextIn[i] > 0) {
                if (TextIn[i] >= 97 && TextIn[i] <= 122) {
                    TextIn[i] &= ~0x20;
                }
                SingleModelFile[j] = TextIn[i];
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            }
            if (InStrng(ModExt, SingleModelFile) == 0) strcat(SingleModelFile, ModExt);
            SingleModelFlag = true;
            SendValue(Progress, 10);
            Procrastinate(10);
            CloseModelsFile();
            ReadOneModel(1);
            SendValue(Progress, 50);
            Procrastinate(10);
            SingleModelFlag = false;
            CloseModelsFile();
            SendValue(Progress, 75);
            Procrastinate(10);
            SaveAllParameters();
            CloseModelsFile();
            UpdateModelsNameEveryWhere();
            SendValue(Progress, 100);
            Procrastinate(10);
            SendCommand(ProgressEnd);
            if (FileError) ShowFileErrorMsg();
            ClearText();
            return;
        }
        if (InStrng(ListFiles, TextIn) > 0) {  
            SendCommand(PageFilesView);
            CurrentView = FILESVIEW;  // Can't change this until I can change it back!  :-)
            SavedCurrentView = FILESVIEW;
            ShowDirectory();      
            ClearText();
            return;
        }

        if (InStrng(GoSetupView, TextIn) > 0) {
            CurrentView = MAINSETUPVIEW;
            b5isGrey = false;
            SendCommand(page_SetupView);
            ModelNameTimeCheck = 0;
            ClearText();
            return;
        }

         if (InStrng(ColoursView, TextIn) > 0) {  
            CurrentView = COLOURS_VIEW;
            SendCommand(page_ColoursView);
            SendCommand(StartBackGround);      
            ClearText();
            return;
        }
        
          if (InStrng(SetupCol, TextIn) > 0) {  // This is the return from Colours setup 
            HighlightColour  = GetOtherValue(High_pco);
            ForeGroundColour = GetOtherValue(b0_pco);
            BackGroundColour = GetOtherValue(b0_bco);
            SpecialColour    = GetOtherValue(Fm_pco);
            SendValue(FrontView_BackGround,BackGroundColour);
            SendValue(FrontView_ForeGround,ForeGroundColour);
            SendValue(FrontView_Special,SpecialColour);
            SendValue(FrontView_Highlight,HighlightColour);
            SaveTXStuff();
            CurrentView = MAINSETUPVIEW;
            b5isGrey = false;
            SendCommand(page_SetupView);
            ModelNameTimeCheck = 0;
            ClearText();
            return;
        }

        if (InStrng(TypeView, TextIn) > 0) {
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

        if (InStrng(RXBAT, TextIn) > 0) { // UPdate RX batt cell count
            if (GetValue(r2s) == 1) RXCellCount = 2;
            if (GetValue(r3s) == 1) RXCellCount = 3;
            if (GetValue(r4s) == 1) RXCellCount = 4;
            if (GetValue(r5s) == 1) RXCellCount = 5;
            if (GetValue(r6s) == 1) RXCellCount = 6;
            SaveOneModel(ModelNumber);
            ClearText();
            return;
        }

        if (InStrng(TrimView, TextIn) > 0) { // TrimView just appeared, so update it. 
            SendCommand(pTrimView);
            CurrentView = TRIM_VIEW;
            UpdateModelsNameEveryWhere(); // also updates trimview (If CurrentView == TRIM_VIEW!! :-)
            if (!UkRules){
                SendText(b17,Htext0);
            }else{
                SendText(b17,Htext1);
            } 
            ClearText();
            return;
        }
        if (InStrng(RTRIM, TextIn) > 0) {
            TrimsReversed[FlightMode][0] = GetValue(TrimView_r1);  
            TrimsReversed[FlightMode][1] = GetValue(TrimView_r4);
            TrimsReversed[FlightMode][2] = GetValue(TrimView_r2);
            TrimsReversed[FlightMode][3] = GetValue(TrimView_r3);
            if (CopyTrimsToAll){
                for (j = 0; j < 4;++j){
                    for (i = 1; i < 5; ++i){
                         TrimsReversed[i][j] = TrimsReversed[FlightMode][j];
                    }
                }
            }
            ClearText();
            return;
        }
        if (InStrng(TRIMS50, TextIn) > 0) {  
            for (i = 0; i < 4; ++i) {
                Trims[FlightMode][i] = 80; // Mid value is 80
                if (CopyTrimsToAll){
                  for (i = 0; i < 4; ++i) {
                      for (int fm = 1; fm < 5;++fm){
                      Trims[fm][i] = 80; 
                      TrimsReversed[fm][i] =  TrimsReversed[FlightMode][i];
                  }
                }
            }
         }
        ClearText(); 
        return;
        }

        if (InStrng(TR1, TextIn) > 0) { //  TR1->0
            if ((TextIn[3] <= 80 + 40) && (TextIn[3] >= 80 - 40)) Trims[FlightMode][0] = TextIn[3]; // Check in case out of range!
            if (CopyTrimsToAll){
            for (int fm = 1; fm < 5;++fm){
                      Trims[fm][0] = TextIn[3]; 
                      TrimsReversed[fm][i] =  TrimsReversed[FlightMode][i];
            }
            }
            ClearText(); 
            return;
        }

        if (InStrng(TR4, TextIn) > 0) { // TR4 ->1
            if (SticksMode == 1)  {
               if ((TextIn[3] <= 80 + 40) && (TextIn[3] >= 80 - 40)) Trims[FlightMode][1] = TextIn[3];
                if (CopyTrimsToAll){
                    for (int fm = 1; fm < 5;++fm){
                      Trims[fm][1] = TextIn[3];
                      TrimsReversed[fm][i] =  TrimsReversed[FlightMode][i];
                    }
                }
            }
            if (SticksMode == 2)  {
                if ((TextIn[3] <= 80 + 40) && (TextIn[3] >= 80 - 40)) Trims[FlightMode][2] = TextIn[3];
                if (CopyTrimsToAll){
                        for (int fm = 0; fm < 5;++fm){
                        Trims[fm][2] = TextIn[3]; 
                        TrimsReversed[fm][i] =  TrimsReversed[FlightMode][i];
                        }
                }
            }
            ClearText(); 
            return;
        }
        if (InStrng(TR2, TextIn) > 0) { // TR2 ->2
            if (SticksMode == 1) {
                if ((TextIn[3] <= 80 + 40) && (TextIn[3] >= 80 - 40)) Trims[FlightMode][2] = TextIn[3];
                if (CopyTrimsToAll){
                        for (int fm = 1; fm < 5;++fm)
                        Trims[fm][2] = TextIn[3]; 
                }
            }
            if (SticksMode == 2)  { 
                if ((TextIn[3] <= 80 + 40) && (TextIn[3] >= 80 - 40)) Trims[FlightMode][1] = TextIn[3];
                if (CopyTrimsToAll){
                    for (int fm = 1; fm < 5;++fm){
                        Trims[fm][1] = TextIn[3]; 
                        TrimsReversed[fm][i] =  TrimsReversed[FlightMode][i];
                    }
                }
            }
            ClearText(); 
            return;
        }
        if (InStrng(TR3, TextIn) > 0) { // TR3 ->3
            if ((TextIn[3] <= 80 + 40) && (TextIn[3] >= 80 - 40)) Trims[FlightMode][3] = TextIn[3];
            if (CopyTrimsToAll){
            for (int fm = 0; fm < 5;++fm){
                    Trims[fm][3] = TextIn[3]; 
                    TrimsReversed[fm][i] =  TrimsReversed[FlightMode][i];
                }
            }
            ClearText(); 
            return;
        }

        if (InStrng(Trim, TextIn) > 0) {                       // This is the return from Trim view
            if (GetValue(Mode1)==1) SticksMode = 1;
            if (GetValue(Mode2)==1) SticksMode = 2; 
            SaveAllParameters(); // save trims to SDcard
            SendCommand(page_SetupView);
            ModelNameTimeCheck = 0;
            CurrentMode = NORMAL;
            CurrentView = MAINSETUPVIEW;
            b5isGrey = false;
            ClearText(); 
            return;
        }
     

        if (InStrng(Write, TextIn) > 0) { //  write new data to SD
            p = GetValue(CopyToAllFlightModes);
            if (p == 1) {
                for (p = 1; p <= 4; ++p) {
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
            CurrentView = STICKSVIEW;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(Setup, TextIn) > 0) { // Which channel to setup ... Goes to GraphView
            ChanneltoSet = GetChannel();
            ClearText();
            SendCommand(page_GraphView); // Set to GraphView
            CurrentView = GRAPHVIEW;
            DisplayCurveAndServoPos();
            updateInterpolationTypes();
            UpdateModelsNameEveryWhere();
            SendValue(CopyToAllFlightModes, 0);
            ClearText();
            return;
        }

        if (InStrng(Front_View, TextIn)) {
            CurrentView = FRONTVIEW;
            ClearText();
            PreviousFlightMode = 250; // sure to be different
            CurrentMode        = NORMAL;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(Sticks_View, TextIn)) {
            SendCommand(page_SticksView);
            Force_ReDisplay();
            CurrentView = STICKSVIEW;
            SendCommand(page_SticksView); // Set to SticksView
            UpdateModelsNameEveryWhere();
            UpdateButtonLabels();
            ClearText();
            return;
        }

        if (InStrng(Fhss_View, TextIn)) {
             if (!b5isGrey)  // no scan while connected!!!
               {
                    SendCommand(page_FhssView);
                    DrawFhssBox();
                    DoScanInit();
                    CurrentMode = SCANWAVEBAND;
                    CurrentView = SCANVIEW;
                    BlueLedOn();        
                }
            ClearText();
            return;
        }

        if (InStrng(ReScan, TextIn)) 
        {
            DrawFhssBox();
            DoScanInit();
            ClearText();
            return;
        }

        if (InStrng(MIXES_VIEW, TextIn)) {
            SendCommand(pMixesView); 
            CurrentView = MIXESVIEW;
            UpdateModelsNameEveryWhere();
            if (MixNumber == 0) MixNumber = 1;
            LastMixNumber = 33;                        // just to be differernt
            SendValue(MixesView_MixNumber, MixNumber); // New load of mix window
            SendMixValues();
            ClearText();
            return;
        }

        if (InStrng(Mixes_View, TextIn)) {
            CurrentView = MIXESVIEW;
            Procrastinate(100);               // allow screen changes to appear
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

        if (InStrng(Graph_View, TextIn))
        {
            CurrentView = GRAPHVIEW;
            ClearText();
            return;
        }

        if (InStrng(Data_View, TextIn))
        {
            CurrentMode  = NORMAL;
            CurrentView  = DATAVIEW;
            LastShowTime = 0;
            SendCommand(pDataView);
            ClearText();
            return;
        }

        if (InStrng(bind, TextIn))
        {
            BindNow();
            ClearText();
            return;
        }

        if (InStrng(FM1, TextIn))
        {
            FlightMode         = 1;
            PreviousFlightMode = 1;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(FM2, TextIn))
            {
                FlightMode         = 2;
                PreviousFlightMode = 2;
                UpdateModelsNameEveryWhere();
                ClearText();
                return;
            }

        if (InStrng(FM3, TextIn))
        {
            FlightMode         = 3;
            PreviousFlightMode = 3;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(FM4, TextIn))
        {
            FlightMode         = 4;
            PreviousFlightMode = 4;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }
        if (InStrng(midyup, TextIn)) // midy up?
        {
            CentreDegrees[FlightMode][ChanneltoSet - 1]++;
            DisplayCurveAndServoPos();
            return;
        }

        if (InStrng(midydown, TextIn)) // midy down?
        {
            if (CentreDegrees[FlightMode][ChanneltoSet - 1] > 0 ) {CentreDegrees[FlightMode][ChanneltoSet - 1]--;}
            DisplayCurveAndServoPos();
            return;
        }

        if (InStrng(midhiyup, TextIn)) // midhiy up?
        {
            MidHiDegrees[FlightMode][ChanneltoSet - 1]++;
            DisplayCurveAndServoPos();
            return;
        }

        if (InStrng(midhiydown, TextIn)) // midhiy down?
        {
            if (MidHiDegrees[FlightMode][ChanneltoSet - 1] > 0) {MidHiDegrees[FlightMode][ChanneltoSet - 1]--;}
            DisplayCurveAndServoPos();
            return;
        }

        if (InStrng(midlowyup, TextIn)) // midlowy up?
        {
            MidLowDegrees[FlightMode][ChanneltoSet - 1]++;
            DisplayCurveAndServoPos();
            return;
        }

        if (InStrng(midlowydown, TextIn)) // midlowy down?
        {
            if (MidLowDegrees[FlightMode][ChanneltoSet - 1] > 0 ) {MidLowDegrees[FlightMode][ChanneltoSet - 1]--;}
            DisplayCurveAndServoPos();
            return;
        }

        if (InStrng(yy1up, TextIn)) // yy1 up?
        {
            MaxDegrees[FlightMode][ChanneltoSet - 1]++;
            DisplayCurveAndServoPos();
            return;
        }

        if (InStrng(yy1down, TextIn)) // yy1 down?
        {
            if(MaxDegrees[FlightMode][ChanneltoSet - 1] > 0) {MaxDegrees[FlightMode][ChanneltoSet - 1]--;}
            DisplayCurveAndServoPos();
            return;
        }

        if (InStrng(yy2up, TextIn)) // yy1 up?
        {
            MinDegrees[FlightMode][ChanneltoSet - 1]++;
            DisplayCurveAndServoPos();
            return;
        }

        if (InStrng(yy2down, TextIn)) // yy1 down?
        {
            if (MinDegrees[FlightMode][ChanneltoSet - 1] > 0) {MinDegrees[FlightMode][ChanneltoSet - 1]--;}
            DisplayCurveAndServoPos();
            return;
        }

        if (InStrng(Reset, TextIn)) // RESET?
        {
            MinDegrees[FlightMode][ChanneltoSet - 1]         = 30;
            MidLowDegrees[FlightMode][ChanneltoSet - 1]      = 60;
            CentreDegrees[FlightMode][ChanneltoSet - 1]      = 90;
            MidHiDegrees[FlightMode][ChanneltoSet - 1]       = 120;
            MaxDegrees[FlightMode][ChanneltoSet - 1]         = 150;
            Exponential[FlightMode][ChanneltoSet - 1]        = DEFAULT_EXPO;
            InterpolationTypes[FlightMode][ChanneltoSet - 1] = EXPONENTIALCURVES; // expo = default
            DisplayCurveAndServoPos();
            return;
        }

        if (InStrng(Reverse, TextIn)) // REVERSE?
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
            DisplayCurveAndServoPos();
            return;
        }
        p = (InStrng(ClickX, TextIn)); // Clicked to move point?
        if (p > 0) {
            XtouchPlace = GetNextNumber(p + 7, TextIn);
            // This drops through to get Y as well before moving the point
        }
        p = (InStrng(ClickY, TextIn)); // Clicked to move point?
        if (p > 0) {
            YtouchPlace = GetNextNumber(p + 7, TextIn);
            MovePoint();
            DisplayCurveAndServoPos();
            return;
        }
        if (CurrentMode == NORMAL) { 
            if (strcmp(TextIn, "Calibrate1") == 0) {
                ReduceLimits();                       // Get setup for sticks calibration
                CurrentMode = CALIBRATELIMITS;
                CurrentView = CALIBRATEVIEW ;
                SendText1(SvT11, CMsg1);
                SendText(SvB0, CMsg2);
                ClearText();
                BlueLedOn();
                return;
            }
        }

        if (CurrentMode == CALIBRATELIMITS) {
            if (strcmp(TextIn, "Calibrate1") == 0) {
                CurrentMode = CENTRESTICKS;
                CurrentView = CALIBRATEVIEW ;
                SendText(SvT11, Cmsg3);
                SendText(SvB0, Cmsg4);
                ClearText();
                return;
            }
        }
        if (CurrentMode == CENTRESTICKS) {
            if (strcmp(TextIn, "Calibrate1") == 0) {
                CurrentMode = NORMAL;
                SaveTXStuff();                     // Save calibrations
                LoadAllParameters();               // Restore all current model settings
                SendCommand(page_SetupView);
                ModelNameTimeCheck = 0;
                ClearText();
                return;
            }
        }
    }
    ClearText(); // Let's have cleared text for next one!
} // end ButtonWasPressed()

/************************************************************************************************************/

uint16_t MakeTwobytes(bool* f)
{                    // Pass arraypointer. Returns the two bytes
    uint16_t tb = 0; // all false is default
    for (int i = 0; i < 16; ++i) {
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
    if ((sw1 == false) && (sw2 == false)) 
    {
        FlightMode = 2;
    }else{
        if (rev) {
            if (sw1) FlightMode = 1;
            if (sw2) FlightMode = 3;
        }
        else {
            if (sw1) FlightMode = 3;
            if (sw2) FlightMode = 1;
        }
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
    if (swt == 1) rtv = ReadCHSwitch(Switch[7], Switch[6], SWITCH1Reversed);
    if (swt == 2) rtv = ReadCHSwitch(Switch[5], Switch[4], SWITCH2Reversed);
    if (swt == 3) rtv = ReadCHSwitch(Switch[0], Switch[1], SWITCH3Reversed);
    if (swt == 4) rtv = ReadCHSwitch(Switch[2], Switch[3], SWITCH4Reversed);
    return rtv;
}
/************************************************************************************************************/

void GetFlightMode()
{ //  and AUTO and other switchy things ...

    if (FMSwitch == 4) ReadFMSwitch(Switch[2], Switch[3], SWITCH4Reversed);
    if (FMSwitch == 3) ReadFMSwitch(Switch[0], Switch[1], SWITCH3Reversed);
    if (FMSwitch == 2) ReadFMSwitch(Switch[4], Switch[5], SWITCH2Reversed);
    if (FMSwitch == 1) ReadFMSwitch(Switch[6], Switch[7], SWITCH1Reversed);
    if (AutoSwitch == 1 && Switch[6] == SWITCH1Reversed) FlightMode = 4; // Flight mode 4 (Auto) overrides modes 1,2,3.
    if (AutoSwitch == 2 && Switch[4] == SWITCH2Reversed) FlightMode = 4;
    if (AutoSwitch == 3 && Switch[1] == SWITCH3Reversed) FlightMode = 4;
    if (AutoSwitch == 4 && Switch[3] == SWITCH4Reversed) FlightMode = 4;

    Channel9SwitchValue  = CheckSwitch(Channel9Switch);
    Channel10SwitchValue = CheckSwitch(Channel10Switch);
    Channel11SwitchValue = CheckSwitch(Channel11Switch);
    Channel12SwitchValue = CheckSwitch(Channel12Switch);

    if (FlightMode != PreviousFlightMode) {
        if (Connected) LogNewFlightMode();
        if (AnnounceBanks) SoundFlightMode();
        if (CurrentView == FRONTVIEW) ShowFlightMode();
        if (PreviousFlightMode == 4)  TimerMillis = millis();
        if (FlightMode == 4) PausedSecs = Secs + (Mins * 60) + (Hours * 3600); // Remember how long so far
        LastSeconds = 0;                                                       // to force redisplay of timer
        CheckTimer(); 
        UpdateModelsNameEveryWhere();
        if (CurrentView == GRAPHVIEW) DisplayCurveAndServoPos();
    }
    PreviousFlightMode = FlightMode;
}

/*********************************************************************************************************************************/
 // This updates only the trim that was editied - for extra speed.

void UpdateTrimViewPart(uint8_t ch)
{
    char TrimView_ch1[] = "ch1";
    char TrimView_ch2[] = "ch2";
    char TrimView_ch3[] = "ch3";
    char TrimView_ch4[] = "ch4"; 
    char TrimView_n1[]  = "n1";
    char TrimView_n2[]  = "n2";
    char TrimView_n3[]  = "n3";
    char TrimView_n4[]  = "n4";

switch(ch){
    case 0:
    SendValue(TrimView_ch1, (Trims[FlightMode][0]));
    SendValue(TrimView_n1,  (Trims[FlightMode][0] - 80));
    break;
    case 1:
    SendValue(TrimView_ch4, (Trims[FlightMode][1]));
    SendValue(TrimView_n4,  (Trims[FlightMode][1] - 80));
    break;
    case 2:
    SendValue(TrimView_ch2, (Trims[FlightMode][2]));
    SendValue(TrimView_n2,  (Trims[FlightMode][2] - 80));
    break;
    case 3:
    SendValue(TrimView_ch3, (Trims[FlightMode][3]));
    SendValue(TrimView_n3,  (Trims[FlightMode][3] - 80));
    break;
    default:
    break;
    }
}

// *************************************************************************************************************

void IncTrim(uint8_t t){
       
        bool Sounded = false;
        Trims[FlightMode][t] += 1;
        if (Trims[FlightMode][t] >= 120) {
            Trims[FlightMode][t]  = 120;
            if (TrimClicks) {
               
                PlaySound(BEEPCOMPLETE);
                Sounded = true;
                TrimRepeatSpeed = DefaultTrimRepeatSpeed; 
             }
        }
        if (Trims[FlightMode][t] == 80)  {
            TrimRepeatSpeed = DefaultTrimRepeatSpeed;         // Restore default trim repeat speed at centre
             if (TrimClicks) {
                 PlaySound(BEEPMIDDLE);
                Sounded = true;
            }
        }
        if ((CurrentView == TRIM_VIEW) || (CurrentView == FRONTVIEW)) UpdateTrimViewPart(t);
        if ((TrimClicks) && (!Sounded)) PlaySound(CLICKZERO);
}
// *************************************************************************************************************

void DecTrim(uint8_t t){
        
        bool Sounded = false;
         Trims[FlightMode][t] -= 1;
         if (Trims[FlightMode][t] <= 40) {
             Trims[FlightMode][t] = 40;
             if (TrimClicks) {
                PlaySound(BEEPCOMPLETE);
                Sounded = true;
                 TrimRepeatSpeed = DefaultTrimRepeatSpeed; 
             }
         }
         if (Trims[FlightMode][t] == 80)  {
             TrimRepeatSpeed = DefaultTrimRepeatSpeed;         // Restore default trim repeat speed at centre
            if (TrimClicks) {
                PlaySound(BEEPMIDDLE);
                Sounded = true;
            }
         }
        if ((CurrentView == TRIM_VIEW) || (CurrentView == FRONTVIEW)) UpdateTrimViewPart(t);
        if ((TrimClicks) && (!Sounded)) PlaySound(CLICKZERO);
}
// *************************************************************************************************************

void  MoveaTrim(uint8_t i){
    uint8_t Elevator = 1; 
    uint8_t Throttle = 2; 
    
    if (SticksMode == 2) {
        Elevator = 2; 
        Throttle = 1; 
    }
  
    switch(i){
    case 0:
            IncTrim(0);   // Aileron
            break;
    case 1: 
            DecTrim(0);  // Aileron
            break;
    case 2:
            IncTrim(Elevator); 
            break;
    case 3: 
            DecTrim(Elevator);
            break;
    case 4:
            DecTrim(Throttle); 
            break;
    case 5: 
            IncTrim(Throttle);
            break;
    case 6:
            IncTrim(3); // Rudder
            break;
    case 7: 
            DecTrim(3); // Rudder
            break;
    default:
    break;
    }
    if (ScreenIsOff) RestoreBrightness();
    if (CopyTrimsToAll){  
      for (i = 0;i < 4; ++i)
         for (int fm = 1; fm < 5; ++fm) {
                      Trims[fm][i] = Trims[FlightMode][i];  
                      TrimsReversed[fm][i] =  TrimsReversed[FlightMode][i];
            }
        }
} 

/************************************************************************************************************/
void CheckHardwareTrims(){  
    int i;
    if ((millis() - TrimTimer) < TrimRepeatSpeed) return;    // check occasionally for trim press
    TrimTimer = millis();
    for (i = 0; i < 8; ++i) {
        if (TrimSwitch[i]) {
            MoveaTrim(i);
            TrimRepeatSpeed -= (TrimRepeatSpeed/6);           // accelerate repeat...
            if (TrimRepeatSpeed < 40) TrimRepeatSpeed = 40;   // ... up to a point... 
        }
    }
}
/************************************************************************************************************/
void swap(uint8_t *a, uint8_t *b){                                  // Just swap over two bytes, a & b :-)
    uint8_t c; 
     c = *a;
    *a = *b;
    *b = c;
}
/************************************************************************************************************/
void CalibrateEdgeSwitches(){                                        // This function avoids the need to rotate the four edge switches if installed backwards
    for (int i = 0; i < 8; ++i) {
        if (digitalRead(SwitchNumber[i])){
            if (i == 0) swap(&SwitchNumber[i],&SwitchNumber[i+1]);   // swap over switches' pin number if wrongly installed    
            if (i == 2) swap(&SwitchNumber[i],&SwitchNumber[i+1]);   // swap over switches' pin number if wrongly installed        
            if (i == 4) swap(&SwitchNumber[i],&SwitchNumber[i+1]);   // swap over switches' pin number if wrongly installed      
            if (i == 6) swap(&SwitchNumber[i],&SwitchNumber[i+1]);   // swap over switches' pin number if wrongly installed           
        }         
    }
}  
/************************************************************************************************************/

FASTRUN void ReadSwitches()  // and indeed read digital trims if these are fitted
{
    byte flag = 0;
    for (int i = 0; i < 8; ++i) {
        Switch[i]     = !digitalRead(SwitchNumber[i]);    // These are reversed because they are active low
        TrimSwitch[i] = !digitalRead(TrimNumber[i]);      // These are reversed because they are active low
        if (TrimSwitch[i])  ++flag;                       // a finger is on a trim lever...   
        if ((TrimSwitch[i]) &&  (PreviousTrim != i)) {    // is it a new one?
            TrimTimer    = 0;                             // it IS a new one, so no delay please.  
            PreviousTrim = i;                             // remember which trim it was  
        }
    }
    if (flag > 1 ){                                       // one at a time please!!
        TrimRepeatSpeed = DefaultTrimRepeatSpeed;         // Restore default trim repeat speed
        for (int i = 0; i < 8; ++i) {
           (TrimSwitch[i]) = 0;
           flag = 0;
        }
    }
    if (!flag) { 
        PreviousTrim = 254;                               // Previous trim must now match none
        TrimRepeatSpeed = DefaultTrimRepeatSpeed;         // Restore default trim repeat speed
    }
    GetFlightMode();
    CheckHardwareTrims();
}
/************************************************************************************************************/

void GetRXVersionNumber()
{
    char nbuf[5];
    Str(nbuf,AckPayload.Byte1, 0);
    strcpy(ThisRadio, nbuf);
    if (LastRadio != AckPayload.Byte1) {
        LastRadio = AckPayload.Byte1;
        if (LogRXSwaps && UseLog) LogThisRX();
    }
    Str(ReceiverVersionNumber, AckPayload.Byte2, 2);
    Str(nbuf, AckPayload.Byte3, 2);
    strcat(ReceiverVersionNumber, nbuf);
    Str(nbuf, AckPayload.Byte4, 0);
    strcat(ReceiverVersionNumber, nbuf);
}

/************************************************************************************************************/

FASTRUN float GetFromAckPayload(){
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
FASTRUN void ParseAckPayload()
{
    if (AckPayload.Purpose & 0x80)                                       // Hi bit is now the **HOP NOW!!** flag
    {
        NextChannelNumber   =  AckPayload.Byte5;                         // This is just the array pointer or offset  
        NextChannel         =  * (FHSSChPointer + NextChannelNumber);    // The actual channel number pointed to.
        HopToNextChannel();
        AckPayload.Purpose &= 0x7f;                                      // Clear the high BIT, use the remainder ...
    }    
        switch (AckPayload.Purpose)                                      // Only look at the low 7 BITS
        {
            case 0:
                GetRXVersionNumber();
                break;
            case 1:
                SbusRepeats   = GetFromAckPayload(); 
                break;
            case 2:
                RadioSwaps     = GetFromAckPayload(); 
                break;
            case 3:
                RX1TotalTime   = GetFromAckPayload(); 
                break;
            case 4:
                RX2TotalTime   = GetFromAckPayload(); 
                break;
            case 5:
                 RXModelVolts = GetFromAckPayload();
                if (RXModelVolts > 0) {
                    RXVoltsDetected = true;
                    snprintf(ModelVolts, 5, "%f", RXModelVolts);
                }
                break;
            case 6:
                GetAltitude();
                break;
            case 7:
                GetTemperature();
                break;
            case 8:
                GPSLatitude = GetFromAckPayload(); 
                break;
            case 9:
                GPSLongitude = GetFromAckPayload(); 
                break;
            case 10:
                GPSAngle     = GetFromAckPayload();
                break;
            case 11:
                GPSSpeed     = GetFromAckPayload(); 
                if (GPSMaxSpeed < GPSSpeed) GPSMaxSpeed = GPSSpeed;
                break;
            case 12:
                GpsFix       =  GetFromAckPayload();
                break;
            case 13:
                GPSAltitude  = GetFromAckPayload() - GPSGroundAltitude;
                if (GPSAltitude < 0) GPSAltitude = 0;
                if (GPSMaxAltitude < GPSAltitude) GPSMaxAltitude = GPSAltitude;
                break;
            case 14:
                 GPSDistanceTo = GetFromAckPayload();
                 if (GPSMaxDistance < GPSDistanceTo) GPSMaxDistance = GPSDistanceTo;
                 break;
            case 15:
                 GPSCourseTo = GetFromAckPayload();  
                 break;
            case 16:
                 GPSSatellites = (uint8_t) GetFromAckPayload();
                 break;
            case 17:
                 GetDateFromAckPayload();
                 break;
            case 18:
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
FASTRUN void CheckGapsLength()
{
    if (GapStart > 0) { // when reconnected, how long was connection lost?
        ++GapCount;
        ThisGap = (millis() - GapStart); // AND in fact RX sends no data for 20 ms after reconnection
        if (ThisGap  >= MinimumGap  && UseLog ) LogThisGap();
        if (ThisGap > GapLongest)  GapLongest = ThisGap;
        GapSum += ThisGap;
        GapStart   = 0;
        GapAverage = GapSum / GapCount;
#ifdef DB_GAPS
        Serial.print("GapCount: ");
        Serial.println(GapCount);
        Serial.print("GapAverage: ");
        Serial.println(GapAverage);
        Serial.print("GapLongest: ");
        Serial.println(GapLongest);
        Serial.println(" ");
#endif
    }
}

/************************************************************************************************************/
void CheckModelName(){                               // In ModelsView, this function checks correct name is displayed.
char ModelsView_ModelNumber[]   = "ModelNumber";    
char ModelsView_ModelName[]     = "ModelName";
char NewName[35];
    if (!InhibitNameCheck){                          // If name is being edited, do not check it.
        ModelNumber = GetValue(ModelsView_ModelNumber); 
        GetText(ModelsView_ModelName,NewName);
        if (GetButtonPress()) ButtonWasPressed();     // Deal with button ... don't want to miss one!
        if (strlen(NewName) > 3)  {                   // Short texts come in from kbd screen
                if (strcmp(ModelName,NewName) != 0) { // Change?
                    strcpy(ModelName,NewName);        // Edited name!
                    SaveOneModel(ModelNumber);        // Save it!
                }
        }
        if (GetButtonPress()) ButtonWasPressed();     // Deal with button ... don't want to miss one!
        if (LastModelLoaded != ModelNumber) {
            if (ModelNumber >= 1) {                   // Don't use number zero
                 ReadOneModel(ModelNumber);   
                 if (UseLog) LogThisModel();
                 LastModelLoaded = ModelNumber;
                 UpdateModelsNameEveryWhere();  
            }
        }
    ClearText(); 
    }
}
/************************************************************************************************************/

void  CheckScanButton(){
        char b5Greyed[]= "b5.pco=33840";
        if (!LostContactFlag & !b5isGrey){
             SendCommand(b5Greyed);
             b5isGrey = true;
        }   
}

/************************************************************************************************************/
// LOOP
/************************************************************************************************************/
FASTRUN void loop()
{  
    KickTheDog();                    // Watchdog
    if (GetButtonPress()) {
        ButtonWasPressed();          // Deal with button
    }
    if ((millis()-ModelNameTimeCheck) > 1000) {  
        ModelNameTimeCheck  = millis();
        if (CurrentView == MAINSETUPVIEW){ 
            CheckScanButton();           
        }
        if (CurrentView == MODELSVIEW){ 
            CheckModelName();                                  // In MODELSVIEW, this function checks correct name is displayed.
            if (GetButtonPress()) ButtonWasPressed();          // Deal with button ... don't want to miss one!
        }
    }
    if (millis() - LastTimeRead >= 1000) {
        ReadTime();                  // Do the clock
        LastTimeRead = millis();
    }
    if (millis() - RangeTestStart >= 1000) {
        GetStatistics();             // Do stats
        RangeTestStart = millis();
    }
    if (PreviousUkRules != UkRules){
        LogUKRules();
        PreviousUkRules = UkRules;
    }
    ShowComms();                              // Screen Data 
    ReadSwitches();                           // Check switch positions
    CheckTimer();                             // Screen Timer 
    GetNewChannelValues();                    // Load SendBuffer with new servo positions
    if (UseMacros) ExecuteMacro();            // Modify it if macro is running
    ShowServoPos();                           // Servo positions use channel values
    if (!BoundFlag)  BufferNewPipe();         // if not yet bound, insert our pipe into sendbuffer
    if (BuddyMaster) GetSlaveChannelValues(); // If buddy master, check where student's sticks etc. are.
    Compress(CompressedData, SendBuffer, UNCOMPRESSEDWORDS);               // Compress 32 bytes down to 24
   
    if ((millis() - TxOnTime) > 2000) {                                    // Transmit nothing for first 2 seconds

        switch (CurrentMode) {
            case NORMAL:            // 0
                SendData();
                break;
            case CALIBRATELIMITS:   // 1
                CalibrateSticks();
                break;
            case CENTRESTICKS:      // 2
                ChannelCentres();
                break;
            case SCANWAVEBAND:      // 3
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
            MakeBindButtonInvisible();
            SetThePipe(NewPipe);
            BindingNow  = 0;
            BoundFlag   = true;
            LostPackets = 0;
            GapLongest  = 0;
            GapSum      = 0;
            GapAverage  = 0;
            GapCount    = 0;
            GreenLedOn();
          
        }
    }
} // end loop()
