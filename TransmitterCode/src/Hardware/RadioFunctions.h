
#ifndef RadioFunctions_H
#define RadioFunctions_H

// **************************************************************************
//                TX VERSION NUMBER   (June 9th 2022 Malcolm Messiter) *
//***************************************************************************
#define TXVERSION_MAJOR   1
#define TXVERSION_MINOR   7
#define TXVERSION_MINIMUS 7



#define CHANNELSUSED       16                  // 16 Channels
#define MAXMIXES           32                  // 32 mixes
#define TICKSPERMINUTE     60000               // millis() += 60000 per minute
#define PROPOCHANNELS      8                   // Only 4 have knobs / 2 sticks (= 4 hall sensors)
#define FLIGHTMODESWITCH   4                   // Default MODE switch
#define AUTOSWITCH         1                   // Default AUTO switch
#define DEFAULTPIPEADDRESS 0xBABE1E5420LL      // Pipe address for startup - any value but MUST match RX
#define LOWBATTERY         42                  // percent for warning (User definable now)
#define CE_PIN             9                   // for SPI to nRF24L01
#define CSN_PIN            10                  // for SPI to nRF24L01
#define INACTIVITYTIMEOUT  10 * TICKSPERMINUTE // Default time after which to switch off
#define INACTIVITYMINIMUM  5  * TICKSPERMINUTE // Inactivity timeout minimum is 5 minutes
#define INACTIVITYMAXIMUM  30 * TICKSPERMINUTE // Inactivity timeout maximum is 30 minutes
#define DS1307_ADDRESS     0x68



// **************************************************************************
//                              Macros                                      *
// **************************************************************************

#define MAXMACROS               8                       // 8 macros enough for now?
#define BYTESPERMACRO           6                       // 6 bytes each         

// ********************* Offsets within macros' buffer ***********************

#define MACROTRIGGERCHANNEL     0                       // 1 - 16. 0 means dissabled.
#define MACROSTARTTIME          1                       // In ** >> 10ths << ** of a second since trigger. ( = millis() * 100 ) up to 25.4 seconds
#define MACRODURATION           2                       // In ** >> 10ths << ** of a second since start    ( = millis() * 100 ) up to 25.4 seconds
#define MACROMOVECHANNEL        3                       // Which channel to move.
#define MACROMOVETOPOSITION     4                       // Where to put said channel for said duration. (0 -180)
#define MACRORUNNINGNOW         5                       // Running flag (BIT 0 running/not running,  BIT 1 = Timer active / inactive)

// **************************************************************************
//                    AUDIO (ie  INTELLIGENT NEXTION)                       *
//***************************************************************************
#define USEAUDIO

// **************************************************************************
//                            SBUS PARAMETERS   (FOR BUDDY BOXING)          *
//***************************************************************************

#define SBUSRATE 10 // SBUS frame every 10 milliseconds ( = 100 Hz)
#define SBUSPORT Serial2
#define RANGEMAX 2047 // = Frsky at 150 %
#define RANGEMIN 0    // = Frsky at 0 %

// **************************************************************************
//                            SERVO RANGE PARAMETERS                        *
//***************************************************************************

#define MINMICROS    500
#define MAXMICROS    2500
#define DEFAULT_EXPO 50 // = ZERO EXPO (Range is 0 - 200. Below 50 is negative Expo)

// **************************************************************************
//                            FHSS PARAMETERS                               *
//***************************************************************************

#define PACEMAKER                   8  // (was 7) MINIMUM ms between sent packets of data. These brief pauses allow the receiver to poll its i2c Sensor hub, and TX to ShowComms();
#define RETRYCOUNT                  3   // auto retries from nRF24L01
#define RETRYWAIT                   1   // Wait between retries is RetryWait+1 * 250us. A failed packet therefore takes (RetryWait+1 * 250us) * RetryCount
#define LOSTCONTACTCUTOFF           6   // How many packets to lose before reconnect triggers  (>6)
#define RECONNECT_CHANNELS_COUNT    3   // How many channels to try when reconnecting
#define RECONNECT_CHANNELS_START    12
#define RED_LED_ON_TIME             1000 // How many ms of no connection before RED led comes on

// **************************************************************************
//                            SEND MODE PARAMETERS                          *
//***************************************************************************

#define NORMAL          0 // Normal for transmit as usual
#define CALIBRATELIMITS 1 // Calibrate limits
#define CENTRESTICKS    2 // Calibrate Centres
#define SCANWAVEBAND    3 // Scan waveband
#define SENDNOTHING     4 // Transmission off
                          // ************************************************

// **************************************************************************
//                          NEXTION SERIAL CONNECTION                       *
//***************************************************************************
#define NEXTION Serial1      // NEXTION is connected to Serial1
#define SHOWCOMMSDELAY 500   // 250   // ms pauses between updated info on NEXTION
// ***************************************************************************

// **************************************************************************
//                            WATCHDOG PARAMETERS                           *
//***************************************************************************

#define USE_WATCHDOG
#define WATCHDOGTIMEOUT 10000 // 10 Seconds before reboot (32ms -> 500 seconds)
#define KICKRATE        1000  // Kick once a second (must be between WATCHDOGMAXRATE and WATCHDOGTIMEOUT)
#define WATCHDOGMAXRATE 500   // 500 ms secs between kicks is max rate allowed

// **************************************************************************

// **************************************************************************
//                     DEBUG OPTIONS                                        *
//             UNCOMMENT ANY OF THESE for that bit of debug info            *
//***************************************************************************

 //  #define DB_NEXTION        // Debug NEXTION and SD card data
// #define DB_FHSS           // Debug real time FHSS data
// #define DB_SENSORS        // Debug Sensors
// #define DB_BIND           // Debug Binding
// #define DB_SWITCHES       // Debug Switches
// #define DB_MODEL_EXCHANGE // Debug MODEL EXCHANGE (by RF link)
// #define DB_GAPS           // Debug Connection Gap assessment

#include <RF24.h>

/*********************************************************************************************************************************/
// external (global vars) needed here

extern RF24           Radio1;
extern int            PipeTimeout;
extern uint8_t        CurrentMode;
extern uint8_t        NoCarrier[];
extern uint8_t        AllChannels[];
extern char           NoSleeping[];
extern char           NEXTIONSleepTime[];
extern uint8_t        ScanStart;
extern uint8_t        ScanEnd;
extern uint8_t        BadChannelPointer;
extern uint8_t        BadChannels[];
extern uint8_t        NextChannel;
extern uint8_t        PacketNumber;
extern bool           JustHoppedFlag;
extern uint32_t       TxPace;
extern bool           BoundFlag;
extern uint8_t        CurrentView;
extern uint16_t       SendBuffer[];
extern uint64_t       NewPipe;
extern bool           LostContactFlag;
extern uint64_t       DefaultPipe;
extern long int       RecoveryTimer;
extern bool           Connected;
extern uint16_t       CompressedData[];
extern uint8_t        FHSS_Channels[];
extern struct         Payload AckPayload;
extern int            RangeTestLostPackets;
extern uint8_t        RecentPacketsLost;
extern uint8_t        AckPayloadSize;
extern uint8_t        SizeOfCompressedData;
extern int            RangeTestGoodPackets;
extern uint8_t        NextChannelNumber;
extern uint32_t       TotalledRecentPacketsLost;
extern uint32_t       TxOnTime;
extern uint32_t       TXTimeStamp;
extern uint32_t       HopStart;
extern char           ThisRadio[4];
extern uint32_t       GapSum;
extern uint32_t       GapStart;
extern uint8_t *      FHSSChPointer;
extern bool           DoSbusSendOnly;
extern bool           BuddyMaster;
extern uint16_t       BackGroundColour;
extern uint16_t       HighlightColour;
extern uint16_t       ForeGroundColour;
extern uint16_t       ChannelMax[CHANNELSUSED + 1];       //    output of pots at max
extern uint16_t       ChannelMidHi[CHANNELSUSED + 1];     //    output of pots at MidHi
extern uint16_t       ChannelCentre[CHANNELSUSED + 1];    //    output of pots at Centre
extern uint16_t       ChannelMidLow[CHANNELSUSED + 1];    //    output of pots at MidLow
extern uint16_t       ChannelMin[CHANNELSUSED + 1];       //    output of pots at min
extern uint8_t        MacrosBuffer[MAXMACROS][BYTESPERMACRO];    // macros' buffer
extern uint32_t       MacroStartTime[MAXMACROS];
extern uint32_t       MacroStopTime[MAXMACROS];
extern bool           UseMacros;
extern unsigned int   LostPackets;



extern void  GetSlaveChannelValues();
extern void  KickTheDog();
extern void  SendCommand(char* tbox);
extern void  ReadSwitches();
extern void  ShowComms();
extern void  CheckTimer();
extern void  SendCharArray(char* ch0, char* ch1, char* ch2, char* ch3, char* ch4, char* ch5, char* ch6, char* ch7, char* ch8, char* ch9, char* ch10, char* ch11, char* ch12);
extern char* Str(char* s, int n, int comma);
extern void  GetNewChannelValues();
extern void  LoadPacketData();
extern void  GreenLedOn();
extern void  CheckGapsLength();
extern void  ParseAckPayload();
extern void  FailedPacket();
extern void  StartInactvityTimeout();
extern void  ShowServoPos();
extern void  MapToSBUS();
extern void  ZeroDataScreen();


/*********************************************************************************************************************************/
// function prototypes

void InitRadio(uint64_t Pipe);
void SetThePipe(uint64_t WhichPipe);
void DoScanInit();
void DoScanEnd();
void PreScan();
void HopToNextChannel();
void ScanAllChannels();
void SendData();
void Procrastinate(uint32_t HowLong);
void DrawFhssBox(); 

/*********************************************************************************************************************************/

#endif
