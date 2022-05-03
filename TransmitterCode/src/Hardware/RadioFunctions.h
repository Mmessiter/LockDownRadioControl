
#ifndef RadioFunctions_H
#define RadioFunctions_H

// **************************************************************************
//                TX VERSION NUMBER   (April 12th 2022 Malcolm Messiter) *
//***************************************************************************
#define TXVERSION_MAJOR   1
#define TXVERSION_MINOR   7
#define TXVERSION_MINIMUS 0


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

#define MINMICROS 500
#define MAXMICROS 2500

// **************************************************************************
//                            FHSS PARAMETERS                               *
//***************************************************************************

#define PACEMAKER                   7   // MINIMUM ms between sent packets of data. These brief pauses allow the receiver to poll its i2c Sensor hub, and TX to ShowComms();
#define RETRYCOUNT                  3   // auto retries from nRF24L01
#define RETRYWAIT                   1   // Wait between retries is RetryWait+1 * 250us. A failed packet therefore takes (RetryWait+1 * 250us) * RetryCount
#define LOSTCONTACTCUTOFF           2   // How many packets to lose before reconnect triggers  (>2)
#define RECONNECT_CHANNELS_COUNT    3   // How many channels to try when reconnecting
#define RECONNECT_CHANNELS_START    12
#define RED_LED_ON_TIME             500 // How many ms of no connection before RED led comes on

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
#define ShowCommsDelay 500   // 250   // ms pauses between updated info on NEXTION
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

// #define DB_NEXTION        // Debug NEXTION and SD card data
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
extern struct Payload AckPayload;
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
