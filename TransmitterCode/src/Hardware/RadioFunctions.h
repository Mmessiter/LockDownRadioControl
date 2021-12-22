
#ifndef RadioFunctions_H
#define RadioFunctions_H

 // **************************************************************************
 //                     TX VERSION NUMBER   (Dec 21st 2021 Malcolm Messiter) *
 //***************************************************************************
#define TXVERSION_MAJOR   1  
#define TXVERSION_MINOR   4
#define TXVERSION_MINIMUS 4
 
 // **************************************************************************
 //                            SBUS PARAMETERS   (FOR BUDDY BOXING)          *
 //***************************************************************************

#define SBUSRATE        10        // SBUS frame every 10 milliseconds ( = 100 Hz)
#define SBUSPORT        Serial2
#define RANGEMAX        2047      // = Frsky at 150 %
#define RANGEMIN        0         // = Frsky at 0 %

 // **************************************************************************
 //                            SERVO RANGE PARAMETERS                        *
 //***************************************************************************

#define MINMICROS   500 
#define MAXMICROS   2500 

 // **************************************************************************
 //                            FHSS PARAMETERS                               *
 //***************************************************************************

#define HOPTIME           55    // A New frequency hop every 55 ms (must match receiver setting)
#define PACEMAKER         2     // MINIMUM Ms between packets of data.
#define FREQUENCYSCOUNT   82    // How many frequencies to use before wrapping to first
#define RETRYCOUNT        3     // auto retries from nRF24L01
#define RETRYWAIT         1     // NB ACTUAL wait between retries will be RetryWait+1 * 250us
                                // A failed packet therefore takes (RetryWait+1 * 250us) * RetryCount 
// #define NOISYWIFI            // if defined (BOTH ENDS!) this uses channels well above most wifi (but not licence free in some countries).
#define LOSTCONTACTCUTOFF 1     // How many packets to lose before reconnect triggers  (>1)
#ifndef NOISYWIFI               // Use this for UK legal flying 
#define RECONNECT_CH     83
#endif
#ifdef NOISYWIFI 
#define RECONNECT_CH     120
#endif

 // **************************************************************************
 //                            SEND MODE PARAMETERS                          * 
 //***************************************************************************

#define NORMAL          0 // Normal for transmit as usual
#define CALIBRATELIMITS 1 // Calibrate limits
#define CENTRESTICKS    2 // Calibrate Centres
#define SCANWAVEBAND    3 // Scan waveband
#define SENDNOTHING     4 // Transmission off
 // **************************************************************************

 // **************************************************************************
 //                          NEXTION SERIAL CONNECTION                       *
 //***************************************************************************
#define Nextion         Serial1 // Nextion is connected to Serial1
// ***************************************************************************

 // **************************************************************************
 //                            WATCHDOG PARAMETERS                           *
 //***************************************************************************
#define USE_WATCHDOG         
#define WATCHDOGTIMEOUT   10000 // 10 Seconds before reboot (32ms -> 500 seconds)
#define KICKRATE          1000  // Kick once a second (must be between WATCHDOGMAXRATE and WATCHDOGTIMEOUT)
#define WATCHDOGMAXRATE   500   // 500 ms secs between kicks is max rate allowed

// **************************************************************************

 // **************************************************************************
 //                     DEBUG OPTIONS                                        *
 //             UNCOMMENT ANY OF THESE for that bit of debug info            *
 //***************************************************************************

// #define DB_NEXTION        // Debug Nextion and SD card data
// #define DB_FHSS           // Debug real time FHSS data
// #define DB_FHSS1          // Debug new FHSS data
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
extern unsigned int   i;
extern char           NoSleeping[];
extern char           NextionSleepTime[];
extern uint8_t        ScanStart;
extern uint8_t        ScanEnd;
extern uint8_t        BadChannelPointer;
extern uint8_t        BadChannels[];
extern uint8_t        NextFrequency;
extern uint8_t        PacketNumber;
extern uint8_t        ThisFrequency;
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
extern uint32_t       RXTimeStamp;
extern bool           DoSbusSendOnly;
extern bool           BuddyMaster;


extern void  GetSlaveChannelValues();
extern void  KickTheDog();
extern void  SendCommand(char* tbox);
extern void  ReadSwitches();
extern void  ShowComms();
extern void  CheckTimer();
extern void  SendCharArray(char* ch0, char* ch1, char* ch2, char* ch3, char* ch4, char* ch5, char* ch6, char* ch7, char* ch8, char* ch9, char* ch10, char* ch11, char* ch12);
extern char* Str(char* s, int n, int comma);
extern void  get_new_channels_values();
extern void  LoadPacketData();
extern void  GetNextHopChannelNumber();
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
void HopToNextFrequency();
void ScanAllChannels();
void SendData();

/*********************************************************************************************************************************/


#endif
