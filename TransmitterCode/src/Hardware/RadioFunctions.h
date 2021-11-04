/* All radio functions will be moved here.
*/

#ifndef RadioFunctions_H
#define RadioFunctions_H

#include <RF24.h>

#define NORMAL          0 // Normal for transmit as usual
#define CALIBRATELIMITS 1 // Calibrate limits
#define CENTRESTICKS    2 // Calibrate Centres
#define SCANWAVEBAND    3 // Scan waveband
#define SENDNOTHING     4 // Transmission off
#define BAD_CHANNEL_MAX 40
#define Nextion         Serial1 // Nextion is connected to Serial1

/*********************************************************************************************************************************/
// external (global vars) needed here

extern RF24           Radio1;
extern uint8_t        PowerSetting;
extern uint8_t        DataRate;
extern int            PipeTimeout;
extern int            GapSum;
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
extern bool           SetupFlag;
extern bool           BoundFlag;
extern uint8_t        CurrentView;
extern uint16_t       SendBuffer[];
extern uint64_t       NewPipe;
extern bool           LostContactFlag;
extern uint64_t       DefaultPipe;
extern long int       RecoveryTimer;
extern bool           Connected;
extern uint16_t       CompressedData[];
extern struct Payload AckPayload;
extern int            RangeTestLostPackets;
extern uint8_t        RecentPacketsLost;
extern uint8_t        AckPayloadSize;
extern uint8_t        SizeOfCompressedData;
extern int            RangeTestGoodPackets;

extern uint8_t FHSSBottom;
extern uint8_t FHSSTop;
extern uint32_t TxOnTime;

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
extern uint32_t   GapStart;
extern uint8_t BadChannelMax;

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
