
/*********************************************************************************************************************************
*                  All radio functions will be moved here.
**********************************************************************************************************************************/

#ifndef RadioFunctions_H    
#define RadioFunctions_H

#include "RF24.h"
#include "compress.h"

#define NORMAL          0 // Normal for transmit as usual
#define CALIBRATELIMITS 1 // Calibrate limits
#define CENTRESTICKS    2 // Calibrate Centres
#define SCANWAVEBAND    3 // Scan waveband
#define SENDNOTHING     4 // Transmission off
#define BAD_CHANNEL_MAX    40  

/*********************************************************************************************************************************/
// external (global vars) needed here

extern RF24     Radio1;
extern Compress compress;
extern byte     PowerSetting;
extern byte     DataRate;
extern int      PipeTimeout;   
extern int      GapSum;    
extern byte     CurrentMode; 
extern byte     NoCarrier[];
extern byte     AllChannels[];
extern char     NoSleeping[];
extern char     NextionSleepTime[];
extern byte     ScanStart;
extern byte     ScanEnd;
extern byte     BadChannelPointer;
extern byte     BadChannels[];
extern byte     NextFrequency;  
extern byte     PacketNumber;
extern byte     ThisFrequency; 
extern bool     JustHoppedFlag; 
extern uint32_t TxPace;  
extern bool     SetupFlag;
extern bool     BoundFlag;  
extern byte     CurrentView ;
extern uint16_t SendBuffer[];
extern uint64_t NewPipe;
extern bool     LostContactFlag;
extern uint64_t DefaultPipe;
extern bool     LostPacketFlag ;
extern long int RecoveryTimer;
extern bool     Connected; 
extern uint16_t CompressedData[];
extern char     AckPayLoad[];
extern int      RangeTestLostPackets;
extern byte     RecentPacketsLost; 
extern int      RangeTestGoodPackets;
extern unsigned int i;

/*********************************************************************************************************************************/
// external functions needed here

extern void SendCommand(char* tbox);
extern void KickTheDog();
extern void ReadSwitches();
extern void ShowComms();
extern void CheckTimer();
extern void SendCharArray(char* ch0, char* ch1, char* ch2, char* ch3, char* ch4, char* ch5, char* ch6, char* ch7, char* ch8, char* ch9, char* ch10, char* ch11, char* ch12);
extern char* Str(char* s, int n, int comma);
extern void get_new_channels_values();
extern void LoadPacketData();
extern void GetNextHopChannelNumber();
extern void GreenLedOn();
extern void CheckGapsLength();
extern void ReadExtraData();
extern void FailedPacket();

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