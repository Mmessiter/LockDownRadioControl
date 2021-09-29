/* All radio functions will be moved here.
*/

#ifndef RadioFunctions_H    
#define RadioFunctions_H

#include "RF24.h"

#define NORMAL          0 // Normal for transmit as usual
#define CALIBRATELIMITS 1 // Calibrate limits
#define CENTRESTICKS    2 // Calibrate Centres
#define SCANWAVEBAND    3 // Scan waveband
#define SENDNOTHING     4 // Transmission off
#define BAD_CHANNEL_MAX    40  

/*********************************************************************************************************************************/
// external (global vars) needed here

extern RF24 Radio1;
extern byte PowerSetting;
extern byte DataRate;
extern int  PipeTimeout;   
extern int  GapSum;    
extern byte CurrentMode; 
extern byte NoCarrier[];
extern byte AllChannels[];
extern unsigned int i;
extern void SendCommand(char* tbox);
extern char NoSleeping[];
extern char NextionSleepTime[];
extern byte ScanStart;
extern byte ScanEnd;
extern byte BadChannelPointer;
extern byte BadChannels[];
extern byte NextFrequency;  
extern byte PacketNumber;
extern byte ThisFrequency; 
extern bool JustHoppedFlag; 

extern void KickTheDog();
extern void ReadSwitches();
extern void ShowComms();
extern void CheckTimer();
extern void SendCharArray(char* ch0, char* ch1, char* ch2, char* ch3, char* ch4, char* ch5, char* ch6, char* ch7, char* ch8, char* ch9, char* ch10, char* ch11, char* ch12);
extern char* Str(char* s, int n, int comma);

/*********************************************************************************************************************************/
// function prototypes

void InitRadio(uint64_t Pipe);
void SetThePipe(uint64_t WhichPipe);
void DoScanInit();
void DoScanEnd();
void PreScan();
void HopToNextFrequency();
void ScanAllChannels();

/*********************************************************************************************************************************/

#endif