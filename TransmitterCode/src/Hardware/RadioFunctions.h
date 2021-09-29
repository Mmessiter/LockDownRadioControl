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

/*********************************************************************************************************************************/
// function prototypes

void InitRadio(uint64_t Pipe);
void SetThePipe(uint64_t WhichPipe);
void DoScanInit();
void DoScanEnd();

/*********************************************************************************************************************************/

#endif