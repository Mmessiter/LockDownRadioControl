
#include "Arduino.h"
#include "RadioFunctions.h"  

/*********************************************************************************************************************************/

void InitRadio(uint64_t Pipe)
{
    Radio1.begin();
    switch (PowerSetting) {
        case 1: Radio1.setPALevel(RF24_PA_MIN); break;
        case 2: Radio1.setPALevel(RF24_PA_LOW); break;
        case 3: Radio1.setPALevel(RF24_PA_HIGH); break;
        case 4: Radio1.setPALevel(RF24_PA_MAX); break;
        default: break;
    }

    switch (DataRate) {
        case 1: Radio1.setDataRate(RF24_250KBPS); break;
        case 2: Radio1.setDataRate(RF24_1MBPS); break;
        case 3: Radio1.setDataRate(RF24_2MBPS); break;
        default: break;
    }
    Radio1.enableAckPayload();                  // Needed
    Radio1.openWritingPipe(Pipe);               // Current Pipe address used for Binding
    Radio1.setRetries(15, 15);                  // Max automatic retries = (15,15). Packet failure will take 0.06 seconds
    Radio1.stopListening();                     // It's a true Messiter
    Radio1.enableDynamicPayloads();             // Needed
    Radio1.setAddressWidth(5);                  // was 4, is now 5
    Radio1.setCRCLength(RF24_CRC_8);            // could be 16 
    PipeTimeout = millis();                     // Initialise timeout
    GapSum      = 0;
}
/*********************************************************************************************************************************/

   void SetThePipe(uint64_t WhichPipe)
{ 
        Radio1.openWritingPipe(WhichPipe);
        Radio1.stopListening();
}

/*********************************************************************************************************************************/

void DoScanInit()
{
    Radio1.setDataRate(RF24_1MBPS); // Scan only works at this default rate
    CurrentMode = SCANWAVEBAND;     // Fhss == No transmitting please, we are scanning.
    SendCommand(NoSleeping);
    for (i = 0; i < 127; i++) {
        NoCarrier[i]   = 0;
        AllChannels[i] = 0;
    }
}

/*********************************************************************************************************************************/

void DoScanEnd()
{
    Radio1.setDataRate(RF24_250KBPS);
    CurrentMode = NORMAL;
    SendCommand(NextionSleepTime);
}