
#include "Arduino.h"
#include "RadioFunctions.h"  




/************************************************************************************************************/

/************************************************************************************************************/
#define xx1 90 // was 75
#define yy1 90 // Needed below... Edit xx1,yy1 to move box

/** @brief This scans and displays result */
void ScanAllChannels()
{
    int  x1 = xx1;
    int  y1 = yy1;
    int  Sc;
    int  x2, y2;
    int  BlobHeight = 4; // Blobs are 4x4 pixels
    char NB[12];         // Number Buffer
    char NB1[12];
    char NB2[12];
    char NB3[12];
    char NB4[12];
    char CB[100]; // COMMAND BUFFER
    char fyll[]   = "fill ";
    char IELLOW[] = "YELLOW";
    char NA[1]    = ""; // blank one

    for (Sc = ScanStart; Sc <= ScanEnd; Sc++) {
        Radio1.setChannel(Sc);
        Radio1.startListening();
        x2 = x1 + (Sc * 5);
        y2 = y1 + 255;
        y2 = y2 - BlobHeight;
        y2 = y2 - AllChannels[Sc];
        delayMicroseconds(120); // Minimum!?
        Radio1.stopListening();
        if (Radio1.testCarrier()) {
            if (AllChannels[Sc] < (250)) {
                AllChannels[Sc] += BlobHeight;
                SendCharArray(CB, fyll, Str(NB, x2, 1), Str(NB1, (y2), 1), Str(NB2, 5, 1), Str(NB3, BlobHeight, 1), IELLOW, NA, NA, NA, NA, NA, NA);
            }
        }
        else {
            NoCarrier[Sc]++;
            if (NoCarrier[Sc] > 15) { // must see no carrier >15 times before reducing the trace
                if (AllChannels[Sc] >= (BlobHeight)) {
                    SendCharArray(CB, fyll, Str(NB, x2, 1), Str(NB1, (y2 + BlobHeight), 1), Str(NB2, 5, 1), Str(NB3, BlobHeight, 1), Str(NB4, 214, 0), NA, NA, NA, NA, NA, NA);
                    AllChannels[Sc] -= (BlobHeight);
                    NoCarrier[Sc] = 0;
                }
            }
        }
    }
}





void HopToNextFrequency()
{
    Radio1.setChannel(NextFrequency);
    Radio1.stopListening();
    ReadSwitches();
    ShowComms();
    PacketNumber = 0;
    CheckTimer(); // update timer if on
#ifdef DB_FHSS
    PENDTIME  = millis();
    PDURATION = (PENDTIME - PSTARTTIME) / 1000;
    Serial.print("Hop duration: ");
    Serial.print(PDURATION);
    Serial.print(" seconds. Good packets per hop: ");
    Serial.print(PACKETS_PER_HOP);
    Serial.print(" Next channel: (range: ");
    Serial.print(FHSSBottom);
    Serial.print("-");
    Serial.print(FHSSTop);
    Serial.print(") ");
    Serial.println(NextFrequency);
    PSTARTTIME = millis();
#endif
    ThisFrequency  = NextFrequency;
    JustHoppedFlag = true;
}



/*********************************************************************************************************************************/

/** @brief This scans quietly at startup */
void PreScan()
{
    int ScanTime = 0;
#ifdef DB_CHANNEL_AVOID
    int scount = 0;
#endif
    int Sc;
#ifdef DB_CHANNEL_AVOID
    Serial.println("Prescanning ...");
#endif
    DoScanInit();
    ScanTime = millis();
    while ((millis() - ScanTime) < 1000) {
        KickTheDog(); // Watchdog
        for (Sc = ScanStart; Sc <= ScanEnd; Sc++) {
            Radio1.setChannel(Sc);
            Radio1.startListening();
            delayMicroseconds(120); // Minimum!?
            Radio1.stopListening();
            if (Radio1.testCarrier()) {
                AllChannels[Sc]++;
            }
        }
    }
    for (Sc = ScanStart + 1; Sc <= ScanEnd - 1; Sc++) {
        if (AllChannels[Sc] > 0) {
            if (BadChannelPointer < BAD_CHANNEL_MAX) {
                BadChannels[BadChannelPointer] = Sc;
                BadChannelPointer++;
            }
#ifdef DB_CHANNEL_AVOID
            Serial.println(Sc);
            scount++;
#endif
        }
    }
    DoScanEnd();
#ifdef DB_CHANNEL_AVOID
    Serial.print("Found ");
    Serial.println(scount);
#endif
}


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