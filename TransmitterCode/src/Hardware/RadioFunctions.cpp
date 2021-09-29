
#include "Arduino.h"

#include "RadioFunctions.h"  


#define SticksView         1
#define GraphView          2
#define MixesView          3
#define FhssView           4
#define ModelsView         5
#define CalibrateView      6 
#define MainSetupView      7
#define GainsView          8
#define DataView           9
#define Trim_View          10
#define Mode_View          11
#define Switches_View      12
#define One_Switch_View    13
#define Help_View          14
#define Options_View       15
#define BINDPIPETIMEOUT    10                        // timeout for switching from Bound to Default pipe
#define FHSS_RESCUE_BOTTOM 118                       // reduced range for recovery
#define FHSS_RESCUE_TOP    125                       // reduced range for recovery
#define UNCOMPRESSEDWORDS  20                        // DATA TO SEND = 40  Bytes
#define COMPRESSEDWORDS    UNCOMPRESSEDWORDS * 3 / 4 // COMPRESSED DATA SENT = 30  Bytes
#define PACKETS_PER_HOP    20                        // Must match RX setting


/************************************************************************************************************/
/************************************************************************************************************/

void TryOtherPipe()
{
    if (BoundFlag == true) {
        BoundFlag = false;
        SetThePipe(DefaultPipe);   
    }
    else 
    {
        BoundFlag = true;
        SetThePipe(NewPipe); 
    }
}
/************************************************************************************************************/

#define PACEMAKER          5                         // MINIMUM Ms between packets of data. - Probably needs to be between 7 and 20
void SendData()
{
    if ((millis() - TxPace) >= PACEMAKER) {
        TxPace = millis();
        get_new_channels_values(); // Load SendBuffer with new servo positions

        if (SetupFlag) {
            ReadSwitches();
            return; // Don't try to send data when just setting up.
        } 
        if (!BoundFlag && !(CurrentView == CalibrateView) && !(CurrentView == SticksView)) {
            SendBuffer[0] = (byte)((NewPipe >> 56) & 0xFF); // if not yet bound, send pipe
            SendBuffer[1] = (byte)((NewPipe >> 48) & 0xFF);
            SendBuffer[2] = (byte)((NewPipe >> 40) & 0xFF);
            SendBuffer[3] = (byte)((NewPipe >> 32) & 0xFF);
            SendBuffer[4] = (byte)((NewPipe >> 24) & 0xFF);
            SendBuffer[5] = (byte)((NewPipe >> 16) & 0xFF);
            SendBuffer[6] = (byte)((NewPipe >> 8) & 0xFF);
            SendBuffer[7] = (byte)((NewPipe)&0xFF);
        }
        LoadPacketData();
        if (JustHoppedFlag) {
            GetNextHopChannelNumber();
            JustHoppedFlag = false;
        }
        if (LostContactFlag) {
            ShowComms();
            if ((millis() - PipeTimeout) > BINDPIPETIMEOUT) {
                TryOtherPipe();
            }
        }
        if (LostPacketFlag) {
            if ((millis() - RecoveryTimer) > 75) {
                NextFrequency = random(FHSS_RESCUE_BOTTOM, FHSS_RESCUE_TOP); // more limited range for recovery
                HopToNextFrequency();
                RecoveryTimer = millis();
#ifdef DB_CHANNEL_AVOID
                Serial.print("Reconnect channel: ");
                Serial.println(NextFrequency);
#endif
            }
        }
        Connected = false;
        compress.Comp(CompressedData, SendBuffer, UNCOMPRESSEDWORDS); // Compress with my library - 32 bytes down to 24
        if (Radio1.write(&CompressedData, 30)) {  // ********** ACTUALLY SEND THE DATA *************
            if (Radio1.isAckPayloadAvailable()) {
                Radio1.read(&AckPayLoad, 15);
                RangeTestGoodPackets++;
                Connected = true;
                if (BoundFlag) {
                    GreenLedOn();
                }
            }
            else {
                RangeTestLostPackets++;
            }
            CheckGapsLength();
            LostPacketFlag  = false;
            LostContactFlag = false;
            PacketNumber++;
            ReadExtraData();
            RecentPacketsLost = 0;
            if (PacketNumber > PACKETS_PER_HOP) {
                HopToNextFrequency();
            }
        }
        else {
            FailedPacket();
        }
    }
}

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

/************************************************************************************************************/

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