
/************************************************************************************************************/
//                                      Radio Functions
/************************************************************************************************************/
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
#define BINDPIPETIMEOUT    1000                      // timeout for switching from Bound to Default pipe
#define FHSS_RESCUE_BOTTOM 118                       // reduced range for recovery
#define FHSS_RESCUE_TOP    125                       // reduced range for recovery
#define UNCOMPRESSEDWORDS  20                        // DATA TO SEND = 40  Bytes
#define COMPRESSEDWORDS    UNCOMPRESSEDWORDS * 3 / 4 // COMPRESSED DATA SENT = 30  Bytes
#define PACKETS_PER_HOP    20                        // Must match RX setting

/************************************************************************************************************/

#ifdef DB_FHSS
float PStartTime = 0;
float PEndTime   = 0;
float Pduration  = 0;
#endif

// **** Decompresses cc*3/4 x 16 BIT values up to cc loading only their low 12 BITS cc must be divisble by 4! ******************
void DeComp(uint16_t* d, uint16_t* c, int cc)
{
    int p = 0, l = 0;
    for (l = 0; l < (cc * 3 / 4); l += 3) {
        d[p] = c[l] >> 4;
        p++;
        d[p] = (c[l] & 0xf) << 8 | c[l + 1] >> 8;
        p++;
        d[p] = (c[l + 1] & 0xff) << 4 | c[l + 2] >> 12;
        p++;
        d[p] = c[l + 2] & 0xfff;
        p++;
    }
}

/**
 * Compresses uint16_t* buffer values (each with 12 bit resolution - the lower 12 bits).
 * @param compressed_buf[out] Must have allocated 3/4 the size of uncompressed_buf
 * @param uncompressed_buf[in]
 * @param uncompressed_size Size is in units of uint16_t (aka word or unsigned short). This *must* be divisible by 4.
 */
void Compress(uint16_t* compressed_buf, uint16_t* uncompressed_buf, int uncompressed_size)
{
    int p = 0;
    for (int l = 0; l < (uncompressed_size * 3 / 4); l += 3) {
        compressed_buf[l] = uncompressed_buf[p] << 4 | uncompressed_buf[p + 1] >> 8;
        p++;
        compressed_buf[l + 1] = uncompressed_buf[p] << 8 | uncompressed_buf[p + 1] >> 4;
        p++;
        compressed_buf[l + 2] = uncompressed_buf[p] << 12 | uncompressed_buf[p + 1];
        p++;
        p++;
    }
}
/************************************************************************************************************/

void TryOtherPipe()
{
    if (TotalledRecentPacketsLost > 10 || (!BoundFlag))  {        // This perhaps avoids needless pipe swapping during poor connection
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
}
/************************************************************************************************************/

void BufferNewPipe()
{
            SendBuffer[0] = (uint8_t)((NewPipe >> 56) & 0xFF); // if not yet bound, send pipe
            SendBuffer[1] = (uint8_t)((NewPipe >> 48) & 0xFF);
            SendBuffer[2] = (uint8_t)((NewPipe >> 40) & 0xFF);
            SendBuffer[3] = (uint8_t)((NewPipe >> 32) & 0xFF);
            SendBuffer[4] = (uint8_t)((NewPipe >> 24) & 0xFF);
            SendBuffer[5] = (uint8_t)((NewPipe >> 16) & 0xFF);
            SendBuffer[6] = (uint8_t)((NewPipe >> 8) & 0xFF);
            SendBuffer[7] = (uint8_t)((NewPipe)&0xFF);
}
/************************************************************************************************************/

void SendData()
{
    if (Nextion.available()) return;   // was a button pressed?
   
    if ((millis() - TxPace) >= PACEMAKER) {
        TxPace = millis();
        get_new_channels_values(); // Load SendBuffer with new servo positions
        if (!BoundFlag && !(CurrentView == CalibrateView) && !(CurrentView == SticksView)) 
            {
            BufferNewPipe();       // if not yet bound, send our pipe
            }
        LoadPacketData();          // extra parameters appended to the data packet    
        if (LostContactFlag) {
                if ((millis() - PipeTimeout) > BINDPIPETIMEOUT) {
                        TryOtherPipe();
                        PipeTimeout=millis();
                }
                NextFrequency = RECONNECT_CH;
                HopToNextFrequency();
        }
        Connected = false;      
        Compress(CompressedData, SendBuffer, UNCOMPRESSEDWORDS);        // Compress 32 bytes down to 24
        Radio1.flush_rx();
        Radio1.flush_tx();

//  *************************************** SEND *************************************************************************************
        Radio1.write(&CompressedData, SizeOfCompressedData);  //  ******** !SEND! ********     
//  *************************************** SEND *************************************************************************************
        if (Radio1.isAckPayloadAvailable()) 
               {
                    (Radio1.read(&AckPayload, AckPayloadSize));         //  "sizeof" doesn't work with externs, hence 2 new vars.
                    ++RangeTestGoodPackets;
                    LostContactFlag = false;
                    ++PacketNumber;
                    ParseAckPayload();
                    RecentPacketsLost = 0;
                    TotalledRecentPacketsLost =0;
                    Connected = true;
                    if (BoundFlag) GreenLedOn();    
                    CheckGapsLength();
                    StartInactvityTimeout();
               }
        else 
               {
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
    for (Sc = ScanStart; Sc <= ScanEnd; ++Sc) {
        if (Nextion.available()) return; // in case someone wants to stop!
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
            ++NoCarrier[Sc];
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
    Radio1.stopListening();   // needed???? Heer......
    delay(4);
    ReadSwitches();
    CheckTimer(); // update timer if on
    ShowComms();
#ifdef DB_FHSS
    PEndTime  = millis();
    Pduration = (PEndTime - PStartTime) / 1000;
    Serial.print("Hop duration: ");
    Serial.print(Pduration);
    Serial.print(" seconds. Good packets per hop: ");
    Serial.print(PacketNumber);
    Serial.print(" Next channel: ");
    Serial.print(FHSS_Channels[NextChannelNumber]);
    if ((FHSS_Channels[NextChannelNumber]) < 10) Serial.print(" ");
    Serial.print(BoundFlag ? " Bound!" : " NOT BOUND.");
    Serial.print(" RX Radio: ");
    Serial.println(ThisRadio);
    PStartTime = millis();
#endif
    ThisFrequency  = NextFrequency;
    PacketNumber = 0;
}

/*********************************************************************************************************************************/

void InitRadio(uint64_t Pipe)
{
    Radio1.begin();
    Radio1.setPALevel(RF24_PA_MAX);
    Radio1.setDataRate(RF24_250KBPS);
    Radio1.enableAckPayload();                        
    Radio1.openWritingPipe(Pipe);                     // Current Pipe address used for Binding
    Radio1.setRetries(RETRYCOUNT, RETRYWAIT);         // automatic retries *** WAS 15,15 *** !! 
    Radio1.stopListening();          
    Radio1.enableDynamicPayloads();  
    Radio1.setAddressWidth(5);       // was 4, is now 5
    Radio1.setCRCLength(RF24_CRC_8); // could be 16
    PipeTimeout = millis();          // Initialise timeout
    GapSum      = 0;
    HopStart = millis()+2;
}
/*********************************************************************************************************************************/

void SetThePipe(uint64_t WhichPipe)
{
    Radio1.openWritingPipe(WhichPipe);
    Radio1.stopListening();
    delay (4);  // alllow things to happen
 }

/*********************************************************************************************************************************/

void DoScanInit()
{
    Radio1.setDataRate(RF24_1MBPS); // Scan only works at this default rate
    CurrentMode = SCANWAVEBAND;     // Fhss == No transmitting please, we are scanning.
    BoundFlag = false;
    SendCommand(NoSleeping);
    for (i = 0; i < 125; i++) {
        NoCarrier[i]   = 0;
        AllChannels[i] = 0;
    }
}

/*********************************************************************************************************************************/

void DoScanEnd()
{
    SendCommand(NextionSleepTime);
    Radio1.setDataRate(RF24_250KBPS);
    Radio1.openWritingPipe(DefaultPipe);
    CurrentMode = NORMAL;
}
/*********************************************************************************************************************************/
