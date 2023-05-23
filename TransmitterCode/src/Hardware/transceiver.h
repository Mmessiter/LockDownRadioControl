// ********************** Transceiver.h ***********************************************

#include <Arduino.h>
/*********************************************************************************************************************************/

/************************************************************************************************************/
//                                 Most Radio Functions 
/************************************************************************************************************/

 /* Compresses uint16_t* buffer values (each with 12 bit resolution - the lower 12 bits).
 * @param compressed_buf[out] Must have allocated 3/4 the size of uncompressed_buf
 * @param uncompressed_buf[in]
 * @param uncompressed_size Size is in units of uint16_t (aka word or unsigned short). This *must* be divisible by 4.
 */
FASTRUN void Compress(uint16_t* compressed_buf, uint16_t* uncompressed_buf, uint8_t uncompressed_size)
{
    if (NewCompressNeeded){    // no need to recompress old data
        NewCompressNeeded = false;
        uint8_t p         = 0;
        for (int l = 0; l < (uncompressed_size * 3 / 4); l += 3) {
            compressed_buf[l] = uncompressed_buf[p] << 4 | uncompressed_buf[p + 1] >> 8;
            ++p;
            compressed_buf[l + 1] = uncompressed_buf[p] << 8 | uncompressed_buf[p + 1] >> 4;
            ++p;
            compressed_buf[l + 2] = uncompressed_buf[p] << 12 | uncompressed_buf[p + 1];
            ++p;
            ++p;
        }
    } 
}


/************************************************************************************************************/
void RecordsPacketSuccess(uint8_t s)
{ // or failure according to s
static uint16_t PacketsHistoryIndex    = 0;
    PacketsHistoryBuffer[PacketsHistoryIndex] = s;
    ++PacketsHistoryIndex;
    if (PacketsHistoryIndex >= (PERFECTPACKETSPERSECOND * ConnectionAssessSeconds)) PacketsHistoryIndex = 0; //
}


/************************************************************************************************************/
void ForceNextChannel(){
        NextChannel = *(FHSS_data::FHSSChPointer + FHSS_data::NextChannelNumber); // We already have index to next channel
        HopToNextChannel();
}

/************************************************************************************************************/

FASTRUN void FailedPacket()
{
    RecordsPacketSuccess(0);                      // Record a failure
    ++RecentPacketsLost;                          // this is to keep track of events when receiver is off
    ++TotalLostPackets;                           // This is total - never zeroed

    if (FirstPacketLost){
        ForceNextChannel();                      // If first lost, simply jump to next channel hoping receiver will too
        FirstPacketLost = false;
    }else{

    if (RecentPacketsLost >= LOSTCONTACTCUTOFF) { // Don't panic until at least LOSTCONTACTCUTOFF packets are lost.
        if (!GapStart) GapStart = millis();       // To keep track of this gap's length
        LostContactFlag = true;
        Reconnected     = false;
        if ((millis() - GapStart) > RED_LED_ON_TIME) { // there's no need to blink red for every single lost packet. Only after 1/2 second of no connection.
            if (LedWasGreen && UseLog) {
                LogThisLongGap();
            }
            if (!LedWasRed){
                RedLedOn(); 
                ReEnableScanButton();
            }
        }
    }
    if (LostContactFlag) TryToReconnect();
   
    int SecondsRemaining = (Inactivity_Timeout / 1000) - (millis() - Inactivity_Start) / 1000;
    if (SecondsRemaining <= 0) digitalWrite(POWER_OFF_PIN, HIGH); // INACTIVITY POWER OFF HERE!!
  }
}


/************************************************************************************************************/

FASTRUN void TryOtherPipe()
{
    if (BoundFlag == true) {
        BoundFlag = false;
        SetThePipe(DefaultPipe);
    }
    else 
    {
        BoundFlag = true;               //  ... but not modelmatched yet
        SetThePipe(TeensyMACAddPipe);   // 
    }
}
/************************************************************************************************************/

void TryToReconnect()
{
    if (BuddyPupilOnPPM) return;
    if (RecentPacketsLost > 25) { 
        TryOtherPipe();
        RecentPacketsLost = 0;
    }
    ++ReconnectionIndex;
    if (ReconnectionIndex >= RECONNECT_CHANNELS_COUNT) ReconnectionIndex = 0;
    NextChannel = *(FHSS_data::FHSSRecoveryPointer + RECONNECT_CHANNELS_START + ReconnectionIndex); //  reconnect channel (selected from three)
    HopToNextChannel();
}

/************************************************************************************************************/
void FlushFifos()
{
    Radio1.flush_tx(); // This avoids a lockup that happens when the FIFO gets full.
    delayMicroseconds(250);
    Radio1.flush_rx();
    delayMicroseconds(250);
}

/************************************************************************************************************/

void SuccessfulPacket()
{
    ++RangeTestGoodPackets;
    ++PacketNumber;
    FirstPacketLost = true;  // ie next lost will be 'first'
    RecordsPacketSuccess(1);   
    RecentPacketsLost = 0;
    Connected         = true;
    if (BoundFlag && !LedWasGreen) GreenLedOn();
    CheckGapsLength();
    Radio1.read(&AckPayload, AckPayloadSize); //  "sizeof" doesn't work with externs,
    ParseAckPayload();
    StartInactvityTimeout();
    LostContactFlag   = false;
}

/************************************************************************************************************/

FLASHMEM void InitRadio(uint64_t Pipe)
{
    Radio1.begin(); 
    Radio1.setPALevel(RF24_PA_MAX, true);
    Radio1.setDataRate(RF24_250KBPS);
    Radio1.enableAckPayload();
    Radio1.openWritingPipe(Pipe);               // Current Pipe address used for Binding
    Radio1.setRetries(RETRYCOUNT, RETRYWAIT);   // automatic retries and pauses
    Radio1.stopListening();
    delayMicroseconds(500);
    Radio1.enableDynamicPayloads();
    Radio1.setAddressWidth(5);          
    Radio1.setCRCLength(RF24_CRC_16);           // could be (RF24_CRC_8);
    GapSum  = 0;
}

/************************************************************************************************************/
//****************** Function to send data to receiver ***************************************
/************************************************************************************************************/

FASTRUN void SendData()
{
    if (SendNoData) return;
  
    uint16_t PacketGap = PACEMAKER;
    if (LostContactFlag && LedWasGreen && !BuddyPupilOnPPM) PacketGap = 0;
    if ((millis() - LastPacketSentTime) >= PacketGap) {
   
        LastPacketSentTime = millis();
        if (BuddyPupilOnPPM) {
            SendViaPPM();
            return;
        }                                                           // If buddying (SLAVE) by wire, send SBUS data down wire only and transmit nothing.
        Connected = false;                                          // Assume the worst until ACK is received.
        FlushFifos();
        LoadPacketData();                                           // extra parameters appended to the data packet   
        Compress(CompressedData, SendBuffer, UNCOMPRESSEDWORDS);    // Compress 32 bytes down to 24 (40 -> 30??)
        if (Radio1.write(&CompressedData, SizeOfCompressedData)) {  //  ************************** >>>>> SEND DATA (30 bytes) TO RX <<<<< ***************************************
            SuccessfulPacket(); 
            FlushFifos();
        } else {
            FlushFifos();
            FailedPacket();
        }
    }
}

/***********************************************************************************************************/

void DoScanEnd()
{
    Radio1.setDataRate(RF24_250KBPS);
    Radio1.openWritingPipe(DefaultPipe);
    CurrentMode = NORMAL;
}
/*********************************************************************************************************************************/

void DoScanInit()
{
    Radio1.setDataRate(RF24_1MBPS); // Scan only works at this default rate
    CurrentMode = SCANWAVEBAND;     // Fhss == No transmitting please, we are scanning.
    BoundFlag   = false;
    ScanAllChannels(true);
}

/************************************************************************************************************/
// This function draws or re-draws and clears the box that display wave band scanning information
#define xx1      90 // Needed below... Edit xx1,yy1 to move box ....
#define yy1      70 // Needed below... Edit xx1,yy1 to move box ....
#define YY1EXTRA 15

void DrawFhssBox()
{
    int  x1          = xx1;
    int  y1          = yy1;
    int  x2          = x1 + (128 * 5);
    int  y2          = y1 + 255;
    int  y3          = y2 + YY1EXTRA;
    int  xd1         = 20;
    char STR125GHZ[] = "\"2.525\"";
    char STR96GHZ[]  = "\"2.496\"";
    char STR64GHZ[]  = "\"2.464\"";
    char STR32GHZ[]  = "\"2.432\"";
    char STR1GHZ[]   = "\"2.400\"";
    char GHZ[]       = "\"GHz\"";
    char CB[150]; // COMMAND BUFFER
    char draw[] = "draw ";
    char xstr[] = "xstr ";
    char NB[9]; // Number Buffers...
    char NB1[9];
    char NB2[9];
    char NB3[9];
    char NB4[9];
    char NB5[9];
    char NB6[9];
    char NB7[9];
    char NB8[9];
    char NA[2] = ""; // blank one
    char NewWhite[15];
    char NewWhite1[15];
    char fyll[] = "fill ";
    Str(NewWhite, ForeGroundColour, 0);
    Str(NewWhite1, ForeGroundColour, 1);

    SendCharArray(CB, draw, Str(NB1, x1, 1), Str(NB2, y1, 1), Str(NB3, x2, 1), Str(NB4, y2, 1), NewWhite, NA, NA, NA, NA, NA, NA);
    SendCharArray(CB, xstr, Str(NB, 0, 1), Str(NB1, y3, 1), Str(NB2, 70, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), NewWhite1, Str(NB5, BackGroundColour, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), GHZ);
    SendCharArray(CB, xstr, Str(NB, x1 - xd1, 1), Str(NB1, y3, 1), Str(NB2, 80, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), NewWhite1, Str(NB5, BackGroundColour, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR1GHZ);
    SendCharArray(CB, xstr, Str(NB, (x1 + ((x2 - x1) / 4) - xd1), 1), Str(NB1, y3, 1), Str(NB2, 90, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), NewWhite1, Str(NB5, BackGroundColour, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR32GHZ);
    SendCharArray(CB, xstr, Str(NB, (x1 + ((x2 - x1) / 2) - xd1), 1), Str(NB1, y3, 1), Str(NB2, 90, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), NewWhite1, Str(NB5, BackGroundColour, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR64GHZ);
    SendCharArray(CB, xstr, Str(NB, (x1 + (((x2 - x1) / 4) * 3) - xd1), 1), Str(NB1, y3, 1), Str(NB2, 90, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), NewWhite1, Str(NB5, BackGroundColour, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR96GHZ);
    SendCharArray(CB, xstr, Str(NB, (x2 - xd1), 1), Str(NB1, y3, 1), Str(NB2, 80, 1), Str(NB3, 25, 1), Str(NB4, 0, 1), NewWhite1, Str(NB5, BackGroundColour, 1), Str(NB6, 1, 1), Str(NB7, 1, 1), Str(NB8, 1, 1), STR125GHZ);
    SendCharArray(CB, fyll, Str(NB, (x1 + 1), 1), Str(NB1, (y1 + 1), 1), Str(NB2, ((128 * 5) - 2), 1), Str(NB3, 254, 1), Str(NB4, BackGroundColour, 0), NA, NA, NA, NA, NA, NA);
}

/************************************************************************************************************/

// This function scans the waveband and displays result in the box that was drawn by the function above.

void ScanAllChannels(bool cls)
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
    char fyll[] = "fill ";
    char NewYellow[15];
    char NA[1] = ""; // blank one
    static uint8_t  AllChannels[127];
    static uint8_t  NoCarrier[127];

    if (cls){
        for (int i = 0; i < 125; i++) {
            NoCarrier[i]   = 0;
            AllChannels[i] = 0;
        }
        return;
    }
        Str(NewYellow, HighlightColour, 0);
    for (Sc = 1; Sc <= 125; ++Sc) { 
       if (NEXTION.available()) return; // in case someone wants to stop!
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
                SendCharArray(CB, fyll, Str(NB, x2, 1), Str(NB1, (y2), 1), Str(NB2, 5, 1), Str(NB3, BlobHeight, 1), NewYellow, NA, NA, NA, NA, NA, NA);
            }
        } else {
            ++NoCarrier[Sc];
            if (NoCarrier[Sc] > 15) { // must see no carrier >15 times before reducing the trace
                if (AllChannels[Sc] >= (BlobHeight)) {
                    SendCharArray(CB, fyll, Str(NB, x2, 1), Str(NB1, (y2 + BlobHeight), 1), Str(NB2, 5, 1), Str(NB3, BlobHeight, 1), Str(NB4, BackGroundColour, 0), NA, NA, NA, NA, NA, NA);
                    AllChannels[Sc] -= (BlobHeight);
                    NoCarrier[Sc] = 0;
                }
            }
        }
    }
}

#ifdef DB_FHSS
float PStartTime = 0;
float PEndTime   = 0;
float Pduration  = 0;
#endif



/************************************************************************************************************/

// This function hops to the next channel in the FFHS array (about 16 times a second)

FASTRUN void HopToNextChannel()
{
    Radio1.setChannel(NextChannel); // Hop !
    delayMicroseconds(500);
    Radio1.stopListening();         // Transmit only
    delayMicroseconds(500);

#ifdef DB_FHSS 
    if (BoundFlag && Connected && ModelMatched){
        float ch   = *(FHSS_data::FHSSChPointer +  FHSS_data::NextChannelNumber);
        float Freq = 2.4;
        PEndTime   = millis();
        Pduration  = (PEndTime - PStartTime) / 1000;
        Serial.print("Hop duration: ");
        Serial.print(Pduration);
        Serial.print(" seconds. Good packets per hop: ");
        Serial.print(PacketNumber);
        Serial.print(" Next frequency: ");
        Freq += ch / 1000;
        Serial.print(Freq, 3);
        Serial.print(" Ghz.");
        Serial.print(" RX transceiver number: ");
        Serial.println(ThisRadio);
        PStartTime = millis();
    }
#endif
    PacketNumber = 0;
}

/*********************************************************************************************************************************/

void SetThePipe(uint64_t WhichPipe)
{
    Radio1.openWritingPipe(WhichPipe);
    delayMicroseconds(500);
    Radio1.stopListening();
    delayMicroseconds(500); // allow things to happen
}
/************************************************************************************************************/
#define BADNIBBLECOUNT 6

uint8_t CheckPipeNibbles(uint8_t b){ 

    uint8_t      temp;
    uint8_t     BadLowerNibble[BADNIBBLECOUNT]   = {0x05,0x0a,0x02,0x01,0x00,0x0f};
    uint8_t    BadHigherNibble[BADNIBBLECOUNT]   = {0x50,0xa0,0x20,0x10,0x00,0xf0};
    uint8_t  BetterLowerNibble[BADNIBBLECOUNT]   = {0x03,0x04,0x06,0x07,0x08,0x09};
    uint8_t BetterHigherNibble[BADNIBBLECOUNT]   = {0x30,0x40,0x60,0x70,0x80,0x90};

    if (!b) return 0x36;                                     // return an acceptable byte for a zero
        
    for (int i = 0; i < BADNIBBLECOUNT; ++i) {               // ********** check LOWER nibble **********
        if ((b & 0x0f) == BadLowerNibble[i])
        {
            temp = b & 0xf0;                                 // save only the hi nibble in temp
            b    = temp | BetterLowerNibble[i];              // put an acceptable nibble into lower nibble
        }
    }
    for (int i = 0; i < BADNIBBLECOUNT;++i){                 // ********** check HIGHER nibble **********
        if ((b & 0xf0) == BadHigherNibble[i]) 
        {
            temp = b & 0x0f;                                  // save only the Low nibble in temp
            b = temp | BetterHigherNibble[i];                 // put an acceptable nibble into Higher nibble
        }
    }
    return b;
}

/************************************************************************************************************/

FASTRUN void BufferTeensyMACAddPipe() // heeer
{
    for (int q = 1; q < 6; ++q) {
        SendBuffer[q] = MacAddress[q];
    }
}
/************************************************************************************************************/
void SendBindingPipe()
{
    static uint32_t BindingTimer   = 0;
    if (PPMdata.UseTXModule) return;
    uint16_t BindPause = 1200; // failed at 200, OK just at 250. // 1200? heer                      
    if (!BoundFlag || !ModelMatched) BindingTimer = millis();
    if ((millis() - BindingTimer) < BindPause) BufferTeensyMACAddPipe(); 
}