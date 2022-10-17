
/************************************************************************************************************/
//                                      Radio Functions
/************************************************************************************************************/
// Malcolm Messiter 2022
#include "RadioFunctions.h"

/************************************************************************************************************/

/**
 * Compresses uint16_t* buffer values (each with 12 bit resolution - the lower 12 bits).
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

FASTRUN void TryOtherPipe()
{
    if (BoundFlag == true) {
        BoundFlag = false;
        SetThePipe(DefaultPipe);
    }
    else {
        BoundFlag = true;
        SetThePipe(NewPipe);
    }
}

/************************************************************************************************************/

FASTRUN void BufferNewPipe()
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

bool RecursedAlready = false;

/************************************************************************************************************/
// This function replaces delay() without freezing critical tasks
void Procrastinate(uint32_t HowLong)
{
    uint32_t ThisMoment = millis();

    if (RecursedAlready) 
    {
        while ((millis() - ThisMoment) < HowLong) {
            KickTheDog(); 
        }
        return;   // do not allow indirect recursion any further
    }
    
    RecursedAlready = true;
    while ((millis() - ThisMoment) < HowLong) {
        KickTheDog();                                                  // keep watchdog happy
        if (Connected && BoundFlag && ModelMatched) SendData();        // keep receiver happy too ... but guard against recursion 
    }
    RecursedAlready = false;
}

//***********************************************************************************************************
void Look(int p)
{
    Serial.println(p);
}

//***********************************************************************************************************
// *************************************** Functions to run macros  *****************************************
// **********************************************************************************************************
void StartMacro(uint8_t m)
{                                                                                     // Start a macro
    MacrosBuffer[m][MACRORUNNINGNOW] |= 1;                                            // LOW BIT = "running now" flag
    MacroStartTime[m] = millis() + ((MacrosBuffer[m][MACROSTARTTIME]) * 100);         // Note its Start moment
    MacroStopTime[m]  = MacroStartTime[m] + ((MacrosBuffer[m][MACRODURATION]) * 100); // Note its Stop moment
}
/************************************************************************************************************/
void RunMacro(uint8_t m)
{ // Move a servo to a place
    uint32_t RightNow = millis();
    if (RightNow >= MacroStartTime[m]) MacrosBuffer[m][MACRORUNNINGNOW] |= 2; // Set the ACTIVE Bit if started (BIT 1)
    if (RightNow >= MacroStopTime[m]) MacrosBuffer[m][MACRORUNNINGNOW] &= 1;  // Clear the ACTIVE Bit if expired (BIT 1)
    if (MacrosBuffer[m][MACRORUNNINGNOW] & 2) {
        SendBuffer[(MacrosBuffer[m][MACROMOVECHANNEL]) - 1] = map(MacrosBuffer[m][MACROMOVETOPOSITION], 0, 180, MINMICROS, MAXMICROS); // Do it if currently active!
    }
}
/************************************************************************************************************/
void StopMacro(uint8_t m)
{ // Stop a macro
    MacrosBuffer[m][MACRORUNNINGNOW] = 0;
}
/************************************************************************************************************/

void ExecuteMacro()
{ // Main entry point from SendData()  ... START/STOP/RUN
    uint8_t TriggerChannel = 0;
    for (u_int8_t i = 0; i < MAXMACROS; ++i) {
        // ***************************** START OR STOP ******************************
        TriggerChannel = (MacrosBuffer[i][MACROTRIGGERCHANNEL]) - 1;  // Down by one as channels are really 0 - 15
        if (TriggerChannel) {                                         // Is trigger channel non-zero?
            if (SendBuffer[TriggerChannel] >= MAXMICROS - 1) {        // Is the trigger point is close to its highest value?
                if (!MacrosBuffer[i][MACRORUNNINGNOW]) StartMacro(i); // Yes. Start if not already started
            }
            else {
                if (MacrosBuffer[i][MACRORUNNINGNOW]) StopMacro(i); // No. Stop it if it was running
            }
            // ****************************  RUN ****************************************
            if (MacrosBuffer[i][MACRORUNNINGNOW]) { // If running, move the servo ... if timer agrees.
                RunMacro(i);
            }
        }
    }
}

// *************** END OF MACROS ZONE ************************************************

/************************************************************************************************************/
void RecordsPacketSuccess(uint8_t s)
{ // or failure according to s
    PacketsHistoryBuffer[PacketsHistoryIndex] = s;
    ++PacketsHistoryIndex;
    if (PacketsHistoryIndex >= (PERFECTPACKETSPERSECOND * ConnectionAssessSeconds)) PacketsHistoryIndex = 0; //
}
/***************************************************************************************/

FASTRUN void FailedPacket()
{
    RecordsPacketSuccess(0);                      // Record a failure
    ++RecentPacketsLost;                          // this is to keep track of events when receiver is off
    ++TotalLostPackets;                           // This is total - never zeroed
    if (RecentPacketsLost >= LOSTCONTACTCUTOFF) { // Don't panic until at two packets are lost.
        if (!GapStart) GapStart = millis();       // To keep track of this gap's length
        LostContactFlag = true;
        Reconnected     = false;
        if ((millis() - GapStart) > RED_LED_ON_TIME) { // there's no need to blink red for every single lost packet. Only after 1/2 second of no connection.
            if (LedWasGreen && UseLog) {
                LogThisLongGap();
            }
            if (!LedWasRed) RedLedOn(); 
            ReEnableScanButton();
        }
    }
    int SecondsRemaining = (Inactivity_Timeout / 1000) - (millis() - Inactivity_Start) / 1000;
    if (SecondsRemaining <= 0) digitalWrite(POWER_OFF_PIN, HIGH); // INACTIVITY POWER OFF HERE!!
}

/************************************************************************************************************/

void TryToReconnect()
{

    if ((RecentPacketsLost > 200 || (!BoundFlag))) TryOtherPipe();                                // In case the receiver has re-booted
    NextChannel = *(FHSSChPointer + random(RECONNECT_CHANNELS_COUNT) + RECONNECT_CHANNELS_START); // random reconnect channel (selected from first three)
    HopToNextChannel();
}

/************************************************************************************************************/
void SuccessfulPacket()
{

    ++RangeTestGoodPackets;
    ++PacketNumber;
    RecordsPacketSuccess(1);
    LostContactFlag   = false;
    RecentPacketsLost = 0;
    Connected         = true;
    if (BoundFlag) GreenLedOn();
    CheckGapsLength();
    Radio1.read(&AckPayload, AckPayloadSize); //  "sizeof" doesn't work with externs,
    ParseAckPayload();
    StartInactvityTimeout();
}

/************************************************************************************************************/
void FlushFifos()
{
    Radio1.flush_rx(); // This avoids a lockup that happens when the FIFO gets full.
    Radio1.flush_tx();
}
/************************************************************************************************************/
//****************** Function to send pre-compressed data to receiver ***************************************
/************************************************************************************************************/

FASTRUN void SendData()
{
    if (((millis() - TxPace) >= PACEMAKER) || (LostContactFlag)) {
        TxPace = millis();
        if (DoSbusSendOnly) {
            MapToSBUS();
            return;
        } // If buddying (SLAVE) by wire, send SBUS data down wire only and transmit nothing.
        if (LostContactFlag) TryToReconnect();
        LoadPacketData();  // extra parameters appended to the data packet
        Connected = false; // Assume the worst until ACK is received.
        FlushFifos();
        if (Radio1.write(&CompressedData, SizeOfCompressedData)) { //  ************************** >>>>> SEND DATA TO RX <<<<< ***************************************
            SuccessfulPacket();
        }
        else {
            FailedPacket();
        }
    }
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
    char fyll[] = "fill ";
    char NewYellow[15];
    char NA[1] = ""; // blank one

    Str(NewYellow, HighlightColour, 0);
    for (Sc = ScanStart; Sc <= ScanEnd; ++Sc) {
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
        }
        else {
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
    Radio1.stopListening();         // Transmit only (no need for any extra delay() as this is here followed by several tasks)
   
#ifdef DB_FHSS 
    if (BoundFlag && Connected && ModelMatched){
        float ch   = *(FHSSChPointer + NextChannelNumber);
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

FLASHMEM void InitRadio(uint64_t Pipe)
{
    Radio1.begin(); 
    Radio1.setPALevel(RF24_PA_MAX, true);
    Radio1.setDataRate(RF24_250KBPS);
    Radio1.enableAckPayload();
    Radio1.openWritingPipe(Pipe);             // Current Pipe address used for Binding
    Radio1.setRetries(RETRYCOUNT, RETRYWAIT); // automatic retries and pauses *** WAS 15,15 *** !!
    Radio1.stopListening();
    Procrastinate(1);
    Radio1.enableDynamicPayloads();
    Radio1.setAddressWidth(5);       // was 4, is now 5
    Radio1.setCRCLength(RF24_CRC_8); // could be 16
    GapSum      = 0;
}
/*********************************************************************************************************************************/

void SetThePipe(uint64_t WhichPipe)
{
    Radio1.openWritingPipe(WhichPipe);
    Radio1.stopListening();
    Procrastinate(1); // alllow things to happen
}

/*********************************************************************************************************************************/

void DoScanInit()
{
    Radio1.setDataRate(RF24_1MBPS); // Scan only works at this default rate
    CurrentMode = SCANWAVEBAND;     // Fhss == No transmitting please, we are scanning.
    BoundFlag   = false;
    for (int i = 0; i < 125; i++) {
        NoCarrier[i]   = 0;
        AllChannels[i] = 0;
    }
}

/*********************************************************************************************************************************/

void DoScanEnd()
{
    Radio1.setDataRate(RF24_250KBPS);
    Radio1.openWritingPipe(DefaultPipe);
    CurrentMode = NORMAL;
}
/*********************************************************************************************************************************/
