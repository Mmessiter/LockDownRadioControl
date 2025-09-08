// ********************** Transceiver.h ***********************************************

#include <Arduino.h>
#include "1Definitions.h"

#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

/************************************************************************************************************/

FLASHMEM void ConfigureRadio()
{
    Radio1.setPALevel(RF24_PA_MAX, true);
    Radio1.setDataRate(DATARATE);
    Radio1.enableAckPayload();
    Radio1.openWritingPipe(DefaultPipe);
    Radio1.setRetries(FHSS_data::RetryCount, FHSS_data::RetryWait);
    Radio1.stopListening();
    delayMicroseconds(STOPLISTENINGDELAY);
    Radio1.enableDynamicPayloads();
    Radio1.setAddressWidth(5);
    Radio1.setCRCLength(RF24_CRC_16); //
    GapSum = 0;
}

/************************************************************************************************************/
FLASHMEM void InitRadio(uint64_t Pipe)
{
    Radio1.begin();
    ConfigureRadio();
}
/************************************************************************************************************/
#ifdef DB_PACKETDATA
// ***********************************************************************************************************
void ShowPacketData(uint32_t ThisPacketLength, uint8_t NumberOfChangedChannels)
{ // Just for debugging

    static uint32_t TotalPacketLength = 0;
    static uint32_t PacketsCount = 0;
    uint32_t AveragePacketLength = 0;

    TotalPacketLength += ThisPacketLength;
    ++PacketsCount;

    static uint32_t timer = 0;
    if ((millis() - timer) >= 100)
    {
        timer = millis();
        AveragePacketLength = TotalPacketLength / PacketsCount;
        Look1("Packet length (bytes): ");
        Look1(ThisPacketLength);
        Look1("\tNumber of sent channels: ");
        Look1(NumberOfChangedChannels);
        Look1("\tAverage packet length (bytes): ");
        Look1(AveragePacketLength);
        Look1("\tPackets count: ");
        Look1(PacketsCount);
        Look1("\tTotal data sent so far (k): ");
        Look1(TotalPacketLength / 1024);
        Look1("\tPackets per second: ");
        Look1(PacketsCount * 10);
        Look1("\tTotal lost packets: ");
        Look(TotalLostPackets);
        PacketsCount = 0;
    }
}
#endif
/************************************************************************************************************/
//                                       Most Radio Functions
/************************************************************************************************************/

void Decompress(uint16_t *uncompressed_buf, uint16_t *compressed_buf, uint8_t uncompressed_size)
{
    uint8_t p = 0;
    uint8_t compressed_size = (uncompressed_size * 3) / 4;

    for (uint8_t i = 0; i < compressed_size; i += 3)
    {
        uint16_t w0 = compressed_buf[i];
        uint16_t w1 = compressed_buf[i + 1];
        uint16_t w2 = compressed_buf[i + 2];

        uncompressed_buf[p++] = w0 >> 4;
        uncompressed_buf[p++] = ((w0 & 0x0F) << 8) | (w1 >> 8);
        uncompressed_buf[p++] = ((w1 & 0xFF) << 4) | (w2 >> 12);
        uncompressed_buf[p++] = w2 & 0x0FFF;
    }
}

FASTRUN void Compress(uint16_t *compressed_buf, uint16_t *uncompressed_buf, uint8_t uncompressed_size)
{
    if (!NewCompressNeeded)
        return;

    NewCompressNeeded = false;

    uint8_t p = 0;
    uint8_t compressed_size = (uncompressed_size * 3) / 4;

    for (uint8_t i = 0; i < compressed_size; i += 3)
    {
        uint16_t u0 = uncompressed_buf[p++];
        uint16_t u1 = uncompressed_buf[p++];
        uint16_t u2 = uncompressed_buf[p++];
        uint16_t u3 = uncompressed_buf[p++];

        compressed_buf[i] = (u0 << 4) | (u1 >> 8);
        compressed_buf[i + 1] = (u1 << 8) | (u2 >> 4);
        compressed_buf[i + 2] = (u2 << 12) | u3;
    }
}

/************************************************************************************************************/
void RecordsPacketSuccess(uint8_t s)
{ // or failure according to s which is 1 or 0
    static uint16_t PacketsHistoryIndex = 0;
    ReconnectingNow = false;
    PacketsHistoryBuffer[PacketsHistoryIndex] = s;
    ++PacketsHistoryIndex;
    if (PacketsHistoryIndex >= PACKET_HISTORY_WINDOW)
        PacketsHistoryIndex = 0; // wrap around

    if (s)
        ++TotalGoodPackets;
}

/************************************************************************************************************/
void CheckGap()
{
    if (!GapStart)
    {
        GapStart = millis(); // To keep track of this gap's length
    }
    else
    {
        if (((millis() - GapStart) > RED_LED_ON_TIME) && !LedWasRed)
            RedLedOn(); // Put on red led - receiver must be off
    }
}

/************************************************************************************************************/
// void CheckLostContact()
// {
//     if (RecentPacketsLost >= LOSTCONTACTCUTOFF)
//     { // If we have lost contact
//         LostContactFlag = true;
//         TryToReconnect();
//     }
// }
/************************************************************************************************************/
void CheckInactivityTimeout()
{
    int SecondsRemaining = (Inactivity_Timeout / 1000) - (millis() - Inactivity_Start) / 1000;
    if (SecondsRemaining <= 0)
        digitalWrite(POWER_OFF_PIN, HIGH); // INACTIVITY POWER OFF HERE!!
}
/************************************************************************************************************/
FASTRUN void FailedPacket()
{
    RecordsPacketSuccess(0); // Record a failure
    if (!ReconnectingNow)
        ++RecentPacketsLost; // this is to keep track of events when receiver is off
    Reconnected = false;
    ReconnectingNow = true;
    LastPacketSentTime = 0; // Force a new packet to be sent immediately
    CheckGap();
    TryToReconnect();
    CheckInactivityTimeout();
    PreviousPacketFailed = true;
}
/************************************************************************************************************/
FASTRUN void TryOtherPipe()
{
    if (BoundFlag == true)
    {
        BoundFlag = false;
        SetThePipe(DefaultPipe);
        UsingDefaultPipeAddress = true;
    }
    else
    {
        BoundFlag = true; //  ... but not modelmatched yet
        if (!BuddyPupilOnWireless)
        {
            SetThePipe(TeensyMACAddPipe);
            UsingDefaultPipeAddress = false;
        }
        else
        {
            SetThePipe(BuddyMACAddPipe);
        }
    }
}

/************************************************************************************************************/
void TryToReconnect() /* This function attempt to reconnect by very fast pings. And it has speeded up reconnection a lot.
                         It will attempt up to nine times and then revert to the main loop method in order that
                         other functions are also serviced. */
{
#ifdef DB_Reconnect
    uint32_t starttime = millis();
#endif
    uint8_t Iterations = 0;
    uint16_t Ping = 0;
    const uint8_t max_iterations = 9;
    static uint8_t ReconnectionIndex = 0;
    if (!DontChangePipeAddress)
        TryOtherPipe();
    while (Iterations <= max_iterations)
    {
        ++ReconnectionIndex;
        if (ReconnectionIndex >= 3)
        {
            ReconnectionIndex = 0;
            KickTheDog();
            delayMicroseconds(1500 + (rand() % 500) - 250); // ~1.5 ms base pause with ±250 us jitter
        }
        NextChannel = FHSS_data::Used_Recovery_Channels[ReconnectionIndex];
        HopToNextChannel();
        if (!LedWasGreen)
            return;
        ++Iterations;
        if (Radio1.write(&Ping, 2))
        {
            SuccessfulPacket();
#ifdef DB_Reconnect
            Look1("YES! Iterations: ");
            Look(Iterations);
            Look1("Time taken: ");
            Look(millis() - starttime);
#endif
            return;
        }
    }
#ifdef DB_Reconnect
    Look1("No! Iterations: ");
    Look(Iterations);
    Look1("Time taken: ");
    Look(millis() - starttime);
#endif
}
/************************************************************************************************************/
void FlushFifos()
{
    Radio1.flush_tx(); // This avoids a lockup that happens when the FIFO gets full.
    Radio1.flush_rx();
    delayMicroseconds(STOPLISTENINGDELAY);
}
/************************************************************************************************************/
void SuccessfulPacket()
{
    CheckGapsLength();
    RecordsPacketSuccess(1);
    ++RecentGoodPacketsCount;
    ++PacketNumber;
    if (RecentPacketsLost)
    {
        TotalLostPackets += RecentPacketsLost;
        RecentPacketsLost = 0;
        if (!TotalLostPackets)
            TotalLostPackets = 1; // RecentPacketsLost was only 1
    }
    Connected = true;
    if (Radio1.available())
    {
        uint8_t PayloadSize = Radio1.getDynamicPayloadSize();
        Radio1.read(&AckPayload, PayloadSize);
        ParseLongerAckPayload();
        // using a short payload added complexity without driving an advantage! So it is dropped.
    }
    if (BoundFlag && (!LedWasGreen || LedIsBlinking) && !UsingDefaultPipeAddress)
    {
        GreenLedOn();
        DontChangePipeAddress = true;
    }
    StartInactvityTimeout();
    LostContactFlag = false;
    PreviousPacketFailed = false; // Remember that the last packet was successful
}
// **********************************************************************************************************

// This function encodes the changed or timed-out channels. Only these are sent to receiver, compressed because the low 12 BITS only are needed.
// If the receiver doesn't get a particular channel, it just uses the last known value.
// This revised version has per-channel 'repeat-even-if-no-change' priorities (coz a changed packet might get lost ...)

FASTRUN uint8_t EncodeTheChangedChannels()
{
    const uint8_t Smallest_Change = 4;           // Very tiny changes in channel values are ignored. That's most likely only noise...
    const uint8_t MaximumChannelsPerPacket = 8;  // Not more that 8 channels will be sent in one packet
    uint8_t NumberOfChangedChannels = 0;         // Number of channels that have changed or timed out since last packet
    static uint32_t LastSendTime[CHANNELSUSED] = // Place to store the last moment when we sent each packet
        {
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0};
    const uint8_t Channel_Priority[CHANNELSUSED] = // At the current 200 Hz packet rate, the first 6 channels are repeated at >=20Hz even if unchanged.
        {                                          // first 6 channels (Ail/Ele/Col/Rud)+2 more, have the highest priority
         50, 50, 50, 50,
         50, 50, 150, 150,
         150, 150, 150, 150,
         150, 150, 150, 150};

    if (ParametersToBeSentPointer && !ParamPause) // If we are sending parameters, don't send any channels.
        return 0;

    uint32_t RightNow = millis();                           // Carpe diem
    DataTosend.ChannelBitMask ^= DataTosend.ChannelBitMask; // Clear the ChannelBitMask 16 BIT WORD (1 bit per channel)
    for (uint8_t i = 0; i < CHANNELSUSED; ++i)              // Check for changed channels and load them into the rawdatabuffer
    {
        if ((abs(SendBuffer[i] - PreviousBuffer[i]) >= Smallest_Change) || (LastSendTime[i] + Channel_Priority[i] < RightNow)) // Check if the channel has changed significantly
        {
            RawDataBuffer[NumberOfChangedChannels] = SendBuffer[i];  // Load a changed channel into the rawdatabuffer.
            PreviousBuffer[i] = SendBuffer[i];                       // Save the current value as the previous value so that we can detect changes.
            LastSendTime[i] = RightNow;                              // Save the time we sent this channel
            DataTosend.ChannelBitMask |= (1 << i);                   // Set the current bit in the ChannelBitMask word.
            ++NumberOfChangedChannels;                               // Increment the number of channel changes (rawdatabuffer index pointer).
            if (NumberOfChangedChannels >= MaximumChannelsPerPacket) // If we have reached the maximum number of channels to send
                break;                                               // Stop checking for more changes
        }
    }
    return NumberOfChangedChannels; // Return the number of channels that have changed or timed out
}
/************************************************************************************************************/
/********************************* Function to send data to receiver ****************************************/
/************************************************************************************************************/

FASTRUN void SendData()
{
    uint8_t NumberOfChangedChannels = 0;
    static uint8_t ByteCountToTransmit = 2;
    if (SendNoData)
        return;

    if (((millis() - LastPacketSentTime) >= FHSS_data::PaceMaker) || PreviousPacketFailed)
    {
        Connected = false; // Assume failure until an ACK is received.
        FlushFifos();      // This flush avoids a lockup that happens when the FIFO gets full.
        LastPacketSentTime = millis();
        if (ParametersToBeSentPointer && !ParamPause)
        {
            NumberOfChangedChannels = GetExtraParameters();
            --ParametersToBeSentPointer;
            if (!ParametersToBeSentPointer)
                ParamPause = true; // Pause sending parameters until next time
        }
        else
        {
            NumberOfChangedChannels = EncodeTheChangedChannels(); // Returns the number of channels that have changed, as well as loading the raw data buffer with the changed channels.
        }
        if (NumberOfChangedChannels)
        {                                                                      // Any channels changed? Or parameters to send?
            ByteCountToTransmit = ((float)NumberOfChangedChannels * 1.5f) + 4; // 1.5 is the compression ratio. 2 is the number of extra bytes for flags - plus 1 word because int rounds downwards.
            uint8_t SizeOfUnCompressedData = (ByteCountToTransmit / 1.5);
            Compress(DataTosend.CompressedData, RawDataBuffer, SizeOfUnCompressedData); // Compress the raw data buffer into the compressed data buffer (reduces it to 75% of original size)
        }
        else
        {
            ByteCountToTransmit = 2;
            NewCompressNeeded = false; // No channels changed nor any params to send, so just send the flag
        }

        if (BuddyMasterOnWireless)
            SendSpecialPacket(); // Talk to the buddy pupil if we are a master also 200 x per second
        ++TotalPacketsAttempted;
        if (Radio1.write(&DataTosend, ByteCountToTransmit))
        {
            SuccessfulPacket();
        }
        else
        {
            FailedPacket();
        } // Send the data packet complete with ChannelBitMask and compressed data
#ifdef DB_PACKETDATA
        ShowPacketData(ByteCountToTransmit, NumberOfChangedChannels); // Just for debugging
#endif
    }
}
/***********************************************************************************************************/
void DoScanEnd()
{
    ConfigureRadio();
    DontChangePipeAddress = false;
    CurrentMode = NORMAL;
}
/*********************************************************************************************************************************/

void DoScanInit()
{
    Radio1.setDataRate(RF24_1MBPS); // Scan only works at this default rate
    CurrentMode = SCANWAVEBAND;     // Fhss == No transmitting please, we are scanning.
    BoundFlag = false;
    ScanAllChannels(true);
}

/************************************************************************************************************/
// This function draws or re-draws and clears the box that display wave band scanning information
#define xx1 90      // Needed below... Edit xx1,yy1 to move box ....
#define yy1 70      // Needed below... Edit xx1,yy1 to move box ....
#define YY1EXTRA 15 // 15

void DrawFhssBox()
{
    int x1 = xx1;
    int y1 = yy1;
    int x2 = x1 + (128 * 5);
    int y2 = y1 + 255;
    int y3 = y2 + YY1EXTRA;
    int xd1 = 20;
    char STR125GHZ[] = "\"2.525\"";
    char STR96GHZ[] = "\"2.496\"";
    char STR64GHZ[] = "\"2.464\"";
    char STR32GHZ[] = "\"2.432\"";
    char STR1GHZ[] = "\"2.400\"";
    char GHZ[] = "\"GHz\"";
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
    int x1 = xx1;
    int y1 = yy1;
    int Sc;
    int x2, y2;
    int BlobHeight = 4; // Blobs are 4x4 pixels
    char NB[12];        // Number Buffer
    char NB1[12];
    char NB2[12];
    char NB3[12];
    char NB4[12];
    char CB[100]; // COMMAND BUFFER
    char fyll[] = "fill ";
    char NewYellow[15];
    char NA[1] = ""; // blank one
    static uint8_t AllChannels[127];
    static uint8_t NoCarrier[127];
    static uint32_t TotalHits[127];

    char Quietest[] = "Quietest";
    char Noisyest[] = "Noisyest";
    char Count[] = "Count";
    char Count1[] = "Count1";

    if (cls)
    {
        for (int i = 0; i < 125; i++)
        {
            NoCarrier[i] = 0;
            AllChannels[i] = 0;
            TotalHits[i] = 0;
        }
        return;
    }
    Str(NewYellow, HighlightColour, 0);
    for (Sc = 0; Sc <= 125; ++Sc)
    {
        if (NEXTION.available())
            return; // in case someone wants to stop!
        Radio1.setChannel(Sc);
        Radio1.startListening();
        x2 = x1 + (Sc * 5);
        y2 = y1 + 255;
        y2 = y2 - BlobHeight;
        y2 = y2 - AllChannels[Sc];
        delayMicroseconds(120); // Minimum!?
        Radio1.stopListening();
        if (Radio1.testCarrier())
        {
            if (AllChannels[Sc] < (250))
            {
                AllChannels[Sc] += BlobHeight;
                ++TotalHits[Sc];
                SendCharArray(CB, fyll, Str(NB, x2, 1), Str(NB1, (y2), 1), Str(NB2, 5, 1), Str(NB3, BlobHeight, 1), NewYellow, NA, NA, NA, NA, NA, NA);
            }
        }
        else
        {
            ++NoCarrier[Sc];
            if (NoCarrier[Sc] >= ScanSensitivity)
            { // must see no carrier >= ScanSensitivity times before reducing the trace
                if (AllChannels[Sc] >= (BlobHeight))
                {
                    SendCharArray(CB, fyll, Str(NB, x2, 1), Str(NB1, (y2 + BlobHeight), 1), Str(NB2, 5, 1), Str(NB3, BlobHeight, 1), Str(NB4, BackGroundColour, 0), NA, NA, NA, NA, NA, NA);
                    AllChannels[Sc] -= (BlobHeight);
                    NoCarrier[Sc] = 0;
                }
            }
        }
    }

    static uint16_t BestScore = 0;
    static uint16_t WorstScore = 0;

    for (Sc = 0; Sc < 83; ++Sc)
    {
        if (TotalHits[Sc] <= TotalHits[BestScore])
            BestScore = Sc;
        if (TotalHits[Sc] >= TotalHits[WorstScore])
            WorstScore = Sc;
    }

    SendValue(Quietest, BestScore);
    SendValue(Noisyest, WorstScore);
    SendValue(Count1, TotalHits[WorstScore]);
    SendValue(Count, TotalHits[BestScore]);
}

#ifdef DB_FHSS
float PStartTime = 0;
float PEndTime = 0;
float Pduration = 0;
float HopsPerSec = 0;
#endif
/************************************************************************************************************/
// This function hops to the next channel in the FFHS array

FASTRUN void HopToNextChannel()
{
    static uint16_t hopcount = 0;
    Radio1.stopListening();                // Transmit only
    delayMicroseconds(STOPLISTENINGDELAY); // very very short delay!
    Radio1.setChannel(NextChannel);        // Hop !
    delayMicroseconds(STOPLISTENINGDELAY); // very very short delay!
    CurrentChannel = NextChannel;          // save it for later
    ++hopcount;
    LastHopTime = millis();

#ifdef DB_FHSS
    if (BoundFlag && Connected && ModelMatched)
    {
        float Freq = 2.4;
        static uint32_t hoptime = 0;
        static uint16_t HopsPerSec = 0;
        Look1("  Hops per second: ");
        Serial.print(HopsPerSec);

        if ((millis() - hoptime) >= 1000)
        {
            hoptime = millis();
            HopsPerSec = hopcount;
            hopcount = 0;
        }
        Look1("  Next frequency: ");
        Freq += float(NextChannel) / 1000;
        Serial.print(Freq, 3);
        Look1(" Ghz");
        Look1("  RX transceiver number: ");
        Look1(ThisRadio);
        Look1(" Packets per hop: ");
        Look(PacketNumber);
        PStartTime = millis();
        PacketNumber = 0;
    }
#endif
}
/*********************************************************************************************************************************/

void DisplayPipe(uint64_t WhichPipe) // for debugging
{
    union
    {
        uint64_t i;
        uint8_t b[8];
    } Pipeu;
    Pipeu.i = WhichPipe;
    Look1("Pipe: ");
    Serial.print(Pipeu.b[0], HEX);
    Serial.print(" ");
    Serial.print(Pipeu.b[1], HEX);
    Serial.print(" ");
    Serial.print(Pipeu.b[2], HEX);
    Serial.print(" ");
    Serial.print(Pipeu.b[3], HEX);
    Serial.print(" ");
    Serial.print(Pipeu.b[4], HEX);
    Look("");
}
/*********************************************************************************************************************************/
void SetThePipe(uint64_t WhichPipe)
{
    Radio1.openWritingPipe(WhichPipe);
    delayMicroseconds(STOPLISTENINGDELAY);
    Radio1.stopListening();
    delayMicroseconds(STOPLISTENINGDELAY); // allow things to happen
                                           // DisplayPipe(WhichPipe);
}
/************************************************************************************************************/
#define BADNIBBLECOUNT 6

uint8_t CheckPipeNibbles(uint8_t b)
{
    uint8_t temp;
    uint8_t BadLowerNibble[BADNIBBLECOUNT] = {0x05, 0x0a, 0x02, 0x01, 0x00, 0x0f};
    uint8_t BadHigherNibble[BADNIBBLECOUNT] = {0x50, 0xa0, 0x20, 0x10, 0x00, 0xf0};
    uint8_t BetterLowerNibble[BADNIBBLECOUNT] = {0x03, 0x04, 0x06, 0x07, 0x08, 0x09};
    uint8_t BetterHigherNibble[BADNIBBLECOUNT] = {0x30, 0x40, 0x60, 0x70, 0x80, 0x90};
    if (!b)
        return 0x36; // return an acceptable byte for a zero

    for (int i = 0; i < BADNIBBLECOUNT; ++i)
    { // ********** check LOWER nibble **********
        if ((b & 0x0f) == BadLowerNibble[i])
        {
            temp = b & 0xf0;                 // save only the hi nibble in temp
            b = temp | BetterLowerNibble[i]; // put an acceptable nibble into lower nibble
        }
    }
    for (int i = 0; i < BADNIBBLECOUNT; ++i)
    { // ********** check HIGHER nibble **********
        if ((b & 0xf0) == BadHigherNibble[i])
        {
            temp = b & 0x0f;                  // save only the Low nibble in temp
            b = temp | BetterHigherNibble[i]; // put an acceptable nibble into Higher nibble
        }
    }
    return b;
}

/************************************************************************************************************/
void SendBindingPipe()
{
    if (BoundFlag || BuddyPupilOnWireless || !UsingDefaultPipeAddress)
        return; // No need to send the pipe if we are bound and model matched
    BufferTeensyMACAddPipe();
}
/*********************************************************************************************************************************/
void NormaliseTheRadio()
{
    SetThePipe(DefaultPipe);
    Radio1.setCRCLength(RF24_CRC_16);
    Radio1.setRetries(FHSS_data::RetryCount, FHSS_data::RetryWait);
}
/************************************************************************************************************/

FASTRUN float GetFloatFromAckPayload()
{
    union
    {
        float Val32;
        uint8_t Val8[4];
    } ThisUnion;
    ThisUnion.Val8[0] = AckPayload.Byte1;
    ThisUnion.Val8[1] = AckPayload.Byte2;
    ThisUnion.Val8[2] = AckPayload.Byte3;
    ThisUnion.Val8[3] = AckPayload.Byte4;
    return ThisUnion.Val32;
}
/************************************************************************************************************/
void GetTimeFromAckPayload()
{
    GPS_RX_SECS = AckPayload.Byte1;
    GPS_RX_Mins = AckPayload.Byte2;
    GPS_RX_Hours = AckPayload.Byte3;
}
/************************************************************************************************************/
void GetDateFromAckPayload()
{
    GPS_RX_DAY = AckPayload.Byte1;
    GPS_RX_MONTH = AckPayload.Byte2;
    GPS_RX_YEAR = AckPayload.Byte3;
}
// ************************************************************************************************************/
void FixInches(int *inches, int *feet)
{
    if (*inches >= 12)
    {
        *inches = 0;
        ++*feet;
    }
}
/************************************************************************************************************/
void GetAltitude()
{
    RXModelAltitudeBMP280 = GetFloatFromAckPayload();              // actual reading from BMP280
    RXModelAltitude = RXModelAltitudeBMP280 - GroundModelAltitude; // might be above ground only if GroundModelAltitude isnt zero
    if (RXMAXModelAltitude < RXModelAltitude)
        RXMAXModelAltitude = RXModelAltitude;
    int feet = (int)RXModelAltitude;
    int inches = (int)((RXModelAltitude - feet) * 12.0f + 0.5f); // round to nearest inch
    FixInches(&inches, &feet);
    snprintf(ModelAltitude, sizeof(ModelAltitude), "%d' %d''", feet, inches);
    feet = (int)RXMAXModelAltitude;
    inches = (int)((RXMAXModelAltitude - feet) * 12.0f + 0.5f); // round to nearest inch
    FixInches(&inches, &feet);
    snprintf(Maxaltitude, sizeof(ModelAltitude), "%d' %d''", feet, inches);
}
/************************************************************************************************************/
void GetTemperature()
{
    RXTemperature = GetFloatFromAckPayload();
    snprintf(ModelTempRX, 9, "%1.1f C.", RXTemperature);
}
/************************************************************************************************************/
FASTRUN uint32_t GetIntFromAckPayload() // This one uses a uint32_t int
{
    union
    {
        uint32_t Val32 = 0;
        uint8_t Val8[4];
    } ThisUnion;
    ThisUnion.Val8[0] = AckPayload.Byte1;
    ThisUnion.Val8[1] = AckPayload.Byte2;
    ThisUnion.Val8[2] = AckPayload.Byte3;
    ThisUnion.Val8[3] = AckPayload.Byte4;
    return ThisUnion.Val32;
}

/************************************************************************************************************/

void GetRXVersionNumber()
{
    char nbuf[5];
    RadioNumber = AckPayload.Byte1;
    Str(nbuf, RadioNumber, 0);
    strcpy(ThisRadio, nbuf);
    if (LastRadio != AckPayload.Byte1)
    {
        LastRadio = AckPayload.Byte1;
        if (LogRXSwaps && UseLog && LastRadio <= 2 && (LastRadio))
            LogThisRX();
    }
    Str(ReceiverVersionNumber, AckPayload.Byte2, 2);
    Str(nbuf, AckPayload.Byte3, 2);
    strcat(ReceiverVersionNumber, nbuf);
    Str(nbuf, AckPayload.Byte4, 0);
    strcat(ReceiverVersionNumber, nbuf);
    nbuf[0] = AckPayload.Byte5; // this appends the letter
    nbuf[1] = 0;
    strcat(ReceiverVersionNumber, nbuf);
    strcat(ReceiverVersionNumber, " (RX)");
    CompareVersionNumbers();
}
/************************************************************************************************************/
void GetModelsMacAddress()
{ // Gets a 64 bit value in two hunks of 32 bits

    if (BuddyPupilOnWireless)
        return; //  Don't do this if we are a pupil
    switch (AckPayload.Purpose)
    {
    case 0:
        ModelsMacUnion.Val32[0] = GetIntFromAckPayload();
        break;
    case 1:
        ModelsMacUnion.Val32[1] = GetIntFromAckPayload();
        break;
    default:
        break;
    }

    if (!ModelMatched)
    {
        if (ModelsMacUnion.Val32[0] && ModelsMacUnion.Val32[1])
        { // got both bits yet?
            ModelIdentified = true;
            CompareModelsIDs();
        }
    }
}
// ******************************************************************************************
// Returns true if we should refresh the display with `rpm_new`.
// - deadband: ignore ±2 RPM jitter (tweak to 3–5 if you like)
// - hysteresis: once we cross the band, we update and recenter
// - maxAgeMs: force an update occasionally even if inside the band
bool rpmShouldUpdate(uint16_t rpm_new)
{
    static uint16_t shown = 0;
    static uint32_t lastUpdate = 0;
    const uint16_t deadband = 2;    // try 2..5
    const uint32_t maxAgeMs = 1000; // force update at least 1 Hz

    uint32_t now = millis();
    int diff = int(rpm_new) - int(shown);

    if (abs(diff) > deadband || (now - lastUpdate) > maxAgeMs)
    {
        shown = rpm_new;
        lastUpdate = now;
        return true;
    }
    return false;
}
/************************************************************************************************************/
FASTRUN void ParseLongerAckPayload() // It's already pretty short!
{
    FHSS_data::NextChannelNumber = AckPayload.Byte5; // every packet tells of next hop destination
    if (AckPayload.Purpose & 0x80)
    {                                                                             // Hi bit is now the **HOP NOW!!** flag
        NextChannel = *(FHSS_data::FHSSChPointer + FHSS_data::NextChannelNumber); // The actual channel number pointed to.
        HopToNextChannel();
        AckPayload.Purpose &= 0x7f; // Clear the high BIT, use the remainder ...
    }

    if (!ModelMatched && !LedWasGreen)
    {
        GetModelsMacAddress();
        return;
    }

    switch (AckPayload.Purpose) // Only look at the low 7 BITS
    {
    case 0:
        if (millis() - LedGreenMoment < 5000)
        {
            GetRXVersionNumber();
        }
        else
        {
            RXSuccessfulPackets = GetIntFromAckPayload();
        }

        break;
    case 1:
        SbusRepeats = GetIntFromAckPayload();
        break;
    case 2:
        RadioSwaps = GetIntFromAckPayload();
        break;
    case 3:
        RX1TotalTime = GetIntFromAckPayload();
        break;
    case 4:
        RX2TotalTime = GetIntFromAckPayload();
        break;
    case 5:
        RXModelVolts = GetFloatFromAckPayload();
        RXVoltsDetected = false;
        if (RXModelVolts > 0)
        {
            RXVoltsDetected = true;
            if (RXCellCount == 12)
                RXModelVolts *= 2; // voltage divider needed !
            snprintf(ModelVolts, 5, "%1.2f", RXModelVolts);
        }
        break;
    case 6:
        GetAltitude();
        break;
    case 7:
        GetTemperature();
        break;
    case 8:
        GPS_RX_Latitude = GetFloatFromAckPayload();
        break;
    case 9:
        GPS_RX_Longitude = GetFloatFromAckPayload();
        break;
    case 10:
        GPS_RX_ANGLE = GetFloatFromAckPayload();
        break;
    case 11:
        GPS_RX_Speed = GetFloatFromAckPayload();
        if (GPS_RX_MaxSpeed < GPS_RX_Speed)
            GPS_RX_MaxSpeed = GPS_RX_Speed;
        break;
    case 12:
        GPS_RX_FIX = GetFloatFromAckPayload();
        break;
    case 13:
        GPS_RX_Altitude = GetFloatFromAckPayload() - GPS_RX_GroundAltitude;
        if (GPS_RX_Altitude < 0)
            GPS_RX_Altitude = 0;
        if (GPS_RX_Maxaltitude < GPS_RX_Altitude)
            GPS_RX_Maxaltitude = GPS_RX_Altitude;
        break;
    case 14:
        GPS_RX_DistanceTo = GetFloatFromAckPayload(); // now calculated locally
        if (GPS_RX_MaxDistance < GPS_RX_DistanceTo)
            GPS_RX_MaxDistance = GPS_RX_DistanceTo;
        break;
    case 15:
        GPS_RX_CourseTo = GetFloatFromAckPayload();
        break;
    case 16:
        GPS_RX_Satellites = (uint8_t)GetIntFromAckPayload();
        break;
    case 17:
        GetDateFromAckPayload();
        break;
    case 18:
        GetTimeFromAckPayload();
        ReadTheRTC();
        if (GPS_RX_DAY != GmonthDay)
            GPSTimeSynched = false;
        if (GPS_RX_MONTH != Gmonth)
            GPSTimeSynched = false;
        if (GPS_RX_Mins != Gminute)
            GPSTimeSynched = false;
        if (GPS_RX_Hours != Ghour)
            GPSTimeSynched = false;
        if (GPS_RX_YEAR != Gyear)
            GPSTimeSynched = false;
        if (abs(GPS_RX_SECS - Gsecond) > 5)
            GPSTimeSynched = false; // this is not very accurate because of latency
        if (GPS_RX_FIX && !GPSTimeSynched)
        {
            SynchRTCwithGPSTime();
        }
        break;
    case 19:
        RateOfClimb = GetFloatFromAckPayload();
        if (RateOfClimb > MaxRateOfClimb)
            MaxRateOfClimb = RateOfClimb;
        break;
    case 20:
        if (CurrentView != FRONTVIEW)
            break;
        RotorRPM = GetIntFromAckPayload(); // Get the current RPM value from the payload
        if (RotorRPM == 0xffff)            // this means no valid RPM data is available
            break;
        if (First_RPM_Data) // If this is the first time we get RPM data
        {
            First_RPM_Data = false;
            SendCommand((char *)"vis rpm,1");               // This will make the RPM display visible
            SendText((char *)"Owner", (char *)"Rotor RPM"); // Change the owner text so user knows it's RPM data
        }
        if (rpmShouldUpdate(RotorRPM))
            SendValue((char *)"rpm", RotorRPM); // Send the updated RPM value to Nextion Frontscreen only if it has changed sufficiently
        break;

    default:
        break;
    }
}
/************************************************************************************************************/
FASTRUN void CheckGapsLength()
{
    if (GapStart > 0)
    { // when reconnected, find out how long was connection lost
        ++GapCount;
        ThisGap = (millis() - GapStart);
        if (ThisGap >= MinimumGap && UseLog)
            LogThisGap();
        if (ThisGap > GapLongest)
            GapLongest = ThisGap;
        GapSum += ThisGap;
        GapStart = 0;
        GapAverage = GapSum / GapCount;
#ifdef DB_GAPS
        Serial.print("GapCount: ");
        Serial.println(GapCount);
        Serial.print("GapAverage: ");
        Serial.println(GapAverage);
        Serial.print("GapLongest: ");
        Serial.println(GapLongest);
        Serial.println(" ");
#endif
    }
}

#endif