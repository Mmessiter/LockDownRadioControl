// ********************** Transceiver.h ***********************************************

#include <Arduino.h>
#include "Hardware/1Definitions.h"

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

// ***********************************************************************************************************
void ShowPacketData(uint32_t ThisPacketLength, uint8_t NumberOfChangedChannels)
{ // Just for debugging

    static uint32_t TotalPacketLength = 0;
    static uint32_t PacketsCount = 0;
    static uint32_t PacketsCount1 = 0;
    uint32_t AveragePacketLength = 0;

    TotalPacketLength += ThisPacketLength;
    ++PacketsCount;
    ++PacketsCount1;

    static uint32_t timer = 0;
    if ((millis() - timer) >= 250)
    {
        timer = millis();
        AveragePacketLength = TotalPacketLength / PacketsCount;
        Look1("Packet length (bytes): ");
        Look1(ThisPacketLength);
        Look1("\tNumber of changed channels: ");
        Look1(NumberOfChangedChannels);
        Look1("\tAverage packet length (bytes): ");
        Look1(AveragePacketLength);
        Look1("\tPackets count: ");
        Look1(PacketsCount);
        Look1("\tTotal data sent so far (k): ");
        Look1(TotalPacketLength / 1024);
        Look1("\tPackets per second: ");
        Look1(PacketsCount1);
        Look1("\tTotal lost packets: ");
        Look(TotalLostPackets);
        PacketsCount1 = 0;
    }
}
/************************************************************************************************************/
//                                       Most Radio Functions
/************************************************************************************************************/

/************************************************************************************************************/
/*
 * Decompresses uint16_t* buffer values (each with 12 bit resolution - the lower 12 bits).
 * @param uncompressed_buf[in]
 * @param compressed_buf[out] Must have allocated 3/4 the size of uncompressed_buf
 * @param uncompressed_size Size is in units of uint16_t (aka word or unsigned short)
 */
void Decompress(uint16_t *uncompressed_buf, uint16_t *compressed_buf, uint8_t uncompressed_size)
{
    uint8_t p = 0;
    for (uint8_t l = 0; l < (uncompressed_size * 3 / 4); l += 3)
    {
        uncompressed_buf[p] = compressed_buf[l] >> 4;
        ++p;
        uncompressed_buf[p] = (compressed_buf[l] & 0xf) << 8 | compressed_buf[l + 1] >> 8;
        ++p;
        uncompressed_buf[p] = (compressed_buf[l + 1] & 0xff) << 4 | compressed_buf[l + 2] >> 12;
        ++p;
        uncompressed_buf[p] = compressed_buf[l + 2] & 0xfff;
        ++p;
    }
}

/************************************************************************************************************/

// This functions performs lossless compression on the data sent to receiver, to keep it below 32 bytes
// and minimise latency. The receiver de-compresses of course.

/* Compresses uint16_t* buffer values (each with 12 bit resolution - the lower 12 bits).
 * @param compressed_buf[out] Must have allocated 3/4 the size of uncompressed_buf
 * @param uncompressed_buf[in]
 * @param uncompressed_size Size is in units of uint16_t (aka word or unsigned short). This *must* be divisible by 4.
 */
FASTRUN void Compress(uint16_t *compressed_buf, uint16_t *uncompressed_buf, uint8_t uncompressed_size)
{
    if (NewCompressNeeded)
    { // no need to recompress old data
        NewCompressNeeded = false;
        uint8_t p = 0;
        for (int l = 0; l < (uncompressed_size * 3 / 4); l += 3)
        {
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
// NB ONLY THE LOW 12 BITS ARE ACTUALLY SENT! (Because of the compression)

void LoadParameters()
{
    uint16_t Twobytes = 0;
    uint8_t FS_Byte1;
    uint8_t FS_Byte2;

    for (int i = 0; i < 12; ++i)
        Parameters.word[i] = 0; // clear the parameters

    switch (Parameters.ID)
    {                                             // ID is the parameter number. Each has 8 elements
    case 1:                                       // 1 = FailSafeChannels
        Twobytes = MakeTwobytes(FailSafeChannel); // 16 bool values compressed to 16 bits
        FS_Byte1 = uint8_t(Twobytes >> 8);        // Send as two bytes
        FS_Byte2 = uint8_t(Twobytes & 0x00FF);
        Parameters.word[1] = FS_Byte1; // These are failsafe flags
        Parameters.word[2] = FS_Byte2; // These are failsafe flags
        break;
    case 2: // 2 = QNH

        Parameters.word[1] = (uint16_t)Qnh;
        Parameters.word[2] = 1234;

        break;
    case 3: // 3 = GPSMarkHere
        if (GPSMarkHere)
        {
            Parameters.word[1] = 0;
            Parameters.word[2] = GPSMarkHere;
            GPSMarkHere = 0;
            GPS_RX_MaxDistance = 0;
        }
        break;
    case 4: // 4 = NOT USED YET
        break;
    case 5:                                         // 5 = SBUS/PPM
        Parameters.word[1] = PPMdata.UseSBUSFromRX; // 1 - 0
        Parameters.word[2] = PPMdata.PPMChannelCount;
        break;
    case 6: // 6 = Servo Frequencies
        for (int i = 0; i < 11; ++i)
            Parameters.word[i + 1] = ServoFrequency[i];
        break;
    case 7: // 7 = Servo Pulse Widths
        for (int i = 0; i < 11; ++i)
            Parameters.word[i + 1] = ServoCentrePulse[i];
        break;
    default:
        break;
    }
}

/************************************************************************************************************/
void RecordsPacketSuccess(uint8_t s)
{ // or failure according to s which is 1 or 0
    static uint16_t PacketsHistoryIndex = 0;
    ReconnectingNow = false;
    PacketsHistoryBuffer[PacketsHistoryIndex] = s;
    ++PacketsHistoryIndex;
    if (PacketsHistoryIndex >= PERFECTPACKETSPERSECOND)
        PacketsHistoryIndex = 0; // wrap around

    if (s)
        ++TotalGoodPackets;
}
/************************************************************************************************************/
void SendAllAgain()
{
    for (int i = 0; i < CHANNELSUSED; ++i)
    {
        PrePreviousBuffer[i] = 0;
        PreviousBuffer[i] = PrePreviousBuffer[i];
    }
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
void CheckDataRepeat()
{
    if (!AddExtraParameters)
    {
        for (int i = 0; i < CHANNELSUSED; ++i)
            PreviousBuffer[i] = PrePreviousBuffer[i]; // force last update repeat if not sending parameters
    }
}
/************************************************************************************************************/
void CheckLostContact()
{
    if (RecentPacketsLost >= LOSTCONTACTCUTOFF)
    { // If we have lost contact
        LostContactFlag = true;
        TryToReconnect();
    }
}
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
    CheckDataRepeat();
    CheckLostContact();
    CheckInactivityTimeout();
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
void TryToReconnect()
{
    // static uint32_t localtimer = 0;
    if (BuddyPupilOnPPM)
        return;
    if (!DontChangePipeAddress)
        TryOtherPipe();
    ++ReconnectionIndex;
    delayMicroseconds(42); // ???
    if (ReconnectionIndex >= 3)
    {

        ReconnectionIndex = 0;
        // Look(millis() - localtimer); // This rotates much faster that the RX so a Hit will happen fast,
        // localtimer = millis();
    }
    NextChannel = FHSS_data::Used_Recovery_Channels[ReconnectionIndex];
    HopToNextChannel();
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
    AddExtraParameters = false;
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
        ParseAckPayload(); // PayloadSize == 6 always now. Short Acks not used anymore.
    }
    if (BoundFlag && (!LedWasGreen || LedIsBlinking) && !UsingDefaultPipeAddress)
    {
        GreenLedOn();
        DontChangePipeAddress = true;
    }
    StartInactvityTimeout();
    LostContactFlag = false;
}
/************************************************************************************************************/

void LoadRawDataWithParameters()
{
    RawDataBuffer[0] = Parameters.ID; // copy current parameter values into the rawdatabuffer instead of the channels
    for (int i = 1; i < 12; ++i)
    {
        RawDataBuffer[i] = Parameters.word[i]; // copy current parameter values into the rawdatabuffer instead of the channels
    }
}

/************************************************************************************************************/

void DebugParamsOut()
{
    Look1("Parameter ID: ");
    Look1(Parameters.ID);
    Look1(" ");
    Look(ParaNames[Parameters.ID - 1]);
    for (int i = 1; i < 12; ++i)
    {
        Look1("  Word[");
        Look1(i);
        Look1("]: ");
        Look(Parameters.word[i]);
    }
}
/************************************************************************************************************/
int SendExtraParamemters() // parameters must be loaded before this function is called
{                          // only the ***low 12 bits*** of each parameter are actually sent because of compression
    if ((Parameters.ID == 0) || (Parameters.ID > MAXPARAMETERS))
    {
        Look1("Parameter error: ID is ");
        Look(Parameters.ID);
        return 8;
    }
    LoadParameters();
    LoadRawDataWithParameters();
    DataTosend.ChannelBitMask = 0; //  zero channels to send with this packet
                                   //  DebugParamsOut();
    return 11;                     //  was 8 is the number of parameters to send
}
/************************************************************************************************************/

// This function sends ONLY the changed channels, which it encodes into 'rawdatabuffer'.
// DataTosend.ChannelBitMask is a 16 bit word that indicates which channels have changed if any.
// It returns the number of channels that have changed since the last packet was sent.
// The rawdatabuffer is then compressed and sent to the receiver.
// The rawdatabuffer is a 16 bit word array, so each channel is 2 bytes before compression.
// The compression ratio is 1.5, so the compressed data buffer is 75% of the original size.

uint8_t EncodeTheChangedChannels()
{
#define MIN_CHANGE1 4                    // Very tiny changes in channel values are ignored. That's most likely only noise...
#define MAXCHANNELSATONCE 8              // Not more that 8 channels will be sent in one packet
    uint8_t NumberOfChangedChannels = 0; // Number of channels that have changed since last packet
    DataTosend.ChannelBitMask = 0;       // Clear the ChannelBitMask 16 BIT WORD (1 bit per channel)
    if (!AddExtraParameters)
    { // If sending parameters, don't send any channels ... yet.
        for (uint8_t i = 0; i < CHANNELSUSED; ++i)
        {                                                                                                                 // Check for changed channels and load them into the rawdatabuffer
            if ((abs(SendBuffer[i] - PreviousBuffer[i]) >= MIN_CHANGE1) && (NumberOfChangedChannels < MAXCHANNELSATONCE)) // MAXCHANNELSATONCE is the maximum number of channel changes that will be sent in one packet ...
            {                                                                                                             // ... any other changes will be sent in the next packet, only 5ms later.
                RawDataBuffer[NumberOfChangedChannels] = SendBuffer[i];                                                   // Load a changed channel into the rawdatabuffer.
                PrePreviousBuffer[i] = PreviousBuffer[i];                                                                 // Save previous buffer in case we need to repeat it.
                PreviousBuffer[i] = SendBuffer[i];                                                                        // Save it for next time in case it succeeds this time.
                DataTosend.ChannelBitMask |= (1 << i);                                                                    // Set the current bit in the ChannelBitMask word.
                ++NumberOfChangedChannels;                                                                                // Increment the number of channel changes (rawdatabuffer index pointer).
            }
        }
    }
    return NumberOfChangedChannels; // Return the number of channels that have changed
}
/************************************************************************************************************/
/********************************* Function to send data to receiver ****************************************/
/************************************************************************************************************/

FASTRUN void SendData()
{
    uint8_t NumberOfChangedChannels = 0;
    uint8_t ByteCountToTransmit;
    if (SendNoData)
        return;

    if ((millis() - LastPacketSentTime) >= FHSS_data::PaceMaker)
    {

        LastPacketSentTime = millis();
        if (BuddyPupilOnPPM)
        {
            SendViaPPM();
            return;
        } // If buddying (SLAVE) by wire, send SBUS data down wire only and transmit nothing.
        Connected = false; // Assume failure until an ACK is received.
        FlushFifos();      // This flush avoids a lockup that happens when the FIFO gets full.

        if (AddExtraParameters)
        {
            NumberOfChangedChannels = SendExtraParamemters();
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

        // ShowPacketData(ByteCountToTransmit, NumberOfChangedChannels);                                  // Just for debugging
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
FASTRUN void BufferTeensyMACAddPipe() // heeer
{
    for (int q = 1; q < 6; ++q)
    {
        SendBuffer[q] = MacAddress[q];
    }
}
/************************************************************************************************************/
void SendBindingPipe()
{

    if (BuddyPupilOnWireless)
        return;
    if (PPMdata.UseTXModule)
        return;
    if (!BoundFlag || !ModelMatched)
        BindingTimer = millis();
    if ((millis() - BindingTimer) < 1200)
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

FASTRUN float GetFromAckPayload()
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
    RXModelAltitudeBMP280 = GetFromAckPayload();                   // actual reading from BMP280
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
    RXTemperature = GetFromAckPayload();
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
/************************************************************************************************************/
FASTRUN void ParseAckPayload()
{
    if (BuddyPupilOnPPM)
        return; // buddy pupil need none of this

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
        RXModelVolts = GetFromAckPayload();
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
        // Look(ModelAltitude);
        break;
    case 7:
        GetTemperature();
        break;
    case 8:
        GPS_RX_Latitude = GetFromAckPayload();
        break;
    case 9:
        GPS_RX_Longitude = GetFromAckPayload();
        break;
    case 10:
        GPS_RX_ANGLE = GetFromAckPayload();
        break;
    case 11:
        GPS_RX_Speed = GetFromAckPayload();
        if (GPS_RX_MaxSpeed < GPS_RX_Speed)
            GPS_RX_MaxSpeed = GPS_RX_Speed;
        break;
    case 12:
        GPS_RX_FIX = GetFromAckPayload();
        break;
    case 13:
        GPS_RX_Altitude = GetFromAckPayload() - GPS_RX_GroundAltitude;
        if (GPS_RX_Altitude < 0)
            GPS_RX_Altitude = 0;
        if (GPS_RX_Maxaltitude < GPS_RX_Altitude)
            GPS_RX_Maxaltitude = GPS_RX_Altitude;
        // Look1("GPS_RX_Altitude: ");
        // Serial.println(GPS_RX_Altitude);
        break;
    case 14:
        GPS_RX_DistanceTo = GetFromAckPayload(); // now calculated locally
        if (GPS_RX_MaxDistance < GPS_RX_DistanceTo)
            GPS_RX_MaxDistance = GPS_RX_DistanceTo;
        break;
    case 15:
        GPS_RX_CourseTo = GetFromAckPayload();
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
            // Look("Synching RTC with GPS time");
            // Look(GPS_RX_SECS - Gsecond);
            SynchRTCwithGPSTime();
        }
        break;
    default:
        break;
    }
}
/************************************************************************************************************/
FASTRUN void CheckGapsLength()
{
    if (GapStart > 0)
    { // when reconnected, how long was connection lost?
        ++GapCount;
        ThisGap = (millis() - GapStart); // AND in fact RX sends no data for 20 ms after reconnection
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