/** @file ReceiverCode/src/utilities/radio.h */
// Malcolm Messiter 2020 - 2025
#ifndef _SRC_UTILITIES_RADIO_H
#define _SRC_UTILITIES_RADIO_H

#include "utilities/common.h"

/************************************************************************************************************/
// This function sends the SBUS data to the receiver.
// It is called every 10ms to keep the SBUS happy.
// The data might be new, or it might be the same as before. It doesn't matter much.
void SendSBUSData()
{
    static uint32_t LocalTime = millis();
    if (((millis() - LocalTime) < SBUSRATE) || (!BoundFlag) || (!CheckCrazyValues()))
        return;                 // Don't send SBUS data except when due
    MySbus.write(SbusChannels); // Send SBUS data
    LocalTime = millis();       // reset the timer
}
/************************************************************************************************************/
/** Read extra parameters from the transmitter.
 * extra parameters are sent using the last few words bytes in every data packet.
 * the parameter sent is defined by the packet number & the packet number defined the transmitter.
 */
void UseExtraParameters()
{
    uint16_t TwoBytes = 0;
    switch (Parameters.ID)
    {
    case 1:                            // working!
        FS_byte1 = Parameters.word[1]; // These 2 bytes are 16 failsafe flags
        FS_byte2 = Parameters.word[2]; // These 2 bytes are 16 failsafe flags
        TwoBytes = uint16_t(FS_byte2) + uint16_t(FS_byte1 << 8);
        RebuildFlags(FailSafeChannel, TwoBytes);
        SaveFailSafeData();
        break;
    case 2:
        Qnh = (float)Parameters.word[1];
        break;
    case 3:
        if (Parameters.word[2] == 255)
        { // Mark this location
            MarkHere();
            Parameters.word[2] = 0; // ... Once only
        }
        break;
    case 4:

        break;
    case 5:
        UseSBUS = (bool)Parameters.word[1]; // if false means PPM
        PPMChannelCount = Parameters.word[2];
        break;
    case 6:
        for (int i = 0; i < SERVOSUSED; ++i)
            ServoFrequency[i] = Parameters.word[i + 1];
        SetServoFrequency();
        break;
    case 7:
        for (int i = 0; i < SERVOSUSED; ++i)
            ServoCentrePulse[i] = Parameters.word[i + 1];
        SetServoFrequency();
        break;
    default:
        break;
    }
    return;
}

/************************************************************************************************************/

/** Map servo channels' data from ReceivedData buffer into SbusChannels buffer */
void MapToSBUS()
{
    if (Connected)
    {
        for (int j = 0; j < CHANNELSUSED; ++j)
        {
            SbusChannels[j] = static_cast<uint16_t>(map(ReceivedData[j], MINMICROS, MAXMICROS, RANGEMIN, RANGEMAX));
        }
    }
}
/************************************************************************************************************/
/*
 * Decompresses uint16_t* buffer values (each with 12 bit resolution - the lower 12 bits).
 * @param uncompressed_buf[in]
 * @param compressed_buf[out] Must have allocated 3/4 the size of uncompressed_buf
 * @param uncompressed_size Size is in units of uint16_t
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

void RearrangeTheChannels()
{
    //  This function looks at the 16 BITS of DataReceived.ChannelBitMask and rearranges the channels accordingly.
    uint8_t p = 0;
    for (int i = 0; i < CHANNELSUSED; ++i)
    {
        if (DataReceived.ChannelBitMask & (1 << i))
        {
            ReceivedData[i] = RawDataIn[p];
            ++p;

        } // if bit is set, set the channel otherwise leave it at the old value
    }
    return;
}
/************************************************************************************************************/

void DebugParameters()
{
    Look1("Parameters.ID:\t\t");
    Look1(Parameters.ID);
    Look1(" ");
    Look(ParaNames[Parameters.ID - 1]);
    for (int i = 1; i < 12; ++i)
    {
        Look1("Parameters.word[");
        Look1(i);
        Look1("]:\t");
        Look(Parameters.word[i]);
    }
}
/************************************************************************************************************/
void ReadMoreParameters()
{
    Parameters.ID = RawDataIn[0]; // NumberOfChangedChannels points past the end of the changed channels
    if ((Parameters.ID == 0) || (Parameters.ID > MAXPARAMETERS))
    {
        Look1("Invalid ID: ");
        Look(Parameters.ID);
        return; // not a valid ID
    }
    for (int i = 1; i < 12; ++i)
    {
        Parameters.word[i] = RawDataIn[i]; // 8 words - of 12 useful BITs each
    }
    UseExtraParameters();
}
/************************************************************************************************************/
void UseReceivedData(uint8_t DynamicPayloadSize) // DynamicPayloadSize is length of incomming data
{
    LastPacketArrivalTime = millis();                          // Note the arrival time
    if (DataReceived.ChannelBitMask)                           // Any changed channels?
    {                                                          // yes
        Decompress(RawDataIn, DataReceived.CompressedData, 8); // Decompress the most recent data 8 enough? Don't know yet how may channels will be sent
        RearrangeTheChannels();                                // Rearrange the channels for actual control since only changed ones are sent
    }
    else
    {
        if (DynamicPayloadSize > 2)                                 // no changed channels, but params
        {                                                           // parameter packet
            Decompress(RawDataIn, DataReceived.CompressedData, 10); // 10 allows 8 parameter elements per packet
            ReadMoreParameters();
        }
    }
    MapToSBUS();             // Get SBUS data ready
    SendSBUSData();          // maybe send SBUS data if its time
    ++SuccessfulPackets;     // These packets did arrive, but our acknowledgement might yet fail...
    if (HopNow)              // time to hop?
    {                        // This flag gets set in LoadLongerAckPayload();
        HopToNextChannel();  // Ack payload instructed us to Hop at next opportunity. So hop now ...
        HopNow = false;      // ... and clear the flag,
        HopStart = millis(); // ... and start the timer.
    }
}
// ************************************************************************************************************/
void LoadaPayload() // This function loads the acknowledgement payload It also delays briefly before returning, and during the delay it might read the INA219 to get volts.
{
#define FULLDELAYNEEDED 615 // delay required between writing ack payload and reading data
#define READVOLTSTIME 481   // 481 is the time needed to read the voltage from the INA219
#define DELAYNEEDED (FULLDELAYNEEDED - READVOLTSTIME)
    LoadLongerAckPayload();                                        // Load the AckPayload with telemetry data
    CurrentRadio->writeAckPayload(1, &AckPayload, AckPayloadSize); // send Full PAYLOAD (6 bytes)
    delayMicroseconds(DELAYNEEDED);                                // delay DELAYNEEDED
    GetRXVolts();                                                  // Takes 481us
}

/************************************************************************************************************/
bool ReadData()
{
    Connected = false;
    if (CurrentRadio->available(&Pipnum))
    {
        uint8_t DynamicPayloadSize = CurrentRadio->getDynamicPayloadSize(); // Get the size of the new data (14)
        if ((DynamicPayloadSize == 0) || (DynamicPayloadSize > 32))
            return false;
        LoadaPayload();                                        // ... and reads INA219 volts
        CurrentRadio->read(&DataReceived, DynamicPayloadSize); // Get received data from nRF24L01+
        SendSBUSData();                                        // Maybe send SBUS data if its time
        Connected = true;                                      // we are connected
        NewData = true;                                        // we have new data
        UseReceivedData(DynamicPayloadSize);                   // use the received data
        CurrentRadio->flush_tx();                              // This avoids a lockup that happens when the FIFO gets full
    }
    return Connected; // inform the caller of success or failure
}

/************************************************************************************************************/
float MetersToFeet(float Meters)
{
    return Meters * 3.28084;
}
/************************************************************************************************************/
void GetRateOfClimb()
{
    const uint32_t now = millis();
    static uint32_t lastTime = 0;
    static float lastAltitudeFt = 0.0f; // feet (or metres—keep units consistent)
    static float FilterRoc = 0;

    if (lastTime == 0)
    { // first call → just prime history
        lastTime = now;
        lastAltitudeFt = BaroAltitude;
        RateOfClimb = 0;
        return;
    }

    uint32_t dt_ms = now - lastTime;
    if (dt_ms < 5) // guard against very small Δt
        return;

    // feet per minute = Δalt (ft) / Δt (min)
    float dAlt = float(BaroAltitude) - lastAltitudeFt; // float early!
    float roc = (dAlt * 60000.0f) / float(dt_ms);      // 60000 ms/min
    roc = 0.8f * FilterRoc + 0.2f * roc; //  light smoothing
    FilterRoc = roc;
    RateOfClimb = static_cast<int32_t>(roc);
    lastAltitudeFt = float(BaroAltitude);
    lastTime = now;
}
// ************************************************************************************************************/
void ReadDPS310()
{
    if (!DPS310Connected || millis() < 10000)
        return;

    static uint32_t lastTime = 0;
    uint32_t now = millis();

    if (now - lastTime >= 125) // 8 Hz
    {
        lastTime = now;
        sensors_event_t temp_event, pressure_event;
        dps310.getEvents(&temp_event, &pressure_event);
        BaroTemperature = temp_event.temperature;
        float pressure_hPa = pressure_event.pressure;
        BaroAltitude = MetersToFeet(44330.0 * (1.0 - pow(pressure_hPa / Qnh, 0.1903)));
        GetRateOfClimb();
    }
}
// ******************************************************************************************************************************************************************
void ReadBMP280()
{
    if (!BMP280Connected || millis() < 10000)
        return; // warm-up guard

    static uint32_t lastTime = 0;
    uint32_t now = millis();

    if (now - lastTime >= 250) // 4 Hz
    { 
        lastTime = now;
        BaroTemperature = bmp.readTemperature();
        BaroAltitude = MetersToFeet(bmp.readAltitude(Qnh));
        GetRateOfClimb();
    }
}
// ******************************************************************************************************************************************************************
//  Get RX LIPO volts if connected
void GetRXVolts()
{
    static uint32_t LastTime = 0;
    if ((millis() - LastTime > 1007) && (INA219Connected))
    {
        LastTime = millis();
        INA219Volts = ina219.getBusVoltage_V(); //  Get RX LIPO volts if connected
    }
    else
    {
        delayMicroseconds(481); // because of the delay in the INA219 library
    }
}
// ******************************************************************************************************************************************************************

FASTRUN void ReceiveData()
{
    if (!ReadData()) // Get new data if available
    {
        if (millis() - LastPacketArrivalTime < 1) //  if no data yet, allow almost the full 5ms to read these before next packet is due
        {
            ReadBMP280();
            ReadDPS310();
            ReadGPS();
        }

#ifdef USE_STABILISATION
        if (MPU6050Connected) // no new packet yet, so look at the gyro and accelerometer
            DoStabilsation();
#endif
        SendSBUSData(); // Send SBUS data if it's time to do so
        if ((!CurrentRadio->available(&Pipnum)) && (millis() - LastPacketArrivalTime >= RECEIVE_TIMEOUT))
            Reconnect(); // Try to reconnect.
    }
}

/************************************************************************************************************/
void CopyCurrentPipe(uint8_t *p, uint8_t pn)
{
    for (int i = 0; i < 6; ++i)
        CurrentPipe[i] = p[i];
    PipePointer = p;
    Pipnum = pn;
}
//************************************************************************************************************/
void SetNewPipe() // new pipe from TX
{
    CurrentRadio->openReadingPipe(Pipnum, PipePointer); //  5 * byte array
#ifdef DB_BIND
    if (BoundFlag)
        Serial.println("BOUND TO TX'S PIPE");
#endif
    BoundFlag = true;
}

/************************************************************************************************************/

void SendVersionNumberToAckPayload() // AND which radio transceiver is currently in use
{
    AckPayload.Byte1 = ThisRadio;
    AckPayload.Byte2 = RXVERSION_MAJOR;
    AckPayload.Byte3 = RXVERSION_MINOR;
    AckPayload.Byte4 = RXVERSION_MINIMUS;
    AckPayload.Byte5 = toascii(RXVERSION_EXTRA);
}

/************************************************************************************************************/
// This function compares the just-received pipe with several of the previous ones
// if it matches most of them then its probably not corrupted.

bool ValidateNewPipe()
{

    uint8_t MatchedCounter = 0;

    if (pcount < 2)
        return false; // ignore first few

    PreviousNewPipes[PreviousNewPipesIndex] = NewPipeMaybe;
    PreviousNewPipesIndex++;
    if (PreviousNewPipesIndex > PIPES_TO_COMPARE)
        PreviousNewPipesIndex = 0;

    for (int i = 0; i < PIPES_TO_COMPARE; ++i)
    {
        if (NewPipeMaybe == PreviousNewPipes[i])
            ++MatchedCounter;
    }

    if (MatchedCounter >= 2)
        return true;
    return false;
}

/************************************************************************************************************/

void GetNewPipe() // from TX
{
    if (!NewData)
        return;
    NewData = false;
    if (PipeSeen)
        return;
    NewPipeMaybe = (uint64_t)ReceivedData[0] << 40;
    NewPipeMaybe += (uint64_t)ReceivedData[1] << 32;
    NewPipeMaybe += (uint64_t)ReceivedData[2] << 24;
    NewPipeMaybe += (uint64_t)ReceivedData[3] << 16;
    NewPipeMaybe += (uint64_t)ReceivedData[4] << 8;
    NewPipeMaybe += (uint64_t)ReceivedData[5];

    if (ValidateNewPipe()) // was this pipe corrupted?
    {
#ifdef DB_BIND
        Serial.println("Received TX ID!");
#endif
        for (int i = 0; i < 5; ++i)
        {
            TheReceivedPipe[4 - i] = ReceivedData[i + 1] & 0xff; // reversed byte array for our use
#ifdef DB_BIND
            Serial.print((uint8_t)ReceivedData[i + 1], HEX);
            Serial.print(" ");
#endif
        }
        TheReceivedPipe[5] = 0;
#ifdef DB_BIND
        Serial.println(" ");
#endif
        CopyCurrentPipe(TheReceivedPipe, BOUNDPIPENUMBER);
        BindModel();
        PipeSeen = true;
    }
#ifdef DB_BIND
    Look(pcount);
#endif
    ++pcount; // inc pipes received
}

/************************************************************************************************************/

/**
 * Get pipe address from EEPROM.
 * @note Address data in EEPORM is valid only after a previous power cycle observed
 * a completed binding process .
 */
FLASHMEM void GetOldPipe()
{
    ReadSavedPipe();
    CopyCurrentPipe(TheReceivedPipe, BOUNDPIPENUMBER);

#ifdef DB_BIND
    Serial.println("Loaded old PIPE:");
    for (int i = 0; i < 5; ++i)
    {
        Serial.print(TheReceivedPipe[i], HEX);
        Serial.print(" ");
    }
    Serial.println(" ");
#endif
}

/************************************************************************************************************/
/*
 * Print out some FHSS information about the channel hopping implementation
 */

#ifdef DB_FHSS
void ShowHopDurationEtc()
{
    static uint32_t LastTimeCalled = 0;
    static uint16_t hps = 0;
    static uint16_t HopsPerSecond = 0;
    static uint32_t PacketStartTime = 0;

    if (millis() - LastTimeCalled >= 1000)
    {
        HopsPerSecond = hps;
        hps = 0;
        LastTimeCalled = millis();
    }
    ++hps;

    float freq = 2.4 + (float)NextChannel / 1000;
    Serial.print("Hop duration: ");
    Serial.print(millis() - PacketStartTime);
    Serial.print("ms ");
    if ((millis() - PacketStartTime) < 100)
        Serial.print(" ");
    Serial.print(" Next frequency: ");
    Serial.print(freq, 3);
    Serial.print("  Hops per second: ");
    Serial.print(HopsPerSecond);
    Serial.println("");
    PacketStartTime = millis();
}
#endif

/************************************************************************************************************/

void HopToNextChannel()
{
    CurrentRadio->stopListening();
    delayMicroseconds(STOPLISTENINGDELAY);
    CurrentRadio->setChannel(NextChannel);
    CurrentRadio->startListening();
    delayMicroseconds(STOPLISTENINGDELAY);
    HopMoment = millis();
#ifdef DB_FHSS
    ShowHopDurationEtc();
#endif
}

/**************************************************************************************************************/

void ConfigureRadio()
{
    CurrentRadio->setPALevel(RF24_PA_MAX);
    CurrentRadio->setDataRate(DATARATE);
    CurrentRadio->enableAckPayload();        // needed
    CurrentRadio->setRetries(2, 2);          // automatic retries // was 2,0
    CurrentRadio->enableDynamicPayloads();   // needed
    CurrentRadio->setAddressWidth(5);        // use 5 bytes for addresses
    CurrentRadio->setCRCLength(RF24_CRC_16); // could be 8 or disabled
    CurrentRadio->setAutoAck(true);          // we want acks
    CurrentRadio->maskIRQ(1, 1, 1);          // no interrupts - seems NEEDED at the moment
    CurrentRadio->openReadingPipe(PIPENUMBER, PipePointer);
    CurrentRadio->startListening();
    delayMicroseconds(STOPLISTENINGDELAY);
}

/**************************************************************************************************************/

/** Initialize a radio transceiver. */
FLASHMEM void InitCurrentRadio()
{
    CurrentRadio->begin();
    ConfigureRadio();
    SaveNewBind = true;
    HopStart = millis();
}

/************************************ Try to connect  ... *********************************************/

void TryToConnectNow()
{
    uint32_t ATimer;
    CurrentRadio->startListening();
    ATimer = millis();
    while ((!CurrentRadio->available(&Pipnum)) && (millis() - ATimer) < LISTEN_PERIOD)
    {
        SendSBUSData();
        KickTheDog();
#ifdef USE_STABILISATION
        if (MPU6050Connected)
            DoStabilsation();
#endif
    }
    Connected = CurrentRadio->available(&Pipnum);
}

/************************************************************************************************************/

void ProdRadio(uint8_t Recon_Ch)
{ // After switching radios, this prod allows EITHER to connect. Don't know why - yet!
    ConfigureRadio();
    CurrentRadio->stopListening();
    delayMicroseconds(STOPLISTENINGDELAY);
    CurrentRadio->setChannel(Recon_Ch);
    CurrentRadio->startListening();
    delayMicroseconds(STOPLISTENINGDELAY);
    TryToConnectNow();
}

/************************************************************************************************************/

#ifdef SECOND_TRANSCEIVER
void SwapChipEnableLines()
{
    if (ThisRadio == 1)
    {
        digitalWrite(pinCE2, CE_OFF);
        digitalWrite(pinCSN2, CSN_OFF);
        digitalWrite(pinCE1, CE_ON);
        digitalWrite(pinCSN1, CSN_ON);
    }
    else
    {
        digitalWrite(pinCSN1, CSN_OFF);
        digitalWrite(pinCE1, CE_OFF);
        digitalWrite(pinCSN2, CSN_ON);
        digitalWrite(pinCE2, CE_ON);
    }
    delayMicroseconds(STOPLISTENINGDELAY);
}

/************************************************************************************************************/

void TryTheOtherTransceiver(uint8_t Recon_Ch)
{
    CurrentRadio->stopListening();
    if (ThisRadio == 2)
    {
        CurrentRadio = &Radio1;
        ThisRadio = 1;
    }
    else
    {
        CurrentRadio = &Radio2;
        ThisRadio = 2;
    }
    SwapChipEnableLines();
    ProdRadio(Recon_Ch);
    DelayMillis(1);
}
#endif // defined (SECOND_TRANSCEIVER)

/************************************************************************************************************/

FASTRUN void Reconnect()
{ // This is called when contact is lost, to reconnect ASAP
#define MAXTRIESPERTRANSCEIVER 3

    uint32_t SearchStartTime = millis();
    uint8_t PreviousRadio = ThisRadio;
    uint8_t Attempts = 0;

    if (ThisRadio == 1)
        RX1TotalTime += (millis() - ReconnectedMoment); // keep track of how long on each
    if (ThisRadio == 2)
        RX2TotalTime += (millis() - ReconnectedMoment);

    while (!Connected)
    {
        if (Blinking)
            BlinkLed();
        KickTheDog();
        SendSBUSData();
        CurrentRadio->stopListening();
        delayMicroseconds(STOPLISTENINGDELAY);
        CurrentRadio->flush_tx();
        CurrentRadio->flush_rx();
        ReconnectChannel = FHSS_Recovery_Channels[ReconnectIndex];
        ++ReconnectIndex;
        if (ReconnectIndex >= 3)
            ReconnectIndex = 0;
        CurrentRadio->stopListening();
        delayMicroseconds(STOPLISTENINGDELAY);
        CurrentRadio->setChannel(ReconnectChannel);
        CurrentRadio->startListening();
        delayMicroseconds(STOPLISTENINGDELAY);
        ++Attempts;
        TryToConnectNow();
        if (!Connected)
        {

#ifdef SECOND_TRANSCEIVER
            if (Attempts >= MAXTRIESPERTRANSCEIVER)
            {
                TryTheOtherTransceiver(ReconnectChannel);
                Attempts = 0;
            }
#else
            if (Attempts >= 3)
            {
                ProdRadio(ReconnectChannel); // This avoids a lockup of the nRF24L01+ !
                Attempts = 0;
            }
#endif
            if ((millis() - SearchStartTime) > FAILSAFE_TIMEOUT)
            {
                if (!FailSafeSent)
                    FailSafe();
            }
        }
    }

    // cannot pass here if not connected
    // must have connected by here
    // Look1 ("Reconnected on channel ");
    // Look  (ReconnectChannel);

    FailSafeSent = false;
    if (PreviousRadio != ThisRadio)
        ++RadioSwaps;             // Count the radio swaps
    ReconnectedMoment = millis(); // Save this moment

    if (ModelMatched)
    {
        Blinking = false;
    }
    if (FailedSafe)
    {
        FailedSafe = false;
        NewConnectionMoment = millis();
        ConnectMoment = millis();
        SuccessfulPackets = 0; // Reset the packet count
                               // BoundFlag = true;
    }

#ifdef DB_RXTIMERS
    Serial.print("Transceiver1 use so far: ");
    Serial.print(RX1TotalTime / 1000);
    Serial.println(" seconds");
    Serial.print("Transceiver2 use so far: ");
    Serial.print(RX2TotalTime / 1000);
    Serial.println(" seconds");
    Serial.print("Now connected on transceiver number: ");
    Serial.print(ThisRadio);
    Serial.println(" ...");
    Serial.println("");
#endif
}
/************************************************************************************************************/

void IncChannelNumber()
{
    ++NextChannelNumber; // Move up the channels' array
    if (NextChannelNumber >= FREQUENCYSCOUNT)
    {
        NextChannelNumber = 0;
    } // If needed, wrap the channels' array pointer
    AckPayload.Byte5 = NextChannelNumber;               // Tell the transmitter which element of the array to use next.
    NextChannel = *(FHSSChPointer + NextChannelNumber); // Get the actual channel number from the array.
}

/************************************************************************************************************/
// This function checks the time since last hop.
// If it's time to HOP, it sets the high bit in AckPayload.Purpose and both ends then HOP to new channel before next packet.
// The other 7 BITS of AckPayload.Purpose dictate the Payload's function (therefore 127 possibities.)
// This happens for *every* AckPayload, which return telemetry data as well as this hoptime information.
// Hence a single BIT directs the transmitter to hop.

void CheckWhetherItsTimeToHop()
{
    AckPayload.Purpose &= 0x7f; // Clear the HOP flag
    if ((millis() - HopStart) >= HOPTIME)
    {                               // Time to hop??
        AckPayload.Purpose |= 0x80; // Yes. So set the HOP flag leaving lower 7 bits unchanged
        IncChannelNumber();
        HopNow = true; // Set local flag and hop when ready BUT NOT BEFORE.
    }
}
/************************************************************************************************************/
void SendFloatToAckPayload(float U)
{ // This one function now works with most float parameters
    union
    {
        float Val32;
        uint8_t Val8[4];
    } ThisUnion;
    CheckWhetherItsTimeToHop();
    ThisUnion.Val32 = U;
    AckPayload.Byte1 = ThisUnion.Val8[0]; // These values are herewith delivered to Transmitter in Ack Payload
    AckPayload.Byte2 = ThisUnion.Val8[1];
    AckPayload.Byte3 = ThisUnion.Val8[2];
    AckPayload.Byte4 = ThisUnion.Val8[3];
}
/************************************************************************************************************/
void SendTimeToAckPayload()
{
    CheckWhetherItsTimeToHop();
    AckPayload.Byte1 = SecsGPS;
    AckPayload.Byte2 = MinsGPS;
    AckPayload.Byte3 = HoursGPS;
}
/************************************************************************************************************/
void SendDateToAckPayload()
{
    CheckWhetherItsTimeToHop();
    AckPayload.Byte1 = DayGPS;
    AckPayload.Byte2 = MonthGPS;
    AckPayload.Byte3 = YearGPS;
}

/************************************************************************************************************/
void SendIntToAckPayload(uint32_t U)
{ // This one function now works with most int parameters
    union
    {
        uint32_t Val32;
        uint8_t Val8[4];
    } ThisUnion;
    CheckWhetherItsTimeToHop();
    ThisUnion.Val32 = U;
    AckPayload.Byte1 = ThisUnion.Val8[0]; // These values are herewith delivered to Transmitter in Ack Payload
    AckPayload.Byte2 = ThisUnion.Val8[1];
    AckPayload.Byte3 = ThisUnion.Val8[2];
    AckPayload.Byte4 = ThisUnion.Val8[3];
}

/************************************************************************************************************/
// The unique Mac address of this Teensy 4.0 is sent to transmitter while binding to identify this model.
// This is to avoid the wrong model memory being used.

void SendMacAddress()
{
    union
    {
        uint32_t Val32[2];
        uint8_t Val8[8]; // the highest two bytes will always be zero. We didn't need all 8.
    } ThisUnion;
    uint8_t MaxAckP = 1; // only packets 0 and 1 are needed here.

    ++MacAddressSentCounter;

    AckPayload.Purpose &= 0x7F;
    ++AckPayload.Purpose;
    if (AckPayload.Purpose > MaxAckP)
        AckPayload.Purpose = 0; // wrap after max

    for (int i = 0; i < 8; ++i)
        ThisUnion.Val8[i] = MacAddress[i];

    switch (AckPayload.Purpose)
    {
    case 0:
        SendIntToAckPayload(ThisUnion.Val32[0]);
        break;
    case 1:
        SendIntToAckPayload(ThisUnion.Val32[1]);
        break;

    default:
        break;
    }
}
/************************************************************************************************************/
void LoadLongerAckPayload()
{
    if (MacAddressSentCounter < 20)
    {
        SendMacAddress();
        return;
    }
    AckPayload.Purpose &= 0x7F; // NOTE: The HIGH BIT of "purpose" bit is the HOPNOW flag. It gets set only when it's time to hop.
    ++AckPayload.Purpose;
    AckPayloadSize = 6;          // 6 bytes of telemetry data
    if (AckPayload.Purpose > 19) // number of telemetry items
        AckPayload.Purpose = 0;  // wrap after max
    switch (AckPayload.Purpose)
    {
    case 0:
        if (millis() - ConnectMoment < 11000) // first 11 seconds send only version number
        {
            SendVersionNumberToAckPayload();
        }
        else
        {
            SendIntToAckPayload(SuccessfulPackets); // then send the number of successful packets
        }
        break;
    case 1:
        SendIntToAckPayload(0);
        break;
    case 2:
        SendIntToAckPayload(RadioSwaps);
        break;
    case 3:
        if (ThisRadio == 1)
        {
            SendIntToAckPayload((RX1TotalTime + (millis() - ReconnectedMoment)) / 1000); // addon time since last reconnection
        }
        else
        {
            SendIntToAckPayload(RX1TotalTime / 1000);
        }
        break;
    case 4:
        if (ThisRadio == 2)
        {
            SendIntToAckPayload((RX2TotalTime + (millis() - ReconnectedMoment)) / 1000); // addon time since last reconnection
        }
        else
        {
            SendIntToAckPayload(RX2TotalTime / 1000);
        }
        break;
    case 5:
        SendFloatToAckPayload(INA219Volts);
        break;
    case 6:
        SendFloatToAckPayload(BaroAltitude);
        break;
    case 7:
        SendFloatToAckPayload(BaroTemperature);
        break;
    case 8:
        SendFloatToAckPayload(LatitudeGPS);
        break;
    case 9:
        SendFloatToAckPayload(LongitudeGPS);
        break;
    case 10:
        SendFloatToAckPayload(AngleGPS);
        break;
    case 11:
        SendFloatToAckPayload(SpeedGPS);
        break;
    case 12:
        SendFloatToAckPayload(GpsFix);
        break;
    case 13:
        SendFloatToAckPayload(AltitudeGPS);
        break;
    case 14:
        SendFloatToAckPayload(DistanceGPS);
        break;
    case 15:
        SendFloatToAckPayload(CourseToGPS);
        break;
    case 16:
        SendIntToAckPayload(SatellitesGPS);
        break;
    case 17:
        SendDateToAckPayload();
        break;
    case 18:
        SendTimeToAckPayload();
        break;
    case 19:
        SendFloatToAckPayload(RateOfClimb);
        break;
    default:
        break;
    }
}

#endif // defined (_SRC_UTILITIES_RADIO_H)
