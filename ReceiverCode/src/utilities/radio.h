/** @file ReceiverCode/src/utilities/radio.h */
// Malcolm Messiter 2020 - 2025
#ifndef _SRC_UTILITIES_RADIO_H
#define _SRC_UTILITIES_RADIO_H

#include "utilities/1Definitions.h"

#ifdef USE_SBUS
/************************************************************************************************************/
// This function sends the SBUS data to the receiver.
// It is called every 10ms to keep the SBUS happy.
// The data might be new, or it might be the same as before. It doesn't matter much.
void SendSBUSData()
{

    static uint32_t LocalTime = millis();

    if (((millis() - LocalTime) < SBUSRATE) || (!BoundFlag) || (!CheckForCrazyValues()))
        return;                 // Don't send SBUS data except when due
    MySbus.write(SbusChannels); // Send SBUS data
    LocalTime = millis();       // reset the timer
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
#endif
//************************************************************************************************************/
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
    if ((Parameters.ID == 0) || (Parameters.ID > PARAMETERS_MAX_ID))
    {
        Look1("Invalid ID: ");
        Look(Parameters.ID);
        return; // not a valid ID
    }
    for (int i = 0; i < 12; ++i)
    {
        Parameters.word[i] = RawDataIn[i]; // 8 words - of 12 useful BITs each
    }
    ReadExtraParameters();
}
// ************************************************************************************************************/
inline uint8_t GetDecompressedSize(uint8_t DynamicPayloadSize)
{
    uint8_t Ds = (((DynamicPayloadSize - 2) * 4) / 3) / 2; // first 2 bytes are ChannelBitMask, the rest is the 3:4 compressed data (hence the "-2")
    while (Ds % 4)                                         // make sure Ds is a multiple of 4
        ++Ds;                                              // increment until it is a multiple of 4
    return Ds;
}

/************************************************************************************************************/
void UseReceivedData(uint8_t DynamicPayloadSize) // DynamicPayloadSize is total length of incoming data
{
    LastPacketArrivalTime = millis();                     // Note the arrival time
    uint8_t Ds = GetDecompressedSize(DynamicPayloadSize); // Get the decompressed size of the data
    if (Ds)                                               // not zero?
    {
        if (DataReceived.ChannelBitMask)                            // Any changed channels?
        {                                                           // yes
            Decompress(RawDataIn, DataReceived.CompressedData, Ds); // Decompress the most recent data (8?)
            RearrangeTheChannels();                                 // Rearrange the channels for actual control since only changed ones are sent
        }
        else
        {
            if (DynamicPayloadSize > 2)                                 // no changed channels, but params
            {                                                           // parameter packet
                Decompress(RawDataIn, DataReceived.CompressedData, Ds); // Decompress the parameters
                ReadMoreParameters();
            }
        }
#ifdef USE_SBUS
        MapToSBUS(); // Get SBUS data ready
#endif
    }
#ifdef USE_SBUS
    SendSBUSData(); // maybe send SBUS data if its time
#endif
    ++SuccessfulPackets;     // These packets did arrive, but our acknowledgement might yet fail...
    if (HopNow)              // time to hop?
    {                        // This flag gets set in LoadLongerAckPayload();
        HopToNextChannel();  // Ack payload instructed us to Hop at next opportunity. So hop now ...
        HopNow = false;      // ... and clear the flag,
        HopStart = millis(); // ... and start the timer.
    }
}
// ************************************************************************************************************/
void SendAckWithPayload() // This function loads the acknowledgement payload It also delays briefly before returning, and during the delay it might read the INA219 to get volts.
{
#define FULLDELAYNEEDED 615 // delay required between writing ack payload and reading data
#define READVOLTSTIME 481   // 481 is the time needed to read the voltage from the INA219
#define DELAYNEEDED (FULLDELAYNEEDED - READVOLTSTIME)
    LoadLongerAckPayload();                                        // Load the AckPayload with telemetry data
    CurrentRadio->writeAckPayload(1, &AckPayload, AckPayloadSize); // send Full PAYLOAD (6 bytes)
    delayMicroseconds(DELAYNEEDED);                                // delay DELAYNEEDED
    GetRXVolts();                                                  // Takes 481us
}
// ************************************************************************************************************/
uint8_t TimeThePackets()
{
    constexpr uint32_t PERIOD = 500;     // Count for 1/2 second
    static uint32_t lastTime = millis(); // remember last time
    static uint32_t counter = 0;         // counter for packets
    uint32_t now = millis();             // carpe diem
    if (now - lastTime < PERIOD)         // Time up yet ?
    {
        ++counter;
        return 0; // not yet
    }
    if (!counter) // no packets?
        return 0;
    uint8_t Result = (uint8_t)(PERIOD / counter); // derive result
    // Look(counter);
    lastTime = now; // remember when we did
    counter = 0;    // Zero the counter
    return Result;
}
// ***************************************************************************************************************
inline void AdjustTimeout() // Adjust the receive timeout based on packet rate
{
    static uint8_t RT = 10; // default 10ms
    uint8_t t = TimeThePackets();
    if (t)
    {
        ReceiveTimeout = t + 4; // add four to handle auto retries etc.
        RT = ReceiveTimeout;
        // Look(RT);
    }
    else
    {
        ReceiveTimeout = RT;
    }
}
/************************************************************************************************************/
bool ReadData()
{
    Connected = false;

    if (CurrentRadio->available(&Pipnum))
    {

        AdjustTimeout();                                                    // adjust timeout for packet rate
        uint8_t DynamicPayloadSize = CurrentRadio->getDynamicPayloadSize(); // Get the size of the new data (14)
        if ((DynamicPayloadSize == 0) || (DynamicPayloadSize > 32))
            return false;
        SendAckWithPayload();                                  // ... and reads INA219 volts
        CurrentRadio->read(&DataReceived, DynamicPayloadSize); // Get received data from nRF24L01+
#ifdef USE_SBUS
        SendSBUSData(); // Maybe send SBUS data if its time
#endif
        Connected = true;                    // we are connected
        NewData = true;                      // we have new data
        UseReceivedData(DynamicPayloadSize); // use the received data
        CurrentRadio->flush_tx();            // This avoids a lockup that happens when the FIFO gets full
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
    roc = 0.8f * FilterRoc + 0.2f * roc;               //  light smoothing
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
        RXModelVolts = ina219.getBusVoltage_V(); //  Get RX LIPO volts if connected
    }
    else
    {
        delayMicroseconds(481); // because of the delay in the INA219 library
    }
}
// ******************************************************************************************************************************************************************

FASTRUN void ReceiveData()
{
    uint16_t ThisWait = millis() - LastPacketArrivalTime;

    if (!ReadData()) // Get new data if available
    {
        if (ThisWait < 1) //  if no data yet, allow almost the full 5ms to read these before next packet is due
        {
            ReadBMP280();
            ReadDPS310();
            ReadGPS();
        }
#ifdef USE_SBUS
        SendSBUSData(); // Send SBUS data if it's time to do so
#endif
        if ((!CurrentRadio->available(&Pipnum)) && (ThisWait >= ReceiveTimeout)) // 6ms at 200Hz, 3-4ms at 500Hz
            Reconnect();                                                         // Try to reconnect.
    }
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
/*
 * Print out some FHSS information about the channel hopping implementation
 */
#ifdef DB_FHSS
void ShowHopDurationEtc()
{
    static uint32_t LastTimeCalled = 0;
    static uint16_t hps = 0;
    static uint16_t HopsPerSecond = 0;
    ++hps;
    if (millis() - LastTimeCalled >= 1000)
    {
        HopsPerSecond = hps;
        hps = 0;
        LastTimeCalled = millis();
    }
    float freq = 2.4 + (float)NextChannel / 1000;
    Serial.print("Frequency: ");
    Serial.print(freq, 3);
    Serial.print("  Hopping Hz: ");
    Serial.print(HopsPerSecond);
    Serial.println("");
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
    // CurrentRadio->setStatusFlags(0);         // disables all IRQs
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
    HopStart = millis();
}

/************************************ Try to connect  ... *********************************************/

void TryToConnectNow()
{
    uint32_t ATimer;
    CurrentRadio->startListening();
    ATimer = millis();
    while ((!CurrentRadio->available(&Pipnum)) && (millis() - ATimer) < LISTEN_PERIOD) // LISTEN_PERIOD = 14 ...  just long enough to test all three
    {
#ifdef USE_SBUS
        SendSBUSData();
#endif // USE_SBUS
        KickTheDog();
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

void SwapChipEnableLines()
{
    if (ThisRadio == 1)
    {
        digitalWrite(V_Pin_Ce2, CE_OFF);
        digitalWrite(V_Pin_Csn2, CSN_OFF);
        digitalWrite(V_Pin_Ce1, CE_ON);
        digitalWrite(V_Pin_Csn1, CSN_ON);
    }
    else
    {
        digitalWrite(V_Pin_Csn1, CSN_OFF);
        digitalWrite(V_Pin_Ce1, CE_OFF);
        digitalWrite(V_Pin_Csn2, CSN_ON);
        digitalWrite(V_Pin_Ce2, CE_ON);
    }
    delayMicroseconds(STOPLISTENINGDELAY);
}
// ************************************************************************************************************/

void PointToRadio1()
{
    if (Use_eleven_PWM_Outputs)
    {
        Servos_Used = 11;
        CurrentRadio = &Radio1;
    }
    else
    {
        CurrentRadio = &Radio1a; // old RX with 8 PWMs (and 9!)
        Servos_Used = 9;
    }
    ThisRadio = 1;
}
//************************************************************************************************************/
void PointToRadio2()
{
    CurrentRadio = &Radio2;
    ThisRadio = 2;
}

/************************************************************************************************************/

void TryTheOtherTransceiver(uint8_t Recon_Ch)
{
    CurrentRadio->stopListening();
    if (ThisRadio == 2)
        PointToRadio1();
    else
        PointToRadio2();
    SwapChipEnableLines();
    ProdRadio(Recon_Ch);
    DelayMillis(1);
}

//************************************************************************************************************/
void DisplayPipe()
{
    static uint32_t lastTime = 0;
    if (millis() - lastTime < 1000)
        return;
    lastTime = millis();

    Serial.print("Listening for pipe: ");
    for (uint8_t i = 0; i < 5; ++i)
    {
        Serial.print(CurrentPipe[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");
}
/************************************************************************************************************/
FASTRUN void Reconnect()
{
#define MAXTRIESPERTRANSCEIVER 3

    const uint32_t start = millis();
    uint8_t prevRadio = ThisRadio;
    uint8_t attempts = 0;

    uint32_t now = millis();
    if (ThisRadio == 1)
        RX1TotalTime += (now - ReconnectedMoment);
    else if (ThisRadio == 2)
        RX2TotalTime += (now - ReconnectedMoment);
    ReconnectIndex = 2; // *** The best number for starting te search !!!!
    while (!Connected)
    {
        if (Blinking)
            BlinkLed();
        KickTheDog();
#ifdef USE_SBUS
        SendSBUSData();
#endif
        // Flush and prepare
        CurrentRadio->stopListening();
        delayMicroseconds(STOPLISTENINGDELAY);
        CurrentRadio->flush_tx();
        CurrentRadio->flush_rx();

        ReconnectChannel = FHSS_Recovery_Channels[ReconnectIndex];
        CurrentRadio->setChannel(ReconnectChannel);
        CurrentRadio->startListening();
        delayMicroseconds(STOPLISTENINGDELAY);
        ++attempts;
        TryToConnectNow();
        ReconnectIndex = (ReconnectIndex + 1) % 3;
        if (!Connected)
        {
            if (Use_Second_Transceiver)
            {
                if (attempts >= MAXTRIESPERTRANSCEIVER)
                {
                    TryTheOtherTransceiver(ReconnectChannel);
                    attempts = 0;
                }
            }
            else
            {
                if (attempts >= MAXTRIESPERTRANSCEIVER)
                {
                    ProdRadio(ReconnectChannel);
                    DelayMillis(1);
                    attempts = 0;
                }
            }
            if ((millis() - start) > FAILSAFE_TIMEOUT && !FailSafeSent)
            {
                FailSafe();
            }
        }
    }
    // Successful reconnection
    ReconnectedMoment = millis();
    FailSafeSent = false;

    if (prevRadio != ThisRadio)
        ++RadioSwaps;

    if (FailedSafe)
    {
        FailedSafe = false;
        uint32_t m = millis();
        NewConnectionMoment = m;
        ConnectMoment = m;
        SuccessfulPackets = 0;
    }
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

void teensyMAC(uint8_t *mac)
{ // GET UNIQUE TEENSY 4.0 ID
    for (uint8_t by = 0; by < 2; by++)
        mac[by] = (HW_OCOTP_MAC1 >> ((1 - by) * 8)) & 0xFF;
    for (uint8_t by = 0; by < 4; by++)
        mac[by + 2] = (HW_OCOTP_MAC0 >> ((3 - by) * 8)) & 0xFF;
}
// ***************************************************************************************************************************************************
void SetupRadios()
{
    teensyMAC(MacAddress);
    PipePointer = DefaultPipe;
    CopyToCurrentPipe(DefaultPipe, PIPENUMBER);
    PointToRadio1();
    if (Use_Second_Transceiver)
    {
        digitalWrite(V_Pin_Csn2, CSN_OFF);
        digitalWrite(V_Pin_Ce2, CE_OFF);
    }
    digitalWrite(V_Pin_Csn1, CSN_ON);
    digitalWrite(V_Pin_Ce1, CE_ON);
    delay(4);
    InitCurrentRadio();
    ThisRadio = 1;
    if (Use_Second_Transceiver)
    {
        PointToRadio2();
        digitalWrite(V_Pin_Csn1, CSN_OFF);
        digitalWrite(V_Pin_Ce1, CE_OFF);
        digitalWrite(V_Pin_Csn2, CSN_ON);
        digitalWrite(V_Pin_Ce2, CE_ON);
        InitCurrentRadio();
        ThisRadio = 2;
    }
    delay(4);
}

/************************************************************************************************************/
void LoadLongerAckPayload()
{
    const uint8_t MAX_TELEMETERY_ITEMS = 24; // Max number of telemetry items to send...
    if (MacAddressSentCounter < 20)
    {
        SendMacAddress();
        return;
    }
    AckPayload.Purpose &= 0x7F; // NOTE: The HIGH BIT of "purpose" bit is the HOPNOW flag. It gets set only when it's time to hop.
    ++AckPayload.Purpose;
    if (AckPayload.Purpose > MAX_TELEMETERY_ITEMS) // max number of telemetry items
        AckPayload.Purpose = 0;                    // wrap after max

    switch (AckPayload.Purpose)
    {
    case 0:
        SendVersionNumberToAckPayload();
        break;
    case 1:
        SendIntToAckPayload(SuccessfulPackets); // total successful packets we received here
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
        SendFloatToAckPayload(RXModelVolts);
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
    case 20:
        SendIntToAckPayload(RotorRPM); // RPM from Nexus
        break;
    case 21:
        SendFloatToAckPayload(Battery_Amps); // Amps from Nexus
        break;
    case 22:
        SendFloatToAckPayload(Battery_mAh); // mAh from Nexus
        break;
    case 23:
        SendIntToAckPayload(Receiver_Type); // Receiver Type
        break;
    case 24:
        SendFloatToAckPayload(escTempC); // ESC temperature in degrees C from Nexus
        // Look1("sent: ");
        // Look(escTempC);
        break;

    default:
        break;
    }
}

#endif // defined (_SRC_UTILITIES_RADIO_H)
