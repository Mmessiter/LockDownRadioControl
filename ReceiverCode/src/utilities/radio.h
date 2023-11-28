/** @file ReceiverCode/src/utilities/radio.h */
// Malcolm Messiter 2020 - 2023
#ifndef _SRC_UTILITIES_RADIO_H
#define _SRC_UTILITIES_RADIO_H

#include "utilities/common.h"

/** AckPayload Stucture for data returned to transmitter. */
struct Payload
{
    /**
     * This first byte "Purpose" defines what all the other bytes mean, AND ...
     * the highest BIT of Purpose means ** HOP TO NEXT CHANNEL A.S.A.P. (IF ON) **
     * the lower 7 BITs then define the meaning of the remainder of the ackpayload bytes
     * If Purpose = 1 then ...
     *
     * AckPayload.Byte2           =  ThisRadio;            // Radio in current use  Byte1 and Byte2 are free
     * AckPayload.Byte3           =  RXVERSION_MAJOR;
     * AckPayload.Byte4           =  RXVERSION_MINOR;
     * AckPayload.Byte5           =  RXVERSION_MINIMUS;
     *
     *  If Purpose = 2 then ...
     **/

    uint8_t Purpose = 0; // 0  Purpose
    uint8_t Byte1   = 0; // 1
    uint8_t Byte2   = 0; // 2
    uint8_t Byte3   = 0; // 3
    uint8_t Byte4   = 0; // 4
    uint8_t Byte5   = 0; // 5
};
Payload AckPayload;
uint8_t AckPayloadSize = sizeof(AckPayload); // Size for later externs if needed etc. (=6)



// ******************************************************************************************************************************
void UseRandomizedRecoveryChannels(){

      FHSS_Recovery_Channels[0] =  Randomized_Recovery_Channels[0];
      FHSS_Recovery_Channels[1] =  Randomized_Recovery_Channels[1];
      FHSS_Recovery_Channels[2] =  Randomized_Recovery_Channels[2];
}
// ******************************************************************************************************************************
void UseDefaultRecoveryChannels(){
       FHSS_Recovery_Channels[0]        =  Default_Recovery_Channels[0];
       FHSS_Recovery_Channels[1]        =  Default_Recovery_Channels[1];
       FHSS_Recovery_Channels[2]        =  Default_Recovery_Channels[2];
       Randomized_Recovery_Channels[0]  =  Default_Recovery_Channels[0]; // reset Randomized_Recovery_Channels to default in case they are used too early
       Randomized_Recovery_Channels[1]  =  Default_Recovery_Channels[1];
       Randomized_Recovery_Channels[2]  =  Default_Recovery_Channels[2];

       Randomized_Recovery_Channels_Counter = 0;
}

/************************************************************************************************************/
/** Read extra parameters from the transmitter.
 * extra parameters are sent using the last few words bytes in every data packet.
 * the parameter sent is defined by the packet number & the packet number defined the transmitter.
 */
void ReadExtraParameters()
{
    uint16_t TwoBytes = 0;
    uint8_t  SwapWaveBand;
    PacketNumber = ReceivedData[CHANNELSUSED];
    
    // NB: ReceivedData[CHANNELSUSED + 3]; cannot be used
   

    switch (PacketNumber) {
        case 0:
            FailSafeSave = bool(ReceivedData[CHANNELSUSED + 1]);
   
            if (FailSafeSave) {
                TwoBytes = uint16_t(FS_byte2) + uint16_t(FS_byte1 << 8);
                RebuildFlags(FailSafeChannel, TwoBytes);
            }
            break;
        case 1:
            FS_byte1  = ReceivedData[CHANNELSUSED + 1]; // These 2 bytes are 16 failsafe flags
            FS_byte2  = ReceivedData[CHANNELSUSED + 2]; // These 2 bytes are 16 failsafe flags
            break;
        case 2:
            Qnh = (ReceivedData[CHANNELSUSED + 1]) << 8; // 16 bits sent as two bytes for pressure here at sea level
            Qnh += ReceivedData[CHANNELSUSED + 2];
            if (OldQnh != Qnh) SendQnhToSensorHub();
            OldQnh = Qnh; // Send new one once only
            break;
        case 3:
        //  GuessWhat = ReceivedData[CHANNELSUSED + 1]; // not used yet
            if ((ReceivedData[CHANNELSUSED + 2]) == 255) { // Mark this location
                MarkHere();
                ReceivedData[CHANNELSUSED + 2] = 0; // ... Once only
            }
            break;
        case 4:
            ModelMatched = ReceivedData[CHANNELSUSED + 1];
            SwapWaveBand = ReceivedData[CHANNELSUSED + 2];
            if (SwapWaveBand > 0) {
                if (SwapWaveBand == 1) SetUKFrequencies();
                if (SwapWaveBand == 2) SetTestFrequencies();
            }
            break;
        case 5:
            UseSBUS         = (bool)ReceivedData[CHANNELSUSED + 1]; // if false means PPM
            PPMChannelCount = ReceivedData[CHANNELSUSED + 2];
            break;

        case 6:
          ++ Randomized_Recovery_Channels_Counter;
            if (Randomized_Recovery_Channels_Counter < 20) { // not forever!
                Randomized_Recovery_Channels[0] = ReceivedData[CHANNELSUSED + 1];
                Randomized_Recovery_Channels[1] = ReceivedData[CHANNELSUSED + 2];
                UseRandomizedRecoveryChannels();                             // Use randomized reconnection channels so that won't be the same as other user 
                
            }
            case 7:
            if (Randomized_Recovery_Channels_Counter < 20) { // not forever!
                Randomized_Recovery_Channels[2] = ReceivedData[CHANNELSUSED + 1];
                UseRandomizedRecoveryChannels();                             // Use randomized reconnection channels so that won't be the same as other user 
            }



        default:
            break;
    }
    return;
}

/************************************************************************************************************/

/** Map servo channels' data from ReceivedData buffer into SbusChannels buffer */
void MapToSBUS()
{
    if (Connected) {
        for (int j = 0; j < CHANNELSUSED; ++j) {
            SbusChannels[j] = static_cast<uint16_t>(map(ReceivedData[j], MINMICROS, MAXMICROS, RANGEMIN, RANGEMAX));
        }
    }
}
/************************************************************************************************************/
void UseReceivedData()
{
    Decompress(ReceivedData, CompressedData, UNCOMPRESSEDWORDS); // Decompress only the most recent data
    ReadExtraParameters();                                       // Look at parameters sent with packet
    MapToSBUS();                                                 // Get SBUS data ready
    LastPacketArrivalTime = millis();                            // Note the arrival time
    if (HopNow) {                                                // This flag gets set in LoadAckPayload();
        HopToNextChannel();                                      // Ack payload instructed us to Hop at next opportunity. So hop now ...
        HopNow   = false;                                        // ... and clear the flag,
        HopStart = millis();                                     // ... and start the timer.
    }
}



/************************************************************************************************************/
bool ReadData()
{
    Connected = false;
    if (CurrentRadio->available(&Pipnum))
    { // This is the only call that actually reads the radio
        LoadAckPayload();
        CurrentRadio->flush_tx();                                      // This avoids a lockup that happens when the FIFO gets full
        CurrentRadio->writeAckPayload(1, &AckPayload, AckPayloadSize); // Send telemetry
        DelayMillis(5);                                                // must allow time for the ack payload to be sent
        CurrentRadio->read(&CompressedData, sizeof(CompressedData));   //  ** >> Read new data from master << **
        Connected = true;
        NewData   = true;
    }
    if (Connected) UseReceivedData();
    return Connected;
}


// ******************************************************************************************************************************************************************

FASTRUN void ReceiveData()
{
    uint32_t TimeTest;

    if (Connected) {

        if ((millis() - SensorHubAccessed) > 10) {                               //  Reading Sensor hub 100 x per second should be enough
            if (millis() - LastPacketArrivalTime < 1) {                          //  If, and only if, we have still absolutely loads of time, do stuff now while waiting ...
                SensorHubAccessed = millis();                                    //  Note the moment of last attempted read.
                if (!SensorHubDead) {                                            //  Better check it hasn't died.
                    TimeTest = millis();                                         //  Time the I2C calls. If too long, don't repeat it ... save the model.
                    if (SensorHubConnected) ReadTheSensorHub();                  //  Sensor now has its own MCU. Calls return in far less that 6 ms unless it lost I2C synch
                    if (INA219Connected) INA219Volts = ina219.getBusVoltage_V(); //  Get RX LIPO volts if connected separately (as will be needed on 'planes with no GPS fitted.)
                    if ((millis() - NewConnectionMoment) > 5000) {
                        if ((millis() - TimeTest) > 6) SensorHubHasFailed(); //  If sensor hub and/or INA219 fails, don't bother calling either again (It normally returns within 2 ms.
                    }
                }
            }
        }
    }
    if (millis() - LastPacketArrivalTime >= RECEIVE_TIMEOUT) {
        Reconnect(); // Try to reconnect.
    }

    if (!ReadData()) {
        if (millis() - SBUSTimer >= SBUSRATE) { // No new packet yet - but maybe it's time to dispatch the last?
            if (BoundFlag && (millis() > 10000)) {
                if (Connected) {
                    KeepSbusHappy(); // if it's time - send a SBUS packet. It might be new data.
                    --SbusRepeats;   // It's not really a "repeat".
                }
            }
        }
    }
}
/************************************************************************************************************/



/************************************************************************************************************/

// This allows a new array of pseudo-random channel numbers to be used.
// "FHSSChPointer" and "FrequencyCount" simply need to be set appropriately.

void SetTestFrequencies()
{
    FHSSRecoveryPointer = FHSS_Channels1;
    FHSSChPointer       = FHSS_Channels1;
    FrequencyCount      = FREQUENCYSCOUNT1;
}
/************************************************************************************************************/

// This allows a new array of pseudo-random channel numbers to be used.
// "FHSSChPointer" and "FrequencyCount" simply need to be set appropriately.

void SetUKFrequencies()
{
    FHSSRecoveryPointer = FHSS_Channels;
    FHSSChPointer       = FHSS_Channels;
    FrequencyCount      = FREQUENCYSCOUNT;
}

/************************************************************************************************************/
void CopyCurrentPipe(uint8_t* p, uint8_t pn)
{

    for (int i = 0; i < 6; ++i) {
        CurrentPipe[i] = p[i];
    }
    PipePointer = p;
    Pipnum      = pn;
    // Serial.print("Current pipenum: ");
    // Serial.println(Pipnum);
}
//************************************************************************************************************/
void SetNewPipe()
{
    CurrentRadio->openReadingPipe(Pipnum, PipePointer); //  5 * byte array
#ifdef DB_BIND
    if (BoundFlag) Serial.println("BOUND TO TX'S PIPE");
#endif
}

/************************************************************************************************************/

void SendVersionNumberToAckPayload() // AND which radio transceiver is currently in use
{
    AckPayload.Byte1 = ThisRadio;
    AckPayload.Byte2 = RXVERSION_MAJOR;
    AckPayload.Byte3 = RXVERSION_MINOR;
    AckPayload.Byte4 = RXVERSION_MINIMUS;
}

bool PipeSeen = false;

/************************************************************************************************************/
// This function compares the just-received pipe with several of the previous ones
// if it matches most of them then its probably not corrupted.

bool ValidateNewPipe()
{

    uint8_t MatchedCounter = 0;

    if (pcount < 2) return false; // ignore first few

    PreviousNewPipes[PreviousNewPipesIndex] = NewPipeMaybe;
    PreviousNewPipesIndex++;
    if (PreviousNewPipesIndex > PIPES_TO_COMPARE) PreviousNewPipesIndex = 0;

    for (int i = 0; i < PIPES_TO_COMPARE; ++i) {
        if (NewPipeMaybe == PreviousNewPipes[i]) ++MatchedCounter;
    }

    if (MatchedCounter >= PIPES_TO_COMPARE / 2) return true; // half or more is OK
    return false;
}

/************************************************************************************************************/

void GetNewPipe() // from TX
{
    if (!NewData) return;
    NewData = false;
    if (PipeSeen) return;
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
        for (int i = 0; i < 5; ++i) {                            // heeer
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
    for (int i = 0; i < 5; ++i) {
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
    
    float   freq  = 2.4 + (float)NextChannel / 1000;
    Serial.print("Hop duration: ");
    Serial.print(millis() - PacketStartTime);
    Serial.print("ms ");
    if ((millis() - PacketStartTime)<100) Serial.print(" ");
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
    delayMicroseconds(100);
    CurrentRadio->setChannel(NextChannel);
    delayMicroseconds(100);
    CurrentRadio->startListening();
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
}

/**************************************************************************************************************/

/** Initialize a radio transceiver. */
FLASHMEM void InitCurrentRadio()
{
    CurrentRadio->begin();
    ConfigureRadio();
    SaveNewBind = true;
    HopStart    = millis();
}


/************************************ Try to connect  ... *********************************************/

void TryToConnectNow()
{
    uint32_t ATimer;
    CurrentRadio->startListening();
    ATimer = millis();
    while ((!CurrentRadio->available(&Pipnum)) && (millis() - ATimer) < LISTEN_PERIOD) {DoStabilsation();}       // while connecting, do some other stuff (Stabilisation, GPS, etc)
    Connected = CurrentRadio->available(&Pipnum);
    
}

/************************************************************************************************************/

void ProdRadio(uint8_t Recon_Ch)
{ // After switching radios, this prod allows EITHER to connect. Don't know why - yet!
    ConfigureRadio();
    CurrentRadio->setChannel(Recon_Ch);
    delayMicroseconds(200); // NEEDED ???
    TryToConnectNow();
}

/************************************************************************************************************/

#ifdef SECOND_TRANSCEIVER
void SwapChipEnableLines()
{
    if (ThisRadio == 1) {
        digitalWrite(pinCE2, CE_OFF);
        digitalWrite(pinCSN2, CSN_OFF);
        digitalWrite(pinCE1, CE_ON);
        digitalWrite(pinCSN1, CSN_ON);
    }
    else {
        digitalWrite(pinCSN1, CSN_OFF);
        digitalWrite(pinCE1, CE_OFF);
        digitalWrite(pinCSN2, CSN_ON);
        digitalWrite(pinCE2, CE_ON);
    }
    delayMicroseconds(200); // Allow swap over a little time to be noticed ...
}

/************************************************************************************************************/

void TryTheOtherTransceiver(uint8_t Recon_Ch)
{
    CurrentRadio->stopListening();
    if (ThisRadio == 2) {
        CurrentRadio = &Radio1;
        ThisRadio    = 1;
    }
    else {
        CurrentRadio = &Radio2;
        ThisRadio    = 2;
    }
    SwapChipEnableLines();
    ProdRadio(Recon_Ch);
}
#endif // defined (SECOND_TRANSCEIVER)

/************************************************************************************************************/

// This function is called when the system is busy but not receiving - to prevent very short SBUS timeouts (eg DJI).

void KeepSbusHappy()
{
    if ((millis() - NewConnectionMoment) < 20000) return; // Let things settle down after connection for 20 seconds or so before using this
    if (millis() - SBUSTimer >= SBUSRATE) {               // Does SBUS expect a packet?
        SBUSTimer = millis();                             // Yes...
        if (!FailSafeSent)                                // But don't send after failsafe
        {
            ++SbusRepeats;     // Count these repeats out of pure curiosity
            Connected = true;  // To force re-sending this older data
            MoveServos();      // This call also sends an SBUS packet
            Connected = false; // Not in fact connnected of course
        }
    }
}

/************************************************************************************************************/

FASTRUN void Reconnect()
{ // This is called when contact is lost, to reconnect ASAP
    #define MAXTRIESPERTRANSCEIVER 6
   
    uint32_t SearchStartTime  = millis(); 
    uint8_t  PreviousRadio    = ThisRadio;
    uint8_t  Attempts         = 0;
    
    ReconnectChannel = *(FHSSRecoveryPointer + ReconnectIndex); // Get a reconnect channel     
    if (ThisRadio == 1) RX1TotalTime += (millis() - ReconnectedMoment); // keep track of how long on each
    if (ThisRadio == 2) RX2TotalTime += (millis() - ReconnectedMoment);

    while (!Connected) {
        if (Blinking) BlinkLed();
        KickTheDog();
        if (BoundFlag) KeepSbusHappy(); // Some SBUS systems timeout FAST, so resend old data to keep it happy
        CurrentRadio->stopListening();
        CurrentRadio->flush_tx();
        CurrentRadio->flush_rx();
        ReconnectChannel = FHSS_Recovery_Channels[ReconnectIndex];
        ++ReconnectIndex;           
        if (ReconnectIndex >= 3) ReconnectIndex = 0;
        CurrentRadio->setChannel(ReconnectChannel);
        delayMicroseconds(200);
        ++Attempts;
        if (Attempts < MAXTRIESPERTRANSCEIVER) {
            TryToConnectNow();
        }
        if (!Connected) {

#ifdef SECOND_TRANSCEIVER
            if (Attempts >= MAXTRIESPERTRANSCEIVER) {
                TryTheOtherTransceiver(ReconnectChannel);
                Attempts = 0;
            }
#else
            if (Attempts >= 3) {
                ProdRadio(ReconnectChannel); // This avoids a lockup of the nRF24L01+ !
                Attempts = 0;
            }
#endif
            if ((millis() - SearchStartTime) > FAILSAFE_TIMEOUT) {
                if (!FailSafeSent) FailSafe();
            }
        }
    }   //  cannot pass here if not connected
        //  must have connected by here
       
        //  Look1 ("Reconnected on channel ");
        //  Look  (ReconnectChannel);
       
  
    FailSafeSent = false;
    if (PreviousRadio != ThisRadio) ++RadioSwaps; // Count the radio swaps
    ReconnectedMoment = millis();                 // Save this moment
    if (ModelMatched) {
        Blinking = false;
    }
    if (FailedSafe) {
        FailedSafe          = false;
        NewConnectionMoment = millis();
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
    if (NextChannelNumber >= FrequencyCount) {
        NextChannelNumber = 0;
    }                                                        // If needed, wrap the channels' array pointer
    AckPayload.Byte5 = NextChannelNumber;                    // Tell the transmitter which element of the array to use next.
    NextChannel      = *(FHSSChPointer + NextChannelNumber); // Get the actual channel number from the array.
}

/************************************************************************************************************/
// This function checks the time since last hop.
// If it's time to HOP, it sets the high bit in AckPayload.Purpose and both ends then HOP to new channel before next packet.
// The other 7 BITS of AckPayload.Purpose dictate the Payload's function (therefore 127 possibities.)
// This happens for *every* AckPayload, which return telemetry data as well as this hoptime information.
// Hence a single BIT directs the transmitter to hop.

void CheckWhetherItsTimeToHop()
{
    AckPayload.Purpose &= 0x7f;             // Clear the HOP flag
    if ((millis() - HopStart) >= HOPTIME) { // Time to hop??
        AckPayload.Purpose |= 0x80;         // Yes. So set the HOP flag leaving lower 7 bits unchanged
        IncChannelNumber();
        HopNow = true; // Set local flag and hop when ready BUT NOT BEFORE.
    }
}
/************************************************************************************************************/
void SendToAckPayload(float U)
{ // This one function now works with most float parameters
    union
    {
        float   Val32;
        uint8_t Val8[4];
    } ThisUnion;
    CheckWhetherItsTimeToHop();
    ThisUnion.Val32  = U;
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
        uint8_t  Val8[4];
    } ThisUnion;
    CheckWhetherItsTimeToHop();
    ThisUnion.Val32  = U;
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
        uint8_t  Val8[8]; // the highest two bytes will always be zero. We didn't need all 8.
    } ThisUnion;
    uint8_t MaxAckP = 1; // only packets 0 and 1 are needed here.

    ++MacAddressSentCounter;

    AckPayload.Purpose &= 0x7F;
    ++AckPayload.Purpose;
    if (AckPayload.Purpose > MaxAckP) AckPayload.Purpose = 0; // wrap after max

    for (int i = 0; i < 8; ++i) ThisUnion.Val8[i] = MacAddress[i];

    switch (AckPayload.Purpose) {
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
void LoadAckPayload()
{

    if (MacAddressSentCounter < 16) {
        SendMacAddress();
        return;
    }

    uint8_t MaxAckP = 4;        // 4 if only RX
    AckPayload.Purpose &= 0x7F; // NOTE: The HIGH BIT of "purpose" bit is the HOPNOW flag. It gets set only when it's time to hop.
    ++AckPayload.Purpose;
    if (INA219Connected) MaxAckP = 5;
    if (SensorHubConnected) MaxAckP = 18;                     // its 14 + GPS
    if (AckPayload.Purpose > MaxAckP) AckPayload.Purpose = 0; // wrap after max
    switch (AckPayload.Purpose) {
        case 0:
            SendVersionNumberToAckPayload();
            break;
        case 1:
            SendToAckPayload(SbusRepeats);
            break;
        case 2:
            SendToAckPayload(RadioSwaps);
            break;
        case 3:
            SendToAckPayload(RX1TotalTime / 1000);
            break;
        case 4:
            SendToAckPayload(RX2TotalTime / 1000);
            break;
        case 5:
            SendToAckPayload(INA219Volts);
            break;
        case 6:
            SendToAckPayload(BaroAltitude);
            break;
        case 7:
            SendToAckPayload(BaroTemperature);
            break;
        case 8:
            SendToAckPayload(LatitudeGPS);
            break;
        case 9:
            SendToAckPayload(LongitudeGPS);
            break;
        case 10:
            SendToAckPayload(AngleGPS);
            break;
        case 11:
            SendToAckPayload(SpeedGPS);
            break;
        case 12:
            SendToAckPayload(GpsFix);
            break;
        case 13:
            SendToAckPayload(AltitudeGPS);
            break;
        case 14:
            SendToAckPayload(DistanceGPS);
            break;
        case 15:
            SendToAckPayload(CourseToGPS);
            break;
        case 16:
            SendToAckPayload(SatellitesGPS);
            break;
        case 17:
            SendDateToAckPayload();
            break;
        case 18:
            SendTimeToAckPayload();
            break;
        default:
            break;
    }
}
/************************************************************************************************************/
/*
 * Decompresses uint16_t* buffer values (each with 12 bit resolution - the lower 12 bits).
 * @param uncompressed_buf[in]
 * @param compressed_buf[out] Must have allocated 3/4 the size of uncompressed_buf
 * @param uncompressed_size Size is in units of uint16_t (aka word or unsigned short)
 */
void Decompress(uint16_t* uncompressed_buf, uint16_t* compressed_buf, uint8_t uncompressed_size)
{
    uint8_t p = 0;
    for (uint8_t l = 0; l < (uncompressed_size * 3 / 4); l += 3) {
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
#endif // defined (_SRC_UTILITIES_RADIO_H)
