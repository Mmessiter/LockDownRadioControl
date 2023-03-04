/** @file ReceiverCode/src/utilities/radio.h */
// Malcolm Messiter 2020 - 2023
#ifndef _SRC_UTILITIES_RADIO_H
#define _SRC_UTILITIES_RADIO_H
#include "common.h"

RF24     Radio1(pinCE1, pinCSN1);
RF24     Radio2(pinCE2, pinCSN2);
RF24*    CurrentRadio = &Radio1; 
bool     Connected    = false;
bool     SaveNewBind  = true;
bool     HopNow       = false;
uint8_t  ThisRadio    = 1;
uint8_t  SavedPipeAddress[8];
uint8_t  NextChannelNumber = 0;
uint8_t  NextChannel;
uint8_t  ReconnectIndex = RECONNECT_CHANNELS_START;
uint8_t  PacketNumber;
uint16_t ReceivedData[UNCOMPRESSEDWORDS]; //  20 x 16 BIT words
uint16_t PreviousData[UNCOMPRESSEDWORDS]; /** Previously received data (used for servos. Hence not sent if unchanged) */
uint16_t Interations = 0;
uint32_t HopStart;
uint64_t ThisPipe     = DEFAULTPIPE; // default startup
uint64_t NewPipe      = 0;
uint64_t NewPipeMaybe = 0;
uint64_t PreviousNewPipes[PIPES_TO_COMPARE];
uint8_t  PreviousNewPipesIndex = 0;
uint64_t OldPipe      = 0;
bool     FailSafeSent = true;
uint16_t SbusRepeats  = 0;
uint32_t RX1TotalTime = 0;
uint32_t RX2TotalTime = 0;
uint32_t RadioSwaps   = 0;

extern bool     BoundFlag;
extern bool     GpsFix;
extern bool     SensorHubConnected;
extern uint8_t  HoursGPS;
extern uint8_t  MinsGPS;
extern uint8_t  SecsGPS;
extern uint8_t  YearGPS;
extern uint8_t  MonthGPS;
extern uint8_t  DayGPS;
extern uint8_t  SatellitesGPS;
extern uint16_t BaroAltitude;
extern uint32_t ReconnectedMoment;
extern uint32_t SBUSTimer;
extern float    INA219Volts;
extern float    BaroTemperature;
extern float    LatitudeGPS;
extern float    LongitudeGPS;
extern float    SpeedGPS;
extern float    AngleGPS;
extern float    AltitudeGPS;
extern float    DistanceGPS;
extern float    CourseToGPS;
extern uint8_t  MacAddress[9];
extern uint8_t  TheReceivedPipe[9];
extern uint32_t NewConnectionMoment;
extern void BindModel();
extern void FailSafe(); // defined in main.cpp
extern void ClearAckPayload();
extern void ShowHopDurationEtc();
extern void ReadSensorHub();
extern void SetUKFrequencies();
extern void MoveServos();
extern FASTRUN void ReceiveData();
extern bool         FailedSafe;
extern bool         NewData;
extern uint16_t     pcount;
extern bool         ModelMatched;
extern bool         Blinking;
extern void         BlinkLed();
extern uint8_t MacAddressSentCounter;
extern void         ReadSavedPipe();
extern void         KickTheDog();

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


/************************************************************************************************************/

void HopNowAnyway(){ 
        ++NextChannelNumber;                                            // Move up the channels' array
        if (NextChannelNumber >= FrequencyCount) NextChannelNumber = 1; // If needed, wrap the channels' array pointer
        NextChannel           = *(FHSSChPointer + NextChannelNumber);
        HopToNextChannel();
}
/************************************************************************************************************/

void SetNewPipe()
{
    CurrentRadio->openReadingPipe(1, ThisPipe);
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

bool ValidateNewPipe(){ 
    
    uint8_t MatchedCounter = 0;
    
    if (pcount < 2) return false;  // ignore first few
    
    PreviousNewPipes[PreviousNewPipesIndex] = NewPipeMaybe;
    PreviousNewPipesIndex++;
    if (PreviousNewPipesIndex > PIPES_TO_COMPARE) PreviousNewPipesIndex = 0;
    
    for (int i = 0; i < PIPES_TO_COMPARE;++i){
        if (NewPipeMaybe == PreviousNewPipes[i]) ++ MatchedCounter;
    }
    
    if (MatchedCounter >= PIPES_TO_COMPARE / 2 ) return true; // half or more is OK
    return false;
}

/************************************************************************************************************/
#define BADNIBBLECOUNT 6
uint8_t CheckPipeNibbles(uint8_t b){ // heer

    uint8_t temp;
    uint8_t BadLowerNibble[BADNIBBLECOUNT]       = {0x05,0x0a,0x02,0x01,0x00,0x0f};
    uint8_t BadHigherNibble[BADNIBBLECOUNT]      = {0x50,0xa0,0x20,0x10,0x00,0xf0};
    uint8_t BetterLowerNibble[BADNIBBLECOUNT]    = {0x03,0x04,0x06,0x07,0x08,0x09};
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


void GetNewPipe()  // from TX
{
     if (!NewData) return;
     NewData = false;
   
    NewPipeMaybe = (uint64_t)ReceivedData[0]  << 40;
    NewPipeMaybe += (uint64_t)ReceivedData[1] << 32;
    NewPipeMaybe += (uint64_t)ReceivedData[2] << 24;
    NewPipeMaybe += (uint64_t)ReceivedData[3] << 16;
    NewPipeMaybe += (uint64_t)ReceivedData[4] << 8;
    NewPipeMaybe += (uint64_t)ReceivedData[5];

    if (ValidateNewPipe())  // was this pipe corrupted?
    {
        NewPipe      = NewPipeMaybe;
        for (int i = 0; i < 6; ++i) {
            TheReceivedPipe[i] = ReceivedData[i];          
        }
        BindModel();
        PipeSeen = true;
    }
    ++pcount; // inc pipes received
}

/************************************************************************************************************/

/**
 * Get pipe address from EEPROM.
 * @note Address data in EEPORM is valid only after a previous power cycle observed
 * a completed binding process (pairing was successful at least once during last flight's session).
 */
FLASHMEM void GetOldPipe()
{
    ReadSavedPipe();
    
    OldPipe  = (uint64_t)SavedPipeAddress[0] << 40;
    OldPipe += (uint64_t)SavedPipeAddress[1] << 32;
    OldPipe += (uint64_t)SavedPipeAddress[2] << 24;
    OldPipe += (uint64_t)SavedPipeAddress[3] << 16;
    OldPipe += (uint64_t)SavedPipeAddress[4] << 8;
    OldPipe += (uint64_t)SavedPipeAddress[5];
    
#ifdef DB_BIND
    Serial.println("Loading PIPE:");
    for (int i = 0; i < 6; ++i){
        Serial.print(SavedPipeAddress[i], HEX);
        Serial.print(" ");
    }
    Serial.println(" ");
#endif
    
}

/************************************************************************************************************/

void     HopToNextChannel()
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

/** Initialize a radio transceiver. */
FLASHMEM void InitCurrentRadio()
{
    CurrentRadio->begin();
    CurrentRadio->enableAckPayload();       // needed
    CurrentRadio->enableDynamicPayloads();  // needed
    CurrentRadio->maskIRQ(1, 1, 1);         // no interrupts - seems NEEDED at the moment - (line *IS* connected)
    CurrentRadio->setCRCLength(RF24_CRC_8); // (RF24_CRC_8); // could be 16 or disabled
    CurrentRadio->setPALevel(RF24_PA_MAX);
    CurrentRadio->setDataRate(RF24_250KBPS);
    CurrentRadio->openReadingPipe(1, ThisPipe);
    CurrentRadio->setRetries(0, 2);         // automatic retries
    CurrentRadio->setAutoAck(true);
    SaveNewBind = true;
    HopStart    = millis();
}

/************************************ Try to connect  ... *********************************************/

void TryToConnectNow()
{
    uint32_t ATimer;
  
    delayMicroseconds(250);
    CurrentRadio->startListening();
    ATimer = millis();
    while ((!CurrentRadio->available()) && (millis() - ATimer) < LISTEN_PERIOD) {delayMicroseconds(10);}// *** > Lock up sometimes happens here!! < ***
    Connected = CurrentRadio->available();
}

/************************************************************************************************************/


void ProdRadio(uint8_t Recon_Ch)
{ // After switching radios, this prod allows EITHER to connect. Don't know why - yet!

    CurrentRadio->enableAckPayload();
    CurrentRadio->enableDynamicPayloads();
    CurrentRadio->maskIRQ(1, 1, 1);
    CurrentRadio->setCRCLength(RF24_CRC_8);
    CurrentRadio->setPALevel(RF24_PA_MAX);
    CurrentRadio->setDataRate(RF24_250KBPS);
    CurrentRadio->openReadingPipe(1, ThisPipe);
    CurrentRadio->setChannel(Recon_Ch);
    delay(3);
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
    delay(1); // Allow swap over a little time to be noticed ...
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
    if ((millis() - NewConnectionMoment) < 20000) return;           // Let things settle down after connection for 20 seconds or so before using this
    if (millis() - SBUSTimer >= SBUSRATE) { // Does SBUS expect a packet?
        SBUSTimer = millis();               // Yes...
        if (!FailSafeSent)                  // But don't send after failsafe
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
    uint32_t SearchStartTime = millis();
    uint8_t ReconnectChannel = *(FHSSRecoveryPointer + ReconnectIndex); // Get a reconnect channel
    uint8_t PreviousRadio    = ThisRadio;
    uint8_t Attempts         = 0;

    if (ThisRadio == 1) RX1TotalTime += (millis() - ReconnectedMoment); // keep track of how long on each
    if (ThisRadio == 2) RX2TotalTime += (millis() - ReconnectedMoment);
    
    while (!Connected) {
        if (Blinking) BlinkLed();
        KickTheDog();
        if (BoundFlag) KeepSbusHappy(); // Some SBUS systems timeout FAST, so resend old data to keep it happy
        delayMicroseconds(300);
        CurrentRadio->stopListening();
        CurrentRadio->flush_tx(); 
        CurrentRadio->flush_rx(); 
        delay(3);                             // NEEDED!
        ReconnectChannel = *(FHSSRecoveryPointer + ReconnectIndex); // Get a reconnect channel
        ++ReconnectIndex;
        if (ReconnectIndex >= RECONNECT_CHANNELS_COUNT + RECONNECT_CHANNELS_START) ReconnectIndex = RECONNECT_CHANNELS_START;
        CurrentRadio->setChannel(ReconnectChannel);
        delay(1); 
        ++Attempts;
         
       
        if (Attempts < 3) {
            TryToConnectNow();
        }
       
        if (!Connected) {
           

#ifdef SECOND_TRANSCEIVER
          
            if (Attempts >= 3) {
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
    } // cannot pass here if not connected

     // must have connected by here
    FailSafeSent = false;
    if (PreviousRadio != ThisRadio) ++RadioSwaps; // Count the radio swaps
    ReconnectedMoment = millis();                 // Save this moment
    if (ModelMatched) {
        digitalWrite(LED_RED, HIGH);
        Blinking = false;
    }
    if (FailedSafe){
        FailedSafe = false;
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

void IncChannelNumber(){
        ++NextChannelNumber;                                            // Move up the channels' array
        if (NextChannelNumber >= FrequencyCount) NextChannelNumber = 1; // If needed, wrap the channels' array pointer
        AckPayload.Byte5 = NextChannelNumber;                           // Tell the transmitter which element of the array to use next.
        NextChannel      = *(FHSSChPointer + NextChannelNumber);        // Get the actual channel number from the array.      
}

/************************************************************************************************************/
// This function checks the time since last hop.
// If it's time to HOP, it sets the high bit in AckPayload.Purpose and both ends then HOP to new channel before next packet.
// The other 7 BITS of AckPayload.Purpose dictate the Payload's function (therefore 127 possibities.)
// This happens for *every* AckPayload, which return telemetry data as well as this hoptime information.
// Hence a single BIT directs the transmitter to hop.

void CheckWhetherItsTimeToHop()
{
    AckPayload.Purpose &= 0x7f;                                         // Clear the HOP flag
    if ((millis() - HopStart) >= HOPTIME) {                             // Time to hop??
        AckPayload.Purpose |= 0x80;                                     // Yes. So set the HOP flag leaving lower 7 bits unchanged
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
void SendIntToAckPayload(uint32_t U){                        // This one function now works with most int parameters
    union  {uint32_t Val32; uint8_t Val8[4];} ThisUnion;
    CheckWhetherItsTimeToHop();
    ThisUnion.Val32     = U;
    AckPayload.Byte1    = ThisUnion.Val8[0];           // These values are herewith delivered to Transmitter in Ack Payload
    AckPayload.Byte2    = ThisUnion.Val8[1];
    AckPayload.Byte3    = ThisUnion.Val8[2];
    AckPayload.Byte4    = ThisUnion.Val8[3];
}

/************************************************************************************************************/
// The unique Mac address of this Teensy 4.0 is sent to transmitter while binding to identify this model.
// This is to avoid the wrong model memory being used. 

void  SendMacAddress()
{
  union  {
      uint32_t Val32[2];
      uint8_t  Val8[8];     // the highest two bytes will always be zero. We didn't need all 8.
  } ThisUnion;
  uint8_t MaxAckP = 1;      // only packets 0 and 1 are needed here.

  ++ MacAddressSentCounter;

  AckPayload.Purpose &= 0x7F;
  ++AckPayload.Purpose;
  if (AckPayload.Purpose > MaxAckP) AckPayload.Purpose = 0; // wrap after max

  for (int i = 0; i < 8; ++i)  ThisUnion.Val8[i] = MacAddress[i];

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
    if (SensorHubConnected) MaxAckP = 18;                   // its 14 + GPS
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
