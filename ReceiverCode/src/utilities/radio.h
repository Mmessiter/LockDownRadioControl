/** @file ReceiverCode/src/utilities/radio.h */
#ifndef _SRC_UTILITIES_RADIO_H
#define _SRC_UTILITIES_RADIO_H
#include "common.h"

RF24            Radio1(pinCE1, pinCSN1);
RF24            Radio2(pinCE2, pinCSN2);
RF24*           CurrentRadio = &Radio1;
bool            Connected    = false;
bool            SaveNewBind  = true;
bool            HopNow       = false;
uint8_t         ThisRadio    = 1;
uint8_t         SavedPipeAddress[8];
uint8_t         NextChannelNumber = 0;
uint8_t         NextChannel;
uint8_t         ReconnectIndex = 0;
uint8_t         PacketNumber; 
uint16_t        ReceivedData[UNCOMPRESSEDWORDS];           //  20 x 16 BIT words
uint16_t        PreviousData[UNCOMPRESSEDWORDS];           /** Previously received data (used for servos. Hence not sent if unchanged) */
uint16_t        Interations = 0;
uint32_t        HopStart;
uint64_t        ThisPipe = 0xBABE1E5420LL; // default startup
uint64_t        NewPipe  = 0;
uint64_t        OldPipe  = 0;


extern bool     BoundFlag;
extern bool     GpsFix;
extern bool     SENSOR_HUB_CONNECTED;  
extern uint8_t  HoursGPS;
extern uint8_t  MinsGPS;
extern uint8_t  SecsGPS;
extern uint8_t  YearGPS;
extern uint8_t  MonthGPS;
extern uint8_t  DayGPS;
extern uint8_t  SatellitesGPS; 
extern uint16_t BaroAltitude;
extern uint32_t ReconnectedMoment;
extern float    INA219Volts;
extern float    BaroTemperature;
extern float    LatitudeGPS;
extern float    LongitudeGPS;
extern float    SpeedGPS;
extern float    AngleGPS;
extern float    AltitudeGPS;
extern float    DistanceGPS;
extern float    CourseToGPS;
  
extern void     FailSafe();                               // defined in main.cpp
extern void     ClearAckPayload();
extern void     ShowHopDurationEtc();
extern void     ReadSensorHub();
extern void     SetUKFrequencies();

 
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
uint8_t AckPayloadSize = sizeof(AckPayload);        // Size for later externs if needed etc. (=6)


/************************************************************************************************************/

void SetNewPipe()
{
    CurrentRadio->openReadingPipe(1, ThisPipe);
}

/************************************************************************************************************/

void ReadSavedPipe()
{
    for (uint8_t i = 0; i < 8; ++i) {
        SavedPipeAddress[i] = EEPROM.read(i); // uses first 8 bytes only.
    }
}

/************************************************************************************************************/

void LoadVersioNumber() // and which radio is currently in use
{
    AckPayload.Byte1 = ThisRadio;
    AckPayload.Byte2 = RXVERSION_MAJOR;
    AckPayload.Byte3 = RXVERSION_MINOR;
    AckPayload.Byte4 = RXVERSION_MINIMUS;
}

/************************************************************************************************************/

/** Store bounded pipe address from the received pairing payload. */
void GetNewPipe()
{
    NewPipe = (uint64_t)ReceivedData[0] << 56;
    NewPipe += (uint64_t)ReceivedData[1] << 48;
    NewPipe += (uint64_t)ReceivedData[2] << 40;
    NewPipe += (uint64_t)ReceivedData[3] << 32;
    NewPipe += (uint64_t)ReceivedData[4] << 24;
    NewPipe += (uint64_t)ReceivedData[5] << 16;
    NewPipe += (uint64_t)ReceivedData[6] << 8;
    NewPipe += (uint64_t)ReceivedData[7];
}

/************************************************************************************************************/

/**
 * Get pipe address from EEPROM.
 * @note Address data in EEPORM is valid only after a previous power cycle observed
 * a completed binding process (pairing was successful at least once during last flight's session).
 */
void GetOldPipe()
{
    ReadSavedPipe();
    OldPipe = (uint64_t)SavedPipeAddress[0] << 56;
    OldPipe += (uint64_t)SavedPipeAddress[1] << 48;
    OldPipe += (uint64_t)SavedPipeAddress[2] << 40;
    OldPipe += (uint64_t)SavedPipeAddress[3] << 32;
    OldPipe += (uint64_t)SavedPipeAddress[4] << 24;
    OldPipe += (uint64_t)SavedPipeAddress[5] << 16;
    OldPipe += (uint64_t)SavedPipeAddress[6] << 8;
    OldPipe += (uint64_t)SavedPipeAddress[7];
}

/************************************************************************************************************/

void HopToNextChannel()
{
    CurrentRadio->stopListening();
    delay(1);
    CurrentRadio->setChannel(NextChannel);
    CurrentRadio->startListening();
#ifdef DEBUG
    ShowHopDurationEtc();
#endif
}

/************************************************************************************************************/

/** Initialize a radio transceiver. */
void InitCurrentRadio()
{
    CurrentRadio->begin();
    CurrentRadio->enableAckPayload();       // needed
    CurrentRadio->enableDynamicPayloads();  // needed
    CurrentRadio->maskIRQ(1, 1, 1);         // no interrupts - seems NEEDED at the moment - (line *IS* connected)
    CurrentRadio->setCRCLength(RF24_CRC_8); // could be 16 or disabled
    CurrentRadio->setPALevel(RF24_PA_MAX);
    CurrentRadio->setDataRate(RF24_250KBPS);
    CurrentRadio->openReadingPipe(1, ThisPipe);
    SaveNewBind = true;
    HopStart    = millis();
    return;
}

/************************************ Try to connect  ... *********************************************/

void ListenALittle(uint32_t HowLong){
    uint32_t ATimer = millis();
    while ((!CurrentRadio->available()) && (millis() - ATimer) < HowLong) { }    // Wait here until connected, or timeout
    if (CurrentRadio->available()) Connected = true;
}

/************************************************************************************************************/

#ifdef SECOND_TRANSCEIVER
void ProdRadio(uint8_t Recon_Ch)
{ // After switching radios, this prod allows EITHER to connect. Don't know why - yet!
    // Re-setting up the radio may be required on power failure or unsufficient current supply. While this function
    // addresses a power "drop-out" during Reconnect(), there may be times that require this while connected.
    CurrentRadio->enableAckPayload();
    CurrentRadio->enableDynamicPayloads();
    CurrentRadio->maskIRQ(1, 1, 1);                                                        // no interrupts - seems NEEDED at the moment - (line *IS* connected)
    CurrentRadio->setCRCLength(RF24_CRC_8);
    CurrentRadio->setPALevel(RF24_PA_MAX);
    CurrentRadio->setDataRate(RF24_250KBPS);
    CurrentRadio->openReadingPipe(1, ThisPipe);
    CurrentRadio->setChannel(Recon_Ch);
    CurrentRadio->startListening();
    ListenALittle(PROD_LISTEN_PERIOD);
}
#endif // defined (SECOND_TRANSCEIVER)

/************************************************************************************************************/

void Reconnect(){

    uint32_t SearchStartTime  = 0;
    uint8_t  ReconnectChannel = 0;

    SearchStartTime = millis();
    FailSafeSent    = false;
    while (!Connected) {
        CurrentRadio->stopListening();
        delay(1);           
        ReconnectChannel = * (ReConPointer + ReconnectIndex);                     // Get a reconnect channel - not always the same one - one of 5 now.
        ++ ReconnectIndex;
        if (ReconnectIndex >= RECONNECT_CHANNELS_COUNT) ReconnectIndex = 0;
        CurrentRadio->setChannel(ReconnectChannel);  
        CurrentRadio->startListening();
        ListenALittle(START_LISTEN_PERIOD);
#ifdef SECOND_TRANSCEIVER
        CurrentRadio->stopListening();
        delay(1);
        if (ThisRadio == 2) {
            CurrentRadio = &Radio1;
            ThisRadio    = 1;
        } else {
            CurrentRadio = &Radio2;
            ThisRadio    = 2;
        }
        ProdRadio(ReconnectChannel);
#endif // defined (SECOND_TRANSCEIVER)
        ListenALittle(LISTEN_PERIOD);
        if (!Connected) {
            if ((millis() - SearchStartTime) > FAILSAFE_TIMEOUT){
                if (!FailSafeSent){
                    FailSafe();
                    FailSafeSent = true; // Once is enough
                    SetUKFrequencies();
                }
            }
        }
    }
    ReconnectedMoment    = millis();  // Save this moment, then don't move a servo for a few ms ...
}
/************************************************************************************************************/
// This function checks the time since last hop. 
// If it's time to HOP, it sets the high bit in AckPayload.Purpose and both ends then HOP to new channel before next packet.
// The other 7 BITS of AckPayload.Purpose still dictate the Payload's function ( < 127 possibities. )
// This now happens for *every* AckPayload, which can and does return telemetry data as well as this hoptime information.
// Hence a single BIT now directs the transmitter to hop.

void CheckIfItsHopTime(){
    AckPayload.Purpose  &= 0x7f;                   // Clear the HOP flag
    if ((millis() - HopStart) >= HOPTIME){         // Time to hop?? 
         AckPayload.Purpose |= 0x80;               // Yes. So set the HOP flag leaving lower 7 bits unchanged
         ++NextChannelNumber;                      // Move up the channels' array
         if (NextChannelNumber >= FrequencyCount)  NextChannelNumber = 1; // If needed, wrap the channels' array pointer
         AckPayload.Byte5 = NextChannelNumber;     // Tell the transmitter which element of the array to use next.
         NextChannel =  * (FHSSChPointer + NextChannelNumber);   // Get the actual channel number from the array.
         HopNow = true;                            // Set local flag and hop when ready BUT NOT BEFORE.
    }
}
/************************************************************************************************************/
void SendToAckPayload(float U){                        // This one function now works with most parameters
    union  {float Val32; uint8_t Val8[4];} ThisUnion;
    CheckIfItsHopTime();
    ThisUnion.Val32     = U;
    AckPayload.Byte1    = ThisUnion.Val8[0];           // These values are herewith delivered to Transmitter in Ack Payload
    AckPayload.Byte2    = ThisUnion.Val8[1];
    AckPayload.Byte3    = ThisUnion.Val8[2];
    AckPayload.Byte4    = ThisUnion.Val8[3];
}
/************************************************************************************************************/
void SendTimeToAckPayload(){    
    CheckIfItsHopTime();                  
    AckPayload.Byte1    = SecsGPS;    
    AckPayload.Byte2    = MinsGPS;
    AckPayload.Byte3    = HoursGPS;
}
/************************************************************************************************************/
void SendDateToAckPayload(){   
    CheckIfItsHopTime();                         
    AckPayload.Byte1    = DayGPS;  
    AckPayload.Byte2    = MonthGPS;
    AckPayload.Byte3    = YearGPS;
}
/************************************************************************************************************/
void LoadAckPayload()
{
    uint8_t MaxAckP     = 0;                                      // 0 if only RX
    AckPayload.Purpose &= 0x7F;                                   // NOTE: The HIGH BIT of "purpose" bit is the HOPNOW flag. It gets set only when it's time to hop.
    ++AckPayload.Purpose;   
    if (INA219_CONNECTED) MaxAckP = 1;
    if (SENSOR_HUB_CONNECTED) MaxAckP = 14;                       // its 14 + GPS
    if (AckPayload.Purpose > MaxAckP) AckPayload.Purpose = 0;     // wrap after max
    switch (AckPayload.Purpose) {
        case 0: 
            LoadVersioNumber();
            break;  
        case 1: 
            SendToAckPayload(INA219Volts);
            break;
        case 2:
            SendToAckPayload(BaroAltitude);
            break;
        case 3: 
            SendToAckPayload(BaroTemperature);
            break;
        case 4: 
            SendToAckPayload (LatitudeGPS);       
            break;
        case 5: 
             SendToAckPayload (LongitudeGPS);
            break;
        case 6: 
             SendToAckPayload (AngleGPS); 
            break;
        case 7: 
             SendToAckPayload(SpeedGPS);  
            break;
        case 8: 
            SendToAckPayload(GpsFix);    
            break;
        case 9: 
            SendToAckPayload(AltitudeGPS);      
            break;
        case 10: 
            SendToAckPayload(DistanceGPS);    
            break;
        case 11: 
            SendToAckPayload(CourseToGPS);       
            break;
        case 12: 
            SendToAckPayload(SatellitesGPS);     
            break;
        case 13: 
            SendDateToAckPayload();
            break;
        case 14: 
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
void Decompress(uint16_t* uncompressed_buf, uint16_t* compressed_buf, int uncompressed_size)
{
    int p = 0;
    for (int l = 0; l < (uncompressed_size * 3 / 4); l += 3) {
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
