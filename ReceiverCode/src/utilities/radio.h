/** @file ReceiverCode/src/utilities/radio.h */
#ifndef _SRC_UTILITIES_RADIO_H
#define _SRC_UTILITIES_RADIO_H

#include <SPI.h>
#include <RF24.h>
#include "common.h"

#define pinCE1  9  // NRF1
#define pinCSN1 10 // NRF1

#define pinCSN2 20 // NRF2
#define pinCE2  21 // NRF2

#define FAILSAFE_TIMEOUT 2000

RF24    Radio1(pinCE1, pinCSN1);
RF24    Radio2(pinCE2, pinCSN2);
RF24*   CurrentRadio = &Radio1;
uint8_t ThisRadio    = 1;

uint64_t ThisPipe = 0xBABE1E5420LL; // default startup
uint64_t NewPipe  = 0;
uint64_t OldPipe  = 0;

bool Connected          = false;
int  SearchStartTime    = 0;
int  StillSearchingTime = 0;

bool     SaveNewBind = true;
uint8_t  SavedPipeAddress[8];
uint32_t HopStart;
uint8_t  NextChannelNumber = 0;
uint32_t RXTimeStamp;
bool     HopNow = false;

extern void     ShowHopDurationEtc();
extern void     DoSensors();
extern bool     Radio1Exists;
extern bool     Radio2Exists;
extern float    BaroAltitude;
extern float    INA219Volts;
extern float    BaroTemperature;
extern double   LatitudeGPS;
extern double   LongitudeGPS;
extern double   SpeedGPS;
extern double   AngleGPS;
extern double   AltitudeGPS;
extern bool     GpsFix;
extern bool     USE_AdafruitUltimateGps;    
extern bool     USE_BMP280;
extern bool     USE_INA219;
 

/** AckPayload Stucture for data returned to transmitter. */
struct Payload
{
    /**
     * This byte (Purpose) determines what the remainder (offset 19) represent.
     * @warning Highest BIT of Purpose means **IGNORE IF ON**
     * @note If Purpose = 1 then ...
     * @code
     * AckPayload.Byte2           =  ThisRadio;            // Radio in current use  Byte1 and Byte2 are free
     * AckPayload.Byte3           =  RXVERSION_MAJOR;
     * AckPayload.Byte4           =  RXVERSION_MINOR;
     * AckPayload.Byte5           =  RXVERSION_MINIMUS;

     *
     * @note If Purpose = 2 then ...
     * @code
     * AckPayload.Byte1     = Time.Stamp8[0];       // Time stamp is 32 BIT divided up here.
     * AckPayload.Byte2     = Time.Stamp8[1];
     * AckPayload.Byte3     = Time.Stamp8[2];
     * AckPayload.Byte4     = Time.Stamp8[3];
     * AckPayload.Byte5     = Next array pointer for next channel
     *
     * * @note If Purpose = 3 then ...
     * @code
     * AckPayload.Byte1  ... Byte4   = Volts (float)      Byte5 is free

     * @endcode
     **/

    uint8_t Purpose = 0; // 0  Purpose
    uint8_t Byte1   = 0; // 1  (was volt)
    uint8_t Byte2   = 0; // 2  (was CurrentAltitude)
    uint8_t Byte3   = 0; // 3  (was ReportedPitch)
    uint8_t Byte4   = 0; // 4  (was ReportedRoll)
    uint8_t Byte5   = 0; // 5  (was ReportedYaw)
};
Payload AckPayload;                          /** object allocated for returned ACK data. */
uint8_t AckPayloadSize = sizeof(AckPayload); // Size for later externs if needed etc.

uint8_t PacketNumber; /** A counter for packets between channel hops. */

uint32_t ConnectionStart;

#define UNCOMPRESSEDWORDS 20                        //   16 Channels plus extra 4 16 BIT values
#define COMPRESSEDWORDS   UNCOMPRESSEDWORDS * 3 / 4 // = 16 WORDS  with no extra

uint16_t ReceivedData[UNCOMPRESSEDWORDS]; //  20  words
uint16_t PreviousData[UNCOMPRESSEDWORDS]; /** Previously received data (used for servos. Hence not used if unchanged) */

extern void     FailSafe(); // defined in main.cpp
extern uint32_t ReconnectedMoment;
extern bool     BoundFlag;
extern void     ClearAckPayload();
byte            NextFrequency;
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
    AckPayload.Byte2 = ThisRadio;
    AckPayload.Byte3 = RXVERSION_MAJOR;
    AckPayload.Byte4 = RXVERSION_MINOR;
    AckPayload.Byte5 = RXVERSION_MINIMUS;
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

/**
 * Make radio transceiver "hop" over to the new frequency.
 * @param freq The next frequency to use.
 */

void GetNextFrequency()
{
    NextFrequency = FHSS_Channels[NextChannelNumber];
}

/************************************************************************************************************/

void HopToNextFrequency()
{
    CurrentRadio->stopListening();
    CurrentRadio->setChannel(NextFrequency);
    CurrentRadio->startListening();
#ifdef DEBUG
    ShowHopDurationEtc();
#endif
}

/************************************************************************************************************/

/** Initialize a radio transceiver. */
bool InitCurrentRadio()
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
    return true; // let's assume its connected
}

/************************************************************************************************************/
#ifdef SECOND_TRANSCEIVER
void ProdRadio()
{ // After switching radios, this prod allows EITHER to connect. Don't know why - yet!
    // Re-setting up the radio may be required on power failure or unsufficient current supply. While this function
    // addresses a power "drop-out" during Reconnect(), there may be times that require this while connected.
    CurrentRadio->enableAckPayload();
    CurrentRadio->enableDynamicPayloads();
    CurrentRadio->maskIRQ(1, 1, 1); // no interrupts - seems NEEDED at the moment - (line *IS* connected)
    CurrentRadio->setCRCLength(RF24_CRC_8);
    CurrentRadio->setPALevel(RF24_PA_MAX);
    CurrentRadio->setDataRate(RF24_250KBPS);
    CurrentRadio->openReadingPipe(1, ThisPipe);
    CurrentRadio->setChannel(RECONNECT_CH);
    CurrentRadio->startListening();
    delay(1); // This might help
}
#endif // defined (SECOND_TRANSCEIVER)

/************************************************************************************************************/

void Reconnect()
{
    uint32_t TryTimer;
    uint8_t  ReconnectAttempts = 0;

    SearchStartTime = millis();
    FailSafeSent    = false;
    CurrentRadio->stopListening();
    delay(1);
    CurrentRadio->setChannel(RECONNECT_CH);
    CurrentRadio->startListening();
    delay(1);
    if (CurrentRadio->available())
    {
        Connected = true;
    }
    while (!Connected)
    {
        ++ReconnectAttempts;
#ifdef SECOND_TRANSCEIVER
        if (ReconnectAttempts > 2)
        {
            CurrentRadio->stopListening();
            delay(1);
            if (ThisRadio == 2)
            {
                CurrentRadio = &Radio1;
                ThisRadio    = 1;
            }
            else
            {
                CurrentRadio = &Radio2;
                ThisRadio    = 2;
            }
            ProdRadio();
            ReconnectAttempts = 0;
        }
#endif // defined (SECOND_TRANSCEIVER)
        TryTimer = millis();
        while ((!CurrentRadio->available()) && (millis() - TryTimer) < 100) {
        } // wait a mo during attempt to connect ...
        if (CurrentRadio->available()) Connected = true;
        if (!Connected)
        {
            StillSearchingTime = millis() - SearchStartTime;
            if (StillSearchingTime > FAILSAFE_TIMEOUT)
            {
                if (!FailSafeSent)
                {
                    FailSafe();
                    FailSafeSent = true; // Once is enough
                }
            }
        }
    }
    ConnectionStart    = millis();
    StillSearchingTime = 0;
    ReconnectedMoment  = ConnectionStart; // Save this moment, then don't move a servo for a few ms ...
}
/************************************************************************************************************/
void LoadTimeStamp()
{ // This will load time stamp and array index for return to TX for synch purposes

#define HOPTIME         55 // ms between channel changes
#define FREQUENCYSCOUNT 82 // use 82 different channels

    union // union used to allow access to each byte of 32 bit value
    {
        uint32_t Stamp32;
        uint8_t  Stamp8[4];
    } Time;

    Time.Stamp32 = millis() - HopStart;
    RXTimeStamp  = Time.Stamp32;
    if (RXTimeStamp >= HOPTIME) {
        HopStart     = millis();
        Time.Stamp32 = 0;
        RXTimeStamp  = 0;
        ++NextChannelNumber;
        if (NextChannelNumber >= FREQUENCYSCOUNT) {
            NextChannelNumber = 1;
        } // Zero will mean error (so that element not used)
        GetNextFrequency();
        HopNow = true; // Set flag and hop when ready *** BUT NOT BEFORE ****  !!!!!
    }
    AckPayload.Byte1 = Time.Stamp8[0]; // These values are herewith delivered to Transmitter in Ack Payload
    AckPayload.Byte2 = Time.Stamp8[1];
    AckPayload.Byte3 = Time.Stamp8[2];
    AckPayload.Byte4 = Time.Stamp8[3];
    AckPayload.Byte5 = NextChannelNumber;
}


/************************************************************************************************************/

void LoadGPSLongitude()
{
    union // union used to allow access to each byte of 32 bit float
    {
        float   Val32;
        uint8_t Val8[4];
    } ThisUnion;

    ThisUnion.Val32     = LongitudeGPS;
    AckPayload.Byte1    = ThisUnion.Val8[0];    // These values are herewith delivered to Transmitter in Ack Payload
    AckPayload.Byte2    = ThisUnion.Val8[1];
    AckPayload.Byte3    = ThisUnion.Val8[2];
    AckPayload.Byte4    = ThisUnion.Val8[3];
}

/************************************************************************************************************/

void LoadGPSLatitude()
{
    union // union used to allow access to each byte of 32 bit float
    {
        float   Val32;
        uint8_t Val8[4];
    } ThisUnion;

    ThisUnion.Val32     = LatitudeGPS;
    AckPayload.Byte1    = ThisUnion.Val8[0];    // These values are herewith delivered to Transmitter in Ack Payload
    AckPayload.Byte2    = ThisUnion.Val8[1];
    AckPayload.Byte3    = ThisUnion.Val8[2];
    AckPayload.Byte4    = ThisUnion.Val8[3];
}

/************************************************************************************************************/

void LoadGPSAngle()
{
    union // union used to allow access to each byte of 32 bit float
    {
        float   Val32;
        uint8_t Val8[4];
    } ThisUnion;

    ThisUnion.Val32     = AngleGPS;
    AckPayload.Byte1    = ThisUnion.Val8[0];    // These values are herewith delivered to Transmitter in Ack Payload
    AckPayload.Byte2    = ThisUnion.Val8[1];
    AckPayload.Byte3    = ThisUnion.Val8[2];
    AckPayload.Byte4    = ThisUnion.Val8[3];
}

/************************************************************************************************************/

void LoadGPSSpeed()
{
    union // union used to allow access to each byte of 32 bit float
    {
        float   Val32;
        uint8_t Val8[4];
    } ThisUnion;

    ThisUnion.Val32     = SpeedGPS;
    AckPayload.Byte1    = ThisUnion.Val8[0];    // These values are herewith delivered to Transmitter in Ack Payload
    AckPayload.Byte2    = ThisUnion.Val8[1];
    AckPayload.Byte3    = ThisUnion.Val8[2];
    AckPayload.Byte4    = ThisUnion.Val8[3];
}

/************************************************************************************************************/

void LoadGPSFix()
{
    union // union used to allow access to each byte of 32 bit float
    {
        float   Val32;
        uint8_t Val8[4];
    } ThisUnion;

    ThisUnion.Val32     = GpsFix;
    AckPayload.Byte1    = ThisUnion.Val8[0];    // These values are herewith delivered to Transmitter in Ack Payload
    AckPayload.Byte2    = ThisUnion.Val8[1];
    AckPayload.Byte3    = ThisUnion.Val8[2];
    AckPayload.Byte4    = ThisUnion.Val8[3];
}
/************************************************************************************************************/

void LoadRXVolts()
{
    union // union used to allow access to each byte of 32 bit float
    {
        float   Val32;
        uint8_t Val8[4];
    } RXVolts;

    RXVolts.Val32    = INA219Volts;
    AckPayload.Byte1 = RXVolts.Val8[0];    // These values are herewith delivered to Transmitter in Ack Payload
    AckPayload.Byte2 = RXVolts.Val8[1];
    AckPayload.Byte3 = RXVolts.Val8[2];
    AckPayload.Byte4 = RXVolts.Val8[3];
}

/************************************************************************************************************/
void LoadAltitude()  // Baro (not GPS)
{

    union // union used to allow access to each byte of 32 bit float
    {
        float   Val32;
        uint8_t Val8[4];
    } AltitudeUnion;

    AltitudeUnion.Val32 = AltitudeGPS ;        // BaroAltitude;
    AckPayload.Byte1    = AltitudeUnion.Val8[0]; // These values are herewith delivered to Transmitter in Ack Payload
    AckPayload.Byte2    = AltitudeUnion.Val8[1];
    AckPayload.Byte3    = AltitudeUnion.Val8[2];
    AckPayload.Byte4    = AltitudeUnion.Val8[3];
}

/************************************************************************************************************/

void LoadTemperature()
{
    union // union used to allow access to each byte of 32 bit float
    {
        float   Val32;
        uint8_t Val8[4];
    } TemperatureUnion;
    TemperatureUnion.Val32 = BaroTemperature;
    AckPayload.Byte1       = TemperatureUnion.Val8[0]; // These values are herewith delivered to Transmitter in Ack Payload
    AckPayload.Byte2       = TemperatureUnion.Val8[1];
    AckPayload.Byte3       = TemperatureUnion.Val8[2];
    AckPayload.Byte4       = TemperatureUnion.Val8[3];
}

/************************************************************************************************************/

void LoadAckPayload()
{
    uint8_t MaxAckP     = 2;    // 2 if only RX
    AckPayload.Purpose &= 0x7F; // Clear hi bit ( = do not ignore)
    ++AckPayload.Purpose;       // 0 = ...
                                // 1 =  Version number
                                // 2 =  Time stamp for FHSS
                                // ... etc ...
      if (USE_BMP280) MaxAckP              = 4;                     // 4 + volts
      if (USE_INA219) MaxAckP              = 8;                     // 8 + Baro
      if (USE_AdafruitUltimateGps) MaxAckP = 18;                    // 18 + GPS
      if (AckPayload.Purpose > MaxAckP) AckPayload.Purpose = 0;     // wrap after max

    switch (AckPayload.Purpose) {
        case 0: 
            LoadTimeStamp();
            break;
        case 1: 
            LoadVersioNumber();
            break;
        case 2: 
            LoadTimeStamp();
            break;
        case 3: 
            LoadRXVolts();
            break;
        case 4: 
            LoadTimeStamp();
            break;
        case 5:
            LoadAltitude();
            break;
        case 6: 
            LoadTimeStamp();
            break;
        case 7: 
            LoadTemperature();
            break;
        case 8: 
            LoadTimeStamp();
            break;
        case 9: 
            LoadGPSLatitude();    // ********* GPS *******************
            break;
        case 10: 
            LoadTimeStamp();
            break;
        case 11: 
            LoadGPSLongitude();   // ********* GPS *******************
            break;
        case 12: 
            LoadTimeStamp();
            break;
        case 13: 
            LoadGPSAngle();       // ********* GPS *******************
            break;
        case 14: 
            LoadTimeStamp();
            break;
        case 15: 
            LoadGPSSpeed();       // ********* GPS *******************
            break;
        case 16:
            LoadTimeStamp();
            break;
        case 17: 
            LoadGPSFix();         // ********* GPS *******************
            break;
        case 18:
            LoadTimeStamp();
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
