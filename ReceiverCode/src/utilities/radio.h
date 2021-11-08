#ifndef _SRC_UTILITIES_RADIO_H
#define _SRC_UTILITIES_RADIO_H

#include <SPI.h>
#include <RF24.h>
#include "common.h"

#define pinCE1  9  // NRF1
#define pinCSN1 10 // NRF1

#define pinCSN2 20 // NRF2
#define pinCE2  21 // NRF2

#define FHSS_RESCUE_BOTTOM 118
#define FHSS_RESCUE_TOP    125

#define FAILSAFE_TIMEOUT 2000

RF24  Radio1(pinCE1, pinCSN1);
RF24  Radio2(pinCE2, pinCSN2);
RF24* CurrentRadio = &Radio1;

uint64_t ThisPipe = 0xBABE1E5420LL; // default startup
uint64_t NewPipe  = 0;
uint64_t OldPipe  = 0;

bool    Connected          = false;
int     SearchStartTime    = 0;
int     StillSearchingTime = 0;
uint8_t ReconnectAttempts  = 0;
uint8_t ThisRadio          = 1;
bool    SaveNewBind        = true;
uint8_t SavedPipeAddress[8];

/** AckPayload Stucture for data returned to transmitter. */
struct Payload
{
    /**
     * This byte (Purpose) determines what the remainder (offset 19) represent.
     * Highest BIT of Purpose means >>IGNORE IF ON<<
     * If Purpose = 1 then ...
     * AckPayload.ReportedPitch   =  RXVERSION_MAJOR;
     * AckPayload.ReportedRoll    =  RXVERSION_MINOR;
     * AckPayload.ReportedYaw     =  RXVERSION_MINIMUS;
     * AckPayload.CurrentAltitude =  ThisRadio;  // Radio in current use
     **/
    uint8_t Purpose         = 0;
    uint8_t volt            = 0; /** Voltage of RX battery, if measured. */
    uint8_t CurrentAltitude = 0; /** Altitude, if measured. */
    uint8_t ReportedPitch   = 0;
    uint8_t ReportedRoll    = 0;
    uint8_t ReportedYaw     = 0;
};
Payload AckPayload;                          /** object allocated for returned ACK data. */
uint8_t AckPayloadSize = sizeof(AckPayload); // Size for later externs if needed etc.

uint8_t PacketNumber; /** A counter for packets between channel hops. */

uint32_t ConnectionStart;

#define UNCOMPRESSEDWORDS 20                        //   16 Channels plus extra 4 16 BIT values
#define COMPRESSEDWORDS   UNCOMPRESSEDWORDS * 3 / 4 // = 16 WORDS  with no extra

uint16_t ReceivedData[UNCOMPRESSEDWORDS]; //  20  words
uint16_t PreviousData[UNCOMPRESSEDWORDS]; /** Previously received data (used for servos). */

extern void FailSafe(); // defined in main.cpp
extern uint32_t ReconnectedMoment;
extern bool BoundFlag;
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
    AckPayload.ReportedPitch = RXVERSION_MAJOR;
    AckPayload.ReportedRoll  = RXVERSION_MINOR;
    AckPayload.ReportedYaw   = RXVERSION_MINIMUS;
    AckPayload.CurrentAltitude = ThisRadio;
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

/**
 * Make radio transceiver "hop" over to the new frequency.
 * @param freq The next frequency to use.
 */
void HopToNextFrequency(uint8_t freq)
{
    CurrentRadio->stopListening();
    CurrentRadio->setChannel(freq);
    CurrentRadio->startListening();
    LastConnectionMoment = millis();
#ifdef DEBUG
    ShowHopDurationEtc(freq);
#endif
}

/** Initialize a radio transceiver. */
void InitCurrentRadio()
{
    if (CurrentRadio->begin()) {
        CurrentRadio->enableAckPayload();       // needed
        CurrentRadio->enableDynamicPayloads();  // needed
        CurrentRadio->maskIRQ(1, 1, 1);         // no interrupts - seems NEEDED at the moment - (line *IS* connected)
        CurrentRadio->setCRCLength(RF24_CRC_8); // could be 16 or disabled
        CurrentRadio->setPALevel(RF24_PA_MAX);
        CurrentRadio->setDataRate(RF24_250KBPS);
        CurrentRadio->openReadingPipe(1, ThisPipe);
        SaveNewBind = true;
    }
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
    delay(9);                           // This might help
}
 #endif // defined (SECOND_TRANSCEIVER)
/************************************************************************************************************/

void Reconnect()
{
    SearchStartTime   = millis();
    ReconnectAttempts = 0;

#ifdef SECOND_TRANSCEIVER
#ifdef SECOND_TRANSCEIVER_DEBUG
    if (((millis()-ConnectionStart)/1000) > 0){
    Serial.print ("                         ********    Duration on Radio: ");
    Serial.print (ThisRadio);
    Serial.print (" = ");
    Serial.print ((millis()-ConnectionStart)/1000);
    Serial.println  (" Seconds ********");
    }
 #endif // defined (SECOND_TRANSCEIVER)
 #endif // defined (SECOND_TRANSCEIVER_DEBUG)
    while (!Connected)
    {
        StillSearchingTime = millis() - SearchStartTime;
        ++ReconnectAttempts;
        
        uint8_t i = FHSS_RESCUE_BOTTOM;
        while (!CurrentRadio->available() && i <= FHSS_RESCUE_TOP) // This loop exits as soon as connection is detected.
        {
            CurrentRadio->stopListening();
            CurrentRadio->setChannel(i);
            CurrentRadio->startListening();
            delay(4); // was 4
            i++;
        }
        
#ifdef SECOND_TRANSCEIVER
        if (!CurrentRadio->available()) {
            if (ReconnectAttempts > 5) {            // This might be a bigger number after tests
                CurrentRadio->stopListening();      // This has helped massively
                delay(4);                           // This might help
                ReconnectAttempts = 0;
                if (ThisRadio == 1) {
                    ThisRadio    = 2;
                    CurrentRadio = &Radio2;
                    ProdRadio();
                }
                else {
                    ThisRadio    = 1;
                    CurrentRadio = &Radio1;
                    ProdRadio();
                }
#ifdef SECOND_TRANSCEIVER_DEBUG
                Serial.print(millis());              // These lines are just to help fix this area!!
                Serial.print("  ? Testing Radio: "); // These lines are just to help fix this area!!
                Serial.println(ThisRadio);           // These lines are just to help fix this area!!
#endif
            }
        }
#endif // defined (SECOND_TRANSCEIVER)

        if (CurrentRadio->available())
        {
            ConnectionStart=millis();
            ReconnectedMoment=ConnectionStart;        // Save this moment, then don't move a servo for a few ms ....

#ifdef SECOND_TRANSCEIVER    
#ifdef SECOND_TRANSCEIVER_DEBUG
            Serial.print(millis());                   // These lines are just to help fix this area!!
            Serial.print("  ! Connected on Radio: "); // These lines are just to help fix this area!!
            Serial.println(ThisRadio);                // These lines are just to help fix this area!!
#endif  // defined (SECOND_TRANSCEIVER)
#endif  // defined (SECOND_TRANSCEIVER_DEBUG)        

            Connected          = true; // Connection is re-established so return, smiling!
            FailSafeSent       = false;
            ReconnectAttempts  = 0;
            StillSearchingTime = 0;
        }
        else if (StillSearchingTime >= FAILSAFE_TIMEOUT)
        {
            if (!FailSafeSent)
            {
                FailSafe();
                FailSafeSent = true; // Once is enough
#ifdef DB_FAILSAFE
                Serial.println("FailSafe sent");
#endif
            }
        }
        delay (20); // This seems to prevent the occasional lockup??
    }
}

/************************************************************************************************************/

void LoadAckPayload()
{
    AckPayload.Purpose &= 0x7F; // Clear hi bit (=do not ignore)

    ++AckPayload.Purpose;                               // 0 =  Roll, Pitch, Yaw, Volts.
                                                        // 1 =  Version number
    if (AckPayload.Purpose > 1) AckPayload.Purpose = 0; // 1 is currently max
    if (AckPayload.Purpose == 1)
    {
        LoadVersioNumber();                             // if 1 send version info AND RX number
    }
}

/**
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
        p++;
        uncompressed_buf[p] = (compressed_buf[l] & 0xf) << 8 | compressed_buf[l + 1] >> 8;
        p++;
        uncompressed_buf[p] = (compressed_buf[l + 1] & 0xff) << 4 | compressed_buf[l + 2] >> 12;
        p++;
        uncompressed_buf[p] = compressed_buf[l + 2] & 0xfff;
        p++;
    }
}

#endif // defined (_SRC_UTILITIES_RADIO_H)
