// ****************************** Structs etc *******************************************************
#include <Arduino.h>
#include "Hardware/Definitions.h"
#ifndef STRUCTSETC_H
    #define STRUCTSETC_H

/* ************************************* AckPayload structure ******************************************************

 * This first byte "Purpose" defines what all the other bytes mean, AND ...
 * the highest BIT of Purpose means ** HOP TO NEXT CHANNEL A.S.A.P. (IF ON) **
 * the lower 7 BITs then define the meaning of the remainder of the ackpayload bytes
 */

struct Payload
{
    uint8_t Purpose; // Defines meaning of the remainder
                     // Highest BIT of Purpose means HOP NOW! when ON
    uint8_t Byte1;   //
    uint8_t Byte2;   //
    uint8_t Byte3;   //
    uint8_t Byte4;   //
    uint8_t Byte5;   //
};
Payload AckPayload;

const uint8_t AckPayloadSize = sizeof(AckPayload); // i.e. 6

// *****************************************************************************************************************

// **********************************************************************************************************************************
// **********************************  Area & struct for PPM & TX MODULE ************************************************************
// **********************************************************************************************************************************

struct PPMArea
{
    PulsePositionOutput PPMOutputModule;        // PPM for TX Modules
    PulsePositionOutput PPMOutputBuddy;         // PPM for buddy boxing
    PulsePositionInput  PPMInputBuddy;          // PPM for buddy boxing
    bool                UseSBUSFromRX   = true; // at receiver. false = PPM
    uint16_t            PPMChannelCount = 8;    // for our RX - NOT TX module

                                                //   A  E  T  R       // TRANSLATION HAPPENS *BACKWARDS* !
    uint8_t PPMChannelOrder1[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    //   T  A  E  R
    uint8_t PPMChannelOrder2[16] = {2, 3, 1, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    //   E  T  A  R
    uint8_t PPMChannelOrder3[16] = {3, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

    uint8_t* PPMChannelOrder   = PPMChannelOrder2; // This will point to needed channel order
    uint32_t LastPPMFrame      = 0;
    uint8_t  PPMOrderSelection = 2;
    uint8_t  PPMChannelsNumber = 6;
    bool     UseTXModule       = false;
};
PPMArea PPMdata;

#endif
