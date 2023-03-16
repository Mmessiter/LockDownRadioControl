
#ifndef TXstructs_h

#define TXstructs_h

#include <Arduino.h>
#include <PulsePosition.h>

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
// **********************************  Area & namespace for FHSS data ************************************************************
// **********************************************************************************************************************************

namespace FHSS_data{

uint8_t    FHSS_Channels1[42] = {93, 111, 107, 103, 106, 97, 108, 102, 118, // TEST array
                              104, 101, 109, 98, 113, 124, 115, 91, 96, 85, 117, 89, 99, 114, 87, 112,
                              86, 94, 92, 119, 120, 100, 121, 123, 95, 122, 105, 84, 116, 90, 110, 88};


uint8_t   FHSS_Channels[83] = {51, 28, 24, 61, 64, 55, 66, 19, 76, 21, 59, 67, 15, 71, 82, 32, 49, 69, 13, 2, 34, 47, 20, 16, 72, // UK array
                               35, 57, 45, 29, 75, 3, 41, 62, 11, 9, 77, 37, 8, 31, 36, 18, 17, 50, 78, 73, 30, 79, 6, 23, 40,
                               54, 12, 80, 53, 22, 1, 74, 39, 58, 63, 70, 52, 42, 25, 43, 26, 14, 38, 48, 68, 33, 27, 60, 44, 46,
                               56, 7, 81, 5, 65, 4, 10};
uint8_t   UkRulesCounter    = 0;
bool      UkRules           = true;
uint8_t*  FHSSRecoveryPointer;
uint8_t*  FHSSChPointer;                // pointer for channels array (three only used for Recovery)
uint8_t   NextChannelNumber  = 0;
}

// **********************************************************************************************************************************
// **********************************  Area & struct for PPM & TX MODULE ************************************************************
// **********************************************************************************************************************************

    #define A 1
    #define E 2
    #define T 3
    #define R 4

struct PPMArea{
    PulsePositionOutput     PPMOutputModule;                          // PPM for TX Modules
    PulsePositionOutput     PPMOutputBuddy;                           // PPM for buddy boxing 
    PulsePositionInput      PPMInputBuddy;                            // PPM for buddy boxing
    bool                    UseSBUSFromRX         = true;             // at receiver. false = PPM // heer
    uint16_t                PPMChannelCount       = 8;                // for our RX - NOT TX module  
    uint8_t                 PPMChannelOrder1[16]  = {A, E, T, R, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    uint8_t                 PPMChannelOrder2[16]  = {T, A, E, R, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    uint8_t                 PPMChannelOrder3[16]  = {E, T, A, R, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    uint8_t                 * PPMChannelOrder     = PPMChannelOrder2; // will point to needed channel order
    uint32_t                LastPPMFrame          = 0;
    uint8_t                 PPMOrderSelection     = 2;
    uint8_t                 PPMChannelsNumber     = 6;
    uint8_t                 PPMMillis             = 22;               // Not used!!! (yet)
    bool                    UseTXModule           = false;
};
PPMArea PPMdata;

#endif
