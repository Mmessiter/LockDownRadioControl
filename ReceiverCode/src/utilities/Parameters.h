/** @file ReceiverCode/src/utilities/Parameters.h */
// Malcolm Messiter 2020 - 2025
#ifndef _SRC_PARAMETERS_RADIO_H
#define _SRC_PARAMETERS_RADIO_H
#include "utilities/1Definitions.h"

// ************************************************************************/
/**
 * This function decodes a float value from four 16-bit words.
 * Only the lower 8 bits of each word are used.
 * Each word is cast to a uint8_t and placed into a union representing the bytes of a float.
 * This reconstructs the original float value which was sent as four bytes.
 */
float DecodeFloat(uint16_t word1, uint16_t word2, uint16_t word3, uint16_t word4)
{
    union
    {
        float f;
        uint8_t b[4];
    } u;
    u.b[0] = uint8_t(word1);
    u.b[1] = uint8_t(word2);
    u.b[2] = uint8_t(word3);
    u.b[3] = uint8_t(word4);
    return u.f;
}
///************************************************************************************************************/
void ShowValues(const char *name, float value) // // Show values in the serial monitor for debugging purposes
{
    Look1(name);
    Look1(": ");
    Look(value, 3); // 3 decimal places
}
/************************************************************************************************************/
/** Read extra parameters from the transmitter.
 * extra parameters are sent using the last few words bytes in every data packet.
 * the parameter sent is defined by the packet number & the packet number defined the transmitter.
 */
void ReadExtraParameters()
{
    uint16_t TwoBytes = 0;
    switch (Parameters.ID)
    {
    case FAILSAFE_SETTINGS:                                      // 1
        FS_byte1 = Parameters.word[1] & 0xff;                    // These 2 bytes are 16 failsafe flags
        FS_byte2 = Parameters.word[2] & 0xff;                    // These 2 bytes are 16 failsafe flags
        TwoBytes = uint16_t(FS_byte2) + uint16_t(FS_byte1 << 8); // because of the shift left 8, adding here is the same as ORing them.
        RebuildFlags(FailSafeChannel, TwoBytes);
        SaveFailSafeDataToEEPROM();
        break;
    case QNH_SETTING: // 2
        Qnh = (float)Parameters.word[1];
        break;
    case GPS_MARK_LOCATION: // 3
        if (Parameters.word[2] == 255)
        { // Mark this location
            MarkHere();
            Parameters.word[2] = 0; // ... Once only
        }
        break;
#ifdef USE_PWM
    case SERVO_FREQUENCIES: // 6
        for (int i = 0; i < SERVOSUSED; ++i)
            ServoFrequency[i] = Parameters.word[i + 1];
        SetServoFrequency();
        break;
    case SERVO_PULSE_WIDTHS: // 7
        for (int i = 0; i < SERVOSUSED; ++i)
            ServoCentrePulse[i] = Parameters.word[i + 1];
        break;
#endif

    default:
        break;
    }
    return;
}

#endif