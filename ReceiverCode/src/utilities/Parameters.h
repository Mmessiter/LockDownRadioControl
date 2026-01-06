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
float DecodeAFloat(uint16_t word1, uint16_t word2, uint16_t word3, uint16_t word4)
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
//************************************************************************************************************/
bool pidsLookValid(const uint16_t p[12])
{
    for (int i = 0; i < 12; i++)
        if ((p[i] > 750) || (p[i] < 1))
            return false; // absurd for RF PID scales
    return true;
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
        for (int i = 0; i < Servos_Used; ++i)
            ServoFrequency[i] = Parameters.word[i + 1];
        SetServoFrequency();
        break;
    case SERVO_PULSE_WIDTHS: // 7
        for (int i = 0; i < Servos_Used; ++i)
            ServoCentrePulse[i] = Parameters.word[i + 1];
        break;
#endif

    case GEAR_RATIO: // 8
        Ratio = DecodeAFloat(Parameters.word[1], Parameters.word[2], Parameters.word[3], Parameters.word[4]);
        break;

    case SEND_PID_VALUES: // 9
        if (!Rotorflight22Detected)
            break;
        if (Parameters.word[1] == 321) // 321 is the command to send PIDs NOW!
        {
            SendRotorFlightParametresNow = SEND_PID_RF;
            Started_Sending_PIDs = millis();
            PID_Send_Duration = Parameters.word[2];
            // Look("Request to send PID values received");
        }
        break;

    case GET_FIRST_6_PID_VALUES: // 10
        if (!Rotorflight22Detected)
            break;

        All_PIDs[0] = Parameters.word[1];
        All_PIDs[1] = Parameters.word[2];
        All_PIDs[2] = Parameters.word[3];
        All_PIDs[3] = Parameters.word[4];
        All_PIDs[4] = Parameters.word[5];
        All_PIDs[5] = Parameters.word[6];
        break;

    case GET_SECOND_6_PID_VALUES: // 11
        if (!Rotorflight22Detected)
            break;
        All_PIDs[6] = Parameters.word[1];
        All_PIDs[7] = Parameters.word[2];
        All_PIDs[8] = Parameters.word[3];
        All_PIDs[9] = Parameters.word[4];
        All_PIDs[10] = Parameters.word[5];
        All_PIDs[11] = Parameters.word[6];
      //  if (pidsLookValid(All_PIDs)) // LATER!
      //  {
            WritePIDsToNexusAndSave(All_PIDs);
      //  }
        break;

    case SEND_RATES_VALUES: // 12
        if (!Rotorflight22Detected)
            break;
        if (Parameters.word[1] == 321) // 321 is the command to send PIDs NOW!
        {
            SendRotorFlightParametresNow = SEND_RATES_RF; // send rates now =2
            Started_Sending_RATEs = millis();
            RATES_Send_Duration = Parameters.word[2];
            // Look("Request to send RATES values received");
        }
    case GET_FIRST_7_RATES_VALUES: // 13
        if (!Rotorflight22Detected)
            break;
        Rates_Type = Parameters.word[1];
        Roll_Centre_Rate = Parameters.word[2];
        Roll_Max_Rate = Parameters.word[3];
        Roll_Expo = Parameters.word[4];
        Pitch_Centre_Rate = Parameters.word[5];
        Pitch_Max_Rate = Parameters.word[6];
        Pitch_Expo = Parameters.word[7];
        break;

    case GET_SECOND_6_RATES_VALUES: // 14
        if (!Rotorflight22Detected)
            break;
        Yaw_Centre_Rate = Parameters.word[1];
        Yaw_Max_Rate = Parameters.word[2];
        Yaw_Expo = Parameters.word[3];
        Collective_Centre_Rate = Parameters.word[4];
        Collective_Max_Rate = Parameters.word[5];
        Collective_Expo = Parameters.word[6];
        WriteRatesToNexusAndSave();
        break;

    default:
        break;
    }
}

#endif