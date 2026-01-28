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
        }
        break;
    case GET_FIRST_6_PID_VALUES: // 10
        if (!Rotorflight22Detected)
            break;
        for (int i = 0; i < 6; ++i)
        {
            All_PIDs[i] = Parameters.word[i + 1];
        }
        break;

    case GET_SECOND_11_PID_VALUES: // 11
        if (!Rotorflight22Detected)
            break;
        for (int i = 0; i < 11; ++i)
        {
            All_PIDs[i + 6] = Parameters.word[i + 1]; // 6 to 16
        }
        WritePIDsToNexusAndSave(All_PIDs);
        break;

    case SEND_RATES_VALUES: // 12
        if (!Rotorflight22Detected)
            break;
        if (Parameters.word[1] == 321) // 321 is the command to send RATES NOW!
        {
            SendRotorFlightParametresNow = SEND_RATES_RF; // send rates now =2
            Started_Sending_RATEs = millis();
            RATES_Send_Duration = Parameters.word[2];
        }
        break;
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
        WriteRatesToNexusAndSave(); // no checking here yet for validity. Maybe later if needed
        break;
    case SEND_RATES_ADVANCED_VALUES: // 15
        if (!Rotorflight22Detected)
            break;
        if (Parameters.word[1] == 321) // 321 is the command to send RATES NOW!
        {
            SendRotorFlightParametresNow = SEND_RATES_ADVANCED_RF; // send rates now =2
            Started_Sending_RATES_ADVANCED = millis();
            RATES_ADVANCED_Send_Duration = Parameters.word[2];
        }
        break;
    case GET_RATES_ADVANCED_VALUES_SECOND_8:
        if (!Rotorflight22Detected)
            break;
        Collective_Setpoint_Boost_Gain = Parameters.word[1];
        Roll_Setpoint_Boost_Cutoff = Parameters.word[2];
        Pitch_Setpoint_Boost_Cutoff = Parameters.word[3];
        Yaw_Setpoint_Boost_Cutoff = Parameters.word[4];
        Collective_Setpoint_Boost_Cutoff = Parameters.word[5];
        Yaw_Dynamic_Ceiling_Gain = Parameters.word[6];
        Yaw_Dynamic_Deadband_Gain = Parameters.word[7];
        Yaw_Dynamic_Deadband_Filter = Parameters.word[8];
        WriteRatesToNexusAndSave(); // no checking here yet for validity. Maybe later if needed
        break;
    case GET_RATES_ADVANCED_VALUES_FIRST_7:
        if (!Rotorflight22Detected)
            break;
        Roll_Response_Time = Parameters.word[1];
        Pitch_Response_Time = Parameters.word[2];
        Yaw_Response_Time = Parameters.word[3];
        Collective_Response_Time = Parameters.word[4];
        Roll_Setpoint_Boost_Gain = Parameters.word[5];
        Pitch_Setpoint_Boost_Gain = Parameters.word[6];
        Yaw_Setpoint_Boost_Gain = Parameters.word[7];
        break;
    case SEND_PID_ADVANCED_VALUES: // 18
        if (!Rotorflight22Detected)
            break;
        if (Parameters.word[1] == 321) // 321 is the command to send RATES NOW!
        {
            SendRotorFlightParametresNow = SEND_PID_ADVANCED_RF;
            Started_Sending_PID_ADVANCED = millis();
            PID_ADVANCED_Send_Duration = Parameters.word[2];
        }
        break;
    case GET_FIRST_9_ADVANCED_PID_VALUES: // 19
        if (!Rotorflight22Detected)
            break;
        for (int i = 0; i < 9; i++)
            PID_Advanced_Bytes[i] = Parameters.word[i + 1];
        break;
    case GET_SECOND_9_ADVANCED_PID_VALUES: // 20
        if (!Rotorflight22Detected)
            break;
        for (int i = 0; i < 9; i++)
            PID_Advanced_Bytes[i + 9] = Parameters.word[i + 1];
        break;
    case GET_THIRD_8_ADVANCED_PID_VALUES: // 21
        if (!Rotorflight22Detected)
            break;
        for (int i = 0; i < 8; i++)
            PID_Advanced_Bytes[i + 18] = Parameters.word[i + 1];
        WritePIDAdvancedToNexusAndSave();
        break;
    default:
        break;
    }
}

#endif