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
    static uint8_t BankRequested = 255;
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
        if (!Rotorflight_Version)
            break;
        if (Parameters.word[1] == 321) // 321 is the command to send PIDs NOW!
        {
            SendRotorFlightParametresNow = SEND_PID_RF;
            Started_Sending_PIDs = millis();
            PID_Send_Duration = Parameters.word[2];
        }
        break;
    case GET_FIRST_6_PID_VALUES: // 10
        if (!Rotorflight_Version)
            break;
        for (int i = 0; i < 6; ++i)
        {
            All_PIDs[i] = Parameters.word[i + 1];
        }
        break;

    case GET_SECOND_11_PID_VALUES: // 11
        if (!Rotorflight_Version)
            break;
        for (int i = 0; i < 11; ++i)
        {
            All_PIDs[i + 6] = Parameters.word[i + 1]; // 6 to 16
        }
        WritePIDsToNexusAndSave(All_PIDs);
        break;

    case SEND_RATES_VALUES: // 12
        if (!Rotorflight_Version)
            break;
        if (Parameters.word[1] == 321) // 321 is the command to send RATES NOW!
        {
            SendRotorFlightParametresNow = SEND_RATES_RF; // send rates now =2
            Started_Sending_RATEs = millis();
            RATES_Send_Duration = Parameters.word[2];
        }
        break;
    case GET_FIRST_7_RATES_VALUES: // 13
        if (!Rotorflight_Version)
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
        if (!Rotorflight_Version)
            break;
        Yaw_Centre_Rate = Parameters.word[1];
        Yaw_Max_Rate = Parameters.word[2];
        Yaw_Expo = Parameters.word[3];
        Collective_Centre_Rate = Parameters.word[4];
        Collective_Max_Rate = Parameters.word[5];
        Collective_Expo = Parameters.word[6];
        if (!Parameters.word[7])        // if this flag was set we must await the advanced rates values before writing to Nexus. If this flag isn't set, we can write to Nexus now.
            WriteRatesToNexusAndSave(); // That's because they are all written together.
        break;
    case SEND_RATES_ADVANCED_VALUES: // 15
        if (!Rotorflight_Version)
            break;
        if (Parameters.word[1] == 321) // 321 is the command to send RATES NOW!
        {
            SendRotorFlightParametresNow = SEND_RATES_ADVANCED_RF; // send rates now =2
            Started_Sending_RATES_ADVANCED = millis();
            RATES_ADVANCED_Send_Duration = Parameters.word[2];
        }
        break;
    case GET_RATES_ADVANCED_VALUES_SECOND_8:
        if (!Rotorflight_Version)
            break;
        Collective_Setpoint_Boost_Gain = Parameters.word[1];
        Roll_Setpoint_Boost_Cutoff = Parameters.word[2];
        Pitch_Setpoint_Boost_Cutoff = Parameters.word[3];
        Yaw_Setpoint_Boost_Cutoff = Parameters.word[4];
        Collective_Setpoint_Boost_Cutoff = Parameters.word[5];
        Yaw_Dynamic_Ceiling_Gain = Parameters.word[6];
        Yaw_Dynamic_Deadband_Gain = Parameters.word[7];
        Yaw_Dynamic_Deadband_Filter = Parameters.word[8];
        WriteRatesToNexusAndSave(); // That's because they are all written together.
                                    //  Look("ALL rates updated from transmitter");
        break;
    case GET_RATES_ADVANCED_VALUES_FIRST_7:
        if (!Rotorflight_Version)
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
        if (!Rotorflight_Version)
            break;
        if (Parameters.word[1] == 321) // 321 is the command to send RATES NOW!
        {
            SendRotorFlightParametresNow = SEND_PID_ADVANCED_RF;
            Started_Sending_PID_ADVANCED = millis();
            PID_ADVANCED_Send_Duration = Parameters.word[2];
        }
        break;
    case GET_FIRST_9_ADVANCED_PID_VALUES: // 19
        if (!Rotorflight_Version)
            break;
        for (int i = 0; i < 9; i++)
            PID_Advanced_Bytes[i] = Parameters.word[i + 1];
        break;
    case GET_SECOND_9_ADVANCED_PID_VALUES: // 20
        if (!Rotorflight_Version)
            break;
        for (int i = 0; i < 9; i++)
            PID_Advanced_Bytes[i + 9] = Parameters.word[i + 1];
        break;
    case GET_THIRD_8_ADVANCED_PID_VALUES: // 21
        if (!Rotorflight_Version)
            break;
        for (int i = 0; i < 8; i++)
            PID_Advanced_Bytes[i + 18] = Parameters.word[i + 1];
        WritePIDAdvancedToNexusAndSave();
        break;
    case MSP_BANK_CHANGE: // 22....  drop though is intended here

    case MSP_RATES_CHANGE: // 23
        if (BankRequested != (Parameters.word[2]))
        {
            BankRequested = Parameters.word[2]; // the hi bit is set by the transmitter to indicate this is a RATES bank change, not a PID bank change
            NewBank = Parameters.word[2];       // The HI BIT is set for RATES. else it would be PID profile
            SetNexusProfile(NewBank - 1);       // set high bit to indicate we're sending RATES bank change, not PID bank change
        }
        break;
    //  case MSP_BANK_CHANGE_CONFIRMATION: // 24
    //  Look1("Bank change confirmation received for bank ");
    //  Look(Parameters.word[2]);
    //  SendRotorFlightParametresNow = SEND_STATUS_EX;
    //  Started_Sending_STATUS_EX = millis();

    //  break;
    case MSP_INHIBIT_TELEMETRY: // 25
        InhibitTelemetry = true;
        break;
    case MSP_ENABLE_TELEMETRY: // 26
        InhibitTelemetry = false;
        break;
    case SEND_GOV_VALUES: // 27
        if (!Rotorflight_Version)
            break;
        if (Parameters.word[1] == 321) // 321 is the command to send GOVERNOR PROFILE NOW!
        {
            SendRotorFlightParametresNow = SEND_GOV_PROFILE_RF;
            Started_Sending_GOV_PROFILE = millis();
            GOV_Send_Duration = Parameters.word[2];
        }
        break;

    case SEND_GOV_CONFIG_VALUES: // 28
        if (!Rotorflight_Version)
            break;
        if (Parameters.word[1] == 321)
        {
            SendRotorFlightParametresNow = SEND_GOV_CONFIG_RF;
            Started_Sending_GOV_CONFIG = millis();
            GOV_Config_Send_Duration = Parameters.word[2];
        }
        break;

    case SEND_GOV_WRITE_PROFILE1: // 29 — profile bytes 1-11
        if (!Rotorflight_Version)
            break;
        GovWritePayload[1] = (uint8_t)Parameters.word[1];   // Headspeed lo
        GovWritePayload[2] = (uint8_t)Parameters.word[2];   // Headspeed hi
        GovWritePayload[3] = (uint8_t)Parameters.word[3];   // Gain
        GovWritePayload[4] = (uint8_t)Parameters.word[4];   // P
        GovWritePayload[5] = (uint8_t)Parameters.word[5];   // I
        GovWritePayload[6] = (uint8_t)Parameters.word[6];   // D
        GovWritePayload[7] = (uint8_t)Parameters.word[7];   // F
        GovWritePayload[8] = (uint8_t)Parameters.word[8];   // TTA gain
        GovWritePayload[9] = (uint8_t)Parameters.word[9];   // TTA limit
        GovWritePayload[10] = (uint8_t)Parameters.word[10]; // Max throttle
        GovWritePayload[11] = (uint8_t)Parameters.word[11]; // Min throttle
        break;

    case SEND_GOV_WRITE_PROFILE2: // 30 — profile bytes 12-17 + flags
        if (!Rotorflight_Version)
            break;
        GovWritePayload[12] = (uint8_t)Parameters.word[1]; // Fallback drop
        GovWritePayload[13] = (uint8_t)Parameters.word[2]; // Yaw weight
        GovWritePayload[14] = (uint8_t)Parameters.word[3]; // Cyclic weight
        GovWritePayload[15] = (uint8_t)Parameters.word[4]; // Collective weight
        GovWritePayload[16] = (uint8_t)Parameters.word[5]; // Flags lo
        GovWritePayload[17] = (uint8_t)Parameters.word[6]; // Flags hi
        // Profile complete — unpack and write to FC
        UnpackGovernorFromTxPayload(GovWritePayload);
        WriteGovernorProfileToNexusAndSave();
        break;

    case SEND_GOV_WRITE_CONFIG1: // 31 — config bytes 18-28
        if (!Rotorflight_Version)
            break;
        GovWritePayload[18] = (uint8_t)Parameters.word[1];  // Gov mode
        GovWritePayload[19] = (uint8_t)Parameters.word[2];  // Handover throttle
        GovWritePayload[20] = (uint8_t)Parameters.word[3];  // Startup lo
        GovWritePayload[21] = (uint8_t)Parameters.word[4];  // Startup hi
        GovWritePayload[22] = (uint8_t)Parameters.word[5];  // Spoolup lo
        GovWritePayload[23] = (uint8_t)Parameters.word[6];  // Spoolup hi
        GovWritePayload[24] = (uint8_t)Parameters.word[7];  // Spooldown lo
        GovWritePayload[25] = (uint8_t)Parameters.word[8];  // Spooldown hi
        GovWritePayload[26] = (uint8_t)Parameters.word[9];  // Tracking lo
        GovWritePayload[27] = (uint8_t)Parameters.word[10]; // Tracking hi
        GovWritePayload[28] = (uint8_t)Parameters.word[11]; // Recovery lo
        break;

    case SEND_GOV_WRITE_CONFIG2: // 32 — config bytes 29-39
        if (!Rotorflight_Version)
            break;
        GovWritePayload[29] = (uint8_t)Parameters.word[1];  // Recovery hi
        GovWritePayload[30] = (uint8_t)Parameters.word[2];  // Hold timeout lo
        GovWritePayload[31] = (uint8_t)Parameters.word[3];  // Hold timeout hi
        GovWritePayload[32] = (uint8_t)Parameters.word[4];  // Autorot timeout lo
        GovWritePayload[33] = (uint8_t)Parameters.word[5];  // Autorot timeout hi
        GovWritePayload[34] = (uint8_t)Parameters.word[6];  // RPM filter
        GovWritePayload[35] = (uint8_t)Parameters.word[7];  // Pwr filter
        GovWritePayload[36] = (uint8_t)Parameters.word[8];  // D filter
        GovWritePayload[37] = (uint8_t)Parameters.word[9];  // FF filter
        GovWritePayload[38] = (uint8_t)Parameters.word[10]; // TTA filter
        GovWritePayload[39] = (uint8_t)Parameters.word[11]; // Throttle type
        break;

    case SEND_GOV_WRITE_CONFIG3: // 33 — config bytes 40-45
        if (!Rotorflight_Version)
            break;
        GovWritePayload[40] = (uint8_t)Parameters.word[1]; // Idle throttle
        GovWritePayload[41] = (uint8_t)Parameters.word[2]; // Auto throttle
        GovWritePayload[42] = (uint8_t)Parameters.word[3]; // Volt comp flag
        GovWritePayload[43] = (uint8_t)Parameters.word[4]; // PID spoolup flag
        GovWritePayload[44] = (uint8_t)Parameters.word[5]; // Fallback precomp flag
        GovWritePayload[45] = (uint8_t)Parameters.word[6]; // Dyn min thr flag
        // Config complete — unpack and write to FC
        UnpackGovernorFromTxPayload(GovWritePayload);
        WriteGovernorConfigToNexusAndSave();
        break;

    default:
        break;
    }

}

#endif