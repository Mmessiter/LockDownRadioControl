/** @file ReceiverCode/src/utilities/Parameters.h */
// Malcolm Messiter 2020 - 2025
#ifndef _SRC_PARAMETERS_RADIO_H
#define _SRC_PARAMETERS_RADIO_H
#include "utilities/1Definitions.h"

// ************************************************************************/
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
    // Look(ParaNames[Parameters.ID-1]); // Look at the ID of the parameters packet
    // for (int i = 1; i < 12; ++i) // Read the parameters
    // Look(Parameters.word[i]);

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
    case PID_VALUES: // 4
        PID_P = DecodeFloat(Parameters.word[0], Parameters.word[1], Parameters.word[2], Parameters.word[3]);
        PID_I = DecodeFloat(Parameters.word[4], Parameters.word[5], Parameters.word[6], Parameters.word[7]);
        PID_D = DecodeFloat(Parameters.word[8], Parameters.word[9], Parameters.word[10], Parameters.word[11]);
        ShowValues("PID P", PID_P);
        ShowValues("PID I", PID_I);
        ShowValues("PID D", PID_D);
        break;
    case KALMAN_VALUES: // 5
        Kalman_Q_angle = DecodeFloat(Parameters.word[0], Parameters.word[1], Parameters.word[2], Parameters.word[3]);
        Kalman_Q_bias = DecodeFloat(Parameters.word[4], Parameters.word[5], Parameters.word[6], Parameters.word[7]);
        Kalman_R_measure = DecodeFloat(Parameters.word[8], Parameters.word[9], Parameters.word[10], Parameters.word[11]);
        ShowValues("Kalman Q_angle", Kalman_Q_angle);
        ShowValues("Kalman Q_bias", Kalman_Q_bias);
        ShowValues("Kalman R_measure", Kalman_R_measure);
        break;
    case SERVO_FREQUENCIES: // 6
        for (int i = 0; i < SERVOSUSED; ++i)
            ServoFrequency[i] = Parameters.word[i + 1];
        SetServoFrequency();
        break;
    case SERVO_PULSE_WIDTHS: // 7
        for (int i = 0; i < SERVOSUSED; ++i)
            ServoCentrePulse[i] = Parameters.word[i + 1];
        break;
    case ALPHA_BETA: // 8
        alpha = DecodeFloat(Parameters.word[0], Parameters.word[1], Parameters.word[2], Parameters.word[3]);
        beta = DecodeFloat(Parameters.word[4], Parameters.word[5], Parameters.word[6], Parameters.word[7]);
        ShowValues("Alpha", alpha);
        ShowValues("Beta", beta);
        break;
    case BOOLEANS: // 9
        StabilisationOn = (bool)(Parameters.word[1] & 0x01);
        SelfLevellingOn = (bool)(Parameters.word[2] & 0x01);
        UseKalmanFilter = (bool)(Parameters.word[3] & 0x01);
        UseRateLFP = (bool)(Parameters.word[4] & 0x01);
        Look1("StabilisationOn: ");
        Look(StabilisationOn);
        Look1("SelfLevellingOn: ");
        Look(SelfLevellingOn);
        Look1("UseKalmanFilter: ");
        Look(UseKalmanFilter);
        Look1("UseRateLFP: ");
        Look(UseRateLFP);

        break;
    default:
        break;
    }
    return;
}

#endif