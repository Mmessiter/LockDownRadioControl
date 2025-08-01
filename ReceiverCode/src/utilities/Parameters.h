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
#ifdef USE_STABILISATION
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
#ifndef USE_STABILISATION
        for (int i = 0; i < SERVOSUSED; ++i)
            ServoFrequency[i] = Parameters.word[i + 1];
        SetServoFrequency();
#endif
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
        UseRateLPF = (bool)(Parameters.word[4] & 0x01);
        Look1("StabilisationOn: ");
        Look(StabilisationOn ? "Yes" : "No");
        Look1("SelfLevellingOn: ");
        Look(SelfLevellingOn ? "Yes" : "No");
        Look1("UseKalmanFilter: ");
        Look(UseKalmanFilter ? "Yes" : "No");
        Look1("UseRateLPF: ");
        Look(UseRateLPF ? "Yes" : "No");
        break;
    case RECALIBRATE_MPU6050:                                         // 10
        if ((Parameters.word[1] == 42) && (Parameters.word[2] == 42)) // these two just to confirm
            PerformMPU6050Calibration();
        break;
    case TAIL_PID_VALUES: // 11 - Tail PID values for helicopters, etc.
        TAIL_PID_P = DecodeFloat(Parameters.word[0], Parameters.word[1], Parameters.word[2], Parameters.word[3]);
        TAIL_PID_I = DecodeFloat(Parameters.word[4], Parameters.word[5], Parameters.word[6], Parameters.word[7]);
        TAIL_PID_D = DecodeFloat(Parameters.word[8], Parameters.word[9], Parameters.word[10], Parameters.word[11]);
        ShowValues("Tail PID P", TAIL_PID_P);
        ShowValues("Tail PID I", TAIL_PID_I);
        ShowValues("Tail PID D", TAIL_PID_D);
        break;

#endif // ifdef USE_STABILISATION
        break;
    default:
        break;
    }
    return;
}

#endif