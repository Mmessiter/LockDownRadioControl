/** @file ReceiverCode/src/utilities/Nexus.h */
// Malcolm Messiter 2020 - 2025 (with help from ChatGPT :-)
#ifndef NEXUS_H
#define NEXUS_H
#include "utilities/1Definitions.h"

// ************************************************************************************************************
// MSP SUPPORT FUNCTIONS FOR NEXUS etc.
// ************************************************************************************************************
#define MSP_MOTOR_TELEMETRY 139 // Motor telemetry data
#define MSP_PID 112             // PID settings

// ************************************************************************************************************
inline void DetectNexusAtBoot()
{
#define NEXUS_DETECT_WINDOW_MS 1500 // 1.5 seconds time window to detect Nexus at boot

    MSP_UART.begin(115200);
    uint32_t start = millis();
    uint8_t buf[80];

    while (millis() - start < NEXUS_DETECT_WINDOW_MS)
    {
        RequestFromMSP(MSP_MOTOR_TELEMETRY);
        delay(20); // give it a moment to reply
        uint8_t p = 0;
        while (MSP_UART.available() && p < sizeof(buf))
        {
            buf[p++] = MSP_UART.read();
        }
        if (Parse_MSP_Motor_Telemetry(&buf[0], p))
        {
            NexusPresent = true;
            return;
        }
    }
    MSP_UART.end();
    NexusPresent = false;
}
// ************************************************************************************************************
// @brief Structure representing an MSP frame

struct MspFrame
{
    uint8_t size;
    uint8_t cmd;
    const uint8_t *payload; // points into your input buffer
};
// ************************************************************************************************************
// Finds the *first* valid MSPv1 response frame ($M>) in data[].
// If found, fills out 'out' and returns true.
inline bool FindMspV1ResponseFrame(const uint8_t *data, uint8_t n, MspFrame &out)
{
    for (uint8_t i = 0; i + 6 < n; ++i)
    {
        if (data[i] != '$' || data[i + 1] != 'M' || data[i + 2] != '>')
            continue;

        uint8_t size = data[i + 3];
        uint8_t cmd = data[i + 4];

        uint16_t frame_end = i + 5 + size; // index of checksum byte
        if (frame_end >= n)
            return false; // header found but frame not complete in buffer

        uint8_t ck = size ^ cmd;
        for (uint8_t k = 0; k < size; ++k)
            ck ^= data[i + 5 + k];

        if (ck != data[frame_end])
            continue; // bad checksum, keep scanning

        out.size = size;
        out.cmd = cmd;
        out.payload = &data[i + 5];
        return true;
    }
    return false;
}

// ************************************************************************************************************
inline void RequestFromMSP(uint8_t command) // send a request to the flight controller
{
    uint8_t checksum = 0;
    MSP_UART.write('$');
    MSP_UART.write('M');
    MSP_UART.write('<');
    MSP_UART.write(0);
    checksum ^= 0;
    MSP_UART.write(command);
    checksum ^= command;
    MSP_UART.write(checksum);
}
// ************************************************************************************************************
void DebugPIDValues(char const *msg){

    Look(msg);
    Look1(" Roll P: ");
    Look(PID_Roll_P);
    Look1(" Roll I: ");
    Look(PID_Roll_I);
    Look1(" Roll D: ");
    Look(PID_Roll_D);
    Look1(" Roll FF: ");
    Look(PID_Roll_FF);

    Look1(" Pitch P: ");
    Look(PID_Pitch_P);
    Look1(" Pitch I: ");
    Look(PID_Pitch_I);
    Look1(" Pitch D: ");
    Look(PID_Pitch_D);
    Look1(" Pitch FF: ");
    Look(PID_Pitch_FF);

    Look1(" Yaw P: ");
    Look(PID_Yaw_P);
    Look1(" Yaw I: ");
    Look(PID_Yaw_I);
    Look1(" Yaw D: ");
    Look(PID_Yaw_D);
    Look1(" Yaw FF: ");
    Look(PID_Yaw_FF);

    Look("---------------------");
}
// ************************************************************************************************************
//         MSP_PID FROM ROTORFLIGHT FIRMWARE

inline bool Parse_MSP_PID(const uint8_t *data, uint8_t n)
{
    MspFrame f;
    if (!FindMspV1ResponseFrame(data, n, f))
        return false;
    if (f.cmd != MSP_PID)
        return false;
    if (f.size < 9)
        return false;
    const uint8_t *p = f.payload;

    PID_Roll_P = p[0] | (p[1] << 8);
    PID_Roll_I = p[2] | (p[3] << 8);
    PID_Roll_D = p[4] | (p[5] << 8);
    PID_Roll_FF = p[6] | (p[7] << 8);
    PID_Pitch_P = p[8] | (p[9] << 8);
    PID_Pitch_I = p[10] | (p[11] << 8);
    PID_Pitch_D = p[12] | (p[13] << 8);
    PID_Pitch_FF = p[14] | (p[15] << 8);
    PID_Yaw_P = p[16] | (p[17] << 8);
    PID_Yaw_I = p[18] | (p[19] << 8);
    PID_Yaw_D = p[20] | (p[21] << 8);
    PID_Yaw_FF = p[22] | (p[23] << 8);
    DebugPIDValues("Current Nexus PID Values");
    
    return true;
}
// ************************************************************************************************************

//         MSP_MOTOR_TELEMETRY FROM ROTORFLIGHT FIRMWARE
//         uint32_t motorRpm = 0;                        // offset 0
//         uint16_t errorRatio = 0;                      // offset 5
//         uint16_t escVoltage = 0;      // 1mV per unit // offset 7
//         uint16_t escCurrent = 0;      // 1mA per unit // offset 9
//         uint16_t escConsumption = 0;  // mAh          // offset 11
//         uint16_t escTemperature = 0;  // 0.1C         // offset 13
//         uint16_t escTemperature2 = 0; // 0.1C         // offset 15

inline bool Parse_MSP_Motor_Telemetry(const uint8_t *data, uint8_t n)
{
    if (!Ratio)
        Ratio = 10.3f;

    MspFrame f;
    if (!FindMspV1ResponseFrame(data, n, f))
        return false;

    if (f.cmd != MSP_MOTOR_TELEMETRY)
        return false;

    const uint8_t size = f.size;
    const uint8_t *payload = f.payload;

    // Need enough bytes for what we read:
    // index 14 used for temp1 => size must be >= 15
    if (size < 15)
        return false;

    // motor index check
    if (payload[0] != 1)
        return false;

    RotorRPM = ((uint32_t)payload[1] |
                ((uint32_t)payload[2] << 8) |
                ((uint32_t)payload[3] << 16) |
                ((uint32_t)payload[4] << 24)) /
               Ratio; // RPM: decode 24-bit, and include the 4th byte even if zero
    RXModelVolts = (float)((uint16_t)payload[7] | ((uint16_t)payload[8] << 8)) / 1000.0f;
    Battery_Amps = (float)((uint16_t)payload[9] | ((uint16_t)payload[10] << 8)) / 1000.0f;
    Battery_mAh = (float)((uint16_t)payload[11] | ((uint16_t)payload[12] << 8));
    ESC_Temp_C = (float)((uint16_t)payload[13] | ((uint16_t)payload[14] << 8)) / 10.0f;

    if (RXModelVolts > 26.0f) // this is to handle 12S which were read with INA219 which cannot read above 26V ...
        RXModelVolts /= 2.0f; // ... so transmitter still halves the voltage for 12S for backwards compatibility.

    return true;
}

// ************************************************************************************************************/
inline void CheckMSPSerial()
{
    static uint32_t Localtimer = 0;

    if (millis() - Localtimer < 250) // 4 x per second
        return;
    Localtimer = millis();
    uint8_t data_in[80];
    uint8_t p = 0;
    while (MSP_UART.available() && p < sizeof(data_in))
    {
        data_in[p++] = MSP_UART.read();
    }

    if ((millis() - Started_Sending_PIDs) > PID_Send_Duration)
    {
        SendPIDsNow = false;
    }

    if (!SendPIDsNow)
    {
        Parse_MSP_Motor_Telemetry(&data_in[0], p);
        RequestFromMSP(MSP_MOTOR_TELEMETRY);
    }
    else
    {
        Parse_MSP_PID(data_in, p);
        RequestFromMSP(MSP_PID);
    }
}

// ************************************************************************************************************
#endif // NEXUS_H