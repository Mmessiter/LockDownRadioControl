/** @file ReceiverCode/src/utilities/Nexus.h */
// Malcolm Messiter 2020 - 2025
#ifndef NEXUS_H
#define NEXUS_H
#include "utilities/1Definitions.h"

// ************************************************************************************************************
// MSP SUPPORT FUNCTIONS FOR ROTORFLIGHT (Nexus) etc.
// ************************************************************************************************************
#define MSP_MOTOR_TELEMETRY 139 // Motor telemetry data
#define MSP_PID 112             // PID settings (read)
#define MSP_SET_PID 202         // write PID settings
#define MSP_EEPROM_WRITE 250    // save settings to EEPROM/flash
#define MSP_RC_TUNING 111       // RC Tuning settings (RATES etc)
#define MSP_SET_RC_TUNING 204   // write RC Tuning settings (RATES etc)
// ************************************************************************************************************
enum : uint8_t
{
    ROLL_P = 0,
    ROLL_I,
    ROLL_D,
    ROLL_FF,
    PITCH_P,
    PITCH_I,
    PITCH_D,
    PITCH_FF,
    YAW_P,
    YAW_I,
    YAW_D,
    YAW_FF
};
// ************************************************************************************************************
#define MSP_API_VERSION 1

static bool
Parse_MSP_API_VERSION(const uint8_t *buf, uint8_t len, uint8_t &mspProto, uint8_t &apiMaj, uint8_t &apiMin)
{
    for (uint8_t i = 0; i + 6 <= len; i++)
    {
        if (buf[i] != '$' || buf[i + 1] != 'M' || buf[i + 2] != '>')
            continue;

        const uint8_t size = buf[i + 3];
        const uint8_t cmd = buf[i + 4];

        if (cmd != MSP_API_VERSION)
            continue;
        if (size < 3)
            continue;
        if (i + 6 + size > len)
            continue;

        uint8_t cs = 0;
        cs ^= size;
        cs ^= cmd;
        for (uint8_t k = 0; k < size; k++)
            cs ^= buf[i + 5 + k];

        if (cs != buf[i + 5 + size])
            continue;

        mspProto = buf[i + 5 + 0];
        apiMaj = buf[i + 5 + 1];
        apiMin = buf[i + 5 + 2];
        api100 = (uint16_t)apiMaj * 100u + (uint16_t)apiMin; // 12.08 -> 1208
        return true;
    }
    return false;
}
// ************************************************************************************************************
// Detect if Rotorflight 2.2 is present at boot time
// If detected, fetch API version and protocol version
// Keeps the MSP_UART open if Rotorflight 2.2 is present
// Closes it if not present
// ************************************************************************************************************
inline void DetectRotorFlightAtBoot()
{
#define NEXUS_DETECT_WINDOW_MS 1500

    MSP_UART.begin(115200);
    while (MSP_UART.available())
        (void)MSP_UART.read();

    uint32_t start = millis();
    uint8_t buf[80];

    Rotorflight22Detected = false;

    while (millis() - start < NEXUS_DETECT_WINDOW_MS)
    {
        RequestFromMSP(MSP_MOTOR_TELEMETRY);
        delay(20);

        uint8_t p = 0;
        while (MSP_UART.available() && p < sizeof(buf))
            buf[p++] = (uint8_t)MSP_UART.read();
        if (Parse_MSP_Motor_Telemetry(buf, p))
        {
            // Now fetch API version
            RequestFromMSP(MSP_API_VERSION);
            delay(20);

            p = 0;
            while (MSP_UART.available() && p < sizeof(buf))
                buf[p++] = (uint8_t)MSP_UART.read();

            uint8_t apiMaj = 0, apiMin = 0, proto = 0;
            if (Parse_MSP_API_VERSION(buf, p, proto, apiMaj, apiMin))
            {
                if (api100 >= 1208) // (x 100 to deal with floats more easily)
                    Rotorflight22Detected = true;
            }
            return; // keep UART open
        }
    }
    MSP_UART.end();
    Rotorflight22Detected = false;
}
// **********************************************************************************************************// @brief Structure representing an MSP frame

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
    MSP_UART.write((uint8_t)0);
    checksum ^= 0;
    MSP_UART.write(command);
    checksum ^= command;
    MSP_UART.write(checksum);
}

// ************************************************************************************************************
// Send an MSPv1 COMMAND with a payload (this is what we need for SET_PID + EEPROM_WRITE) ***
// This transmits: $ M <  [size] [cmd] [payload bytes...] [checksum]
// - This only "sends". It does NOT wait for a reply.
// - Many flight controllers will ignore writes if ARMED. So do this only when disarmed.
//
inline void SendToMSP(uint8_t command, const uint8_t *payload, uint8_t payloadSize)
{
    uint8_t checksum = 0;

    MSP_UART.write('$');
    MSP_UART.write('M');
    MSP_UART.write('<');

    MSP_UART.write(payloadSize);
    checksum ^= payloadSize;

    MSP_UART.write(command);
    checksum ^= command;

    for (uint8_t i = 0; i < payloadSize; i++)
    {
        uint8_t b = payload[i];
        MSP_UART.write(b);
        checksum ^= b;
    }

    MSP_UART.write(checksum);
}

// ************************************************************************************************************
// Write the given array of 12 PID values to Nexus via MSP, and then save to EEPROM/flash.
inline void WritePIDsToNexusAndSave(const uint16_t pid[12])
{
    static uint32_t lastWriteTime = 0;
    const uint32_t WRITE_COOLDOWN_MS = 5000; // 5 seconds
    uint32_t now = millis();

    if ((now - lastWriteTime < WRITE_COOLDOWN_MS) || (!Rotorflight22Detected))
        return;
    uint8_t payload[24];
    for (uint8_t i = 0; i < 12; i++)
    {
        payload[i * 2 + 0] = (uint8_t)(pid[i] & 0xFF);
        payload[i * 2 + 1] = (uint8_t)(pid[i] >> 8);
    }

    SendToMSP(MSP_SET_PID, payload, sizeof(payload));
    delay(50);

    SendToMSP(MSP_EEPROM_WRITE, nullptr, 0);
    delay(50);

    lastWriteTime = now; // set cooldown only after we actually did the write+save
}
// ************************************************************************************************************
void DebugPIDValues(char const *msg)
{
    Look(msg);
    for (int i = 0; i < 12; ++i)
    {
        Look1(" PID[");
        Look1(i);
        Look1("]: ");
        Look(All_PIDs[i]);
    }
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

    // *** FIX: you read 24 bytes below, so size must be at least 24 ***
    if (f.size < 24)
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

    //  DebugPIDValues("Current Nexus PID Values");

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
        // Parse_MSP_Motor_Telemetry(&data_in[0], p);
        // RequestFromMSP(MSP_MOTOR_TELEMETRY);
        Parse_MSP_RC_TUNING(data_in, p);
        RequestFromMSP(MSP_RC_TUNING);
    }
    else
    {
        Parse_MSP_PID(data_in, p);
        RequestFromMSP(MSP_PID);
    }
}

//// ************************************************************************************************************
//// ************************************************************************************************************
//// ************************************************************************************************************
//// *************************************           ************************************************************
//// *************************************   RATES   ************************************************************
//// *************************************           ************************************************************
//// ************************************************************************************************************
//// ************************************************************************************************************
//// ************************************************************************************************************

// Rates globals - add these as global variables
uint8_t Rates_Type;
float Roll_RC_Rate, Roll_RC_Expo, Roll_Rate;
float Pitch_RC_Rate, Pitch_RC_Expo, Pitch_Rate;
float Yaw_RC_Rate, Yaw_RC_Expo, Yaw_Rate;
float Collective_RC_Rate, Collective_RC_Expo, Collective_Rate;
uint8_t Roll_Response_Time, Pitch_Response_Time, Yaw_Response_Time, Collective_Response_Time;
uint16_t Roll_Accel_Limit, Pitch_Accel_Limit, Yaw_Accel_Limit, Collective_Accel_Limit;
uint8_t Roll_Setpoint_Boost_Gain, Roll_Setpoint_Boost_Cutoff, Pitch_Setpoint_Boost_Gain, Pitch_Setpoint_Boost_Cutoff;
uint8_t Yaw_Setpoint_Boost_Gain, Yaw_Setpoint_Boost_Cutoff, Collective_Setpoint_Boost_Gain, Collective_Setpoint_Boost_Cutoff;
uint8_t Yaw_Dynamic_Ceiling_Gain, Yaw_Dynamic_Deadband_Gain, Yaw_Dynamic_Deadband_Filter;

// Roll_RC_Rate, Pitch_RC_Rate, Yaw_RC_Rate, Collective_RC_Rate are center sensitivity (*10 scaling)
// Roll_Rate, Pitch_Rate, Yaw_Rate, Collective_Rate are max rate (*10 scaling, or for collective perhaps *1 if units differ)

// Function prototypes if needed, but since inline, place after dependencies
// Ensure MspFrame, FindMspV1ResponseFrame, SendToMSP are declared before these

inline bool Parse_MSP_RC_TUNING(const uint8_t *data, uint8_t n)
{
    MspFrame f;
    if (!FindMspV1ResponseFrame(data, n, f))
        return false;
    if (f.cmd != MSP_RC_TUNING)
        return false;
    uint8_t min_size = 25;
    if (api100 >= 1208)
        min_size += 11;
    if (f.size < min_size)
        return false;
    const uint8_t *p = f.payload;
    uint8_t offset = 0;
    Rates_Type = p[offset++];
    Roll_RC_Rate = (float)p[offset++] * 10.0f;
    Roll_RC_Expo = (float)p[offset++] / 100.0f;
    Roll_Rate = (float)p[offset++] * 10.0f;
    Roll_Response_Time = p[offset++];
    Roll_Accel_Limit = p[offset] | (p[offset + 1] << 8);
    offset += 2;
    Pitch_RC_Rate = (float)p[offset++] * 10.0f;
    Pitch_RC_Expo = (float)p[offset++] / 100.0f;
    Pitch_Rate = (float)p[offset++] * 10.0f;
    Pitch_Response_Time = p[offset++];
    Pitch_Accel_Limit = p[offset] | (p[offset + 1] << 8);
    offset += 2;
    Yaw_RC_Rate = (float)p[offset++] * 10.0f;
    Yaw_RC_Expo = (float)p[offset++] / 100.0f;
    Yaw_Rate = (float)p[offset++] * 10.0f;
    Yaw_Response_Time = p[offset++];
    Yaw_Accel_Limit = p[offset] | (p[offset + 1] << 8);
    offset += 2;
    Collective_RC_Rate = (float)p[offset++] * 10.0f;
    Collective_RC_Expo = (float)p[offset++] / 100.0f;
    Collective_Rate = (float)p[offset++] * 10.0f; // If matches 48, fine; if needs 12, change to *1.0f
    Collective_Response_Time = p[offset++];
    Collective_Accel_Limit = p[offset] | (p[offset + 1] << 8);
    offset += 2;
    if (api100 >= 1208)
    {
        Roll_Setpoint_Boost_Gain = p[offset++];
        Roll_Setpoint_Boost_Cutoff = p[offset++];
        Pitch_Setpoint_Boost_Gain = p[offset++];
        Pitch_Setpoint_Boost_Cutoff = p[offset++];
        Yaw_Setpoint_Boost_Gain = p[offset++];
        Yaw_Setpoint_Boost_Cutoff = p[offset++];
        Collective_Setpoint_Boost_Gain = p[offset++];
        Collective_Setpoint_Boost_Cutoff = p[offset++];
        Yaw_Dynamic_Ceiling_Gain = p[offset++];
        Yaw_Dynamic_Deadband_Gain = p[offset++];
        Yaw_Dynamic_Deadband_Filter = p[offset++];
    }

    Look("---- Nexus RC Tuning Values ----");
    Look1("Rates Type: ");
    Look(Rates_Type);
    Look1("Roll RC Rate: ");
    Look(Roll_RC_Rate);
    Look1("Roll RC Expo: ");
    Look(Roll_RC_Expo);
    Look1("Roll Rate: ");
    Look(Roll_Rate);
    Look1("Roll Response Time: ");
    Look(Roll_Response_Time);
    Look1("Roll Accel Limit: ");
    Look(Roll_Accel_Limit);
    Look1("Pitch RC Rate: ");
    Look(Pitch_RC_Rate);
    Look1("Pitch RC Expo: ");
    Look(Pitch_RC_Expo);
    Look1("Pitch Rate: ");
    Look(Pitch_Rate);
    Look1("Pitch Response Time: ");
    Look(Pitch_Response_Time);
    Look1("Pitch Accel Limit: ");
    Look(Pitch_Accel_Limit);
    Look1("Yaw RC Rate: ");
    Look(Yaw_RC_Rate);
    Look1("Yaw RC Expo: ");
    Look(Yaw_RC_Expo);
    Look1("Yaw Rate: ");
    Look(Yaw_Rate);
    Look1("Yaw Response Time: ");
    Look(Yaw_Response_Time);
    Look1("Yaw Accel Limit: ");
    Look(Yaw_Accel_Limit);
    Look1("Collective RC Rate: ");
    Look(Collective_RC_Rate);
    Look1("Collective RC Expo: ");
    Look(Collective_RC_Expo);
    Look1("Collective Rate: ");
    Look(Collective_Rate);
    Look1("Collective Response Time: ");
    Look(Collective_Response_Time);
    Look1("Collective Accel Limit: ");
    Look(Collective_Accel_Limit);

    if (api100 >= 1208)
    {
        Look1("Roll Setpoint Boost Gain: ");
        Look(Roll_Setpoint_Boost_Gain);
        Look1("Roll Setpoint Boost Cutoff: ");
        Look(Roll_Setpoint_Boost_Cutoff);
        Look1("Pitch Setpoint Boost Gain: ");
        Look(Pitch_Setpoint_Boost_Gain);
        Look1("Pitch Setpoint Boost Cutoff: ");
        Look(Pitch_Setpoint_Boost_Cutoff);
        Look1("Yaw Setpoint Boost Gain: ");
        Look(Yaw_Setpoint_Boost_Gain);
        Look1("Yaw Setpoint Boost Cutoff: ");
        Look(Yaw_Setpoint_Boost_Cutoff);
        Look1("Collective Setpoint Boost Gain: ");
        Look(Collective_Setpoint_Boost_Gain);
        Look1("Collective Setpoint Boost Cutoff: ");
        Look(Collective_Setpoint_Boost_Cutoff);
        Look1("Yaw Dynamic Ceiling Gain: ");
        Look(Yaw_Dynamic_Ceiling_Gain);
        Look1("Yaw Dynamic Deadband Gain: ");
        Look(Yaw_Dynamic_Deadband_Gain);
        Look1("Yaw Dynamic Deadband Filter: ");
        Look(Yaw_Dynamic_Deadband_Filter);
    }

    return true;
}
// ************************************************************************************************************

inline void WriteRatesToNexusAndSave()
{
    static uint32_t lastWriteTime = 0;
    const uint32_t WRITE_COOLDOWN_MS = 5000;
    uint32_t now = millis();
    if ((now - lastWriteTime < WRITE_COOLDOWN_MS) || (!Rotorflight22Detected))
        return;
    uint8_t payload_size = 25;
    if (api100 >= 1208)
        payload_size += 11;
    uint8_t payload[36];
    uint8_t offset = 0;
    payload[offset++] = Rates_Type;
    payload[offset++] = (uint8_t)(Roll_RC_Rate / 10.0f);
    payload[offset++] = (uint8_t)(Roll_RC_Expo * 100.0f);
    payload[offset++] = (uint8_t)(Roll_Rate / 10.0f);
    payload[offset++] = Roll_Response_Time;
    payload[offset++] = (uint8_t)(Roll_Accel_Limit & 0xFF);
    payload[offset++] = (uint8_t)(Roll_Accel_Limit >> 8);
    payload[offset++] = (uint8_t)(Pitch_RC_Rate / 10.0f);
    payload[offset++] = (uint8_t)(Pitch_RC_Expo * 100.0f);
    payload[offset++] = (uint8_t)(Pitch_Rate / 10.0f);
    payload[offset++] = Pitch_Response_Time;
    payload[offset++] = (uint8_t)(Pitch_Accel_Limit & 0xFF);
    payload[offset++] = (uint8_t)(Pitch_Accel_Limit >> 8);
    payload[offset++] = (uint8_t)(Yaw_RC_Rate / 10.0f);
    payload[offset++] = (uint8_t)(Yaw_RC_Expo * 100.0f);
    payload[offset++] = (uint8_t)(Yaw_Rate / 10.0f);
    payload[offset++] = Yaw_Response_Time;
    payload[offset++] = (uint8_t)(Yaw_Accel_Limit & 0xFF);
    payload[offset++] = (uint8_t)(Yaw_Accel_Limit >> 8);
    payload[offset++] = (uint8_t)(Collective_RC_Rate / 10.0f);
    payload[offset++] = (uint8_t)(Collective_RC_Expo * 100.0f);
    payload[offset++] = (uint8_t)(Collective_Rate / 10.0f); // If needs *1, change to /1.0f
    payload[offset++] = Collective_Response_Time;
    payload[offset++] = (uint8_t)(Collective_Accel_Limit & 0xFF);
    payload[offset++] = (uint8_t)(Collective_Accel_Limit >> 8);
    if (api100 >= 1208)
    {
        payload[offset++] = Roll_Setpoint_Boost_Gain;
        payload[offset++] = Roll_Setpoint_Boost_Cutoff;
        payload[offset++] = Pitch_Setpoint_Boost_Gain;
        payload[offset++] = Pitch_Setpoint_Boost_Cutoff;
        payload[offset++] = Yaw_Setpoint_Boost_Gain;
        payload[offset++] = Yaw_Setpoint_Boost_Cutoff;
        payload[offset++] = Collective_Setpoint_Boost_Gain;
        payload[offset++] = Collective_Setpoint_Boost_Cutoff;
        payload[offset++] = Yaw_Dynamic_Ceiling_Gain;
        payload[offset++] = Yaw_Dynamic_Deadband_Gain;
        payload[offset++] = Yaw_Dynamic_Deadband_Filter;
    }
    SendToMSP(MSP_SET_RC_TUNING, payload, payload_size);
    delay(50);
    SendToMSP(MSP_EEPROM_WRITE, nullptr, 0);
    delay(50);
    lastWriteTime = now;
}
#endif // NEXUS_H