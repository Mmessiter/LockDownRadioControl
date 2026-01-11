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
#define MSP_PID_ADVANCED 94     // Advanced PID settings (read)
#define MSP_SET_PID_ADVANCED 95 // write Advanced PID settings

// Add defines for send states
#define SEND_NO_RF 0
#define SEND_PID_RF 1
#define SEND_RATES_RF 2
#define SEND_RATES_ADVANCED_RF 3
#define SEND_PID_ADVANCED_RF 4

// Add global variables for timings
uint32_t Started_Sending_PID_ADVANCED = 0;
uint32_t PID_ADVANCED_Send_Duration = 0;

// -------------------------------------------------------------------------------------------------
// MSP PID ADVANCED parameters (MSP_PID_ADVANCED / MSP_SET_PID_ADVANCED)
//
// These correspond to Rotorflight Configurator > Profiles page shown in your photo.
// NOTE: A few fields exist in the MSP payload but are NOT shown on that screen.
// Those are tagged "NOT ON SCREEN".
// -------------------------------------------------------------------------------------------------

// Tail Rotor Settings (left, bottom in your screenshot)
uint8_t cyclicFeedforwardGain = 0;     // "Cyclic Feedforward Gain"
uint8_t collectiveFeedforwardGain = 0; // "Collective Feedforward Gain"

// Main Rotor Settings (left, middle in your screenshot)
uint8_t collectiveToPitchCompensation = 0; // "Collective to Pitch Compensation" (Configurator shows as a toggle)
uint8_t crossCouplingGain = 0;             // "Cross-Coupling Gain"
uint8_t crossCouplingRatioPercent = 0;     // "Cross-Coupling Ratio [%]"
uint8_t crossCouplingCutoffHz = 0;         // "Cross-Coupling Cutoff Frequency [Hz]"

// PID Controller Settings (left, upper part in your screenshot)
uint8_t iTermRelaxType = 0;       // "I-Term Relax Type" (e.g. RPY)
uint8_t iTermRelaxCutoff = 0;     // "Cutoff Point for Roll/Pitch/Yaw" (Configurator shows 3 boxes; often same value)
uint8_t groundErrorDecayTime = 0; // "Ground Error Decay -> Decay Time [s]" (scaling unknown; often 0.1s units)

// Error limits (left, upper part in your screenshot)
uint16_t errorLimitRollDeg = 0;  // "Error Limit for Roll Axis [°]"
uint16_t errorLimitPitchDeg = 0; // "Error Limit for Pitch Axis [°]"
uint16_t errorLimitYawDeg = 0;   // "Error Limit for Yaw Axis [°]"

// HS Offset (left, upper part in your screenshot)
uint16_t hsOffsetLimitRollDeg = 0;  // "HS Offset Limit for Roll Axis [°]"
uint16_t hsOffsetLimitPitchDeg = 0; // "HS Offset Limit for Pitch Axis [°]"
uint16_t hsOffsetLimitYawDeg = 0;   // NOT ON SCREEN (present in MSP payload)

uint8_t hsOffsetGainRoll = 0;  // "HS Offset Gain for Roll Axis"
uint8_t hsOffsetGainPitch = 0; // "HS Offset Gain for Pitch Axis"
uint8_t hsOffsetGainYaw = 0;   // NOT ON SCREEN (present in MSP payload)

// PID Controller Bandwidth (right side of your screenshot)
uint8_t rollBandwidthHz = 0;  // "Roll Bandwidth"
uint8_t pitchBandwidthHz = 0; // "Pitch Bandwidth"
uint8_t yawBandwidthHz = 0;   // "Yaw Bandwidth"

uint8_t rollDtermCutoffHz = 0;  // "Roll D-term Cutoff"
uint8_t pitchDtermCutoffHz = 0; // "Pitch D-term Cutoff"
uint8_t yawDtermCutoffHz = 0;   // "Yaw D-term Cutoff"

uint8_t bTermCutoffHz = 0; // Configurator shows Roll/Pitch/Yaw B-term Cutoff (3 fields), but MSP gives 1 value here

// Main Rotor Settings (left, lower part in your screenshot)
uint8_t errorDecayMaxRateDps = 0; // likely "Error Decay maximum rate [°/s]"

// NOT ON SCREEN (present in MSP payload)
uint8_t collectiveImpulseFeedforwardGain = 0;      // NOT ON SCREEN
uint8_t collectiveImpulseFeedforwardDecayTime = 0; // NOT ON SCREEN
uint8_t PID_Advanced_Values[19];                  // for ack payload

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
    delay(100);
    SendToMSP(MSP_EEPROM_WRITE, nullptr, 0);
    delay(100);
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
    if ((millis() - Started_Sending_PIDs > PID_Send_Duration) && (SendRotorFlightParametresNow == SEND_PID_RF))
    {
        SendRotorFlightParametresNow = SEND_NO_RF;
    }
    if ((millis() - Started_Sending_RATEs > RATES_Send_Duration) && (SendRotorFlightParametresNow == SEND_RATES_RF))
    {
        SendRotorFlightParametresNow = SEND_NO_RF;
    }
    if ((millis() - Started_Sending_RATES_ADVANCED > RATES_ADVANCED_Send_Duration) && (SendRotorFlightParametresNow == SEND_RATES_ADVANCED_RF))
    {
        SendRotorFlightParametresNow = SEND_NO_RF;
    }
    if ((millis() - Started_Sending_PID_ADVANCED > PID_ADVANCED_Send_Duration) && (SendRotorFlightParametresNow == SEND_PID_ADVANCED_RF))
    {
        SendRotorFlightParametresNow = SEND_NO_RF;
    }
    switch (SendRotorFlightParametresNow)
    {
    case SEND_NO_RF:
        Parse_MSP_Motor_Telemetry(&data_in[0], p);
        RequestFromMSP(MSP_MOTOR_TELEMETRY); // parse reply next time around
                                             //  Look("No RF Send");
        break;
    case SEND_PID_RF:
        Parse_MSP_PID(data_in, p);
        RequestFromMSP(MSP_PID); // parse reply next time around
        break;
    case SEND_RATES_RF:
        Parse_MSP_RC_TUNING(data_in, p);
        RequestFromMSP(MSP_RC_TUNING); // parse reply next time around
        break;
    case SEND_RATES_ADVANCED_RF:
        Parse_MSP_RC_TUNING(data_in, p);
        RequestFromMSP(MSP_RC_TUNING); // parse reply next time around
        break;
    case SEND_PID_ADVANCED_RF:
        Parse_MSP_PID_ADVANCED(data_in, p);
        RequestFromMSP(MSP_PID_ADVANCED); // parse reply next time around
        break;
    default:
        break;
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

// this code will read all of the parameters, allow editing for some of them, and then write all of them.
// They are all stored as uint8_t values in Nexus firmware
// so we need to convert to/from float as needed only for display / editing.
// ************************************************************************************************************

// ************************************************************************************************************
// Store the currently used rates bytes ready for ack payload
// ************************************************************************************************************
void StoreRatesBytesForAckPayload()
{
    RatesBytes[0] = Rates_Type;
    RatesBytes[1] = Roll_Centre_Rate;
    RatesBytes[2] = Roll_Max_Rate;
    RatesBytes[3] = Roll_Expo;
    RatesBytes[4] = Pitch_Centre_Rate;
    RatesBytes[5] = Pitch_Max_Rate;
    RatesBytes[6] = Pitch_Expo;
    RatesBytes[7] = Yaw_Centre_Rate;
    RatesBytes[8] = Yaw_Max_Rate;
    RatesBytes[9] = Yaw_Expo;
    RatesBytes[10] = Collective_Centre_Rate;
    RatesBytes[11] = Collective_Max_Rate;
    RatesBytes[12] = Collective_Expo;
}
// ************************************************************************************************************
// Store the currently used advanced rates bytes ready for ack payload
// ************************************************************************************************************
void StoreAdvancedRatesBytesForAckPayload()
{
    RatesBytesAdvanced[0] = Roll_Response_Time;
    RatesBytesAdvanced[1] = Pitch_Response_Time;
    RatesBytesAdvanced[2] = Yaw_Response_Time;
    RatesBytesAdvanced[3] = Collective_Response_Time;

    RatesBytesAdvanced[4] = Roll_Setpoint_Boost_Gain;
    RatesBytesAdvanced[5] = Pitch_Setpoint_Boost_Gain;
    RatesBytesAdvanced[6] = Yaw_Setpoint_Boost_Gain;
    RatesBytesAdvanced[7] = Collective_Setpoint_Boost_Gain;

    RatesBytesAdvanced[8] = Roll_Setpoint_Boost_Cutoff;
    RatesBytesAdvanced[9] = Pitch_Setpoint_Boost_Cutoff;
    RatesBytesAdvanced[10] = Yaw_Setpoint_Boost_Cutoff;
    RatesBytesAdvanced[11] = Collective_Setpoint_Boost_Cutoff;

    RatesBytesAdvanced[12] = Yaw_Dynamic_Ceiling_Gain;
    RatesBytesAdvanced[13] = Yaw_Dynamic_Deadband_Gain;
    RatesBytesAdvanced[14] = Yaw_Dynamic_Deadband_Filter;
}
// ************************************************************************************************************

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
    Rates_Type = p[offset++];                             // Banner
    Roll_Centre_Rate = p[offset++];                       // * 10         - n0
    Roll_Expo = p[offset++];                              // / 100.0f     - n2
    Roll_Max_Rate = p[offset++];                          // * 10.0f      - n1
    Roll_Response_Time = p[offset++];                     // not yet used
    Roll_Accel_Limit = p[offset] | (p[offset + 1] << 8);  // not yet used
    offset += 2;                                          // ---
    Pitch_Centre_Rate = p[offset++];                      //  * 10.0f;    - n3
    Pitch_Expo = p[offset++];                             // / 100.0f;    - n5
    Pitch_Max_Rate = p[offset++];                         // * 10.0f;     - n4
    Pitch_Response_Time = p[offset++];                    // not yet used
    Pitch_Accel_Limit = p[offset] | (p[offset + 1] << 8); // not yet used
    offset += 2;                                          // ---
    Yaw_Centre_Rate = p[offset++];                        // * 10.0f       - n6
    Yaw_Expo = p[offset++];                               // / 100.0f      - n8
    Yaw_Max_Rate = p[offset++];                           // * 10.0f       - n7
    Yaw_Response_Time = p[offset++];                      // not yet used
    Yaw_Accel_Limit = p[offset] | (p[offset + 1] << 8);   // not yet used
    offset += 2;                                          // ---
    Collective_Centre_Rate = p[offset++];                 // / 4.0f        - n9
    Collective_Expo = p[offset++];                        // / 100.0f      - n11
    Collective_Max_Rate = p[offset++];                    // / 4.0f        - n10

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

    StoreRatesBytesForAckPayload();
    StoreAdvancedRatesBytesForAckPayload();
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
    payload[offset++] = (uint8_t)(Roll_Centre_Rate); // These factors are now handled in the Transmitter so BYTES only are sent
    payload[offset++] = (uint8_t)(Roll_Expo);
    payload[offset++] = (uint8_t)(Roll_Max_Rate);

    payload[offset++] = Roll_Response_Time;
    payload[offset++] = (uint8_t)(Roll_Accel_Limit & 0xFF);
    payload[offset++] = (uint8_t)(Roll_Accel_Limit >> 8);

    payload[offset++] = (uint8_t)(Pitch_Centre_Rate);
    payload[offset++] = (uint8_t)(Pitch_Expo);
    payload[offset++] = (uint8_t)(Pitch_Max_Rate);

    payload[offset++] = Pitch_Response_Time;
    payload[offset++] = (uint8_t)(Pitch_Accel_Limit & 0xFF);
    payload[offset++] = (uint8_t)(Pitch_Accel_Limit >> 8);

    payload[offset++] = (uint8_t)(Yaw_Centre_Rate);
    payload[offset++] = (uint8_t)(Yaw_Expo);
    payload[offset++] = (uint8_t)(Yaw_Max_Rate);

    payload[offset++] = Yaw_Response_Time;
    payload[offset++] = (uint8_t)(Yaw_Accel_Limit & 0xFF);
    payload[offset++] = (uint8_t)(Yaw_Accel_Limit >> 8);

    payload[offset++] = (uint8_t)(Collective_Centre_Rate);
    payload[offset++] = (uint8_t)(Collective_Expo);
    payload[offset++] = (uint8_t)(Collective_Max_Rate);

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
    delay(100);
    SendToMSP(MSP_EEPROM_WRITE, nullptr, 0);
    delay(100);
    lastWriteTime = now;
}

// ************************************************************************************************************
// Store the currently used PID advanced *byte* values ready for ack payload
// (19 values — includes the toggle at [0]) heer!
// ************************************************************************************************************
void StorePIDAdvancedBytesForAckPayload()
{
    PID_Advanced_Values[0] = collectiveToPitchCompensation; // "Collective to Pitch Compensation" (toggle)

    PID_Advanced_Values[1] = cyclicFeedforwardGain;     // "Cyclic Feedforward Gain"
    PID_Advanced_Values[2] = collectiveFeedforwardGain; // "Collective Feedforward Gain"

    PID_Advanced_Values[3] = bTermCutoffHz; // "B-Term Cutoff Hz"

    PID_Advanced_Values[4] = iTermRelaxType;   // "I-Term Relax Type"
    PID_Advanced_Values[5] = iTermRelaxCutoff; // "I-Term Relax Cutoff"

    PID_Advanced_Values[6] = errorDecayMaxRateDps; // "Error Decay Max Rate [deg/s]"
    PID_Advanced_Values[7] = groundErrorDecayTime; // "Ground Error Decay Time"

    PID_Advanced_Values[8] = hsOffsetGainRoll;  // "HS Offset Gain Roll"
    PID_Advanced_Values[9] = hsOffsetGainPitch; // "HS Offset Gain Pitch"

    PID_Advanced_Values[10] = crossCouplingGain;         // "Cross-Coupling Gain"
    PID_Advanced_Values[11] = crossCouplingRatioPercent; // "Cross-Coupling Ratio [%]"
    PID_Advanced_Values[12] = crossCouplingCutoffHz;     // "Cross-Coupling Cutoff [Hz]"


    
    PID_Advanced_Values[13] = rollBandwidthHz;  // "Roll Bandwidth [Hz]"
    PID_Advanced_Values[14] = pitchBandwidthHz; // "Pitch Bandwidth [Hz]"
    PID_Advanced_Values[15] = yawBandwidthHz;   // "Yaw Bandwidth [Hz]"

    PID_Advanced_Values[16] = rollDtermCutoffHz;  // "Roll D-term Cutoff [Hz]"
    PID_Advanced_Values[17] = pitchDtermCutoffHz; // "Pitch D-term Cutoff [Hz]"
    PID_Advanced_Values[18] = yawDtermCutoffHz;   // "Yaw D-term Cutoff [Hz]"
}
// ************************************************************************************************************

//         MSP_PID_ADVANCED FROM ROTORFLIGHT FIRMWARE

inline bool Parse_MSP_PID_ADVANCED(const uint8_t *data, uint8_t n)
{
    MspFrame f;
    if (!FindMspV1ResponseFrame(data, n, f))
        return false;
    if (f.cmd != MSP_PID_ADVANCED)
        return false;

    if (f.size < 34)
        return false;

    const uint8_t *p = f.payload;
    uint8_t offset = 0;

    // Main Rotor Settings
    collectiveToPitchCompensation = p[offset++]; // "Collective to Pitch Compensation" (shown as toggle)

    // Tail Rotor Settings
    cyclicFeedforwardGain = p[offset++];     // "Cyclic Feedforward Gain"
    collectiveFeedforwardGain = p[offset++]; // "Collective Feedforward Gain"

    // NOT ON SCREEN (present in MSP payload)
    collectiveImpulseFeedforwardGain = p[offset++];      // NOT ON SCREEN
    collectiveImpulseFeedforwardDecayTime = p[offset++]; // NOT ON SCREEN

    // Bandwidth / Cutoffs
    bTermCutoffHz = p[offset++]; // UI shows per-axis; MSP provides one value

    // PID Controller Settings
    iTermRelaxType = p[offset++];       // "I-Term Relax Type"
    iTermRelaxCutoff = p[offset++];     // "Cutoff Point for Roll/Pitch/Yaw" (often shared)
    errorDecayMaxRateDps = p[offset++]; // likely "Error Decay maximum rate [°/s]"
    groundErrorDecayTime = p[offset++]; // "Ground Error Decay -> Decay Time [s]" (scaling unknown)

    // Error limits
    errorLimitRollDeg = (uint16_t)p[offset] | ((uint16_t)p[offset + 1] << 8);
    offset += 2;
    errorLimitPitchDeg = (uint16_t)p[offset] | ((uint16_t)p[offset + 1] << 8);
    offset += 2;
    errorLimitYawDeg = (uint16_t)p[offset] | ((uint16_t)p[offset + 1] << 8);
    offset += 2;

    // HS Offset limits
    hsOffsetLimitRollDeg = (uint16_t)p[offset] | ((uint16_t)p[offset + 1] << 8);
    offset += 2;
    hsOffsetLimitPitchDeg = (uint16_t)p[offset] | ((uint16_t)p[offset + 1] << 8);
    offset += 2;
    hsOffsetLimitYawDeg = (uint16_t)p[offset] | ((uint16_t)p[offset + 1] << 8);
    offset += 2; // NOT ON SCREEN

    // HS Offset gains
    hsOffsetGainRoll = p[offset++];  // shown
    hsOffsetGainPitch = p[offset++]; // shown
    hsOffsetGainYaw = p[offset++];   // NOT ON SCREEN

    // Cross-coupling
    crossCouplingGain = p[offset++];         // "Cross-Coupling Gain"
    crossCouplingRatioPercent = p[offset++]; // "Cross-Coupling Ratio [%]"
    crossCouplingCutoffHz = p[offset++];     // "Cross-Coupling Cutoff Frequency [Hz]"

    // Bandwidth
    rollBandwidthHz = p[offset++];  // "Roll Bandwidth"
    pitchBandwidthHz = p[offset++]; // "Pitch Bandwidth"
    yawBandwidthHz = p[offset++];   // "Yaw Bandwidth"

    // D-term cutoff
    rollDtermCutoffHz = p[offset++];  // "Roll D-term Cutoff"
    pitchDtermCutoffHz = p[offset++]; // "Pitch D-term Cutoff"
    yawDtermCutoffHz = p[offset++];   // "Yaw D-term Cutoff"

    return true;
}
// ************************************************************************************************************

inline void WritePIDAdvancedToNexusAndSave()
{
    static uint32_t lastWriteTime = 0;
    const uint32_t WRITE_COOLDOWN_MS = 5000;
    uint32_t now = millis();
    if ((now - lastWriteTime < WRITE_COOLDOWN_MS) || (!Rotorflight22Detected))
        return;

    uint8_t payload[34];
    uint8_t offset = 0;

    payload[offset++] = collectiveToPitchCompensation;

    payload[offset++] = cyclicFeedforwardGain;
    payload[offset++] = collectiveFeedforwardGain;

    payload[offset++] = collectiveImpulseFeedforwardGain;      // NOT ON SCREEN
    payload[offset++] = collectiveImpulseFeedforwardDecayTime; // NOT ON SCREEN

    payload[offset++] = bTermCutoffHz;

    payload[offset++] = iTermRelaxType;
    payload[offset++] = iTermRelaxCutoff;
    payload[offset++] = errorDecayMaxRateDps;
    payload[offset++] = groundErrorDecayTime;

    payload[offset++] = (uint8_t)(errorLimitRollDeg & 0xFF);
    payload[offset++] = (uint8_t)(errorLimitRollDeg >> 8);
    payload[offset++] = (uint8_t)(errorLimitPitchDeg & 0xFF);
    payload[offset++] = (uint8_t)(errorLimitPitchDeg >> 8);
    payload[offset++] = (uint8_t)(errorLimitYawDeg & 0xFF);
    payload[offset++] = (uint8_t)(errorLimitYawDeg >> 8);

    payload[offset++] = (uint8_t)(hsOffsetLimitRollDeg & 0xFF);
    payload[offset++] = (uint8_t)(hsOffsetLimitRollDeg >> 8);
    payload[offset++] = (uint8_t)(hsOffsetLimitPitchDeg & 0xFF);
    payload[offset++] = (uint8_t)(hsOffsetLimitPitchDeg >> 8);
    payload[offset++] = (uint8_t)(hsOffsetLimitYawDeg & 0xFF); // NOT ON SCREEN
    payload[offset++] = (uint8_t)(hsOffsetLimitYawDeg >> 8);   // NOT ON SCREEN

    payload[offset++] = hsOffsetGainRoll;
    payload[offset++] = hsOffsetGainPitch;
    payload[offset++] = hsOffsetGainYaw; // NOT ON SCREEN

    payload[offset++] = crossCouplingGain;
    payload[offset++] = crossCouplingRatioPercent;
    payload[offset++] = crossCouplingCutoffHz;

    payload[offset++] = rollBandwidthHz;
    payload[offset++] = pitchBandwidthHz;
    payload[offset++] = yawBandwidthHz;

    payload[offset++] = rollDtermCutoffHz;
    payload[offset++] = pitchDtermCutoffHz;
    payload[offset++] = yawDtermCutoffHz;

    SendToMSP(MSP_SET_PID_ADVANCED, payload, sizeof(payload));
    delay(100);
    SendToMSP(MSP_EEPROM_WRITE, nullptr, 0);
    delay(100);
    lastWriteTime = now;
}

#endif // NEXUS_H