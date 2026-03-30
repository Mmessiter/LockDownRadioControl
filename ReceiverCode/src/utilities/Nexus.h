/** @file ReceiverCode/src/utilities/Nexus.h */
// Malcolm Messiter 2020 - 2025
#ifndef NEXUS_H
#define NEXUS_H
#include "utilities/1Definitions.h"

// ************************************************************************************************************
// MSP SUPPORT FUNCTIONS FOR ROTORFLIGHT (Nexus) etc.
// ************************************************************************************************************
#define MSP_API_VERSION 1
#define MSP_MOTOR_TELEMETRY 139 // Motor telemetry data
#define MSP_PID 112             // PID settings (read)
#define MSP_SET_PID 202         // write PID settings
#define MSP_EEPROM_WRITE 250    // save settings to EEPROM/flash
#define MSP_RC_TUNING 111       // RC Tuning settings (RATES etc)
#define MSP_SET_RC_TUNING 204   // write RC Tuning settings (RATES etc)
#define MSP_PID_PROFILE 94      // Advanced PID settings (read)
#define MSP_SET_PID_PROFILE 95  // write Advanced PID settings
#define MSP_SELECT_SETTING 210  // select setting bank

// Governor MSP command IDs (RF 2.3+ only, api100 >= 1209)
#define MSP_GOVERNOR_CONFIG 142      // read  global governor settings
#define MSP_SET_GOVERNOR_CONFIG 143  // write global governor settings
#define MSP_GOVERNOR_PROFILE 148     // read  per-profile governor tuning
#define MSP_SET_GOVERNOR_PROFILE 149 // write per-profile governor tuning

// Send states
#define SEND_NO_RF 0
#define SEND_PID_RF 1
#define SEND_RATES_RF 2
#define SEND_RATES_ADVANCED_RF 3
#define SEND_PID_ADVANCED_RF 4
#define SEND_GOV_CONFIG_RF 5
#define SEND_GOV_PROFILE_RF 6

// Governor flags bitmap helpers
#define GOV_FLAG_FALLBACK_PRECOMP (1u << 2)
#define GOV_FLAG_VOLTAGE_COMP (1u << 3)
#define GOV_FLAG_PID_SPOOLUP (1u << 4)
#define GOV_FLAG_DYN_MIN_THROTTLE (1u << 6)

// Governor payload sizes
#define GOV_CONFIG_PAYLOAD_SIZE 42
#define GOV_PROFILE_PAYLOAD_SIZE 17

// Governor ACK payload size
// 35 display values packed as described in PackGovernorForAckPayload()
#define GOV_ACK_PAYLOAD_SIZE 59

// ************************************************************************************************************
// Detect if Rotorflight 2.2 is present at boot time
// If detected, fetch API version and protocol version
// Keeps the MSP_UART open if Rotorflight 2.2 is present
// Closes it if not present
// ************************************************************************************************************

inline void DetectRotorFlightAtBoot()
{
#define Ports_Count 2
    HardwareSerial *Ports[Ports_Count] = {&Serial6, &Serial1};
    for (uint8_t i = 0; i < Ports_Count; i++)
    {
        This_MSP_Uart = Ports[i];
        DetectRotorFlightAtBoot1();
        if (Rotorflight_Version)
            return;
    }
}

// ************************************************************************************************************
inline void DetectRotorFlightAtBoot1()
{
#define NEXUS_DETECT_WINDOW_MS 1500

    This_MSP_Uart->begin(115200);
    while (This_MSP_Uart->available())
        (void)This_MSP_Uart->read();

    uint32_t start = millis();
    uint8_t buf[80];

    Rotorflight_Version = 0;

    while (millis() - start < NEXUS_DETECT_WINDOW_MS)
    {
        RequestFromMSP(MSP_MOTOR_TELEMETRY);
        delay(20);

        uint8_t p = 0;
        while (This_MSP_Uart->available() && p < sizeof(buf))
            buf[p++] = (uint8_t)This_MSP_Uart->read();
        if (Parse_MSP_Motor_Telemetry(buf, p))
        {
            RequestFromMSP(MSP_API_VERSION);
            delay(20);

            p = 0;
            while (This_MSP_Uart->available() && p < sizeof(buf))
                buf[p++] = (uint8_t)This_MSP_Uart->read();

            uint8_t apiMaj = 0, apiMin = 0, proto = 0;
            if (Parse_MSP_API_VERSION(buf, p, proto, apiMaj, apiMin))
            {
                if (api100 == 1208)
                    Rotorflight_Version = 1; // RF 2.2
                if (api100 >= 1209)
                    Rotorflight_Version = 2; // RF 2.3+
            }
            return;
        }
    }
    This_MSP_Uart->end();
    Rotorflight_Version = 0;
}

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
// ACK / reply wait support
// ************************************************************************************************************
static volatile bool MspAckWaitInProgress = false;

inline bool WaitForMspAck(uint8_t expectedCmd, uint32_t timeoutMs)
{
    enum
    {
        ST_IDLE = 0,
        ST_DOLLAR,
        ST_M,
        ST_DIR,
        ST_SIZE,
        ST_CMD,
        ST_PAYLOAD,
        ST_CHECKSUM
    };

    uint8_t state = ST_IDLE;
    uint8_t dir = 0;
    uint8_t size = 0;
    uint8_t cmd = 0;
    uint8_t idx = 0;
    uint8_t checksum = 0;

    const uint32_t start = millis();
    MspAckWaitInProgress = true;

    while ((uint32_t)(millis() - start) < timeoutMs)
    {
        while (This_MSP_Uart->available())
        {
            uint8_t c = (uint8_t)This_MSP_Uart->read();
            switch (state)
            {
            case ST_IDLE:
                if (c == '$')
                    state = ST_DOLLAR;
                break;
            case ST_DOLLAR:
                state = (c == 'M') ? ST_M : ST_IDLE;
                break;
            case ST_M:
                if (c == '>' || c == '!')
                {
                    dir = c;
                    state = ST_DIR;
                }
                else
                    state = ST_IDLE;
                break;
            case ST_DIR:
                size = c;
                checksum = c;
                idx = 0;
                state = ST_SIZE;
                break;
            case ST_SIZE:
                cmd = c;
                checksum ^= c;
                state = (size == 0) ? ST_CHECKSUM : ST_PAYLOAD;
                break;
            case ST_PAYLOAD:
                checksum ^= c;
                idx++;
                if (idx >= size)
                    state = ST_CHECKSUM;
                break;
            case ST_CHECKSUM:
                if (checksum == c && cmd == expectedCmd)
                {
                    MspAckWaitInProgress = false;
                    return (dir == '>');
                }
                state = ST_IDLE;
                break;
            default:
                state = ST_IDLE;
                break;
            }
        }
        delay(1);
    }
    MspAckWaitInProgress = false;
    return false;
}

// ************************************************************************************************************
static bool Parse_MSP_API_VERSION(const uint8_t *buf, uint8_t len, uint8_t &mspProto, uint8_t &apiMaj, uint8_t &apiMin)
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
        api100 = (uint16_t)apiMaj * 100u + (uint16_t)apiMin;
        return true;
    }
    return false;
}

// ************************************************************************************************************
struct MspFrame
{
    uint8_t size;
    uint8_t cmd;
    const uint8_t *payload;
};

// ************************************************************************************************************
inline bool FindMspV1ResponseFrame(const uint8_t *data, uint8_t n, MspFrame &out)
{
    for (uint8_t i = 0; i + 6 < n; ++i)
    {
        if (data[i] != '$' || data[i + 1] != 'M' || data[i + 2] != '>')
            continue;
        uint8_t size = data[i + 3], cmd = data[i + 4];
        uint16_t frame_end = i + 5 + size;
        if (frame_end >= n)
            continue;
        uint8_t ck = size ^ cmd;
        for (uint8_t k = 0; k < size; ++k)
            ck ^= data[i + 5 + k];
        if (ck != data[frame_end])
            continue;
        out.size = size;
        out.cmd = cmd;
        out.payload = &data[i + 5];
        return true;
    }
    return false;
}

// ************************************************************************************************************
inline bool FindMspV1ResponseFrameForCmd(const uint8_t *data, uint8_t n, uint8_t wantCmd, MspFrame &out)
{
    for (uint8_t i = 0; i + 6 < n; ++i)
    {
        if (data[i] != '$' || data[i + 1] != 'M' || data[i + 2] != '>')
            continue;
        uint8_t size = data[i + 3], cmd = data[i + 4];
        uint16_t frame_end = i + 5 + size;
        if (frame_end >= n)
            continue;
        uint8_t ck = size ^ cmd;
        for (uint8_t k = 0; k < size; ++k)
            ck ^= data[i + 5 + k];
        if (ck != data[frame_end])
            continue;
        if (cmd != wantCmd)
            continue;
        out.size = size;
        out.cmd = cmd;
        out.payload = &data[i + 5];
        return true;
    }
    return false;
}

// ************************************************************************************************************
inline void RequestFromMSP(uint8_t command)
{
    uint8_t checksum = 0;
    This_MSP_Uart->write('$');
    This_MSP_Uart->write('M');
    This_MSP_Uart->write('<');
    This_MSP_Uart->write((uint8_t)0);
    checksum ^= 0;
    This_MSP_Uart->write(command);
    checksum ^= command;
    This_MSP_Uart->write(checksum);
}

// ************************************************************************************************************
inline void SendToMSP(uint8_t command, const uint8_t *payload, uint8_t payloadSize)
{
    uint8_t checksum = 0;
    This_MSP_Uart->write('$');
    This_MSP_Uart->write('M');
    This_MSP_Uart->write('<');
    This_MSP_Uart->write(payloadSize);
    checksum ^= payloadSize;
    This_MSP_Uart->write(command);
    checksum ^= command;
    for (uint8_t i = 0; i < payloadSize; i++)
    {
        uint8_t b = payload[i];
        This_MSP_Uart->write(b);
        checksum ^= b;
    }
    This_MSP_Uart->write(checksum);
}

// ************************************************************************************************************
inline void WritePIDsToNexusAndSave(const uint16_t pid[17])
{
    static uint32_t lastWriteTime = 0;
    const uint32_t WRITE_COOLDOWN_MS = 5000;
    uint32_t now = millis();
    if ((now - lastWriteTime < WRITE_COOLDOWN_MS) || (!Rotorflight_Version))
        return;

    uint8_t payload[34] = {0};
    for (int i = 0; i < 34; i++)
        payload[i] = Original_PID_Values[i];
    for (uint8_t i = 0; i < 17; i++)
    {
        payload[i * 2 + 0] = (uint8_t)(pid[i] & 0xFF);
        payload[i * 2 + 1] = (uint8_t)(pid[i] >> 8);
    }
    SendToMSP(MSP_SET_PID, payload, sizeof(payload));
    This_MSP_Uart->flush();
    if (!WaitForMspAck(MSP_SET_PID, 300))
        return;
    SendToMSP(MSP_EEPROM_WRITE, nullptr, 0);
    This_MSP_Uart->flush();
    if (!WaitForMspAck(MSP_EEPROM_WRITE, 1000))
        return;
    lastWriteTime = millis();
}

// ************************************************************************************************************
void DebugPIDValues(const char *msg)
{
    Look(msg);
    Look1("Roll P: ");
    Look(PID_Roll_P);
    Look1("Roll I: ");
    Look(PID_Roll_I);
    Look1("Roll D: ");
    Look(PID_Roll_D);
    Look1("Roll FF: ");
    Look(PID_Roll_FF);
    Look1("Pitch P: ");
    Look(PID_Pitch_P);
    Look1("Pitch I: ");
    Look(PID_Pitch_I);
    Look1("Pitch D: ");
    Look(PID_Pitch_D);
    Look1("Pitch FF: ");
    Look(PID_Pitch_FF);
    Look1("Yaw P: ");
    Look(PID_Yaw_P);
    Look1("Yaw I: ");
    Look(PID_Yaw_I);
    Look1("Yaw D: ");
    Look(PID_Yaw_D);
    Look1("Yaw FF: ");
    Look(PID_Yaw_FF);
    Look1("Roll Boost: ");
    Look(PID_Roll_Boost);
    Look1("Pitch Boost: ");
    Look(PID_Pitch_Boost);
    Look1("Yaw Boost: ");
    Look(PID_Yaw_Boost);
}

// ************************************************************************************************************
//         MSP_PID FROM ROTORFLIGHT FIRMWARE
// ************************************************************************************************************
inline bool Parse_MSP_PID(const uint8_t *data, uint8_t n)
{
    MspFrame f;
    if (!FindMspV1ResponseFrameForCmd(data, n, MSP_PID, f))
        return false;
    if (f.size < 34)
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
    PID_Roll_Boost = p[24] | (p[25] << 8);
    PID_Pitch_Boost = p[26] | (p[27] << 8);
    PID_Yaw_Boost = p[28] | (p[29] << 8);
    PID_HSI_Offset_Roll = p[30] | (p[31] << 8);
    PID_HSI_Offset_Pitch = p[32] | (p[33] << 8);

    for (int i = 0; i < 34; i++)
        Original_PID_Values[i] = p[i];
    return true;
}

// ************************************************************************************************************
//         MSP_MOTOR_TELEMETRY FROM ROTORFLIGHT FIRMWARE
// ************************************************************************************************************
inline bool Parse_MSP_Motor_Telemetry(const uint8_t *data, uint8_t n)
{
    if (!Ratio)
        Ratio = 10.3f;
    MspFrame f;
    if (!FindMspV1ResponseFrameForCmd(data, n, MSP_MOTOR_TELEMETRY, f))
        return false;
    if (f.size < 15)
        return false;
    if (f.payload[0] != 1)
        return false;

    const uint8_t *payload = f.payload;
    RotorRPM = ((uint32_t)payload[1] | ((uint32_t)payload[2] << 8) |
                ((uint32_t)payload[3] << 16) | ((uint32_t)payload[4] << 24)) /
               Ratio;
    RXModelVolts = (float)((uint16_t)payload[7] | ((uint16_t)payload[8] << 8)) / 1000.0f;
    Battery_Amps = (float)((uint16_t)payload[9] | ((uint16_t)payload[10] << 8)) / 1000.0f;
    Battery_mAh = (float)((uint16_t)payload[11] | ((uint16_t)payload[12] << 8));
    ESC_Temp_C = (float)((uint16_t)payload[13] | ((uint16_t)payload[14] << 8)) / 10.0f;
    if (RXModelVolts > 26.0f)
        RXModelVolts /= 2.0f;
    return true;
}

// ************************************************************************************************************
inline void CheckMSPSerial()
{
    if (MspAckWaitInProgress)
        return;
    uint32_t interval = (SendRotorFlightParametresNow == SEND_NO_RF) ? 250 : 50;
    static uint32_t Localtimer = 0;
    uint32_t Now = millis();
    if (Now - Localtimer < interval)
        return;
    Localtimer = Now;

    bool overflow = false;
    uint8_t data_in[180];
    uint16_t p = 0;
    while (This_MSP_Uart->available())
    {
        uint8_t c = This_MSP_Uart->read();
        if (overflow)
            continue;
        if (p < sizeof(data_in))
            data_in[p++] = c;
        else
            overflow = true;
    }
    if (overflow)
        return;

    bool looksLikeMSP =
        (p >= 6 && data_in[0] == '$' && data_in[1] == 'M') ||
        (p >= 8 && data_in[0] == '$' && data_in[1] == 'X');

    if (!looksLikeMSP)
    {
        switch (SendRotorFlightParametresNow)
        {
        case SEND_NO_RF:
            if (!InhibitTelemetry)
                RequestFromMSP(MSP_MOTOR_TELEMETRY);
            break;
        case SEND_PID_RF:
            RequestFromMSP(MSP_PID);
            break;
        case SEND_RATES_RF:
            RequestFromMSP(MSP_RC_TUNING);
            break;
        case SEND_RATES_ADVANCED_RF:
            RequestFromMSP(MSP_RC_TUNING);
            break;
        case SEND_PID_ADVANCED_RF:
            RequestFromMSP(MSP_PID_PROFILE);
            break;
        case SEND_GOV_CONFIG_RF:
            RequestFromMSP(MSP_GOVERNOR_CONFIG);
            break;
        case SEND_GOV_PROFILE_RF:
            RequestFromMSP(MSP_GOVERNOR_PROFILE);
            break;
        default:
            break;
        }
        return;
    }

    // Timeout checks
    if ((Now - Started_Sending_PIDs > PID_Send_Duration) && (SendRotorFlightParametresNow == SEND_PID_RF))
        SendRotorFlightParametresNow = SEND_NO_RF;
    if ((Now - Started_Sending_RATEs > RATES_Send_Duration) && (SendRotorFlightParametresNow == SEND_RATES_RF))
        SendRotorFlightParametresNow = SEND_NO_RF;
    if ((Now - Started_Sending_RATES_ADVANCED > RATES_ADVANCED_Send_Duration) && (SendRotorFlightParametresNow == SEND_RATES_ADVANCED_RF))
        SendRotorFlightParametresNow = SEND_NO_RF;
    if ((Now - Started_Sending_PID_ADVANCED > PID_ADVANCED_Send_Duration) && (SendRotorFlightParametresNow == SEND_PID_ADVANCED_RF))
        SendRotorFlightParametresNow = SEND_NO_RF;
    if ((Now - Started_Sending_STATUS_EX > STATUS_EX_Send_Duration) && (SendRotorFlightParametresNow == SEND_STATUS_EX))
        SendRotorFlightParametresNow = SEND_NO_RF;
    if ((Now - Started_Sending_GOV_CONFIG > GOV_Send_Duration) && (SendRotorFlightParametresNow == SEND_GOV_CONFIG_RF))
        SendRotorFlightParametresNow = SEND_NO_RF;
    if ((Now - Started_Sending_GOV_PROFILE > GOV_Send_Duration) && (SendRotorFlightParametresNow == SEND_GOV_PROFILE_RF))
        SendRotorFlightParametresNow = SEND_NO_RF;

    // Parse and re-request
    switch (SendRotorFlightParametresNow)
    {
    case SEND_NO_RF:
        if (!InhibitTelemetry)
        {
            Parse_MSP_Motor_Telemetry(&data_in[0], p);
            RequestFromMSP(MSP_MOTOR_TELEMETRY);
        }
        break;
    case SEND_PID_RF:
        Parse_MSP_PID(data_in, p);
        RequestFromMSP(MSP_PID);
        break;
    case SEND_RATES_RF:
        Parse_MSP_RC_TUNING(data_in, p);
        RequestFromMSP(MSP_RC_TUNING);
        break;
    case SEND_RATES_ADVANCED_RF:
        Parse_MSP_RC_TUNING(data_in, p);
        RequestFromMSP(MSP_RC_TUNING);
        break;
    case SEND_PID_ADVANCED_RF:
        Parse_MSP_PID_PROFILE(data_in, p);
        RequestFromMSP(MSP_PID_PROFILE);
        break;
    case SEND_GOV_CONFIG_RF:
        if (api100 >= 1209)
            Parse_MSP_Governor_Config(data_in, p);
        else
            Governor_RF23_Required = true;
        RequestFromMSP(MSP_GOVERNOR_CONFIG);
        break;
    case SEND_GOV_PROFILE_RF:
        if (api100 >= 1209)
            Parse_MSP_Governor_Profile(data_in, p);
        else
            Governor_RF23_Required = true;
        RequestFromMSP(MSP_GOVERNOR_PROFILE);
        break;
    default:
        break;
    }
}

// ************************************************************************************************************
//         RATES
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

inline bool Parse_MSP_RC_TUNING(const uint8_t *data, uint8_t n)
{
    MspFrame f;
    if (!FindMspV1ResponseFrameForCmd(data, n, MSP_RC_TUNING, f))
        return false;

    uint8_t min_size = 25;
    if (api100 >= 1208)
        min_size += 11;
    if (f.size < min_size)
        return false;

    const uint8_t *p = f.payload;
    uint8_t offset = 0;

    Rates_Type = p[offset++];
    Roll_Centre_Rate = p[offset++];
    Roll_Expo = p[offset++];
    Roll_Max_Rate = p[offset++];
    Roll_Response_Time = p[offset++];
    Roll_Accel_Limit = p[offset] | (p[offset + 1] << 8);
    offset += 2;

    Pitch_Centre_Rate = p[offset++];
    Pitch_Expo = p[offset++];
    Pitch_Max_Rate = p[offset++];
    Pitch_Response_Time = p[offset++];
    Pitch_Accel_Limit = p[offset] | (p[offset + 1] << 8);
    offset += 2;

    Yaw_Centre_Rate = p[offset++];
    Yaw_Expo = p[offset++];
    Yaw_Max_Rate = p[offset++];
    Yaw_Response_Time = p[offset++];
    Yaw_Accel_Limit = p[offset] | (p[offset + 1] << 8);
    offset += 2;

    Collective_Centre_Rate = p[offset++];
    Collective_Expo = p[offset++];
    Collective_Max_Rate = p[offset++];
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

inline void WriteRatesToNexusAndSave()
{
    static uint32_t lastWriteTime = 0;
    const uint32_t WRITE_COOLDOWN_MS = 5000;
    uint32_t now = millis();
    if ((now - lastWriteTime < WRITE_COOLDOWN_MS) || (!Rotorflight_Version))
        return;

    delay(20);
    This_MSP_Uart->flush();

    uint8_t payload_size = 25;
    if (api100 >= 1208)
        payload_size += 11;

    uint8_t payload[36] = {0};
    uint8_t offset = 0;

    payload[offset++] = Rates_Type;
    payload[offset++] = (uint8_t)(Roll_Centre_Rate);
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

    if (offset != payload_size)
        return;

    SendToMSP(MSP_SET_RC_TUNING, payload, payload_size);
    This_MSP_Uart->flush();
    if (!WaitForMspAck(MSP_SET_RC_TUNING, 600))
        return;

    delay(50);
    SendToMSP(MSP_EEPROM_WRITE, nullptr, 0);
    This_MSP_Uart->flush();
    if (!WaitForMspAck(MSP_EEPROM_WRITE, 1500))
        return;

    lastWriteTime = millis();
}

// ************************************************************************************************************
//         MSP_PID_PROFILE FROM ROTORFLIGHT FIRMWARE
// ************************************************************************************************************
inline bool Parse_MSP_PID_PROFILE(const uint8_t *data, uint8_t n)
{
    MspFrame f;
    if (!FindMspV1ResponseFrameForCmd(data, n, MSP_PID_PROFILE, f))
        return false;
    if (f.size < MAX_PID_ADVANCED_BYTES)
        return false;

    const uint8_t *p = f.payload;
    for (uint8_t i = 0; i < MAX_PID_ADVANCED_BYTES; i++)
        Original_PID_Advanced_Bytes[i] = p[i];

    Piro_Compensation6 = p[6];
    PID_Advanced_Bytes[0] = Piro_Compensation6;
    Ground_Error_Decay1 = p[1];
    PID_Advanced_Bytes[1] = Ground_Error_Decay1;
    Cutoff_Roll17 = p[17];
    PID_Advanced_Bytes[2] = Cutoff_Roll17;
    Cutoff_Pitch18 = p[18];
    PID_Advanced_Bytes[3] = Cutoff_Pitch18;
    Cutoff_Yaw19 = p[19];
    PID_Advanced_Bytes[4] = Cutoff_Yaw19;
    Error_Limit_Roll7 = p[7];
    PID_Advanced_Bytes[5] = Error_Limit_Roll7;
    Error_Limit_Pitch8 = p[8];
    PID_Advanced_Bytes[6] = Error_Limit_Pitch8;
    Error_Limit_Yaw9 = p[9];
    PID_Advanced_Bytes[7] = Error_Limit_Yaw9;
    HSI_Offset_Limit_Roll36 = p[36];
    PID_Advanced_Bytes[8] = HSI_Offset_Limit_Roll36;
    HSI_Offset_Limit_Pitch37 = p[37];
    PID_Advanced_Bytes[9] = HSI_Offset_Limit_Pitch37;
    HSI_Offset_Bandwidth_Roll10 = p[10];
    PID_Advanced_Bytes[10] = HSI_Offset_Bandwidth_Roll10;
    HSI_Offset_Bandwidth_Pitch11 = p[11];
    PID_Advanced_Bytes[11] = HSI_Offset_Bandwidth_Pitch11;
    HSI_Offset_Bandwidth_Yaw12 = p[12];
    PID_Advanced_Bytes[12] = HSI_Offset_Bandwidth_Yaw12;
    Roll_D_Term_Cutoff13 = p[13];
    PID_Advanced_Bytes[13] = Roll_D_Term_Cutoff13;
    Pitch_D_Term_Cutoff14 = p[14];
    PID_Advanced_Bytes[14] = Pitch_D_Term_Cutoff14;
    Yaw_D_Term_Cutoff15 = p[15];
    PID_Advanced_Bytes[15] = Yaw_D_Term_Cutoff15;
    Roll_B_Term_Cutoff38 = p[38];
    PID_Advanced_Bytes[16] = Roll_B_Term_Cutoff38;
    Pitch_B_Term_Cutoff39 = p[39];
    PID_Advanced_Bytes[17] = Pitch_B_Term_Cutoff39;
    Yaw_B_Term_Cutoff40 = p[40];
    PID_Advanced_Bytes[18] = Yaw_B_Term_Cutoff40;
    CW_Yaw_Stop_Gain20 = p[20];
    PID_Advanced_Bytes[19] = CW_Yaw_Stop_Gain20;
    CCW_Yaw_Stop_Gain21 = p[21];
    PID_Advanced_Bytes[20] = CCW_Yaw_Stop_Gain21;
    Yaw_Precomp_Cutoff22 = p[22];
    PID_Advanced_Bytes[21] = Yaw_Precomp_Cutoff22;
    Cyclic_FF_Gain23 = p[23];
    PID_Advanced_Bytes[22] = Cyclic_FF_Gain23;
    Collective_FF_Gain24 = p[24];
    PID_Advanced_Bytes[23] = Collective_FF_Gain24;
    Inertia_Precomp_Gain41 = p[41];
    PID_Advanced_Bytes[24] = Inertia_Precomp_Gain41;
    Inertia_Precomp_Cutoff42 = p[42];
    PID_Advanced_Bytes[25] = Inertia_Precomp_Cutoff42;

    return true;
}

inline void WritePIDAdvancedToNexusAndSave()
{
    static uint32_t lastWriteTime = 0;
    const uint32_t WRITE_COOLDOWN_MS = 5000;
    uint32_t now = millis();
    if ((now - lastWriteTime < WRITE_COOLDOWN_MS) || (!Rotorflight_Version))
        return;

    uint8_t payload[MAX_PID_ADVANCED_BYTES];
    for (uint8_t i = 0; i < MAX_PID_ADVANCED_BYTES; i++)
        payload[i] = Original_PID_Advanced_Bytes[i];

    payload[6] = PID_Advanced_Bytes[0];
    payload[1] = PID_Advanced_Bytes[1];
    payload[17] = PID_Advanced_Bytes[2];
    payload[18] = PID_Advanced_Bytes[3];
    payload[19] = PID_Advanced_Bytes[4];
    payload[7] = PID_Advanced_Bytes[5];
    payload[8] = PID_Advanced_Bytes[6];
    payload[9] = PID_Advanced_Bytes[7];
    payload[36] = PID_Advanced_Bytes[8];
    payload[37] = PID_Advanced_Bytes[9];
    payload[10] = PID_Advanced_Bytes[10];
    payload[11] = PID_Advanced_Bytes[11];
    payload[12] = PID_Advanced_Bytes[12];
    payload[13] = PID_Advanced_Bytes[13];
    payload[14] = PID_Advanced_Bytes[14];
    payload[15] = PID_Advanced_Bytes[15];
    payload[38] = PID_Advanced_Bytes[16];
    payload[39] = PID_Advanced_Bytes[17];
    payload[40] = PID_Advanced_Bytes[18];
    payload[20] = PID_Advanced_Bytes[19];
    payload[21] = PID_Advanced_Bytes[20];
    payload[22] = PID_Advanced_Bytes[21];
    payload[23] = PID_Advanced_Bytes[22];
    payload[24] = PID_Advanced_Bytes[23];
    payload[41] = PID_Advanced_Bytes[24];
    payload[42] = PID_Advanced_Bytes[25];

    SendToMSP(MSP_SET_PID_PROFILE, payload, sizeof(payload));
    This_MSP_Uart->flush();
    if (!WaitForMspAck(MSP_SET_PID_PROFILE, 300))
        return;
    SendToMSP(MSP_EEPROM_WRITE, nullptr, 0);
    This_MSP_Uart->flush();
    if (!WaitForMspAck(MSP_EEPROM_WRITE, 1000))
        return;
    lastWriteTime = millis();
}

inline void SetNexusProfile(uint8_t index)
{
    if (!Rotorflight_Version)
        return;
    SendToMSP(MSP_SELECT_SETTING, &index, 1);
    This_MSP_Uart->flush();
    (void)WaitForMspAck(MSP_SELECT_SETTING, 300);
}

// ************************************************************************************************************
// ************************************************************************************************************
//         GOVERNOR MSP SUPPORT — ROTORFLIGHT 2.3+ ONLY (api100 >= 1209)
// ************************************************************************************************************
// ************************************************************************************************************

// ====================================================
// Governor global variables
// ====================================================

// Upgrade-nag flag — transmitter should show "Please upgrade to Rotorflight 2.3"
// when this arrives in the ACK payload as true.
//static bool Governor_RF23_Required = false;

// Raw round-trip buffers — complete payloads preserved for safe write-back
static uint8_t Original_Gov_Config_Bytes[GOV_CONFIG_PAYLOAD_SIZE] = {0};
static uint8_t Original_Gov_Profile_Bytes[GOV_PROFILE_PAYLOAD_SIZE] = {0};

// Polling timers
//static uint32_t Started_Sending_GOV_CONFIG = 0;
//static uint32_t Started_Sending_GOV_PROFILE = 0;
//static const uint32_t GOV_Send_Duration = 2000; // ms

// GOVERNOR_CONFIG fields
// Payload layout (42 bytes) — see byte map in comments below parse function
static uint8_t Gov_Mode = 0;
static uint16_t Gov_Startup_Time = 200;         // raw (/10 = seconds)
static uint16_t Gov_Spoolup_Time = 100;         // raw (/10 = %/s)
static uint16_t Gov_Tracking_Time = 20;         // raw (/10 = %/s)
static uint16_t Gov_Recovery_Time = 20;         // raw (/10 = %/s)
static uint16_t Gov_Throttle_Hold_Timeout = 50; // raw (/10 = seconds)
static uint16_t Gov_Autorotation_Timeout = 0;   // seconds (no scale)
static uint8_t Gov_Handover_Throttle = 20;      // %
static uint8_t Gov_Pwr_Filter = 20;             // Hz
static uint8_t Gov_Rpm_Filter = 20;             // Hz
static uint8_t Gov_Tta_Filter = 0;              // Hz
static uint8_t Gov_Ff_Filter = 10;              // Hz
static uint8_t Gov_D_Filter = 50;               // raw (/10 = Hz)
static uint16_t Gov_Spooldown_Time = 30;        // raw (/10 = %/s)
static uint8_t Gov_Throttle_Type = 0;           // 0=Normal 1=Switch 2=Function
static uint8_t Gov_Idle_Throttle = 10;          // raw (/10 = %)
static uint8_t Gov_Auto_Throttle = 10;          // raw (/10 = %)
static uint8_t Gov_Bypass_Curve[9] = {0, 10, 20, 30, 50, 60, 70, 80, 100};

// GOVERNOR_PROFILE fields
// Payload layout (17 bytes) — see byte map in comments below parse function
static uint16_t Gov_Headspeed = 2000;
static uint8_t Gov_Gain = 100;
static uint8_t Gov_P_Gain = 10;
static uint8_t Gov_I_Gain = 125;
static uint8_t Gov_D_Gain = 5;
static uint8_t Gov_F_Gain = 20;
static uint8_t Gov_TTA_Gain = 0;
static uint8_t Gov_TTA_Limit = 20;
static uint8_t Gov_Yaw_Weight = 10;
static uint8_t Gov_Cyclic_Weight = 40;
static uint8_t Gov_Collective_Weight = 100;
static uint8_t Gov_Max_Throttle = 100;
static uint8_t Gov_Min_Throttle = 10;
static uint8_t Gov_Fallback_Drop = 10;
static uint16_t Gov_Flags = 0;

// ACK payload byte array — packed by PackGovernorForAckPayload()
// and sent to the transmitter in your existing ACK payload mechanism.
static uint8_t GovAckPayload[GOV_ACK_PAYLOAD_SIZE] = {0};

// ====================================================
// Parse — GOVERNOR_CONFIG (MSP read cmd 142)
// ====================================================
// Byte map (42 bytes total):
//  0       gov_mode                U8   0=Off 1=Limit 2=Direct 3=Electric 4=Nitro
//  1-2     gov_startup_time        U16  raw/10 = seconds
//  3-4     gov_spoolup_time        U16  raw/10 = %/s
//  5-6     gov_tracking_time       U16  raw/10 = %/s
//  7-8     gov_recovery_time       U16  raw/10 = %/s
//  9-10    gov_throttle_hold_to    U16  raw/10 = seconds
//  11-12   spare_0                 U16  reserved — round-tripped unchanged
//  13-14   gov_autorotation_to     U16  seconds (no scale)
//  15-16   spare_1                 U16  reserved
//  17-18   spare_2                 U16  reserved
//  19      gov_handover_throttle   U8   %
//  20      gov_pwr_filter          U8   Hz
//  21      gov_rpm_filter          U8   Hz
//  22      gov_tta_filter          U8   Hz
//  23      gov_ff_filter           U8   Hz
//  24      spare_3                 U8   reserved
//  25      gov_d_filter            U8   raw/10 = Hz
//  26-27   gov_spooldown_time      U16  raw/10 = %/s
//  28      gov_throttle_type       U8   0=Normal 1=Switch 2=Function
//  29      spare_4                 S8   reserved
//  30      spare_5                 S8   reserved
//  31      gov_idle_throttle       U8   raw/10 = %
//  32      gov_auto_throttle       U8   raw/10 = %
//  33-41   gov_bypass_curve[9]     U8   bypass throttle curve points

inline bool Parse_MSP_Governor_Config(const uint8_t *data, uint8_t n)
{
    if (api100 < 1209)
    {
        Governor_RF23_Required = true;
        return false;
    }
    Governor_RF23_Required = false;

    MspFrame f;
    if (!FindMspV1ResponseFrameForCmd(data, n, MSP_GOVERNOR_CONFIG, f))
        return false;
    if (f.size < GOV_CONFIG_PAYLOAD_SIZE)
        return false;

    const uint8_t *p = f.payload;

    // Stash complete raw payload — spare bytes preserved for write-back
    for (uint8_t i = 0; i < GOV_CONFIG_PAYLOAD_SIZE; i++)
        Original_Gov_Config_Bytes[i] = p[i];

    uint8_t o = 0;
    Gov_Mode = p[o++];
    Gov_Startup_Time = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;
    Gov_Spoolup_Time = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;
    Gov_Tracking_Time = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;
    Gov_Recovery_Time = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;
    Gov_Throttle_Hold_Timeout = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;
    o += 2; // spare_0
    Gov_Autorotation_Timeout = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;
    o += 2; // spare_1
    o += 2; // spare_2
    Gov_Handover_Throttle = p[o++];
    Gov_Pwr_Filter = p[o++];
    Gov_Rpm_Filter = p[o++];
    Gov_Tta_Filter = p[o++];
    Gov_Ff_Filter = p[o++];
    o++; // spare_3
    Gov_D_Filter = p[o++];
    Gov_Spooldown_Time = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;
    Gov_Throttle_Type = p[o++];
    o++; // spare_4
    o++; // spare_5
    Gov_Idle_Throttle = p[o++];
    Gov_Auto_Throttle = p[o++];
    for (uint8_t i = 0; i < 9; i++)
        Gov_Bypass_Curve[i] = p[o++];

    return true;
}

// ====================================================
// Parse — GOVERNOR_PROFILE (MSP read cmd 148)
// ====================================================
// Byte map (17 bytes total):
//  0-1     governor_headspeed          U16  RPM
//  2       governor_gain               U8
//  3       governor_p_gain             U8
//  4       governor_i_gain             U8
//  5       governor_d_gain             U8
//  6       governor_f_gain             U8
//  7       governor_tta_gain           U8
//  8       governor_tta_limit          U8   %
//  9       governor_yaw_weight         U8   (was yaw_ff_weight in RF 2.2)
//  10      governor_cyclic_weight      U8   (was cyclic_ff_weight in RF 2.2)
//  11      governor_collective_weight  U8   (was collective_ff_weight in RF 2.2)
//  12      governor_max_throttle       U8   %
//  13      governor_min_throttle       U8   %
//  14      governor_fallback_drop      U8   %  (new in RF 2.3)
//  15-16   governor_flags              U16  bitmap — use GOV_FLAG_* defines

inline bool Parse_MSP_Governor_Profile(const uint8_t *data, uint8_t n)
{
    if (api100 < 1209)
    {
        Governor_RF23_Required = true;
        return false;
    }
    Governor_RF23_Required = false;

    MspFrame f;
    if (!FindMspV1ResponseFrameForCmd(data, n, MSP_GOVERNOR_PROFILE, f))
        return false;
    if (f.size < GOV_PROFILE_PAYLOAD_SIZE)
        return false;

    const uint8_t *p = f.payload;
    for (uint8_t i = 0; i < GOV_PROFILE_PAYLOAD_SIZE; i++)
        Original_Gov_Profile_Bytes[i] = p[i];

    uint8_t o = 0;
    Gov_Headspeed = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;
    Gov_Gain = p[o++];
    Gov_P_Gain = p[o++];
    Gov_I_Gain = p[o++];
    Gov_D_Gain = p[o++];
    Gov_F_Gain = p[o++];
    Gov_TTA_Gain = p[o++];
    Gov_TTA_Limit = p[o++];
    Gov_Yaw_Weight = p[o++];
    Gov_Cyclic_Weight = p[o++];
    Gov_Collective_Weight = p[o++];
    Gov_Max_Throttle = p[o++];
    Gov_Min_Throttle = p[o++];
    Gov_Fallback_Drop = p[o++];
    Gov_Flags = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;

    return true;
}

// ====================================================
// Write — GOVERNOR_CONFIG (MSP write cmd 143)
// ====================================================

inline void WriteGovernorConfigToNexusAndSave()
{
    static uint32_t lastWriteTime = 0;
    const uint32_t WRITE_COOLDOWN_MS = 5000;
    uint32_t now = millis();
    if ((now - lastWriteTime < WRITE_COOLDOWN_MS) || (!Rotorflight_Version))
        return;
    if (api100 < 1209)
        return;

    uint8_t payload[GOV_CONFIG_PAYLOAD_SIZE];
    for (uint8_t i = 0; i < GOV_CONFIG_PAYLOAD_SIZE; i++)
        payload[i] = Original_Gov_Config_Bytes[i];

    uint8_t o = 0;
    payload[o++] = Gov_Mode;
    payload[o++] = (uint8_t)(Gov_Startup_Time & 0xFF);
    payload[o++] = (uint8_t)(Gov_Startup_Time >> 8);
    payload[o++] = (uint8_t)(Gov_Spoolup_Time & 0xFF);
    payload[o++] = (uint8_t)(Gov_Spoolup_Time >> 8);
    payload[o++] = (uint8_t)(Gov_Tracking_Time & 0xFF);
    payload[o++] = (uint8_t)(Gov_Tracking_Time >> 8);
    payload[o++] = (uint8_t)(Gov_Recovery_Time & 0xFF);
    payload[o++] = (uint8_t)(Gov_Recovery_Time >> 8);
    payload[o++] = (uint8_t)(Gov_Throttle_Hold_Timeout & 0xFF);
    payload[o++] = (uint8_t)(Gov_Throttle_Hold_Timeout >> 8);
    o += 2; // spare_0 already correct from Original
    payload[o++] = (uint8_t)(Gov_Autorotation_Timeout & 0xFF);
    payload[o++] = (uint8_t)(Gov_Autorotation_Timeout >> 8);
    o += 2; // spare_1
    o += 2; // spare_2
    payload[o++] = Gov_Handover_Throttle;
    payload[o++] = Gov_Pwr_Filter;
    payload[o++] = Gov_Rpm_Filter;
    payload[o++] = Gov_Tta_Filter;
    payload[o++] = Gov_Ff_Filter;
    o++; // spare_3
    payload[o++] = Gov_D_Filter;
    payload[o++] = (uint8_t)(Gov_Spooldown_Time & 0xFF);
    payload[o++] = (uint8_t)(Gov_Spooldown_Time >> 8);
    payload[o++] = Gov_Throttle_Type;
    o++; // spare_4
    o++; // spare_5
    payload[o++] = Gov_Idle_Throttle;
    payload[o++] = Gov_Auto_Throttle;
    for (uint8_t i = 0; i < 9; i++)
        payload[o++] = Gov_Bypass_Curve[i];

    if (o != GOV_CONFIG_PAYLOAD_SIZE)
        return; // defensive

    delay(20);
    This_MSP_Uart->flush();
    SendToMSP(MSP_SET_GOVERNOR_CONFIG, payload, GOV_CONFIG_PAYLOAD_SIZE);
    This_MSP_Uart->flush();
    if (!WaitForMspAck(MSP_SET_GOVERNOR_CONFIG, 300))
        return;
    delay(50);
    SendToMSP(MSP_EEPROM_WRITE, nullptr, 0);
    This_MSP_Uart->flush();
    if (!WaitForMspAck(MSP_EEPROM_WRITE, 1500))
        return;
    lastWriteTime = millis();
}

// ====================================================
// Write — GOVERNOR_PROFILE (MSP write cmd 149)
// ====================================================

inline void WriteGovernorProfileToNexusAndSave()
{
    static uint32_t lastWriteTime = 0;
    const uint32_t WRITE_COOLDOWN_MS = 5000;
    uint32_t now = millis();
    if ((now - lastWriteTime < WRITE_COOLDOWN_MS) || (!Rotorflight_Version))
        return;
    if (api100 < 1209)
        return;

    uint8_t payload[GOV_PROFILE_PAYLOAD_SIZE];
    for (uint8_t i = 0; i < GOV_PROFILE_PAYLOAD_SIZE; i++)
        payload[i] = Original_Gov_Profile_Bytes[i];

    uint8_t o = 0;
    payload[o++] = (uint8_t)(Gov_Headspeed & 0xFF);
    payload[o++] = (uint8_t)(Gov_Headspeed >> 8);
    payload[o++] = Gov_Gain;
    payload[o++] = Gov_P_Gain;
    payload[o++] = Gov_I_Gain;
    payload[o++] = Gov_D_Gain;
    payload[o++] = Gov_F_Gain;
    payload[o++] = Gov_TTA_Gain;
    payload[o++] = Gov_TTA_Limit;
    payload[o++] = Gov_Yaw_Weight;
    payload[o++] = Gov_Cyclic_Weight;
    payload[o++] = Gov_Collective_Weight;
    payload[o++] = Gov_Max_Throttle;
    payload[o++] = Gov_Min_Throttle;
    payload[o++] = Gov_Fallback_Drop;
    payload[o++] = (uint8_t)(Gov_Flags & 0xFF);
    payload[o++] = (uint8_t)(Gov_Flags >> 8);

    if (o != GOV_PROFILE_PAYLOAD_SIZE)
        return;

    delay(20);
    This_MSP_Uart->flush();
    SendToMSP(MSP_SET_GOVERNOR_PROFILE, payload, GOV_PROFILE_PAYLOAD_SIZE);
    This_MSP_Uart->flush();
    if (!WaitForMspAck(MSP_SET_GOVERNOR_PROFILE, 300))
        return;
    delay(50);
    SendToMSP(MSP_EEPROM_WRITE, nullptr, 0);
    This_MSP_Uart->flush();
    if (!WaitForMspAck(MSP_EEPROM_WRITE, 1500))
        return;
    lastWriteTime = millis();
}

// ====================================================
// Pack governor values for ACK payload to transmitter
// ====================================================
// Call this after a successful parse of both config and
// profile. The resulting GovAckPayload[] byte array is
// ready to insert into your existing ACK payload mechanism.
//
// Layout of GovAckPayload[GOV_ACK_PAYLOAD_SIZE] (59 bytes):
//
//  [0]     Governor_RF23_Required  (0 or 1)
//
//  Profile values (matching RFGovView column 1 + col 2 top):
//  [1-2]   Gov_Headspeed           U16 LE
//  [3]     Gov_Gain
//  [4]     Gov_P_Gain
//  [5]     Gov_I_Gain
//  [6]     Gov_D_Gain
//  [7]     Gov_F_Gain
//  [8]     Gov_TTA_Gain
//  [9]     Gov_TTA_Limit
//  [10]    Gov_Max_Throttle
//  [11]    Gov_Min_Throttle
//  [12]    Gov_Fallback_Drop
//  [13]    Gov_Yaw_Weight
//  [14]    Gov_Cyclic_Weight
//  [15]    Gov_Collective_Weight
//  [16-17] Gov_Flags               U16 LE
//
//  Config values (matching RFGovView column 2 + column 3):
//  [18]    Gov_Mode
//  [19]    Gov_Handover_Throttle
//  [20-21] Gov_Startup_Time        U16 LE  (raw, /10 = seconds)
//  [22-23] Gov_Spoolup_Time        U16 LE  (raw, /10 = %/s)
//  [24-25] Gov_Spooldown_Time      U16 LE  (raw, /10 = %/s)
//  [26-27] Gov_Tracking_Time       U16 LE  (raw, /10 = %/s)
//  [28-29] Gov_Recovery_Time       U16 LE  (raw, /10 = %/s)
//  [30-31] Gov_Throttle_Hold_Timeout U16 LE (raw, /10 = seconds)
//  [32-33] Gov_Autorotation_Timeout  U16 LE (seconds, no scale)
//  [34]    Gov_Rpm_Filter
//  [35]    Gov_Pwr_Filter
//  [36]    Gov_D_Filter            (raw, /10 = Hz)
//  [37]    Gov_Ff_Filter
//  [38]    Gov_Tta_Filter
//  [39]    Gov_Throttle_Type
//  [40]    Gov_Idle_Throttle       (raw, /10 = %)
//  [41]    Gov_Auto_Throttle       (raw, /10 = %)
//
//  Flags unpacked as individual bytes for easy display:
//  [42]    GOV_FLAG_VOLTAGE_COMP    (0 or 1)
//  [43]    GOV_FLAG_PID_SPOOLUP     (0 or 1)
//  [44]    GOV_FLAG_FALLBACK_PRECOMP(0 or 1)
//  [45]    GOV_FLAG_DYN_MIN_THROTTLE(0 or 1)
//
//  Spare/future use:
//  [46-58] reserved, set to 0
// ====================================================

inline void PackGovernorForAckPayload()
{
    uint8_t *b = GovAckPayload;

    // Clear entire buffer first
    for (uint8_t i = 0; i < GOV_ACK_PAYLOAD_SIZE; i++)
        b[i] = 0;

    // [0] RF23 required flag
    b[0] = Governor_RF23_Required ? 1 : 0;

    // [1-17] Profile
    b[1] = (uint8_t)(Gov_Headspeed & 0xFF);
    b[2] = (uint8_t)(Gov_Headspeed >> 8);
    b[3] = Gov_Gain;
    b[4] = Gov_P_Gain;
    b[5] = Gov_I_Gain;
    b[6] = Gov_D_Gain;
    b[7] = Gov_F_Gain;
    b[8] = Gov_TTA_Gain;
    b[9] = Gov_TTA_Limit;
    b[10] = Gov_Max_Throttle;
    b[11] = Gov_Min_Throttle;
    b[12] = Gov_Fallback_Drop;
    b[13] = Gov_Yaw_Weight;
    b[14] = Gov_Cyclic_Weight;
    b[15] = Gov_Collective_Weight;
    b[16] = (uint8_t)(Gov_Flags & 0xFF);
    b[17] = (uint8_t)(Gov_Flags >> 8);

    // [18-41] Config
    b[18] = Gov_Mode;
    b[19] = Gov_Handover_Throttle;
    b[20] = (uint8_t)(Gov_Startup_Time & 0xFF);
    b[21] = (uint8_t)(Gov_Startup_Time >> 8);
    b[22] = (uint8_t)(Gov_Spoolup_Time & 0xFF);
    b[23] = (uint8_t)(Gov_Spoolup_Time >> 8);
    b[24] = (uint8_t)(Gov_Spooldown_Time & 0xFF);
    b[25] = (uint8_t)(Gov_Spooldown_Time >> 8);
    b[26] = (uint8_t)(Gov_Tracking_Time & 0xFF);
    b[27] = (uint8_t)(Gov_Tracking_Time >> 8);
    b[28] = (uint8_t)(Gov_Recovery_Time & 0xFF);
    b[29] = (uint8_t)(Gov_Recovery_Time >> 8);
    b[30] = (uint8_t)(Gov_Throttle_Hold_Timeout & 0xFF);
    b[31] = (uint8_t)(Gov_Throttle_Hold_Timeout >> 8);
    b[32] = (uint8_t)(Gov_Autorotation_Timeout & 0xFF);
    b[33] = (uint8_t)(Gov_Autorotation_Timeout >> 8);
    b[34] = Gov_Rpm_Filter;
    b[35] = Gov_Pwr_Filter;
    b[36] = Gov_D_Filter;
    b[37] = Gov_Ff_Filter;
    b[38] = Gov_Tta_Filter;
    b[39] = Gov_Throttle_Type;
    b[40] = Gov_Idle_Throttle;
    b[41] = Gov_Auto_Throttle;

    // [42-45] Flags unpacked
    b[42] = (Gov_Flags & GOV_FLAG_VOLTAGE_COMP) ? 1 : 0;
    b[43] = (Gov_Flags & GOV_FLAG_PID_SPOOLUP) ? 1 : 0;
    b[44] = (Gov_Flags & GOV_FLAG_FALLBACK_PRECOMP) ? 1 : 0;
    b[45] = (Gov_Flags & GOV_FLAG_DYN_MIN_THROTTLE) ? 1 : 0;

    // [46-58] reserved — already zeroed above
}

// ====================================================
// Unpack governor write-request from transmitter ACK
// ====================================================
// When the user edits values on the Nextion and presses
// Save, the transmitter packs the edited values into an
// outgoing payload using the same layout as GovAckPayload
// and sends them back in the next packet to the receiver.
// Call this function to unpack that incoming payload into
// the global variables, then call WriteGovernorProfileToNexusAndSave()
// and/or WriteGovernorConfigToNexusAndSave() as appropriate.
//
// src must point to GOV_ACK_PAYLOAD_SIZE bytes.
// ====================================================

inline void UnpackGovernorFromTxPayload(const uint8_t *src)
{
    // [1-17] Profile
    Gov_Headspeed = (uint16_t)src[1] | ((uint16_t)src[2] << 8);
    Gov_Gain = src[3];
    Gov_P_Gain = src[4];
    Gov_I_Gain = src[5];
    Gov_D_Gain = src[6];
    Gov_F_Gain = src[7];
    Gov_TTA_Gain = src[8];
    Gov_TTA_Limit = src[9];
    Gov_Max_Throttle = src[10];
    Gov_Min_Throttle = src[11];
    Gov_Fallback_Drop = src[12];
    Gov_Yaw_Weight = src[13];
    Gov_Cyclic_Weight = src[14];
    Gov_Collective_Weight = src[15];
    Gov_Flags = (uint16_t)src[16] | ((uint16_t)src[17] << 8);

    // [18-41] Config
    Gov_Mode = src[18];
    Gov_Handover_Throttle = src[19];
    Gov_Startup_Time = (uint16_t)src[20] | ((uint16_t)src[21] << 8);
    Gov_Spoolup_Time = (uint16_t)src[22] | ((uint16_t)src[23] << 8);
    Gov_Spooldown_Time = (uint16_t)src[24] | ((uint16_t)src[25] << 8);
    Gov_Tracking_Time = (uint16_t)src[26] | ((uint16_t)src[27] << 8);
    Gov_Recovery_Time = (uint16_t)src[28] | ((uint16_t)src[29] << 8);
    Gov_Throttle_Hold_Timeout = (uint16_t)src[30] | ((uint16_t)src[31] << 8);
    Gov_Autorotation_Timeout = (uint16_t)src[32] | ((uint16_t)src[33] << 8);
    Gov_Rpm_Filter = src[34];
    Gov_Pwr_Filter = src[35];
    Gov_D_Filter = src[36];
    Gov_Ff_Filter = src[37];
    Gov_Tta_Filter = src[38];
    Gov_Throttle_Type = src[39];
    Gov_Idle_Throttle = src[40];
    Gov_Auto_Throttle = src[41];

    // [42-45] Flags — reconstruct Gov_Flags bitmap from unpacked bits
    Gov_Flags &= ~(GOV_FLAG_VOLTAGE_COMP | GOV_FLAG_PID_SPOOLUP |
                   GOV_FLAG_FALLBACK_PRECOMP | GOV_FLAG_DYN_MIN_THROTTLE);
    if (src[42])
        Gov_Flags |= GOV_FLAG_VOLTAGE_COMP;
    if (src[43])
        Gov_Flags |= GOV_FLAG_PID_SPOOLUP;
    if (src[44])
        Gov_Flags |= GOV_FLAG_FALLBACK_PRECOMP;
    if (src[45])
        Gov_Flags |= GOV_FLAG_DYN_MIN_THROTTLE;
}

// ************************************************************************************************************

#endif // NEXUS_H