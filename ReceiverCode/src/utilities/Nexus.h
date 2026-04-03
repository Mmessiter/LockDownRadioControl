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
// SEND_GOV_CONFIG_RF and SEND_GOV_PROFILE_RF defined in 1Definitions.h

// Governor flags bitmap helpers
#define GOV_FLAG_FALLBACK_PRECOMP (1u << 2)
#define GOV_FLAG_VOLTAGE_COMP (1u << 3)
#define GOV_FLAG_PID_SPOOLUP (1u << 4)
#define GOV_FLAG_DYN_MIN_THROTTLE (1u << 6)

// Governor payload sizes for api100 >= 1209 (firmware 4.6.0)
#define GOV_CONFIG_PAYLOAD_SIZE 42
#define GOV_PROFILE_PAYLOAD_SIZE 17

// GOV_ACK_PAYLOAD_SIZE defined in 1Definitions.h

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
                    Rotorflight_Version = 1;
                if (api100 >= 1209)
                    Rotorflight_Version = 2;
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
    uint8_t state = ST_IDLE, dir = 0, size = 0, cmd = 0, idx = 0, checksum = 0;
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
        const uint8_t size = buf[i + 3], cmd = buf[i + 4];
        if (cmd != MSP_API_VERSION || size < 3 || i + 6 + size > len)
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
    uint8_t size, cmd;
    const uint8_t *payload;
};

inline bool FindMspV1ResponseFrame(const uint8_t *data, uint8_t n, MspFrame &out)
{
    for (uint8_t i = 0; i + 6 < n; ++i)
    {
        if (data[i] != '$' || data[i + 1] != 'M' || data[i + 2] != '>')
            continue;
        uint8_t size = data[i + 3], cmd = data[i + 4];
        uint16_t fe = i + 5 + size;
        if (fe >= n)
            continue;
        uint8_t ck = size ^ cmd;
        for (uint8_t k = 0; k < size; ++k)
            ck ^= data[i + 5 + k];
        if (ck != data[fe])
            continue;
        out.size = size;
        out.cmd = cmd;
        out.payload = &data[i + 5];
        return true;
    }
    return false;
}

inline bool FindMspV1ResponseFrameForCmd(const uint8_t *data, uint8_t n, uint8_t wantCmd, MspFrame &out)
{
    for (uint8_t i = 0; i + 6 < n; ++i)
    {
        if (data[i] != '$' || data[i + 1] != 'M' || data[i + 2] != '>')
            continue;
        uint8_t size = data[i + 3], cmd = data[i + 4];
        uint16_t fe = i + 5 + size;
        if (fe >= n)
            continue;
        uint8_t ck = size ^ cmd;
        for (uint8_t k = 0; k < size; ++k)
            ck ^= data[i + 5 + k];
        if (ck != data[fe])
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
    uint32_t now = millis();
    if ((now - lastWriteTime < 5000) || (!Rotorflight_Version))
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
inline bool Parse_MSP_PID(const uint8_t *data, uint8_t n)
{
    MspFrame f;
    if (!FindMspV1ResponseFrameForCmd(data, n, MSP_PID, f) || f.size < 34)
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
inline bool Parse_MSP_Motor_Telemetry(const uint8_t *data, uint8_t n)
{
    if (!Ratio)
        Ratio = 10.3f;
    MspFrame f;
    if (!FindMspV1ResponseFrameForCmd(data, n, MSP_MOTOR_TELEMETRY, f) || f.size < 15)
        return false;
    if (f.payload[0] != 1)
        return false;
    const uint8_t *payload = f.payload;
    RotorRPM = ((uint32_t)payload[1] | ((uint32_t)payload[2] << 8) | ((uint32_t)payload[3] << 16) | ((uint32_t)payload[4] << 24)) / Ratio;
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

    // For config phase, wait briefly to allow cmd 142 response to arrive after cmd 148 push
    if (SendRotorFlightParametresNow == SEND_GOV_CONFIG_RF)
        delay(20);

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

    // GOV_CONFIG timeout must be checked here — sync parse runs in !looksLikeMSP branch
    if ((Now - Started_Sending_GOV_CONFIG > GOV_Config_Send_Duration) && (SendRotorFlightParametresNow == SEND_GOV_CONFIG_RF))
        SendRotorFlightParametresNow = SEND_NO_RF;

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
        case SEND_GOV_PROFILE_RF:
            RequestFromMSP(MSP_GOVERNOR_PROFILE);
            break;
        case SEND_GOV_CONFIG_RF:
            RequestFromMSP(MSP_GOVERNOR_CONFIG);
            delay(20);
            {
                uint8_t buf[180];
                uint8_t bp = 0;
                while (This_MSP_Uart->available() && bp < sizeof(buf))
                    buf[bp++] = This_MSP_Uart->read();
                MspFrame ff;
                if (FindMspV1ResponseFrameForCmd(buf, bp, MSP_GOVERNOR_CONFIG, ff))
                    Parse_MSP_Governor_Config(buf, bp);
            }
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
    if ((Now - Started_Sending_GOV_CONFIG > GOV_Config_Send_Duration) && (SendRotorFlightParametresNow == SEND_GOV_CONFIG_RF))
        SendRotorFlightParametresNow = SEND_NO_RF;
    if ((Now - Started_Sending_GOV_PROFILE > PROFILE_Send_Duration) && (SendRotorFlightParametresNow == SEND_GOV_PROFILE_RF))
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
    case SEND_GOV_PROFILE_RF:
        // Phase 1 — FC pushes cmd 148 continuously, just parse it
        Parse_MSP_Governor_Profile(data_in, p);
        RequestFromMSP(MSP_GOVERNOR_PROFILE);
        break;
    case SEND_GOV_CONFIG_RF:
        // Already handled synchronously in !looksLikeMSP branch above
        // But if we get here with data, scan for cmd 142 anyway
        {
            MspFrame fCfg;
            if (FindMspV1ResponseFrameForCmd(data_in, (uint8_t)p, MSP_GOVERNOR_CONFIG, fCfg))
                Parse_MSP_Governor_Config(data_in, (uint8_t)p);
        }
        break;
    default:
        break;
    }
}

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
    uint8_t o = 0;
    Rates_Type = p[o++];
    Roll_Centre_Rate = p[o++];
    Roll_Expo = p[o++];
    Roll_Max_Rate = p[o++];
    Roll_Response_Time = p[o++];
    Roll_Accel_Limit = p[o] | (p[o + 1] << 8);
    o += 2;
    Pitch_Centre_Rate = p[o++];
    Pitch_Expo = p[o++];
    Pitch_Max_Rate = p[o++];
    Pitch_Response_Time = p[o++];
    Pitch_Accel_Limit = p[o] | (p[o + 1] << 8);
    o += 2;
    Yaw_Centre_Rate = p[o++];
    Yaw_Expo = p[o++];
    Yaw_Max_Rate = p[o++];
    Yaw_Response_Time = p[o++];
    Yaw_Accel_Limit = p[o] | (p[o + 1] << 8);
    o += 2;
    Collective_Centre_Rate = p[o++];
    Collective_Expo = p[o++];
    Collective_Max_Rate = p[o++];
    Collective_Response_Time = p[o++];
    Collective_Accel_Limit = p[o] | (p[o + 1] << 8);
    o += 2;
    if (api100 >= 1208)
    {
        Roll_Setpoint_Boost_Gain = p[o++];
        Roll_Setpoint_Boost_Cutoff = p[o++];
        Pitch_Setpoint_Boost_Gain = p[o++];
        Pitch_Setpoint_Boost_Cutoff = p[o++];
        Yaw_Setpoint_Boost_Gain = p[o++];
        Yaw_Setpoint_Boost_Cutoff = p[o++];
        Collective_Setpoint_Boost_Gain = p[o++];
        Collective_Setpoint_Boost_Cutoff = p[o++];
        Yaw_Dynamic_Ceiling_Gain = p[o++];
        Yaw_Dynamic_Deadband_Gain = p[o++];
        Yaw_Dynamic_Deadband_Filter = p[o++];
    }
    StoreRatesBytesForAckPayload();
    StoreAdvancedRatesBytesForAckPayload();
    return true;
}

inline void WriteRatesToNexusAndSave()
{
    static uint32_t lastWriteTime = 0;
    uint32_t now = millis();
    if ((now - lastWriteTime < 5000) || (!Rotorflight_Version))
        return;
    delay(20);
    This_MSP_Uart->flush();
    uint8_t payload_size = 25;
    if (api100 >= 1208)
        payload_size += 11;
    uint8_t payload[36] = {0};
    uint8_t o = 0;
    payload[o++] = Rates_Type;
    payload[o++] = (uint8_t)Roll_Centre_Rate;
    payload[o++] = (uint8_t)Roll_Expo;
    payload[o++] = (uint8_t)Roll_Max_Rate;
    payload[o++] = Roll_Response_Time;
    payload[o++] = (uint8_t)(Roll_Accel_Limit & 0xFF);
    payload[o++] = (uint8_t)(Roll_Accel_Limit >> 8);
    payload[o++] = (uint8_t)Pitch_Centre_Rate;
    payload[o++] = (uint8_t)Pitch_Expo;
    payload[o++] = (uint8_t)Pitch_Max_Rate;
    payload[o++] = Pitch_Response_Time;
    payload[o++] = (uint8_t)(Pitch_Accel_Limit & 0xFF);
    payload[o++] = (uint8_t)(Pitch_Accel_Limit >> 8);
    payload[o++] = (uint8_t)Yaw_Centre_Rate;
    payload[o++] = (uint8_t)Yaw_Expo;
    payload[o++] = (uint8_t)Yaw_Max_Rate;
    payload[o++] = Yaw_Response_Time;
    payload[o++] = (uint8_t)(Yaw_Accel_Limit & 0xFF);
    payload[o++] = (uint8_t)(Yaw_Accel_Limit >> 8);
    payload[o++] = (uint8_t)Collective_Centre_Rate;
    payload[o++] = (uint8_t)Collective_Expo;
    payload[o++] = (uint8_t)Collective_Max_Rate;
    payload[o++] = Collective_Response_Time;
    payload[o++] = (uint8_t)(Collective_Accel_Limit & 0xFF);
    payload[o++] = (uint8_t)(Collective_Accel_Limit >> 8);
    if (api100 >= 1208)
    {
        payload[o++] = Roll_Setpoint_Boost_Gain;
        payload[o++] = Roll_Setpoint_Boost_Cutoff;
        payload[o++] = Pitch_Setpoint_Boost_Gain;
        payload[o++] = Pitch_Setpoint_Boost_Cutoff;
        payload[o++] = Yaw_Setpoint_Boost_Gain;
        payload[o++] = Yaw_Setpoint_Boost_Cutoff;
        payload[o++] = Collective_Setpoint_Boost_Gain;
        payload[o++] = Collective_Setpoint_Boost_Cutoff;
        payload[o++] = Yaw_Dynamic_Ceiling_Gain;
        payload[o++] = Yaw_Dynamic_Deadband_Gain;
        payload[o++] = Yaw_Dynamic_Deadband_Filter;
    }
    if (o != payload_size)
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
inline bool Parse_MSP_PID_PROFILE(const uint8_t *data, uint8_t n)
{
    MspFrame f;
    if (!FindMspV1ResponseFrameForCmd(data, n, MSP_PID_PROFILE, f) || f.size < MAX_PID_ADVANCED_BYTES)
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
    uint32_t now = millis();
    if ((now - lastWriteTime < 5000) || (!Rotorflight_Version))
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
//         GOVERNOR MSP SUPPORT — ROTORFLIGHT 2.3+ ONLY (api100 >= 1209)
//         TX already guards against pre-2.3 so no api100 checks needed here
// ************************************************************************************************************

static uint8_t Original_Gov_Config_Bytes[GOV_CONFIG_PAYLOAD_SIZE] = {0};
static uint8_t Original_Gov_Profile_Bytes[GOV_PROFILE_PAYLOAD_SIZE] = {0};

// GOVERNOR_CONFIG fields
static uint8_t Gov_Mode = 0;
static uint16_t Gov_Startup_Time = 200;
static uint16_t Gov_Spoolup_Time = 100;
static uint16_t Gov_Tracking_Time = 20;
static uint16_t Gov_Recovery_Time = 20;
static uint16_t Gov_Throttle_Hold_Timeout = 50;
static uint16_t Gov_Lost_Headspeed_Timeout = 10;
static uint16_t Gov_Autorotation_Timeout = 0;
static uint16_t Gov_Autorotation_Bailout_Time = 0;
static uint16_t Gov_Autorotation_Min_Entry_Time = 0;
static uint8_t Gov_Handover_Throttle = 20;
static uint8_t Gov_Pwr_Filter = 20;
static uint8_t Gov_Rpm_Filter = 20;
static uint8_t Gov_Tta_Filter = 0;
static uint8_t Gov_Ff_Filter = 10;
static uint8_t Gov_Spoolup_Min_Throttle = 5;
static uint8_t Gov_D_Filter = 50;
static uint16_t Gov_Spooldown_Time = 30;
static uint8_t Gov_Throttle_Type = 0;
static uint8_t Gov_Idle_Throttle = 10;
static uint8_t Gov_Auto_Throttle = 10;
static uint8_t Gov_Bypass_Curve[9] = {0, 10, 20, 30, 50, 60, 70, 80, 100};

// GOVERNOR_PROFILE fields
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

// ====================================================
inline void PackGovernorForAckPayload()
{
    uint8_t *b = GovAckPayload;
    Look1("GovMode=");
    Look(Gov_Mode);
    // Only zero unused tail — all used bytes explicitly written below
    for (uint8_t i = 46; i < GOV_ACK_PAYLOAD_SIZE; i++)
        b[i] = 0;

    b[0] = Governor_RF23_Required ? 1 : 0;
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
    b[42] = (Gov_Flags & GOV_FLAG_VOLTAGE_COMP) ? 1 : 0;
    b[43] = (Gov_Flags & GOV_FLAG_PID_SPOOLUP) ? 1 : 0;
    b[44] = (Gov_Flags & GOV_FLAG_FALLBACK_PRECOMP) ? 1 : 0;
    b[45] = (Gov_Flags & GOV_FLAG_DYN_MIN_THROTTLE) ? 1 : 0;
    Look1("Pack b18=");
    Look(b[18]);
}

// ====================================================
inline bool Parse_MSP_Governor_Config(const uint8_t *data, uint8_t n)
{
    MspFrame f;
    if (!FindMspV1ResponseFrameForCmd(data, n, MSP_GOVERNOR_CONFIG, f))
        return false;
    if (f.size < GOV_CONFIG_PAYLOAD_SIZE)
        return false;

    const uint8_t *p = f.payload;
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
    Gov_Lost_Headspeed_Timeout = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;
    Gov_Autorotation_Timeout = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;
    Gov_Autorotation_Bailout_Time = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;
    Gov_Autorotation_Min_Entry_Time = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;
    Gov_Handover_Throttle = p[o++];
    Gov_Pwr_Filter = p[o++];
    Gov_Rpm_Filter = p[o++];
    Gov_Tta_Filter = p[o++];
    Gov_Ff_Filter = p[o++];
    Gov_Spoolup_Min_Throttle = p[o++];
    Gov_D_Filter = p[o++];
    Gov_Spooldown_Time = (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
    o += 2;
    Gov_Throttle_Type = p[o++];
    o++;
    o++; // spare, spare
    Gov_Idle_Throttle = p[o++];
    Gov_Auto_Throttle = p[o++];
    for (uint8_t i = 0; i < 9; i++)
        Gov_Bypass_Curve[i] = p[o++];

    PackGovernorForAckPayload();
    return true;
}

// ====================================================
inline bool Parse_MSP_Governor_Profile(const uint8_t *data, uint8_t n)
{
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
    PackGovernorForAckPayload();
    return true;
}

// ====================================================
inline void WriteGovernorConfigToNexusAndSave()
{
    static uint32_t lastWriteTime = 0;
    uint32_t now = millis();
    if ((now - lastWriteTime < 5000) || (!Rotorflight_Version))
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
    payload[o++] = (uint8_t)(Gov_Lost_Headspeed_Timeout & 0xFF);
    payload[o++] = (uint8_t)(Gov_Lost_Headspeed_Timeout >> 8);
    payload[o++] = (uint8_t)(Gov_Autorotation_Timeout & 0xFF);
    payload[o++] = (uint8_t)(Gov_Autorotation_Timeout >> 8);
    payload[o++] = (uint8_t)(Gov_Autorotation_Bailout_Time & 0xFF);
    payload[o++] = (uint8_t)(Gov_Autorotation_Bailout_Time >> 8);
    payload[o++] = (uint8_t)(Gov_Autorotation_Min_Entry_Time & 0xFF);
    payload[o++] = (uint8_t)(Gov_Autorotation_Min_Entry_Time >> 8);
    payload[o++] = Gov_Handover_Throttle;
    payload[o++] = Gov_Pwr_Filter;
    payload[o++] = Gov_Rpm_Filter;
    payload[o++] = Gov_Tta_Filter;
    payload[o++] = Gov_Ff_Filter;
    payload[o++] = Gov_Spoolup_Min_Throttle;
    payload[o++] = Gov_D_Filter;
    payload[o++] = (uint8_t)(Gov_Spooldown_Time & 0xFF);
    payload[o++] = (uint8_t)(Gov_Spooldown_Time >> 8);
    payload[o++] = Gov_Throttle_Type;
    o++;
    o++; // spare, spare
    payload[o++] = Gov_Idle_Throttle;
    payload[o++] = Gov_Auto_Throttle;
    for (uint8_t i = 0; i < 9; i++)
        payload[o++] = Gov_Bypass_Curve[i];
    if (o != GOV_CONFIG_PAYLOAD_SIZE)
        return;
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
inline void WriteGovernorProfileToNexusAndSave()
{
    static uint32_t lastWriteTime = 0;
    uint32_t now = millis();
    if ((now - lastWriteTime < 5000) || (!Rotorflight_Version))
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
inline void UnpackGovernorFromTxPayload(const uint8_t *src)
{
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
    Gov_Flags &= ~(GOV_FLAG_VOLTAGE_COMP | GOV_FLAG_PID_SPOOLUP | GOV_FLAG_FALLBACK_PRECOMP | GOV_FLAG_DYN_MIN_THROTTLE);
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
