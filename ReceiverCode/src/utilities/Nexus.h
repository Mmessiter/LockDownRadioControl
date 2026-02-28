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
// #define MSP_STATUS_EX 150       // extended status with more telemetry data WRONG

// Add defines for send states
#define SEND_NO_RF 0
#define SEND_PID_RF 1
#define SEND_RATES_RF 2
#define SEND_RATES_ADVANCED_RF 3
#define SEND_PID_ADVANCED_RF 4

// ************************************************************************************************************
// Detect if Rotorflight 2.2 is present at boot time
// If detected, fetch API version and protocol version
// Keeps the MSP_UART open if Rotorflight 2.2 is present
// Closes it if not present
// ************************************************************************************************************

inline void DetectRotorFlightAtBoot()
{

#define Ports_Count 2
   // uint32_t LocalTimer = millis();
   // char Pnames[Ports_Count][10] = {"Serial6", "Serial1"};
    HardwareSerial *Ports[Ports_Count] = {&Serial6, &Serial1};
    for (uint8_t i = 0; i < Ports_Count; i++)
    {
        This_MSP_Uart = Ports[i];
        DetectRotorFlightAtBoot1();
        if (Rotorflight22Detected)
        {
            // Look1("Rotorflight 2.2 detected on ");
            // Look(Pnames[i]);
            // Look1("After ");
            // Look1((float)(millis() - LocalTimer)/1000);
            // Look(" seconds");  
            return;
        }
        // Look1("Rotorflight 2.2 not detected on ");
        // Look(Pnames[i]);
        // Look1("After ");        
        // Look1((float)(millis() - LocalTimer)/1000);
        // Look(" seconds");
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

    Rotorflight22Detected = false;

    while (millis() - start < NEXUS_DETECT_WINDOW_MS)
    {
        RequestFromMSP(MSP_MOTOR_TELEMETRY);
        delay(20);

        uint8_t p = 0;
        while (This_MSP_Uart->available() && p < sizeof(buf))
            buf[p++] = (uint8_t)This_MSP_Uart->read();
        if (Parse_MSP_Motor_Telemetry(buf, p))
        {
            // Now fetch API version
            RequestFromMSP(MSP_API_VERSION);
            delay(20);

            p = 0;
            while (This_MSP_Uart->available() && p < sizeof(buf))
                buf[p++] = (uint8_t)This_MSP_Uart->read();

            uint8_t apiMaj = 0, apiMin = 0, proto = 0;
            if (Parse_MSP_API_VERSION(buf, p, proto, apiMaj, apiMin))
            {
                if (api100 >= 1208) // (x 100 to deal with floats more easily)
                    Rotorflight22Detected = true;
            }
            return; // keep UART open
        }
    }
    This_MSP_Uart->end();
    Rotorflight22Detected = false;
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

// ************************************************************************************************************
// ACK / reply wait support (prevents EEPROM save racing ahead of SET command processing)
// ************************************************************************************************************
static volatile bool MspAckWaitInProgress = false;

// Waits for an MSPv1 reply frame ($M> or $M!) for the specified command.
// Returns true only if the reply was $M> for expectedCmd.
// Returns false on timeout or $M! error for expectedCmd.
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
    uint8_t dir = 0; // '>' or '!'
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
                {
                    state = ST_IDLE;
                }
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
            {
                // c is received checksum
                if (checksum == c)
                {
                    if (cmd == expectedCmd)
                    {
                        MspAckWaitInProgress = false;
                        return (dir == '>');
                    }
                    // else valid frame, but not the one we are waiting for -> ignore and keep waiting
                }
                state = ST_IDLE;
                break;
            }

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
/// ************************************************************************************************************

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
        api100 = (uint16_t)apiMaj * 100u + (uint16_t)apiMin; // 12.08 -> 1208
        return true;
    }
    return false;
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
            continue; // header found but frame not complete in buffer; keep scanning

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
// Finds the *first* valid MSPv1 response frame ($M>) for a SPECIFIC cmd.
// This fixes the problem you saw: your buffer can contain multiple valid frames, so f.cmd might be "39" etc.
inline bool FindMspV1ResponseFrameForCmd(const uint8_t *data, uint8_t n, uint8_t wantCmd, MspFrame &out)
{
    for (uint8_t i = 0; i + 6 < n; ++i)
    {
        if (data[i] != '$' || data[i + 1] != 'M' || data[i + 2] != '>')
            continue;

        uint8_t size = data[i + 3];
        uint8_t cmd = data[i + 4];

        uint16_t frame_end = i + 5 + size; // index of checksum byte
        if (frame_end >= n)
            continue; // frame not complete yet, keep scanning

        uint8_t ck = size ^ cmd;
        for (uint8_t k = 0; k < size; ++k)
            ck ^= data[i + 5 + k];

        if (ck != data[frame_end])
            continue; // bad checksum

        if (cmd != wantCmd)
            continue; // valid frame, but not the one we want

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
// Send an MSPv1 COMMAND with a payload (this is what we need for SET_PID + EEPROM_WRITE) ***
// This transmits: $ M <  [size] [cmd] [payload bytes...] [checksum]
// - This only "sends". It does NOT wait for a reply.
// - Many flight controllers will ignore writes if ARMED. So do this only when disarmed.
//
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
// Write the given array of 17 PID values to Nexus via MSP, and then save to EEPROM/flash.
inline void WritePIDsToNexusAndSave(const uint16_t pid[17])
{
    static uint32_t lastWriteTime = 0;
    const uint32_t WRITE_COOLDOWN_MS = 5000; // 5 seconds
    uint32_t now = millis();

    if ((now - lastWriteTime < WRITE_COOLDOWN_MS) || (!Rotorflight22Detected))
        return;

    uint8_t payload[34] = {0};

    for (int i = 0; i < 34; i++)
        payload[i] = Original_PID_Values[i]; // start with original values

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

    lastWriteTime = millis(); // set cooldown only after confirmed write+save
                              //  InhibitTelemetry = false; // re-enable telemetry after successful write+save
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

inline bool Parse_MSP_PID(const uint8_t *data, uint8_t n)
{
    MspFrame f;
    if (!FindMspV1ResponseFrameForCmd(data, n, MSP_PID, f))
        return false;

    if (f.size < 34) // 34 bytes expected
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
    {
        Original_PID_Values[i] = p[i];
    }

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
    if (!FindMspV1ResponseFrameForCmd(data, n, MSP_MOTOR_TELEMETRY, f))
        return false;

    const uint8_t size = f.size;
    const uint8_t *payload = f.payload;

    if (size < 15)
        return false;

    if (payload[0] != 1)
        return false;

    RotorRPM = ((uint32_t)payload[1] |
                ((uint32_t)payload[2] << 8) |
                ((uint32_t)payload[3] << 16) |
                ((uint32_t)payload[4] << 24)) /
               Ratio;
    RXModelVolts = (float)((uint16_t)payload[7] | ((uint16_t)payload[8] << 8)) / 1000.0f;
    Battery_Amps = (float)((uint16_t)payload[9] | ((uint16_t)payload[10] << 8)) / 1000.0f;
    Battery_mAh = (float)((uint16_t)payload[11] | ((uint16_t)payload[12] << 8));
    ESC_Temp_C = (float)((uint16_t)payload[13] | ((uint16_t)payload[14] << 8)) / 10.0f;

    if (RXModelVolts > 26.0f)
        RXModelVolts /= 2.0f;

    return true;
}

// ************************************************************************************************************/
inline void CheckMSPSerial()
{
    if (MspAckWaitInProgress)
        return; // prevent this poller from stealing reply bytes during a write/save transaction
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
        {
            data_in[p++] = c;
        }
        else
        {
            overflow = true;
        }
    }
    if (overflow)
    {
        return;
    }
    bool looksLikeMSP =
        (p >= 6 && data_in[0] == '$' && data_in[1] == 'M') ||
        (p >= 8 && data_in[0] == '$' && data_in[1] == 'X');

    if (!looksLikeMSP)
    {
        switch (SendRotorFlightParametresNow)
        {
        case SEND_NO_RF:
            if (InhibitTelemetry)
            {
                break;
            }
            else
            {
                RequestFromMSP(MSP_MOTOR_TELEMETRY);
            }
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
        default:
            break;
        }
        return;
    }
    if ((Now - Started_Sending_PIDs > PID_Send_Duration) && (SendRotorFlightParametresNow == SEND_PID_RF))
    {
        SendRotorFlightParametresNow = SEND_NO_RF;
    }
    if ((Now - Started_Sending_RATEs > RATES_Send_Duration) && (SendRotorFlightParametresNow == SEND_RATES_RF))
    {
        SendRotorFlightParametresNow = SEND_NO_RF;
    }
    if ((Now - Started_Sending_RATES_ADVANCED > RATES_ADVANCED_Send_Duration) && (SendRotorFlightParametresNow == SEND_RATES_ADVANCED_RF))
    {
        SendRotorFlightParametresNow = SEND_NO_RF;
    }
    if ((Now - Started_Sending_PID_ADVANCED > PID_ADVANCED_Send_Duration) && (SendRotorFlightParametresNow == SEND_PID_ADVANCED_RF))
    {
        SendRotorFlightParametresNow = SEND_NO_RF;
    }
    if ((Now - Started_Sending_STATUS_EX > STATUS_EX_Send_Duration) && (SendRotorFlightParametresNow == SEND_STATUS_EX))
    {
        SendRotorFlightParametresNow = SEND_NO_RF;
    }

    switch (SendRotorFlightParametresNow)
    {
    case SEND_NO_RF:
        if (InhibitTelemetry)
        {
            break;
        }
        else
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

        //  case SEND_STATUS_EX:
        //      Parse_MSP_STATUS_EX(data_in, p);
        //      RequestFromMSP(MSP_STATUS_EX); // parse reply next time around
        //      break;
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

// ************************************************************************************************************

inline void WriteRatesToNexusAndSave()
{
    static uint32_t lastWriteTime = 0;
    const uint32_t WRITE_COOLDOWN_MS = 5000;
    uint32_t now = millis();

    if ((now - lastWriteTime < WRITE_COOLDOWN_MS) || (!Rotorflight22Detected))
        return;
    delay(20); // optional: give FC a breather before sending
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
        return; // defensive: never send malformed payload

    SendToMSP(MSP_SET_RC_TUNING, payload, payload_size);
    This_MSP_Uart->flush();
    if (!WaitForMspAck(MSP_SET_RC_TUNING, 600))
    {
        // Look("MSP_SET_RC_TUNING Ack Failed!");
        return;
    }
    // Look("MSP_SET_RC_TUNING Ack Success!");

    delay(50); // optional: give FC a breather before EEPROM write

    SendToMSP(MSP_EEPROM_WRITE, nullptr, 0);
    This_MSP_Uart->flush();
    if (!WaitForMspAck(MSP_EEPROM_WRITE, 1500))
    {
        //   Look("MSP_EEPROM_WRITE Ack Failed!");
        return;
    }
    // Look("MSP_EEPROM_WRITE Ack Success!");

    lastWriteTime = millis();
    //  InhibitTelemetry = false; // re-enable telemetry after successful write+save
}

// ************************************************************************************************************
//         MSP_PID_PROFILE FROM ROTORFLIGHT FIRMWARE

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

// ************************************************************************************************************

inline void WritePIDAdvancedToNexusAndSave()
{
    static uint32_t lastWriteTime = 0;
    const uint32_t WRITE_COOLDOWN_MS = 5000;
    uint32_t now = millis();

    if ((now - lastWriteTime < WRITE_COOLDOWN_MS) || (!Rotorflight22Detected))
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
    // InhibitTelemetry = false; // re-enable telemetry after successful write+save
}

// ************************************************************************************************************

inline void SetNexusProfile(uint8_t index)
{
    if (!Rotorflight22Detected)
        return;

    SendToMSP(MSP_SELECT_SETTING, &index, 1);
    This_MSP_Uart->flush();
    (void)WaitForMspAck(MSP_SELECT_SETTING, 300); // optional but useful
}

// ************************************************************************************************************

#endif // NEXUS_H