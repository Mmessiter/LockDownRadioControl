/** @file ReceiverCode/src/utilities/Nexus.h */
// Malcolm Messiter 2020 - 2025
#ifndef NEXUS_H
#define NEXUS_H
#include "utilities/1Definitions.h"

// ************************************************************************************************************
// MSP SUPPORT FUNCTIONS FOR NEXUS etc.
// ************************************************************************************************************
#define MSP_MOTOR_TELEMETRY 139 // Motor telemetry data
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
        if (Get_MSP_Motor_Telemetry(&buf[0], p))
        {
            NexusPresent = true;
            return;
        }
    }
    MSP_UART.end();
    NexusPresent = false;
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

// *************************************************************************************************************

//         MSP_MOTOR_TELEMETRY FROM ROTORFLIGHT FIRMWARE
//         uint32_t motorRpm = 0;                        // offset 0
//         uint16_t errorRatio = 0;                      // offset 5
//         uint16_t escVoltage = 0;      // 1mV per unit // offset 7
//         uint16_t escCurrent = 0;      // 1mA per unit // offset 9
//         uint16_t escConsumption = 0;  // mAh          // offset 11
//         uint16_t escTemperature = 0;  // 0.1C         // offset 13
//         uint16_t escTemperature2 = 0; // 0.1C         // offset 15

inline bool Get_MSP_Motor_Telemetry(const uint8_t *data, uint8_t n)
{
    const uint8_t CMD = MSP_MOTOR_TELEMETRY; // 0x8B
    if (!Ratio)
        Ratio = 10.3f; // Default ratio if not set
    for (uint8_t i = 0; i + 6 < n; ++i)
    {
        if (data[i] != '$' || data[i + 1] != 'M' || data[i + 2] != '>')
            continue;
        uint8_t size = data[i + 3];
        uint8_t cmd = data[i + 4];
        uint16_t frame_end = i + 5 + size;
        if (frame_end >= n)
            break;
        uint8_t ck = size ^ cmd;
        for (uint8_t k = 0; k < size; ++k)
            ck ^= data[i + 5 + k];
        if (ck != data[frame_end])
            continue;
        if (cmd != CMD)
            continue;
        if (size < 3)
            continue;
        const uint8_t *payload = &data[i + 5];
        if (size >= 16)
        {
           RotorRPM = ((uint32_t)payload[1] | ((uint32_t)payload[2] << 8) | ((uint32_t)payload[3] << 16)) / Ratio; // Ignore payload[0] which is motor index???
           RXModelVolts = (float)((uint16_t)payload[7] | ((uint16_t)payload[8] << 8)) / 1000.00f; // divide by 1000 to get volts 
           Battery_Amps = (float)((uint16_t)payload[9] | ((uint16_t)payload[10] << 8)) / 1000.00f;
           Battery_mAh = (float)((uint16_t)payload[11] | ((uint16_t)payload[12] << 8));
           escTempC = (float)((uint16_t)payload[13] | ((uint16_t)payload[14] << 8)) / 10.00f;
           if (RXModelVolts > 26) // 6S max volt is 25.2V so anything above that must be 12S
               RXModelVolts /= 2; // must be 12S battery and the TX will double the voltage reading because INA219 can't handle above 26V
           return true;
        }
    }
    return false;
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
    Get_MSP_Motor_Telemetry(&data_in[0], p);
    RequestFromMSP(MSP_MOTOR_TELEMETRY);
}
#endif // NEXUS_H