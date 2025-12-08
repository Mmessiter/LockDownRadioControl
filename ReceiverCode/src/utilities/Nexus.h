/** @file ReceiverCode/src/utilities/Nexus.h */
// Malcolm Messiter 2020 - 2025
#ifndef NEXUS_H
#define NEXUS_H
#include "utilities/1Definitions.h"

// ************************************************************************************************************
// NEXUS TELEMETRY SUPPORT FUNCTIONS
// ************************************************************************************************************

#define MSP_MOTOR_TELEMETRY 139 // Motor telemetry data
#define MSP_ANALOG 110          // vbat, mAh, RSSI, amps

// ************************************************************************************************************
void DetectNexusAtBoot()
{
#define NEXUS_DETECT_WINDOW_MS 1500 // 1.5 seconds time window to detect Nexus at boot

    NEXUS_SERIAL_TELEMETRY.begin(115200);
    uint32_t start = millis();
    uint8_t buf[80];

    while (millis() - start < NEXUS_DETECT_WINDOW_MS)
    {
        requestAnalog();
        delay(20); // give it a moment to reply
        uint8_t p = 0;
        while (NEXUS_SERIAL_TELEMETRY.available() && p < sizeof(buf))
        {
            buf[p++] = NEXUS_SERIAL_TELEMETRY.read();
        }
        float vbat, amps;
        uint16_t mAh;
        if (GetAnalog(buf, p, vbat, amps, mAh))
        {
            NexusPresent = true;
            return;
        }
    }
    NEXUS_SERIAL_TELEMETRY.end();
    NexusPresent = false;
}

// ************************************************************************************************************

void requestRPM()
{
    uint8_t checksum = 0;
    NEXUS_SERIAL_TELEMETRY.write('$');
    NEXUS_SERIAL_TELEMETRY.write('M');
    NEXUS_SERIAL_TELEMETRY.write('<');
    NEXUS_SERIAL_TELEMETRY.write(0);
    checksum ^= 0;
    NEXUS_SERIAL_TELEMETRY.write(MSP_MOTOR_TELEMETRY);
    checksum ^= MSP_MOTOR_TELEMETRY;
    NEXUS_SERIAL_TELEMETRY.write(checksum);
}
// ************************************************************************************************************

void requestAnalog()
{
    uint8_t checksum = 0;
    NEXUS_SERIAL_TELEMETRY.write('$');
    NEXUS_SERIAL_TELEMETRY.write('M');
    NEXUS_SERIAL_TELEMETRY.write('<');
    NEXUS_SERIAL_TELEMETRY.write(0); // size = 0
    checksum ^= 0;
    NEXUS_SERIAL_TELEMETRY.write(MSP_ANALOG);
    checksum ^= MSP_ANALOG;
    NEXUS_SERIAL_TELEMETRY.write(checksum);
}

// *************************************************************************************************************

uint16_t GetRPM(const uint8_t *data, uint8_t n)
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

        if (size >= 9)
        {
            escTempC = (float)((uint8_t)payload[7]);
            Look((uint8_t)payload[7]);
        }
       
        // Serial.print("MSP Motor Telemetry size: ");
        // Serial.print(size);
        // Serial.print(" bytes: ");
        // for (int j = 0; j < size; j++)
        // {
        //     Serial.print(" j=");
        //     Serial.print(j, DEC);
        //     Serial.print("->");
        //     Serial.print(payload[j], DEC);
        // }
        // Serial.println();
        //**** */

        uint16_t motor_rpm = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
        float head = float(motor_rpm) / Ratio;
        if (head < 0.0f)
            head = 0.0f;
        if (head > 65535.0f)
            head = 65535.0f;

        return (uint16_t)(head + 0.5f);
    }
    return 0xffff;
}
// ************************************************************************************************************
// Returns true if a valid MSP_ANALOG frame was found and decoded.
bool GetAnalog(const uint8_t *data, uint8_t n,
               float &vbatOut, float &ampsOut, uint16_t &mAhOut)
{
    const uint8_t CMD = MSP_ANALOG;

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
        if (size < 1) // must at least have vbat
            continue;

        const uint8_t *payload = &data[i + 5];

        // MSP_ANALOG layout:
        // payload[0] = vbat (0.1 V units) // Only 8 BITS. So set Rotorflight correction to -50% for 12S, otherwise use INA219 on i2c with voltage divider
        // payload[1..2] = powerMeterSum (raw)
        // payload[3..4] = RSSI (0–1023)
        // payload[5..6] = amperage (0.1 A units)
        // uint8_t vbat_raw = payload[0];
        // vbatOut = vbat_raw / 10.0f;

        mAhOut = 0;
        ampsOut = 0.0f;

        if (size >= 7)
        {
            uint16_t powerMeterSum = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
            mAhOut = powerMeterSum;
            uint16_t amps_raw = (uint16_t)payload[5] | ((uint16_t)payload[6] << 8);
            ampsOut = amps_raw / 10.0f; // 0.1 A units
            if (!INA219Connected)
                RXModelVolts = (float)((uint8_t)payload[0]) / 10.0f; // decivolts from analog input if INA219 not present (set Rotorflight to -50% for 12S)
        }
        return true;
    }
    return false;
}
// ************************************************************************************************************/
void CheckMSPSerial()
{
    static uint32_t Localtimer = 0;
    if (millis() - Localtimer < 100) // 10 x per second
        return;
    Localtimer = millis();
    uint8_t data_in[80];
    uint8_t p = 0;
    while (NEXUS_SERIAL_TELEMETRY.available() && p < sizeof(data_in))
    {
        data_in[p++] = NEXUS_SERIAL_TELEMETRY.read();
    }
    uint16_t tempRPM = GetRPM(&data_in[0], p);
    if (tempRPM != 0xffff)
        RotorRPM = tempRPM;

    float vbatAnalog, ampsAnalog;
    uint16_t mAhAnalog;
    if (GetAnalog(&data_in[0], p, vbatAnalog, ampsAnalog, mAhAnalog)) // this volts reading is not reliable after 25v
    {
        Battery_Amps = ampsAnalog / 10; // amps (already in A from GetAnalog)
        Battery_mAh = mAhAnalog;        // rough mAh
    }
    // 3) Request new data for next cycle
    requestRPM();
    requestAnalog();
}

#endif // NEXUS_H