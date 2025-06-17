/**
 * EEPROM Management – Overview
 * ----------------------------
 * This module manages persistent data storage for the receiver via the internal EEPROM.
 * The layout is structured, extensible, and uses EEPROM.update() to minimize flash wear.
 *
 * Current EEPROM Layout:
 *   0   -  7  : Transmitter binding address (5 bytes used, 3 bytes reserved)
 *   8   - 23  : Failsafe settings
 *              - 8 bytes: one per channel (mapped 0–180)
 *              - 8 bytes: channel enable flags (non-zero = active)
 *   24  - 45  : MPU6050 calibration block
 *              - Byte 24: magic byte (42) to confirm validity
 *              - 5 floats = 20 bytes (for calibration values)
 *   46  - ... : Reserved for future use (see THE_NEXT_USE_OF_EEPROM_OFFSET)
 *
 * Functions Provided:
 *   - Save/Load transmitter MAC (5 bytes)
 *   - Save/Load failsafe settings
 *   - Save/Load MPU6050 calibration data (with validation marker)
 *   - Utility functions for float serialization to EEPROM
 *
 * Notes:
 *   - All writes are throttled with DelayMillis(1) for EEPROM safety.
 *   - Float values are stored using a union-based method for byte-level control.
 *   - Calibration block is versionable by changing the magic number (currently 42).
 *
 * Author: Malcolm Messiter
 * Years active: 2020–2025
 */

#ifndef _SRC_EEPROM_H
#define _SRC_EEPROM_H
#include <Arduino.h>
#include <EEPROM.h>
#include "utilities/1Definitions.h"

#define BIND_EEPROM_OFFSET 0                                     // use 8 bytes from here (in fact 5 bytes only, but we reserve 8 bytes for future use)
#define FS_EEPROM_OFFSET BIND_EEPROM_OFFSET + 8                  // use 16 bytes from here
#define MPU6050_EEPROM_OFFSET FS_EEPROM_OFFSET + 16              // use *21* bytes from here (for the MPU6050 calibration data)
#define THE_NEXT_USE_OF_EEPROM_OFFSET MPU6050_EEPROM_OFFSET + 21 // For future use ...
#define MPU6050_CALIBRATIONS_SAVED 42                            // this flag indicates whether calibrations were saved.

/************************************************************************************************************/

void LoadFailSafeDataFromEEPROM() //
{
    uint8_t FS_Offset = FS_EEPROM_OFFSET;
    uint16_t s[CHANNELSUSED];

    for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        s[i] = map(EEPROM.read(i + FS_Offset), 0, 180, MINMICROS, MAXMICROS); // load failsafe values and simulate better resolution
    }
    FS_Offset += CHANNELSUSED;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        if (EEPROM.read(i + FS_Offset))
        {
            ReceivedData[i] = s[i];
        }
    }
    FailSafeDataLoaded = true;

#ifdef DB_FAILSAFE
    Serial.println("Fail safe settings are loaded!");
#endif
}

/************************************************************************************************************/
void SaveFailSafeDataToEEPROM()
{
    // FailSafe data occupies EEPROM from offset FS_EEPROM_OFFSET
    uint8_t FS_Offset = FS_EEPROM_OFFSET;

    for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        EEPROM.update(i + FS_Offset, (map(ReceivedData[i], MINMICROS, MAXMICROS, 0, 180))); // save servo positions lower res: 8 bits
        DelayMillis(1);
    }
    FS_Offset += CHANNELSUSED;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        EEPROM.update(i + FS_Offset, FailSafeChannel[i]); // save flags
        DelayMillis(1);
    }
#ifdef DB_FAILSAFE
    Serial.println("Fail safe settings are saved!");
#endif
}

/************************************************************************************************************/
void LoadSavedPipeFromEEPROM() // read only 5 bytes
{
    for (uint8_t i = 0; i < 5; ++i)
        TheSavedPipe[i] = EEPROM.read(i + BIND_EEPROM_OFFSET); // uses first 5 bytes only.
    TheSavedPipe[5] = 0;
}
// /************************************************************************************************************/
void SavePipeToEEPROM() // // Save 5 bytes: the MAC address of the transmitter's Teensy 4.1
{
    for (uint8_t i = 0; i < 5; ++i)
    {
        EEPROM.update(i + BIND_EEPROM_OFFSET, TheSavedPipe[i]);
        DelayMillis(1);
    }
}

// **************************************************************************************************************
void SaveOneCalibrationToEEPROM(uint8_t *ExtraOffset, float TheValue)
{ // This function saves a float as next four bytes in EEPROM
    union
    {
        float Val32;
        uint8_t Val8[4];
    } ThisUnion;
    ThisUnion.Val32 = TheValue;
    for (uint8_t i = 0; i < 4; ++i)
    {
        EEPROM.update(i + MPU6050_EEPROM_OFFSET + *ExtraOffset, ThisUnion.Val8[i]);
        DelayMillis(1);
    }
    *ExtraOffset += 4;
}
//************************************************************************************************************/
float LoadOneCalibrationFromEEPROM(uint8_t *ExtraOffset)
{
    union
    {
        float Val32;
        uint8_t Val8[4];
    } ThisUnion;
    for (uint8_t i = 0; i < 4; ++i)
        ThisUnion.Val8[i] = EEPROM.read(i + MPU6050_EEPROM_OFFSET + *ExtraOffset);
    *ExtraOffset += 4;
    return ThisUnion.Val32;
}
//************************************************************************************************************/
bool LoadMPU6050CalibrationDataFromEEPROM()
{
    uint8_t ExtraOffset = 1;
    if (EEPROM.read(MPU6050_EEPROM_OFFSET) != MPU6050_CALIBRATIONS_SAVED) // do we have valid calibration data?
        return false;
    RateCalibrationRoll = LoadOneCalibrationFromEEPROM(&ExtraOffset);
    RateCalibrationPitch = LoadOneCalibrationFromEEPROM(&ExtraOffset);
    RateCalibrationYaw = LoadOneCalibrationFromEEPROM(&ExtraOffset);
    CalibrationRollReading = LoadOneCalibrationFromEEPROM(&ExtraOffset);
    CalibrationPitchReading = LoadOneCalibrationFromEEPROM(&ExtraOffset);
    return true;
}
//************************************************************************************************************/
void SaveMPU6050CalibrationDataToEEPROM()
{
    uint8_t ExtraOffset = 1;
    EEPROM.update(MPU6050_EEPROM_OFFSET, MPU6050_CALIBRATIONS_SAVED); // this number indicates that a valid calibration was probably written
    SaveOneCalibrationToEEPROM(&ExtraOffset, RateCalibrationRoll);
    SaveOneCalibrationToEEPROM(&ExtraOffset, RateCalibrationPitch);
    SaveOneCalibrationToEEPROM(&ExtraOffset, RateCalibrationYaw);
    SaveOneCalibrationToEEPROM(&ExtraOffset, CalibrationRollReading);
    SaveOneCalibrationToEEPROM(&ExtraOffset, CalibrationPitchReading);
}
#endif