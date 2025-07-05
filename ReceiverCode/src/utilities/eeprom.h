#ifndef _SRC_EEPROM_H
#define _SRC_EEPROM_H
#include <Arduino.h>
#include "utilities/1Definitions.h"
#include <EEPROM.h>
/************************************************************************************************************/

void LoadFailSafeDataFromEEPROM() //
{
    uint8_t FS_Offset = FAILSAFE_EEPROM_OFFSET;
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
void SaveFailSafeDataToEEPROM() // uses 32 bytes in EEPROM
{
    // FailSafe data occupies EEPROM from offset FAILSAFE_EEPROM_OFFSET
    uint8_t FS_Offset = FAILSAFE_EEPROM_OFFSET;

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
// MAC addresses use 6 bytes, but Pipes only use 5 bytes

void LoadSavedPipeFromEEPROM() // read only 5 bytes
{
    for (uint8_t i = 0; i < 5; ++i)
        TheSavedPipe[i] = EEPROM.read(i + BIND_EEPROM_OFFSET); // uses first 5 bytes only.
    TheSavedPipe[5] = 0;
}

//************************************************************************************************************/
void SavePipeToEEPROM()
{
    // DisplayAPipe(TheReceivedPipe); // Show the received pipe being saved
    for (uint8_t i = 0; i < 5; ++i)
    {
        EEPROM.update(i + BIND_EEPROM_OFFSET, TheReceivedPipe[i]);
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

    uint8_t VerificationNumber = EEPROM.read(MPU6050_EEPROM_OFFSET); // read the first byte to see if we have valid calibration data

    RateCalibrationRoll = LoadOneCalibrationFromEEPROM(&ExtraOffset);
    RateCalibrationPitch = LoadOneCalibrationFromEEPROM(&ExtraOffset);
    RateCalibrationYaw = LoadOneCalibrationFromEEPROM(&ExtraOffset);
    CalibrationRollReading = LoadOneCalibrationFromEEPROM(&ExtraOffset);
    CalibrationPitchReading = LoadOneCalibrationFromEEPROM(&ExtraOffset);
    Aileron_Centre = (uint32_t) LoadOneCalibrationFromEEPROM(&ExtraOffset);
    Elevator_Centre = (uint32_t) LoadOneCalibrationFromEEPROM(&ExtraOffset);
    Rudder_Centre = (uint32_t) LoadOneCalibrationFromEEPROM(&ExtraOffset);
    Throttle_Centre = (uint32_t) LoadOneCalibrationFromEEPROM(&ExtraOffset);

    if (VerificationNumber != MPU6050_CALIBRATIONS_SAVED)
        return false;
    else
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
    SaveOneCalibrationToEEPROM(&ExtraOffset, (float)Aileron_Centre);
    SaveOneCalibrationToEEPROM(&ExtraOffset, (float)Elevator_Centre);
    SaveOneCalibrationToEEPROM(&ExtraOffset, (float)Rudder_Centre);
    SaveOneCalibrationToEEPROM(&ExtraOffset, (float)Throttle_Centre);
}
#endif