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
 #endif