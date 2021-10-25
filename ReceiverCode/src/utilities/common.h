#ifndef _SRC_UTILITIES_COMMON_H
#define _SRC_UTILITIES_COMMON_H

#include <stdint.h>
#include <EEPROM.h>

#define RXVERSION_MAJOR   1 // Oct 19th 2021
#define RXVERSION_MINOR   0
#define RXVERSION_MINIMUS 5 // Oct 25th 2021

// #define DEBUG
// #define DB_SENSORS
// #define DB_PID
// #define DB_BIND
// #define DB_FAILSAFE
// #define SECOND_TRANSCEIVER

int  LastConnectionMoment = 0;
bool FailSafeSave         = false;
bool FailSafeSent         = false;

/************************************************************************************************************/

uint8_t LoadModelNumber()
{
    return EEPROM.read(28);
}

#endif // defined (_SRC_UTILITIES_COMMON_H)
