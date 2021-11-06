#ifndef _SRC_UTILITIES_COMMON_H
#define _SRC_UTILITIES_COMMON_H

#include <stdint.h>
#include <EEPROM.h>

#define RXVERSION_MAJOR   1 
#define RXVERSION_MINOR   1
#define RXVERSION_MINIMUS 4 // Nov 5th 2021

// #define DEBUG
// #define DB_SENSORS
// #define DB_PID
// #define DB_BIND
// #define DB_FAILSAFE
  #define SECOND_TRANSCEIVER              // *** DON'T FORGET TO SET THIS ONE!!! ***
// #define SECOND_TRANSCEIVER_DEBUG

int  LastConnectionMoment = 0;
bool FailSafeSave         = false;
bool FailSafeSent         = false;

/************************************************************************************************************/

uint8_t LoadModelNumber()
{
    return EEPROM.read(28);
}

#endif // defined (_SRC_UTILITIES_COMMON_H)
