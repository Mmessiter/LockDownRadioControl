/** @file ReceiverCode/src/utilities/common.h */
#ifndef _SRC_UTILITIES_COMMON_H
#define _SRC_UTILITIES_COMMON_H

#include <stdint.h>
#include <EEPROM.h>

#define RXVERSION_MAJOR   1
#define RXVERSION_MINOR   1
#define RXVERSION_MINIMUS 6 // Nov 12th 2021

#define NEW_FHSS   // Working!! but unfinished ...
// #define OLD_FHSS
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

// Use this array for UK legal flying
//uint8_t FHSS_Channels[84] = {28,24,61,64,28,55,66,19,76,21,59,67,15,71,82,32,49,69,13,2,34,47,20,34,69,16,2,72,35,76,35,57,45,29,76,75,49,59,3,57,20,16,41,59,62,59,67,11,3,9,77,37,8,31,36,34,18,75,17,9,50,78,77,73,30,50,79,6,36,20,23,79,40,54,51,19,69,12,18,80,53,41,24};

// Use this array for testing in noisy wifi
uint8_t FHSS_Channels[84] = {93,111,107,103,106,111,97,108,102,118,104,101,109,98,113,113,124,115,91,111,96,85,117,89,103,117,111,99,85,114,118,118,118,99,87,112,118,117,91,101,86,99,103,99,124,101,104,101,109,94,86,92,119,120,91,114,119,117,101,117,100,92,92,120,119,115,113,92,121,89,119,103,106,121,123,96,93,102,111,95,101,122,95};



/************************************************************************************************************/

uint8_t LoadModelNumber()
{
    return EEPROM.read(28);
}

#endif // defined (_SRC_UTILITIES_COMMON_H)
