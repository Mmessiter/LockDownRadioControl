
#ifndef BTLE_H
    #define BTLE_H

#ifdef USE_BTLE
#include "Hardware/1Definitions.h"
#include <Arduino.h>


// ************************************************************************************************************
void SendViaBLE(){

static bool BLEStarted = false;
static uint8_t buf[] = {6,5,4,3,2,1};

if (!BLEStarted) {
    btle.begin("LOCKDOWN"); // seems to allow only 8 characters
    BLEStarted = true;
 }
  btle.advertise(&buf,6);  // 6 bytes only per send.
  btle.hopChannel();
  Look1(".");
}
// ************************************************************************************************************
    #endif // #ifndef USE_BTLE 
#endif // #ifndef BTLE_H
