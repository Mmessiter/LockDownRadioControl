#ifndef _SRC_DETECT_TRANSCEIVERS_H
#define _SRC_DETECT_TRANSCEIVERS_H
#include <Arduino.h>
#include "utilities/1Definitions.h"
#include <SPI.h>
#include <RF24.h>

void DetectTransceivers()
{
    RF24 tempRadio1(9, 10);
    RF24 tempRadio2(22, 23);
    RF24 tempRadio3(21, 20);
    SPI.begin();
    delay(100); // let SPI settle
    tempRadio1.begin();
    tempRadio2.begin();
    tempRadio3.begin();
    RadioAt_9_10 = false;
    RadioAt_22_23 = false;
    RadioAt_21_20 = false;
    delay(5); // let radios settle
    if (tempRadio1.isChipConnected())
        RadioAt_9_10 = true;
    if (tempRadio2.isChipConnected())
        RadioAt_22_23 = true;
    if (tempRadio3.isChipConnected())
        RadioAt_21_20 = true;
}
#endif // _SRC_DETECT_TRANSCEIVERS_H