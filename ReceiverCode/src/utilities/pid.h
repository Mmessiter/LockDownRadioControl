#ifndef _SRC_PID_H
#define _SRC_PID_H

#include <Arduino.h>

#include "utilities/common.h"


/************************************************************************************************************/

void PIDEntryPoint(){
    static uint32_t LastTime = 0;
    static uint32_t lcount = 0;
    if (millis()-LastTime >= 1000){
        LastTime = millis();
        Serial.print("Loop speed: ");
        Serial.print(lcount);
        Serial.print(" Hz  - ");
        Serial.println(millis());
        lcount = 0;
   }
   ++lcount;
}

/************************************************************************************************************/

void DoStabilsation(){  // This is called from the main loop and from all DelayMillis() loops
    
    
    return;         // later! <<<-----------------<<<<<<<<< ********* <<<<<<<<<<<<<<<<<

    static uint32_t LastTime = 0;
    if (millis()-LastTime >= 4){
        LastTime = millis();
        PIDEntryPoint();            // here we can call a timed stabilisation event at exactly 250 Hz
   }
}

#endif // _SRC_PID_H

// End of file: src/utilities/pid.h

