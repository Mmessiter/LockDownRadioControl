// *************************************** BuddyPPM.h  *****************************************
// This file does Buddy by a wire and PPM. 
// It's very rarely used now, but it's here for those who want to use it.
// The new Wireless Buddy function is just better in many ways.

#include <Arduino.h>
#include "Hardware/1Definitions.h"

#ifndef BUDDYPPM_H
    #define BUDDYPPM_H

//*************************************************************************************************************************


// This function reads data from BUDDY (Slave) BUT uses it ONLY WHILE buddy switch is on

void GetSlaveChannelValuesPPM() // MASTER code
{
    if (BuddyON) {

        if (PPMdata.PPMInputBuddy.available() == CHANNELSUSED) {
            for (int j = 0; j < CHANNELSUSED; ++j) {                // While slave has control, his stick data replaces all ours
                uint16_t PpmIn = PPMdata.PPMInputBuddy.read(j + 1); // read EVERY channel
                if (BuddyControlled & 1 << (j)) {                   // Test if this channel is buddy controlled. If not leave it unchanged
                    SendBuffer[j] = PpmIn;
                    BuddyBuffer[j]  = PpmIn;//
                }
            }
            if (!SlaveHasControl) { // Buddy is now On
                PlaySound(BUDDYMSG);
                LastShowTime    = 0;
                SlaveHasControl = true;
            }
        }
        else {
            for (int j = 0; j < CHANNELSUSED; ++j) { // reuse old data if no new is available.
                if (BuddyControlled & 1 << (j)) SendBuffer[j] = BuddyBuffer[j];
            }
        }
    }
    else { // Buddy is now Off
        if (SlaveHasControl) {
            PlaySound(MASTERMSG);
            LastShowTime    = 0;
            SlaveHasControl = false;
        }
    }
}

/************************************************************************************************************/
//  Send via PPM for Buddy box pupil

FASTRUN void SendViaPPM()
{
    static uint32_t PPMTimer = 0;

    if (millis() - PPMTimer >= PPMBUDDYFRAMERATE) {
        PPMTimer = millis();
        for (int j = 0; j < CHANNELSUSED; ++j) PPMdata.PPMOutputBuddy.write(j + 1, SendBuffer[j]);
    }
}

//***********************************************************************************************************
#endif