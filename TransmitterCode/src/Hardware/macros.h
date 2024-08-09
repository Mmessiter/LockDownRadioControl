// *************************************** Macros.h  *****************************************
#include <Arduino.h>
#include "Hardware/1Definitions.h"

#ifndef MACROS_H
    #define MACROS_H
    
    
//***********************************************************************************************************

void StartMacro(uint8_t m)
{                                                                                     // Start a macro
    MacrosBuffer[m][MACRORUNNINGNOW] |= 1;                                            // LOW BIT = "running now" flag
    MacroStartTime[m] = millis() + ((MacrosBuffer[m][MACROSTARTTIME]) * 100);         // Note its Start moment
    MacroStopTime[m]  = MacroStartTime[m] + ((MacrosBuffer[m][MACRODURATION]) * 100); // Note its Stop moment
}
/************************************************************************************************************/
void RunMacro(uint8_t m)
{                                                                                                                                      // Move a servo to a place
    uint32_t RightNow = millis();//
    if (RightNow >= MacroStartTime[m]) MacrosBuffer[m][MACRORUNNINGNOW] |= 2;                                                          // Set the ACTIVE Bit if started (BIT 1)
    if (RightNow >= MacroStopTime[m]) MacrosBuffer[m][MACRORUNNINGNOW] &= 1;                                                           // Clear the ACTIVE Bit if expired (BIT 1)
    if (MacrosBuffer[m][MACRORUNNINGNOW] & 2) {
        SendBuffer[(MacrosBuffer[m][MACROMOVECHANNEL]) - 1] = map(MacrosBuffer[m][MACROMOVETOPOSITION], 0, 180, MINMICROS, MAXMICROS); // Do it if currently active!
    }
}
/************************************************************************************************************/
void StopMacro(uint8_t m)
{ // Stop a macro
    MacrosBuffer[m][MACRORUNNINGNOW] = 0;
}
/************************************************************************************************************/

void ExecuteMacro()
{ // Main entry point from SendData()  ... START/STOP/RUN
    uint8_t TriggerChannel = 0;
    for (u_int8_t i = 0; i < MAXMACROS; ++i) {
        // ***************************** START OR STOP ******************************
        TriggerChannel = (MacrosBuffer[i][MACROTRIGGERCHANNEL]) - 1;  // Down by one as channels are really 0 - 15
        if (TriggerChannel) {                                         // Is trigger channel non-zero?
            if (SendBuffer[TriggerChannel] >= MAXMICROS - 1) {        // Is the trigger point is close to its highest value?
                if (!MacrosBuffer[i][MACRORUNNINGNOW]) StartMacro(i); // Yes. Start if not already started
            }
            else {
                if (MacrosBuffer[i][MACRORUNNINGNOW]) StopMacro(i); // No. Stop it if it was running
            }
            // ****************************  RUN ****************************************
            if (MacrosBuffer[i][MACRORUNNINGNOW]) { // If running, move the servo ... if timer agrees.
                RunMacro(i);
            }
        }
    }
}

// *************** END OF MACROS ZONE ************************************************

/*********************************************************************************************************************************/
#endif
