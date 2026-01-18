// **********************************************************************************************************
// This file will handles Save & Restore all Rates and PID values for Rotorflight
// **********************************************************************************************************

#ifndef RF_SAVE_RESTORE_H
#define RF_SAVE_RESTORE_H
#include <Arduino.h>
#include "1Definitions.h"

uint8_t Saving_Parameters_Index = 0; // index of parameter being saved
uint8_t Restoring_Parameters_Index = 0; // index of parameter being restored

// ************************************************************************************************************/
void SaveRFParameters()
{
    if (GetConfirmation((char *)"page RFView", (char *)"Save ALL these values?"))
    {
        MsgBox((char *)"page RFView", (char *)"Not yet implemented!");
        Saving_Parameters_Index = 0;
        // CurrentMode = SAVINGRFPARAMETERS; etc...
    }
}

// ************************************************************************************************************/
void RestoreRFParameters()
{
    if (GetConfirmation((char *)"page RFView", (char *)"Restore ALL these values?"))
    {
        MsgBox((char *)"page RFView", (char *)"Implementation coming soon!");
        Restoring_Parameters_Index = 0;
        // CurrentMode = RESTORINGRFPARAMETERS; etc...
    }
}
// ************************************************************************************************************/

#endif // RF_SAVE_RESTORE_H