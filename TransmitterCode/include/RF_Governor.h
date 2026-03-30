// ******************************************** Rotorflight PID Advanced ******************************************************
// This module handles the Rotorflight Governor on the Nextion display

// **********************************************************************************************************
#ifndef GOVERNOR_H
#define GOVERNOR_H
#include <Arduino.h>
#include "1Definitions.h"

void Start_RF_Governor()
{
SendCommand((char *)"page RFGovView"); // Make Governor view visible
CurrentView = RFGOVERNORVIEW;


}

void End_RF_Governor()
{
    RotorFlightStart();
}

#endif // GOVERNOR_H
       // *********************************************************************************************************************************/