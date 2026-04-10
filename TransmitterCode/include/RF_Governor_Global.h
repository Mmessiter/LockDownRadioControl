
// ********************************************************************************************************
// RF_Governor_Global.h
// This module handles global/config parameters for the Rotorflight Governor Profile screen on the Nextion display
// Profile-specific parameters (n0-n13) are handled separately in RF_Governor_Profile.h
// **********************************************************************************************************

#ifndef GOVERNOR_GLOBAL_H
#define GOVERNOR_GLOBAL_H

#include <Arduino.h>
#include "1Definitions.h"

void Start_Gov_Global()
{
    SendCommand((char *)"page RFGovViewGlbl");
    CurrentView = RFGOVERNORVIEW_GLOBAL;
    SendText((char *)"t27", ModelName);
}

void End_Gov_Global()
{
    RotorFlightStart();
}

void Save_Gov_Global()
{
    Look("Save_Gov_Global was called");
}
#endif // GOVERNOR_GLOBAL_H