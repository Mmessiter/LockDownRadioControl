// ********************************************** Help.h ************************************************************
// This file displays the help screens ... it now uses the log screen
#ifndef HELP_H
#define HELP_H
#include <Arduino.h>
#include "Hardware/1Definitions.h"
// ******************************************************************************************************************************/
void SendHelp(){ // load new help file
    uint8_t  i = 0;
    while (TextIn[i+9] != 0 && i < 15) { // then load the help filename
        LogFileName[i] = TextIn[i+9];
        ++i;
        LogFileName[i] = 0;
    }
    LogVIEWNew(); // use the same function as for the log files
}
#endif
