// **********************************************************************************************************
// This file handles Rotorflight
// **********************************************************************************************************

#ifndef RotorFlight_H
#define RotorFlight_H
#include <Arduino.h>
#include "1Definitions.h"
// **********************************************************************************************************/
void RotorFlightStart()
{
    char t12[] = "t12";
    char t7[] = "t7";
    char Vbuf[15];

    SendCommand((char *)"page RFView");
    CurrentView = ROTORFLIGHTVIEW;
    SendText((char *)"t5", (char *) "2.2"); // Show version
    SendText((char *)"t11", ModelName); // Show model name
    snprintf(Vbuf, 5, "%1.2f", GearRatio);
    SendText(t12, Vbuf);
    snprintf(Vbuf, 5, "%d", ArmingChannel);
    SendText(t7, Vbuf);

  
      
}
// **********************************************************************************************************/
void RotorFlightEnd()
{

char temp[10];
    GetText((char *)"t12", temp);
    GearRatio = atof(temp);
    GetText((char *)"t7", temp);
    ArmingChannel = atoi(temp); // TODO SAVE!!
    SaveOneModel(ModelNumber); // save the model including gear ratio
    SendCommand((char *)"page RXOptionsView");
    CurrentView = RXSETUPVIEW1;
}

#endif // RotorFlight_H