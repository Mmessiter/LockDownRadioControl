// **********************************************************************************************************
// This file handles Rotorflight central functions
// **********************************************************************************************************

#ifndef RotorFlight_H
#define RotorFlight_H
#include <Arduino.h>
#include "1Definitions.h"


void ShowRFBank()
{
    char NB[10];
    Str(NB, Bank, 0);
    char msg[70];
    strcpy(msg, "Restore to Bank ");
    strcat(msg, NB);
    SendText((char *)"b3", msg);
    strcpy(msg, "Save from Bank ");
    strcat(msg, NB);
    SendText((char *)"b2", msg);
    strcpy(msg, "Bank ");
    strcat(msg, NB);
    SendText((char *)"t14", msg);
}


// **********************************************************************************************************/
void RotorFlightStart()
{
    char t12[] = "t12";
    char t7[] = "t7";
    char Vbuf[15];
   
    
    SendCommand((char *)"page RFView");
    CurrentView = ROTORFLIGHTVIEW;
    SendText((char *)"t5", (char *)"2.2"); // Show version
    SendText((char *)"t11", ModelName);    // Show model name
    snprintf(Vbuf, 5, "%1.2f", GearRatio); // 10.3 usually
    SendText(t12, Vbuf);
    snprintf(Vbuf, 5, "%d", ArmingChannel);
    SendText(t7, Vbuf);
    ShowRFBank();
}
// **********************************************************************************************************
void RotorFlightEnd()
{
    char temp[10];
    GetText((char *)"t12", temp);
    GearRatio = atof(temp);
    GetText((char *)"t7", temp);
    ArmingChannel = atoi(temp);
    SaveOneModel(ModelNumber); // save the model including gear ratio and arming channel
    ZeroDataScreen();          // clear the screen data because editing Rotorflight parameters may have created misleading comms gaps
    SendCommand((char *)"page RXOptionsView");
    CurrentView = RXSETUPVIEW1;
}

#endif // RotorFlight_H