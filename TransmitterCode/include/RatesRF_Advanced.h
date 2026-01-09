
// **********************************************************************************************************
// This file handles basic rates editing for Rotorflight
// It uses a factors' table to convert between screen display values and the actual byte values needed by Rotorflight
// **********************************************************************************************************

#ifndef RATESRF_ADVANCED_H
#define RATESRF_ADVANCED_H
#include <Arduino.h>
#include "1Definitions.h"

char RatesAWindows[15][4] = {"n0", "n1", "n2", "n3", "n4", "n5", "n6", "n7", "n8", "n9", "n10", "n11", "n12", "n13", "n14"}; // Text boxes for Rates Advanced view

// ************************************************************************************************************/
void ForegroundColourRATESALabels(uint16_t Colour)
{
    for (int i = 0; i < 14; ++i)
    {
        SendForegroundColour(RatesAWindows[i], Colour);
    }
}
// ********************************************************************************************************
void RatesAMsg(const char *msg, uint16_t Colour)
{
    if (CurrentView == RATESADVANCEDVIEW) // Must be in RATESADVANCEDVIEW view
    {
        ForegroundColourRATESALabels(Colour);  // make text gray so it isn't visible
        SendText((char *)"busy", (char *)msg); // Show RATES message
        SendCommand((char *)"vis busy,1");     // Make it visible
        SendCommand((char *)"vis b3,0");       // hide "Send" button
    }
}
// ************************************************************************************************************/
void ShowRatesADBank()
{

    if (CurrentView != RATESADVANCEDVIEW) // Must be in  RATESADVANCEDVIEW
        return;
    char buf[40];
    if (LedWasGreen)
    {
        strcpy(buf, "Loading values for ");
        strcat(buf, BankNames[BanksInUse[Bank - 1]]);
    }
    else
    {
        snprintf(buf, sizeof(buf), "Model is not connected!");
    }
    RatesAMsg(buf, Gray);
}

// **********************************************************************************************************/
void StartRatesAdvancedView()
{
    if (SendBuffer[ArmingChannel - 1] > 1000) // Safety is on if value > 1000
    {
        PlaySound(WHAHWHAHMSG); // let user know we're in trouble
        MsgBox((char *)"page RFView", (char *)"Model is armed and dangerous!\r\n(Please disarm model.)");
        return;
    }
    SendCommand((char *)"page Rates_A_View");
    CurrentView = RATESADVANCEDVIEW;
    ShowRatesADBank();                  // Show the current bank's RATES
    SendText((char *)"t11", ModelName); // Show model name
    Rates_Advanced_Were_Edited = false;
}

#endif // RATESRF_ADVANCED_H