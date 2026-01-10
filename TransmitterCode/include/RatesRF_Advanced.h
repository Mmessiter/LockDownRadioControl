
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
void HideRATES_Advanced_Msg(){

    if (CurrentView == RATESADVANCEDVIEW) // Must be in RATESADVANCEDVIEW view
    {
        SendCommand((char *)"vis busy,0"); // Hide RATES message

        ForegroundColourRATESALabels(Black); // make text black so it is visible again
        
    }   




}
// ************************************************************************************************************/
void DisplayRates_Advanced_Values(uint8_t n, uint8_t m)
{

    for (uint8_t i = n; i < m; ++i)
    {
        if (i < MAX_RATES_ADVANCED_BYTES)
        {
            SendValue(RatesAWindows[i], Rate_Advanced_Values[i]); // Send to screen
        }
    }
}

// ************************************************************************************************************/
void ForegroundColourRATESALabels(uint16_t Colour)
{
    for (int i = 0; i < 15; ++i)
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
void ShowRatesAdvancedBank()
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
    SendText((char *)"t9", BankNames[BanksInUse[Bank - 1]]); // Show bank number etc
    RatesAMsg(buf, Gray);
    RATES_A_Send_Duration = 1000;                          // how many milliseconds to await RATES values
    Reading_RATES_Advanced_Now = true;                     // This tells the Ack payload parser to
    AddParameterstoQueue(SEND_RATES_ADVANCED_VALUES);      // Request RATES values from RX
    RATES_Advanced_Start_Time = millis();                  // record start time as it's not long
    Rates_Were_Edited = false;                             // reset edited flag
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
    ShowRatesAdvancedBank();                  // Show the current bank's RATES
    SendText((char *)"t11", ModelName); // Show model name
    Rates_Advanced_Were_Edited = false;
}

#endif // RATESRF_ADVANCED_H