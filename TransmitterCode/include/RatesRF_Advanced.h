
// **********************************************************************************************************
// This file handles basic rates editing for Rotorflight
// It uses a factors' table to convert between screen display values and the actual byte values needed by Rotorflight
// **********************************************************************************************************

#ifndef RATESRF_ADVANCED_H
#define RATESRF_ADVANCED_H
#include <Arduino.h>
#include "1Definitions.h"

char RatesAWindows[15][4] = {"n0", "n1", "n2", "n3", "n4", "n5", "n6", "n7", "n8", "n9", "n10", "n11", "n12", "n13", "t14"}; // Text boxes for Rates Advanced view
// ************************************************************************************************************/
void Hide_Advanced_Rates_Msg()
{

    if (CurrentView == RATESADVANCEDVIEW) // Must be in RATESADVANCEDVIEW view
    {
        SendCommand((char *)"vis busy,0");    // Hide RATES message
        ForegroundColourAdvancedRates(Black); // make text black so it is visible again
    }
}
// ************************************************************************************************************/
void Display_Advanced_Rates_Values(uint8_t n, uint8_t m)
{
    char TextFloat[10];
    for (uint8_t i = n; i < m; ++i)
    {
        if (i < MAX_RATES_ADVANCED_BYTES)
        {
            if (i == 14) // last one is  divided by 10 and therefore must be text
            {
                float q = (float)(Rate_Advanced_Values[i] / (float)10);
                (void)snprintf(TextFloat, 10, "%.1f", (float)q);
                SendText(RatesAWindows[i], TextFloat);
                continue;
            }
            SendValue(RatesAWindows[i], Rate_Advanced_Values[i]); // Send to screen
        }
    }
}

// ************************************************************************************************************/
void ForegroundColourAdvancedRates(uint16_t Colour)
{
    for (int i = 0; i < 15; ++i)
    {
        SendForegroundColour(RatesAWindows[i], Colour);
    }
}
// ********************************************************************************************************
void RatesAdvancedMsg(const char *msg, uint16_t Colour)
{
    if (CurrentView == RATESADVANCEDVIEW) // Must be in RATESADVANCEDVIEW view
    {
        ForegroundColourAdvancedRates(Colour); // make text gray so it isn't visible
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
    RatesAdvancedMsg(buf, Gray);
    RATES_A_Send_Duration = 1000;                     // how many milliseconds to await RATES values
    Reading_RATES_Advanced_Now = true;                // This tells the Ack payload parser to
    AddParameterstoQueue(SEND_RATES_ADVANCED_VALUES); // Request RATES values from RX
    RATES_Advanced_Start_Time = millis();             // record start time as it's not long
    Rates_Were_Edited = false;                        // reset edited flag
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
    ShowRatesAdvancedBank();            // Show the current bank's RATES
    SendText((char *)"t11", ModelName); // Show model name
    Rates_Advanced_Were_Edited = false;
}

#endif // RATESRF_ADVANCED_H