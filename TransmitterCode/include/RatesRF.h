// This file handles rates for Rotorflight
#ifndef RATESRF_H
#define RATESRF_H
#include <Arduino.h>
#include "1Definitions.h"

// ************************************************************************************************************/

float FixFactor(uint8_t val, uint8_t i)
{

    switch (i)
    {
    case 0:
        return ((float)val);
    case 1:
        return ((float)val) * 10;
    case 2:
        return ((float)val) * 10;
    case 3:
        return ((float)val) / 100;
    case 4:
        return ((float)val) * 10;
    case 5:
        return ((float)val) * 10;
    case 6:
        return ((float)val) / 100;
    case 7:
        return ((float)val) * 10;
    case 8:
        return ((float)val) * 10;
    case 9:
        return ((float)val) / 100;
    case 10:
        return ((float)val) / 4.0;
    case 11:
        return ((float)val) / 4.0;
    case 12:
        return ((float)val) / 100;

    default:
        return ((float)1111);
    }
}

// ************************************************************************************************************/
void DisplayRatesValues(uint8_t startIndex, uint8_t stopIndex)
{
    for (uint8_t i = startIndex; i < stopIndex; ++i)
    {
        if (i < MAX_RATES_BYTES)
        {
            if (i == 0)
            {
                SendText(RatesWindows[0], Rate_Types[Rate_Values[0]]); // Show rate type
                continue;
            }
            char buf[10];
            float ThisValue = FixFactor(Rate_Values[i], i);
            snprintf(buf, sizeof(buf), "%.2f", ThisValue);
            SendText(RatesWindows[i], buf);
        }
    }
}

// ********************************************************************************************************
void ForegroundColourRATESLabels(uint16_t Colour)
{
    for (int i = 1; i < 13; ++i)
    {
        SendForegroundColour(RatesWindows[i], Colour);
    }
}
// ********************************************************************************************************
void RatesMsg(const char *msg, uint16_t Colour)
{
    if (CurrentView == RATESVIEW1) // Must be in RATES view
    {
        ForegroundColourRATESLabels(Colour);   // make text white so it isnt visible
        SendText((char *)"busy", (char *)msg); // Show RATES message
        SendCommand((char *)"vis busy,1");     // Make it visible
        SendCommand((char *)"vis b3,0");       // hide "Send" button
    }
}
// ************************************************************************************************************/
void HideRATESMsg()
{
    if (CurrentView == RATESVIEW1) // Must be in RATES view
    {
        SendCommand((char *)"vis busy,0"); // Hide  message
        ForegroundColourRATESLabels(Black);  // make text black so it is visible again
    }
}
// ******************************************************************************************************************************/

void ShowRatesBank()
{
    char buf[40];
    if (LedWasGreen)
    {
        snprintf(buf, sizeof(buf), "Loading RATESs for Bank %d ...", Bank);
    }
    else
    {
        snprintf(buf, sizeof(buf), "Model is not connected!");
    }
    RatesMsg(buf, Gray);
    RATES_Send_Duration = 1000;                   // how many milliseconds to await RATES values
    Reading_RATES_Now = true;                     // This tells the Ack payload parser to get RATES values
    AddParameterstoQueue(SEND_RATES_VALUES);      // Request RATES values from RX
    snprintf(buf, sizeof(buf), "Bank: %d", Bank); // Display which Bank
    SendText((char *)"t9", buf);                  // Show bank number etc
    Rates_Were_Edited = false;                    // reset edited flag
    RATES_Start_Time = millis();                  // record start time as it's not long
}

// ******************************************************************************************************************************/
void StartRatesView()
{
    if (!SafetyON)
    {
        MsgBox(RXOptionsView, (char *)"Please enable 'Safety'!");
        return;
    }
    SendCommand((char *)"page RatesView");
    CurrentView = RATESVIEW1;
    ShowRatesBank();                    // Show the current bank's RATES
    SendText((char *)"t11", ModelName); // Show model name
    Rates_Were_Edited = false;
}
// ******************************************************************************************************************************/
void EndRatesView()
{
    Rates_Were_Edited = false;
    SendCommand((char *)"page RXOptionsView");
}
// ************************************************************************************************************/
void Rates_Were_edited()
{
    // SendCommand((char *)"vis b3,1"); // show "Send" button
    Rates_Were_Edited = true;
}
// ************************************************************************************************************/
void SendEditedRates()
{
    Rates_Were_Edited = false;
}
#endif
// RATESRF_H
// ************************************************************************************************************/