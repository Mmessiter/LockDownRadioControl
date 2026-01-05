// This file handles basic rates editing for Rotorflight

#ifndef RATESRF_H
#define RATESRF_H
#include <Arduino.h>
#include "1Definitions.h"
const float FactorTableRF[13] = {1, 10, 10, .01, 10, 10, .01, 10, 10, .01, 0.25, 0.25, .01};
// ************************************************************************************************************/

float FixFactor(uint8_t val, uint8_t i)
{
    return ((float)val) * FactorTableRF[i];
}

// ************************************************************************************************************/

uint8_t UnFixFactor(float val, uint8_t i)
{
    return (uint8_t)(val / FactorTableRF[i]);
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
            if (ThisValue == int(ThisValue))
            {
                if (i < 10)
                {
                    snprintf(buf, sizeof(buf), "%.0f", ThisValue);
                }
                else
                {
                    snprintf(buf, sizeof(buf), "%.1f", ThisValue);
                }
            }
            else
            {
                snprintf(buf, sizeof(buf), "%.2f", ThisValue);
            }
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
        SendCommand((char *)"vis busy,0");  // Hide  message
        ForegroundColourRATESLabels(Black); // make text black so it is visible again
    }
}
// ******************************************************************************************************************************/

void ShowRatesBank()
{
    if (CurrentView == RATESVIEW1) // Must be in RATES view
    {
        char buf[40];
        if (LedWasGreen)
        {
            snprintf(buf, sizeof(buf), "Loading rates for Bank %d ...", Bank);
        }
        else
        {
            snprintf(buf, sizeof(buf), "Model is not connected!");
        }
        if (Rates_Were_Edited)
        {
            char NB[10];
            char Wmsg[120];
            char w1[] = "Rates for Bank ";
            char w2[] = " were edited \r\nbut not saved. (Too late now!)\r\nSo you may want to check them.";
            strcpy(Wmsg, w1);
            strcat(Wmsg, Str(NB, PreviousBank, 0));
            strcat(Wmsg, w2);
            MsgBox((char *)"page RatesView", Wmsg); // Warn about unsaved edits
        }
        RatesMsg(buf, Gray);
        RATES_Send_Duration = 1000;                   // how many milliseconds to await RATES values
        Reading_RATES_Now = true;                     // This tells the Ack payload parser to get RATES values
        AddParameterstoQueue(SEND_RATES_VALUES);      // Request RATES values from RX
        snprintf(buf, sizeof(buf), "Bank: %d", Bank); // Display which Bank
        SendText((char *)"t9", buf);                  // Show bank number etc
        RATES_Start_Time = millis();                  // record start time as it's not long

        Rates_Were_Edited = false; // reset edited flag
    }
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
    if (Rates_Were_Edited)
    {
        if (!GetConfirmation((char *)"page RatesView", (char *)"Discard edited RATES?"))
            return;
    }
    Rates_Were_Edited = false;
    SendCommand((char *)"page RXOptionsView");
}
// ************************************************************************************************************/
void RatesWereEdited()
{
    SendCommand((char *)"vis b3,1"); // show "Send" button
    Rates_Were_Edited = true;
}

// ************************************************************************************************************/

void ReadEditedRATES()
{
    char temp[20];
    for (int i = 1; i < MAX_RATES_BYTES; ++i)
    {
        GetText(RatesWindows[i], temp);
        Rate_Values[i] = (uint8_t)UnFixFactor(atof(temp), i);
    }
}
// ************************************************************************************************************/
void SendEditedRates()
{
    if (!GetConfirmation((char *)"page RatesView", (char *)"Send edited RATES to Nexus?"))
    {
        ShowRatesBank(); // reload old RATES, undoing any edits
        return;
    }
    RatesMsg((char *)"Sending edited Rates ...", Gray); // Show sending message
    DelayWithDog(150);                                  // allow time for screen to update
    ReadEditedRATES();                                  // read the edited RATES from the screen;
    AddParameterstoQueue(GET_SECOND_6_RATES_VALUES);    // SECOND MUST BE SENT FIRST!!! Send RATES 7-12 values from TX to RX
    AddParameterstoQueue(GET_FIRST_7_RATES_VALUES);     // SECOND MUST BE SENT FIRST!!! Send RATES 1-6 values from TX to RX
    HideRATESMsg();                                     // SECOND MUST BE SENT FIRST!!!  because this queue is a LIFO stack
    SendCommand((char *)"vis b3,0");                    // hide "Send" button
    Rates_Were_Edited = false;
}
#endif
// RATESRF_H
// ************************************************************************************************************/