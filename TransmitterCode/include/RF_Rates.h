// **********************************************************************************************************
// This file handles basic rates editing for Rotorflight
// It uses a factors' table to convert between screen display values and the actual byte values needed by Rotorflight
// **********************************************************************************************************

#ifndef RATESRF_H
#define RATESRF_H
#include <Arduino.h>
#include "1Definitions.h"

// ************************************************************************************************************/
void ReadEditedRateValues()
{

    for (int i = 1; i < MAX_RATES_BYTES; ++i) // start at 1 because 0 is Rates Type and not yet edited here
    {
        char temp[10];
        GetText(RatesWindows[i], temp);
        Rate_Values[i] = (uint8_t)(atof(temp) / FactorTableRF[i]); // DIVIDE BY factor to get byte value
    }
}

// ************************************************************************************************************/
void SaveRatesLocalBank()
{

    RatesMsg((char *)"Saving edited Rates ...", Gray); // Show sending message
    Rates_Were_Edited = false;
    ReadEditedRateValues();
    Rate_Values[0] = 4; // set to "Actual" type
    for (int i = 0; i < MAX_RATES_BYTES; ++i)
    {
        Saved_Rate_Values[i][Bank - 1] = Rate_Values[i];
    }
    SaveOneModel(ModelNumber);       // save all to SD card
    HideRATESMsg();                  // ...because this queue is a LIFO stack
    SendCommand((char *)"vis b3,0"); // hide "Send" button
    PlaySound(BEEPCOMPLETE);         // let user know we're done
}
// ************************************************************************************************************/
void SendEditedRates()
{
    if (!LedWasGreen) // Model not connected so save to local RATES
    {
        SaveRatesLocalBank();
        return;
    }
    Rates_Were_Edited = false;
    BlockBankChanges = true;
    PlaySound(BEEPMIDDLE);
    DelayWithDog(100);                                  // allow LOTS of time for screen to update BEFORE sending another Nextion command
    RatesMsg((char *)"Sending edited Rates ...", Gray); // Show sending message
    ReadEditedRateValues();                             // read the edited RATES from the screen;
    AddParameterstoQueue(GET_SECOND_6_RATES_VALUES);    // SECOND MUST BE QUEUED FIRST!!! Send RATES 7-12 values from TX to RX
    AddParameterstoQueue(GET_FIRST_7_RATES_VALUES);     // SECOND MUST BE QUEUED FIRST!!! Send RATES 1-6 values from TX to RX
    HideRATESMsg();                                     // ...because this queue is a LIFO stack
    SendCommand((char *)"vis b3,0");                    // hide "Send" button
    PlaySound(BEEPCOMPLETE);                            // let user know we're done
}
// ************************************************************************************************************/
void DisplayRatesValues(uint8_t startIndex, uint8_t stopIndex) // Displays RATES values on screen a few at a time as they arrive in Ack payloads
{
    for (uint8_t i = startIndex; i < stopIndex; ++i)
    {
        if (i < MAX_RATES_BYTES)
        {
            char temp[10];
            if (i == 0)
            {
                SendText(RatesWindows[0], Rate_Types[Rate_Values[0]]); // Show rates type but it cannot be edited here
                continue;
            }
            float ThisValue = (Rate_Values[i] * FactorTableRF[i]); // MULTIPLY BY factor to get display value
            if (ThisValue == int(ThisValue))
            {
                if (i < 10)
                {
                    snprintf(temp, sizeof(temp), "%.0f", ThisValue);
                }
                else
                {
                    snprintf(temp, sizeof(temp), "%.1f", ThisValue);
                }
            }
            else
            {
                snprintf(temp, sizeof(temp), "%.2f", ThisValue);
            }
            SendText(RatesWindows[i], temp);
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
    if (CurrentView == RATESVIEW_RF) // Must be in RATES view
    {
        ForegroundColourRATESLabels(Colour);   // make text white so it isn't visible
        SendText((char *)"busy", (char *)msg); // Show RATES message
        SendCommand((char *)"vis busy,1");     // Make it visible
        SendCommand((char *)"vis b2,0");       // Make Advanced invisible
        SendCommand((char *)"vis b3,0");       // hide "Send" button
    }
}

// ************************************************************************************************************/
void HideRATESMsg()
{
    if (CurrentView == RATESVIEW_RF) // Must be in RATES view
    {
        SendCommand((char *)"vis busy,0");  // Hide  message
        SendCommand((char *)"vis b2,1");    // Make Advanced visible
        ForegroundColourRATESLabels(Black); // make text black so it is visible again
        BlockBankChanges = false;
    }
}
// ******************************************************************************************************************************/
void ShowRatesLocalBank()
{
    for (int i = 0; i < MAX_RATES_BYTES; ++i)
    {
        float ThisValue = (Saved_Rate_Values[i][Bank - 1] * FactorTableRF[i]); // MULTIPLY BY factor to get display value
        char temp[10];
        if (ThisValue == int(ThisValue))
        {
            if (i < 10)
            {
                snprintf(temp, sizeof(temp), "%.0f", ThisValue);
            }
            else
            {
                snprintf(temp, sizeof(temp), "%.1f", ThisValue);
            }
        }
        else
        {
            snprintf(temp, sizeof(temp), "%.2f", ThisValue);
        }
        if (i == 0)
        {
            SendText(RatesWindows[0], Rate_Types[Saved_Rate_Values[0][Bank - 1]]); // Show rates type but it cannot be edited here
        }
        else
        {
            SendText(RatesWindows[i], temp);
        }
    }
    HideRATESMsg();                  // ...because this queue is a LIFO stack
    SendCommand((char *)"vis b3,0"); // hide "Send" button
    BlockBankChanges = false;
}
// ******************************************************************************************************************************/

void ShowRatesBank()
{
    if (CurrentView == RATESVIEW_RF) // Must be in RATES view
    {
        char buf[40];
        char Rmsg[50];
        char NB[10];

        snprintf(Rmsg, sizeof(Rmsg), "Rate %u", (unsigned)DualRateInUse);
        SendText((char *)"t9", Rmsg); // Show rate number etc

        if (LedWasGreen)
        {
            strcpy(buf, "Loading rates for Rate ");
            strcat(buf, Str(NB, DualRateInUse, 0));
        }
        else
        {
            ShowRatesLocalBank(); // Model not connected so show local saved RATES
            return;
        }

        if (Rates_Were_Edited) // if RATES were edited but not sent and bank changed
        {
            char w1[] = "Values for last Rate were edited \r\nbut not saved.(Too late now !)\r\nSo you may want to check them.";  
            MsgBox((char *)"page RatesView", w1); // Warn about unsaved edits
        }
        RatesMsg(buf, Gray);
        BlockBankChanges = true;
        RATES_Send_Duration = MSP_WAIT_TIME;     // how many milliseconds to await RATES values
        Reading_RATES_Now = true;                // This tells the Ack payload parser to get RATES values
        AddParameterstoQueue(SEND_RATES_VALUES); // Request RATES values from RX
        RATES_Start_Time = millis();             // record start time as it's not long
        Rates_Were_Edited = false;               // reset edited flag
    }
}

// ******************************************************************************************************************************/
void StartRFRatesView()
{
    if (SendBuffer[ArmingChannel - 1] > 1000) // Safety is on if value > 1000
    {
        PlaySound(WHAHWHAHMSG); // let user know we're in trouble
        MsgBox((char *)"page RFView", (char *)"Model is armed and dangerous!\r\n(Disarm model to edit Rates.)");
        return;
    }
    SendCommand((char *)"page RatesView");
    CurrentView = RATESVIEW_RF;
    ShowRatesBank();                    // Show the current bank's RATES
    SendText((char *)"t11", ModelName); // Show model name
    Rates_Were_Edited = false;
}
// ******************************************************************************************************************************/
void EndRFRatesView()
{
    if (Rates_Were_Edited)
    {
        if (!GetConfirmation((char *)"page RatesView", (char *)"Discard edited RATES?"))
            return;
    }
    Rates_Were_Edited = false;
    SendCommand((char *)"page RFView");
    CurrentView = ROTORFLIGHTVIEW;
    ShowRFBank();
    ShowRFRate();
    BlockBankChanges = false;
}
// ************************************************************************************************************/
void RatesWereEdited()
{
    SendCommand((char *)"vis b3,1"); // show "Send" button
    Rates_Were_Edited = true;
}
#endif // RATESRF_H

// ************************************************************************************************************/