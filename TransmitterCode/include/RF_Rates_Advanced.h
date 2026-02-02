
// **********************************************************************************************************
// This file handles advanced rates editing for Rotorflight compatible flight controllers
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
        BlockBankChanges = false;
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
// ************************************************************************************************************/
void RatesAdvancedWereEdited()
{
    SendCommand((char *)"vis b3,1"); // show "Send" button
    Rates_Advanced_Were_Edited = true;
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
void ShowLocalRatesAdvancedBank()
{
    for (uint8_t i = 0; i < MAX_RATES_ADVANCED_BYTES; ++i)
    {
        float ThisValue;
        if (i == 14) // last one is divided by 10 and therefore must be text
        {
            ThisValue = (Saved_Rate_Advanced_Values[i][Bank - 1] / (float)10);
            char temp[10];
            snprintf(temp, sizeof(temp), "%.1f", ThisValue);
            SendText(RatesAWindows[i], temp);
            continue;
        }
        ThisValue = (float)Saved_Rate_Advanced_Values[i][Bank - 1];
        SendValue(RatesAWindows[i], (uint8_t)ThisValue);
    }
    Hide_Advanced_Rates_Msg();
    SendCommand((char *)"vis b3,0"); // hide "Send" button
}

// ************************************************************************************************************/
void ShowRatesAdvancedBank()
{
    if (CurrentView != RATESADVANCEDVIEW) // Must be in  RATESADVANCEDVIEW
        return;
    SendText((char *)"t9", BankNames[BanksInUse[Bank - 1]]); // Show bank number etc
    if (!(LedWasGreen))
    {
        ShowLocalRatesAdvancedBank();
        return;
    }

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
    if (Rates_Advanced_Were_Edited) // if RATES were edited but not sent and bank changed
    {
        char Wmsg[120];
        char w1[] = "Values for "; // heer
        char w2[] = " were edited \r\nbut not saved. (Too late now!)\r\nSo you may want to check them.";
        strcpy(Wmsg, w1);
        strcat(Wmsg, BankNames[BanksInUse[Bank - 1]]);
        strcat(Wmsg, w2);
        MsgBox((char *)"page Rates_A_View", Wmsg); // Warn about unsaved edits
    }
    RatesAdvancedMsg(buf, Gray);
    Rates_Advanced_Send_Duration = MSP_WAIT_TIME;     // how many milliseconds to await RATES values
    Reading_RATES_Advanced_Now = true;                // This tells the Ack payload parser to
    BlockBankChanges = true;                  // block bank changes while we do this
    AddParameterstoQueue(SEND_RATES_ADVANCED_VALUES); // Request RATES values from RX
    RATES_Advanced_Start_Time = millis();             // record start time as it's not long
    Rates_Advanced_Were_Edited = false;               // reset edited flag
}

// **********************************************************************************************************/
void StartRatesAdvancedView()
{
    if (Rates_Were_Edited)
    {
        if (GetConfirmation((char *)"page RatesView", (char *)"Discard edited RATES values?"))
        {
            Rates_Were_Edited = false;
        }
        else
        {
            return;
        }
    }
    SendCommand((char *)"page Rates_A_View");
    CurrentView = RATESADVANCEDVIEW;
    ShowRatesAdvancedBank();            // Show the current bank's RATES
    SendText((char *)"t11", ModelName); // Show model name
    Rates_Advanced_Were_Edited = false;
} // **********************************************************************************************************/
void EndRatesAdvancedView()
{
    if (Rates_Advanced_Were_Edited)
    {
        if (GetConfirmation((char *)"page Rates_A_View", (char *)"Discard edited values?"))
            GotoFrontView();
    }
    else
    {
        Rates_Advanced_Were_Edited = false;
        SendCommand((char *)"page RFView");
        CurrentView = ROTORFLIGHTVIEW;
        ShowRFBank();
    }
}
// ************************************************************************************************************/
void ReadRatesAdvanced()
{
    for (uint8_t i = 0; i < MAX_RATES_ADVANCED_BYTES; ++i)
    {
        if (i == 14) // last one is divided by 10 and therefore must be text
        {
            char temp[10];
            GetText(RatesAWindows[i], temp);
            float q = atof(temp) * 10.0;
            Rate_Advanced_Values[i] = (uint8_t)q;
            continue;
        }
        Rate_Advanced_Values[i] = (uint8_t)GetValue(RatesAWindows[i]);
    }
}

// ************************************************************************************************************/
void SaveLocalRatesAdvancedBank()
{
    RatesAdvancedMsg((char *)"Saving edited Advanced Rates ...", Gray); // Show sending message
    Rates_Advanced_Were_Edited = false;
    ReadRatesAdvanced();
    for (uint8_t i = 0; i < MAX_RATES_ADVANCED_BYTES; ++i)
    {
        Saved_Rate_Advanced_Values[i][Bank - 1] = Rate_Advanced_Values[i];
    }
    SaveOneModel(ModelNumber);       // save all to SD card
    Hide_Advanced_Rates_Msg();       // ...because this queue is a LIFO stack
    SendCommand((char *)"vis b3,0"); // hide "Send" button
    PlaySound(BEEPCOMPLETE);         // let user know we're done
}

// **********************************************************************************************************/
void SendEditedRatesAdvanced()
{
    Rates_Advanced_Were_Edited = false;
    if (!(LedWasGreen)) // Model not connected so save to local Advanced RATES
    {
        SaveLocalRatesAdvancedBank();
        return;
    }
    PlaySound(BEEPMIDDLE);
    BlockBankChanges = true;
    DelayWithDog(100);                                           //  allow LOTS of time for screen to update BEFORE sending another Nextion command
    RatesAdvancedMsg((char *)"Sending edited values ...", Gray); // Show sending message
    ReadRatesAdvanced();
    AddParameterstoQueue(GET_RATES_ADVANCED_VALUES_SECOND_8); // Send RATES ADVANCED values from TX to RX
    AddParameterstoQueue(GET_RATES_ADVANCED_VALUES_FIRST_7);  // Send RATES ADVANCED values from TX to RX ...because this queue is a LIFO stack
    Hide_Advanced_Rates_Msg();                                // SECOND MUST BE QUEUED FIRST!!!  because this queue is a LIFO stack
    PlaySound(BEEPCOMPLETE);                                  // let user know we're done
}
#endif // RATESRF_ADVANCED_H