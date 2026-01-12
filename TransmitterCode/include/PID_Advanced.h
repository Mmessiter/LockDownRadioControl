// ******************************************** PID Advanced ******************************************************
// This module handles the PID Advanced  screen on the Nextion display

// **********************************************************************************************************
#ifndef PIDADVANCED_H
#define PIDADVANCED_H
#include <Arduino.h>
#include "1Definitions.h"

char PID_Advanced_Labels[26][4] = {"sw0", "t1", "t2", "t3", "t4", "t5", "t6", "t7", "t8", "t9", "t10", "t11", "t12",
                                  "t13", "t14", "t15", "t16", "t17", "t18", "t19", "t20", "t21", "t22", "t23", "t24", "t25"}; // Text boxes for PID Advanced view

// ************************************************************************************************************/
void ForegroundColourPIDAdvancedLabels(uint16_t Colour)
{
    for (int i = 0; i < 26; ++i)
        SendForegroundColour(PID_Advanced_Labels[i], Colour);
}
// ************************************************************************************************************/
void HidePID_Advanced_Msg()
{
    if (CurrentView == PIDADVANCEDVIEW) // Must be in PIDAdvanced view
    {
        SendCommand((char *)"vis busy,0");        // Hide PID message
        ForegroundColourPIDAdvancedLabels(Black); // make text black so it is visible again
    }
}
// **********************************************************************************************************/
void PIDAdvancedMsg(const char *msg, uint16_t Colour)
{
    if (CurrentView == PIDADVANCEDVIEW) // Must be in PIDAdvanced view
    {
        ForegroundColourPIDAdvancedLabels(Colour);     // make text white so it isnt visible
        SendText((char *)"busy", (char *)msg); // Show PID message
        SendCommand((char *)"vis busy,1");     // Make it visible
        SendCommand((char *)"vis b3,0");       // hide "Send" button
    }
}

//************************************************************************************************************/
void ShowPIDAdvancedBank() // this is called when bank is changed so new bank's PID Advanced values are requested from Nexus and shown
{
    if (CurrentView == PIDADVANCEDVIEW) // Must be in PIDAdvanced view
    {
        char buf[40];
        if (LedWasGreen)
        {
            strcpy(buf, "Loading Values for  ");
            strcat(buf, BankNames[BanksInUse[Bank - 1]]);
            strcat(buf, " ...");
        }
        else
        {
            snprintf(buf, sizeof(buf), "Model is not connected!"); // Model not connected message
        }
        if (PIDS_Advanced_Were_Edited)
        {
            char NB[10];
            char Wmsg[120];
            char w1[] = "Values for Bank ";
            char w2[] = " were edited \r\nbut not saved. (Too late now!)\r\nSo you may want to check them.";
            strcpy(Wmsg, w1);
            strcat(Wmsg, Str(NB, PreviousBank, 0));
            strcat(Wmsg, w2);
            MsgBox((char *)"page PID_A_View", Wmsg); // Warn about unsaved edits
        }

        PIDAdvancedMsg(buf, Gray);                                       // Show loading message and hides old PIDs
        PID_Advanced_Send_Duration = 1000;                               // how many milliseconds to await PID values
        Reading_PIDS_Advanced_Now = true;                                // This tells the Ack payload parser to get PID values
        AddParameterstoQueue(SEND_PID_ADVANCED_VALUES);                   // Request PID values from RX
        SendText((char *)"t26", BankNames[BanksInUse[Bank - 1]]);           // Show bank number etc
        PIDS_Advanced_Were_Edited = false;
        PID_Advanced_Start_Time = millis(); // record start time as it's not long
    }
}

// ********************************************************************************************************
void StartPIDAdvancedView()
{
  SendCommand((char *)"page PID_A_View");   // Make Advanced visible
  CurrentView = PIDADVANCEDVIEW;
  ShowPIDAdvancedBank();                    // Show the current bank's PID Advanced values
  SendText((char *)"t27", ModelName);      // Show model name
}
#endif // PIDADVANCED_H
// *********************************************************************************************************************************/