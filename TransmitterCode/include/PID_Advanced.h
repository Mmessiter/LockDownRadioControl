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
void Display_PID_Advanced_Values(uint8_t n, uint8_t m) // display PID Advanced values n to m on screen as they are read from RX
{

    if ((millis() - PID_Advanced_Start_Time) < 500)
    return; // wait at least 500 ms because RX may be slow to respond after bank change and earlier values may be junk

    char TextFloat[10];
    for (uint8_t i = n; i < m; ++i)
    {
        if (i < MAX_PIDS_ADVANCED_BYTES)
        {
            if (i == 0) // piro compensation is boolean swich
            {
                SendValue(PID_Advanced_Labels[i], (bool)PID_Advanced_Values[i]);
                continue;
            }
            if ((i == 1) || (i == 25)) // these two are floats divided by 10
                snprintf(TextFloat, sizeof(TextFloat), "%.1f", (float)PID_Advanced_Values[i] / 10.0f);
            else
                snprintf(TextFloat, sizeof(TextFloat), "%u", (unsigned)PID_Advanced_Values[i]);

            SendText(PID_Advanced_Labels[i], TextFloat); // Send to screen
        }
    }
}
// ************************************************************************************************************/
void ReadEditedPIDAdvancedValues()
{
    for (uint8_t i = 0; i < MAX_PIDS_ADVANCED_BYTES; ++i)
    {
        if (i == 0) // Piro compensation is boolean switch
        {
            PID_Advanced_Values[i] = (uint8_t)(GetValue(PID_Advanced_Labels[i]) != 0);
            continue;
        }
        char temp[10];
        GetText(PID_Advanced_Labels[i], temp);
        if ((i == 1) || (i == 25)) // these two are floats multiplied by 10
            PID_Advanced_Values[i] = (uint8_t)(atof(temp) * 10.0f);
        else
            PID_Advanced_Values[i] = (uint8_t)(atoi(temp));
    }
}
// ************************************************************************************************************/
void ForegroundColourPIDAdvancedLabels(uint16_t Colour)
{
    for (int i = 0; i < MAX_PIDS_ADVANCED_BYTES; ++i)
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
        ForegroundColourPIDAdvancedLabels(Colour); // make text white so it isnt visible
        SendText((char *)"busy", (char *)msg);     // Show PID message
        SendCommand((char *)"vis busy,1");         // Make it visible
        SendCommand((char *)"vis b3,0");           // hide "Send" button
    }
}
// ************************************************************************************************************/
void PIDsAdvancedWereEdited()
{
    SendCommand((char *)"vis b3,1"); // show "Send" button
    PIDS_Advanced_Were_Edited = true;
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

        PIDAdvancedMsg(buf, Gray);                                // Show loading message and hides old PIDs
        PID_Advanced_Send_Duration = 1000;                        // how many milliseconds to await PID values
        Reading_PIDS_Advanced_Now = true;                         // This tells the Ack payload parser to get PID values
        AddParameterstoQueue(SEND_PID_ADVANCED_VALUES);           // Request PID values from RX
        SendText((char *)"t26", BankNames[BanksInUse[Bank - 1]]); // Show bank number etc
        PIDS_Advanced_Were_Edited = false;                        // reset edited flag
        PID_Advanced_Start_Time = millis();                       // record start time as it's not long
    }
}
// ************************************************************************************************************/
void SendEditedPID_Advanced()
{
    if (!GetConfirmation((char *)"page PID_A_View", (char *)"Send edited values to Nexus?"))
    {
        PIDS_Advanced_Were_Edited = false; // reset edited flag
        ShowPIDAdvancedBank();             // reload old PID Advanced values, undoing any edits
        return;
    }
    DelayWithDog(200);                                                      // allow LOTS of time for screen to update BEFORE sending another Nextion command
    PIDAdvancedMsg((char *)"Sending edited PID Advanced values ...", Gray); // Show sending message
    ReadEditedPIDAdvancedValues();                                          // read the edited PID Advanced values from the screen;
    AddParameterstoQueue(GET_THIRD_8_ADVANCED_PID_VALUES);                  // LAST MUST BE QUEUED FIRST!!!
    AddParameterstoQueue(GET_SECOND_9_ADVANCED_PID_VALUES);                 // the order of the other two doesn't matter ...
    AddParameterstoQueue(GET_FIRST_9_ADVANCED_PID_VALUES);                  // its a LIFO stack with 26 parametres to send
    PID_Advanced_Send_Duration = 3000;                                      // allow 3 seconds for sending all PID Advanced values
    HidePID_Advanced_Msg();                                                 // ...because this queue is a LIFO stack
    SendCommand((char *)"vis b3,0");                                        // hide "Send" button
    PIDS_Advanced_Were_Edited = false;                                      // reset edited flag
    PlaySound(BEEPCOMPLETE);                                                // let user know we're done
}
// ********************************************************************************************************
void StartPIDAdvancedView()
{
    SendCommand((char *)"page PID_A_View"); // Make Advanced visible
    CurrentView = PIDADVANCEDVIEW;
    ShowPIDAdvancedBank();              // Show the current bank's PID Advanced values
    SendText((char *)"t27", ModelName); // Show model name
}
// **********************************************************************************************************/
void EndPIDsAdvancedView()
{
    if (PIDS_Advanced_Were_Edited)
    {
        if (GetConfirmation((char *)"page PID_A_View", (char *)"Discard edited values?"))
            GotoFrontView();
    }
    else
    {
        PIDS_Advanced_Were_Edited = false;
        SendCommand((char *)"page RFView");
        CurrentView = ROTORFLIGHTVIEW;
    }
}
#endif // PIDADVANCED_H
       // *********************************************************************************************************************************/