// ******************************************** PID ******************************************************
// This module handles the PID View screen on the Nextion display
// It allows the user to view and edit the 12 PID values for the current bank
// It also handles sending the edited PID values back to the Rotorflight compatible flight controller via our receiver ...
// ... by queuing the appropriate parameter packets ** WHICH ARE SENT IN REVERSE ORDER! ** (- a LIFO stack!)
// **********************************************************************************************************
#ifndef PIDVIEW_H
#define PIDVIEW_H
#include <Arduino.h>
#include "1Definitions.h"

// ********************************************************************************************************
void SendBackgroundColour(const char *label, uint16_t colour)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "%s.bco=%u", label, (unsigned)colour);
    SendCommand(cmd);
}
// ********************************************************************************************************
void SendForegroundColour(const char *label, uint16_t colour)
{
    char cmd[701]; // big enough buffer?!
    snprintf(cmd, sizeof(cmd), "%s.pco=%u", label, (unsigned)colour);
    SendCommand(cmd);
}
// ********************************************************************************************************
void BackgroundColourPIDLabels(uint16_t Colour)
{
    for (int i = 0; i < 12; ++i)
    {
        SendBackgroundColour(PID_Labels[i], Colour);
    }
}
// ********************************************************************************************************
void ForegroundColourPIDLabels(uint16_t Colour)
{
    for (int i = 0; i < 12; ++i)
    {
        SendForegroundColour(PID_Labels[i], Colour);
    }
}
// ********************************************************************************************************
void ReadEditedPIDs()
{
    for (int i = 0; i < 12; ++i)
    {
        PID_Values[i] = GetValue(PID_Labels[i]); // Read edited PID values from screen ASAP!
    }
}
// ********************************************************************************************************
void Display2PIDValues(uint8_t i) // Displays two PID values as soon as they arrive in Ack payload
{                                 // (They arrive in pairs because Ack payload has four usable bytes)

    if (CurrentView == PIDVIEW && i + 1 < 12) // Must be in PID view and a valid index
    {
        SendValue(PID_Labels[i], PID_Values[i]);
        SendValue(PID_Labels[i + 1], PID_Values[i + 1]);
    }
}
// **********************************************************************************************************/
void PIDMsg(const char *msg, uint16_t Colour)
{
    if (CurrentView == PIDVIEW) // Must be in PID view
    {
        ForegroundColourPIDLabels(Colour);     // make text white so it isnt visible
        SendText((char *)"busy", (char *)msg); // Show PID message
        SendCommand((char *)"vis busy,1");     // Make it visible
        SendCommand((char *)"vis b3,0");       // hide "Send" button
    }
}
// **********************************************************************************************************/

void HidePIDMsg()
{
    if (CurrentView == PIDVIEW) // Must be in PID view
    {
        SendCommand((char *)"vis busy,0"); // Hide  message
        ForegroundColourPIDLabels(Black);  // make text black so it is visible again
    }
}
//************************************************************************************************************/
void ShowPIDBank() // this is called when bank is changed so new bank's PID values are requested from Nexus and shown
{
    if (CurrentView == PIDVIEW) // Must be in PID view
    {
        char buf[40];
        if (LedWasGreen)
        {
            snprintf(buf, sizeof(buf), "Loading PIDs for Bank %d ...", Bank);
        }
        else
        {
            snprintf(buf, sizeof(buf), "Model is not connected!"); // Model not connected message
        }
        if (PIDS_Were_Edited)
        {
            char NB[10];
            char Wmsg[120];
            char w1[] = "PIDs for Bank ";
            char w2[] = " were edited \r\nbut not saved. (Too late now!)\r\nSo you may want to check them.";
            strcpy(Wmsg, w1);
            strcat(Wmsg, Str(NB, PreviousBank, 0));
            strcat(Wmsg, w2);
            MsgBox((char *)"page PIDView", Wmsg); // Warn about unsaved edits
        }

        PIDMsg(buf, Gray);                            // Show loading message and hides old PIDs
        PID_Send_Duration = 1000;                     // how many milliseconds to await PID values
        Reading_PIDS_Now = true;                      // This tells the Ack payload parser to get PID values
        AddParameterstoQueue(SEND_PID_VALUES);        // Request PID values from RX
        snprintf(buf, sizeof(buf), "Bank: %d", Bank); // Display which Bank
        SendText((char *)"t9", buf);                  // Show bank number etc
        PIDS_Were_Edited = false;
        PID_Start_Time = millis(); // record start time as it's not long
    }
}
//************************************************************************************************************/
void PIDs_Were_edited()
{
    SendCommand((char *)"vis b3,1"); // show "Send" button
    PIDS_Were_Edited = true;
}
//***********************************************************************************************************/
void EndPIDView()
{
    if (PIDS_Were_Edited)
    {
        if (GetConfirmation((char *)"page PIDView", (char *)"Discard edited PIDs?"))
            GotoFrontView();
    }
    else
    {
        PIDS_Were_Edited = false;
        SendCommand((char *)"page RXOptionsView");
    }
}
/***********************************************************************************************************/ 

void SendEditedPIDs()
{
    if (!GetConfirmation((char *)"page PIDView", (char *)"Send edited PIDs to Nexus?"))
    {
        ShowPIDBank(); // reload old PIDs, undoing any edits
        return;
    }
    PIDMsg((char *)"Sending edited PIDs ...", Gray); // Show sending message
    DelayWithDog(150);                             // allow time for screen to update
    ReadEditedPIDs();                              // read the edited PIDs from the screen;
    AddParameterstoQueue(GET_SECOND_6_PID_VALUES); // SECOND MUST BE SENT FIRST!!! Send PID 7-12 values from TX to RX
    AddParameterstoQueue(GET_FIRST_6_PID_VALUES);  // SECOND MUST BE SENT FIRST!!! Send PID 1-6 values from TX to RX
    HidePIDMsg();                                  // SECOND MUST BE SENT FIRST!!!  because this queue is a LIFO stack
    SendCommand((char *)"vis b3,0");               // hide "Send" button
    PIDS_Were_Edited = false;
    PlaySound(BEEPCOMPLETE); // let user know we're done
}
//************************************************************************************************************/
void StartPIDView() // this starts PID view
{
    if (!SafetyON)
    {
        MsgBox(RXOptionsView, (char *)"Please enable 'Safety'!");
        return;
    }
    CurrentView = PIDVIEW;               // Set current view
    SendCommand((char *)"page PIDView"); // Go to PID view page
    ShowPIDBank();                       // Show the current bank's PIDs
    SendText((char *)"t11", ModelName);  // Show model name
    PIDS_Were_Edited = false;
}
#endif
