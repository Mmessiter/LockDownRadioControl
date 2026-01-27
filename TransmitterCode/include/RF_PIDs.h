// ******************************************** Rotorflight PID ******************************************************
// This module handles the  Rotorflight PID View screen on the Nextion display
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
    char cmd[80];
    snprintf(cmd, sizeof(cmd), "%s.bco=%u", label, (unsigned)colour);
    SendCommand(cmd);
}
// ********************************************************************************************************
void SendForegroundColour(const char *label, uint16_t colour)
{
    char cmd[148];
    snprintf(cmd, sizeof(cmd), "%s.pco=%u", label, (unsigned)colour);
    SendCommand(cmd);
}
// ********************************************************************************************************
void BackgroundColourPIDLabels(uint16_t Colour)
{
    for (int i = 0; i < MAX_PID_WORDS + 3; ++i)
    {
        SendBackgroundColour(PID_Labels[i], Colour);
    }
}
// ********************************************************************************************************
void ForegroundColourPIDLabels(uint16_t Colour)
{
    for (int i = 0; i < MAX_PID_WORDS + 3; ++i)
    {
        SendForegroundColour(PID_Labels[i], Colour);
    }
}

// ********************************************************************************************************
void ReadEditedPIDs()
{
    for (int i = 0; i < MAX_PID_WORDS; ++i)
    {
        PID_Values[i] = GetValue(PID_Labels[i]);
    }
    uint8_t p = MAX_PID_WORDS;
    for (int i = 0; i < 3; ++i)
    {
        PID_Boost_Values[i] = GetValue(PID_Labels[i + p]);
       
    }   
    return;
}
// ********************************************************************************************************
void Display2PIDValues(uint8_t i) // Displays two PID values as soon as they arrive in Ack payload th
{                                 // (They arrive in pairs because Ack payload has four usable bytes)
    if (CurrentMode == RESTORE_RF_SETTINGS)
        return;
    if (CurrentView == PIDVIEW && i + 1 < MAX_PID_WORDS) // Must be in PID view and a valid index
    {
        SendValue(PID_Labels[i], PID_Values[i]);
        SendValue(PID_Labels[i + 1], PID_Values[i + 1]);
    }
}
// **********************************************************************************************************/
void DisplayBoostPidValues() // Displays Boost PID values as soon as they arrive in Ack payload Theses are at the end of the normal PIDs ... tacked on
{
    if (CurrentMode == RESTORE_RF_SETTINGS)
        return;
    if (CurrentView == PIDVIEW) // Must be in PID view
    {
        SendValue(PID_Labels[12], PID_Boost_Values[0]); // Roll Boost
        SendValue(PID_Labels[13], PID_Boost_Values[1]); // Pitch Boost
        SendValue(PID_Labels[14], PID_Boost_Values[2]); // Yaw Boost
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
        SendCommand((char *)"vis b2,0");       // Make Advanced invisible
        SendCommand((char *)"vis b3,0");       // hide "Send" button
    }
}
// **********************************************************************************************************/
void HidePIDMsg()
{
    if (CurrentView == PIDVIEW) // Must be in PID view
    {
        SendCommand((char *)"vis busy,0"); // Hide  message
        SendCommand((char *)"vis b2,1");   // Make Advanced visible
        ForegroundColourPIDLabels(Black);  // make text black so it is visible again
    }
}
//***********************************************************************************************************/
void ShowLocalBank()
{
    for (int i = 0; i < MAX_PID_WORDS; ++i)
        if (CurrentView == PIDVIEW && i + 1 < MAX_PID_WORDS) // Must be in PID view and a valid index
        {
            SendValue(PID_Labels[i], Saved_PID_Values[i][Bank - 1]);
        }
    uint8_t p = MAX_PID_WORDS;
    for (int i = 0; i < 3; ++i)
    {
        SendValue(PID_Labels[i + p], Saved_PID_Values[i + p][Bank - 1]);
    }
    HidePIDMsg();
}
//************************************************************************************************************/
void ShowPIDBank() // this is called when bank is changed so new bank's PID values are requested from Nexus and shown
{
    if (CurrentView == PIDVIEW) // Must be in PID view
    {
        char buf[40];
        strcpy(buf, "Loading PIDs for  ");
        strcat(buf, BankNames[BanksInUse[Bank - 1]]);
        strcat(buf, " ...");
        SendText((char *)"t9", BankNames[BanksInUse[Bank - 1]]); // Show bank number etc
        if (!LedWasGreen)
        {                      // Model not connected so show local saved PIDs
            PIDMsg(buf, Gray); // Show loading message and hides old PIDs
            ShowLocalBank();
            return;
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
        PIDMsg(buf, Gray);                     // Show loading message and hides old PIDs
        PID_Send_Duration = MSP_WAIT_TIME;              // how many milliseconds to await PID values
        Reading_PIDS_Now = true;               // This tells the Ack payload parser to get PID values
        AddParameterstoQueue(SEND_PID_VALUES); // Request PID values from RX

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
//************************************************************************************************************/
void SaveToLocalBank()
{
    PIDMsg((char *)"Saving edited PIDs ...", Gray); // Show sending message
    ReadEditedPIDs();                               // read the edited PIDs from the screen;
    for (int i = 0; i < MAX_PID_WORDS; ++i)
    {
        Saved_PID_Values[i][Bank - 1] = PID_Values[i];
    }
    for (int i = 0; i < 3; ++i)
    {
        Saved_PID_Values[i + MAX_PID_WORDS][Bank - 1] = PID_Boost_Values[i];
    }
    SaveOneModel(ModelNumber);       // save all to SD card
    SendCommand((char *)"vis b3,0"); // hide "Send" button
    HidePIDMsg();
    PIDS_Were_Edited = false; // reset edited flag
    PlaySound(BEEPCOMPLETE);  // let user know we're done
}

/***********************************************************************************************************/

void SendEditedPIDs()
{
    if (!LedWasGreen) // Model not connected so save to local PIDs
    {
        SaveToLocalBank();
        return;
    }
    PlaySound(BEEPMIDDLE);
    PIDS_Were_Edited = false;
    HidePIDMsg();
    PIDMsg((char *)"Sending edited PIDs ...", Gray); // Show sending message
    DelayWithDog(100);                               // allow LOTS of time for screen to update BEFORE sending another Nextion command
    ReadEditedPIDs();                                // read the edited PIDs from the screen;
    AddParameterstoQueue(GET_SECOND_9_PID_VALUES);   // SECOND MUST BE QUEUED FIRST!!! Send PID 7-12 values from TX to RX
    AddParameterstoQueue(GET_FIRST_6_PID_VALUES);    // SECOND MUST BE QUEUED FIRST!!! Send PID 1-6 values from TX to RX
    HidePIDMsg();                                    // SECOND MUST BE QUEUED FIRST!!!  because this queue is a LIFO stack
    SendCommand((char *)"vis b3,0");                 // hide "Send" button
    PlaySound(BEEPCOMPLETE);                         // let user know we're done
}
//************************************************************************************************************/
void StartPIDView() // this starts PID view
{
    if (SendBuffer[ArmingChannel - 1] > 1000) // Safety is on if value > 1000
    {
        PlaySound(WHAHWHAHMSG); // let user know we're in trouble
        MsgBox((char *)"page RFView", (char *)"Model is armed and dangerous!\r\n(Disarm model to edit PIDs.)");
        return;
    }
    CurrentView = PIDVIEW;               // Set current view
    SendCommand((char *)"page PIDView"); // Go to PID view page
    ShowPIDBank();                       // Show the current bank's PIDs
    SendText((char *)"t11", ModelName);  // Show model name
    PIDS_Were_Edited = false;
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
        SendCommand((char *)"page RFView");
        CurrentView = ROTORFLIGHTVIEW;
        ShowRFBank();
    }
}
#endif
