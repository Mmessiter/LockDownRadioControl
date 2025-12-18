// ******************************************** PID ******************************************************
#ifndef PIDVIEW_H
#define PIDVIEW_H
#include <Arduino.h>
#include "1Definitions.h"
char PID_Labels[12][4] = {"n0", "n1", "n2", "n3", "n4", "n5", "n6", "n7", "n8", "n9", "n10", "n11"};

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
    char cmd[64];
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
        PID_Values[i] = GetValue(PID_Labels[i]);
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
    }
}
// **********************************************************************************************************/

void HidePIDMsg()
{
    if (CurrentView == PIDVIEW) // Must be in PID view
    {
        SendCommand((char *)"vis busy,0"); // Hide  message
        ForegroundColourPIDLabels(0);      // make text black so it is visible again
    }
}
//************************************************************************************************************/
void ShowPIDBank() // this is called when bank is changed so new bank's PID values are requested from Nexus and shown
{
    if (CurrentView == PIDVIEW) // Must be in PID view
    {
        char buf[40];
        snprintf(buf, sizeof(buf), "Loading PIDs for Bank %d ...", Bank);
        PIDMsg(buf, Gray);                            // Show loading message and hides old PIDs
        PID_Send_Duration = 1000;                     // how many milliseconds to await PID values
        Reading_PIDS_Now = true;                      // This tells the Ack payload parser to get PID values
        AddParameterstoQueue(SEND_PID_VALUES);        // Request PID values from RX
        snprintf(buf, sizeof(buf), "Bank: %d", Bank); // Display which Bank
        SendText((char *)"t9", buf);                  // Show bank number etc
        PID_Start_Time = millis();                    // record start time as it's not long
    }
}
//************************************************************************************************************/
void StartPIDView() // this starts PID view
{
    CurrentView = PIDVIEW;               // Set current view
    SendCommand((char *)"page PIDView"); // Go to PID view page
    ShowPIDBank();                       // Show the current bank's PIDs
}
/************************************************************************************************************/

void SendEditedPIDs()
{
    PIDMsg((char *)"Sending edited PIDs ...", Gray); // Show sending message
    ReadEditedPIDs();                                 // read the edited PIDs from the screen;
    AddParameterstoQueue(GET_FIRST_6_PID_VALUES);     // Send PID 1-6 values from TX to RX
    AddParameterstoQueue(GET_SECOND_6_PID_VALUES);    // Send PID 7-12 values from TX to RX
    HidePIDMsg();
}
#endif
