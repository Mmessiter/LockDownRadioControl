// ******************************************** PID ************************************************************
#ifndef PIDVIEW_H
#define PIDVIEW_H
#include <Arduino.h>
#include "1Definitions.h"
// ********************************************************************************************************
void Display2PIDValues(uint8_t i) // Displays two PID values as soon as they arrive in Ack payload
{                                 // (They arrive in pairs because Ack payload has four usable bytes)
    char PID_Labels[12][4] = {"n0", "n1", "n2", "n3", "n4", "n5", "n6", "n7", "n8", "n9", "n10", "n11"};
    if (CurrentView == PIDVIEW) // Must be in PID view
    {
        SendValue(PID_Labels[i], PID_Values[i]);
        SendValue(PID_Labels[i+1], PID_Values[i + 1]);
    }
}
//************************************************************************************************************/
void ShowPIDBank() // this is called when bank is changed so new bank's PID values are shown
{
    char buf[20];
    AddParameterstoQueue(SEND_PID_VALUES); // Request PID values from RX
    sprintf(buf, "Bank: %d", Bank);        // Display which Bank
    SendText((char *)"t9", buf);
}
//************************************************************************************************************/
void StartPIDView() // this starts PID view
{
    CurrentView = PIDVIEW;
    SendCommand((char *)"page PIDView");
    ShowPIDBank();
}
/************************************************************************************************************/
#endif