
#ifndef PIDVIEW_H
#define PIDVIEW_H
#include <Arduino.h>
#include "1Definitions.h"

uint16_t PID_Values[12][4];
char PID_Labels[12][4] = {
    "n0", "n1", "n2", "n3",
    "n4", "n5", "n6", "n7",
    "n8", "n9", "n10", "n11"};

// *********************************************************************************************************
void LoadScreenPIDsArray()
{
    uint8_t b = Bank - 1;
    PID_Values[0][b] = PID_Roll_P;
    PID_Values[1][b] = PID_Roll_I;
    PID_Values[2][b] = PID_Roll_D;
    PID_Values[3][b] = PID_Roll_FF;
    PID_Values[4][b] = PID_Pitch_P;
    PID_Values[5][b] = PID_Pitch_I;
    PID_Values[6][b] = PID_Pitch_D;
    PID_Values[7][b] = PID_Pitch_FF;
    PID_Values[8][b] = PID_Yaw_P;
    PID_Values[9][b] = PID_Yaw_I;
    PID_Values[10][b] = PID_Yaw_D;
    PID_Values[11][b] = PID_Yaw_FF;

    DisplayPIDValues(Bank);
}
// ********************************************************************************************************
void DisplayPIDValues(uint8_t b) // b is 1..4
{
    if (CurrentView != PIDVIEW)
        return;
    if (b < 1 || b > 4)
        return;
    b -= 1;
    for (uint8_t i = 0; i < 12; ++i)
    {
        SendValue(PID_Labels[i], PID_Values[i][b]);
    }
}
//************************************************************************************************************/
void ShowPIDBank() // this is called when bank is changed
{
    char buf[20];
    AddParameterstoQueue(SEND_PID_VALUES);
    sprintf(buf, "Bank: %d", Bank);
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