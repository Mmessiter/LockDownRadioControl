#include <Arduino.h>
#include "1Definitions.h"
#ifndef PIDVIEW_H
#define PIDVIEW_H

uint16_t PID_Values[12][4] = { // 12 settings for each of 4 banks. dummy values only for editing tests....

    //  BANKS 1-4 are columns

    {111, 112, 113, 114}, // P   ROLL
    {115, 116, 117, 117}, // I   ROLL
    {119, 120, 121, 122}, // D   ROLL
    {123, 124, 125, 126}, // FF  ROLL

    {300, 200, 100, 400}, // P   PITCH
    {301, 201, 101, 401}, // I   PITCH
    {302, 202, 102, 402}, // D   PITCH
    {303, 203, 103, 403}, // FF  PITCH

    {300, 200, 100, 400},  // P  YAW
    {301, 201, 101, 401},  // I  YAW
    {302, 202, 102, 402},  // D  YAW
    {303, 203, 103, 403}}; // FF YAW

char PID_Labels[12][4] = {
    "n0", "n1", "n2", "n3",
    "n4", "n5", "n6", "n7",
    "n8", "n9", "n10", "n11"};

uint8_t PrevBank = 0;

// *******************************************************************************************************
void GetPidValues(uint8_t b) // read values before displaying new bank's values
{
    if (b < 1 || b > 4)
        return;
    b -= 1;
    SendCommand((char *)"vis Progress,1");
    for (uint8_t i = 0; i < 12; ++i)
    {
        PID_Values[i][b] = GetValue(PID_Labels[i]);
        SendValue((char *)"Progress", (100 * (i + 1) / 12));
    }
}
// ********************************************************************************************************
// ********************************************************************************************************
void DisplayPIDValues(uint8_t b) // b is 1..4
{
    if (b < 1 || b > 4)
        return;
    b -= 1;
    for (uint8_t i = 0; i < 12; ++i)
        SendValue(PID_Labels[i], PID_Values[i][b]);
    PrevBank = Bank;
    SendCommand((char *)"vis Progress,0");
}

//************************************************************************************************************/

void ShowPIDBank() // this is called when bank is changed
{
    char buf[20];
    sprintf(buf, "Bank: %d", Bank);
    SendText((char *)"t9", buf);
    GetPidValues(PrevBank);
    DisplayPIDValues(Bank);
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