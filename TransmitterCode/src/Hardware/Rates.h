// ********************** Rates.h ***********************************************
// This file contains  the functions for reading and writing to the SD card for MODEL MEMORIES and TRANSMITTER PARAMETERS

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#include <SD.h>
#include <SPI.h>

#ifndef RATES_H
    #define RATES_H


/************************************************************************************************************/

void ReadDRSwitch(bool sw1, bool sw2, bool rev) // Dual Rate Switch
{
    if ((sw1 == false) && (sw2 == false))
    {
        DualRateInUse = 2;
    }
    else {
        if (rev) {
            if (sw1) DualRateInUse = 1;
            if (sw2) DualRateInUse = 3;
        }
        else {
            if (sw1) DualRateInUse = 3;
            if (sw2) DualRateInUse = 1;
        }
    }
}


/************************************************************************************************************/

void ReadDualRatesFromScreen(){

    char rate1[]     = "rate1";
    char rate2[]     = "rate2";
    char rate3[]     = "rate3";
    char ChNumber1[] = "n2";
    char ChNumber2[] = "n0";
    char ChNumber3[] = "n1";
    char ChNumber4[] = "n3";
    char ChNumber5[] = "n6";
    char ChNumber6[] = "n4";
    char ChNumber7[] = "n5";
    char ChNumber8[] = "n7";

    Drate1 = GetValue(rate1);
    Drate2 = GetValue(rate2);
    Drate3 = GetValue(rate3);
    DualRateChannels[0] = GetValue(ChNumber1);
    DualRateChannels[1] = GetValue(ChNumber2);
    DualRateChannels[2] = GetValue(ChNumber3);
    DualRateChannels[3] = GetValue(ChNumber4);
    DualRateChannels[4] = GetValue(ChNumber5);
    DualRateChannels[5] = GetValue(ChNumber6);
    DualRateChannels[6] = GetValue(ChNumber7);
    DualRateChannels[7] = GetValue(ChNumber8);
}

/************************************************************************************************************/

void RefreshDualRatesNew(){
        ReadDualRatesFromScreen();
        DisplayDualRateValues();
}
/************************************************************************************************************/
 void ReadDualRateSwitch(){
     
    DualRateInUse = 4; // default to 100%

    if (DualRatesSwitch == 4) ReadDRSwitch(Switch[2], Switch[3], SWITCH4Reversed);
    if (DualRatesSwitch == 3) ReadDRSwitch(Switch[0], Switch[1], SWITCH3Reversed);
    if (DualRatesSwitch == 2) ReadDRSwitch(Switch[4], Switch[5], SWITCH2Reversed);
    if (DualRatesSwitch == 1) ReadDRSwitch(Switch[6], Switch[7], SWITCH1Reversed);
    
    if (DualRateRate[Bank - 1] > 0) DualRateInUse = DualRateRate[Bank - 1];   // if using a forced rate !
   
    if (DualRateInUse == 1) DualRateValue = Drate1;
    if (DualRateInUse == 2) DualRateValue = Drate2;
    if (DualRateInUse == 3) DualRateValue = Drate3;
    if (DualRateInUse == 4) DualRateValue = 100; // Switch not in use, so use 100%

    if (PreviousDualRateInUse != DualRateInUse) {
        PreviousDualRateInUse = DualRateInUse;
        LogNewRateInUse();
        LastShowTime          = 0;
        LastTimeRead          = 0;
        if (AnnounceBanks) {
            switch (DualRateInUse)
            {
                case 1:
                    PlaySound(RATE1);
                    break;
                case 2:
                    PlaySound(RATE2);
                    break;
                case 3:
                    PlaySound(RATE3);
                    break;
                default:
                    break;   
            }
        }
    }
 }

/******************************************************************************************************************************/
void SetNewDualRate(){

        char buf[40];
        char t3[] = "t3";
        char join[] = " will use ";
        char quote[] = "\'";
        char rates[4][12] = {{"rate switch"}, {"rate 1"}, {"rate 2"}, {"rate 3"}};
        char rate[] = "rate";
        
        DualRateRate[Bank-1] = TextIn[4];

        if (DualRateRate[Bank-1] > 3)  {
            DualRateRate[Bank-1] = 0;
            SendValue(rate, 0);
        }
        strcpy(buf,quote);
        strcat(buf, BankTexts[BanksInUse[Bank-1]]);
        strcat(buf,quote);
        strcat(buf, join);
        if (DualRateRate[Bank-1]){    // if not zero which means "the switch"
            strcat(buf, rates[DualRateRate[Bank-1]]);
        }else{
            strcat(buf, rates[0]);
        }
        SendText(t3, buf); 
}

/******************************************************************************************************************************/
 void CheckSelectedRatesMode(){ 

    char rate[]= "rate";
    static uint8_t lastbank = 0;

    uint8_t rateMode = GetValue(rate);
    if ((rateMode != DualRateRate[Bank-1]) || (Bank !=lastbank))    // if the rate has changed
    {
        TextIn[4] = rateMode;
        SetNewDualRate();
        lastbank = Bank;
    }
 }


/******************************************************************************************************************************/

void ReadDualRatesValues() 
{
    if (ScreenData[10]  > 15) return;                   //  if channel number is out of range means it is not updated yet
    if(!ScreenData[0]) return;                          // if no data, return
    if(!ScreenData[1]) return;                          // if no data, return
    if(!ScreenData[2]) return;                          // if no data, return
    Drate1 = ScreenData[0];
    if (Drate1 > MAXDUALRATE) Drate1 = MAXDUALRATE;
    Drate2 = ScreenData[1];
    if (Drate2 > MAXDUALRATE) Drate2 = MAXDUALRATE;
     Drate3 = ScreenData[2];
    if (Drate3 > MAXDUALRATE) Drate3 = MAXDUALRATE;
    DualRateChannels[0]     = CheckRange(ScreenData[3], 0, 8);
    DualRateChannels[1]     = CheckRange(ScreenData[4], 0, 8);
    DualRateChannels[2]     = CheckRange(ScreenData[5], 0, 8);
    DualRateChannels[3]     = CheckRange(ScreenData[6], 0, 8);
    DualRateChannels[4]     = CheckRange(ScreenData[7], 0, 8);
    DualRateChannels[5]     = CheckRange(ScreenData[8], 0, 8);
    DualRateChannels[6]     = CheckRange(ScreenData[9], 0, 8);
    DualRateChannels[7]     = CheckRange(ScreenData[10], 0, 8);
    DualRateRate[Bank-1]    = ScreenData[11]; 
}


/******************************************************************************************************************************/

void ShowDualRateChannelsName(char* nm, uint8_t n)
{
    char nu[] = "Not used";

    if (n) {
        SendText(nm, ChannelNames[n - 1]); 
    }
    else {
        SendText(nm, nu); 
    } // not used if zero
}


/******************************************************************************************************************************/

void    DisplayNewDualRateBank(){
        char rate[] = "rate";
        char t1[] = "t1";                                              
        SendValue(rate, DualRateRate[Bank - 1]);
        SendText(t1, BankTexts[BanksInUse[Bank - 1]]);                  // display bank name in Rates view 
        CheckSelectedRatesMode();                                       
}

/******************************************************************************************************************************/

void DualRatesEnd() 
{
    ReadDualRatesFromScreen();
    SaveOneModel(ModelNumber);
    StartModelSetup();
}

/******************************************************************************************************************************/

void DisplayDualRateValues()
{
    char rate1[]     = "rate1";
    char rate2[]     = "rate2";
    char rate3[]     = "rate3";
    char ChName1[]   = "t12";
    char ChName2[]   = "t8";
    char ChName3[]   = "t11";
    char ChName4[]   = "t13";
    char ChName5[]   = "t16";
    char ChName6[]   = "t14";
    char ChName7[]   = "t15";
    char ChName8[]   = "t17";
    char ChNumber1[] = "n2";
    char ChNumber2[] = "n0";
    char ChNumber3[] = "n1";
    char ChNumber4[] = "n3";
    char ChNumber5[] = "n6";
    char ChNumber6[] = "n4";
    char ChNumber7[] = "n5";
    char ChNumber8[] = "n7";
    SendValue(rate1, Drate1);
    SendValue(rate2, Drate2);
    SendValue(rate3, Drate3);
    SendValue(ChNumber1, DualRateChannels[0]);
    SendValue(ChNumber2, DualRateChannels[1]);
    SendValue(ChNumber3, DualRateChannels[2]);
    SendValue(ChNumber4, DualRateChannels[3]);
    SendValue(ChNumber5, DualRateChannels[4]);
    SendValue(ChNumber6, DualRateChannels[5]);
    SendValue(ChNumber7, DualRateChannels[6]);
    SendValue(ChNumber8, DualRateChannels[7]);
    ShowDualRateChannelsName(ChName1, DualRateChannels[0]);
    ShowDualRateChannelsName(ChName2, DualRateChannels[1]);
    ShowDualRateChannelsName(ChName3, DualRateChannels[2]);
    ShowDualRateChannelsName(ChName4, DualRateChannels[3]);
    ShowDualRateChannelsName(ChName5, DualRateChannels[4]);
    ShowDualRateChannelsName(ChName6, DualRateChannels[5]);
    ShowDualRateChannelsName(ChName7, DualRateChannels[6]);
    ShowDualRateChannelsName(ChName8, DualRateChannels[7]);
    DisplayNewDualRateBank();
}

/******************************************************************************************************************************/

void DualRatesRefresh()
{
    DelayWithDog(200);
    ReadDualRatesValues();
    DisplayDualRateValues();
}

/******************************************************************************************************************************/

void DualRatesStart()
{
    char GotoDualRates[] = "page DualRatesView";
    SendCommand(GotoDualRates);
    CurrentView = DUALRATESVIEW;
    DisplayDualRateValues();
    UpdateModelsNameEveryWhere();
}
/*********************************************************************************************************************************/

void CheckDualRatesValues()
{

    bool KO = false;
    if (Drate1 > MAXDUALRATE) KO = true;
    if (Drate2 > MAXDUALRATE) KO = true;
    if (Drate3 > MAXDUALRATE) KO = true;

    for (int i = 0; i < 8; ++i) {
        if (DualRateChannels[i] > 16) KO = true;
    }
    if (KO) {
        //  UseDualRates = false;
        Drate1 = 100;
        Drate2 = 75; // 1 is nearly always 100%
        Drate3 = 50;

        DualRateChannels[0] = 1;
        DualRateChannels[1] = 2;
        DualRateChannels[2] = 4;
        DualRateChannels[3] = 0;
        DualRateChannels[4] = 0;
        DualRateChannels[5] = 0;
        DualRateChannels[6] = 0;
        DualRateChannels[7] = 0;
    }
}


/*********************************************************************************************************************************/

float CalculateRate(short int Curve, short int OutputChannel, float rate)
{

    switch (Curve) { // Curve is 1 - 5, low to hi.
        case 0:
            return (((MinDegrees[Bank][OutputChannel]) - 90) * (rate / 100)) + 90;
        case 1:
            return (((MidLowDegrees[Bank][OutputChannel]) - 90) * (rate / 100)) + 90;
        case 2:
            return (((CentreDegrees[Bank][OutputChannel]) - 90) * (rate / 100)) + 90;
        case 3:
            return (((MidHiDegrees[Bank][OutputChannel]) - 90) * (rate / 100)) + 90;
        case 4:
            return (((MaxDegrees[Bank][OutputChannel]) - 90) * (rate / 100)) + 90;
        default:
            return 0;
    }
}

/*********************************************************************************************************************************/

float UseFullRate(short int Curve, uint8_t OutputChannel)
{

    switch (Curve) { // Curve is 1 - 5, low to hi.
        case 0:
            return MinDegrees[Bank][OutputChannel];
        case 1:
            return MidLowDegrees[Bank][OutputChannel];
        case 2:
            return CentreDegrees[Bank][OutputChannel];
        case 3:
            return MidHiDegrees[Bank][OutputChannel];
        case 4:
            return MaxDegrees[Bank][OutputChannel];
        default:
            return 0;
    }
}

/*********************************************************************************************************************************/

void GetCurveDots(uint16_t OutputChannel, uint16_t TheRate) // This for the Dual Rates function
{                                                           // This for the Dual Rates function
                                                            // Effectively, it just copies the Y dot's magnitude on the curve, but might reduce the extent if rate is not 100 and channel specified
    if (TheRate != 100) {                                   // Not 100% ?
        for (int j = 0; j < 8; ++j) {                       // 8 possible rates in any position of output
            if (DualRateChannels[j]) {                      // non zero?
                if (OutputChannel + 1 == DualRateChannels[j]) {
                    for (int i = 0; i < 5; ++i) CurveDots[i] = CalculateRate(i, OutputChannel, TheRate);
                    return;
                }
            }
        }
    }
    for (int i = 0; i < 5; ++i) CurveDots[i] = UseFullRate(i, OutputChannel); // ... channel not used so 100%
}

#endif
