// *************************************** Switches.h  *****************************************

// This is the code for reading the switch positions and dealing with them

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef SWITCHES_H
#define SWITCHES_H

/************************************************************************************************************/

FASTRUN void ReadSwitches() // and indeed read digital trims if these are fitted
{
    byte flag = 0;
    static uint8_t PreviousTrim = 255;
    for (int i = 0; i < 8; ++i)
    {
        Switch[i] = !digitalRead(SwitchNumber[i]);   // These are reversed because they are active low
        TrimSwitch[i] = !digitalRead(TrimNumber[i]); // These are reversed because they are active low
        if (TrimSwitch[i])
            ++flag; // A finger is on a trim lever...
        if ((TrimSwitch[i]) && (PreviousTrim != i))
        {                     // is it a new one?
            TrimTimer = 0;    // it IS a new one, so no delay please.
            PreviousTrim = i; // remember which trim it was
        }
    }
    if (flag > 1)
    {                                             // One at a time please!!
        TrimRepeatSpeed = DEFAULTTRIMREPEATSPEED; // Restore default trim repeat speed
        for (int i = 0; i < 8; ++i)
        {
            (TrimSwitch[i]) = 0;
            flag = 0;
        }
    }
    if (!flag)
    {
        PreviousTrim = 254;                       // Previous trim must now match none
        TrimRepeatSpeed = DEFAULTTRIMREPEATSPEED; // Restore default trim repeat speed
    }
}

/************************************************************************************************************/

void ReadSafetySwitch()
{
    SafetyON = (SafetySwitch && GetSwitchPosition(SafetySwitch) != 3);// if the switch is not defined or is in position three safety is off
}
/************************************************************************************************************/

uint8_t ReadCHSwitch(bool sw1, bool sw2, bool rev) //
{
    uint8_t ttmp = 90;
    if (sw1 == false && sw2 == false)
        ttmp = 90;
    if (rev)
    {
        if (sw1)
            ttmp = 0;
        if (sw2)
            ttmp = 180;
    }
    else
    {
        if (sw1)
            ttmp = 180;
        if (sw2)
            ttmp = 0;
    }
    return ttmp;
}
/************************************************************************************************************/

uint8_t CheckSwitch(uint8_t swt)
{
    uint8_t rtv = 90;
    if (swt == 1)
        rtv = ReadCHSwitch(Switch[7], Switch[6], SwitchReversed[0]);
    if (swt == 2)
        rtv = ReadCHSwitch(Switch[5], Switch[4], SwitchReversed[1]);
    if (swt == 3)
        rtv = ReadCHSwitch(Switch[0], Switch[1], SwitchReversed[2]);
    if (swt == 4)
        rtv = ReadCHSwitch(Switch[2], Switch[3], SwitchReversed[3]);
    return rtv;
}
/************************************************************************************************************/

void ReadChannelSwitches9to12(){
    Channel9SwitchValue = CheckSwitch(Channel9Switch);
    Channel10SwitchValue = CheckSwitch(Channel10Switch);
    Channel11SwitchValue = CheckSwitch(Channel11Switch);
    Channel12SwitchValue = CheckSwitch(Channel12Switch);
}

/************************************************************************************************************/

void ViewSwitches() // shows state of all 8 switches' pin numbers (DEBUG TESTS ONLY)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        Look1("Switch");
        Look1(i);
        Look1(": ");
        Look(Switch[i]);
    }
    Look("");
}

/************************************************************************************************************/
uint8_t GetSwitchPosition(uint8_t ThisSwitchNumber) // returns 1,2, or 3 for any of the four switches and handles reversed too. :-)
{
    static const uint8_t Pin[4][2] = {
        {7, 6},  // Switch 1
        {5, 4},  // Switch 2
        {1, 0},  // Switch 3
        {3, 2}}; // Switch 4

    if (ThisSwitchNumber < 1 || ThisSwitchNumber > 4)
        return 0; // or some error code

    uint8_t top = Pin[ThisSwitchNumber - 1][0];
    uint8_t bottom = Pin[ThisSwitchNumber - 1][1];

    if (SwitchReversed[ThisSwitchNumber - 1])
        swap(&top, &bottom);

    if (Switch[top])
        return 3; // top
    else if (!Switch[bottom])
        return 2; // middle
    else
        return 1; // bottom
}

/************************************************************************************************************/
void ReadBuddySwitch()
{
    if (!BuddyMasterOnWireless)
        return;

    static uint8_t LastBuddyState = BUDDY_OFF;

    switch (GetSwitchPosition(BuddySwitch))
    {
    case 1:
        BuddyState = Buddy_Low_Position;
        break;
    case 2:
        BuddyState = Buddy_Mid_Position;
        break;
    case 3:
        BuddyState = Buddy_Hi_Position;
        break;
    default:
        return; // Ignore other values
    }

    if (BuddyState != LastBuddyState)
    {
        LastBuddyState = BuddyState;
        LogBuddyChange();
    }
}
/************************************************************************************************************/
void ReadBankSwitch()
{
    Bank = GetSwitchPosition(BankSwitch);
}

/************************************************************************************************************/
void ReadAutoAndMotorSwitch()
{
    switch (GetSwitchPosition(Autoswitch))
    {
    case 1:
        MotorEnabled = true;
        break;
    case 2:
        Bank = 4;
        MotorEnabled = true;
        break;
    case 3:
        Bank = 4;
        MotorEnabled = false;
        break;
    default:
        break;
    }
}
/************************************************************************************************************/
void ReadDualRateSwitch()
{
    if (!BuddyPupilOnWireless || BuddyHasAllSwitches)
    {
        // Read hardware switch only when allowed
        DualRateInUse = GetSwitchPosition(DualRatesSwitch);
        if (DualRateInUse == 0) // if switch not used
            DualRateInUse = 4;
    }

    // Assign actual dual rate value
    switch (DualRateInUse)
    {
    case 1:
        DualRateValue = Drate1;
        break;
    case 2:
        DualRateValue = Drate2;
        break;
    case 3:
        DualRateValue = Drate3;
        break;
    case 4:
    default:
        DualRateValue = 100;
        break;
    }

    // Respond to a change
    if (PreviousDualRateInUse != DualRateInUse)
    {
        PreviousDualRateInUse = DualRateInUse;
        LogNewRateInUse();
        LastShowTime = 0;
        LastTimeRead = 0;

        if (AnnounceBanks)
        {
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

/*********************************************************************************************************************************/
FASTRUN uint16_t ReadThreePositionSwitch(uint8_t l) // This returns the input only
{
    uint16_t k = 0;
    switch (l)
    {
    case 8:
        if (Channel9SwitchValue == 0)
            k = ChannelMin[l];
        if (Channel9SwitchValue == 90)
            k = ChannelCentre[l];
        if (Channel9SwitchValue == 180)
            k = ChannelMax[l];
        break;
    case 9:
        if (Channel10SwitchValue == 0)
            k = ChannelMin[l];
        if (Channel10SwitchValue == 90)
            k = ChannelCentre[l];
        if (Channel10SwitchValue == 180)
            k = ChannelMax[l];
        break;
    case 10:
        if (Channel11SwitchValue == 0)
            k = ChannelMin[l];
        if (Channel11SwitchValue == 90)
            k = ChannelCentre[l];
        if (Channel11SwitchValue == 180)
            k = ChannelMax[l];
        break;
    case 11:
        if (Channel12SwitchValue == 0)
            k = ChannelMin[l];
        if (Channel12SwitchValue == 90)
            k = ChannelCentre[l];
        if (Channel12SwitchValue == 180)
            k = ChannelMax[l];
        break;
    default:
        k = 1500; // Centre is default for channels 13,14,15 & 16 because no input is possible
        break;
    }
    return k;
}

/************************************************************************************************************/
void CalibrateEdgeSwitches()
{ // This function avoids the need to rotate the four edge switches if installed backwards
    for (int i = 0; i < 8; ++i)
    {
        if (digitalRead(SwitchNumber[i]))
        {
            if (i == 0)
                swap(&SwitchNumber[i], &SwitchNumber[i + 1]); // swap over switches' pin number if wrongly installed
            if (i == 2)
                swap(&SwitchNumber[i], &SwitchNumber[i + 1]); // swap over switches' pin number if wrongly installed
            if (i == 4)
                swap(&SwitchNumber[i], &SwitchNumber[i + 1]); // swap over switches' pin number if wrongly installed
            if (i == 6)
                swap(&SwitchNumber[i], &SwitchNumber[i + 1]); // swap over switches' pin number if wrongly installed
        }
    }
}

#endif