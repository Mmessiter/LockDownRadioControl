// *************************************** Switches.h  *****************************************

// This is the code for reading the switch positions and dealing with them

#include <Arduino.h>
#include "1Definitions.h"
#ifndef SWITCHES_H
#define SWITCHES_H

/************************************************************************************************************/

FASTRUN void ReadTheSwitchesAndTrims() // and indeed read digital trims if these are fitted
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
    SafetyON = (SafetySwitch && GetSwitchPosition(SafetySwitch) != 3); // if the switch is not defined or is in position three safety is off
}

/************************************************************************************************************/
// if one or more of the top switches is defined as a channel 9 10 11 or 12, this function reads that switch as channel value: three positions only of course.
void ReadChannelSwitches9to12()
{
    uint8_t Values[3] = {0, 90, 180};
    for (uint8_t ChSwith = Ch9_SW; ChSwith <= Ch12_SW; ChSwith++)
    {
        if (TopChannelSwitch[ChSwith])                                                                   // if this switch is defined.... 1 2 3 or 4 ...
            TopChannelSwitchValue[ChSwith] = Values[(GetSwitchPosition(TopChannelSwitch[ChSwith])) - 1]; // get its position
    }
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
    if (!BuddyMasterOnWireless) // ????
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
    if (BlockBankChanges) // do not change bank if blocked
        return;
    Bank = GetSwitchPosition(BankSwitch);
    if (!Bank)
        Bank = 1; // If no bank switch was defined, use Bank 1 by default
}

/************************************************************************************************************/
void ReadAutoAndMotorSwitch()
{
    if (BlockBankChanges) // do not change bank if blocked
        return;
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

FASTRUN uint16_t ReadThreePositionSwitch(uint8_t InputDevice)
{

    if (InputDevice >= 8 && InputDevice <= 11)
    {
        uint8_t switchIndex = InputDevice - 8; // Because Ch9_SW = 0
        uint16_t value = TopChannelSwitchValue[switchIndex];
        if (value == 0)
            return ChannelMin[InputDevice];
        else if (value == 90)
            return ChannelCentre[InputDevice];
        else if (value == 180)
            return ChannelMax[InputDevice];
    }

    return 1500; // Default for channels 12+
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