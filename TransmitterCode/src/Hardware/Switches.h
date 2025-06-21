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
    SafetyON = false;
    if ((SafetySwitch == 1) && (Switch[7] == SwitchReversed[0]))
        SafetyON = true;
    if ((SafetySwitch == 2) && (Switch[5] == SwitchReversed[1]))
        SafetyON = true;
    if ((SafetySwitch == 3) && (Switch[1] == SwitchReversed[2]))
        SafetyON = true;
    if ((SafetySwitch == 4) && (Switch[2] == SwitchReversed[3]))
        SafetyON = true;
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
uint8_t GetSwitchPosition(uint8_t Sw_Number) // returns 1,2, or 3 for any of the four switches and handles reversed too. :-)
{
    static const uint8_t Pin[4][2] = {
        {7, 6},  // Switch 1
        {5, 4},  // Switch 2
        {1, 0},  // Switch 3
        {3, 2}}; // Switch 4

    if (Sw_Number < 1 || Sw_Number > 4)
        return 0; // or some error code

    uint8_t top = Pin[Sw_Number - 1][0];
    uint8_t bottom = Pin[Sw_Number - 1][1];

    if (SwitchReversed[Sw_Number - 1])
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
    if (BuddyMasterOnWireless)
    {
        static uint8_t LastBuddyState = BUDDY_OFF;
        uint8_t Pos = GetSwitchPosition(BuddySwitch);
        if (Pos == 1)
            BuddyState = Buddy_Low_Position;

        if (Pos == 2)
            BuddyState = Buddy_Mid_Position;

        if (Pos == 3)
            BuddyState = Buddy_Hi_Position;
        
        if (BuddyState != LastBuddyState)
        {
            LastBuddyState = BuddyState;
            LogBuddyChange();
        }
    }
}

/************************************************************************************************************/
// void ReadBuddySwitchOLD()
// {

//     //  ViewSwitches();
//     // Look(GetSwitchPosition(3));

//     if (BuddyMasterOnWireless)
//     {
//         BuddyState = BUDDY_OFF; // default = MASTER
//         static uint8_t LastBuddyState = BUDDY_OFF;

//         if ((Buddy_Switch_Mode == M_M_B) || (Buddy_Switch_Mode == M_N_B))
//         {
//             if ((BuddySwitch == 1) && (Switch[7] == SwitchReversed[0])) // Switch is at top
//                 BuddyState = BUDDY_ON;
//             if ((BuddySwitch == 2) && (Switch[5] == SwitchReversed[1]))
//                 BuddyState = BUDDY_ON;
//             if ((BuddySwitch == 3) && (Switch[1] == SwitchReversed[2]))
//                 BuddyState = BUDDY_ON;
//             if ((BuddySwitch == 4) && (Switch[2] == SwitchReversed[3]))
//                 BuddyState = BUDDY_ON;
//         }

//         if ((Buddy_Switch_Mode == M_M_N))
//         {
//             if ((BuddySwitch == 1) && (Switch[7] == SwitchReversed[0])) // Switch is also at top
//                 BuddyState = BUDDY_NUDGE;
//             if ((BuddySwitch == 2) && (Switch[5] == SwitchReversed[1]))
//                 BuddyState = BUDDY_NUDGE;
//             if ((BuddySwitch == 3) && (Switch[1] == SwitchReversed[2]))
//                 BuddyState = BUDDY_NUDGE;
//             if ((BuddySwitch == 4) && (Switch[2] == SwitchReversed[3]))
//                 BuddyState = BUDDY_NUDGE;
//         }

//         if (Buddy_Switch_Mode == M_N_B)
//         {
//             if ((BuddySwitch == 1) && !(Switch[6] || Switch[7])) // Switch is at middle
//                 BuddyState = BUDDY_NUDGE;
//             if ((BuddySwitch == 2) && !(Switch[4] || Switch[5]))
//                 BuddyState = BUDDY_NUDGE;
//             if ((BuddySwitch == 3) && !(Switch[0] || Switch[1]))
//                 BuddyState = BUDDY_NUDGE;
//             if ((BuddySwitch == 4) && !(Switch[2] || Switch[3]))
//                 BuddyState = BUDDY_NUDGE;
//         }

//         if (BuddyState != LastBuddyState)
//         {
//             LastBuddyState = BuddyState;
//             LogBuddyChange();
//         }
//     }
// }
/************************************************************************************************************/

void ReadBankSwitch1(bool sw1, bool sw2, bool rev) // Bank Switch
{
    if ((sw1 == false) && (sw2 == false))
    {
        Bank = 2;
    }
    else
    {
        if (rev)
        {
            if (sw1)
                Bank = 1;
            if (sw2)
                Bank = 3;
        }
        else
        {
            if (sw1)
                Bank = 3;
            if (sw2)
                Bank = 1;
        }
    }
}

/************************************************************************************************************/
void ReadBankSwitch()
{
    if (BankSwitch == 4)
        ReadBankSwitch1(Switch[2], Switch[3], SwitchReversed[3]);
    if (BankSwitch == 3)
        ReadBankSwitch1(Switch[0], Switch[1], SwitchReversed[2]);
    if (BankSwitch == 2)
        ReadBankSwitch1(Switch[4], Switch[5], SwitchReversed[1]);
    if (BankSwitch == 1)
        ReadBankSwitch1(Switch[6], Switch[7], SwitchReversed[0]);
}
/************************************************************************************************************/
void ReadAutoAndMotorSwitch()
{

    MotorEnabled = !UseMotorKill; //  If not using motor switch then motor is always enabled.
    if (SwitchReversed[0])
    {
        if ((Autoswitch == 1) && (!Switch[6]))
            MotorEnabled = true;
    }
    else
    {
        if ((Autoswitch == 1) && (!Switch[7]))
            MotorEnabled = true;
    }
    if (SwitchReversed[1])
    {
        if ((Autoswitch == 2) && (!Switch[4]))
            MotorEnabled = true;
    }
    else
    {
        if ((Autoswitch == 2) && (!Switch[5]))
            MotorEnabled = true;
    }
    if (SwitchReversed[2])
    {
        if ((Autoswitch == 3) && (!Switch[0]))
            MotorEnabled = true;
    }
    else
    {
        if ((Autoswitch == 3) && (!Switch[1]))
            MotorEnabled = true;
    }
    if (SwitchReversed[3])
    {
        if ((Autoswitch == 4) && (!Switch[2]))
            MotorEnabled = true;
    }
    else
    {
        if ((Autoswitch == 4) && (!Switch[3]))
            MotorEnabled = true;
    }
    if ((Autoswitch == 1) && (!Switch[6] && !Switch[7]))
        Bank = 4; // Flight mode 4 (Auto) overrides modes 1,2,3.
    if ((Autoswitch == 2) && (!Switch[4] && !Switch[5]))
        Bank = 4; // Flight mode 4 (Auto) overrides modes 1,2,3.
    if ((Autoswitch == 3) && (!Switch[1] && !Switch[0]))
        Bank = 4; // Flight mode 4 (Auto) overrides modes 1,2,3.
    if ((Autoswitch == 4) && (!Switch[3] && !Switch[2]))
        Bank = 4; // Flight mode 4 (Auto) overrides modes 1,2,3.
    if (!MotorEnabled)
        Bank = 4; // Motor off uses bank 4
}

/************************************************************************************************************/
void ReadDualRateSwitch()
{
    if ((!BuddyPupilOnWireless) || (BuddyHasAllSwitches)) // only read hardware switches if not using wireless buddy box or if buddy has all switches
    {
        DualRateInUse = 4; // default to 100%
        if (DualRatesSwitch == 4)
            ReadDRSwitch(Switch[2], Switch[3], SwitchReversed[3]);
        if (DualRatesSwitch == 3)
            ReadDRSwitch(Switch[0], Switch[1], SwitchReversed[2]);
        if (DualRatesSwitch == 2)
            ReadDRSwitch(Switch[4], Switch[5], SwitchReversed[1]);
        if (DualRatesSwitch == 1)
            ReadDRSwitch(Switch[6], Switch[7], SwitchReversed[0]);
        if (DualRateRate[Bank - 1] > 0)
            DualRateInUse = DualRateRate[Bank - 1]; // if using a forced rate !
    }
    if (DualRateInUse == 1)
        DualRateValue = Drate1;
    if (DualRateInUse == 2)
        DualRateValue = Drate2;
    if (DualRateInUse == 3)
        DualRateValue = Drate3;
    if (DualRateInUse == 4)
        DualRateValue = 100; // Switch not in use, so use 100%

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

/************************************************************************************************************/

void ReadDRSwitch(bool sw1, bool sw2, bool rev) // Dual Rate Switch
{
    if ((sw1 == false) && (sw2 == false))
    {
        DualRateInUse = 2;
    }
    else
    {
        if (rev)
        {
            if (sw1)
                DualRateInUse = 1;
            if (sw2)
                DualRateInUse = 3;
        }
        else
        {
            if (sw1)
                DualRateInUse = 3;
            if (sw2)
                DualRateInUse = 1;
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