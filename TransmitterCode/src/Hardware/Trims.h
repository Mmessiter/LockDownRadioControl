// *********************************************** Trims.h *******************************************

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef TRIMS_H
#define TRIMS_H

// **********************************************************************************************************************************
void StartTrimView()
{
    char n0[] = "n0";
    char c0[] = "c0";
    SendCommand(pTrimView);
    CurrentView = TRIM_VIEW;
    SendValue(n0, TrimMultiplier);
    SendValue(c0, CopyTrimsToAll);
    ForceDataRedisplay();
    UpdateModelsNameEveryWhere(); // also updates trimview (If CurrentView == TRIM_VIEW!! :-)
    ClearText();                  //
}

/******************************************************************************************************************************/
void StartTrimDefView()
{

    CurrentView = TRIMDEFVIEW;
    SendCommand(pTrimDefView);
    ResetAllTrims();
    BlueLedOn();
    CurrentMode = SENDNOTHING;
    for (int i = 0; i < 4; ++i)
        TrimDefined[i] = false;
    DefiningTrims = true;
}
/******************************************************************************************************************************/
void DefineTrimsEnd()
{ // exit from trim defining screen
    char pCalibrateView[] = "page CalibrateView";
    CurrentView = TXSETUPVIEW;
    SendCommand(pCalibrateView);
    Force_ReDisplay();
    CurrentView = CALIBRATEVIEW;
    DefiningTrims = false;
    CurrentMode = NORMAL;
    SaveTransmitterParameters();
    UpdateModelsNameEveryWhere();
}
/******************************************************************************************************************************/
void ResetAllTrims()
{

    if (SticksMode == 1)
    {
        TrimNumber[0] = TRIM1A; // these will change when redefined
        TrimNumber[1] = TRIM1B;
        TrimNumber[2] = TRIM2A;
        TrimNumber[3] = TRIM2B;
        TrimNumber[4] = TRIM3A;
        TrimNumber[5] = TRIM3B;
        TrimNumber[6] = TRIM4A;
        TrimNumber[7] = TRIM4B;
    }

    if (SticksMode == 2)
    {
        TrimNumber[0] = TRIM1A;
        TrimNumber[1] = TRIM1B;
        TrimNumber[4] = TRIM2A;
        TrimNumber[5] = TRIM2B;
        TrimNumber[2] = TRIM3A;
        TrimNumber[3] = TRIM3B;
        TrimNumber[6] = TRIM4A;
        TrimNumber[7] = TRIM4B;
    }
}

/************************************************************************************************************/

void SetATrimDefinition(int i)
{
    char AilDone[] = "Aileron trim is defined!";
    char EleDone[] = "Elevator trim is defined!";
    char ThrDone[] = "Throttle trim is defined!";
    char RudDone[] = "Rudder trim is defined!";

    char ail[] = "ail";
    char ele[] = "ele";
    char thr[] = "thr";
    char rud[] = "rud";

    // Aileron
    if (!TrimDefined[0])
    {
        if ((i == 0) || (i == 1))
        {
            PlaySound(BEEPCOMPLETE);
            SendText(ail, AilDone);
            TrimDefined[0] = true;
        }
        if (i == 0)
        {
            TrimNumber[0] = TRIM1A;
            TrimNumber[1] = TRIM1B;
        }
        if (i == 1)
        {
            TrimNumber[1] = TRIM1A;
            TrimNumber[0] = TRIM1B;
        }
    }

    if (SticksMode == 1)
    {
        // Elevator
        if (!TrimDefined[1])
        {
            if ((i == 2) || (i == 3))
            {
                PlaySound(BEEPCOMPLETE);
                SendText(ele, EleDone);
                TrimDefined[1] = true;
            }
            if (i == 3)
            {
                TrimNumber[2] = TRIM2A;
                TrimNumber[3] = TRIM2B;
            }
            if (i == 2)
            {
                TrimNumber[3] = TRIM2A;
                TrimNumber[2] = TRIM2B;
            }
        }

        // Throttle
        if (!TrimDefined[2])
        {
            if ((i == 4) || (i == 5))
            {
                PlaySound(BEEPCOMPLETE);
                SendText(thr, ThrDone);
                TrimDefined[2] = true;
            }
            if (i == 4)
            {
                TrimNumber[4] = TRIM3A; // heer
                TrimNumber[5] = TRIM3B;
            }
            if (i == 5)
            {
                TrimNumber[5] = TRIM3A;
                TrimNumber[4] = TRIM3B;
            }
        }
    }

    if (SticksMode == 2)
    {
        // Throttle
        if (!TrimDefined[1])
        {
            if ((i == 4) || (i == 5))
            {
                PlaySound(BEEPCOMPLETE);
                SendText(thr, ThrDone);
                TrimDefined[1] = true;
            }
            if (i == 5)
            {
                TrimNumber[5] = TRIM2A;
                TrimNumber[4] = TRIM2B;
            }
            if (i == 4)
            {
                TrimNumber[4] = TRIM2A;
                TrimNumber[5] = TRIM2B;
            }
        }

        // Elevator
        if (!TrimDefined[2])
        {
            if ((i == 2) || (i == 3))
            {
                PlaySound(BEEPCOMPLETE);
                SendText(ele, EleDone);
                TrimDefined[2] = true;
            }
            if (i == 3)
            {
                TrimNumber[2] = TRIM3A;
                TrimNumber[3] = TRIM3B;
            }
            if (i == 2)
            {
                TrimNumber[3] = TRIM3A;
                TrimNumber[2] = TRIM3B;
            }
        }
    }

    // Rudder
    if (!TrimDefined[3])
    {
        if ((i == 6) || (i == 7))
        {
            PlaySound(BEEPCOMPLETE);
            SendText(rud, RudDone);
            TrimDefined[3] = true;
        }
        if (i == 6)
        {
            TrimNumber[6] = TRIM4A;
            TrimNumber[7] = TRIM4B;
        }
        if (i == 7)
        {
            TrimNumber[7] = TRIM4A;
            TrimNumber[6] = TRIM4B;
        }
    }
}

/************************************************************************************************************/

void CheckHardwareTrims()
{
    if ((millis() - TrimTimer) < TrimRepeatSpeed)
        return; // check occasionally for trim press
    TrimTimer = millis();
    for (int i = 0; i < 8; ++i)
    {
        if (TrimSwitch[i])
        {
            if (DefiningTrims)
            {
                SetATrimDefinition(i);
                return;
            }
            MoveaTrim(i);
            // TransmitterLastManaged = 0;                     //  to speed up repeat
            TrimRepeatSpeed -= (TrimRepeatSpeed / 4); //  accelerate repeat...
            if (TrimRepeatSpeed < 10)
                TrimRepeatSpeed = 30; //  ... up to a point...
        }
    }
}

/*********************************************************************************************************************************/

/**
 * Retrieves the trim amount for the specified input channel.
 *
 * @param InputChannel The input channel for which to retrieve the trim amount.
 * @return The trim amount for the specified input channel.
 */
int GetTrimAmount(uint8_t InputChannel)
{
    int tt = InputChannel;
    if (SticksMode == 2)
    {
        if (InputChannel == 1)
            tt = 2;
        if (InputChannel == 2)
            tt = 1;
    }
    return (Trims[Bank][tt] - 80) * TrimMultiplier;
}

/*********************************************************************************************************************************/
/**
 * Updates the trim view based on the current trim values.
 */
void UpdateTrimView()
{
    uint8_t p;
    char TrimViewChannels[4][4] = {"ch1", "ch4", "ch2", "ch3"};
    char TrimViewNumbers[4][3] = {"n1", "n4", "n2", "n3"};
    char TrimChannelNames[4][3] = {"c1", "c2", "c3", "c4"};

    if (CurrentView == FRONTVIEW || (CurrentView == TRIM_VIEW))
    {
        for (int i = 0; i < 4; ++i)
        {
            p = i;
            if (SticksMode == 2)
            {
                if (i == 1)
                    p = 2;
                if (i == 2)
                    p = 1;
            }
            uint8_t pp = InputTrim[p];
            if (LastTrim[Bank][pp] != Trims[Bank][pp])
            {
                LastTrim[Bank][pp] = Trims[Bank][pp];
                SendValue(TrimViewChannels[p], (Trims[Bank][pp]));
                SendValue(TrimViewNumbers[p], (Trims[Bank][pp] - 80));
                if (CurrentView == TRIM_VIEW)
                    SendText(TrimChannelNames[i], ChannelNames[pp]);
            }
        }
    }
}

/*********************************************************************************************************************************/

FLASHMEM void CentreTrims()
{
    for (int j = 0; j <= BANKS_USED; ++j)
    {
        for (int i = 0; i < CHANNELSUSED; ++i)
        {
            Trims[j][i] = 80;
        }
    }
}
/*********************************************************************************************************************************/
void CheckSavedTrimValues()
{
    bool OK = true;
    for (int i = 0; i < 4; ++i)
    {
        if ((InputTrim[i] > 15) || (InputTrim[i] < 0))
            OK = false;
    }
    if (!OK)
    {
        for (int i = 0; i < 4; ++i)
        {
            InputTrim[i] = i;
        }
    }
}
/*********************************************************************************************************************************/
void CheckTrimValues()
{
    bool KO = false;
    for (int j = 0; j < 8; ++j)
    {
        if ((TrimNumber[j] > TRIM4B) || (TrimNumber[j] < TRIM1A))
            KO = true;
    }
    if (KO)
        ResetAllTrims();
}

/******************************************************************************************************************************/

/// @brief
void StartSubTrimView()
{ // Subtrim view start

    char n0[] = "n0";
    char h0[] = "h0";
    SendCommand(pSubTrimView);
    SubTrimToEdit = 0;
    CurrentView = SUBTRIMVIEW;
    UpdateButtonLabels();
    SendValue(n0, SubTrims[SubTrimToEdit] - 127);
    SendValue(h0, SubTrims[SubTrimToEdit]);
    UpdateModelsNameEveryWhere();
}

/******************************************************************************************************************************/
void EndSubTrimView()
{ // Subtrim view exit
    char page_RXSetupView[] = "page RXSetupView";
    SaveOneModel(ModelNumber);
    CurrentView = RXSETUPVIEW;
    SendCommand(page_RXSetupView);
    LastTimeRead = 0;
    UpdateModelsNameEveryWhere();
}

// *************************************************************************************************************

void IncTrim(uint8_t t)
{
    bool Sounded = false;
    Trims[Bank][t] += 1;
    if (Trims[Bank][t] >= 120)
    {
        Trims[Bank][t] = 120;
        if (TrimClicks)
        {

            PlaySound(BEEPCOMPLETE);
            Sounded = true;
            TrimRepeatSpeed = DEFAULTTRIMREPEATSPEED;
        }
    }
    if (Trims[Bank][t] == 80)
    {
        TrimRepeatSpeed = DEFAULTTRIMREPEATSPEED; // Restore default trim repeat speed at centre
        if (TrimClicks)
        {
            PlaySound(BEEPMIDDLE);
            Sounded = true;
        }
    }
    if ((CurrentView == TRIM_VIEW) || (CurrentView == FRONTVIEW))
        UpdateTrimView();
    if ((TrimClicks) && (!Sounded))
        PlaySound(CLICKZERO);
}
// *************************************************************************************************************

void DecTrim(uint8_t t)
{

    bool Sounded = false;
    Trims[Bank][t] -= 1;
    if (Trims[Bank][t] <= 40)
    {
        Trims[Bank][t] = 40;
        if (TrimClicks)
        {
            PlaySound(BEEPCOMPLETE);
            Sounded = true;
            TrimRepeatSpeed = DEFAULTTRIMREPEATSPEED;
        }
    }
    if (Trims[Bank][t] == 80)
    {
        TrimRepeatSpeed = DEFAULTTRIMREPEATSPEED; // Restore default trim repeat speed at centre
        if (TrimClicks)
        {
            PlaySound(BEEPMIDDLE);
            Sounded = true;
        }
    }
    if ((CurrentView == TRIM_VIEW) || (CurrentView == FRONTVIEW))
        UpdateTrimView();
    if ((TrimClicks) && (!Sounded))
        PlaySound(CLICKZERO);
}

// *************************************************************************************************************

void MoveaTrim(uint8_t i)
{
    uint8_t Elevator = 1;
    uint8_t Throttle = 2;

    if (SticksMode == 2)
    {
        Elevator = 2;
        Throttle = 1;
    }

    switch (i)
    {
    case 0:
        IncTrim(InputTrim[0]); // Aileron
        break;
    case 1:
        DecTrim(InputTrim[0]); // Aileron
        break;
    case 2:
        IncTrim(InputTrim[Elevator]);
        break;
    case 3:
        DecTrim(InputTrim[Elevator]);
        break;
    case 4:
        DecTrim(InputTrim[Throttle]);
        break;
    case 5:
        IncTrim(InputTrim[Throttle]);
        break;
    case 6:
        IncTrim(InputTrim[3]); // Rudder
        break;
    case 7:
        DecTrim(InputTrim[3]); // Rudder
        break;
    default:
        break;
    }
    if (CopyTrimsToAll)
    {
        for (i = 0; i < 4; ++i)
        {
            for (int fm = 1; fm < 5; ++fm)
            {
                Trims[fm][i] = Trims[Bank][i];
            }
        }
    }
}

/******************************************************************************************************************************/
void TrimsToSubtrim()
{ //   Store trims to subtrims and reset trims to centre

    // Trims range from 40 to 120. 80 is centre.

    char Prompt[] = "Move these trims to Subtrims?\r\n\r\nThis would re-centre these trims.";
    char ItsCancelled[] = "Cancelled.\r\nSubtrims were not changed.\r\nLook at the Subtrim view \r\nif you'd like to check.";
    char ItsAllDone[] = "Done!\r\nSubtrims have been amended.\r\n\r\nLook at the Subtrim view \r\nif you'd like to check.";

    if (GetConfirmation(pTrimView, Prompt))
    {
        for (int i = 0; i < 4; ++i)
        {
            int temp = (int)(Trims[Bank][InputTrim[i]] - 80) * (int)TrimMultiplier; // 'int temp' is needed to avoid overflow
            temp /= 5;                                                              // Subtrims' multiplier is 5 always
            if (temp + SubTrims[InputTrim[i]] > 255)
                temp = 255 - SubTrims[InputTrim[i]]; // Avoid overflow
            SubTrims[InputTrim[i]] += temp;          // Trims *affected* channels are in the InputTrim[] array
            if (SubTrims[i] > 255)
                SubTrims[i] = 255; // Avoid overflow
            for (int j = 1; j < 5; ++j)
                Trims[j][InputTrim[i]] = 80; // Reset all four normal trims to centre
        }
        PlaySound(BEEPCOMPLETE);
        MsgBox(pTrimView, ItsAllDone); // Done!
        ForceDataRedisplay();          // force Update trim view
        UpdateTrimView();              // Update trim view
        SaveOneModel(ModelNumber);     // Save model memory
    }
    else
    {
        MsgBox(pTrimView, ItsCancelled); // Cancelled
        ForceDataRedisplay();            // force Update trim view
        UpdateTrimView();                // Update trim view
    }
}

/******************************************************************************************************************************/
void RudderLeftTrim() { MoveaTrim(7); }
/******************************************************************************************************************************/
void RudderRightTrim() { MoveaTrim(6); }
/******************************************************************************************************************************/
void AileronRightTrim() { MoveaTrim(0); }
/******************************************************************************************************************************/
void AileronLeftTrim() { MoveaTrim(1); }
/******************************************************************************************************************************/
void ElevatorUpTrim()
{
    uint8_t tt = 2;
    if (SticksMode == 2)
        tt = 5;
    MoveaTrim(tt);
}
/******************************************************************************************************************************/
void ElevatorDownTrim()
{
    uint8_t tt = 3;
    if (SticksMode == 2)
        tt = 4;
    MoveaTrim(tt);
}
/******************************************************************************************************************************/
void ThrottleUpTrim()
{
    uint8_t tt = 4;
    if (SticksMode == 2)
        tt = 3;
    MoveaTrim(tt);
}
/******************************************************************************************************************************/
void ThrottleDownTrim()
{
    uint8_t tt = 5;
    if (SticksMode == 2)
        tt = 2;
    MoveaTrim(tt);
}

#endif