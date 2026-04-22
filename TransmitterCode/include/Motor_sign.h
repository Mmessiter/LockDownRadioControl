#include <Arduino.h>
#include "1Definitions.h"
#ifndef MOTOR_SIGN_H
#define MOTOR_SIGN_H

/*********************************************************************************************************************************/
//                        MOTOR_SIGN and announcements functions
/*********************************************************************************************************************************/

/*********************************************************************************************************************************/
void SendColour(char *but, int Colour) // Set the colour of a button on the Nextion screen
{
    char lbut[60];
    char nb[10] = " ";
    strcpy(lbut, but);
    strcat(lbut, Str(nb, Colour, 0));
    SendCommand(lbut);
}
/*********************************************************************************************************************************/
void ShowSafety() // Show the state of the safety switch on the front view by changing the colour of the motor button and by playing a sound if it has just been turned on or off
{
    uint16_t Fg = GREEN;
    uint16_t Bg = BLACK;
    uint16_t AnnounceMent = SAFEOFF;

    char bco[] = "bt0.bco=";
    char bco2[] = "bt0.bco2=";
    char pco[] = "bt0.pco=";
    char pco2[] = "bt0.pco2=";

    if (SafetyON)
    {
        Fg = RED;
        Bg = WHITE;
        AnnounceMent = SAFEON;
    }
    if (AnnounceBanks && UseMotorKill && millis()>4000)// Don't want to hear this at power on because of the startup sounds etc.
    {
        PlaySound(AnnounceMent);
        if (UseLog)
            LogSafety();
    }
    if ((CurrentView == FRONTVIEW) && UseMotorKill)
    {
        SendColour(bco, Fg);
        SendColour(bco2, Fg);
        SendColour(pco, Bg);
        SendColour(pco2, Bg);
    }
}
// *********************************************************************************************************************************/
FASTRUN void ShowMotorTimer() // Show timer for the motor on the front view and play announcements if timer is running down
{
    char FrontView_Hours[] = "Hours";
    char FrontView_Mins[] = "Mins";
    char FrontView_Secs[] = "Secs";

    if (TimesUp)
        return;

    uint8_t Recording[10] = {ONEMINUTE, TWOMINUTES, THREEMINUTES, FOURMINUTES, FIVEMINUTES, SIXMINUTES, SEVENMINUTES, EIGHTMINUTES, NINEMINUTES, TENMINUTES};

    uint8_t Cdown[10] = {TEN, NINE, EIGHT, SEVEN, SIX, FIVE, FOUR, THREE, TWO, ONE};

    if ((MotorEnabled && (!LostContactFlag)))
    {
        MotorOnSeconds = ((millis() - MotorStartTime) / 1000) + PausedSecs;
        Secs = MotorOnSeconds;
        if (TimerDownwards)
            Secs = TimerStartTime - MotorOnSeconds;
        Hours = Secs / 3600;
        Secs %= 3600;
        Mins = Secs / 60;
        Secs %= 60;
    }
    if (LastSeconds != Secs)
    {
        ClockSpoken = false;
        ClockSpoken1 = false;
        if (CurrentView == FRONTVIEW)
        {
            SendValue(FrontView_Secs, Secs);
            SendValue(FrontView_Mins, Mins);
            SendValue(FrontView_Hours, Hours);
        }
        LastSeconds = Secs;
    }

    if (TimerDownwards)
    {
        if ((Secs < 11) && !Mins && !ClockSpoken1 && (MotorOnSeconds > 2))
        {
            PlaySound(Cdown[CountDownIndex]);
            ++CountDownIndex;
            ClockSpoken1 = true;
        }
    }

    if (!Secs && SpeakingClock && !ClockSpoken)
    {
        ClockSpoken = true;
        if ((Mins <= 10) && (Mins > 0))
        {
            PlaySound(Recording[Mins - 1]);
        }
        if (Mins)
        {
            if (UseLog)
            {
                LogTimer(Mins);
                LogRPM(RotorRPM);
            }
        }
        if (TimerDownwards)
        {
            if ((!Mins) && (!Secs) && (MotorOnSeconds > 2))
            {
                PlaySound(STORAGECHARGE); // = Stop Flying!
                TimesUp = true;
                CountDownIndex = 0;
            }
        }
    }
}
// ************************************************************************************************************/
void ShowMotor() // Show the state of the motor on the front view by changing the colour of the motor button and by playing a sound if it has just been turned on or off
{
    char bt0[] = "bt0";
    char OnMsg[] = "Motor is ON";
    char OffMsg[] = "Motor is OFF";
    if (MotorEnabled)
        SendText(bt0, OnMsg);
    else
        SendText(bt0, OffMsg);
}
// ************************************************************************************************************/
void MotorEnabledHasChanged() //
{
    if (MotorEnabled)
    {
        if (LedWasRed && !BuddyPupilOnWireless)
        {
            MotorEnabled = false;  // user has not turned on motor, so turn it back off
            PlaySound(PLSTURNOFF); // Tell the pilot to turn off motor before it will work
            SendNoData = true;     // send no data until motor is turned on again (so that pilot has to turn it on again before it will work)
            DelayWithDog(4000);    // allow time for sound to play and for pilot to react
            return;
        }
        ShowMotor();
        if (AnnounceBanks)
        {
            PlaySound(MOTORON); // Tell the pilot motor is on!
            DelayWithDog(750);  // allow time for sound to play as Bank change sound would be played at the same time and we don't want them to overlap
        } 
        if (UseLog)
            LogMotor(1);
        MotorStartTime = millis(); // Motor ON timerpause off
    }
    else
    {
        if (AnnounceBanks)
            PlaySound(MOTOROFF);
        SendNoData = false; // can send data!
        if (UseLog)
            LogMotor(0);
        SendCommand(WarnOff);
        ShowMotor(); // Tell the pilot motor is off
        if (SendNoData)
        {
            SendCommand(WarnOff);
            SendNoData = false; // user turned off motor
        }
        PausedSecs = MotorOnSeconds; //    Motor OFF timerpause started
    }
    LastSeconds = 0;
    ShowMotorTimer();
}
//************************************************************************************************************/
void SafetySwitchChanged()
{
    ShowSafety();
    if (SafetyON)
    {
        SendNoData = false;
        Armed = false;
    }
    else
    {
        Armed = true;
    }
    SafetyWasOn = SafetyON;
}

#endif
