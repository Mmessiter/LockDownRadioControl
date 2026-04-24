// **********************************************************************************************************
// This file handles Rotorflight central functions
// **********************************************************************************************************

#ifndef RotorFlight_H
#define RotorFlight_H
#include <Arduino.h>
#include "1Definitions.h"

void ShowRFRate()
{
    char NB[10];
    Str(NB, DualRateInUse, 0);
    char msg[70];
    strcpy(msg, "Rate ");
    strcat(msg, NB);
    SendText((char *)"t8", msg);
}

// **********************************************************************************************************/
void ShowRFBank()
{
    char NB[10];
    Str(NB, Bank, 0);
    char msg[70];
    strcpy(msg, "Bank ");
    strcat(msg, NB);
    SendText((char *)"t14", msg);
}

// **********************************************************************************************************/
void RotorFlightStart()
{
    char Vbuf[15];

    if (MotorEnabled || !SafetyON)
    {
        MsgBox(pRXSetupView, (char *)"Please disarm and turn on safety.");
        return;
    }

    SendCommand((char *)"page RFView");
    CurrentView = ROTORFLIGHTVIEW;
    AddParameterstoQueue(MSP_INHIBIT_TELEMETRY); // Inhibit telemetry for a short time to allow MSP data to be sent without interference from telemetry data (for MSP data transmission)
    SendText((char *)"t11", ModelName);          // Show model name
    snprintf(Vbuf, 5, "%1.2f", GearRatio);       // 10.3 usually
    SendText((char *)"Ratio", Vbuf);
    snprintf(Vbuf, 5, "%d", ArmingChannel);
    SendText((char *)"Arming", Vbuf);
    SendValue((char *)"sw0", LinkRatesToBanks);
    RotorFlight_Version = RFVersions[RotorFlight_V];
    snprintf(Vbuf, 5, "%1.1f", RotorFlight_Version);
    SendText((char *)"t5", Vbuf);
    ShowRFBank();
    ShowRFRate();
}

// **********************************************************************************************************/
void RotorFlightEnd()
{
    char temp[15];
    // Nextion serial can carry stale bytes after heavy MSP traffic, so GetText may fail silently.
    // Only overwrite the globals if the read-back parses to a plausible value — otherwise keep the current one.
    GetText((char *)"Ratio", temp);
    float newRatio = atof(temp);
    if (newRatio > 0.0f)
        GearRatio = newRatio;

    GetText((char *)"Arming", temp);
    int newArming = atoi(temp);
    if (newArming > 0 && newArming <= CHANNELSUSED)
        ArmingChannel = (uint8_t)newArming;

    LinkRatesToBanks = GetValue((char *)"sw0");
    SaveOneModel(ModelNumber); // save the model including gear ratio and arming channel
    ZeroDataScreen();        // clear the screen data because editing Rotorflight parameters may have created misleading comms gaps
    GotoFrontView();
}
// **********************************************************************************************************/
void LinkRatesToBanksChanged()
{
    LinkRatesToBanks = GetValue((char *)"sw0");
    SaveOneModel(ModelNumber); // save the model including LinkRatesToBanks
}

#endif // RotorFlight_H