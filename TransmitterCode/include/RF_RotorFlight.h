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
    snprintf(Vbuf, sizeof(Vbuf), "%1.2f", GearRatio);       // 10.3 usually (ClaudeFix-2-7-2026 size 5 truncated 10.35 to "10.3" -- which RotorFlightEnd then read back and SAVED, silently degrading the ratio)
    SendText((char *)"Ratio", Vbuf);
    snprintf(Vbuf, sizeof(Vbuf), "%d", ArmingChannel);
    SendText((char *)"Arming", Vbuf);
    SendValue((char *)"sw0", LinkRatesToBanks);
    RotorFlight_Version = RFVersions[RotorFlight_V];
    snprintf(Vbuf, sizeof(Vbuf), "%1.1f", RotorFlight_Version);
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
    GetText((char *)"Ratio", temp, sizeof(temp));  // ClaudeFix-2-7-2026
    float newRatio = atof(temp);
    if (newRatio > 0.0f)
        GearRatio = newRatio;

    GetText((char *)"Arming", temp, sizeof(temp));  // ClaudeFix-2-7-2026
    // ClaudeFix-2-7-2026 Digits ONLY: a desynced read-back once returned the Ratio box's "10.30"
    // here, and atoi turned it into a perfectly plausible channel 10. A real
    // channel number can never contain a dot (GetText is fixed too, but this
    // field guards a safety-critical value -- belt AND braces).
    bool armDigitsOnly = (strlen(temp) > 0);
    for (uint8_t adc = 0; adc < strlen(temp); ++adc)
        if (temp[adc] < '0' || temp[adc] > '9')
            armDigitsOnly = false;
    int newArming = atoi(temp);
    if (armDigitsOnly && newArming > 0 && newArming <= CHANNELSUSED)
        ArmingChannel = (uint8_t)newArming;

    {
        uint32_t sw = GetValue((char *)"sw0");
        if (sw <= 1)
            LinkRatesToBanks = (bool)sw; // ClaudeFix-2-7-2026 ignore a comms-error 65535 (would have saved as 'true')
    }
    SaveOneModel(ModelNumber); // save the model including gear ratio and arming channel
    ZeroDataScreen();        // clear the screen data because editing Rotorflight parameters may have created misleading comms gaps
    GotoFrontView();
}
// **********************************************************************************************************/
void LinkRatesToBanksChanged()
{
    {
        uint32_t sw = GetValue((char *)"sw0");
        if (sw <= 1)
            LinkRatesToBanks = (bool)sw; // ClaudeFix-2-7-2026 ignore a comms-error 65535
    }
    SaveOneModel(ModelNumber); // save the model including LinkRatesToBanks
}

#endif // RotorFlight_H