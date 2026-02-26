// *********************************************** Telemetry.h for Transmitter code *******************************************

#include <Arduino.h>
#include "1Definitions.h"
#ifndef TELEMETRY_H
#define TELEMETRY_H

/*********************************************************************************************************************************/
FASTRUN bool CheckTXVolts()
{
    char JTX[] = "JTX";             // Labels on Nextion
    char FrontView_TXBV[] = "TXBV"; // Labels on Nextion
    bool TXWarningFlag = false;
    float TransmitterBatteryPercentLeft, TXVoltsRaw;
    char TXBattInfo[80];
    char pc[] = "%";
    char nbuf[60]; //  buffer for numbers
    char NB[10];
    char PerCell[] = "V per cell)";
    char v[] = "V (";
    char t17[] = "t17";

    if (USE_INA219)
    {
        TXVoltsRaw = ((ina219.getBusVoltage_V()) * 100) + (TxVoltageCorrection * 2); // Correction for inaccurate ina219
        dtostrf(TXVoltsRaw / 100, 2, 2, nbuf);                                       // Volts per cell
        TXVoltsTotal = TXVoltsRaw / 100;                                             // total volts
        float TXVoltsTotalPerCell = TXVoltsTotal / 2;
        if (TXLiPo)
        {                                                                                                  // Does TX have a LiPo or a LiFePo4?
            TransmitterBatteryPercentLeft = map(TXVoltsTotalPerCell * 100, 3.5 * 100, 4.00 * 100, 0, 100); // LIPO Battery 3.50 -> c. 4.00  volts per cell
        }
        else
        {                                                                                                  // No, it's a LiFePo4
            TransmitterBatteryPercentLeft = map(TXVoltsTotalPerCell * 100, 3.2 * 100, 3.33 * 100, 0, 100); // LiFePo4 Battery 3.1 -> 3.35  volts per cell
        }
        if (TransmitterBatteryPercentLeft < LowBattery)
        {
            TXWarningFlag = true;
            WarningSound = BATTERYISLOW;
        }
        TransmitterBatteryPercentLeft = constrain(TransmitterBatteryPercentLeft, 0, 100);
        strcat(TXBattInfo, pc);
        if (CurrentView == FRONTVIEW)
        {
            SendValue(JTX, TransmitterBatteryPercentLeft);
            strcat(nbuf, v);
            dtostrf(TXVoltsTotalPerCell, 2, 2, NB);
            strcat(nbuf, NB);
            strcat(nbuf, PerCell);

            SendText(FrontView_TXBV, nbuf);
        }
        if (CurrentView == DATAVIEW)
        {
            dtostrf(TXVoltsTotalPerCell, 2, 2, NB);
            SendText(t17, NB);
        }
    }
    SimplePing();
    return TXWarningFlag;
}

/*********************************************************************************************************************************/

FASTRUN bool CheckRXVolts()
{
    static bool RXWarningFlag = false;
    float ReadVolts = 0;
    uint8_t GreenPercentBar = 0;
    char JRX[] = "JRX";
    char Vbuf[10];
    char RXBattInfo[80];
    char FrontView_RXBV[] = "RXBV";
    char PerCell[] = "V per cell)";
    char v[] = "V (";
    char t6[] = "t6";

    ReadVolts = (RXModelVolts * 100) + (RxVoltageCorrection * RXCellCount);
    GreenPercentBar = map(ReadVolts, 3.4f * RXCellCount * 100, 4.2f * RXCellCount * 100, 0, 100);
    if (RXVoltsDetected)
    {
        GreenPercentBar = constrain(GreenPercentBar, 0, 100);
        WarningSound = BATTERYISLOW;
        if (BoundFlag)
        {
            RXVoltsPerCell = (ReadVolts / RXCellCount) / 100;
            if (CurrentView == FRONTVIEW)
            {
                SendValue(JRX, GreenPercentBar);
                strcpy(RXBattInfo, ModelVolts);
                strcat(RXBattInfo, v);
                dtostrf(RXVoltsPerCell, 2, 2, Vbuf);
                strcat(RXBattInfo, Vbuf);
                strcat(RXBattInfo, PerCell);
                SendText(FrontView_RXBV, RXBattInfo);
            }
            if (CurrentView == DATAVIEW)
            {
                dtostrf(RXVoltsPerCell, 2, 2, Vbuf);
                SendText(t6, Vbuf);
            }
            RXWarningFlag = false; // new as bit below is removed now.
            if ((RXVoltsPerCell < StopFlyingVoltsPerCell) && (RXVoltsPerCell > 2))
            {
                RXWarningFlag = true;
                WarningSound = STORAGECHARGE;
            }
        }
    }
    else
    {
        if (BoundFlag && CurrentView == FRONTVIEW)
        {
            SendText(FrontView_RXBV, (char *)" ");
            SendValue(JRX, 0);
        }
    }
    SimplePing();
    return RXWarningFlag;
}
//*********************************************************************************************************************************/
char *Hours_Mins_Secs(uint32_t total_seconds, char *buffer, size_t buflen)
{
    uint32_t hours = total_seconds / 3600;
    uint32_t minutes = (total_seconds % 3600) / 60;
    uint32_t seconds = total_seconds % 60;
    snprintf(buffer, buflen, "%" PRIu32 ":%02" PRIu32 ":%02" PRIu32, hours, minutes, seconds);
    return buffer;
}
/*********************************************************************************************************************************/

void CheckBatteryStates()
{
    static uint32_t WarnTimer = 0;
    char WarnNow[] = "vis Warning,1";
    char WarnOff[] = "vis Warning,0";
    uint32_t Now = millis();

    if ((CheckTXVolts() || CheckRXVolts()))
    { // Note: If TX Battery is low, then CheckRXVolts() is not even called
        if (Now - WarnTimer > 10000)
        { // issue warning every 10 seconds
            WarnTimer = Now;
            if (ModelMatched && Connected)
            {
                PlaySound(WarningSound); // Issue audible warning
                LogStopFlyingMsg();      // Log the stop flying message
                LogRXVoltsPerCell();     // Log the RX volts per cell
                                         // LedIsBlinking = true; // too annoying
                if (CurrentView == FRONTVIEW)
                    SendCommand(WarnNow);
            }
        }
    }
    else
    {
        if (CurrentView == FRONTVIEW)
            SendCommand(WarnOff);
        LedIsBlinking = false;
        WarnTimer = Now;
    }
    SimplePing();
}

/*********************************************************************************************************************************/
void ShowCurrentRate()
{ // if it has changed
    char rate[] = "rate";
    char rate1[] = "Rate 1";
    char rate2[] = "Rate 2";
    char rate3[] = "Rate 3";
    char rate4[] = "Rate 4";

    switch (DualRateInUse)
    {
    case 1:
        SendText(rate, rate1); //
        SendCommand((char *)"vis rate,1");
        break;
    case 2:
        SendText(rate, rate2);
        SendCommand((char *)"vis rate,1");
        break;
    case 3:
        SendText(rate, rate3);
        SendCommand((char *)"vis rate,1");
        break;
    case 4:
        if (LinkRatesToBanks)
            SendText(rate, rate4);
        else
            SendCommand((char *)"vis rate,0");
        break;
    default:
        break;
    }
    SimplePing();
}

/*********************************************************************************************************************************/

void ShowAMS()
{
    char ams[] = "ams";
    char AmsOnMsg[14] = "AMS is on";
    char AmsOffMsg[14] = "AMS is off";

    if (LedWasGreen)
    {
        SendCommand((char *)"vis ams,0");
        return;
    }
    else
    {
        SendCommand((char *)"vis ams,1");
    }

    if (!ModelsMacUnionSaved.Val64)
    {
        strcpy(AmsOnMsg, "[AMS is on]");
    }

    if (AutoModelSelect)
    {
        SendText(ams, AmsOnMsg);
    }
    else
    {
        SendText(ams, AmsOffMsg);
    }
    SimplePing();
}

/*********************************************************************************************************************************/
void ShowTrimToAll()
{
    char trall[] = "trall";
    char CopyTrimsToAllMSG[] = "TrimAll is on";
    char CopyTrimsToNoneMSG[] = "TrimAll is off";

    if (CopyTrimsToAll)
    {
        SendText(trall, CopyTrimsToAllMSG);
    }
    else
    {
        SendText(trall, CopyTrimsToNoneMSG);
    }
    SimplePing();
}

/*********************************************************************************************************************************/

/**
 * @brief Populates the GPS view on the NEXTION data screen with the current GPS data.
 *
 * This function updates the labels on the GPS view screen with the current GPS data. It displays information such as GPS fix status, longitude, latitude, bearing, distance, speed, altitude, and satellite count.
 */
void PopulateGPSView()
{
    char Vbuf[50];
    char Fix[] = "Fix"; // These are label names in the NEXTION data screen. They are best kept short.
    char Lon[] = "Lon";
    char Lat[] = "Lat";
    char Bear[] = "Bear";
    char Dist[] = "Dist";
    char Sped[] = "Sped";
    char yes[] = "Yes";
    char no[] = "No";
    char ALT[] = "ALT";
    char MALT[] = "MALT";
    char MxS[] = "MxS";
    char Mxd[] = "Mxd";
    char BTo[] = "BTo";
    char Sat[] = "Sat";

    snprintf(Vbuf, 7, "%d", GPS_RX_Satellites);
    SendText(Sat, Vbuf);

    if (GPS_RX_FIX)
    { // if no fix, then leave display as before
        SendText(Fix, yes);
        snprintf(Vbuf, 7, "%.3f", GPS_RX_ANGLE);
        SendText(Bear, Vbuf);
        snprintf(Vbuf, 6, "%.3f", GPS_RX_Altitude);
        SendText(ALT, Vbuf);
        snprintf(Vbuf, 7, "%.3f", GPS_RX_DistanceTo);
        SendText(Dist, Vbuf);
        snprintf(Vbuf, 6, "%.3f", GPS_RX_CourseTo);
        SendText(BTo, Vbuf);
        SimplePing();
        snprintf(Vbuf, 15, "%.12f", GPS_RX_Longitude);
        SendText(Lon, Vbuf);
        snprintf(Vbuf, 15, "%.12f", GPS_RX_Latitude);
        SendText(Lat, Vbuf);
        snprintf(Vbuf, 6, "%.3f", GPS_RX_Speed);
        SendText(Sped, Vbuf);
        snprintf(Vbuf, 6, "%.3f", GPS_RX_MaxSpeed);
        SendText(MxS, Vbuf);
        snprintf(Vbuf, 6, "%.3f", GPS_RX_Maxaltitude); // ?? todo
        SendText(MALT, Vbuf);
        snprintf(Vbuf, 6, "%.3f", GPS_RX_MaxDistance); //  ??
        SendText(Mxd, Vbuf);
    }
    else
    {
        SendText(Fix, no);
    }
}

/*********************************************************************************************************************************/
void PopulateDataView()
{
    // This function now concatenates all the required Nextion commands into a single string and sends it once only to save time
    // Sending each separately consumed too much time!
    // The maximum string length for the next year is 255. If this would be exceeded, it sends what it has and starts building again.
    // The string is built in the NextionCommand array, which is 255 bytes long.
    // The old commands are shown in comments to the right of the new replacements.

    char DataView_pps[] = "pps"; // These are label names in the NEXTION data screen. They are best kept short.
    char DataView_lps[] = "lps";
    char DataView_Alt[] = "alt";
    char DataView_Temp[] = "Temp";
    char DataView_MaxAlt[] = "MaxAlt";
    char DataView_rxv[] = "rxv";
    char DataView_Ls[] = "Ls";
    char DataView_Ts[] = "Ts";
    char DataView_Rx[] = "rx";
    char DataView_Sg[] = "Sg";
    char DataView_Ag[] = "Ag";
    char DataView_Gc[] = "Gc";
    char Sbs[] = "Sbus";
    char Vbuf[50];
    char DataView_txv[] = "txv";
    char MeanFrameRate[] = "n0";
    char TimeSinceBoot[] = "n1";
    uint32_t BootedSeconds = millis() / 1000;
    char tempbuf[25];
    AddParameterstoQueue(MSP_ENABLE_TELEMETRY); // 26 = ENABLE telemetry after MSP data has been sent (for MSP data transmission)
    ClearNextionCommand();

    BuildText(DataView_txv, TransmitterVersionNumber);
    if (BoundFlag && ModelMatched)
        BuildValue(MeanFrameRate, AverageFrameRate);
    BuildText(DataView_rxv, ReceiverVersionNumber);
    BuildValue(DataView_pps, PacketsPerSecond);
    BuildValue(DataView_lps, TotalLostPackets);
    BuildValue(DataView_Ls, GapLongest);
    BuildValue(DataView_Ts, RadioSwaps);
    Hours_Mins_Secs(RX1TotalTime, tempbuf, sizeof(tempbuf));
    BuildText(DataView_Sg, tempbuf);
    BuildValue(DataView_Ag, GapAverage);
    Hours_Mins_Secs(RX2TotalTime, tempbuf, sizeof(tempbuf));
    BuildText(DataView_Gc, tempbuf);
    sprintf(Max_Rotor_RPM, "%" PRIu32 " RPM", Max_RotorRPM);
    BuildText(DataView_Alt, Max_Rotor_RPM);
    sprintf(ESC_Temperature, "%.1f C.", ESC_Temp);
    sprintf(MAX_ESC_Temperature, "%.1f C.", Max_ESC_Temp);
    BuildText(DataView_MaxAlt, MAX_ESC_Temperature);
    BuildText(DataView_Temp, ESC_Temperature);
    snprintf(Vbuf, 7, "%" PRIu32, RXSuccessfulPackets);
    BuildText(DataView_Rx, Vbuf);
    BuildText(Sbs, Rx_type[Receiver_type]);
    Hours_Mins_Secs(BootedSeconds, tempbuf, sizeof(tempbuf));
    BuildText(TimeSinceBoot, tempbuf);
    SimplePing();
    SendCommand(NextionCommand); // takes about 3 ms to send all at once
    SimplePing();
    ClearNextionCommand();
}

/*********************************************************************************************************************************/
// this function looks at the most recent ((uint16_t) ConnectionAssessSeconds) few seconds of packets which succeeded and expresses these
// as a percentage of total attempted packets using a progress bar on the screen.

void ShowConnectionQuality()
{
    static uint16_t lastparam = 0;
    uint16_t ThisParam = ParametersToBeSentPointer;

    char Quality[] = "Quality";
    char FrontView_Connected[] = "Connected"; // this is both the label name and the text to be displayed :=)
    char Visible[] = "vis Quality,1";
    uint16_t ConnectionQuality = GetSuccessRate();
    if (!LedWasGreen)
        return;
    if ((ConnectionQuality != LastConnectionQuality) || (lastparam != ThisParam && !ParametersToBeSentPointer))
    {
        SimplePing();
        SendText(FrontView_Connected, FrontView_Connected);
        SendCommand(Visible);
        LastConnectionQuality = ConnectionQuality;
        SendValue(Quality, ConnectionQuality);
    } // only if changed
    lastparam = ThisParam;
}

/*********************************************************************************************************************************/

void PopulateFrontView()
{

    char FrontView_AckPayload[] = "AckPayload";
    char FrontView_RXBV[] = "RXBV";
    char MsgBuddying[] = "* Buddy *";
    char FrontView_Connected[] = "Connected";
    char InVisible[] = "vis Quality,0";

    if ((ParametersToBeSentPointer == 0) || (millis() - LedGreenMoment < PAUSE_BEFORE_PARAMETER_SEND))
    {
        ShowConnectionQuality();
    }
    else
    {
        ShowSendingParameters();
    }

    if ((LastAutoModelSelect != AutoModelSelect) || (!ModelsMacUnionSaved.Val64))
    {
        LastAutoModelSelect = AutoModelSelect;
        ShowAMS();
    }
    if (LastCopyTrimsToAll != CopyTrimsToAll)
    {
        LastCopyTrimsToAll = CopyTrimsToAll;
        ShowTrimToAll();
    }
    if (OldRate != DualRateInUse)
    {
        OldRate = DualRateInUse;
        ShowCurrentRate();
    }

    if (BuddyPupilOnWireless)
    {
        SendText(FrontView_Connected, MsgBuddying);
    }

    if (LedWasGreen)
    {
        if (BoundFlag)
        {
            if (!Reconnected)
                Reconnected = true;
            StartInactvityTimeout();
        }
        else
        {
            SendText(FrontView_RXBV, na);       // data not available
            SendText(FrontView_AckPayload, na); // no need to optimise as not connected
            SendCommand(InVisible);             // no need to optimise as not connected
        }
    }
}

/*********************************************************************************************************************************/

/** @brief SHOW COMMS */

// This displays many telemetry data onto the current screen
// It is called once a second
// it's re-optimised now to run in about 0.5ms usually

FASTRUN void ShowComms()
{
    if (millis() - LastShowTime < SHOWCOMMSDELAY)
        return; // 10x a second is more than enough .. only called once a second anyway
    LastShowTime = millis();

    if (GPS_RX_FIX)
    {
        if (FirstGPSfix)
        {
            FirstGPSfix = false;
            PlaySound(BEEPCOMPLETE); // todo add new sound
        }
    }
    else
    {
        FirstGPSfix = true;
    }

    switch (CurrentView)
    {
    case FRONTVIEW:
        PopulateFrontView(); // This is the main screen
        break;
    case DATAVIEW:
        PopulateDataView(); // This is the telemetry data screen
        break;
    case GPSVIEW:
        PopulateGPSView(); // This is the GPS screen
        break;
    case GAPSVIEW:
        PopulateGapsView();
        break;
    default:
        break;
    }
    // Look(millis() - LastShowTime);    // This is to see how long it takes to run for optimisation purposes
} // end ShowComms()
// *********************************************************************************************************************************/
int GetTestRateOfClimb()
{
    // This is a test function to simulate the rate of climb
    // It will be replaced with the actual rate of climb from the sensor
    static int testRateOfClimb = 0;
    static uint32_t lastTestRateOfClimbCheck = 0;
    static int16_t RateChange = 200;
    if (millis() - lastTestRateOfClimbCheck < 1200)
        return testRateOfClimb;
    lastTestRateOfClimbCheck = millis();
    testRateOfClimb += RateChange; // Simulate a climb of 100 fpm

    if (testRateOfClimb > 2000)
    {
        RateChange = -100; // Simulate a sink of 100 fpm
    }
    else if (testRateOfClimb < -2000)
    {
        RateChange = 100; // Simulate a climb of 100 fpm
    }
    return testRateOfClimb;
}
// *******************************************************************************************
// Ultra‑snappy variometer – Teensy 4.1 + Nextion edition
// *** 10 climbing + 10 descending zones  (21 inc. neutral) ***
// *******************************************************************************************

// ──────────────────────────────────────────────────────────────────────────────
// 0.  USER‑TUNEABLE CONSTANTS
// ──────────────────────────────────────────────────────────────────────────────
//   • Edit SCALE to swap between flight (1.0) and bench (0.01) sensitivity.
//   • Replace the GOINGUPn / GOINGDOWNn IDs with your own WAV handles.
//   • Tweak WAV_MS to match real clip lengths.
// ----------------------------------------------------------------------------

static constexpr float SCALE = 1;      // 1 → in‑flight; 0.01 → bench wind‑tube
static constexpr int GAP_FRAC_NUM = 1; // gap = dur * 1 / 3  (≈33 %)
static constexpr int GAP_FRAC_DEN = 3;

// ──────────────────────────────────────────────────────────────────────────────
// 1.  ENUM & SOUND TABLES  (compile‑time, one copy in flash)
// ──────────────────────────────────────────────────────────────────────────────

enum Zone : uint8_t
{
    Z_NEUTRAL = 0,
    Z_CLIMB1,
    Z_CLIMB2,
    Z_CLIMB3,
    Z_CLIMB4,
    Z_CLIMB5,
    Z_CLIMB6,
    Z_CLIMB7,
    Z_CLIMB8,
    Z_CLIMB9,
    Z_CLIMB10,
    Z_SINK1,
    Z_SINK2,
    Z_SINK3,
    Z_SINK4,
    Z_SINK5,
    Z_SINK6,
    Z_SINK7,
    Z_SINK8,
    Z_SINK9,
    Z_SINK10,
    Z_COUNT
};

static constexpr uint16_t WAV_ID[Z_COUNT] = {
    0, // neutral (silent)
    GOINGUP1, GOINGUP2, GOINGUP3, GOINGUP4, GOINGUP5,
    GOINGUP6, GOINGUP7, GOINGUP8, GOINGUP9, GOINGUP10,
    GOINGDOWN1, GOINGDOWN2, GOINGDOWN3, GOINGDOWN4, GOINGDOWN5,
    GOINGDOWN6, GOINGDOWN7, GOINGDOWN8, GOINGDOWN9, GOINGDOWN10};

static constexpr uint16_t WAV_MS[Z_COUNT] = {
    // ***** These are actual lengths of the audio clips In milliseconds **************
    0,                       // neutral – no loop
    500, 400, 350, 300, 250, // climbs 1‑5
    200, 175, 150, 125, 100, // climbs 6‑10  (shorter & more urgent)
    500, 450, 400, 350, 300, // sinks  1‑5
    250, 200, 175, 150, 120  // sinks  6‑10
};

static_assert(sizeof(WAV_ID) / sizeof(WAV_ID[0]) == Z_COUNT, "WAV_ID size mismatch");
static_assert(sizeof(WAV_MS) / sizeof(WAV_MS[0]) == Z_COUNT, "WAV_MS size mismatch");

// ──────────────────────────────────────────────────────────────────────────────
// 2.  VARIOMETER ROUTINE   (call freely from loop())
// ──────────────────────────────────────────────────────────────────────────────

void DoTheVariometer()
{
    //------------------------------------------------------------------
    // 2.1  Early exits (rate‑limit and pilot conditions)
    //------------------------------------------------------------------
    static uint32_t nextCheckMs = 0; // 20 Hz cadence (50 ms)
    uint32_t now = millis();

    if (now < nextCheckMs)
        return;
    nextCheckMs = now + 50;
    if (!UseVariometer || !(BoundFlag && ModelMatched) ||
        (now - LedGreenMoment < 10000) ||
        ((Bank != VariometerBank) && (VariometerBank != 0)))
        return;

    //------------------------------------------------------------------
    // 2.2  One‑shot threshold initialisation
    //------------------------------------------------------------------
    static int16_t T[10];   // climb thresholds (ft/min)
    static int16_t HYS_FPM; // hysteresis band

    if (!Variometer_InitDone)
    {
        uint16_t base = VariometerThreshold; // first climb or sink band begins at 400 fpm, so silent means little change.
        for (uint8_t i = 0; i < 10; ++i)
        {
            T[i] = int(base * SCALE + 0.5f);
            base += VariometerSpacing; // spacing between bands (edit ... was 200. 125 might be better)
        }
        HYS_FPM = int(25 * SCALE + 0.5f);
        Variometer_InitDone = true;
    }

    //------------------------------------------------------------------
    // 2.3  Map ft/min → Zone
    //------------------------------------------------------------------
    int roc = RateOfClimb; // +ve climb, ‑ve sink
#ifdef DB_Variometer
    roc = GetTestRateOfClimb(); // test function to simulate the rate of climb
    Look1(roc);                 // for debugging
    Look1("fpm. zone: ");
#endif
    Zone zone = Z_NEUTRAL;

    // climbs (scan highest to lowest)
    if (roc > T[9] + HYS_FPM)
        zone = Z_CLIMB10;
    else if (roc > T[8] + HYS_FPM)
        zone = Z_CLIMB9;
    else if (roc > T[7] + HYS_FPM)
        zone = Z_CLIMB8;
    else if (roc > T[6] + HYS_FPM)
        zone = Z_CLIMB7;
    else if (roc > T[5] + HYS_FPM)
        zone = Z_CLIMB6;
    else if (roc > T[4] + HYS_FPM)
        zone = Z_CLIMB5;
    else if (roc > T[3] + HYS_FPM)
        zone = Z_CLIMB4;
    else if (roc > T[2] + HYS_FPM)
        zone = Z_CLIMB3;
    else if (roc > T[1] + HYS_FPM)
        zone = Z_CLIMB2;
    else if (roc > T[0] + HYS_FPM)
        zone = Z_CLIMB1;

    // sinks (mirror thresholds)
    else if (roc < -T[9] - HYS_FPM)
        zone = Z_SINK10;
    else if (roc < -T[8] - HYS_FPM)
        zone = Z_SINK9;
    else if (roc < -T[7] - HYS_FPM)
        zone = Z_SINK8;
    else if (roc < -T[6] - HYS_FPM)
        zone = Z_SINK7;
    else if (roc < -T[5] - HYS_FPM)
        zone = Z_SINK6;
    else if (roc < -T[4] - HYS_FPM)
        zone = Z_SINK5;
    else if (roc < -T[3] - HYS_FPM)
        zone = Z_SINK4;
    else if (roc < -T[2] - HYS_FPM)
        zone = Z_SINK3;
    else if (roc < -T[1] - HYS_FPM)
        zone = Z_SINK2;
    else if (roc < -T[0] - HYS_FPM)
        zone = Z_SINK1;

    //------------------------------------------------------------------
    // 2.4  Clip scheduling / looping
    //------------------------------------------------------------------
    static Zone lastZone = Z_NEUTRAL;
    static uint32_t lastPlayMs = 0;

    bool needPlay = false;
#ifdef DB_Variometer
    Look(zone); // for debugging
#endif
    if (zone != lastZone)
    {
        lastZone = zone;
        lastPlayMs = now;
        needPlay = (zone != Z_NEUTRAL);
    }
    else if (zone != Z_NEUTRAL)
    {
        uint32_t dur = WAV_MS[zone];
        if (dur == 0)
            dur = 100;                                      // safety default
        uint32_t gap = (dur * GAP_FRAC_NUM) / GAP_FRAC_DEN; // e.g. dur / 3
        if (now - lastPlayMs >= dur + gap)
        {
            lastPlayMs = now;
            needPlay = true;
        }
    }

    if (needPlay)
        PlaySound(WAV_ID[zone]);
}
// **********************************************************************************************************
void CalculateGapPercentages()
{
    if (!GapCount)
        return;
    for (int i = 0; i < 11; ++i)
    {
        GapPercentages[i] = (GapSets[i] * 100.0f) / GapCount;
        SimplePing();
    }
}
// **********************************************************************************************************
void PopulateGapsView()
{
    char PerCents[11][6] = {"t13", "t15", "t17", "t18", "t19", "t20", "t21", "t22", "t23", "t24", "t25"};
    char visible[11][12] = {"vis n0,1", "vis n1,1", "vis n2,1", "vis n3,1", "vis n4,1", "vis n5,1", "vis n6,1", "vis n7,1", "vis n8,1", "vis n9,1", "vis n10,1"};
    char invisible[11][12] = {"vis n0,0", "vis n1,0", "vis n2,0", "vis n3,0", "vis n4,0", "vis n5,0", "vis n6,0", "vis n7,0", "vis n8,0", "vis n9,0", "vis n10,0"};

    char Js[11][4] = {
        "j0",
        "j10",
        "j11",
        "j6",
        "j5",
        "j4",
        "j3",
        "j9",
        "j2",
        "j8",
        "j7",
    };
    char Ns[11][4] = {"n0", "n1", "n2", "n3", "n4", "n5", "n6", "n7", "n8", "n9", "n10"};
    char PercentText[6];
    CalculateGapPercentages();
    ClearNextionCommand();
    for (int i = 0; i < 11; ++i)
    {
        SimplePing();
        if (GapSets[i] != PrevGapSets[i])
        {
            BuildValue(Ns[i], GapSets[i]);
            if (GapSets[i])
                BuildNextionCommand(visible[i]);
            else
                BuildNextionCommand(invisible[i]);
            uint32_t n = GapSets[i];
            if (n > MaxBin)
                MaxBin = n;
            n = MapWithExponential(n, 0, MaxBin, 0, 100, -15);
            BuildValue(Js[i], n);
            PrevGapSets[i] = GapSets[i];
        }
        if (GapSets[i])
        {
            if (!GapPercentages[i])
            {
                BuildText(PerCents[i], (char *)"< 1%");
            }
            else
            {
                Str(PercentText, GapPercentages[i], 0);
                strcat(PercentText, "%");
                BuildText(PerCents[i], PercentText);
            }
        }
        else
        {
            BuildText(PerCents[i], (char *)" ");
        }
        BuildValue((char *)"n13", GapLongest);
        BuildValue((char *)"n12", GapAverage);

        if (GapShortest < 1000)
            BuildValue((char *)"n11", GapShortest);
    }
    SimplePing();
    SendCommand(NextionCommand);
    SimplePing();
    ClearNextionCommand();
    SimplePing();
}
// **********************************************************************************************************
void StartGapsView()
{
    static bool FirstCall = true;
    SendCommand(pGapsView);
    CurrentView = GAPSVIEW;
    if (FirstCall)
        InitializeCommsGapScreen();
    FirstCall = false;
    PopulateGapsView();
}
#endif