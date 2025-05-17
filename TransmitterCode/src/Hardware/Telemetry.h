// *********************************************** Telemetry.h for Transmitter code *******************************************

#include <Arduino.h>
#include "Hardware/1Definitions.h"
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
    char nbuf[10]; // Little buffer for numbers
    char v[] = "V";
    char t17[] = "t17";

    if (USE_INA219)
    {
        TXVoltsRaw = ((ina219.getBusVoltage_V()) * 100) + (TxVoltageCorrection * 2); // Correction for inaccurate ina219
        dtostrf(TXVoltsRaw / 200, 2, 2, nbuf);                                       // Volts per cell
        TXVoltsPerCell = TXVoltsRaw / 200;
        if (TXLiPo)
        {                                                                                   // Does TX have a LiPo or a LiFePo4?
            TransmitterBatteryPercentLeft = map(TXVoltsRaw, 3.5 * 200, 4.00 * 200, 0, 100); // LIPO Battery 3.50 -> c. 4.00  volts per cell
        }
        else
        {                                                                                   // No, it's a LiFePo4
            TransmitterBatteryPercentLeft = map(TXVoltsRaw, 3.2 * 200, 3.33 * 200, 0, 100); // LiFePo4 Battery 3.1 -> 3.35  volts per cell
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
            SendText(FrontView_TXBV, nbuf);
        }
        if (CurrentView == DATAVIEW)
        {
            SendText(t17, nbuf);
        }
    }

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
    char RXPC[] = "RXPC";
    char PerCell[] = " per cell)";
    char RXBattNA[] = ""; //"(No data from RX)";
    char v[] = "V  (";
    char pc[] = "%";
    char spaces[] = "  ";
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
                strcat(Str(Vbuf, GreenPercentBar, 0), pc);
                SendText(RXPC, Vbuf);
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
            SendText(FrontView_RXBV, RXBattNA);
            SendValue(JRX, 0);
            SendText(RXPC, spaces);
        }
    }
    return RXWarningFlag;
}

/*********************************************************************************************************************************/

void CheckBatteryStates()
{
    static uint32_t WarnTimer = 0;
    char WarnNow[] = "vis Warning,1";
    char WarnOff[] = "vis Warning,0";

    if ((CheckTXVolts() || CheckRXVolts()))
    { // Note: If TX Battery is low, then CheckRXVolts() is not even called
        if (millis() - WarnTimer > 5000)
        { // issue warning every 5 seconds
            WarnTimer = millis();
            if (ModelMatched && Connected)
            {
                PlaySound(WarningSound); // Issue audible warning // heer
                LogStopFlyingMsg();      // Log the stop flying message
                LogRXVoltsPerCell();     // Log the RX volts per cell
                LedIsBlinking = true;
                if (CurrentView == FRONTVIEW)
                    SendCommand(WarnNow);
            }
        }
    }
    else
    {
        if (LedIsBlinking && (CurrentView == FRONTVIEW))
            SendCommand(WarnOff);
        LedIsBlinking = false;
    }
}

/*********************************************************************************************************************************/
void ShowCurrentRate()
{ // if it has changed
    char rate[] = "rate";
    char rate1[] = "Rate 1";
    char rate2[] = "Rate 2";
    char rate3[] = "Rate 3";
    char rate4[] = "      ";

    switch (DualRateInUse)
    {
    case 1:
        SendText(rate, rate1); //
        break;
    case 2:
        SendText(rate, rate2);
        break;
    case 3:
        SendText(rate, rate3);
        break;
    case 4:
        SendText(rate, rate4); // rates not in use = 4
        break;
    default:
        break;
    }
}

/*********************************************************************************************************************************/

void ShowAMS()
{
    char ams[] = "ams";
    char AmsOnMsg[] = "AMS  ";
    char AmsOffMsg[] = "     ";

    if (!ModelsMacUnionSaved.Val64)
    {
        strcpy(AmsOnMsg, "[AMS]");
    }

    if (AutoModelSelect)
    {
        SendText(ams, AmsOnMsg);
    }
    else
    {
        SendText(ams, AmsOffMsg);
    }
}

/*********************************************************************************************************************************/
void ShowTrimToAll()
{
    char trall[] = "trall";
    char CopyTrimsToAllMSG[] = "TrimAll";
    char CopyTrimsToNoneMSG[] = "       ";

    if (CopyTrimsToAll)
    {
        SendText(trall, CopyTrimsToAllMSG);
    }
    else
    {
        SendText(trall, CopyTrimsToNoneMSG);
    }
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
    char IdReceived[] = "t22";
    char IdStored[] = "t19";
    char IdReceived1[] = "t23";
    char IdStored1[] = "t24";
    char LocalMacID[] = "t26";
    char MasterID[] = "t28";
    char Vbuf[50];
    char nb2[5];
    char DataView_txv[] = "txv";
    char MeanFrameRate[] = "n0";
    char TimeSinceBoot[] = "n1";
    unsigned int TempModelId = 0;
    uint32_t BootedMinutes = millis() / 60000;
    static int LastGroundModelAltitude = 0;

    ClearNextionCommand();

    if (!LastPacketsPerSecond)
    { // these only need displaying once - they will not change
        TempModelId = ModelsMacUnionSaved.Val32[0];
        snprintf(Vbuf, 9, "%X", TempModelId);
        if (TempModelId)
            BuildText(IdStored, Vbuf); // was SendText(IdStored, Vbuf);
        TempModelId = ModelsMacUnionSaved.Val32[1];
        snprintf(Vbuf, 9, "%X", TempModelId);
        if (TempModelId)

            BuildText(IdStored1, Vbuf); // was SendText(IdStored1, Vbuf);
        TempModelId = ModelsMacUnion.Val32[0];
        snprintf(Vbuf, 9, "%X", TempModelId);
        if (TempModelId)
            BuildText(IdReceived, Vbuf); // was SendText(IdReceived, Vbuf);
        TempModelId = ModelsMacUnion.Val32[1];
        snprintf(Vbuf, 9, "%X", TempModelId);
        if (TempModelId)
            BuildText(IdReceived1, Vbuf); // was SendText(IdReceived1, Vbuf);
        for (int i = 0; i < 5; ++i)
        {
            Vbuf[i] = 0;
        }
        for (int i = 4; i >= 0; --i)
        { //**
            snprintf(nb2, 4, "%X", BuddyMacAddress[i]);
            strcat(Vbuf, nb2);
            strcat(Vbuf, " ");
        }

        BuildText(MasterID, Vbuf); // was SendText(MasterID, Vbuf);

        for (int i = 0; i < 5; ++i)
        {
            Vbuf[i] = 0;
        }
        for (int i = 5; i > 0; --i)
        {
            snprintf(nb2, 4, "%X", MacAddress[i]);
            strcat(Vbuf, nb2);
            strcat(Vbuf, " ");
        }
        BuildText(LocalMacID, Vbuf);                       // SendText(LocalMacID, Vbuf);
        BuildText(DataView_txv, TransmitterVersionNumber); // SendText(DataView_txv, TransmitterVersionNumber);
        if (BoundFlag && ModelMatched)
            BuildText(DataView_rxv, ReceiverVersionNumber); // SendText(DataView_rxv, ReceiverVersionNumber);
    }

    if (LastPacketsPerSecond != PacketsPerSecond)
    {
        LastPacketsPerSecond = PacketsPerSecond;
        BuildValue(DataView_pps, PacketsPerSecond); // SendValue(DataView_pps, PacketsPerSecond);
    }
    if (LastLostPackets != TotalLostPackets)
    {
        LastLostPackets = TotalLostPackets;
        BuildValue(DataView_lps, TotalLostPackets); // SendValue(DataView_lps, TotalLostPackets);
    }
    if (LastGapLongest != GapLongest)
    {
        LastGapLongest = GapLongest;
        BuildValue(DataView_Ls, GapLongest); // SendValue(DataView_Ls, GapLongest);
    }
    if (LastRadioSwaps != RadioSwaps)
    {
        LastRadioSwaps = RadioSwaps;
        BuildValue(DataView_Ts, RadioSwaps); // SendValue(DataView_Ts, RadioSwaps);
    }
    if (LastRX1TotalTime != RX1TotalTime)
    {
        LastRX1TotalTime = RX1TotalTime;
        BuildValue(DataView_Sg, RX1TotalTime); // SendValue(DataView_Sg, RX1TotalTime);
    }
    if (LastGapAverage != GapAverage)
    {
        LastGapAverage = GapAverage;
        BuildValue(DataView_Ag, GapAverage); // SendValue(DataView_Ag, GapAverage);
    }
    if (LastRX2TotalTime != RX2TotalTime)
    {
        LastRX2TotalTime = RX2TotalTime;
        BuildValue(DataView_Gc, RX2TotalTime); // SendValue(DataView_Gc, RX2TotalTime);
    }

    if ((RXModelAltitude != LastRXModelAltitude) || (GroundModelAltitude != LastGroundModelAltitude))
    {
        LastRXModelAltitude = RXModelAltitude;
        BuildText(DataView_Alt, ModelAltitude); // SendText(DataView_Alt, ModelAltitude);
    }

    if ((RXMAXModelAltitude != LastRXModelMaxAltitude) || (GroundModelAltitude != LastGroundModelAltitude))
    {
        LastRXModelMaxAltitude = RXMAXModelAltitude;
        BuildText(DataView_MaxAlt, Maxaltitude); // SendText(DataView_MaxAlt, Maxaltitude);
        LastGroundModelAltitude = GroundModelAltitude;
    }

    if (RXTemperature != LastRXTemperature)
    {
        LastRXTemperature = RXTemperature;
        BuildText(DataView_Temp, ModelTempRX); // SendText(DataView_Temp, ModelTempRX);
    }

    if (RadioNumber != LastRXReceivedPackets)
    {
        LastRXReceivedPackets = RXSuccessfulPackets;
        snprintf(Vbuf, 7, "%" PRIu32, RXSuccessfulPackets);
        BuildText(DataView_Rx, Vbuf); // SendText(DataView_Rx, Vbuf);
    }

    if (LastMaxRateOfClimb != MaxRateOfClimb)
    {
        LastMaxRateOfClimb = MaxRateOfClimb;
        int feet = (int)MaxRateOfClimb + 0.5; // round to nearest integer
        snprintf(Vbuf, 26, "%d feet / minute", feet);
        BuildText(Sbs, Vbuf);
    }

    if (LastTimeSinceBoot != BootedMinutes)
    {
        BuildValue(TimeSinceBoot, BootedMinutes); // SendValue(TimeSinceBoot, BootedMinutes);
        LastTimeSinceBoot = BootedMinutes;
    }
    if (BoundFlag && ModelMatched)
        if (AverageFrameRate != LastAverageFrameRate)
        {
            BuildValue(MeanFrameRate, AverageFrameRate); // SendValue(MeanFrameRate, AverageFrameRate);
            LastAverageFrameRate = AverageFrameRate;
        }
    SendCommand(NextionCommand);
    // Look(NextionCommand); // This is to see how many were included for optimisation purposes
    ClearNextionCommand();
}

/*********************************************************************************************************************************/
// this function looks at the most recent ((uint16_t) ConnectionAssessSeconds) few seconds of packets which succeeded and expresses these
// as a percentage of total attempted packets using a progress bar on the screen.

void ShowConnectionQuality()
{
    char Quality[] = "Quality";
    char FrontView_Connected[] = "Connected"; // this is both the label name and the text to be displayed :=)
    char Visible[] = "vis Quality,1";
    char TXModuleMSG[] = "** Using TX module **";
    uint16_t ConnectionQuality = GetSuccessRate();
    if (PPMdata.UseTXModule)
        SendText(FrontView_Connected, TXModuleMSG);
    if (!LedWasGreen)
        return;
    if (!LastConnectionQuality)
    {
        SendText(FrontView_Connected, FrontView_Connected);
        SendCommand(Visible);
    } // once only!
    if (ConnectionQuality != LastConnectionQuality)
    {
        LastConnectionQuality = ConnectionQuality;
        SendValue(Quality, ConnectionQuality);
    } // only if changed
}

/*********************************************************************************************************************************/

void ShowSendingParameters()
{
    char FrontView_Connected[] = "Connected";
    char SendingParameters[] = "Sending parameters to RX";
    if (!LedWasGreen)
        return;
    SendText(FrontView_Connected, SendingParameters);
}
/*********************************************************************************************************************************/

void PopulateFrontView()
{

    char FrontView_AckPayload[] = "AckPayload";
    char FrontView_RXBV[] = "RXBV";
    char MsgBuddying[] = "* Buddy *";
    char FrontView_Connected[] = "Connected";
    char InVisible[] = "vis Quality,0";

    if (ParametersToBeSentPointer == 0)
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

    if (BuddyPupilOnPPM || BuddyPupilOnWireless)
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
        return; // 10x a second is more than enough
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
    if (millis() - lastTestRateOfClimbCheck < 100)
        return testRateOfClimb;
    lastTestRateOfClimbCheck = millis();
    testRateOfClimb += 100; // Simulate a climb of 100 fpm
    if (testRateOfClimb > 1000)
        testRateOfClimb = -1000; // Reset to -1000 fpm after reaching 1000 fpm
    return testRateOfClimb;
}

// ***** USER-TUNEABLE CONSTANTS ********************************************************************************************************************************************************************
// Variometer settings
// These are the thresholds for the variometer sounds. The values are in feet per minute (fpm). 200, 500, & 800
constexpr float SCALE = 1; // 1 for flight, 0.01 on the test bench
constexpr int T1_FPM = int(200 * SCALE + 0.5f);
constexpr int T2_FPM = int(500 * SCALE + 0.5f);
constexpr int T3_FPM = int(800 * SCALE + 0.5f);
constexpr int HYS_FPM = int(25 * SCALE + 0.5f);
constexpr uint16_t WAV_ID[] = {0, GOINGUP1, GOINGUP2, GOINGUP3, GOINGDOWN1, GOINGDOWN2, GOINGDOWN3};
constexpr uint16_t WAV_MS[] = {395, 395, 395, 395, 395, 395, 395}; //  lengths of wave files in ms
// ****************************************************************************
enum Zone : uint8_t
{
    Z_NEUTRAL = 0,
    Z_CLIMB1,
    Z_CLIMB2,
    Z_CLIMB3,
    Z_SINK1,
    Z_SINK2,
    Z_SINK3
};
// *******************************************************************************************
// Ultra-snappy variometer – first tone within ~50-100 ms of a climb/sink starting
// *******************************************************************************************
void DoTheVariometer() // call freely from loop(), it rate-limits itself
{
    static Zone lastZone = Z_NEUTRAL;
    static uint32_t nextCheckMs = 0; // scheduler for our own 20 Hz cadence
    static uint32_t lastPlayMs = 0;  // when the current clip began

    uint32_t now = millis();
    if (now < nextCheckMs) // run at 20 Hz (every 50 ms)
        return;
    nextCheckMs = now + 50; // -----------------------------

    // Abort unless: vario ON, model bound, >10 s since bind, bank = 3
    if (!UseVariometer || !(BoundFlag && ModelMatched) ||
        (now - LedGreenMoment < 10000) || (Bank != 3))
        return;

    // ---------- 1. Work out which rate-of-climb “zone” we are in ----------
    int roc = RateOfClimb; // ft / min, +ve = climb

   // roc = GetTestRateOfClimb(); // this is ONLY the test rate of climb
   // Look(roc); // this is ONLY the test rate of climb
   
   
    Zone zone;
    if (roc > T3_FPM + HYS_FPM)
        zone = Z_CLIMB3;
    else if (roc > T2_FPM + HYS_FPM)
        zone = Z_CLIMB2;
    else if (roc > T1_FPM + HYS_FPM)
        zone = Z_CLIMB1;
    else if (roc < -T3_FPM - HYS_FPM)
        zone = Z_SINK3;
    else if (roc < -T2_FPM - HYS_FPM)
        zone = Z_SINK2;
    else if (roc < -T1_FPM - HYS_FPM)
        zone = Z_SINK1;
    else
        zone = Z_NEUTRAL;

    // ---------- 2. Should we start (or restart) a sound? ------------------
    bool needPlay = false;

    if (zone != lastZone)
    { // BAND CHANGED  → play at once
        lastZone = zone;
        lastPlayMs = now;
        needPlay = (zone != Z_NEUTRAL); // silent in neutral
    }
    else if (zone != Z_NEUTRAL)
    {                                // SAME BAND  → maybe loop
        uint32_t dur = WAV_MS[zone]; // declared clip length
        if (dur == 0)
            dur = 100;                    // fail-safe: assume 100 ms
        if (now - lastPlayMs >= dur + 20) // 20 ms gap then re-arm
        {
            lastPlayMs = now;
            needPlay = true;
        }
    }

    if (needPlay)
        PlaySound(WAV_ID[zone]); // PlaySound is pre-emptive
}

// *******************************************************************************************
void DoTheVariometerOLD()
{
    static uint32_t LastRateOfClimbCheck = 0;
    static bool GoingUp = false;
    static bool GoingDown = false;
    static uint32_t UpStart = 0;
    static uint32_t DownStart = 0;

#define MINIMUMRATE 500    // 500 feet per second
#define NOISEDURATION 1500 // 1.5 seconds

    if (millis() - LastRateOfClimbCheck < 100)
        return;
    LastRateOfClimbCheck = millis();
    if (!UseVariometer || !(BoundFlag && ModelMatched))
        return;

    if (RateOfClimb > MINIMUMRATE)
    {
        if (millis() - UpStart > NOISEDURATION)
            GoingUp = false;

        if (!GoingUp)
        {
            PlaySound(GOINGUP);
            GoingUp = true;
            GoingDown = false;
            UpStart = millis();
        }
    }
    if (RateOfClimb < -MINIMUMRATE)
    {
        if (millis() - DownStart > NOISEDURATION)
            GoingDown = false;
        if (!GoingDown)
        {
            PlaySound(GOINGDOWN);
            GoingUp = false;
            GoingDown = true;
            DownStart = millis();
        }
    }
    // Look(RateOfClimb);
}

#endif
