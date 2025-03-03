// *********************************************** Telemetry.h for Transmitter code *******************************************

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef TELEMETRY_H
    #define TELEMETRY_H

/*********************************************************************************************************************************/
FASTRUN bool CheckTXVolts()
{
    char  JTX[]            = "JTX";  // Labels on Nextion
    char  FrontView_TXBV[] = "TXBV"; // Labels on Nextion
    bool  TXWarningFlag    = false;
    float TransmitterBatteryPercentLeft, TXVoltsRaw;
    char  TXBattInfo[80];
    char  pc[] = "%";
    char  nbuf[10]; // Little buffer for numbers
    char  v[]   = "V";
    char  t17[] = "t17";

    if (USE_INA219) {
        TXVoltsRaw = ((ina219.getBusVoltage_V()) * 100) + (TxVoltageCorrection * 2)    ;        // Correction for inaccurate ina219
        dtostrf(TXVoltsRaw / 200, 2, 2, nbuf);                                                  // Volts per cell
        TXVoltsPerCell = TXVoltsRaw / 200;
        if (TXLiPo) {                                                                           // Does TX have a LiPo or a LiFePo4?
            TransmitterBatteryPercentLeft = map(TXVoltsRaw, 3.5 * 200, 4.00 * 200, 0, 100);     // LIPO Battery 3.50 -> c. 4.00  volts per cell
        }
        else {                                                                                   // No, it's a LiFePo4
            TransmitterBatteryPercentLeft = map(TXVoltsRaw, 3.2 * 200, 3.33 * 200, 0, 100);      // LiFePo4 Battery 3.1 -> 3.35  volts per cell
        }
        if (TransmitterBatteryPercentLeft < LowBattery) {
            TXWarningFlag = true;
            // Always set warning sound, but PlaySound is controlled elsewhere
            WarningSound = BATTERYISLOW;
        }
        TransmitterBatteryPercentLeft = constrain(TransmitterBatteryPercentLeft, 0, 100);
        strcat(TXBattInfo, pc);
        if (CurrentView == FRONTVIEW) {
            SendValue(JTX, TransmitterBatteryPercentLeft);
            strcat(nbuf, v);
            SendText(FrontView_TXBV, nbuf);
        }
        if (CurrentView == DATAVIEW) {
            SendText(t17, nbuf);
        }
    }
  
    return TXWarningFlag;
}

/*********************************************************************************************************************************/

FASTRUN bool CheckRXVolts()
{
    static bool    RXWarningFlag   = false;
    float   ReadVolts       = 0;
    uint8_t GreenPercentBar = 0;
    char    JRX[]           = "JRX";
    
    char    Vbuf[10];
    char    RXBattInfo[80];
    
    char    FrontView_RXBV[] = "RXBV";
    char    RXPC[]           = "RXPC";
    char    PerCell[]        = " per cell)";
    char    RXBattNA[]       = ""; //"(No data from RX)";
    char    v[]              = "V  (";
    char    pc[]             = "%";
    char    spaces[]         = "  ";
    char    t6[]             = "t6";

    ReadVolts       = (RXModelVolts * 100) + (RxVoltageCorrection * RXCellCount);
    GreenPercentBar = map(ReadVolts, 3.4f * RXCellCount * 100, 4.2f * RXCellCount * 100, 0, 100);
    if (RXVoltsDetected) {
        GreenPercentBar = constrain(GreenPercentBar, 0, 100);
        if (BoundFlag) {
            RXVoltsPerCell = (ReadVolts / RXCellCount) / 100;
            if (CurrentView == FRONTVIEW) {
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
            if (CurrentView == DATAVIEW) {
                dtostrf(RXVoltsPerCell, 2, 2, Vbuf);
                SendText(t6, Vbuf);
            }
            RXWarningFlag = false;
            if ((RXVoltsPerCell < StopFlyingVoltsPerCell) && (RXVoltsPerCell > 2)){
                RXWarningFlag = true;
                // Always set warning sound, but PlaySound is controlled elsewhere
                WarningSound = STORAGECHARGE;
            }
        }
    }
    else {
        if (BoundFlag && CurrentView == FRONTVIEW) {
            SendText(FrontView_RXBV, RXBattNA);
            SendValue(JRX, 0);
            SendText(RXPC, spaces);
        }
    }
    return RXWarningFlag;
}

/*********************************************************************************************************************************/

void CheckBatteryStates(){
    static uint32_t WarnTimer = 0;
    // Removed WarnNow - we never want to show the Warning label
    char         WarnOff[]             = "vis Warning,0";
    char         LowBatteryMsg[]       = "*** LOW BATTERY ***";
    char         FrontView_Connected[] = "Connected";

    // ULTRA-NUCLEAR FIX: Always force Warning label off regardless of what else happens
    if (CurrentView == FRONTVIEW) {
        SendCommand(WarnOff);
    }
  
    // Store whether we have low battery conditions
    bool txLowBattery = CheckTXVolts();
    bool rxLowBattery = CheckRXVolts();
    bool lowBatteryCondition = (txLowBattery || rxLowBattery);
    
    // If signal quality warning is active, just record the condition but don't display
    // FIXED BY CLAUDE 3.7 CODE MAR 3 2025: Record battery state even when signal warning is active
    if (SignalQualityWarningActive) {
        // Just record the low battery condition - no UI updates
        LedIsBlinking = lowBatteryCondition || LedIsBlinking;
        return; // Let CheckSignalQuality handle the UI updates
    }
   
    // Only proceed with battery warnings if no signal quality warning is active
    if (lowBatteryCondition) {
        if (millis() - WarnTimer > 5000){  // issue warning every 5 seconds
            WarnTimer = millis();
            if (ModelMatched && Connected) {
                // Check again - signal quality might have become active while checking voltages
                if (SignalQualityWarningActive) {
                    return; // Let CheckSignalQuality handle the UI updates
                }
                
                // Increase volume for battery warnings
                if (!UsingHighVolumeForWarning) {
                    // Store current volume setting
                    SavedAudioVolume = AudioVolume;
                    // Set to high volume for warnings
                    SetAudioVolume(90);
                    UsingHighVolumeForWarning = true;
                }
                
                PlaySound(WarningSound);      // Issue audible warning
                LogStopFlyingMsg();           // Log the stop flying message
                LogRXVoltsPerCell();          // Log the RX volts per cell
                LedIsBlinking = true;
                
                if (CurrentView == FRONTVIEW) {
                    // One more check before displaying battery warning
                    if (SignalQualityWarningActive) {
                        return; // Let CheckSignalQuality handle the UI updates
                    }
                    // ULTRA-NUCLEAR FIX: Don't use WarnNow at all - just show the text message
                    // SendCommand(WarnNow); - DISABLED BY CLAUDE 3.7 CODE MAR 2 2025
                    SendCommand(WarnOff); // Always keep Warning label off
                    SendText(FrontView_Connected, LowBatteryMsg);
                }
            }
        }
    }
    else {
        // Only clear battery warnings if signal quality warning is not active
        if (SignalQualityWarningActive) {
            return; // Let CheckSignalQuality handle the UI updates
        }
        
        if (LedIsBlinking && (CurrentView == FRONTVIEW)) {
            SendCommand(WarnOff);
            SendText(FrontView_Connected, FrontView_Connected);
            LedIsBlinking = false;
            
            // Restore volume when clearing battery warnings
            if (UsingHighVolumeForWarning) {
                SetAudioVolume(SavedAudioVolume);
                UsingHighVolumeForWarning = false;
            }
        }
    }
    
    // No final check - our new CheckSignalQuality will handle all UI updates
    // This greatly simplifies the code and avoids conflicts
}

/*********************************************************************************************************************************/
void ShowCurrentRate(){ // if it has changed
    char         rate[]                = "rate";
    char         rate1[]               = "Rate 1";
    char         rate2[]               = "Rate 2";
    char         rate3[]               = "Rate 3";
    char         rate4[]               = "      ";
 
 switch (DualRateInUse)
            {
                case 1:
                    SendText(rate, rate1);//
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

void ShowAMS(){
    char         ams[]                 = "ams";
    char         AmsOnMsg[]            = "AMS  ";
    char         AmsOffMsg[]           = "     ";
    
    if(!ModelsMacUnionSaved.Val64){ 
        strcpy(AmsOnMsg, "[AMS]"); 
    }
    
    if (AutoModelSelect) {
        SendText(ams, AmsOnMsg);
    }
    else {
        SendText(ams, AmsOffMsg);
    }
}

/*********************************************************************************************************************************/
void ShowTrimToAll(){
    char         trall[]               = "trall";
    char         CopyTrimsToAllMSG[]   = "TrimAll";
    char         CopyTrimsToNoneMSG[]  = "       ";

        if (CopyTrimsToAll) {
            SendText(trall, CopyTrimsToAllMSG);
        }
        else {
            SendText(trall, CopyTrimsToNoneMSG);
        }
}

/*********************************************************************************************************************************/

/**
 * @brief Populates the GPS view on the NEXTION data screen with the current GPS data.
 * 
 * This function updates the labels on the GPS view screen with the current GPS data. It displays information such as GPS fix status, longitude, latitude, bearing, distance, speed, altitude, and satellite count.
 * 
 * @note The function assumes that the GPS data variables (e.g., GPS_RX_FIX, GPS_RX_Satellites, GPS_RX_Longitude, etc.) have been properly initialized before calling this function.
 */
void PopulateGPSView(){

    char         Vbuf[50];
    char         Fix[]                 = "Fix"; // These are label names in the NEXTION data screen. They are best kept short.
    char         Lon[]                 = "Lon";
    char         Lat[]                 = "Lat";
    char         Bear[]                = "Bear";
    char         Dist[]                = "Dist";
    char         Sped[]                = "Sped";    
    char         yes[]                 = "Yes";
    char         no[]                  = "No";  
    char         ALT[]                 = "ALT";
    char         MALT[]                = "MALT";
    char         MxS[]                 = "MxS";
    char         Mxd[]                 = "Mxd";
    char         BTo[]                 = "BTo";
    char         Sat[]                 = "Sat";
     
        if (GPS_RX_FIX) { // if no fix, then leave display as before
            SendText(Fix, yes);
        }
        else {
            SendText(Fix, no);
        }
        snprintf(Vbuf, 7, "%d", GPS_RX_Satellites);
        SendText(Sat, Vbuf);
        snprintf(Vbuf, 7, "%.3f",GPS_RX_ANGLE);
        SendText(Bear, Vbuf);

        snprintf(Vbuf, 6, "%.3f", GPS_RX_Altitude);
        SendText(ALT, Vbuf);
        snprintf(Vbuf, 7, "%.3f", GPS_RX_DistanceTo);
        SendText(Dist, Vbuf);
        snprintf(Vbuf, 6, "%.3f", GPS_RX_CourseTo);
        SendText(BTo, Vbuf);
        snprintf(Vbuf, 15, "%.12f", GPS_RX_Longitude );
        SendText(Lon, Vbuf);
        snprintf(Vbuf, 15, "%.12f", GPS_RX_Latitude );
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

/*********************************************************************************************************************************/
void PopulateDataView(){

    char         DataView_pps[]         = "pps"; // These are label names in the NEXTION data screen. They are best kept short.
    char         DataView_lps[]         = "lps";
    char         DataView_Alt[]         = "alt";
    char         DataView_Temp[]        = "Temp";
    char         DataView_MaxAlt[]      = "MaxAlt";
    char         DataView_rxv[]         = "rxv";
    char         DataView_Ls[]          = "Ls";
    char         DataView_Ts[]          = "Ts";
    char         DataView_Rx[]          = "rx";
    char         DataView_Sg[]          = "Sg";
    char         DataView_Ag[]          = "Ag";
    char         DataView_Gc[]          = "Gc";
    char         Sbs[]                 = "Sbus";
    char         IdReceived[]          = "t22";
    char         IdStored[]            = "t19";
    char         IdReceived1[]         = "t23";
    char         IdStored1[]           = "t24";
    char         LocalMacID[]          = "t26";
    char         MasterID[]            = "t28";
    char         Vbuf[50];
    char         nb2[5];
    char         DataView_txv[]         = "txv";  
    char         MeanFrameRate[]        = "n0";
    char         TimeSinceBoot[]        = "n1";

    unsigned int TempModelId = 0;
    uint32_t BootedMinutes = millis() / 60000;

if (!LastPacketsPerSecond) 
        {                                                    // these only need displaying once - they will not change
            TempModelId = ModelsMacUnionSaved.Val32[0];
            snprintf(Vbuf, 9, "%X", TempModelId);
            if (TempModelId) SendText(IdStored, Vbuf);
            TempModelId = ModelsMacUnionSaved.Val32[1];
            snprintf(Vbuf, 9, "%X", TempModelId);
            if (TempModelId) SendText(IdStored1, Vbuf);
            TempModelId = ModelsMacUnion.Val32[0];
            snprintf(Vbuf, 9, "%X", TempModelId);
            if (TempModelId) SendText(IdReceived, Vbuf);
            TempModelId = ModelsMacUnion.Val32[1];
            snprintf(Vbuf, 9, "%X", TempModelId);
            if (TempModelId) SendText(IdReceived1, Vbuf);
            for (int i = 0; i < 5; ++i) {
                Vbuf[i] = 0;
            }
            for (int i = 4; i >= 0; --i) { //**
                snprintf(nb2, 4, "%X", BuddyMacAddress[i]);
                strcat(Vbuf, nb2);
                strcat(Vbuf, " ");
            }
            SendText(MasterID, Vbuf);
            for (int i = 0; i < 5; ++i) {
                Vbuf[i] = 0;
            }
            for (int i = 5; i > 0; --i) {            
                snprintf(nb2, 4, "%X", MacAddress[i]);
                strcat(Vbuf, nb2);
                strcat(Vbuf, " ");
            }
            SendText(LocalMacID, Vbuf);
            SendText(DataView_txv, TransmitterVersionNumber);
            if (BoundFlag && ModelMatched) SendText(DataView_rxv, ReceiverVersionNumber);
        }

        if (LastPacketsPerSecond != PacketsPerSecond) {
            LastPacketsPerSecond = PacketsPerSecond;
            SendValue(DataView_pps, PacketsPerSecond);
        }
        if (LastLostPackets != TotalLostPackets) {
            LastLostPackets = TotalLostPackets;
            SendValue(DataView_lps, TotalLostPackets);   
        }
        if (LastGapLongest != GapLongest) {
            LastGapLongest = GapLongest;
            SendValue(DataView_Ls, GapLongest);
        }
        if (LastRadioSwaps != RadioSwaps) {
            LastRadioSwaps = RadioSwaps;
            SendValue(DataView_Ts, RadioSwaps);
        }
        if (LastRX1TotalTime != RX1TotalTime) {
            LastRX1TotalTime = RX1TotalTime;
            SendValue(DataView_Sg, RX1TotalTime);
        }
        if (LastGapAverage != GapAverage) {
            LastGapAverage = GapAverage;
            SendValue(DataView_Ag, GapAverage);
        }
        if (LastRX2TotalTime != RX2TotalTime) {
            LastRX2TotalTime = RX2TotalTime;
            SendValue(DataView_Gc, RX2TotalTime);
        }
        
        if (RXModelAltitude != LastRXModelAltitude) {
            LastRXModelAltitude = RXModelAltitude;
            SendText(DataView_Alt,   ModelAltitude);
        }
        
        if (RXMAXModelAltitude != LastRXModelMaxAltitude) {
            LastRXModelMaxAltitude = RXMAXModelAltitude;
            SendText(DataView_MaxAlt, Maxaltitude);
        }
        
        if (RXTemperature != LastRXTemperature) {
            LastRXTemperature = RXTemperature;
            SendText(DataView_Temp,   ModelTempRX);
        }

        if (RadioNumber != LastRadioNumber) {
            LastRadioNumber = RadioNumber;
            SendText(DataView_Rx, ThisRadio);
        }

        if (LastSbusRepeats != SbusRepeats) {
            LastSbusRepeats = SbusRepeats;
            snprintf(Vbuf, 7, "%d", (int)SbusRepeats);
            SendText(Sbs, Vbuf);
        }
            SendValue(TimeSinceBoot, BootedMinutes);
        if (BoundFlag && ModelMatched){
            SendValue(MeanFrameRate, AverageFrameRate);
        }else{
            SendValue(MeanFrameRate, 0);
        }
    }


/*********************************************************************************************************************************/
// this function looks at the most recent ((uint16_t) ConnectionAssessSeconds) few seconds of packets which succeeded and expresses these
// as a percentage of total attempted packets using a progress bar on the screen.

void ShowConnectionQuality()
{
    char Quality[] = "Quality";
    char FrontView_Connected[]    = "Connected";                    // this is both the label name and the text to be displayed :=)
    char Visible[]                = "vis Quality,1";
    char TXModuleMSG[]            = "** Using TX module **";
    
#ifdef TESTSIGNALMSGS
    // In test mode, calculate quality from the PreviousConnectionQualityState
    uint16_t ConnectionQuality;
    if (PreviousConnectionQualityState == 2) {
        // Critical - around 25% quality
        ConnectionQuality = 25;
    } else if (PreviousConnectionQualityState == 1) {
        // Warning - around 60% quality
        ConnectionQuality = 60;
    } else {
        // Good - 95% quality
        ConnectionQuality = 95;
    }
#else
    // Normal operation - calculate from success rate
    uint16_t ConnectionQuality = GetSuccessRate();
#endif

    // FIXED BY CLAUDE 3.7 CODE MAR 2 2025: Don't overwrite signal quality warning with TX module message
    if (PPMdata.UseTXModule && !SignalQualityWarningActive) {
        SendText(FrontView_Connected, TXModuleMSG);
    }
    
    if (!LedWasGreen) return;
    
    // FIXED BY CLAUDE 3.7 CODE MAR 2 2025: Don't update connected text if signal quality warning is active
    if (!LastConnectionQuality && !SignalQualityWarningActive) {
        SendText(FrontView_Connected, FrontView_Connected);
        SendCommand(Visible);
    }
    
    if (ConnectionQuality != LastConnectionQuality) {
        LastConnectionQuality = ConnectionQuality;
        SendValue(Quality, ConnectionQuality);  // Update the progress bar but don't change warning text
    }
    
    
}

/*********************************************************************************************************************************/

void ShowSendingParameters(){
    char FrontView_Connected[]    = "Connected"; 
    char SendingParameters[]      = "Sending parameters to RX";
    if (!LedWasGreen) return;
    SendText(FrontView_Connected, SendingParameters);
}
/*********************************************************************************************************************************/

void  PopulateFrontView(){
    
    char         FrontView_AckPayload[] = "AckPayload";
    char         FrontView_RXBV[]       = "RXBV";
    char         MsgBuddying[]          = "* Buddy *";
    char         FrontView_Connected[]  = "Connected";
    char         InVisible[]            = "vis Quality,0";
    
    // FIXED BY CLAUDE 3.7 CODE MAR 2 2025: No need to force updates here - CheckSignalQuality handles everything
    // Just ensure the flag is set if a warning is active
    if (SignalQualityWarningActive) {
        // The periodic checker in CheckSignalQuality will handle UI updates
    }
        
    if (ParametersToBeSentPointer == 0) {
        ShowConnectionQuality();
    } else {
        // Only show parameters message if no signal quality warning is active
        if (!SignalQualityWarningActive) {
            ShowSendingParameters();
        }
    }
    
    // FIXED BY CLAUDE 3.7 CODE MAR 2 2025: No need to force updates here - CheckSignalQuality handles everything
    
    if ((LastAutoModelSelect != AutoModelSelect) || (!ModelsMacUnionSaved.Val64)){
        LastAutoModelSelect = AutoModelSelect;
        ShowAMS();
    }
    
    // FIXED BY CLAUDE 3.7 CODE MAR 2 2025: No need to force updates here - CheckSignalQuality handles everything
    
    if (LastCopyTrimsToAll != CopyTrimsToAll) {
        LastCopyTrimsToAll = CopyTrimsToAll;
        ShowTrimToAll();
    }
    
    // FIXED BY CLAUDE 3.7 CODE MAR 2 2025: No need to force updates here - CheckSignalQuality handles everything
    
    if (OldRate != DualRateInUse) {
        OldRate = DualRateInUse;
        ShowCurrentRate();
    }
    
    // FIXED BY CLAUDE 3.7 CODE MAR 2 2025: No need to force updates here - CheckSignalQuality handles everything
    
    // Only show buddy message if no signal quality warning is active
    if ((BuddyPupilOnPPM || BuddyPupilOnWireless) && !SignalQualityWarningActive) {
        SendText(FrontView_Connected, MsgBuddying); 
    }
    
    // FIXED BY CLAUDE 3.7 CODE MAR 2 2025: No need to force updates here - CheckSignalQuality handles everything

    if (LedWasGreen) {
        if (BoundFlag) {
            if (!Reconnected) Reconnected = true;        
            StartInactvityTimeout();
        }
        else {
            SendText(FrontView_RXBV, na);               // data not available
            SendText(FrontView_AckPayload, na);         // no need to optimise as not connected
            SendCommand(InVisible);                     // no need to optimise as not connected
        }
    }
    
    // FIXED BY CLAUDE 3.7 CODE MAR 2 2025: No need to force updates here - CheckSignalQuality handles everything
}



/*********************************************************************************************************************************/

/** @brief SHOW COMMS */

// This displays many telemetry data onto the current screen
// It is called once a second
// it's re-optimised now to run in about 0.5ms usually 

FASTRUN void ShowComms() 
{
    // ULTRA-NUCLEAR SOLUTION: Force Warning label off regardless of other code
    // This runs even if we skip the rest of the function due to timing
    ForceWarningLabelOff();
    
    if (millis() - LastShowTime < SHOWCOMMSDELAY) return;  // 10x a second is enough
    LastShowTime = millis();
    
    // FIXED BY CLAUDE 3.7 CODE MAR 2 2025: No need to force updates here - CheckSignalQuality handles everything
    
#ifndef TESTSIGNALMSGS
    // Check signal quality - this function will warn about weak signals if needed
    // ADDED BY CLAUDE 3.7 CODE FEB 28 2025: Early warning system for connection quality
    CheckSignalQuality();
#else
    // In test mode, we manually update the signal quality display here
    // This is handled in the main loop by forcing PreviousConnectionQualityState
    if (CurrentView == FRONTVIEW) {
        // Removed WarnNow - we never want to show the Warning label
        char WarnOff[] = "vis Warning,0";
        char FrontView_Connected[] = "Connected";
        char SignalWarningCritical[] = "!!! CRITICAL SIGNAL !!!";
        char SignalWarning[] = "** WEAK SIGNAL **";
        char SignalGood[] = "CONNECTION GOOD";
        
        // Update based on the current test-forced quality state
        if (PreviousConnectionQualityState == 2) {
            // Critical quality - but ALWAYS keep Warning label hidden
            SendCommand(WarnOff);
            SendText(FrontView_Connected, SignalWarningCritical);
            SignalQualityWarningActive = true;
            LastSignalQualityRefresh = millis();
        } else if (PreviousConnectionQualityState == 1) {
            // Warning quality - but ALWAYS keep Warning label hidden
            SendCommand(WarnOff);
            SendText(FrontView_Connected, SignalWarning);
            SignalQualityWarningActive = true;
            LastSignalQualityRefresh = millis();
        } else {
            // Good quality
            SendCommand(WarnOff);
            SendText(FrontView_Connected, SignalGood);
            SignalQualityWarningActive = false;
        }
    }
#endif

    // FIXED BY CLAUDE 3.7 CODE MAR 2 2025: Force critical warning system before navigation
    if (SignalQualityWarningActive && CurrentView == FRONTVIEW) {
        ForceCriticalWarningSystem();
    }
    
    switch (CurrentView) {
            case FRONTVIEW:
                // No need to force updates here - CheckSignalQuality handles everything
                
                PopulateFrontView();    // This is the main screen
                
                // No need to force updates here - CheckSignalQuality handles everything
                break;
            case DATAVIEW:
                PopulateDataView();     // This is the telemetry data screen
                break;
            case GPSVIEW:
                PopulateGPSView();      // This is the GPS screen
                break;  
            default:    
                break;
    }
    
    // FIXED BY CLAUDE 3.7 CODE MAR 2 2025: No need to force updates here - CheckSignalQuality handles everything
   
   // Look(millis() - LastShowTime);    // This is to see how long it takes to run for optimisation purposes
}  // end ShowComms()

    #endif
