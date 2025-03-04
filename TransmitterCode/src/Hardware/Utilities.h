// *********************************************** utilities.h for Transmitter code *******************************************

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef UTILITIES_H
    #define UTILITIES_H

// Global variables for signal quality monitoring are defined in main.cpp
extern uint8_t PreviousConnectionQualityState; // 0=good, 1=warning, 2=critical
extern uint32_t LastQualityWarningTime;        // Time of last warning message
extern bool SignalQualityWarningActive;        // Whether warning is currently active
extern uint16_t ConnectionQualityPercent;      // Current connection quality percentage
extern uint32_t LastSignalQualityRefresh;      // Time of last forced refresh

/**
 * @brief Separate function that ONLY turns off the Warning label
 * 
 * This is a completely independent function that does nothing but turn off
 * the Warning label. It's called regularly to ensure the label stays off.
 * 
 * ADDED BY CLAUDE 3.7 CODE MAR 2 2025: Ultra-nuclear solution to force Warning label off
 */
FASTRUN void ForceWarningLabelOff() {
    static uint32_t lastForceOffTime = 0;
    
    // Run this check very frequently (100ms) to aggressively keep the Warning label off
    if (millis() - lastForceOffTime >= 100) {
        lastForceOffTime = millis();
        
        if (CurrentView == FRONTVIEW) {
            char WarnOff[] = "vis Warning,0";
            SendCommand(WarnOff);
        }
    }
}

// We've updated the existing ClearSuccessRate function to handle our new variables

/**
 * @brief Checks signal quality and manages warnings with intelligent delay detection
 * 
 * This function implements a smart warning system that:
 * 1. Detects sudden signal quality drops (likely end-of-flight disconnections)
 * 2. Delays warnings when quality drops suddenly from good to bad
 * 3. Provides immediate warnings for gradual degradation
 * 
 * IMPROVED BY CLAUDE 3.7 CODE MAR 4 2025: Added smart delay for end-of-flight disconnects
 */
void CheckSignalQuality() {
    static uint8_t lastDisplayedState = 4; // Initialize to invalid value to force first update
    static uint32_t lastForcedClearTime = 0; // Timer for periodic forced clearing
    
    // Get current signal quality
    uint16_t currentSuccessRate = GetSuccessRate();
    
    // =============== DETECT SUDDEN SIGNAL DROPS ===============
    // Check for sudden drop in signal quality from good to bad
    // This is the key insight: when signal was good and suddenly drops, it's often
    // because the model is being disconnected at the end of a flight
    if (PreviousConnectionQuality >= SIGNAL_QUALITY_GOOD && currentSuccessRate <= SIGNAL_QUALITY_WARNING) {
        // First time detecting a sudden drop
        if (QualityDropStartTime == 0) {
            QualityDropStartTime = millis();
            
            // Immediately suppress warnings
            SignalQualityWarningActive = false;
            InSuddenDisconnect = true;
            
            // Force UI cleanup
            if (CurrentView == FRONTVIEW) {
                char WarnOff[] = "vis Warning,0";
                char FrontView_Connected[] = "Connected";
                SendCommand(WarnOff);
                SendText(FrontView_Connected, FrontView_Connected);
            }
        }
        
        // Continue suppressing warnings during delay period
        if (millis() - QualityDropStartTime < SIGNAL_DEGRADATION_DELAY) {
            // Keep warnings suppressed during delay period
            InSuddenDisconnect = true;
            SignalQualityWarningActive = false;
            
            // Periodically update UI to ensure warnings stay suppressed
            if (millis() - lastForcedClearTime >= 250) {
                lastForcedClearTime = millis();
                
                if (CurrentView == FRONTVIEW) {
                    char WarnOff[] = "vis Warning,0";
                    char FrontView_Connected[] = "Connected";
                    SendCommand(WarnOff);
                    SendText(FrontView_Connected, FrontView_Connected);
                }
            }
            
            // Save current quality for next comparison
            PreviousConnectionQuality = currentSuccessRate;
            return;
        }
    }
    // If signal returns to good quality, reset the drop timer
    else if (currentSuccessRate >= SIGNAL_QUALITY_GOOD) {
        QualityDropStartTime = 0;
        InSuddenDisconnect = false;
    }
    
    // Handle the case of complete or very severe disconnection (receiver power off)
    // This is a secondary protection for very low quality
    if (LedWasGreen && currentSuccessRate <= 30) {  // Increased threshold from 5 to 30
        // This is very likely a receiver power-off or end-of-flight disconnection
        InSuddenDisconnect = true;
        
        // Force all warnings off immediately
        SignalQualityWarningActive = false;
        LedIsBlinking = false;
        PreviousConnectionQualityState = 0; // Reset to good state
        
        // Restore audio volume if it was increased for warning
        if (UsingHighVolumeForWarning) {
            SetAudioVolume(SavedAudioVolume);
            UsingHighVolumeForWarning = false;
        }
        
        // Hide warnings on display - only visual aspects depend on current view
        if (CurrentView == FRONTVIEW) {
            char WarnOff[] = "vis Warning,0";
            char FrontView_Connected[] = "Connected";
            SendCommand(WarnOff);
            SendText(FrontView_Connected, FrontView_Connected);
        }
        
        // Save current quality for next comparison
        PreviousConnectionQuality = currentSuccessRate;
        return;
    }
    
    // Save current quality for next comparison
    PreviousConnectionQuality = currentSuccessRate;
    
    if (millis() - lastForcedClearTime >= 500) { // Every 500ms
        lastForcedClearTime = millis();
        
        if (CurrentView == FRONTVIEW) {
            char WarnOff[] = "vis Warning,0";
            char FrontView_Connected[] = "Connected";
            char SignalWarningCritical[] = "!!! CRITICAL SIGNAL !!!";
            char SignalWarning[] = "** WEAK SIGNAL **";
            SendCommand(WarnOff);
            
            // If signal quality warning should be active, display the warning text
            if (SignalQualityWarningActive) {
                if (PreviousConnectionQualityState == 2) {
                    SendText(FrontView_Connected, SignalWarningCritical);
                } else {
                    SendText(FrontView_Connected, SignalWarning);
                }
            } 
            // If no signal quality warning is active, make sure warning is off
            else if (!LedIsBlinking) {
                SendCommand(WarnOff);
            }
        }
    }
    
    // Check for intentional disconnection (power off button pressed)
    if (!digitalRead(BUTTON_SENSE_PIN)) {
        // Power button is pressed - this is likely an intentional disconnect
        // Clear any active warnings immediately
        if (SignalQualityWarningActive) {
            // Always turn off warning state and LED blinking regardless of current view
            SignalQualityWarningActive = false;
            LedIsBlinking = false;
            
            // Restore audio volume regardless of current view
            if (UsingHighVolumeForWarning) {
                SetAudioVolume(SavedAudioVolume);
                UsingHighVolumeForWarning = false;
            }
            
            // Update visual elements only when on FRONTVIEW
            if (CurrentView == FRONTVIEW) {
                char WarnOff[] = "vis Warning,0";
                char FrontView_Connected[] = "Connected";
                SendCommand(WarnOff);
                SendText(FrontView_Connected, FrontView_Connected);
            }
        }
        
        return; // Skip further checks when intentionally disconnecting
    }
    
    // Only check if we're connected and model matched
    if (!LedWasGreen || !BoundFlag || !ModelMatched) {
        // If warning was active but connection is lost, explicitly clear it
        if (SignalQualityWarningActive) {
            // Always turn off warnings and LED blinking regardless of current view
            SignalQualityWarningActive = false;
            LedIsBlinking = false;
            lastDisplayedState = 4; // Reset to invalid state
            
            // Restore audio volume regardless of current view
            if (UsingHighVolumeForWarning) {
                SetAudioVolume(SavedAudioVolume);
                UsingHighVolumeForWarning = false;
            }
            
            // Update visual elements only when on FRONTVIEW
            if (CurrentView == FRONTVIEW) {
                char WarnOff[] = "vis Warning,0";
                char FrontView_Connected[] = "Connected";
                SendCommand(WarnOff);
                SendText(FrontView_Connected, FrontView_Connected);
            }
        }
        
        // Reset connection established time and quality drop timer when disconnected
        connectionEstablishedTime = 0;
        QualityDropStartTime = 0;
        return;
    }
    
    // Don't show warnings during initial connection period either
    // Track when the connection was established
    if (connectionEstablishedTime == 0) {
        connectionEstablishedTime = millis();
        // Reset quality drop timer on new connection
        QualityDropStartTime = 0;
    }
    
    // Wait for initial connection stability before showing warnings
    const uint32_t CONNECTION_STABILITY_DELAY = 5000; // 5 seconds
    if (millis() - connectionEstablishedTime < CONNECTION_STABILITY_DELAY) {
        // Don't show warnings during initial connection period
        return;
    }
    
    // Calculate current quality state (good, warning, critical)
    uint8_t currentState = 0;  // 0=good, 1=warning, 2=critical
    
    if (currentSuccessRate <= SIGNAL_QUALITY_CRITICAL) {
        currentState = 2; // Critical signal quality
    } else if (currentSuccessRate <= SIGNAL_QUALITY_WARNING) {
        currentState = 1; // Warning level signal quality
    } else {
        currentState = 0; // Good signal quality
    }
    
    // Update the global state
    PreviousConnectionQualityState = currentState;
    
    // Don't show warnings if we're in a sudden disconnect scenario
    if (InSuddenDisconnect) {
        return;
    }
    
    char WarnOff[] = "vis Warning,0";
    char FrontView_Connected[] = "Connected";
    char SignalWarningCritical[] = "!!! CRITICAL SIGNAL !!!";
    char SignalWarning[] = "** WEAK SIGNAL **";
    char SignalGood[] = "CONNECTION GOOD";
    
    // Only update UI when:
    // 1. State changes
    // 2. State is warning/critical and periodic update is needed
    bool stateChanged = (currentState != lastDisplayedState);
    bool periodicUpdateNeeded = (currentState > 0) && 
                               (millis() - LastQualityWarningTime >= WARNING_NOTIFICATION_INTERVAL);
    
    if (stateChanged || periodicUpdateNeeded) {
        // Increase volume for warnings - do this regardless of current view
        if ((currentState > 0) && !UsingHighVolumeForWarning) {
            // Only save the volume once when transitioning from good to warning state
            SavedAudioVolume = AudioVolume;
            SetAudioVolume(90); // Set to 90% volume for warnings
            UsingHighVolumeForWarning = true;
        } 
        // Restore audio volume when returning to good state
        else if ((currentState == 0) && UsingHighVolumeForWarning) {
            SetAudioVolume(SavedAudioVolume);
            UsingHighVolumeForWarning = false;
        }
        
        // Play sound based on state transition - do this regardless of current view
        if (stateChanged) {
            if (currentState == 0) {
                // Only announce "CONNECTION GOOD" if we're recovering from a bad state
                if (lastDisplayedState > 0 && lastDisplayedState != 4) {
                    PlaySound(CONNECTION_GOOD);
                }
            } else if (currentState == 1) {
                PlaySound(CONNECTION_WARNING);
                WarningSound = CONNECTION_WARNING;
            } else {
                PlaySound(CONNECTION_CRITICAL);
                WarningSound = CONNECTION_CRITICAL;
            }
        } 
        // For periodic updates of warning/critical states - do this regardless of current view
        else if (periodicUpdateNeeded) {
            if (currentState == 2) {
                PlaySound(CONNECTION_CRITICAL);
                WarningSound = CONNECTION_CRITICAL;
            } else if (currentState == 1) {
                PlaySound(CONNECTION_WARNING);
                WarningSound = CONNECTION_WARNING;
            }
        }
        
        // Set LED blinking state regardless of current view
        if (currentState == 0) {
            LedIsBlinking = false;
        } else {
            LedIsBlinking = true;
        }
        
        // Signal quality warning state tracking happens regardless of current view
        SignalQualityWarningActive = (currentState > 0);
        
        // Update visual UI only when on FRONTVIEW
        if (CurrentView == FRONTVIEW) {
            // Always update the UI when state changes or periodic update is needed
            if (currentState == 0) {
                // Only show "CONNECTION GOOD" message when transitioning from a bad state
                if (stateChanged && lastDisplayedState > 0 && lastDisplayedState != 4) {
                    SendCommand(WarnOff);
                    SendText(FrontView_Connected, SignalGood);
                } else if (stateChanged) {
                    // Just reset the state without showing a message
                    SendCommand(WarnOff);
                    SendText(FrontView_Connected, FrontView_Connected);
                }
            } else {
                // For warning/critical, always update UI text but HIDE the Warning label
                SendCommand(WarnOff); // NUCLEAR FIX: Always hide the Warning label even for warnings
                SendText(FrontView_Connected, (currentState == 2) ? SignalWarningCritical : SignalWarning);
            }
        }
        
        // Update timers and state tracking
        LastQualityWarningTime = millis();
        lastDisplayedState = currentState;
    }
}

/**
 * @brief Legacy function kept for compatibility but not actively used
 * 
 * This function is now just a wrapper for direct UI updates in CheckSignalQuality() 
 * 
 * @param show True to show the warning, false to hide it
 * 
 * ADDED BY CLAUDE 3.7 CODE FEB 28 2025: Warning display for degrading signal quality
 * FIXED BY CLAUDE 3.7 CODE MAR 2 2025: Completely refactored to a direct UI approach
 */
void ShowSignalQualityWarning(bool show) {
    // This function is now a legacy stub - actual UI updates are done directly in CheckSignalQuality
    // This change ensures that no other code can interfere with our warning display
    
    // Only track the flag state for compatibility with other code
    SignalQualityWarningActive = show;
}

/**
 * @brief Nuclear option to force warning display - now integrated with the periodic system
 * 
 * This function supports compatibility with existing code. It no longer directly updates
 * the UI but ensures SignalQualityWarningActive is set so the periodic checker will
 * pick it up.
 * 
 * ADDED BY CLAUDE 3.7 CODE MAR 2 2025: Legacy wrapper now that we use direct UI updates
 * FIXED BY CLAUDE 3.7 CODE MAR 2 2025: Updated to work with periodic update system
 */
void ForceCriticalWarningSystem() {
    // Mark warning as active, ensuring that the periodic checker will display it properly
    SignalQualityWarningActive = true;
    
    // The actual update will happen on the next cycle of the periodic checker in CheckSignalQuality
}


// ********************************************************************************************************************************

// Because data are only displayed when different, this sets all to zero so that they will be displayed on the first pass.
void ForceDataRedisplay(){
    LastShowTime         = 0;//
    LastPacketsPerSecond = 0;
    LastLostPackets      = 0;
    LastGapLongest       = 0;
    LastRadioSwaps       = 0;
    LastRX1TotalTime     = 0;
    LastRX2TotalTime     = 0;
    LastGapAverage       = 0;
    LastSbusRepeats      = 0;
    LastRXModelAltitude  = 0;
    LastRXModelMaxAltitude = 0;
    LastRXTemperature    = 0;
    LastRadioNumber      = 0;
    ForceVoltDisplay     = true; 
    for (int i = 0; i < 5; ++i) {
       for (int j = 0; j < 17; ++j) {
           LastTrim[i][j] = 0;
       }
    }
    return;
}

// ********************************************************************************************************************************

void SaveOrRestoreScreen(bool Restore){

static uint8_t LastScreen = 0;

    if (!Restore){
        LastScreen = CurrentView;                                                                             // Saved the ID of the screen we are leaving
        return; 
    }
    else 
    {
    if (LastScreen == DATAVIEW)        {ForceDataRedisplay();SendCommand(pDataView); CurrentView = DATAVIEW;       return;}
    if (LastScreen == SUBTRIMVIEW)     {SendCommand(pSubTrimView);                   CurrentView = SUBTRIMVIEW;    return;}
    if (LastScreen == TXSETUPVIEW)     {SendCommand(pTXSetupView);                     CurrentView = TXSETUPVIEW;    return;}
    if (LastScreen == BUDDYVIEW)       {SendCommand(pBuddyView);                     CurrentView = BUDDYVIEW;      return;} 
    if (LastScreen == BUDDYCHVIEW)     {SendCommand(pBuddyChView);                   CurrentView = BUDDYCHVIEW;    return;} 
    if (LastScreen == OPTIONVIEW2)     {SendCommand(pOptionView2);                   CurrentView = OPTIONVIEW2;    return;}

    // if (LastScreen == SCANVIEW)        {SendCommand(pFhssView);DrawFhssBox();     CurrentView = SCANVIEW;       return;}     
    // if (LastScreen == GRAPHVIEW)       {SendCommand(pGraphView);                  CurrentView = GRAPHVIEW;      return;}     
    // if (LastScreen == STICKSVIEW)      {SendCommand(pSticksView);                 CurrentView = STICKSVIEW;     return;}     
    // if (LastScreen == COLOURS_VIEW)    {SendCommand(pColoursView);                CurrentView = COLOURS_VIEW;   return;}   
    // if (LastScreen == AUDIOVIEW)       {SendCommand(pAudioView);                  CurrentView = AUDIOVIEW;      return;}    
    // if (LastScreen == HELP_VIEW)       {SendCommand(pHelpView);                   CurrentView = HELP_VIEW;      return;}    
    // if (LastScreen == PONGVIEW)        {SendCommand(pPongView);                   CurrentView = PONGVIEW;       return;}
    // if (LastScreen == CALIBRATEVIEW)   {SendCommand(pCalibrateView);              CurrentView = CALIBRATEVIEW;  return;}
   
    if (LastScreen == FRONTVIEW)       {GotoFrontView();                             CurrentView = FRONTVIEW;      return;}
    if (LastScreen == SWITCHES_VIEW)   {SendCommand(pSwitchesView);                  CurrentView = SWITCHES_VIEW;  return;}
    if (LastScreen == INPUTS_VIEW)     {SendCommand(pInputsView);                    CurrentView = OPTIONS_VIEW;   return;}
    if (LastScreen == OPTIONS_VIEW)    {SendCommand(pOptionsViewS);                  CurrentView = OPTIONS_VIEW;   return;}
    if (LastScreen == MIXESVIEW)       {SendCommand(pMixesView);                     CurrentView = MIXESVIEW;      return;}
    if (LastScreen == TYPEVIEW)        {SendCommand(pTypeView);                      CurrentView = TYPEVIEW;       return;}
    if (LastScreen == FAILSAFE_VIEW)   {SendCommand(pFailSafe);                      CurrentView = FAILSAFE_VIEW;  return;}
    if (LastScreen == MODELSVIEW)      {SendCommand(pModelsView);                    CurrentView = MODELSVIEW;     return;}
    if (LastScreen == COLOURS_VIEW)    {SendCommand(pColoursView);                   CurrentView = COLOURS_VIEW;   return;}
    if (LastScreen == RXSETUPVIEW)     {SendCommand(pRXSetupView);                   CurrentView = RXSETUPVIEW;    return;} // ... might add more later. Default is front view
    }
   GotoFrontView();                                                                                          // Default is front view
}

/*********************************************************************************************************************************/
void RestoreBrightness()
{
    char cmd[20];
    char dim[] = "dim=";
    char nb[10];
    if (Brightness < 10) Brightness = 10;
    strcpy(cmd, dim);
    Str(nb, Brightness, 0);
    strcat(cmd, nb);
    ScreenIsOff     = false;
    SendCommand(cmd);
    ScreenTimeTimer = millis(); // reset screen counter
}

/******************************************************************************************************************************/
void ShowScreenAgain(){
        RestoreBrightness();
        SaveOrRestoreScreen(true);
        ScreenIsOff  = false;
}

/******************************************************************************************************************************/
void HideScreenAgain(){
    char ScreenOff[]    = "page BlankView";
    char NoBrightness[] = "dim=0";


    if (CurrentView == SCANVIEW)        return; // recovery fails
    if (CurrentView == GRAPHVIEW)       return;
    if (CurrentView == COLOURS_VIEW)    return;
    if (CurrentView == AUDIOVIEW)       return;
    if (CurrentView == HELP_VIEW)       return;
    if (CurrentView == PONGVIEW)        return;
    if (CurrentView == CALIBRATEVIEW)   return;


    SaveOrRestoreScreen(false);
    SendCommand(ScreenOff);     // move to blank screen
    DelayWithDog(10);           // wait a moment for screen to change
    SendCommand(NoBrightness);  // turn off backlight
    ScreenIsOff     = true;
    CurrentView     = BLANKVIEW;
}

/*********************************************************************************************************************************/
void CheckScreenTime()          // turn off screen after a timeout
{
    if (((millis() - ScreenTimeTimer) > ScreenTimeout * 1000) && (ScreenIsOff == false)) HideScreenAgain();  
}

// ********************************************************************************************************************************

void CheckForNextionButtonPress()
{
    if (GetButtonPress()) ButtonWasPressed();
}

// **********************************************************************************************************************************

uint8_t Ascii(char c)
{
    return (uint8_t)c;
}

// **************************************************************** Play a sound from RAM *********************************************
void PlaySound(uint16_t TheSound)
{ // Plays a sound identified by a number

    // static uint32_t SoundTimer = millis();
    // static uint16_t LastSound   = 0;
    // uint32_t m = millis(); // one call to millis() is enough
    // if (CurrentView != PONGVIEW){
    //     if (((m - SoundTimer) < 1000) && (m > 5000)  && (TheSound == LastSound)) return; // prevent sound repeats unless in pongview
    // }
    // SoundTimer = millis();
    // LastSound = TheSound;
    
    char Sound[20];
    char SoundPrefix[]  = "play 0,";
    char SoundPostfix[] = "0";
    char NB[6];
    if (CurrentView == MODELSVIEW) {
        if (TheSound != CLICKONE) return;
    }
    Str(NB, TheSound, 1);
    strcpy(Sound, SoundPrefix);
    strcat(Sound, NB);
    strcat(Sound, SoundPostfix);
    SendCommand(Sound);
}
/*********************************************************************************************************************************/

// This function converts an int to a char[] array, then adds a comma, a dot, or nothing at the end.
// It builds the char[] array at a pointer (*s) Where there MUST be enough space for all characters plus a zero terminator.
// It dates for a very early time when I didn't know about standard library functions!
// But it works just fine, so it says in.

FASTRUN char* Str(char* s, int n, int comma) // comma = 0 for nothing, 1 for a comma, 2 for a dot.
{
    int  r, i, m, flag;
    char cma[] = ",";
    char dot[] = ".";

    flag = 0;
    i    = 0;
    m    = 1000000000;
    if (n < 0) {
        s[0] = '-';
        i    = 1;
        n    = -n;
    }
    if (n == 0) {
        s[0] = 48;
        s[1] = 0;

        if (comma == 1) {
            strcat(s, cma);
        }
        if (comma == 2) {
            strcat(s, dot);
        }
        return s;
    }
    while (m >= 1) {
        r = n / m;
        if (r > 0) {
            flag = 1;
        } //  first digit
        if (flag == 1) {
            s[i] = 48 + r;
            ++i;
            s[i] = 0;
        }
        n -= (r * m);
        m /= 10;
    }
    if (comma == 1) {
        strcat(s, cma);
    }
    if (comma == 2) {
        strcat(s, dot);
    }
    return s;
}

/*********************************************************************************************************************************/

void SetAudioVolume(uint16_t v)
{ // sets audio volume v (0-100)
    char vol[] = "volume=";
    char cmd[20];
    char nb[6];
    
    // Update the global volume variable with range check
    AudioVolume = constrain(v, 0, 100);
    
    strcpy(cmd, vol);
    Str(nb, AudioVolume, 0);
    strcat(cmd, nb);
    SendCommand(cmd);
}

/*********************************************************************************************************************************/

void Reboot()
{
    while (true) {
        TeensyWatchDog.feed();
    }
}

/*********************************************************************************************************************************/
void KickTheDog()
{
    static uint32_t LastDogKick = 0;
    if (millis() - LastDogKick >= KICKRATE) {
        TeensyWatchDog.feed();
        LastDogKick = millis();
    }
}
// **********************************************************************************************************************************

void ConfigureStickMode()
{ // This sets stick mode without moving any wires. Must be wired as for Mode 1

    if (SticksMode == 1) {
        AnalogueInput[0] = A0;
        AnalogueInput[1] = A1;
        AnalogueInput[2] = A2;
        AnalogueInput[3] = A3;
        AnalogueInput[4] = A6;
        AnalogueInput[5] = A7;
        AnalogueInput[6] = A8;
        AnalogueInput[7] = A9;
    }

    if (SticksMode == 2) {
        AnalogueInput[0] = A0;
        AnalogueInput[1] = A2;
        AnalogueInput[2] = A1;
        AnalogueInput[3] = A3;
        AnalogueInput[4] = A6;
        AnalogueInput[5] = A7;
        AnalogueInput[6] = A8;
        AnalogueInput[7] = A9;
    }
}

/******************* DeltaGMT is a user defined representation of time zone. It should never exceed 24. Not on this planet. **********/
void FixDeltaGMTSign()
{
    if (DeltaGMT < -24) DeltaGMT = 0; // Undefined value?f
    if (DeltaGMT > 24) {              // This fixes the sign bit if negative !!!! (There's surely a better way !!!)
        DeltaGMT ^= 0xffff;           // toggle every bit! :-)
        ++DeltaGMT;                   // Add one
        DeltaGMT = -DeltaGMT;         // it's definately meant to be negative!
    }
}
/*********************************************************************************************************************************/

uint8_t decToBcd(uint8_t val)
{
    return ((val / 10 * 16) + (val % 10));
}

/*********************************************************************************************************************************/

uint8_t bcdToDec(uint8_t val)
{
    return ((val / 16 * 10) + (val % 16));
}

/*********************************************************************************************************************************/

FLASHMEM void SetTheRTC()
{
    uint8_t zero = 0x00;
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(zero); // Stop the oscillator
    Wire.write(decToBcd(Gsecond));
    Wire.write(decToBcd(Gminute));
    Wire.write(decToBcd(Ghour));
    Wire.write(decToBcd(GweekDay));
    Wire.write(decToBcd(GmonthDay));
    Wire.write(decToBcd(Gmonth));
    Wire.write(decToBcd(Gyear));
    Wire.write(zero); //  Re-start it
    Wire.endTransmission();
}
/*********************************************************************************************************************************/
void SynchRTCwithGPSTime()
{ // This function corrects the time and the date.
    if (!GPSTimeSynched) {
        GPSTimeSynched = true;
        Gsecond        = GPS_RX_SECS;
        Gminute        = GPS_RX_Mins;
        Ghour          = GPS_RX_Hours;
        GmonthDay      = GPS_RX_DAY;
        Gmonth         = GPS_RX_MONTH;
        Gyear          = GPS_RX_YEAR;
        SetTheRTC();
        GPSTimeSynched = true;
    }
}
/*********************************************************************************************************************************/
void AdjustDateTime(uint8_t MinChange, uint8_t HourChange, uint8_t YearChange, uint8_t MonthChange, uint8_t DateChange)
{
    ReadTheRTC();
    Gminute += MinChange;
    if (Gminute > 59) {
        Gminute = 0;
        if (Ghour < 23) {
            ++Ghour;
        }
    }
    if (Gminute < 1) {
        Gminute = 0;
        if (Ghour > 0) {
            --Ghour;
        }
    }
    Ghour += HourChange;
    if (Ghour < 0) Ghour = 0;
    if (Ghour > 23) Ghour = 23;
    Gyear += YearChange;
    if (Gyear < 0) Gyear = 0;
    if (Gyear > 99) Gyear = 99;
    Gmonth += MonthChange;
    if (Gmonth < 1) Gmonth = 1;
    if (Gmonth > 12) Gmonth = 12;
    GmonthDay += DateChange;
    if (GmonthDay < 1) GmonthDay = 1;
    if (GmonthDay > 31) GmonthDay = 31;
    SetTheRTC();
}

/*********************************************************************************************************************************/
void ReadTheRTC()
{
    uint8_t second   = tm.Second; // 0-59
    uint8_t minute   = tm.Minute; // 0-59
    uint8_t hour     = tm.Hour;   // 0-23
    uint8_t weekDay  = tm.Wday;   // 1-7
    uint8_t monthDay = tm.Day;    // 1-31
    uint8_t month    = tm.Month;  // 1-12
    uint8_t year     = tm.Year;   // 0-99
    Gsecond          = second;
    Gminute          = minute;
    Ghour            = hour;
    GweekDay         = weekDay;
    GmonthDay        = monthDay;
    Gmonth           = month;
    Gyear            = year - 30; // ???
}
/*********************************************************************************************************************************/

void IncMinute()
{
    uint8_t zero = 0x00;
    uint8_t c    = 1;
    if (RTC.read(tm)) {
        AdjustDateTime(c, zero, zero, zero, zero);
    }
}

/*********************************************************************************************************************************/

void DecMinute()
{
    uint8_t zero = 0x00;
    uint8_t c    = -1;
    if (RTC.read(tm)) {
        AdjustDateTime(c, zero, zero, zero, zero);
    }
}

/*********************************************************************************************************************************/

void IncHour()
{
    uint8_t c    = 1;
    uint8_t zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, c, zero, zero, zero);
    }
}

/*********************************************************************************************************************************/

void DecHour()
{
    uint8_t c    = -1;
    uint8_t zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, c, zero, zero, zero);
    }
}

/*********************************************************************************************************************************/

void IncYear()
{
    uint8_t c    = 1;
    uint8_t zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, c, zero, zero);
    }
}

/*********************************************************************************************************************************/

void DecYear()
{
    uint8_t c    = -1;
    uint8_t zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, c, zero, zero);
    }
}

/*********************************************************************************************************************************/

void IncMonth()
{
    uint8_t c    = 1;
    uint8_t zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, zero, c, zero);
    }
}

/*********************************************************************************************************************************/

void DecMonth()
{
    uint8_t c    = -1;
    uint8_t zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, zero, c, zero);
    }
}

/*********************************************************************************************************************************/

void IncDate()
{
    uint8_t c    = 1;
    uint8_t zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, zero, zero, c);
    }
}

/*********************************************************************************************************************************/

void DecDate()
{
    uint8_t c    = -1;
    uint8_t zero = 0x00;
    if (RTC.read(tm)) {
        AdjustDateTime(zero, zero, zero, zero, c);
    }
}

/*********************************************************************************************************************************/

bool getTime(const char* str)
{
    int Hour, Min, Sec;
    if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
    tm.Hour   = Hour;
    tm.Minute = Min;
    tm.Second = Sec;
    return true;
}

/*********************************************************************************************************************************/

bool getDate(const char* str)
{
    char        Month[12];
    int         Day, Year;
    uint8_t     monthIndex;
    const char* monthName[12] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
    for (monthIndex = 0; monthIndex < 12; ++monthIndex) {
        if (strcmp(Month, monthName[monthIndex]) == 0) break;
    }
    if (monthIndex >= 12) return false;
    tm.Day   = Day;
    tm.Month = monthIndex + 1;
    tm.Year  = CalendarYrToTm(Year);
    return true;
}

/*********************************************************************************************************************************/

bool MayBeAddZero(uint8_t nn)
{
    if (nn >= 0 && nn < 10) {
        return true;
    }
    return false;
}

/*********************************************************************************************************************************/

void ReadTime()
{

    static char month[12][15]     = {"January", "February", "March", "April", "May", "June", "July", "August", "Sept", "October", "November", "December"};
    static char ShortMonth[12][7] = {"Jan. ", "Feb. ", "Mar. ", "Apr. ", "May  ", "June ", "July ", "Aug. ", "Sept ", "Oct. ", "Nov. ", "Dec. "};

    char NB[10];
    char TimeString[80];
    char Space[]    = " ";
    char colon[]    = ":";
    char zero[]     = "0";
    char DateTime[] = "DateTime";

    uint8_t DisplayedHour;
    FixDeltaGMTSign();
    if (CurrentView == FRONTVIEW || CurrentView == OPTIONVIEW2) {
        if (RTC.read(tm)) {
            strcpy(TimeString, Str(NB, tm.Day + DateFix, 0));
            if (CurrentView == OPTIONVIEW2)
            {
                if ((tm.Day) < 10) {
                    strcat(TimeString, Space); // to align better the rest of the data
                    strcat(TimeString, Space);
                }
            }
            strcat(TimeString, Space);
            if (CurrentView == OPTIONVIEW2)
            {
                strcat(TimeString, ShortMonth[tm.Month - 1]);
            }
            else
            {
                strcat(TimeString, month[tm.Month - 1]);
            }
            strcat(TimeString, Space);
            strcat(TimeString, (Str(NB, tmYearToCalendar(tm.Year), 0)));
            strcat(TimeString, Space);
            DisplayedHour = tm.Hour + DeltaGMT;
            DateFix       = 0;
            if (DisplayedHour > 24) {
                DisplayedHour -= 24;
                DateFix = 1;
            }
            if (DisplayedHour < 0) {
                DisplayedHour += 24;
                DateFix = -1;
            }
            if (MayBeAddZero(DisplayedHour)) strcat(TimeString, zero);
            strcat(TimeString, Str(NB, DisplayedHour, 0));
            strcat(TimeString, colon);
            if (MayBeAddZero(tm.Minute)) strcat(TimeString, zero);
            strcat(TimeString, Str(NB, tm.Minute, 0));
            strcat(TimeString, colon);
            if (MayBeAddZero(tm.Second)) strcat(TimeString, zero);
            strcat(TimeString, Str(NB, tm.Second, 0));
            SendText(DateTime, TimeString);
        }
    }
}
/************************************************************************************************************/
FLASHMEM void ResetSubTrims()
{
    for (int i = 0; i < 16; ++i) {
        SubTrims[i] = 127;
    }
}
/**************************** Clear Macros if junk was loaded from SD ********************************************************************************/
void CheckMacrosBuffer()
{

    bool junk = false;
    for (uint8_t i = 0; i < MAXMACROS; ++i) {
        if (MacrosBuffer[i][MACROTRIGGERCHANNEL] > 16) junk = true;
        if (MacrosBuffer[i][MACROMOVECHANNEL] > 16) junk = true;
        if (MacrosBuffer[i][MACROMOVETOPOSITION] > 180) junk = true;
        if (MacrosBuffer[i][MACROTRIGGERCHANNEL] > 0) UseMacros = true;
    }
    if (junk == false) return;
    UseMacros = false;
    for (uint8_t j = 0; j < BYTESPERMACRO; ++j) {
        for (uint8_t i = 0; i < MAXMACROS; ++i) {
            MacrosBuffer[i][j] = 0;
        }
    }
}

/*********************************************************************************************************************************/

void StartInactvityTimeout()
{
    Inactivity_Start = millis();
}

/*********************************************************************************************************************************/

uint8_t GetLEDBrightness()
{
    static uint8_t BlinkOnPhase = 1;

    if (LEDBrightness < 15) {
        LEDBrightness = DEFAULTLEDBRIGHTNESS;
        SaveTransmitterParameters();
    }

    if (LedIsBlinking) {
        if ((millis() - BlinkTimer) > (1000 / BlinkHertz)) {
            BlinkOnPhase ^= 1;
            BlinkTimer = millis();
        }
    }
    else {
        BlinkOnPhase = 1;
    }
    if (BlinkOnPhase) {
        if (LedIsBlinking) return 255;
        return LEDBrightness; // 0 - 254 (= brightness)
    }
    else {
        return 0;
    }
}

/*********************************************************************************************************************************/
uint8_t IntoDegrees(uint16_t HiRes) // convert to lower resolution for screen display
{
    return (map(HiRes, MINMICROS, MAXMICROS, 0, 180));
}
/*********************************************************************************************************************************/
uint8_t IntoLowerRes(uint16_t HiRes) // convert to lower resolution for screen display
{
    return (map(HiRes, MINMICROS, MAXMICROS, 0, 100));
}

/*********************************************************************************************************************************/
uint16_t IntoHigherRes(uint8_t LowRes) // This returns the main curve-points at the higher resolution for Servo output
{
    return map(LowRes, 0, 180, MINMICROS, MAXMICROS);
}
/*********************************************************************************************************************************/
void ClearText()
{
    for (int i = 0; i < CHARSMAX; ++i) {
        TextIn[i] = 0;
    }
}


/*********************************************************************************************************************************/

/** Send 13 joined together char arrays to NEXTION */
void SendCharArray(char* ch0, char* ch1, char* ch2, char* ch3, char* ch4, char* ch5, char* ch6, char* ch7, char* ch8, char* ch9, char* ch10, char* ch11, char* ch12)
{
    strcpy(ch0, ch1);
    strcat(ch0, ch2);
    strcat(ch0, ch3);
    strcat(ch0, ch4);
    strcat(ch0, ch5);
    strcat(ch0, ch6);
    strcat(ch0, ch7);
    strcat(ch0, ch8);
    strcat(ch0, ch9);
    strcat(ch0, ch10);
    strcat(ch0, ch11);
    strcat(ch0, ch12);
    SendCommand(ch0);
}

/*********************************************************************************************************************************/

int GetNextNumber(int p1, char text1[CHARSMAX])
{
    char text2[CHARSMAX];
    int  j = 0;
    int  i = p1 - 1;
    while (isDigit(text1[i]) && i < CHARSMAX) {
        text2[j] = text1[i];
        ++i;
        ++j;
        text2[j] = 0;
    }
    i = j; // = strlen only simpler
    if (i == 3) {
        j = (text2[0] - 48) * 100;
        j += (text2[1] - 48) * 10;
        j += (text2[2] - 48);
    }
    if (i == 2) {
        j = (text2[0] - 48) * 10;
        j += (text2[1] - 48);
    }
    if (i == 1) j = (text2[0] - 48);
    return j;
}

/*********************************************************************************************************************************/

/**
 * @brief Resets connection quality monitoring
 * 
 * This function resets all connection quality monitoring variables to their initial state.
 * It's called when a new connection is established or when the system wants to clear
 * any existing warning state.
 * 
 * IMPROVED BY CLAUDE 3.7 CODE MAR 4 2025: Added reset for warnings on disconnect
 */
void ClearSuccessRate()
{
    for (int i = 0; i < (PERFECTPACKETSPERSECOND * (uint16_t)ConnectionAssessSeconds); ++i) { // 126 packets per second start off good
        PacketsHistoryBuffer[i] = 1;
    }
    // Update our signal quality monitoring variables - ADDED BY CLAUDE 3.7 CODE FEB 28 2025
    // MODIFIED BY CLAUDE 3.7 CODE MAR 3 2025: Also reset connection established time
    ConnectionQualityPercent = 100;
    SignalQualityWarningActive = false;
    LastQualityWarningTime = 0;
    PreviousConnectionQualityState = 0; // Reset to "good" state
    
    // Reset the connection established time to ensure the 5-second delay is respected
    connectionEstablishedTime = 0; 
    
    // Since we can't access the static variable in CheckSignalQuality,
    // we'll ensure our state is consistently initialized
    // ADDED BY CLAUDE 3.7 CODE MAR 3 2025: Prevent unnecessary "connection good" announcements
    PreviousConnectionQualityState = 0; // Set to good state to avoid changing state
    
    // Reset drop timer to handle new disconnections properly
    QualityDropStartTime = 0;
    
    // Reset previous connection quality to ensure fresh comparisons
    PreviousConnectionQuality = 100;
    
    // Ensure sudden disconnect flag is cleared
    InSuddenDisconnect = false;
    
    // Restore audio volume if it was changed for warning
    if (UsingHighVolumeForWarning) {
        SetAudioVolume(SavedAudioVolume);
        UsingHighVolumeForWarning = false;
    }
    
    // Clear visual warning when on FRONTVIEW
    if (CurrentView == FRONTVIEW) {
        char WarnOff[] = "vis Warning,0";
        char FrontView_Connected[] = "Connected";
        SendCommand(WarnOff);
        SendText(FrontView_Connected, FrontView_Connected);
    }
}

/*********************************************************************************************************************************/

FASTRUN void DrawDot(int xx, int yy, int rad, int colr)
{
    char cirs[] = "cirs ";
    char nb[12];
    char cb[60];
    char comma[] = ",";
    strcpy(cb, cirs);
    strcat(cb, Str(nb, xx, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, yy, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, rad, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, colr, 0));
    SendCommand(cb);
}

/*********************************************************************************************************************************/

/**
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param color
 */
FASTRUN void DrawBox(int x1, int y1, int x2, int y2, int c)
{
    char line[] = "draw ";
    char nb[12];
    char cb[60];
    char comma[] = ",";
    strcpy(cb, line);
    strcat(cb, Str(nb, x1, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, y1, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, x2, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, y2, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, c, 0));
    SendCommand(cb);
}

/*********************************************************************************************************************************/
/**
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param color
 */
void FillBox(int x1, int y1, int w, int h, int c)
{
    char line[] = "fill ";
    char nb[12];
    char cb[60];
    char comma[] = ",";
    strcpy(cb, line);
    strcat(cb, Str(nb, x1, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, y1, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, w, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, h, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, c, 0));
    SendCommand(cb);
}

/*********************************************************************************************************************************/

/**
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param color
 */
void DrawLine(int x1, int y1, int x2, int y2, int c)
{
    char line[] = "line ";
    char nb[12];
    char cb[60];
    char comma[] = ",";
    strcpy(cb, line);
    strcat(cb, Str(nb, x1, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, y1, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, x2, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, y2, 0));
    strcat(cb, comma);
    strcat(cb, Str(nb, c, 0));
    SendCommand(cb);
}

/*********************************************************************************************************************************/

int DegsToPercent(int degs)
{
    return map(degs, 0, 180, -100, 100);
}

/*********************************************************************************************************************************/

void ClearBox()
{
    char nb[10];
    char cmd[80];
    // char fillcmd[] = "fill 30,30,380,365,";
    char fillcmd[] = "fill 20,20,388,375,";
    strcpy(cmd, fillcmd);
    Str(nb, BackGroundColour, 0);
    strcat(cmd, nb);
    SendCommand(cmd);
}

/*********************************************************************************************************************************/
/**
 * @brief Returns the current connection quality percentage
 * 
 * This function uses the packet history buffer to calculate connection quality
 * and applies a special correction to account for packets that likely made it 
 * through but weren't acknowledged.
 * 
 * MODIFIED BY CLAUDE 3.7 CODE FEB 28 2025: Updated to calculate and store connection quality
 * 
 * @return Connection quality percentage (0-100)
 */
uint16_t GetSuccessRate()
{
    // Calculate connection quality and store in global variable
    uint16_t Perfection = (PERFECTPACKETSPERSECOND * (uint16_t)ConnectionAssessSeconds);
    uint16_t successCount = 0;
    
    // Count successful packets in the buffer
    for (uint16_t i = 0; i < Perfection; i++) {
        if (PacketsHistoryBuffer[i] == 1) {
            successCount++;
        }
    }
    
    // Calculate percentage and store in global variable
    ConnectionQualityPercent = (successCount * 100) / Perfection;
    
    // Apply the correction for the UI display (assuming some packets made it but weren't acknowledged)
    uint16_t Total = successCount;
    
    // Apply correction - assume about half of "failed" packets actually succeeded
    Total += ((Perfection - Total) / 2);
    uint16_t SuccessRate = (Total * 100) / Perfection;
    
    return SuccessRate;   // return a percentage of total good packets
}

/*********************************************************************************************************************************/
//                  My new version of the the traditional "map()" function -- but here with exponential added.
/*********************************************************************************************************************************/

FASTRUN float MapWithExponential(float xx, float Xxmin, float Xxmax, float Yymin, float Yymax, float Expo)
{
    Expo  = map(Expo, -100, 100, -0.25, 0.75);
    xx    = pow(xx * xx, Expo);
    Xxmin = pow(Xxmin * Xxmin, Expo);
    Xxmax = pow(Xxmax * Xxmax, Expo);
    return map(xx, Xxmin, Xxmax, Yymin, Yymax);
}

// ******************************************************************************************************************************
bool AnyMatches(uint8_t a, uint8_t b, uint8_t c)
{
    if (a == b) return true;
    if (a == c) return true;
    if (b == c) return true;
    return false;
}

/******************************************** CHANNEL REVERSE FUNCTION **********************************************************/

FASTRUN void ServoReverse()
{
    for (uint8_t i = 0; i < 16; i++) {
        if (ReversedChannelBITS & 1 << i) {                                                   // Is this channel reversed?
            PreMixBuffer[i] = map(SendBuffer[i], MINMICROS, MAXMICROS, MAXMICROS, MINMICROS); // Yes so reverse the channel
            SendBuffer[i]   = PreMixBuffer[i];
        }
    }
}

/************************************************************************************************************/
void DelayWithDog(uint32_t HowLong)
{                                                                       // Implements delay() and meanwhile keeps kicking the dog (very cruelly) to keep it quiet.
    uint32_t ThisMoment = millis();
    static bool AlreadyKicking = false;
    if (AlreadyKicking) return;                                         // Guard against recursive calls
    AlreadyKicking = true;
    while ((millis() - ThisMoment) < HowLong) {
        KickTheDog();
        if (ModelMatched && BoundFlag){
            GetNewChannelValues();                                      // Get new channel values from tx even during delay
            FixMotorChannel();
            SendData();                                                 // Send new channel values to servos even during delay
        }
    }
    AlreadyKicking = false;
}
/************************************************************************************************************/
template<typename any>
void Look(const any& value) // this is a template function that can print anything but cannot be used to change anything
{
    Serial.println(value);
}

/************************************************************************************************************/
template<typename any>
void Look1(const any& value) // this is a template function that can print anything but cannot be used to change anything
{
    Serial.print(value);
}


// ********************************************************************************************************************
void SpeedTest(){

  // calculate how many prime numbers are there between 1 and 100000
  // and also how long it takes to calculate them
  Look("Calculating prime numbers between 1 and 100000 ...");
  int count = 0;
  unsigned long start = millis();
  for (int i = 1; i < 100000; i++)
  {
    KickTheDog();
    bool isPrime = true;
    for (int j = 2; j <= i / 2; j++)
    {
      if (i % j == 0)
      {
        isPrime = false;
        break;
      }
    }
    if (isPrime)
    {
      count++;
    }
  }
  unsigned long end = millis();
  Look1("There are ");
  Look1(count);
  Look(" prime numbers between 1 and 100000");
  Look1("Time taken to calculate them is ");
  Look1(end - start);
  Look(" milliseconds");
  DelayWithDog(1000);
}

/******************************************************************************************************************************/
// Gets Windows style confirmation (FILE OVERWRITE ETC.)
// params:
// Prompt is the prompt
// goback is the command needed to return to calling page

bool GetConfirmation(char* goback, char* Prompt)
{
    char GoPopupView[] = "page PopupView";
    char Dialog[]      = "Dialog";
    SendCommand(GoPopupView);
    SendText(Dialog, Prompt);
    GetYesOrNo();
    SendCommand(goback);
    LastFileInView = 120;
    if (Confirmed[0] == 'Y') return true; // tell caller OK to continue
    return false;                         // tell caller STOP!
}

/******************************************************************************************************************************/
// Windows style MSGBOX
// params:
// Prompt is the prompt
// goback is the command needed to return to calling page

void MsgBox(char* goback, char* Prompt)
{
    char GoPopupView[] = "page PopupView";
    char Dialog[]      = "Dialog";
    char NoCancel[]    = "vis b1,0"; // hide cancel button
    SendCommand(GoPopupView);
    SendCommand(NoCancel);
    SendText(Dialog, Prompt);
    GetYesOrNo();
    SendCommand(goback);
    LastFileInView = 120;
    return;
}

/******************************************************************************************************************************/
void YesPressed() { Confirmed[0] = 'Y'; }
/******************************************************************************************************************************/
void NoPressed() { Confirmed[0] = 'N'; }
/******************************************************************************************************************************/


/******************************************************************************************************************************/

void GetYesOrNo(){                  // on return from here, Confirmed[0] will be Y or N
    Confirmed[0] = '?';
    while (Confirmed[0] == '?') { // await user response
        CheckForNextionButtonPress();
        CheckPowerOffButton();// heer
        KickTheDog();
        if (BoundFlag && ModelMatched) {
           GetNewChannelValues();
           FixMotorChannel();
           SendData();
        }
    }
}
/******************************************************************************************************************************/
bool GetBackupFilename(char* goback, char* tt1, char* MMname, char* heading, char* pprompt)
{ // HERE THE USER CAN REPLACE DEFAULT FILENAME IF HE WANTS TO

    char GoBackupView[] = "page BackupView";
    char t0[]           = "t0";        // prompt
    char t1[]           = "t1";        // default filename
    char t3[]           = "t3";        // heading
    char Mname[]        = "Modelname"; // model name
    SendCommand(GoBackupView);
    SendText(t0, pprompt);   // prompt
    SendText(t1, tt1);       // filename
    SendText(Mname, MMname); // Model name
    SendText(t3, heading);   // heading
    GetYesOrNo();
    GetText(t1, SingleModelFile);
    SendCommand(goback);
    if (Confirmed[0] == 'Y') return true;
    return false;
}
/******************************************************************************************************************************/
void SaveCurrentModel()
{
    SavedModelNumber = ModelNumber;
}

// ************************************************************************
// This function looks at the TextIn for an int and returns it as an integer
int GetIntFromTextIn(uint8_t offset)
{
union{
        uint8_t     F4Bytes[4];
        int         FDWord;
     }              NextionCommand;
        
        for (uint8_t i = 0; i < 4; i++) NextionCommand.F4Bytes[i] = TextIn[i+offset];
        return NextionCommand.FDWord;
}
#endif 