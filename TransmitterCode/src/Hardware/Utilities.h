// *********************************************** utilities.h for Transmitter code *******************************************

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef UTILITIES_H
    #define UTILITIES_H



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
    LastMaxRateOfClimb   = 0;
    LastRXModelAltitude  = 0;
    LastRXModelMaxAltitude = 0;
    LastRXTemperature    = 0;
    LastRXReceivedPackets = 0;
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
    strcpy(cmd, vol);
    Str(nb, v, 0);
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
    for (int i = 0; i < MAXTEXTIN; ++i)
    {
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

void ClearSuccessRate()
{
    for (int i = 0; i < (PERFECTPACKETSPERSECOND * (uint16_t)ConnectionAssessSeconds); ++i) { // 126 packets per second start off good
        PacketsHistoryBuffer[i] = 1;
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
uint16_t GetSuccessRate()
{
    uint32_t Total = 0;
    uint16_t SuccessRate;

    for (uint16_t i = 0; i < PERFECTPACKETSPERSECOND; ++i)
        Total += PacketsHistoryBuffer[i];
   
    SuccessRate = (Total * 100) / PERFECTPACKETSPERSECOND; // return a percentage of total good packets
    return SuccessRate;
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