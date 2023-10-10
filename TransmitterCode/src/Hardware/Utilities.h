// *********************************************** utilities.h for Transmitter code *******************************************


#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef UTILITIES_H
    #define UTILITIES_H
// *****************************************************************************************************

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
        Gsecond        = GPSSecs;
        Gminute        = GPSMins;
        Ghour          = GPSHours;
        GmonthDay      = GPSDay;
        Gmonth         = GPSMonth;
        Gyear          = GPSYear + 1744; // ????
        SetTheRTC();
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
    char colon1[]   = ".";
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
            if (FHSS_data::UkRules) strcat(TimeString, colon);
            if (!FHSS_data::UkRules) strcat(TimeString, colon1);
            if (MayBeAddZero(tm.Minute)) strcat(TimeString, zero);
            strcat(TimeString, Str(NB, tm.Minute, 0));
            if (FHSS_data::UkRules) strcat(TimeString, colon);
            if (!FHSS_data::UkRules) strcat(TimeString, colon1);
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
        if ((millis() - BlinkTimer) > (750 / BlinkHertz)) {
            BlinkOnPhase ^= 1;
            BlinkTimer = millis();
        }
    }
    else {
        BlinkOnPhase = 1;
    }
    if (BlinkOnPhase) {
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
void CheckScreenTime()
{
    char ScreenOff[] = "dim=10";
    if ((millis() - ScreenTimeTimer) > ScreenTimeout * 1000) {
        SendCommand(ScreenOff);
        ScreenTimeTimer = millis();
        ScreenIsOff     = true;
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

void FixCHNames()
{

    char MixesView_chM[]           = "chM";
    char MixesView_chS[]           = "chS";
    char MixesView_MasterChannel[] = "MasterChannel";
    char MixesView_SlaveChannel[]  = "SlaveChannel";

    Mixes[MixNumber][M_MasterChannel] = GetValue(MixesView_MasterChannel);
    Mixes[MixNumber][M_SlaveChannel]  = GetValue(MixesView_SlaveChannel);
    SendText(MixesView_chM, ChannelNames[Mixes[MixNumber][M_MasterChannel] - 1]);
    SendText(MixesView_chS, ChannelNames[Mixes[MixNumber][M_SlaveChannel] - 1]);
}

/*********************************************************************************************************************************/

void BlueLedOn()
{
    LedWasGreen = false;
    LedWasRed   = false;
    analogWrite(REDLED, 0);
    analogWrite(GREENLED, 0);
    analogWrite(BLUELED, GetLEDBrightness()); // Brightness is a function of maybe blinking
}

/*********************************************************************************************************************************/
void ClearSuccessRate()
{
    for (int i = 0; i < (PERFECTPACKETSPERSECOND * (uint16_t)ConnectionAssessSeconds); ++i) { // 126 packets per second start off good
        PacketsHistoryBuffer[i] = 1;
    }
}

/*********************************************************************************************************************************/

void SaveMixValues()
{

    char MixesView_Enabled[]       = "Enabled";
    char MixesView_Bank[]          = "FlightMode";
    char MixesView_MasterChannel[] = "MasterChannel";
    char MixesView_SlaveChannel[]  = "SlaveChannel";
    char MixesView_Reversed[]      = "Reversed";
    char MixesView_Percent[]       = "Percent";
    char MixesView_od[]            = "od"; // One direction
    char MixesView_offset[]        = "Offset";
    char ProgressStart[]           = "vis Progress,1";
    char Progress[]                = "Progress";

    SendCommand(ProgressStart);
    SendValue(Progress, 5);
    Mixes[MixNumber][M_Enabled] = GetValue(MixesView_Enabled);
    SendValue(Progress, 10);
    Mixes[MixNumber][M_Bank] = GetValue(MixesView_Bank);
    SendValue(Progress, 25);
    Mixes[MixNumber][M_MasterChannel] = GetValue(MixesView_MasterChannel);
    SendValue(Progress, 40);
    Mixes[MixNumber][M_SlaveChannel] = GetValue(MixesView_SlaveChannel);
    SendValue(Progress, 55);
    Mixes[MixNumber][M_Reversed] = GetValue(MixesView_Reversed);
    SendValue(Progress, 70);
    Mixes[MixNumber][M_Percent] = GetValue(MixesView_Percent);
    SendValue(Progress, 87);
    Mixes[MixNumber][M_ONEDIRECTION] = GetValue(MixesView_od);
    SendValue(Progress, 95);
    Mixes[MixNumber][M_OFFSET] = GetValue(MixesView_offset) + 127; // because it's unsigned
    SendValue(Progress, 100);
}

/*********************************************************************************************************************************/

void ShowMixValues() // sends mix values to Nextion screen
{
    char MixesView_Enabled[]       = "Enabled";
    char MixesView_Bank[]          = "FlightMode";
    char MixesView_MasterChannel[] = "MasterChannel";
    char MixesView_SlaveChannel[]  = "SlaveChannel";
    char MixesView_Reversed[]      = "Reversed";
    char MixesView_Percent[]       = "Percent";
    char MixesView_chM[]           = "chM";
    char MixesView_chS[]           = "chS";
    char MixesView_od[]            = "od";
    char MixesView_offset[]        = "Offset";

    SendText(MixesView_chM, ChannelNames[Mixes[MixNumber][M_MasterChannel] - 1]);
    SendText(MixesView_chS, ChannelNames[Mixes[MixNumber][M_SlaveChannel] - 1]);
    SendValue(MixesView_Enabled, Mixes[MixNumber][M_Enabled]);
    SendValue(MixesView_Bank, Mixes[MixNumber][M_Bank]);
    if (Mixes[MixNumber][M_MasterChannel] == 0) Mixes[MixNumber][M_MasterChannel] = 1;
    SendValue(MixesView_MasterChannel, Mixes[MixNumber][M_MasterChannel]);
    if (Mixes[MixNumber][M_SlaveChannel] == 0) Mixes[MixNumber][M_SlaveChannel] = 1;
    SendValue(MixesView_SlaveChannel, Mixes[MixNumber][M_SlaveChannel]);
    SendValue(MixesView_Reversed, Mixes[MixNumber][M_Reversed]);
    if (Mixes[MixNumber][M_Percent] == 0) Mixes[MixNumber][M_Percent] = 100;
    if (Mixes[MixNumber][M_SlaveChannel] == Mixes[MixNumber][M_MasterChannel]) {
        Mixes[MixNumber][M_SlaveChannel]++;
        SendValue(MixesView_SlaveChannel, Mixes[MixNumber][M_SlaveChannel]);
    }
    SendValue(MixesView_Percent, Mixes[MixNumber][M_Percent]);
    SendValue(MixesView_od, Mixes[MixNumber][M_ONEDIRECTION]);
    if (((Mixes[MixNumber][M_OFFSET]) > 227) || ((Mixes[MixNumber][M_OFFSET]) < 27)) Mixes[MixNumber][M_OFFSET] = 127; // zeroed if out of range
    SendValue(MixesView_offset, Mixes[MixNumber][M_OFFSET] - 127);                                                     // because it's 'unsigned'
    SendText(MixesView_chM, ChannelNames[Mixes[MixNumber][M_MasterChannel] - 1]);
    SendText(MixesView_chS, ChannelNames[Mixes[MixNumber][M_SlaveChannel] - 1]);
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
int GetSuccessRate()
{
    uint16_t Total = 0;
    uint16_t SuccessRate;
    uint16_t Perfection = (PERFECTPACKETSPERSECOND * (uint16_t)ConnectionAssessSeconds);

    for (uint16_t i = 0; i < Perfection; ++i) { // PERFECTPACKETSPERSECOND (126) packets per second are either good or bad
        Total += PacketsHistoryBuffer[i];
    }
    Total += (Perfection - Total) / 2;        // about half made it but were simply unacknowledged
    SuccessRate = (Total * 100) / Perfection; // return a percentage of total good packets
    return SuccessRate;
}

/*********************************************************************************************************************************/
// this function looks at the most recent ((uint16_t) ConnectionAssessSeconds) few seconds of packets which succeeded and expresses these
// as a percentage of total attempted packets.

void ShowConnectionQuality()
{
    char Quality[] = "Quality";
    char Msgbuf[80];
    char Msg_Connected[]          = "Connection: ";
    char Msg_ConnectedPerfect[]   = "Perfect";
    char Msg_ConnectedExcellent[] = "Excellent";
    char Msg_ConnectedVGood[]     = "Very Good";
    char Msg_ConnectedGood[]      = "Good";
    char Msg_ConnectedMarginal[]  = "Marginal";
    char Msg_ConnectedWeak[]      = "Weak";
    char Msg_ConnectedVWeak[]     = "Very weak";
    int  ConnectionQuality        = GetSuccessRate();
    char FrontView_Connected[]    = "Connected";
    char Visible[]                = "vis Quality,1";
    char TXModuleMSG[]            = "** Using TX module **";

    if (PPMdata.UseTXModule) SendText(FrontView_Connected, TXModuleMSG);
    if (!LedWasGreen) return;
    SendValue(Quality, ConnectionQuality); // show quality of connection in progress bar
    strcpy(Msgbuf, Msg_Connected);
    if (ConnectionQuality >= 100) strcat(Msgbuf, Msg_ConnectedPerfect); // show quality as a comment
    if ((ConnectionQuality >= 95) && (ConnectionQuality < 100)) strcat(Msgbuf, Msg_ConnectedExcellent);
    if ((ConnectionQuality >= 90) && (ConnectionQuality < 95)) strcat(Msgbuf, Msg_ConnectedVGood);
    if ((ConnectionQuality >= 75) && (ConnectionQuality < 90)) strcat(Msgbuf, Msg_ConnectedGood);
    if ((ConnectionQuality >= 50) && (ConnectionQuality < 75)) strcat(Msgbuf, Msg_ConnectedMarginal);
    if ((ConnectionQuality >= 25) && (ConnectionQuality < 50)) strcat(Msgbuf, Msg_ConnectedWeak);
    if ((ConnectionQuality >= 1) && (ConnectionQuality < 25)) strcat(Msgbuf, Msg_ConnectedVWeak);
    
     if(!WirelessBuddy)  SendText(FrontView_Connected, Msgbuf);
    
    SendCommand(Visible);
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

/******************************************** CHANNEL REVERSE FUNCTION **********************************************************/

FASTRUN void DoReverseSense()
{
    for (uint8_t i = 0; i < 16; i++) {
        if (ReversedChannelBITS & 1 << i) {                                                   // Is this channel reversed?
            PreMixBuffer[i] = map(SendBuffer[i], MINMICROS, MAXMICROS, MAXMICROS, MINMICROS); // Yes so reverse the channel
            SendBuffer[i]   = PreMixBuffer[i];
        }
    }
}
/************************************************************************************************************/
void ReEnableScanButton() // Scan button AND models button
{
    char b5NOTGreyed[]  = "b5.pco=";
    char b12NOTGreyed[] = "b12.pco=";
    char b1NOTGreyed[]  = "b1.pco=";

    char nb[20];
    char cmd[40];

    Str(nb, ForeGroundColour, 0);

    if (CurrentView == TXSETUPVIEW && b5isGrey) {
        strcpy(cmd, b5NOTGreyed);
        strcat(cmd, nb);
        SendCommand(cmd);
        strcpy(cmd, b1NOTGreyed);
        strcat(cmd, nb);
        SendCommand(cmd);
        b5isGrey = false;
    }

    if (CurrentView == RXSETUPVIEW && b12isGrey) {
        strcpy(cmd, b12NOTGreyed);
        strcat(cmd, nb);
        SendCommand(cmd);
        b12isGrey = false;
    }
}

/************************************************************************************************************/
void DelayWithDog(uint32_t HowLong)
{ // Implements delay() and also kicks the dog cruelly to keep it quiet.
    uint32_t ThisMoment = millis();
    while ((millis() - ThisMoment) < HowLong) {
        KickTheDog();
    
        //  if (BoundFlag && Connected && ModelMatched && CurrentView != FRONTVIEW) SendData();
    }
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

/************************************************************************************************************/

void CheckScanButton() // Scan button AND models button
{
    char b5Greyed[]  = "b5.pco=33840";
    char b12Greyed[] = "b12.pco=33840";
    char b1Greyed[]  = "b1.pco=33840";

    if (ModelMatched) {
        if (CurrentView == TXSETUPVIEW) {
            if (!b5isGrey) {
                SendCommand(b5Greyed);
                SendCommand(b1Greyed);
                b5isGrey = true;
            }
        }
        if (CurrentView == RXSETUPVIEW) {
            if (!b12isGrey) {
                SendCommand(b12Greyed);
                b12isGrey = true;
            }
        }
    }
}
/************************************************************************************************************/
#endif 