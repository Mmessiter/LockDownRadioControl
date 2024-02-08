// *************************************** LOGFILES.h  *****************************************

// This is the code for the wireless buddy system. Its functions are called from the main loop, and from the SendData() function.

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef LOGFILES_H
    #define LOGFILES_H

/************************************************************************************************************/
FASTRUN void CreateTimeStamp(char* DateAndTime)
{
    char NB[10];
    char zero[]  = "0";
    char Colon[] = "."; // a dot!
    char Slash[] = "/";
    char null[]  = "";
    char space[] = " ";

    if (RTC.read(tm)) {
        strcpy(DateAndTime, null);

        if (MayBeAddZero(tm.Day)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Day, 0));
        strcat(DateAndTime, Slash);
        if (MayBeAddZero(tm.Month)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Month, 0));
        strcat(DateAndTime, Slash);
        strcat(DateAndTime, (Str(NB, tmYearToCalendar(tm.Year), 0)));
        strcat(DateAndTime, space);

        if (MayBeAddZero(tm.Hour)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Hour, 0));
        strcat(DateAndTime, Colon);
        if (MayBeAddZero(tm.Minute)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Minute, 0));
        strcat(DateAndTime, Colon);
        if (MayBeAddZero(tm.Second)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Second, 0));
    }
}

/************************************************************************************************************/
FASTRUN void CreateTimeDateStamp(char* DateAndTime)
{
    char NB[10];
    char zero[]  = "0";
    char Dash[]  = "-";
    char Colon[] = "."; // a dot!
    char Space[] = " ";

    if (RTC.read(tm)) {
        if (MayBeAddZero(tm.Day)) strcat(DateAndTime, zero);
        strcpy(DateAndTime, Str(NB, tm.Day, 0));
        strcat(DateAndTime, Dash);
        if (MayBeAddZero(tm.Month)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Month, 0));
        strcat(DateAndTime, Dash);
        strcat(DateAndTime, (Str(NB, tmYearToCalendar(tm.Year), 0)));
        strcat(DateAndTime, Space);
        if (MayBeAddZero(tm.Hour)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Hour, 0));
        strcat(DateAndTime, Colon);
        if (MayBeAddZero(tm.Minute)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Minute, 0));
        strcat(DateAndTime, Colon);
        if (MayBeAddZero(tm.Second)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Second, 0));
    }
}

/************************************************************************************************************/
FASTRUN void MakeLogFileName(char* LogFileName)
{
    char NB[10];
    char Ext[]  = ".LOG";
    char dash[] = "-";
    char zero[] = "0";
    if (RTC.read(tm)) {

        if (MayBeAddZero(tm.Day)) strcat(LogFileName, zero);
        strcpy(LogFileName, Str(NB, tm.Day, 0));
        strcat(LogFileName, dash);
        if (MayBeAddZero(tm.Month)) strcat(LogFileName, zero);
        strcat(LogFileName, Str(NB, tm.Month, 0));
        strcat(LogFileName, Ext);
    }
}
/************************************************************************************************************/
FASTRUN void OpenLogFileW(char* LogFileName)
{
    if (!LogFileOpen) {
        LogFileNumber = SD.open(LogFileName, FILE_WRITE);
        LogFileOpen   = true;
    }
}
// ************************************************************************
FASTRUN void CheckLogFileIsOpen()
{
    char LogFileName[20];
    if (!LogFileOpen) {
        MakeLogFileName(LogFileName); // Create a "today" filename
        OpenLogFileW(LogFileName);    // Open file for writing
    }
}

/************************************************************************************************************/
FASTRUN void DeleteLogFile(char* LogFileName)
{
    SD.remove(LogFileName);
}
/************************************************************************************************************/
FASTRUN void DeleteLogFile1()
{
    char LogFileName[20];
    char LogTeXt[]   = "LogText";
    char BlankText[] = " ";
    CloseLogFile();
    MakeLogFileName(LogFileName);
    DeleteLogFile(LogFileName);
    SendText1(LogTeXt, BlankText);
    if (!UseLog) return;
    if (LedWasGreen) {
        RecentStartLine = 0;
        ShowLogFile(RecentStartLine);
    }
}

/************************************************************************************************************/
FASTRUN void OpenLogFileR(char* LogFileName)
{
    if (!LogFileOpen) {
        LogFileNumber = SD.open(LogFileName, FILE_READ);
        LogFileOpen   = true;
    }
}
/************************************************************************************************************/
FASTRUN void CloseLogFile()
{
    if (LogFileOpen) LogFileNumber.close();
    LogFileOpen = false;
}
/************************************************************************************************************/
FASTRUN void WriteToLogFile(char* SomeData, uint16_t len)
{
    LogFileNumber.write(SomeData, len);
}
// ************************************************************************
FASTRUN void LogFilePreamble()
{
    char dbuf[22];
    char Divider[] = " - ";
    CheckLogFileIsOpen();
    CreateTimeStamp(dbuf);    // Put time stamp into buffer
    WriteToLogFile(dbuf, 20); // Add time stamp
    WriteToLogFile(Divider, sizeof(Divider));
}

// ************************************************************************
FASTRUN void LogText(char* TheText, uint16_t len)
{
    char crlf[] = {'|', 13, 10, 0};
    LogFilePreamble();
    WriteToLogFile(TheText, len);
    WriteToLogFile(crlf, sizeof(crlf));
    CloseLogFile();
}
// ************************************************************************
FASTRUN void LogMinGap()
{
    char TheText[] = "Min logged gap: ";
    char buf[25]   = " ";
    char NB[]      = "    ";

    Str(NB, MinimumGap, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    LogText(buf, sizeof(buf));
}
// ************************************************************************
FASTRUN void LogConnection()
{
    char TheText[] = "Connected to ";
    char buf[40]   = " ";
    strcpy(buf, TheText);
    strcat(buf, ModelName);
    LogText(buf, sizeof(buf));
    LogMinGap();
}


// ************************************************************************

uint32_t GetOverallSuccessRate()
{
    return (TotalGoodPackets * 100) / (TotalGoodPackets + TotalLostPackets);
}
// ************************************************************************
FASTRUN void LogDisConnection()
{
    char buf[40]   = " ";
    char TheText[] = "Disconnected from ";

    strcpy(buf, TheText);
    strcat(buf, ModelName);
    LogText(buf, sizeof(buf));

    LogLongestGap();
    LogTotalLostPackets();
    LogTotalGoodPackets();
    LogOverallSuccessRate();
}
// ************************************************************************
FASTRUN void LogNewBank()
{
    char Ltext[] = "Bank: ";
    char NB[5];
    char thetext[20];
    Str(NB, Bank, 0);
    strcpy(thetext, Ltext);
    strcat(thetext, NB);
    LogText(thetext, 7);
}

// ************************************************************************
FASTRUN void LogMotor(bool On)
{
    char Ltext1[] = "Motor On";
    char Ltext0[] = "Motor Off";
    char thetext[20];
    if (On)
        strcpy(thetext, Ltext1);
    else
        strcpy(thetext, Ltext0);
    LogText(thetext, 9);
}


// ************************************************************************
FASTRUN void LogSafety(bool On)
{
    char Ltext1[] = "Safety On";
    char Ltext0[] = "Safety Off";
    char thetext[20];
    if (On)
        strcpy(thetext, Ltext1);
    else
        strcpy(thetext, Ltext0);
    LogText(thetext, 10);
}
// ************************************************************************

FASTRUN void LogThisRX()
{
    char Ltext[] = "RX: ";
    char thetext[10];
    strcpy(thetext, Ltext);
    strcat(thetext, ThisRadio);
    LogText(thetext, 5);
}

// ************************************************************************
FASTRUN void LogLowBattery()
{ // Not yet implemented
    char TheText[] = "Low battery";
    LogText(TheText, strlen(TheText));
}
// ************************************************************************

FASTRUN void LogThisGap()
{
    char Ltext[] = "Gap: ";
    char NB[5];
    char thetext[10];
    if (ThisGap > 1000) return;
    Str(NB, ThisGap, 0);
    strcpy(thetext, Ltext);
    strcat(thetext, NB);
    LogText(thetext, 8);
}
// ************************************************************************

FASTRUN void LogLongestGap()
{
    char thetext[50];
    snprintf(thetext, 45, "Longest gap: %d", (short int)GapLongest);
    LogText(thetext, strlen(thetext));
}

// ************************************************************************

void LogTotalLostPackets()
{
    char thetext[50];
    snprintf(thetext, 45, "Total lost packets: %d", (short int)TotalLostPackets);
    LogText(thetext, strlen(thetext));
}


// ************************************************************************

void LogTotalGoodPackets()
{
    char thetext[50];
    snprintf(thetext, 45, "Total good packets: %d", (uint16_t)TotalGoodPackets);
    LogText(thetext, strlen(thetext));
}

// ************************************************************************

void LogOverallSuccessRate()
{
    char     thetext[50];
    uint32_t OverallSuccessRate = 0;
    OverallSuccessRate          = GetOverallSuccessRate();
    snprintf(thetext, 45, "Overall success rate: %d%%", (uint8_t)OverallSuccessRate);
    LogText(thetext, strlen(thetext));
}
// ************************************************************************

FASTRUN void
LogPowerOn()
{
    char Ltext[] = "Power ON";
    char sp[]    = "*******************************************";
    LogText(sp, strlen(sp));
    LogText(Ltext, strlen(Ltext));
}

// ************************************************************************

FASTRUN void LogPowerOff()
{
    char Ltext[] = "Power OFF";
    LogText(Ltext, strlen(Ltext));
}

// ************************************************************************

FASTRUN void LogThisModel()
{
    char Ltext[] = "Model loaded: ";
    char thetext[75];
    strcpy(thetext, Ltext);
    strcat(thetext, ModelName);
    LogText(thetext, strlen(Ltext) + strlen(ModelName));
}
// ************************************************************************

void ShowLogFile(uint8_t StartLine)
{
    char TheText[MAXFILELEN + 10]; // MAX = 5K or so
    char LogFileName[20];
    char LogTeXt[] = "LogText";
    CloseLogFile();
    MakeLogFileName(LogFileName);                            // Create "today" filename
    ReadTextFile(LogFileName, TheText, StartLine, MAXLINES); // Then load text
    SendText1(LogTeXt, TheText);                             // Then send it
}




/******************************************************************************************************************************/
void UpLog()
{
    if (RecentStartLine > 0 && (CurrentView == LOGVIEW)) {
        RecentStartLine -= 1;
        if (RecentStartLine < 0) RecentStartLine = 0;
        ShowLogFile(RecentStartLine);
    }
    if (RecentStartLine > 0 && (CurrentView == HELP_VIEW)) {
        RecentStartLine -= 1;
        if (RecentStartLine < 1) RecentStartLine = 0;
        ScrollHelpFile();
    }
}
/******************************************************************************************************************************/
void DownLog()
{

    if (ThereIsMoreToSee && (CurrentView == LOGVIEW)) {
        RecentStartLine += 1;
        ShowLogFile(RecentStartLine);
    }
    if (ThereIsMoreToSee && (CurrentView == HELP_VIEW)) {
        RecentStartLine += 1;
        ScrollHelpFile();
    }
}
/******************************************************************************************************************************/
void RefreshLog()
{ // refresh log screen
    if (UseLog) {
        RecentStartLine = 0;
        ShowLogFile(RecentStartLine);
        ClearText();
    }
}
/******************************************************************************************************************************/
void LogEND()
{ // close log screen
    char n0[]        = "n0";
    char c0[]        = "c0";
    char sw0[]       = "sw0";
    char pDataView[] = "page DataView";
    CurrentView      = DATAVIEW;
    LastShowTime     = 0;
    MinimumGap       = GetValue(n0);
    if (MinimumGap < 50) MinimumGap = 50;
    LogRXSwaps       = GetValue(c0);
    UseLog           = GetValue(sw0);
    SaveTransmitterParameters();
    SendCommand(pDataView);
}
/******************************************************************************************************************************/
void DelLOG()
{ // delete log and start new one

    char prompt[]   = "Delete log file?";
    char pLogView[] = "page LogView";
    if (GetConfirmation(pLogView, prompt)) {
        DeleteLogFile1();
    }
    ClearText();
}
/******************************************************************************************************************************/
void LogVIEW()
{ // Start log screen
    char n0[]       = "n0";
    char c0[]       = "c0";
    char sw0[]      = "sw0";
    char pLogView[] = "page LogView";
    SendCommand(pLogView);
    CurrentView = LOGVIEW;
    SendValue(n0, MinimumGap);
    SendValue(c0, LogRXSwaps);
    SendValue(sw0, UseLog);
    if (UseLog) {
        RecentStartLine = 0;
        ShowLogFile(RecentStartLine);
    }
}

#endif