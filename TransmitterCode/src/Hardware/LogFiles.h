// *************************************** LOGFILES.h  *****************************************

// This is the code for the LOG DATA screen

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef LOGFILES_H
    #define LOGFILES_H

/******************************************************************************************************************************/
void          LogAverageFrameRate(){
    char TheText[] = "Average frame rate: ";
    char buf[40]   = " ";
    char NB[10];
    char fps[]     = " fps";
    Str(NB, AverageFrameRate, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    strcat(buf, fps);
    LogText(buf, sizeof(buf));
}

/******************************************************************************************************************************/
// This function logs the number of minutes the motor has been running

void   LogTimer(uint32_t Mins){
    char TheText[] = "Motor running for ";
    char buf[40]   = " ";
    char NB[10];
    Str(NB, Mins, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    strcat(buf, " minute");
    if (Mins > 1) strcat(buf, "s");
    LogText(buf, sizeof(buf));
}

/*********************************************************************************************************************************/
// This function now scrolls by loading only part of the text file
// This allows very long files to be handled ok.

void ReadTextFile(char* fname, char* htext, uint16_t StartLineNumber, uint16_t MaxLines)
{
#define MAXWIDTH 68
    char     errormsg[]     = "File not found! -> ";
    uint16_t LineCounter    = 0;
    uint16_t StopLineNumber = 0;
    uint16_t i              = 0;
    uint8_t  Column         = 0;
    File     fnumber;

    char crlf[]         = {13, 10, 0};
    char a[]            = " ";
    char dots[]         = "(Hit 'Down' for more ...) ";
    char dots1[]        = "(Hit 'Up' to scroll up ...) ";
    char slash[]        = "/";
    char OpenBracket[]  = "( ";
    char CloseBracket[] = " )";
    char SearchFile[40];

    StopLineNumber = StartLineNumber + MaxLines;
    strcpy(SearchFile, slash);
    strcat(SearchFile, fname);
    strcpy(htext, OpenBracket);
    strcat(htext, SearchFile);
    strcat(htext, CloseBracket);
    strcat(htext, crlf);
    strcat(htext, crlf);
    if (StartLineNumber > 1) {
        strcat(htext, crlf);
        strcat(htext, dots1);
        strcat(htext, crlf);
    }

    ThereIsMoreToSee = false;
    fnumber          = SD.open(SearchFile, FILE_READ);
    if (fnumber) {
        while (fnumber.available() && i < (MAXFILELEN - 10)) {
            a[0] = fnumber.read(); //  Read in one byte at a time.
            if (a[0] == '|') {     //  New Line character = '|'
                if ((LineCounter >= StartLineNumber) && (LineCounter <= StopLineNumber)) {
                    strcat(htext, crlf);
                    ++i;
                    ++i;
                }
                Column = 0;
                ++LineCounter;
                a[0] = 34; // a '34'  ( " ) is ignored.
            }
            if (Column >= MAXWIDTH) {
                Column = WordWrap(htext);
                ++LineCounter;
            }
            if ((a[0] == 13) || (a[0] == 10)) a[0] = 34; // Ignore CrLfs
            if (a[0] != 34) {
                if ((LineCounter >= StartLineNumber) && (LineCounter <= StopLineNumber)) {
                    strcat(htext, a);
                    ++Column;
                    ++i;
                }
            }
            if (LineCounter > StopLineNumber) {
                strcat(htext, crlf);
                strcat(htext, dots);
                ThereIsMoreToSee = true;
                break;
            }
        }
    }
    else
    {
        strcpy(htext, errormsg);
        strcat(htext, fname);
    }
    fnumber.close();
}



/************************************************************************************************************/
FASTRUN void CreateTimeStamp(char* DateAndTime)
{
    char NB[10];
    char zero[]  = "0";
    char Colon[] = "."; // a dot!
    char null[]  = "";

    if (RTC.read(tm)) {
        strcpy(DateAndTime, null);
        if (MayBeAddZero(tm.Hour + DeltaGMT)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Hour + DeltaGMT, 0));
        strcat(DateAndTime, Colon);
        if (MayBeAddZero(tm.Minute)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Minute, 0));
        strcat(DateAndTime, Colon);
        if (MayBeAddZero(tm.Second)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Second, 0));
    }
}
/************************************************************************************************************/
FASTRUN void MakeLogFileName()
{
    char NB[10];
    char dash[] = "-";
    if (strlen(LogFileName) > 0) return; // Already done
    if (RTC.read(tm)) {
        if (MayBeAddZero(tm.Day)) 
        {
            strcpy(LogFileName, "0");
            strcat(LogFileName, Str(NB, tm.Day, 0));
        }
        else
        {
            strcpy(LogFileName, Str(NB, tm.Day, 0));
        }
        strcat(LogFileName, dash);
        if (MayBeAddZero(tm.Month)) strcat(LogFileName, "0");
        strcat(LogFileName, Str(NB, tm.Month, 0));
        strcat(LogFileName, ".LOG");
    }
}
/************************************************************************************************************/
FASTRUN void OpenLogFileW()
{
    if (!LogFileOpen) {
        LogFileNumber = SD.open(LogFileName, FILE_WRITE);
        LogFileOpen   = true;
    }
}
// ************************************************************************
FASTRUN void CheckLogFileIsOpen()
{
    if (!LogFileOpen) {
        MakeLogFileName(); // Create a "today" filename
        OpenLogFileW();    // Open file for writing
    }
}

/************************************************************************************************************/
FASTRUN void DeleteLogFile()
{
    SD.remove(LogFileName);
}
/************************************************************************************************************/
FASTRUN void DeleteLogFile1()
{
    
    char LogTeXt[]   = "LogText";
    char BlankText[] = " ";
    CloseLogFile();
    MakeLogFileName();
    DeleteLogFile();
    SendText1(LogTeXt, BlankText);
    if (!UseLog) return;
    if (LedWasGreen) {
        RecentStartLine = 0;
            MakeLogFileName();  
            ShowLogFile(RecentStartLine);
    }
}

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
    WriteToLogFile(dbuf, 9);  // Add time stamp 9 bytes long
    WriteToLogFile(Divider, 3);
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

void MMLog(char * TheText){ // Model Memory Log called from below
  char buf[40]   = " ";
    strcpy(buf, TheText);
    strcat(buf, ModelName);
    LogText(buf, sizeof(buf));
}
// ************************************************************************

FASTRUN void LogModelMatched()// Model Memory Log
{
    char TheText[] = "Model memory matched: ";
    MMLog(TheText);
}
// ************************************************************************
FASTRUN void LogModelFound()// Model Memory Log
{
    char TheText[] = "Model memory found: ";
    MMLog(TheText);
}
// ************************************************************************

FASTRUN void LogModelNotFound()// Model Memory Log
{
    char TheText[] = "Model memory NOT found: ";
    MMLog(TheText);
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
    LogAverageFrameRate();
}
// ************************************************************************
FASTRUN void LogNewBank()
{
    char Ltext[] = "Bank: ";//
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
 FASTRUN void LogBuddyChange(){

    char OnText[] = "Buddy ON";
    char OffText[] = "Buddy OFF";
    if (BuddyON) LogText(OnText, strlen(OnText)); else LogText(OffText, strlen(OffText));  
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
    char sp[]    = "*******************************************";
    LogText(Ltext, strlen(Ltext));
    LogText(sp, strlen(sp));
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

void ShowLogFile(uint16_t StartLine)
{
    char TheText[MAXFILELEN + 10]; // MAX = 5K or so
    char LogTeXt[] = "LogText";
    CloseLogFile();
    ReadTextFile(LogFileName, TheText, StartLine, MAXLINES); // Then load text
    SendText1(LogTeXt, TheText);                             // Then send it
}

/******************************************************************************************************************************/
void RefreshLog()
{ // refresh log screen
    if (UseLog) {
        RecentStartLine = 0;
        MakeLogFileName();  
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
    if (MinimumGap < 10) MinimumGap = 10;
    LogRXSwaps       = GetValue(c0);
    UseLog           = GetValue(sw0);
    SaveTransmitterParameters();
    ForceDataRedisplay();
    SendCommand(pDataView);
}

/******************************************************************************************************************************/

void DelLOG() // This is not called anymore
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
    char HelpView[] = "HelpText";
    char blank[]    = " ";
    SendCommand(pLogView);
    CurrentView = LOGVIEW;
    ClearFilesList();
    SendText(HelpView, blank);
    SendValue(n0, MinimumGap);
    SendValue(c0, LogRXSwaps);
    SendValue(sw0, UseLog);
    if (UseLog) {
        RecentStartLine = 0;
        MakeLogFileName();  
        ShowLogFile(RecentStartLine);
    }
}

#endif