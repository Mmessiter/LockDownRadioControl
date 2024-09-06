// *************************************** LOGFILES.h  *****************************************

// This is the code for the LOG DATA screen

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef LOGFILES_H
    #define LOGFILES_H

/******************************************************************************************************************************/
void LogAverageFrameRate(){
    char TheText[] = "Average frame rate: ";
    char buf[40]   = " ";
    char NB[10];
    char fps[]     = " fps";
    Str(NB, AverageFrameRate, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    strcat(buf, fps);
    LogText(buf, sizeof(buf),false);
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
    LogText(buf, sizeof(buf),true);
    LogRXVoltsPerCell();
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
            if (CurrentView != LOGVIEW) { // If not a log file word wrap is allowed 
                if (Column >= MAXWIDTH) {
                    Column = WordWrap(htext);
                    ++LineCounter;
                }
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
    }else{
        strcpy(DateAndTime, "NO_CLOCK");
    }
}
/************************************************************************************************************/
FASTRUN void MakeLogFileName()
{
    char NB[10];
    char dash[] = "-";
    if (strlen(LogFileName) > 0) return; // Already done
    if (RTC.read(tm)) {
        ReadTheRTC();
          if (MayBeAddZero(GmonthDay)) 
        {
            strcpy(LogFileName, "0");
            strcat(LogFileName, Str(NB, GmonthDay, 0));
        }
        else
        {
            strcpy(LogFileName, Str(NB,GmonthDay, 0));
        }
        strcat(LogFileName, dash);
        if (MayBeAddZero(Gmonth)) strcat(LogFileName, "0");
        strcat(LogFileName, Str(NB, Gmonth, 0));
        strcat(LogFileName, dash);
        strcat(LogFileName, Str(NB, Gyear, 0));
        strcat(LogFileName, ".LOG");
    }
    else
    {
        strcpy(LogFileName, "NO_CLOCK.LOG");
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
    
    char LogTeXt1[]   = "LogText";
    char BlankText[] = " ";
    CloseLogFile();
    MakeLogFileName();
    DeleteLogFile();
    SendText1(LogTeXt1, BlankText);
    if (!UseLog) return;
    if (LedWasGreen) {
        RecentStartLine = 0;
            MakeLogFileName();  
            ShowLogFile(RecentStartLine);
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
FASTRUN bool LogFilePreamble()
{
    char dbuf[22];
    char Divider[] = " - ";
    static char LastTimeStamp[20];
    CreateTimeStamp(dbuf);    // Put time stamp into buffer
    if (strcmp(dbuf, LastTimeStamp) == 0) return false; // Don't log the same time twice
    strcpy(LastTimeStamp, dbuf);
    WriteToLogFile(dbuf, 9);  // Add time stamp 9 bytes long
    WriteToLogFile(Divider, 3);
    return true;
}

// ************************************************************************
FASTRUN void LogText(char* TheText, uint16_t len, bool TimeStamp)

{
    char crlf[] = {'|', 13, 10, 0};
    char Tab[]   = "               - ";
    static char LastText[100];
    if (!len) return;
    if (strcmp(TheText, LastText) == 0) return;  // Don't log the same thing twice
    
    CheckLogFileIsOpen();
    strcpy(LastText, TheText);
  
    if (TimeStamp) 
        {
           if(!LogFilePreamble()) WriteToLogFile(Tab, strlen(Tab)); 
        }   
        else
        {
            WriteToLogFile(Tab, strlen(Tab)); 
        }
  
    WriteToLogFile(TheText, len);
    WriteToLogFile(crlf, sizeof(crlf));
    CloseLogFile();
}

// ************************************************************************
FASTRUN void LogMinGap()
{
    char TheText[] = "Minimum logged gap: ";
    char buf[25]   = " ";
    char NB[]      = "    ";
    char ms[]      = " ms";

    Str(NB, MinimumGap, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    strcat(buf, ms);
    LogText(buf, sizeof(buf), false);
}
// ************************************************************************
FASTRUN void LogConnection()
{
    char TheText[] = "Connected to ";
    char buf[40]   = " ";
    strcpy(buf, TheText);
    strcat(buf, ModelName);
    LogText(buf, sizeof(buf),true);
    LogMinGap();
    LogRXVoltsPerCell();
    LogTXVoltsPerCell();
    LogTimeSinceBoot();
}

// ************************************************************************

void MMLog(char * TheText){ // Model Memory Log called from below
  char buf[40]   = " ";
    strcpy(buf, TheText);
    strcat(buf, ModelName);
    LogText(buf, sizeof(buf),false);
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

void LogTXVoltsPerCell(){    
    char TheText[] = "TX Volts per cell: ";
    char buf[40]   = " ";
    char NB[10];
    if (TXVoltsPerCell < 0.1) return;
    dtostrf(TXVoltsPerCell, 2, 2, NB);
    strcpy(buf, TheText);
    strcat(buf, NB);
    LogText(buf, sizeof(buf),false);
}
// ************************************************************************

void LogRXVoltsPerCell(){    
    char TheText[] = "RX Volts per cell: ";
    char buf[40]   = " ";
    char NB[10];
    if (RXVoltsPerCell < 2) return;                                                     // No point in logging if not connected
    dtostrf(RXVoltsPerCell, 2, 2, NB);
    strcpy(buf, TheText);
    strcat(buf, NB);
    LogText(buf, sizeof(buf),false);
    
}
// ************************************************************************
void LogStopFlyingMsg(){
    char TheText[] = "'Stop flying' warning issued!";
    LogText(TheText, sizeof(TheText),true);
}
// ************************************************************************

FASTRUN void LogEndLine()
{
    char sp[]    = "(End of this flight log.)";
    LogText(sp, strlen(sp),false);
}
// ************************************************************************
FASTRUN void LogDisConnection()
{
    char buf[40]   = " ";
    char TheText[] = "Disconnected from ";
    strcpy(buf, TheText);
    strcat(buf, ModelName);
    LogText(buf, sizeof(buf),true);
    LogConnectedDuration();
    LogLongestGap();
    LogTotalLostPackets();
    LogTotalGoodPackets();
    LogTotalRXSwaps();
    LogRXVoltsPerCell();
    LogTXVoltsPerCell();
    LogOverallSuccessRate();
    LogAverageFrameRate();
    LogTimeSinceBoot();
    LogEndLine();
}
// ************************************************************************
FASTRUN void LogNewBank()
{
    char Ltext[50];
    char bk[]    = "Bank";
    char thetext[40];

    strcpy(Ltext, BankTexts[BanksInUse[Bank - 1]]); // Get the bank name text
     if (!InStrng(bk,Ltext))                         // If not already there, add the bank number    
     {
        char colon[] = " (Bank ";
        char rhb[]   = ")";
        char NB[5];
        strcat(Ltext, colon);
        Str(NB, Bank, 0);
        strcpy(thetext, Ltext);
        strcat(thetext, NB);
        strcat(thetext, rhb);
     }else{
        strcpy(thetext, Ltext);
     }
    LogText(thetext, strlen(thetext),true);
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
    LogText(thetext, 9,true);
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
    LogText(thetext, 10,true);
}
// ************************************************************************

FASTRUN void LogThisRX()
{
    char Ltext[] = "RX: ";
    char thetext[10];
    strcpy(thetext, Ltext);
    strcat(thetext, ThisRadio);
    LogText(thetext, 5,true);
}

// ************************************************************************
FASTRUN void LogLowBattery()
{ // Not yet implemented
    char TheText[] = "Low battery";
    LogText(TheText, strlen(TheText),true);
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
    LogText(thetext, 8,true);
}
// ************************************************************************
FASTRUN void LogLongestGap()
{
    char thetext[50];
    char ms[] = " ms";
    snprintf(thetext, 45, "Longest gap: %lu", (unsigned long)GapLongest);
    strcat(thetext, ms);
    LogText(thetext, strlen(thetext),false);
}
// ************************************************************************
 FASTRUN void LogBuddyChange(){

    char OnText[] = "Buddy ON";
    char OffText[] = "Buddy OFF";
    if (BuddyON) LogText(OnText, strlen(OnText),true); else LogText(OffText, strlen(OffText),true);  
 }
// ************************************************************************
void LogTotalLostPackets()
{
    char thetext[50];
    snprintf(thetext, 45, "Lost data packets:  %lu", (unsigned long)TotalLostPackets);
    LogText(thetext, strlen(thetext),false);
}
// ************************************************************************

void LogTotalGoodPackets()
{
    char thetext[50];
    snprintf(thetext, 45, "Good data packets: %lu", (unsigned long)TotalGoodPackets);
    LogText(thetext, strlen(thetext),false);
}
// ************************************************************************

void LogOverallSuccessRate()
{
    char     thetext[50];
    uint32_t OverallSuccessRate = 0;
    OverallSuccessRate          = GetOverallSuccessRate();
    snprintf(thetext, 45, "Success rate: %d%%", (uint8_t)OverallSuccessRate);
    LogText(thetext, strlen(thetext),false);
}
// ************************************************************************

FASTRUN void LogPowerOn()
{
    char Ltext[] = "Power ON";
    char sp[]    = "*******************************************";
    LogText(sp, strlen(sp),false);
    LogText(Ltext, strlen(Ltext),true);
    CheckTXVolts();
    LogTXVoltsPerCell();
}

// ************************************************************************

FASTRUN void LogPowerOff()
{
    char Ltext[] = "Power OFF";
    char sp[]    = "*******************************************";
    LogText(Ltext, strlen(Ltext),true);
    LogText(sp, strlen(sp),false);
}

// ************************************************************************

void LogNewRateInUse(){

    char TheText[] = "Rate: ";
    char buf[40]   = " ";
    char NB[10];
    if (DualRateInUse > 3) return;
    Str(NB, DualRateInUse, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    LogText(buf, sizeof(buf),true);
}

// ************************************************************************

FASTRUN void LogThisModel()
{
    char Ltext[] = "Model loaded: ";
    char thetext[75];
    strcpy(thetext, Ltext);
    strcat(thetext, ModelName);
    LogText(thetext, strlen(Ltext) + strlen(ModelName),false);
}

// ************************************************************************

void LogReleased(){

}
 
// ************************************************************************

    
void LogTouched(){                                              // *********** heer
    // char Screen_Y[] = SCREEN_Y; 
    // DelayWithDog(25);
    // char logtextvaly[] = "LogText.val_y";
    // char HitDown[] = "Hit 'Down'";
    // char buf[40]   = " ";
    // char NB[10];
    // char TheText[] = "Screen touched at Y: ";
    // Str(NB, GetOtherValue(Screen_Y), 0); // position of touch on screen
    // strcpy(buf, TheText);
    // strcat(buf, NB);
    // Look (buf);
    // Look(GetOtherValue(logtextvaly));// how far scrolling has gone
    // Look("");
}

// ************************************************************************

void ShowLogFile(uint16_t StartLine)
{
    char TheText[MAXFILELEN + 10]; // MAX = 5K or so
    char LogTeXt1[] = "LogText";
    char t0[]       = "t0";
    char Logtitle[] = "Log date: ";
    char buf[40]    = " ";
    
    CloseLogFile();
    ReadTextFile(LogFileName, TheText, StartLine, MAXLINES); // Then load text
    strcpy(buf, Logtitle);
    for (uint8_t i = 0; i < 8; i++) {
        buf[i + 10] = LogFileName[i];
        if (buf[i + 10] == '-') buf[i + 10] = '/';
        buf[i + 11] = 0;
    }
    SendText(t0, buf);
    SendText1(LogTeXt1, TheText); // Send it to the screen
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
    
    char pDataView[] = "page DataView";
    CurrentView      = DATAVIEW;
    LastShowTime     = 0;
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
    
    char pLogView[] = "page LogView";
    SendCommand(pLogView);
    DelayWithDog(200);
    CurrentView = LOGVIEW;
    ClearFilesList();
    if (UseLog) {
        RecentStartLine = 0;
        MakeLogFileName();  
        ShowLogFile(RecentStartLine);
    }
}

/******************************************************************************************************************************/

void  LogTotalRXSwaps(){
    char TheText[] = "RX swaps: ";
    char buf[40]   = " ";
    char NB[10];
    Str(NB, RadioSwaps, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    LogText(buf, sizeof(buf),false);
}



/******************************************************************************************************************************/

void LogTimeSince(char* TheText, uint32_t Time){
   
    char buf[90]   = " ";
    char NB[20];
    // how many minutes and seconds since Time

    Str(NB, Time / 60000, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    strcat(buf, " minute");
    if ((Time / 60000 > 1) || (Time / 60000 == 0)) strcat(buf, "s");
    strcat(buf, " and ");
    Str(NB, (Time % 60000) / 1000, 0);
    strcat(buf, NB);
    strcat(buf, " second");
    if ((Time % 60000) / 1000 > 1) strcat(buf, "s");
    LogText(buf, sizeof(buf),false);
}
/******************************************************************************************************************************/
void LogTimeSinceBoot(){
    char TheText[] = "Time since boot: ";
    LogTimeSince(TheText, millis());// how many minutes and seconds since boot

}
/******************************************************************************************************************************/
void LogConnectedDuration(){
    uint32_t Duration = millis() - LedGreenMoment;
    char TheText[] = "Time connected: ";
    LogTimeSince(TheText, Duration); // how many minutes and seconds since connection
}
#endif