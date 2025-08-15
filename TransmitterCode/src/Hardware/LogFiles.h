// *************************************** LOGFILES.h  *****************************************

// This is the code to write log data to a logfile

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef LOGFILES_H
#define LOGFILES_H

/******************************************************************************************************************************/
void LogAverageFrameRate()
{
    char TheText[] = "Average frame rate: ";
    char buf[40] = " ";
    char NB[10];
    char fps[] = " fps";
    Str(NB, AverageFrameRate, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    strcat(buf, fps);
    LogText(buf, sizeof(buf), false);
}

/******************************************************************************************************************************/
// This function logs the number of minutes the motor has been running

void LogTimer(uint32_t Mins)
{
    char TheText[] = "Motor running for ";
    char buf[40] = " ";
    char NB[10];
    Str(NB, Mins, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    strcat(buf, " minute");
    if (Mins > 1)
        strcat(buf, "s");
    LogText(buf, sizeof(buf), true);
    LogRXVoltsPerCell();
}
// ******************************************************************************************************************************/
// This function logs the RPM of the rotor if it is greater than 0
void LogRPM(uint32_t RPM)
{
    if (RPM)
    {
        char TheText[] = "Main rotor RPM: ";
        char buf[40] = " ";
        char NB[10];
        Str(NB, RPM, 0);
        strcpy(buf, TheText);
        strcat(buf, NB);
        LogText(buf, sizeof(buf), false);
    }
}
/************************************************************************************************************/
FASTRUN void CreateTimeStamp(char *DateAndTime)
{
    char NB[10];
    char zero[] = "0";
    char Colon[] = "."; // a dot!
    char null[] = "";

    if (RTC.read(tm))
    {
        strcpy(DateAndTime, null);
        if (MayBeAddZero(tm.Hour + DeltaGMT))
            strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Hour + DeltaGMT, 0));
        strcat(DateAndTime, Colon);
        if (MayBeAddZero(tm.Minute))
            strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Minute, 0));
        strcat(DateAndTime, Colon);
        if (MayBeAddZero(tm.Second))
            strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Second, 0));
    }
    else
    {
        strcpy(DateAndTime, "NO_CLOCK");
    }
}
/************************************************************************************************************/
FASTRUN void MakeLogFileName()
{
    char NB[10];
    char dash[] = "-";
    if (strlen(LogFileName) > 0)
        return; // Already done
    if (RTC.read(tm))
    {
        ReadTheRTC();
        if (MayBeAddZero(GmonthDay))
        {
            strcpy(LogFileName, "0");
            strcat(LogFileName, Str(NB, GmonthDay, 0));
        }
        else
        {
            strcpy(LogFileName, Str(NB, GmonthDay, 0));
        }
        strcat(LogFileName, dash);
        if (MayBeAddZero(Gmonth))
            strcat(LogFileName, "0");
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

FASTRUN void LogRXsTotalTimes()
{
    // This function logs time spent with RX1 and RX2 in Minutes and seconds
    // It uses RX1TotalTime and RX2TotalTime which are in seconds even when over 60

    char TheText[] = "RX1 use: ";
    char buf[40] = " ";
    char NB[10];
    char colon[] = " minutes and ";
    char secs[] = " seconds";
    uint16_t RX1Mins = RX1TotalTime / 60;
    uint8_t RX1Secs = RX1TotalTime % 60;
    uint16_t RX2Mins = RX2TotalTime / 60;
    uint8_t RX2Secs = RX2TotalTime % 60;
    // Now RX1
    Str(NB, RX1Mins, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    strcat(buf, colon);
    Str(NB, RX1Secs, 0);
    strcat(buf, NB);
    strcat(buf, secs);
    LogText(buf, sizeof(buf), false);
    // Now RX2
    strcpy(buf, "RX2 use: ");
    Str(NB, RX2Mins, 0);
    strcat(buf, NB);
    strcat(buf, colon);
    Str(NB, RX2Secs, 0);
    strcat(buf, NB);
    strcat(buf, secs);
    LogText(buf, sizeof(buf), false);
}
/************************************************************************************************************/
FASTRUN void OpenLogFileW()
{
    if (!LogFileOpen)
    {
        LogFileNumber = SD.open(LogFileName, FILE_WRITE);
        LogFileOpen = true;
    }
}
// ************************************************************************
FASTRUN void CheckLogFileIsOpen()
{
    if (!LogFileOpen)
    {
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
    char LogTeXt1[] = "LogText";
    char BlankText[] = " ";
    CloseLogFile();
    MakeLogFileName();
    DeleteLogFile();
    SendText1(LogTeXt1, BlankText);
}

/************************************************************************************************************/

FASTRUN void CloseLogFile()
{
    if (LogFileOpen)
        LogFileNumber.close();
    LogFileOpen = false;
}
/************************************************************************************************************/
FASTRUN void WriteToLogFile(char *SomeData, uint16_t len)
{
    char txt[] = ".TXT";
    if (InStrng(txt, LogFileName))
        return; // Don't write to a HELP file
    LogFileNumber.write(SomeData, len);
}
// ************************************************************************
/**
 * @brief Checks if the current time stamp is different from the last time stamp and logs the time stamp to a file.
 *
 * @return true if the time stamp is different and successfully logged, false otherwise.
 */
FASTRUN bool LogFilePreamble()
{
    char dbuf[22];
    char Divider[] = " - ";
    static char LastTimeStamp[20];
    CreateTimeStamp(dbuf); // Put time stamp into buffer
    if (strcmp(dbuf, LastTimeStamp) == 0)
        return false; // Don't log the same time twice
    strcpy(LastTimeStamp, dbuf);
    WriteToLogFile(dbuf, 9); // Add time stamp 9 bytes long
    WriteToLogFile(Divider, 3);
    return true;
}

// ************************************************************************
/**
 * @brief Logs the given text to a file.
 *
 * This function logs the provided text to a file. It checks if the text is the same as the last logged text and avoids logging it again.
 * The function also supports adding a timestamp to the log entry.
 *
 * @param TheText The text to be logged.
 * @param len The length of the text.
 * @param TimeStamp Flag indicating whether to add a new timestamp to the log entry.
 */
FASTRUN void LogText(char *TheText, uint16_t len, bool TimeStamp)

{
    char crlf[] = {'|', 13, 10, 0};
    char Tab[] = "               - ";
    static char LastText[100];
    if (!len)
        return;
    if (strcmp(TheText, LastText) == 0)
        return; // Don't log the same thing twice
    CheckLogFileIsOpen();
    if (TimeStamp)
    {
        if (!LogFilePreamble())
            WriteToLogFile(Tab, strlen(Tab));
    }
    else
    {
        WriteToLogFile(Tab, strlen(Tab));
    }
    WriteToLogFile(TheText, len);
    WriteToLogFile(crlf, sizeof(crlf));
    CloseLogFile();
    strcpy(TheText, "");
}

// ************************************************************************
FASTRUN void LogMinGap()
{
    char TheText[] = "Minimum logged gap: ";
    char buf[45] = " ";
    char NB[] = "    ";
    char ms[] = " ms";

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
    char buf[40] = " ";
    strcpy(buf, TheText);
    strcat(buf, ModelName);
    LogText(buf, sizeof(buf), true);
    LogMinGap();
    LogRXVoltsPerCell();
    LogTXVoltsPerCell();
    LogTimeSinceBoot();
}

// ************************************************************************

void MMLog(char *TheText)
{ // Model Memory Log called from below
    char buf[40] = " ";
    strcpy(buf, TheText);
    strcat(buf, ModelName);
    LogText(buf, sizeof(buf), false);
}
// ************************************************************************

FASTRUN void LogModelMatched() // Model Memory Log
{
    char TheText[] = "Model memory matched: ";
    MMLog(TheText);
}
// ************************************************************************
FASTRUN void LogModelFound() // Model Memory Log
{
    char TheText[] = "Model memory found: ";
    MMLog(TheText);
}
// ************************************************************************

FASTRUN void LogModelNotFound() // Model Memory Log
{
    char TheText[] = "Model memory NOT found: ";
    MMLog(TheText);
}
// ************************************************************************

uint16_t GetOverallSuccessRate()
{
    return (RXSuccessfulPackets * 100) / (TotalPacketsAttempted + 1); // Avoid division by zero
}

// ************************************************************************

void LogTXVoltsPerCell()
{
    char TheText[] = "TX Volts per cell: ";
    char buf[40] = " ";
    char NB[10];
    if (TXVoltsPerCell < 0.1)
        return;
    dtostrf(TXVoltsPerCell, 2, 2, NB);
    strcpy(buf, TheText);
    strcat(buf, NB);
    LogText(buf, sizeof(buf), false);
}
// ************************************************************************

void LogRXVoltsPerCell()
{
    char TheText[] = "RX Volts per cell: ";
    char buf[40] = " ";
    char NB[10];
    if (RXVoltsPerCell < 2)
        return; // No point in logging if not connected
    dtostrf(RXVoltsPerCell, 2, 2, NB);
    strcpy(buf, TheText);
    strcat(buf, NB);
    LogText(buf, sizeof(buf), false);
}
// ************************************************************************
void LogStopFlyingMsg()
{
    char TheText[] = "'Stop flying' warning issued!";
    LogText(TheText, sizeof(TheText), true);
}
// ************************************************************************

FASTRUN void LogEndLine()
{
    char sp[] = "(End of this flight log.)";
    LogText(sp, strlen(sp), false);
}
// ************************************************************************
void LogAllGPSMaxs()
{
    char buf[160];
    char NB[16]; // Allow a bit more room for safety

    // Max Distance GPS
    strcpy(buf, "Max distance from mark: ");
    dtostrf(GPS_RX_MaxDistance, 2, 2, NB);
    strcat(buf, NB);
    strcat(buf, " yards");
    LogText(buf, strlen(buf), false);

    // Max Altitude
    strcpy(buf, "Max altitude (from GPS): ");
    dtostrf(GPS_RX_Maxaltitude, 2, 2, NB);
    strcat(buf, NB);
    strcat(buf, " feet");
    LogText(buf, strlen(buf), false);

    // Max Speed
    strcpy(buf, "Max Speed: ");
    dtostrf(GPS_RX_MaxSpeed, 2, 2, NB);
    strcat(buf, NB);
    strcat(buf, " MPH");
    LogText(buf, strlen(buf), false);

    // Satellites
    strcpy(buf, "Number of satellites: ");
    Str(NB, GPS_RX_Satellites, 0);
    strcat(buf, NB);
    LogText(buf, strlen(buf), false);
}
// ************************************************************************
void LogMotorOnDuration()
{
    char TheText[] = "Motor was on for ";
    char buf[80] = " ";
    char NB[10];
    Str(NB, MotorOnSeconds / 60, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    strcat(buf, " minutes and ");
    Str(NB, MotorOnSeconds % 60, 0);
    strcat(buf, NB);
    strcat(buf, " seconds");
    LogText(buf, sizeof(buf), false);
}
// ************************************************************************
void Log_RXMAXModelAltitude()
{
    char TheText[] = "Max (Baro) Model Altitude: ";
    char buf[60] = " ";
    strcpy(buf, TheText);
    strcat(buf, Maxaltitude);
    LogText(buf, sizeof(buf), false);
}
// ************************************************************************
/**
 * @brief Logs the disconnection event.
 *
 * This function logs the disconnection event by creating a log message with the information about the disconnection.
 * It includes the model name and various statistics related to the radio control system.
 * The log message is then passed to the LogText function for logging.

 */
FASTRUN void LogDisConnection()
{
    char buf[40] = " ";
    char TheText[] = "Disconnected from ";

    strcpy(buf, TheText);
    strcat(buf, ModelName);
    LogText(buf, sizeof(buf), true);
    LogConnectedDuration();
    LogMotorOnDuration();
    LogTotalRXSwaps();
    LogRXsTotalTimes();
    if (RXMAXModelAltitude > 0)
        Log_RXMAXModelAltitude();
    if (GPS_RX_FIX)
        LogAllGPSMaxs();
    LogLongestGap();
    LogAverageGap();
    LogAverageFrameRate();
    LogTotalLostPackets();
    LogTotalGoodPackets();
    LogTotalRXGoodPackets();
    LogTotalPacketsAttempted();
    LogRXVoltsPerCell();
    LogTXVoltsPerCell();
    LogOverallSuccessRate();
    LogTimeSinceBoot();
    LogEndLine();
}
// ************************************************************************
FASTRUN void LogNewBank()
{
    char Ltext[50];
    char bk[] = "Bank";
    char thetext[40];

    strcpy(Ltext, BankNames[BanksInUse[Bank - 1]]); // Get the bank name text
    if (!InStrng(bk, Ltext))                        // If not already there, add the bank number
    {
        char colon[] = " (Bank ";
        char rhb[] = ")";
        char NB[5];
        strcat(Ltext, colon);
        Str(NB, Bank, 0);
        strcpy(thetext, Ltext);
        strcat(thetext, NB);
        strcat(thetext, rhb);
    }
    else
    {
        strcpy(thetext, Ltext);
    }
    LogText(thetext, strlen(thetext), true);
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
    LogText(thetext, 9, true);
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
    LogText(thetext, 10, true);
}
// ************************************************************************

FASTRUN void LogThisRX()
{
    char Ltext[] = "RX: ";
    char thetext[10];
    strcpy(thetext, Ltext);
    strcat(thetext, ThisRadio);
    LogText(thetext, 5, true);
}

// ************************************************************************
FASTRUN void LogLowBattery()
{ // Not yet implemented
    char TheText[] = "Low battery";
    LogText(TheText, strlen(TheText), true);
}
// ************************************************************************
void Log_GPS_RX_Altitude()
{
    char TheText[] = "GPS Altitude: ";
    char buf[60] = " ";
    char NB[10];
    dtostrf(GPS_RX_Altitude, 2, 2, NB);
    strcpy(buf, TheText);
    strcat(buf, NB);
    LogText(buf, sizeof(buf), false);
}
// ************************************************************************
void Log_GPS_RX_DistanceTo()
{
    char TheText[] = "GPS Distance: ";
    char buf[60] = " ";
    char NB[10];
    dtostrf(GPS_RX_DistanceTo, 2, 2, NB);
    strcpy(buf, TheText);
    strcat(buf, NB);
    LogText(buf, sizeof(buf), false);
}
// ************************************************************************

FASTRUN void LogThisGap()
{
    char Ltext[] = "Gap: ";
    char NB[5];
    char thetext[10];
    if (ThisGap > 1000)
        return;
    Str(NB, ThisGap, 0);
    strcpy(thetext, Ltext);
    strcat(thetext, NB);
    LogText(thetext, 8, true);
    if (GPS_RX_FIX)
    {
        Log_GPS_RX_Altitude();
        Log_GPS_RX_DistanceTo();
    }
}
// ************************************************************************
FASTRUN void LogLongestGap()
{
    char thetext[50];
    char ms[] = " ms";
    snprintf(thetext, 45, "Longest gap: %lu", (unsigned long)GapLongest);
    strcat(thetext, ms);
    LogText(thetext, strlen(thetext), false);
}
// ************************************************************************
FASTRUN void LogAverageGap()
{
    char thetext[50];
    char ms[] = " ms";
    snprintf(thetext, 45, "Average gap: %lu", (unsigned long)GapAverage);
    strcat(thetext, ms);
    LogText(thetext, strlen(thetext), false);
}
// ************************************************************************
FASTRUN void LogBuddyChange()
{
    char OnText[] = "Buddy ON";
    char OffText[] = "Buddy OFF";
    char NudgeText[] = "Buddy Nudge";
    if (BuddyState == BUDDY_ON)
        LogText(OnText, strlen(OnText), true);
    if (BuddyState == BUDDY_OFF)
        LogText(OffText, strlen(OffText), true);
    if (BuddyState == BUDDY_NUDGE)
        LogText(NudgeText, strlen(NudgeText), true);
}
// ************************************************************************
void LogTotalLostPackets()
{
    char thetext[50];
    snprintf(thetext, 45, "Unacknowledged data packets:  %lu", (unsigned long)TotalLostPackets); // these are the packets that were not acknowledged
    LogText(thetext, strlen(thetext), false);
}
// ************************************************************************

void LogTotalGoodPackets()
{
    char thetext[56];
    snprintf(thetext, 55, "Acknowledgements received: %lu", (unsigned long)TotalGoodPackets); // TotalGoodPackets are the acknowledged packets
    LogText(thetext, strlen(thetext), false);
}

// ************************************************************************

void LogTotalRXGoodPackets()
{
    char thetext[56];
    snprintf(thetext, 55, "Data packets received at RX: %lu", (unsigned long)RXSuccessfulPackets); // RXSuccessfulPackets are the good packets counted at RX
    LogText(thetext, strlen(thetext), false);
}
// ************************************************************************
void LogTotalPacketsAttempted()
{
    char thetext[56];
    snprintf(thetext, 55, "Total packets attempted: %lu", (unsigned long)TotalPacketsAttempted); // TotalPacketsAttempted are the packets sent
    LogText(thetext, strlen(thetext), false);
}
// ************************************************************************

void LogOverallSuccessRate()
{
    char thetext[50];
    snprintf(thetext, 45, "Success rate: %d%%", GetOverallSuccessRate());
    LogText(thetext, strlen(thetext), false);
}
// ************************************************************************

FASTRUN void LogPowerOn()
{
    char Ltext[] = "Power ON";
    char sp[] = "*******************************************";
    LogText(sp, strlen(sp), false);
    LogText(Ltext, strlen(Ltext), true);
    CheckTXVolts();
    LogTXVoltsPerCell();
}

// ************************************************************************

FASTRUN void LogPowerOff()
{
    char Ltext[] = "Power OFF";
    char sp[] = "*******************************************";
    LogText(Ltext, strlen(Ltext), true);
    LogText(sp, strlen(sp), false);
}

// ************************************************************************

void LogNewRateInUse()
{

    char TheText[] = "Rate: ";
    char buf[40] = " ";
    char NB[10];
    if (DualRateInUse > 3)
        return;
    Str(NB, DualRateInUse, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    LogText(buf, sizeof(buf), true);
}

// ************************************************************************

FASTRUN void LogThisModel()
{
    char Ltext[] = "Model loaded: ";
    char thetext[75];
    strcpy(thetext, Ltext);
    strcat(thetext, ModelName);
    LogText(thetext, strlen(Ltext) + strlen(ModelName), false);
}

// ************************************************************************
void LogTouched()
{ // not used yet
}
// ************************************************************************

/******************************************************************************************************************************/
void LogEND()
{ // close log screen
    char pDataView[] = "page DataView";
    CurrentView = DATAVIEW;
    LastShowTime = 0;
    ForceDataRedisplay();
    SendCommand(pDataView);
}

/******************************************************************************************************************************/

void LogTotalRXSwaps()
{
    char TheText[] = "RX swaps: ";
    char buf[40] = " ";
    char NB[10];
    Str(NB, RadioSwaps, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    LogText(buf, sizeof(buf), false);
}

/******************************************************************************************************************************/

void LogTimeSince(char *TheText, uint32_t Time)
{

    char buf[90] = " ";
    char NB[20];
    // how many minutes and seconds since Time

    Str(NB, Time / 60000, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    strcat(buf, " minute");
    if ((Time / 60000 > 1) || (Time / 60000 == 0))
        strcat(buf, "s");
    strcat(buf, " and ");
    Str(NB, (Time % 60000) / 1000, 0);
    strcat(buf, NB);
    strcat(buf, " second");
    if ((Time % 60000) / 1000 > 1)
        strcat(buf, "s");
    LogText(buf, sizeof(buf), false);
}
/******************************************************************************************************************************/
void LogTimeSinceBoot()
{
    char TheText[] = "Time since boot: ";
    LogTimeSince(TheText, millis()); // how many minutes and seconds since boot
}
/******************************************************************************************************************************/
void LogConnectedDuration()
{
    uint32_t Duration = millis() - LedGreenMoment;
    char TheText[] = "Time connected: ";
    LogTimeSince(TheText, Duration); // how many minutes and seconds since connection
}
/*********************************************************************************************************************************/

#endif