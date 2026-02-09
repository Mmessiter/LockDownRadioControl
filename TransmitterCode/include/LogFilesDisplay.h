// *************************************** LOGFILESDISPLAY.h  *****************************************

// This is the code for the LOG FILES DISPLAY screen AND the help screens.
// It reads the log file a screenful at a time and displays it on the Nextion screen.
// It also allows scrolling up and down the entire log file by reading only the needed bit of it and displaying it.
// The text files it reads have CR LFs that we ignore here, and we use | as line ending marker.
// Here we also implement word wrap that adds complexity when counting lines.

#include <Arduino.h>
#include "1Definitions.h"
#ifndef LOGFILESDISPLAY_H
#define LOGFILESDISPLAY_H
/******************************************************************************************************************************/
// Later, might put these into definitions.h ( ... Much later!)
#define READBUFFERSIZE (1024 + 512) // (EDIT THESE NUMBERS WITH GREAT CAUTION!!!!)
#define BUFFEREDLINES 10            // (EDIT THESE NUMBERS WITH GREAT CAUTION!!!!)
#define MXLINES BUFFEREDLINES * 4   // (EDIT THESE NUMBERS WITH GREAT CAUTION!!!!)
#define SCROLLTRIGGER 0.75          // (EDIT THESE NUMBERS WITH GREAT CAUTION!!!!)
#define MXLINELENGTH 110
#define WRAPPOINT 65          // where to word-wrap
#define MAXSEEKPOSITIONS 5000 // hope it's enough
#define FONTPOINTS 24         // 24 point font at Nextion
#define GOING_NOWHERE 0
#define GOING_UP 1
#define GOING_DOWN 2

char LogLines[MXLINES + 1][MXLINELENGTH + 1];
uint32_t SeekPosition[MAXSEEKPOSITIONS];
int16_t StartReadLine = 0;
uint32_t ThisSeekPosition = 0;
uint16_t FinalReadStartLine = 0xFFFF;

// ******************************************************************************************************************************/
void TopOfLogFileNEW()
{
    char LogTeXt_val_y[] = "LogText.val_y";
    StartReadLine = 0;
    ShowLogFileNew(ReadAFewLines());
    SendOtherValue(LogTeXt_val_y, 0);
    Previous_Current_Y = 0;
}
/******************************************************************************************************************************/
/**
 * This function scrolls to the bottom of the log file and displays the most recent log entries.
 */
void BottomOfLogFileNEW() // this isn't perfect but it usually works Ok ...
{
    char Current_Y_Nextion_Label[] = "LogText.val_y";
    CloseLogFile();
    LogFileNumber = OpenTheLogFileForReading();
    LogFileOpen = true;
    while (LogFileOpen)
    {
        StartReadLine += BUFFEREDLINES;
        ReadAFewLines();
    }
    ShowLogFileNew(ReadAFewLines());
    SendOtherValue(Current_Y_Nextion_Label, Max_Y);
    Previous_Current_Y = Max_Y;
}

// ************************************************************************
// This function reads more of the log file automatically and scroll up or as needed. IT WORKS !!!!!
void LogReleasedNEW()
{
    uint8_t Direction = GOING_NOWHERE;
    char Current_Y_Nextion_Label[] = "LogText.val_y";
    int Current_Y = GetIntFromTextIn(4); // Get the current scroll position every time
    Max_Y = GetIntFromTextIn(8);         // Get the maximum scroll position once only
    if (Current_Y > Previous_Current_Y)
        Direction = GOING_DOWN;
    if (Current_Y < Previous_Current_Y)
        Direction = GOING_UP;
    if (Current_Y == Previous_Current_Y)
        return;

    if (Direction == GOING_DOWN)
    {
        if (Current_Y > (Max_Y * SCROLLTRIGGER))
        {
            StartReadLine += BUFFEREDLINES;
            if (StartReadLine >= FinalReadStartLine)
                return;
            Current_Y -= (BUFFEREDLINES * FONTPOINTS);
            ShowLogFileNew(ReadAFewLines());
            SendOtherValue(Current_Y_Nextion_Label, Current_Y);
            Previous_Current_Y = Current_Y;
            return;
        }
    }
    if (Direction == GOING_UP)
    {
        if (Current_Y < (Max_Y * (1 - SCROLLTRIGGER)))
        {
            if (!StartReadLine)
                return;
            Current_Y += (BUFFEREDLINES * FONTPOINTS);
            StartReadLine -= BUFFEREDLINES;
            if (StartReadLine < 0)
            {
                StartReadLine = 0;
                Current_Y = 0;
            }
            ShowLogFileNew(ReadAFewLines());
            SendOtherValue(Current_Y_Nextion_Label, Current_Y);
            Previous_Current_Y = Current_Y;
            return;
        }
    }
    Previous_Current_Y = Current_Y;
}
/****************************************************************************************************************************/
void ShowLogFileNew(uint16_t LinesCounter)
{
    char TheText[MAXFILELEN + 10]; // MAX = 5K or so
    char LogTeXt1[] = "LogText";
    char t0[] = "t0";
    char buf[40] = " ";
    uint8_t i = 0;
    char b15OFF[] = "vis b15,0";
    char b15ON[] = "vis b15,1";
    char log[] = ".LOG";

    strcpy(buf, "");
    while (LogFileName[i] > 0)
    {
        buf[i] = LogFileName[i];
        if (buf[10] == '-')
            buf[10] = '/';
        ++i;
        buf[i] = 0;
    }
    if (ReadingaFile)
        SendText(t0, LogFileName);
    ReadingaFile = false;

    if (!InStrng(log, LogFileName))
    {
        SendCommand(b15OFF);
    }
    else
    {
        SendCommand(b15ON);
        SendText(t0, LogFileName);
    }

    strcpy(TheText, "");
    for (uint16_t i = 0; i < LinesCounter; ++i)
    {
        strcat(TheText, LogLines[i]);
        strcat(TheText, "\r\n");
    }
    SendText1(LogTeXt1, TheText); // Send it to the screen
   
    DelayWithDog(50);
}



/******************************************************************************************************************/
// if there is still room in the array, this function stores the file pointer so scrolling up is possible

void StoreThisNewSeekPosition(uint16_t ThisPosition, uint32_t ThisValue)
{
    if (ThisPosition < MAXSEEKPOSITIONS)
        SeekPosition[ThisPosition] = ThisValue;
}

// ****************************************************************************************
bool WrapNow(uint16_t ColumnIndex, uint8_t LastChar)
{
    if (ColumnIndex >= WRAPPOINT)
    {
        if (LastChar == 32) // if last char is a space, we can wrap now
            return true;
        if (LastChar == '-') // if last char is a hyphen, we can wrap now
            return true;
        if (LastChar == '.') // if last char is a period, we can wrap now
            return true;
        if (LastChar == '=') // if last char is an equals sign, we can wrap now
            return true;
        if (LastChar == ',') // if last char is a comma, we can wrap now
            return true;
    }
    return false;
}
/******************************************************************************************************************/
// This function reads the log file buffer and builds an array of lines.
// It also stores the seek positions so scrolling up is possible.

uint16_t BuildLinesArray(char *ReadBuffer, uint16_t BytesRead, uint32_t StartSeekPosition)
{
    uint16_t LinesCounter = 0;
    uint32_t BufferIndex = 0;
    uint16_t ColumnIndex = 0;

    for (int i = 0; i < MXLINES; ++i)
        for (int j = 0; j < MXLINELENGTH; ++j)
            LogLines[i][j] = 0;
    StoreThisNewSeekPosition((LinesCounter + StartReadLine), StartSeekPosition);
    while (BufferIndex < BytesRead)
    {
        while (ReadBuffer[BufferIndex] == '|') // skip the | and zero the column
        {
            ++BufferIndex;
            ColumnIndex = 0;
            if (LinesCounter < MXLINES - 1)
                ++LinesCounter;
            else
                break;
            StoreThisNewSeekPosition(LinesCounter + StartReadLine, StartSeekPosition + BufferIndex);
        }
        if (ReadBuffer[BufferIndex] >= 32)
        {
            LogLines[LinesCounter][ColumnIndex] = ReadBuffer[BufferIndex]; // get one character
            ++ColumnIndex;
            LogLines[LinesCounter][ColumnIndex] = 0;

            if (WrapNow(ColumnIndex, LogLines[LinesCounter][ColumnIndex - 1]))
            {
                if (LinesCounter < MXLINES - 1)
                    ++LinesCounter;
                ColumnIndex = 0;
                StoreThisNewSeekPosition(LinesCounter + StartReadLine, StartSeekPosition + BufferIndex);
            }
        }
        ++BufferIndex;
    }
    return LinesCounter;
}
/******************************************************************************************************************************/

uint16_t ReadAFewLines()
{
    char ReadBuffer[READBUFFERSIZE];
    char EndMarker[] = "||* END OF FILE *||";

    if (!StartReadLine)
    {
        ThisSeekPosition = 0; // if at top of file
    }
    else
    {
        ThisSeekPosition = SeekPosition[StartReadLine]; // else get the seek position
    }
    if (!LogFileOpen)
    {
        LogFileNumber = OpenTheLogFileForReading(); // if not open, open it
    }
    LogFileNumber.seek(ThisSeekPosition);                                // seek to the position
    uint16_t BytesRead = LogFileNumber.read(ReadBuffer, READBUFFERSIZE); // read in the buffer

    if (BytesRead < READBUFFERSIZE) // if we read less than the buffer size, we are at the end of the file
    {
        ReadBuffer[BytesRead] = 0; // null-terminate the buffer
        CloseLogFile();
        FinalReadStartLine = StartReadLine;
        strcat(ReadBuffer, EndMarker);
        BytesRead += (strlen(EndMarker));
        ReadBuffer[BytesRead] = 0; // null-terminate the buffer
    }
    return BuildLinesArray(ReadBuffer, BytesRead, ThisSeekPosition); // build the lines array and return the number of lines
}
/******************************************************************************************************************************/

void StartLogFileView() // This is the entry point
{
    strcpy(LogFileName, "");
    LogVIEWNew();
}
/******************************************************************************************************************************/

void LogVIEWNew() // Start log screen
{
    char Current_Y_Nextion_Label[] = "LogText.val_y"; // the Y position of the log text on the Nextion screen
    char fnf[] = "File not found: ";
    char LogText[] = "LogText"; // the label on the Nextion screen
    char fbuffer[40];

    SendCommand(pLogView); // Show the right view
    FinalReadStartLine = 0xFFFF;
    CurrentView = LOGVIEW;
    ClearFilesList();
    MakeLogFileName();
    CloseLogFile();
    for (uint16_t i = 0; i < MAXSEEKPOSITIONS; ++i)
        SeekPosition[i] = 0; // clear all seekpositions
    StartReadLine = 0;
    LogFileNumber = OpenTheLogFileForReading();
    if (LogFileNumber)
    {
        ShowLogFileNew(ReadAFewLines()); // builds the lines array and uses it to display the log file
       
    }
    else
    {
        LogFileOpen = false;
        ShowLogFileNew(6);
        strcpy(fbuffer, fnf);
        strcat(fbuffer, LogFileName);
        SendText(LogText, fbuffer);
    }
    CloseLogFile();
    SendOtherValue(Current_Y_Nextion_Label, 0);
}
#endif