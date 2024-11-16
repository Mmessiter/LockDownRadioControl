// *************************************** LOGFILESDISPLAY.h  *****************************************

// This is the code for the LOG FILES DISPLAY screen.
// It reads the log file a screenful at a time and displays it on the Nextion screen.
// It also allows scrolling up and down the entire log file by reading only the needed bit of it and displaying it.

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef LOGFILESDISPLAY_H
#define LOGFILESDISPLAY_H
/******************************************************************************************************************************/
// Later, might put these into definitions.h ( ... Much later!)
#define READBUFFERSIZE 2048         // Buffer to read in
#define BUFFEREDLINES 7             // 6? 15 might be too much
#define MXLINES BUFFEREDLINES * 4   // must be an integer and about 4 * BUFFEREDLINES
#define SCROLLTRIGGER 0.75          // was 0.75  ... and apparently still is!
#define MXLINELENGTH 110
#define WRAPPOINT 66                // where to word-wrap
#define MAXSEEKPOSITIONS 5000       // hope it's enough
#define FONTPOINTS 24               // 24 point font at Nextion
#define GOING_NOWHERE 0
#define GOING_UP 1
#define GOING_DOWN 2

char LogLines[MXLINES + 1][MXLINELENGTH + 1];
uint32_t SeekPosition[MAXSEEKPOSITIONS];
short StartReadLine = 0;
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
void BottomOfLogFileNEW()
{
    char Current_Y_Nextion_Label[] = "LogText.val_y";
    ScrollWithoutDisplaying = true;
    if (!LogFileOpen)
    {
        LogFileNumber = OpenTheLogFileForReading();
        LogFileOpen = true;
    }
    while (LogFileOpen)
    {
        StartReadLine += BUFFEREDLINES;
        ShowLogFileNew(ReadAFewLines());
    }
    ScrollWithoutDisplaying = false;
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
            if (!StartReadLine) return;
            if (StartReadLine <= MXLINES) // if we are almost at the top of the file (or at the top) go there!
            {
                StartReadLine = 0;
                Current_Y = 0;
            }
            else
            {
                Current_Y += (BUFFEREDLINES * FONTPOINTS);
                StartReadLine -= BUFFEREDLINES;
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
    if (ScrollWithoutDisplaying)
        return;                    // if we are just scrolling to bottom, don't display anything
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

/****************************************************************************************************************************/
File OpenTheLogFileForReading()
{
    char SearchFile[40];
    strcpy(SearchFile, "/");
    strcat(SearchFile, LogFileName);
    File fnumber = SD.open(SearchFile, FILE_READ);
    if (fnumber) LogFileOpen = true;
    return fnumber;
}

/******************************************************************************************************************/
// if there is still room in the array, this function stores the file pointer so scrolling up is possible

void StoreThisNewSeekPosition(uint16_t ThisPosition, uint32_t ThisValue)
{
    if (ThisPosition < MAXSEEKPOSITIONS)
        SeekPosition[ThisPosition] = ThisValue;
}

/******************************************************************************************************************/
// This function reads the log file buffer and builds an array of lines.
// It also stores the file seek position of each line so we can scroll back to it later.
// It returns the number of lines created.
// The lines are stored in a global 2D array of characters (LogLines[]).
// It implements fairly crude word wrapping at the WRAPPOINT and line wrapping at the MXLINELENGTH.

uint16_t BuildLinesArray(char *ReadBuffer, uint16_t BytesRead, uint32_t StartSeekPosition)
{
    uint16_t LinesCounter = 0;
    uint32_t BufferIndex = 0;
    uint16_t ColumnIndex = 0;
    uint16_t BytesSentToNextion = 0;

    for (int i = 0; i < MXLINES; ++i)
    {
        for (int j = 0; j < MXLINELENGTH; ++j)
        {
            LogLines[i][j] = 0;
        }
    }
    StoreThisNewSeekPosition((LinesCounter + StartReadLine), StartSeekPosition);
    LogLines[LinesCounter][ColumnIndex] = 0; // make sure it's a null string at start
    while (BufferIndex < BytesRead)
    { // while not at end of buffer
        while (ReadBuffer[BufferIndex] == 124)
        { // 124 is the pipe character (means new line)
            ++BufferIndex;
            ColumnIndex = 0;
            if (LinesCounter < MXLINES - 1)
            {
                ++LinesCounter;                          // increment the number of lines read if less than max
                LogLines[LinesCounter][ColumnIndex] = 0; // make sure this line is null terminated
                ++BytesSentToNextion;                    // cr (later)
                ++BytesSentToNextion;                    // lf (later)
            } // skip the 124 and zero the column
            StoreThisNewSeekPosition(LinesCounter + StartReadLine, StartSeekPosition + BufferIndex);
        }
        if (ReadBuffer[BufferIndex] >= 32)
        {
            LogLines[LinesCounter][ColumnIndex] = ReadBuffer[BufferIndex]; // copy one ASCII character
            ++BytesSentToNextion;
            ++ColumnIndex;
            LogLines[LinesCounter][ColumnIndex] = 0; // inc column & make sure this line is null terminated
            if ((ColumnIndex >= WRAPPOINT) && (LogLines[LinesCounter][ColumnIndex - 1] == 32 || LogLines[LinesCounter][ColumnIndex - 1] == '-'))
            { // Were we past the word-wrap point?
                if (LinesCounter < MXLINES - 1)
                    ++LinesCounter; // increment the number of lines read if less than max
                ColumnIndex = 0;
                StoreThisNewSeekPosition(LinesCounter + StartReadLine, StartSeekPosition + BufferIndex);
            }
        }
        ++BufferIndex;
    }
    // Look1("Bytes actually Sent To Nextion: " );
    // Look(BytesSentToNextion);
    // Look1("LinesCounter: ");
    // Look(LinesCounter);
   
    return LinesCounter;
}
/******************************************************************************************************************************/

uint16_t ReadAFewLines()
{
    char ReadBuffer[READBUFFERSIZE];

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
    
   
    if (BytesRead < READBUFFERSIZE)
    {
        CloseLogFile();
        FinalReadStartLine = StartReadLine;
    }
    // Look1("This Seek Position: ");
    // Look(StartReadLine);
    return BuildLinesArray(ReadBuffer, BytesRead, ThisSeekPosition); // build the lines array and return the number of lines
}
/******************************************************************************************************************************/

void StartLogFileView()
{
    strcpy(LogFileName, "");
    LogVIEWNew();
}
/******************************************************************************************************************************/

void LogVIEWNew() // Start log screen
{
    char Current_Y_Nextion_Label[] = "LogText.val_y";
    char fnf[] = "File not found: ";
    char LogText[] = "LogText";
    char fbuffer[40];

    SendCommand(pLogView);
    FinalReadStartLine = 0xFFFF;
    CurrentView = LOGVIEW;
    SavedCurrentView = LOGVIEW;
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
        ShowLogFileNew(0);
        strcpy(fbuffer, fnf);
        strcat(fbuffer, LogFileName);
        SendText(LogText, fbuffer);
    }
    CloseLogFile();
    SendOtherValue(Current_Y_Nextion_Label, 0);
}
#endif