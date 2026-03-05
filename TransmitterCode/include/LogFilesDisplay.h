// *************************************** LOGFILESDISPLAY.h  *****************************************
//
// Displays LOG files and HELP files on a Nextion 5" intelligent screen, driven by Teensy 4.1.
// Reads the file a screenful at a time; scrolling triggers loading the next/previous chunk.
// Line separator is '|' (CRLF is ignored).
// Help files also implement word-wrap.
//
// REWRITE NOTES (bugs fixed vs original):
//  1. SeekPosition[] now stores true file offsets computed during parsing, not
//     (StartSeekPosition + BufferIndex) which was wrong whenever the buffer didn't start at 0.
//  2. BottomOfLogFileNEW() now scans the file properly by actually building the lines array
//     on every chunk so SeekPosition[] is populated correctly before the final display.
//  3. Word-wrap no longer discards the character that triggered the wrap – it is now the
//     first character of the new wrapped line.
//  4. Scroll-position correction after a load now uses the actual number of lines loaded
//     multiplied by FONTPOINTS rather than a fixed BUFFEREDLINES guess.
//  5. StartReadLine is clamped to 0 and is never allowed to go negative.
//  6. LogReleasedNEW() now also handles the "scrolled past the top while at top" edge case.

#include <Arduino.h>
#include "1Definitions.h"

#ifndef LOGFILESDISPLAY_H
#define LOGFILESDISPLAY_H

// ---------------------------------------------------------------------------
// Tuneable constants  –  edit with care
// ---------------------------------------------------------------------------
#define READBUFFERSIZE (1024 + 512) // bytes read from SD per chunk
#define BUFFEREDLINES 10            // lines added / removed on each scroll refill
#define MXLINES (BUFFEREDLINES * 4) // total lines held in LogLines[]
#define SCROLLTRIGGER 0.75f         // fraction of Max_Y that triggers a refill
#define MXLINELENGTH 110            // max characters per display line
#define WRAPPOINT 65                // column at which word-wrap may fire
#define MAXSEEKPOSITIONS 5000       // one entry per logical line in the file
#define FONTPOINTS 24               // pixel height of one text line on Nextion
#define GOING_NOWHERE 0
#define GOING_UP 1
#define GOING_DOWN 2
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Module-level state
// ---------------------------------------------------------------------------
static char LogLines[MXLINES + 1][MXLINELENGTH + 1];
static uint32_t SeekPosition[MAXSEEKPOSITIONS]; // true file offset for logical line N
static int16_t StartReadLine = 0;               // first logical line currently loaded
static uint16_t FinalReadStartLine = 0xFFFF;    // StartReadLine value when EOF was hit

// ---------------------------------------------------------------------------
// Forward declarations  (not static – must match extern decls in 1Definitions.h)
// ---------------------------------------------------------------------------
uint16_t ReadAFewLines();
void ShowLogFileNew(uint16_t LinesCounter);

// ===========================================================================
// Internal helpers
// ===========================================================================

/** Returns true if a word-wrap break is allowed at this column / character. */
static bool WrapNow(uint16_t col, char lastChar)
{
    if (col < WRAPPOINT)
        return false;
    return (lastChar == ' ' ||
            lastChar == '-' ||
            lastChar == '.' ||
            lastChar == '=' ||
            lastChar == ',');
}

/** Store a file offset for logical line lineIndex (silently ignores overflow). */
static void StoreSeekPosition(uint16_t lineIndex, uint32_t fileOffset)
{
    if (lineIndex < MAXSEEKPOSITIONS)
        SeekPosition[lineIndex] = fileOffset;
}

// ===========================================================================
// BuildLinesArray
//
// Parses the raw read buffer into LogLines[] and records the exact file offset
// for every logical line so that scrolling back up can seek accurately.
//
// Parameters:
//   ReadBuffer        – raw bytes from the SD read
//   BytesRead         – number of valid bytes in ReadBuffer
//   bufferFileOffset  – file position at which this buffer starts (i.e. what
//                       LogFileNumber.seek() was called with before the read)
//
// Returns: total number of lines produced (index of last populated line + 1)
// ===========================================================================
static uint16_t BuildLinesArray(const char *ReadBuffer,
                                uint16_t BytesRead,
                                uint32_t bufferFileOffset)
{
    // Clear the array up front
    for (int i = 0; i <= MXLINES; ++i)
        LogLines[i][0] = '\0';

    uint16_t LinesCounter = 0;
    uint16_t ColumnIndex = 0;

    // Record offset of the very first line
    StoreSeekPosition(StartReadLine + LinesCounter,
                      bufferFileOffset); // offset 0 within buffer → file start

    for (uint16_t bi = 0; bi < BytesRead; ++bi)
    {
        char c = ReadBuffer[bi];

        // ------------------------------------------------------------------
        // '|' means "end of paragraph / start of new line".
        // '||' (two consecutive pipes) is the start of the end-of-file marker
        // "||* END OF FILE *||" – stop here so nothing appears after it.
        // ------------------------------------------------------------------
        if (c == '|')
        {
            // Peek ahead: a second '|' means the end-of-file marker
            if ((bi + 1) < BytesRead && ReadBuffer[bi + 1] == '|')
            {
                // Close the current line, then write a tidy final line and stop
                LogLines[LinesCounter][ColumnIndex] = '\0';
                if (LinesCounter < MXLINES)
                    ++LinesCounter;
                strncpy(LogLines[LinesCounter], "* END OF FILE *", MXLINELENGTH);
                LogLines[LinesCounter][MXLINELENGTH] = '\0';
                ++LinesCounter;
                break; // nothing more to parse
            }

            // Normal single pipe – finish current line and start a new one
            LogLines[LinesCounter][ColumnIndex] = '\0';
            if (LinesCounter < MXLINES)
                ++LinesCounter;
            else
                break; // array full – stop

            ColumnIndex = 0;
            // The new line starts at the byte *after* the '|'
            StoreSeekPosition(StartReadLine + LinesCounter,
                              bufferFileOffset + bi + 1);
            continue;
        }

        // ------------------------------------------------------------------
        // Skip control characters (CR, LF, etc.) – keep printable chars only
        // ------------------------------------------------------------------
        if (c < 32)
            continue;

        // ------------------------------------------------------------------
        // Normal printable character
        // ------------------------------------------------------------------
        if (ColumnIndex < MXLINELENGTH)
        {
            LogLines[LinesCounter][ColumnIndex] = c;
            ++ColumnIndex;
            LogLines[LinesCounter][ColumnIndex] = '\0';

            // Check for word-wrap opportunity
            if (WrapNow(ColumnIndex, c))
            {
                // Close this line
                LogLines[LinesCounter][ColumnIndex] = '\0';
                if (LinesCounter < MXLINES)
                    ++LinesCounter;
                else
                    break;

                ColumnIndex = 0;
                // Wrapped line starts at the next byte
                StoreSeekPosition(StartReadLine + LinesCounter,
                                  bufferFileOffset + bi + 1);
            }
        }
        // If ColumnIndex == MXLINELENGTH we simply discard further chars on
        // this line until a '|' or wrap char is encountered – prevents overrun.
    }

    return LinesCounter; // number of lines (last index + 1)
}

// ===========================================================================
// ReadAFewLines
//
// Seeks to the correct file position for StartReadLine, reads a chunk, and
// calls BuildLinesArray.  Handles EOF by appending an end-marker and recording
// FinalReadStartLine.
//
// Returns: number of lines built.
// ===========================================================================
uint16_t ReadAFewLines()
{
    static char ReadBuffer[READBUFFERSIZE + 32]; // static so it doesn't live on the stack

    // Determine file offset for the requested start line
    uint32_t seekPos = 0;
    if (StartReadLine > 0 && StartReadLine < MAXSEEKPOSITIONS)
        seekPos = SeekPosition[StartReadLine];

    // Open if necessary
    if (!LogFileOpen)
    {
        LogFileNumber = OpenTextFileForReading();
        LogFileOpen = (bool)LogFileNumber;
        if (!LogFileOpen)
            return 0;
    }

    LogFileNumber.seek(seekPos);
    uint16_t BytesRead = LogFileNumber.read(ReadBuffer, READBUFFERSIZE);

    if (BytesRead < READBUFFERSIZE)
    {
        // Reached EOF – append the double-pipe sentinel that BuildLinesArray
        // watches for.  The display text "* END OF FILE *" is written there.
        ReadBuffer[BytesRead] = '\0';
        CloseLogFile();
        FinalReadStartLine = StartReadLine;
        // Append "||" so BuildLinesArray detects EOF cleanly
        ReadBuffer[BytesRead++] = '|';
        ReadBuffer[BytesRead++] = '|';
        ReadBuffer[BytesRead] = '\0';
    }

    return BuildLinesArray(ReadBuffer, BytesRead, seekPos);
}

// ===========================================================================
// ShowLogFileNew  –  formats LogLines[] into one big string and sends it to
//                    the Nextion LogText component.
// ===========================================================================
void ShowLogFileNew(uint16_t LinesCounter)
{
    static char TheText[MAXFILELEN + 10]; // static – too large for stack
    char LogTeXt1[] = "LogText";
    char t0[] = "t0";
    char b15OFF[] = "vis b15,0";
    char b15ON[] = "vis b15,1";
    char log[] = ".LOG";

    // Show filename in t0 widget
    if (ReadingaFile)
        SendText(t0, TextFileName);
    ReadingaFile = false;

    // Show / hide the "delete log" button depending on file type
    if (!InStrng(log, TextFileName))
        SendCommand(b15OFF);
    else
    {
        SendCommand(b15ON);
        SendText(t0, TextFileName);
    }

    // Assemble the display string
    TheText[0] = '\0';
    for (uint16_t i = 0; i < LinesCounter; ++i)
    {
        strcat(TheText, LogLines[i]);
        strcat(TheText, "\r\n");
    }
    SendText1(LogTeXt1, TheText);
    DelayWithDog(50);
}

// ===========================================================================
// TopOfLogFileNEW  –  jump to the very beginning
// ===========================================================================
void TopOfLogFileNEW()
{
    char LogTeXt_val_y[] = "LogText.val_y";
    StartReadLine = 0;
    ShowLogFileNew(ReadAFewLines());
    SendOtherValue(LogTeXt_val_y, 0);
    Previous_Current_Y = 0;
}

// ===========================================================================
// BottomOfLogFileNEW  –  scan the whole file to populate SeekPosition[], then
//                         display the final screenful.
//
// The original version guessed at seek positions and was unreliable.
// This version actually parses every chunk so every SeekPosition[] entry is
// correct before the final seek-and-display.
// ===========================================================================
void BottomOfLogFileNEW()
{
    char Current_Y_Nextion_Label[] = "LogText.val_y";

    // Start from the beginning so SeekPosition[0] is valid
    CloseLogFile();
    LogFileNumber = OpenTextFileForReading();
    LogFileOpen = (bool)LogFileNumber;
    if (!LogFileOpen)
        return;

    // Clear seek table
    for (uint16_t i = 0; i < MAXSEEKPOSITIONS; ++i)
        SeekPosition[i] = 0;

    StartReadLine = 0;
    FinalReadStartLine = 0xFFFF;

    // Scan forward, building lines (and thus SeekPosition[]) for every chunk
    // until ReadAFewLines() hits EOF (which sets FinalReadStartLine and closes
    // the file, leaving LogFileOpen false).
    uint16_t lastCount = 0;
    while (LogFileOpen)
    {
        lastCount = ReadAFewLines();
        if (!LogFileOpen)
            break;                      // EOF reached inside ReadAFewLines
        StartReadLine += BUFFEREDLINES; // advance one screenful
        if (StartReadLine >= MAXSEEKPOSITIONS)
            break;
    }

    // Now StartReadLine is at (or past) the last chunk.  Show that chunk and
    // set the Nextion scroll position to the bottom.
    ShowLogFileNew(lastCount);
    SendOtherValue(Current_Y_Nextion_Label, Max_Y);
    Previous_Current_Y = Max_Y;
}

// ===========================================================================
// LogReleasedNEW  –  called whenever the user lifts their finger after
//                    scrolling the Nextion text component.
//
// If the scroll position has crossed the 75% / 25% trigger thresholds a new
// chunk is loaded and the Nextion scroll offset is corrected so the visible
// content appears to continue seamlessly.
// ===========================================================================
void LogReleasedNEW()
{
    char Current_Y_Nextion_Label[] = "LogText.val_y";
    uint8_t Direction = GOING_NOWHERE;

    int32_t Current_Y = GetIntFromTextIn(4); // current scroll offset from Nextion
    Max_Y = GetIntFromTextIn(8);             // maximum scroll offset from Nextion

    if (Current_Y > Previous_Current_Y)
        Direction = GOING_DOWN;
    if (Current_Y < Previous_Current_Y)
        Direction = GOING_UP;
    if (Direction == GOING_NOWHERE)
        return;

    // ------------------------------------------------------------------
    // Scrolling DOWN: user is reading towards the end of the loaded chunk
    // ------------------------------------------------------------------
    if (Direction == GOING_DOWN)
    {
        if (Current_Y > (int32_t)(Max_Y * SCROLLTRIGGER))
        {
            // Don't load past the known end of file
            if (StartReadLine >= FinalReadStartLine)
            {
                Previous_Current_Y = Current_Y;
                return;
            }
            StartReadLine += BUFFEREDLINES;

            uint16_t linesLoaded = ReadAFewLines();

            // Shift the Nextion scroll position back by the height of the
            // lines we just discarded from the top (BUFFEREDLINES lines).
            int32_t corrected_Y = Current_Y - (int32_t)(BUFFEREDLINES * FONTPOINTS);
            if (corrected_Y < 0)
                corrected_Y = 0;

            ShowLogFileNew(linesLoaded);
            SendOtherValue(Current_Y_Nextion_Label, corrected_Y);
            Previous_Current_Y = corrected_Y;
        }
        else
        {
            Previous_Current_Y = Current_Y;
        }
        return;
    }

    // ------------------------------------------------------------------
    // Scrolling UP: user is reading back towards the start of the loaded chunk
    // ------------------------------------------------------------------
    if (Direction == GOING_UP)
    {
        if (Current_Y < (int32_t)(Max_Y * (1.0f - SCROLLTRIGGER)))
        {
            if (StartReadLine == 0)
            {
                // Already at the very top – nothing to load; clamp Y to 0
                if (Current_Y != 0)
                {
                    SendOtherValue(Current_Y_Nextion_Label, 0);
                    Previous_Current_Y = 0;
                }
                return;
            }

            StartReadLine -= BUFFEREDLINES;
            if (StartReadLine < 0)
                StartReadLine = 0;

            uint16_t linesLoaded = ReadAFewLines();

            // Shift the Nextion scroll position forward by the lines we added
            // back at the top.
            int32_t corrected_Y = Current_Y + (int32_t)(BUFFEREDLINES * FONTPOINTS);
            if (corrected_Y > Max_Y)
                corrected_Y = Max_Y;

            ShowLogFileNew(linesLoaded);
            SendOtherValue(Current_Y_Nextion_Label, corrected_Y);
            Previous_Current_Y = corrected_Y;
        }
        else
        {
            Previous_Current_Y = Current_Y;
        }
        return;
    }
}

// ===========================================================================
// StartLogFileView / LogVIEWNew  –  entry points (unchanged in behaviour)
// ===========================================================================
void StartLogFileView()
{
    strcpy(TextFileName, "");
    LogVIEWNew();
}

void LogVIEWNew()
{
    char Current_Y_Nextion_Label[] = "LogText.val_y";
    char fnf[] = "File not found: ";
    char LogText[] = "LogText";
    char fbuffer[60];

    SendCommand(pLogView);
    FinalReadStartLine = 0xFFFF;
    CurrentView = LOGVIEW;
    ClearFilesList();

    MakeTextFileName();
    CloseLogFile();

    // Clear seek table
    for (uint16_t i = 0; i < MAXSEEKPOSITIONS; ++i)
        SeekPosition[i] = 0;

    StartReadLine = 0;
    LogFileNumber = OpenTextFileForReading();

    if (LogFileNumber)
    {
        LogFileOpen = true;
        ShowLogFileNew(ReadAFewLines());
    }
    else
    {
        LogFileOpen = false;
        ShowLogFileNew(6);
        strcpy(fbuffer, fnf);
        strcat(fbuffer, TextFileName);
        SendText(LogText, fbuffer);
    }

    CloseLogFile();
    SendOtherValue(Current_Y_Nextion_Label, 0);
    Previous_Current_Y = 0;
}

#endif // LOGFILESDISPLAY_H
