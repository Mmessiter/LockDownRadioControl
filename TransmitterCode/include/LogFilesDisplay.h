// *************************************** LOGFILESDISPLAY.h  *****************************************

// This is the code for the LOG FILES DISPLAY screen AND the help screens.
// It reads the log file a screenful at a time and displays it on the Nextion screen.
// It also allows scrolling up and down the entire log file by reading only the needed bit of it and displaying it.
// The text files it reads have CR LFs that we ignore here, and we use | as line ending marker.
// Word wrap is done ONCE, up front, into a pre-wrapped temp file (/WRAP.TMP) when a file is
// opened for viewing — so the display/scroll path only ever splits on '|' and line numbering
// is identical no matter which window reads a line. (Wrapping during display made line counts
// depend on where the read-buffer boundary fell, which corrupted the seek-position map.)

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
#define MXLINELENGTH 110            // (EDIT THIS NUMBER WITH GREAT CAUTION!!!!) This is the number of characters that can fit on a line. It is not exact as it depends on the characters, but it's a good starting point. Too high and it will cause problems with word wrap, too low and it will cause too much scrolling.
#define WRAPPOINT 65                // where to word-wrap
#define MAXSEEKPOSITIONS 5000       // hope it's enough
#define FONTPOINTS 24               // 24 point font at Nextion
#define GOING_NOWHERE 0
#define GOING_UP 1
#define GOING_DOWN 2

char LogLines[MXLINES + 1][MXLINELENGTH + 1];
uint32_t SeekPosition[MAXSEEKPOSITIONS];
int16_t StartReadLine = 0;
uint32_t ThisSeekPosition = 0;
uint16_t FinalReadStartLine = 0xFFFF;
int LastShownLines = 0;                             // how many lines the current window displays (set by ShowLogFileNew)
int LogViewHeight = BUFFEREDLINES * FONTPOINTS;     // visible height of the LogText box in px. Learned exactly from the
                                                    // HMI's own report on the first scroll (content − Max_Y); this is
                                                    // the fallback until then. Needed so End can scroll to the true
                                                    // bottom of the final window: val_y = content − height.

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
 * Fixed 2026-07-02: it used to walk the file in 10-line steps (re-reading every byte ~4 times — slow),
 * and it never reset the screen's val_y scroll offset — the old offset applied to the new (short)
 * final window left the view blank or garbled. Now: one near-window stride per read to find the end,
 * then display the final window and scroll to its true bottom (content height − visible box height).
 */
void BottomOfLogFileNEW()
{
    char LogTeXt_val_y[] = "LogText.val_y";
    while (FinalReadStartLine == 0xFFFF) // end of file not discovered yet — walk forward to find it
    {
        uint16_t L = ReadAFewLines();  // reads at StartReadLine; sets FinalReadStartLine when a
                                       // window holds BOTH the end of the file AND all its lines
        if (FinalReadStartLine != 0xFFFF)
            break;
        if (L < 2)                     // safety: cannot make progress (shouldn't happen)
            break;
        StartReadLine += (L - 1);      // stride almost a whole window — every line's seek position
                                       // in this window was stored, so the next start is always valid
    }
    if (FinalReadStartLine != 0xFFFF)
        StartReadLine = FinalReadStartLine; // the window that contains the end of the file
    uint16_t L = ReadAFewLines();
    ShowLogFileNew(L);
    int y = (int)L * FONTPOINTS - LogViewHeight; // scroll to the BOTTOM of that window
    if (y < 0)
        y = 0;
    SendOtherValue(LogTeXt_val_y, y);
    Previous_Current_Y = y;
}

// ************************************************************************
// This function reads more of the log file automatically and scroll up or as needed. IT WORKS !!!!!
void LogReleasedNEW()
{
    uint8_t Direction = GOING_NOWHERE;
    char Current_Y_Nextion_Label[] = "LogText.val_y";
    int Current_Y = GetIntFromTextIn(4); // Get the current scroll position every time
    Max_Y = GetIntFromTextIn(8);         // Get the maximum scroll position every time (it changes per window)

    // Learn the LogText box's exact visible height from the HMI's own numbers:
    // content height (lines x font) minus its reported maximum scroll. This is
    // what lets the End button land exactly on the last line.
    if (Max_Y > 0 && LastShownLines > 0)
    {
        int h = LastShownLines * FONTPOINTS - Max_Y;
        if (h > 0)
            LogViewHeight = h;
    }

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
            // Decide the step FIRST and clamp it to the final window. The old
            // code advanced StartReadLine before checking and bailed out
            // without undoing it — so the last window could never be shown,
            // and every further attempt corrupted the position by +10 lines,
            // which is why scrolling UP after reaching the bottom jumped to
            // the wrong place (those seek positions were never stored).
            int next = StartReadLine + BUFFEREDLINES;
            if (FinalReadStartLine != 0xFFFF && next > (int)FinalReadStartLine)
                next = FinalReadStartLine;
            if (next == StartReadLine) // already showing the final window — nothing to fetch
            {
                Previous_Current_Y = Current_Y;
                return;
            }
            Current_Y -= (next - StartReadLine) * FONTPOINTS; // exact pixels for the lines removed at the top
            if (Current_Y < 0)
                Current_Y = 0;
            StartReadLine = next;
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
            {
                Previous_Current_Y = Current_Y;
                return;
            }
            int prev = StartReadLine;
            StartReadLine -= BUFFEREDLINES;
            if (StartReadLine < 0)
                StartReadLine = 0;
            Current_Y += (prev - StartReadLine) * FONTPOINTS; // exact pixels for the lines added at the top
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
    while (TextFileName[i] > 0)
    {
        buf[i] = TextFileName[i];
        if (buf[10] == '-')
            buf[10] = '/';
        ++i;
        buf[i] = 0;
    }
    if (ReadingaFile)
        SendText(t0, TextFileName);
    ReadingaFile = false;

    if (!InStrng(log, TextFileName))
    {
        SendCommand(b15OFF);
    }
    else
    {
        SendCommand(b15ON);
        SendText(t0, TextFileName);
    }

    strcpy(TheText, "");
    for (uint16_t i = 0; i < LinesCounter; ++i)
    {
        strcat(TheText, LogLines[i]);
        strcat(TheText, "\r\n");
    }
    SendText1(LogTeXt1, TheText); // Send it to the screen
    LastShownLines = LinesCounter; // remembered so the HMI's Max_Y report can teach us the box's visible height
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
// PRE-WRAP (Malcolm's idea, 2026-07-02): word-wrapping DURING display was the
// last source of scrolling bugs — a long line cut by a read-buffer boundary
// wraps differently depending on which window read it, so the line-number ->
// seek-position map disagreed between windows and scrolling wrapped files
// sometimes jumped or garbled. Instead we now wrap ONCE, sequentially, with
// full context, into a temporary file on the SD card where EVERY display line
// ends with '|'. The display/scroll path then only ever splits on '|' — line
// numbering is stable forever. Log files (short lines) pass through unchanged.

char WrappedFileName[] = "/WRAP.TMP";
bool ViewingWrappedCopy = false;

bool MakeWrappedCopy()
{
    char in[512];
    char out[1024];
    uint16_t col = 0;
    uint16_t oi = 0;
    int n;

    File src = OpenTextFileForReading(); // the real file (TextFileName, with its /help/ or /log/ path)
    if (!src)
    {
        LogFileOpen = false;
        return false;
    }
    SD.remove(WrappedFileName);          // FILE_WRITE appends, so start from nothing (TINYCARD alias is defined in SDFiles.h, included after this file)
    File dst = SD.open(WrappedFileName, FILE_WRITE);
    if (!dst)
    {
        src.close();
        LogFileOpen = false;
        return false;
    }
    while ((n = src.read(in, sizeof(in))) > 0)
    {
        for (int i = 0; i < n; ++i)
        {
            char c = in[i];
            if (c == '|')
            {
                out[oi++] = '|';
                col = 0;
            }
            else if ((uint8_t)c >= 32) // CR/LFs dropped, exactly as the display always did
            {
                if (c == '"')
                    c = '\''; // Nextion txt="..." has NO escape for an embedded double quote —
                              // it terminates the command string and the display REJECTS the
                              // whole update. FRONT.TXT ("STOP FLYING!") and BUDDY.TXT contain
                              // one, which made every window past it appear frozen: scrolling
                              // "stopped" and the Bottom button "did nothing" on exactly those
                              // two files. Swap for an apostrophe in the display copy.
                out[oi++] = c;
                ++col;
                if (WrapNow(col, (uint8_t)c) || col >= MXLINELENGTH - 2) // same wrap rules as before
                {
                    out[oi++] = '|';
                    col = 0;
                }
            }
            if (oi >= sizeof(out) - 4)
            {
                dst.write(out, oi);
                oi = 0;
            }
        }
        KickTheDog(); // big file: keep the watchdog fed while we copy
    }
    if (oi)
        dst.write(out, oi);
    dst.close();
    src.close();
    LogFileOpen = false;
    return true;
}

// Open whichever file the viewer should actually read: the pre-wrapped copy
// when one was made, else the original.
File OpenViewFile()
{
    if (ViewingWrappedCopy)
    {
        File f = SD.open(WrappedFileName, FILE_READ);
        if (f)
            LogFileOpen = true;
        return f;
    }
    return OpenTextFileForReading();
}

/******************************************************************************************************************/
// This function reads the log file buffer and builds an array of lines.
// It also stores the seek positions so scrolling up is possible.

// Set when a buffer held MORE lines than the window can show (MXLINES). Vital
// distinction: reaching end-of-FILE in the buffer is NOT the final window if
// the window itself overflowed — lines beyond the cap still need windows of
// their own. Ignoring this made short-line files (e.g. the front-screen help,
// >40 wrapped lines in under one read buffer) truncate at 40 lines with the
// tail unreachable — the "can't scroll to the end on SOME help files" bug.
bool WindowOverflowed = false;

uint16_t BuildLinesArray(char *ReadBuffer, uint16_t BytesRead, uint32_t StartSeekPosition)
{
    uint16_t LinesCounter = 0;
    uint32_t BufferIndex = 0;
    uint16_t ColumnIndex = 0;

    WindowOverflowed = false;
    for (int i = 0; i < MXLINES; ++i)
        for (int j = 0; j < MXLINELENGTH; ++j)
            LogLines[i][j] = 0;
    StoreThisNewSeekPosition((LinesCounter + StartReadLine), StartSeekPosition);
    while (BufferIndex < BytesRead)
    {
        while (BufferIndex < BytesRead && ReadBuffer[BufferIndex] == '|') // skip the | and zero the column
        {
            ++BufferIndex;
            ColumnIndex = 0;
            if (LinesCounter < MXLINES - 1)
            {
                ++LinesCounter;
                StoreThisNewSeekPosition(LinesCounter + StartReadLine, StartSeekPosition + BufferIndex);
            }
            else
            {
                // Window full. STOP consuming — the old code kept going and
                // mangled the last row with the rest of the buffer. All
                // MXLINES lines are complete and intact; the remainder gets
                // its own window(s) when the user scrolls on.
                WindowOverflowed = true;
                return MXLINES;
            }
        }
        if (BufferIndex < BytesRead && ReadBuffer[BufferIndex] >= 32)
        {
            // No word-wrap decisions here any more: the file we're reading is
            // PRE-WRAPPED (every display line already ends with '|'), which is
            // what makes line numbering stable across windows. The only guard
            // left is the hard row limit, in case of a corrupt/foreign file.
            if (ColumnIndex < MXLINELENGTH - 1)
            {
                LogLines[LinesCounter][ColumnIndex] = ReadBuffer[BufferIndex]; // get one character
                ++ColumnIndex;
                LogLines[LinesCounter][ColumnIndex] = 0;
            }
        }
        ++BufferIndex;
    }
    if (LogLines[LinesCounter][0]) // count the in-progress final line so the file's true last line displays
        ++LinesCounter;
    return LinesCounter;
}
/******************************************************************************************************************************/

uint16_t ReadAFewLines()
{
    char ReadBuffer[READBUFFERSIZE + 50]; // a bit of extra just in case

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
        LogFileNumber = OpenViewFile(); // if not open, open it (the pre-wrapped copy when one exists)
    }
    LogFileNumber.seek(ThisSeekPosition);                     // seek to the position
    int GotBytes = LogFileNumber.read(ReadBuffer, READBUFFERSIZE); // read in the buffer
    if (GotBytes < 0)
        GotBytes = 0; // a failed read returns -1; as a uint16_t that became 65535 "bytes" of stack garbage
    uint16_t BytesRead = (uint16_t)GotBytes;

    if (BytesRead < READBUFFERSIZE) // if we read less than the buffer size, we are at the end of the file
    {
        CloseLogFile(); // (re-opened on demand by the next read)
    }
    ReadBuffer[BytesRead] = 0; // null-terminate AFTER the data. Terminating at [BytesRead - 1] chopped the
                               // file's final byte — usually the last '|' — which made the last log line
                               // vanish; and with a 0-byte read it wrote one byte BEFORE the buffer.
    uint16_t Lines = BuildLinesArray(ReadBuffer, BytesRead, ThisSeekPosition);
    // This window is the FINAL one only if the buffer held the end of the file
    // AND every line of the buffer fitted into the window. An overflowed window
    // still has lines beyond its cap — declaring it final made those lines
    // unreachable (the front-screen-help "can't scroll to the end" bug).
    if (BytesRead < READBUFFERSIZE && !WindowOverflowed)
        FinalReadStartLine = StartReadLine;
    return Lines;
}
/******************************************************************************************************************************/

void StartLogFileView() // This is the entry point
{
    strcpy(TextFileName, "");
    LogVIEWNew();
}
/******************************************************************************************************************************/

void LogVIEWNew() // Start log screen
{
    char Current_Y_Nextion_Label[] = "LogText.val_y"; // the Y position of the log text on the Nextion screen
    char fnf[] = "File not found: ";
    char LogText[] = "LogText"; // the label on the Nextion screen
    char fbuffer[60];

    SendCommand(pLogView); // Show the right view
    FinalReadStartLine = 0xFFFF;
    CurrentView = LOGVIEW;
    ClearFilesList();
    MakeTextFileName();
    CloseLogFile();
    for (uint16_t i = 0; i < MAXSEEKPOSITIONS; ++i)
        SeekPosition[i] = 0; // clear all seekpositions
    StartReadLine = 0;
    ViewingWrappedCopy = MakeWrappedCopy(); // pre-wrap into /WRAP.TMP — display then only splits on '|'
    if (ViewingWrappedCopy)
        LogFileNumber = OpenViewFile();
    if (ViewingWrappedCopy && LogFileNumber)
    {
        ShowLogFileNew(ReadAFewLines()); // builds the lines array and uses it to display the log file
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
    Previous_Current_Y = 0; // fresh file starts at the top — stale value here made the
                            // first scroll gesture's direction detection wrong
}
#endif