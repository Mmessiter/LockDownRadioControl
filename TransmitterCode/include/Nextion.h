#include <Arduino.h>
#include "1Definitions.h"
#ifndef NEXTION_H
#define NEXTION_H

/*********************************************************************************************************************************/
//                        NEXTION functions
/*********************************************************************************************************************************/
void GetReturnCode(char *tbox)
{
    while (NEXTION.available())
    {
        delayMicroseconds(70);
        NEXTION.read();
    }
}
/*********************************************************************************************************************************/
void ClearNextionCommand()
{
    strcpy(NextionCommand, "");
}
/*********************************************************************************************************************************/
void BuildNextionCommand(char *cmd) // these strings can be <= 255 bytes but here we limit to 127 to avoid delays etc.
{
    if (strlen(NextionCommand) + strlen(cmd) + 3 >= MAXNEXTIONCOMMANDLENGTH) // If too long, send what we have and start building again.
    {
        SendCommand(NextionCommand);
        ClearNextionCommand();
    }
    if (strlen(NextionCommand) > 0)
        strcat(NextionCommand, "\xFF\xFF\xFF"); // add ending beforehand if this is not  first
    strcat(NextionCommand, cmd);
}

/*********************************************************************************************************************************/
void BuildValue(char *nbox, int value)
{ // Same as SendValue only delayed send
    char Val[] = ".val=";
    char CB[100];
    char NB[25];
    strcpy(CB, nbox);
    strcat(CB, Val);
    strcat(CB, Str(NB, value, 0));
    BuildNextionCommand(CB);
}
/*********************************************************************************************************************************/
// ClaudeFix-2-7-2026 Bounded, sanitising copy for anything sent inside a Nextion txt="..."
// assignment. Two characters are LETHAL in there: an embedded double quote
// terminates the string so the display SILENTLY REJECTS the whole command
// (the frozen-help-screen bug), and a 0xFF byte ends the command early,
// letting the remaining bytes execute as a NEW command (injection from
// garbage telemetry). maxroom includes the terminating NUL. CR/LF pass
// through (the log viewer needs them).
void CopyTextForNextion(char *dst, const char *src, uint16_t maxroom)
{
    uint16_t i = 0;
    while (src[i] && (i + 1) < maxroom)
    {
        char c = src[i];
        if (c == '"')
            c = '\'';
        if ((uint8_t)c >= 0xFE)
            c = ' ';
        dst[i] = c;
        ++i;
    }
    dst[i] = 0;
}
/*********************************************************************************************************************************/
void BuildText(char *tbox, char *NewWord) // same as sendtext only delayed send
{
    char CB[120];
    // ClaudeFix-2-7-2026 The old cap forgot the name + ".txt=\"" + quote overhead (and replaced
    // the CALLER's buffer with "Too long!") -- truncate safely instead.
    strcpy(CB, tbox);
    strcat(CB, ".txt=\"");
    uint16_t used = strlen(CB);
    CopyTextForNextion(CB + used, NewWord, (uint16_t)(sizeof(CB) - used - 2));
    strcat(CB, "\"");
    BuildNextionCommand(CB);
}


/*********************************************************************************************************************************/
void SendText1(char *tbox, char *NewWord)
{
    char CB[2048];
    strcpy(CB, tbox);
    strcat(CB, ".txt=\"");
    uint16_t used = strlen(CB);
    CopyTextForNextion(CB + used, NewWord, (uint16_t)(sizeof(CB) - used - 2));
    strcat(CB, "\"");
    SendCommand(CB);
    GetReturnCode(tbox);
}
/*********************************************************************************************************************************/
void SendText(char *tbox, char *NewWord)
{
    SendText1(tbox, NewWord);
}

/*********************************************************************************************************************************/
void SendOtherText(char *tbox, char *NewWord)
{
    char CB[MAXBUFFERSIZE];
    strcpy(CB, tbox); // ClaudeFix-2-7-2026 tbox carries its own prefix incl. the opening quote
    uint16_t used = strlen(CB);
    CopyTextForNextion(CB + used, NewWord, (uint16_t)(sizeof(CB) - used - 2));
    strcat(CB, "\"");
    SendCommand(CB);
    // Look(CB);
    GetReturnCode(tbox);
}

/*********************************************************************************************************************************/
void SendOtherValue(char *nbox, int value)
{
    char Val[] = "=";
    char CB[100];
    char NB[25];
    strcpy(CB, nbox);
    strcat(CB, Val);
    strcat(CB, Str(NB, value, 0));
    SendCommand(CB);
    ValueSent = true;
    GetReturnCode(nbox);
}

/*********************************************************************************************************************************/
// ClaudeFix-14-7-2026 The Nextion sends UNSOLICITED touch-event strings ("Import x.MOD",
// "Yes", ...) into the same serial buffer as get-replies. The 2-7 fix flushed
// that buffer before every get -- which DESTROYED any button press that was
// waiting there (the restore button needed 2-3 presses). Instead of discarding,
// printable bytes are rescued here and served to GetButtonPress afterwards.
// (Return-code debris -- 0x01/0x1A + FF FF FF -- is non-printable and still dropped.)
char PendingEvent[128];
uint16_t PendingEventLen = 0;

void StashPrintableBytes(const char *src, uint16_t len)
{
    for (uint16_t i = 0; i < len; ++i)
    {
        uint8_t b = (uint8_t)src[i];
        if (b >= 32 && b < 0x7F && PendingEventLen < sizeof(PendingEvent) - 1)
            PendingEvent[PendingEventLen++] = b;
    }
    PendingEvent[PendingEventLen] = 0;
}

void StashPendingEventBytes() // replaces the blind pre-get flush
{
    while (NEXTION.available())
    {
        uint8_t b = NEXTION.read();
        if (b >= 32 && b < 0x7F && PendingEventLen < sizeof(PendingEvent) - 1)
            PendingEvent[PendingEventLen++] = b;
        delayMicroseconds(20); // one byte at 921600 baud takes ~11 us
    }
    PendingEvent[PendingEventLen] = 0;
}

void GetTextIn() 
{
    delayMicroseconds(20);
    if (NEXTION.available())
    {
        int j = 0;
        while (NEXTION.available())
        {
            TextIn[j] = NEXTION.read();
            KickTheDog();
            if (j < MAXTEXTIN)
                ++j;
            delayMicroseconds(20);
        }
    }
}

/*********************************************************************************************************************************/
bool GetButtonPress() 
{
    if (PendingEventLen)
    {   // a touch event rescued from a get-reply window -- deliver it now
        memcpy(TextIn, PendingEvent, PendingEventLen);
        TextIn[PendingEventLen] = 0;
        PendingEventLen = 0;
        return true;
    }
    if (NEXTION.available())
    {
        GetTextIn();
        return true;
    }
    return false;
}

/*********************************************************************************************************************************/
void SendCommand(char *tbox)
{
    char page[] = "page ";
    char blankview[] = "BlankView";
    NEXTION.print(tbox);
    for (int i = 0; i < 3; ++i)
        NEXTION.write(0xff);
    delayMicroseconds(70);
    GetReturnCode(tbox);
    if (InStrng(blankview, tbox))
        return; // Don't need to wait for blankview
    if (InStrng(page, tbox))
        DelayWithDog(SCREENCHANGEWAIT); // Allow time for new page to appear
}
/*********************************************************************************************************************************/
void EndSend()
{
    for (u_int8_t pp = 0; pp < 3; ++pp)
    {
        NEXTION.write(0xff); // Send end of Input message //
    }
    DelayWithDog(55); // ** A DELAY ** (>=50 ms) was needed if an answer might come! (!! Shorter with Intelligent dislay)
}
/*********************************************************************************************************************************/
void SendValue(char *nbox, int value)
{
    char Val[] = ".val=";
    char CB[100];
    char NB[25];
    strcpy(CB, nbox);
    strcat(CB, Val);
    strcat(CB, Str(NB, value, 0));
    SendCommand(CB);
    ValueSent = true;
    GetReturnCode(nbox);
}

/*********************************************************************************************************************************/
uint32_t getvalue(char *nbox)
{
    uint32_t ValueIn = 0;
    char GET[] = "get ";
    char VAL[] = ".val";
    char CB[100];

    strcpy(CB, GET);
    strcat(CB, nbox);
    strcat(CB, VAL);
    // ClaudeFix-2-7-2026 Same discipline as the fixed GetText: FLUSH stale bytes first (a
    // leftover reply used to be parsed as THIS one -- the ArmingChannel bug
    // family, which also fed shifted-by-one PIDs/rates to the FC), then WAIT
    // for the framed 'q' reply instead of hoping it already arrived.
    StashPendingEventBytes(); // ClaudeFix-14-7-2026 rescue any waiting button press, don't flush it
    TextIn[0] = 0;
    NEXTION.print(CB);
    EndSend();
    {
        uint32_t begun = millis();
        uint16_t k = 0;
        uint8_t ffs = 0;
        bool done = false;
        while (!done && (millis() - begun) < 100)
        {
            while (NEXTION.available())
            {
                uint8_t b = NEXTION.read();
                if (k < MAXTEXTIN)
                    TextIn[k++] = b;
                if (b == 0xFF)
                {
                    if (++ffs >= 3)
                    {
                        done = true;
                        break;
                    }
                }
                else
                    ffs = 0;
            }
            KickTheDog();
        }
        // ClaudeFix-14-7-2026 A touch event that arrived just before the reply sits IN FRONT
        // of the 8-byte 'q' frame. Rescue it and realign -- otherwise the read
        // "fails" (65535), GetValue retries 25x with flushes, and the press dies.
        if (done && k > 8)
        {
            StashPrintableBytes((char *)TextIn, k - 8);
            memmove(TextIn, TextIn + (k - 8), 8);
        }
    }
    if (TextIn[0] == 'q')
    {
        ValueIn = TextIn[1]; // Collect and build 32 bit value from 4 bytes
        ValueIn += (TextIn[2] << 8);
        ValueIn += (TextIn[3] << 16);
        ValueIn += (TextIn[4] << 24);
    }
    else
    {
        ValueIn = 65535; // = THERE WAS AN ERROR !
    }
    return ValueIn;
}

/*********************************************************************************************************************************/

uint32_t GetValue(char *nbox)
{ // This function calls the function above until it returns no error

    int i = 0;
    uint32_t ValueIn = getvalue(nbox);

    while (ValueIn == 65535 && i < 25)
    { // if error read again!
        DelayWithDog(10);
        ValueIn = getvalue(nbox);
        ++i;
    }
    return ValueIn;
}

// ***************************************************************************************************************
// This function gets Nextion textbox Text into a char array pointed to by * TheText. There better be room!
// It returns the length of array
uint16_t GetText(char *TextBoxName, char *TheText, uint16_t maxlen)
{
    TheText[0] = 0; // guarantee a defined result even if the Nextion doesn't answer with 'p'
    if (maxlen < 2)
        return 0;
    char get[] = "get ";
    char _txt[] = ".txt";
    char CB[100];
    uint16_t j = 0;
    strcpy(CB, get);
    strcat(CB, TextBoxName);
    strcat(CB, _txt);

    // ClaudeFix-2-7-2026 Discard any STALE bytes before asking. Leftovers from earlier traffic
    // (especially after heavy MSP work on the Rotorflight screens) used to be
    // parsed as THIS field's reply — so field N's answer became field N+1's
    // value. That is exactly how ArmingChannel "mysteriously" changed: the
    // Ratio box's "10.30" was read back as Arming and atoi'd to a perfectly
    // plausible channel 10, which then sailed past the range check.
    StashPendingEventBytes(); // ClaudeFix-14-7-2026 rescue any waiting button press, don't flush it
    TextIn[0] = 0; // if no reply arrives, stale TextIn content must not be re-parsed

    NEXTION.print(CB);
    EndSend();

    // WAIT for the actual reply. The old code waited 20 MICROseconds — the
    // Nextion cannot even begin answering that fast, so what got read was
    // whatever the buffer already held (usually the PREVIOUS get's answer).
    // A reply is 'p' + text + FF FF FF: collect until that terminator, with
    // a timeout so a dead display can't hang us (normal replies take ~2 ms).
    {
        uint32_t begun = millis();
        uint16_t k = 0;
        uint8_t ffs = 0;
        bool done = false;
        while (!done && (millis() - begun) < 100)
        {
            while (NEXTION.available())
            {
                uint8_t b = NEXTION.read();
                if (k < MAXTEXTIN)
                    TextIn[k++] = b;
                if (b == 0xFF)
                {
                    if (++ffs >= 3)
                    {
                        done = true;
                        break;
                    }
                }
                else
                    ffs = 0;
            }
            KickTheDog();
        }
    }

    if (TextIn[0] == 'p')
    {
        while (TextIn[j + 1] < 0xFF && j < MAXTEXTIN - 2 && j < maxlen - 1)
        {   // ClaudeFix-2-7-2026 maxlen: the reply lands in CALLER buffers as small as 10 bytes
            // ("There better be room!" -- now there is)
            TheText[j] = TextIn[j + 1];
            ++j;
            KickTheDog(); // ??
        }
        TheText[j] = 0;
    }
    return strlen(TheText);
}
/*********************************************************************************************************************************/
int GetOtherValue(char *nbox)
{
    // don't add .val as other thingy is already there ...
    double ValueIn = 0;
    char GET[] = "get ";
    char CB[100];
    strcpy(CB, GET);
    strcat(CB, nbox);
    StashPendingEventBytes(); // ClaudeFix-14-7-2026 rescue any waiting button press, don't flush it
    TextIn[0] = 0;
    NEXTION.print(CB);
    EndSend();
    {
        uint32_t begun = millis();
        uint16_t k = 0;
        uint8_t ffs = 0;
        bool done = false;
        while (!done && (millis() - begun) < 100)
        {
            while (NEXTION.available())
            {
                uint8_t b = NEXTION.read();
                if (k < MAXTEXTIN)
                    TextIn[k++] = b;
                if (b == 0xFF)
                {
                    if (++ffs >= 3)
                    {
                        done = true;
                        break;
                    }
                }
                else
                    ffs = 0;
            }
            KickTheDog();
        }
        // ClaudeFix-14-7-2026 A touch event that arrived just before the reply sits IN FRONT
        // of the 8-byte 'q' frame. Rescue it and realign -- otherwise the read
        // "fails" (65535), GetValue retries 25x with flushes, and the press dies.
        if (done && k > 8)
        {
            StashPrintableBytes((char *)TextIn, k - 8);
            memmove(TextIn, TextIn + (k - 8), 8);
        }
    }
    if (TextIn[0] == 'q')
    {
        ValueIn = TextIn[1]; // Collect and build 32 bit value from 4 bytes
        ValueIn += (TextIn[2] << 8);
        ValueIn += (TextIn[3] << 16);
        ValueIn += (TextIn[4] << 24);
    }
    return ValueIn;
}

// ******************************************************************************************************************************
static inline void NextionTerminator()
{
    NEXTION.write(0xFF);
    NEXTION.write(0xFF);
    NEXTION.write(0xFF);
}

// ******************************************************************************************************************************
static void FlushNextionInput(uint32_t ms = 30)
{
    uint32_t start = millis();
    while (millis() - start < ms)
    {
        while (NEXTION.available())
            (void)NEXTION.read();
        delay(1);
        KickTheDog();
    }
}

// ******************************************************************************************************************************
static bool ReadNextionNumber(int32_t &out, uint32_t timeout_ms)
{
    uint32_t start = millis();

    // wait for 0x71
    while (millis() - start < timeout_ms)
    {
        if (NEXTION.available())
        {
            if ((uint8_t)NEXTION.read() == 0x71)
                break;
        }
        delay(1);
        KickTheDog();
    }
    if (millis() - start >= timeout_ms)
        return false;

    uint8_t b[4];
    for (int i = 0; i < 4; i++)
    {
        uint32_t t0 = millis();
        while (!NEXTION.available())
        {
            if (millis() - t0 > timeout_ms)
                return false;
            delay(1);
            KickTheDog();
        }
        b[i] = (uint8_t)NEXTION.read();
    }

    out = (int32_t)((uint32_t)b[0] |
                    ((uint32_t)b[1] << 8) |
                    ((uint32_t)b[2] << 16) |
                    ((uint32_t)b[3] << 24));
    return true;
}
// ******************************************************************************************************************************
int GetIntFromTextBox(char *tbox)
{
    char get[] = "get ";
    char _txt[] = ".txt";
    char CB[100];
    uint8_t j = 0;
    char Text[50];
    strcpy(CB, get);
    strcat(CB, tbox);
    strcat(CB, _txt);
    NEXTION.print(CB);
    EndSend();
    GetTextIn();
    if (TextIn[0] == 'p')
    {
        while (TextIn[j + 1] < 0xFF && j < sizeof(Text) - 1) // ClaudeFix-2-7-2026 bounded: a fragment reply used to copy forever (uint8_t j wraps, smashing the stack past Text[50])
        {
            Text[j] = TextIn[j + 1];
            ++j;
            KickTheDog(); // ??
        }
        Text[j] = 0;

        return atoi(Text);
    }
    return 0;
}
// ******************************************************************************************************************************
float GetFloatFromTextBox(char *tbox)
{
    char get[] = "get ";
    char _txt[] = ".txt";
    char CB[100];
    uint8_t j = 0;
    char Text[50];
    strcpy(CB, get);
    strcat(CB, tbox);
    strcat(CB, _txt);
    NEXTION.print(CB);
    EndSend();
    GetTextIn();
    if (TextIn[0] == 'p')
    {
        while (TextIn[j + 1] < 0xFF && j < sizeof(Text) - 1) // ClaudeFix-2-7-2026 bounded: a fragment reply used to copy forever (uint8_t j wraps, smashing the stack past Text[50])
        {
            Text[j] = TextIn[j + 1];
            ++j;
            KickTheDog(); // ??
        }
        Text[j] = 0;

        return atof(Text);
    }
    return 0;
}

// ******************************************************************************************************************************
bool NextionFileExistsOnSD(char *filename, bool verbose = false)
{
    // filename like "Adrians.jpg" in /images
    char path[96];
    int n = snprintf(path, sizeof(path), "sd0/images/%s", filename); // <-- changed
    if (n < 0 || n >= (int)sizeof(path))
        return false;

    if (verbose)
    {
        Look1("NextionFileExists: ");
        Look(path);
    }

    FlushNextionInput(50);

    NEXTION.print("findfile \"");
    NEXTION.print(path);
    NEXTION.print("\",sys0");
    NextionTerminator();

    delay(10);

    NEXTION.print("get sys0");
    NextionTerminator();

    int32_t v = 0;
    if (!ReadNextionNumber(v, 500))
    {
        if (verbose)
            Look("  read sys0 FAILED.");
        return false;
    }

    if (verbose)
    {
        Look1("  sys0 = ");
        Look(v);
    }
    return (v != 0);
}
/*********************************************************************************************************************************/
//             END OF NEXTION FUNCTIONS
/*********************************************************************************************************************************/
#endif