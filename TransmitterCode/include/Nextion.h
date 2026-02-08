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
void BuildText(char *tbox, char *NewWord) // same as sendtext only delayed send
{
    char txt[] = ".txt=\"";
    char quote[] = "\"";
    char CB[120];
    char TooLong[] = "Too long!";
    if (strlen(NewWord) > 110)
    {
        strcpy(NewWord, TooLong); //
    }
    strcpy(CB, tbox);
    strcat(CB, txt);
    strcat(CB, NewWord);
    strcat(CB, quote);
    BuildNextionCommand(CB);
}


/*********************************************************************************************************************************/
void SendText1(char *tbox, char *NewWord)
{
    char txt[] = ".txt=\"";
    char quote[] = "\"";
    char CB[2048];
    char TooLong[] = "Too long!";
    if (strlen(NewWord) > 2048-5)
    {
        strcpy(NewWord, TooLong); //
    }
    strcpy(CB, tbox);
    strcat(CB, txt);
    strcat(CB, NewWord);
    strcat(CB, quote);
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
    char quote[] = "\"";
    char CB[MAXBUFFERSIZE];
    char TooLong[] = "Too long!";

    if (strlen(NewWord) > MAXBUFFERSIZE - 2)
    {
        strcpy(NewWord, TooLong);
    }
    strcpy(CB, tbox);
    strcat(CB, NewWord);
    strcat(CB, quote);
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
    NEXTION.print(CB);
    EndSend();
    GetTextIn();
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
uint16_t GetText(char *TextBoxName, char *TheText)
{
    
    char get[] = "get ";
    char _txt[] = ".txt";
    char CB[100];
    uint8_t j = 0;
    strcpy(CB, get);
    strcat(CB, TextBoxName);
    strcat(CB, _txt);
    NEXTION.print(CB);
    EndSend();
    GetTextIn();
    if (TextIn[0] == 'p')
    {
        while (TextIn[j + 1] < 0xFF)
        {
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
    NEXTION.print(CB);
    EndSend();
    GetTextIn();
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
bool NextionFileExistsOnSD(char *filename, bool verbose = false)
{
    // filename like "Adrians.jpg" in root
    char path[96];
    int n = snprintf(path, sizeof(path), "sd0/%s", filename);
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