#include <Arduino.h>
#include "Hardware/Definitions.h"
/*********************************************************************************************************************************/
//                        NEXTION functions
/*********************************************************************************************************************************/
void GetReturnCode()
{ // Currently this is absorbed but ignored. This seems to be essential.
    delayMicroseconds(70);
    while (NEXTION.available()) {
        NEXTION.read();
        delayMicroseconds(70);
    }
}
/*********************************************************************************************************************************/
void SendText(char* tbox, char* NewWord)
{
    char txt[]   = ".txt=\"";
    char quote[] = "\"";
    char CB[100];
    char TooLong[] = "Too long!";
    if (strlen(NewWord) > 90) {
        strcpy(NewWord, TooLong);
    }
    strcpy(CB, tbox);
    strcat(CB, txt);
    strcat(CB, NewWord);
    strcat(CB, quote);
    SendCommand(CB);
    GetReturnCode();
}

/*********************************************************************************************************************************/
void SendOtherText(char* tbox, char* NewWord)
{
    char quote[] = "\"";
    char CB[MAXBUFFERSIZE];
    char TooLong[] = "Too long!";

    if (strlen(NewWord) > MAXBUFFERSIZE - 2) {
        strcpy(NewWord, TooLong);
    }
    strcpy(CB, tbox);
    strcat(CB, NewWord);
    strcat(CB, quote);
    SendCommand(CB);
    GetReturnCode();
}

/*********************************************************************************************************************************/
void SendText1(char* tbox, char* NewWord)
{
    char txt[]   = ".txt=\"";
    char quote[] = "\"";
    char CB[MAXFILELEN + 10];
    char TooLong[] = "Too long!";

    if (strlen(NewWord) > MAXFILELEN) {
        strcpy(NewWord, TooLong);
    }
    strcpy(CB, tbox);
    strcat(CB, txt);
    strcat(CB, NewWord);
    strcat(CB, quote);
    SendCommand(CB);
    GetReturnCode();
}
/*********************************************************************************************************************************/
void SendOtherValue(char* nbox, int value)
{
    char Val[] = "=";
    char CB[100];
    char NB[25];
    strcpy(CB, nbox);
    strcat(CB, Val);
    strcat(CB, Str(NB, value, 0));
    SendCommand(CB);
    ValueSent = true;
    GetReturnCode();
}

/*********************************************************************************************************************************/
void GetTextIn()
{
    int j = 0;
    delayMicroseconds(20);
    if (NEXTION.available()) {
        while (NEXTION.available()) {
            TextIn[j] = uint8_t(NEXTION.read());
            if (TextIn[j] == '$') TextIn[j] = 0;
            if (j < CHARSMAX) ++j;
            delayMicroseconds(20);
        }
    }
}

/*********************************************************************************************************************************/
void SendCommand(char* tbox)
{
    char page[] = "page ";
    NEXTION.print(tbox);
    for (int i = 0; i < 3; ++i) {
        NEXTION.write(0xff);
        delayMicroseconds(70);
    }
    GetReturnCode();
    if (InStrng(page, tbox)) DelayWithDog(SCREENCHANGEWAIT); // Allow time for new page to appear
}
/*********************************************************************************************************************************/
void EndSend()
{
    for (u_int8_t pp = 0; pp < 3; ++pp) {
        NEXTION.write(0xff); // Send end of Input message //
    }
    DelayWithDog(55);        // ** A DELAY ** (>=50 ms) was needed if an answer might come! (!! Shorter with Intelligent dislay)
}

/*********************************************************************************************************************************/
void SendValue(char* nbox, int value)
{
    char Val[] = ".val=";
    char CB[100];
    char NB[25];
    strcpy(CB, nbox);
    strcat(CB, Val);
    strcat(CB, Str(NB, value, 0));
    SendCommand(CB);
    ValueSent = true;
    GetReturnCode();
}

/*********************************************************************************************************************************/
bool GetButtonPress()
{
    uint8_t a             = 0;
    int     i             = 0;
    bool    ButtonPressed = false;
    while (NEXTION.available()) {
        a = NEXTION.read();
        if (a > 31 && a < 254) {
            ButtonPressed = true;
            TextIn[i]     = a;
            if (TextIn[i] == '$') TextIn[i] = 0;
            TextIn[i + 1] = 0;
        }
        if (i < CHARSMAX - 1) ++i;
        delay(1); // needed!!
    }
    if (ButtonPressed && ButtonClicks) PlaySound(CLICKONE);
    return ButtonPressed;
}

/*********************************************************************************************************************************/
uint32_t getvalue(char* nbox)
{
    uint32_t ValueIn = 0;
    char     GET[]   = "get ";
    char     VAL[]   = ".val";
    char     CB[100];

    strcpy(CB, GET);
    strcat(CB, nbox);
    strcat(CB, VAL);
    NEXTION.print(CB);
    EndSend();
    GetTextIn();
    if (TextIn[0] == 'q') {
        ValueIn = TextIn[1]; // Collect and build 32 bit value from 4 bytes
        ValueIn += (TextIn[2] << 8);
        ValueIn += (TextIn[3] << 16);
        ValueIn += (TextIn[4] << 24);
    }
    else {
        ValueIn = 65535; // = THERE WAS AN ERROR !
    }
    return ValueIn;
}

/*********************************************************************************************************************************/

uint32_t GetValue(char* nbox)
{ // This function calls the function above until it returns no error
    int      i       = 0;
    uint32_t ValueIn = getvalue(nbox);

    while (ValueIn == 65535 && i < 25) { // if error read again!
        DelayWithDog(10);
        ValueIn = getvalue(nbox);
        ++i;
    }
    return ValueIn;
}

// ***************************************************************************************************************
// This function gets Nextion textbox Text into a char array pointed to by * TheText. There better be room!
// It returns the length of array
uint16_t GetText(char* TextBoxName, char* TheText)
{
    char    get[]  = "get ";
    char    _txt[] = ".txt";
    char    CB[100];
    uint8_t j = 0;
    strcpy(CB, get);
    strcat(CB, TextBoxName);
    strcat(CB, _txt);
    NEXTION.print(CB);
    EndSend();
    GetTextIn();
    if (TextIn[0] == 'p') {
        while (TextIn[j + 1] < 0xFF) {
            TheText[j] = TextIn[j + 1];
            ++j;
        }
        TheText[j] = 0;
    }
    return strlen(TheText);
}
/*********************************************************************************************************************************/
int GetOtherValue(char* nbox)
{ // don't add .val as other thingy is already there ...
    double ValueIn = 0;
    char   GET[]   = "get ";
    char   CB[100];
    strcpy(CB, GET);
    strcat(CB, nbox);
    NEXTION.print(CB);
    EndSend();
    GetTextIn();
    if (TextIn[0] == 'q') {
        ValueIn = TextIn[1]; // Collect and build 32 bit value from 4 bytes
        ValueIn += (TextIn[2] << 8);
        ValueIn += (TextIn[3] << 16);
        ValueIn += (TextIn[4] << 24);
    }
    return ValueIn;
}
/*********************************************************************************************************************************/
//             END OF NEXTION FUNCTIONS
/*********************************************************************************************************************************/
