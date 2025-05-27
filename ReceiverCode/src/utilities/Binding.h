
/** @file ReceiverCode/src/utilities/binding.h */
// Malcolm Messiter 2020 - 2025
#ifndef _SRC_BINDING_H
#define _SRC_BINDING_H
#include <Arduino.h>
#include "utilities/common.h"
#include <EEPROM.h>

/************************************************************************************************************/
// This function compares the just-received pipe with several of the previous ones
// if it matches most of them then its probably not corrupted.

bool ValidateNewPipe()
{
    uint8_t MatchedCounter = 0;
    if (pcount < 2)
        return false; // ignore first few

    PreviousNewPipes[PreviousNewPipesIndex] = NewPipeMaybe;
    PreviousNewPipesIndex++;
    if (PreviousNewPipesIndex > PIPES_TO_COMPARE)
        PreviousNewPipesIndex = 0;

    for (int i = 0; i < PIPES_TO_COMPARE; ++i)
    {
        if (NewPipeMaybe == PreviousNewPipes[i])
            ++MatchedCounter;
    }

    if (MatchedCounter >= 2)
        return true;
    return false;
}

/************************************************************************************************************/

void GetNewPipe() // from TX
{
    if (!NewData)
        return;
    NewData = false;
    if (PipeSeen)
        return;
    NewPipeMaybe = (uint64_t)ReceivedData[0] << 40;
    NewPipeMaybe += (uint64_t)ReceivedData[1] << 32;
    NewPipeMaybe += (uint64_t)ReceivedData[2] << 24;
    NewPipeMaybe += (uint64_t)ReceivedData[3] << 16;
    NewPipeMaybe += (uint64_t)ReceivedData[4] << 8;
    NewPipeMaybe += (uint64_t)ReceivedData[5];

    if (ValidateNewPipe()) // was this pipe corrupted?
    {
        for (int i = 0; i < 5; ++i)
            TheReceivedPipe[4 - i] = ReceivedData[i + 1] & 0xff; // reversed byte array for our use
        TheReceivedPipe[5] = 0;
        if (Blinking)
            CopyCurrentPipe(TheReceivedPipe, BOUNDPIPENUMBER);
        else
            CopyCurrentPipe(TheSavedPipe, BOUNDPIPENUMBER);
        BoundFlag = true;
        Connected = true;
        SetNewPipe();
        BindModel();
        PipeSeen = true;
    }
    ++pcount; // inc pipes received
}

/************************************************************************************************************/
void ReadSavedPipe() // read only 6 bytes
{
    for (uint8_t i = 0; i < 5; ++i)
    {
        TheSavedPipe[i] = EEPROM.read(i + BIND_EEPROM_OFFSET); // uses first 5 bytes only.
    }
    TheSavedPipe[5] = 0;
}
/************************************************************************************************************/
void CopyCurrentPipe(uint8_t *p, uint8_t pn)
{
    for (int i = 0; i < 6; ++i)
        CurrentPipe[i] = p[i];
    PipePointer = p;
    Pipnum = pn;
}
//************************************************************************************************************/
void SetNewPipe()
{
    CurrentRadio->openReadingPipe(Pipnum, PipePointer); //  5 * byte array
}
// ************************************************************************************************************/
void ReadBindPlug()
{
    PipePointer = DefaultPipe;
    ReadSavedPipe();
    CopyCurrentPipe(DefaultPipe, PIPENUMBER);
    if (!digitalRead(BINDPLUG_PIN))
    {                    // Bind Plug needed to bind!
        Blinking = true; // Blinking = binding to new TX
        SaveNewBind = true;
    }
    else
    {
        SaveNewBind = false;
    }
}
/************************************************************************************************************/
// This function binds the model using the TX supplied Pipe instead of the default one.
// If not already saved, this saves it to the eeprom too for next time.

void BindModel()
{
    BoundFlag = true;
    ModelMatched = true;
    Connected = true;
    if (Blinking)
    {
        SetNewPipe(); // change to bound pipe <<< ***************************************
        for (uint8_t i = 0; i < 5; ++i)
        {
            EEPROM.update(i + BIND_EEPROM_OFFSET, TheReceivedPipe[i]);
        }
    }
    Blinking = false;
    if (FirstConnection)
    {
        AttachServos(); // AND START SBUS / PPM
        FirstConnection = false;
    }
    SaveNewBind = false;
    ConnectMoment = millis();
    SuccessfulPackets = 0; // Reset the packet count
}

// ************************************************************************************************************/
void UnbindModel()
{
    Connected = false;
    BoundFlag = false;
    ModelMatched = false;
    LongAcknowledgementsCounter = 0;
    PipeSeen = false;
    pcount = 0;
    MacAddressSentCounter = 0;
    CopyCurrentPipe(DefaultPipe, BOUNDPIPENUMBER);
    SetNewPipe(); // set the default pipe
}
#endif