
/** @file ReceiverCode/src/utilities/binding.h */
// Malcolm Messiter 2020 - 2025
#ifndef _SRC_BINDING_H
#define _SRC_BINDING_H
#include <Arduino.h>
#include "utilities/1Definitions.h"

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
    if (MatchedCounter > 2)
        return true;
    return false;
}
// ************************************************************************************************************/

bool TestTheNewPipe() // Check that the set pipe can actually receive data before going further.
{
    if (Blinking) // if binding, then we don't need to test the pipe
        return true;
    uint8_t idx = 1;

    uint32_t LookTime = millis();
    while (millis() - LookTime < 350)
    {
        CurrentRadio->stopListening();
        delayMicroseconds(STOPLISTENINGDELAY);
        CurrentRadio->setChannel(FHSS_Recovery_Channels[idx]);
        delayMicroseconds(STOPLISTENINGDELAY);
        CurrentRadio->startListening();
        delayMicroseconds(STOPLISTENINGDELAY);
        if (CurrentRadio->available(&Pipnum))
        {
            return true;
        }
        else
        {
            ++idx;
            if (idx >= 3)
                idx = 0;
        }
        KickTheDog(); // keep the watchdog happy
        delay(1);     // wait a bit
    }
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

    if (ValidateNewPipe()) // was this pipe uncorrupted?
    {
        for (int i = 0; i < 5; ++i)
            TheReceivedPipe[4 - i] = ReceivedData[i + 1] & 0xff; // reversed byte array for our use
        TheReceivedPipe[5] = 0;
        if (Blinking) // if binding, then use the received pipe
        {
            CopyToCurrentPipe(TheReceivedPipe, BOUNDPIPENUMBER);
        }
        else // if not binding, then use the saved pipe
        {
            CopyToCurrentPipe(TheSavedPipe, BOUNDPIPENUMBER);
        }
        SetNewPipe();
        delay(5);
        CurrentRadio->flush_tx();
        CurrentRadio->flush_rx();
        delay(5);
        if (TestTheNewPipe())
        {
            BindModel(); // don't bind if the pipe is not valid
        }
        PipeSeen = true;
    }
    ++pcount; // inc pipes received
}

/************************************************************************************************************/
void CopyToCurrentPipe(uint8_t *p, uint8_t pn)
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

//************************************************************************************************************/
void DisplayAPipe(const uint8_t *pipe) // for debug
{
    char buffer[44];
    snprintf(buffer, 44, "Pipe: %02X %02X %02X %02X %02X", pipe[0], pipe[1], pipe[2], pipe[3], pipe[4]);
    Look(buffer);
}

/************************************************************************************************************/
// This function binds the model using the TX supplied Pipe instead of the default one.
// If not already saved, this saves it to the eeprom too for next time.

void BindModel()
{
    BoundFlag = true;

    if (Blinking)
    {
        SavePipeToEEPROM();
        Blinking = false;
    }
    if (FirstConnection)
    {
        AttachServos(); // AND START SBUS
        FirstConnection = false;
    }
    ConnectMoment = millis();
    SuccessfulPackets = 0; // Reset the packet count
}

// ************************************************************************************************************/
void UnbindModel()
{
    BoundFlag = false;
    LongAcknowledgementsCounter = 0;
    PipeSeen = false;
    pcount = 0;
    MacAddressSentCounter = 0;
    CopyToCurrentPipe(DefaultPipe, BOUNDPIPENUMBER);
    SetNewPipe(); // set the default pipe
}
#endif