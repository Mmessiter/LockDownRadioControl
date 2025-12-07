
/** @file ReceiverCode/src/utilities/binding.h */
// Malcolm Messiter 2020 - 2025
#ifndef _SRC_BINDING_H
#define _SRC_BINDING_H
#include <Arduino.h>
#include "utilities/1Definitions.h"
//************************************************************************************************************/

bool TestTheNewPipe() // Check that the set pipe can actually receive data before going further.
{
    uint8_t idx = 1;
    uint32_t LookTime = millis();
    while (millis() - LookTime < 650)
    {
        CurrentRadio->stopListening();
        delayMicroseconds(STOPLISTENINGDELAY);
        CurrentRadio->setChannel(FHSS_Recovery_Channels[idx]);
        delayMicroseconds(STOPLISTENINGDELAY);
        CurrentRadio->startListening();
        delayMicroseconds(STOPLISTENINGDELAY);
        uint32_t t = millis();
        while (!CurrentRadio->available(&Pipnum) && (millis() - t < 30))
        {
            delay(1);
            KickTheDog();
        }
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
    }
    return false;
}

/************************************************************************************************************/

void GetNewPipe() // from TX
{
    if (!NewData)
        return;
    NewData = false;
    for (int i = 0; i < 5; ++i)
        TheReceivedPipe[4 - i] = ReceivedData[i + 1] & 0xff; // reversed byte array for our use
    TheReceivedPipe[5] = 0;                                  // need a terminating zero
    if (Blinking)                                            // if binding, then use the received pipe
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
      //  Look("Pipe test succeeded. Model bound.");
    }
    else
    {
       // Look("Pipe test failed. Not bound.");
    }
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
void DisplayAPipe(const uint8_t *pipe) // for debug
{
    char buffer[44];
    snprintf(buffer, 44, "Pipe: %02X %02X %02X %02X %02X %02X", pipe[0], pipe[1], pipe[2], pipe[3], pipe[4], pipe[5]);
    Look(buffer);
}
//************************************************************************************************************/
void SetNewPipe()
{
    CurrentRadio->openReadingPipe(Pipnum, PipePointer); //  5 * byte array
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
        StartSBUSandSERVOS(); // start SBUS and SERVOS on first connection
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
    MacAddressSentCounter = 0;
    CopyToCurrentPipe(DefaultPipe, BOUNDPIPENUMBER);
    SetNewPipe(); // set the default pipe
}
#endif