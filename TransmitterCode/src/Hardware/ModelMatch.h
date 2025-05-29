// This file is used to compare the model ID with the saved model ID. If the model ID is not found, it will NOT display a warning message.

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef MODEL_MATCH_H
#define MODEL_MATCH_H

/************************************************************************************************************/

void CompareModelsIDs()
{
    /*
    This function prevents flying (and most likely, crashing) a model while a wrong model memory is accidentally still loaded.
    The saved MAC address of the model's Teensy 4.0 is compared with the one just received from the model - during binding.
    If the two match, the match is announced and the the 'ModelMatched' flag is set to true.
    If it doesn't match, then all the locally stored models are rapidly searched in the hope of finding the one that does match.
    If a match is found, that model memory is loaded and the model's name is displayed. The find is also announced.
    If no match is found, the previously loaded model is used and a 'Model not found' warning message is announced.
    (This is needed for new models that have not had their IDs saved yet.)
    This function also saves the pilot from having always to select the correct model manually, which can get tedious and is also error prone.
    5-byte MAC addresses give 2^40 (≈1.1 trillion) unique combinations, making false matches vanishingly unlikely.
    */

    uint8_t SavedModelNumber = ModelNumber;
    if ((BuddyPupilOnWireless) || (BuddyON) || (ModelMatched))
        return; //  Don't do this if any of these are ON
    GotoFrontView();
    RestoreBrightness();
    if (!ModelIdentified) //  We have both bits of Model ID?
        return;
    if ((ModelsMacUnion.Val64 == ModelsMacUnionSaved.Val64)) //  Is it a match for current model?
    {
        if (AnnounceConnected) // Yes ...
        {
            if (AutoModelSelect)
            {
                PlaySound(MMMATCHED);
                if (UseLog)
                    LogModelMatched();
                DelayWithDog(1400); // allow time to say "Matched"
            }
        }
        ModelMatched = true; //  It's a match so start flying!
        return;
    }
    else
    {
        if (AutoModelSelect) //  It's not a match so search for it if autoselect is on.
        {
            for (ModelNumber = 1; ModelNumber < MAXMODELNUMBER; ++ModelNumber) //  Try to match the ID with a saved one
            {
                ReadOneModel(ModelNumber);
                if ((ModelsMacUnion.Val64 == ModelsMacUnionSaved.Val64))
                {
                    ModelMatched = true; //  Found it!
                    break;               //  No need to search further
                }
            }
            if (ModelMatched)
            {                                 //  Found it!
                UpdateModelsNameEveryWhere(); //  Use it everywhere.
                if (AnnounceConnected)
                {
                    PlaySound(MMFOUND);
                    DelayWithDog(1400); // allow time to say "Found"
                    if (UseLog)
                        LogModelFound();
                }
                SaveAllParameters(); //  Save it
                GotoFrontView();     //
            }
            else
            {
                ModelNumber = SavedModelNumber; // on second thoughts, just use the saved one
                ReadOneModel(ModelNumber);
                if (UseLog)
                    LogModelNotFound();
                PlaySound(NOTFOUND);
                DelayWithDog(2000);
                BindNow();
            }
        }
        else
        {
            BindNow();
        }
    }
}
#endif