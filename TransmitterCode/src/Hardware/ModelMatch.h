// This file is used to compare the model ID with the saved model ID. If the model ID is not found, it will NOT display a warning message.

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef MODEL_MATCH_H
#define MODEL_MATCH_H

/************************************************************************************************************/
#define MACS_MATCHED (ModelsMacUnion.Val64 == ModelsMacUnionSaved.Val64)
void CompareModelsIDs()
{
    /*
    This function prevents flying (and most likely, crashing) a model while a wrong model memory is accidentally still loaded.
    The saved MAC address of the model's Teensy 4.0 is compared with the one just received from the model - during binding.
    If the two match, the match is announced and the 'ModelMatched' flag is set to true.
    If it doesn't match, then all the locally stored models are rapidly searched in the hope of finding the one that does match.
    If a match is found, that model memory is loaded and the model's name is displayed. The find is also announced.
    If no match is found, the previously loaded model is used and a 'Model not found' warning message is announced.
    (This is needed for new models that have not had their IDs saved yet.)
    This function also saves the pilot from having always to select the correct model manually, which can get tedious and is also error prone.
    5-byte MAC addresses give 2^40 (≈1.1 trillion) unique combinations, making false matches vanishingly unlikely.
    */

    uint8_t SavedModelNumber = ModelNumber;
    if ((BuddyPupilOnWireless) || (BuddyON) || (ModelMatched && MACS_MATCHED))
        return; //  Don't do this if any of these are ON
    GotoFrontView();
    RestoreBrightness();
    if (!AutoModelSelect)
    {
        BindNow();
        return; //  If AutoModelSelect is OFF, just bind with the current model
    }
    if (!ModelIdentified) //  We have both bits of Model ID?
        return;
    if (MACS_MATCHED) //  Is it a match for current model?
    {
        if (AnnounceConnected) // Yes ...
        {
            PlaySound(MMMATCHED);
            if (UseLog)
                LogModelMatched();
            DelayWithDog(1400); // allow time to say "Matched"
        }
        ModelMatched = true; //  It's a match so start flying!
        return;
    }
    else
    {
        for (ModelNumber = 1; ModelNumber < MAXMODELNUMBER; ++ModelNumber) //  Try to match the ID with a saved one
        {
            ModelMatched = false; //  Reset the flag
            ReadOneModel(ModelNumber);
            if (MACS_MATCHED) //  Is it a match for any of the saved models?
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
        }
        else
        {
            ModelNumber = SavedModelNumber; // Was not found. So use the one that had been loaded. It might be a new model! Every model was new once!
            ReadOneModel(ModelNumber);
            if (UseLog)
                LogModelNotFound();
            PlaySound(NOTFOUND);
            DelayWithDog(2000); // MUST allow time to say "Model not found", otherwise it gets interrupted by "CONNECTED!" announcement
        }
    }
    BindNow(); // Always bind after considering model's MAC ID, whether matched, found, or not found (this last one is needed for New Models).
}
#endif