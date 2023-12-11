#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef MODEL_MATCH_H
    #define MODEL_MATCH_H
    
/************************************************************************************************************/

void CompareModelsIDs()
{                             // The saved MacAddress is compared with the one just received from the model ... etc ...
    uint8_t SavedModelNumber = ModelNumber;
    if (BuddyPupilOnWireless) return;   //  Don't do this if we are a pupil
    if (BuddyON) return;                //  Don't do this if buddy is on
    if (ModelMatched) return;           // must not change when model connected
    GotoFrontView();
    RestoreBrightness();
    if (ModelIdentified) { //  We have both bits of Model ID?
        if ((ModelsMacUnion.Val64 == ModelsMacUnionSaved.Val64)) { //  Is it a match for current model?
            if (AnnounceConnected) {
                if (AutoModelSelect) {
                    PlaySound(MMMATCHED);
                    DelayWithDog(1500);
                }
            }
            ModelMatched = true; //  It's a match so start flying!
            return;
        }
        else {
            if (AutoModelSelect)
            {                                                                           //  It's not a match so search for it.
                ModelNumber = 0;
                while ((ModelMatched == false) && (ModelNumber < MAXMODELNUMBER - 1)) { //  Try to match the ID with a saved one
                    ++ModelNumber;
                    ReadOneModel(ModelNumber);
                    if ((ModelsMacUnion.Val64 == ModelsMacUnionSaved.Val64)) ModelMatched = true; //  Found it!
                }
                if (ModelMatched) {               //  Found it!
                    UpdateModelsNameEveryWhere(); //  Use it everywhere.
                    if (AnnounceConnected)
                    {
                        PlaySound(MMFOUND);
                        DelayWithDog(1500);
                    }
                    SaveAllParameters(); //  Save it
                    GotoFrontView();
                }
                else {
                    ModelNumber = SavedModelNumber;  // on second thoughts, just use the saved one
                    ReadOneModel(ModelNumber);
                    BindNow();
                }
            }
            if (!AutoModelSelect) BindNow();
        }
    }
}

/************************************************************************************************************/
#endif