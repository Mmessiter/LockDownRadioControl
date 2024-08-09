// This file is used to compare the model ID with the saved model ID. If the model ID is not found, it will NOT display a warning message.

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef MODEL_MATCH_H
    #define MODEL_MATCH_H
    
    
/************************************************************************************************************/

void CompareModelsIDs()
{                                       // The saved MacAddress is compared with the one just received from the model ... etc ...
    uint8_t SavedModelNumber = ModelNumber;
    char NotFound[] = "           ** WARNING! **\r\n\r\n   Model memory not found!\r\n If correct, please store its ID.";
    
    
    if (BuddyPupilOnWireless) return;   //  Don't do this if we are a pupil
    if (BuddyON) return;                //  Don't do this if buddy is on
    if (ModelMatched) return;           //  must not change when model connected
    GotoFrontView();
    RestoreBrightness();
    if (ModelIdentified) {              //  We have both bits of Model ID?
        if ((ModelsMacUnion.Val64 == ModelsMacUnionSaved.Val64)) { //  Is it a match for current model?
            if (AnnounceConnected) {
                if (AutoModelSelect) {
                    PlaySound(MMMATCHED);
                    if (UseLog) LogModelMatched();
                    DelayWithDog(1400);
                }
            }
            ModelMatched = true;        //  It's a match so start flying!
            return;
        }
        else {
            if (AutoModelSelect)
            {                           //  It's not a match so search for it.
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
                        DelayWithDog(1000);     // allow time to say "Connected"
                        if (UseLog) LogModelFound();
                        PlaySound(MMFOUND);
                    }
                    SaveAllParameters();        //  Save it
                    GotoFrontView();//
                }
                else 
                {
                ModelNumber = SavedModelNumber;  // on second thoughts, just use the saved one
                ReadOneModel(ModelNumber);
                BindNow();
                if (UseLog) LogModelNotFound();
                MsgBox(pFrontView, NotFound);   //  Tell the user it's not found
                }
            }
            if (!AutoModelSelect) 
                {
                BindNow();
                }
        }
    }
}

/************************************************************************************************************/
#endif