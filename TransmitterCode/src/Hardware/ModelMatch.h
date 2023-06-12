#include <Arduino.h>

/************************************************************************************************************/

void CompareModelsIDs()
{                             // The saved MacAddress is compared with the one just received from the model ... etc ...
    uint8_t SavedModelNumber = ModelNumber;
    if (ModelMatched) return; // must not change when model connected
    GotoFrontView();
    RestoreBrightness();
    if (ModelIdentified) { //  We have both bits of Model ID?
        if ((ModelsMacUnion.Val64 == ModelsMacUnionSaved.Val64)) {
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
                    if ((ModelsMacUnion.Val64 == ModelsMacUnionSaved.Val64)) ModelMatched = true;
                }
                if (ModelMatched) {               //  Found it!
                    UpdateModelsNameEveryWhere(); //  Use it.
                    if (AnnounceConnected)
                    {
                        PlaySound(MMFOUND);
                        DelayWithDog(1500);
                    }
                    SaveAllParameters(); //  Save it
                    GotoFrontView();
                }
                else {
                    ModelNumber = SavedModelNumber; //  Not found, so bind to the restored selected one
                    ReadOneModel(ModelNumber);
                    BindNow();
                }
            }
            if (!AutoModelSelect) BindNow();
        }
    }
}
/************************************************************************************************************/
void GetModelsMacAddress()
{ // Gets a 64 bit value in two hunks of 32 bits

    switch (AckPayload.Purpose)
    {
        case 0:
            ModelsMacUnion.Val32[0] = GetIntFromAckPayload();
            break;
        case 1:
            ModelsMacUnion.Val32[1] = GetIntFromAckPayload();
            break;
        default:
            break;
    }
    if (ModelMatched == false) {
        if ((ModelsMacUnion.Val32[0] > 0) && (ModelsMacUnion.Val32[1] > 0)) { // got both bits yet?
            ModelIdentified = true;
            CompareModelsIDs();
        }
    }
}

/************************************************************************************************************/
