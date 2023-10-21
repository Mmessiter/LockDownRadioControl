#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef MODEL_MATCH_H
    #define MODEL_MATCH_H
    
/************************************************************************************************************/
// If AMS (Auto Model Select) fails to find a match for the model ID, here it (optionally) asks the user to
// select a model from the list and then bind to it.
//
void IDNotFound(uint8_t SavedModelNumber)
{
    char GoModelsView[] = "page ModelsView";
    char GoFrontView[]  = "page FrontView";
    char buf[]          = "ID not found! Select model?";
    char buf1[]         = "Connecting to ";
    char buf2[70];
    
    GetConfirmation(GoFrontView, buf);
    if (Confirmed[0] != 'Y') {
        ModelNumber = PreviousModelNumber;
        ReadOneModel(ModelNumber);
        strcpy(buf2, buf1);
        strcat(buf2, ModelName);
        MsgBox(GoFrontView, buf2);
        BindNow();
        UpdateModelsNameEveryWhere(); 
        AMSnotfound = false;
        return;
    }
    SendCommand(GoModelsView);
    CurrentView = MODELSVIEW;
    CurrentMode = SENDNOTHING;
    BlueLedOn();
    UpdateModelsNameEveryWhere();
    BuildDirectory();
    LoadFileSelector();
    ModelNumber = SavedModelNumber;
    ShowFileNumber();
    AMSnotfound=true;
    LoadModelSelector();
}
/************************************************************************************************************/

void CompareModelsIDs()
{                             // The saved MacAddress is compared with the one just received from the model ... etc ...
    uint8_t SavedModelNumber = ModelNumber;
    if (BuddyPupilOnWireless) return;   //  Don't do this if we are a pupil
    if (BuddyON) return;                //  Don't do this if buddy is on
    if (BuddyMasterOnWireless) {
        ModelMatched = true;
        GreenLedOn();
        return;
    }
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
                    IDNotFound(SavedModelNumber); //  Not found, so ask user to select one
                }
            }
            if (!AutoModelSelect) BindNow();
        }
    }
}
/************************************************************************************************************/
void GetModelsMacAddress()
{ // Gets a 64 bit value in two hunks of 32 bits

    if (BuddyPupilOnWireless) return; //  Don't do this if we are a pupil
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
#endif