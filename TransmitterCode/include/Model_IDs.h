// *************************************** Model_IDs.h *****************************************

// This is the code for handling model IDs

#include <Arduino.h>
#include "1Definitions.h"

#ifndef MODEL_IDS_H
#define MODEL_IDS_H

void CheckAllModelIds()
{
    char Vbuf[15];
    char MMemsp[] = "MMems.path=\"";
    char GoIDview[] = "page IDsView";
    char crlf[] = {13, 10, 0};
    char lb[] = "(";
    char rb[] = ")  ";
    char KO[] = ">Duplicate:";
    char Okay[] = " (ID OK)";
    char n0[] = "n0";
    char nb[4];
    char buf[MAXBUFFERSIZE];
    uint64_t ModelIDs[92];
    uint8_t DuplicatesCount = 0;
    uint32_t SavedModelNumber = ModelNumber;

    SendCommand(GoIDview);
    CurrentView = IDCHECKVIEW;
    for (ModelNumber = 1; ModelNumber < MAXMODELNUMBER - 1; ++ModelNumber)
    {
        ReadOneModel(ModelNumber);
        ModelIDs[ModelNumber] = ModelsMacUnionSaved.Val64;
    }
    for (ModelNumber = 1; ModelNumber < MAXMODELNUMBER - 1; ++ModelNumber)
    {
        ReadOneModel(ModelNumber);
        if (ModelNumber == 1)
        {
            strcpy(buf, lb);
        }
        else
        {
            strcat(buf, lb);
        }
        Str(nb, ModelNumber, 0);
        strcat(buf, nb);
        strcat(buf, rb);
        strcat(buf, ModelName);

        uint64_t mac = ModelsMacUnionSaved.Val64 & 0x0000FFFFFFFFFFFF;
        ModelIDs[ModelNumber] = ModelsMacUnionSaved.Val64;

        if (mac)
        {
            snprintf(Vbuf, sizeof(Vbuf), " %012llX", mac);
            strcat(buf, Vbuf);

            int p = 0;
            for (unsigned int i = 1; i < MAXMODELNUMBER - 1; ++i)
            {
                if ((ModelIDs[i] == ModelIDs[ModelNumber]) && (i != ModelNumber))
                    p = i;
            }
            if (p > 0)
            {
                strcat(buf, KO);
                strcat(buf, Str(nb, p, 0));
                strcat(buf, "<");
                ++DuplicatesCount;
            }
            else
            {
                strcat(buf, Okay);
            }
        }
        strcat(buf, crlf);
    }
    SendOtherText(MMemsp, buf);
    SendValue(n0, DuplicatesCount);
    ModelNumber = SavedModelNumber;
    ReadOneModel(ModelNumber);
}

// *********************************************************************************************************************************/
void DisplayStoredID()
{
    char buf[20];
    if (ModelsMacUnionSaved.Val64)
    {
        sprintf(buf, "%012llX", ModelsMacUnionSaved.Val64 & 0x0000FFFFFFFFFFFF);
        SendText((char *)"t4", buf);
    }
    else
    {
        SendText((char *)"t4", (char *)"No ID stored");
    }
}

void DisplayCurrentID()
{
    char buf[20];
    if (ModelsMacUnion.Val64)
    {
        sprintf(buf, "%012llX", ModelsMacUnion.Val64 & 0x0000FFFFFFFFFFFF);
        SendText((char *)"t5", buf);
    }
    else
    {
        SendText((char *)"t5", (char *)"No ID received");
    }
}
/******************************************************************************************************************************/
void DeleteModelID()
{

    // This deletes current model ID
    char prompt[60];
    char p[] = "Delete ID for ";
    char p1[] = "?";
    char Done[] = "Model ID deleted.";
    char DoneAlready[] = "No model ID not found.";
    char NotDone[] = "Model ID retained.";

    strcpy(prompt, p);
    strcat(prompt, ModelName);
    strcat(prompt, p1);

    if (!ModelsMacUnionSaved.Val64)
    {
        MsgBox(pModelsIDView, DoneAlready);
        return;
    }
    if (GetConfirmation(pModelsIDView, prompt))
    {
        ModelsMacUnionSaved.Val64 = 0;
        SaveOneModel(ModelNumber);
        MsgBox(pModelsIDView, Done);
    }
    else
    {
        MsgBox(pModelsIDView, NotDone);
    }
    DisplayStoredID();
    DisplayCurrentID();
}
/*********************************************************************************************************************************/

void StoreModelID()
{ // This stores current model ID

    char prompt[60];
    char p[] = "Store ID for ";
    char p1[] = "?";
    char Done[] = "Model ID stored.";
    char NotDone[] = "Model ID not stored.";
    char DoneAlready[] = "ID was already stored.";
    char NotConnected[] = "No ID to store.";

    strcpy(prompt, p);
    strcat(prompt, ModelName);
    strcat(prompt, p1);

    if (ModelsMacUnion.Val64 == ModelsMacUnionSaved.Val64)
    {
        MsgBox(pModelsIDView, DoneAlready);
        return;
    }
    if (!ModelsMacUnion.Val64)
    {
        MsgBox(pModelsIDView, NotConnected);
        return;
    }

    if (GetConfirmation(pModelsIDView, prompt))
    {
        // ClearDuplicateModelIDs(ModelsMacUnion.Val64); // ensure no duplicates
        PlaySound(MMSAVED);
        ModelsMacUnionSaved.Val64 = ModelsMacUnion.Val64;
        SaveOneModel(ModelNumber);
        MsgBox(pModelsIDView, Done);
    }
    else
    {
        MsgBox(pModelsIDView, NotDone);
    }
    DisplayStoredID();
    DisplayCurrentID();
}
/// *********************************************************************************************************************************/
void StartModelIDView()
{
    SendCommand((char *)"page IDsStartView");
    CurrentView = MODELIDVIEW;
    SendText((char *)"t1", ModelName);
    DisplayStoredID();
    DisplayCurrentID();
}
// *********************************************************************************************************************************/
void EndModelIDView()
{
    GotoFrontView();
}

#endif // MODEL_IDS_H