// **********************************************************************************************************
// This file will handles Save & Restore all Rates and PID values for Rotorflight
// **********************************************************************************************************

#ifndef RF_SAVE_RESTORE_H
#define RF_SAVE_RESTORE_H
#include <Arduino.h>
#include "1Definitions.h"
#define DO_PIDS 1
#define DO_PIDS_ADVANCED 2
#define DO_RATES 4
#define DO_RATES_ADVANCED 8
// ************************************************************************************************************/

uint16_t Which_Case_Now = 0;   // Which case we are up to in the state machine
uint16_t ProgressSoFar = 0;    // progress bar value
uint16_t OneProgressItem;      // total steps is 16 for progress bar
uint8_t Which_Params = 0;      // bitmask of which parameters to save or restore
uint8_t LocalBank = 1;         // bank being saved to or restored from
uint8_t SelectedItemCount = 0; // number of items selected to save or restore

// ************************************************************************************************************/
uint8_t CountSelectedParams(uint8_t mask)
{
    uint8_t n = 0;
    if (mask & DO_PIDS)
        n++;
    if (mask & DO_PIDS_ADVANCED)
        n++;
    if (mask & DO_RATES)
        n++;
    if (mask & DO_RATES_ADVANCED)
        n++;
    return n;
}
//  ************************************************************************************************************/
void Restore_SOME_RF_Parameters()
{
    char msg[120];
    char t2[20] = "t2";
    char NB[10];
    char NB1[10];
    Str(NB, Bank, 0);
    Str(NB1, LocalBank, 0);
    static uint32_t LTimer = 0;
    if (!SelectedItemCount)
    {
        SendCommand((char *)"vis Progress,0"); // hide progress bar
        SendCommand((char *)"vis t2,0");       // hide please wait text
        CurrentMode = NORMAL;
        return;
    }
    switch (Which_Case_Now)
    {
    case 0:
        if (!(Which_Params & DO_PIDS))
        {
            Which_Case_Now = 2; // skip to next if not doing PIDs
            break;
        }
        strcpy(msg, "Restoring PIDs: LocalBank ");
        strcat(msg, NB1);
        strcat(msg, " to FC Bank ");
        strcat(msg, NB);
        SendText(t2, msg);

        for (int i = 0; i < MAX_PID_WORDS; ++i)
            PID_Values[i] = Saved_PID_Values[i][LocalBank - 1];
        for (int i = 0; i < 3; ++i)
            PID_Boost_Values[i] = Saved_PID_Values[i + MAX_PID_WORDS][LocalBank - 1];
        for (int i = 0; i < 2; ++i)
            PID_HSI_Offset_Values[i] = Saved_PID_Values[i + MAX_PID_WORDS + 3][LocalBank - 1];

        AddParameterstoQueue(GET_SECOND_11_PID_VALUES); // SECOND MUST BE QUEUED FIRST!!! Send PID 7-12 values from TX to RX
        AddParameterstoQueue(GET_FIRST_6_PID_VALUES);   // SECOND MUST BE QUEUED FIRST!!! Send PID 1-6 values from TX to RX
        ProgressSoFar += OneProgressItem;               // update progress bar
        SendValue((char *)"Progress", ProgressSoFar);   // update progress bar
        Which_Case_Now = 1;
        LTimer = millis();
        break;
    case 1:
        if ((millis() - LTimer) >= MSP_WAIT_TIME) // wait to allow PIDs to be sent
            Which_Case_Now = 2;
        break;
    case 2:
        if (!(Which_Params & DO_PIDS_ADVANCED))
        {
            Which_Case_Now = 4; // skip to next if not doing Advanced PIDs
            break;
        }
        strcpy(msg, "Restoring PIDs Advanced: LocalBank ");
        strcat(msg, NB1);
        strcat(msg, " to FC Bank ");
        strcat(msg, NB);
        SendText(t2, msg);
        for (int i = 0; i < MAX_PIDS_ADVANCED_BYTES; ++i)
            PID_Advanced_Values[i] = Saved_PID_Advanced_Values[i][LocalBank - 1];
        AddParameterstoQueue(GET_THIRD_8_ADVANCED_PID_VALUES);  // LAST MUST BE QUEUED FIRST!!!
        AddParameterstoQueue(GET_SECOND_9_ADVANCED_PID_VALUES); // the order of the other two doesn't matter ...
        AddParameterstoQueue(GET_FIRST_9_ADVANCED_PID_VALUES);  // its a LIFO stack with 26 parametres to send
        ProgressSoFar += OneProgressItem;                       // update progress bar
        SendValue((char *)"Progress", ProgressSoFar);           // update progress bar
        Which_Case_Now = 3;
        LTimer = millis();
        break;
    case 3:
        if ((millis() - LTimer) >= MSP_WAIT_TIME) // wait to allow Advanced PIDs to be sent
            Which_Case_Now = 4;
        break;
    case 4:
        if (!(Which_Params & DO_RATES))
        {
            Which_Case_Now = 6; // skip to next if not doing RATES
            break;
        }
        strcpy(msg, "Restoring Rates: LocalRate ");
        strcat(msg, NB1);
        strcat(msg, " to FC Rates ");
        strcat(msg, NB);
        SendText(t2, msg);
        for (int i = 0; i < MAX_RATES_BYTES; ++i)
            Rate_Values[i] = Saved_Rate_Values[i][LocalBank - 1];
        AddParameterstoQueue(GET_SECOND_6_RATES_VALUES); // SECOND MUST BE QUEUED FIRST!!! Send RATES from TX to RX
        AddParameterstoQueue(GET_FIRST_7_RATES_VALUES);  // SECOND MUST BE QUEUED FIRST!!! Send RATES from TX to RX
        ProgressSoFar += OneProgressItem;                // update progress bar
        SendValue((char *)"Progress", ProgressSoFar);    // update progress bar
        Which_Case_Now = 5;
        LTimer = millis();
        break;
    case 5:
        if ((millis() - LTimer) >= MSP_WAIT_TIME) // wait to allow RATES to be sent
            Which_Case_Now = 6;
        break;
    case 6:
        if (!(Which_Params & DO_RATES_ADVANCED))
        {
            Which_Case_Now = 150; // skip to next if not doing RATES ADVANCED
            break;
        }
        strcpy(msg, "Restoring Rates Advanced: LocalRates ");
        strcat(msg, NB1);
        strcat(msg, " to FC Rates ");
        strcat(msg, NB);
        SendText(t2, msg);
        for (int i = 0; i < MAX_RATES_ADVANCED_BYTES; ++i)
        {
            Rate_Advanced_Values[i] = Saved_Rate_Advanced_Values[i][LocalBank - 1];
        }
        AddParameterstoQueue(GET_RATES_ADVANCED_VALUES_SECOND_8); // Send RATES ADVANCED values from TX to RX
        AddParameterstoQueue(GET_RATES_ADVANCED_VALUES_FIRST_7);  // Send RATES ADVANCED values from TX to RX ...because this queue is a LIFO stack
        ProgressSoFar += OneProgressItem;                         // update progress bar
        SendValue((char *)"Progress", ProgressSoFar);             // update progress bar
        Which_Case_Now = 150;
        LTimer = millis();
        break;

    case 150:
        if ((millis() - LTimer) >= MSP_WAIT_TIME) // wait to allow RATES ADVANCED to be sent
        {
            LTimer = millis();
            Which_Case_Now = 200;
            SendValue((char *)"Progress", 100); // update progress bar
        }
        break;

    case 200:
        if ((millis() - LTimer) >= MSP_WAIT_TIME) // wait
        {
            LTimer = millis();
            Which_Case_Now = 400;
        }
        break;
    case 400:
        SendText(t2, (char *)"Success!");
        CurrentMode = NORMAL;               // no further calls will come here
        PlaySound(BEEPCOMPLETE);
        SendCommand((char *)"vis Progress,0"); // hide progress bar
        SendCommand((char *)"vis t2,0");       // hide please wait text
        BlockBankChanges = false;
    default:
        break;
    }
}

// ************************************************************************************************************/
void Save_SOME_RF_Parameters()
{
    char msg[120];
    char t2[20] = "t2";
    char NB[10];
    char NB1[10];
    Str(NB, Bank, 0);
    Str(NB1, LocalBank, 0);

    if (!SelectedItemCount)
    {
        SendCommand((char *)"vis Progress,0"); // hide progress bar
        SendCommand((char *)"vis t2,0");       // hide please wait text
        CurrentMode = NORMAL;
        return;
    }
    switch (Which_Case_Now)
    {
    case 0:
        if (!(Which_Params & DO_PIDS))
        {
            Which_Case_Now = 3; // skip to next if not doing PIDs
            break;
        }
        strcpy(msg, "Saving PIDs: FC Bank ");
        strcat(msg, NB);
        strcat(msg, " to LocalBank ");
        strcat(msg, NB1);

        SendText(t2, msg);
        PID_Send_Duration = MSP_WAIT_TIME;     // how many milliseconds to await PID values
        Reading_PIDS_Now = true;               // This tells the Ack payload parser to get PID values
        AddParameterstoQueue(SEND_PID_VALUES); // Request PID values from RX
        PID_Start_Time = millis();             // record start time as it's not long
        Which_Case_Now = 1;                    // move to next stage
        break;                                 //--
    case 1:                                    // wait until PIDs have been read
        if (!Reading_PIDS_Now)                 // wait until PIDs have been read
            Which_Case_Now = 2;                // move to next stage when PIDs have been read
        break;                                 //--
    case 2:                                    // save PIDs for this LocalBank
        for (int i = 0; i < MAX_PID_WORDS; ++i)
            Saved_PID_Values[i][LocalBank - 1] = PID_Values[i];
        for (int i = 0; i < 3; ++i)
            Saved_PID_Values[i + MAX_PID_WORDS][LocalBank - 1] = PID_Boost_Values[i];
        for (int i = 0; i < 2; ++i)
            Saved_PID_Values[i + MAX_PID_WORDS + 3][LocalBank - 1] = PID_HSI_Offset_Values[i];

        Which_Case_Now = 3;                           // move to next stage
        ProgressSoFar += OneProgressItem;             // update progress bar
        SendValue((char *)"Progress", ProgressSoFar); // update progress bar
        break;                                        //--
    case 3:                                           // advced PIDs
        if (!(Which_Params & DO_PIDS_ADVANCED))
        {
            Which_Case_Now = 6; // skip to next if not doing Advanced PIDs
            break;
        }
        strcpy(msg, "Saving Advanced PIDs: FC Bank ");
        strcat(msg, NB);
        strcat(msg, " to LocalBank ");
        strcat(msg, NB1);
        SendText(t2, msg);
        PID_Advanced_Send_Duration = MSP_WAIT_TIME;     // how many milliseconds to await PID Advanced values
        Reading_PIDS_Advanced_Now = true;               // This tells the Ack payload parser to get PID Advanced values
        AddParameterstoQueue(SEND_PID_ADVANCED_VALUES); // Request PID Advanced values from RX
        PID_Advanced_Start_Time = millis();             // record start time as it's not long
        Which_Case_Now = 4;                             // move to next stage
        break;                                          //--
    case 4:                                             // wait until Advanced PIDs have been read
        if (!Reading_PIDS_Advanced_Now)                 // wait until Advanced PIDs have been read
            Which_Case_Now = 5;                         // move to next stage when Advanced PIDs have been read
        break;                                          //--
    case 5:                                             // save Advanced PIDs for this LocalBank
        for (int i = 0; i < MAX_PIDS_ADVANCED_BYTES; ++i)
            Saved_PID_Advanced_Values[i][LocalBank - 1] = PID_Advanced_Values[i];
        Which_Case_Now = 6;                           // move to next stage
        ProgressSoFar += OneProgressItem;             // update progress bar
        SendValue((char *)"Progress", ProgressSoFar); // update progress bar
        break;
    case 6: // Rates here
        if (!(Which_Params & DO_RATES))
        {
            Which_Case_Now = 9; // skip to next if not doing RATES
            break;
        }
        strcpy(msg, "Saving Rates: FC Rates ");
        strcat(msg, NB);
        strcat(msg, " to LocalRates ");
        strcat(msg, NB1);
        SendText(t2, msg);
        RATES_Send_Duration = MSP_WAIT_TIME;     // how many milliseconds to await RATES values
        Reading_RATES_Now = true;                // This tells the Ack payload parser to get RATES values
        AddParameterstoQueue(SEND_RATES_VALUES); // Request RATES values from RX
        RATES_Start_Time = millis();             // record start time as it's not long
        Which_Case_Now = 7;                      // move to next stage
        break;                                   //--
    case 7:                                      // wait until RATES have been read
        if (!Reading_RATES_Now)                  // wait until RATES have been read
            Which_Case_Now = 8;                  // move to next stage when RATES have been read
        break;                                   //--
    case 8:                                      // save RATES for this LocalBank
        for (int i = 0; i < MAX_RATES_BYTES; ++i)
            Saved_Rate_Values[i][LocalBank - 1] = Rate_Values[i];
        Which_Case_Now = 9;                           // move to next stage
        ProgressSoFar += OneProgressItem;             // update progress bar
        SendValue((char *)"Progress", ProgressSoFar); // update progress bar
        break;                                        //--
    case 9:                                           // advanced RATES
        if (!(Which_Params & DO_RATES_ADVANCED))
        {
            Which_Case_Now = 200; // skip to next if not doing Advanced RATES
            break;
        }
        strcpy(msg, "Saving Advanced Rates: FC Rates ");
        strcat(msg, NB);
        strcat(msg, " to LocalRates ");
        strcat(msg, NB1);
        SendText(t2, msg);
        Rates_Advanced_Send_Duration = MSP_WAIT_TIME;     // how many milliseconds to await RATES Advanced values
        Reading_RATES_Advanced_Now = true;                // This tells the Ack payload parser to get RATES Advanced values
        AddParameterstoQueue(SEND_RATES_ADVANCED_VALUES); // Request RATES Advanced values from RX
        RATES_Advanced_Start_Time = millis();             // record start time as it's not long
        Which_Case_Now = 10;                              // move to next stage
        break;                                            //--
    case 10:                                              // wait until Advanced RATES have been read
        if (!Reading_RATES_Advanced_Now)                  // wait until Advanced RATES have been read
            Which_Case_Now = 11;                          // move to next stage when Advanced RATES have been read
        break;                                            //--
    case 11:                                              // save Advanced RATES for this LocalBank
        for (int i = 0; i < MAX_RATES_ADVANCED_BYTES; ++i)
        {
            Saved_Rate_Advanced_Values[i][LocalBank - 1] = Rate_Advanced_Values[i];
        }
        Rates_Advanced_Send_Duration = MSP_WAIT_TIME; // how many milliseconds to await RATES Advanced values
        SendValue((char *)"Progress", 100);           // update progress bar
        Reading_RATES_Advanced_Now = true;            // This tells the Ack payload parser to get RATES Advanced values .... extra sec
        RATES_Advanced_Start_Time = millis();         // record start time as it's not long
        Which_Case_Now = 200;                         // move to next bank
        break;
    case 200:
        if (!Reading_RATES_Advanced_Now) // wait until Advanced RATES have been read
            Which_Case_Now = 400;         // move to next stage when Advanced RATES have been read
        break;

    case 400:
        SendText(t2, (char *)"Success!");
        CurrentMode = NORMAL;               // no further calls will come here
        SaveOneModel(ModelNumber);          // save all to SD card
        PlaySound(BEEPCOMPLETE);
        SendCommand((char *)"vis Progress,0"); // hide progress bar
        SendCommand((char *)"vis t2,0");       // hide please wait text
        BlockBankChanges = false;

    default:
        break;
    }
}
// ************************************************************************************************************/
void Collect_data_from_dialog() // and close it
{
    LocalBank = GetValue((char *)"n0");
    Which_Params = 0;
    if (GetValue((char *)"sw0"))
        Which_Params |= DO_PIDS;
    if (GetValue((char *)"sw1"))
        Which_Params |= DO_PIDS_ADVANCED;
    if (GetValue((char *)"sw2"))
        Which_Params |= DO_RATES;
    if (GetValue((char *)"sw3"))
        Which_Params |= DO_RATES_ADVANCED;
    SelectedItemCount = CountSelectedParams(Which_Params);
    if (SelectedItemCount > 0)
        OneProgressItem = 99 / SelectedItemCount;
    RotorFlightStart();
}
// ************************************************************************************************************/
void RestoreRFParameters() // show dialog to pick bank and params to save
{
    if (!(LedWasGreen))
    {
        MsgBox((char *)"page RFView", (char *)"Not connected!");
        return;
    }
    SendCommand((char *)"page PickBankView1"); // Save is View2
    CurrentView = PICKBANKVIEW1;
    SendValue((char *)"n0", Bank);
    SendValue((char *)"n1", Bank);
}

// ************************************************************************************************************/
void SaveRFParameters() // show dialog to pick bank and params to save
{
    if (!(LedWasGreen))
    {
        MsgBox((char *)"page RFView", (char *)"Not connected!");
        return;
    }
    SendCommand((char *)"page PickBankView2"); // Restore is View1
    CurrentView = PICKBANKVIEW2;
    SendValue((char *)"n0", Bank);
    SendValue((char *)"n1", Bank);
}

// ************************************************************************************************************/
void Start_RESTORE()
{
    bool PreviousLinkRatesToBanks = LinkRatesToBanks;
    LinkRatesToBanks = true;
    BlockBankChanges = true;
    Collect_data_from_dialog();            // get bank number and which params to save
    Which_Case_Now = 0;                    // start saving first bank
    SendCommand((char *)"vis Progress,1"); // show progress bar
    SendCommand((char *)"vis t2,1");       // show please wait text
    ProgressSoFar = 1;
    SendValue((char *)"Progress", ProgressSoFar);
    DelayWithDog(MSP_WAIT_TIME); // allow time for progress bar to update before starting to send values to FC
    CurrentMode = RESTORE_RF_SETTINGS;
    LinkRatesToBanks = PreviousLinkRatesToBanks;
}
// ************************************************************************************************************/
void Start_SAVE()
{
    bool PreviousLinkRatesToBanks = LinkRatesToBanks;
    LinkRatesToBanks = true;

    BlockBankChanges = true;
    Collect_data_from_dialog();            // get bank number and which params to save
    Which_Case_Now = 0;                    // start saving first bank
    SendCommand((char *)"vis Progress,1"); // show progress bar
    SendCommand((char *)"vis t2,1");       // show please wait text
    ProgressSoFar = 1;
    SendValue((char *)"Progress", ProgressSoFar);
    DelayWithDog(MSP_WAIT_TIME); // allow time for progress bar to update before starting to send values to FC
    CurrentMode = SAVE_RF_SETTINGS;
    LinkRatesToBanks = PreviousLinkRatesToBanks;
}
// ************************************************************************************************************/
void Cancel_SAVE()
{
    RotorFlightStart();
    BlockBankChanges = false;
}
// ************************************************************************************************************/
void Cancel_RESTORE()
{
    RotorFlightStart();
    BlockBankChanges = false;
}

// ************************************************************************************************************/

#endif // RF_SAVE_RESTORE_H