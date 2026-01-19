// **********************************************************************************************************
// This file will handles Save & Restore all Rates and PID values for Rotorflight
// **********************************************************************************************************

#ifndef RF_SAVE_RESTORE_H
#define RF_SAVE_RESTORE_H
#include <Arduino.h>
#include "1Definitions.h"
#define WAIT_TIME_BETWEEN_PARAMETERS 1000         // milliseconds to wait between receiving parameter blocks
#define WAIT_TIME_BETWEEN_WRITING_PARAMETERS 1000 // milliseconds to wait between sending parameter blocks
uint8_t Parameter_Progress_Index = 0;             // index of parameter being saved/restored
uint16_t ProgressSoFar = 0;                       // progress bar value
uint16_t OneProgressItem = 100 / 16;              // total steps is 16 for progress bar

// ************************************************************************************************************/

void Restore_SOME_RF_Parameters()
{
    char msg[70];
    char t2[20] = "t2";
    char NB[10];
    Str(NB, Bank, 0);
    static uint32_t LTimer = 0;
    switch (Parameter_Progress_Index)
    {
    case 0:
        strcpy(msg, "Restoring PIDs for Bank ");
        strcat(msg, NB);
        SendText(t2, msg);
        for (int i = 0; i < MAX_PID_WORDS; ++i)
            PID_Values[i] = Saved_PID_Values[i][Bank - 1];
        AddParameterstoQueue(GET_SECOND_6_PID_VALUES); // SECOND MUST BE QUEUED FIRST!!! Send PID 7-12 values from TX to RX
        AddParameterstoQueue(GET_FIRST_6_PID_VALUES);  // SECOND MUST BE QUEUED FIRST!!! Send PID 1-6 values from TX to RX
        ProgressSoFar += OneProgressItem;              // update progress bar
        SendValue((char *)"Progress", ProgressSoFar);  // update progress bar
        Parameter_Progress_Index = 1;
        LTimer = millis();
        break;
    case 1:
        if ((millis() - LTimer) >= WAIT_TIME_BETWEEN_WRITING_PARAMETERS) // wait to allow PIDs to be sent
            Parameter_Progress_Index = 2;
        break;
    case 2:
        strcpy(msg, "Restoring PIDs Advanced for Bank ");
        strcat(msg, NB);
        SendText(t2, msg);
        for (int i = 0; i < MAX_PIDS_ADVANCED_BYTES; ++i)
            PID_Advanced_Values[i] = Saved_PID_Advanced_Values[i][Bank - 1];
        AddParameterstoQueue(GET_THIRD_8_ADVANCED_PID_VALUES);  // LAST MUST BE QUEUED FIRST!!!
        AddParameterstoQueue(GET_SECOND_9_ADVANCED_PID_VALUES); // the order of the other two doesn't matter ...
        AddParameterstoQueue(GET_FIRST_9_ADVANCED_PID_VALUES);  // its a LIFO stack with 26 parametres to send
        ProgressSoFar += OneProgressItem;                       // update progress bar
        SendValue((char *)"Progress", ProgressSoFar);           // update progress bar
        Parameter_Progress_Index = 3;
        LTimer = millis();
        break;
    case 3:
        if ((millis() - LTimer) >= WAIT_TIME_BETWEEN_WRITING_PARAMETERS) // wait to allow Advanced PIDs to be sent
            Parameter_Progress_Index = 4;
        break;
    case 4:
        strcpy(msg, "Restoring Rates for Bank ");
        strcat(msg, NB);
        SendText(t2, msg);
        for (int i = 0; i < MAX_RATES_BYTES; ++i)
            Rate_Values[i] = Saved_Rate_Values[i][Bank - 1];
        AddParameterstoQueue(GET_SECOND_6_RATES_VALUES); // SECOND MUST BE QUEUED FIRST!!! Send RATES 7-12 values from TX to RX
        AddParameterstoQueue(GET_FIRST_7_RATES_VALUES);  // SECOND MUST BE QUEUED FIRST!!! Send RATES 1-6 values from TX to RX
        ProgressSoFar += OneProgressItem;                // update progress bar
        SendValue((char *)"Progress", ProgressSoFar);    // update progress bar
        Parameter_Progress_Index = 5;
        LTimer = millis();
        break;
    case 5:
        if ((millis() - LTimer) >= WAIT_TIME_BETWEEN_WRITING_PARAMETERS) // wait to allow RATES to be sent
            Parameter_Progress_Index = 6;
        break;
    case 6:
        strcpy(msg, "Restoring Rates Advanced for Bank ");
        strcat(msg, NB);
        SendText(t2, msg);
        for (int i = 0; i < MAX_RATES_ADVANCED_BYTES; ++i)
            Rate_Advanced_Values[i] = Saved_Rate_Advanced_Values[i][Bank - 1];
        AddParameterstoQueue(GET_RATES_ADVANCED_VALUES_SECOND_8); // Send RATES ADVANCED values from TX to RX
        AddParameterstoQueue(GET_RATES_ADVANCED_VALUES_FIRST_7);  // Send RATES ADVANCED values from TX to RX ...because this queue is a LIFO stack
        ProgressSoFar += OneProgressItem;                         // update progress bar
        SendValue((char *)"Progress", ProgressSoFar);             // update progress bar
        Parameter_Progress_Index = 7;
        LTimer = millis();
        break;
    case 7:
        if ((millis() - LTimer) >= WAIT_TIME_BETWEEN_WRITING_PARAMETERS) // wait to allow
            Parameter_Progress_Index = 8;
        break;
    case 8:
        ++Bank;
        if (Bank > 4)
        {
            Parameter_Progress_Index = 200; // all done
            break;
        }
        else
        {
            Parameter_Progress_Index = 0; // move to next bank
        }
        break;
    case 200:
        SendText(t2, (char *)"Success!");
        SendValue((char *)"Progress", 100); // update progress bar
        CurrentMode = NORMAL;               // no further calls will come here
        SaveOneModel(ModelNumber);          // save all to SD card
        PlaySound(BEEPCOMPLETE);
        DelayWithDog(1000);
        SendCommand((char *)"vis Progress,0"); // hide progress bar
        SendCommand((char *)"vis t2,0");       // hide please wait text
        break;
    default:
        break;
    }
}

// ************************************************************************************************************/
void RestoreRFParameters()
{
    if (GetConfirmation((char *)"page RFView", (char *)"Restore ALL these values?"))
    {
        ReadOneModel(ModelNumber);             // reload model to get saved values
        SendCommand((char *)"vis Progress,1"); // show progress bar
        SendCommand((char *)"vis t2,1");       // show please wait text
        ProgressSoFar = 1;
        Parameter_Progress_Index = 0;
        SendValue((char *)"Progress", ProgressSoFar);
        Bank = 1;
        CurrentMode = RESTORE_RF_SETTINGS;
    }
}

// ************************************************************************************************************/
void Save_SOME_RF_Parameters()
{
    char msg[70];
    char t2[20] = "t2";
    char NB[10];
    Str(NB, Bank, 0);
    switch (Parameter_Progress_Index)
    {
    case 0:
        strcpy(msg, "Saving PIDs for Bank ");
        strcat(msg, NB);
        SendText(t2, msg);
        PID_Send_Duration = WAIT_TIME_BETWEEN_PARAMETERS; // how many milliseconds to await PID values
        Reading_PIDS_Now = true;                          // This tells the Ack payload parser to get PID values
        AddParameterstoQueue(SEND_PID_VALUES);            // Request PID values from RX
        PID_Start_Time = millis();                        // record start time as it's not long
        Parameter_Progress_Index = 1;                     // move to next stage
        break;                                            //--
    case 1:                                               // wait until PIDs have been read
        if (!Reading_PIDS_Now)                            // wait until PIDs have been read
            Parameter_Progress_Index = 2;                 // move to next stage when PIDs have been read
        break;                                            //--
    case 2:                                               // save PIDs for this bank
        for (int i = 0; i < MAX_PID_WORDS; ++i)
        {
            Saved_PID_Values[i][Bank - 1] = PID_Values[i];
        }
        Parameter_Progress_Index = 3;                 // move to next stage
        ProgressSoFar += OneProgressItem;             // update progress bar
        SendValue((char *)"Progress", ProgressSoFar); // update progress bar
        break;                                        //--
    case 3:                                           // advced PIDs
        strcpy(msg, "Saving Advanced PIDs for Bank ");
        strcat(msg, NB);
        SendText(t2, msg);
        PID_Advanced_Send_Duration = WAIT_TIME_BETWEEN_PARAMETERS; // how many milliseconds to await PID Advanced values
        Reading_PIDS_Advanced_Now = true;                          // This tells the Ack payload parser to get PID Advanced values
        AddParameterstoQueue(SEND_PID_ADVANCED_VALUES);            // Request PID Advanced values from RX
        PID_Advanced_Start_Time = millis();                        // record start time as it's not long
        Parameter_Progress_Index = 4;                              // move to next stage
        break;                                                     //--
    case 4:                                                        // wait until Advanced PIDs have been read
        if (!Reading_PIDS_Advanced_Now)                            // wait until Advanced PIDs have been read
            Parameter_Progress_Index = 5;                          // move to next stage when Advanced PIDs have been read
        break;                                                     //--
    case 5:                                                        // save Advanced PIDs for this bank
        for (int i = 0; i < MAX_PIDS_ADVANCED_BYTES; ++i)
        {
            Saved_PID_Advanced_Values[i][Bank - 1] = PID_Advanced_Values[i];
        }
        Parameter_Progress_Index = 6;                 // move to next stage
        ProgressSoFar += OneProgressItem;             // update progress bar
        SendValue((char *)"Progress", ProgressSoFar); // update progress bar
        break;
    case 6: // Rates here
        strcpy(msg, "Saving Rates for Bank ");
        strcat(msg, NB);
        SendText(t2, msg);
        RATES_Send_Duration = WAIT_TIME_BETWEEN_PARAMETERS; // how many milliseconds to await RATES values
        Reading_RATES_Now = true;                           // This tells the Ack payload parser to get RATES values
        AddParameterstoQueue(SEND_RATES_VALUES);            // Request RATES values from RX
        RATES_Start_Time = millis();                        // record start time as it's not long
        Parameter_Progress_Index = 7;                       // move to next stage
        break;                                              //--
    case 7:                                                 // wait until RATES have been read
        if (!Reading_RATES_Now)                             // wait until RATES have been read
            Parameter_Progress_Index = 8;                   // move to next stage when RATES have been read
        break;                                              //--
    case 8:                                                 // save RATES for this bank
        for (int i = 0; i < MAX_RATES_BYTES; ++i)
        {
            Saved_Rate_Values[i][Bank - 1] = Rate_Values[i];
        }
        Parameter_Progress_Index = 9;                 // move to next stage
        ProgressSoFar += OneProgressItem;             // update progress bar
        SendValue((char *)"Progress", ProgressSoFar); // update progress bar
        break;                                        //--
    case 9:                                           // advanced RATES
        strcpy(msg, "Saving Advanced Rates for Bank ");
        strcat(msg, NB);
        SendText(t2, msg);
        Rates_Advanced_Send_Duration = WAIT_TIME_BETWEEN_PARAMETERS; // how many milliseconds to await RATES Advanced values
        Reading_RATES_Advanced_Now = true;                           // This tells the Ack payload parser to get RATES Advanced values
        AddParameterstoQueue(SEND_RATES_ADVANCED_VALUES);            // Request RATES Advanced values from RX
        RATES_Advanced_Start_Time = millis();                        // record start time as it's not long
        Parameter_Progress_Index = 10;                               // move to next stage
        break;                                                       //--
    case 10:                                                         // wait until Advanced RATES have been read
        if (!Reading_RATES_Advanced_Now)                             // wait until Advanced RATES have been read
            Parameter_Progress_Index = 11;                           // move to next stage when Advanced RATES have been read
        break;                                                       //--
    case 11:                                                         // save Advanced RATES for this bank
        for (int i = 0; i < MAX_RATES_ADVANCED_BYTES; ++i)
        {
            Saved_Rate_Advanced_Values[i][Bank - 1] = Rate_Advanced_Values[i];
        }
        ProgressSoFar += OneProgressItem;             // update progress bar
        SendValue((char *)"Progress", ProgressSoFar); // update progress bar
        ++Bank;
        if (Bank > 4)
        {
            Parameter_Progress_Index = 200; // all done
            break;
        }
        else
        {
            Parameter_Progress_Index = 0; // move to next bank
        }
        break;

    case 200:
        SendText(t2, (char *)"Success!");
        SendValue((char *)"Progress", 100); // update progress bar
        CurrentMode = NORMAL;               // no further calls will come here
        SaveOneModel(ModelNumber);          // save all to SD card
        PlaySound(BEEPCOMPLETE);
        DelayWithDog(1000);
        SendCommand((char *)"vis Progress,0"); // hide progress bar
        SendCommand((char *)"vis t2,0");       // hide please wait text
        break;
    default:
        break;
    }
}
// ************************************************************************************************************/
void SaveRFParameters()
{
    if (LedWasGreen == false)
    {
        MsgBox((char *)"page RFView", (char *)"Please connect first!");
        return;
    }
    if (GetConfirmation((char *)"page RFView", (char *)"Save ALL these values?"))
    {
        Parameter_Progress_Index = 0;
        SendCommand((char *)"vis Progress,1"); // show progress bar
        SendCommand((char *)"vis t2,1");       // show please wait text
        ProgressSoFar = 1;
        SendValue((char *)"Progress", ProgressSoFar);
        Bank = 1;
        CurrentMode = SAVE_RF_SETTINGS;
    }
}

// ************************************************************************************************************/

#endif // RF_SAVE_RESTORE_H