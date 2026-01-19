// **********************************************************************************************************
// This file will handles Save & Restore all Rates and PID values for Rotorflight
// **********************************************************************************************************

#ifndef RF_SAVE_RESTORE_H
#define RF_SAVE_RESTORE_H
#include <Arduino.h>
#include "1Definitions.h"

uint8_t Saving_Parameters_Index = 0;    // index of parameter being saved
uint8_t Restoring_Parameters_Index = 0; // index of parameter being restored

// ************************************************************************************************************/
void Save_SOME_RF_Parameters()
{
    switch (Saving_Parameters_Index)
    {
    case 0:
        PID_Send_Duration = 1000;              // how many milliseconds to await PID values
        Reading_PIDS_Now = true;               // This tells the Ack payload parser to get PID values
        AddParameterstoQueue(SEND_PID_VALUES); // Request PID values from RX
        PID_Start_Time = millis();             // record start time as it's not long
        Saving_Parameters_Index = 1;           // move to next stage
        break;                                 //--
    case 1:                                    // wait until PIDs have been read
        if (!Reading_PIDS_Now)                 // wait until PIDs have been read
            Saving_Parameters_Index = 2;       // move to next stage when PIDs have been read
        break;                                 //--
    case 2:                                    // save PIDs for this bank
        Look("");
        Look1("Saving PIDs for Bank: ");
        Look(Bank);
        for (int i = 0; i < MAX_PID_WORDS; ++i)
        {
            Look1("Saving PID ");
            Look1(i);
            Look1(" Value: ");
            Look(PID_Values[i]);
            Saved_PID_Values[i][Bank - 1] = PID_Values[i];
        }
        Saving_Parameters_Index = 3;                    // move to next stage
        break;                                          //--
    case 3:                                             // advced PIDs
        PID_Advanced_Send_Duration = 1000;              // how many milliseconds to await PID Advanced values
        Reading_PIDS_Advanced_Now = true;               // This tells the Ack payload parser to get PID Advanced values
        AddParameterstoQueue(SEND_PID_ADVANCED_VALUES); // Request PID Advanced values from RX
        PID_Advanced_Start_Time = millis();             // record start time as it's not long
        Saving_Parameters_Index = 4;                    // move to next stage
        break;                                          //--
    case 4:                                             // wait until Advanced PIDs have been read
        if (!Reading_PIDS_Advanced_Now)                 // wait until Advanced PIDs have been read
            Saving_Parameters_Index = 5;                // move to next stage when Advanced PIDs have been read
        break;                                          //--
    case 5:                                             // save Advanced PIDs for this bank
        Look("");
        Look1("Saving Advanced PIDs for Bank: ");
        Look(Bank);
        for (int i = 0; i < MAX_PIDS_ADVANCED_BYTES; ++i)
        {
            Look1("Saving Advanced PID ");
            Look1(i);
            Look1(" Value: ");
            Look(PID_Advanced_Values[i]);
            Saved_PID_Advanced_Values[i][Bank - 1] = PID_Advanced_Values[i];
        }
        Saving_Parameters_Index = 6;             // move to next stage
    case 6:                                      // Rates here
        RATES_Send_Duration = 1000;              // how many milliseconds to await RATES values
        Reading_RATES_Now = true;                // This tells the Ack payload parser to get RATES values
        AddParameterstoQueue(SEND_RATES_VALUES); // Request RATES values from RX
        RATES_Start_Time = millis();             // record start time as it's not long
        Saving_Parameters_Index = 7;             // move to next stage
        break;                                   //--
    case 7:                                      // wait until RATES have been read
        if (!Reading_RATES_Now)                  // wait until RATES have been read
            Saving_Parameters_Index = 8;         // move to next stage when RATES have been read
        break;                                   //--
    case 8:                                      // save RATES for this bank
        Look("");
        Look1("Saving RATES for Bank: ");
        Look(Bank);
        for (int i = 0; i < MAX_RATES_BYTES; ++i)
        {
            Look1("Saving RATES ");
            Look1(i);
            Look1(" Value: ");
            Look(Rate_Values[i]);
            Saved_Rate_Values[i][Bank - 1] = Rate_Values[i];
        }
        Saving_Parameters_Index = 9;                      // move to next stage
        break;                                            //--
    case 9:                                               // advanced RATES
        Rates_Advanced_Send_Duration = 1000;              // how many milliseconds to await RATES Advanced values
        Reading_RATES_Advanced_Now = true;                // This tells the Ack payload parser to get RATES Advanced values
        AddParameterstoQueue(SEND_RATES_ADVANCED_VALUES); // Request RATES Advanced values from RX
        RATES_Advanced_Start_Time = millis();             // record start time as it's not long
        Saving_Parameters_Index = 10;                     // move to next stage
        break;                                            //--
    case 10:                                              // wait until Advanced RATES have been read
        if (!Reading_RATES_Advanced_Now)                  // wait until Advanced RATES have been read
            Saving_Parameters_Index = 11;                 // move to next stage when Advanced RATES have been read
        break;                                            //--
    case 11:                                              // save Advanced RATES for this bank
        Look("");
        Look1("Saving Advanced RATES for Bank: ");
        Look(Bank);
        for (int i = 0; i < MAX_RATES_ADVANCED_BYTES; ++i)
        {
            Look1("Saving Advanced RATES ");
            Look1(i);
            Look1(" Value: ");
            Look(Rate_Advanced_Values[i]);
            Saved_Rate_Advanced_Values[i][Bank - 1] = Rate_Advanced_Values[i];
        }
        Saving_Parameters_Index = 0; // move to next bank
        ++Bank;
        if (Bank > 4)
        {
            Saving_Parameters_Index = 200; // all done
            break;
        }
        else
        {
            if (AnnounceBanks)
                SoundBank();
        }
        break; //--

    case 200:
        Look("");
        Look1("Save_SOME_RF_Parameters completed. ");
        CurrentMode = NORMAL;

    default:

        break;
    }
}
// ************************************************************************************************************/
void Restore_SOME_RF_Parameters()
{
    Look1("Restore_SOME_RF_Parameters called  ");
    Look(millis());
}
// ************************************************************************************************************/
void SaveRFParameters()
{
    // if (GetConfirmation((char *)"page RFView", (char *)"Save ALL these values?"))
    // {
    Saving_Parameters_Index = 0;
    Bank = 1;
    if (AnnounceBanks)
        SoundBank();
    CurrentMode = SAVE_RF_SETTINGS;
    // }
}
// ************************************************************************************************************/
void RestoreRFParameters()
{
    if (GetConfirmation((char *)"page RFView", (char *)"Restore ALL these values?"))
    {
        MsgBox((char *)"page RFView", (char *)"Implementation coming soon!");
        Restoring_Parameters_Index = 0;
        CurrentMode = RESTORE_RF_SETTINGS;
    }
}

// ************************************************************************************************************/

#endif // RF_SAVE_RESTORE_H