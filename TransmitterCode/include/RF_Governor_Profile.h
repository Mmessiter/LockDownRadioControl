// ******************************************** Rotorflight Governor (Profile) ****************************************
// This module handles the Rotorflight Governor Profile screen on the Nextion display.
// Shows 14 profile-specific governor parameters (n0-n13).
// Global/config parameters are handled separately in GovConfig.h.
// **********************************************************************************************************

#ifndef GOVERNOR_H
#define GOVERNOR_H
#define GOVERNOR_LABELS_COUNT 14 // 14 profile fields only (n0-n13)
#include <Arduino.h>
#include "1Definitions.h"

char GOV_Labels[GOVERNOR_LABELS_COUNT][4] = {
    "n0", "n1", "n2", "n3", "n4", "n5", "n6",
    "n7", "n8", "n9", "n10", "n11", "n12", "n13"};

// ====================================================
// ShowLocalGovBank()
// Shows saved governor profile values from SD card when not connected
// ====================================================
void ShowLocalGovBank()
{
    for (int i = 0; i < GOVERNOR_LABELS_COUNT; ++i)
    {
        uint16_t v;
        switch (i)
        {
        case 0: // Headspeed U16
            v = (uint16_t)Saved_GOV_Profiles_Values[1][Bank - 1] |
                ((uint16_t)Saved_GOV_Profiles_Values[2][Bank - 1] << 8);
            SendValue(GOV_Labels[0], v);
            break;
        default: // single byte fields — payload index is i+2 (bytes 3-15 → labels 1-13)
            SendValue(GOV_Labels[i], Saved_GOV_Profiles_Values[i + 2][Bank - 1]);
            break;
        }
    }
    HideGOVMsg();
    BlockBankChanges = false;
}

// ====================================================
// SaveToLocalGovBank()
// Saves edited governor profile values to SD card when not connected
// ====================================================
void SaveToLocalGovBank()
{
    ShowGOVMsg((char *)"Saving governor values ...", Gray);
    LoadGovWritePayload(); // read from Nextion into GovWritePayload[]

    // Copy into saved array for current bank
    for (int i = 0; i < GOV_PROFILE_PAYLOAD_SIZE; ++i)
        Saved_GOV_Profiles_Values[i][Bank - 1] = GovWritePayload[i];

    SaveOneModel(ModelNumber);       // save to SD card
    SendCommand((char *)"vis b3,0"); // hide Save button
    HideGOVMsg();
    GOVS_PROFILE_Were_Edited = false;
    PlaySound(BEEPCOMPLETE);
}

// ====================================================
// LoadGovWritePayload()
// Reads all 14 profile fields from Nextion into GovWritePayload[]
// Flags (b[16-17]) preserved from last read — not editable on this screen
// ====================================================
void LoadGovWritePayload()
{
    GovWritePayload[0] = 0; // RF23 flag — not written back

    uint16_t hs = (uint16_t)GetValue((char *)"n0");
    GovWritePayload[1] = (uint8_t)(hs & 0xFF);
    GovWritePayload[2] = (uint8_t)(hs >> 8);

    GovWritePayload[3] = (uint8_t)GetValue((char *)"n1");   // Gain
    GovWritePayload[4] = (uint8_t)GetValue((char *)"n2");   // P
    GovWritePayload[5] = (uint8_t)GetValue((char *)"n3");   // I
    GovWritePayload[6] = (uint8_t)GetValue((char *)"n4");   // D
    GovWritePayload[7] = (uint8_t)GetValue((char *)"n5");   // F
    GovWritePayload[8] = (uint8_t)GetValue((char *)"n6");   // TTA gain
    GovWritePayload[9] = (uint8_t)GetValue((char *)"n7");   // TTA limit
    GovWritePayload[10] = (uint8_t)GetValue((char *)"n8");  // Max throttle
    GovWritePayload[11] = (uint8_t)GetValue((char *)"n9");  // Min throttle
    GovWritePayload[12] = (uint8_t)GetValue((char *)"n10"); // Fallback drop
    GovWritePayload[13] = (uint8_t)GetValue((char *)"n11"); // Yaw weight
    GovWritePayload[14] = (uint8_t)GetValue((char *)"n12"); // Cyclic weight
    GovWritePayload[15] = (uint8_t)GetValue((char *)"n13"); // Collective weight

    // Flags — preserve existing values. Use saved values if not connected, GovAckPayload if connected
    GovWritePayload[16] = LedWasGreen ? GovAckPayload[16] : Saved_GOV_Profiles_Values[16][Bank - 1];
    GovWritePayload[17] = LedWasGreen ? GovAckPayload[17] : Saved_GOV_Profiles_Values[17][Bank - 1];
}

// ====================================================
// DisplayGovValues()
// Sends profile governor values to Nextion numeric fields n0-n13
// Called with byte range [n, m) from GovAckPayload[]
// ====================================================
void DisplayGovValues(uint8_t n, uint8_t m)
{
    if (CurrentView != RFGOVERNORVIEW_PROFILE)
        return;

    for (uint8_t i = n; i < m; ++i)
    {
        if (i >= GOV_ACK_PAYLOAD_SIZE)
            break;

        switch (i)
        {
        case 1:
        {
            uint16_t v = (uint16_t)GovAckPayload[1] | ((uint16_t)GovAckPayload[2] << 8);
            SendValue((char *)"n0", v);
            break;
        }
        case 2:
            break; // high byte of Headspeed — handled above
        case 3:
            SendValue((char *)"n1", GovAckPayload[i]);
            break; // Gain
        case 4:
            SendValue((char *)"n2", GovAckPayload[i]);
            break; // P
        case 5:
            SendValue((char *)"n3", GovAckPayload[i]);
            break; // I
        case 6:
            SendValue((char *)"n4", GovAckPayload[i]);
            break; // D
        case 7:
            SendValue((char *)"n5", GovAckPayload[i]);
            break; // F
        case 8:
            SendValue((char *)"n6", GovAckPayload[i]);
            break; // TTA gain
        case 9:
            SendValue((char *)"n7", GovAckPayload[i]);
            break; // TTA limit
        case 10:
            SendValue((char *)"n8", GovAckPayload[i]);
            break; // Max throttle
        case 11:
            SendValue((char *)"n9", GovAckPayload[i]);
            break; // Min throttle
        case 12:
            SendValue((char *)"n10", GovAckPayload[i]);
            break; // Fallback drop
        case 13:
            SendValue((char *)"n11", GovAckPayload[i]);
            break; // Yaw weight
        case 14:
            SendValue((char *)"n12", GovAckPayload[i]);
            break; // Cyclic weight
        case 15:
            SendValue((char *)"n13", GovAckPayload[i]);
            break; // Collective weight
        default:
            break;
        }
    }
}

// ====================================================
void ForegroundColourGOVLabels(uint16_t Colour)
{
    for (int i = 0; i < GOVERNOR_LABELS_COUNT; ++i)
        SendForegroundColour(GOV_Labels[i], Colour);
}

// ====================================================
void HideGOVMsg()
{
    if (CurrentView == RFGOVERNORVIEW_PROFILE)
    {
        SendCommand((char *)"vis busy,0");
        SendCommand((char *)"vis b2,1");
        ForegroundColourGOVLabels(Black);
        BlockBankChanges = false;
    }
}

// ====================================================
void ShowGOVMsg(const char *msg, uint16_t Colour)
{
    if (CurrentView == RFGOVERNORVIEW_PROFILE)
    {
        ForegroundColourGOVLabels(Colour);
        SendText((char *)"busy", (char *)msg);
        SendCommand((char *)"vis busy,1");
        SendCommand((char *)"vis b2,0");
        BlockBankChanges = true;
    }
}
// ====================================================
void FixHeading()
{
    char temp[60];
    char NB[10];
    strcpy(temp, "RF Governor (Profile: ");
    strcat(temp, Str(NB, Bank, 0));
    strcat(temp, ")");
    SendText((char *)"t0", temp);
}

// ====================================================
void ShowGOVBank()
{
    if (CurrentView == RFGOVERNORVIEW_PROFILE)
    {
        char buf[40];

        // Clear all 14 profile fields to 0 before loading fresh values
        for (int i = 0; i < GOVERNOR_LABELS_COUNT; ++i)
            SendValue(GOV_Labels[i], 0);

        if (GOVS_PROFILE_Were_Edited)
        {
            char NB[10];
            char Wmsg[120];
            char w1[] = "Governor values for Bank ";
            char w2[] = " were edited \r\nbut not saved. (Too late now!)\r\nSo you may want to check them.";
            strcpy(Wmsg, w1);
            strcat(Wmsg, Str(NB, PreviousBank, 0));
            strcat(Wmsg, w2);
            MsgBox((char *)"page RFGovView", Wmsg);
        }

        strcpy(buf, "Loading governor values ...");
        SendText((char *)"t26", BankNames[BanksInUse[Bank - 1]]);
        FixHeading();
        if (!LedWasGreen)
        {
            ShowGOVMsg(buf, Gray);
            GOVS_PROFILE_Were_Edited = false; // reset before showing local values
            ShowLocalGovBank();
            return;
        }

        BlockBankChanges = true;
        ShowGOVMsg(buf, Gray);
        GOV_Send_Duration = GOV_PROFILE_WAIT_TIME;
        Reading_GOV_Now = true;
        AddParameterstoQueue(SEND_GOV_VALUES);
        GOVS_PROFILE_Were_Edited = false;
        SendCommand((char *)"vis b3,0"); // hide Save button
        GOV_Start_Time = millis();
    }
}

// ====================================================
void Start_RF_Governor()
{
    if (RotorFlight_Version < 2.3)
    {
        char msg[160];
        char w1[] = "RotorFlight V2.3 is needed for all\r\ngovernor tuning features.\r\n\r\nYour current version is only V";
        char w2[] = ".\r\nPlease update your flight controller!";
        char Vbuf[10];
        snprintf(Vbuf, 10, "%1.1f", RotorFlight_Version);
        strcpy(msg, w1);
        strcat(msg, Vbuf);
        strcat(msg, w2);
        MsgBox((char *)"page RFView", msg);
        return;
    }
    AddParameterstoQueue(MSP_INHIBIT_TELEMETRY);
    SendCommand((char *)"page RFGovView");
    CurrentView = RFGOVERNORVIEW_PROFILE;
    SendText((char *)"t27", ModelName);
    FixHeading();
    ShowGOVBank();
}

// ====================================================
void End_RF_Governor()
{
    if (GOVS_PROFILE_Were_Edited)
    {
        if (GetConfirmation((char *)"page RFGovView", (char *)"Discard edited governor values?"))
        {
            GOVS_PROFILE_Were_Edited = false;
            AddParameterstoQueue(MSP_ENABLE_TELEMETRY);
            RotorFlightStart();
        }
        // if user says No, stay on page
    }
    else
    {
        AddParameterstoQueue(MSP_ENABLE_TELEMETRY);
        RotorFlightStart();
    }
}

// ====================================================
void SendEditedGovValues()
{
    if (!LedWasGreen)
    {
        // Not connected — save to local SD card instead
        SaveToLocalGovBank();
        return;
    }

    if (SendBuffer[ArmingChannel - 1] > 1000)
    {
        PlaySound(WHAHWHAHMSG);
        MsgBox((char *)"page RFGovView", (char *)"Model is armed!\r\nDisarm before writing governor values.");
        return;
    }

    PlaySound(BEEPMIDDLE);
    DelayWithDog(100);
    LoadGovWritePayload(); // read all 14 profile fields from Nextion

    // Queue in reverse order — LIFO executes PROFILE1 first, PROFILE2 second
    AddParameterstoQueue(SEND_GOV_WRITE_PROFILE2); // executes 2nd
    AddParameterstoQueue(SEND_GOV_WRITE_PROFILE1); // executes 1st

    GOVS_PROFILE_Were_Edited = false;
    SendCommand((char *)"vis b3,0"); // hide Save button
    PlaySound(BEEPCOMPLETE);
}

// ====================================================
void GOVS_P_Were_Edited()
{
    SendCommand((char *)"vis b3,1"); // show Save button
    GOVS_PROFILE_Were_Edited = true;
}

// ====================================================
void Save_RF_Governor()
{
    SendEditedGovValues();
}

#endif // GOVERNOR_H
