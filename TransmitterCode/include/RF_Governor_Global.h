// ********************************************************************************************************
// RF_Governor_Global.h
// Handles global/config parameters for the Rotorflight Governor on the Nextion display.
// Config parameters — MSP cmd 142, payload bytes [18]-[41]
// Off-screen (not editable): Startup [20-21], RPM/Pwr/D/FF/TTA filters [34-38]
// Profile-specific parameters are handled in RF_Governor_Profile.h
// ********************************************************************************************************

#ifndef GOVERNOR_GLOBAL_H
#define GOVERNOR_GLOBAL_H
#define GOVERNOR_GLOBAL_LABELS_COUNT 17

#include <Arduino.h>
#include "1Definitions.h"

char GOV_Global_Labels[GOVERNOR_GLOBAL_LABELS_COUNT][4] = {
    "n14", "n15", "n16", "n17", "n18", "n19", "n20",
    "n21", "n22", "n23", "n24", "n25", "n26", "n27",
    "n28", "n29", "n30"};

// ====================================================
void ForegroundColourGOVConfigLabels(uint16_t Colour)
{
    for (int i = 0; i < GOVERNOR_GLOBAL_LABELS_COUNT; ++i)
        SendForegroundColour(GOV_Global_Labels[i], Colour);
}

// ====================================================
void ShowGOVConfigMsg(const char *msg, uint16_t Colour)
{
    if (CurrentView == RFGOVERNORVIEW_GLOBAL)
    {
        ForegroundColourGOVConfigLabels(Colour);
        SendText((char *)"t4", (char *)msg);
        SendCommand((char *)"vis t4,1");
        SendCommand((char *)"vis b1,0");
        BlockBankChanges = true;
        PlaySound(BEEPMIDDLE);
    }
}

// ====================================================
void HideGOVConfigMsg()
{
    if (CurrentView == RFGOVERNORVIEW_GLOBAL)
    {
        SendCommand((char *)"vis t4,0");
        SendCommand((char *)"vis b1,1");
        ForegroundColourGOVConfigLabels(Black);
        BlockBankChanges = false;
        PlaySound(BEEPCOMPLETE);
    }
}

// ====================================================
void GOVS_G_Were_Edited()
{
    SendCommand((char *)"vis b3,1");
    GOVS_GLOBAL_Were_Edited = true;
}

// ====================================================
// LoadGovConfigWritePayload()
// Reads visible config fields from Nextion into GovWritePayload[18..41]
// Off-screen fields preserved directly from GovAckPayload[]
// U16 fields split into lo/hi byte pairs
// ====================================================
void LoadGovConfigWritePayload()
{
    // Gov mode and Handover thr — visible, editable
    GovWritePayload[18] = (uint8_t)GetValue((char *)"n14"); // Gov mode
    GovWritePayload[19] = (uint8_t)GetValue((char *)"n15"); // Handover thr %

    // Startup ×.1s — off-screen, preserve from last FC read
    GovWritePayload[20] = GovAckPayload[20];
    GovWritePayload[21] = GovAckPayload[21];

    // Spoolup ×.1%/s — visible, editable
    uint16_t v;
    v = (uint16_t)GetValue((char *)"n17");
    GovWritePayload[22] = (uint8_t)(v & 0xFF);
    GovWritePayload[23] = (uint8_t)(v >> 8);

    // Spooldown ×.1%/s — visible, editable
    v = (uint16_t)GetValue((char *)"n18");
    GovWritePayload[24] = (uint8_t)(v & 0xFF);
    GovWritePayload[25] = (uint8_t)(v >> 8);

    // Tracking ×.1%/s — visible, editable
    v = (uint16_t)GetValue((char *)"n19");
    GovWritePayload[26] = (uint8_t)(v & 0xFF);
    GovWritePayload[27] = (uint8_t)(v >> 8);

    // Recovery ×.1%/s — visible, editable
    v = (uint16_t)GetValue((char *)"n20");
    GovWritePayload[28] = (uint8_t)(v & 0xFF);
    GovWritePayload[29] = (uint8_t)(v >> 8);

    // Hold timeout ×.1s — visible, editable
    v = (uint16_t)GetValue((char *)"n21");
    GovWritePayload[30] = (uint8_t)(v & 0xFF);
    GovWritePayload[31] = (uint8_t)(v >> 8);

    // Autorot timeout s — visible, editable
    v = (uint16_t)GetValue((char *)"n22");
    GovWritePayload[32] = (uint8_t)(v & 0xFF);
    GovWritePayload[33] = (uint8_t)(v >> 8);

    // Filter bytes — off-screen, preserve from last FC read
    GovWritePayload[34] = GovAckPayload[34]; // RPM filter Hz
    GovWritePayload[35] = GovAckPayload[35]; // Power filter Hz
    GovWritePayload[36] = GovAckPayload[36]; // D filter ×.1Hz
    GovWritePayload[37] = GovAckPayload[37]; // FF filter Hz
    GovWritePayload[38] = GovAckPayload[38]; // TTA filter Hz

    // Throttle section — visible, editable
    GovWritePayload[39] = (uint8_t)GetValue((char *)"n28"); // Throttle type
    GovWritePayload[40] = (uint8_t)GetValue((char *)"n29"); // Idle thr ×.1%
    GovWritePayload[41] = (uint8_t)GetValue((char *)"n30"); // Auto thr ×.1%
}

// ====================================================
// DisplayGovConfigValues()
// Called with byte range [n, m) from GovAckPayload[]
// Receives only two values at a time — mirrors DisplayGovValues()
// Off-screen fields: stored in GovAckPayload[] but not sent to Nextion
// All U16 pairs: skip lo byte, display on hi byte once both are in buffer
// ====================================================
void DisplayGovConfigValues(uint8_t n, uint8_t m)
{
    if (CurrentView != RFGOVERNORVIEW_GLOBAL)
        return;

    for (uint8_t i = n; i < m; ++i)
    {
        if (i >= GOV_ACK_PAYLOAD_SIZE)
            break;

        switch (i)
        {
        case 18:
            SendValue((char *)"n14", GovAckPayload[i]);
            break; // Gov mode
        case 19:
            SendValue((char *)"n15", GovAckPayload[i]);
            break; // Handover thr %

        // Startup — off-screen, silent
        case 20:
            break; // lo byte of Startup
        case 21:
            break; // hi byte of Startup — off-screen, not displayed

        case 22:
            break; // lo byte of Spoolup
        case 23:
            SendValue((char *)"n17", (uint16_t)GovAckPayload[22] | ((uint16_t)GovAckPayload[23] << 8));
            break; // Spoolup ×.1%/s

        case 24:
            break; // lo byte of Spooldown
        case 25:
            SendValue((char *)"n18", (uint16_t)GovAckPayload[24] | ((uint16_t)GovAckPayload[25] << 8));
            break; // Spooldown ×.1%/s

        case 26:
            break; // lo byte of Tracking
        case 27:
            SendValue((char *)"n19", (uint16_t)GovAckPayload[26] | ((uint16_t)GovAckPayload[27] << 8));
            break; // Tracking ×.1%/s

        case 28:
            break; // lo byte of Recovery
        case 29:
            SendValue((char *)"n20", (uint16_t)GovAckPayload[28] | ((uint16_t)GovAckPayload[29] << 8));
            break; // Recovery ×.1%/s

        case 30:
            break; // lo byte of Hold timeout
        case 31:
            SendValue((char *)"n21", (uint16_t)GovAckPayload[30] | ((uint16_t)GovAckPayload[31] << 8));
            break; // Hold timeout ×.1s

        case 32:
            break; // lo byte of Autorot timeout
        case 33:
            SendValue((char *)"n22", (uint16_t)GovAckPayload[32] | ((uint16_t)GovAckPayload[33] << 8));
            break; // Autorot timeout s

        // Filter bytes — off-screen, silent (stored in GovAckPayload[] for write-back)
        case 34:
            break; // RPM filter Hz
        case 35:
            break; // Power filter Hz
        case 36:
            break; // D filter ×.1Hz
        case 37:
            break; // FF filter Hz
        case 38:
            break; // TTA filter Hz

        case 39:
            SendValue((char *)"n28", GovAckPayload[i]);
            break; // Throttle type
        case 40:
            SendValue((char *)"n29", GovAckPayload[i]);
            break; // Idle thr ×.1%
        case 41:
            SendValue((char *)"n30", GovAckPayload[i]);
            break; // Auto thr ×.1%

        default:
            break;
        }
    }
}

// ====================================================
// ShowLocalGovConfigBank()
// Shows saved config values from SD card when not connected
// Off-screen fields not sent to Nextion
// ====================================================
void ShowLocalGovConfigBank()
{
    SendValue((char *)"n14", Saved_GOV_Config_Values[18]); // Gov mode
    SendValue((char *)"n15", Saved_GOV_Config_Values[19]); // Handover thr %

    // Startup off-screen — skip

    SendValue((char *)"n17", (uint16_t)Saved_GOV_Config_Values[22] | ((uint16_t)Saved_GOV_Config_Values[23] << 8)); // Spoolup
    SendValue((char *)"n18", (uint16_t)Saved_GOV_Config_Values[24] | ((uint16_t)Saved_GOV_Config_Values[25] << 8)); // Spooldown
    SendValue((char *)"n19", (uint16_t)Saved_GOV_Config_Values[26] | ((uint16_t)Saved_GOV_Config_Values[27] << 8)); // Tracking
    SendValue((char *)"n20", (uint16_t)Saved_GOV_Config_Values[28] | ((uint16_t)Saved_GOV_Config_Values[29] << 8)); // Recovery
    SendValue((char *)"n21", (uint16_t)Saved_GOV_Config_Values[30] | ((uint16_t)Saved_GOV_Config_Values[31] << 8)); // Hold timeout
    SendValue((char *)"n22", (uint16_t)Saved_GOV_Config_Values[32] | ((uint16_t)Saved_GOV_Config_Values[33] << 8)); // Autorot timeout

    // Filter bytes off-screen — skip

    SendValue((char *)"n28", Saved_GOV_Config_Values[39]); // Throttle type
    SendValue((char *)"n29", Saved_GOV_Config_Values[40]); // Idle thr ×.1%
    SendValue((char *)"n30", Saved_GOV_Config_Values[41]); // Auto thr ×.1%

    HideGOVConfigMsg();
    BlockBankChanges = false;
}

// ====================================================
// SaveToLocalGovConfigBank()
// ====================================================
void SaveToLocalGovConfigBank()
{
    ShowGOVConfigMsg((char *)"Saving config values ...", Gray);
    LoadGovConfigWritePayload();

    for (int i = 18; i < GOV_CONFIG_PAYLOAD_SIZE; ++i)
        Saved_GOV_Config_Values[i] = GovWritePayload[i];

    SaveOneModel(ModelNumber);
    SendCommand((char *)"vis b3,0");
    HideGOVConfigMsg();
    GOVS_GLOBAL_Were_Edited = false;
    PlaySound(BEEPCOMPLETE);
}

// ====================================================
// SendEditedGovConfigValues()
// Writes config values to FC via MSP, then reboots FC
// ====================================================
void SendEditedGovConfigValues()
{
    if (!LedWasGreen)
    {
        SaveToLocalGovConfigBank();
        return;
    }

    if (SendBuffer[ArmingChannel - 1] > 1000)
    {
        PlaySound(WHAHWHAHMSG);
        MsgBox((char *)"page RFGovViewGlbl", (char *)"Model is armed!\r\nDisarm before writing governor config.");
        return;
    }

    PlaySound(BEEPMIDDLE);
    DelayWithDog(100);
    LoadGovConfigWritePayload();

    // Queue in reverse order — LIFO executes CONFIG1 first, CONFIG2 second, CONFIG3 third
    AddParameterstoQueue(SEND_GOV_WRITE_CONFIG3); // executes 3rd
    AddParameterstoQueue(SEND_GOV_WRITE_CONFIG2); // executes 2nd
    AddParameterstoQueue(SEND_GOV_WRITE_CONFIG1); // executes 1st

    GOVS_GLOBAL_Were_Edited = false;
    SendCommand((char *)"vis b3,0");

    // Reboot FC so config changes take effect
    // AddParameterstoQueue(MSP_REBOOT);

    PlaySound(BEEPCOMPLETE);
}

// ====================================================
void ShowGOV_Global_Bank()
{
    if (CurrentView == RFGOVERNORVIEW_GLOBAL)
    {
        for (int i = 0; i < GOVERNOR_GLOBAL_LABELS_COUNT; ++i)
            SendValue(GOV_Global_Labels[i], 0);

        if (!LedWasGreen)
        {
            ShowGOVConfigMsg((char *)"Loading config values ...", Gray);
            GOVS_GLOBAL_Were_Edited = false;
            ShowLocalGovConfigBank();
            return;
        }

        BlockBankChanges = true;
        ShowGOVConfigMsg((char *)"Loading config values ...", Gray);
        GOV_Config_Send_Duration = GOV_GLOBAL_WAIT_TIME;
        Reading_GOV_Config_Now = true;
        AddParameterstoQueue(SEND_GOV_CONFIG_VALUES);
        GOVS_GLOBAL_Were_Edited = false;
        SendCommand((char *)"vis b3,0");
        GOV_Global_Start_Time = millis();
    }
}

// ====================================================
void Start_Gov_Global()
{
    AddParameterstoQueue(MSP_INHIBIT_TELEMETRY);
    SendCommand((char *)"page RFGovViewGlbl");
    CurrentView = RFGOVERNORVIEW_GLOBAL;
    SendText((char *)"t27", ModelName);
    ShowGOV_Global_Bank();
}

// ====================================================
void End_Gov_Global()
{
    if (GOVS_GLOBAL_Were_Edited)
    {
        if (GetConfirmation((char *)"page RFGovViewGlbl", (char *)"Discard edited governor config?"))
        {
            GOVS_GLOBAL_Were_Edited = false;
            AddParameterstoQueue(MSP_ENABLE_TELEMETRY);
            RotorFlightStart();
        }
    }
    else
    {
        AddParameterstoQueue(MSP_ENABLE_TELEMETRY);
        RotorFlightStart();
    }
}
// ====================================================
void Gov_Global_Were_Edited()
{
    SendCommand((char *)"vis b3,1");
    GOVS_GLOBAL_Were_Edited = true;
}

// ====================================================
void Save_Gov_Global()
{
    SendEditedGovConfigValues();
}

#endif // RF_GOVERNOR_GLOBAL_H