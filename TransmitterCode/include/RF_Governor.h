// ******************************************** Rotorflight Governor ******************************************************
// This module handles the Rotorflight Governor on the Nextion display
// **********************************************************************************************************
#ifndef GOVERNOR_H
#define GOVERNOR_H
#define GOVERNOR_LABELS_COUNT 35
#include <Arduino.h>
#include "1Definitions.h"

uint8_t GOV_Items_Received[GOVERNOR_LABELS_COUNT] = {0};
uint16_t Total_Received_GOV_Values = 0;
char GOV_Labels[GOVERNOR_LABELS_COUNT][4] = {
    "n0", "n1", "n2", "n3", "n4", "n5", "n6", "n7", "n8", "n9",
    "n10", "n11", "n12", "n13", "n14", "n15", "n16", "n17", "n18", "n19",
    "n20", "n21", "n22", "n23", "n24", "n25", "n26", "n27", "n28", "n29",
    "n30", "n31", "n32", "n33", "n34"};

// ====================================================
// LoadGovWritePayload()
// Reads all governor numeric fields from Nextion into GovWritePayload[]
// ====================================================
void LoadGovWritePayload()
{
    // ── Profile fields (bytes 0-17) ──────────────────────────────────
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

    // Flags — read individual flag fields, pack into uint16_t
    uint16_t flags = 0;
    if (GetValue((char *)"n31"))
        flags |= GOV_FLAG_VOLTAGE_COMP;
    if (GetValue((char *)"n32"))
        flags |= GOV_FLAG_PID_SPOOLUP;
    if (GetValue((char *)"n33"))
        flags |= GOV_FLAG_FALLBACK_PRECOMP;
    if (GetValue((char *)"n34"))
        flags |= GOV_FLAG_DYN_MIN_THROTTLE;
    GovWritePayload[16] = (uint8_t)(flags & 0xFF);
    GovWritePayload[17] = (uint8_t)(flags >> 8);

    // ── Config fields (bytes 18-45) ──────────────────────────────────
    GovWritePayload[18] = (uint8_t)GetValue((char *)"n14"); // Gov mode
    GovWritePayload[19] = (uint8_t)GetValue((char *)"n15"); // Handover throttle

    uint16_t startup = (uint16_t)GetValue((char *)"n16");
    GovWritePayload[20] = (uint8_t)(startup & 0xFF);
    GovWritePayload[21] = (uint8_t)(startup >> 8);

    uint16_t spoolup = (uint16_t)GetValue((char *)"n17");
    GovWritePayload[22] = (uint8_t)(spoolup & 0xFF);
    GovWritePayload[23] = (uint8_t)(spoolup >> 8);

    uint16_t spooldown = (uint16_t)GetValue((char *)"n18");
    GovWritePayload[24] = (uint8_t)(spooldown & 0xFF);
    GovWritePayload[25] = (uint8_t)(spooldown >> 8);

    uint16_t tracking = (uint16_t)GetValue((char *)"n19");
    GovWritePayload[26] = (uint8_t)(tracking & 0xFF);
    GovWritePayload[27] = (uint8_t)(tracking >> 8);

    uint16_t recovery = (uint16_t)GetValue((char *)"n20");
    GovWritePayload[28] = (uint8_t)(recovery & 0xFF);
    GovWritePayload[29] = (uint8_t)(recovery >> 8);

    uint16_t holdtimeout = (uint16_t)GetValue((char *)"n21");
    GovWritePayload[30] = (uint8_t)(holdtimeout & 0xFF);
    GovWritePayload[31] = (uint8_t)(holdtimeout >> 8);

    uint16_t autotimeout = (uint16_t)GetValue((char *)"n22");
    GovWritePayload[32] = (uint8_t)(autotimeout & 0xFF);
    GovWritePayload[33] = (uint8_t)(autotimeout >> 8);

    GovWritePayload[34] = (uint8_t)GetValue((char *)"n23"); // RPM filter
    GovWritePayload[35] = (uint8_t)GetValue((char *)"n24"); // Pwr filter
    GovWritePayload[36] = (uint8_t)GetValue((char *)"n25"); // D filter
    GovWritePayload[37] = (uint8_t)GetValue((char *)"n26"); // FF filter
    GovWritePayload[38] = (uint8_t)GetValue((char *)"n27"); // TTA filter
    GovWritePayload[39] = (uint8_t)GetValue((char *)"n28"); // Throttle type
    GovWritePayload[40] = (uint8_t)GetValue((char *)"n29"); // Idle throttle
    GovWritePayload[41] = (uint8_t)GetValue((char *)"n30"); // Auto throttle

    // Individual flag bytes for RX unpacker
    GovWritePayload[42] = (flags & GOV_FLAG_VOLTAGE_COMP) ? 1 : 0;
    GovWritePayload[43] = (flags & GOV_FLAG_PID_SPOOLUP) ? 1 : 0;
    GovWritePayload[44] = (flags & GOV_FLAG_FALLBACK_PRECOMP) ? 1 : 0;
    GovWritePayload[45] = (flags & GOV_FLAG_DYN_MIN_THROTTLE) ? 1 : 0;
}

// ************************************************************************************************/

int GetTotalSoFar()
{
    int total = 0;
    for (int i = 0; i < GOVERNOR_LABELS_COUNT; ++i)
        total += GOV_Items_Received[i];
    if (total == GOVERNOR_LABELS_COUNT)
        GOV_Config_Send_Duration = 0; // stop timeout once all items received
    return total;
}

//** ****************************************************************************************************
void Show_Progress()
{
    if (NeedGlobalsToo)
    {
        SendValue((char *)"Progress", GetTotalSoFar() * 100 / 35); // progress value for progress bar
    }
}

// ************************************************************************************************************/

// ====================================================
// DisplayGovValues()
// Sends governor values to Nextion numeric fields
// Called with byte range [n, m) from GovAckPayload[]
// ====================================================

void DisplayGovValues(uint8_t n, uint8_t m)
{
    if (CurrentView != RFGOVERNORVIEW)
        return;

    for (uint8_t i = n; i < m; ++i)
    {
        if (i >= GOV_ACK_PAYLOAD_SIZE)
            break;

        GOV_Items_Received[i] = 1;
        Show_Progress();

        switch (i)
        {
        // Profile — U16 Headspeed
        case 1:
        {
            uint16_t v = (uint16_t)GovAckPayload[1] | ((uint16_t)GovAckPayload[2] << 8);
            SendValue((char *)"n0", v);
            break;
        }
        case 2:
            break; // high byte handled above

        // Profile — single byte fields
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

        case 16:
            break; // Flags lo — not displayed directly
        case 17:
            break; // Flags hi — not displayed directly

        // Config — single byte fields
        case 18:
            SendValue((char *)"n14", GovAckPayload[i]);
            break; // Gov mode
        case 19:
            SendValue((char *)"n15", GovAckPayload[i]);
            break; // Handover throttle

        // Config — U16 timing fields (stored as raw integer, label shows ×.1s)
        case 20:
        {
            uint16_t v = (uint16_t)GovAckPayload[20] | ((uint16_t)GovAckPayload[21] << 8);
            SendValue((char *)"n16", v);
            break;
        }
        case 21:
            break;

        case 22:
        {
            uint16_t v = (uint16_t)GovAckPayload[22] | ((uint16_t)GovAckPayload[23] << 8);
            SendValue((char *)"n17", v);
            break;
        }
        case 23:
            break;

        case 24:
        {
            uint16_t v = (uint16_t)GovAckPayload[24] | ((uint16_t)GovAckPayload[25] << 8);
            SendValue((char *)"n18", v);
            break;
        }
        case 25:
            break;

        case 26:
        {
            uint16_t v = (uint16_t)GovAckPayload[26] | ((uint16_t)GovAckPayload[27] << 8);
            SendValue((char *)"n19", v);
            break;
        }
        case 27:
            break;

        case 28:
        {
            uint16_t v = (uint16_t)GovAckPayload[28] | ((uint16_t)GovAckPayload[29] << 8);
            SendValue((char *)"n20", v);
            break;
        }
        case 29:
            break;

        case 30:
        {
            uint16_t v = (uint16_t)GovAckPayload[30] | ((uint16_t)GovAckPayload[31] << 8);
            SendValue((char *)"n21", v);
            break;
        }
        case 31:
            break;

        case 32:
        {
            uint16_t v = (uint16_t)GovAckPayload[32] | ((uint16_t)GovAckPayload[33] << 8);
            SendValue((char *)"n22", v);
            break;
        }
        case 33:
            break;

        // Config — single byte filter/config fields
        case 34:
            SendValue((char *)"n23", GovAckPayload[i]);
            break; // RPM filter
        case 35:
            SendValue((char *)"n24", GovAckPayload[i]);
            break; // Pwr filter
        case 36:
            SendValue((char *)"n25", GovAckPayload[i]);
            break; // D filter
        case 37:
            SendValue((char *)"n26", GovAckPayload[i]);
            break; // FF filter
        case 38:
            SendValue((char *)"n27", GovAckPayload[i]);
            break; // TTA filter
        case 39:
            SendValue((char *)"n28", GovAckPayload[i]);
            break; // Throttle type
        case 40:
            SendValue((char *)"n29", GovAckPayload[i]);
            break; // Idle throttle
        case 41:
            SendValue((char *)"n30", GovAckPayload[i]);
            break; // Auto throttle

        // Flag fields (0 or 1)
        case 42:
            SendValue((char *)"n31", GovAckPayload[i]);
            break; // Volt comp
        case 43:
            SendValue((char *)"n32", GovAckPayload[i]);
            break; // PID spoolup
        case 44:
            SendValue((char *)"n33", GovAckPayload[i]);
            break; // Fallback precomp
        case 45:
            SendValue((char *)"n34", GovAckPayload[i]);
            break; // Dyn min thr

        default:
            break;
        }
    }
}

// ====================================================
void ForegroundColourGOVLabels(uint16_t Colour)
{
    uint16_t n = GOVERNOR_LABELS_COUNT;
    // if we're only reading profile values, only colour the first 18 labels,
    // to avoid colouring the config labels which aren't being updated
    if (!NeedGlobalsToo)
        n = 18;
    for (int i = 0; i < n; ++i)
        SendForegroundColour(GOV_Labels[i], Colour);
}

// ====================================================
void HideGOVMsg()
{
    if (CurrentView == RFGOVERNORVIEW)
    {
        SendCommand((char *)"vis busy,0");
        SendCommand((char *)"vis b2,1");
        ForegroundColourGOVLabels(Black);
        BlockBankChanges = false;

        if (NeedGlobalsToo)
        {
            Total_Received_GOV_Values = GetTotalSoFar();
            if (Total_Received_GOV_Values > 18) // if over 18 values received, assume phase 2
            {
                if (Total_Received_GOV_Values < GOVERNOR_LABELS_COUNT) // if not all values were received, show error message
                {
                    MsgBox((char *)"page RFGovView", (char *)" Error - try again! ");
                    NeedGlobalsToo = false;
                }
                else
                {
                    SendCommand((char *)"vis Progress,0");
                    NeedGlobalsToo = false;
                }
            }
        }
    }
}

// ====================================================
void ShowGOVMsg(const char *msg, uint16_t Colour)
{
    if (CurrentView == RFGOVERNORVIEW)
    {
        ForegroundColourGOVLabels(Colour);
        SendText((char *)"busy", (char *)msg);
        SendCommand((char *)"vis busy,1");
        SendCommand((char *)"vis b2,0");
        BlockBankChanges = true;
        if (NeedGlobalsToo)
            SendCommand((char *)"vis Progress,1");
    }
}

// ******************************************************************************************************
// ====================================================
void ShowGOVBank()
{
    if (CurrentView == RFGOVERNORVIEW)
    {
        char buf[40];

        if (NeedGlobalsToo)
        {
            for (int i = 0; i < GOVERNOR_LABELS_COUNT; ++i)
            {
                GOV_Items_Received[i] = 0;
                SendValue(GOV_Labels[i], 0); // clear all numeric fields to 0
            }
        }

        strcpy(buf, "Loading governor values ...");
        SendText((char *)"t26", BankNames[BanksInUse[Bank - 1]]);
        BlockBankChanges = true;
        ShowGOVMsg(buf, Gray);
        GOV_Send_Duration = GOV_1_WAIT_TIME;
        Reading_GOV_Now = true;
        AddParameterstoQueue(SEND_GOV_VALUES);
        GOVS_Were_Edited = false;
        GOV_Start_Time = millis();
    }
}

// ====================================================
void Start_RF_Governor()
{
    AddParameterstoQueue(MSP_INHIBIT_TELEMETRY);
    SendCommand((char *)"page RFGovView");
    CurrentView = RFGOVERNORVIEW;
    SendText((char *)"t27", ModelName);
    NeedGlobalsToo = true;
    ShowGOVBank();
}

// ====================================================
void End_RF_Governor()
{
    AddParameterstoQueue(MSP_ENABLE_TELEMETRY);
    RotorFlightStart();
}

// ====================================================
void SendEditedGovValues()
{
    if (!LedWasGreen)
        return;

    if (SendBuffer[ArmingChannel - 1] > 1000)
    {
        PlaySound(WHAHWHAHMSG);
        MsgBox((char *)"page RFGovView", (char *)"Model is armed!\r\nDisarm before writing governor values.");
        return;
    }

    PlaySound(BEEPMIDDLE);
    DelayWithDog(100);
    LoadGovWritePayload(); // read all values from Nextion into GovWritePayload[]

    // Queue in reverse order — LIFO executes profile first, then config
    AddParameterstoQueue(SEND_GOV_WRITE_CONFIG3);  // executes 5th
    AddParameterstoQueue(SEND_GOV_WRITE_CONFIG2);  // executes 4th
    AddParameterstoQueue(SEND_GOV_WRITE_CONFIG1);  // executes 3rd
    AddParameterstoQueue(SEND_GOV_WRITE_PROFILE2); // executes 2nd
    AddParameterstoQueue(SEND_GOV_WRITE_PROFILE1); // executes 1st

    GOVS_Were_Edited = false;
    PlaySound(BEEPCOMPLETE);
}

// ====================================================
void Save_RF_Governor()
{
    SendEditedGovValues();
}

#endif // GOVERNOR_H
