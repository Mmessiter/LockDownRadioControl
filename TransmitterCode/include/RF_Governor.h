// ******************************************** Rotorflight Governor ******************************************************
// This module handles the Rotorflight Governor on the Nextion display
// **********************************************************************************************************
#ifndef GOVERNOR_H
#define GOVERNOR_H
// #define GOVERNOR_LABELS_COUNT 35
#define GOVERNOR_LABELS_COUNT 18 // Profile fields only, for now
#include <Arduino.h>
#include "1Definitions.h"

uint8_t GOV_Items_Received[GOVERNOR_LABELS_COUNT] = {0};
uint16_t Total_Received_GOV_Values = 0;
char GOV_Labels[GOVERNOR_LABELS_COUNT][4] = {
    "n0", "n1", "n2", "n3", "n4", "n5", "n6", "n7", "n8", "n9",
    "n10", "n11", "n12", "n13", "n14", "n15", "n16", "n17"};
//"n18", "n19",
//   "n20", "n21", "n22", "n23", "n24", "n25", "n26", "n27", "n28", "n29",
//   "n30", "n31", "n32", "n33", "n34"

// ====================================================
// LoadGovWritePayload()
// Reads all governor numeric fields from Nextion into GovWritePayload[]
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

    // Flags — preserve existing values from last read, not editable on this screen
    GovWritePayload[16] = GovAckPayload[16];
    GovWritePayload[17] = GovAckPayload[17];
}
// ====================================================
// int GetTotalSoFar()
// {
//     int total = 0;
//     for (int i = 0; i < GOVERNOR_LABELS_COUNT; ++i)
//         total += GOV_Items_Received[i];
//     if (total == GOVERNOR_LABELS_COUNT)
//         GOV_Config_Send_Duration = 0; // stop timeout once all items received
//     return total;
// }

// ====================================================
// void Show_Progress()
// {
//     if (NeedGlobalsToo)
//         SendValue((char *)"Progress", GetTotalSoFar() * 100 / 35);
// }

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
        // case 15:
        //     SendValue((char *)"n13", GovAckPayload[i]);
        //     break; // Collective weight
        // case 16:
        //     break; // Flags lo — not displayed directly
        // case 17:
        //     break; // Flags hi — not displayed directly
        // case 18:
        //     SendValue((char *)"n14", GovAckPayload[i]);
        //     break; // Gov mode
        // case 19:
        //     SendValue((char *)"n15", GovAckPayload[i]);
        //     break; // Handover throttle
        // case 20:
        // {
        //     uint16_t v = (uint16_t)GovAckPayload[20] | ((uint16_t)GovAckPayload[21] << 8);
        //     SendValue((char *)"n16", v);
        //     break;
        // }
        // case 21:
        //     break;
        // case 22:
        // {
        //     uint16_t v = (uint16_t)GovAckPayload[22] | ((uint16_t)GovAckPayload[23] << 8);
        //     SendValue((char *)"n17", v);
        //     break;
        // }
        // case 23:
        //     break;
        // case 24:
        // {
        //     uint16_t v = (uint16_t)GovAckPayload[24] | ((uint16_t)GovAckPayload[25] << 8);
        //     SendValue((char *)"n18", v);
        //     break;
        // }
        // case 25:
        //     break;
        // case 26:
        // {
        //     uint16_t v = (uint16_t)GovAckPayload[26] | ((uint16_t)GovAckPayload[27] << 8);
        //     SendValue((char *)"n19", v);
        //     break;
        // }
        // case 27:
        //     break;
        // case 28:
        // {
        //     uint16_t v = (uint16_t)GovAckPayload[28] | ((uint16_t)GovAckPayload[29] << 8);
        //     SendValue((char *)"n20", v);
        //     break;
        // }
        // case 29:
        //     break;
        // case 30:
        // {
        //     uint16_t v = (uint16_t)GovAckPayload[30] | ((uint16_t)GovAckPayload[31] << 8);
        //     SendValue((char *)"n21", v);
        //     break;
        // }
        // case 31:
        //     break;
        // case 32:
        // {
        //     uint16_t v = (uint16_t)GovAckPayload[32] | ((uint16_t)GovAckPayload[33] << 8);
        //     SendValue((char *)"n22", v);
        //     break;
        // }
        // case 33:
        //     break;
        // case 34:
        //     SendValue((char *)"n23", GovAckPayload[i]);
        //     break; // RPM filter
        // case 35:
        //     SendValue((char *)"n24", GovAckPayload[i]);
        //     break; // Pwr filter
        // case 36:
        //     SendValue((char *)"n25", GovAckPayload[i]);
        //     break; // D filter
        // case 37:
        //     SendValue((char *)"n26", GovAckPayload[i]);
        //     break; // FF filter
        // case 38:
        //     SendValue((char *)"n27", GovAckPayload[i]);
        //     break; // TTA filter
        // case 39:
        //     SendValue((char *)"n28", GovAckPayload[i]);
        //     break; // Throttle type
        // case 40:
        //     SendValue((char *)"n29", GovAckPayload[i]);
        //     break; // Idle throttle
        // case 41:
        //     SendValue((char *)"n30", GovAckPayload[i]);
        //     break; // Auto throttle
        // case 42:
        //     SendValue((char *)"n31", GovAckPayload[i]);
        //     break; // Volt comp
        // case 43:
        //     SendValue((char *)"n32", GovAckPayload[i]);
        //     break; // PID spoolup
        // case 44:
        //     SendValue((char *)"n33", GovAckPayload[i]);
        //     break; // Fallback precomp
        // case 45:
        //     SendValue((char *)"n34", GovAckPayload[i]);
        //     break; // Dyn min thr
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
    if (CurrentView == RFGOVERNORVIEW)
    {
        SendCommand((char *)"vis busy,0");
        SendCommand((char *)"vis b2,1");
        ForegroundColourGOVLabels(Black);
        BlockBankChanges = false;

        // if (NeedGlobalsToo)
        // {
        //     Total_Received_GOV_Values = GetTotalSoFar();
        //     if (Total_Received_GOV_Values > 18)
        //     {
        //         if (Total_Received_GOV_Values < GOVERNOR_LABELS_COUNT)
        //         {
        //             MsgBox((char *)"page RFGovView", (char *)" Error - try again! ");
        //             NeedGlobalsToo = false;
        //         }
        //         else
        //         {
        //             SendCommand((char *)"vis Progress,0");
        //             NeedGlobalsToo = false;
        //         }
        //     }
        // }
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
    }
}

// ====================================================
void ShowGOVBank()
{
    if (CurrentView == RFGOVERNORVIEW)
    {
        char buf[40];
        // if (NeedGlobalsToo)
        //  {
        //      for (int i = 0; i < GOVERNOR_LABELS_COUNT; ++i)
        //      {
        //          GOV_Items_Received[i] = 0;
        //          SendValue(GOV_Labels[i], 0);
        //      }
        //  }
        strcpy(buf, "Loading governor values ...");
        SendText((char *)"t26", BankNames[BanksInUse[Bank - 1]]);
        BlockBankChanges = true;
        ShowGOVMsg(buf, Gray);
        GOV_Send_Duration = GOV_1_WAIT_TIME;
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
    AddParameterstoQueue(MSP_INHIBIT_TELEMETRY);
    SendCommand((char *)"page RFGovView");
    CurrentView = RFGOVERNORVIEW;
    SendText((char *)"t27", ModelName);
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
    // AddParameterstoQueue(SEND_GOV_WRITE_CONFIG3);  // executes 5th
    // AddParameterstoQueue(SEND_GOV_WRITE_CONFIG2);  // executes 4th
    // AddParameterstoQueue(SEND_GOV_WRITE_CONFIG1);  // executes 3rd
    // DelayWithDog(1000);

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
