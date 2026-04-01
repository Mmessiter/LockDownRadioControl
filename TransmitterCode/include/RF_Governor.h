// ******************************************** Rotorflight PID Advanced ******************************************************
// This module handles the Rotorflight Governor on the Nextion display

// **********************************************************************************************************
#ifndef GOVERNOR_H
#define GOVERNOR_H
#define GOVERNOR_LABELS_COUNT 35
#include <Arduino.h>
#include "1Definitions.h"

char GOV_Labels[GOVERNOR_LABELS_COUNT][4] = {
    "t78",
    "t1",
    "t2",
    "t3",
    "t4",
    "t5",
    "t6",
    "t7",
    "t8",
    "t9",
    "t10",
    "t11",
    "t12",
    "t13",
    "t14",
    "t15",
    "t16",
    "t17",
    "t18",
    "t19",
    "t20",
    "t21",
    "t22",
    "t23",
    "t24",
    "t25",
    "t54",
    "t55",
    "t56",
    "t57",
    "t58",
    "t59",
    "t60",
    "t61",
    "t62",
};

// ********************************************************************************************************
void ForegroundColourGOVLabels(uint16_t Colour)
{
    for (int i = 0; i < GOVERNOR_LABELS_COUNT; ++i)
    {
        SendForegroundColour(GOV_Labels[i], Colour);
    }
}
// ********************************************************************************************************
void HideGOVMsg()
{
    if (CurrentView == RFGOVERNORVIEW) // Must be in RFGOVERNORVIEW view
    {
        SendCommand((char *)"vis busy,0"); // Hide  message
        SendCommand((char *)"vis b2,1");   // Make Advanced visible
        ForegroundColourGOVLabels(Black);  // make text black so it is visible again
        BlockBankChanges = false;
    }
}

// ************************************************************************************************************/
// ******************************************** Rotorflight Governor ******************************************************

void DisplayGovValues(uint8_t n, uint8_t m)
{
    if (CurrentView != RFGOVERNORVIEW)
        return;

    // Look1("DisplayGov n=");
    // Look1(n);
    // Look1(" m=");
    // Look(m);
    // Look1("  CurrentView=");
    // Look(CurrentView);

    // Look1("DGV bytes: ");
    // for (uint8_t i = n; i < m; ++i)
    // {
    //     Look1(GovAckPayload[i]);
    //     Look1(" ");
    // }
    // Look("");
    char buf[12];

    for (uint8_t i = n; i < m; ++i)
    {
        if (i >= GOV_ACK_PAYLOAD_SIZE)
            break;

        switch (i)
        {
        // --- U16 fields ---
        case 1: // Headspeed (U16, no scale) — consumes bytes [1] and [2]
        {
            uint16_t v = (uint16_t)GovAckPayload[1] | ((uint16_t)GovAckPayload[2] << 8);
            snprintf(buf, sizeof(buf), "%u", (unsigned)v);
            SendText((char *)"t78", buf);
            break;
        }
        case 2:
            break; // high byte of Headspeed — already handled above

        // --- Single-byte integer fields (profile) ---
        case 3:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t1", buf);
            break;
        case 4:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t2", buf);
            break;
        case 5:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t3", buf);
            break;
        case 6:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t4", buf);
            break;
        case 7:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t5", buf);
            break;
        case 8:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t6", buf);
            break;
        case 9:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t7", buf);
            break;
        case 10:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t8", buf);
            break;
        case 11:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t9", buf);
            break;
        case 12:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t10", buf);
            break;
        case 13:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t11", buf);
            break;
        case 14:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t12", buf);
            break;
        case 15:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t13", buf);
            break;

        // --- bytes [16] and [17] are Gov_Flags lo/hi — not displayed directly, skip ---
        case 16:
            break;
        case 17:
            break;

        // --- Config single-byte fields ---
        case 18:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t14", buf);
            break;
        case 19:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t15", buf);
            break;

        // --- U16 /10 timing fields ---
        case 20: // Startup_Time /10
        {
            uint16_t v = (uint16_t)GovAckPayload[20] | ((uint16_t)GovAckPayload[21] << 8);
            snprintf(buf, sizeof(buf), "%.1f", (float)v / 10.0f);
            SendText((char *)"t16", buf);
            break;
        }
        case 21:
            break; // high byte handled above

        case 22: // Spoolup_Time /10
        {
            uint16_t v = (uint16_t)GovAckPayload[22] | ((uint16_t)GovAckPayload[23] << 8);
            snprintf(buf, sizeof(buf), "%.1f", (float)v / 10.0f);
            SendText((char *)"t17", buf);
            break;
        }
        case 23:
            break;

        case 24: // Spooldown_Time /10
        {
            uint16_t v = (uint16_t)GovAckPayload[24] | ((uint16_t)GovAckPayload[25] << 8);
            snprintf(buf, sizeof(buf), "%.1f", (float)v / 10.0f);
            SendText((char *)"t18", buf);
            break;
        }
        case 25:
            break;

        case 26: // Tracking_Time /10
        {
            uint16_t v = (uint16_t)GovAckPayload[26] | ((uint16_t)GovAckPayload[27] << 8);
            snprintf(buf, sizeof(buf), "%.1f", (float)v / 10.0f);
            SendText((char *)"t19", buf);
            break;
        }
        case 27:
            break;

        case 28: // Recovery_Time /10
        {
            uint16_t v = (uint16_t)GovAckPayload[28] | ((uint16_t)GovAckPayload[29] << 8);
            snprintf(buf, sizeof(buf), "%.1f", (float)v / 10.0f);
            SendText((char *)"t20", buf);
            break;
        }
        case 29:
            break;

        case 30: // Hold_Timeout /10
        {
            uint16_t v = (uint16_t)GovAckPayload[30] | ((uint16_t)GovAckPayload[31] << 8);
            snprintf(buf, sizeof(buf), "%.1f", (float)v / 10.0f);
            SendText((char *)"t21", buf);
            break;
        }
        case 31:
            break;

        case 32: // Autorot_Timeout (no scale)
        {
            uint16_t v = (uint16_t)GovAckPayload[32] | ((uint16_t)GovAckPayload[33] << 8);
            snprintf(buf, sizeof(buf), "%u", (unsigned)v);
            SendText((char *)"t22", buf);
            break;
        }
        case 33:
            break;

        // --- Single-byte filter/config fields ---
        case 34:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t23", buf);
            break;
        case 35:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t24", buf);
            break;
        case 36:
            snprintf(buf, sizeof(buf), "%.1f", (float)GovAckPayload[i] / 10.0f);
            SendText((char *)"t25", buf);
            break; // D_Filter /10
        case 37:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t54", buf);
            break;
        case 38:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t55", buf);
            break;
        case 39:
            snprintf(buf, sizeof(buf), "%u", (unsigned)GovAckPayload[i]);
            SendText((char *)"t56", buf);
            break;
        case 40:
            snprintf(buf, sizeof(buf), "%.1f", (float)GovAckPayload[i] / 10.0f);
            SendText((char *)"t57", buf);
            break; // Idle_Throttle /10
        case 41:
            snprintf(buf, sizeof(buf), "%.1f", (float)GovAckPayload[i] / 10.0f);
            SendText((char *)"t58", buf);
            break; // Auto_Throttle /10

        // --- Flag fields (0 or 1) ---
        case 42:
            SendValue((char *)"t59", (bool)GovAckPayload[i]);
            break; // Volt comp
        case 43:
            SendValue((char *)"t60", (bool)GovAckPayload[i]);
            break; // PID spoolup
        case 44:
            SendValue((char *)"t61", (bool)GovAckPayload[i]);
            break; // Fallback precomp
        case 45:
            SendValue((char *)"t62", (bool)GovAckPayload[i]);
            break; // Dyn min thr

        default:
            break;
        }
    }
}
// **********************************************************************************************************/
void govMsg(const char *msg, uint16_t Colour)
{
    if (CurrentView == RFGOVERNORVIEW) // Must be in GOVERNOR view
    {
        ForegroundColourGOVLabels(Colour);     // make text white so it isnt visible
        SendText((char *)"busy", (char *)msg); // Show  message
        SendCommand((char *)"vis busy,1");     // Make it visible
        SendCommand((char *)"vis b2,0");       // Make Advanced invisible
        SendCommand((char *)"vis b3,0");       // hide "Send" button
        BlockBankChanges = true;
    }
}

//************************************************************************************************************/
void ShowGOVBank() // this is called when bank is changed so new bank's GOVERNOR values are requested from Nexus and shown
{
    if (CurrentView == RFGOVERNORVIEW) // Must be in GOVERNOR view
    {
        char buf[40];
        strcpy(buf, "Loading GOVERNOR VALUES for  ");
        strcat(buf, BankNames[BanksInUse[Bank - 1]]);
        strcat(buf, " ...");
        SendText((char *)"t26", BankNames[BanksInUse[Bank - 1]]);

        BlockBankChanges = true;               // block bank changes while we do this
        govMsg(buf, Gray);                     //  Show loading message and hides old PIDs
        GOV_Send_Duration = MSP_WAIT_TIME;     // how many milliseconds to await GOV values
        Reading_GOV_Now = true;                // This tells the Ack payload parser to get GOV values
        AddParameterstoQueue(SEND_GOV_VALUES); // Request GOV values from RX
        GOVS_Were_Edited = false;
        GOV_Start_Time = millis(); // record start time as it's not long
    }
}

// ************************************************************************************************************/
void Start_RF_Governor()
{
    SendCommand((char *)"page RFGovView"); // Make Governor view visible
    CurrentView = RFGOVERNORVIEW;
    ShowGOVBank();
}
// ************************************************************************************************************/
void End_RF_Governor()
{
    RotorFlightStart();
}

#endif // GOVERNOR_H
       // *********************************************************************************************************************************/