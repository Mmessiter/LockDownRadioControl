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


// ************************************************************************************************************/
// uint8_t GovWritePayload[GOV_ACK_PAYLOAD_SIZE] = {0};

void LoadGovWritePayload()
{
    // ── Profile fields (bytes 0-17) ──────────────────────────────────
    GovWritePayload[0] = 0; // RF23 flag — not written back

    uint16_t hs = (uint16_t)GetIntFromTextBox((char *)"t78");
    GovWritePayload[1] = (uint8_t)(hs & 0xFF);
    GovWritePayload[2] = (uint8_t)(hs >> 8);

    GovWritePayload[3] = (uint8_t)GetIntFromTextBox((char *)"t1");   // Gain
    GovWritePayload[4] = (uint8_t)GetIntFromTextBox((char *)"t2");   // P
    GovWritePayload[5] = (uint8_t)GetIntFromTextBox((char *)"t3");   // I
    GovWritePayload[6] = (uint8_t)GetIntFromTextBox((char *)"t4");   // D
    GovWritePayload[7] = (uint8_t)GetIntFromTextBox((char *)"t5");   // F
    GovWritePayload[8] = (uint8_t)GetIntFromTextBox((char *)"t6");   // TTA gain
    GovWritePayload[9] = (uint8_t)GetIntFromTextBox((char *)"t7");   // TTA limit
    GovWritePayload[10] = (uint8_t)GetIntFromTextBox((char *)"t8");  // Max throttle
    GovWritePayload[11] = (uint8_t)GetIntFromTextBox((char *)"t9");  // Min throttle
    GovWritePayload[12] = (uint8_t)GetIntFromTextBox((char *)"t10"); // Fallback drop
    GovWritePayload[13] = (uint8_t)GetIntFromTextBox((char *)"t11"); // Yaw weight
    GovWritePayload[14] = (uint8_t)GetIntFromTextBox((char *)"t12"); // Cyclic weight
    GovWritePayload[15] = (uint8_t)GetIntFromTextBox((char *)"t13"); // Collective weight

    // Flags — read individual flag fields, pack into uint16_t
    uint16_t flags = 0;
    if (GetIntFromTextBox((char *)"t59"))
        flags |= GOV_FLAG_VOLTAGE_COMP;
    if (GetIntFromTextBox((char *)"t60"))
        flags |= GOV_FLAG_PID_SPOOLUP;
    if (GetIntFromTextBox((char *)"t61"))
        flags |= GOV_FLAG_FALLBACK_PRECOMP;
    if (GetIntFromTextBox((char *)"t62"))
        flags |= GOV_FLAG_DYN_MIN_THROTTLE;
    GovWritePayload[16] = (uint8_t)(flags & 0xFF);
    GovWritePayload[17] = (uint8_t)(flags >> 8);

    // ── Config fields (bytes 18-45) ──────────────────────────────────
    GovWritePayload[18] = (uint8_t)GetIntFromTextBox((char *)"t14"); // Gov mode

    GovWritePayload[19] = (uint8_t)GetIntFromTextBox((char *)"t15"); // Handover throttle

    uint16_t startup = (uint16_t)GetIntFromTextBox((char *)"t16");
    GovWritePayload[20] = (uint8_t)(startup & 0xFF);
    GovWritePayload[21] = (uint8_t)(startup >> 8);

    uint16_t spoolup = (uint16_t)GetIntFromTextBox((char *)"t17");
    GovWritePayload[22] = (uint8_t)(spoolup & 0xFF);
    GovWritePayload[23] = (uint8_t)(spoolup >> 8);

    uint16_t spooldown = (uint16_t)GetIntFromTextBox((char *)"t18");
    GovWritePayload[24] = (uint8_t)(spooldown & 0xFF);
    GovWritePayload[25] = (uint8_t)(spooldown >> 8);

    uint16_t tracking = (uint16_t)GetIntFromTextBox((char *)"t19");
    GovWritePayload[26] = (uint8_t)(tracking & 0xFF);
    GovWritePayload[27] = (uint8_t)(tracking >> 8);

    uint16_t recovery = (uint16_t)GetIntFromTextBox((char *)"t20");
    GovWritePayload[28] = (uint8_t)(recovery & 0xFF);
    GovWritePayload[29] = (uint8_t)(recovery >> 8);

    uint16_t holdtimeout = (uint16_t)GetIntFromTextBox((char *)"t21");
    GovWritePayload[30] = (uint8_t)(holdtimeout & 0xFF);
    GovWritePayload[31] = (uint8_t)(holdtimeout >> 8);

    uint16_t autotimeout = (uint16_t)GetIntFromTextBox((char *)"t22");
    GovWritePayload[32] = (uint8_t)(autotimeout & 0xFF);
    GovWritePayload[33] = (uint8_t)(autotimeout >> 8);

    GovWritePayload[34] = (uint8_t)GetIntFromTextBox((char *)"t23"); // RPM filter
    GovWritePayload[35] = (uint8_t)GetIntFromTextBox((char *)"t24"); // Pwr filter
    GovWritePayload[36] = (uint8_t)GetIntFromTextBox((char *)"t25"); // D filter
    GovWritePayload[37] = (uint8_t)GetIntFromTextBox((char *)"t54"); // FF filter
    GovWritePayload[38] = (uint8_t)GetIntFromTextBox((char *)"t55"); // TTA filter
    GovWritePayload[39] = (uint8_t)GetIntFromTextBox((char *)"t56"); // Throttle type
    GovWritePayload[40] = (uint8_t)GetIntFromTextBox((char *)"t57"); // Idle throttle
    GovWritePayload[41] = (uint8_t)GetIntFromTextBox((char *)"t58"); // Auto throttle

    // Individual flag bytes for RX unpacker
    GovWritePayload[42] = (flags & GOV_FLAG_VOLTAGE_COMP) ? 1 : 0;
    GovWritePayload[43] = (flags & GOV_FLAG_PID_SPOOLUP) ? 1 : 0;
    GovWritePayload[44] = (flags & GOV_FLAG_FALLBACK_PRECOMP) ? 1 : 0;
    GovWritePayload[45] = (flags & GOV_FLAG_DYN_MIN_THROTTLE) ? 1 : 0;

    // uint8_t GovWritePayload[GOV_ACK_PAYLOAD_SIZE] = {0};

    // For debugging: print the payload to the Serial console
    Serial.println("GovWritePayload:");
    for (uint8_t i = 0; i < GOV_ACK_PAYLOAD_SIZE; ++i)
    {
        Serial.print("Byte ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(GovWritePayload[i]);
    }
}

// ********************************************************************************************************
void ForegroundColourGOVLabels(uint16_t Colour)
{
    for (int i = 0; i < GOVERNOR_LABELS_COUNT; ++i)
    {
        SendForegroundColour(GOV_Labels[i], Colour);
    }
}

// ************************************************************************************************************/
// ******************************************** Rotorflight Governor ******************************************************

void DisplayGovValues(uint8_t n, uint8_t m)
{
    if (CurrentView != RFGOVERNORVIEW)
        return;

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
// **********************************************************************************************************/
void ShowGOVMsg(const char *msg, uint16_t Colour)
{
    if (CurrentView == RFGOVERNORVIEW) // Must be in GOVERNOR view
    {
        ForegroundColourGOVLabels(Colour);     // make text white so it isnt visible
        SendText((char *)"busy", (char *)msg); // Show  message
        SendCommand((char *)"vis busy,1");     // Make it visible
        SendCommand((char *)"vis b2,0");       // Make Advanced invisible
                                               // SendCommand((char *)"vis b3,0");       // hide "Send" button
        BlockBankChanges = true;
    }
}

//************************************************************************************************************/
void ShowGOVBank() // this is called when bank is changed so new bank's GOVERNOR values are requested from Nexus and shown
{
    if (CurrentView == RFGOVERNORVIEW) // Must be in GOVERNOR view
    {
        char buf[40];

        strcpy(buf, "Loading governor values ...");
        SendText((char *)"t26", BankNames[BanksInUse[Bank - 1]]);
        BlockBankChanges = true;               // block bank changes while we do this
        ShowGOVMsg(buf, Gray);                 // Show loading message and hides GOV values
        GOV_Send_Duration = GOV_1_WAIT_TIME;   // how many milliseconds to await GOV values
        Reading_GOV_Now = true;                // This tells the Ack payload parser to get GOV values
        AddParameterstoQueue(SEND_GOV_VALUES); // Request GOV values from RX
        GOVS_Were_Edited = false;              // reset this flag as we are now back in sync with the RX GOV values
        GOV_Start_Time = millis();             // record start time as it's not long
    }
}

// ************************************************************************************************************/
void Start_RF_Governor()
{
    SendCommand((char *)"page RFGovView"); // Make Governor view visible
    CurrentView = RFGOVERNORVIEW;
    SendText((char *)"t27", ModelName);
    ShowGOVBank();
}
// ************************************************************************************************************/
void End_RF_Governor()
{
    RotorFlightStart();
}
// 
void SendEditedGovValues()
{
    if (!LedWasGreen)
    {
        // GovMsg((char *)"Not connected!");
        // DelayWithDog(2000);
        // HideGovMsg();
        return;
    }
    if (SendBuffer[ArmingChannel - 1] > 1000)
    {
        PlaySound(WHAHWHAHMSG);
        MsgBox((char *)"page RFGovView", (char *)"Model is armed!\r\nDisarm before writing governor values.");
        return;
    }
    PlaySound(BEEPMIDDLE);
   // GovMsg((char *)"Sending governor values...");
    DelayWithDog(100);
    LoadGovWritePayload(); // read all values from Nextion
    // Queue in reverse order — LIFO executes profile first, then config
    AddParameterstoQueue(SEND_GOV_WRITE_CONFIG3);  // executes 5th
    AddParameterstoQueue(SEND_GOV_WRITE_CONFIG2);  // executes 4th
    AddParameterstoQueue(SEND_GOV_WRITE_CONFIG1);  // executes 3rd
    AddParameterstoQueue(SEND_GOV_WRITE_PROFILE2); // executes 2nd
    AddParameterstoQueue(SEND_GOV_WRITE_PROFILE1); // executes 1st
   // GOV_Were_Edited = false;
    PlaySound(BEEPCOMPLETE);
   // HideGovMsg();
}

// ************************************************************************************************************/
void Save_RF_Governor()
{
    
    SendEditedGovValues(); // this function will check if we're connected and armed, and if not will show a message box and return without sending. If all is good, it will send the values to the RX and play a completion beep.
       
}

#endif // GOVERNOR_H
       // *********************************************************************************************************************************/