// *************************************** MenuOptions.h  *****************************************
#include <Arduino.h>
#include "Hardware/1Definitions.h"

#ifndef MENUOPTIONS_H
#define MENUOPTIONS_H

/*********************************************************************************************************************************/

void SystemPage1Start()
{
    char lpm[] = "c0"; // Auto model selection
    char Bwn[] = "Bwn";
    char n0[] = "n0";
    char ScreenViewTimeout[] = "Sto"; // needed for display info
    char Pto[] = "Pto";
    char dGMT[] = "dGMT";
    char Tx_Name[] = "TxName";

    FixDeltaGMTSign();
    if (CurrentView == OPTIONVIEW2)
        DeltaGMT = GetValue(dGMT);
    SendCommand(pOptionsViewS); // TX options view
    SendValue(n0, SticksMode);
    SendValue(ScreenViewTimeout, ScreenTimeout);
    SendValue(Pto, (Inactivity_Timeout / TICKSPERMINUTE));
    SendText(Tx_Name, TxName);
    SendValue(lpm, AutoModelSelect);
    SendValue(Bwn, LowBattery);
    CurrentView = OPTIONS_VIEW;
    ClearText();
}

/*********************************************************************************************************************************/

void SystemPage1End()
{
    char ProgressStart[] = "vis Progress,1";
    char ProgressEnd[] = "vis Progress,0";
    char Progress[] = "Progress";
    char TxNme[] = "TxName";
    char lpm[] = "c0";
    char Bwn[] = "Bwn";
    char n0[] = "n0";
    char ScreenViewTimeout[] = "Sto"; // needed for display info
    char Pto[] = "Pto";

    SendCommand(ProgressStart);
    SendValue(Progress, 10);
    SticksMode = CheckRange(GetValue(n0), 1, 2);
    GetText(TxNme, TxName);
    SendValue(Progress, 40);
    AutoModelSelect = GetValue(lpm);
    SendValue(Progress, 50);
    LowBattery = GetValue(Bwn);
    SendValue(Progress, 60);
    ScreenTimeout = GetValue(ScreenViewTimeout);
    SendValue(Progress, 70);
    SendValue(Progress, 80);
    Inactivity_Timeout = GetValue(Pto) * TICKSPERMINUTE;
    if (Inactivity_Timeout < INACTIVITYMINIMUM)
        Inactivity_Timeout = INACTIVITYMINIMUM;
    if (Inactivity_Timeout > INACTIVITYMAXIMUM)
        Inactivity_Timeout = INACTIVITYMAXIMUM;
    SendValue(Progress, 90);
    FixDeltaGMTSign();
    if (BuddyPupilOnPPM)
    {
        Connected = false;
        LostContactFlag = true;
        PacketsPerSecond = 0;
        RecentGoodPacketsCount = 0; //
        BlueLedOn();
    }
    SaveAllParameters();
    SendValue(Progress, 95);
    SendValue(Progress, 100);
    CurrentView = TXSETUPVIEW;
    SendCommand(pTXSetupView);
    LastTimeRead = 0;
    SendCommand(ProgressEnd);
    UpdateModelsNameEveryWhere();
    ClearText();
    ConfigureStickMode();
}

/*****************************************************************************************************/
void Blank()
{
    return;
}

/*********************************************************************************************************************************/

void EndReverseView()
{ // channel reverse flags are 16 individual BITs in var 'ReversedChannelBITS'
    char fs[16][5] = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
    uint8_t i;
    char ProgressStart[] = "vis Progress,1";
    char Progress[] = "Progress";
    SendCommand(ProgressStart);
    ReversedChannelBITS = 0;
    for (i = 0; i < 16; ++i)
    {
        SendValue(Progress, (i * (100 / 16)));
        if (GetValue(fs[i]))
            ReversedChannelBITS |= 1 << i; // set a BIT
    }
    SaveOneModel(ModelNumber);
    SendCommand(pRXSetupView);
    CurrentView = RXSETUPVIEW;
    UpdateModelsNameEveryWhere();
}

/*********************************************************************************************************************************/

void StartReverseView()
{ // channel reverse flags are 16 individual BITs in ReversedChannelBITS
    char pReverseView[] = "page ReverseView";
    char fs[16][5] = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
    uint8_t i;
    char ProgressStart[] = "vis Progress,1";
    char ProgressEnd[] = "vis Progress,0";
    char Progress[] = "Progress";

    CurrentView = REVERSEVIEW;
    SendCommand(pReverseView);
    UpdateButtonLabels();
    SendCommand(ProgressStart);
    for (i = 0; i < 16; ++i)
    {
        SendValue(Progress, (i * (100 / 16)));
        if (ReversedChannelBITS & 1 << i)
        { // is BIT set??
            SendValue(fs[i], 1);
        }
        else
        {
            SendValue(fs[i], 0);
        }
    }
    SendCommand(ProgressEnd);
    UpdateModelsNameEveryWhere();
}

/*********************************************************************************************************************************/

void FixCHNames() // channel names on Mix screen now with Bank name and enabled status too.
{
    char MixesView_chM[] = "chM";
    char MixesView_chS[] = "chS";
    char BankNameLable[] = "t10";
    char All[] = "All banks";
    SendText(MixesView_chM, ChannelNames[ScreenData[MASTERCHANNEL] - 1]); // show master channel
    SendText(MixesView_chS, ChannelNames[ScreenData[SLAVECHANNEL] - 1]);  // show slave channel
    if (ScreenData[BANK] > 0)
    {
        SendText(BankNameLable, BankTexts[BanksInUse[ScreenData[BANK] - 1]]); // Show bank name
    }
    else
    {
        SendText(BankNameLable, All);
    }
}

/*********************************************************************************************************************************/

void ReadMixValues() // just reads from the screen and saves to Mixes array

{
    Mixes[MixNumber][M_MIX_INPUTS] = ScreenData[0];
    Mixes[MixNumber][M_MIX_OUTPUTS] = ScreenData[1];
    Mixes[MixNumber][M_Bank] = ScreenData[2];
    Mixes[MixNumber][M_MasterChannel] = ScreenData[3];
    Mixes[MixNumber][M_SlaveChannel] = ScreenData[4];
    Mixes[MixNumber][M_ONEDIRECTION] = ScreenData[5];
    Mixes[MixNumber][M_Reversed] = ScreenData[6];
    Mixes[MixNumber][M_OFFSET] = ScreenData[7] + 127; // because it's unsigned
    Mixes[MixNumber][M_Percent] = ScreenData[8];
    FixCHNames();
}

/*********************************************************************************************************************************/

void ShowMixValues() // sends mix values to Nextion screen
{
    char MixesView_MixOutput[] = "Enabled";
    char MixesView_MixInput[] = "c0";
    char MixesView_Bank[] = "FlightMode";
    char MixesView_MasterChannel[] = "MasterChannel";
    char MixesView_SlaveChannel[] = "SlaveChannel";
    char MixesView_Reversed[] = "Reversed";
    char MixesView_Percent[] = "Percent";
    char MixesView_chM[] = "chM";
    char MixesView_chS[] = "chS";
    char MixesView_od[] = "od";
    char MixesView_offset[] = "Offset";

    SendValue(MixesView_MixOutput, Mixes[MixNumber][M_MIX_OUTPUTS]); // heer load the ScreenData array with the mix values tomorrow
    ScreenData[MIXINPUT] = Mixes[MixNumber][M_MIX_INPUTS];
    SendValue(MixesView_MixInput, Mixes[MixNumber][M_MIX_INPUTS]);
    ScreenData[MIXOUTPUT] = Mixes[MixNumber][M_MIX_INPUTS];
    SendValue(MixesView_Bank, Mixes[MixNumber][M_Bank]);
    ScreenData[BANK] = Mixes[MixNumber][M_Bank];
    if (Mixes[MixNumber][M_MasterChannel] == 0)
        Mixes[MixNumber][M_MasterChannel] = 1;
    SendValue(MixesView_MasterChannel, Mixes[MixNumber][M_MasterChannel]);
    ScreenData[MASTERCHANNEL] = Mixes[MixNumber][M_MasterChannel];
    if (Mixes[MixNumber][M_SlaveChannel] == 0)
        Mixes[MixNumber][M_SlaveChannel] = 1;
    SendValue(MixesView_SlaveChannel, Mixes[MixNumber][M_SlaveChannel]);
    ScreenData[SLAVECHANNEL] = Mixes[MixNumber][M_SlaveChannel];
    SendValue(MixesView_Reversed, Mixes[MixNumber][M_Reversed]);
    ScreenData[REVERSED] = Mixes[MixNumber][M_Reversed];
    if (Mixes[MixNumber][M_Percent] == 0)
        Mixes[MixNumber][M_Percent] = 100;
    if (Mixes[MixNumber][M_SlaveChannel] == Mixes[MixNumber][M_MasterChannel])
    {
        Mixes[MixNumber][M_SlaveChannel]++;
        SendValue(MixesView_SlaveChannel, Mixes[MixNumber][M_SlaveChannel]);
        ScreenData[SLAVECHANNEL] = Mixes[MixNumber][M_SlaveChannel];
    }
    SendValue(MixesView_Percent, Mixes[MixNumber][M_Percent]);
    ScreenData[PERCENT] = Mixes[MixNumber][M_Percent];
    SendValue(MixesView_od, Mixes[MixNumber][M_ONEDIRECTION]);
    ScreenData[ONEDIRECTION] = Mixes[MixNumber][M_ONEDIRECTION];
    if (((Mixes[MixNumber][M_OFFSET]) > 227) || ((Mixes[MixNumber][M_OFFSET]) < 27))
        Mixes[MixNumber][M_OFFSET] = 127;                          // zeroed if out of range
    SendValue(MixesView_offset, Mixes[MixNumber][M_OFFSET] - 127); // because it's 'unsigned'
    ScreenData[OFFSET] = Mixes[MixNumber][M_OFFSET] - 127;
    SendText(MixesView_chM, ChannelNames[Mixes[MixNumber][M_MasterChannel] - 1]);
    SendText(MixesView_chS, ChannelNames[Mixes[MixNumber][M_SlaveChannel] - 1]);
}

/***************************************************** ShowChannelName ****************************************************************************/

void ShowChannelName()
{
    char MoveToChannel[] = "Mch";
    char MacrosView_chM[] = "chM";
    uint8_t ch = GetValue(MoveToChannel);
    if (ch > 0)
        --ch; // no zero
    SendText(MacrosView_chM, ChannelNames[ch]);
}
/*********************************************************************************************************************************/

void ExitMacrosView()
{
    char MacroNumber[] = "Mno";
    char TriggerChannel[] = "Tch";
    char MoveToChannel[] = "Mch";
    char MoveToPosition[] = "Pos";
    char Delay[] = "Del";
    char Duration[] = "Dur";
    uint8_t n = GetValue(MacroNumber) - 1;
    MacrosBuffer[n][MACROTRIGGERCHANNEL] = GetValue(TriggerChannel);
    MacrosBuffer[n][MACROMOVECHANNEL] = GetValue(MoveToChannel);
    MacrosBuffer[n][MACROMOVETOPOSITION] = GetValue(MoveToPosition);
    MacrosBuffer[n][MACROSTARTTIME] = GetValue(Delay);
    MacrosBuffer[n][MACRODURATION] = GetValue(Duration);
    UseMacros = true;
    SaveOneModel(ModelNumber);
    SendCommand(pRXSetupView);
    CurrentView = RXSETUPVIEW;
    UpdateModelsNameEveryWhere();
}

/*********************************************************************************************************************************/
void EndWifiScan()
{
    CurrentView = TXSETUPVIEW;
    SendCommand(pTXSetupView);
    LastTimeRead = 0;
    DoScanEnd();
    UpdateModelsNameEveryWhere();
    ClearText();
}
/*********************************************************************************************************************************/

void StartWifiScan()
{
    char prompt[] = "Model still connected! Continue?";
    if (ModelMatched && BoundFlag)
    {
        if (!GetConfirmation(pTXSetupView, prompt))
            return;
    }
    if (PPMdata.UseTXModule)
    {
        InitRadio(DefaultPipe); // because scan fails if radio isn't initialised
        ConfigureRadio();
    }
    SendCommand(pFhssView);
    DrawFhssBox();
    DoScanInit();
    CurrentMode = SCANWAVEBAND;
    CurrentView = SCANVIEW;
    BlueLedOn();
    ClearText();
}

/***************************************************** Populate Macros View ****************************************************************************/

void PopulateMacrosView()
{
    char MacroNumber[] = "Mno";
    char TriggerChannel[] = "Tch";
    char MoveToChannel[] = "Mch";
    char MoveToPosition[] = "Pos";
    char Delay[] = "Del";
    char Duration[] = "Dur";
    uint8_t n = PreviousMacroNumber;

    if (n < 8)
    { // Read previous values before moveing to next
        MacrosBuffer[n][MACROTRIGGERCHANNEL] = GetValue(TriggerChannel);
        MacrosBuffer[n][MACROMOVECHANNEL] = GetValue(MoveToChannel);
        MacrosBuffer[n][MACROMOVETOPOSITION] = GetValue(MoveToPosition);
        MacrosBuffer[n][MACROSTARTTIME] = GetValue(Delay);
        MacrosBuffer[n][MACRODURATION] = GetValue(Duration);
    }
    n = GetValue(MacroNumber) - 1;
    SendValue(TriggerChannel, MacrosBuffer[n][MACROTRIGGERCHANNEL]);
    SendValue(MoveToChannel, MacrosBuffer[n][MACROMOVECHANNEL]);
    SendValue(MoveToPosition, MacrosBuffer[n][MACROMOVETOPOSITION]);
    SendValue(Delay, MacrosBuffer[n][MACROSTARTTIME]);
    SendValue(Duration, MacrosBuffer[n][MACRODURATION]);
    ShowChannelName();
    PreviousMacroNumber = n;
}

/******************************************************************************************************************************/
void GotoMacrosView()
{
    char pMacrosView[] = "page MacrosView";
    PreviousMacroNumber = 200; // i.e. no usable number
    SendCommand(pMacrosView);  // Display MacroView
    CurrentView = MACROS_VIEW;
    DelayWithDog(200); // allow enough time for screen to display
    UpdateModelsNameEveryWhere();
    PopulateMacrosView();
}

/******************************************************************************************************************************/
void GotoModelsView()
{
    char prompt[] = "Model still connected! Continue?";
    if (ModelMatched && BoundFlag)
    {
        if (!GetConfirmation(pRXSetupView, prompt))
            return;
    }
    SaveCurrentModel();
    SendCommand(pModelsView);
    CurrentView = MODELSVIEW;
    UpdateModelsNameEveryWhere();
    strcpy(MOD, ".MOD");
    BuildDirectory();
    strcpy(Mfiles, "Mfiles");
    LoadFileSelector();
    ShowFileNumber();
    PreviousModelNumber = ModelNumber; // save number
    LoadModelSelector();
}

/******************************************************************************************************************************/
void DoLastTimeRead()
{
    LastTimeRead = 0;
}
/******************************************************************************************************************************/

void ModelViewEnd()
{
    char pr[] = "Select ";
    char buf[60];
    char q[] = "?";
    if (PreviousModelNumber != ModelNumber)
    {
        strcpy(buf, pr);
        strcat(buf, ModelName);
        strcat(buf, q);
        GetConfirmation(pModelsView, buf);
        if (Confirmed[0] != 'Y')
        {
            ModelNumber = PreviousModelNumber;
            ReadOneModel(ModelNumber);
        }
    }
    SaveAllParameters();
    GotoFrontView();
}

/******************************************************************************************************************************/

void DoMFName()
{
    DelayWithDog(100);
    CheckModelName();
}

/******************************************************************************************************************************/
// This function receives upto 50 data elements from the Nextion display and loads it into ScreenData array of uint16_t

void ReceiveLotsofData()
{
    int i = 0;
    union
    {
        uint8_t First4Bytes[4];
        uint32_t FirstDWord;
    } NextionData;

    for (int field = 1; field < 49; ++field)
    {
        int offset = field * 4;
        for (int p = 0; p < 4; ++p)
            NextionData.First4Bytes[p] = TextIn[offset + p];
        if (NextionData.FirstDWord < 0xFFFF)
        {
            ScreenData[i] = NextionData.FirstDWord;
            ++i;
        }
        else
        {
            break;
        }
    }
    switch (CurrentView)
    {
    case MIXESVIEW:
        ReadMixValues();
        break;
    case DUALRATESVIEW:
        DualRatesRefresh();
        break;
    default:
        break;
    }
}

/******************************************************************************************************************************/

void TXModuleViewEnd()
{
    char GoBack[] = "page TXModuleView";
    char c1[] = "c1"; // Use module
    char n3[] = "n3"; // number of channels
    char r0[] = "r0";
    char r1[] = "r1";
    char r2[] = "r2";
    char prompt[] = "Power off transmitter?";
    bool oldUseTxModule = PPMdata.UseTXModule;
    char ProgressStart[] = "vis Progress,1";
    char Progress[] = "Progress";

    SendCommand(ProgressStart);
    SendValue(Progress, 10);
    PPMdata.UseTXModule = GetValue(c1);
    if (PPMdata.UseTXModule != oldUseTxModule)
    {
        if (!GetConfirmation(GoBack, prompt))
        {
            PPMdata.UseTXModule = oldUseTxModule;
            SendValue(c1, PPMdata.UseTXModule);
            return;
        }
    }
    SendValue(Progress, 30);
    DelayWithDog(100);
    PPMdata.PPMChannelsNumber = GetValue(n3);
    SendValue(Progress, 51);
    if (GetValue(r0))
        PPMdata.PPMOrderSelection = 1;
    SendValue(Progress, 63);
    DelayWithDog(10);
    if (GetValue(r1))
        PPMdata.PPMOrderSelection = 2;
    SendValue(Progress, 88);
    DelayWithDog(10);
    if (GetValue(r2))
        PPMdata.PPMOrderSelection = 3;
    SelectChannelOrder();
    SendValue(Progress, 99);
    DelayWithDog(10);
    SaveTransmitterParameters();
    DelayWithDog(10);
    SendCommand(pTXSetupView);
    CurrentView = TXSETUPVIEW;
    DelayWithDog(10);
    if (PPMdata.UseTXModule != oldUseTxModule)
    {
        digitalWrite(POWER_OFF_PIN, HIGH);
    }
}

/******************************************************************************************************************************/

void SaveSwitches()
{
    SaveTransmitterParameters();
    DelayWithDog(100);
    char pTXSetupView[] = "page TXSetupView";
    SendCommand(pTXSetupView);
}

/******************************************************************************************************************************/

void StartTXSetupView()
{
    CurrentView = TXSETUPVIEW;
    SendCommand(pTXSetupView);
    UpdateModelsNameEveryWhere();
    ClearText();
}

/******************************************************************************************************************************/

void StartAudioVisualView()
{

    char n0[] = "n0";
    char n1[] = "n1";
    char h0[] = "h0";
    char Ex1[] = "Ex1"; // slider
    char c0[] = "c0";
    char c1[] = "c1";
    char c2[] = "c2";
    char c3[] = "c3";
    char c4[] = "c4";
    char c5[] = "c5";
    CurrentView = AUDIOVIEW;
    SendCommand(pAudioView);
    SendValue(n0, AudioVolume);
    SendValue(Ex1, AudioVolume);
    SendValue(n1, Brightness);
    SendValue(h0, Brightness);
    SendValue(c0, PlayFanfare);
    SendValue(c1, TrimClicks);
    SendValue(c2, ButtonClicks);
    SendValue(c3, SpeakingClock);
    SendValue(c4, AnnounceBanks);
    SendValue(c5, AnnounceConnected);
    SetAudioVolume(AudioVolume);
    ClearText();
}

/******************************************************************************************************************************/

void EndAudioVisualView()
{
    char n0[] = "n0";
    char n1[] = "n1";
    char c0[] = "c0";
    char c1[] = "c1";
    char c2[] = "c2";
    char c3[] = "c3";
    char c4[] = "c4";
    char c5[] = "c5";
    AudioVolume = GetValue(n0);
    Brightness = GetValue(n1);
    PlayFanfare = GetValue(c0);
    TrimClicks = GetValue(c1);
    ButtonClicks = GetValue(c2);
    SpeakingClock = GetValue(c3);
    AnnounceBanks = GetValue(c4);
    AnnounceConnected = GetValue(c5);
    SetAudioVolume(AudioVolume);
    CurrentView = TXSETUPVIEW;
    SendCommand(pTXSetupView);
    LastTimeRead = 0;
    SaveTransmitterParameters();
    UpdateModelsNameEveryWhere();
    ClearText();
}

// ********************************************************************************************************************************************

void DeleteModel()
{

    char Prompt[60];
    char del[] = "Delete ";
    char ques[] = "?";
    char MMems[] = "MMems";
    char msg[] = "Model deleted!";
    strcpy(Prompt, del);
    strcat(Prompt, ModelName);
    strcat(Prompt, ques);
    if (GetConfirmation(pModelsView, Prompt))
    {
        ModelNumber = GetValue(MMems) + 1;
        SetDefaultValues();
        SaveOneModel(ModelNumber);
        LoadModelSelector();
        MsgBox(pModelsView, msg);
    }
    ClearText();
}

// ********************************************************************************************************************************************

void InputsViewEnd()
{
    char ProgressStart[] = "vis Progress,1";
    char Progress[] = "Progress";
    char InputStick_Labels[16][4] = {"c1", "c2", "c3", "c4", "c5", "c6", "c7", "c8", "c9", "c10", "c11", "c12", "c13", "c14", "c15", "c16"};
    char OutputStick_Labels[16][4] = {"n4", "n5", "n6", "n7", "n8", "n9", "n10", "n11", "n12", "n13", "n14", "n15", "n16", "n17", "n18", "n19"};
    char InputTrim_labels[4][4] = {"n0", "n1", "n2", "n3"};
    char changes[260];
    char changes_label[] = "change";
    char ChangesCleared[] = "XX:"; // clear the changes label The two Xs are just so that I can see it
    bool Altered = false;

    for (uint16_t i = 0; i < 250; ++i) // get the changes from the Nextion TextIn
    {
        changes[i] = TextIn[i + 4];
        changes[i + 1] = 0;
    }
    // Look(changes); // A string of changed labels sent by nextion screen
    SendCommand(ProgressStart);
    for (uint8_t i = 0; i < 16; ++i)
    {
        if (InStrng(InputStick_Labels[i], changes)) // if this input stick has changed
        {
            InPutStick[i] = CheckRange((GetValue(InputStick_Labels[i]) - 1), 0, 15); // get the value and check it
            Altered = true;                                                          // set the flag
        }
        if (InStrng(OutputStick_Labels[i], changes)) // if this output channel has changed
        {
            ChannelOutPut[i] = CheckRange((GetValue(OutputStick_Labels[i]) - 1), 0, 15); // get the value and check it
            Altered = true;                                                              // set the flag
        }
        if (i < 4)
        {
            if (InStrng(InputTrim_labels[i], changes)) // if this  trim output has changed
            {
                InputTrim[i] = CheckRange((GetValue(InputTrim_labels[i]) - 1), 0, 15); // get the value and check it
                Altered = true;                                                        // set the flag
            }
        }
        if (Altered)
            SendValue(Progress, (((i + 1) * 100) / 16) - 1);
    }

    if (Altered)
    {
        SendValue(Progress, 95);
        SendText(changes_label, ChangesCleared);
        CheckOutPutChannels();
        SaveOneModel(ModelNumber);
    }
    SendValue(Progress, 100);
    UpdateButtonLabels();
    CurrentView = RXSETUPVIEW;
    SendCommand(pRXSetupView);
    LastTimeRead = 0;
    ClearText();
}

/*********************************************************************************************************************************/

void StartBuddyView()
{
    char BuddyM[] = "BuddyM";
    char BuddyP[] = "BuddyP";
    char BuddyWireless[] = "wireless";
    char pBuddyView[] = "page BuddyView";
    SendCommand(pBuddyView);
    CurrentView = BUDDYVIEW;
    WirelessBuddy = true; // default to wireless
    SendValue(BuddyM, BuddyMasterOnPPM || BuddyMasterOnWireless);
    SendValue(BuddyP, BuddyPupilOnPPM || BuddyPupilOnWireless);
    SendValue(BuddyWireless, WirelessBuddy);
}

/*********************************************************************************************************************************/

void EndBuddyView()
{
    char BuddyM[] = "BuddyM";
    char BuddyP[] = "BuddyP";
    bool Pupil = false;
    bool Master = false;
    char BuddyWireless[] = "wireless";

    Pupil = GetValue(BuddyP);                // Pupil ?
    Master = GetValue(BuddyM);               // Master ?
    WirelessBuddy = GetValue(BuddyWireless); // wireless ?
    if (Pupil)
    {
        if (WirelessBuddy) // wireless switch on?
        {
            BuddyPupilOnWireless = true;
            BuddyPupilOnPPM = false;
        }
        if (!WirelessBuddy)
        {
            BuddyPupilOnWireless = false;
            BuddyPupilOnPPM = true;
        }
    }
    if (!Pupil)
    {
        BuddyPupilOnWireless = false;
        BuddyPupilOnPPM = false;
    }

    if (Master)
    {
        if (WirelessBuddy)
        { // wireless switch on?
            BuddyMasterOnWireless = true;
            BuddyMasterOnPPM = false;
        }
        if (!WirelessBuddy)
        { // wireless switch off?
            BuddyMasterOnWireless = false;
            BuddyMasterOnPPM = true;
        }
    }
    if (!Master)
    {
        BuddyMasterOnWireless = false;
        BuddyMasterOnPPM = false;
    }
    SaveAllParameters();
    SendCommand(pRXSetupView);
    CurrentView = RXSETUPVIEW;
    UpdateModelsNameEveryWhere();
    RationaliseBuddy();
    if (BuddyPupilOnWireless)
    {
        GotoFrontView();
    }
}

/******************************************************************************************************************************/
void LoadModelSelector()
{
    char MMemsp[] = "MMems.path=\"";
    char MMems[] = "MMems";
    char crlf[] = {13, 10, 0};
    char lb[] = " (";
    char rb[] = ")";
    char nb[4];
    char buf[MAXBUFFERSIZE];
    char mn[] = "modelname";

    int32_t SavedModelNumber = ModelNumber;
    for (ModelNumber = 1; ModelNumber < MAXMODELNUMBER; ++ModelNumber)
    {
        ReadOneModel(ModelNumber);
        if (!ModelsMacUnionSaved.Val64)
        {
            strcpy(lb, " [");
            strcpy(rb, "]");
        }
        else
        {
            strcpy(lb, " (");
            strcpy(rb, ")");
        }
        if (ModelNumber == 1)
        {
            strcpy(buf, ModelName);
            strcat(buf, lb);
            Str(nb, ModelNumber, 0);
            strcat(buf, nb);
            strcat(buf, rb);
            strcat(buf, crlf);
        }
        else
        {
            strcat(buf, ModelName);
            strcat(buf, lb);
            Str(nb, ModelNumber, 0);
            strcat(buf, nb);
            strcat(buf, rb);
            strcat(buf, crlf);
        }
    }
    SendOtherText(MMemsp, buf);
    ModelNumber = SavedModelNumber;
    ReadOneModel(ModelNumber);
    SendValue(MMems, ModelNumber - 1);
    SendText(mn, ModelName);
}

/******************************************************************************************************************************/
void SetupViewFM()
{
    SaveAllParameters();
    CurrentView = RXSETUPVIEW;
    SendCommand(pRXSetupView);
    UpdateModelsNameEveryWhere();
}

/******************************************************************************************************************************/
void Options2End()
{ // back to setup?
    char dGMT[] = "dGMT";
    char pTXSetupView[] = "page TXSetupView";
    DeltaGMT = GetValue(dGMT);
    SaveTransmitterParameters();
    CurrentView = TXSETUPVIEW;
    SendCommand(pTXSetupView);
    UpdateModelsNameEveryWhere();
}
/******************************************************************************************************************************/

void OptionView3End() //
{
    char TxVCorrextion[] = "t2";
    char n1[] = "n1";
    char n2[] = "n2";
    char n3[] = "n3";
    char n4[] = "n4";
    char pTXSetupView[] = "page TXSetupView";
    char QNH[] = "Qnh";

    char sw0[] = "sw0";
    char sw1[] = "sw1";
    char n0[] = "n0";

    TxVoltageCorrection = GetValue(TxVCorrextion);
    PowerOffWarningSeconds = GetValue(n2);
    PowerOffWarningSeconds = CheckRange(PowerOffWarningSeconds, 1, 10);
    Qnh = (uint16_t)GetValue(QNH);
    if (LEDBrightness != GetValue(n1))
        UpdateLED();
    ConnectionAssessSeconds = GetValue(n3);
    ConnectionAssessSeconds = CheckRange(ConnectionAssessSeconds, 1, 6);
    ScanSensitivity = GetValue(n4);
    ScanSensitivity = CheckRange(ScanSensitivity, 1, 255);
    MinimumGap = GetValue(n0);
    if (MinimumGap < 10)
        MinimumGap = 10;
    UseLog = GetValue(sw0);
    LogRXSwaps = GetValue(sw1);

    SaveTransmitterParameters();
    CloseModelsFile();
    AddParameterstoQueue(2); // 2 is the ID for sending QNH value to RX
    CurrentView = TXSETUPVIEW;
    SendCommand(pTXSetupView);
    UpdateModelsNameEveryWhere();
}

/******************************************************************************************************************************/

void OptionView3Start()
{
    char TxVCorrextion[] = "t2";
    char n1[] = "n1";
    char n2[] = "n2";
    char n3[] = "n3";
    char n4[] = "n4";
    char lpm[] = "c0"; // Low power mode
    char OptionV3Start[] = "page OptionView3";
    char QNH[] = "Qnh";
    char sw0[] = "sw0";
    char sw1[] = "sw1";
    char n0[] = "n0";

    CurrentView = OPTIONVIEW3;
    SendCommand(OptionV3Start);
    DelayWithDog(250);
    SendValue(TxVCorrextion, TxVoltageCorrection);
    SendValue(n2, PowerOffWarningSeconds);
    SendValue(n3, ConnectionAssessSeconds);
    SendValue(n4, ScanSensitivity);
    SendValue(lpm, AutoModelSelect);
    if (LEDBrightness < 15)
        LEDBrightness = DEFAULTLEDBRIGHTNESS;
    SendValue(n1, LEDBrightness);
    SendValue(QNH, Qnh);
    SendValue(sw0, UseLog);
    SendValue(sw1, LogRXSwaps);
    SendValue(n0, MinimumGap);
}

/******************************************************************************************************************************/

void OptionView2Start()
{
    char dGMT[] = "dGMT"; // Time zone
    char n1[] = "n1";
    char n2[] = "n2";
    char n3[] = "n3";

    char OptionV2Start[] = "page OptionView2";
    char TxVCorrextion[] = "t2";

    if (CurrentView == OPTIONVIEW3)
    { //  TODO: And what if was Options 1??

        TxVoltageCorrection = GetValue(TxVCorrextion);
        PowerOffWarningSeconds = GetValue(n2);
        PowerOffWarningSeconds = CheckRange(PowerOffWarningSeconds, 1, 10);
        if (LEDBrightness != GetValue(n1))
            UpdateLED();
        ConnectionAssessSeconds = GetValue(n3);
        ConnectionAssessSeconds = CheckRange(ConnectionAssessSeconds, 1, 6);
        SaveAllParameters();
    }

    CurrentView = OPTIONVIEW2;
    LastTimeRead = 0;
    SendCommand(OptionV2Start);
    DelayWithDog(100);
    SendValue(dGMT, DeltaGMT);
}

/******************************************************************************************************************************/
void ResetClock()
{

    char Prompt[] = "Reset clock?";
    char Done[] = "Clock reset!";

    if (GetConfirmation(pOptionView2, Prompt))
    {
        SetDS1307ToCompilerTime();
        MsgBox(pOptionView2, Done);
    }
}

/******************************************************************************************************************************/

void TXModuleViewStart()
{

    char msg[] = "Please disconnect from model first!";
    if (ModelMatched)
    {
        MsgBox(pTXSetupView, msg);
        return;
    }
    CurrentView = TXMODULEVIEW;

    char c1[] = "c1"; // Use module
    char n3[] = "n3"; // number of channels
    char r0[] = "r0";
    char r1[] = "r1";
    char r2[] = "r2";

    SendCommand(pTXModule);
    SendValue(c1, PPMdata.UseTXModule);
    SendValue(n3, PPMdata.PPMChannelsNumber);
    if (PPMdata.PPMOrderSelection == 1)
    {
        SendValue(r0, 1);
    }
    else
    {
        SendValue(r0, 0);
    }
    if (PPMdata.PPMOrderSelection == 2)
    {
        SendValue(r1, 1);
    }
    else
    {
        SendValue(r1, 0);
    }
    if (PPMdata.PPMOrderSelection == 3)
    {
        SendValue(r2, 1);
    }
    else
    {
        SendValue(r2, 0);
    }
}

/******************************************************************************************************************************/

// This implements the impossible "SD card rename file" ... by reading, re-saveing under new name, then deleting old file.

void RenameFile()
{
    char ModelsView_filename[] = "filename";
    char Head[] = "Rename this backup";
    char model[] = "(i.e. just change its filename)";
    char prompt[] = "New filename?";
    char Prompt[50];
    char overwr[] = "Overwrite ";
    char ques[] = "?";
    char Deleteable[42];

    SaveCurrentModel();
    GetText(ModelsView_filename, SingleModelFile);
    strcpy(Deleteable, SingleModelFile);
    LoadModelForRenaming();
    if (GetBackupFilename(pModelsView, SingleModelFile, model, Head, prompt))
    {
        FixFileName();
        if (strcmp(Deleteable, SingleModelFile) == 0)
            return;
        Serial.println(SingleModelFile);
        if (CheckFileExists(SingleModelFile))
        {
            strcpy(Prompt, overwr);
            strcat(Prompt, SingleModelFile);
            strcat(Prompt, ques);
            if (GetConfirmation(pModelsView, Prompt))
            {
                WriteBackup();
                SD.remove(Deleteable);
            }
        }
        else
        {
            WriteBackup();
            SD.remove(Deleteable);
        }
    }
    strcpy(MOD, ".MOD");
    BuildDirectory();
    strcpy(Mfiles, "Mfiles");
    LoadFileSelector();
    RestoreCurrentModel();
}

/******************************************************************************************************************************/

void BuddyChViewStart()
{
    char pBuddyChView[] = "page BuddyChView";
    char fs[16][5] = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
    char mSwitch[] = "mSwitch";
    SendCommand(pBuddyChView);
    CurrentView = BUDDYCHVIEW;
    UpdateButtonLabels();
    SendValue(mSwitch, BuddyHasAllSwitches);

    for (int i = 0; i < 16; ++i)
    {
        if (BuddyControlled & 1 << i)
        {
            SendValue(fs[i], 1);
        }
        else
        {
            SendValue(fs[i], 0);
        }
    }
}

/******************************************************************************************************************************/

void BuddyChViewEnd()
{
    char Progress[] = "Progress";
    char ProgressStart[] = "vis Progress,1";
    char fs[16][5] = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
    char mSwitch[] = "mSwitch";

    SendCommand(ProgressStart);
    BuddyHasAllSwitches = GetValue(mSwitch);
    BuddyControlled = 0;

    for (int i = 0; i < 16; ++i)
    {
        BuddyControlled |= GetValue(fs[i]) << i;
        SendValue(Progress, i * (100 / 16));
    }
    SaveOneModel(ModelNumber);
    CloseModelsFile();
    SendCommand(pBuddyView);
    CurrentView = BUDDYVIEW;
}

#endif
