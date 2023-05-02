// *********************************************** Trims.h *******************************************

#include <Arduino.h>

uint32_t TrimTimer              = 0;


extern uint8_t   SticksMode;
extern int       Trims[BANKSUSED + 1][CHANNELSUSED + 1]; // Trims to store
extern uint8_t   Bank;
extern uint16_t  TrimMultiplier;                         // How much to multiply trim by
extern bool      CopyTrimsToAll;
extern uint8_t   SubTrimToEdit;
extern uint32_t  ModelNumber;
extern uint32_t  LastTimeRead;
extern void      SaveOneModel(uint32_t mnum);
extern bool      DefiningTrims;
extern bool      TrimDefined[4];
extern uint8_t   TrimNumber[8];
extern uint16_t  TrimRepeatSpeed;
extern bool     TrimSwitch[8];
extern uint32_t  TransmitterLastManaged;
// **********************************************************************************************************************************
void StartTrimView()
{
    char pTrimView[] = "page TrimView";
    char n0[]        = "n0";
    char c0[]        = "c0";
    SendCommand(pTrimView);
    CurrentView = TRIM_VIEW;
    SendValue(n0, TrimMultiplier);
    SendValue(c0, CopyTrimsToAll);
    UpdateModelsNameEveryWhere(); // also updates trimview (If CurrentView == TRIM_VIEW!! :-)
    ClearText();
 }
       

/*********************************************************************************************************************************/

int GetTrimAmount(uint8_t InputChannel){ 
    int TrimAmount, tt = InputChannel;
        
        if (SticksMode == 2) {
            if (InputChannel == 1) tt = 2;
            if (InputChannel == 2) tt = 1; 
        }
        TrimAmount = (Trims[Bank][tt] - 80) * TrimMultiplier; // TRIMS on lower four input channels (80 is mid point !! (range 40 - 80 - 120)) 
        return TrimAmount;
}

/******************************************************************************************************************************/
void StartSubTrimView()
{ // Subtrim view start
    char pSubTrimView[] = "page SubTrimView";
    char t2[]           = "t2";
    char n0[]           = "n0";
    char h0[]           = "h0";
    SendCommand(pSubTrimView);
    SubTrimToEdit = 0;
    CurrentView   = SUBTRIMVIEW;
    SendText(t2, ChannelNames[SubTrimToEdit]);
    SendValue(n0, SubTrims[SubTrimToEdit] - 127);
    SendValue(h0, SubTrims[SubTrimToEdit]);
    UpdateModelsNameEveryWhere();
}
/******************************************************************************************************************************/
void EndSubTrimView()
{ // Subtrim view exit
   char page_RXSetupView[] = "page RXSetupView";

    SaveOneModel(ModelNumber);
    CurrentView = RXSETUPVIEW;
    SendCommand(page_RXSetupView);
    LastTimeRead = 0;
    UpdateModelsNameEveryWhere();
}
/******************************************************************************************************************************/
void StartTrimDefView()
{
    char pTrimDefView[] = "page TrimDefView";
    CurrentView         = TRIMDEFVIEW;
    SendCommand(pTrimDefView);
    ResetAllTrims();
    BlueLedOn();
    CurrentMode = SENDNOTHING;
    for (int i = 0; i < 4; ++i) TrimDefined[i] = false; 
    DefiningTrims = true;
   
}
/******************************************************************************************************************************/
void DefineTrimsEnd()
{ // exit from trim defining screen
    char pCalibrateView[] = "page CalibrateView";
    CurrentView           = TXSETUPVIEW;
    SendCommand(pCalibrateView);
    Force_ReDisplay();
    CurrentView   = CALIBRATEVIEW;
    DefiningTrims = false;
    CurrentMode   = NORMAL;
    SaveTransmitterParameters();
    UpdateModelsNameEveryWhere();
  
}
/******************************************************************************************************************************/
void ResetAllTrims() 
{
     
if (SticksMode == 1) { 
        TrimNumber[0] = TRIM1A;  // these will change when redefined
        TrimNumber[1] = TRIM1B;
        TrimNumber[2] = TRIM2A;
        TrimNumber[3] = TRIM2B;
        TrimNumber[4] = TRIM3A;
        TrimNumber[5] = TRIM3B;
        TrimNumber[6] = TRIM4A;
        TrimNumber[7] = TRIM4B;
    }

if (SticksMode == 2) {
        TrimNumber[0] = TRIM1A; 
        TrimNumber[1] = TRIM1B;
        TrimNumber[4] = TRIM2A;
        TrimNumber[5] = TRIM2B;
        TrimNumber[2] = TRIM3A;
        TrimNumber[3] = TRIM3B;
        TrimNumber[6] = TRIM4A;
        TrimNumber[7] = TRIM4B;
    }          
}


/************************************************************************************************************/

void SetATrimDefinition(int i) 
{
    char AilDone[] = "Aileron trim is defined!";
    char EleDone[] = "Elevator trim is defined!";
    char ThrDone[] = "Throttle trim is defined!";
    char RudDone[] = "Rudder trim is defined!";
   
    char ail[]     = "ail";
    char ele[]     = "ele";
    char thr[]     = "thr";
    char rud[]     = "rud";
   
    // Aileron
    if (!TrimDefined[0]) {
        if ((i == 0) || (i == 1)) {
            PlaySound(BEEPCOMPLETE);
            SendText(ail, AilDone);
            TrimDefined[0] = true;
        }
        if (i == 0) {
            TrimNumber[0] = TRIM1A;
            TrimNumber[1] = TRIM1B;
        }
        if (i == 1) {
            TrimNumber[1] = TRIM1A;
            TrimNumber[0] = TRIM1B;
        }
    }

if (SticksMode == 1){
    // Elevator
    if (!TrimDefined[1]) {
        if ((i == 2) || (i == 3)) {
            PlaySound(BEEPCOMPLETE);
            SendText(ele, EleDone);
            TrimDefined[1] = true;
        }
        if (i == 3) {
            TrimNumber[2] = TRIM2A;
            TrimNumber[3] = TRIM2B;
        }
        if (i == 2) {
            TrimNumber[3] = TRIM2A;
            TrimNumber[2] = TRIM2B;
        }
    }

    // Throttle
    if (!TrimDefined[2]) {
        if ((i == 4) || (i == 5)) {
            PlaySound(BEEPCOMPLETE);
            SendText(thr, ThrDone);
            TrimDefined[2] = true;
        }
        if (i == 4) {
            TrimNumber[4] = TRIM3A;
            TrimNumber[5] = TRIM3B;
        }
        if (i == 5) {
            TrimNumber[5] = TRIM3A;
            TrimNumber[4] = TRIM3B;
        }
    }
}

if (SticksMode == 2){
    // Throttle
    if (!TrimDefined[1]) {
        if ((i == 4) || (i == 5)) {
            PlaySound(BEEPCOMPLETE);
            SendText(thr, ThrDone);
            TrimDefined[1] = true;
        }
        if (i == 5) {
            TrimNumber[5] = TRIM2A;
            TrimNumber[4] = TRIM2B;
        }
        if (i == 4) {
            TrimNumber[4] = TRIM2A;
            TrimNumber[5] = TRIM2B;
        }
    }

    // Elevator
    if (!TrimDefined[2]) {
        if ((i == 2) || (i == 3)) {
            PlaySound(BEEPCOMPLETE);
            SendText(ele, EleDone);
            TrimDefined[2] = true;
        }
        if (i == 3) {
            TrimNumber[2] = TRIM3A;
            TrimNumber[3] = TRIM3B;
        }
        if (i == 2) {
            TrimNumber[3] = TRIM3A;
            TrimNumber[2] = TRIM3B;
        }
    }
}

    // Rudder
    if (!TrimDefined[3]) {
        if ((i == 6) || (i == 7)) {
            PlaySound(BEEPCOMPLETE);
            SendText(rud, RudDone);
            TrimDefined[3] = true;
        }
        if (i == 6) {
            TrimNumber[6] = TRIM4A;
            TrimNumber[7] = TRIM4B;
        }
        if (i == 7) {
            TrimNumber[7] = TRIM4A;
            TrimNumber[6] = TRIM4B;
        }
    }
}

/************************************************************************************************************/

void CheckHardwareTrims()
{
    int i;
    if ((millis() - TrimTimer) < TrimRepeatSpeed) return; // check occasionally for trim press 
    TrimTimer = millis();
    for (i = 0; i < 8; ++i) {
        if (TrimSwitch[i]) {
            if (DefiningTrims) {
                SetATrimDefinition(i);
                return;
            }
            MoveaTrim(i);
            TransmitterLastManaged = 0;                     //  to speed up repeat
            TrimRepeatSpeed -= (TrimRepeatSpeed / 4);       //  accelerate repeat...
            if (TrimRepeatSpeed < 10) TrimRepeatSpeed = 30; //  ... up to a point...
        }
    }
}
