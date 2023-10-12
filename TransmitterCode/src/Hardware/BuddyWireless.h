// *************************************** BuddyWireless.h  *****************************************
#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef BUDDYWIRELESS_H
    #define BUDDYWIRELESS_H
    #define SPECIAL_PACKET_CHANNEL 125 // Which channel for the special packets
    #define INTERBUDDYRATE         205 // 5 times a second (Fails below 200)
    #define SHORT_DELAY            200 // ... microseconds
    #define LONGER_DELAY           1   // ... milliseconds
    #define LOSTCONTACTTHRESHOLD   3   // 3 fails in a row and we declare the buddy dead

//*************************************************************************************************************************
// This function is called by Pupil and the Master was Detected - or not Detected.
// After 3 failed detections, the Master is declared dead, and a message is changed on front view.
void MasterDetected(bool Detected)
{
    static uint16_t LostMasterCount = 0;
    char            Mlost[]         = "Master not found";
    char            Mfound[]        = "Master found!";
    char            wb[]            = "wb"; // wb is the name of the label on front view
    char            YesVisible[]    = "vis wb,1";

    if (Detected) {
        LostMasterCount = 0;
        if (MasterIsAlive != 1) { // MasterIsAlive = 1 means Master was already alive
            SendText(wb, Mfound);
            SendCommand(YesVisible);
            MasterIsAlive = 1;
        }
    }
    else {
        ++LostMasterCount;
        if (LostMasterCount > LOSTCONTACTTHRESHOLD) {
            if (MasterIsAlive != 2) { // MasterIsAlive = 2 means Master was already dead
                SendText(wb, Mlost);
                SendCommand(YesVisible);
                MasterIsAlive = 2;
            }
        }
    }
}

//*************************************************************************************************************************
// This function is called by Master when the Pupil was Detected - or not Detected.
// After 3 failed detections, the Pupil is declared dead, and a message is changed on front view.
void PupilDetected(bool Detected)
{
    static uint16_t LostPupilCount = 0;
    char            Mlost[]        = "Buddy not found";
    char            Mfound[]       = "Buddy found!";
    char            wb[]           = "wb"; // wb is the name of the label on front view
    char            YesVisible[]   = "vis wb,1";
    if (Detected) {
        LostPupilCount = 0;
        if (PupilIsAlive != 1) { // PupilIsAlive = 1 means Pupil was already alive
            SendText(wb, Mfound);
            SendCommand(YesVisible);
            PupilIsAlive = 1;
        }
    }
    else {
        ++LostPupilCount;
        if (LostPupilCount > LOSTCONTACTTHRESHOLD) {
            if (PupilIsAlive != 2) { // PupilIsAlive = 2 means Pupil was already dead
                SendText(wb, Mlost);
                SendCommand(YesVisible);
                PupilIsAlive = 2;
            }
        }
    }
}

//*************************************************************************************************************************

void GetMasterAck() // Here Pupil gets Ack from master while Pupil is in control
// if ack is 'S' then master is taking control back
// if ack is 'O' then master is ok with pupil staying in control
{
    char AckSpecial[2] = " "; // simple ack
    Radio1.read(&AckSpecial, 2);
    if (AckSpecial[0] == 'O') return; // OK to continue sending. no action needed
    if (AckSpecial[0] == 'S') {       // PUPIL -> LISTEN  <<<<<< *******
        PlaySound(MASTERMSG);
        StartBuddyListen(0); // 'S' from Master = Shut Up now, send nothing
        DelayWithDog(LONGER_DELAY);
    }
    FlushFifos();
    MasterDetected(true); // Master is alive
    return;
}

//*************************************************************************************************************************

void GetPupilAck() // Master gets Ack from pupil while MASTER in control
// if ack exists at all then pupil is alive
{
    uint8_t AckSpecial[2];       // simple ack
    Radio1.read(&AckSpecial, 2); // don't care what it is
    PupilDetected(true);         // Pupil is alive
    FlushFifos();
    return;
}
//*************************************************************************************************************************

void SendSpecialPacket(bool IamMaster) // here the sender sends to other tx while in control. This is called about 5 times a second.
// hence the sender always stays in send mode, and the silent tx stays in receive (LISTEN) mode, only switching to send when takes control.
{
    uint8_t Master_in_Control[2] = "S"; // = Shut Up, send nothing
    uint8_t Pupil_in_Control[2]  = "O"; // = OK to send data
    uint8_t Pupil_is_Alive[2]    = "P";
    FlushFifos();
    Radio1.setChannel(SPECIAL_PACKET_CHANNEL);
    delayMicroseconds(SHORT_DELAY);
    Radio1.stopListening();
    delayMicroseconds(SHORT_DELAY);

    if (IamMaster) {
        if (!BuddyON)
        {
            delayMicroseconds(SHORT_DELAY);
            if (Radio1.write(&Master_in_Control, sizeof(Master_in_Control))) {
                GetPupilAck(); // get ack from pupil
            }
            else {
                PupilDetected(false);
            }
        }
        if (BuddyON) //  PUPIL goes into CONTROL
        {
            if (Radio1.write(&Pupil_in_Control, sizeof(Pupil_in_Control))) { // SEND O to Pupil. If we can't, he's dead
                GetPupilAck();
                StartBuddyListen(1); // MASTER -> LISTEN  (Pupil took control)  <<<<<< *******
                PlaySound(BUDDYMSG);
                DelayWithDog(LONGER_DELAY);
                LastPassivePacketTime = millis();
                return;
            }
            else {
                PupilDetected(false);
            }
        }
    }
    if (!IamMaster) {                                                // pupil area when in control *************************************************************
        if (Radio1.write(&Pupil_is_Alive, sizeof(Pupil_is_Alive))) { // send P to master
            MasterDetected(true);
            delayMicroseconds(SHORT_DELAY);
            GetMasterAck();
        }
        else {                   // master must be either back in control, or ** dead ***. Pupil will shut up in either case
            StartBuddyListen(0); // <<<<<<<<<<<<<<<<<<<  PUPIL -> LISTEN <<<<<< *******
            MasterDetected(false);
            PlaySound(MASTERMSG);
            DelayWithDog(LONGER_DELAY);
            FlushFifos();
        }
        if (CurrentMode == LISTENMODE) return; // control might have ceased
    }
    Radio1.setChannel(CurrentChannel); // restore the proper frequency
    delayMicroseconds(SHORT_DELAY);
    Radio1.stopListening();
    delayMicroseconds(SHORT_DELAY);
}

//*************************************************************************************************************************

void GetSpecialPacket(bool IamMaster) // here the passive tx gets from active tx
{
    char DataPacket[2]        = " ";
    char Master_in_Control[2] = "S"; // = Shut Up, send nothing
    char Pupil_in_Control[2]  = "O"; // = OK to send data
    char Ack[2]               = "P"; // Pupil ack
    bool TakeControlBackNow   = false;

    if (Radio1.available()) {
        if (IamMaster) { // master here
            if (BuddyON)
            {
                Ack[0]             = 'O'; // ok to continue sending - you're in charge
                TakeControlBackNow = false;
            }
            if (!BuddyON)
            {
                Ack[0]             = 'S'; // shut up now as buddy is now off.... *********
                TakeControlBackNow = true;
            }
        }
        delayMicroseconds(SHORT_DELAY);
        Radio1.writeAckPayload(1, &Ack, 2); // Acknowledge the packet
        delayMicroseconds(SHORT_DELAY);
        Radio1.read(&DataPacket, sizeof(DataPacket));
        if (!IamMaster) { // pupil here
            MasterDetected(true);
            if (DataPacket[0] == Master_in_Control[0]) {
            } // no action needed
            if (DataPacket[0] == Pupil_in_Control[0]) {
                StopBuddyListen(0); // PUPIL -> CONTROL   <<<<<< *******
                DelayWithDog(LONGER_DELAY);
                return;
            }
        }
        DelayWithDog(LONGER_DELAY);
        if (TakeControlBackNow) {
            DelayWithDog(LONGER_DELAY);
            StopBuddyListen(1); // MASTER RECLAIMS CONTROL   <<<<<< *******
        }
        LastPassivePacketTime = millis();
        FlushFifos();
    }
    else { // no packet arrived so maybe Pupil's dead, better grab control back
        if (IamMaster) {
            if (millis() - LastPassivePacketTime > INTERBUDDYRATE + 10) {
                StopBuddyListen(1); // MASTER RECLAIMS CONTROL   <<<<<< *******
            }
        }
        if (!IamMaster) {
            if (millis() - LastPassivePacketTime > INTERBUDDYRATE + 10) {
                MasterDetected(false);
                LastPassivePacketTime = millis();
            }
        }
    }
}
//*************************************************************************************************************************

void DoWirelessBuddy() // only called in when not in LISTENMODE
{
    static uint32_t InterBuddyTimer = 0;
    if (((millis() - LastPacketSentTime)) > 6) { // if there is time, communicate with other buddy tx
        {
            if ((millis() - InterBuddyTimer) >= INTERBUDDYRATE) {
                if (BuddyPupilOnWireless) SendSpecialPacket(0);
                if (BuddyMasterOnWireless) SendSpecialPacket(1);
                InterBuddyTimer = millis();
            }
        }
    }
}

//*************************************************************************************************************************
void StopBuddyListen(bool IamMaster) // here the transmitter takes control
{

    char     FrontView_Connected[] = "Connected";
    char     buddymsg[]            = "* WIRELESS BUDDY! *";
    char     MasterMsg[]           = "* WIRELESS MASTER! *";
    char     InVisible[]           = "vis Quality,0";
    uint64_t pip                   = TeensyMACAddPipe;
    if (BuddyPupilOnWireless) pip = BuddyMACAddPipe;
    Radio1.setPALevel(RF24_PA_MAX);
    Radio1.setDataRate(RF24_250KBPS);
    Radio1.enableAckPayload();
    Radio1.enableDynamicPayloads();   // needed
    Radio1.setAddressWidth(5);        // use 5 bytes for addresses
    Radio1.setCRCLength(RF24_CRC_16); // could be 8 or disabled
    Radio1.setAutoAck(true);          // we want acks
    Radio1.maskIRQ(1, 1, 1);          // no interrupts - seems NEEDED at the moment
    Radio1.openWritingPipe(pip);
    Radio1.stopListening();
    delayMicroseconds(SHORT_DELAY);
    ModelMatched = true;
    BoundFlag    = true;
    Connected    = true;
    GreenLedOn();
    CurrentMode = NORMAL;
    RestoreBrightness();

    if (IamMaster)
        SendText(FrontView_Connected, MasterMsg);
    else
        SendText(FrontView_Connected, buddymsg);
    SendCommand(InVisible);
    ShowConnectionQuality();
}
//*************************************************************************************************************************

void StartBuddyListen(bool IamMaster)
{
    char     FrontView_Connected[] = "Connected";
    char     buddymsg[]            = "(Wireless Buddy)";
    char     MasterMsg[]           = "(Wireless Master)";
    char     InVisible[]           = "vis Quality,0";
    uint64_t pip                   = TeensyMACAddPipe;
    if (BuddyPupilOnWireless) pip = BuddyMACAddPipe;
    char Ch_Lables[16][5] = {"Ch1", "Ch2", "Ch3", "Ch4", "Ch5", "Ch6", "Ch7", "Ch8", "Ch9", "Ch10", "Ch11", "Ch12", "Ch13", "Ch14", "Ch15", "Ch16"};
    Radio1.setPALevel(RF24_PA_MAX);
    Radio1.setDataRate(RF24_250KBPS);
    Radio1.enableAckPayload();        // needed
    Radio1.enableDynamicPayloads();   // needed
    Radio1.setAddressWidth(5);        // use 5 bytes for addresses
    Radio1.setCRCLength(RF24_CRC_16); // could be 8 or disabled
    Radio1.setChannel(SPECIAL_PACKET_CHANNEL);
    Radio1.setAutoAck(true); // we want acks
    Radio1.maskIRQ(1, 1, 1); // no interrupts - seems NEEDED at the moment
    Radio1.openReadingPipe(1, pip);
    delayMicroseconds(SHORT_DELAY);
    Radio1.startListening();
    ModelMatched = false;
    BoundFlag    = false;
    Connected    = false;
    FlushFifos();
    BlueLedOn();
    CurrentMode = LISTENMODE;
    for (int i = 0; i < CHANNELSUSED; ++i) {
        SendValue(Ch_Lables[i], 0);
        ShownBuffer[i] = 0;
    }
    if (IamMaster)
        SendText(FrontView_Connected, MasterMsg);
    else
        SendText(FrontView_Connected, buddymsg);

    SendCommand(InVisible);
}
#endif
