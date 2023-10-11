// *************************************** BuddyWireless.h  *****************************************
#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef BUDDYWIRELESS_H
    #define BUDDYWIRELESS_H
    #define SPECIAL_PACKET_CHANNEL 125 // Which channel for the special packets
    #define INTERBUDDYRATE         205 // 5 times a second (Fails below 200)
    #define SHORT_DELAY            200 // ... microseconds
    #define LONGER_DELAY           1   // ... milliseconds

//*************************************************************************************************************************

void GetMasterAck() // Here Pupil gets Ack from master while Pupil is in control
{
    char AckSpecial[2] = " "; // simple ack
    Radio1.read(&AckSpecial, 2);
    if (AckSpecial[0] == 'O') return; // OK to continue sending. no action needed
    if (AckSpecial[0] == 'S') {       // PUPIL -> LISTEN HEER <<<<<< *******
        PlaySound(MASTERMSG);
        StartBuddyListen(0);
        DelayWithDog(LONGER_DELAY);
    }
    FlushFifos();
    return;
}

//*************************************************************************************************************************

void GetPupilAck() // Master gets Ack from pupil while MASTER in control
{
    uint8_t AckSpecial[2];       // simple ack
    Radio1.read(&AckSpecial, 2); // don't care what it is
    FlushFifos();
    return;
}
//*************************************************************************************************************************

void SendSpecialPacket(bool IamMaster) // here the sender sends to other tx
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
        }
        if (BuddyON) //  PUPIL goes into CONTROL
        {
            if (Radio1.write(&Pupil_in_Control, sizeof(Pupil_in_Control))) { // SEND O to Pupil. If we can't, he's dead
                GetPupilAck();
                StartBuddyListen(1); // MASTER -> LISTEN  (Pupil took control) HEER <<<<<< *******
                PlaySound(BUDDYMSG);
                DelayWithDog(LONGER_DELAY);
                LastPassivePacketTime = millis();
                return;
            }
        }
    }
    if (!IamMaster) {                                                // pupil area when in control *************************************************************
        if (Radio1.write(&Pupil_is_Alive, sizeof(Pupil_is_Alive))) { // send P to master
            delayMicroseconds(SHORT_DELAY);
            GetMasterAck();
           // Look("OK!");
        }
        else {
            StartBuddyListen(0);  // <<<<<<<<<<<<<<<<<<< FAILSAFE PUPIL -> LISTEN <<<<<< *******
            PlaySound(MASTERMSG); // master must be back in control so pupil must shut up
            DelayWithDog(LONGER_DELAY);
            FlushFifos();
          //  Look("Failsafed back to Master");
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
            if (DataPacket[0] == Master_in_Control[0]) {
            } // no action needed
            if (DataPacket[0] == Pupil_in_Control[0]) {
                StopBuddyListen(0); // PUPIL -> CONTROL  HEER <<<<<< *******
                DelayWithDog(LONGER_DELAY);
                return;
            }
        }
        DelayWithDog(LONGER_DELAY);
        if (TakeControlBackNow) {
            DelayWithDog(LONGER_DELAY);
            StopBuddyListen(1); // MASTER RECLAIMS CONTROL  HEER <<<<<< *******
        }
        LastPassivePacketTime = millis();
        FlushFifos();
    }
    else { // no packet arrived  Pupil's dead!!
        if (IamMaster) {
            if (millis() - LastPassivePacketTime > INTERBUDDYRATE + 50) {
                StopBuddyListen(1); // MASTER RECLAIMS CONTROL  HEER <<<<<< *******
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
