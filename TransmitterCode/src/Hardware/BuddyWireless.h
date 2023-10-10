// *************************************** BuddyWireless.h  *****************************************
#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef BUDDYWIRELESS_H
    #define BUDDYWIRELESS_H
    #define SPECIAL_PACKET_COUNT   3   // How many special packets to send
    #define SPECIAL_PACKET_CHANNEL 125 // Which channel for the special packets
    #define INTERBUDDYRATE         205 // 5 times a second (Fails below 200)
    #define SHORT_DELAY            200 // ... microseconds
    #define LONGER_DELAY           1   // ... milliseconds

//*************************************************************************************************************************

bool GetMasterAck() // Here Pupil gets Ack from master while Pupil is in control
{
    static bool    Master_is_Alive = false;
    static uint8_t ErrorCounter    = 0;
    char           AckSpecial[2]   = " "; // simple ack

    Radio1.read(&AckSpecial, 2);
    if (AckSpecial[0] == 'O') { // OK to continue sending. no action needed
        Master_is_Alive = true;
        ErrorCounter    = 0;
    }

    if (AckSpecial[0] == 'S') { // PUPIL -> LISTEN HEER <<<<<< *******
        Master_is_Alive = true;
        ErrorCounter    = 0;
        PlaySound(MASTERMSG);
        StartBuddyListen();
        DelayWithDog(LONGER_DELAY);
        FlushFifos();
        return Master_is_Alive;
    }
    if (ErrorCounter > SPECIAL_PACKET_COUNT) {
        Master_is_Alive = false;
        ErrorCounter    = 0;
    }
    FlushFifos();
    return Master_is_Alive;
}

//*************************************************************************************************************************

bool GetPupilAck() // Master gets Ack from pupil while MASTER in control
{
    static uint8_t FailureCounter = 0;
    static uint8_t SuccessCounter = 0;
    static bool    Pupil_is_Alive = false;
    uint8_t        AckSpecial[2]; // simple ack

    Radio1.read(&AckSpecial, 2);
    if (AckSpecial[0] == 'P') // ='P' pupil's only possible ack
    {
        FailureCounter = 0;
        ++SuccessCounter;
    }
    else { // bad ack
        ++FailureCounter;
        SuccessCounter = 0;
    }
    FlushFifos();
    if (FailureCounter > SPECIAL_PACKET_COUNT) {
        Pupil_is_Alive = false;
    }
    if (SuccessCounter == SPECIAL_PACKET_COUNT) {
        Pupil_is_Alive = true;
    }
    return Pupil_is_Alive;
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
                GetPupilAck(); // get ack from pupil - it's handled there
            }
        }
        if (BuddyON) //  PUPIL goes into CONTROL
        {
            if (Radio1.write(&Pupil_in_Control, sizeof(Pupil_in_Control))) { // SEND O to Pupil
                if (GetPupilAck()) {
                    StartBuddyListen(); // MASTER -> LISTEN  HEER <<<<<< *******
                    PlaySound(BUDDYMSG);
                    DelayWithDog(LONGER_DELAY);
                    LastPassivePacketTime = millis();
                    return;
                }
            }
        }
    }
    if (!IamMaster) {                                                // pupil area when in control *************************************************************
        if (Radio1.write(&Pupil_is_Alive, sizeof(Pupil_is_Alive))) { // send P to master
            delayMicroseconds(SHORT_DELAY);
            GetMasterAck();
        }
        else {                  // failed to send P to master while pupil in control
                                //  Look("Failed even to send a P to Master"); //  copout
            StartBuddyListen(); // <<<<<<<<<<<<<<<<<<< PUPIL -> LISTEN <<<<<< *******
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
            if (DataPacket[0] == Master_in_Control[0]) {
            } // no action needed
            if (DataPacket[0] == Pupil_in_Control[0]) {
                StopBuddyListen(); // PUPIL -> CONTROL  HEER <<<<<< *******

                DelayWithDog(LONGER_DELAY);
                return;
            }
        }
        DelayWithDog(LONGER_DELAY);
        FlushFifos();
        if (TakeControlBackNow) {
            DelayWithDog(LONGER_DELAY);
            StopBuddyListen(); // MASTER RECLAIMS CONTROL  HEER <<<<<< *******
        }
        LastPassivePacketTime = millis();
    }
    else { // no packet arrived  Pupil's dead!!
        if (IamMaster) {
            if (millis() - LastPassivePacketTime > INTERBUDDYRATE + 50) {
                StopBuddyListen(); // MASTER RECLAIMS CONTROL  HEER <<<<<< *******
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
void StopBuddyListen()
{
    uint64_t pip = TeensyMACAddPipe;
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
    for (int i = 0; i < CHANNELSUSED; ++i) ShownBuffer[i] = 1700; // to force a redraw
}
//*************************************************************************************************************************

void StartBuddyListen()
{
    uint64_t pip = TeensyMACAddPipe;
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
}
#endif
