// *************************************** BuddyWireless.h  *****************************************
#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef BUDDYWIRELESS_H
    #define BUDDYWIRELESS_H
    #define SPECIAL_PACKET_COUNT   3   // How many special packets to send
    #define SPECIAL_PACKET_CHANNEL 124 // Which channel for the special packets
    #define SHORT_DELAY            150


//*************************************************************************************************************************

bool GetMasterAck(){ // Pupil gets Ack from master while in control 
    static bool Master_is_Alive = false;






    return Master_is_Alive;   
}

//*************************************************************************************************************************

bool GetPupilAck() // Master gets Ack from pupil while in control
{
    static uint8_t FailureCounter = 0;
    static uint8_t SuccessCounter = 0;
    static bool    Pupil_is_Alive = false;

    uint8_t AckSpecial[2]; // simple ack

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
        Look1(millis());
        Look(" Pupil is sending wierd acknowledgements");
        Pupil_is_Alive = false;
        FailureCounter = 0;
        Look(AckSpecial[0]);
    }
    if (SuccessCounter == SPECIAL_PACKET_COUNT) {
        Look1(millis());
        Look("  Pupil is alive and well");
        Pupil_is_Alive = true;
        FailureCounter = 0;
        SuccessCounter = 0;
    }
    return Pupil_is_Alive;
}
//*************************************************************************************************************************

void SendSpecialPacket(bool IamMaster) // here the sender sends to other tx
{
    uint8_t Master_in_Control[2] = "S"; // = Shut Up, send nothing
    uint8_t Pupil_in_Control[2]  = "O"; // = OK to send data
    uint8_t Pupil_is_Alive[]     = "P";

    Radio1.setChannel(SPECIAL_PACKET_CHANNEL);
    delayMicroseconds(SHORT_DELAY);
    Radio1.stopListening();
    delayMicroseconds(SHORT_DELAY);

    if (IamMaster) {
        if (!BuddyON) // MASTER IN CONTROL
        {
            if (Radio1.write(&Master_in_Control, sizeof(Master_in_Control))) {
                if (!GetPupilAck()) {
                    Look("Failed to get Pupil S Ack");
                }
            }
            else {
                Look("Failed even to send an S to Buddy");
            }
        }
        if (BuddyON) // PUPIL IN CONTROL - TAKE OVER NOW ******************************************************
        {
            if (Radio1.write(&Pupil_in_Control, sizeof(Pupil_in_Control))) {
                if (!GetPupilAck()) {
                    Look("Failed to get Pupil O Ack");
                }
            }
            else {
                Look("Failed even to send O to Buddy");
            }
        }
    }

    if (!IamMaster) { // pupil area when in control **************************************************************

        if (Radio1.write(&Pupil_is_Alive, sizeof(Pupil_is_Alive))) {
            if (!GetMasterAck()) {
                Look("Failed to get Master P Ack");
            }
        }
        else {
            Look("Failed even to send an P to Master");
        }
    }

    Radio1.setChannel(CurrentChannel); // restore the proper frequency
    delayMicroseconds(SHORT_DELAY);
    Radio1.stopListening();
    delayMicroseconds(SHORT_DELAY);
}

//*************************************************************************************************************************

void StartBuddyListen()
{
    uint64_t pip = TeensyMACAddPipe;
    if (BuddyPupilOnWireless) pip = BuddyMACAddPipe;
    Radio1.setPALevel(RF24_PA_MAX);
    Radio1.setDataRate(RF24_250KBPS);
    Radio1.enableAckPayload();                // needed
    Radio1.setRetries(RETRYWAIT, RETRYCOUNT); // automatic retries
    Radio1.enableDynamicPayloads();           // needed
    Radio1.setAddressWidth(5);                // use 5 bytes for addresses
    Radio1.setCRCLength(RF24_CRC_16);         // could be 8 or disabled
    Radio1.setAutoAck(true);                  // we want acks
    Radio1.maskIRQ(1, 1, 1);                  // no interrupts - seems NEEDED at the moment
    Radio1.openReadingPipe(1, pip);
    Radio1.setChannel(SPECIAL_PACKET_CHANNEL);
    delayMicroseconds(SHORT_DELAY);
    Radio1.startListening();
    CurrentMode = LISTENMODE;
}
//*************************************************************************************************************************
void StopBuddyListen()
{
    Radio1.stopListening();
    delayMicroseconds(SHORT_DELAY);
    CurrentMode = NORMAL;
}

//*************************************************************************************************************************

void GetSpecialPacket(bool IamMaster) // here the passive tx gets from active tx
{
    uint8_t DataPacket[2]        = " ";
    uint8_t Master_in_Control[2] = "S"; // = Shut Up, send nothing
    uint8_t Pupil_in_Control[2]  = "O"; // = OK to send data
    uint8_t Ack[]                = "P"; // Pupil ack

    if (IamMaster) { // if I am not master, I am pupil and Ack is already set to 'P'
        if (BuddyON) {
            Ack[0] = 'O'; // ok to send - you're in charge
        }
        else
        {
            Ack[0] = 'S'; // shut up as buddy is not on
        }
    }
    if (Radio1.available()) {
        delayMicroseconds(SHORT_DELAY);
        Radio1.writeAckPayload(1, &Ack, 2); // Acknowledge the packet
        delayMicroseconds(SHORT_DELAY);
        Radio1.read(&DataPacket, sizeof(DataPacket));

        if (!IamMaster) { // pupil here
            if (DataPacket[0] == Master_in_Control[0]) {
                Look("Got S from Master");
                BuddyON = false;
            }
            else if (DataPacket[0] == Pupil_in_Control[0]) {
                Look("Got O from Master");
                BuddyON = true;
            }
        }
        if (IamMaster) { // master here
            if (DataPacket[0] == Ack[0]) {
                Look("Phew ... Got a P from a happy Pupil");
            }
            else {
                Look1("Got rubbish from unhappy Pupil: ");
                Look(DataPacket[0]);
            }
        }
        DelayWithDog(3);
        FlushFifos();
    }
}

//*************************************************************************************************************************

void DoWirelessBuddy()
{
    static uint32_t InterBuddyTimer = 0;
    if (((millis() - LastPacketSentTime)) > 6) { // if there is time, communicate with other buddy tx
        {
            if ((millis() - InterBuddyTimer) >= 250) { // 4 times a second
                if (BuddyPupilOnWireless && SlaveHasControl) SendSpecialPacket(0);
                if (ModelMatched && BoundFlag && BuddyMasterOnWireless && !SlaveHasControl) SendSpecialPacket(1);
                InterBuddyTimer = millis();
            }
        }
    }
}

//*************************************************************************************************************************

#endif
