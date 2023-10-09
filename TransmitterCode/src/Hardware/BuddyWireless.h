// *************************************** BuddyWireless.h  *****************************************
#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef BUDDYWIRELESS_H
    #define BUDDYWIRELESS_H
    #define SPECIAL_PACKET_COUNT   3   // How many special packets to send
    #define SPECIAL_PACKET_CHANNEL 125 // Which channel for the special packets
    #define INTERBUDDYRATE         200 // 5 times a second (Fails below 200)
    #define SHORT_DELAY            200 // ... microseconds

//*************************************************************************************************************************

bool GetMasterAck() // Here Pupil gets Ack from master while Pupil is in control
{
    static bool    Master_is_Alive = false;
    static uint8_t ErrorCounter    = 0;
    char AckSpecial[2] = " "; // simple ack

    Radio1.read(&AckSpecial, 2);

    if (AckSpecial[0] == 'O') { // OK to continue sending. no action needed
        Master_is_Alive = true;
        ErrorCounter    = 0;
    }

    if (AckSpecial[0] == 'S') { // PUPIL -> LISTEN HEER <<<<<< *******
        Master_is_Alive = true;
        ErrorCounter    = 0;
        Look("Master is telling us to shut up");
        StartBuddyListen(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        DelayWithDog(50);
        FlushFifos();
        return Master_is_Alive;
    }

    if (ErrorCounter > SPECIAL_PACKET_COUNT) {
        Look1(millis());
        Look(" Master is sending wierd acknowledgements");
        Master_is_Alive = false;
        ErrorCounter    = 0;
        Look(AckSpecial[0]);
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
    uint8_t Pupil_is_Alive[2]    = "P";

    FlushFifos();
    Radio1.setChannel(SPECIAL_PACKET_CHANNEL);
    delayMicroseconds(SHORT_DELAY);
    Radio1.stopListening();
    delayMicroseconds(SHORT_DELAY);

    if (IamMaster) {
        if (!BuddyON) // MASTER stays IN CONTROL OR RETAKES CONTROL!!! <--- todo
        {
            if (Radio1.write(&Master_in_Control, sizeof(Master_in_Control))) {
                if (!GetPupilAck()) Look("Failed to get Pupil S Ack");
            }
            else {
                Look("Pupil is not responding to S");
            }
        }
        if (BuddyON) //  PUPIL goes into CONTROL
        {
            if (Radio1.write(&Pupil_in_Control, sizeof(Pupil_in_Control))) { // SEND O to Pupil
                if (!GetPupilAck()) {
                    Look("Failed to get Pupil O Ack");
                }
                if (GetPupilAck()) {
                    Look("Got Pupil O Ack - GIVING CONTROL TO PUPIL");
                    StartBuddyListen(); // MASTER -> LISTEN  HEER <<<<<< *******
                    DelayWithDog(50);
                    return;
                }
            }
            else {
                Look("Pupil is not responding to O");
            }
        }
    }

    if (!IamMaster) {                                                // pupil area when in control **************************************************************
        if (Radio1.write(&Pupil_is_Alive, sizeof(Pupil_is_Alive))) { // send P to master
            if (GetMasterAck()) {
             //   Look("Master saw our P");
            }
            if (!GetMasterAck()) {
                Look("Failed to get Master properly to Acknowledge our P");
            }
        }
        else {                                         // failed to send P to master while pupil in control
            Look("Failed even to send a P to Master"); // HEER is a copout
            StartBuddyListen();                        // <<<<<<<<<<<<<<<<<<< PUPIL -> LISTEN HEER <<<<<< *******
            DelayWithDog(50);
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

void StartBuddyListen()
{
    uint64_t pip = TeensyMACAddPipe;
    if (BuddyPupilOnWireless) pip = BuddyMACAddPipe;
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
    CurrentMode  = LISTENMODE;
    FlushFifos();
    BlueLedOn();
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
}

//*************************************************************************************************************************

void GetSpecialPacket(bool IamMaster) // here the passive tx gets from active tx
{
    char DataPacket[2]        = " ";
    char Master_in_Control[2] = "S"; // = Shut Up, send nothing
    char Pupil_in_Control[2]  = "O"; // = OK to send data
    char Ack[2]               = "P"; // Pupil ack

    if (Radio1.available()) {
        if (IamMaster) { // master here
            if (BuddyON)
            {
                Ack[0] = 'O'; // ok to continue sending - you're in charge
                Look("Sending ACK O to Pupil");
            }
            if (!BuddyON)
            {
                Ack[0] = 'S'; // shut up now as buddy is now off.... *********
                Look("Sending ACK S to Pupil");
            }
        }
        delayMicroseconds(SHORT_DELAY);
        Radio1.writeAckPayload(1, &Ack, 2); // Acknowledge the packet
        delayMicroseconds(SHORT_DELAY);
        Radio1.read(&DataPacket, sizeof(DataPacket));

        if (!IamMaster) { // pupil here
            if (DataPacket[0] == Master_in_Control[0]) {
                 // Look("Got S from Master so will shut up");
            }
            if (DataPacket[0] == Pupil_in_Control[0]) {
                //  Look("Got O from Master");
                StopBuddyListen(); // PUPIL -> CONTROL  HEER <<<<<< *******
                DelayWithDog(50);
                return;
            }
        }

        if (IamMaster) { // master here
            if (DataPacket[0] == 'P') {
                //  Look1(millis());
                //  Look(" Pupil is alive");
            }
            if (!BuddyON)
            {
                DelayWithDog(50);
                StopBuddyListen(); // MASTER RECLAIMS CONTROL  HEER <<<<<< *******
               // DelayWithDog(50);
            }
        }
        DelayWithDog(5);
        FlushFifos();
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

#endif
