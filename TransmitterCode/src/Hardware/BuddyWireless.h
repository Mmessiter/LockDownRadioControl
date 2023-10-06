// *************************************** BuddyWireless.h  *****************************************
#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef BUDDYWIRELESS_H
    #define BUDDYWIRELESS_H
//*************************************************************************************************************************

void SendSpecialPacket(bool IamMaster) // here the sender sends to other tx
{
    uint8_t DataPacket1[] = "SH"; // = Shut Up, send nothing
    uint8_t DataPacket2[] = "OK"; // = OK to send data

    Radio1.setChannel(SPECIAL_PACKET_CHANNEL);
    delayMicroseconds(50);
    Radio1.stopListening();
    delayMicroseconds(50);

    if (IamMaster && !BuddyON)
    {
        if (Radio1.write(&DataPacket1, sizeof(DataPacket1))) {
            Look("Sent SH to Buddy");
        }
        else {
            Look("Failed to send SH to Buddy");
        }
    }

    if (IamMaster && BuddyON)
    {
        if (Radio1.write(&DataPacket2, sizeof(DataPacket2))) {
            Look("Sent OK to Buddy");
        }
        else {
            Look("Failed to send OK to Buddy");
        }
    }

    Radio1.setChannel(CurrentChannel); // restore the proper frequency
    delayMicroseconds(50);
    Radio1.stopListening();
    delayMicroseconds(50);
}

//*************************************************************************************************************************

void StartBuddyListen()
{

    uint64_t pip = TeensyMACAddPipe;

    if (BuddyPupilOnWireless) pip = BuddyMACAddPipe;
    Radio1.setChannel(SPECIAL_PACKET_CHANNEL);
    delayMicroseconds(50);
    Radio1.openReadingPipe(1, pip);
    Radio1.startListening();
    delayMicroseconds(50);
    CurrentMode = LISTENMODE;
}
//*************************************************************************************************************************
void StopBuddyListen()
{
    Radio1.stopListening();
    CurrentMode = NORMAL;
}

//*************************************************************************************************************************

void GetSpecialPacket(bool IamMaster)
{

    uint8_t DataPacket[2];
    uint8_t DataPacket1[] = "SH"; // = Shut Up, send nothing
    uint8_t DataPacket2[] = "OK"; // = OK to send data

    if (Radio1.available()) {

        Radio1.writeAckPayload(1, &AckPayload, AckPayloadSize); // Send telemetry
        Radio1.read(&DataPacket, sizeof(DataPacket));
        if (DataPacket[0] == DataPacket1[0] && DataPacket[1] == DataPacket1[1]) {
            Look("Got SH from Master");
            BuddyON = false;
        }
        else if (DataPacket[0] == DataPacket2[0] && DataPacket[1] == DataPacket2[1]) {
            Look("Got OK from Master");
            BuddyON = true;
        }
        else {
            Look("Got unknown from Master");
        }
    }


}

//*************************************************************************************************************************

#endif
