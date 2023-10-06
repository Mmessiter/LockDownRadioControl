// *************************************** BuddyWireless.h  *****************************************
#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef BUDDYWIRELESS_H
    #define BUDDYWIRELESS_H
//*************************************************************************************************************************

void SendSpecialPacket(bool IamMaster) // here the sender sends to other tx
{
    uint8_t DataPacket1[] = "SH"; // = Shut Up
    uint8_t DataPacket2[] = "OK"; // = OK to talk

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

void GetSpecialPacket(bool IamMaster)
{
}
#endif
