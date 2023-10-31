// *************************************** BuddyWireless.h  *****************************************

// This is the code for the wireless buddy system. Its functions are called from the main loop, and from the SendData() function.

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef BUDDYWIRELESS_H
    #define BUDDYWIRELESS_H
    #define SPECIAL_PACKET_CHANNEL 123              // Which channel for the special packets
    #define SHORT_DELAY            130              // ... microseconds
    #define LONGER_DELAY           1                // ... milliseconds
    #define LOSTCONTACTTHRESHOLD   2                // 2 fails in a row and we declare the buddy dead
    #define INTERBUDDYRATE         10               // 100 times a second 
    #define DELAYAFTERACK          1                // ms
    #define ENCRYPT_KEY            0xFEADFEADBB     // The encryption key used for the Pipe address between transmitters :-)
    #define FASTDATARATE           RF24_2MBPS       // 2MBPS
    #define NORMALDATARATE         RF24_250KBPS     // 250KBPS
         
//*************************************************************************************************************************
void GetSlaveChannelValuesWireless(){                                           // Very Like the PPM function only a bit simpler

        if (PupilIsAlive == 2) BuddyON = false;                                 // If pupil is dead, then Buddy is off
        if (BuddyON) {
        for (int j = 0; j < CHANNELSUSED; ++j) {                                // While slave has control, his stick data replaces some of ours
            if (BuddyControlled & 1 << (j)) SendBuffer[j] = BuddyBuffer[j];     // Test if this channel is buddy controlled. If not leave it unchanged
        }

        if (!SlaveHasControl) { // Buddy is now On
            PlaySound(BUDDYMSG);                                                // Announce the Buddy is now in control
            LastShowTime    = 0;
            SlaveHasControl = true;
        }
    }
    else { // Buddy is now Off
        if (SlaveHasControl) {
            PlaySound(MASTERMSG);                                               // Announce the Master is now in control
            LastShowTime    = 0;
            SlaveHasControl = false;
        }
    }
}
//*************************************************************************************************************************
// This function is called by Pupil and the Master was Detected - or not Detected.
// After 6 failed detections, the Master is declared dead, and a message is changed on front view.
void MasterDetected(bool Detected)
{
    static uint16_t LostMasterCount = 0;
    char            Mlost[]         = "Master not found";
    char            Mfound[]        = "Master found!";
    char            wb[]            = "wb"; // wb is the name of the label on front view
    char            YesVisible[]    = "vis wb,1";

    if (Detected) {
        LostMasterCount = 0;
        LastPassivePacketTime = millis(); // reset the timer
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
// After 6 failed detections, the Pupil is declared dead, and a message is changed on front view.
void PupilDetected(bool Detected)
{
    static uint16_t LostPupilCount = 0;
    char            Mlost[]        = "Buddy not found";
    char            Mfound[]       = "Buddy found!";
    char            wb[]           = "wb"; // wb is the name of the label on front view
    char            YesVisible[]   = "vis wb,1";
    if (Detected) {
        LostPupilCount = 0;
        LastPassivePacketTime = millis(); // reset the timer
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
void GetPupilAck()                                              // Master gets Ack from pupil which contains all the pupil's control data !! << ************** !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
{
    uint16_t AckSpecial[COMPRESSEDWORDS];                       // Longer ack contains all the pupil's control data
    if (Radio1.available()) {
        Radio1.read(&AckSpecial, sizeof AckSpecial);
        Decompress(BuddyBuffer, AckSpecial, UNCOMPRESSEDWORDS); // decompress the data
    }
}
//*************************************************************************************************************************

void SendSpecialPacket() // here the MASTER sends to PUPIL tx. This is called about 100 times a second.
{
    char AnyData[] = "M";                                       // Send M to indicate Master is ON
    if (BuddyON) AnyData[0] = 'B';                              // Send B to indicate Buddy is ON
    Radio1.openWritingPipe(TeensyMACAddPipe ^ ENCRYPT_KEY);     // send to encrypted pipe address
    Radio1.setDataRate(FASTDATARATE);                           //  2MBPS
    Radio1.setChannel(SPECIAL_PACKET_CHANNEL);
    if (Radio1.write(&AnyData, sizeof(AnyData))) {
        GetPupilAck();                                          // get ack from pupil WITH HIS CONTROL DATA!!
        PupilDetected(true); 
    } else {
        PupilDetected(false);                                   // pupil is dead
    }
    Radio1.setDataRate(NORMALDATARATE);                         // restore the proper data rate
    Radio1.openWritingPipe(TeensyMACAddPipe);                   // restore the proper pipe address
    Radio1.setChannel(CurrentChannel);                          // restore the proper frequency ....stop listening ???
}

//*************************************************************************************************************************

void GetSpecialPacket()                                                               // here the PUPIL tx gets from MASTER tx. This function is called from main loop
{
    static bool MasterIsInControl = true;
    char DataPacket[2];
    if (Radio1.available()) {
        Radio1.writeAckPayload(1, &CompressedData, sizeof CompressedData);              // Acknowledge the packet BY SENDING MY CHANNEL DATA!
        DelayWithDog(DELAYAFTERACK);                                                    // <-  ** MUST ** allow the ACK time to get going, otherwise the sender sees a failed packet      <<<<<<<<<<<<<< ************** !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        Radio1.read(&DataPacket, sizeof(DataPacket));                                   // read the packet
        if ((DataPacket[0] == 'B') && (MasterIsInControl)) {                            // Buddy is now in control
            MasterIsInControl = false;
            PlaySound(BUDDYMSG);
        }
        if ((DataPacket[0] == 'M') && (!MasterIsInControl)) {                           // Master is now in control
            MasterIsInControl = true;
            PlaySound(MASTERMSG);
        }
        MasterDetected(true);
        LastPassivePacketTime = millis();
        FlushFifos();
    }
    else { // no packet arrived so maybe master's dead
        if (millis() - LastPassivePacketTime > 700) {
            MasterDetected(false);
            LastPassivePacketTime = millis();
        }
    }
}
//*************************************************************************************************************************
void StartBuddyListen(bool IamMaster)
{
    Radio1.setDataRate(FASTDATARATE);
    Radio1.enableAckPayload();                  // needed
    Radio1.enableDynamicPayloads();             // needed
    Radio1.setAutoAck(true);                    // we want acks
    Radio1.maskIRQ(1, 1, 1);                    // no interrupts - seems NEEDED at the moment
    Radio1.openReadingPipe(1, BuddyMACAddPipe ^  ENCRYPT_KEY);
    delayMicroseconds(SHORT_DELAY);
    Radio1.setChannel(SPECIAL_PACKET_CHANNEL);
    Radio1.startListening();
    FlushFifos();
    BlueLedOn();
    CurrentMode = LISTENMODE;
    LostContactFlag = false;
    RestoreBrightness();
}
//************************************************************************************************************************
#endif
