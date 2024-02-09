// *************************************** BuddyWireless.h  *****************************************

// This is the code for the wireless buddy system. Its functions are called from the main loop, and from the SendData() function.

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef BUDDYWIRELESS_H
    #define BUDDYWIRELESS_H

    #define LOSTCONTACTTHRESHOLD   12                                           // 12 fails in a row and we declare the buddy or master dead
    #define ENCRYPT_KEY            0xFEADFEADBB                                 // The encryption key is used for the Pipe address between the transmitters 
   
//*************************************************************************************************************************

void LoadCorrectModel(uint64_t ModelID){                                        //  Gets Buddy onto same model as master, quietly.                                            
    uint32_t SavedModelNumber = ModelNumber;                                    //  Save the current model number
    ModelMatched = false;
    ModelNumber = 0;
    while (!ModelMatched && (ModelNumber < MAXMODELNUMBER - 1)) {               //  Try to match the ID with a saved one
        ++ModelNumber;
        ReadOneModel(ModelNumber);
        if ((ModelID == ModelsMacUnionSaved.Val64)) ModelMatched = true;        //  Found it!
    }
    if (ModelMatched) {                                                         //  Found it!
        UpdateModelsNameEveryWhere();                                           //  Show it everywhere.
        SaveAllParameters();                                                    //  Save it
        GotoFrontView();
    }else{
        ModelNumber = SavedModelNumber;                                         //  Restore the current model number
        ReadOneModel(ModelNumber);                                              //  Restore the current model 
        GotoFrontView();
    }
}

//*************************************************************************************************************************
void GetSlaveChannelValuesWireless(){                                           // Very Like the PPM function only a bit simpler

        if (PupilIsAlive == 2) BuddyON = false;                                 // If pupil is dead, then Buddy is off
        if (BuddyON) {
        for (int j = 0; j < CHANNELSUSED; ++j) {                                // While slave has control, his stick data replaces some of ours
           if (BuddyControlled & (1 << j)) SendBuffer[j] = BuddyBuffer[j];      // Test if this channel is buddy controlled. If not leave it unchanged otherwise replace it with the buddy's value
        }
        if (!SlaveHasControl) {                                                 // Buddy has turned On
            PlaySound(BUDDYMSG);                                                // Announce the Buddy is now in control
            SlaveHasControl = true;
        }
    }
    else {                                                                      
        if (SlaveHasControl) {                                                  // Buddy has turned Off
            PlaySound(MASTERMSG);                                               // Announce the Master is now in control
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
        LastPassivePacketTime = millis();                                   // reset the timer
        if (MasterIsAlive != 1) {                                           // MasterIsAlive = 1 means Master was already alive
            SendText(wb, Mfound);
            SendCommand(YesVisible);
            MasterIsAlive = 1;
        }
    }
    else {
        ++LostMasterCount;
        if (LostMasterCount > LOSTCONTACTTHRESHOLD) {
            if (MasterIsAlive != 2) {                                        // MasterIsAlive = 2 means Master was already dead
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
// Master gets Ack from pupil. 
// Pupil's Ack Payload contains the pupil's control data (in BuddyBuffer[]) which is compressed and must be decompressed before use.
// This function is called from SendSpecialPacket() which is called from the main loop.
// GetSlaveChannelValuesWireless() then uses the BuddyBuffer data to replace some or all of the Master's control data. 
void GetPupilAck()                                    
{
    uint16_t AckSpecial[COMPRESSEDWORDS];                                   // Ack payload will contain all the pupil's control data
    if (Radio1.available()) {                                               // if a packet has arrived
        Radio1.read(&AckSpecial, sizeof AckSpecial);                        // read the packet
        Decompress(BuddyBuffer, AckSpecial, UNCOMPRESSEDWORDS);             // decompress the data into buddybuffer array
    }
}

//*************************************************************************************************************************
void ChangeTXTarget(uint8_t ch,uint64_t p, rf24_datarate_e speed)           // Swap between Buddy and Master
{                                                                           // ch is the channel, p is the pipe address, speed is the data rate
     Radio1.setDataRate(speed);                                             // Set the data rate
     Radio1.openWritingPipe(p);                                             // Set the pipe address
     Radio1.stopListening();
     delayMicroseconds(STOPLISTENINGDELAY);
     Radio1.setChannel(ch);                                                 // Set the frequency channel
     delayMicroseconds(STOPLISTENINGDELAY);
}

//*************************************************************************************************************************
void SendSpecialPacket()                                                    // Here the MASTER sends to PUPIL tx. This is called about 200 times a second.
{                                                                           // Master Sends M or B to indicate whether Buddy is on or off, and the ID of the model which should be loaded.
                                                                            // The Pupil's Ack Payload contains the pupil's control data (in BuddyBuffer[]) which is compressed and must be decompressed before use.
                                                                            // This function is called from SendData() function which is called from the main loop.
                                                                            // GetSlaveChannelValuesWireless() then uses the BuddyBuffer data to replace some or all of the Master's control data. 
                                                                            // Because the datarate is 2 meg the exchange is very fast.
    static uint32_t LocalTimer          = 0;
    static bool     NeedToRecover       = false;
    static uint8_t  ChannelSentLastTime = 0;                                // The old channel number
    static uint8_t  Index               = 82;                               // The current channel number
    struct spd
    {
        char        Command[2];
        uint64_t    ModelID;
        uint8_t     Channel = QUIETCHANNEL;
    };
    static spd SpecialPacketData;

    rf24_datarate_e faster = FASTDATARATE, slower = DATARATE;               // Save the data rates
    if ((!BoundFlag || !ModelMatched  || (PupilIsAlive != 1))){        // Don't send too often if not connected, nor if buddy was not found
        if (((millis() - LocalTimer) < 20)) return;    
            LocalTimer = millis();
    }
    SpecialPacketData.ModelID =  ModelsMacUnionSaved.Val64;                 // Send the model ID so that pupil can check it
    SpecialPacketData.Command[0]                    = 'M';                  // Send M to indicate Master is ON
    if (BuddyON)       SpecialPacketData.Command[0] = 'B';                  // Send B to indicate Buddy is ON
    ChannelSentLastTime = SpecialPacketData.Channel;                        // Use the old channel number because Buddy hasn't yet hopped
    --Index; if (Index < 1) Index = 82;                                     // use the same array but in reverse order
    SpecialPacketData.Channel = FHSS_data::FHSS_Channels[Index];            // Set the  new channel number for next time
    if (NeedToRecover) SpecialPacketData.Channel = QUIETCHANNEL;            // If contact lost, then use the recovery channel to recover
    ChangeTXTarget(ChannelSentLastTime,TeensyMACAddPipe ^ ENCRYPT_KEY,faster); // Set the TX target to the Buddy
    if (Radio1.write(&SpecialPacketData, sizeof SpecialPacketData)) {       // Send the packet
        GetPupilAck();                                                      // Get ack from pupil WITH HIS CONTROL DATA!!
        PupilDetected(true);                                                // Pupil is alive
        NeedToRecover = false;                                              // No need to recover             
    } else {
        PupilDetected(false);                                               // Pupil is dead
        NeedToRecover = true;                                               // Need to recover                 
    }
    ChangeTXTarget(CurrentChannel,TeensyMACAddPipe,slower);                 // Set the TX target back to the receiver in model.
}

//*************************************************************************************************************************
void SendSpecialPacketFromPPMModule()                                       // Here the (PPM!) MASTER sends to PUPIL tx. This is called about 200 times a second.
{                                                                           // Master Sends M or B to indicate whether Buddy is on or off, and the ID of the model which should be loaded.
                                                                            // The Pupil's Ack Payload contains the pupil's control data (in BuddyBuffer[]) which is compressed and must be decompressed before use.
                                                                            // This function is called from SendData() function which is called from the main loop.
                                                                            // GetSlaveChannelValuesWireless() then uses the BuddyBuffer data to replace some or all of the Master's control data. 
                                                                            // Because the datarate is 2 meg the exchange is very fast.
                                                                            // This uses the nRF24L01 while the model is connected via TX module. Only the Buddy is using the nRF24L01 in here
    static uint32_t LocalTimer          = 0;
    static bool     NeedToRecover       = false;
    static uint8_t  ChannelSentLastTime = 0;                                // The old channel number
    static uint8_t  Index               = 82;                               // The current channel number
    struct spd
    {
        char        Command[2];
        uint64_t    ModelID;
        uint8_t     Channel = QUIETCHANNEL;
    };
    static spd SpecialPacketData;
    if  (PupilIsAlive != 1){                                                 // Don't send too often if buddy was not found
        if (((millis() - LocalTimer) < 20)) return;    
            LocalTimer = millis();
    }
    
    SpecialPacketData.ModelID =  ModelsMacUnionSaved.Val64;                 // Send the model ID so that pupil can check it
    SpecialPacketData.Command[0]                    = 'M';                  // Send M to indicate Master is ON
    if (BuddyON)       SpecialPacketData.Command[0] = 'B';                  // Send B to indicate Buddy is ON
    ChannelSentLastTime = SpecialPacketData.Channel;                        // Use the old channel number because Buddy hasn't yet hopped
    --Index; if (Index < 1) Index = 82;                                     // use the same array but in reverse order
    SpecialPacketData.Channel = FHSS_data::FHSS_Channels[Index];            // Set the  new channel number for next time
    if (NeedToRecover) SpecialPacketData.Channel = QUIETCHANNEL;            // If contact lost, then use the recovery channel to recover
    Radio1.stopListening();
    delayMicroseconds(STOPLISTENINGDELAY);
    Radio1.setChannel(ChannelSentLastTime);
    delayMicroseconds(STOPLISTENINGDELAY);
    if (Radio1.write(&SpecialPacketData, sizeof SpecialPacketData)) {       // Send the packet
        GetPupilAck();                                                      // Get ack from pupil WITH HIS CONTROL DATA!!
        PupilDetected(true);                                                // Pupil is alive
        NeedToRecover = false;                                              // No need to recover             
    } else {
        PupilDetected(false);                                               // Pupil is dead
        NeedToRecover = true;                                               // Need to recover                 
    }
}

//*************************************************************************************************************************
void SetUpTargetForBuddy()                                                           // Once only because the model is connected via module. Only the Buddy is using the nRF24L01 in here
{
        Radio1.setDataRate(FASTDATARATE);                                            // Set the data rate
        delayMicroseconds(STOPLISTENINGDELAY);
        Radio1.openWritingPipe(TeensyMACAddPipe ^ ENCRYPT_KEY);                      // Set the pipe address
        delayMicroseconds(STOPLISTENINGDELAY);
        Radio1.stopListening();
        delayMicroseconds(STOPLISTENINGDELAY);
}

//*************************************************************************************************************************
void GetSpecialPacket()                                                                 // Here the PUPIL tx gets a packet from MASTER tx. This function is called from main loop very fast.
{                                                                                       // Master tells Pupil whether buddy is on or off, and the ID of the model which should be loaded.
                                                                                        // The Pupil's Ack Payload contains the pupil's control data (in BuddyBuffer[]) which is compressed and must be decompressed before use.
                                                                                        // GetSlaveChannelValuesWireless() in Master then uses the BuddyBuffer data to replace some or all of the Master's control data. 
                                                                                        // Because the datarate is 2 meg the exchange is very fast. 
    static bool MasterIsInControl = true; 
    static uint16_t PacketCounter = 0;
    static uint32_t LocalTimer = 0;
    static uint32_t LastModelID = 0;
    struct spd {
        char        Command[2];
        uint64_t    ModelID = 0;
        uint8_t     Channel = QUIETCHANNEL;

    };
    spd SpecialPacketData;
    Compress(DataTosend.CompressedData, SendBuffer, UNCOMPRESSEDWORDS);                 // Compress 
    if (Radio1.available()) {                                                           // if a packet has arrived
        Radio1.writeAckPayload(1, &DataTosend.CompressedData, SizeOfCompressedData);    // Acknowledge the packet BY SENDING MY CHANNEL DATA!
        delay(1);                                                                       // <-  *MUST* allow the ACK time to get going, otherwise the sender sees a failed packet          
        Radio1.read(&SpecialPacketData, sizeof SpecialPacketData);                      // read the packet if its still there
        if ((SpecialPacketData.Command[0] == 'B') && (MasterIsInControl)) {             // Buddy is now in control
            MasterIsInControl = false;                                                  // Buddy is now in control
            PlaySound(BUDDYMSG);                                                        // Announce the Buddy is now in control
        }
        if ((SpecialPacketData.Command[0] == 'M') && (!MasterIsInControl)) {            // Master is now in control
            MasterIsInControl = true;                                                   // Master is now in control
            PlaySound(MASTERMSG);                                                       // Announce the Master is now in control
        }
        if (SpecialPacketData.ModelID != LastModelID) {                                 // if the model ID has changed, match it if we can
            LastModelID = SpecialPacketData.ModelID;                                    // Save the model ID so we can know if it changed
            if (SpecialPacketData.ModelID != ModelsMacUnionSaved.Val64){                // is it the currently loaded model?
                LoadCorrectModel(SpecialPacketData.ModelID);                            // if not then try only once to load the correct model 
            }   
        }
        MasterDetected(true);                                                           // Master is alive
        ++PacketCounter;                                                                // Count the packets
        if (millis() - LocalTimer > 1000) {                                             // Every second
            LocalTimer = millis();                                                      // reset the timer
            PacketsPerSecond = PacketCounter;                                           // save the packets per second for use on the data diplay
            PacketCounter = 0;                                                          // reset the counter
        }
        LastPassivePacketTime = millis();                                               // reset the timer
        Radio1.stopListening(); 
        Radio1.setChannel(SpecialPacketData.Channel);                                   // Set the frequency channel
        Radio1.startListening();                                                        // Start listening
    }else{                                                                              // No packet arrived so maybe master's dead? 
        if (millis() - LastPassivePacketTime > PACEMAKER) {                             // We expect a packet every PACEMAKER ( = 5 ) milliseconds. If none, a packet was lost.
            Radio1.stopListening(); 
            Radio1.setChannel(QUIETCHANNEL);                                            // Set the recovery channel         
            Radio1.startListening();
            MasterDetected(false);                                
            LastPassivePacketTime = millis();                                           // Reset the timer
            ++TotalLostPackets;                                                         // Count the lost packets
        }
    }
}

//*************************************************************************************************************************
void StartBuddyListen()
{
    Radio1.setDataRate(FASTDATARATE);           // 2 MBPS
    Radio1.enableAckPayload();                  // needed
    Radio1.enableDynamicPayloads();             // needed
    Radio1.setAutoAck(true);                    // we want acks
    Radio1.maskIRQ(1, 1, 1);                    // no interrupts - seems NEEDED at the moment
    Radio1.openReadingPipe(1, BuddyMACAddPipe ^  ENCRYPT_KEY);
    delayMicroseconds(STOPLISTENINGDELAY);      // to allow the pipe to open
    Radio1.setChannel(QUIETCHANNEL);            // set the channel to the recovery channel
    Radio1.startListening();                    // start listening
    FlushFifos();                               // flush the fifos
    BlueLedOn();                                // turn on the blue led
    CurrentMode = LISTENMODE;                   // set the mode to listen
    WasBuddyPupilOnWireless = true;             // flag to indicate that the buddy was on wireless
}
//************************************************************************************************************************
#endif
