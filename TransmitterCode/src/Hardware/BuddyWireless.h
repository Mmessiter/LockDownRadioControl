// *************************************** BuddyWireless.h  *****************************************

// This is the code for the wireless buddy system. Its functions are called from the main loop, and from the SendData() function.

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef BUDDYWIRELESS_H
    #define BUDDYWIRELESS_H

    
    #define LONGER_DELAY           1                // ... milliseconds
    #define LOSTCONTACTTHRESHOLD   6                // 6 fails in a row and we declare the buddy or master dead
    #define DELAYAFTERACK          1                // ms
    #define ENCRYPT_KEY            0xFEADFEADBB     // The encryption key used for the Pipe address between the transmitters 
   
   
 
//*************************************************************************************************************************

void LoadCorrectModel(uint64_t ModelID){                                        //  Gets Buddy onto same model as master, quietly.                                            
    uint32_t SavedModelNumber = ModelNumber;                                    //  Save the current model number
    if (!ModelExistsAtBuddy) return;
    ModelMatched = false;
    ModelNumber = 0;
    while (!ModelMatched && (ModelNumber < MAXMODELNUMBER - 1)) {               //  Try to match the ID with a saved one
        ++ModelNumber;
        ReadOneModel(ModelNumber);
        if ((ModelID == ModelsMacUnionSaved.Val64)) ModelMatched = true;        //  Found it!
    }
    if (ModelMatched) {                                                         //  Found it!
        UpdateModelsNameEveryWhere();                                           //  Show it everywhere.
        ModelExistsAtBuddy = true;                                              //  Flag to indicate that the model is here
        SaveAllParameters();                                                    //  Save it
        GotoFrontView();
    }else{
        ModelNumber = SavedModelNumber;                                         //  Restore the current model number
        ReadOneModel(ModelNumber);                                              //  Restore the current model 
        ModelExistsAtBuddy = false;                                             //  Flag to indicate that the model isn't here so don't look again
      //  Look("Model not found");                                                //  Tell the user
        GotoFrontView();
    }
}


//*************************************************************************************************************************

void CheckModelMatchesMaster(uint64_t ModelID){
    static uint32_t LastModelIDCheckTime = 0;                                           // The pupil checks the model ID once every second and loads the correct model if it !matches
    if (millis()-LastModelIDCheckTime > 1000) {                                         // check the model ID once every second
        LastModelIDCheckTime = millis();
        if (ModelID!= ModelsMacUnionSaved.Val64) LoadCorrectModel(ModelID);             // if the model ID !matches, then load the correct model    
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
        LastPassivePacketTime = millis();                   // reset the timer
        if (MasterIsAlive != 1) {                           // MasterIsAlive = 1 means Master was already alive
            SendText(wb, Mfound);
            SendCommand(YesVisible);
            MasterIsAlive = 1;
        }
    }
    else {
        ++LostMasterCount;
        if (LostMasterCount > LOSTCONTACTTHRESHOLD) {
            if (MasterIsAlive != 2) {                       // MasterIsAlive = 2 means Master was already dead
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
    uint16_t AckSpecial[COMPRESSEDWORDS];                       // Ack payload will contain all the pupil's control data
    if (Radio1.available()) {                                   // if a packet has arrived
        Radio1.read(&AckSpecial, sizeof AckSpecial);            // read the packet
        Decompress(BuddyBuffer, AckSpecial, UNCOMPRESSEDWORDS); // decompress the data into buddybuffer array
    }
}

//*************************************************************************************************************************
void ChangeTXTarget(uint8_t ch,uint64_t p, rf24_datarate_e speed)       // Swap between Buddy and Master
{                                                                       // ch is the channel, p is the pipe address, speed is the data rate
     Radio1.setDataRate(speed);                                         // Set the data rate
     Radio1.openWritingPipe(p);                                         // Set the pipe address
     Radio1.stopListening();
     delayMicroseconds(STOPLISTENINGDELAY);
     Radio1.setChannel(ch);                                             // Set the frequency channel
     delayMicroseconds(STOPLISTENINGDELAY);
}

//*************************************************************************************************************************
void SendSpecialPacket()                                                    // Here the MASTER sends to PUPIL tx. This is called about 100 times a second.
{                                                                           // Master Sends M or B to indicate whether Buddy is on or off, and the ID of the model which should be loaded.
    static uint32_t LocalTimer      = 0;
    static bool     NeedToRecover   = false;
    static uint8_t  NpOld           = 0;                                    // The old channel number
  
     struct spd
    {
        char        Command[2];
        uint64_t    ModelID;
        uint8_t     Np = QUIETCHANNEL;
    };

    static spd SpecialPacketData;

    rf24_datarate_e faster = FASTDATARATE, slower = DATARATE;               // Save the data rates
    if ((!BoundFlag || !ModelMatched  || (PupilIsAlive != 1))){             // Don't send too often if not connected, nor if buddy was not found
        if (((millis() - LocalTimer) < 20)) return;    
            LocalTimer = millis();
    }
    SpecialPacketData.ModelID =  ModelsMacUnionSaved.Val64;                 // Send the model ID so that pupil can check it
    SpecialPacketData.Command[0]                    = 'M';                  // Send M to indicate Master is ON
    if (BuddyON)       SpecialPacketData.Command[0] = 'B';                  // Send B to indicate Buddy is ON
    NpOld = SpecialPacketData.Np;                                           // Use the old channel number because Buddy hasnt yet hopped
    ++SpecialPacketData.Np;                                                 // Hop to the next channel               
    if (SpecialPacketData.Np > 82) SpecialPacketData.Np = 1;                // Wrap around if needed
    if (NeedToRecover) SpecialPacketData.Np = QUIETCHANNEL;                 // If contact lost, then use the recovery channel to recover
    ChangeTXTarget(NpOld,TeensyMACAddPipe ^ ENCRYPT_KEY,faster);            // Set the TX target to the Buddy
    if (Radio1.write(&SpecialPacketData, sizeof SpecialPacketData)) {
        GetPupilAck();                                                      // Get ack from pupil WITH HIS CONTROL DATA!!
        PupilDetected(true);                                                // Pupil is alive
        NeedToRecover = false;
    } else {
        PupilDetected(false);                                               // Pupil is dead
        NeedToRecover = true;
    }
    ChangeTXTarget(CurrentChannel,TeensyMACAddPipe,slower);
}
//*************************************************************************************************************************
void GetSpecialPacket()                                                                 // here the PUPIL tx gets from MASTER tx. This function is called from main loop about 100 times a second.
{                                                                                       // Master tells Pupil whether buddy is on or off, and the ID of the model which should be loaded.
    static bool MasterIsInControl = true; 
    struct spd {
        char        Command[2];
        uint64_t    ModelID = 0;
        uint8_t     Np = QUIETCHANNEL;

    };
    spd SpecialPacketData;
    if (Radio1.available()) {                                                           // if a packet has arrived
        Radio1.writeAckPayload(1, &DataTosend.CompressedData, SizeOfCompressedData);    // Acknowledge the packet BY SENDING MY CHANNEL DATA!
        delay(DELAYAFTERACK);                                                           // <-  *MUST* allow the ACK time to get going, otherwise the sender sees a failed packet    
        Radio1.read(&SpecialPacketData, sizeof SpecialPacketData);                      // read the packet if its still there
        if ((SpecialPacketData.Command[0] == 'B') && (MasterIsInControl)) {             // Buddy is now in control
            MasterIsInControl = false;                                                  // Buddy is now in control
            PlaySound(BUDDYMSG);                                                        // Announce the Buddy is now in control
        }
        if ((SpecialPacketData.Command[0] == 'M') && (!MasterIsInControl)) {            // Master is now in control
            MasterIsInControl = true;                                                   // Master is now in control
            PlaySound(MASTERMSG);                                                       // Announce the Master is now in control
        }
        CheckModelMatchesMaster(SpecialPacketData.ModelID);                             // check the model ID (once every second) and load the correct model if it !matches
        MasterDetected(true);                                                           // Master is alive
        LastPassivePacketTime = millis();                                               // reset the timer
        Radio1.stopListening(); 
        Radio1.setChannel(SpecialPacketData.Np);                                        // Set the frequency channel
        Radio1.startListening();                                                        // start listening
    }else{ // no packet arrived so maybe master's dead? 
        if (millis() - LastPassivePacketTime > 10) {                                    // We expect a packet every 5 milliseconds
           Radio1.stopListening(); 
           Radio1.setChannel(QUIETCHANNEL);                                             // Set the recovery channel         
           Radio1.startListening();
           LastPassivePacketTime = millis();                                            // reset the timer
           MasterDetected(false);
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
    Radio1.setChannel(QUIETCHANNEL);  // set the channel to the special packet channel
    Radio1.startListening();                    // start listening
    FlushFifos();                               // flush the fifos
    BlueLedOn();                                // turn on the blue led
    CurrentMode = LISTENMODE;                   // set the mode to listen
    RestoreBrightness();                        // restore the brightness
}
//************************************************************************************************************************
#endif
