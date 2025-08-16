// *************************************** BuddyWireless.h  *****************************************

// This is the code for the wireless buddy system. Its functions are called from the main loop, and from the SendData() function.

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef BUDDYWIRELESS_H
#define BUDDYWIRELESS_H

#define LOSTCONTACTTHRESHOLD 100 // 100 fails in a row and we declare the buddy or master dead (0.5 seconds)
#define ENCRYPT_KEY 0xFEADFEADBB // The encryption key is used for the Pipe address between the transmitters

#define MASTER_HAS_CONTROL 0 // possible values for CurrentBuddyState
#define SLAVE_HAS_CONTROL 1
#define MASTER_CAN_NUDGE 2

// ********* BIT MAPPED SWITCH POSITIONS ENCODING IN CONTROL BYTE FROM MASTER TO PUPIL ************************************
//     BIT 0 is the BUDDY'S SWITCHES ENABLED
//     BIT 1 and 2 are the RATE - 1
//     BIT 3 and 4 are the BANK - 1
//     BIT 5 is MOTOR ON
//     BIT 6 is Safety ON
//     BIT 7 is SPARE
//*************************************************************************************************************************

bool LoadCorrectModel(uint64_t ModelID)
{                                            //  Gets Buddy onto same model as master, quietly.
    uint32_t SavedModelNumber = ModelNumber; //  Save the current model number
    static uint64_t FailedID = 0;            //  The ID of the model that failed to be found
    if (ModelID == FailedID)
        return false; // If the model failed to load, then don't try again
    ModelMatched = false;
    ModelNumber = 0;                                            //  Start from the first model
    while (!ModelMatched && (ModelNumber < MAXMODELNUMBER - 1)) //  Try to match the ID with a saved one
    {
        ++ModelNumber;
        ReadOneModel(ModelNumber);
        if ((ModelID == ModelsMacUnionSaved.Val64))
        {
            ModelMatched = true; //  Found it!
            PlaySound(MMFOUND);  //  Play the sound
        }
    }
    if (ModelMatched)
    {                                 //  Found it!
        UpdateModelsNameEveryWhere(); //  Show it everywhere.
        SaveAllParameters();          //  Save it
        GotoFrontView();              //  pretty obvious really
        FailedID = 0;                 //  Reset the failed ID
        return true;                  //  Success
    }
    else
    {
        PlaySound(NOTFOUND);            //  Play the not found sound
        ModelNumber = SavedModelNumber; //  Restore the current model number
        FailedID = ModelID;             //  Save the failed ID so we don't try again
        ReadOneModel(ModelNumber);      //  Restore the current model
        GotoFrontView();                //  pretty obvious really
        UpdateModelsNameEveryWhere();   //  Show model name everywhere.
        return false;                   //  Failed to match the model
    }
}

//*************************************************************************************************************************

void GetTheChannelData()
{
    for (int j = 0; j < CHANNELSUSED; ++j)
    { // While slave has control, his stick data replaces some of ours
        if (BuddyControlled & (1 << j))
            SendBuffer[j] = BuddyBuffer[j]; // Test if this channel is buddy controlled. If not leave it unchanged otherwise replace it with the buddy's value
    }
    if (!BuddyHasAllSwitches && !MotorEnabled)
    {
        SendBuffer[MotorChannel] = IntoHigherRes(MotorChannelZero); // If safety is on, throttle will be zero whatever was shown.
    }
}
// ********************************************************************************************************************************************

void GetTheChannelDataToMixWithOurs()
// This function takes the buddy's stick data, and mixes in ours —
// allowing the MASTER to subtly assist the BUDDY on any channel except 2 (e.g., motor or collective pitch).
{
    for (int j = 0; j < CHANNELSUSED; ++j)
    {
        if (BuddyControlled & (1 << j)) // Buddy has control of this channel
        {
            if (j != 2) // Avoid nudging motor or heli pitch control!
            {
                int16_t m = map(PreMixBuffer[j], MINMICROS, MAXMICROS, -HALFMICROSRANGE, HALFMICROSRANGE);
                SendBuffer[j] = constrain(BuddyBuffer[j] + m, MINMICROS, MAXMICROS);
            }
            else
            {
                SendBuffer[j] = BuddyBuffer[j]; // no nudge allowed on pitch or throttle.
            }
        }
    }

    if (!BuddyHasAllSwitches && !MotorEnabled)
    {
        SendBuffer[MotorChannel] = IntoHigherRes(MotorChannelZero);
    }
}

//*************************************************************************************************************************
void GetSlaveChannelValuesWireless()
{
    if (PupilIsAlive == 2)
        BuddyState = BUDDY_OFF; // If pupil is dead, then Buddy is off

    if (BuddyState == BUDDY_ON)
    {
        GetTheChannelData();
        if (CurrentBuddyState != SLAVE_HAS_CONTROL)
        {                        // Buddy has turned On
            PlaySound(BUDDYMSG); // Announce the Buddy is now in control
            CurrentBuddyState = SLAVE_HAS_CONTROL;
        }
        return;
    }

    if (BuddyState == BUDDY_NUDGE)
    {
        GetTheChannelDataToMixWithOurs(); //
        if (CurrentBuddyState != MASTER_CAN_NUDGE)
        {
            PlaySound(NUDGE_MSG);
            CurrentBuddyState = MASTER_CAN_NUDGE;
        }
        return;
    }

    if (BuddyState == BUDDY_OFF)
    {
        if (CurrentBuddyState != MASTER_HAS_CONTROL)
        {                         // Buddy has turned Off
            PlaySound(MASTERMSG); // Announce the Master is now in control
            CurrentBuddyState = MASTER_HAS_CONTROL;
        }
        return;
    }
}
//*************************************************************************************************************************
// This function is called by Pupil and the Master was Detected - or not Detected.
// After 6 failed detections, the Master is declared dead, and a message is changed on front view.
void MasterDetected(bool Detected)
{
    static uint16_t LostMasterCount = 0;
    char Mlost[] = "Master not found";
    char Mfound[] = "Master found!";
    char wb[] = "wb"; // wb is the name of the label on front view
    char YesVisible[] = "vis wb,1";

    if (Detected)
    {
        LostMasterCount = 0;
        LastPassivePacketTime = millis(); // reset the timer
        if (MasterIsAlive != 1)
        { // MasterIsAlive = 1 means Master was already alive
            SendText(wb, Mfound);
            SendCommand(YesVisible);
            MasterIsAlive = 1;
        }
    }
    else
    {
        ++LostMasterCount;
        if (LostMasterCount > LOSTCONTACTTHRESHOLD)
        {
            if (MasterIsAlive != 2)
            { // MasterIsAlive = 2 means Master was already dead
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
    char Mlost[] = "Buddy not found";
    char Mfound[] = "Buddy found!";
    char wb[] = "wb"; // wb is the name of the label on front view
    char YesVisible[] = "vis wb,1";
    if (Detected)
    {
        LostPupilCount = 0;
        LastPassivePacketTime = millis(); // reset the timer
        if (PupilIsAlive != 1)
        { // PupilIsAlive = 1 means Pupil was already alive
            SendText(wb, Mfound);
            SendCommand(YesVisible);
            PupilIsAlive = 1;
        }
    }
    else
    {
        ++LostPupilCount;
        if (LostPupilCount > LOSTCONTACTTHRESHOLD)
        {
            if (PupilIsAlive != 2)
            { // PupilIsAlive = 2 means Pupil was already dead
                SendText(wb, Mlost);
                SendCommand(YesVisible);
                PupilIsAlive = 2;
            }
        }
    }
}

/************************************************************************************************************/

void RearrangeTheChannels()
{
    //  This function looks at the 16 BITS of DataReceived.ChannelBitMask and rearranges the channels accordingly.
    uint8_t p = 0;
    for (int i = 0; i < CHANNELSUSED; ++i)
    {
        if (DataReceived.ChannelBitMask & (1 << i))
        {
            BuddyBuffer[i] = RawDataIn[p];
            ++p;
        } // if bit is set, set the channel
    }
    return;
}
// ************************************************************************************************************/
uint8_t GetDecompressedSize(uint8_t DynamicPayloadSize)
{
    uint8_t Ds = (((DynamicPayloadSize - 2) * 4) / 3) / 2; // first 2 bytes are ChannelBitMask, the rest is the 3:4 compressed data (hence the "-2")
    while (Ds % 4)                                         // make sure Ds is a multiple of 4
        ++Ds;                                              // increment until it is a multiple of 4
    return Ds;
}

//*************************************************************************************************************************
// Master gets Ack from pupil.

void GetPupilAck()
{
    if (Radio1.available())
    {
        uint8_t DynamicPayloadSize = Radio1.getDynamicPayloadSize(); // if a packet has arrived
        Radio1.read(&DataReceived, DynamicPayloadSize);              // read only bytes sent in the packet
        if (DataReceived.ChannelBitMask)                             // any channel changes?
        {
            Decompress(RawDataIn, DataReceived.CompressedData, GetDecompressedSize(DynamicPayloadSize)); // yes, decompress the data into RawDataIn array
            RearrangeTheChannels();                                                                      // Rearrange the channels
        }
    }
}

//*************************************************************************************************************************
void ChangeTXTarget(uint8_t ch, uint64_t p, rf24_datarate_e rate) // Swap between Buddy and Master
{                                                                 // ch is the channel, p is the pipe address, speed is the data rate
    Radio1.setDataRate(rate);                                     // Set the data rate
    Radio1.openWritingPipe(p);                                    // Set the pipe address
    Radio1.stopListening();
    delayMicroseconds(STOPLISTENINGDELAY);
    Radio1.setChannel(ch); // Set the frequency channel
    delayMicroseconds(STOPLISTENINGDELAY);
}

// ************************************************************************************************************
void DoTheLongerSpecialPacket()
{

    SpecialPacketData.ModelID = ModelsMacUnionSaved.Val64; // Send the model ID so that pupil can check it
    GetCommandbytes(&SpecialPacketData.Command[0], &SpecialPacketData.Command[1]);
    ChannelSentLastTime = SpecialPacketData.Channel; // Use the old channel number because Buddy hasn't yet hopped
    --Index;
    if (Index < 1)
        Index = 82;                                              // use the same array but in reverse order
    SpecialPacketData.Channel = FHSS_data::FHSS_Channels[Index]; // Set the  new channel number for next time
    if (NeedToRecover)
        SpecialPacketData.Channel = QUIETCHANNEL; // If contact lost, then use the recovery channel to recover

    ChangeTXTarget(ChannelSentLastTime, TeensyMACAddPipe ^ ENCRYPT_KEY, FASTDATARATE); // Set the TX target to the Buddy

    if (Radio1.write(&SpecialPacketData, sizeof SpecialPacketData))
    {                          // Send the packet
        GetPupilAck();         // Get ack from pupil WITH HIS CONTROL DATA!!
        PupilDetected(true);   // Pupil is alive
        NeedToRecover = false; // No need to recover
    }
    else
    {
        PupilDetected(false);                       // Pupil is dead
        NeedToRecover = true;                       // Need to recover
    }
}

// ********* BIT MAPPED SWITCH POSITIONS ENCODING IN CONTROL BYTE FROM MASTER TO PUPIL ************************************
//     BIT 0 is the BUDDY'S SWITCHES ENABLED
//     BIT 1 and 2 are the RATE - 1
//     BIT 3 and 4 are the BANK - 1
//     BIT 5 is MOTOR ON
//     BIT 6 is Safety ON
//     BIT 7 is SPARE
//*************************************************************************************************************************

void GetCommandbytes(uint8_t *C, uint8_t *C1)
{ // we have two bytes to send to the buddy.
    if (BuddyState == BUDDY_OFF)
    {
        *C = 'M'; // Send B to indicate Buddy is ON
    }
    if (BuddyState == BUDDY_ON)
    {
        *C = 'B'; // Send M to indicate Master is ON
    }
    if (BuddyState == BUDDY_NUDGE)
    {
        *C = 'N'; // Send M to indicate Master is ON
    }

    if (BuddyHasAllSwitches)
    {
        *C1 = 1; // Buddy has all the switches
    }
    else // Otherwise, inform pupil of all our switch positions
    {
        *C1 = (DualRateInUse - 1) << 1; // Uses low two bits, clears all the rest.
        *C1 |= (Bank - 1) << 3;         // Sets the bank bits in the command byte at bits 3 & 4
        *C1 |= MotorEnabled << 5;       // Sets the motor bit in the command byte at bit 5
        *C1 |= SafetyON << 6;           // Sets the safety bit in the command byte at bit 6
    }
}
//*************************************************************************************************************************
void SendSpecialPacket() // Here the master sends a packet to the buddy Hoping to receive in the ack payload All of his channel positions.
{
    static uint32_t LocalTimer = 0;

    if ((!BoundFlag || !ModelMatched || (PupilIsAlive != 1)))
    {
        if (((millis() - LocalTimer) < 100))
            return;
        LocalTimer = millis();
    }
    DoTheLongerSpecialPacket();                                 // Send the longer packet (model ID sent) EVERYTIME!
    ChangeTXTarget(CurrentChannel, TeensyMACAddPipe, DATARATE); // Set the TX target back to the receiver in model.
}

//*************************************************************************************************************************
void SetUpTargetForBuddy() // Once only because the model is connected via module. Only the Buddy is using the nRF24L01 in here
{
    Radio1.setDataRate(FASTDATARATE); // Set the data rate
    delayMicroseconds(SELECTTARGETDELAY);
    Radio1.openWritingPipe(TeensyMACAddPipe ^ ENCRYPT_KEY); // Set the pipe address
    delayMicroseconds(SELECTTARGETDELAY);
    Radio1.stopListening();
    delayMicroseconds(SELECTTARGETDELAY);
}

// ************************************************************************************************************
void SendTheSpecialAckPayload()
{

    uint8_t ByteCountToTransmit;
    uint8_t NumberOfChangedChannels = EncodeTheChangedChannels(); // Encode the changed channels (re-use other function!)

    if (NumberOfChangedChannels)
    {                                                                      // Any channels changed? Or parameters to send?
        ByteCountToTransmit = ((float)NumberOfChangedChannels * 1.5f) + 4; // Calculate the number of bytes to transmit
        uint8_t SizeOfUnCompressedData = static_cast<uint8_t>(ByteCountToTransmit / 1.5f);
        Compress(DataTosend.CompressedData, RawDataBuffer, SizeOfUnCompressedData);
    }
    else
    {
        ByteCountToTransmit = 2;
        NewCompressNeeded = false;
    }
    Radio1.writeAckPayload(1, &DataTosend, ByteCountToTransmit); // Acknowledge the packet BY SENDING MY CHANNEL DATA!
    delayMicroseconds(30);
}

// ********* BIT MAPPED SWITCH POSITIONS ENCODING IN CONTROL BYTE FROM MASTER TO PUPIL ************************************
//     BIT 0 is the BUDDY'S SWITCHES ENABLED
//     BIT 1 and 2 are the RATE - 1
//     BIT 3 and 4 are the BANK - 1
//     BIT 5 is MOTOR ON
//     BIT 6 is Safety ON
//     BIT 7 is SPARE
//*************************************************************************************************************************

void TestTheCommandByte(uint8_t C, uint8_t C1) // two bytes really
{
    static uint8_t Last_C = 42; // simply not M or B or N

    if (C != Last_C) // if the C command has changed ...
    {
        Last_C = C; // save new C, and ...
        if (C == 'B')
        {
            MasterIsInControl = false; // Buddy is now in control
            PlaySound(BUDDYMSG);       // Announce Buddy is now in control
        }
        if (C == 'M')
        {
            MasterIsInControl = true; // Master is now in control
            PlaySound(MASTERMSG);     // Announce Master is now in control
        }
        if (C == 'N')
        {
            MasterIsInControl = false; // Master is not in control but can nudge
            PlaySound(NUDGE_MSG);      // Announce Nudge mode
        }
    }
    BuddyHasAllSwitches = (C1 & 1); // the 0th BIT of C1 is switch control
    if (!BuddyHasAllSwitches)       // If Buddy does not have switch control, he must get these from Master
    {
        DualRateInUse = ((C1 >> 1) & 3) + 1; // Get the dual rate bits
        Bank = ((C1 >> 3) & 3) + 1;          // Get the bank bits
        MotorEnabled = (C1 >> 5) & 1;        // Get the motor bit
        SafetyON = (C1 >> 6) & 1;            // Get the safety bit
    }
}
// ************************************************************************************************************

void SetNewListenChannel(uint8_t Channel)
{
    Radio1.stopListening();
    Radio1.setChannel(Channel);
    Radio1.startListening();
}
// ************************************************************************************************************
void ParseLongerSpecialPacket()
{ // This is called from GetSpecialPacket() function just below

    static uint64_t LastModelID = 0;
    Radio1.read(&SpecialPacketData, sizeof SpecialPacketData);                      // read the packet if its still there
    TestTheCommandByte(SpecialPacketData.Command[0], SpecialPacketData.Command[1]); // Test the command byte
    if (AutoModelSelect)
    { // If auto model select is on
        if (SpecialPacketData.ModelID != LastModelID)
        {                                            // if the model ID has changed, match it if we can
            LastModelID = SpecialPacketData.ModelID; // Save the model ID so we can know if it changed
            if (SpecialPacketData.ModelID != ModelsMacUnionSaved.Val64)
            {                                                     // is it the currently loaded model?
                if (!LoadCorrectModel(SpecialPacketData.ModelID)) // If not, try to load it
                {
                    LastModelID = 0; // Reset the model ID so we can try again
                }
            }
        }
    }
    SetNewListenChannel(SpecialPacketData.Channel); // Set the frequency channel
}

//*************************************************************************************************************************
void GetSpecialPacket()
{ // Here the buddy receives a packet from the master to which it must respond with its channel positions.

    static uint32_t LocalTimer = 0;
    static uint16_t PacketCounter = 0;
    char ExsBd[] = "Extra buddies!";
    char wb[] = "wb"; // wb is the name of the label on front view
    char YesVisible[] = "vis wb,1";

    if (Radio1.available())
    {                                                                   // if a packet has arrived
        if (Radio1.getDynamicPayloadSize() != sizeof SpecialPacketData) // MUST be sizeof SpecialPacketData (= 24!) Heer
        {
            SendText(wb, ExsBd); // If not, declare an error: excess buddies !
            SendCommand(YesVisible);
            PlaySound(EXTRABUDDIES);
            DelayWithDog(10000);
            return;
        }

        SendTheSpecialAckPayload();
        ParseLongerSpecialPacket(); // Parse the packet
        MasterDetected(true);       // Master is alive
        ++PacketCounter;            // Count the packets

        if (millis() - LocalTimer >= 1000)
        {                                     // Every second
            LocalTimer = millis();            // reset the timer
            PacketsPerSecond = PacketCounter; // save the packets per second for use on the data diplay
            PacketCounter = 0;                // reset the counter
        }
        LastPassivePacketTime = millis(); // reset the timer
    }
    else
    { // No packet arrived so maybe master's dead?
        if (millis() - LastPassivePacketTime > PACEMAKER)
        { // We expect a packet every PACEMAKER ( = 5 ) milliseconds. If none, a packet was lost.
            Radio1.stopListening();
            Radio1.setChannel(QUIETCHANNEL); // Set the recovery channel
            Radio1.startListening();
            MasterDetected(false);
            LastPassivePacketTime = millis(); // Reset the timer
            ++TotalLostPackets;               // Count the lost packets
        }
    }
}

//*************************************************************************************************************************
void StartBuddyListen()
{
    Radio1.setDataRate(FASTDATARATE); // 2 MBPS
    Radio1.enableAckPayload();        // needed
    Radio1.enableDynamicPayloads();   // needed
    Radio1.setAutoAck(true);          // we want acks
    Radio1.maskIRQ(1, 1, 1);          // no interrupts - seems NEEDED at the moment
    Radio1.openReadingPipe(1, BuddyMACAddPipe ^ ENCRYPT_KEY);
    delayMicroseconds(STOPLISTENINGDELAY); // to allow the pipe to open
    Radio1.setChannel(QUIETCHANNEL);       // set the channel to the recovery channel
    Radio1.startListening();               // start listening
    FlushFifos();                          // flush the fifos
    BlueLedOn();                           // turn on the blue led
    CurrentMode = LISTENMODE;              // set the mode to listen
    WasBuddyPupilOnWireless = true;        // flag to indicate that the buddy was on wireless
}
//************************************************************************************************************************
#endif
