// *************************************** ModelExchange.h  *****************************************
#include <Arduino.h>
#include <SD.h>
#include <RF24.h>
#include "Hardware/1Definitions.h"

#ifndef MODELEXCHANGE_H
    #define MODELEXCHANGE_H


/*********************************************************************************************************************************/
// SEND AND RECEIVE A MODEL FILE
/*********************************************************************************************************************************/

    #define FILEPIPEADDRESS 0xFEFEFEFEFELL // Unique pipe address for FILE EXCHANGE
    #define BUFFERSIZE      28             // + 4 = 32
    #define FILEDATARATE    RF24_250KBPS
    #define FILEPALEVEL     RF24_PA_MAX
    #define FILECHANNEL     QUIETCHANNEL
    #define FILETIMEOUT     30

/*********************************************************************************************************************************/
void ShowFileProgress(char* Msg)
{
    char t1[] = "t1";
    SendText(t1, Msg);//
}
/*********************************************************************************************************************************/
void ShowFileTransferWindow()
{
    char gofw[] = "page FileExchView";
    SendCommand(gofw);
    CurrentView = FILEEXCHANGEVIEW;
}
/*********************************************************************************************************************************/
void StoreBuffer(char* Buf, uint32_t len)
{
    for (uint16_t i = 0; i < len; ++i) {
        if ((NewFileBufferPointer + i) >= MAXFILELEN) break;
        NewFileBuffer[NewFileBufferPointer + i] = Buf[i];
    }
    NewFileBufferPointer += len;
}
/*********************************************************************************************************************************/

void WriteEntireBuffer()
{
    ModelsFileNumber.close();
    CloseModelsFile();
    for (int i = 0; i < 3; ++i) {
        ModelsFileNumber = SD.open(SingleModelFile, FILE_WRITE);
        ModelsFileNumber.seek(0);
        ShortishDelay();
        ModelsFileNumber.write(NewFileBuffer, NewFileBufferPointer);
        ShortishDelay();
        ModelsFileNumber.close();
        ShortishDelay();
    }
}

/*********************************************************************************************************************************/

void ShowRemoteID()
{ // Show remote ID
    char nb2[5];
    char msg[50];
    char GoModelsView[] = "page ModelsView";
    strcpy(msg, "Remote ID = ");
    for (int i = 0; i < 5; ++i) {
        snprintf(nb2, 4, "%X", BuddyMacAddress[i]);
        strcat(msg, nb2);
        strcat(msg, " ");
    }
    MsgBox(GoModelsView, msg);
}

/*********************************************************************************************************************************/

/** @brief SEND A MODEL FILE */
void SendModelFile()
{
    uint64_t      TXPipe;
    uint8_t       Fack      = 1;
    unsigned long Fsize     = 0;
    unsigned long Fposition = 0;
    char          Fbuffer[BUFFERSIZE + 8]; // spare space
    uint8_t       PacketNumber = 0;
    int           p            = 5;
    char          nb1[20];
    char          Sent[] = "Sent ";
    char          of[]   = " of ";
    char          msg[150];
    char          bytes[]               = " bytes.";
    char          ModelsView_filename[] = "filename";
    char          t0[]                  = "t0";
    char          Fsend[]               = "Sending file";
    char          ProgressStart[]       = "vis Progress,1";
    char          ProgressEnd[]         = "vis Progress,0";
    char          GoModelsView[]        = "page ModelsView";
    char          Progress[]            = "Progress";
    bool          ReceiverConnected     = false;

    BlueLedOn();
    CloseModelsFile();
    ShowFileTransferWindow();
    SendText(ModelsView_filename, SingleModelFile);
    SendText(t0, Fsend);
    SendCommand(ProgressStart);
    SendValue(Progress, p);
    DelayWithDog(10);
    #ifdef DB_MODEL_EXCHANGE
    Serial.print("Sending model: ");
    Serial.println(SingleModelFile);
    #endif
    TXPipe           = FILEPIPEADDRESS;
    ModelsFileNumber = SD.open(SingleModelFile, O_READ); // Open file for reading
    Fsize            = ModelsFileNumber.size();          // Get file size
    #ifdef DB_MODEL_EXCHANGE
    Serial.print("File Size: ");
    Serial.print(Fsize);
    Serial.println(" bytes.");
    #endif
    ConfigureRadio(); //  Start from known state
    Radio1.setChannel(FILECHANNEL);
    Radio1.setPALevel(FILEPALEVEL, true);
    Radio1.setRetries(2, 15);
    Radio1.openWritingPipe(TXPipe);
    Radio1.stopListening();
    DelayWithDog(4);

    while ((Fposition < Fsize)) {
        KickTheDog(); // Watchdog
        p = ((float)Fposition / (float)Fsize) * 100;
        strcpy(msg, Sent);
        strcat(msg, Str(nb1, Fposition, 0));
        strcat(msg, of);
        strcat(msg, Str(nb1, Fsize, 0));
        ShowFileProgress(msg);
        SendValue(Progress, p);
        ++PacketNumber;
        for (int q = 0; q < 36; ++q) { // clear buffer
            Fbuffer[q] = 0;
        }
        if (PacketNumber == 1) {
            strcpy(Fbuffer, SingleModelFile); // Filename in first packet (add in the mac address  as there is space!)
            Fbuffer[BUFFERSIZE]     = Fsize;
            Fbuffer[BUFFERSIZE + 1] = Fsize >> 8;
            Fbuffer[BUFFERSIZE + 2] = Fsize >> 16;
            Fbuffer[BUFFERSIZE + 3] = Fsize >> 24; // SEND FILE SIZE (four bytes)
            for (int q = 0; q < 5; ++q) {
                Fbuffer[q + 16] = MacAddress[q + 1]; // Add only 5 bytes of the macaddress to buffer at offset 16 and sent it too! heer
            }
        }
        else {
            ModelsFileNumber.seek(Fposition); // Move filepointer
            ShortDelay();
            ModelsFileNumber.read(Fbuffer, BUFFERSIZE); // Read part of file
            Fposition += BUFFERSIZE;
            if (Fposition > Fsize) Fposition = Fsize;
        }
        ReceiverConnected = (Radio1.write(&Fbuffer, BUFFERSIZE + 4)); //  we added error checking for RX not ready. Now we maybe need a real checksum
        if (!ReceiverConnected) break;
        Radio1.read(&Fack, sizeof(Fack));

    #ifdef DB_MODEL_EXCHANGE
        Serial.println(PacketNumber);
    #endif
    }
    ModelsFileNumber.close();
    #ifdef DB_MODEL_EXCHANGE
    Serial.println("ALL SENT.");
    #endif
    SendValue(Progress, 100);
    if (ReceiverConnected) DelayWithDog(750);
    NormaliseTheRadio();
    SendCommand(ProgressEnd);
    RedLedOn();
    if (ReceiverConnected) {
        strcpy(msg, Sent);
        strcat(msg, Str(nb1, Fsize, 0));
        strcat(msg, bytes);
        PlaySound(BEEPCOMPLETE);
        ShowFileProgress(msg);
        DelayWithDog(2000);
    }
    else {
        strcpy(msg, "                Cannot send!\r\n\r\nReceiving transmitter not ready. \r\n\r\n                File not sent.");
        for (int i = 0; i < 3; ++i) {
            PlaySound(BEEPMIDDLE);
            DelayWithDog(130);
        }
        MsgBox(GoModelsView, msg);
    }
    SendCommand(GoModelsView);
    CurrentView = MODELSVIEW;
    CloseModelsFile();
    ConfigureRadio();
}

/***************************************************************************************************************************************************/
/**************************************************** RECEIVE A MODEL FILE *************************************************************************/
/***************************************************************************************************************************************************/

/** @brief RECEIVE A MODEL FILE */
void ReceiveModelFile()
{
    uint64_t      RXPipe;
    uint32_t      RXTimer               = 0;
    char          ModelsView_filename[] = "filename";
    char          Fbuffer[BUFFERSIZE + 8]; // spare space
    uint8_t       Fack      = 1;           // just a token byte
    char          Waiting[] = "Waiting ";
    char          WaitTime[6];
    char          WaitMsg[17];
    char          ThreeDots[] = "...";
    char          Receiving[] = "Receiving: ";
    char          fnamebuf[30];
    char          TimeoutMsg[]   = "TIMEOUT";
    char          Success[]      = "* Success! *";
    unsigned long Fsize          = 0;
    unsigned long Fposition      = 0;
    float         SecondsElapsed = 0;
    uint8_t       p              = 5;
    char          nb1[40];

    char Received[]      = "Received ";
    char of[]            = " of ";
    char msg[50];
    char bytes[]         = " bytes.";
    char t0[]            = "t0";
    char RXheader[]      = "File receive";
    char ovwr[]          = "Overwrite ";
    char ques[]          = "?";
    char ProgressStart[] = "vis Progress,1";
    char ProgressEnd[]   = "vis Progress,0";
    char GoModelsView[]  = "page ModelsView";
    char Progress[]      = "Progress";
    
    if (strcmp(ModelName, "Not in use")) {      // If not in use, we can overwrite it without asking
        strcpy(msg, ovwr);                      // Ask user if they want to overwrite current model
        strcat(msg, ModelName);
        strcat(msg, ques);
        if (!GetConfirmation(GoModelsView, msg)) return; // Get confirmation or quit
    }
    
    BlueLedOn();  
    ShowFileTransferWindow();
    SendText(ModelsView_filename, Waiting);
    SendText(t0, RXheader);
    
    ConfigureRadio(); //  Start from known state

    RXPipe = FILEPIPEADDRESS;
    Radio1.setRetries(2, 15);
    Radio1.setChannel(FILECHANNEL);
    Radio1.flush_tx();
    Radio1.flush_rx();
    Radio1.openReadingPipe(1, RXPipe);
    Radio1.startListening();
    NewFileBufferPointer = 0;
   
    CloseModelsFile();
   
    for (int q = 0; q < 36; ++q) {                         
        Fbuffer[q] = 0;                                     // clear buffer for the sender's macaddress
        delay(5);                                           // give the radio a chance to clear the buffer      
        KickTheDog();                                       // Keep Watchdog happy
        GetButtonPress();                                   // Clear any pending button presses so they won't stop next bit ...             
        ClearText();                                        // Clear any pending text           
    }
    RXTimer              = millis();                        // Start timer
    while (!Radio1.available()) {                           // Await the sender....
        delay(1);
        if (GetButtonPress()) {                            // user can abandon the transfer wait by hitting a button now
            GotoModelsView();
            ClearText();
            NormaliseTheRadio();
            RedLedOn();
            ButtonWasPressed();
            return;
        }
        KickTheDog(); // Watchdog
        if ((millis() - RXTimer) / 1000 >= FILETIMEOUT) {
            SendText(ModelsView_filename, TimeoutMsg);
            NormaliseTheRadio();
            
            return; // Give up waiting
        }
        else
        {
            SecondsElapsed = (millis() - RXTimer) / 1000;
            if (SecondsElapsed == (int)SecondsElapsed) { // whole number of seconds?
                strcpy(WaitMsg, Waiting);
                strcat(WaitMsg, Str(WaitTime, (FILETIMEOUT - SecondsElapsed), 0));
                strcat(WaitMsg, ThreeDots);
                SendText(ModelsView_filename, WaitMsg); // Show user how long remains to wait
                if (GetButtonPress()) {
                    GotoModelsView();
                    ClearText();
                    NormaliseTheRadio();
                    RedLedOn();
                    ButtonWasPressed();
                    return;
                }
            }
        }
    } // *First* packet must have arrived!

    Radio1.writeAckPayload(1, &Fack, sizeof(Fack)); //  Send first ack
    DelayWithDog(5);
    Radio1.read(&Fbuffer, BUFFERSIZE + 4);          //  Read first packet
    SendCommand(ProgressStart);
    SendValue(Progress, p);
    SendText(ModelsView_filename, Receiving);
    strcpy(SingleModelFile, Fbuffer); //  Get filename
    Fsize = Fbuffer[BUFFERSIZE];
    Fsize += Fbuffer[BUFFERSIZE + 1] << 8;
    Fsize += Fbuffer[BUFFERSIZE + 2] << 16;
    Fsize += Fbuffer[BUFFERSIZE + 3] << 24; //  Get file size

    for (int q = 0; q < 5; ++q) {
        BuddyMacAddress[q] = Fbuffer[q + 16]; // sender's macaddress is in buffer at offset 16 - get it heer!
                                              //  if (q == 0) ++BuddyMacAddress[q];     // add one to lowest byte to make it unique // heer!!
    }

    #ifdef DB_MODEL_EXCHANGE
    Serial.println("CONNECTED!");
    Serial.print("FileName=");
    Serial.println(SingleModelFile);
    Serial.print("File size = ");
    Serial.println(Fsize);
    #endif

    Fposition = 0;
    strcpy(fnamebuf, Receiving);
    strcat(fnamebuf, SingleModelFile);
    SendText(ModelsView_filename, fnamebuf);
    RXTimer = millis();         //  zero timeout
    while (Fposition < Fsize) { //  (Fposition<Fsize) ********************
        KickTheDog();           //  Watchdog
        if (GetButtonPress()) { // user can abandon the transfer by hitting a button
            ButtonWasPressed();
            NormaliseTheRadio();
            RedLedOn();
            GotoModelsView();
            return;
        }
        if (Radio1.available()) {
            Radio1.writeAckPayload(1, &Fack, sizeof(Fack));
            DelayWithDog(5);
            Radio1.read(&Fbuffer, BUFFERSIZE + 4);
            StoreBuffer(Fbuffer, BUFFERSIZE); // Store it in ram for now rather than disk it
            Fposition += BUFFERSIZE;
            if (Fposition > Fsize) Fposition = Fsize;
            p = ((float)Fposition / (float)Fsize) * 100;
            SendValue(Progress, p);
            strcpy(msg, Received);
            strcat(msg, Str(nb1, Fposition, 0));
            strcat(msg, of);
            strcat(msg, Str(nb1, Fsize, 0));
            ShowFileProgress(msg);
        }
        else {
           // NoPacketArrivedYet(PacketArrivalTime); // error handling here! ....
        }
    }
    SendValue(Progress, 100);
    WriteEntireBuffer();
    BuildDirectory();
    SendText(ModelsView_filename, Success);
    DelayWithDog(500);
    SendText(ModelsView_filename, SingleModelFile);

    // Below Here the new model is imported for immediate use

    SingleModelFlag = true;
    ReadOneModel(1);
    if (SavedSticksMode != SticksMode) { // swap over trims (elevator -  Throttle)
        for (int ba = 1; ba < 5; ++ba) {
            uint8_t temp = Trims[ba][1];
            Trims[ba][1] = Trims[ba][2];
            Trims[ba][2] = temp;
        }
    }
    SingleModelFlag = false;
    CloseModelsFile();
    SaveAllParameters();
    CloseModelsFile();
    NormaliseTheRadio();
    SendCommand(ProgressEnd);
    strcpy(msg, Received);
    strcat(msg, Str(nb1, Fsize, 0));
    strcat(msg, bytes);
    ShowFileProgress(msg);
    ClearText();
    PlaySound(BEEPCOMPLETE);
    CloseModelsFile();
    DelayWithDog(2000);
    SaveTransmitterParameters();
    GotoModelsView();
    ClearText();
    RedLedOn();
    BoundFlag = true; // This just prevents jump to front screen (Cleared on leaving models area)
    ConfigureRadio();
    if (BuddyPupilOnWireless) {
        StartBuddyListen();
         BoundFlag = false;
    }
}
// ***********************************************************************************************************

#endif