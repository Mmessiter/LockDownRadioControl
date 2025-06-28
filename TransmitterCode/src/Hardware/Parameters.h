#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef PARAMETERS_H
#define PARAMETERS_H

/*********************************************************************************************************************************/

void AddParameterstoQueue(uint8_t ID) // todo:  This function repeats the same parameter 3 times.  Should not be necessary.
{

    for (int i = 0; i < PARAMETERSENDREPEATS; ++i)
    {
        if (ParametersToBeSentPointer < 78)
        {
            ++ParametersToBeSentPointer;
            ParametersToBeSent[ParametersToBeSentPointer] = ID;
        }
    }
    // Look1("Queued: ");
    // Look1(ID);
    // Look1(" ");
    // Look(ParaNames[ID - 1]);
}

/*********************************************************************************************************************************/
void SendInitialSetupParams()
{
    AddParameterstoQueue(QNH_SETTING);        // QNH 2
    AddParameterstoQueue(SERVO_FREQUENCIES);  // Servo Frequencies 6
    AddParameterstoQueue(SERVO_PULSE_WIDTHS); // Servo Pulse Widths 7
    AddParameterstoQueue(DUMMY4);
    AddParameterstoQueue(DUMMY5);
}

/************************************************************************************************************/
void SendOutstandingParameters()
{ // Send any QUEUED parameters that have not been sent yet

    if (BoundFlag && ModelMatched && LedWasGreen)
    {
        Parameters.ID = ParametersToBeSent[ParametersToBeSentPointer];
        --ParametersToBeSentPointer;
        AddExtraParameters = true;
        // Look1("Sent: ");
        // Look1(Parameters.ID);
        // Look1(" ");
        // Look(ParaNames[Parameters.ID - 1]);
    }
}

/************************************************************************************************************/
// NB ONLY THE LOW 12 BITS ARE ACTUALLY SENT! (Because of the compression)

void LoadParameters()
{
    uint16_t Twobytes = 0;
    uint8_t FS_Byte1;
    uint8_t FS_Byte2;

    for (int i = 0; i < 12; ++i)
        Parameters.word[i] = 0; // clear the parameters

    switch (Parameters.ID)
    {                                             // ID is the parameter number. Each has 8 elements
    case FAILSAFE_SETTINGS:                       // 1 = FailSafeChannels
        Twobytes = MakeTwobytes(FailSafeChannel); // 16 bool values compressed to 16 bits
        FS_Byte1 = uint8_t(Twobytes >> 8);        // Send as two bytes
        FS_Byte2 = uint8_t(Twobytes & 0x00FF);
        Parameters.word[1] = FS_Byte1; // These are failsafe flags
        Parameters.word[2] = FS_Byte2; // These are failsafe flags
        break;
    case QNH_SETTING: // 2 = QNH

        Parameters.word[1] = (uint16_t)Qnh;
        Parameters.word[2] = 1234;

        break;
    case GPS_MARK_LOCATION: // 3 = GPSMarkHere
        if (GPSMarkHere)
        {
            Parameters.word[1] = 0;
            Parameters.word[2] = GPSMarkHere;
            GPSMarkHere = 0;
            GPS_RX_MaxDistance = 0;
        }
        break;
    case DUMMY4: // 4 = NOT USED YET
        Parameters.word[1] = 44;
        Parameters.word[2] = 444;
        break;
    case DUMMY5:
        Parameters.word[1] = 55;
        Parameters.word[2] = 555;
        break;
    case SERVO_FREQUENCIES: // 6 = Servo Frequencies
        for (int i = 0; i < 11; ++i)
            Parameters.word[i + 1] = ServoFrequency[i];
        break;
    case SERVO_PULSE_WIDTHS: // 7 = Servo Pulse Widths
        for (int i = 0; i < 11; ++i)
            Parameters.word[i + 1] = ServoCentrePulse[i];
        break;
    default:
        break;
    }
}

/************************************************************************************************************/

void DebugParamsOut()
{
    Look1("Parameter ID: ");
    Look1(Parameters.ID);
    Look1(" ");
    Look(ParaNames[Parameters.ID - 1]);
    for (int i = 1; i < 12; ++i)
    {
        Look1("  Word[");
        Look1(i);
        Look1("]: ");
        Look(Parameters.word[i]);
    }
}

/************************************************************************************************************/

void LoadRawDataWithParameters()
{
    RawDataBuffer[0] = Parameters.ID; // copy current parameter values into the rawdatabuffer instead of the channels
    for (int i = 1; i < 12; ++i)
    {
        RawDataBuffer[i] = Parameters.word[i]; // copy current parameter values into the rawdatabuffer instead of the channels
    }
}
/************************************************************************************************************/
int GetExtraParameters() // This gets extra parameters ready for sending and returns the number that will be sent.
{                        // only the ***low 12 bits*** of each parameter are actually sent because of compression
    // if ((Parameters.ID == 0) || (Parameters.ID > MAXPARAMETERS))
    // {
    //     Look1("Parameter error: ID is ");
    //     Look(Parameters.ID);
    //     return 8;
    // }
    LoadParameters();
    LoadRawDataWithParameters();
    DataTosend.ChannelBitMask = 0; //  zero channels to send with this packet
   // DebugParamsOut();
    // Look(ParaNames[Parameters.ID - 1]);
    return 11; //  was 8 is the number of parameters to send
}

/*********************************************************************************************************************************/

void ShowSendingParameters()
{
    char FrontView_Connected[] = "Connected"; // this is the name of the Nextion screen's text box
    char msg[60] = "";
    strcpy(msg, ParaNames[Parameters.ID - 1]);
    strcat(msg, " -> RX");
    if (!LedWasGreen)
        return;
    if (ParametersToBeSentPointer)
        SendText(FrontView_Connected, msg);
}
#endif
