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

//************************************************************************************************************/
void SendStabilationParameters()
{                                        // Send the parameters that are used for stabilisation
    AddParameterstoQueue(BOOLEANS);      // BOOLEANS 9
    AddParameterstoQueue(ALPHA_BETA);    // ALPHA_BETA 8
    AddParameterstoQueue(PID_VALUES);    // PID Values 4
    AddParameterstoQueue(KALMAN_VALUES); // KALMAN Values 5
}
/*********************************************************************************************************************************/
void SendInitialSetupParams()
{
    SendStabilationParameters();              // Send the parameters that are used for stabilisation
    AddParameterstoQueue(SERVO_PULSE_WIDTHS); // Servo Pulse Widths 7
    AddParameterstoQueue(SERVO_FREQUENCIES);  // Servo Frequencies 6
    AddParameterstoQueue(QNH_SETTING);        // QNH 2
    AddParameterstoQueue(FAILSAFE_SETTINGS);  // FailSafeChannels 1
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

// /************************************************************************************************************/
void EncodeFloat(float f, uint16_t *word1, uint16_t *word2, uint16_t *word3, uint16_t *word4)
{ // Encode a float into 4 x 8 bit bytes ignoring the high byte of each 16 BIT word
    union
    {
        float f = 0.0f; // union allows us to access the float as an array of 4 8 bit integers
        uint8_t i[4];
    } u;
    u.f = f;                   // union allows us to access the float as a 4 8 bit integers
    *word1 = uint16_t(u.i[0]); // first 8 bits (The high byte is zeroed in every case because of the compression)
    *word2 = uint16_t(u.i[1]); // second 8 bits
    *word3 = uint16_t(u.i[2]); // third 8 bits
    *word4 = uint16_t(u.i[3]); // fourth 8 bits
}
/************************************************************************************************************/
// NB ONLY THE LOW 12 BITS ARE ACTUALLY SENT! (Because of the compression) so don't use the high bits!

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
    case PID_VALUES: // 4 = PID Values
        EncodeFloat(ActiveSettings->PID_P, &Parameters.word[0], &Parameters.word[1], &Parameters.word[2], &Parameters.word[3]);
        EncodeFloat(ActiveSettings->PID_I, &Parameters.word[4], &Parameters.word[5], &Parameters.word[6], &Parameters.word[7]);
        EncodeFloat(ActiveSettings->PID_D, &Parameters.word[8], &Parameters.word[9], &Parameters.word[10], &Parameters.word[11]);
        break;
    case KALMAN_VALUES: // 5 = Kalman Filter Values
        EncodeFloat(ActiveSettings->Kalman_Q_angle, &Parameters.word[0], &Parameters.word[1], &Parameters.word[2], &Parameters.word[3]);
        EncodeFloat(ActiveSettings->Kalman_Q_bias, &Parameters.word[4], &Parameters.word[5], &Parameters.word[6], &Parameters.word[7]);
        EncodeFloat(ActiveSettings->Kalman_R_measure, &Parameters.word[8], &Parameters.word[9], &Parameters.word[10], &Parameters.word[11]);
        break;
    case SERVO_FREQUENCIES: // 6 = Servo Frequencies
        for (int i = 0; i < 11; ++i)
            Parameters.word[i + 1] = ServoFrequency[i];
        break;
    case SERVO_PULSE_WIDTHS: // 7 = Servo Pulse Widths
        for (int i = 0; i < 11; ++i)
            Parameters.word[i + 1] = ServoCentrePulse[i];
        break;
    case ALPHA_BETA: // 8 = Alpha and Beta
        EncodeFloat(ActiveSettings->alpha, &Parameters.word[0], &Parameters.word[1], &Parameters.word[2], &Parameters.word[3]);
        EncodeFloat(ActiveSettings->beta, &Parameters.word[4], &Parameters.word[5], &Parameters.word[6], &Parameters.word[7]);
        break;
    case BOOLEANS:                                                             // 9 = Booleans
        Parameters.word[1] = (uint16_t)StabilisationOn & 0x01;                 // Stabilisation On
        Parameters.word[2] = (uint16_t)SelfLevellingOn & 0x01;                 // Self Levelling On
        Parameters.word[3] = (uint16_t)ActiveSettings->UseKalmanFilter & 0x01; // Use Kalman Filter
        Parameters.word[4] = (uint16_t)ActiveSettings->UseRateLFP & 0x01;      // Use Rate LFP
        Parameters.word[5] = (uint16_t)ActiveSettings->UseSerialDebug & 0x01;  // Use Serial Debug
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
    for (int i = 0; i < 12; ++i)
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
                                   //  DebugParamsOut();
    // Look(ParaNames[Parameters.ID - 1]);
    return 11; //  was 8 is the number of parameters to send
}

/*********************************************************************************************************************************/

void ShowSendingParameters()
{
    char FrontView_Connected[] = "Connected"; // this is the name of the Nextion screen's text box
    char msg[]= "Sending Parameters ...";
    if (!LedWasGreen)
        return;
    SendText(FrontView_Connected, msg); // show that we are sending parameters
}
// ************************************************************************************************************/
// This function is called when the Self Levelling button is pressed on the Nextion screen.
void SelfLevellingChange()
{
    PlaySound(BEEPMIDDLE); // play a sound to indicate the button was pressed
    ReadStabilisationParameters();
    DisplayStabilisationScreenData();
    PlaySound(BEEPCOMPLETE); // play a sound to indicate the button was pressed
}
#endif
