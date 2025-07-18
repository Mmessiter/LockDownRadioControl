#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef PARAMETERS_H
#define PARAMETERS_H

/*********************************************************************************************************************************/

void AddParameterstoQueue(uint8_t ID) // this queue is essentially a LIFO stack
{
    if (!ModelMatched || !BoundFlag || ID == 0)
        return;
    for (int i = 0; i < PARAMETER_SEND_REPEATS; ++i)
    {
        if (ParametersToBeSentPointer < PARAMETER_QUEUE_MAXIMUM)
        {
            ++ParametersToBeSentPointer;
            ParametersToBeSent[ParametersToBeSentPointer] = ID; // heer
        }
    }
    // Look1("Queued: ");
    // Look1(ID);
    // Look1(" ");
    // Look(ParaNames[ID - 1]);
}

//************************************************************************************************************/

void SwitchLevelling(bool OnOff) // This function switches levelling on or off by bank selected
{
#ifdef USE_STABILISATION // Switch levelling on or off

    static bool PreviousSelfLevellingOn; // This is the previous state of the levelling
    char sw4[] = "sw4";                  // Switch 4 is used for levelling on/off
    SelfLevellingOn = OnOff;

    if (SelfLevellingOn != PreviousSelfLevellingOn) // if the state hasn't changed then just return
    {
        SendStabilationParameters();
        PreviousSelfLevellingOn = SelfLevellingOn;
    }
    if (CurrentView == PIDVIEW)
        SendValue(sw4, SelfLevellingOn); // Send the value to the PID view
#endif
}

//************************************************************************************************************/

void SwitchStabilisation(bool OnOff) // This function switches stabilisation on or off by bank selected
{
#ifdef USE_STABILISATION                 // Switch stabilisation on or off
    static bool PreviousStabilisationOn; // This is the state of the stabilisation switch
    char sw0[] = "sw0";                  // Switch 0 is used for stabilisation on/off
    StabilisationOn = OnOff;
    if (PreviousStabilisationOn != StabilisationOn) // if the state hasn't changed then just return
    {
        SendStabilationParameters();
        PreviousStabilisationOn = StabilisationOn; // remember the state of the stabilisation switch
    }
    if (CurrentView == PIDVIEW)
        SendValue(sw0, StabilisationOn); // Send the value to the PID view
#endif
}
// *******************************************************************************************************************************/
// This function is called from main.cpp to check the state of the stabilisation and self-
void CheckStabilisationAndSelf_levelling()
{
#ifdef USE_STABILISATION
    SwitchStabilisation(Bank == StabilisedBank || StabilisedBank == 0);
    SwitchLevelling(Bank == LevelledBank || LevelledBank == 0);
#endif
}

//************************************************************************************************************/
void SendStabilationParameters() // This function sends the parameters that are used for stabilisation
{
#ifdef USE_STABILISATION                   // Send the parameters that are used for stabilisation
    AddParameterstoQueue(BOOLEANS);        // BOOLEANS 9
    AddParameterstoQueue(ALPHA_BETA);      // ALPHA_BETA 8
    AddParameterstoQueue(PID_VALUES);      // PID Values 4
    AddParameterstoQueue(TAIL_PID_VALUES); // TAIL PID Values 11
    AddParameterstoQueue(KALMAN_VALUES);   // KALMAN Values 5
#endif
}

/*********************************************************************************************************************************/
void SendInitialSetupParams()                 // This function sends the initial setup parameters.
{                                             // Failsafe and stabilisation parameters are NOT sent here because they are sent separately
    SendStabilationParameters();              // Send the parameters that are used for stabilisation ... if its enabled
    AddParameterstoQueue(SERVO_PULSE_WIDTHS); // Servo Pulse Widths 7
    AddParameterstoQueue(SERVO_FREQUENCIES);  // Servo Frequencies 6
    AddParameterstoQueue(QNH_SETTING);        // QNH 2
}
// /************************************************************************************************************/
#ifdef USE_STABILISATION // Switch stabilisation on or off
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
#endif // USE_STABILISATION
/************************************************************************************************************/
// NB ONLY THE LOW 12 BITS ARE ACTUALLY SENT! (Because of the compression) so don't use the high bits!

void LoadParameters()
{
    uint16_t Twobytes = 0;
    uint8_t FS_Byte1;
    uint8_t FS_Byte2;

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

    case SERVO_FREQUENCIES: // 6 = Servo Frequencies
        for (int i = 0; i < 11; ++i)
            Parameters.word[i + 1] = ServoFrequency[i];
        break;
    case SERVO_PULSE_WIDTHS: // 7 = Servo Pulse Widths
        for (int i = 0; i < 11; ++i)
            Parameters.word[i + 1] = ServoCentrePulse[i];
        break;

#ifdef USE_STABILISATION
    case PID_VALUES: // 4 = PID Values
        EncodeFloat(ActiveSettings->PID_P[Bank - 1], &Parameters.word[0], &Parameters.word[1], &Parameters.word[2], &Parameters.word[3]);
        EncodeFloat(ActiveSettings->PID_I[Bank - 1], &Parameters.word[4], &Parameters.word[5], &Parameters.word[6], &Parameters.word[7]);
        EncodeFloat(ActiveSettings->PID_D[Bank - 1], &Parameters.word[8], &Parameters.word[9], &Parameters.word[10], &Parameters.word[11]);
        break;
    case KALMAN_VALUES: // 5 = Kalman Filter Values
        EncodeFloat(ActiveSettings->Kalman_Q_angle, &Parameters.word[0], &Parameters.word[1], &Parameters.word[2], &Parameters.word[3]);
        EncodeFloat(ActiveSettings->Kalman_Q_bias, &Parameters.word[4], &Parameters.word[5], &Parameters.word[6], &Parameters.word[7]);
        EncodeFloat(ActiveSettings->Kalman_R_measure, &Parameters.word[8], &Parameters.word[9], &Parameters.word[10], &Parameters.word[11]);
        break;
    case ALPHA_BETA: // 8 = Alpha and Beta
        EncodeFloat(ActiveSettings->alpha, &Parameters.word[0], &Parameters.word[1], &Parameters.word[2], &Parameters.word[3]);
        EncodeFloat(ActiveSettings->beta, &Parameters.word[4], &Parameters.word[5], &Parameters.word[6], &Parameters.word[7]);
        break;
    case BOOLEANS:                                                             // 9 = Booleans
        Parameters.word[1] = (uint16_t)StabilisationOn & 0x01;                 // Stabilisation On
        Parameters.word[2] = (uint16_t)SelfLevellingOn & 0x01;                 // Self Levelling On
        Parameters.word[3] = (uint16_t)ActiveSettings->UseKalmanFilter & 0x01; // Use Kalman Filter
        Parameters.word[4] = (uint16_t)ActiveSettings->UseRateLPF & 0x01;      // Use Rate LPF
        break;
    case RECALIBRATE_MPU6050: // 10 = Recalibrate MPU6050
        Parameters.word[1] = 42;
        Parameters.word[2] = 42;
        break;
    case TAIL_PID_VALUES: // 11 = Tail PID Values
        EncodeFloat(ActiveSettings->Tail_PID_P[Bank - 1], &Parameters.word[0], &Parameters.word[1], &Parameters.word[2], &Parameters.word[3]);
        EncodeFloat(ActiveSettings->Tail_PID_I[Bank - 1], &Parameters.word[4], &Parameters.word[5], &Parameters.word[6], &Parameters.word[7]);
        EncodeFloat(ActiveSettings->Tail_PID_D[Bank - 1], &Parameters.word[8], &Parameters.word[9], &Parameters.word[10], &Parameters.word[11]);
        break;

#endif // USE_STABILISATION
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
{                        // Only the >>LOW 12 BITS<< of each parameter are actually sent because of compression which loses the highest 4 bits

    Parameters.ID = ParametersToBeSent[ParametersToBeSentPointer];
    if ((Parameters.ID == 0) || (Parameters.ID > PARAMETERS_MAX_ID))
    {
        Look1("Parameter error: ID is ");
        Look(Parameters.ID);
        DataTosend.ChannelBitMask = 0; // IMPORTANT! This flag stops these data being seen as channel data at the RX!
        return 8;
    }
    LoadParameters();
    LoadRawDataWithParameters();
    DataTosend.ChannelBitMask = 0; // IMPORTANT! This flag stops these data being seen as channel data at the RX!
                                   // DebugParamsOut();              // long
    Look1(Parameters.ID);
    Look1(" ");
    Look(ParaNames[Parameters.ID - 1]); // brief

    return 12; //  was 8 - this is the extent of this parameter
}

/*********************************************************************************************************************************/

void ShowSendingParameters()
{
    char FrontView_Connected[] = "Connected"; // this is the name of the Nextion screen's text box
    char msg[] = "Sending Parameters ...";
    if (!LedWasGreen || (millis() - LedGreenMoment) > 5000) // if no connection or connection isn't new any more then return
        return;
    SendText(FrontView_Connected, msg); // show that we are sending parameters
}
// ************************************************************************************************************/
// This function is called when the Self Levelling button is pressed on the Nextion screen.
void SelfLevellingChange()
{
#ifdef USE_STABILISATION                // Switch stabilisation on or off
    ReadPIDScreenData();                // in case the values were changed on the screen
    SelfLevellingOn = !SelfLevellingOn; // toggle the self-levelling on/off state
    DisplayPIDScreenData();             // show the new values on the screen (having changed the SelfLevellingOn value)
#endif                                  // USE_STABILISATION
}
//************************************************************************************************************/
void KFactoryDefaultsPlane()
{
#ifdef USE_STABILISATION
    if (GetConfirmation(pKalmanView, (char *)"Re-Load factory plane defaults?!"))
    {
        ActiveSettings = &RateSettings; // set the active settings to the rate settings
        RateSettings = FactoryPlaneRate;
        SelfLevelSettings = FactoryPlaneLevelling;
        SelfLevellingOn = false;                   // defaults are always without self-levelling
        SendValue((char *)"sw4", SelfLevellingOn); // heer
        DisplayKalmanScreenData();
    }
#endif // USE_STABILISATION
}
//************************************************************************************************************/
void FactoryDefaultsPlane()
{
#ifdef USE_STABILISATION
    if (GetConfirmation(pPIDView, (char *)"Re-Load factory plane defaults?!"))
    {
        ActiveSettings = &RateSettings; // set the active settings to the rate settings
        RateSettings = FactoryPlaneRate;
        SelfLevelSettings = FactoryPlaneLevelling;
        SelfLevellingOn = false;                   // defaults are always without self-levelling
        SendValue((char *)"sw4", SelfLevellingOn); // heer
        DisplayPIDScreenData();
    }
#endif // USE_STABILISATION
}
// /************************************************************************************************************/
void KFactoryDefaultsHeli()
{
#ifdef USE_STABILISATION
    if (GetConfirmation(pKalmanView, (char *)"Re-Load factory heli defaults?!"))
    {
        ActiveSettings = &RateSettings; // set the active settings to the rate settings
        RateSettings = FactoryHeliRate;
        SelfLevelSettings = FactoryHeliLevelling;
        SelfLevellingOn = false;                   // defaults are always without self-levelling
        SendValue((char *)"sw4", SelfLevellingOn); // heer
        DisplayKalmanScreenData();
    }
#endif // USE_STABILISATION
}
//************************************************************************************************************/

void FactoryDefaultsHeli()
{
#ifdef USE_STABILISATION
    if (GetConfirmation(pPIDView, (char *)"Re-Load factory heli defaults?!"))
    {
        ActiveSettings = &RateSettings; // set the active settings to the rate settings
        RateSettings = FactoryHeliRate;
        SelfLevelSettings = FactoryHeliLevelling;
        SelfLevellingOn = false;                   // defaults are always without self-levelling
        SendValue((char *)"sw4", SelfLevellingOn); // heer
        DisplayPIDScreenData();
    }
#endif // USE_STABILISATION
}

/************************************************************************************************************/
void ActuallySendParameters(uint32_t RightNow)
{
    static uint32_t LastParameterSent = 0;

    if (RightNow - LedGreenMoment < PAUSE_BEFORE_PARAMETER_SEND) // wait a little before sending parameters to allow the RX to Bind
    {
       // Look(LedGreenMoment);
        return;
    }

    ShowSendingParameters();
    if (RightNow - LastParameterSent >= PARAMETER_SEND_FREQUENCY)
    {
        ParamPause = false; // Reset pause flag to allow the parameters to be sent
        LastParameterSent = RightNow;
    }
    else if (RightNow - LastParameterSent >= PARAMETER_SEND_DURATION) // it would just keep sending parameters so we must pause it for a while
    {
        ParamPause = true; // Pause sending parameters briefly so we can send data to control the model ! :-)
        LastConnectionQuality = 0;
    }
}
#endif
