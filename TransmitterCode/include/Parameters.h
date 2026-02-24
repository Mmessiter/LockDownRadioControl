#include <Arduino.h>
#include "1Definitions.h"
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
            ParametersToBeSent[ParametersToBeSentPointer] = ID;
        }
    }
    // Look1("Queued: ");
    // Look1(ID);
    // Look1(" ");
    // Look(ParaNames[ID - 1]);
}

/*********************************************************************************************************************************/
void SendInitialSetupParams() // This function sends the initial setup parameters.
{                             // Failsafe and stabilisation parameters are NOT sent here because they are sent separately

    AddParameterstoQueue(MSP_BANK_CHANGE);    // for Rotorflight MSP Bank change
    AddParameterstoQueue(MSP_RATES_CHANGE);   // for Rotorflight MSP Bank change
    AddParameterstoQueue(SERVO_PULSE_WIDTHS); // Servo Pulse Widths 7
    AddParameterstoQueue(SERVO_FREQUENCIES);  // Servo Frequencies 6
    AddParameterstoQueue(QNH_SETTING);        // QNH 2
    AddParameterstoQueue(GEAR_RATIO);         // Gear Ratio 8
}
// ****************************************************************************
void EncodeAFloat(float value)
{
    // Encode a float value into the first four words of the parameter structure (we cannot use the high 4 bits)
    union
    {
        float f = 0.0f;
        byte b[4];
    } floatUnion;
    floatUnion.f = value;
    for (int i = 0; i < 4; ++i)
        Parameters.word[i + 1] = (uint16_t)(floatUnion.b[i]);
}

/************************************************************************************************************/
// NB ONLY THE LOW 12 BITS ARE ACTUALLY SENT! (Because of the compression) so don't use the high four BITs!

void LoadOneParameter() // todo: return length of this parameter (avoid using MAX 12 always)
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
    case GEAR_RATIO: // 8 = Gear Ratio
        EncodeAFloat(GearRatio);
        break;
    case SEND_PID_VALUES:
        Parameters.word[1] = 321;               // Please send PID values
        Parameters.word[2] = PID_Send_Duration; // 1000 - how many milliseconds to send these
        break;
    case GET_FIRST_6_PID_VALUES: // 10 = I'm sending first 9 PID values (TX->RX) (Because we cannot fit > 11 in one go)
        for (int i = 0; i < 6; ++i)
        {
            Parameters.word[i + 1] = PID_Values[i]; // 0 to 5
        }
        break;
    case GET_SECOND_11_PID_VALUES:  // 11 = I'm sending second 11 PID values (TX->RX) (Because we cannot fit > 11 in one go)
        for (int i = 0; i < 6; ++i) //
        {
            Parameters.word[i + 1] = PID_Values[i + 6];
        }
        Parameters.word[7] = PID_Boost_Values[0];       // PID_Roll_Boost
        Parameters.word[8] = PID_Boost_Values[1];       // PID_Pitch_Boost
        Parameters.word[9] = PID_Boost_Values[2];       // PID_Yaw_Boost these were tacked on
        Parameters.word[10] = PID_HSI_Offset_Values[0]; // PID_Roll_HSI_Offset
        Parameters.word[11] = PID_HSI_Offset_Values[1]; // PID_Pitch_HSI_Offset
        break;
    case SEND_RATES_VALUES:                       // 12 = Please send RATES values
        Parameters.word[1] = 321;                 // confirms request for RATES values
        Parameters.word[2] = RATES_Send_Duration; // 1000 - how many milliseconds to send these
        break;
    case GET_FIRST_7_RATES_VALUES: // 13 = I'm sending first 7 RATES values (TX->RX) (Because we cannot fit all 12 in one go)
        for (int i = 0; i < 7; ++i)
            Parameters.word[i + 1] = Rate_Values[i]; // 0 to 6
        break;
    case GET_SECOND_6_RATES_VALUES: // 14 = I'm sending second 6 RATES values (TX->RX) (Because we cannot fit all 12 in one go)
        for (int i = 0; i < 6; ++i)
        {
            Parameters.word[i + 1] = Rate_Values[i + 7]; // 7 to 12
        } 
        Parameters.word[7] = Wait_for_Advanced_Rates_to_Be_Sent_Too; // this flag tells the Ack payload parser whether to wait until these have been sent before allowing bank changes again because rates changes can cause problems if a bank change happens while they are being sent
        break;
    case SEND_RATES_ADVANCED_VALUES:                       // 15 = Please send RATES ADVANCED values
        Parameters.word[1] = 321;                          // confirms request for RATES ADVANCED values
        Parameters.word[2] = Rates_Advanced_Send_Duration; // 1000 - how many milliseconds to send these
        break;
    case GET_RATES_ADVANCED_VALUES_SECOND_8: // 16 = I'm sending last 8 RATES ADVANCED values (TX->RX) (Because we cannot fit all 15 in one go)
        for (int i = 0; i < 8; ++i)
            Parameters.word[i + 1] = Rate_Advanced_Values[i + 7]; // 7 to 14
        break;
    case GET_RATES_ADVANCED_VALUES_FIRST_7: // 17 = I'm sending first 7 RATES ADVANCED values (TX->RX) (Because we cannot fit all 15 in one go)
        for (int i = 0; i < 7; ++i)
            Parameters.word[i + 1] = Rate_Advanced_Values[i]; // 0 to 6
        break;
    case SEND_PID_ADVANCED_VALUES:                       // 18 = Please send PID ADVANCED values
        Parameters.word[1] = 321;                        // confirms request for PID ADVANCED values
        Parameters.word[2] = PID_Advanced_Send_Duration; // 1000 - how many milliseconds to send these
        break;
    case GET_FIRST_9_ADVANCED_PID_VALUES: // 19 = I'm sending first 9 ADVANCED PID values (TX->RX) (Because we cannot fit all 26 in one go)
        for (int i = 0; i < 9; ++i)
            Parameters.word[i + 1] = PID_Advanced_Values[i]; // 0 to 8
        break;
    case GET_SECOND_9_ADVANCED_PID_VALUES: // 20 = I'm sending second 9 ADVANCED PID values (TX->RX) (Because we cannot fit all 26 in one go)
        for (int i = 0; i < 9; ++i)
            Parameters.word[i + 1] = PID_Advanced_Values[i + 9]; // 9 to 17
        break;
    case GET_THIRD_8_ADVANCED_PID_VALUES: // 21 = I'm sending last 8 ADVANCED PID values (TX->RX) (Because we cannot fit all 26 in one go)
        for (int i = 0; i < 8; ++i)
            Parameters.word[i + 1] = PID_Advanced_Values[i + 18]; // 18 to 25
        break;
    case MSP_BANK_CHANGE: // 22 = I'm sending the current bank number to the RX (TX->RX)
        Parameters.word[1] = 1;
        Parameters.word[2] = Bank; // 0 to 3 new bank number
        break;
    case MSP_RATES_CHANGE: // 23 = I'm sending the current rates number to the RX (TX->RX)
        Parameters.word[1] = 1;
        Parameters.word[2] = DualRateInUse | 0x80; // 0 to 3 new rates number RATES = BANK | 128 for now....
        break;
        // case MSP_BANK_CHANGE_CONFIRMATION:      // 24 = I'm confirming that the bank change was successful (TX->RX)
        //     Parameters.word[1] = DualRateInUse; // might be used later for rates change confirmation
        //     Parameters.word[2] = Bank;          // 0 to 3 new bank number
        //     break;

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
    LoadOneParameter();
    LoadRawDataWithParameters();
    DataTosend.ChannelBitMask = 0; // IMPORTANT! This flag stops these data being seen as channel data at the RX!
                                   //  DebugParamsOut();              // long
    //  Look1(Parameters.ID);
    //  Look1(" ");
    //  Look(ParaNames[Parameters.ID - 1]); // brief

    return 12; //  was 8 - this is the max extent of a parameter
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
/************************************************************************************************************/
void ActuallySendParameters(uint32_t RightNow)
{
    static uint32_t LastParameterSent = 0;

    if (RightNow - LedGreenMoment < PAUSE_BEFORE_PARAMETER_SEND) // wait a little before sending parameters to allow the RX to Bind
        return;

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
