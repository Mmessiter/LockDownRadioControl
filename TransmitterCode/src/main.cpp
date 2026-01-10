/** @file TransmitterCode/src/main.cpp
 * // Malcolm Messiter 2020 - 2025
 * @brief This is the main file for the transmitter code.
 *
 * @page TransmitterCode....
 * @section LockDown Radio Control Features list, so far:
 * - Uses Teensy 4.1 MCU (at 600 Mhz) with nRF24L01+ transceiver
 * - (Ebyte's ML01DP5 recommended for TX, two ML01SP4s for RX.)
 * - 16 channels
 * - 12 BIT servo resolution (11 BIT via SBUS)
 * - 32 Mixes
 * - 4 Flight modes with user definable and announced names.
 * - 90 Model memories.
 * - Unlimited model files for backup (32 Gig)
 * - "ModelMatch" plus automatic model memory selection. (Avoid flying with wrong memory loaded.)
 * - "ModelExchange" - share model memories wirelessly.
 * - Wireless or wired Buddyboxing with selectable channels.
 * - Voice messages and other audio prompts.
 * - User defined Channel names
 * - 2.4 Ghz FHSS ISM band licence free in UK and most other countries.
 * - > 2 Km range
 * - Telemetry including GPS, volts, temperature & barometric pressure.
 * - 64 editable 5-point curves (16 channels x 4 flight modes): straight, smoothed or exponential.
 * - FailSafe on any or all channel(s)
 * - 2.4 GHz RF scan
 * - Motor Timer
 * - Lossless data compression.
 * - Trims saved per bank and per model.
 * - Screen timeout to save battery.
 * - FHSS with very fast connect and reconnect
 * - Uses 32 GIG SD card for model memories and help files
 * - Binding - uses unique Mac address as pipe address.
 * - Four User definable three position switches
 * - Channels 5,6,7 & 8 can be switches or knobs.
 * - Input sources definable - any stick, switch or knob can be mapped to any function.
 * - Output channels definable - any channel can be mapped to any output channels
 * - Model memories export and import to backup files on SD card
 * - Model memory files alphabetically sorted
 * - Timer goes on and off with motor to keep track of motor use.
 * - DS1307 Real time clock auto synched to exact GPS time.
 * - Model memory sharing between transmitters - wirelessly.
 * - Sub-trim
 * - Servo reverse
 * - Macros - for snap rolls, heli rescue, etc.
 * - Hardware digital trims with accellerating repeat
 * - Capacitive touch screen GUI
 * - Hardware bug in nRF24L01+ worked-around: FIFO buffers crash the chip when they're full for > 4ms. So these are flushed often.
 * - Screen colours definable
 * - Data screen gives almost all possible telemetry
 * - Log files  (on SD 32 gig card)
 * - Help files display system from plain text files.
 * - Safety switch implemtented (Stops accidental motor starting)
 * - Rates (three rates) implemented
 * - Slow Servos implemented: Any channel can be slowed by almost any amount for realistic flaps, U/C etc,
 * - Support for external third party transmitter modules added (JR type).
 * - Variometer implemented (using a BMP280 or better still, a DPS310)
 *
 *
 * | Teensy 4.1 Pins | Connections |
 * |-----------------|-------------|
 * | GND        | GND |
 * | Vin        | +5V power in  |
 * | 0  (RX1)   | NEXTION  (TX) | // RX1
 * | 1  (TX2)   | NEXTION  (RX) | // TX1
 * | 2  LED     | RED |
 * | 3  LED     | GREEN |
 * | 4  LED     | BLUE |
 * | 5  (POLOLU)| 2808 ALL POWER OFF SIGNAL (When high)|
 **| 6  SPARE   | <<<< *SPARE* (was PPM TX MODULE <<<<<<<<<<<<<<<
 * | 7  (CE)    | nRF24l01 (CE)   // RX2
 * | 8  (CSN)   | nRF24l01 (CSN)  // TX2
 **| 9  SPARE   | <<<< *SPARE* !
 **| 10 SPARE   | <<<< *SPARE* ! (WAS : Wired Buddy PPM IN or OUT (Could become a SPARE if PPM Buddy is not used)
 * | 11 (MOSI)  | nRF24l01 (MOSI) |
 * | 12 (MISO)  | nRF24l01 (MISO) |
 * | 13 (SCK)   | nRF24l01 (SCK) |
 * | 14 (A0)    | Joystick POT CH1 |
 * | 15 (A1)    | Joystick POT CH2 |
 * | 16 (A2)    | Joystick POT CH3 |
 * | 17 (A3)    | Joystick POT CH4 |
 * | 18 (I2C)   | I2C bus  SDA |
 * | 19 (I2C)   | I2C bus  SCL |
 * | 20 (A6)    | POT KNOB CH5 |// TX5
 * | 21 (A7)    | POT KNOB CH6 |// RX5
 * | 22 (A8)    | POT KNOB CH7 |
 * | 23 (A9)    | POT KNOB CH8 |
 **| 24 (A10)   | Spare  ADC!
 * | 26 (A12)   | Switch 1 |
 * | 27 (A13)   | Switch 2 |
 * | 28         | Switch 2 |    // RX7
 * | 29         | Switch 3 |    // TX7
 * | 30         | Switch 3 |
 * | 31         | Switch 4 |
 * | 32         | Switch 4 |
 * | 33         | Sensor for power button press while on
 * | 34         |TRIM (CH1a)|   // RX8  ********  Usable as serial while hardware trims are not used  (Best option)
 * | 35         |TRIM (CH1b)|   // TX8  ********  Usable as serial while hardware trims are not used  (Best option)
 * | 36         |TRIM (CH2a)|
 * | 37         |TRIM (CH2b)|
 * | 38 (A14)   |TRIM (CH3a)|
 * | 39 (A15)   |TRIM (CH3b)|
 * | 40 (A16)   |TRIM (CH4a)|
 * | 41 (A17)   |TRIM (CH4b)|
 * | 42         Built-in SD card
 * | 43         Built-in SD card
 * | 44         Built-in SD card
 * | 45         Built-in SD card
 * | 46         Built-in SD card
 * | 47         Built-in SD card
 * |  Solder pads on the back:
 * | 48<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (SPARE)
 * | 49<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (SPARE)
 * | 50<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (SPARE)
 * | 51<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (SPARE)
 * | 52<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (SPARE)
 * | 53<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (SPARE)
 * | 54<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (SPARE)
 *
 *
 *  (11 spare GPIO PINS: 6,9,10,24,49,50,51,52,53,54 )
 *
 */
// ************************************************** TRANSMITTER CODE **************************************************

#include <Arduino.h>
#include <Watchdog_t4.h>
#include <PulsePosition.h>
#include <RF24.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <DS1307RTC.h>
#include <InterpolationLib.h>

#include "1Definitions.h" // must be here! (not higher or lower);

#include "Utilities.h"
#include "transceiver.h"
#include "ZPong.h"
#include "macros.h"
#include "Trims.h"
#include "ModelMatch.h"
#include "Nextion.h"
#include "ModelExchange.h"
#include "BuddyWireless.h"
#include "SDcard.h"
#include "Telemetry.h"
#include "LogFiles.h"
#include "DualRates.h"
#include "Mixes.h"
#include "MenuOptions.h"
#include "LogFilesList.h"
#include "Help.h"
#include "LogFilesDisplay.h"
#include "Switches.h"
#include "ADC-master/ADC.h"
#include "Parameters.h"
#include "PIDView.h"
#include "RatesRF.h"
#include "RotorFlight.h"
#include "RatesRF_Advanced.h"
#include "PID_Advanced.h"

/*********************************************************************************************************************************/

void ClearMostParameters()
{ // called from RED LED ON
    RXVoltsDetected = false;
    LedGreenMoment = 0;
    ModelsMacUnion.Val64 = 0;
    LedWasGreen = false;
    ModelIdentified = false;
    ModelMatched = false;
    BoundFlag = false;
    LastShowTime = 0;
    TotalGoodPackets = 0;
    RecentGoodPacketsCount = 0;
    RecentPacketsLost = 0;
    DontChangePipeAddress = false;
    UsingDefaultPipeAddress = true;
    VersionsCompared = false;
    LogLineNumber = 0;
    RXSuccessfulPackets = 0;
    TotalLostPackets = 0;
    TotalPacketsAttempted = 0;
    ModelMatchFailed = false;

    strcpy(LogFileName, "");
    for (int i = 0; i < CHANNELSUSED; ++i)
    {
        PreviousBuffer[i] = 0;
        SendBuffer[i] = 0;
        BuddyBuffer[i] = 0;
        ShownBuffer[i] = 0;
        RawDataBuffer[i] = 0;
        LastBuffer[i] = 0;
        PreMixBuffer[i] = 0;
    }
}
// *********************************************************************************************************************************/
void EnsureMotorIsOff()
{
    if (!UseMotorKill)
        return;

    CheckMotorOff();
    while (MotorEnabled) // disconnected now so stay here until motor switch really is off!
    {
        SendCommand(WarnNow);
        SendText(Warning, err_MotorOn);
        SendNoData = true;
        PlaySound(MOTORON);
        DelayWithDog(1200);
        PlaySound(PLSTURNOFF);
        DelayWithDog(3000);
        CheckMotorOff();
    }
    SendCommand(WarnOff);
    SendNoData = false;
}
/*********************************************************************************************************************************/

void RedLedOn()
{
    analogWrite(GREENLED, 0);
    analogWrite(BLUELED, 0);
    analogWrite(REDLED, GetLEDBrightness()); // Brightness is a function of maybe blinking
    if (LedWasGreen)
    {
        if (AnnounceConnected & !BuddyPupilOnWireless)
            PlaySound(DISCONNECTEDMSG);
        if (UseLog)
            LogDisConnection();
        if (CurrentView == FRONTVIEW)
        {
            SendText((char *)"Connected", na);
            SendCommand(WarnOff);
            SendCommand((char *)"vis Quality,0");
        }
    }
    SendCommand((char *)"vis rpm,0"); // This will make the RPM display invisible
    SendCommand((char *)"vis StillConnected,0");
    if (!BindingEnabled)
        SendCommand((char *)"vis wb,0");
    SendText((char *)"Owner", TxName); // Put owner name back
    First_RPM_Data = true;             // ready to start RPM data ...
    ClearMostParameters();
    LedWasRed = true;
    EnsureMotorIsOff();
}

/*********************************************************************************************************************************/

void BlueLedOn()
{
    LedWasGreen = false;
    LedWasRed = false;
    analogWrite(REDLED, 0);
    analogWrite(GREENLED, 0);
    analogWrite(BLUELED, GetLEDBrightness()); // Brightness is a function of maybe blinking
}

/*********************************************************************************************************************************/

void GreenLedOn()
{
    if (!ModelMatched || UsingDefaultPipeAddress)
        return; // no green led for wrong model
    if (!LedWasGreen)
    {
        SendCommand((char *)"vis wb,0"); // Hide the binding button
        BindingEnabled = false;          // Disable new binding after successful bind
        LedGreenMoment = millis();
        LastShowTime = 0;
        ShowComms();
        if (AnnounceConnected)
        {
            PlaySound(CONNECTEDMSG);
            if (Connect_MMmsg) // this is set when Modelmatch is executed during binding.
            {
                DelaySimple(1000);        // wait a second before playing the message
                PlaySound(Connect_MMmsg); // play the message that was set during ModelMatch  ("Model Found!" or "Model matched!" or "Model not found!")
                Connect_MMmsg = 0;        // reset the message to be played later
            }
        }
        if (UseLog)
        {
            LogConnection();
        }

        analogWrite(BLUELED, 0);
        analogWrite(REDLED, 0);
        analogWrite(GREENLED, GetLEDBrightness()); // Brightness is a function of maybe blinking
        LedWasGreen = true;
        LedWasRed = false;
        Reconnected = false;
        ForceVoltDisplay = true; // Force a battery check of the models battery
        SendInitialSetupParams();
        ResetMotorTimer();
        TotalPacketsAttempted = 0;
        strcpy(LogFileName, ""); // avoid logging to the wrong file
    }
    else
    {
        if (LedIsBlinking)
            analogWrite(GREENLED, GetLEDBrightness()); // Blink Led!
    }
}

/*********************************************************************************************************************************/

FASTRUN void ShowMotorTimer()
{
    char FrontView_Hours[] = "Hours";
    char FrontView_Mins[] = "Mins";
    char FrontView_Secs[] = "Secs";

    if (TimesUp)
        return;

    uint8_t Recording[10] = {ONEMINUTE, TWOMINUTES, THREEMINUTES, FOURMINUTES, FIVEMINUTES, SIXMINUTES, SEVENMINUTES, EIGHTMINUTES, NINEMINUTES, TENMINUTES};

    uint8_t Cdown[10] = {TEN, NINE, EIGHT, SEVEN, SIX, FIVE, FOUR, THREE, TWO, ONE};

    if ((MotorEnabled && (!LostContactFlag)))
    {
        MotorOnSeconds = ((millis() - MotorStartTime) / 1000) + PausedSecs;
        Secs = MotorOnSeconds;
        if (TimerDownwards)
            Secs = TimerStartTime - MotorOnSeconds;
        Hours = Secs / 3600;
        Secs %= 3600;
        Mins = Secs / 60;
        Secs %= 60;
    }
    if (LastSeconds != Secs)
    {

        ClockSpoken = false;
        ClockSpoken1 = false;
        if (CurrentView == FRONTVIEW)
        {
            SendValue(FrontView_Secs, Secs);
            SendValue(FrontView_Mins, Mins);
            SendValue(FrontView_Hours, Hours);
        }
        LastSeconds = Secs;
    }

    if (TimerDownwards)
    {
        if ((Secs < 11) && !Mins && !ClockSpoken1 && (MotorOnSeconds > 2))
        {
            PlaySound(Cdown[CountDownIndex]);
            ++CountDownIndex;
            ClockSpoken1 = true;
        }
    }

    if (!Secs && SpeakingClock && !ClockSpoken)
    {
        ClockSpoken = true;
        if ((Mins <= 10) && (Mins > 0))
        {
            PlaySound(Recording[Mins - 1]);
        }
        if (Mins)
        {
            if (UseLog)
            {
                LogTimer(Mins);
                LogRPM(RotorRPM);
            }
        }
        if (TimerDownwards)
        {
            if ((!Mins) && (!Secs) && (MotorOnSeconds > 2))
            {
                PlaySound(STORAGECHARGE); // = Stop Flying!
                TimesUp = true;
                CountDownIndex = 0;
            }
        }
    }
}

/*********************************************************************************************************************************/

FASTRUN void ShowServoPos()
{
    uint8_t MinimumDistance = 3; // if the change is small, don't re-display anything - to reduce flashing. :=)!!
    uint32_t Hertz = 25;         // Fast
    if (CurrentView == GRAPHVIEW)
        Hertz = 40; // Faster update rate for graph (was 200)
    if (millis() - ShowServoTimer < Hertz)
        return;
    ShowServoTimer = millis();

    char Ch_Lables[16][5] = {"Ch1", "Ch2", "Ch3", "Ch4", "Ch5", "Ch6", "Ch7", "Ch8", "Ch9", "Ch10", "Ch11", "Ch12", "Ch13", "Ch14", "Ch15", "Ch16"};
    char ChannelInput[] = "Input";
    char ChannelOutput[] = "Output";
    uint16_t StickPosition = 54321;
    int InputDevice = 0;
    int InputAmount = 0;
    int OutputAmount = 0;

    // The first 8 channels are displayed on all three of these screens
    if ((CurrentView == STICKSVIEW) || (CurrentView == FRONTVIEW) || (CurrentView == CALIBRATEVIEW))
    {
        for (int i = 0; i < 8; ++i)
        {
            if (ChannelOutPut[i] > 3)
                MinimumDistance = 40; // coarse for pots & switches...
            else
                MinimumDistance = 3; // ...finer for sticks
            if (abs(SendBuffer[i] - ShownBuffer[i]) > MinimumDistance)
            { // no need to show tiny movements
                SendValue(Ch_Lables[i], IntoLowerRes(SendBuffer[i]));
                ShownBuffer[i] = SendBuffer[i];
            }
        }
    }
    MinimumDistance = 6;
    // The second 8 channels are displayed on only two screens
    if ((CurrentView == STICKSVIEW) || (CurrentView == FRONTVIEW))
    {
        for (int i = 8; i < 16; ++i)
        {
            if (abs(SendBuffer[i] - ShownBuffer[i]) > MinimumDistance)
            {
                SendValue(Ch_Lables[i], IntoLowerRes(SendBuffer[i]));
                ShownBuffer[i] = SendBuffer[i];
            }
        }
    }
    if ((CurrentView == GRAPHVIEW))
    {
        MinimumDistance = 4;
        InputDevice = (InPutStick[ChanneltoSet - 1]);
        if (InputDevice < 8)
            InputAmount = AnalogueReed(InputDevice);
        else
            InputAmount = ReadThreePositionSwitch(InputDevice);                                                   // not analogue
        InputAmount = map(InputAmount, ChannelCentre[InputDevice], ChannelMax[InputDevice], 0, 100);              // input stick position
        OutputAmount = map(SendBuffer[InputDevice], MINMICROS, MAXMICROS, -100, 100);                             // output servo position
        SendValue(ChannelInput, InputAmount);                                                                     // input stick position
        SendValue(ChannelOutput, OutputAmount);                                                                   // output servo position
        StickPosition = map(constrain((InputAmount + 100) / 2, 0, 100), 0, 100, BOXLEFT + 2, BOXRIGHT - BOXLEFT); // map to box size
        StickPosition = constrain(StickPosition, BOXLEFT + 2, (BOXRIGHT - BOXLEFT) - 1);                          // not outside box!

        // Only redraw if the position has changed enough
        if ((abs(StickPosition - SavedLineX) > MinimumDistance))
        {
            // Simply do a full redraw - this is easiest and most reliable
            DisplayCurve();
            // Then draw vertical line for stick position on top
            DrawLine(StickPosition, BOXTOP + 3, StickPosition, (BOXBOTTOM - 3) - BOXTOP, HighlightColour);
            SavedLineX = StickPosition;
        }
    }
}

/*********************************************************************************************************************************/
uint16_t CatmullSplineInterpolation(uint16_t InputValue, uint16_t InputChannel, uint16_t OutputChannel)
{
    xPoints[0] = ChannelMin[InputChannel];
    xPoints[1] = ChannelMidLow[InputChannel];
    xPoints[2] = ChannelCentre[InputChannel];
    xPoints[3] = ChannelMidHi[InputChannel];
    xPoints[4] = ChannelMax[InputChannel];
    yPoints[4] = IntoHigherRes(CurveDots[4]);
    yPoints[3] = IntoHigherRes(CurveDots[3]);
    yPoints[2] = IntoHigherRes(CurveDots[2]);
    yPoints[1] = IntoHigherRes(CurveDots[1]);
    yPoints[0] = IntoHigherRes(CurveDots[0]);
    return Interpolation::CatmullSpline(xPoints, yPoints, PointsCount, InputValue);
}
/*********************************************************************************************************************************/
uint16_t StraightLineInterpolation(uint16_t InputValue, uint16_t InputChannel, uint16_t OutputChannel)
{
    uint16_t k = 0;
    if (InputValue >= ChannelMidHi[InputChannel])
    {
        k = map(InputValue, ChannelMidHi[InputChannel], ChannelMax[InputChannel], IntoHigherRes(CurveDots[3]), IntoHigherRes(CurveDots[4]));
    }

    if (InputValue >= ChannelCentre[InputChannel] && InputValue <= (ChannelMidHi[InputChannel]))
    {
        k = map(InputValue, ChannelCentre[InputChannel], ChannelMidHi[InputChannel], IntoHigherRes(CurveDots[2]), IntoHigherRes(CurveDots[3]));
    }

    if (InputValue >= ChannelMidLow[InputChannel] && InputValue <= ChannelCentre[InputChannel])
    {
        k = map(InputValue, ChannelMidLow[InputChannel], ChannelCentre[InputChannel], IntoHigherRes(CurveDots[1]), IntoHigherRes(CurveDots[2]));
    }

    if (InputValue <= ChannelMidLow[InputChannel])
    {
        k = map(InputValue, ChannelMin[InputChannel], ChannelMidLow[InputChannel], IntoHigherRes(CurveDots[0]), IntoHigherRes(CurveDots[1]));
    }

    return k;
}
/*********************************************************************************************************************************/
uint16_t ExponentialInterpolation(uint16_t InputValue, uint16_t InputChannel, uint16_t OutputChannel)
{
    uint16_t k = 0;
    if (InputValue >= ChannelCentre[InputChannel])
    {
        k = MapWithExponential(InputValue - ChannelCentre[InputChannel], 0, ChannelMax[InputChannel] - ChannelCentre[InputChannel], 0, IntoHigherRes(CurveDots[4]) - IntoHigherRes(CurveDots[2]), Exponential[Bank][OutputChannel]) + IntoHigherRes(CurveDots[2]);
    }
    else
    {
        k = MapWithExponential(ChannelCentre[InputChannel] - InputValue, 0, ChannelCentre[InputChannel] - ChannelMin[InputChannel], IntoHigherRes(CurveDots[2]) - IntoHigherRes(CurveDots[0]), 0, Exponential[Bank][OutputChannel]) + IntoHigherRes(CurveDots[0]);
    }
    return k;
}

/*********************************************************************************************************************************/
// ************* Small function pointer array for interpolation types ************************************************************

uint16_t (*Interpolate[3])(uint16_t InputValue, uint16_t InputChannel, uint16_t OutputChannel){
    StraightLineInterpolation,  // 0
    CatmullSplineInterpolation, // 1
    ExponentialInterpolation    // 2
};

/*********************************************************************************************************************************/
/**************************** This function implements slowed servos for flaps, U/Cs etc. ****************************************/
/*********************************************************************************************************************************/

void SlowAnyServos() // This function implements slowed servos for flaps, U/Cs etc.
{
    static uint32_t SlowTime[16]; // TODO: Make it BY BANK and not every bank <<<<<<<<<<<<<<*****************************
    for (int i = 0; i < 16; ++i)
    { // Test every channel
        if (ServoSpeed[Bank - 1][i] < 100)
        { // If ServoSpeed = 100, use full speed. No slowing
            if ((millis() - SlowTime[i]) > 10)
            {                           // This next part runs only 100 times per second
                SlowTime[i] = millis(); // Store start time of this iteration
                if (CurrentPosition[i] == 0)
                    CurrentPosition[i] = SendBuffer[i];            // Must start somewhere
                int distance = SendBuffer[i] - CurrentPosition[i]; // Define how far to move
                int SSize = ServoSpeed[Bank - 1][i];               // Get step size
                if (SSize > abs(distance))
                    SSize = 1; // This avoids overshooting the limit
                if (distance < 0)
                    SSize = -SSize; // Negative?
                if (!distance)
                    SSize = 0;               // Already arrived?
                CurrentPosition[i] += SSize; // Move Current Position a little bit towards goal
            }
            SendBuffer[i] = CurrentPosition[i]; // Modify next servo position
            PreMixBuffer[i] = SendBuffer[i];    // Maybe mix the slowed version
        }
    }
}
/*********************************************************************************************************************************/
void RerouteOutputs()
{ // This function re-routes outputs to the defined channels

    uint16_t temp[CHANNELSUSED];
    for (int i = 0; i < CHANNELSUSED; ++i)
    {
        temp[i] = PreMixBuffer[i];
    }
    for (int i = 0; i < CHANNELSUSED; ++i)
    {
        PreMixBuffer[i] = temp[ChannelOutPut[i]];
        SendBuffer[i] = PreMixBuffer[i];
    }
}

/*********************************************************************************************************************************/

void DoTrimsAndSubtrims()
{

    for (uint16_t OutputChannel = 0; OutputChannel < CHANNELSUSED; ++OutputChannel)
    {
        uint16_t InputChannel = InPutStick[OutputChannel];                                      // Input sticks knobs & switches are mapped by user
        SendBuffer[OutputChannel] += (SubTrims[OutputChannel] - 127) * 5;                       // ADD SUBTRIM to output channel, (Range 0 - 127 - 254) multiplier is always 5 otherwise subtrim might keep changing
        SendBuffer[OutputChannel] += GetTrimAmount(InputChannel);                               // ADD TRIM to output channel
        SendBuffer[OutputChannel] = constrain(SendBuffer[OutputChannel], MINMICROS, MAXMICROS); // Keep within limits
        PreMixBuffer[OutputChannel] = SendBuffer[OutputChannel];                                // premixbuffer will be needed again...
    }
}
//**************************************************************************************************************************************************************
void CalculateAllOutputs()
{
    for (uint16_t OutputChannel = 0; OutputChannel < CHANNELSUSED; ++OutputChannel)
    {
        GetCurveDots(OutputChannel, DualRateValue);                                                                                                                // This for the Dual Rates function
        PreMixBuffer[OutputChannel] = Interpolate[InterpolationTypes[Bank][OutputChannel]](InputsBuffer[OutputChannel], InPutStick[OutputChannel], OutputChannel); // Use function pointer array to invoke selected interpolation.
        SendBuffer[OutputChannel] = PreMixBuffer[OutputChannel];                                                                                                   // Copy now to SendBuffer in case no mixes are needed
    }
}
//**************************************************************************************************************************************************************
void GetAllInputs()
{
    for (uint16_t OutputChannel = 0; OutputChannel < CHANNELSUSED; ++OutputChannel)
    {
        if (InPutStick[OutputChannel] < 8)
        {
            InputsBuffer[OutputChannel] = AnalogueReed(InPutStick[OutputChannel]); // Get values from sticks' pots (taking into account mode 1 and mode 2!)
        }
        else
        {
            InputsBuffer[OutputChannel] = ReadThreePositionSwitch(OutputChannel); // Get values from switches
        }
    }
}
/*********************************************************************************************************************************/
//                                               new version of GET NEW SERVO POSITIONS
/*********************************************************************************************************************************/
/** @brief GET NEW SERVO POSITIONS */
FASTRUN void GetNewChannelValues()
{
    if (!NewCompressNeeded)
    {
        NewCompressNeeded = true;
        GetAllInputs();        // Get all user inputs from sticks, pots and switches
        MixInputs();           // Mixes InputsBuffer[] and returns results in InputsBuffer[] (All 16 channels)
        CalculateAllOutputs(); // Calculate all outputs
        SlowAnyServos();       // Some servos may need to be slowed down for flaps etc.
        MixOutputs();          // If needed, Mixes PremixBuffer and returns it in SendBuffer.
        DoTrimsAndSubtrims();  // Add trims to output after mixing.
        RerouteOutputs();      // This function might re-route outputs to user-defined channels.
        ServoReverse();        // This function reverses servos if needed.
    }
}
/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

void ReduceLimits()
{ // Get things setup for sticks calibration
    for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        ChannelMax[i] = MAXRESOLUTION / 2;
        ChannelMin[i] = MAXRESOLUTION / 2;
    }
    for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        MaxDegrees[Bank][i] = 180;
        CentreDegrees[Bank][i] = 90;
        MinDegrees[Bank][i] = 0;
    }
}

/*********************************************************************************************************************************/
void ResetSwitchNumbers()
{
    for (int i = 0; i < 8; ++i)
    {
        SwitchNumber[i] = DefaultSwitchNumber[i];
    }
}
/*********************************************************************************************************************************/
void CalibrateSticks() // This discovers end of travel place for sticks etc.
{
    uint16_t p;
    for (uint8_t i = 0; i < PROPOCHANNELS; ++i)
    {
        p = adc->analogRead(AnalogueInput[i]);
        if (ChannelMax[i] < p)
            ChannelMax[i] = p;
        if (ChannelMin[i] > p)
            ChannelMin[i] = p;
    }
    NewCompressNeeded = false; // fake it as we are not sending data
    GetNewChannelValues();
}
/*********************************************************************************************************************************/
/** @brief Get centre as 90 degrees */
void ChannelCentres()
{
    for (int i = 0; i < PROPOCHANNELS; ++i)
    {
        ChannelCentre[i] = AnalogueReed(i);
        ChannelMidHi[i] = ChannelCentre[i] + ((ChannelMax[i] - ChannelCentre[i]) / 2);
        ChannelMidLow[i] = ChannelMin[i] + ((ChannelCentre[i] - ChannelMin[i]) / 2);
    }
    for (int i = PROPOCHANNELS; i < CHANNELSUSED; ++i)
    {
        ChannelMin[i] = 500;
        ChannelCentre[i] = 1500;
        ChannelMax[i] = 2500;
    }
    NewCompressNeeded = false; // fake it as we are not sending data
    GetNewChannelValues();
    CalibrateEdgeSwitches(); // These are now calibrated too in case some are reversed.
}

/*********************************************************************************************************************************/

FLASHMEM void ScanI2c()
{
    int ii;
    USE_INA219 = false;
    for (ii = 1; ii < 127; ++ii)
    {
        Wire.beginTransmission(ii);
        if (Wire.endTransmission() == 0)
        {
#ifdef DB_SENSORS
            Serial.print(ii, HEX); // in case new one shows up
            Serial.print("   ");
            if (ii == 0x40)
                Serial.println("INA219 voltage meter detected!");
#endif

            if (ii == 0x40)
            {
                USE_INA219 = true;
            }
        }
    }
}

/*********************************************************************************************************************************/

void UpdateModelsNameEveryWhere()
{
    char Owner[] = "Owner";
    char TheModelName[] = "ModelName";
    char GraphView_Channel[] = "Channel";
    char TrimView_Bank[] = "t1";
    char GraphView_fmode[] = "fmode";
    char SticksView_t1[] = "t1";
    char NoName[40];
    char Ch[] = "Channel ";
    char Nbuf[7];
    char mn1[40]; // holds model name plus its number
    char lb[] = " (";
    char rb[] = ")";

    strcpy(mn1, ModelName);

    if (CurrentView == FRONTVIEW)
        SendText(Owner, TxName);

    if (!ModelsMacUnionSaved.Val64)
    {
        strcpy(lb, " [");
        strcpy(rb, "]");
    }
    else
    {
        strcpy(lb, " (");
        strcpy(rb, ")");
    }

    if (CurrentView != MODELSVIEW)
    {
        strcat(mn1, lb);
        strcat(mn1, Str(Nbuf, ModelNumber, 0)); // Add model number for extra clarity
        strcat(mn1, rb);
    }

    SendText(TheModelName, mn1);

    if (CurrentView == GRAPHVIEW)
    {
        if (strlen(ChannelNames[ChanneltoSet - 1]) < 2)
        { // if no name, just show the channel number
            strcpy(NoName, Ch);
            SendText(GraphView_Channel, strcat(NoName, Str(Nbuf, ChanneltoSet, 0)));
        }
        else
        {
            SendText(GraphView_Channel, ChannelNames[ChanneltoSet - 1]);
        }
    }
    if (CurrentView == STICKSVIEW)
        SendText(SticksView_t1, BankNames[BanksInUse[Bank - 1]]);
    if (CurrentView == GRAPHVIEW)
        SendText(GraphView_fmode, BankNames[BanksInUse[Bank - 1]]);
    if (CurrentView == TRIM_VIEW)
    {
        SendText(TrimView_Bank, BankNames[BanksInUse[Bank - 1]]);
        UpdateTrimView();
    }
}

/*********************************************************************************************************************************/

FLASHMEM void InitSwitchesAndTrims()
{
    for (int i = 0; i < 8; ++i)
    {
        pinMode(SwitchNumber[i], INPUT_PULLUP);
        pinMode(TrimNumber[i], INPUT_PULLUP);
    }
    pinMode(BUTTON_SENSE_PIN, INPUT_PULLUP); // New function to sense power button press
}

/*********************************************************************************************************************************/

/** @brief STICKS CALIBRATION */
FLASHMEM void InitMaxMin()
{
    for (int i = 0; i < CHANNELSUSED; ++i)
    {
        ChannelMax[i] = MAXRESOLUTION;
        ChannelMidHi[i] = MAXRESOLUTION * 3 / 4;
        ChannelCentre[i] = MAXRESOLUTION / 2;
        ChannelMidLow[i] = MAXRESOLUTION / 4;
        ChannelMin[i] = 0;
    }
}

/*********************************************************************************************************************************/

FLASHMEM void InitCentreDegrees()
{

    for (int j = 1; j <= 4; ++j)
    {
        for (int i = 0; i < CHANNELSUSED; ++i)
        {
            MaxDegrees[j][i] = 180; //  180 degrees
            MidHiDegrees[j][i] = 135;
            CentreDegrees[j][i] = 90; //  90 degrees
            MidLowDegrees[j][i] = 45;
            MinDegrees[j][i] = 0; //  0 degrees
        }
    }
}

/*********************************************************************************************************************************/

void UpdateButtonLabels()
{
    char InputStick_Labels[16][4] = {"c1", "c2", "c3", "c4", "c5", "c6", "c7", "c8", "c9", "c10", "c11", "c12", "c13", "c14", "c15", "c16"};
    char OutputStick_Labels[16][4] = {"n4", "n5", "n6", "n7", "n8", "n9", "n10", "n11", "n12", "n13", "n14", "n15", "n16", "n17", "n18", "n19"};
    char Subtrim_Labels[16][4] = {"n00", "n1", "n2", "n3", "n4", "n5", "n6", "n7", "n8", "n9", "n10", "n11", "n12", "n13", "n14", "n15"};
    char fsch_labels[16][5] = {"ch1", "ch2", "ch3", "ch4", "ch5", "ch6", "ch7", "ch8", "ch9", "ch10", "ch11", "ch12", "ch13", "ch14", "ch15", "ch16"};
    char fs[16][5] = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
    char ChannelLabels[16][6] = {"Sch1", "Sch2", "Sch3", "Sch4", "Sch5", "Sch6", "Sch7", "Sch8", "Sch9", "Sch10", "Sch11", "Sch12", "Sch13", "Sch14", "Sch15", "Sch16"};
    char ChannelNumber[16][6] = {" (1)", "(2) ", " (3)", "(4) ", " (5)", "(6) ", " (7)", "(8) ", " (9)", "(10) ", " (11)", "(12) ", " (13)", "(14) ", " (15)", "(16) "};
    char ArrowRh[] = " >";
    char ArrowLh[] = "< ";
    char TrimLabels[4][4] = {"n0", "n1", "n2", "n3"};
    char LabelText[20];

    if (CurrentView == STICKSVIEW)
    {
        for (int i = 0; i < 16; ++i)
        {
            if ((float)i / 2 != (int)i / 2)
            { // Odd and even labels are formatted differnently here
                strcpy(LabelText, ArrowLh);
                strcat(LabelText, ChannelNumber[i]);
                strcat(LabelText, ChannelNames[i]);
            }
            else
            {
                strcpy(LabelText, ChannelNames[i]);
                strcat(LabelText, ChannelNumber[i]);
                strcat(LabelText, ArrowRh);
            }
            SendText(ChannelLabels[i], LabelText);
        }
    }
    if (CurrentView == FAILSAFE_VIEW)
    {
        for (int i = 0; i < 16; ++i)
        {
            SendValue(fs[i], FailSafeChannel[i]);
        }
    }
    if (CurrentView == INPUTS_VIEW || CurrentView == FAILSAFE_VIEW || CurrentView == REVERSEVIEW || CurrentView == BUDDYCHVIEW || CurrentView == SLOWSERVOVIEW)
    {
        for (int i = 0; i < 16; ++i)
        {
            SendText(fsch_labels[i], ChannelNames[i]);
            SendValue(InputStick_Labels[i], InPutStick[i] + 1);
            SendValue(OutputStick_Labels[i], ChannelOutPut[i] + 1);
            if (CurrentView != SLOWSERVOVIEW)
            {
                if (i < 4)
                    SendValue(TrimLabels[i], InputTrim[i] + 1); //
            }
        }
    }
    if (CurrentView == SUBTRIMVIEW)
    {
        for (int i = 0; i < 16; ++i)
        {
            SendText(fsch_labels[i], ChannelNames[i]);
            SendValue(Subtrim_Labels[i], SubTrims[i] - 127);
        }
    }
}

/*********************************************************************************************************************************/
void Force_ReDisplay()
{
    for (int i = 0; i < CHANNELSUSED; ++i)
        ShownBuffer[i] = 242; // to force a re-show of servo positions
}

/*********************************************************************************************************************************/
void SendColour(char *but, int Colour)
{
    char lbut[60];
    char nb[10] = " ";
    strcpy(lbut, but);
    strcat(lbut, Str(nb, Colour, 0));
    SendCommand(lbut);
}
/*********************************************************************************************************************************/
void ShowSafetyIsOn()
{
    if (AnnounceBanks && !BeQuiet && UseMotorKill && millis() > 10000)
    {
        PlaySound(SAFEON);
        if (UseLog)
            LogSafety(1);
    }
    if ((CurrentView == FRONTVIEW) && UseMotorKill)
    {
        char bco[] = "bt0.bco=";
        char bco2[] = "bt0.bco2=";
        char pco[] = "bt0.pco=";
        char pco2[] = "bt0.pco2=";
        SendColour(bco, SpecialColour);
        SendColour(bco2, SpecialColour);
        SendColour(pco, HighlightColour);
        SendColour(pco2, HighlightColour);
        BeQuiet = false;
    }
}
/*********************************************************************************************************************************/
void ShowSafetyIsOff()
{
    if (AnnounceBanks && !BeQuiet && UseMotorKill)
    {
        PlaySound(SAFEOFF);
        if (UseLog)
            LogSafety(0);
    }
    if ((CurrentView == FRONTVIEW) && UseMotorKill)
    {
        char bco[] = "bt0.bco=";
        char bco2[] = "bt0.bco2=";
        char pco[] = "bt0.pco=";
        char pco2[] = "bt0.pco2=";
        SendColour(bco, BackGroundColour);
        SendColour(bco2, BackGroundColour);
        SendColour(pco, HighlightColour);
        SendColour(pco2, HighlightColour);
        BeQuiet = false;
    }
}
/*********************************************************************************************************************************/

void WatchDogCallBack()
{
    // Serial.println("RESETTING ...");
}

/*********************************************************************************************************************************/

FLASHMEM void SetDS1307ToCompilerTime()
{
    if (getDate(__DATE__) && getTime(__TIME__))
    {
        RTC.write(tm);
    } // only useful when connected to compiler
}

/************************************************************************************************************/

FLASHMEM void GetTXVersionNumber()
{
    char nbuf[5];
    uint8_t Txv1 = TXVERSION_MAJOR;
    uint8_t Txv2 = TXVERSION_MINOR;
    uint8_t Txv3 = TXVERSION_MINIMUS;
    Str(TransmitterVersionNumber, Txv1, 2);
    Str(nbuf, Txv2, 2);
    strcat(TransmitterVersionNumber, nbuf);
    Str(nbuf, Txv3, 0);
    strcat(TransmitterVersionNumber, nbuf);
    strcat(TransmitterVersionNumber, TXVERSION_EXTRA);
}
/************************************************************************************************************/

FASTRUN void BufferTeensyMACAddPipe()
{
    for (int q = 1; q < 6; ++q)
    {
        SendBuffer[q] = MacAddress[q];
    }
}

// *********************************************************************************************************************************/
void teensyMAC(uint8_t *mac) // only works on Teensy 4.1 and 4.0
{                            // there are 2 MAC addresses each 48bit
    uint32_t m1 = HW_OCOTP_MAC1;
    uint32_t m2 = HW_OCOTP_MAC0;
    mac[0] = m1 >> 8;
    mac[1] = m1 >> 0;
    mac[2] = m2 >> 24;
    mac[3] = m2 >> 16;
    mac[4] = m2 >> 8;
    mac[5] = m2 >> 0;
}

/*********************************************************************************************************************************/
// This function gets the unique MAC address of the Teensy 4.1
// And also fixes it so that it's a more suitable Pipe address for the nRF24L01

void GetTeensyMacAddress() //
{
    teensyMAC(MacAddress); // Get MAC address
    for (int i = 1; i < 6; ++i)
    {
        // Look1(MacAddress[i], HEX); // Show MAC address in HEX
        // Look1(" -> ");
        MacAddress[i] = CheckPipeNibbles(MacAddress[i]); // Fix PIPE if needed !
                                                         // MacAddress[i] += 42;                             // test another random pipe address
                                                         //  Look(MacAddress[i], HEX);                        // Show MAC address in HEX
    }

    TeensyMACAddPipe = (uint64_t)MacAddress[0] << 40;
    TeensyMACAddPipe += (uint64_t)MacAddress[1] << 32;
    TeensyMACAddPipe += (uint64_t)MacAddress[2] << 24;
    TeensyMACAddPipe += (uint64_t)MacAddress[3] << 16;
    TeensyMACAddPipe += (uint64_t)MacAddress[4] << 8;
    TeensyMACAddPipe += (uint64_t)MacAddress[5];
}

/*********************************************************************************************************************************/

void ConvertBuddyPipeTo64BITS()
{
    BuddyMACAddPipe = (uint64_t)BuddyMacAddress[0] << 40;
    BuddyMACAddPipe += (uint64_t)BuddyMacAddress[1] << 32;
    BuddyMACAddPipe += (uint64_t)BuddyMacAddress[2] << 24;
    BuddyMACAddPipe += (uint64_t)BuddyMacAddress[3] << 16;
    BuddyMACAddPipe += (uint64_t)BuddyMacAddress[4] << 8;
    BuddyMACAddPipe += (uint64_t)BuddyMacAddress[5];
    BuddyMACAddPipe >>= 8;
}

/*********************************************************************************************************************************/

void WarnUserIfBuddyBoxIsOn() // This function warns the user if the buddy box is on
{
    if (BuddyPupilOnWireless)
    {
        PlaySound(BUDDYPUPILON);
        DelayWithDog(1500);                     // allow sound to finish
        FHSS_data::PaceMaker = PACEMAKER_BUDDY; // only 200Hz
    }
    if (BuddyMasterOnWireless)
    {
        PlaySound(BUDDYMASTERON);
        DelayWithDog(1500);                     // allow sound to finish
        FHSS_data::PaceMaker = PACEMAKER_BUDDY; // only 200Hz
    }
}
// *********************************************************************************************************************************/
void CheckSDCard()
{
    char err_404[] = "SD card error!";
    char err_405[] = "or not found!";
    bool SDCARDOK = SD.begin(BUILTIN_SDCARD); // MUST return true or SD card is not working
    if (!SDCARDOK)
    {
        delay(1000); // allow screen to warm up
        SendCommand(pFrontView);
        delay(70);
        RestoreBrightness();
        delay(70);
        SendCommand(WarnNow);
        for (uint8_t i = 0; i < 4; ++i)
        {
            SendText(Warning, err_404);
            delay(1000);
            SendText(Warning, err_405);
            delay(1000);
            PlaySound(WHAHWHAHMSG);
        }
        digitalWrite(POWER_OFF_PIN, HIGH); // turn off power now!
    }
}
/*********************************************************************************************************************************/

void initADC() //
{
    // #define MAXRESOLUTION 4095  // 12 BIT
    adc->setResolution(12); // 8, 10, 12 or 16 bits
    adc->setAveraging(4);   // 0, 4, 8, 16 or 32.
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
}
//*********************************************************************************************************************************/
void SetBrightness(uint8_t B) // 0 to 100
{
    char cmd[20];
    char dim[] = "dim=";
    char nb[10];
    strcpy(cmd, dim);
    Str(nb, B, 0);
    strcat(cmd, nb);
    ScreenIsOff = false;
    SendCommand(cmd);
    ScreenTimeTimer = millis(); // reset screen counter
}

/*********************************************************************************************************************************/
// SETUP
/*********************************************************************************************************************************/
FLASHMEM void setup()
{
    char FrontView_BackGround[] = "FrontView.BackGround";
    char FrontView_ForeGround[] = "FrontView.ForeGround";
    char FrontView_Special[] = "FrontView.Special";
    char FrontView_Highlight[] = "FrontView.Highlight";
    char err_chksm[] = "File checksum?";
    char err_404[] = "SD card error!";
    char FrontView_Connected[] = "Connected";
    char FrontView_Hours[] = "Hours";
    char FrontView_Mins[] = "Mins";
    char FrontView_Secs[] = "Secs";
    char ModelsFile[] = "models.dat";

    pinMode(REDLED, OUTPUT);
    pinMode(GREENLED, OUTPUT);
    pinMode(BLUELED, OUTPUT);
    pinMode(POWER_OFF_PIN, OUTPUT);
    digitalWrite(POWER_OFF_PIN, LOW); // default is LOW anyway. HIGH to turn off
    BlueLedOn();
    NEXTION.begin(921600); // BAUD rate also set in display code THIS IS THE MAX (was 115200)
    InitMaxMin();
    InitCentreDegrees();
    ResetSubTrims();
    CentreTrims();
    strcpy(LogFileName, "");
    ErrorState = NOERROR;
    WatchDogConfig.window = WATCHDOGMAXRATE;  //  = MINIMUM RATE in milli seconds, (32ms to 522.232s) must be MUCH smaller than timeout
    WatchDogConfig.timeout = WATCHDOGTIMEOUT; //  = MAX TIMEOUT in milli seconds, (32ms to 522.232s)
    WatchDogConfig.callback = WatchDogCallBack;
    CheckSDCard(); // Check if SD card is present and working and initialise it
    TeensyWatchDog.begin(WatchDogConfig);
    delay(300); // <<********************* MUST ALLOW DOG TO INITIALISE
    DelayWithDog(WARMUPDELAY);
    if (CheckFileExists(ModelsFile))
    {
        if (!LoadAllParameters())
        { // if file not good ..
            ErrorState = CHECKSUMERROR;
        }
    }
    else
    {
        ErrorState = MODELSFILENOTFOUND; // if no file ... or no SD
    }
    SetBrightness(1);         // start with screen almost off
    SendCommand(pSplashView); // show splash screen **************************
    CurrentView = SPLASHVIEW; // while loading ...
    GetTeensyMacAddress();
    ConvertBuddyPipeTo64BITS();
    Wire.begin();
    ScanI2c();
    if (USE_INA219)
        ina219.begin();
    InitSwitchesAndTrims();
    InitRadio(DefaultPipe);
    delay(WARMUPDELAY);                                // Allow Nextion time to warm up
    SendValue(FrontView_BackGround, BackGroundColour); // Get colours ready
    SendValue(FrontView_ForeGround, ForeGroundColour);
    SendValue(FrontView_Special, SpecialColour);
    SendValue(FrontView_Highlight, HighlightColour);
    CurrentView = 254;
    SetAudioVolume(AudioVolume);
    if (PlayFanfare)
        PlaySound(WINDOWS1);
    for (int i = 0; i < 100; ++i) // fade in screen brightness
    {
        SetBrightness(i);
        DelayWithDog(15);
    }
    if (PlayFanfare)
        DelayWithDog(1000);

    SendValue(FrontView_Hours, 0);
    SendValue(FrontView_Mins, 0);
    SendValue(FrontView_Secs, 0);
    //  ***************************************************************************************
    //  SetDS1307ToCompilerTime();    //  **   Uncomment this line to set DS1307 clock to compiler's (Computer's) time.        **
    // **   BUT then re-comment it!! Otherwise it will reset to same time on every boot up! **
    //  ***************************************************************************************
    BoundFlag = false;
    StartInactvityTimeout();
    GetTXVersionNumber();
    ScreenTimeTimer = millis();
    // if (UseLog)
    // {
    // LogPowerOn();
    // LogThisModel();
    // }
    SendText(FrontView_Connected, na);
    UpdateModelsNameEveryWhere();
    ConfigureStickMode();
    WarningTimer = millis();
    EnsureMotorIsOff();
    if (!BuddyPupilOnWireless)
    { // when pupil is buddying wirelessly, these potential errors are ignored

        if (!UseMotorKill)
            ShowMotor(1);
        if (SafetyON)
            ShowSafetyIsOn();
        if (ErrorState)
        {
            SendCommand(WarnNow);
            if (ErrorState == CHECKSUMERROR)
            {
                SendText(Warning, err_chksm);
            }
            if (ErrorState == MODELSFILENOTFOUND)
            {
                SendText(Warning, err_404);
            }
        }
    }
    initADC();
    FHSS_data::PaceMaker = PACEMAKER;
    RationaliseBuddy();
    WarnUserIfBuddyBoxIsOn();
    ClearMostParameters();
    DelayWithDog(500);
    GotoFrontView();
    RedLedOn();
}
// **************************************************************************************************************************************************************
void RationaliseBuddy()
{
    PupilIsAlive = 0;
    MasterIsAlive = 0;
    CurrentMode = NORMAL;
    WirelessBuddy = true;

    if (!BuddyPupilOnWireless && !BuddyMasterOnWireless)
        WirelessBuddy = false;

    if (WasBuddyPupilOnWireless && !BuddyPupilOnWireless)
    {                     // Pupil has just gone off buddy mode
        ConfigureRadio(); // Very like InitRadio but without Radio1.begin() !
        WasBuddyPupilOnWireless = false;
        DontChangePipeAddress = false;
    }

    if (BuddyPupilOnWireless)
    {
        CurrentMode = LISTENMODE; // set the mode to listen only
        ModelMatched = false;
        BoundFlag = false;
        Connected = false;
        StartBuddyListen();
    }
    if (BuddyMasterOnWireless)
    {
        FHSS_data::PaceMaker = PACEMAKER_BUDDY;
    }
}
/*********************************************************************************************************************************/
void GetFrameRate()
{ // This function calculates the frame rate and the average frame rate
    if (RecentGoodPacketsCount)
        PacketsPerSecond = RecentGoodPacketsCount;
    RecentGoodPacketsCount = 0;
    TotalFrameRate += PacketsPerSecond;
    AverageFrameRate = TotalFrameRate / ++FrameRateCounter;
}

/*********************************************************************************************************************************/
/** @returns position of text1 within text2 or 0 if not found */
int InStrng(char *text1, char *text2)
{
    for (uint16_t j = 0; j < strlen(text2); ++j)
    {
        bool flag = false;
        for (uint16_t i = 0; i < strlen(text1); ++i)
        {
            if (text1[i] != text2[i + j])
            {
                flag = true;
                break;
            }
        }
        if (!flag)
            return j + 1; // Found match
    }
    return 0; // Found no match
}

/***********************************************************************************************************/

int GetDateAsInt(int f)
{ // dates are in format 23-08-24.LOG ie 23rd August '24
  // DateInt is 23 + (8 * 31)  + (24 * 31 * 12) =  23 + 248 + 8768 = 9019
    int DateInt = 0;
    char Date[10];
    for (int i = 0; i < 2; ++i)
        Date[i] = TheFilesList[f][i];
    Date[2] = 0;
    DateInt = atoi(Date); // the DAY
    for (int i = 3; i < 5; ++i)
        Date[i - 3] = TheFilesList[f][i];
    Date[2] = 0;
    DateInt += atoi(Date) * 31; // the MONTH
    for (int i = 6; i < 8; ++i)
        Date[i - 6] = TheFilesList[f][i];
    Date[2] = 0;
    DateInt += atoi(Date) * 31 * 12; // the YEAR
    return DateInt;
}

/***********************************************************************************************************/
void SortByDateInDescendingOrder()
{
    int f = 0;
    bool flag = true;
    int Scount = 0;
    char TempArray[18];
    int d1, d2;
    while (flag && Scount < 10000)
    {
        flag = false;
        for (f = 0; f < ExportedFileCounter - 1; ++f)
        {
            d1 = GetDateAsInt(f);
            d2 = GetDateAsInt(f + 1);
            if (d1 < d2)
            { // Sort by date in descending order so recent log files are at the top
                strcpy(TempArray, TheFilesList[f]);
                strcpy(TheFilesList[f], TheFilesList[f + 1]);
                strcpy(TheFilesList[f + 1], TempArray);
                flag = true;
                ++Scount;
            }
        }
    }
}
/****************************************************************************************************************************/

/** Bubble sort */
void SortDirectory() // Bubble sort for alphabetising the files. Not ideal for dates but OK for names.
                     // TODO: Make a version that sorts by date in reverse order
{
    int f = 0;
    bool flag = true;
    int Scount = 0;
    char TempArray[18];

    if (strcmp(MOD, ".LOG") == 0)
    { // if it's a log file, sort by date in descending order
        SortByDateInDescendingOrder();
        return;
    }

    while (flag && Scount < 10000)
    {
        flag = false;
        for (f = 0; f < ExportedFileCounter - 1; ++f)
        {
            if (strcmp(TheFilesList[f], TheFilesList[f + 1]) > 0)
            {
                strcpy(TempArray, TheFilesList[f]);
                strcpy(TheFilesList[f], TheFilesList[f + 1]);
                strcpy(TheFilesList[f + 1], TempArray);
                flag = true;
                ++Scount;
            }
        }
    }
}

/*********************************************************************************************************************************/
void BuildDirectory()
{
    char Entry1[30];
    char fn[20];
    int i = 0;
    File dir = SD.open("/");
    ExportedFileCounter = 0;
    while (true)
    {
        File entry = dir.openNextFile();
        if (!entry || ExportedFileCounter > MAXBACKUPFILES)
            break;
        strcpy(Entry1, entry.name());
        if ((InStrng(MOD, Entry1) > 0) && (!InStrng((char *)"._", Entry1)))
        {
            strcpy(fn, entry.name());
            for (i = 0; i < 12; ++i)
            {
                TheFilesList[ExportedFileCounter][i] = fn[i];
            }
            ExportedFileCounter++;
        }
        entry.close();
    }
    SortDirectory();
}

/*********************************************************************************************************************************/
/** @brief Discover which channel to setup */
int GetChannel()
{
    unsigned int i;
    for (i = 0; i < sizeof(TextIn); ++i)
    {
        if (isdigit(TextIn[i]))
            break;
    }
    return atoi(&TextIn[i]);
}

/*********************************************************************************************************************************/

void ShowSwitchNameWithReversed(char *Sw, uint8_t n, char *text)
{ // this removes the R from switch name if it's not reversed

    char NotReversed[30] = " ";
    if (n >= 1 && n <= 4 && SwitchReversed[n - 1])
    {
        SendText(Sw, text);
        return;
    }
    for (uint8_t i = 0; i < strlen(text) - 1; ++i)
        NotReversed[i] = text[i];

    if (n >= 1 && n <= 4 && !SwitchReversed[n - 1])
    {
        SendText(Sw, NotReversed);
        return;
    }
}

/*********************************************************************************************************************************/

void DoOneSwitch(char *Sw, uint8_t n)
{
    char NotUsed[] = "Not used         ";
    char Banks123[] = "Banks 1 2 3  R";
    char Auto[] = "Bank 4 & Motor  R";
    char Safety_Switch[] = "Safety  R";
    char Buddy_Switch[] = "Buddy  R";
    char DualRates_Switch[] = "Rates  R";
    char cc9[] = " (Ch 9) ";
    char cc10[] = " (Ch 10) ";
    char cc11[] = " (Ch 11) ";
    char cc12[] = " (Ch 12) ";
    char c9[40];
    char c10[40];
    char c11[40];
    char c12[40];
    char R[] = " R";

    strcpy(c9, ChannelNames[8]);
    strcpy(c10, ChannelNames[9]);
    strcpy(c11, ChannelNames[10]);
    strcpy(c12, ChannelNames[11]);

    strcat(c9, cc9);
    strcat(c9, R);
    strcat(c10, cc10);
    strcat(c10, R);
    strcat(c11, cc11);
    strcat(c11, R);
    strcat(c12, cc12);
    strcat(c12, R);

    SendText(Sw, NotUsed);

    if (Autoswitch == n)
    {
        ShowSwitchNameWithReversed(Sw, n, Auto);
        return;
    }
    if (BankSwitch == n)
    {
        ShowSwitchNameWithReversed(Sw, n, Banks123);
        return;
    }
    if (TopChannelSwitch[Ch9_SW] == n)
    {
        ShowSwitchNameWithReversed(Sw, n, c9);
        return;
    }
    if (TopChannelSwitch[Ch10_SW] == n)
    {
        ShowSwitchNameWithReversed(Sw, n, c10);
        return;
    }
    if (TopChannelSwitch[Ch11_SW] == n)
    {
        ShowSwitchNameWithReversed(Sw, n, c11);
        return;
    }
    if (TopChannelSwitch[Ch12_SW] == n)
    {
        ShowSwitchNameWithReversed(Sw, n, c12);
        return;
    }
    if (SafetySwitch == n)
    {
        ShowSwitchNameWithReversed(Sw, n, Safety_Switch);
        return;
    }
    if (DualRatesSwitch == n)
    {
        ShowSwitchNameWithReversed(Sw, n, DualRates_Switch);
        return;
    }
    if (BuddySwitch == n)
    {
        ShowSwitchNameWithReversed(Sw, n, Buddy_Switch);
        return;
    }
}

/*********************************************************************************************************************************/

void UpdateSwitchesView()
{ // now optimised!
    char sw[4][4] = {"sw1", "sw2", "sw3", "sw4"};
    for (int i = 1; i <= 4; ++i)
        DoOneSwitch(sw[i - 1], i);
}

/*********************************************************************************************************************************/
int CheckRange(int v, int min, int max)
{
    if (v > max)
        v = max;
    if (v < min)
        v = min;
    return v;
}
/*********************************************************************************************************************************/

void DoNewChannelName(int ch, int k)
{
    int j = 0;
    ChannelNames[ch - 1][0] = 32;
    ChannelNames[ch - 1][1] = 0; // remove old name
    while (uint8_t(TextIn[k]) > 0)
    {
        ChannelNames[ch - 1][j] = TextIn[k];
        ++j;
        ++k;
        ChannelNames[ch - 1][j] = 0;
    }
    SaveOneModel(ModelNumber);
}

/*********************************************************************************************************************************/

/** @brief updates display in textbox */
void ShowFileNumber()
{
    char dflt[] = "DEFAULT.MOD";
    char ModelsView_filename[] = "filename";
    char newfname[27];

    strcpy(newfname, dflt);
    if (FileNumberInView >= ExportedFileCounter)
        FileNumberInView = 0;
    if (FileNumberInView < 0)
        FileNumberInView = ExportedFileCounter - 1;
    if (ExportedFileCounter)
    {
        for (int i = 0; i < 12; ++i)
        {
            newfname[i] = TheFilesList[FileNumberInView][i];
            newfname[i + 1] = 0;
            if (newfname[i] <= 32 || newfname[i] > 127)
                break;
        }
    }
    SendText(ModelsView_filename, newfname);
}

/*********************************************************************************************************************************/

void ShowFileErrorMsg()
{
    char ErrorOn[] = "vis error,1";
    char ErrorOff[] = "vis error,0";
    for (int pp = 0; pp < 3; ++pp)
    {
        SendCommand(ErrorOn);
        DelayWithDog(200);
        SendCommand(ErrorOff);
        DelayWithDog(100);
    }
    FileError = false;
}

/*********************************************************************************************************************************/

// This function takes account of the fact one gimbal is upside down ... and some people use mode 2.
// Aileron is always reversed, plus either throttle or elevator according to mode 1 or 2
// It reverses some pins

int AnalogueReed(uint8_t InputChannel)
{

    int value = adc->analogRead(AnalogueInput[InputChannel]); //
    if (SticksMode == 2)
    {
        if ((InputChannel == 0) || (InputChannel == 2))
        {
            value = map(value, ChannelMin[InputChannel], ChannelMax[InputChannel], ChannelMax[InputChannel], ChannelMin[InputChannel]);
        }
    }
    else
    {
        if ((InputChannel == 0) || (InputChannel == 1))
        {
            value = map(value, ChannelMin[InputChannel], ChannelMax[InputChannel], ChannelMax[InputChannel], ChannelMin[InputChannel]);
        }
    }
    return value;
}

/*********************************************************************************************************************************/

void SetDefaultValues()
{
    uint16_t j = 0;
    uint16_t i = 0;
    char empty[33] = "Not in use";

    CloseModelsFile();
    OpenModelsFile();

    while ((empty[i]) && (i < 29))
    {
        ModelName[i] = empty[i];
        ModelName[i + 1] = 0;
        ++i;
    }

    char DefaultChannelNames[CHANNELSUSED][11] = {{"Aileron"}, {"Elevator"}, {"Throttle"}, {"Rudder"}, {"Ch 5"}, {"Ch 6"}, {"Ch 7"}, {"Ch 8"}, {"Ch 9"}, {"Ch 10"}, {"Ch 11"}, {"Ch 12"}, {"Ch 13"}, {"Ch 14"}, {"Ch 15"}, {"Ch 16"}};

    for (i = 0; i < CHANNELSUSED; ++i)
    {
        for (j = 1; j <= 4; ++j)
        {
            MaxDegrees[j][i] = 150;
            MidHiDegrees[j][i] = 120;
            CentreDegrees[j][i] = 90;
            MidLowDegrees[j][i] = 60;
            MinDegrees[j][i] = 30;
        }
    }

    for (j = 0; j < MAXMIXES; ++j)
    {
        for (i = 0; i < CHANNELSUSED; ++i)
        {
            Mixes[j][i] = 0;
        }
    }

    for (j = 0; j < BANKS_USED + 1; ++j)
    { // must have fudged this somewhere.... Probably 1-5 instead of 0-4
        for (i = 0; i < CHANNELSUSED; ++i)
        {
            Trims[j][i] = 80; // MIDPOINT is 80 !
        }
    }
    RXCellCount = 3;

    for (i = 0; i < CHANNELSUSED; ++i)
    {
        InPutStick[i] = i;
        ChannelOutPut[i] = i;
    }

    for (i = 0; i < CHANNELSUSED; ++i)
    {
        FailSafeChannel[i] = false;
    }

    for (i = 0; i < CHANNELSUSED; ++i)
    {
        for (j = 0; j < 10; ++j)
        {
            ChannelNames[i][j] = DefaultChannelNames[i][j];
        }
    }
    for (j = 1; j <= BANKS_USED; ++j)
    {
        for (i = 0; i < CHANNELSUSED; ++i)
        {
            Exponential[j][i] = DEFAULT_EXPO; // 0% (50) expo = default
        }
    }
    for (j = 0; j < BANKS_USED; ++j)
    {
        for (i = 0; i < CHANNELSUSED; ++i)
        {
            InterpolationTypes[j][i] = EXPONENTIALCURVES; // Expo is default
        }
    }
    for (i = 0; i < CHANNELSUSED; ++i)
    {
        SubTrims[i] = 127; // centre (0 - 254)
    }
    for (j = 0; j < BYTESPERMACRO; ++j)
    {
        for (i = 0; i < MAXMACROS; ++i)
        {
            MacrosBuffer[i][j] = 0;
        }
    }
    for (int i = 0; i < 4; ++i)
    {
        InputTrim[i] = i;
    }

    if (CurrentView == CALIBRATEVIEW)
    {
        UseMotorKill = false;
        MotorChannelZero = 50;
        MotorChannel = 15;
    }
    else
    {
        UseMotorKill = true;
        MotorChannelZero = 30;
        MotorChannel = 2;
    }

    ReversedChannelBITS = 0; //  No channel reversed
    RxVoltageCorrection = 0;
    ModelsMacUnionSaved.Val64 = 0;
    UseDualRates = false;
    Drate1 = 100;
    Drate2 = 75;
    Drate3 = 50;
    DualRateChannels[0] = 1;
    DualRateChannels[1] = 2;
    DualRateChannels[2] = 4;
    DualRateChannels[3] = 0;
    DualRateChannels[4] = 0;
    DualRateChannels[5] = 0;
    DualRateChannels[6] = 0;
    DualRateChannels[7] = 0;

    for (int i = 0; i < 4; ++i)
    {
        BanksInUse[i] = i + 4;
    }
    for (int j = 0; j < 4; ++j)
    {
        for (int i = 0; i < 16; ++i)
        {
            ServoSpeed[j][i] = 100;
        }
    }
    for (i = 0; i < 4; ++i)
    {
        DualRateRate[i] = 0;
    }
    TrimMultiplier = 5;
    for (int i = 0; i < 11; ++i)
    {
        ServoFrequency[i] = 50;
        ServoCentrePulse[i] = 1500;
    }

    ModelDefined = 42;
}

/*********************************************************************************************************************************/

/** @brief Uses servo degrees to position dots */
FASTRUN void GetDotPositions()
{
    int p = 0;
    xPoints[0] = BOXLEFT;
    xPoints[4] = BOXRIGHT - BOXLEFT;
    p = map(MinDegrees[Bank][ChanneltoSet - 1], 0, 180, BOXWIDTH, BOXLEFT);
    yPoints[0] = constrain(p, 39, 391);
    p = map(MidLowDegrees[Bank][ChanneltoSet - 1], 0, 180, BOXWIDTH, BOXLEFT);
    yPoints[1] = constrain(p, 39, 391);
    xPoints[1] = BOXLEFT + 90;
    xPoints[2] = BOXLEFT + 180;
    p = map(CentreDegrees[Bank][ChanneltoSet - 1], 0, 180, BOXWIDTH, BOXLEFT);
    yPoints[2] = constrain(p, 39, 391);
    xPoints[3] = BOXLEFT + 270;
    p = map(MidHiDegrees[Bank][ChanneltoSet - 1], 0, 180, BOXWIDTH, BOXLEFT);
    yPoints[3] = constrain(p, 39, 391);
    p = map(MaxDegrees[Bank][ChanneltoSet - 1], 0, 180, BOXWIDTH, BOXLEFT);
    yPoints[4] = constrain(p, 39, 391);
}

/*********************************************************************************************************************************/

FASTRUN void updateInterpolationTypes()
{
    char ExpR[] = "Exp";
    char Smooth[] = "Smooth";
    char Lines[] = "Lines";
    char Expon[] = "Expo";          // number display
    char Ex1[] = "Ex1";             // slider
    char t3on[] = "vis t3,1";       // expo value lable
    char b13on[] = "vis b13,1";     // expo -
    char b12on[] = "vis b12,1";     // expo +
    char ExponOn[] = "vis Expo,1";  // number display
    char Ex1On[] = "vis Ex1,1";     // slider
    char t3off[] = "vis t3,0";      // expo value lable
    char b13off[] = "vis b13,0";    // expo -
    char b12off[] = "vis b12,0";    // expo +
    char ExponOff[] = "vis Expo,0"; // number display
    char Ex1Off[] = "vis Ex1,0";    // slider

    switch (InterpolationTypes[Bank][ChanneltoSet - 1])
    {
    case STRAIGHTLINES:
        SendValue(Lines, 1);
        SendValue(Smooth, 0);
        SendValue(ExpR, 0);
        SendCommand(t3off);
        SendCommand(b13off);
        SendCommand(b12off);
        SendCommand(ExponOff);
        SendCommand(Ex1Off);
        break;
    case SMOOTHEDCURVES:
        SendValue(Lines, 0);
        SendValue(Smooth, 1);
        SendValue(ExpR, 0);
        SendCommand(t3off);
        SendCommand(b13off);
        SendCommand(b12off);
        SendCommand(ExponOff);
        SendCommand(Ex1Off);
        break;
    case EXPONENTIALCURVES:
        SendValue(Lines, 0);
        SendValue(Smooth, 0);
        SendValue(ExpR, 1);
        SendCommand(t3on);
        SendCommand(b13on);
        SendCommand(b12on);
        SendCommand(ExponOn);
        SendCommand(Ex1On);
        SendValue(Ex1, (Exponential[Bank][ChanneltoSet - 1]));        // The slider
        SendValue(Expon, (Exponential[Bank][ChanneltoSet - 1]) - 50); // the number
        break;
    default:
        break;
    }
}

/*********************************************************************************************************************************/

FASTRUN void DisplayCurve()
{
    int p = 0;
    char cmdBuffer[512] = "";          // Buffer for commands
    char tempCmd[80];                  // Temporary buffer for individual commands
    char endMarker[] = "\xFF\xFF\xFF"; // Nextion end marker

    float HalfXRange;
    float TopHalfYRange;
    float BottomHalfYRange;
    int xDot1;
    int yDot1;
    int xDot2 = 0;
    int yDot2 = 0;
    int DotSize = 4;
    int DotColour = ForeGroundColour;
    double xPoint, yPoint;

    // Constrain degrees values
    p = constrain(MinDegrees[Bank][ChanneltoSet - 1], 0, 180);
    MinDegrees[Bank][ChanneltoSet - 1] = p;
    p = constrain(MidLowDegrees[Bank][ChanneltoSet - 1], 0, 180);
    MidLowDegrees[Bank][ChanneltoSet - 1] = p;
    p = constrain(CentreDegrees[Bank][ChanneltoSet - 1], 0, 180);
    CentreDegrees[Bank][ChanneltoSet - 1] = p;
    p = constrain(MidHiDegrees[Bank][ChanneltoSet - 1], 0, 180);
    MidHiDegrees[Bank][ChanneltoSet - 1] = p;
    p = constrain(MaxDegrees[Bank][ChanneltoSet - 1], 0, 180);
    MaxDegrees[Bank][ChanneltoSet - 1] = p;

    GetDotPositions();

    // 1. First batch: Clear box and send top row values

    // Clear the box
    sprintf(tempCmd, "fill %d,%d,%d,%d,%d", 20, 20, 388, 375, BackGroundColour);
    strcat(cmdBuffer, tempCmd);
    strcat(cmdBuffer, endMarker);

    // Top row values - sent in a single batch
    sprintf(tempCmd, "n1.val=%d", DegsToPercent(MinDegrees[Bank][ChanneltoSet - 1]));
    strcat(cmdBuffer, tempCmd);
    strcat(cmdBuffer, endMarker);

    sprintf(tempCmd, "n2.val=%d", DegsToPercent(MidLowDegrees[Bank][ChanneltoSet - 1]));
    strcat(cmdBuffer, tempCmd);
    strcat(cmdBuffer, endMarker);

    sprintf(tempCmd, "n3.val=%d", DegsToPercent(CentreDegrees[Bank][ChanneltoSet - 1]));
    strcat(cmdBuffer, tempCmd);
    strcat(cmdBuffer, endMarker);

    sprintf(tempCmd, "n4.val=%d", DegsToPercent(MidHiDegrees[Bank][ChanneltoSet - 1]));
    strcat(cmdBuffer, tempCmd);
    strcat(cmdBuffer, endMarker);

    sprintf(tempCmd, "n5.val=%d", DegsToPercent(MaxDegrees[Bank][ChanneltoSet - 1]));
    strcat(cmdBuffer, tempCmd);
    strcat(cmdBuffer, endMarker);

    // Send first batch of commands
    NEXTION.print(cmdBuffer);
    cmdBuffer[0] = '\0'; // Reset buffer

    // 2. Second batch: Draw box and reference lines

    // Draw the box
    sprintf(tempCmd, "draw %d,%d,%d,%d,%d", BOXLEFT, BOXTOP, BOXWIDTH, BOXHEIGHT, HighlightColour);
    strcat(cmdBuffer, tempCmd);
    strcat(cmdBuffer, endMarker);

    // Horizontal reference line
    xDot1 = xPoints[0];
    yDot1 = (BOXHEIGHT / 2) + 20;
    xDot2 = BOXWIDTH;
    yDot2 = yDot1;
    sprintf(tempCmd, "line %d,%d,%d,%d,%d", xDot1, yDot1, xDot2, yDot1, SpecialColour);
    strcat(cmdBuffer, tempCmd);
    strcat(cmdBuffer, endMarker);

    // Vertical reference line
    xDot1 = xPoints[2];
    yDot1 = BOXTOP;
    xDot2 = xDot1;
    yDot2 = BOXHEIGHT;
    sprintf(tempCmd, "line %d,%d,%d,%d,%d", xDot1, yDot1, xDot2, yDot2, SpecialColour);
    strcat(cmdBuffer, tempCmd);
    strcat(cmdBuffer, endMarker);

    // 3. Set visibility based on interpolation type
    if (InterpolationTypes[Bank][ChanneltoSet - 1] == EXPONENTIALCURVES)
    {
        // Hide controls for exponential mode
        strcat(cmdBuffer, "vis b3,0");
        strcat(cmdBuffer, endMarker);
        strcat(cmdBuffer, "vis b4,0");
        strcat(cmdBuffer, endMarker);
        strcat(cmdBuffer, "vis b7,0");
        strcat(cmdBuffer, endMarker);
        strcat(cmdBuffer, "vis b8,0");
        strcat(cmdBuffer, endMarker);
        strcat(cmdBuffer, "vis n2,0");
        strcat(cmdBuffer, endMarker);
        strcat(cmdBuffer, "vis n4,0");
        strcat(cmdBuffer, endMarker);
    }
    else
    {
        // Show controls for other modes
        strcat(cmdBuffer, "vis b3,1");
        strcat(cmdBuffer, endMarker);
        strcat(cmdBuffer, "vis b4,1");
        strcat(cmdBuffer, endMarker);
        strcat(cmdBuffer, "vis b7,1");
        strcat(cmdBuffer, endMarker);
        strcat(cmdBuffer, "vis b8,1");
        strcat(cmdBuffer, endMarker);
        strcat(cmdBuffer, "vis n2,1");
        strcat(cmdBuffer, endMarker);
        strcat(cmdBuffer, "vis n4,1");
        strcat(cmdBuffer, endMarker);
    }

    // Send the second batch of commands
    NEXTION.print(cmdBuffer);
    cmdBuffer[0] = '\0'; // Reset buffer

    // 4. Draw the curve based on interpolation type

    if (InterpolationTypes[Bank][ChanneltoSet - 1] == STRAIGHTLINES)
    { // Linear
        // Draw just 4 straight lines - already efficient, just batch them
        sprintf(tempCmd, "line %d,%d,%d,%d,%d",
                (int)xPoints[0], (int)yPoints[0], (int)xPoints[1], (int)yPoints[1], ForeGroundColour);
        strcat(cmdBuffer, tempCmd);
        strcat(cmdBuffer, endMarker);

        sprintf(tempCmd, "line %d,%d,%d,%d,%d",
                (int)xPoints[1], (int)yPoints[1], (int)xPoints[2], (int)yPoints[2], ForeGroundColour);
        strcat(cmdBuffer, tempCmd);
        strcat(cmdBuffer, endMarker);

        sprintf(tempCmd, "line %d,%d,%d,%d,%d",
                (int)xPoints[2], (int)yPoints[2], (int)xPoints[3], (int)yPoints[3], ForeGroundColour);
        strcat(cmdBuffer, tempCmd);
        strcat(cmdBuffer, endMarker);

        sprintf(tempCmd, "line %d,%d,%d,%d,%d",
                (int)xPoints[3], (int)yPoints[3], (int)xPoints[4], (int)yPoints[4], ForeGroundColour);
        strcat(cmdBuffer, tempCmd);
        strcat(cmdBuffer, endMarker);
    }
    else if (InterpolationTypes[Bank][ChanneltoSet - 1] == SMOOTHEDCURVES)
    { // CatmullSpline
        // Use adaptive step size for smooth curves
        yDot2 = 0;
        // Increase step size to reduce total points
        int adaptiveStep = 12; // Larger than original 8 to reduce points

        for (xPoint = xPoints[0]; xPoint <= xPoints[4]; xPoint += adaptiveStep)
        {
            if (adaptiveStep > xPoints[4] - xPoint)
            {
                adaptiveStep = xPoints[4] - xPoint;
            }
            if (adaptiveStep < 1)
                adaptiveStep = 1;

            yPoint = Interpolation::CatmullSpline(xPoints, yPoints, PointsCount, xPoint);
            xDot1 = xPoint;
            yDot1 = yPoint;

            if (yDot2 == 0)
            {
                xDot2 = xDot1;
                yDot2 = yDot1;
                continue; // Skip first point, nothing to draw yet
            }

            sprintf(tempCmd, "line %d,%d,%d,%d,%d", xDot1, yDot1, xDot2, yDot2, ForeGroundColour);
            strcat(cmdBuffer, tempCmd);
            strcat(cmdBuffer, endMarker);

            // Send if buffer getting full
            if (strlen(cmdBuffer) > 400)
            {
                NEXTION.print(cmdBuffer);
                cmdBuffer[0] = '\0'; // Reset buffer
            }

            xDot2 = xDot1;
            yDot2 = yDot1;
        }
    }
    else if (InterpolationTypes[Bank][ChanneltoSet - 1] == EXPONENTIALCURVES)
    { // EXPO
        // For exponential curves
        CheckInvisiblePoint();

        HalfXRange = xPoints[4] - xPoints[2];
        TopHalfYRange = yPoints[4] - yPoints[2];
        BottomHalfYRange = yPoints[2] - yPoints[0];
        yDot2 = 0;

        // Use adaptive step size but with fewer points
        int adaptiveStep = 14; // Increased from APPROXIMATION = 7

        // Left half of exponential curve
        for (xPoint = 0; xPoint <= HalfXRange; xPoint += adaptiveStep)
        {
            yPoint = MapWithExponential(HalfXRange - xPoint, HalfXRange, 0, 0, BottomHalfYRange,
                                        Exponential[Bank][ChanneltoSet - 1]);

            if (adaptiveStep > HalfXRange - xPoint)
            {
                adaptiveStep = HalfXRange - xPoint;
            }
            if (adaptiveStep < 1)
                adaptiveStep = 1;

            yDot1 = yPoint + yPoints[0];
            xDot1 = xPoint + xPoints[0];

            if (yDot2 == 0)
            {
                xDot2 = xDot1;
                yDot2 = yDot1;
                continue; // Skip first point, nothing to draw yet
            }

            sprintf(tempCmd, "line %d,%d,%d,%d,%d", xDot1, yDot1, xDot2, yDot2, ForeGroundColour);
            strcat(cmdBuffer, tempCmd);
            strcat(cmdBuffer, endMarker);

            // Send if buffer getting full
            if (strlen(cmdBuffer) > 400)
            {
                NEXTION.print(cmdBuffer);
                cmdBuffer[0] = '\0'; // Reset buffer
            }

            xDot2 = xDot1;
            yDot2 = yDot1;
        }

        // Reset for right half of exponential curve
        adaptiveStep = 14;
        yDot2 = 0;

        // Right half of exponential curve
        for (xPoint = HalfXRange; xPoint >= 0; xPoint -= adaptiveStep)
        {
            yPoint = MapWithExponential(xPoint, 0, HalfXRange, 0, TopHalfYRange,
                                        Exponential[Bank][ChanneltoSet - 1]);

            if (adaptiveStep > xPoint)
            {
                adaptiveStep = xPoint;
            }
            if (adaptiveStep < 1)
                adaptiveStep = 1;

            yDot1 = yPoint + yPoints[2];
            xDot1 = xPoint + xPoints[2];

            if (yDot2 == 0)
            {
                xDot2 = xDot1;
                yDot2 = yDot1;
                continue; // Skip first point, nothing to draw yet
            }

            sprintf(tempCmd, "line %d,%d,%d,%d,%d", xDot1, yDot1, xDot2, yDot2, ForeGroundColour);
            strcat(cmdBuffer, tempCmd);
            strcat(cmdBuffer, endMarker);

            // Send if buffer getting full
            if (strlen(cmdBuffer) > 400)
            {
                NEXTION.print(cmdBuffer);
                cmdBuffer[0] = '\0'; // Reset buffer
            }

            xDot2 = xDot1;
            yDot2 = yDot1;
        }
    }

    // Send any remaining curve drawing commands
    if (strlen(cmdBuffer) > 0)
    {
        NEXTION.print(cmdBuffer);
        cmdBuffer[0] = '\0'; // Reset buffer
    }

    // 5. Draw the control points (dots)

    if (InterpolationTypes[Bank][ChanneltoSet - 1] == EXPONENTIALCURVES)
    {
        // Just 3 dots for exponential mode
        sprintf(tempCmd, "cirs %d,%d,%d,%d", (int)xPoints[0], (int)yPoints[0], DotSize, DotColour);
        strcat(cmdBuffer, tempCmd);
        strcat(cmdBuffer, endMarker);

        sprintf(tempCmd, "cirs %d,%d,%d,%d", (int)xPoints[2], (int)yPoints[2], DotSize, DotColour);
        strcat(cmdBuffer, tempCmd);
        strcat(cmdBuffer, endMarker);

        sprintf(tempCmd, "cirs %d,%d,%d,%d", (int)xPoints[4], (int)yPoints[4], DotSize, DotColour);
        strcat(cmdBuffer, tempCmd);
        strcat(cmdBuffer, endMarker);
    }
    else
    {
        // 5 dots for other modes
        for (int i = 0; i < 5; i++)
        {
            sprintf(tempCmd, "cirs %d,%d,%d,%d", (int)xPoints[i], (int)yPoints[i], DotSize, DotColour);
            strcat(cmdBuffer, tempCmd);
            strcat(cmdBuffer, endMarker);
        }
    }

    // Send the dots batch
    NEXTION.print(cmdBuffer);
    cmdBuffer[0] = '\0'; // Reset buffer

    // Create a special command to highlight the selected point with a filled circle
    char fill_cmd[100];
    int selected_x = (int)xPoints[CurrentPoint - 1];
    int selected_y = (int)yPoints[CurrentPoint - 1];

    // Create a circular highlight with concentric circles
    // Draw a large outer circle with white color for visibility on any background
    sprintf(fill_cmd, "cirs %d,%d,%d,%d",
            selected_x, selected_y, 10, White);
    SendCommand(fill_cmd);

    // Draw a medium circle with highlight color
    sprintf(fill_cmd, "cirs %d,%d,%d,%d",
            selected_x, selected_y, 8, HighlightColour);
    SendCommand(fill_cmd);

    // Draw another inner circle with highlight color
    sprintf(fill_cmd, "cirs %d,%d,%d,%d",
            selected_x, selected_y, 6, HighlightColour);
    SendCommand(fill_cmd);

    // Then redraw the selected dot over the highlight in a contrasting color
    sprintf(fill_cmd, "cirs %d,%d,%d,%d",
            selected_x, selected_y, DotSize + 3, Black);
    SendCommand(fill_cmd);

    // Update interpolation types UI
    updateInterpolationTypes();
}

/*********************************************************************************************************************************/

void BindNow()
{
    BoundFlag = true;
    ModelMatched = true;
    Connected = true;
#ifdef DB_BIND
    Serial.println("");
    Serial.println("Remote (model) ID saved:");
    for (int i = 0; i < 6; ++i)
    {
        Serial.print(ModelsMacUnionSaved.Val8[i], HEX);
        Serial.print(" ");
    }
    Serial.println(" ");
    Serial.println("(All 6 bytes of Teensy MAC ID used without modification.) ");
#endif
}
/******************************************************************************************************************************/
void DeleteModelID()
{

    // This deletes current model ID
    char prompt[60];
    char p[] = "Delete ID for ";
    char p1[] = "?";
    char Done[] = "Model ID deleted.";
    char DoneAlready[] = "No model ID not found.";
    char NotDone[] = "Model ID retained.";

    strcpy(prompt, p);
    strcat(prompt, ModelName);
    strcat(prompt, p1);

    if (!ModelsMacUnionSaved.Val64)
    {
        MsgBox(pRXSetupView, DoneAlready);
        return;
    }
    if (GetConfirmation(pRXSetupView, prompt))
    {
        ModelsMacUnionSaved.Val64 = 0;
        SaveOneModel(ModelNumber);
        MsgBox(pRXSetupView, Done);
    }
    else
    {
        MsgBox(pRXSetupView, NotDone);
    }
}
// *********************************************************************************************************************************/
void ClearDuplicateModelIDs(uint64_t ThisModelID)
{
    // This clears any duplicate model IDs
    bool Found = false;
    SavedModelNumber = ModelNumber; // save current
    for (int i = 0; i < MAXMODELNUMBER; ++i)
    {
        ReadOneModel(i);
        if (ModelsMacUnionSaved.Val64 == ThisModelID)
        {
            ModelsMacUnionSaved.Val64 = 0;
            SaveOneModel(i);
            Found = true;
        }
    }
    if (Found)
        MsgBox(pRXSetupView, (char *)"Duplicate Model ID(s) cleared!");
    ModelNumber = SavedModelNumber; // restore current model number
    ReadOneModel(ModelNumber);      // reload current model
}

/*********************************************************************************************************************************/

void StoreModelID()
{ // This stores current model ID

    char prompt[60];
    char p[] = "Store ID for ";
    char p1[] = "?";
    char Done[] = "Model ID stored.";
    char NotDone[] = "Model ID not stored.";
    char DoneAlready[] = "ID was already stored.";
    char NotConnected[] = "No ID to store.";

    strcpy(prompt, p);
    strcat(prompt, ModelName);
    strcat(prompt, p1);

    if (ModelsMacUnion.Val64 == ModelsMacUnionSaved.Val64)
    {
        MsgBox(pRXSetupView, DoneAlready);
        return;
    }
    if (!ModelsMacUnion.Val64)
    {
        MsgBox(pRXSetupView, NotConnected);
        return;
    }

    if (GetConfirmation(pRXSetupView, prompt))
    {
        ClearDuplicateModelIDs(ModelsMacUnion.Val64); // ensure no duplicates
        PlaySound(MMSAVED);
        ModelsMacUnionSaved.Val64 = ModelsMacUnion.Val64;
        SaveOneModel(ModelNumber);
        MsgBox(pRXSetupView, Done);
    }
    else
    {
        MsgBox(pRXSetupView, NotDone);
    }
}
/*********************************************************************************************************************************/
int GetDifference(int YtouchPlace, int oldy)
{
    int dd;
    dd = (YtouchPlace - oldy) / 2;
    if (dd < 0)
        dd = -dd;
    return dd;
}
/*********************************************************************************************************************************/
void MoveCurrentPointUp()
{

    switch (CurrentPoint)
    {
    case 1:
        ++MinDegrees[Bank][ChanneltoSet - 1];
        break;

    case 2:
        ++MidLowDegrees[Bank][ChanneltoSet - 1];
        break;

    case 3:
        ++CentreDegrees[Bank][ChanneltoSet - 1];
        break;
    case 4:
        ++MidHiDegrees[Bank][ChanneltoSet - 1];
        break;

    case 5:
        ++MaxDegrees[Bank][ChanneltoSet - 1];
        break;
    default:
        break;
    }
    DisplayCurve();
}

/*********************************************************************************************************************************/
void MoveCurrentPointDown()
{
    switch (CurrentPoint)
    {
    case 1:
        if (MinDegrees[Bank][ChanneltoSet - 1])
            --MinDegrees[Bank][ChanneltoSet - 1];
        break;

    case 2:
        if (MidLowDegrees[Bank][ChanneltoSet - 1])
            --MidLowDegrees[Bank][ChanneltoSet - 1];
        break;

    case 3:
        if (CentreDegrees[Bank][ChanneltoSet - 1])
            --CentreDegrees[Bank][ChanneltoSet - 1];
        break;

    case 4:
        if (MidHiDegrees[Bank][ChanneltoSet - 1])
            --MidHiDegrees[Bank][ChanneltoSet - 1];
        break;

    case 5:
        if (MaxDegrees[Bank][ChanneltoSet - 1])
            --MaxDegrees[Bank][ChanneltoSet - 1];
        break;
    default:
        break;
    }
    DisplayCurve();
}

/*********************************************************************************************************************************/

/** @brief moves point very close to where user hit screen */
void MovePoint()
{
    int rjump = 0;
    GetDotPositions(); // current
    if (XtouchPlace > BOXWIDTH)
        return; // out of range
    if (XtouchPlace < BOXLEFT)
        return; // out of range
    if (YtouchPlace < BOXTOP)
        return; // out of range
    if (YtouchPlace > BOXHEIGHT)
        return; // out of range

    if (XtouchPlace < BOXLEFT + xPoints[0])
    { // do leftmost point  ?
        CurrentPoint = 1;

        rjump = GetDifference(YtouchPlace, yPoints[0]);
        if (YtouchPlace > yPoints[0])
        {
            if (MinDegrees[Bank][ChanneltoSet - 1] >= rjump)
                MinDegrees[Bank][ChanneltoSet - 1] -= rjump;
        }
        else
        {
            if (MinDegrees[Bank][ChanneltoSet - 1] <= (180 - rjump))
                MinDegrees[Bank][ChanneltoSet - 1] += rjump;
        }
    }

    if (XtouchPlace > xPoints[1] - BOXLEFT && XtouchPlace < xPoints[1] + BOXLEFT)
    { // do next point  ?
        CurrentPoint = 2;
        CheckInvisiblePoint();
        if (InterpolationTypes[Bank][ChanneltoSet - 1] == EXPONENTIALCURVES)
            return; //  expo = ignore this area
        rjump = GetDifference(YtouchPlace, yPoints[1]);
        if (YtouchPlace > yPoints[1])
        {
            if (MidLowDegrees[Bank][ChanneltoSet - 1] >= rjump)
                MidLowDegrees[Bank][ChanneltoSet - 1] -= rjump;
        }
        else
        {
            if (MidLowDegrees[Bank][ChanneltoSet - 1] <= 180 - rjump)
                MidLowDegrees[Bank][ChanneltoSet - 1] += rjump;
        }
    }

    if (XtouchPlace > xPoints[2] - BOXLEFT && XtouchPlace < xPoints[2] + BOXLEFT)
    { // do next point  ?
        CurrentPoint = 3;
        rjump = GetDifference(YtouchPlace, yPoints[2]);
        if (YtouchPlace > yPoints[2])
        {
            if (CentreDegrees[Bank][ChanneltoSet - 1] >= rjump)
                CentreDegrees[Bank][ChanneltoSet - 1] -= rjump;
        }
        else
        {
            if (CentreDegrees[Bank][ChanneltoSet - 1] <= 180 - rjump)
                CentreDegrees[Bank][ChanneltoSet - 1] += rjump;
        }
    }

    if (XtouchPlace > xPoints[3] - BOXLEFT && XtouchPlace < xPoints[3] + BOXLEFT)
    { // do next point  ?
        CurrentPoint = 4;
        CheckInvisiblePoint();
        if (InterpolationTypes[Bank][ChanneltoSet - 1] == EXPONENTIALCURVES)
            return; //  expo = ignore this area
        rjump = GetDifference(YtouchPlace, yPoints[3]);
        if (YtouchPlace > yPoints[3])
        {
            if (MidHiDegrees[Bank][ChanneltoSet - 1] >= rjump)
                MidHiDegrees[Bank][ChanneltoSet - 1] -= rjump;
        }
        else
        {
            if (MidHiDegrees[Bank][ChanneltoSet - 1] <= 180 - rjump)
                MidHiDegrees[Bank][ChanneltoSet - 1] += rjump;
        }
    }
    if (XtouchPlace > xPoints[3] + BOXLEFT) // do hi point  ?
    {
        CurrentPoint = 5;
        rjump = GetDifference(YtouchPlace, yPoints[4]);
        if (YtouchPlace > yPoints[4])
        {
            if (MaxDegrees[Bank][ChanneltoSet - 1] > rjump)
                MaxDegrees[Bank][ChanneltoSet - 1] -= rjump;
        }
        else
        {
            if (MaxDegrees[Bank][ChanneltoSet - 1] <= 180 - rjump)
                MaxDegrees[Bank][ChanneltoSet - 1] += rjump;
        }
    }
}

/*********************************************************************************************************************************/

void SoundBank()
{
    if (millis() < 10000)
        return; // don't announce bank if booted < 10 seconds ago
    PlaySound(BankSounds[BanksInUse[Bank - 1]]);
    ScreenTimeTimer = millis(); // reset screen counter to make sure its updated soon
}
/*********************************************************************************************************************************/
void ShowBank()
{
    char FMPress[4][12] = {"click fm1,1", "click fm2,1", "click fm3,1", "click fm4,1"};
    SendCommand(FMPress[Bank - 1]);
}
/*********************************************************************************************************************************/
void ShowMotor(int on)
{
    char bt0[] = "bt0";
    char OnMsg[] = "Motor ON";
    char OffMsg[] = "Motor OFF";
    if ((on == 1) || (!UseMotorKill))
        SendText(bt0, OnMsg);
    if (on == 0)
        SendText(bt0, OffMsg);
}
/*********************************************************************************************************************************/

void DoOneSwitchView(uint8_t n) // n is 1-4  = number for switch to edit
{
    char chLabels[4][3] = {"t3", "t4", "t5", "t6"};
    char chValues[4][11] = {"Channel 9", "Channel 10", "Channel 11", "Channel 12"};
    char Rlabels[10][3] = {"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"};
    char OneSwitchViewc_revd[] = "c_revd"; // Reversed

    for (int i = 0; i < 10; ++i)
        SendValue(Rlabels[i], 0); // clear all

    ValueSent = false; // If no setting, = 'Not Used'

    if ((BankSwitch == n) && (!ValueSent))
        SendValue(Rlabels[1], 1); // No duplicates allowed!
    if ((Autoswitch == n) && (!ValueSent))
        SendValue(Rlabels[2], 1); // No duplicates allowed!
    if ((TopChannelSwitch[Ch9_SW] == n) && (!ValueSent))
        SendValue(Rlabels[3], 1); // No duplicates allowed!
    if ((TopChannelSwitch[Ch10_SW] == n) && (!ValueSent))
        SendValue(Rlabels[4], 1); // No duplicates allowed!
    if ((TopChannelSwitch[Ch11_SW] == n) && (!ValueSent))
        SendValue(Rlabels[5], 1); // No duplicates allowed!
    if ((TopChannelSwitch[Ch12_SW] == n) && (!ValueSent))
        SendValue(Rlabels[6], 1); // No duplicates allowed!
    if ((SafetySwitch == n) && (!ValueSent))
        SendValue(Rlabels[7], 1); // No duplicates allowed!
    if ((DualRatesSwitch == n) && (!ValueSent))
        SendValue(Rlabels[8], 1); // No duplicates allowed!
    if ((BuddySwitch == n) && (!ValueSent))
        SendValue(Rlabels[9], 1); // No duplicates allowed!

    if (!ValueSent)
        SendValue(Rlabels[0], 1); // nothing yet, so 'not used' is selected

    SendValue(OneSwitchViewc_revd, 0);

    if (n >= 1 && n <= 4 && SwitchReversed[n - 1]) // shows the Reversed check box on the one switch screen
        SendValue(OneSwitchViewc_revd, 1);
    else
        SendValue(OneSwitchViewc_revd, 0); // ... or not

    for (int i = 0; i < 4; ++i)
    { // show channel names
        SendText(chLabels[i], chValues[i]);
        if (strlen(ChannelNames[i + 8]) >= 2)
            SendText(chLabels[i], ChannelNames[i + 8]); // Show EDITED channel names if they exist
    }
}

/*********************************************************************************************************************************/
void UpdateOneSwitchView()
{
    char SwNum[] = "Sw";
    SendValue(SwNum, SwitchEditNumber); // show switch number
    DoOneSwitchView(SwitchEditNumber);
}

/*********************************************************************************************************************************/

void ZeroDataScreen()
{ // ZERO Those parameters that are zeroable

    TotalGoodPackets = 0;
    RecentPacketsLost = 0;
    TotalLostPackets = 0;
    GapLongest = 0;
    GapSum = 0;
    GapAverage = 0;
    GapCount = 0;
    GapStart = 0;
    ThisGap = 0;
    RXMAXModelAltitude = 0;
    MaxRateOfClimb = 0;
    RXModelAltitude = 0;
    GroundModelAltitude = 0;
    GPS_RX_MaxDistance = 0;
    GPS_RX_MaxSpeed = 0;
    LastShowTime = 0;
    TotalFrameRate = 0;
    FrameRateCounter = 0;
    PacketsPerSecond = 0;
    MaxBin = 100;
    GapShortest = 1000;
    for (int i = 0; i < 11; ++i)
    {
        GapSets[i] = 0;
        PrevGapSets[i] = 0xffff;
    }
}
/***************************************************** ReadNewSwitchFunction ****************************************************************************/

void ReadNewSwitchFunction()
{
    char OneSwitchView_r1[] = "r1"; // Flight modes
    char OneSwitchView_r2[] = "r2"; // Auto
    char OneSwitchView_r3[] = "r3"; // Ch9
    char OneSwitchView_r4[] = "r4"; // Ch10
    char OneSwitchView_r5[] = "r5"; // Ch11
    char OneSwitchView_r6[] = "r6"; // Ch12
    char OneSwitchView_r7[] = "r7"; // Safety
    char OneSwitchView_r8[] = "r8"; // Dual Rates
    char OneSwitchView_r9[] = "r9"; // Buddy
    char ProgressStart[] = "vis Progress,1";
    char ProgressEnd[] = "vis Progress,0";
    char PageSwitchView[] = "page SwitchesView";
    char OneSwitchViewc_revd[] = "c_revd"; // Reversed
    char Progress[] = "Progress";

    SendCommand(ProgressStart);
    SendValue(Progress, 10);

    if (GetValue(OneSwitchView_r1))
    {
        BankSwitch = SwitchEditNumber;
    }
    else
    {
        if (BankSwitch == SwitchEditNumber)
            BankSwitch = 0;
    }

    SendValue(Progress, 15);
    if (GetValue(OneSwitchView_r2))
    {
        Autoswitch = SwitchEditNumber;
    }
    else
    {
        if (Autoswitch == SwitchEditNumber)
            Autoswitch = 0;
    }
    SendValue(Progress, 25);
    if (GetValue(OneSwitchView_r3))
    {
        TopChannelSwitch[Ch9_SW] = SwitchEditNumber;
    }
    else
    {
        if (TopChannelSwitch[Ch9_SW] == SwitchEditNumber)
            TopChannelSwitch[Ch9_SW] = 0;
    }
    SendValue(Progress, 30);
    if (GetValue(OneSwitchView_r4))
    {
        TopChannelSwitch[Ch10_SW] = SwitchEditNumber;
    }
    else
    {
        if (TopChannelSwitch[Ch10_SW] == SwitchEditNumber)
            TopChannelSwitch[Ch10_SW] = 0;
    }
    SendValue(Progress, 40);
    if (GetValue(OneSwitchView_r5))
    {
        TopChannelSwitch[Ch11_SW] = SwitchEditNumber;
    }
    else
    {
        if (TopChannelSwitch[Ch11_SW] == SwitchEditNumber)
            TopChannelSwitch[Ch11_SW] = 0;
    }
    SendValue(Progress, 50);
    if (GetValue(OneSwitchView_r6))
    {
        TopChannelSwitch[Ch12_SW] = SwitchEditNumber;
    }
    else
    {
        if (TopChannelSwitch[Ch12_SW] == SwitchEditNumber)
            TopChannelSwitch[Ch12_SW] = 0;
    }
    SendValue(Progress, 60);
    if (GetValue(OneSwitchView_r7))
    {
        SafetySwitch = SwitchEditNumber;
    }
    else
    {
        if (SafetySwitch == SwitchEditNumber)
            SafetySwitch = 0;
    }
    SendValue(Progress, 70);
    if (GetValue(OneSwitchView_r8))
    {
        DualRatesSwitch = SwitchEditNumber;
    }
    else
    {
        if (DualRatesSwitch == SwitchEditNumber)
            DualRatesSwitch = 0;
    }
    SendValue(Progress, 80);
    if (GetValue(OneSwitchView_r9))
    {
        BuddySwitch = SwitchEditNumber;
    }
    else
    {
        if (BuddySwitch == SwitchEditNumber)
            BuddySwitch = 0;
    }

    SendValue(Progress, 90);

    if (SwitchEditNumber >= 1 && SwitchEditNumber <= 4)
    {
        SwitchReversed[SwitchEditNumber - 1] = GetValue(OneSwitchViewc_revd);
        if (SwitchEditNumber == 2)
            SendValue(Progress, 95);
    }

    SendValue(Progress, 100);
    SaveOneModel(ModelNumber);
    SendCommand(PageSwitchView); // change to all switches screen
    CurrentView = SWITCHES_VIEW;
    UpdateSwitchesView(); // update its info
    ClearText();
    SendCommand(ProgressEnd);
    return;
}

/*********************************************************************************************************************************/
FASTRUN void DisplayCurveAndServoPos()
{
    ClearBox();
    DelayWithDog(5);
    SavedLineX = 52735; // just to be massvely different
    ShowServoPos();     // this calls displaycurve!!!
    ClearText();
}

/******************************************************************************************************************************/

void UpdateLED()
{ // LED Brightness has changed so this ensures it is redisplayed
    char n1[] = "n1";
    LEDBrightness = GetValue(n1);
    LEDBrightness = CheckRange(LEDBrightness, 1, 254);
    LedWasGreen = false; // Forces a redisplay if brightness has changed
}

/******************************************************************************************************************************/
void ResetTransmitterSettings()
{ // This function resets all transmitter parameters to the default state.
  // Calibration should  be done next.

    const char Tn[32] = "Unknown";

    char prompt[] = "Delete all settings and models?!";
    int sofar = 0;
    char ProgressStart[] = "vis Progress,1";
    char Progress[] = "Progress";

    if (!GetConfirmation(pCalibrateView, prompt))
        return;
    SendCommand(ProgressStart);
    SendValue(Progress, 2);
    ModelNumber = 1;
    ScreenTimeout = 120;
    Inactivity_Timeout = INACTIVITYTIMEOUT;
    strcpy(TxName, Tn);
    Qnh = 1009;
    DeltaGMT = 0;
    BackGroundColour = 214;
    ForeGroundColour = White;
    SpecialColour = Red;
    HighlightColour = Yellow;
    SticksMode = 2;
    AudioVolume = 20;
    Brightness = 100;
    PlayFanfare = false;
    TrimClicks = true;
    UseVariometer = false;
    SpeakingClock = true;
    AnnounceBanks = true;
    ResetSwitchNumbers();
    MinimumGap = 50;
    LogRXSwaps = true;
    UseLog = false;
    AnnounceConnected = true;
    ResetAllTrims();
    TxVoltageCorrection = 0;
    PowerOffWarningSeconds = DEFAULTPOWEROFFWARNING;
    LEDBrightness = DEFAULTLEDBRIGHTNESS;
    ConnectionAssessSeconds = 1;
    AutoModelSelect = false;
    MotorChannel = 2; // = 3 really
    MotorChannelZero = 0;
    TimerDownwards = false;
    char ProgressEnd[] = "vis Progress,0";

    SetDS1307ToCompilerTime();
    for (int k = 1; k < 5; ++k)
    { // writes default four times!
        for (ModelNumber = 1; ModelNumber <= MAXMODELNUMBER; ++ModelNumber)
        {
            ++sofar;
            SetDefaultValues();
            SaveOneModel(ModelNumber);
            SendValue(Progress, (sofar * (100 / MAXMODELNUMBER)) / 4);
            KickTheDog();
        }
        CloseModelsFile();
    }

    Autoswitch = 1;
    SafetySwitch = 2;
    BuddySwitch = 3;
    BankSwitch = 4;
    TopChannelSwitch[Ch9_SW] = 0;
    TopChannelSwitch[Ch10_SW] = 0;
    TopChannelSwitch[Ch11_SW] = 0;
    TopChannelSwitch[Ch12_SW] = 0;
    for (int i = 0; i < 4; ++i)
        SwitchReversed[i] = false;

    SendValue(Progress, 100);
    ModelNumber = 1;
    SaveTransmitterParameters();
    SaveTransmitterParameters();
    SendCommand(ProgressEnd);
}

/*********************************************************************************************************************************/

void PointUp()
{
    MoveCurrentPointUp();
}
/******************************************************************************************************************************/

void PointDown()
{
    MoveCurrentPointDown();
}

/******************************************************************************************************************************/

void CheckInvisiblePoint()
{
    if (InterpolationTypes[Bank][ChanneltoSet - 1] == EXPONENTIALCURVES)
    {
        if ((CurrentPoint == 2) || (CurrentPoint == 4))
            ++CurrentPoint;
    }
}

/******************************************************************************************************************************/

void PointSelect()
{
    ++CurrentPoint;
    if (CurrentPoint > 5)
        CurrentPoint = 1;
    CheckInvisiblePoint();
    DisplayCurve();
}

/******************************************************************************************************************************/

void GotoGPSView()
{
    char NoFixMsg[] = "Continue without GPS fix?";

    if (!GPS_RX_FIX)
    {
        if (!GetConfirmation(pRXSetupView, NoFixMsg))
            return;
    }
    char GotoGPSView[] = "page GPSView";
    SendCommand(GotoGPSView);
    CurrentView = GPSVIEW;
}

/******************************************************************************************************************************/

void StartBankNames()
{

    char GotoBankNames[] = "page BankNameView";
    char BKS[4][4] = {{"BK1"}, {"BK2"}, {"BK3"}, {"BK4"}};

    SendCommand(GotoBankNames);
    CurrentView = BANKSNAMESVIEW;

    for (int i = 0; i < 4; ++i)
    {
        SendValue(BKS[i], BanksInUse[i]);
    }
    UpdateModelsNameEveryWhere();
}

/******************************************************************************************************************************/

void EndBankNames()
{

    char BKS[4][4] = {{"BK1"}, {"BK2"}, {"BK3"}, {"BK4"}};
    for (int i = 0; i < 4; ++i)
    {
        BanksInUse[i] = GetValue(BKS[i]);
    }
    SaveOneModel(ModelNumber);
    StartModelSetup();
}

/******************************************************************************************************************************/

void ListenToBanks()
{
    char BKS[4][4] = {{"BK1"}, {"BK2"}, {"BK3"}, {"BK4"}};
    for (int i = 0; i < 4; ++i)
    {
        BanksInUse[i] = GetValue(BKS[i]);
        PlaySound(BankSounds[BanksInUse[i]]);
        DelayWithDog(1200);
    }
}
// ******************************************************************************************************************************/
void SaveFailSafeChannels()
{
    char fs[16][5] = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
    char ProgressStart[] = "vis Progress,1";
    char ProgressEnd[] = "vis Progress,0";
    char Progress[] = "Progress";
    char chgs[512];
    char change[] = "change";
    char cleared[] = "XX:";
    for (uint16_t i = 0; i < 500; ++i)
    { // get copy of any changes
        chgs[i] = TextIn[i + 4];
        chgs[i + 1] = 0;
    }
    SendCommand(ProgressStart);
    for (int i = 0; i < 16; ++i)
    {
        if (InStrng(fs[i], chgs)) // if this channel has been changed
        {
            FailSafeChannel[i] = GetValue(fs[i]);
        }
        SendValue(Progress, i * (100 / 16));
    }
    SendText(change, cleared);
    SendCommand(ProgressEnd);
}

/******************************************************************************************************************************/
void StartModelSetup()
{

    if (CurrentView == FAILSAFE_VIEW)
    { //  read failsafe blobs
        SaveFailSafeChannels();
    }

    if (CurrentView == MIXESVIEW)
    { //  read mixes
        ReadMixValues();
    }

    SendCommand(pRXSetupView);
    CurrentView = RXSETUPVIEW;
    SaveOneModel(ModelNumber);
    UpdateModelsNameEveryWhere();
    LastTimeRead = 0;
}

/******************************************************************************************************************************/
void EndModelSetup()
{
    GotoFrontView();
}

/******************************************************************************************************************************/
void UpdateSpeedScreen()
{
    char ns[16][4] = {{"n0"}, {"n1"}, {"n2"}, {"n3"}, {"n4"}, {"n5"}, {"n6"}, {"n7"}, {"n8"}, {"n9"}, {"n10"}, {"n11"}, {"n12"}, {"n13"}, {"n14"}, {"n15"}};
    char t14[] = "t14";
    CheckServoSpeeds();
    for (int i = 0; i < 16; ++i)
    {
        SendValue(ns[i], ServoSpeed[Bank - 1][i]);
    }
    SendText(t14, BankNames[BanksInUse[Bank - 1]]);
}

/******************************************************************************************************************************/
void StartSlowView()
{
    char GoSlowServoScreen[] = "page SlowServoView";
    SendCommand(GoSlowServoScreen);
    CurrentView = SLOWSERVOVIEW;
    UpdateButtonLabels();
    UpdateModelsNameEveryWhere();
    UpdateSpeedScreen();
}

/******************************************************************************************************************************/
void ReadSpeedsScreen(uint8_t bk)
{

    char ns[16][4] = {{"n0"}, {"n1"}, {"n2"}, {"n3"}, {"n4"}, {"n5"}, {"n6"}, {"n7"}, {"n8"}, {"n9"}, {"n10"}, {"n11"}, {"n12"}, {"n13"}, {"n14"}, {"n15"}};
    char Progress[] = "Progress";
    char ProgressStart[] = "vis Progress,1";
    char ProgressEnd[] = "vis Progress,0";
    char chgs[512];
    char change[] = "change";
    char cleared[] = "XX:";
    for (uint16_t i = 0; i < 500; ++i)
    { // get copy of any changes
        chgs[i] = TextIn[i + 4];
        chgs[i + 1] = 0;
    }
    SendCommand(ProgressStart);
    for (int i = 0; i < 16; ++i)
    {
        if (InStrng(ns[i], chgs)) // if this channel has been changed
        {
            ServoSpeed[bk][i] = GetValue(ns[i]);
        }
        SendValue(Progress, i * (100 / 16));
    }
    SendText(change, cleared);
    SendValue(Progress, 100);
    SendCommand(ProgressEnd);
}

/******************************************************************************************************************************/
void EndSpeedsScreen()
{
    ReadSpeedsScreen(Bank - 1);
    CheckServoSpeeds();
    SaveOneModel(ModelNumber);
    StartModelSetup();
}
/******************************************************************************************************************************/
void WriteNewCurve()
{
    char CopyToAllBanks[] = "callfm";
    char pSticksView[] = "page SticksView";

    if (GetValue(CopyToAllBanks))
    {
        for (int p = 1; p <= 4; ++p)
        {
            if (p != Bank)
            {
                MinDegrees[p][ChanneltoSet - 1] = MinDegrees[Bank][ChanneltoSet - 1];
                MidLowDegrees[p][ChanneltoSet - 1] = MidLowDegrees[Bank][ChanneltoSet - 1];
                CentreDegrees[p][ChanneltoSet - 1] = CentreDegrees[Bank][ChanneltoSet - 1];
                MidHiDegrees[p][ChanneltoSet - 1] = MidHiDegrees[Bank][ChanneltoSet - 1];
                MaxDegrees[p][ChanneltoSet - 1] = MaxDegrees[Bank][ChanneltoSet - 1];
                InterpolationTypes[p][ChanneltoSet - 1] = InterpolationTypes[Bank][ChanneltoSet - 1];
                Exponential[p][ChanneltoSet - 1] = Exponential[Bank][ChanneltoSet - 1];
            }
        }
    }
    ChanneltoSet = 0;
    SaveOneModel(ModelNumber);
    Force_ReDisplay();
    SendCommand(pSticksView); // Set to SticksView
    CurrentView = STICKSVIEW;
    UpdateModelsNameEveryWhere();
    ClearText();
}

/******************************************************************************************************************************/
void StartRenameModel()
{

    char GoRenameModelScreen[] = "page RenameView";
    char NewName[] = "NewName";
    SendCommand(GoRenameModelScreen);
    CurrentView = RENAMEMODELVIEW;
    SendText(NewName, ModelName);
}

/******************************************************************************************************************************/

void GetDefaultFilename()
{ // Build filename from ModelName as best we can using first 8 chars upper cased
    uint8_t j = 0;
    uint8_t i = 0;
    char mod[] = ".MOD";
    while (i < 8)
    {
        if (ModelName[j] > 32)
        {
            SingleModelFile[i] = toUpperCase(ModelName[j]);
            ++i;
            SingleModelFile[i] = 0;
        }
        if (ModelName[j] < 32)
            break;
        ++j;
        if (i >= 8)
            break;
    }
    strcat(SingleModelFile, mod);
}

/******************************************************************************************************************************/

void FixFileName()
{
    char ModExt[] = ".MOD";
    for (uint8_t i = 0; i < strlen(SingleModelFile); ++i)
        SingleModelFile[i] = toUpperCase(SingleModelFile[i]);
    if (!InStrng(ModExt, SingleModelFile) && (strlen(SingleModelFile) <= 8))
        strcat(SingleModelFile, ModExt);
}

/******************************************************************************************************************************/
void WriteBackup()
{

    char ModExt[] = ".MOD";
    uint8_t Iterations = 4;
    char ProgressStart[] = "vis Progress,1";
    char ProgressEnd[] = "vis Progress,0";
    char Progress[] = "Progress";
    SendValue(Progress, 1);
    FixFileName();
    if ((strlen(SingleModelFile) <= 12) && (InStrng(ModExt, SingleModelFile) > 0))
    {
        SendCommand(ProgressStart);
        for (int i = 0; i < Iterations; i++)
        {
            CloseModelsFile();
            SingleModelFlag = true;
            SaveOneModel(1);
            SendValue(Progress, i * (100 / Iterations)); // write several times is needed!! :-)
        }
        CloseModelsFile();
        SingleModelFlag = false;
    }
    else
    {
        FileError = true;
    }
    if (FileError)
        ShowFileErrorMsg();
    SendValue(Progress, 100);
    DelayWithDog(100);
    SendCommand(ProgressEnd);
    LastFileInView = 120;
}

/******************************************************************************************************************************/
void EndRenameModel()
{
    char NewName[31] = "NewName";
    GetText(NewName, ModelName);
    SaveOneModel(ModelNumber);
    GotoModelsView();
}

/******************************************************************************************************************************/

void LoadModelForRenaming()
{
    CloseModelsFile();
    SingleModelFlag = true;
    ReadOneModel(1);
}
/******************************************************************************************************************************/

void RestoreCurrentModel()
{
    CloseModelsFile();
    ModelNumber = SavedModelNumber;
    SingleModelFlag = false;
    ReadOneModel(ModelNumber);
    SaveAllParameters();
}

/******************************************************************************************************************************/
// Function to display all model IDs and check for duplcates

void CheckAllModelIds()
{
    unsigned int TempModelId;
    char Vbuf[15];
    char MMemsp[] = "MMems.path=\"";
    char GoIDview[] = "page IDsView";
    char crlf[] = {13, 10, 0};
    char lb[] = "(";
    char rb[] = ")  ";
    char KO[] = ">Duplicate:";
    char Okay[] = " (ID OK)";
    char n0[] = "n0";
    char nb[4];
    char buf[MAXBUFFERSIZE];
    uint64_t ModelIDs[92];
    uint8_t DuplicatesCount = 0;
    uint32_t SavedModelNumber = ModelNumber;

    SendCommand(GoIDview);
    CurrentView = IDCHECKVIEW;
    for (ModelNumber = 1; ModelNumber < MAXMODELNUMBER - 1; ++ModelNumber)
    {
        ReadOneModel(ModelNumber);
        ModelIDs[ModelNumber] = ModelsMacUnionSaved.Val64;
    }
    for (ModelNumber = 1; ModelNumber < MAXMODELNUMBER - 1; ++ModelNumber)
    {
        ReadOneModel(ModelNumber);
        if (ModelNumber == 1)
        {
            strcpy(buf, lb);
        }
        else
        {
            strcat(buf, lb);
        }
        Str(nb, ModelNumber, 0);
        strcat(buf, nb);
        strcat(buf, rb);
        strcat(buf, ModelName);
        TempModelId = ModelsMacUnionSaved.Val32[0];
        snprintf(Vbuf, 10, "%X", TempModelId);
        if (TempModelId)
        {
            strcat(buf, " ");
            strcat(buf, Vbuf);
        }
        TempModelId = ModelsMacUnionSaved.Val32[1];
        ModelIDs[ModelNumber] = ModelsMacUnionSaved.Val64;
        snprintf(Vbuf, 10, "%X", TempModelId);
        if (TempModelId)
        {
            strcat(buf, Vbuf);
            int p = 0;
            for (unsigned int i = 1; i < MAXMODELNUMBER - 1; ++i)
            {
                if ((ModelIDs[i] == ModelIDs[ModelNumber]) && (i != ModelNumber))
                    p = i;
            }
            if (p > 0)
            {
                strcat(buf, KO);
                strcat(buf, Str(nb, p, 0));
                strcat(buf, "<");
                ++DuplicatesCount;
            }
            else
            {
                strcat(buf, Okay);
            }
        }
        strcat(buf, crlf);
    }
    SendOtherText(MMemsp, buf);
    SendValue(n0, DuplicatesCount);
    ModelNumber = SavedModelNumber;
    ReadOneModel(ModelNumber);
}

// ******************************** Global Array1 of numbered function pointers OK up the **********************************

// This new list can be huge - up to 24 BITS unsigned!  ( Use "NUMBER<<8" )
#define LASTFUNCTION1 30 // One more than final one

void (*NumberedFunctions1[LASTFUNCTION1])(){
    Blank,                  // 0 Cannot be used
    DeleteModel,            // 1
    StartAudioVisualView,   // 2
    EndAudioVisualView,     // 3
    StartTXSetupView,       // 4
    InputsViewEnd,          // 5
    SystemPage1End,         // 6
    SystemPage1Start,       // 7
    StartWifiScan,          // 8
    EndWifiScan,            // 9
    StartServosTypeView,    // 10
    EndServoTypeView,       // 11
    LoadNewLogFile,         // 12
    DeleteThisLogFile,      // 13
    LogReleasedNEW,         // 14   // new version
    LogTouched,             // 15   // this does nothing, yet ...
    RefreshDualRatesNew,    // 16
    StartGapsView,          // 17
    StartPIDView,           // 18
    SendEditedPIDs,         // 19
    PIDs_Were_edited,       // 20
    EndPIDView,             // 21
    StartRatesView,         // 22
    EndRatesView,           // 23
    RatesWereEdited,        // 24
    SendEditedRates,        // 25
    RotorFlightStart,       // 26
    RotorFlightEnd,         // 27
    StartRatesAdvancedView, // 28
    StartPIDAdvancedView    // 29

};

// This list migth become MUCH longer as it limit is 24 bits big

// ******************************** Global Array of numbered function pointers - OK up to 127 functions ... **********************************

// This list can be only up to 127 (7 BITS) functions long

#define LASTFUNCTION 80 // One more than final one, because first is number zero

void (*NumberedFunctions[LASTFUNCTION])(){
    Blank,                    // 0  Cannot be used
    DoMFName,                 // 1
    ModelViewEnd,             // 2
    DoLastTimeRead,           // 3
    GotoModelsView,           // 4
    GotoMacrosView,           // 5
    PopulateMacrosView,       // 6
    ExitMacrosView,           // 7
    ShowChannelName,          // 8
    StartReverseView,         // 9
    EndReverseView,           // 10
    StartBuddyView,           // 11
    EndBuddyView,             // 12
    Blank,                    // 13  // spare
    Blank,                    // 14  //spare
    BottomOfLogFileNEW,       // 15  // goto bottom of log file
    LogEND,                   // 16
    StartLogFilesListScreen,  // 17  // was DelLOG. not now though.
    StartLogFileView,         // 18 // revised log view ****************************************************
    SetupViewFM,              // 19
    StartSubTrimView,         // 20
    EndSubTrimView,           // 21
    StartTrimDefView,         // 22
    Options2End,              // 23
    DefineTrimsEnd,           // 24
    OptionView2Start,         // 25
    OptionView3Start,         // 26
    OptionView3End,           // 27
    BuddyChViewStart,         // 28
    BuddyChViewEnd,           // 29
    RudderLeftTrim,           // 30
    RudderRightTrim,          // 31
    AileronRightTrim,         // 32
    AileronLeftTrim,          // 33
    ElevatorUpTrim,           // 34
    ElevatorDownTrim,         // 35
    ThrottleDownTrim,         // 36
    ThrottleUpTrim,           // 37
    RXOptionsViewStart,       // 38
    RXOptionsViewEnd,         // 39
    ResetTransmitterSettings, // 40
    Blank,                    // 41 Not CALLED FrOM Here now
    PointUp,                  // 42
    PointDown,                // 43
    PointSelect,              // 44
    DualRatesStart,           // 45
    DualRatesEnd,             // 46
    DualRatesRefresh,         // 47
    GotoFrontView,            // 48
    GotoGPSView,              // 49
    StartModelSetup,          // 50
    EndModelSetup,            // 51
    StartBankNames,           // 52
    EndBankNames,             // 53
    ListenToBanks,            // 54
    StartSlowView,            // 55
    EndSpeedsScreen,          // 56
    WriteNewCurve,            // 57
    StartRenameModel,         // 58
    EndRenameModel,           // 59
    YesPressed,               // 60
    NoPressed,                // 61
    RenameFile,               // 62
    CheckAllModelIds,         // 63
    Blank,                    // 64  // spare ****************************************************
    ReceiveModelFile,         // 65
    Blank,                    // 66  // spare ****************************************************
    Blank,                    // 67  // spare ****************************************************
    DeleteModelID,            // 68
    StartPong,                // 69
    StoreModelID,             // 70
    ResetClock,               // 71
    SaveSwitches,             // 72 (NOW SAVED FROM TX SETUP MENU GLOBALLY FOR ALL MODELS)
    ShowScreenAgain,          // 73 End of screen timeout when someone touched screen
    HideScreenAgain,          // 74 Force immediate 'screen timeout'
    TrimsToSubtrim,           // 75
    ReceiveLotsofData,        // 76
    SetNewDualRate,           // 77 // when the DualRate behavior scrollable thingy is changed
    EndLogFilesListScreen,    // 78
    TopOfLogFileNEW           // 79 // goto top of log file NEW

    // must stop at 126
}; // This list might become longer but a new one is now started above without the 127 limit

/*********************************************************************************************************************************
 *                          BUTTON WAS PRESSED (DEAL WITH INPUT FROM NEXTION DISPLAY)                                            *
 *********************************************************************************************************************************/
FASTRUN void ButtonWasPressed()
{
    int Command_number = GetIntFromTextIn(0);
    if (Command_number)
    { // is there anything ?
        StartInactvityTimeout();
        ScreenTimeTimer = millis();                       // reset screen timeout counter
        uint32_t NumberedCommand = Command_number & 0x7F; // NextionCommand.FirstDWord & 0x7F; // Just clear the hi BIT ( i.e. -128)
        uint32_t NumberedCommand1 = Command_number >> 8;  // Shift over to the right to get a number up to 24 BITS

#ifdef DB_NEXTION
        if ((TextIn[0] < 128) && (TextIn[0] > 28))
        {
            Look1("Command WORD: ");
            Look(TextIn);
        }

        if ((TextIn[0] > 128) && (TextIn[0] < 255))
        {
            Look1("7 BIT Command NUMBER: ");
            Look(NumberedCommand);
        }

        if (TextIn[0] == 0)
        {
            Look1("24 BIT command NUMBER: ");
            Look(NumberedCommand1);
        }
#endif
        if (!TextIn[0])
        { //  Now expanded to handle FAR more than 127 functions
            if (NumberedCommand1 < LASTFUNCTION1)
            {
                NumberedFunctions1[NumberedCommand1](); // Call the needed 24 BIT function -- with a function pointer
            }
            ClearText();
            return;
        }

        if (TextIn[0] >= 128)
        {
            if (NumberedCommand < LASTFUNCTION)
            {
                NumberedFunctions[NumberedCommand](); // Call the needed 7 BIT function -- with a function pointer
            }
            ClearText();
            return;
        }
        // By here, TextIn[0] must be start of a character based command

        // The very old code below here is gradually and carefully being replaced by new code in multiple functions - it works fine but is not as efficient as the new code

        int i = 0;
        int j = 0;
        int p = 0;
        char Setup[] = "Setup";
        char ClickX[] = "ClickX";
        char ClickY[] = "ClickY";
        char Reset[] = "Reset";
        char Front_View[] = "FrontView";
        char Sticks_View[] = "SticksView";
        char Graph_View[] = "GraphView";
        char SetupView[] = "MainSetup";
        char DataEnd[] = "DataEnd";
        char Data_View[] = "DataView";
        char CalibrateView[] = "CalibrateView";
        char TrimView[] = "TrimView";
        char TRIMS50[] = "TRIMS50";
        char MIXES_VIEW[] = "MIXESVIEW"; // first call
        char Mixes_View[] = "MixesView";
        char FM1[] = "FM 1";
        char FM2[] = "FM 2";
        char FM3[] = "FM 3";
        char FM4[] = "FM 4";
        char ReScan[] = "ReScan";
        char MixesView_MixNumber[] = "MixNumber";
        char ModelsView_ModelNumber[] = "ModelNumber";
        char ColoursView[] = "ColoursView";
        char SvT11[] = "t11";
        char CMsg1[] = "Move all controls to their full\r\nextent several times,\r\nthen press Next.";
        char SvB0[] = "b0";
        char CMsg2[] = "Next ...";
        char Cmsg3[] = "Centre all channels.\r\nPut edge switches fully back,\r\nor fully forward, then press Finish.";
        char Cmsg4[] = "Finish";
        char Cmsg5[] = "Repeat?";
        char Cmsg6[] = "Calbrate again?";
        char TypeView[] = "TypeView";
        char CopyToAllBanks[] = "callfm";
        char RXBAT[] = "RXBAT";
        char r2s[] = "r2s";
        char r3s[] = "r3s";
        char r4s[] = "r4s";
        char r5s[] = "r5s";
        char r6s[] = "r6s";
        char r12s[] = "r12s";
        char r0[] = "r0";
        char r1[] = "r1";
        char SwitchesView[] = "SwitchesView";
        char SwitchesView1[] = "SwitchesView1";
        char OneSwitchView[] = "OneSwitchView";
        char PageOneSwitchView[] = "page OneSwitchView";
        char InputsView[] = "InputsView";
        char Export[] = "Export";
        char Import[] = "Import";
        char DelFile[] = "DelFile";
        char ModExt[] = ".MOD";
        char FailSAVE[] = "FailSAVE";
        char FailSafe[] = "FailSafe";
        char fs[16][5] = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
        char CH1NAME[] = "CH1NAME=";
        char CH2NAME[] = "CH2NAME=";
        char CH3NAME[] = "CH3NAME=";
        char CH4NAME[] = "CH4NAME=";
        char CH5NAME[] = "CH5NAME=";
        char CH6NAME[] = "CH6NAME=";
        char CH7NAME[] = "CH7NAME=";
        char CH8NAME[] = "CH8NAME=";
        char CH9NAME[] = "CH9NAME=";
        char CH10NAME[] = "CH10NAME=";
        char CH11NAME[] = "CH11NAME=";
        char CH12NAME[] = "CH12NAME=";
        char CH13NAME[] = "CH13NAME=";
        char CH14NAME[] = "CH14NAME=";
        char CH15NAME[] = "CH15NAME=";
        char CH16NAME[] = "CH16NAME=";
        char HelpView[] = "HelpView";
        char SendModel[] = "SendModel";
        char Exrite[] = "Exrite";
        char ExpR[] = "Exp";
        char Smooth[] = "Smooth";
        char Lines[] = "Lines";
        char GOTO[] = "GOTO:";
        char WhichPage[] = "page                                 "; // excessive spaces for page name
        char AddMinute[] = "IncMinute";
        char Dec_Minute[] = "DecMinute";
        char Dec_Hour[] = "DecHour";
        char Inc_Hour[] = "IncHour";
        char Inc_Year[] = "IncYear";
        char Dec_Year[] = "DecYear";
        char Inc_Date[] = "IncDate";
        char Dec_Date[] = "DecDate";
        char Inc_Month[] = "IncMonth";
        char Dec_Month[] = "DecMonth";
        char DataView_Clear[] = "Clear";
        char DataView_AltZero[] = "AltZero";
        char Mark[] = "Mark";
        char SetupCol[] = "SetupCol";
        char b0_bco[] = "b0.bco";
        char b0_pco[] = "b0.pco";
        char High_pco[] = "High.pco";
        char Fm_pco[] = "Fm.pco";
        char FrontView_BackGround[] = "FrontView.BackGround";
        char FrontView_ForeGround[] = "FrontView.ForeGround";
        char FrontView_Special[] = "FrontView.Special";
        char FrontView_Highlight[] = "FrontView.Highlight";
        char n0[] = "n0";
        char Expo[] = "Expo";
        char h0[] = "h0";
        char StCH[] = "StCH";
        char s0[] = "s0";
        char StEDIT[] = "StEDIT";
        char Prompt[60];
        char del[] = "Delete ";
        char overwr[] = "Overwrite ";
        char ques[] = "?";
        char hhead[] = "Create backup file for";
        char fprompt[] = "Filename?";
        char ProgressStart[] = "vis Progress,1";
        char ProgressEnd[] = "vis Progress,0";
        char Progress[] = "Progress";
        char FrontView_Hours[] = "Hours";
        char FrontView_Mins[] = "Mins";
        char FrontView_Secs[] = "Secs";
        char StartBackGround[] = "click Background,0";
        char NotConnected[] = "Model isn't connected!";
        char IsConnected[] = "Warning: model is connected!";
        char invisb1[] = "vis b1,0";
        char invisb0[] = "vis b0,0";
        char visb1[] = "vis b1,1";
        char visb0[] = "vis b0,1";
        char LOG[] = ".LOG";
        char PromtBeforeSendingModelFile[100];

        // ************************* test many input words from Nextion *****************

        if (InStrng(StCH, TextIn))
        { // select sub trim channel
            SubTrimToEdit = GetValue(s0) - 1;
            SendValue(n0, SubTrims[SubTrimToEdit] - 127);
            SendValue(h0, SubTrims[SubTrimToEdit]);
            ClearText();
            return;
        }

        if (InStrng(StEDIT, TextIn))
        {                                                 // edit sub trim value
            SubTrims[SubTrimToEdit] = GetValue(n0) + 127; // 127 is mid point in 8 bit value 0 - 254
            ClearText();
            return;
        }

        if (InStrng(SetupView, TextIn) > 0)
        { //  goto main TX setup screen
            ClearText();
            if (CurrentView == CALIBRATEVIEW)
                ReadOneModel(ModelNumber); // because it was cleared for calibration
            SaveAllParameters();
            CurrentView = TXSETUPVIEW;
            SendCommand(pTXSetupView);
            LastTimeRead = 0;
            CurrentView = TXSETUPVIEW;
            ClearText();
            UpdateModelsNameEveryWhere();
            return;
        }

        if (InStrng(Mark, TextIn) > 0)
        {
            GPSMarkHere = 255; // Mark this at RX
            GPS_RX_MaxDistance = 0;
            AddParameterstoQueue(GPS_MARK_LOCATION); // 3 is the ID of the MARK HERE parameter
            ClearText();
            return;
        }

        if (InStrng(DataEnd, TextIn) > 0)
        { //  Exit from Data screen
            SendCommand(pRXSetupView);
            CurrentView = RXSETUPVIEW;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(DataView_Clear, TextIn) > 0)
        { //  Clear Data screen
            ZeroDataScreen();
            ClearSuccessRate();
            ClearText();
            return;
        }

        if (InStrng(DataView_AltZero, TextIn) > 0)
        { //  Set zero altitude on data screen by recording current altitude for subraction later
            GroundModelAltitude = RXModelAltitudeBMP280;
            GPS_RX_GroundAltitude = GPS_RX_Altitude;
            GPS_RX_Maxaltitude = 0;
            RXMAXModelAltitude = 0;
            // LastRXModelMaxAltitude = 0;
            // LastRXModelAltitude = 0;
            ClearText();
            return;
        }

        if (InStrng(HelpView, TextIn) > 0)
        { // Display Help screen(s)
            SavedCurrentView = CurrentView;
            CurrentView = HELP_VIEW;
            SendHelp();
            ClearText();
            return;
        }
        if (InStrng(Dec_Minute, TextIn) > 0)
        {
            DecMinute();
            ClearText();
            return;
        }
        if (InStrng(AddMinute, TextIn) > 0)
        {
            IncMinute();
            ClearText();
            return;
        }

        if (InStrng(Dec_Hour, TextIn) > 0)
        {
            DecHour();
            ClearText();
            return;
        }
        if (InStrng(Inc_Hour, TextIn) > 0)
        {
            IncHour();
            ClearText();
            return;
        }

        if (InStrng(Dec_Year, TextIn) > 0)
        {
            DecYear();
            ClearText();
            return;
        }
        if (InStrng(Inc_Year, TextIn) > 0)
        {
            IncYear();
            ClearText();
            return;
        }

        if (InStrng(Dec_Date, TextIn) > 0)
        {
            DecDate();
            ClearText();
            return;
        }
        if (InStrng(Inc_Date, TextIn) > 0)
        {
            IncDate();
            ClearText();
            return;
        }

        if (InStrng(Dec_Month, TextIn) > 0)
        {
            DecMonth();
            ClearText();
            return;
        }
        if (InStrng(Inc_Month, TextIn) > 0)
        {
            IncMonth();
            ClearText();
            return;
        }

        if (InStrng(GOTO, TextIn) > 0)
        { // Return from Help screen returns here to relevent config screen
            i = 5;
            while (uint8_t(TextIn[i]) && i < 30)
            {
                WhichPage[i] = TextIn[i];
                ++i;
                WhichPage[i] = 0;
            }
            // Look1("Returned from help: ");
            // Look(LogFileName);
            // Look(WhichPage);
            // Look(SavedCurrentView);

            if (InStrng(LOG, LogFileName))
            {
                CurrentView = DATAVIEW;
                SavedCurrentView = DATAVIEW;
                strcpy(WhichPage, pDataView);
                //  Look(WhichPage);
            }
            // Get page name to which to return

            SendCommand(WhichPage); // this sends nextion back to last screen

            CurrentView = SavedCurrentView;

            if (CurrentView == GRAPHVIEW)
            {
                DisplayCurveAndServoPos();
                SendValue(CopyToAllBanks, 0);
            }
            if (CurrentView == SWITCHES_VIEW)
                UpdateSwitchesView();
            if (CurrentView == ONE_SWITCH_VIEW)
                UpdateOneSwitchView();
            if (CurrentView == MODELSVIEW)
                SendValue(ModelsView_ModelNumber, ModelNumber);
            if (CurrentView == REVERSEVIEW)
                StartReverseView();
            if (CurrentView == DATAVIEW)
                ForceDataRedisplay();
            if (CurrentView == PONGVIEW)
                StartPong();

            if (CurrentView == MIXESVIEW)
            {
                if (MixNumber == 0)
                    MixNumber = 1;
                LastMixNumber = 33;                        // just to be differernt
                SendValue(MixesView_MixNumber, MixNumber); // New load of mix window
                ShowMixValues();
            }

            if (CurrentView == CALIBRATEVIEW)
            {
                Force_ReDisplay();
                ShowServoPos();
            }

            if ((CurrentView == STICKSVIEW) || (CurrentView == FRONTVIEW))
            {
                Force_ReDisplay();
                ShowServoPos();
                if (CurrentView == FRONTVIEW)
                {
                    SendValue(FrontView_Secs, Secs);
                    SendValue(FrontView_Mins, Mins);
                    SendValue(FrontView_Hours, Hours);
                    ForceVoltDisplay = true;
                    LastConnectionQuality = 0;
                }
            }

            if (CurrentView == LOGVIEW)
            {
                GotoFrontView(); // otherwise it goes round for ever ... might fix later
            }

            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(Exrite, TextIn) > 0)
        { //  *******************
            if (GetValue(ExpR))
            {
                InterpolationTypes[Bank][ChanneltoSet - 1] = EXPONENTIALCURVES;
            }
            if (GetValue(Smooth))
            {
                InterpolationTypes[Bank][ChanneltoSet - 1] = SMOOTHEDCURVES;
            }
            if (GetValue(Lines))
            {
                InterpolationTypes[Bank][ChanneltoSet - 1] = STRAIGHTLINES;
            }
            Exponential[Bank][ChanneltoSet - 1] = GetValue(Expo) + 50; // Note: Getting this value from slider was not reliable (could not return 36!)
            ClearText();
            DisplayCurveAndServoPos();
            return;
        }

        if (InStrng(SendModel, TextIn) > 0)
        {
            i = strlen(SendModel);
            j = 0;
            while (uint8_t(TextIn[i]) > 0)
            {
                SingleModelFile[j] = TextIn[i];
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            }

            strcpy(PromtBeforeSendingModelFile, "Is '");
            strcat(PromtBeforeSendingModelFile, SingleModelFile);
            strcat(PromtBeforeSendingModelFile, "' up to date?\r\nPress 'OK' to send it now, \r\n(otherwise press 'Cancel').");

            if (!GetConfirmation(pModelsView, PromtBeforeSendingModelFile))
            {
                ClearText();
                return;
            }
            else // OK was pressed
            {
                SendModelFile();
                ClearText();
                return;
            }
        }
        if (InStrng(FailSAVE, TextIn) > 0)
        { //  the FAILSAFE setup is sent to receiver ************** FAILSAFE SETUP **************

            if (!BoundFlag || !ModelMatched)
            {
                MsgBox(pFailSafe, NotConnected);
                ClearText();
                SendCommand(ProgressEnd);
                return;
            }

            SendCommand(ProgressStart);
            for (int i = 0; i < 16; ++i)
            {
                FailSafeChannel[i] = GetValue(fs[i]);
                SendValue(Progress, i * 100 / 16);
            }
            SendValue(Progress, 100);
            AddParameterstoQueue(FAILSAFE_SETTINGS); // 1 is the ID for the FAILSAFE parameters
            ClearText();
            SendCommand(ProgressEnd);
            return;
        }

        if (InStrng(FailSafe, TextIn) > 0)
        {
            SendCommand(pFailSafe);
            CurrentView = FAILSAFE_VIEW;
            UpdateButtonLabels();
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(OneSwitchView, TextIn) > 0)
        {
            SwitchEditNumber = GetChannel(); // which switch?
            CurrentView = ONE_SWITCH_VIEW;
            SendCommand(PageOneSwitchView);
            UpdateOneSwitchView();
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(SwitchesView1, TextIn) > 0)
        { //  read switch values from screen (could be 1-4)
            ReadNewSwitchFunction();
        }

        if (InStrng(InputsView, TextIn) > 0)
        {
            if (ModelMatched) //  model is connected warning
            {
                if (!GetConfirmation(pRXSetupView, IsConnected))
                {
                    ClearText();
                    return;
                }
            }
            SendCommand(pInputsView);
            CurrentView = INPUTS_VIEW;
            UpdateButtonLabels();
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(CH1NAME, TextIn) > 0)
        {
            p = InStrng(CH1NAME, TextIn);
            i = p + 7;
            DoNewChannelName(1, i);
            ClearText();
            return;
        }
        if (InStrng(CH2NAME, TextIn) > 0)
        {
            p = InStrng(CH2NAME, TextIn);
            i = p + 7;
            DoNewChannelName(2, i);
            ClearText();
            return;
        }
        if (InStrng(CH3NAME, TextIn) > 0)
        {
            p = InStrng(CH3NAME, TextIn);
            i = p + 7;
            DoNewChannelName(3, i);
            ClearText();
            return;
        }
        if (InStrng(CH4NAME, TextIn) > 0)
        {
            p = InStrng(CH4NAME, TextIn);
            i = p + 7;
            DoNewChannelName(4, i);
            ClearText();
            return;
        }
        if (InStrng(CH5NAME, TextIn) > 0)
        {
            p = InStrng(CH5NAME, TextIn);
            i = p + 7;
            DoNewChannelName(5, i);
            ClearText();
            return;
        }
        if (InStrng(CH6NAME, TextIn) > 0)
        {
            p = InStrng(CH6NAME, TextIn);
            i = p + 7;
            DoNewChannelName(6, i);
            ClearText();
            return;
        }
        if (InStrng(CH7NAME, TextIn) > 0)
        {
            p = InStrng(CH7NAME, TextIn);
            i = p + 7;
            DoNewChannelName(7, i);
            ClearText();
            return;
        }
        if (InStrng(CH8NAME, TextIn) > 0)
        {
            p = InStrng(CH8NAME, TextIn);
            i = p + 7;
            DoNewChannelName(8, i);
            ClearText();
            return;
        }
        if (InStrng(CH9NAME, TextIn) > 0)
        {
            p = InStrng(CH9NAME, TextIn);
            i = p + 7;
            DoNewChannelName(9, i);
            ClearText();
            return;
        }
        if (InStrng(CH10NAME, TextIn) > 0)
        {
            p = InStrng(CH10NAME, TextIn);
            i = p + 8;
            DoNewChannelName(10, i);
            ClearText();
            return;
        }
        if (InStrng(CH11NAME, TextIn) > 0)
        {
            p = InStrng(CH11NAME, TextIn);
            i = p + 8;
            DoNewChannelName(11, i);
            ClearText();
            return;
        }
        if (InStrng(CH12NAME, TextIn) > 0)
        {
            p = InStrng(CH12NAME, TextIn);
            i = p + 8;
            DoNewChannelName(12, i);
            ClearText();
            return;
        }
        if (InStrng(CH13NAME, TextIn) > 0)
        {
            p = InStrng(CH13NAME, TextIn);
            i = p + 8;
            DoNewChannelName(13, i);
            ClearText();
            return;
        }
        if (InStrng(CH14NAME, TextIn) > 0)
        {
            p = InStrng(CH14NAME, TextIn);
            i = p + 8;
            DoNewChannelName(14, i);
            ClearText();
            return;
        }
        if (InStrng(CH15NAME, TextIn) > 0)
        {
            p = InStrng(CH15NAME, TextIn);
            i = p + 8;
            DoNewChannelName(15, i);
            ClearText();
            return;
        }
        if (InStrng(CH16NAME, TextIn) > 0)
        {
            p = InStrng(CH16NAME, TextIn);
            i = p + 8;
            DoNewChannelName(16, i);
            ClearText();
            return;
        }

        if (InStrng(DelFile, TextIn) > 0)
        { // Delete a file
            j = 0;
            p = InStrng(DelFile, TextIn);
            i = p + 6;
            while (uint8_t(TextIn[i]) > 0)
            {
                SingleModelFile[j] = TextIn[i];
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            }
            strcpy(Prompt, del);
            strcat(Prompt, SingleModelFile);
            strcat(Prompt, ques);
            if (GetConfirmation(pModelsView, Prompt))
            {
                SD.remove(SingleModelFile);
                strcpy(MOD, ".MOD");
                BuildDirectory();
                strcpy(Mfiles, "Mfiles");
                LoadFileSelector();
                --FileNumberInView;
                ShowFileNumber();
            }
            CloseModelsFile();
            ClearText();
            return;
        }

        if (InStrng(SwitchesView, TextIn))
        {
            SendCommand(pSwitchesView);
            UpdateSwitchesView(); // display saved values
            CurrentView = SWITCHES_VIEW;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(CalibrateView, TextIn))
        {
            SendCommand(pCalibrateView);
            Force_ReDisplay();
            CurrentView = CALIBRATEVIEW;
            ClearText();
            return;
        }

        if (InStrng(Export, TextIn))
        {
            GetDefaultFilename();
            if (GetBackupFilename(pModelsView, SingleModelFile, ModelName, hhead, fprompt))
            {
                FixFileName();
                if (CheckFileExists(SingleModelFile))
                {
                    strcpy(Prompt, overwr);
                    strcat(Prompt, SingleModelFile);
                    strcat(Prompt, ques);
                    if (GetConfirmation(pModelsView, Prompt))
                    {
                        WriteBackup();
                    }
                }
                else
                {
                    WriteBackup();
                }
            }
            strcpy(MOD, ".MOD");
            BuildDirectory();
            strcpy(Mfiles, "Mfiles");
            LoadFileSelector();
            ClearText();
            return;
        }

        p = InStrng(Import, TextIn);
        if (p > 0)
        {
            j = 0;
            i = p + 5;
            while (TextIn[i] > 0)
            {
                SingleModelFile[j] = toUpperCase(TextIn[i]);
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            }
            strcpy(Prompt, overwr);
            strcat(Prompt, ModelName);
            strcat(Prompt, ques);
            if (GetConfirmation(pModelsView, Prompt))
            {
                SendCommand(ProgressStart);
                DelayWithDog(10);
                SendValue(Progress, 5);
                DelayWithDog(10);
                if (InStrng(ModExt, SingleModelFile) == 0)
                    strcat(SingleModelFile, ModExt);
                SingleModelFlag = true;
                SendValue(Progress, 10);
                DelayWithDog(10);
                CloseModelsFile();
                ReadOneModel(1);
                SendValue(Progress, 50);
                DelayWithDog(10);
                SingleModelFlag = false;
                CloseModelsFile();
                SendValue(Progress, 75);
                DelayWithDog(10);
                SaveAllParameters();
                CloseModelsFile();
                UpdateModelsNameEveryWhere();
                SendValue(Progress, 100);
                DelayWithDog(10);
                SendCommand(ProgressEnd);
                if (FileError)
                    ShowFileErrorMsg();
                LoadModelSelector();
            }
            ClearText();
            return;
        }

        if (InStrng(ColoursView, TextIn) > 0)
        {
            CurrentView = COLOURS_VIEW;
            SendCommand(pColoursView);
            SendCommand(StartBackGround);
            ClearText();
            return;
        }

        if (InStrng(SetupCol, TextIn) > 0)
        { // This is  return fr Colours setup
            HighlightColour = GetOtherValue(High_pco);
            ForeGroundColour = GetOtherValue(b0_pco);
            BackGroundColour = GetOtherValue(b0_bco);
            SpecialColour = GetOtherValue(Fm_pco);
            SendValue(FrontView_BackGround, BackGroundColour);
            SendValue(FrontView_ForeGround, ForeGroundColour);
            SendValue(FrontView_Special, SpecialColour);
            SendValue(FrontView_Highlight, HighlightColour);
            SaveTransmitterParameters();
            CurrentView = TXSETUPVIEW;
            SendCommand(pTXSetupView);
            LastTimeRead = 0;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(TypeView, TextIn) > 0)
        {
            SendCommand(pTypeView);
            SendValue(r2s, 0); // Zero all RX batt cell count
            SendValue(r3s, 0);
            SendValue(r4s, 0);
            SendValue(r5s, 0);
            SendValue(r6s, 0);
            SendValue(r12s, 0);
            SendValue(r0, 0);
            SendValue(r1, 0);
            if (RXCellCount == 2)
                SendValue(r2s, 1); // Then update RX batt cell count
            if (RXCellCount == 3)
                SendValue(r3s, 1);
            if (RXCellCount == 4)
                SendValue(r4s, 1);
            if (RXCellCount == 5)
                SendValue(r5s, 1);
            if (RXCellCount == 6)
                SendValue(r6s, 1);
            if (RXCellCount == 12)
                SendValue(r12s, 1);
            if (TXLiPo)
                SendValue(r1, 1);
            else
                SendValue(r0, 1);

            ClearText();
            UpdateModelsNameEveryWhere();
            return;
        }

        if (InStrng(RXBAT, TextIn) > 0)
        { // UPdate RX batt cell count
            if (GetValue(r2s) == 1)
                RXCellCount = 2;
            if (GetValue(r3s) == 1)
                RXCellCount = 3;
            if (GetValue(r4s) == 1)
                RXCellCount = 4;
            if (GetValue(r5s) == 1)
                RXCellCount = 5;
            if (GetValue(r6s) == 1)
                RXCellCount = 6;
            if (GetValue(r12s) == 1)
                RXCellCount = 12;
            if (GetValue(r0) == 1)
                TXLiPo = false; // TX LIFE
            if (GetValue(r1) == 1)
                TXLiPo = true; // TX LIPO
            SaveAllParameters();
            ClearText();
            return;
        }

        if (InStrng(TrimView, TextIn) > 0)
        { // TrimView just appeared, so update it.
            StartTrimView();
            return;
        }

        if (InStrng(TRIMS50, TextIn) > 0)
        {
            for (i = 0; i < 15; ++i)
            {
                Trims[Bank][i] = 80; // Mid value is 80
            }
            if (CopyTrimsToAll)
            {
                for (i = 0; i < 15; ++i)
                {
                    for (int fm = 1; fm < 5; ++fm)
                    {
                        Trims[fm][i] = 80;
                    }
                }
            }
            ClearText();
            return;
        }

        if (InStrng(Setup, TextIn) > 0)
        { // Which channel to setup ... Goes to GraphView
            ChanneltoSet = GetChannel();
            CurrentView = GRAPHVIEW;
            SendCommand(pGraphView); // Set to GraphView
            DisplayCurve();
            updateInterpolationTypes();
            UpdateModelsNameEveryWhere();
            SendValue(CopyToAllBanks, 0);
            ClearText();
            return;
        }

        if (InStrng(Front_View, TextIn))
        {
            CurrentView = FRONTVIEW;
            ClearText();
            PreviousBank = 250; // sure to be different
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(Sticks_View, TextIn))
        {
            SendCommand(pSticksView);
            Force_ReDisplay();
            CurrentView = STICKSVIEW;
            SendCommand(pSticksView); // Set to SticksView
            UpdateModelsNameEveryWhere();
            UpdateButtonLabels();
            ClearText();
            return;
        }

        if (InStrng(ReScan, TextIn))
        {
            DrawFhssBox();
            DoScanInit();
            ClearText();
            return;
        }

        if (InStrng(MIXES_VIEW, TextIn))
        { // First call to load screen
            SendCommand(pMixesView);
            CurrentView = MIXESVIEW;
            UpdateModelsNameEveryWhere();
            if (MixNumber == 0)
                MixNumber = 1;
            LastMixNumber = MixNumber;
            SendValue(MixesView_MixNumber, MixNumber); // New load of mix window
            ShowMixValues();
            DelayWithDog(100); // allow time for screen to load  before reading data
            ReceiveLotsofData();
            FixCHNames();
            ClearText();
            return;
        }
        if (InStrng(Mixes_View, TextIn))
        { // Mix number OR a parameter has changed
            CurrentView = MIXESVIEW;
            uint8_t ThisMixNumber = MixNumber; // save it
            MixNumber = GetValue(MixesView_MixNumber);
            if (LastMixNumber != MixNumber) // Did number change?
            {
                SendCommand(invisb1);
                SendCommand(invisb0);
                LastMixNumber = MixNumber; // save new mix number
                MixNumber = ThisMixNumber; // Force back to old number to grab last lot before doing new one
                ReadMixValues();           // Read them from screen
                SaveOneModel(ModelNumber); // Save them to SD card
                MixNumber = LastMixNumber; // back to new one
                ShowMixValues();           // show new lot
                SendCommand(visb1);
                SendCommand(visb0);
            }
            else
            {
                ReadMixValues(); //
            }
            FixCHNames();
            ClearText();
            return;
        }

        if (InStrng(Graph_View, TextIn))
        {
            CurrentView = GRAPHVIEW;
            ClearText();
            return;
        }

        if (InStrng(Data_View, TextIn))
        {
            CurrentView = DATAVIEW;
            LastShowTime = 0;
            ForceDataRedisplay();
            SendCommand(pDataView);
            ClearText();
            return;
        }

        if (InStrng(FM1, TextIn))
        {
            Bank = 1;
            PreviousBank = 1;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(FM2, TextIn))
        {
            Bank = 2;
            PreviousBank = 2;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(FM3, TextIn))
        {
            Bank = 3;
            PreviousBank = 3;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(FM4, TextIn))
        {
            Bank = 4;
            PreviousBank = 4;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(Reset, TextIn)) // Now zeros EXPO only
        {
            Exponential[Bank][ChanneltoSet - 1] = DEFAULT_EXPO;
            InterpolationTypes[Bank][ChanneltoSet - 1] = EXPONENTIALCURVES; // expo = default
            DisplayCurveAndServoPos();
            return;
        }

        p = (InStrng(ClickX, TextIn)); // Clicked to move point?
        if (p > 0)
        {
            XtouchPlace = GetNextNumber(p + 7, TextIn);
            // This drops through to get Y as well before moving the point
        }
        p = (InStrng(ClickY, TextIn)); // Clicked to move point?
        if (p > 0)
        {
            YtouchPlace = GetNextNumber(p + 7, TextIn);
            MovePoint();
            DisplayCurveAndServoPos();
            return;
        }
        if (CurrentMode == NORMAL)
        {
            if (strcmp(TextIn, "Calibrate1") == 0)
            {
                BlueLedOn();
                SetDefaultValues();
                ResetSwitchNumbers();
                SaveTransmitterParameters();
                ReduceLimits(); // Get setup for sticks calibration
                CurrentMode = CALIBRATELIMITS;
                CurrentView = CALIBRATEVIEW;
                SendText1(SvT11, CMsg1);
                SendText(SvB0, CMsg2);
                ClearText();
                BlueLedOn();
                return;
            }
        }

        if (CurrentMode == CALIBRATELIMITS)
        {
            if (strcmp(TextIn, "Calibrate1") == 0)
            {
                CurrentMode = CENTRESTICKS;
                CurrentView = CALIBRATEVIEW;
                SendText1(SvT11, Cmsg3);
                SendText(SvB0, Cmsg4);
                ClearText();
                return;
            }
        }
        if (CurrentMode == CENTRESTICKS)
        {
            if (strcmp(TextIn, "Calibrate1") == 0)
            {
                CurrentMode = NORMAL;
                RedLedOn();
                SaveTransmitterParameters(); // Save calibrations
                LoadAllParameters();         // Restore all current model settings
                SendText(SvB0, Cmsg5);
                SendText(SvT11, Cmsg6);
                LastTimeRead = 0;
                ClearText();
                return;
            }
        }
    }
    ClearText(); // Let's have cleared text for next one!
} // end ButtonWasPressed() (... at last!!!)

/************************************************************************************************************/

uint16_t MakeTwobytes(bool *f)
{                    // Pass arraypointer. Returns the two bytes
    uint16_t tb = 0; // all false is default
    for (int i = 0; i < 16; ++i)
    {
        if (f[15 - i] == true)
        {
            tb |= 1 << (i);
        } // sets a bit if true
    }
    return tb;
}

/************************************************************************************************************/

void CheckMotorOff()
{ // For Safety

    if (!UseMotorKill)
        return;

    ReadTheSwitchesAndTrims();
    MotorEnabled = false;
    SafetyON = false;

    static const uint8_t Pins[4] = {7, 5, 0, 2};

    // Handle AutoSwitch
    if (Autoswitch >= 1 && Autoswitch <= 4)
    {
        uint8_t pin = Pins[Autoswitch - 1];
        bool reversed = SwitchReversed[Autoswitch - 1];
        if (Switch[pin] == reversed)
            MotorEnabled = true;
    }

    // Handle SafetySwitch
    if (SafetySwitch >= 1 && SafetySwitch <= 4)
    {
        uint8_t pin = Pins[SafetySwitch - 1];
        bool reversed = SwitchReversed[SafetySwitch - 1];
        if (Switch[pin] == reversed)
            SafetyON = true;
    }

    if (SafetyON)
        MotorEnabled = false;

    MotorWasEnabled = MotorEnabled;
}

/************************************************************************************************************/
void ResetMotorTimer()
{
    char FrontView_Hours[] = "Hours";
    char FrontView_Mins[] = "Mins";
    char FrontView_Secs[] = "Secs";

    TimesUp = false;
    PausedSecs = 0;
    CountDownIndex = 0;
    if (CurrentView == FRONTVIEW)
    {
        if (TimerDownwards)
        {
            Mins = TimerStartTime / 60;
        }
        else
        {
            Mins = 0;
        }
        SendValue(FrontView_Secs, 0);
        SendValue(FrontView_Mins, Mins);
        SendValue(FrontView_Hours, 0);
    }
}

// ************************************************************************************************************/
void MotorEnabledHasChanged()
{
    if (MotorEnabled)
    {
        if (LedWasRed)
        {
            MotorEnabled = false;
            SendNoData = false;
            if ((millis() - WarningTimer) > 4000)
            {
                PlaySound(PLSTURNOFF);
                SendNoData = true; // user turned on motor
                WarningTimer = millis();
            }
            return;
        }
        ShowMotor(1);
        if (AnnounceBanks)
            PlaySound(MOTORON); // Tell the pilot motor is on!
        if (UseLog)
            LogMotor(1);
        MotorStartTime = millis(); // Motor ON timerpause off
    }
    else
    {
        if (AnnounceBanks)
            PlaySound(MOTOROFF);
        SendNoData = false; // can send data!
        if (UseLog)
            LogMotor(0);
        SendCommand(WarnOff);
        ShowMotor(0); // Tell the pilot motor is off
        if (SendNoData)
        {
            SendCommand(WarnOff);
            SendNoData = false; // user turned off motor
        }
        PausedSecs = MotorOnSeconds; //    Motor OFF timerpause started
    }
    LastSeconds = 0;
    ShowMotorTimer();
}
//************************************************************************************************************/
void SafetySwitchChanged()
{
    if (SafetyON)
    {
        ShowSafetyIsOn();
        SendNoData = false; // can send data!
    }
    else
    {
        ShowSafetyIsOff();
    }
    SafetyWasOn = SafetyON;
}

//************************************************************************************************************/
// This function is called when the bank has changed
void BankHasChanged()
{

    if ((CurrentView == FRONTVIEW) || (CurrentView == TRIM_VIEW))
    {
        for (int pp = 0; pp < 4; ++pp)
            LastTrim[Bank][pp] = 0; // force a trimview update
        UpdateTrimView();
    }
    if (UseLog)
        LogNewBank();
    if (MotorEnabled == MotorWasEnabled)
    { // When turning off motor, don't sound bank too.
        if (AnnounceBanks)
            SoundBank();
    }


    
    if (CurrentView == FRONTVIEW)
    {
        ShowBank();
    }
    else
    {
        UpdateModelsNameEveryWhere();
    }
    if (CurrentView == GRAPHVIEW)
        DisplayCurveAndServoPos();
    if (CurrentView == SLOWSERVOVIEW)
    {
        ReadSpeedsScreen(PreviousBank - 1);
        UpdateSpeedScreen();
    }
    if (CurrentView == DUALRATESVIEW)
    {
        DisplayNewDualRateBank();
    }
    if (CurrentView == PIDVIEW)
    {
        ShowPIDBank();
    }
    if (CurrentView == RATESVIEW1)
    {
        ShowRatesBank();
    }
    if (CurrentView == RATESADVANCEDVIEW)
    {
        ShowRatesAdvancedBank();
    }
}

/************************************************************************************************************/
void GetBank() // ... and the other three switches
{
    if ((CurrentMode != NORMAL) && (CurrentMode != LISTENMODE))
        return; // not needed if calibrating

    if ((!BuddyPupilOnWireless) || (BuddyHasAllSwitches))
    { // if BuddyPupilOnWireless is not using switches
        ReadSafetySwitch();
        ReadBuddySwitch();
        ReadBankSwitch();
        ReadAutoAndMotorSwitch();
    }

    ReadDualRateSwitch(); // only actually read hardware switches if not using wireless buddy box or if buddy has all switches

    if (SafetyWasOn != SafetyON)
        SafetySwitchChanged();
    if (SafetyON)
        MotorEnabled = false;
    if ((MotorEnabled != MotorWasEnabled) && (UseMotorKill))
        MotorEnabledHasChanged();
    ReadChannelSwitches9to12();
    if (Bank != PreviousBank) /// BANK HAS CHANGED ******************************************************************
        BankHasChanged();
    MotorWasEnabled = MotorEnabled; // Remember motor state
    PreviousBank = Bank;            // Remember BANK
}

/************************************************************************************************************/

void GotoFrontView()
{
    char fms[4][4] = {{"fm1"}, {"fm2"}, {"fm3"}, {"fm4"}};
    char FrontView_Connected[] = "Connected";
    PupilIsAlive = 0;
    MasterIsAlive = 0;
    LastAutoModelSelect = true;
    LastCopyTrimsToAll = true;
    OldRate = 235;             // forced different
    ForceVoltDisplay = true;   // force redisplay of voltage
    LastConnectionQuality = 0; // force redisplay of connection quality
    if (CurrentView != FRONTVIEW)
    {
        if (CurrentView == SCANVIEW)
            DoScanEnd(); // Put transceiver back to normal mode
        if (CurrentView == PONGVIEW)
            ReadOneModel(ModelNumber); // Return to current model
        if (CurrentMode != LISTENMODE)
            CurrentMode = NORMAL;     // Return to normal mode unless in BUDDY listen mode
        SendCommand(pFrontView);      // Set to FrontView
        CurrentView = FRONTVIEW;      // Set to FrontView
        UpdateModelsNameEveryWhere(); // Update model name
        SafetyWasOn ^= 1;             // this forces a re-display of safety state
        BeQuiet = true;               // this means no announcement of safety this time
        ShowBank();
        LastTimeRead = 0;
        Reconnected = false; // this is to make '** Connected! **' redisplay (in ShowComms())
        LastSeconds = 0;     // This forces redisplay of timer...
        Force_ReDisplay();
        ShowMotorTimer();
        ClearText();
        LastShowTime = 0; // this is to make redisplay sooner (in ShowComms())
        SendText(FrontView_Connected, na);
    }
    for (int i = 0; i < 4; ++i)
    {
        SendText(fms[i], BankNames[BanksInUse[i]]);
    }
    ForceDataRedisplay();
    ShowAMS();
    UpdateTrimView();
    if (BuddyPupilOnWireless)
        StartBuddyListen();
    CurrentView = FRONTVIEW;
    ModelMatchFailed = false;
    First_RPM_Data = true;
    RestoreBrightness();
}

/************************************************************************************************************/
bool CheckModelName()
{ // In ModelsView, this function checks correct name is displayed.
  // it returns true if it has changed
    char MMems[] = "MMems";
    char Mfiles[] = "Mfiles";
    char mn[] = "modelname";

    ModelNumber = GetValue(MMems) + 1;
    FileNumberInView = GetValue(Mfiles);
    if (FileNumberInView != LastFileInView)
    {
        ShowFileNumber();
        LastFileInView = FileNumberInView;
    }
    if (LastModelLoaded != ModelNumber)
    {
        if ((ModelNumber >= MAXMODELNUMBER) || (ModelNumber < 1))
        {
            ModelNumber = 1;
            SendValue(MMems, ModelNumber - 1);
        }
        ReadOneModel(ModelNumber);
        SendText(mn, ModelName);
        // if (UseLog)
        //     LogThisModel();
        LastModelLoaded = ModelNumber;
        UpdateModelsNameEveryWhere();
        return true;
    }
    ClearText();
    return false;
}

/************************************************************************************************************/
void Close_TX_Down()
{
    char NotInUse[] = "Not in use";
    char ClosingDown[] = "Closing down ...";
    char Saving[] = "Saving ";
    char msg1[80];
    char t0[] = "t0";
    char msgvis[] = "vis t0,1";
    analogWrite(GREENLED, 0);
    analogWrite(BLUELED, 0);
    analogWrite(REDLED, 0);
    RestoreBrightness();
    SendCommand(pBlankView); // Set to BlankView
    SendCommand(msgvis);     // Make it visible
    if (PlayFanfare)
    {
        PlaySound(WINDOWS2);
    } // Play the fanfare
    if (strcmp(ModelName, NotInUse) != 0)
    { // If model is in use
        strcpy(msg1, Saving);
        strcat(msg1, ModelName); // Build up the message
        SendText(t0, msg1);      // Show 'Saving <model> ' on screen
        SaveAllParameters();     // Save the model if it's not 'Not in use'
        DelayWithDog(500);       // Wait for 0.5 seconds to allow the message to be displayed
    }
    SendText(t0, ClosingDown);    // Show 'Closing down ...' on screen
    for (int i = 0; i < 100; ++i) // fade in screen brightness
    {
        SetBrightness(100 - i);
        DelayWithDog(25);
    }
    // if (UseLog)
    //  {
    //      strcpy(LogFileName, ""); // avoid logging to the wrong file
    //      LogPowerOff();
    //  } // log the event
    DelayWithDog(POWERONOFFDELAY);     // 2 seconds delay in case button held down too long
    digitalWrite(POWER_OFF_PIN, HIGH); // Power off the transmitter
    delay(100);                        // Wait for a short time to ensure power off
}

// ************************************************************************************************************/
void CheckWhetherToEnableBinding()
{
    // This function checks whether binding is being enabled for this transmitter.
    // This is done by holding down the power button for two seconds at the start up.

    char Mfound[] = "Binding enabled";
    char wb[] = "wb"; // wb is the name of the label on front view
    char YesVisible[] = "vis wb,1";

    if (!digitalRead(BUTTON_SENSE_PIN) && ((millis()) < 5000) && (millis() >= 2000)) //  To initiate Binding, hold ON button for 2-5 secs
    {
        if (!BindingEnabled)
        {
            SendCommand(pFrontView);      // Go to front view
            CurrentView = FRONTVIEW;      // Set to FrontView
            UpdateModelsNameEveryWhere(); // Update model name
            PlaySound(BEEPCOMPLETE);      // Play sound to indicate binding enabled
            delay(250);                   // Wait for chirp to finish
            PlaySound(BINDINGENABLED);    // Play sound to indicate binding enabled
            SendText(wb, Mfound);         // Show binding enabled
            SendCommand(YesVisible);      // Show binding enabled
            BindingEnabled = true;        // Set binding enabled flag
        }
        return;
    }
}

/************************************************************************************************************/
void CheckPowerOffButton()
{
    static bool CheckingPowerButton = false;
    static bool PowerWarningVisible = false;
    static uint32_t PowerOffTimer = 0;
    static uint8_t TurnOffSecondToGo = PowerOffWarningSeconds;
    char PowerMsg[30];
    char PowerPre[] = "TURN OFF?! ";
    char nb[8];
    char HideStillConnected[] = "vis StillConnected,0";
    char ShowStillConnected[] = "vis StillConnected,1";
    char StillConnectedBox[] = "StillConnected";

    if ((digitalRead(BUTTON_SENSE_PIN)) || CheckingPowerButton) // already checking power button or button not even pressed!
        return;
    CheckingPowerButton = true; // set flag to prevent re-entry
    if (!BuddyMasterOnWireless && !BuddyPupilOnWireless)
        CheckWhetherToEnableBinding(); // Check if binding is enabled Which is done by holding the start button for more than two seconds

    if ((!digitalRead(BUTTON_SENSE_PIN)) && (millis() > POWERONOFFDELAY2)) // no power off for first 10 seconds in case button held down too long
    {                                                                      // power button is pressed!
        if (BoundFlag && ModelMatched && CurrentView != FRONTVIEW)
            GotoFrontView();

        if (!LedWasGreen)
            Close_TX_Down(); // if not connected power off immediately

        if (LedWasGreen)
        {
            if (!PowerOffTimer)
            {
                PowerOffTimer = millis(); // Start a timer for power off button down
                TurnOffSecondToGo = PowerOffWarningSeconds;
            }
        }
    }
    else
    {
        PowerOffTimer = 0;
        TurnOffSecondToGo = PowerOffWarningSeconds;
    }

    if (PowerOffTimer)
    { // count down started
        if (!PowerWarningVisible)
        {
            SendCommand(ShowStillConnected);
            SaveOneModel(ModelNumber); // in case any trim change etc wasn't saved (not really needed)
            PowerWarningVisible = true;
        }
    }
    else
    {
        if (PowerWarningVisible)
        {
            SendCommand(HideStillConnected);
            PowerWarningVisible = false;
            TurnOffSecondToGo = PowerOffWarningSeconds;
        }
        CheckingPowerButton = false;
        return;
    }

    if (PowerWarningVisible)
    {
        if ((millis() - PreviousPowerOffTimer) >= 1000)
        {
            strcpy(PowerMsg, PowerPre);
            Str(nb, TurnOffSecondToGo, 0);
            strcat(PowerMsg, nb);
            SendText(StillConnectedBox, PowerMsg);
            if (TurnOffSecondToGo <= 0)
            { // Time's up!
                Close_TX_Down();
            }
            --TurnOffSecondToGo;
            if (TrimClicks)
                PlaySound(CLICKZERO);
            PreviousPowerOffTimer = millis();
        }
    }
    CheckingPowerButton = false;
}
// /************************************************************************************************************/
// This function checks if the model name has changed in ModelsView
void CheckModelsScreen(uint32_t RightNow)
{
    static uint32_t LastModelScreenCheck = 0;
    if (RightNow - LastModelScreenCheck < 500)
        return; // Only check once every 500 ms
    LastModelScreenCheck = RightNow;
    if (CheckModelName())
        LastModelScreenCheck = RightNow;
}
/************************************************************************************************************/
void FASTRUN ManageTransmitter()
{

    static uint32_t TransmitterLastManaged = 0;

    uint32_t RightNow = millis();
    uint32_t TXPacketElapsed = RightNow - LastPacketSentTime;

    KickTheDog(); // Watchdog ... ALWAYS!
    if ((FHSS_data::PaceMaker - TXPacketElapsed < TIMEFORTXMANAGMENT) && ModelMatched)
    {
        return; // If it's almost time to send data, then do not start some other task which might easily take longer.
    }
    if (ThisGap)
    {
        ProcessRecentCommsGap();
        return;
    }
    CheckPowerOffButton();
    CheckForNextionButtonPress(); // Pretty obvious really ...

    DoTheVariometer(); // Do the variometer
    if (RightNow - LastTimeRead >= 1000)
    { // Only once a second for these..
        if (VersionMismatch)
        {
            ShowMismatchMsg(); // Show version mismatch message if needed
        }
        if (((millis() - LedGreenMoment) <= 6000) && ((millis() - LedGreenMoment) >= 4000))
        {
            ZeroDataScreen(); // this will clear the long gaps that might occur while binding.
        }
        GetFrameRate();       // Get the frame rate
        CheckScreenTime();    // Check if screen needs to be turned off
        CheckBatteryStates(); // Check battery states
        if (CurrentView != BLANKVIEW)
        {
            ReadTime();
            UpdateTrimView();
            ShowComms();
        } // Show time and trim positions
        ShowMotorTimer();        // Show motor timer and send any queued parameters
        LastTimeRead = millis(); // Reset this timer
        return;                  // That's enough housekeeping for this time around
    }
    if (ParametersToBeSentPointer)        // Any parameters to be sent?
        ActuallySendParameters(RightNow); // yes, send them
    if (CurrentView == MODELSVIEW)
        CheckModelsScreen(RightNow);

    if (RightNow - TransmitterLastManaged >= 50)
    { // 50 = 20 times a second
        ReadTheSwitchesAndTrims();
        CheckHardwareTrims();
        GetBank(); // Check switch positions 20 times a secon
        TransmitterLastManaged = millis();
    }
}

/************************************************************************************************************/
void FixMotorChannel()
{
    if (!MotorEnabled && !BuddyState)
    {
        SendBuffer[MotorChannel] = IntoHigherRes(MotorChannelZero); // If safety is on, throttle will be zero whatever was shown.
    }
}

/************************************************************************************************************/
// LOOP
/************************************************************************************************************/

FASTRUN void loop()
{
    ManageTransmitter();   // Do the needed chores ... (if there's time)
    GetNewChannelValues(); // Load SendBuffer with new servo positions very frequently

    if (CurrentMode < 3)
    {
        if (UseMacros)
            ExecuteMacro(); // Modify it if macro is running
        GetBuddyData();     // Only if master
        FixMotorChannel();  // Maybe force it low BEFORE Binding data is added
        ShowServoPos();     // Show servo positions to user
        if (BindingEnabled && !BoundFlag)
            SendBindingPipe(); // Only if binding and not bound yet - override low throttle setting
    }

    switch (CurrentMode)
    {
    case NORMAL:    // 0
        SendData(); // local TX
        break;
    case CALIBRATELIMITS: // 1
        CalibrateSticks();
        break;
    case CENTRESTICKS: // 2
        ChannelCentres();
        break;
    case SCANWAVEBAND: // 3
        ScanAllChannels(false);
        break;
    case SENDNOTHING: // 4
        break;
    case PONGMODE: // 5
        PlayPong();
        break;
    case LISTENMODE: // 6  ... listen only ... for wireless buddy
        DoWirelessBuddyListen();
        break;
    default:
        break;
    }
} // end loop()

/************************************************************************************************************/
