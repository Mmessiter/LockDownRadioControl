/** @file TransmitterCode/src/main.cpp
 * // Malcolm Messiter 2020 - 2023
 *
 * @page TransmitterCode...
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
 * - Buddyboxing with selectable channels.
 * - Voice messages and other audio prompts.
 * - User defined Channel names
 * - 2.4 Ghz FHSS ISM band licence free in UK and most other countries.
 * - > 2 Km range
 * - Telemetry including GPS, volts, temperature & barometric pressure, using custom I2C sensor hub
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
 * - PPM or SBUS now possible from reciever - chosen in transmitter.
 * - Support for external third party transmitter modules added (JR type).
 *
 *
 * @section txPinout Teensy 4.1 Pins
 * | Teensy 4.1 Pins | Connections |
 * |-----------------|-------------|
 * | GND        | GND |
 * | Vin        | +5V power in  |
 * | 0  (RX1)   | NEXTION  (TX) |
 * | 1  (TX2)   | NEXTION  (RX) |
 * | 2  LED     | RED |
 * | 3  LED     | GREEN |
 * | 4  LED     | BLUE |
 * | 5  (POLOLU)| 2808 ALL POWER OFF SIGNAL (When high)  |
 **| 6  (PPM)   | PPM IN or OUT NEW PCB TX MODULE (was Sensor for power button)
 * | 7  (CE)    | nRF24l01 (CE)  | on new PCB
 * | 8  (CSN)   | nRF24l01 (CSN) | on new PCB
 **| 9  .....   | was nRF24l01 (CSN) now SPARE on new PCB <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< SPARE
 **| 10 (PPM)   | was nRF24l01 (CSN) now BUDDY PPM on new PCB)
 **| 11 (MOSI)  | nRF24l01 (MOSI) |
 **| 12 (MISO)  | nRF24l01 (MISO) |
 **| 13 (SCK)   | nRF24l01 (SCK) |
 **| 14 (A0)    | Joystick POT CH1 |
 **| 15 (A1)    | Joystick POT CH2 |
 * | 16 (A2)    | Joystick POT CH3 |
 * | 17 (A3)    | Joystick POT CH4 |
 **| 18 (I2C)   | I2C bus  SDA |
 **| 19 (I2C)   | I2C bus  SCL |
 * | 20 (A6)    | POT KNOB CH5 |
 * | 21 (A7)    | POT KNOB CH6 |
 * | 22 (A8)    | POT KNOB CH7 |
 * | 23 (A9)    | POT KNOB CH8 |
 * | 25         | Switch 1 |
 * | 26         | Switch 1 |
 * | 27         | Switch 2 |
 * | 28         | Switch 2 |
 * | 29         | Switch 3 |
 * | 30         | Switch 3 |
 * | 31         | Switch 4 |
 * | 32         | Switch 4 |
 * | 33         | Sensor for power button press while on | (formally a SPARE)
 * | 34         |TRIM (CH1a)|
 * | 35         |TRIM (CH1b)|
 * | 36         |TRIM (CH2a)|
 * | 37         |TRIM (CH2b)|
 * | 38         |TRIM (CH3a)|
 * | 39         |TRIM (CH3b)|
 * | 40         |TRIM (CH4a)|
 * | 41         |TRIM (CH4b)|
 * | 42         Built-in SD card
 * | 43         Built-in SD card
 * | 44         Built-in SD card
 * | 45         Built-in SD card
 * | 46         Built-in SD card
 * | 47         Built-in SD card
 * |  (Solder pads on the back:)
 * | 48<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (!! SPARE !!)
 * | 49<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (!! SPARE !!)
 * | 50<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (!! SPARE !!)
 * | 51<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (!! SPARE !!)
 * | 52<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (!! SPARE !!)
 * | 53<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (!! SPARE !!)
 * | 54<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (!! SPARE !!)
 * @see TransmitterCode/src/main.cpp
 */
// ************************************************** TRANSMITTER CODE **************************************************

#include <Arduino.h>
#include <Watchdog_t4.h>
#include <TeensyID.h>
#include <PulsePosition.h>
#include <RF24.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <DS1307RTC.h>
#include <InterpolationLib.h>

#include "Hardware/Definitions.h" // must be here! (not higher or lower);

#include "Hardware/StructsEtc.h"
#include "Hardware/Utilities.h"
#include "Hardware/transceiver.h"
#include "Hardware/Pong.h"
#include "Hardware/macros.h"
#include "Hardware/Trims.h"
#include "Hardware/ModelMatch.h"
#include "Hardware/Nextion.h"

//*************************************************************************************************************************

// This function reads data from BUDDY (Slave) BUT uses it ONLY WHILE buddy switch is on

void GetSlaveChannelValuesPPM() // MASTER code
{
    if (BuddyON) {

        if (PPMdata.PPMInputBuddy.available() == CHANNELSUSED) {
            for (int j = 0; j < CHANNELSUSED; ++j) {                // While slave has control, his stick data replaces all ours
                uint16_t PpmIn = PPMdata.PPMInputBuddy.read(j + 1); // read EVERY channel
                if (BuddyControlled & 1 << (j)) {                   // Test if this channel is buddy controlled. If not leave it unchanged
                    SendBuffer[j] = PpmIn;
                    PPMBuffer[j]  = PpmIn;
                }
            }
            if (!SlaveHasControl) { // Buddy is now On
                PlaySound(BUDDYMSG);
                LastShowTime    = 0;
                SlaveHasControl = true;
            }
        }
        else {
            for (int j = 0; j < CHANNELSUSED; ++j) { // reuse old data if no new is available.
                if (BuddyControlled & 1 << (j)) SendBuffer[j] = PPMBuffer[j];
            }
        }
    }
    else { // Buddy is now Off
        if (SlaveHasControl) {
            PlaySound(MASTERMSG);
            LastShowTime    = 0;
            SlaveHasControl = false;
        }
    }
}

/************************************************************************************************************/
//  Send via PPM for Buddy box pupil

FASTRUN void SendViaPPM()
{
    static uint32_t PPMTimer = 0;

    if (millis() - PPMTimer >= PPMBUDDYFRAMERATE) {
        PPMTimer = millis();
        for (int j = 0; j < CHANNELSUSED; ++j) PPMdata.PPMOutputBuddy.write(j + 1, SendBuffer[j]);
    }
}
/*********************************************************************************************************************************/

void RedLedOn()
{
    char InVisible[]           = "vis Quality,0";
    char FrontView_Connected[] = "Connected";
    char WarnOff[]             = "vis Warning,0";

    if (LedWasGreen) {
        RXVoltsDetected      = false;
        LedWasGreen          = false;
        RXVoltsDetected      = false;
        ModelIdentified      = false;
        ModelMatched         = false;
        BoundFlag            = false;
        PacketsPerSecond     = 0;
        LastShowTime         = 0;
        ModelsMacUnion.Val64 = 0;
        RangeTestGoodPackets = 0;
        RecentPacketsLost    = 0;
        SetUKFrequencies();
        if (CurrentView == FRONTVIEW) {
            SendText(FrontView_Connected, na);
            SendCommand(WarnOff);
            SendCommand(InVisible);
        }
        if (UseLog) LogDisConnection();
        if (AnnounceConnected) PlaySound(DISCONNECTEDMSG);
    }

    LedWasRed = true;
    analogWrite(GREENLED, 0);
    analogWrite(BLUELED, 0);
    analogWrite(REDLED, GetLEDBrightness()); // Brightness is a function of maybe blinking
}

/*********************************************************************************************************************************/

void GreenLedOn()
{
    if (!ModelMatched) return; // no green led for wrong model
    if (!LedWasGreen) {
        ClearSuccessRate();
        LastShowTime = 0;
    }

    TotalLostPackets  = 0;
    TotalGoodPackets  = 0;
    RecentPacketsLost = 0;
    GapLongest        = 0;

    if (!LedWasGreen || LedIsBlinking)
    { // no need to repeat unless it is blinking
        if (!LedIsBlinking) {
            ShowComms();
            if (AnnounceConnected) {
                PlaySound(CONNECTEDMSG);
                ModelMatched = true;
                BoundFlag    = true;
                DelayWithDog(750);
            }
        }
        if (UseLog) {
            LogConnection();
        }
        if (FirstConnection) { // Zero data on first connection after reboot
            ZeroDataScreen();
            FirstConnection = false;
        }
        LedWasRed   = false;
        LedWasGreen = true;
        analogWrite(BLUELED, 0);
        analogWrite(REDLED, 0);
        analogWrite(GREENLED, GetLEDBrightness()); // Brightness is a function of maybe blinking
        if (GetLEDBrightness()) LedWasGreen = true;
        Reconnected = false;
    }
}

/*********************************************************************************************************************************/

FASTRUN void ShowMotorTimer()
{
    char FrontView_Hours[] = "Hours";
    char FrontView_Mins[]  = "Mins";
    char FrontView_Secs[]  = "Secs";

    if (TimesUp) return;

    uint8_t Recording[10] = {ONEMINUTE, TWOMINUTES, THREEMINUTES, FOURMINUTES, FIVEMINUTES, SIXMINUTES, SEVENMINUTES, EIGHTMINUTES, NINEMINUTES, TENMINUTES};

    uint8_t Cdown[10] = {TEN, NINE, EIGHT, SEVEN, SIX, FIVE, FOUR, THREE, TWO, ONE};

    if ((MotorEnabled && (!LostContactFlag || PPMdata.UseTXModule))) {
        ElapsedSeconds = ((millis() - TimerMillis) / 1000) + PausedSecs;
        Secs           = ElapsedSeconds;
        if (TimerDownwards) Secs = TimerStartTime - ElapsedSeconds;
        Hours = Secs / 3600;
        Secs %= 3600;
        Mins = Secs / 60;
        Secs %= 60;
    }
    if (LastSeconds != Secs) {

        ClockSpoken  = false;
        ClockSpoken1 = false;
        if (CurrentView == FRONTVIEW) {
            SendValue(FrontView_Secs, Secs);
            SendValue(FrontView_Mins, Mins);
            SendValue(FrontView_Hours, Hours);
        }
        LastSeconds = Secs;
    }

    if (TimerDownwards) {
        if ((Secs < 11) && !Mins && !ClockSpoken1 && (ElapsedSeconds > 2)) {
            PlaySound(Cdown[CountDownIndex]);
            ++CountDownIndex;
            ClockSpoken1 = true;
        }
    }

    if (!Secs && SpeakingClock && !ClockSpoken) {
        ClockSpoken = true;
        if ((Mins <= 10) && (Mins > 0)) {
            PlaySound(Recording[Mins - 1]);
        }
        if (TimerDownwards) {
            if ((!Mins) && (!Secs) && (ElapsedSeconds > 2)) {
                PlaySound(STORAGECHARGE); // = Stop Flyfing!
                TimesUp        = true;
                CountDownIndex = 0;
            }
        }
    }
}

/*********************************************************************************************************************************/

FASTRUN void ShowServoPos()
{
    uint8_t  MinimumDistance = 10;             // if the change is very small, don't re-display anything - to reduce flashing. :=)!!
    uint32_t Hertz           = 25;             // Fast
    if (CurrentView == GRAPHVIEW) Hertz = 200; // Slower!
    if (millis() - ShowServoTimer < Hertz) return;
    ShowServoTimer = millis();

    char     Ch_Lables[16][5] = {"Ch1", "Ch2", "Ch3", "Ch4", "Ch5", "Ch6", "Ch7", "Ch8", "Ch9", "Ch10", "Ch11", "Ch12", "Ch13", "Ch14", "Ch15", "Ch16"};
    char     ChannelInput[]   = "Input";
    char     ChannelOutput[]  = "Output";
    uint16_t StickPosition    = 54321;
    int      l                = 0;
    int      l1               = 0;

    // The first 8 channels are displayed on all three of these screens
    if ((CurrentView == STICKSVIEW) || (CurrentView == FRONTVIEW) || (CurrentView == CALIBRATEVIEW)) {
        for (int i = 0; i < 8; ++i) {
            if (ChannelOutPut[i] > 3)
                MinimumDistance = 40;                                    // coarse for pots & switches...
            else
                MinimumDistance = 10;                                    // ...finer for sticks
            if (abs(SendBuffer[i] - ShownBuffer[i]) > MinimumDistance) { // no need to show tiny movements
                SendValue(Ch_Lables[i], IntoLowerRes(SendBuffer[i]));
                ShownBuffer[i] = SendBuffer[i];
            }
        }
    }
    MinimumDistance = 10;
    // The second 8 channels are displayed on only two screens
    if ((CurrentView == STICKSVIEW) || (CurrentView == FRONTVIEW)) {
        for (int i = 8; i < 16; ++i) {
            if (abs(SendBuffer[i] - ShownBuffer[i]) > MinimumDistance) {
                SendValue(Ch_Lables[i], IntoLowerRes(SendBuffer[i]));
                ShownBuffer[i] = SendBuffer[i];
            }
        }
    }
    if ((CurrentView == GRAPHVIEW)) {
#define fixitx 35

        MinimumDistance = 3; // if the change is very small, don't re-display anything - to reduce flashing. :=)!!

        l = (InPutStick[ChanneltoSet - 1]);
        if (ChanneltoSet <= 8)
            l1 = AnalogueReed(l);
        else
            l1 = GetStickInputInputOnly(l);

        if (l1 <= ChannelCentre[l]) {
            SendValue(ChannelInput, map(l1, ChannelCentre[l], ChannelMin[l], 0, -100));
            StickPosition = map(l1, ChannelMin[l], ChannelCentre[l], BoxLeft - 0, BoxLeft + (((BoxRight - fixitx) - BoxLeft) / 2));
        }
        else {
            SendValue(ChannelInput, map(l1, ChannelCentre[l], ChannelMax[l], 0, 100));
            StickPosition = map(l1, ChannelCentre[l], ChannelMax[l], BoxLeft + (((BoxRight - fixitx) - BoxLeft) / 2), BoxRight - fixitx);
        }
       
        if ((abs(StickPosition - SavedLineX) > MinimumDistance)) {
            DisplayCurve();                                                                                        // needed to clear last line
            DrawLine(StickPosition - 1, BoxTop + 3, StickPosition - 1, (BoxBottom - 3) - BoxTop, HighlightColour); // draws line for stick position
            SendValue(ChannelOutput, map(SendBuffer[ChanneltoSet - 1], MINMICROS, MAXMICROS, -100, 100));
            SavedLineX = StickPosition;
        }
    }
}

/*********************************************************************************************************************************/
FASTRUN bool CheckTXVolts()
{
    char  DataView_txv[]   = "txv";  // Labels on Nextion
    char  JTX[]            = "JTX";  // Labels on Nextion
    char  FrontView_TXBV[] = "TXBV"; // Labels on Nextion
    bool  TXWarningFlag    = false;
    float TransmitterBatteryPercentLeft, TransmitterBatteryVolts;
    char  TXBattInfo[80];
    char  pc[] = "%";
    char  nbuf[10]; // Little buffer for numbers
    char  v[]   = "V";
    char  t17[] = "t17";

    if (USE_INA219) {
        TransmitterBatteryVolts = ((ina219.getBusVoltage_V()) * 100) + (TxVoltageCorrection * 2);        // Correction for inaccurate ina219
        dtostrf(TransmitterBatteryVolts / 200, 2, 2, nbuf);                                              // Volts per cell
        if (TXLiPo) {                                                                                    // Does TX have a LiPo or a LiFePo4?
            TransmitterBatteryPercentLeft = map(TransmitterBatteryVolts, 3.5 * 200, 4.00 * 200, 0, 100); // LIPO Battery 3.50 -> c. 4.00  volts per cell
        }
        else {                                                                                           // No, it's a LiFePo4
            TransmitterBatteryPercentLeft = map(TransmitterBatteryVolts, 3.2 * 200, 3.33 * 200, 0, 100); // LiFePo4 Battery 3.1 -> 3.35  volts per cell
        }
        if (TransmitterBatteryPercentLeft < LowBattery) {
            TXWarningFlag = true;
            WarningSound  = BATTERYISLOW;
        }
        TransmitterBatteryPercentLeft = constrain(TransmitterBatteryPercentLeft, 0, 100);
        strcat(TXBattInfo, pc);
        if (CurrentView == FRONTVIEW) {
            SendValue(JTX, TransmitterBatteryPercentLeft);
            strcat(nbuf, v);
            SendText(FrontView_TXBV, nbuf);
        }
        if (CurrentView == DATAVIEW) {
            SendText(DataView_txv, TransmitterVersionNumber);
            SendText(t17, nbuf);
        }
    }
    return TXWarningFlag;
}
/*********************************************************************************************************************************/

FASTRUN bool CheckRXVolts()
{
    float   ReadVolts       = 0;
    uint8_t GreenPercentBar = 0;
    char    JRX[]           = "JRX";
    bool    RXWarningFlag   = false;
    char    Vbuf[10];
    char    RXBattInfo[80];
    float   VoltsPerCell     = 0;
    char    FrontView_RXBV[] = "RXBV";
    char    RXPC[]           = "RXPC";
    char    PerCell[]        = " per cell)";
    char    RXBattNA[]       = ""; //"(No data from RX)";
    char    v[]              = "V  (";
    char    pc[]             = "%";
    char    spaces[]         = "  ";
    char    t6[]             = "t6";

    ReadVolts       = (RXModelVolts * 100) + (RxVoltageCorrection * RXCellCount);
    GreenPercentBar = map(ReadVolts, 3.4f * RXCellCount * 100, 4.2f * RXCellCount * 100, 0, 100);
    if (RXVoltsDetected) {
        GreenPercentBar = constrain(GreenPercentBar, 0, 100);
        WarningSound    = BATTERYISLOW;
        if (BoundFlag) {
            VoltsPerCell = (ReadVolts / RXCellCount) / 100;
            if (CurrentView == FRONTVIEW) {
                SendValue(JRX, GreenPercentBar);
                strcat(Str(Vbuf, GreenPercentBar, 0), pc);
                SendText(RXPC, Vbuf);
                strcpy(RXBattInfo, ModelVolts);
                strcat(RXBattInfo, v);
                dtostrf(VoltsPerCell, 2, 2, Vbuf);
                strcat(RXBattInfo, Vbuf);
                strcat(RXBattInfo, PerCell);
                SendText(FrontView_RXBV, RXBattInfo);
            }
            if (CurrentView == DATAVIEW) {
                dtostrf(VoltsPerCell, 2, 2, Vbuf);
                SendText(t6, Vbuf);
            }

            if (VoltsPerCell < StopFlyingVoltsPerCell && GreenPercentBar > 0) {
                if (!LowVoltstimer) LowVoltstimer = millis();      // Start a timer if not running already
                if (millis() - LowVoltstimer > LOW_VOLTAGE_TIME) { // Is RX Lipo down to storage volts for over 5 seconds?
                    RXWarningFlag = true;
                    WarningSound  = STORAGECHARGE;
                }
            }
            else {
                LowVoltstimer = 0;
                RXWarningFlag = false; // Reset timer as voltage recovered
            }
        }
    }
    else {
        if (BoundFlag && CurrentView == FRONTVIEW) {
            SendText(FrontView_RXBV, RXBattNA);
            SendValue(JRX, 0);
            SendText(RXPC, spaces);
        }
    }
    RXVoltsDetected = false; // in case it was a glitch
    return RXWarningFlag;
}

/*********************************************************************************************************************************/

/** @brief SHOW COMMS */

// This displays many telemetry data onto the current screen

FASTRUN void ShowComms()
{
    if (millis() - LastShowTime < SHOWCOMMSDELAY) return;

    LastShowTime = millis();

    char         FrontView_AckPayload[] = "AckPayload";
    char         FrontView_RXBV[]       = "RXBV";
    char         Msg_CnctdBuddyMast[]   = "* MASTER has control *";
    char         Msg_CnctdBuddySlave[]  = "* BUDDY has control *";
    char         MsgBuddying[]          = "* Buddy *";
    char         DataView_pps[]         = "pps"; // These are label names in the NEXTION data screen. They are best kept short.
    char         DataView_lps[]         = "lps";
    char         DataView_Alt[]         = "alt";
    char         DataView_Temp[]        = "Temp";
    char         DataView_MaxAlt[]      = "MaxAlt";
    char         DataView_rxv[]         = "rxv";
    char         DataView_Ls[]          = "Ls";
    char         DataView_Ts[]          = "Ts";
    char         DataView_Rx[]          = "rx";
    char         DataView_Sg[]          = "Sg";
    char         DataView_Ag[]          = "Ag";
    char         DataView_Gc[]          = "Gc";
    char         Vbuf[50];
    char         Fix[]                 = "Fix"; // These are label names in the NEXTION data screen. They are best kept short.
    char         Lon[]                 = "Lon";
    char         Lat[]                 = "Lat";
    char         Bear[]                = "Bear";
    char         Dist[]                = "Dist";
    char         Sped[]                = "Sped";
    char         yes[]                 = "Yes";
    char         no[]                  = "No";
    char         ALT[]                 = "ALT";
    char         MALT[]                = "MALT";
    char         MxS[]                 = "MxS";
    char         Mxd[]                 = "Mxd";
    char         BTo[]                 = "BTo";
    char         Sat[]                 = "Sat";
    char         Sbs[]                 = "Sbus";
    char         rate[]                = "rate";
    char         rate1[]               = "Rate 1";
    char         rate2[]               = "Rate 2";
    char         rate3[]               = "Rate 3";
    char         rate4[]               = "      ";
    char         FrontView_Connected[] = "Connected";
    char         WarnNow[]             = "vis Warning,1";
    char         WarnOff[]             = "vis Warning,0";
    char         InVisible[]           = "vis Quality,0";
    char         IdReceived[]          = "t22";
    char         IdStored[]            = "t19";
    char         IdReceived1[]         = "t23";
    char         IdStored1[]           = "t24";
    char         ams[]                 = "ams";
    char         trall[]               = "trall";
    char         AmsOnMsg[]            = "AMS";
    char         AmsOffMsg[]           = "   ";
    char         CopyTrimsToAllMSG[]   = "TrimAll";
    char         CopyTrimsToNoneMSG[]  = "       ";
    unsigned int TempModelId           = 0;

    if (CurrentView == FRONTVIEW) {
        ShowConnectionQuality();
        if (AutoModelSelect) {
            SendText(ams, AmsOnMsg);
        }
        else {
            SendText(ams, AmsOffMsg);
        }
        if (CopyTrimsToAll) {
            SendText(trall, CopyTrimsToAllMSG);
        }
        else {
            SendText(trall, CopyTrimsToNoneMSG);
        }
        switch (DualRateInUse)
        {
            case 1:
                SendText(rate, rate1);
                break;
            case 2:
                SendText(rate, rate2);
                break;
            case 3:
                SendText(rate, rate3);
                break;
            case 4:
                SendText(rate, rate4); // rates not in use = 4
                break;
            default:
                break;
        }
        if (BuddyPupilOnPPM) SendText(FrontView_Connected, MsgBuddying);
        if (LedWasGreen) {
            if (BoundFlag) {
                if (!BuddyMaster) {
                    if (!Reconnected) {
                        ShowConnectionQuality();
                        Reconnected = true;
                    }
                }
                else {
                    if (!SlaveHasControl) {
                        SendText(FrontView_Connected, Msg_CnctdBuddyMast);
                    }
                    else {
                        SendText(FrontView_Connected, Msg_CnctdBuddySlave);
                    }
                }
                StartInactvityTimeout();
            }
            else {
                SendText(FrontView_RXBV, na); // data not available
                SendText(FrontView_AckPayload, na);
                SendCommand(InVisible);
            }
        }
    }
    if (CurrentView == DATAVIEW && Connected) {
        SendValue(DataView_pps, PacketsPerSecond);
        SendValue(DataView_lps, TotalLostPackets / 2); // about half probably made it but went un acknoledged
        SendText(DataView_Alt, ModelAltitude);
        SendText(DataView_MaxAlt, Maxaltitude);
        SendText(DataView_Temp, ModelTempRX);
        SendText(DataView_Rx, ThisRadio);
        SendText(DataView_rxv, ReceiverVersionNumber);
        SendValue(DataView_Ls, GapLongest);
        SendValue(DataView_Ts, RadioSwaps - SavedRadioSwaps);
        SendValue(DataView_Sg, RX1TotalTime - SavedRX1TotalTime);
        SendValue(DataView_Ag, GapAverage);
        SendValue(DataView_Gc, RX2TotalTime - SavedRX2TotalTime);
        snprintf(Vbuf, 7, "%d", (int)SbusRepeats - SavedSbusRepeats);
        SendText(Sbs, Vbuf);
        SendValue(DataView_lps, TotalLostPackets / 2);

        TempModelId = ModelsMacUnionSaved.Val32[0];
        snprintf(Vbuf, 9, "%X", TempModelId);
        if (TempModelId) SendText(IdStored, Vbuf);
        TempModelId = ModelsMacUnionSaved.Val32[1];
        snprintf(Vbuf, 9, "%X", TempModelId);
        if (TempModelId) SendText(IdStored1, Vbuf);

        TempModelId = ModelsMacUnion.Val32[0];
        snprintf(Vbuf, 9, "%X", TempModelId);
        if (TempModelId) SendText(IdReceived, Vbuf);
        TempModelId = ModelsMacUnion.Val32[1];
        snprintf(Vbuf, 9, "%X", TempModelId);
        if (TempModelId) SendText(IdReceived1, Vbuf);
    }
    if (CurrentView == GPSVIEW) {
        if (GpsFix) { // if no fix, then leave display as before
            SendText(Fix, yes);
        }
        else {
            SendText(Fix, no);
        }
        snprintf(Vbuf, 7, "%d", GPSSatellites);
        SendText(Sat, Vbuf);
        snprintf(Vbuf, 10, "%f", GPSLongitude);
        SendText(Lon, Vbuf);
        snprintf(Vbuf, 10, "%f", GPSLatitude);
        SendText(Lat, Vbuf);
        snprintf(Vbuf, 7, "%d", int(GPSAngle));
        SendText(Bear, Vbuf);
        snprintf(Vbuf, 7, "%d", (int)GPSDistanceTo);
        SendText(Dist, Vbuf);
        snprintf(Vbuf, 6, "%d", (int)GPSSpeed);
        SendText(Sped, Vbuf);
        snprintf(Vbuf, 6, "%d", (int)GPSMaxSpeed);
        SendText(MxS, Vbuf);
        snprintf(Vbuf, 6, "%d", (int)GPSAltitude);
        SendText(ALT, Vbuf);
        snprintf(Vbuf, 6, "%d", (int)GPSMaxaltitude);
        SendText(MALT, Vbuf);
        snprintf(Vbuf, 6, "%d", (int)GPSCourseTo);
        SendText(BTo, Vbuf);
        snprintf(Vbuf, 6, "%d", (int)GPSMaxDistance);
        SendText(Mxd, Vbuf);
    }
    CheckScreenTime();
    if (CheckTXVolts() || CheckRXVolts()) { // Note: If TX Battery is low, then CheckRXVolts() is not even called.
        if ((millis() - WarningTimer) > 10000) {
            WarningTimer = millis();
            PlaySound(WarningSound); // Issue audible warning every 10 seconds
        }
        if (CurrentView == FRONTVIEW) SendCommand(WarnNow);
    }
    else {
        if (LedIsBlinking && (CurrentView == FRONTVIEW)) SendCommand(WarnOff);
    }
} // end ShowComms()

/*********************************************************************************************************************************/
FASTRUN uint16_t GetStickInput(uint8_t l) // This returns the proper output - not just input
{
    uint16_t k = 0;
    switch (l) {
        case 8:
            if (Channel9SwitchValue == 0) k = IntoHigherRes(MinDegrees[Bank][8]);
            if (Channel9SwitchValue == 90) k = IntoHigherRes(CentreDegrees[Bank][8]);
            if (Channel9SwitchValue == 180) k = IntoHigherRes(MaxDegrees[Bank][8]);
            break;
        case 9:
            if (Channel10SwitchValue == 0) k = IntoHigherRes(MinDegrees[Bank][9]);
            if (Channel10SwitchValue == 90) k = IntoHigherRes(CentreDegrees[Bank][9]);
            if (Channel10SwitchValue == 180) k = IntoHigherRes(MaxDegrees[Bank][9]);
            break;
        case 10:
            if (Channel11SwitchValue == 0) k = IntoHigherRes(MinDegrees[Bank][10]);
            if (Channel11SwitchValue == 90) k = IntoHigherRes(CentreDegrees[Bank][10]);
            if (Channel11SwitchValue == 180) k = IntoHigherRes(MaxDegrees[Bank][10]);
            break;
        case 11:
            if (Channel12SwitchValue == 0) k = IntoHigherRes(MinDegrees[Bank][11]);
            if (Channel12SwitchValue == 90) k = IntoHigherRes(CentreDegrees[Bank][11]);
            if (Channel12SwitchValue == 180) k = IntoHigherRes(MaxDegrees[Bank][11]);
            break;
        default:
            k = IntoHigherRes(90); // channels 13,14,15,16 are simply centred
            break;
    }
    return k;
}

/*********************************************************************************************************************************/
FASTRUN uint16_t GetStickInputInputOnly(uint8_t l) // This returns the input only
{
    uint16_t k = 0;
    switch (l) {
        case 8:
            if (Channel9SwitchValue == 0) k = ChannelMin[l];
            if (Channel9SwitchValue == 90) k = ChannelCentre[l];
            if (Channel9SwitchValue == 180) k = ChannelMax[l];
            break;
        case 9:
            if (Channel10SwitchValue == 0) k = ChannelMin[l];
            if (Channel10SwitchValue == 90) k = ChannelCentre[l];
            if (Channel10SwitchValue == 180) k = ChannelMax[l];
            break;
        case 10:
            if (Channel11SwitchValue == 0) k = ChannelMin[l];
            if (Channel11SwitchValue == 90) k = ChannelCentre[l];
            if (Channel11SwitchValue == 180) k = ChannelMax[l];
            break;
        case 11:
            if (Channel12SwitchValue == 0) k = ChannelMin[l];
            if (Channel12SwitchValue == 90) k = ChannelCentre[l];
            if (Channel12SwitchValue == 180) k = ChannelMax[l];
            break;
        default:
            k = 1500; // Centre because no input possible
            break;
    }
    return k;
}

/*********************************************************************************************************************************/
// MIXES  (Channel mixes)
/*********************************************************************************************************************************/
FASTRUN void DoMixes()
{
    short MixNumber, ChannelNumber, MixValue, MinimumDeg, MaximumDeg;

    for (MixNumber = 1; MixNumber < MAXMIXES; ++MixNumber)
    {
        if (Mixes[MixNumber][M_Enabled])
        {
            if (Mixes[MixNumber][M_Bank] == Bank || (!Mixes[MixNumber][M_Bank]))
            {
                for (ChannelNumber = 0; ChannelNumber < CHANNELSUSED; ++ChannelNumber)
                {
                    if ((Mixes[MixNumber][M_MasterChannel] - 1) == ChannelNumber)
                    {
                        MixValue = (map(PreMixBuffer[ChannelNumber], MINMICROS, MAXMICROS, -HALFMICROSRANGE, HALFMICROSRANGE)) * (short)Mixes[MixNumber][M_Percent] / 100;

                        if (Mixes[MixNumber][M_ONEDIRECTION])
                        {
                            if (Mixes[MixNumber][M_Reversed])
                            {
                                if (MixValue > 0) MixValue = -MixValue;
                            }
                            else
                            {
                                if (MixValue < 0) MixValue = -MixValue;
                            }
                        }
                        else
                        {
                            if (Mixes[MixNumber][M_Reversed]) MixValue = -MixValue;
                        }
                        MixValue += SendBuffer[(Mixes[MixNumber][M_SlaveChannel]) - 1]; // This is the actual mix moment! (MixValue is now the mixed value)
                        MixValue += (Mixes[MixNumber][M_OFFSET] - 127) * 8;
                        MinimumDeg = IntoHigherRes(MinDegrees[Bank][(Mixes[MixNumber][M_SlaveChannel]) - 1]);
                        MaximumDeg = IntoHigherRes(MaxDegrees[Bank][(Mixes[MixNumber][M_SlaveChannel]) - 1]);
                        if (MinimumDeg > MaximumDeg)
                        {
                            SendBuffer[(Mixes[MixNumber][M_SlaveChannel]) - 1] = constrain(MixValue, MaximumDeg, MinimumDeg);
                        }
                        else
                        {
                            SendBuffer[(Mixes[MixNumber][M_SlaveChannel]) - 1] = constrain(MixValue, MinimumDeg, MaximumDeg);
                        }
                    }
                }
            }
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
    if (InputValue >= ChannelCentre[InputChannel]) {
        k = MapWithExponential(InputValue - ChannelCentre[InputChannel], 0, ChannelMax[InputChannel] - ChannelCentre[InputChannel], 0, IntoHigherRes(CurveDots[4]) - //
                                   IntoHigherRes(CurveDots[2]),
                               Exponential[Bank][OutputChannel]) //
            + IntoHigherRes(CurveDots[2]);                       //
    }
    else {
        k = MapWithExponential(ChannelCentre[InputChannel] - InputValue, 0, ChannelCentre[InputChannel] - ChannelMin[InputChannel], IntoHigherRes(CurveDots[2]) - //
                                   IntoHigherRes(CurveDots[0]),
                               0, Exponential[Bank][OutputChannel]) //
            + IntoHigherRes(CurveDots[0]);
    }
    return k;
}

/*********************************************************************************************************************************/
// ************* Small function pointer array for interpolation types ************************************************************

uint16_t (*Interpolate[3])(uint16_t InputValue, uint16_t InputChannel, uint16_t OutputChannel) {
    StraightLineInterpolation,  // 0
    CatmullSplineInterpolation, // 1
    ExponentialInterpolation    // 2
};

/*********************************************************************************************************************************/

float CalculateRate(short int Curve, short int OutputChannel, float rate)
{

    switch (Curve) { // Curve is 1 - 5, low to hi.
        case 0:
            return (((MinDegrees[Bank][OutputChannel]) - 90) * (rate / 100)) + 90;
        case 1:
            return (((MidLowDegrees[Bank][OutputChannel]) - 90) * (rate / 100)) + 90;
        case 2:
            return (((CentreDegrees[Bank][OutputChannel]) - 90) * (rate / 100)) + 90;
        case 3:
            return (((MidHiDegrees[Bank][OutputChannel]) - 90) * (rate / 100)) + 90;
        case 4:
            return (((MaxDegrees[Bank][OutputChannel]) - 90) * (rate / 100)) + 90;
        default:
            return 0;
    }
}

/*********************************************************************************************************************************/

float UseFullRate(short int Curve, uint8_t OutputChannel)
{

    switch (Curve) { // Curve is 1 - 5, low to hi.
        case 0:
            return MinDegrees[Bank][OutputChannel];
        case 1:
            return MidLowDegrees[Bank][OutputChannel];
        case 2:
            return CentreDegrees[Bank][OutputChannel];
        case 3:
            return MidHiDegrees[Bank][OutputChannel];
        case 4:
            return MaxDegrees[Bank][OutputChannel];
        default:
            return 0;
    }
}

/*********************************************************************************************************************************/

void GetCurveDots(uint16_t OutputChannel, uint16_t TheRate) // This for the Dual Rates function
{                                                           // This for the Dual Rates function
                                                            // Effectively, it just copies the Y dot's magnitude on the curve, but might reduce the extent if rate is not 100 and channel specified
    if (TheRate != 100) {                                   // Not 100% ?
        for (int j = 0; j < 8; ++j) {                       // 8 possible rates in any position of output
            if (DualRateChannels[j]) {                      // non zero?
                if (OutputChannel + 1 == DualRateChannels[j]) {
                    for (int i = 0; i < 5; ++i) CurveDots[i] = CalculateRate(i, OutputChannel, TheRate);
                    return;
                }
            }
        }
    }
    for (int i = 0; i < 5; ++i) CurveDots[i] = UseFullRate(i, OutputChannel); // ... channel not used so 100%
}

/*********************************************************************************************************************************/
/**************************** This function implements slowed servos for flaps, U/Cs etc. ****************************************/
/*********************************************************************************************************************************/

void DoSlowServos()
{
    static uint32_t SlowTime[16];                                                //
    for (int i = 0; i < 16; ++i) {                                               // Test every channel
        if (StepSize[i] < 100) {                                                 // If StepSize = 100, use full speed. No slowing
            if ((millis() - SlowTime[i]) > 10) {                                 // This next part runs only 100 times per second
                SlowTime[i] = millis();                                          // Store start time of this iteration
                if (CurrentPosition[i] == 0) CurrentPosition[i] = SendBuffer[i]; // Must start somewhere
                int distance = SendBuffer[i] - CurrentPosition[i];               // Define how far to move
                int SSize    = StepSize[i];                                      // Get step size
                if (SSize > abs(distance)) SSize = 1;                            // This avoids overshooting the limit
                if (distance < 0) SSize = -SSize;                                // Negative?
                if (!distance) SSize = 0;                                        // Already arrived?
                CurrentPosition[i] += SSize;                                     // Move Current Position a little bit towards goal
            }
            SendBuffer[i]   = CurrentPosition[i]; // Modify next servo position
            PreMixBuffer[i] = SendBuffer[i];      // mix the slowed version
        }
    }
}
/*********************************************************************************************************************************/
void DoRouteOutputs()
{ // This function re-routes outputs to the defined channels

    uint16_t temp[CHANNELSUSED];
    for (int i = 0; i < CHANNELSUSED; ++i) {
        temp[i] = PreMixBuffer[i];
    }
    for (int i = 0; i < CHANNELSUSED; ++i) {
        PreMixBuffer[i] = temp[ChannelOutPut[i]];
        SendBuffer[i]   = PreMixBuffer[i];
    }
}

/*********************************************************************************************************************************/

/** @brief GET NEW SERVO POSITIONS */
FASTRUN void
GetNewChannelValues()
{
    if (NewCompressNeeded) return; // Have we compressed the last one yet?
    NewCompressNeeded = true;      // Yes indeed. It's therefore time for new data.
    uint16_t OutputValue, InputChannel, InputValue, OutputChannel;
    for (OutputChannel = 0; OutputChannel < CHANNELSUSED; ++OutputChannel) { // Do every channel
        InputChannel = InPutStick[OutputChannel];                            // Input sticks knobs & switches are mapped by user
        GetCurveDots(OutputChannel, DualRateValue);                          // This for the Dual Rates function
        if (InputChannel > 7) {                                              // Must be a switch if over 7
            OutputValue = GetStickInput(InputChannel);                       // Four 3 postion switches
        }
        else {                                                                                                           // i.e. l <= 7 so it's a Stick/knob/switch
            InputValue  = AnalogueReed(InputChannel);                                                                    // Get values from sticks' pots then ADD TRIM then interpolate them.
            OutputValue = Interpolate[InterpolationTypes[Bank][OutputChannel]](InputValue, InputChannel, OutputChannel); // Use function pointer array to invoke selected interpolation.
        }
        OutputValue += GetTrimAmount(InputChannel);                        // Add trim AFTER doing rates
        OutputValue += (SubTrims[OutputChannel] - 127) * (TrimMultiplier); // ADD SUBTRIM to output channel, not mapped input channel (Range 0 - 127 - 254)
        PreMixBuffer[OutputChannel] = constrain(OutputValue, MINMICROS, MAXMICROS);
        SendBuffer[OutputChannel]   = PreMixBuffer[OutputChannel];
    }
    if (CurrentMode == NORMAL) {
        DoSlowServos();   // Some servos may need to be slowed down
        DoRouteOutputs(); // This function might re-route outputs to user-defined channels (Before reversing)
        DoReverseSense(); // This function reverses servos if needed (After routing)
        DoMixes();        // Mixes the OUTPUT :-)
    }
}
/*********************************************************************************************************************************/

void ReduceLimits()
{ // Get things setup for sticks calibration
    for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        ChannelMax[i] = 512;
        ChannelMin[i] = 512;
    }
    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        MaxDegrees[Bank][i]    = 180;
        CentreDegrees[Bank][i] = 90;
        MinDegrees[Bank][i]    = 0;
    }
}

/*********************************************************************************************************************************/
void ResetSwitchNumbers()
{
    for (int i = 0; i < 8; ++i) {
        SwitchNumber[i] = DefaultSwitchNumber[i];
    }
}
/*********************************************************************************************************************************/
void CalibrateSticks() // This discovers end of travel place for sticks etc.
{
    uint16_t p;
    for (uint8_t i = 0; i < PROPOCHANNELS; ++i)
    {
        p = analogRead(AnalogueInput[i]);
        if (ChannelMax[i] < p) ChannelMax[i] = p;
        if (ChannelMin[i] > p) ChannelMin[i] = p;
    }
    NewCompressNeeded = false; // fake it as we are not sending data
    GetNewChannelValues();
}
/*********************************************************************************************************************************/
/** @brief Get centre as 90 degrees */
void ChannelCentres()
{
    for (int i = 0; i < PROPOCHANNELS; ++i) {
        ChannelCentre[i] = AnalogueReed(i);
        ChannelMidHi[i]  = ChannelCentre[i] + ((ChannelMax[i] - ChannelCentre[i]) / 2);
        ChannelMidLow[i] = ChannelMin[i] + ((ChannelCentre[i] - ChannelMin[i]) / 2);
    }
    for (int i = PROPOCHANNELS; i < CHANNELSUSED; ++i) {
        ChannelMin[i]    = 500;
        ChannelCentre[i] = 1500;
        ChannelMax[i]    = 2500;
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
    for (ii = 1; ii < 127; ++ii) {
        Wire.beginTransmission(ii);
        if (Wire.endTransmission() == 0) {
#ifdef DB_SENSORS
            Serial.print(ii, HEX); // in case new one shows up
            Serial.print("   ");
            if (ii == 0x40) Serial.println("INA219 voltage meter detected!");
#endif
            if (ii == 0x40) {
                USE_INA219 = true;
            }
        }
    }
}

/*********************************************************************************************************************************/

void CloseModelsFile()
{
    if (ModelsFileOpen) {
        ModelsFileNumber.close();
        ModelsFileOpen = false;
        delayMicroseconds(500);
    }
}

/*********************************************************************************************************************************/

bool CheckFileExists(char* fl)
{
    CloseModelsFile();
    bool exists = false;
    File t;
    t = SD.open(fl, FILE_READ);
    if (t) exists = true;
    t.close();
    return exists;
}

/*********************************************************************************************************************************/

void ShortDelay()
{
    delayMicroseconds(10);
}

void ShortishDelay()
{
    delayMicroseconds(1750);
}

/*********************************************************************************************************************************/

void OpenModelsFile()
{

    char ModelsFile[] = "models.dat";

    if (!ModelsFileOpen) {
        if (SingleModelFlag) {
            ModelsFileNumber = SD.open(SingleModelFile, FILE_WRITE);
            DelayWithDog(100);
        }
        else {
            ModelsFileNumber = SD.open(ModelsFile, FILE_WRITE);
            DelayWithDog(100);
        }
        if (ModelsFileNumber == 0) {
            FileError = true;
        }
        else {
            ModelsFileOpen = true;
        }
    }
}
/*********************************************************************************************************************************/

void BuildCheckSum(int p_address, short int p_value)
{
    if (!DoingCheckSm) FileCheckSum += (p_value * (p_address + 1)); // don't include checksum in its own calculation
}

/*********************************************************************************************************************************/

void SDUpdate32BITS(int p_address, uint32_t p_value)
{
    BuildCheckSum(p_address, p_value);
    ModelsFileNumber.seek(p_address);
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value));
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value >> 8));
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value >> 16));
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value >> 24));
    ShortDelay();
}

/*********************************************************************************************************************************/

uint32_t SDRead32BITS(int p_address)
{
    ModelsFileNumber.seek(p_address);
    uint32_t r = ModelsFileNumber.read();
    r += ModelsFileNumber.read() << 8;
    r += ModelsFileNumber.read() << 16;
    r += ModelsFileNumber.read() << 24;
    return r;
}
/*********************************************************************************************************************************/

void SDUpdate16BITS(int p_address, short int p_value)
{
    BuildCheckSum(p_address, p_value);
    ModelsFileNumber.seek(p_address);
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value));
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value >> 8));
    ShortDelay();
}
/*********************************************************************************************************************************/

void SDUpdate8BITS(int p_address, uint8_t p_value)
{
    BuildCheckSum(p_address, p_value);
    ModelsFileNumber.seek(p_address);
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value));
    ShortDelay();
}

/*********************************************************************************************************************************/

short int SDRead16BITS(int p_address)
{

    ModelsFileNumber.seek(p_address);
    short int r = ModelsFileNumber.read();
    r += ModelsFileNumber.read() << 8;
    BuildCheckSum(p_address, r);
    return r;
}

/*********************************************************************************************************************************/

uint8_t SDRead8BITS(int p_address)
{
    ModelsFileNumber.seek(p_address);
    uint8_t r = ModelsFileNumber.read();
    BuildCheckSum(p_address, r);
    return r;
}

/*********************************************************************************************************************************/

void UpdateModelsNameEveryWhere()
{
    char Owner[]             = "Owner";
    char TheModelName[]      = "ModelName";
    char GraphView_Channel[] = "Channel";
    char TrimView_Bank[]     = "t1";
    char GraphView_fmode[]   = "fmode";
    char SticksView_t1[]     = "t1";
    char NoName[40];
    char Ch[] = "Channel ";
    char Nbuf[7];
    char mn1[40]; // holds model name plus its number
    char lb[] = " (";
    char rb[] = ")";

    strcpy(mn1, ModelName);

    if (CurrentView == FRONTVIEW) SendText(Owner, TxName);

    if (CurrentView != MODELSVIEW) {
        strcat(mn1, lb);
        strcat(mn1, Str(Nbuf, ModelNumber, 0)); // Add model number for extra clarity
        strcat(mn1, rb);
    }

    SendText(TheModelName, mn1);

    if (CurrentView == GRAPHVIEW) {
        if (strlen(ChannelNames[ChanneltoSet - 1]) < 2) { // if no name, just show the channel number
            strcpy(NoName, Ch);
            SendText(GraphView_Channel, strcat(NoName, Str(Nbuf, ChanneltoSet, 0)));
        }
        else {
            SendText(GraphView_Channel, ChannelNames[ChanneltoSet - 1]);
        }
    }
    if (CurrentView == STICKSVIEW) SendText(SticksView_t1, BankTexts[BanksInUse[Bank - 1]]);
    if (CurrentView == GRAPHVIEW) SendText(GraphView_fmode, BankTexts[BanksInUse[Bank - 1]]);
    if (CurrentView == TRIM_VIEW) {
        SendText(TrimView_Bank, BankTexts[BanksInUse[Bank - 1]]);
        UpdateTrimView();
    }
}

/*********************************************************************************************************************************/

FLASHMEM void InitSwitchesAndTrims()
{
    for (int i = 0; i < 8; ++i) {
        pinMode(SwitchNumber[i], INPUT_PULLUP);
        pinMode(TrimNumber[i], INPUT_PULLUP);
    }
    pinMode(BUTTON_SENSE_PIN, INPUT_PULLUP); // New function to sense power button press
}

/*********************************************************************************************************************************/

/** @brief STICKS CALIBRATION */
FLASHMEM void InitMaxMin()
{
    for (int i = 0; i < CHANNELSUSED; ++i) {
        ChannelMax[i]    = 1024;
        ChannelMidHi[i]  = 512 + 256;
        ChannelCentre[i] = 512;
        ChannelMidLow[i] = 256;
        ChannelMin[i]    = 0;
    }
}

/*********************************************************************************************************************************/

FLASHMEM void InitCentreDegrees()
{

    for (int j = 1; j <= 4; ++j) {
        for (int i = 0; i < CHANNELSUSED; ++i) {
            MaxDegrees[j][i]    = 180; //  180 degrees
            MidHiDegrees[j][i]  = 135;
            CentreDegrees[j][i] = 90; //  90 degrees
            MidLowDegrees[j][i] = 45;
            MinDegrees[j][i]    = 0; //  0 degrees
        }
    }
}

/*********************************************************************************************************************************/

void UpdateButtonLabels()
{
    char InputStick_Labels[16][4]  = {"c1", "c2", "c3", "c4", "c5", "c6", "c7", "c8", "c9", "c10", "c11", "c12", "c13", "c14", "c15", "c16"};
    char OutputStick_Labels[16][4] = {"n4", "n5", "n6", "n7", "n8", "n9", "n10", "n11", "n12", "n13", "n14", "n15", "n16", "n17", "n18", "n19"};
    char fsch_labels[16][5]        = {"ch1", "ch2", "ch3", "ch4", "ch5", "ch6", "ch7", "ch8", "ch9", "ch10", "ch11", "ch12", "ch13", "ch14", "ch15", "ch16"};
    char fs[16][5]                 = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
    char ChannelLabels[16][6]      = {"Sch1", "Sch2", "Sch3", "Sch4", "Sch5", "Sch6", "Sch7", "Sch8", "Sch9", "Sch10", "Sch11", "Sch12", "Sch13", "Sch14", "Sch15", "Sch16"};
    char ChannelNumber[16][6]      = {" (1)", "(2) ", " (3)", "(4) ", " (5)", "(6) ", " (7)", "(8) ", " (9)", "(10) ", " (11)", "(12) ", " (13)", "(14) ", " (15)", "(16) "};
    char ArrowRh[]                 = " >";
    char ArrowLh[]                 = "< ";
    char TrimLabels[4][4]          = {"n0", "n1", "n2", "n3"};
    char LabelText[20];

    if (CurrentView == STICKSVIEW) {
        for (int i = 0; i < 16; ++i) {
            if ((float)i / 2 != (int)i / 2) { // Odd and even labels are formatted differnently here
                strcpy(LabelText, ArrowLh);
                strcat(LabelText, ChannelNumber[i]);
                strcat(LabelText, ChannelNames[i]);
            }
            else {
                strcpy(LabelText, ChannelNames[i]);
                strcat(LabelText, ChannelNumber[i]);
                strcat(LabelText, ArrowRh);
            }
            SendText(ChannelLabels[i], LabelText);
        }
    }
    if (CurrentView == FAILSAFE_VIEW) {
        for (int i = 0; i < 16; ++i) {
            SendValue(fs[i], FailSafeChannel[i]);
        }
    }
    if (CurrentView == INPUTS_VIEW || CurrentView == FAILSAFE_VIEW || CurrentView == REVERSEVIEW || CurrentView == BUDDYCHVIEW || CurrentView == SLOWSERVOVIEW) {
        for (int i = 0; i < 16; ++i) {
            SendText(fsch_labels[i], ChannelNames[i]);
            SendValue(InputStick_Labels[i], InPutStick[i] + 1);
            SendValue(OutputStick_Labels[i], ChannelOutPut[i] + 1);
            if (CurrentView != SLOWSERVOVIEW)
            {
                if (i < 4) SendValue(TrimLabels[i], InputTrim[i] + 1); //
            }
        }
    }
}

/*********************************************************************************************************************************/
void CheckBanksInUse()
{

    for (int i = 0; i < 4; ++i) {
        if (BanksInUse[i] > 23) BanksInUse[i] = i;
    }
}

/*********************************************************************************************************************************/
void CheckStepSizes()
{ // for slow servos
    for (int i = 0; i < 16; ++i)
        if (StepSize[i] > 100) StepSize[i] = 100;
}

/*********************************************************************************************************************************/
bool CheckDuplicate(uint8_t ch, int8_t j)
{
    for (int i = 0; i < j; ++i) {
        if (ch == ChannelOutPut[i]) return false; // no duplicates allowed
    }
    return true;
}
/*********************************************************************************************************************************/

void CheckOutPutChannels() // This function checks for bad or duplicate output channels and resets them if found
{
    bool resetit = false;
    for (int i = 0; i < 16; ++i) {
        if ((ChannelOutPut[i] > 15) || (!CheckDuplicate(ChannelOutPut[i], i)))
        {
            resetit = true;
            break;
        }
    }
    if (resetit) {
        for (int i = 0; i < 16; ++i) {
            ChannelOutPut[i] = i;
        }
    }
}
/*********************************************************************************************************************************/

bool ReadOneModel(uint32_t Mnum)
{
    uint16_t j;
    uint16_t i;
    char     NoModelYet[] = "Error! ";

    FileCheckSum = 0;
    if ((ModelNumber > 90) || (ModelNumber <= 0)) ModelNumber = 1;
    OpenModelsFile();
    if (!ModelsFileOpen) {
        DelayWithDog(300);
        OpenModelsFile();
    }

    strcpy(ModelName, NoModelYet); // indicator of error

    if (!ModelsFileOpen) {
        Serial.println("Cannot open file");
        return false;
    }

    SDCardAddress = TXSIZE;
    if (SingleModelFlag) SDCardAddress = 0; // MODELOFFSET;   // Changed from 250 to 0
    SDCardAddress += ((Mnum - 1) * MODELSIZE);
    StartLocation = SDCardAddress;
    ModelDefined  = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    if (ModelDefined != 42) return false;
    for (j = 0; j < 30; ++j) {
        ModelName[j] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }

    for (i = 0; i < CHANNELSUSED; ++i) {
        for (j = 1; j <= 4; ++j) {
            MaxDegrees[j][i] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
            MidHiDegrees[j][i] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
            CentreDegrees[j][i] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
            MidLowDegrees[j][i] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
            MinDegrees[j][i] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < MAXMIXES; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            Mixes[j][i] = SDRead8BITS(SDCardAddress); // Read mixes
            ++SDCardAddress;
        }
    }

    for (j = 0; j < BANKSUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            Trims[j][i] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < BANKSUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            // These are now disabled and this space is spare
            ++SDCardAddress;
        }
    }
    RXCellCount = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    TrimMultiplier = SDRead16BITS(SDCardAddress);
    TrimMultiplier = CheckRange(TrimMultiplier, 0, 25);
    ++SDCardAddress;
    ++SDCardAddress;
    LowBattery = SDRead8BITS(SDCardAddress);
    if (LowBattery > 100) LowBattery = LOWBATTERY;
    if (LowBattery < 10) LowBattery = LOWBATTERY;
    ++SDCardAddress;
    CopyTrimsToAll = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;

    for (i = 0; i < CHANNELSUSED; ++i) {
        SubTrims[i] = SDRead8BITS(SDCardAddress);
        if ((SubTrims[i] < 10) || (SubTrims[i] > 244)) SubTrims[i] = 127; // centre if undefined or zero
        ++SDCardAddress;
    }
    ReversedChannelBITS = SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    for (i = 0; i < 4; ++i) {
        InputTrim[i] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    RxVoltageCorrection = SDRead16BITS(SDCardAddress);

    ++SDCardAddress;
    ++SDCardAddress;
    CheckSavedTrimValues();
    BuddyControlled = SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    SavedSticksMode = SDRead8BITS(SDCardAddress); // save sticks mode in case of rf transfer
    ++SDCardAddress;                              // 0 Spare Bytes here  *****************************

    for (i = 0; i < CHANNELSUSED; ++i) {
        InPutStick[i] = SDRead8BITS(SDCardAddress);
        if (InPutStick[i] > 16) InPutStick[i] = i; // reset if nothing was saved!
        ++SDCardAddress;
    }

    // **************************

    FMSwitch = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    Autoswitch = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    Channel9Switch = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    Channel10Switch = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    Channel11Switch = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    Channel12Switch = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    SWITCH1Reversed = bool(SDRead8BITS(SDCardAddress));
    ++SDCardAddress;
    SWITCH2Reversed = bool(SDRead8BITS(SDCardAddress));
    ++SDCardAddress;
    SWITCH3Reversed = bool(SDRead8BITS(SDCardAddress));
    ++SDCardAddress;
    SWITCH4Reversed = bool(SDRead8BITS(SDCardAddress));
    ++SDCardAddress;
    for (i = 0; i < CHANNELSUSED; ++i) {
        FailSafeChannel[i] = bool(SDRead8BITS(SDCardAddress));
        if (int(FailSafeChannel[i]) > 1) FailSafeChannel[i] = 0;
        ++SDCardAddress;
    }
    for (i = 0; i < CHANNELSUSED; ++i) {
        for (j = 0; j < 10; ++j) {
            ChannelNames[i][j] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
        }
    }

    for (j = 0; j < BANKSUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            Exponential[j][i] = SDRead8BITS(SDCardAddress);
            if (Exponential[j][i] >= 201 || Exponential[j][i] == 0) {
                Exponential[j][i] = DEFAULT_EXPO;
            }
            ++SDCardAddress;
        }
    }
    for (j = 0; j < BANKSUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            InterpolationTypes[j][i] = SDRead8BITS(SDCardAddress);
            if (InterpolationTypes[j][i] < 0 || InterpolationTypes[j][i] > 2) {
                InterpolationTypes[j][i] = EXPONENTIALCURVES;
            }
            ++SDCardAddress;
        }
    }
    for (j = 0; j < BYTESPERMACRO; ++j) {
        for (i = 0; i < MAXMACROS; ++i) {
            MacrosBuffer[i][j] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
        }
    }
    for (i = 0; i < 8; ++i) {
        ModelsMacUnionSaved.Val8[i] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    UseMotorKill = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    MotorChannelZero = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    MotorChannel = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    SafetySwitch = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    SFV = SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    StopFlyingVoltsPerCell = float(SFV) / 100;
    if (StopFlyingVoltsPerCell < 3 || StopFlyingVoltsPerCell > 4) StopFlyingVoltsPerCell = 3.50; // a useful default stop time?!
    Drate2 = SDRead8BITS(SDCardAddress);
    if ((Drate2 < 10) || (Drate2 > 200)) Drate2 = 100;
    ++SDCardAddress;
    Drate3 = SDRead8BITS(SDCardAddress);
    if ((Drate3 < 10) || (Drate3 > 200)) Drate3 = 100;
    ++SDCardAddress;
    Drate1 = SDRead8BITS(SDCardAddress);
    if ((Drate1 < 10) || (Drate1 > 200)) Drate1 = 100;
    ++SDCardAddress;
    for (int i = 0; i < 8; ++i) {
        DualRateChannels[i] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    CheckDualRatesValues();
    BuddySwitch = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    DualRatesSwitch = SDRead8BITS(SDCardAddress);

    ++SDCardAddress;
    for (i = 0; i < 4; ++i) {
        BanksInUse[i] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    CheckBanksInUse();
    for (i = 0; i < 16; ++i) {
        StepSize[i] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    CheckStepSizes();
    TimerDownwards = (bool)SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    TimerStartTime = SDRead16BITS(SDCardAddress);
    if (TimerStartTime > 120 * 60) TimerStartTime = 5 * 60;
    ++SDCardAddress;
    ++SDCardAddress;
    PPMdata.UseSBUSFromRX = (bool)SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    PPMdata.PPMChannelCount = SDRead16BITS(SDCardAddress);
    if ((PPMdata.PPMChannelCount > 16) || (PPMdata.PPMChannelCount < 1)) { // miles out of range PPMChannelCount?
        PPMdata.PPMChannelCount = 8;
    }
    ++SDCardAddress;
    ++SDCardAddress;

    for (i = 0; i < 16; ++i) {
        ChannelOutPut[i] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    CheckOutPutChannels();

    // **************************************

    ReadCheckSum32();
    OneModelMemory = SDCardAddress - StartLocation;

#ifdef DB_SD
    Serial.print(MemoryForTransmtter);
    Serial.println(" bytes were  used for TX data.");
    Serial.print(TXSIZE);
    Serial.println(" bytes had been reserved.");
    Serial.print("So ");
    Serial.print(TXSIZE - MemoryForTransmtter);
    Serial.println(" spare bytes still remain for TX params.");
    Serial.print("Loaded model number: ");
    Serial.println(ModelNumber);
    Serial.print("Model name: ");
    Serial.println(ModelName);
    Serial.println(" ");
    Serial.print(OneModelMemory);
    Serial.println(" bytes used per model.");
    Serial.print(MODELSIZE - OneModelMemory);
    Serial.println(" spare bytes per model.");
    Serial.print(MODELSIZE);
    Serial.println(" bytes reserved per model.)");
    Serial.println(" ");
#endif // defined DB_SD

#ifdef DB_CHECKSUM
    Serial.print("Read Model: ");
    Serial.print(ModelName);
    Serial.print(" Calculated model checksum: ");
    Serial.println(FileCheckSum);
    Serial.println(" ");
#endif

    UpdateButtonLabels();
    CheckMacrosBuffer();
    return true;
}

/*********************************************************************************************************************************/

bool LoadAllParameters()
{
    int j        = 0;
    int i        = 0;
    FileCheckSum = 0;
    if (!ModelsFileOpen) OpenModelsFile();
    if (!ModelsFileOpen) return false;
    SDCardAddress = 0;
    if ((SDRead16BITS(SDCardAddress)) != 12345) return false; // not a good file?!
    SDCardAddress += 2;
    for (i = 0; i < CHANNELSUSED; ++i) {
        ChannelMin[i] = SDRead16BITS(SDCardAddress);
        SDCardAddress += 2;
        ChannelMidLow[i] = SDRead16BITS(SDCardAddress);
        SDCardAddress += 2;
        ChannelCentre[i] = SDRead16BITS(SDCardAddress);
        SDCardAddress += 2;
        ChannelMidHi[i] = SDRead16BITS(SDCardAddress);
        SDCardAddress += 2;
        ChannelMax[i] = SDRead16BITS(SDCardAddress);
        SDCardAddress += 2;
    }
    BuddyPupilOnPPM = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    BuddyMaster = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    ModelNumber = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    ScreenTimeout = SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    Inactivity_Timeout = SDRead8BITS(SDCardAddress) * TICKSPERMINUTE;
    if (Inactivity_Timeout < INACTIVITYMINIMUM) Inactivity_Timeout = INACTIVITYMINIMUM;
    if (Inactivity_Timeout > INACTIVITYMAXIMUM) Inactivity_Timeout = INACTIVITYMAXIMUM;
    ++SDCardAddress;
    for (j = 0; j < 30; ++j) {
        TxName[j] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    Qnh = SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    DeltaGMT = SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    BackGroundColour = SDRead16BITS(SDCardAddress);
    if (BackGroundColour == 0) BackGroundColour = 214;
    ++SDCardAddress;
    ++SDCardAddress;
    ForeGroundColour = SDRead16BITS(SDCardAddress);
    if (ForeGroundColour == 0) ForeGroundColour = 65535;
    ++SDCardAddress;
    ++SDCardAddress;
    SpecialColour = SDRead16BITS(SDCardAddress);
    if (SpecialColour == 0) SpecialColour = Red;
    ++SDCardAddress;
    ++SDCardAddress;
    HighlightColour = SDRead16BITS(SDCardAddress);
    if (HighlightColour == 0) HighlightColour = Yellow;
    ++SDCardAddress;
    ++SDCardAddress;
    SticksMode = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    AudioVolume = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    Brightness = SDRead8BITS(SDCardAddress);
    if (Brightness < 10) Brightness = 10;
    ++SDCardAddress;
    PlayFanfare = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    TrimClicks = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    ButtonClicks = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    SpeakingClock = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    AnnounceBanks = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    for (i = 0; i < 8; ++i) {
        j = SDRead8BITS(SDCardAddress);
        if ((j >= SWITCH7) && (j <= SWITCH0)) {
            SwitchNumber[i] = j;
        }
        ++SDCardAddress;
    }
    //  spare
    ++SDCardAddress;
    MinimumGap = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    LogRXSwaps = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    UseLog = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    AnnounceConnected = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    for (j = 0; j < 8; ++j) {
        TrimNumber[j] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    TxVoltageCorrection = SDRead16BITS(SDCardAddress);
    if ((TxVoltageCorrection > 20) || (TxVoltageCorrection < 0)) TxVoltageCorrection = 0;
    ++SDCardAddress;
    ++SDCardAddress;
    PowerOffWarningSeconds = SDRead8BITS(SDCardAddress);
    PowerOffWarningSeconds = CheckRange(PowerOffWarningSeconds, 1, 30);
    ++SDCardAddress;
    LEDBrightness = SDRead8BITS(SDCardAddress);
    LEDBrightness = CheckRange(LEDBrightness, 1, 254);
    ++SDCardAddress;
    // one spare tx byte here!
    //
    ++SDCardAddress;
    ConnectionAssessSeconds = SDRead8BITS(SDCardAddress);
    ConnectionAssessSeconds = CheckRange(ConnectionAssessSeconds, 1, 6);
    ++SDCardAddress;
    AutoModelSelect = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    TXLiPo = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    PPMdata.PPMOrderSelection = SDRead8BITS(SDCardAddress);
    if ((PPMdata.PPMOrderSelection > 3) || (PPMdata.PPMOrderSelection < 1)) PPMdata.PPMOrderSelection = 2;
    ++SDCardAddress;
    PPMdata.PPMChannelsNumber = SDRead8BITS(SDCardAddress);
    if ((PPMdata.PPMChannelsNumber > 16) || (PPMdata.PPMChannelsNumber < 1)) PPMdata.PPMChannelsNumber = 6;
    ++SDCardAddress;

    // Not used!!! (yet)  1 tx spare

    ++SDCardAddress;
    PPMdata.UseTXModule = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    ReadCheckSum32();
    CheckTrimValues();
    MemoryForTransmtter = SDCardAddress;
    if ((ModelNumber < 1) || (ModelNumber > 99)) ModelNumber = 1;
    ReadOneModel(ModelNumber);
    return true;
}

/*********************************************************************************************************************************/
void Force_ReDisplay()
{
    for (int i = 0; i < CHANNELSUSED; ++i) ShownBuffer[i] = 242; // to force a re-show of servo positions
}

/*********************************************************************************************************************************/
void SendColour(char* but, int Colour)
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
    if (CurrentView == FRONTVIEW) {
        char bco[]  = "bt0.bco=";
        char bco2[] = "bt0.bco2=";
        char pco[]  = "bt0.pco=";
        char pco2[] = "bt0.pco2=";
        SendColour(bco, SpecialColour);
        SendColour(bco2, SpecialColour);
        SendColour(pco, HighlightColour);
        SendColour(pco2, HighlightColour);
    }
    if (UseLog) LogSafety(1);
}
/*********************************************************************************************************************************/
void ShowSafetyIsOff()
{
    if (CurrentView == FRONTVIEW) {
        char bco[]  = "bt0.bco=";
        char bco2[] = "bt0.bco2=";
        char pco[]  = "bt0.pco=";
        char pco2[] = "bt0.pco2=";
        SendColour(bco, BackGroundColour);
        SendColour(bco2, BackGroundColour);
        SendColour(pco, HighlightColour);
        SendColour(pco2, HighlightColour);
    }
    if (UseLog) LogSafety(0);
}
/*********************************************************************************************************************************/

void WatchDogCallBack()
{
    // Serial.println("RESETTING ...");
}

/*********************************************************************************************************************************/

FLASHMEM void SetDS1307ToCompilerTime()
{
    if (getDate(__DATE__) && getTime(__TIME__)) {
        RTC.write(tm);
    } // only useful when connected to compiler
}

/************************************************************************************************************/

FLASHMEM void GetTXVersionNumber()
{
    char    nbuf[5];
    uint8_t Txv1 = TXVERSION_MAJOR;
    uint8_t Txv2 = TXVERSION_MINOR;
    uint8_t Txv3 = TXVERSION_MINIMUS;
    Str(TransmitterVersionNumber, Txv1, 2);
    Str(nbuf, Txv2, 2);
    strcat(TransmitterVersionNumber, nbuf);
    Str(nbuf, Txv3, 0);
    strcat(TransmitterVersionNumber, nbuf);
}
/************************************************************************************************************/
FASTRUN void SetUKFrequencies()
{
    FHSS_data::FHSSChPointer       = FHSS_data::FHSS_Channels;
    FHSS_data::UkRules             = true;
    FHSS_data::FHSSRecoveryPointer = FHSS_data::FHSS_Channels;
}
/************************************************************************************************************/
FASTRUN void SetTestFrequencies()
{
    FHSS_data::FHSSChPointer       = FHSS_data::FHSS_Channels1;
    FHSS_data::UkRules             = false;
    FHSS_data::FHSSRecoveryPointer = FHSS_data::FHSS_Channels1;
}
/************************************************************************************************************/
FASTRUN void CreateTimeStamp(char* DateAndTime)
{
    char NB[10];
    char zero[]  = "0";
    char Colon[] = "."; // a dot!
    char Slash[] = "/";
    char null[]  = "";
    char space[] = " ";

    if (RTC.read(tm)) {
        strcpy(DateAndTime, null);

        if (MayBeAddZero(tm.Day)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Day, 0));
        strcat(DateAndTime, Slash);
        if (MayBeAddZero(tm.Month)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Month, 0));
        strcat(DateAndTime, Slash);
        strcat(DateAndTime, (Str(NB, tmYearToCalendar(tm.Year), 0)));
        strcat(DateAndTime, space);

        if (MayBeAddZero(tm.Hour)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Hour, 0));
        strcat(DateAndTime, Colon);
        if (MayBeAddZero(tm.Minute)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Minute, 0));
        strcat(DateAndTime, Colon);
        if (MayBeAddZero(tm.Second)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Second, 0));
    }
}

/************************************************************************************************************/
FASTRUN void CreateTimeDateStamp(char* DateAndTime)
{
    char NB[10];
    char zero[]  = "0";
    char Dash[]  = "-";
    char Colon[] = "."; // a dot!
    char Space[] = " ";

    if (RTC.read(tm)) {
        if (MayBeAddZero(tm.Day)) strcat(DateAndTime, zero);
        strcpy(DateAndTime, Str(NB, tm.Day, 0));
        strcat(DateAndTime, Dash);
        if (MayBeAddZero(tm.Month)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Month, 0));
        strcat(DateAndTime, Dash);
        strcat(DateAndTime, (Str(NB, tmYearToCalendar(tm.Year), 0)));
        strcat(DateAndTime, Space);
        if (MayBeAddZero(tm.Hour)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Hour, 0));
        strcat(DateAndTime, Colon);
        if (MayBeAddZero(tm.Minute)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Minute, 0));
        strcat(DateAndTime, Colon);
        if (MayBeAddZero(tm.Second)) strcat(DateAndTime, zero);
        strcat(DateAndTime, Str(NB, tm.Second, 0));
    }
}
/************************************************************************************************************/
FASTRUN void MakeLogFileName(char* LogFileName)
{
    char NB[10];
    char Ext[]  = ".LOG";
    char dash[] = "-";
    char zero[] = "0";
    if (RTC.read(tm)) {

        if (MayBeAddZero(tm.Day)) strcat(LogFileName, zero);
        strcpy(LogFileName, Str(NB, tm.Day, 0));
        strcat(LogFileName, dash);
        if (MayBeAddZero(tm.Month)) strcat(LogFileName, zero);
        strcat(LogFileName, Str(NB, tm.Month, 0));
        strcat(LogFileName, Ext);
    }
}
/************************************************************************************************************/
FASTRUN void OpenLogFileW(char* LogFileName)
{
    if (!LogFileOpen) {
        LogFileNumber = SD.open(LogFileName, FILE_WRITE);
        LogFileOpen   = true;
    }
}
// ************************************************************************
FASTRUN void CheckLogFileIsOpen()
{
    char LogFileName[20];
    if (!LogFileOpen) {
        MakeLogFileName(LogFileName); // Create a "today" filename
        OpenLogFileW(LogFileName);    // Open file for writing
    }
}

/************************************************************************************************************/
FASTRUN void DeleteLogFile(char* LogFileName)
{
    SD.remove(LogFileName);
}
/************************************************************************************************************/
FASTRUN void DeleteLogFile1()
{
    char LogFileName[20];
    char LogTeXt[]   = "LogText";
    char BlankText[] = " ";
    CloseLogFile();
    MakeLogFileName(LogFileName);
    DeleteLogFile(LogFileName);
    SendText1(LogTeXt, BlankText);
    if (!UseLog) return;
    if (LedWasGreen) {
        RecentStartLine = 0;
        ShowLogFile(RecentStartLine);
    }
}

/************************************************************************************************************/
FASTRUN void OpenLogFileR(char* LogFileName)
{
    if (!LogFileOpen) {
        LogFileNumber = SD.open(LogFileName, FILE_READ);
        LogFileOpen   = true;
    }
}
/************************************************************************************************************/
FASTRUN void CloseLogFile()
{
    if (LogFileOpen) LogFileNumber.close();
    LogFileOpen = false;
}
/************************************************************************************************************/
FASTRUN void WriteToLogFile(char* SomeData, uint16_t len)
{
    LogFileNumber.write(SomeData, len);
}
// ************************************************************************
FASTRUN void LogFilePreamble()
{
    char dbuf[22];
    char Divider[] = " - ";
    CheckLogFileIsOpen();
    CreateTimeStamp(dbuf);    // Put time stamp into buffer
    WriteToLogFile(dbuf, 20); // Add time stamp
    WriteToLogFile(Divider, sizeof(Divider));
}

// ************************************************************************
FASTRUN void LogText(char* TheText, uint16_t len)
{
    char crlf[] = {'|', 13, 10, 0};
    LogFilePreamble();
    WriteToLogFile(TheText, len);
    WriteToLogFile(crlf, sizeof(crlf));
    CloseLogFile();
}
// ************************************************************************
FASTRUN void LogMinGap()
{
    char TheText[] = "Min logged gap: ";
    char buf[25]   = " ";
    char NB[]      = "    ";

    Str(NB, MinimumGap, 0);
    strcpy(buf, TheText);
    strcat(buf, NB);
    LogText(buf, sizeof(buf));
}
// ************************************************************************
FASTRUN void LogConnection()
{
    char TheText[] = "Connected to ";
    char buf[40]   = " ";
    strcpy(buf, TheText);
    strcat(buf, ModelName);
    LogText(buf, sizeof(buf));
    LogMinGap();
}
// ************************************************************************

uint32_t GetOverallSuccessRate()
{
    return (TotalGoodPackets * 100) / (TotalGoodPackets + TotalLostPackets);
}
// ************************************************************************
FASTRUN void LogDisConnection()
{
    char buf[40]   = " ";
    char TheText[] = "Disconnected from ";

    strcpy(buf, TheText);
    strcat(buf, ModelName);
    LogText(buf, sizeof(buf));

    LogLongestGap();
    LogTotalLostPackets();
    LogTotalGoodPackets();
    LogOverallSuccessRate();
}
// ************************************************************************
FASTRUN void LogNewBank()
{
    char Ltext[] = "Bank: ";
    char NB[5];
    char thetext[20];
    Str(NB, Bank, 0);
    strcpy(thetext, Ltext);
    strcat(thetext, NB);
    LogText(thetext, 7);
}

// ************************************************************************
FASTRUN void LogMotor(bool On)
{
    char Ltext1[] = "Motor On";
    char Ltext0[] = "Motor Off";
    char thetext[20];
    if (On)
        strcpy(thetext, Ltext1);
    else
        strcpy(thetext, Ltext0);
    LogText(thetext, 9);
}

// ************************************************************************
FASTRUN void LogSafety(bool On)
{
    char Ltext1[] = "Safety On";
    char Ltext0[] = "Safety Off";
    char thetext[20];
    if (On)
        strcpy(thetext, Ltext1);
    else
        strcpy(thetext, Ltext0);
    LogText(thetext, 10);
}
// ************************************************************************

FASTRUN void LogThisRX()
{
    char Ltext[] = "RX: ";
    char thetext[10];
    strcpy(thetext, Ltext);
    strcat(thetext, ThisRadio);
    LogText(thetext, 5);
}

// ************************************************************************
FASTRUN void LogLowBattery()
{ // Not yet implemented
    char TheText[] = "Low battery";
    LogText(TheText, strlen(TheText));
}
// ************************************************************************

FASTRUN void LogThisGap()
{
    char Ltext[] = "Gap: ";
    char NB[5];
    char thetext[10];
    if (ThisGap > 1000) return;
    Str(NB, ThisGap, 0);
    strcpy(thetext, Ltext);
    strcat(thetext, NB);
    LogText(thetext, 8);
}
// ************************************************************************

FASTRUN void LogLongestGap()
{
    char thetext[50];
    snprintf(thetext, 45, "Longest gap: %d", (short int)GapLongest);
    LogText(thetext, strlen(thetext));
}

// ************************************************************************

void LogTotalLostPackets()
{
    char thetext[50];
    snprintf(thetext, 45, "Total lost packets: %d", (short int)TotalLostPackets);
    LogText(thetext, strlen(thetext));
}

// ************************************************************************

void LogTotalGoodPackets()
{
    char thetext[50];
    snprintf(thetext, 45, "Total good packets: %d", (uint16_t)TotalGoodPackets);
    LogText(thetext, strlen(thetext));
}

// ************************************************************************

void LogOverallSuccessRate()
{
    char     thetext[50];
    uint32_t OverallSuccessRate = 0;
    OverallSuccessRate          = GetOverallSuccessRate();
    snprintf(thetext, 45, "Overall success rate: %d%%", (uint8_t)OverallSuccessRate);
    LogText(thetext, strlen(thetext));
}
// ************************************************************************

FASTRUN void
LogPowerOn()
{
    char Ltext[] = "Power ON";
    char sp[]    = "*******************************************";
    LogText(sp, strlen(sp));
    LogText(Ltext, strlen(Ltext));
}

// ************************************************************************

FASTRUN void LogPowerOff()
{
    char Ltext[] = "Power OFF";
    LogText(Ltext, strlen(Ltext));
}

// ************************************************************************

FASTRUN void LogThisModel()
{
    char Ltext[] = "Model loaded: ";
    char thetext[75];
    strcpy(thetext, Ltext);
    strcat(thetext, ModelName);
    LogText(thetext, strlen(Ltext) + strlen(ModelName));
}
// ************************************************************************

void ShowLogFile(uint8_t StartLine)
{
    char TheText[MAXFILELEN + 10]; // MAX = 5K or so
    char LogFileName[20];
    char LogTeXt[] = "LogText";
    CloseLogFile();
    MakeLogFileName(LogFileName);                            // Create "today" filename
    ReadTextFile(LogFileName, TheText, StartLine, MAXLINES); // Then load text
    SendText1(LogTeXt, TheText);                             // Then send it
}

/*********************************************************************************************************************************/
// This function gets the unique MAC address of the Teensy 4.1
// And also fixes it so that it's a suitable Pipe address for the nRF24L01

void GetTeensyMacAddress()
{

    teensyMAC(MacAddress); // Get MAC address

    for (int i = 1; i < 6; ++i) {
        MacAddress[i] = CheckPipeNibbles(MacAddress[i]); // Fix PIPE if needed !
    }
    //  for (int i = 1; i < 6; ++i) MacAddress[i] = 0x42; // TEST! (... Force new ID for tests)

#ifdef DB_BIND
    Serial.println("");
    Serial.println("Local (transmitter) ID sent: ");
    for (int q = 1; q < 6; ++q) {
        Serial.print(MacAddress[q], HEX);
        Serial.print(" ");
    }
    Serial.println("");
    Serial.println("(5 bytes of 6 sent, probably with some modified nibbles or bytes)");
#endif
    TeensyMACAddPipe = (uint64_t)MacAddress[0] << 40;
    TeensyMACAddPipe += (uint64_t)MacAddress[1] << 32;
    TeensyMACAddPipe += (uint64_t)MacAddress[2] << 24;
    TeensyMACAddPipe += (uint64_t)MacAddress[3] << 16;
    TeensyMACAddPipe += (uint64_t)MacAddress[4] << 8;
    TeensyMACAddPipe += (uint64_t)MacAddress[5];
    TeensyMACAddPipe &= 0xffffffffff;
}

/*********************************************************************************************************************************/
// SETUP
/*********************************************************************************************************************************/
FLASHMEM void setup()
{
    char FrontView_BackGround[] = "FrontView.BackGround";
    char FrontView_ForeGround[] = "FrontView.ForeGround";
    char FrontView_Special[]    = "FrontView.Special";
    char FrontView_Highlight[]  = "FrontView.Highlight";
    char err_chksm[]            = "File checksum?";
    char err_404[]              = "File not found";
    char FrontView_Connected[]  = "Connected";
    char err_MotorOn[]          = " MOTOR IS ON! ";
    char FrontView_Hours[]      = "Hours";
    char FrontView_Mins[]       = "Mins";
    char FrontView_Secs[]       = "Secs";
    char WarnNow[]              = "vis Warning,1";
    char Warning[]              = "Warning";
    char ModelsFile[]           = "models.dat";

    pinMode(REDLED, OUTPUT);
    pinMode(GREENLED, OUTPUT);
    pinMode(BLUELED, OUTPUT);
    pinMode(POWER_OFF_PIN, OUTPUT);
    BlueLedOn();
    NEXTION.begin(921600); // BAUD rate also set in display code THIS IS THE MAX (was 115200)
    InitMaxMin();
    InitCentreDegrees();
    ResetSubTrims();
    CentreTrims();
    WatchDogConfig.window   = WATCHDOGMAXRATE; //  = MINIMUM RATE in milli seconds, (32ms to 522.232s) must be MUCH smaller than timeout
    WatchDogConfig.timeout  = WATCHDOGTIMEOUT; //  = MAX TIMEOUT in milli seconds, (32ms to 522.232s)
    WatchDogConfig.callback = WatchDogCallBack;
    TeensyWatchDog.begin(WatchDogConfig);
    delay(300); // <<********************* MUST ALLOW DOG TO INITIALISE
    DelayWithDog(WARMUPDELAY);
    if (!SD.begin(BUILTIN_SDCARD)) { // MUST return true or all is lost!
        DelayWithDog(WARMUPDELAY);
        SD.begin(BUILTIN_SDCARD); // a second attempt for iffy sd cards ?!
    }
    ErrorState = NOERROR;
    if (CheckFileExists(ModelsFile)) {
        if (!LoadAllParameters()) { // if file not good ..
            ErrorState = CHECKSUMERROR;
        }
    }
    else {
        ErrorState = MODELSFILENOTFOUND; // if no file ... or no SD
    }

    GetTeensyMacAddress();

    Wire.begin();
    ScanI2c();
    if (USE_INA219) ina219.begin();
    InitSwitchesAndTrims();

#ifdef TXMODULESUPPORT
    if (PPMdata.UseTXModule)
    {
        PPMdata.PPMOutputModule.begin(PPMPORT);
        SelectChannelOrder();
    }
    else
    {
        InitRadio(DefaultPipe);
    }

#else
    InitRadio(DefaultPipe);
#endif

    if (BuddyMaster) {
        PPMdata.PPMInputBuddy.begin(BUDDYPPMPORT);
    }
    else {
        if (!PPMdata.UseTXModule && BuddyPupilOnPPM) {
            PPMdata.PPMOutputBuddy.begin(BUDDYPPMPORT);
        }
    }

    DelayWithDog(WARMUPDELAY);                         // Allow Nextion time to warm up
    SendValue(FrontView_BackGround, BackGroundColour); // Get colours ready
    SendValue(FrontView_ForeGround, ForeGroundColour);
    SendValue(FrontView_Special, SpecialColour);
    SendValue(FrontView_Highlight, HighlightColour);

    CurrentView = 254;
    GotoFrontView();
    SetAudioVolume(AudioVolume);
    if (PlayFanfare) {
        PlaySound(THEFANFARE);
        DelayWithDog(4000); // Fanfare takes about 4 seconds
    }
    SendValue(FrontView_Hours, 0);
    SendValue(FrontView_Mins, 0);
    SendValue(FrontView_Secs, 0);
    //  ***************************************************************************************
    // SetDS1307ToCompilerTime();    //  **   Uncomment this line to set DS1307 clock to compiler's (Computer's) time.        **
    // **   BUT then re-comment it!! Otherwise it will reset to same time on every boot up! **
    //  ***************************************************************************************
    BoundFlag = false;
    StartInactvityTimeout();
    SizeOfCompressedData = sizeof(CompressedData);
    GetTXVersionNumber();

    SetUKFrequencies();
    ScreenTimeTimer = millis();
    RestoreBrightness();
    if (UseLog) {
        LogPowerOn();
        LogThisModel();
    }
    SendText(FrontView_Connected, na);
    UpdateModelsNameEveryWhere();
    ConfigureStickMode();
    WarningTimer = millis();
    CheckMotorOff();
    if (MotorEnabled) {
        ErrorState = MOTORISON;
        SendNoData = true;
    }
    if (!UseMotorKill) ShowMotor(1);
    if (SafetyON)  ShowSafetyIsOn();

    if (ErrorState) {
        SendCommand(WarnNow);
        if (ErrorState == CHECKSUMERROR) {
            SendText(Warning, err_chksm);
        }
        if (ErrorState == MODELSFILENOTFOUND) {
            SendText(Warning, err_404);
        }
        if (ErrorState == MOTORISON) {
            SendText(Warning, err_MotorOn);
            PlaySound(MOTORON);
            DelayWithDog(1200);
            PlaySound(PLSTURNOFF);
        }
    }
}
/*********************************************************************************************************************************/

void GetStatistics()
{
    if (RangeTestGoodPackets) PacketsPerSecond = RangeTestGoodPackets;
    RangeTestGoodPackets = 0;
}

/*********************************************************************************************************************************/
/** @returns position of text1 within text2 or 0 if not found */
int InStrng(char* text1, char* text2)
{
    for (uint16_t j = 0; j < strlen(text2); ++j) {
        bool flag = false;
        for (uint16_t i = 0; i < strlen(text1); ++i) {
            if (text1[i] != text2[i + j]) {
                flag = true;
                break;
            }
        }
        if (!flag) return j + 1; // Found match
    }
    return 0; // Found no match
}

/*********************************************************************************************************************************/
void SaveCheckSum32()
{ // uses 5 bytes. Last one is indicator of use.

    DoingCheckSm = true;
    SDUpdate32BITS(SDCardAddress, FileCheckSum);
    SDCardAddress += 4;
    SDUpdate8BITS(SDCardAddress, 0xFF); // indicator
    SDCardAddress++;
    DoingCheckSm = false;

#ifdef DB_CHECKSUM
    Serial.print("Writing: ");
    Serial.println(FileCheckSum);
#endif
}

/*********************************************************************************************************************************/
void ReadCheckSum32()
{ // uses 5 bytes. Last one is indicator of use.
  // This function sets ErrorState to a non zero value if there'a a file error

    bool UseCheckSm = false;
    DoingCheckSm    = true;
    uint32_t ch;
    ch = SDRead32BITS(SDCardAddress);
    SDCardAddress += 4;
    if (SDRead8BITS(SDCardAddress) == 0xFF) UseCheckSm = true; // if this byte isn't 0xFF then checksum was never written here.
    ++SDCardAddress;
    DoingCheckSm = false;
    if (UseCheckSm) {
        if (ch != FileCheckSum) {
            ErrorState = CHECKSUMERROR;
        }
#ifdef DB_CHECKSUM
        Serial.print("Read from file: ");
        Serial.println(ch);
        if (ch == FileCheckSum) {
            Serial.println("FILE CHECKSUM IS GOOD :-) ! ");
        }
        else {
            Serial.print("FILE CHECKSUM WAS ");
            Serial.println(FileCheckSum);
        }
#endif
    }
}

/*********************************************************************************************************************************/

void SaveTransmitterParameters()
{
    bool EON = false;
    int  j   = 0;
    int  i   = 0;
    if (!ModelsFileOpen) OpenModelsFile();

    SDCardAddress = 0;
    FileCheckSum  = 0;

    SDUpdate16BITS(SDCardAddress, 12345); // marker that file exists!

    SDCardAddress += 2;
    for (i = 0; i < CHANNELSUSED; ++i) {
        SDUpdate16BITS(SDCardAddress, ChannelMin[i]); // Stick min output of pot
        SDCardAddress += 2;
        SDUpdate16BITS(SDCardAddress, ChannelMidLow[i]); //
        SDCardAddress += 2;
        SDUpdate16BITS(SDCardAddress, ChannelCentre[i]); // Stick Centre output of pot
        SDCardAddress += 2;
        SDUpdate16BITS(SDCardAddress, ChannelMidHi[i]); //
        SDCardAddress += 2;
        SDUpdate16BITS(SDCardAddress, ChannelMax[i]); // Stick max output of pot
        SDCardAddress += 2;
    }
    SDUpdate8BITS(SDCardAddress, BuddyPupilOnPPM);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, BuddyMaster);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, ModelNumber);
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, ScreenTimeout);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, (Inactivity_Timeout / TICKSPERMINUTE));
    ++SDCardAddress;
    for (j = 0; j < 30; ++j) {
        if (EON) TxName[j] = 0;
        SDUpdate8BITS(SDCardAddress, TxName[j]);
        if (TxName[j] == 0) EON = true;
        ++SDCardAddress;
    }
    SDUpdate16BITS(SDCardAddress, Qnh);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, DeltaGMT);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, BackGroundColour);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, ForeGroundColour);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, SpecialColour);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, HighlightColour);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SticksMode);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, AudioVolume);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Brightness);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, PlayFanfare);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, TrimClicks);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, ButtonClicks);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SpeakingClock);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, AnnounceBanks);
    ++SDCardAddress;
    for (i = 0; i < 8; ++i) {
        SDUpdate8BITS(SDCardAddress, SwitchNumber[i]);
        ++SDCardAddress;
    }
    // spare
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, MinimumGap);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, LogRXSwaps);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, UseLog);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, AnnounceConnected);
    ++SDCardAddress;
    for (j = 0; j < 8; ++j) {
        SDUpdate8BITS(SDCardAddress, TrimNumber[j]);
        ++SDCardAddress;
    }
    SDUpdate16BITS(SDCardAddress, TxVoltageCorrection);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, PowerOffWarningSeconds);
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, LEDBrightness);
    ++SDCardAddress;
    // one spare byte here!
    //
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, ConnectionAssessSeconds);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, AutoModelSelect);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, TXLiPo);
    ++SDCardAddress;

    SDUpdate8BITS(SDCardAddress, PPMdata.PPMOrderSelection);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, PPMdata.PPMChannelsNumber);
    ++SDCardAddress;
    // Not used!!! (yet) // 1 spare
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, PPMdata.UseTXModule);
    ++SDCardAddress;

    SaveCheckSum32(); // Save the Transmitter parametres checksm
    CloseModelsFile();
}

/*********************************************************************************************************************************/

/** MODEL Specific */
void SaveOneModel(uint32_t mnum)
{
    uint32_t j;
    uint32_t i;
    bool     EndOfName = false;
    FileCheckSum       = 0;
    if ((mnum < 1) || (mnum > MAXMODELNUMBER)) return; // There is no model zero!
    if (!ModelsFileOpen) OpenModelsFile();
    SDCardAddress = TXSIZE;                  //  spare bytes for TX stuff
    SDCardAddress += (mnum - 1) * MODELSIZE; //  spare bytes for Model params
    if (SingleModelFlag) SDCardAddress = 0;
    StartLocation = SDCardAddress;
    ModelDefined  = 42;
    SDUpdate8BITS(SDCardAddress, ModelDefined);
    ++SDCardAddress;
    for (j = 0; j < 30; ++j) {
        if (EndOfName) ModelName[j] = 0;
        SDUpdate8BITS(SDCardAddress, ModelName[j]);
        if (ModelName[j] == 0) EndOfName = true;
        ++SDCardAddress;
    }
    for (i = 0; i < CHANNELSUSED; ++i) {
        for (j = 1; j <= 4; ++j) {
            SDUpdate8BITS(SDCardAddress, MaxDegrees[j][i]); // Max requested in degrees (180)
            ++SDCardAddress;
            SDUpdate8BITS(SDCardAddress, MidHiDegrees[j][i]); // MidHi requested in degrees (135)
            ++SDCardAddress;
            SDUpdate8BITS(SDCardAddress, CentreDegrees[j][i]); // Centre requested in degrees (90)
            ++SDCardAddress;
            SDUpdate8BITS(SDCardAddress, MidLowDegrees[j][i]); // MidLo requested in degrees (45)
            ++SDCardAddress;
            SDUpdate8BITS(SDCardAddress, MinDegrees[j][i]); // Min requested in degrees (0)
            ++SDCardAddress;
        }
    }
    for (j = 0; j < MAXMIXES; ++j) {
        for (i = 0; i < 17; ++i) {
            SDUpdate8BITS(SDCardAddress, Mixes[j][i]); // Save mixes
            ++SDCardAddress;
        }
    }
    for (j = 0; j < BANKSUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            SDUpdate8BITS(SDCardAddress, Trims[j][i]);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < BANKSUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            // SPARE!!!
            ++SDCardAddress;
        }
    }
    SDUpdate8BITS(SDCardAddress, RXCellCount);
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, TrimMultiplier);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, LowBattery);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, CopyTrimsToAll);
    ++SDCardAddress;
    for (i = 0; i < CHANNELSUSED; ++i) {
        SDUpdate8BITS(SDCardAddress, SubTrims[i]);
        ++SDCardAddress;
    }
    SDUpdate16BITS(SDCardAddress, ReversedChannelBITS);
    ++SDCardAddress;
    ++SDCardAddress;
    for (i = 0; i < 4; ++i) {
        SDUpdate8BITS(SDCardAddress, InputTrim[i]);
        ++SDCardAddress;
    }
    SDUpdate16BITS(SDCardAddress, RxVoltageCorrection);
    ++SDCardAddress;
    ++SDCardAddress;

    SDUpdate16BITS(SDCardAddress, BuddyControlled);
    ++SDCardAddress;
    ++SDCardAddress;

    SDUpdate8BITS(SDCardAddress, SticksMode); // save sticks mode in case of rf transfer

    SDCardAddress += 1; // *********************** 0 spare here remaining  **********************

    for (i = 0; i < CHANNELSUSED; ++i) {
        SDUpdate8BITS(SDCardAddress, InPutStick[i]);
        ++SDCardAddress;
    }
    SDUpdate8BITS(SDCardAddress, FMSwitch);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Autoswitch);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Channel9Switch);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Channel10Switch);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Channel11Switch);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Channel12Switch);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SWITCH1Reversed);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SWITCH2Reversed);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SWITCH3Reversed);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SWITCH4Reversed);
    ++SDCardAddress;
    for (i = 0; i < CHANNELSUSED; ++i) {
        SDUpdate8BITS(SDCardAddress, FailSafeChannel[i]);
        ++SDCardAddress;
    }
    for (i = 0; i < CHANNELSUSED; ++i) {
        for (j = 0; j < 10; ++j) {
            SDUpdate8BITS(SDCardAddress, ChannelNames[i][j]);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < BANKSUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            SDUpdate8BITS(SDCardAddress, Exponential[j][i]);
            ++SDCardAddress;
        }
    }

    for (j = 0; j < BANKSUSED + 1; ++j) {
        for (i = 0; i < CHANNELSUSED + 1; ++i) {
            SDUpdate8BITS(SDCardAddress, InterpolationTypes[j][i]);
            ++SDCardAddress;
        }
    }

    for (j = 0; j < BYTESPERMACRO; ++j) {
        for (i = 0; i < MAXMACROS; ++i) {
            SDUpdate8BITS(SDCardAddress, MacrosBuffer[i][j]);
            ++SDCardAddress;
        }
    }
    for (i = 0; i < 8; ++i) {
        SDUpdate8BITS(SDCardAddress, ModelsMacUnionSaved.Val8[i]);
        ++SDCardAddress;
    }

    SDUpdate8BITS(SDCardAddress, UseMotorKill);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, MotorChannelZero);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, MotorChannel);
    if (MotorChannel > 15) MotorChannel = 15;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SafetySwitch);
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, SFV);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Drate2);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Drate3);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Drate1);
    ++SDCardAddress;

    for (int i = 0; i < 8; ++i) {
        SDUpdate8BITS(SDCardAddress, DualRateChannels[i]);
        ++SDCardAddress;
    }
    SDUpdate8BITS(SDCardAddress, BuddySwitch);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, DualRatesSwitch);
    ++SDCardAddress;

    for (i = 0; i < 4; ++i) {
        SDUpdate8BITS(SDCardAddress, BanksInUse[i]);
        ++SDCardAddress;
    }
    for (i = 0; i < 16; ++i) {
        SDUpdate8BITS(SDCardAddress, StepSize[i]);
        ++SDCardAddress;
    }

    SDUpdate8BITS(SDCardAddress, TimerDownwards);
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, TimerStartTime);

    ++SDCardAddress;
    ++SDCardAddress;

    SDUpdate8BITS(SDCardAddress, PPMdata.UseSBUSFromRX);
    ++SDCardAddress;

    SDUpdate16BITS(SDCardAddress, PPMdata.PPMChannelCount);
    ++SDCardAddress;
    ++SDCardAddress;

    for (i = 0; i < 16; ++i) {
        SDUpdate8BITS(SDCardAddress, ChannelOutPut[i]);
        ++SDCardAddress;
    }

    SaveCheckSum32(); // Save the Model parametres checksm

    // ********************** Add more

    OneModelMemory = SDCardAddress - StartLocation;
#ifdef DB_SD
    Serial.print("Saved model: ");
    Serial.println(ModelName);
    Serial.println(" ");
    Serial.print(OneModelMemory);
    Serial.println(" bytes used per model.");
    Serial.print(MODELSIZE - OneModelMemory);
    Serial.println(" spare bytes per model.");
    Serial.print(MODELSIZE);
    Serial.println(" bytes reserved per model.)");
    Serial.print("Write Model ");
    Serial.print(ModelNumber);
    Serial.print(" Checksum: ");
    Serial.println(FileCheckSum);
    Serial.println(" ");
#endif // defined DB_SD

#ifdef DB_CHECKSUM
    Serial.print("Write Model: ");
    Serial.print(ModelName);
    Serial.print(" Checksum: ");
    Serial.println(FileCheckSum);
    Serial.println(" ");
#endif

    CloseModelsFile();
}
/*********************************************************************************************************************************/

/** Bubble sort */
void SortDirectory()
{
    int  f      = 0;
    bool flag   = true;
    int  Scount = 0;
    char TempArray[18];
    while (flag && Scount < 10000) {
        flag = false;
        for (f = 0; f < ExportedFileCounter - 1; ++f) {
            if (strcmp(TheFilesList[f], TheFilesList[f + 1]) > 0) {
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
    char MOD[] = ".MOD";
    char Entry1[30];
    char fn[20];
    int  i              = 0;
    File dir            = SD.open("/");
    ExportedFileCounter = 0;
    while (true) {
        File entry = dir.openNextFile();
        if (!entry || ExportedFileCounter > MAXBACKUPFILES) break;
        strcpy(Entry1, entry.name());
        if (InStrng(MOD, Entry1) > 0) {
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
uint16_t WordWrap(char* htext)
{
    char     crlf[]            = {13, 10, 0};
    char     temp1[MAXFILELEN] = "";
    char     a[]               = " ";
    uint16_t len               = 0;
    uint16_t i, j;

    for (i = strlen(htext) - 1; i > 1; --i) {
        if ((htext[i] == ' ') || (htext[i] == '-')) break; // 'i' now has last space pointer
    }
    for (j = 0; j < i; ++j) { // get text to i ...
        a[0] = htext[j];
        strcat(temp1, a);
    }
    strcat(temp1, crlf);                      // add crlf ...
    for (j = i + 1; j < strlen(htext); ++j) { // then last word on next line by-passing the space
        a[0] = htext[j];
        strcat(temp1, a);
        ++len; // length onto next line
    }
    strcpy(htext, temp1);
    return len;
}
/*********************************************************************************************************************************/
// This function now scrolls by loading only part of the text file
// This allows very long files to be handled ok.

void ReadTextFile(char* fname, char* htext, uint8_t StartLineNumber, uint8_t MaxLines)
{
#define MAXWIDTH 68
    char     errormsg[]     = "File not found! -> ";
    uint16_t LineCounter    = 0;
    uint16_t StopLineNumber = 0;
    uint16_t i              = 0;
    uint8_t  Column         = 0;
    File     fnumber;

    char crlf[]         = {13, 10, 0};
    char a[]            = " ";
    char dots[]         = "(Hit 'Down' for more ...) ";
    char dots1[]        = "(Hit 'Up' to scroll up ...) ";
    char slash[]        = "/";
    char OpenBracket[]  = "( ";
    char CloseBracket[] = " )";
    char SearchFile[40];

    StopLineNumber = StartLineNumber + MaxLines;
    strcpy(SearchFile, slash);
    strcat(SearchFile, fname);
    strcpy(htext, OpenBracket);
    strcat(htext, SearchFile);
    strcat(htext, CloseBracket);
    strcat(htext, crlf);
    strcat(htext, crlf);
    if (StartLineNumber > 1) {
        strcat(htext, crlf);
        strcat(htext, dots1);
        strcat(htext, crlf);
    }

    ThereIsMoreToSee = false;
    fnumber          = SD.open(SearchFile, FILE_READ);
    if (fnumber) {
        while (fnumber.available() && i < (MAXFILELEN - 10)) {
            a[0] = fnumber.read(); //  Read in one byte at a time.
            if (a[0] == '|') {     //  New Line character = '|'
                if ((LineCounter >= StartLineNumber) && (LineCounter <= StopLineNumber)) {
                    strcat(htext, crlf);
                    ++i;
                    ++i;
                }
                Column = 0;
                ++LineCounter;
                a[0] = 34; // a '34'  ( " ) is ignored.
            }
            if (Column >= MAXWIDTH) {
                Column = WordWrap(htext);
                ++LineCounter;
            }
            if ((a[0] == 13) || (a[0] == 10)) a[0] = 34; // Ignore CrLfs
            if (a[0] != 34) {
                if ((LineCounter >= StartLineNumber) && (LineCounter <= StopLineNumber)) {
                    strcat(htext, a);
                    ++Column;
                    ++i;
                }
            }
            if (LineCounter > StopLineNumber) {
                strcat(htext, crlf);
                strcat(htext, dots);
                ThereIsMoreToSee = true;
                break;
            }
        }
    }
    else
    {
        strcpy(htext, errormsg);
        strcat(htext, fname);
    }
    fnumber.close();
}

/*********************************************************************************************************************************/
void ScrollHelpFile()
{                                   // redisplays help file to scroll it ... maybe from top of file
    char HelpText[MAXFILELEN + 10]; // MAX = 3K or so
    char HelpView[] = "HelpText";
    ReadTextFile(RecentTextFile, HelpText, RecentStartLine, MAXLINES); // Then load help text
    SendText1(HelpView, HelpText);                                     // Then send it
}
/*********************************************************************************************************************************/
void SendHelp()
{ // load new help file
    char hcmd[] = "page HelpView";
    char HelpFile[20];
    int  i = 9;
    int  j = 0;
    SendCommand(hcmd); // first load the right screen
    while (TextIn[i] != 0 && j < 19) {
        HelpFile[j] = TextIn[i];
        ++i;
        ++j;
        HelpFile[j] = 0;
    }
    strcpy(RecentTextFile, HelpFile);
    RecentStartLine = 0;
    ScrollHelpFile();
}
/*********************************************************************************************************************************/
/** @brief Discover which channel to setup */
int GetChannel()
{
    unsigned int i;
    for (i = 0; i < sizeof(TextIn); ++i) {
        if (isdigit(TextIn[i])) break;
    }
    return atoi(&TextIn[i]);
}

/*********************************************************************************************************************************/

void DoOneSwitch(char* Sw, uint8_t n)
{

    char NotUsed[]          = "Not used";
    char Banks123[]         = "Banks 1 2 3";
    char Auto[]             = "Bank 4 & Motor";
    char Safety_Switch[]    = "Safety    ";
    char Buddy_Switch[]     = "Buddy     ";
    char DualRates_Switch[] = "Rates     ";
    char c9[40];
    char c10[40];
    char c11[40];
    char c12[40];
    char cc9[]  = " (Ch 9)";
    char cc10[] = " (Ch 10)";
    char cc11[] = " (Ch 11)";
    char cc12[] = " (Ch 12)";

    strcpy(c9, ChannelNames[8]);
    strcpy(c10, ChannelNames[9]);
    strcpy(c11, ChannelNames[10]);
    strcpy(c12, ChannelNames[11]);
    strcat(c9, cc9);
    strcat(c10, cc10);
    strcat(c11, cc11);
    strcat(c12, cc12);

    SendText(Sw, NotUsed);
    if (Autoswitch == n) SendText(Sw, Auto);
    if (FMSwitch == n) SendText(Sw, Banks123);
    if (Channel9Switch == n) SendText(Sw, c9);
    if (Channel10Switch == n) SendText(Sw, c10);
    if (Channel11Switch == n) SendText(Sw, c11);
    if (Channel12Switch == n) SendText(Sw, c12);
    if (SafetySwitch == n) SendText(Sw, Safety_Switch);
    if (DualRatesSwitch == n) SendText(Sw, DualRates_Switch);
    if (BuddySwitch == n) SendText(Sw, Buddy_Switch);
}

/*********************************************************************************************************************************/

void UpdateSwitchesView()
{ // now optimised!
    char sw[4][4] = {"sw1", "sw2", "sw3", "sw4"};
    for (int i = 1; i <= 4; ++i) DoOneSwitch(sw[i - 1], i);
}

/*********************************************************************************************************************************/
int CheckRange(int v, int min, int max)
{
    if (v > max) v = max;
    if (v < min) v = min;
    return v;
}
/*********************************************************************************************************************************/

void DoNewChannelName(int ch, int k)
{
    int j                   = 0;
    ChannelNames[ch - 1][0] = 32;
    ChannelNames[ch - 1][1] = 0; // remove old name
    while (uint8_t(TextIn[k]) > 0) {
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
    char dflt[]                = "DEFAULT.MOD";
    char ModelsView_filename[] = "filename";
    char newfname[27];

    strcpy(newfname, dflt);
    if (FileNumberInView >= ExportedFileCounter) FileNumberInView = 0;
    if (FileNumberInView < 0) FileNumberInView = ExportedFileCounter - 1;
    if (ExportedFileCounter) {
        for (int i = 0; i < 12; ++i) {
            newfname[i]     = TheFilesList[FileNumberInView][i];
            newfname[i + 1] = 0;
            if (newfname[i] <= 32 || newfname[i] > 127) break;
        }
    }
    SendText(ModelsView_filename, newfname);
}

/*********************************************************************************************************************************/

void ShowFileErrorMsg()
{
    char ErrorOn[]  = "vis error,1";
    char ErrorOff[] = "vis error,0";
    for (int pp = 0; pp < 3; ++pp) {
        SendCommand(ErrorOn);
        DelayWithDog(200);
        SendCommand(ErrorOff);
        DelayWithDog(100);
    }
    FileError = false;
}

/*********************************************************************************************************************************/

void SaveAllParameters()
{
    if (!ModelsFileOpen) OpenModelsFile();
    SaveTransmitterParameters();
    MemoryForTransmtter = SDCardAddress - 2;
    SaveOneModel(ModelNumber);
#ifdef DB_SD
    Serial.println(" ");
    Serial.print(MemoryForTransmtter);
    Serial.println(" bytes written to SD CARD FOR TX.");
    Serial.print(TXSIZE);
    Serial.println(" bytes reserved for TX.");
    Serial.print(TXSIZE - MemoryForTransmtter);
    Serial.println(" Spare bytes still for any new TX params.");
    Serial.print("Saved model: ");
    Serial.print(ModelNumber);
    Serial.print(" (");
    Serial.print(ModelName);
    Serial.println(")");
#endif // defined DB_SD
}

/*********************************************************************************************************************************/

// This function takes account of the fact one gimbal is upside down ... and some people use mode 2.
// Aileron is always reversed, plus either throttle or elevator according to mode 1 or 2
// It reverses some pins

int AnalogueReed(uint8_t InputChannel)
{
    int value = analogRead(AnalogueInput[InputChannel]);
    if (SticksMode == 2) {
        if ((InputChannel == 0) || (InputChannel == 2)) {
            value = map(value, ChannelMin[InputChannel], ChannelMax[InputChannel], ChannelMax[InputChannel], ChannelMin[InputChannel]);
        }
    }
    else {
        if ((InputChannel == 0) || (InputChannel == 1)) {
            value = map(value, ChannelMin[InputChannel], ChannelMax[InputChannel], ChannelMax[InputChannel], ChannelMin[InputChannel]);
        }
    }
    return value;
}

/*********************************************************************************************************************************/

void SetDefaultValues()
{
    uint16_t j         = 0;
    uint16_t i         = 0;
    char     empty[33] = "Not in use";

    CloseModelsFile();
    OpenModelsFile();

    while ((empty[i]) && (i < 29)) {
        ModelName[i]     = empty[i];
        ModelName[i + 1] = 0;
        ++i;
    }

    char DefaultChannelNames[CHANNELSUSED][11] = {{"Aileron"}, {"Elevator"}, {"Throttle"}, {"Rudder"}, {"Ch 5"}, {"Ch 6"}, {"Ch 7"}, {"Ch 8"}, {"Ch 9"}, {"Ch 10"}, {"Ch 11"}, {"Ch 12"}, {"Ch 13"}, {"Ch 14"}, {"Ch 15"}, {"Ch 16"}};

    for (i = 0; i < CHANNELSUSED; ++i) {
        for (j = 1; j <= 4; ++j) {
            MaxDegrees[j][i]    = 150;
            MidHiDegrees[j][i]  = 120;
            CentreDegrees[j][i] = 90;
            MidLowDegrees[j][i] = 60;
            MinDegrees[j][i]    = 30;
        }
    }

    for (j = 0; j < MAXMIXES; ++j) {
        for (i = 0; i < CHANNELSUSED; ++i) {
            Mixes[j][i] = 0;
        }
    }

    for (j = 0; j < BANKSUSED + 1; ++j) { // must have fudged this somewhere.... 5?!
        for (i = 0; i < CHANNELSUSED; ++i) {
            Trims[j][i] = 80; // MIDPOINT is 80 !
        }
    }
    RXCellCount = 3;

    for (i = 0; i < CHANNELSUSED; ++i) {
        InPutStick[i]    = i;
        ChannelOutPut[i] = i;
    }
    FMSwitch        = 4;
    Autoswitch      = 1;
    Channel9Switch  = 2;
    Channel10Switch = 3;
    Channel11Switch = 0;
    Channel12Switch = 0;
    SWITCH1Reversed = false;
    SWITCH2Reversed = false;
    SWITCH3Reversed = false;
    SWITCH4Reversed = false;

    for (i = 0; i < CHANNELSUSED; ++i) {
        FailSafeChannel[i] = false;
    }

    for (i = 0; i < CHANNELSUSED; ++i) {
        for (j = 0; j < 10; ++j) {
            ChannelNames[i][j] = DefaultChannelNames[i][j];
        }
    }
    for (j = 0; j < BANKSUSED; ++j) {
        for (i = 0; i < CHANNELSUSED; ++i) {
            Exponential[j][i] = DEFAULT_EXPO; // 0% (50) expo = default
        }
    }
    for (j = 0; j < BANKSUSED; ++j) {
        for (i = 0; i < CHANNELSUSED; ++i) {
            InterpolationTypes[j][i] = EXPONENTIALCURVES; // Expo is default
        }
    }
    for (i = 0; i < CHANNELSUSED; ++i) {
        SubTrims[i] = 127; // centre (0 - 254)
    }
    for (j = 0; j < BYTESPERMACRO; ++j) {
        for (i = 0; i < MAXMACROS; ++i) {
            MacrosBuffer[i][j] = 0;
        }
    }
    for (int i = 0; i < 4; ++i) {
        InputTrim[i] = i;
    }
    UseMotorKill              = true;
    MotorChannelZero          = 0;
    MotorChannel              = 15;
    ReversedChannelBITS       = 0; //  No channel reversed
    RxVoltageCorrection       = 0;
    ModelsMacUnionSaved.Val64 = 0;
    UseDualRates              = false;
    Drate1                    = 100;
    Drate2                    = 75;
    Drate3                    = 50;
    DualRateChannels[0]       = 1;
    DualRateChannels[1]       = 2;
    DualRateChannels[2]       = 4;
    DualRateChannels[3]       = 0;
    DualRateChannels[4]       = 0;
    DualRateChannels[5]       = 0;
    DualRateChannels[6]       = 0;
    DualRateChannels[7]       = 0;

    for (int i = 0; i < 4; ++i) {
        BanksInUse[i] = i + 4;
    }
    for (int i = 0; i < 16; ++i) {
        StepSize[i] = 100;
    }
    ModelDefined = 42;
    SaveOneModel(ModelNumber);
    CloseModelsFile();
}

/*********************************************************************************************************************************/

void CheckDualRatesValues()
{

    bool KO = false;
    if (Drate1 > MAXDUALRATE) KO = true;
    if (Drate2 > MAXDUALRATE) KO = true;
    if (Drate3 > MAXDUALRATE) KO = true;

    for (int i = 0; i < 8; ++i) {
        if (DualRateChannels[i] > 16) KO = true;
    }
    if (KO) {
        //  UseDualRates = false;
        Drate1 = 100;
        Drate2 = 75; // 1 is always 100%
        Drate3 = 50;

        DualRateChannels[0] = 1;
        DualRateChannels[1] = 2;
        DualRateChannels[2] = 4;
        DualRateChannels[3] = 0;
        DualRateChannels[4] = 0;
        DualRateChannels[5] = 0;
        DualRateChannels[6] = 0;
        DualRateChannels[7] = 0;
    }
}

/*********************************************************************************************************************************/

#define BOXLEFT 35
#define BOXTOP  35 // NOT YET FULLY INTEGRATED
#define BOXSIZE 395

/*********************************************************************************************************************************/

/** @brief Uses servo degrees to position dots */
FASTRUN void GetDotPositions()
{
    int p      = 0;
    BoxLeft    = BOXLEFT;
    BoxTop     = BOXTOP;
    BoxRight   = BOXLEFT + BOXSIZE;
    BoxBottom  = BOXTOP + BOXSIZE;
    xPoints[0] = BoxLeft;
    xPoints[4] = BoxRight - BOXLEFT;
    p          = map(MinDegrees[Bank][ChanneltoSet - 1], 0, 180, BOXSIZE, BOXLEFT);
    yPoints[0] = constrain(p, 39, 391);
    p          = map(MidLowDegrees[Bank][ChanneltoSet - 1], 0, 180, BOXSIZE, BOXLEFT);
    yPoints[1] = constrain(p, 39, 391);
    xPoints[1] = BOXLEFT + 90;
    xPoints[2] = BOXLEFT + 180;
    p          = map(CentreDegrees[Bank][ChanneltoSet - 1], 0, 180, BOXSIZE, BOXLEFT);
    yPoints[2] = constrain(p, 39, 391);
    xPoints[3] = BOXLEFT + 270;
    p          = map(MidHiDegrees[Bank][ChanneltoSet - 1], 0, 180, BOXSIZE, BOXLEFT);
    yPoints[3] = constrain(p, 39, 391);
    p          = map(MaxDegrees[Bank][ChanneltoSet - 1], 0, 180, BOXSIZE, BOXLEFT);
    yPoints[4] = constrain(p, 39, 391);
}

/*********************************************************************************************************************************/

FASTRUN void updateInterpolationTypes()
{
    char ExpR[]     = "Exp";
    char Smooth[]   = "Smooth";
    char Lines[]    = "Lines";
    char Expon[]    = "Expo";       // number display
    char Ex1[]      = "Ex1";        // slider
    char t3on[]     = "vis t3,1";   // expo value lable
    char b13on[]    = "vis b13,1";  // expo -
    char b12on[]    = "vis b12,1";  // expo +
    char ExponOn[]  = "vis Expo,1"; // number display
    char Ex1On[]    = "vis Ex1,1";  // slider
    char t3off[]    = "vis t3,0";   // expo value lable
    char b13off[]   = "vis b13,0";  // expo -
    char b12off[]   = "vis b12,0";  // expo +
    char ExponOff[] = "vis Expo,0"; // number display
    char Ex1Off[]   = "vis Ex1,0";  // slider

    switch (InterpolationTypes[Bank][ChanneltoSet - 1]) {
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
    int  p       = 0;
    char Gn1[]   = "n1";
    char Gn2[]   = "n2";
    char Gn3[]   = "n3";
    char Gn4[]   = "n4";
    char Gn5[]   = "n5";
    char b3on[]  = "vis b3,1";
    char b4on[]  = "vis b4,1";
    char b7on[]  = "vis b7,1";
    char b8on[]  = "vis b8,1";
    char n2on[]  = "vis n2,1";
    char n4on[]  = "vis n4,1";
    char b3off[] = "vis b3,0";
    char b4off[] = "vis b4,0";
    char b7off[] = "vis b7,0";
    char b8off[] = "vis b8,0";
    char n2off[] = "vis n2,0";
    char n4off[] = "vis n4,0";

    int   Step = 8;
    float HalfXRange;
    float TopHalfYRange;
    float BottomHalfYRange;
    int   xDot1;
    int   yDot1;
    int   xDot2     = 0;
    int   yDot2     = 0;
    int   DotSize   = 4;
    int   DotColour = ForeGroundColour;
    ClearBox();
    p                                     = constrain(MinDegrees[Bank][ChanneltoSet - 1], 0, 180);
    MinDegrees[Bank][ChanneltoSet - 1]    = p;
    p                                     = constrain(MidLowDegrees[Bank][ChanneltoSet - 1], 0, 180);
    MidLowDegrees[Bank][ChanneltoSet - 1] = p;
    p                                     = constrain(CentreDegrees[Bank][ChanneltoSet - 1], 0, 180);
    CentreDegrees[Bank][ChanneltoSet - 1] = p;
    p                                     = constrain(MidHiDegrees[Bank][ChanneltoSet - 1], 0, 180);
    MidHiDegrees[Bank][ChanneltoSet - 1]  = p;
    p                                     = constrain(MaxDegrees[Bank][ChanneltoSet - 1], 0, 180);
    MaxDegrees[Bank][ChanneltoSet - 1]    = p;

    GetDotPositions();
    SendValue(Gn1, DegsToPercent(MinDegrees[Bank][ChanneltoSet - 1])); // put numbers at top row
    SendValue(Gn2, DegsToPercent(MidLowDegrees[Bank][ChanneltoSet - 1]));
    SendValue(Gn3, DegsToPercent(CentreDegrees[Bank][ChanneltoSet - 1]));
    SendValue(Gn4, DegsToPercent(MidHiDegrees[Bank][ChanneltoSet - 1]));
    SendValue(Gn5, DegsToPercent(MaxDegrees[Bank][ChanneltoSet - 1]));

    DrawBox(BoxLeft, BoxTop, BoxRight - BoxLeft, BoxBottom - BoxTop, HighlightColour);
    xDot1 = xPoints[0];
    yDot1 = ((BoxBottom - BoxTop) / 2) + 20; // ?
    xDot2 = BoxRight - BOXLEFT;
    yDot2 = yDot1;
    DrawLine(xDot1, yDot1, xDot2, yDot1, SpecialColour);

    xDot1 = xPoints[2];
    yDot1 = BoxTop;
    xDot2 = xDot1;
    yDot2 = BoxBottom - BOXTOP; //(BOXLEFT)
    DrawLine(xDot1, yDot1, xDot2, yDot2, SpecialColour);

    if (InterpolationTypes[Bank][ChanneltoSet - 1] == STRAIGHTLINES) { // Linear
        SendCommand(b3on);
        SendCommand(b4on);
        SendCommand(b7on);
        SendCommand(b8on);
        SendCommand(n2on);
        SendCommand(n4on);
        DrawLine(xPoints[0], yPoints[0], xPoints[1], yPoints[1], ForeGroundColour); // this adds the straight version of the curve
        DrawLine(xPoints[1], yPoints[1], xPoints[2], yPoints[2], ForeGroundColour);
        DrawLine(xPoints[2], yPoints[2], xPoints[3], yPoints[3], ForeGroundColour);
        DrawLine(xPoints[3], yPoints[3], xPoints[4], yPoints[4], ForeGroundColour);
    }

    if (InterpolationTypes[Bank][ChanneltoSet - 1] == SMOOTHEDCURVES) { // CatmullSpline

        SendCommand(b3on);
        SendCommand(b4on);
        SendCommand(b7on);
        SendCommand(b8on);
        SendCommand(n2on);
        SendCommand(n4on);
        yDot2 = 0;
        for (xPoint = xPoints[0]; xPoint <= xPoints[4]; xPoint += Step) {
            if (Step > xPoints[4] - xPoint) {
                Step = xPoints[4] - xPoint;
            }
            if (Step < 1) Step = 1;
            yPoint = Interpolation::CatmullSpline(xPoints, yPoints, PointsCount, xPoint);
            xDot1  = xPoint;
            yDot1  = yPoint;
            if (yDot2 == 0) {
                xDot2 = xDot1;
                yDot2 = yDot1;
            }
            DrawLine(xDot1, yDot1, xDot2, yDot2, ForeGroundColour);
            xDot2 = xDot1;
            yDot2 = yDot1;
        }
    }

    if (InterpolationTypes[Bank][ChanneltoSet - 1] == EXPONENTIALCURVES) { // EXPO  ************************************************************************************************
#define APPROXIMATION 7                                                    // This is the approximation of the screen curve
        SendCommand(b3off);
        SendCommand(b4off);
        SendCommand(b7off);
        SendCommand(b8off);
        SendCommand(n2off);
        SendCommand(n4off);
        HalfXRange       = xPoints[4] - xPoints[2];
        TopHalfYRange    = yPoints[4] - yPoints[2];
        BottomHalfYRange = yPoints[2] - yPoints[0];
        yDot2            = 0;
        Step             = APPROXIMATION; // This is the approximation of the screen curve

        CheckInvisiblePoint();

        for (xPoint = 0; xPoint <= HalfXRange; xPoint += Step) { // Simulate a curve with many short lines to speed it up
            yPoint = MapWithExponential(HalfXRange - xPoint, HalfXRange, 0, 0, BottomHalfYRange, Exponential[Bank][ChanneltoSet - 1]);
            if (Step > HalfXRange - xPoint) {
                Step = HalfXRange - xPoint;
            }
            if (Step < 1) Step = 1;
            yDot1 = yPoint + yPoints[0];
            xDot1 = xPoint + xPoints[0];
            if (yDot2 == 0) {
                xDot2 = xDot1;
                yDot2 = yDot1;
            }
            DrawLine(xDot1, yDot1, xDot2, yDot2, ForeGroundColour); // Draw short line from this point to previous point
            xDot2 = xDot1;
            yDot2 = yDot1;
        }
        Step  = APPROXIMATION;
        yDot2 = 0;
        for (xPoint = HalfXRange; xPoint >= 0; xPoint -= Step) { // Simulate a curve with many short lines to speed it up
            yPoint = MapWithExponential(xPoint, 0, HalfXRange, 0, TopHalfYRange, Exponential[Bank][ChanneltoSet - 1]);
            if (Step > xPoint) {
                Step = xPoint;
            }
            if (Step < 1) Step = 1;
            yDot1 = yPoint + yPoints[2];
            xDot1 = xPoint + xPoints[2];
            if (yDot2 == 0) {
                xDot2 = xDot1;
                yDot2 = yDot1;
            }
            DrawLine(xDot1, yDot1, xDot2, yDot2, ForeGroundColour); // Draw short line from this point to previous point
            xDot2 = xDot1;
            yDot2 = yDot1;
        }

        DrawDot(xPoints[0], yPoints[0], DotSize, DotColour); // This adds 3 dots
        DrawDot(xPoints[2], yPoints[2], DotSize, DotColour);
        DrawDot(xPoints[4], yPoints[4], DotSize, DotColour);
    }
    if (InterpolationTypes[Bank][ChanneltoSet - 1] != EXPONENTIALCURVES) {
        DrawDot(xPoints[0], yPoints[0], DotSize, DotColour); // This adds 5 dots
        DrawDot(xPoints[1], yPoints[1], DotSize, DotColour);
        DrawDot(xPoints[2], yPoints[2], DotSize, DotColour);
        DrawDot(xPoints[3], yPoints[3], DotSize, DotColour);
        DrawDot(xPoints[4], yPoints[4], DotSize, DotColour);
    }

    DrawDot(xPoints[CurrentPoint - 1], yPoints[CurrentPoint - 1], DotSize + 5, HighlightColour); // Show selected point
    updateInterpolationTypes();
}

/*********************************************************************************************************************************/

void BindNow()
{
    BoundFlag    = true;
    ModelMatched = true;
    Connected    = true;
#ifdef DB_BIND
    Serial.println("");
    Serial.println("Remote (model) ID saved:");
    for (int i = 0; i < 6; ++i) {
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
    char p[]                = "Delete ID for ";
    char p1[]               = "?";
    char Done[]             = "Model ID deleted.";
    char DoneAlready[]      = "No model ID not found.";
    char NotDone[]          = "Model ID retained.";
    char page_RXSetupView[] = "page RXSetupView";

    strcpy(prompt, p);
    strcat(prompt, ModelName);
    strcat(prompt, p1);

    if (!ModelsMacUnionSaved.Val64) {
        GetConfirmation(page_RXSetupView, DoneAlready);
        return;
    }
    if (GetConfirmation(page_RXSetupView, prompt)) {
        ModelsMacUnionSaved.Val64 = 0;
        SaveOneModel(ModelNumber);
        GetConfirmation(page_RXSetupView, Done);
    }
    else {
        GetConfirmation(page_RXSetupView, NotDone);
    }
}

/*********************************************************************************************************************************/

void StoreModelID()
{ // This stores current model ID    // heer

    char prompt[60];
    char p[]                = "Store ID for ";
    char p1[]               = "?";
    char Done[]             = "Model ID stored.";
    char page_RXSetupView[] = "page RXSetupView";
    char NotDone[]          = "Model ID not stored.";
    char DoneAlready[]      = "No ID to store!";

    strcpy(prompt, p);
    strcat(prompt, ModelName);
    strcat(prompt, p1);

    if (!ModelsMacUnion.Val64) {
        GetConfirmation(page_RXSetupView, DoneAlready);
        return;
    }

    if (GetConfirmation(page_RXSetupView, prompt)) {
        PlaySound(MMSAVED);
        ModelsMacUnionSaved.Val64 = ModelsMacUnion.Val64;
        SaveOneModel(ModelNumber);
        GetConfirmation(page_RXSetupView, Done);
    }
    else {
        GetConfirmation(page_RXSetupView, NotDone);
    }
}
/*********************************************************************************************************************************/
int GetDifference(int YtouchPlace, int oldy)
{
    int dd;
    dd = (YtouchPlace - oldy) / 2;
    if (dd < 0) dd = -dd;
    return dd;
}
/*********************************************************************************************************************************/
void MoveCurrentPointUp()
{

    switch (CurrentPoint) {
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
    switch (CurrentPoint) {
        case 1:
            if (MinDegrees[Bank][ChanneltoSet - 1]) --MinDegrees[Bank][ChanneltoSet - 1];
            break;

        case 2:
            if (MidLowDegrees[Bank][ChanneltoSet - 1]) --MidLowDegrees[Bank][ChanneltoSet - 1];
            break;

        case 3:
            if (CentreDegrees[Bank][ChanneltoSet - 1]) --CentreDegrees[Bank][ChanneltoSet - 1];
            break;

        case 4:
            if (MidHiDegrees[Bank][ChanneltoSet - 1]) --MidHiDegrees[Bank][ChanneltoSet - 1];
            break;

        case 5:
            if (MaxDegrees[Bank][ChanneltoSet - 1]) --MaxDegrees[Bank][ChanneltoSet - 1];
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
    GetDotPositions();                             // current
    if (XtouchPlace > BoxRight - BOXLEFT) return;  // out of range
    if (XtouchPlace < BOXLEFT) return;             // out of range
    if (YtouchPlace < BoxTop) return;              // out of range
    if (YtouchPlace > BoxBottom - BOXLEFT) return; // out of range

    if (XtouchPlace < BOXLEFT + xPoints[0]) { // do leftmost point  ?
        CurrentPoint = 1;

        rjump = GetDifference(YtouchPlace, yPoints[0]);
        if (YtouchPlace > yPoints[0]) {
            if (MinDegrees[Bank][ChanneltoSet - 1] >= rjump) MinDegrees[Bank][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (MinDegrees[Bank][ChanneltoSet - 1] <= (180 - rjump)) MinDegrees[Bank][ChanneltoSet - 1] += rjump;
        }
    }

    if (XtouchPlace > xPoints[1] - BOXLEFT && XtouchPlace < xPoints[1] + BOXLEFT) { // do next point  ?
        CurrentPoint = 2;
        CheckInvisiblePoint();
        if (InterpolationTypes[Bank][ChanneltoSet - 1] == EXPONENTIALCURVES) return; //  expo = ignore this area
        rjump = GetDifference(YtouchPlace, yPoints[1]);
        if (YtouchPlace > yPoints[1]) {
            if (MidLowDegrees[Bank][ChanneltoSet - 1] >= rjump) MidLowDegrees[Bank][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (MidLowDegrees[Bank][ChanneltoSet - 1] <= 180 - rjump) MidLowDegrees[Bank][ChanneltoSet - 1] += rjump;
        }
    }

    if (XtouchPlace > xPoints[2] - BOXLEFT && XtouchPlace < xPoints[2] + BOXLEFT) { // do next point  ?
        CurrentPoint = 3;
        rjump        = GetDifference(YtouchPlace, yPoints[2]);
        if (YtouchPlace > yPoints[2]) {
            if (CentreDegrees[Bank][ChanneltoSet - 1] >= rjump) CentreDegrees[Bank][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (CentreDegrees[Bank][ChanneltoSet - 1] <= 180 - rjump) CentreDegrees[Bank][ChanneltoSet - 1] += rjump;
        }
    }

    if (XtouchPlace > xPoints[3] - BOXLEFT && XtouchPlace < xPoints[3] + BOXLEFT) { // do next point  ?
        CurrentPoint = 4;
        CheckInvisiblePoint();
        if (InterpolationTypes[Bank][ChanneltoSet - 1] == EXPONENTIALCURVES) return; //  expo = ignore this area
        rjump = GetDifference(YtouchPlace, yPoints[3]);
        if (YtouchPlace > yPoints[3]) {
            if (MidHiDegrees[Bank][ChanneltoSet - 1] >= rjump) MidHiDegrees[Bank][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (MidHiDegrees[Bank][ChanneltoSet - 1] <= 180 - rjump) MidHiDegrees[Bank][ChanneltoSet - 1] += rjump;
        }
    }
    if (XtouchPlace > xPoints[3] + BOXLEFT) // do hi point  ?
    {
        CurrentPoint = 5;
        rjump        = GetDifference(YtouchPlace, yPoints[4]);
        if (YtouchPlace > yPoints[4]) {
            if (MaxDegrees[Bank][ChanneltoSet - 1] > rjump) MaxDegrees[Bank][ChanneltoSet - 1] -= rjump;
        }
        else {
            if (MaxDegrees[Bank][ChanneltoSet - 1] <= 180 - rjump) MaxDegrees[Bank][ChanneltoSet - 1] += rjump;
        }
    }
}

/*********************************************************************************************************************************/
void ShowFileProgress(char* Msg)
{
    char t1[] = "t1";
    SendText(t1, Msg);
}
/*********************************************************************************************************************************/
// SEND AND RECEIVE A MODEL FILE
/*********************************************************************************************************************************/

#define FILEPIPEADDRESS 0xFEFEFEFEFELL // Unique pipe address for FILE EXCHANGE
#define BUFFERSIZE      28             // + 4 = 32
#define FILEDATARATE    RF24_250KBPS
#define FILEPALEVEL     RF24_PA_MAX
#define FILECHANNEL     123
#define FILETIMEOUT     30

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
        if ((NewFileBufferPointer + i) > MAXFILELEN) break;
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
    char          nb1[20];
    char          Received[] = "Received ";
    char          of[]       = " of ";
    char          msg[50];
    char          bytes[]         = " bytes.";
    char          t0[]            = "t0";
    char          RXheader[]      = "File receive";
    char          ovwr[]          = "Overwrite ";
    char          ques[]          = "?";
    char          ProgressStart[] = "vis Progress,1";
    char          ProgressEnd[]   = "vis Progress,0";
    char          GoModelsView[]  = "page ModelsView";
    char          Progress[]      = "Progress";

    strcpy(msg, ovwr);
    strcat(msg, ModelName);
    strcat(msg, ques);
    if (!GetConfirmation(GoModelsView, msg)) return; // Get confirmation or quit

    BlueLedOn();
    ShowFileTransferWindow();
    SendText(ModelsView_filename, Waiting);
    SendText(t0, RXheader);
    if (PPMdata.UseTXModule) {
        InitRadio(DefaultPipe);
    }
    RXPipe = FILEPIPEADDRESS;
    Radio1.setRetries(15, 15);
    Radio1.setChannel(FILECHANNEL);
    Radio1.flush_tx();
    Radio1.openReadingPipe(1, RXPipe);
    Radio1.startListening();
    NewFileBufferPointer = 0;
    RXTimer              = millis(); // Start timer
    ClearText();
    CloseModelsFile();
    while (!Radio1.available()) { // Await the sender....
        delay(1);
        if (GetButtonPress()) {
            GotoModelsView();
            ClearText();
            NormaliseTheRadio();
            RedLedOn();
            ButtonWasPressed();
            return;
        }
        KickTheDog(); // Watchdog
        if ((millis() - RXTimer) / 1000 >= FILETIMEOUT) {
#ifdef DB_MODEL_EXCHANGE
            Serial.println("Timeout");
#endif
            SendText(ModelsView_filename, TimeoutMsg);
            NormaliseTheRadio();
            return; // Give up waiting
        }
        else {
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

    SendCommand(ProgressStart);
    SendValue(Progress, p);
    SendText(ModelsView_filename, Receiving);
    Radio1.writeAckPayload(1, &Fack, sizeof(Fack)); //  Ack first packet
    Radio1.read(&Fbuffer, BUFFERSIZE + 4);          //  Read it was 12
    strcpy(SingleModelFile, Fbuffer);               //  Get filename
    Fsize = Fbuffer[BUFFERSIZE];
    Fsize += Fbuffer[BUFFERSIZE + 1] << 8;
    Fsize += Fbuffer[BUFFERSIZE + 2] << 16;
    Fsize += Fbuffer[BUFFERSIZE + 3] << 24; //  Get file size
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
            Radio1.read(&Fbuffer, BUFFERSIZE + 4);
            StoreBuffer(Fbuffer, BUFFERSIZE); // Store it in ram for now rather than disk it
            Radio1.flush_rx();
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
    }
    SendValue(Progress, 100);
    WriteEntireBuffer();
    BuildDirectory();
    SendText(ModelsView_filename, Success);
    DelayWithDog(500);
    SendText(ModelsView_filename, SingleModelFile);

    // **************************************** Below Here the new model is imported for immediate use

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
    GotoModelsView();
    ClearText();
    RedLedOn();
    BoundFlag = true; // This just prevents jump to front screen (Cleared on leaving models area)
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
    char          msg[50];
    char          bytes[]               = " bytes.";
    uint32_t      SentMoment            = 0;
    char          ModelsView_filename[] = "filename";
    char          t0[]                  = "t0";
    char          Fsend[]               = "Sending file";
    char          ProgressStart[]       = "vis Progress,1";
    char          ProgressEnd[]         = "vis Progress,0";
    char          GoModelsView[]        = "page ModelsView";
    char          Progress[]            = "Progress";

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
    if (PPMdata.UseTXModule) {
        InitRadio(DefaultPipe);
    }
    Radio1.setChannel(FILECHANNEL);
    Radio1.setPALevel(FILEPALEVEL, true);
    Radio1.setRetries(15, 15);
    Radio1.openWritingPipe(TXPipe);
    Radio1.stopListening();
    DelayWithDog(4);

    while (Fposition < Fsize) {
        KickTheDog(); // Watchdog
        p = ((float)Fposition / (float)Fsize) * 100;
        strcpy(msg, Sent);
        strcat(msg, Str(nb1, Fposition, 0));
        strcat(msg, of);
        strcat(msg, Str(nb1, Fsize, 0));
        ShowFileProgress(msg);
        SendValue(Progress, p);
        ++PacketNumber;
        if (PacketNumber == 1) {
            strcpy(Fbuffer, SingleModelFile); // Filename in first packet
            Fbuffer[BUFFERSIZE]     = Fsize;
            Fbuffer[BUFFERSIZE + 1] = Fsize >> 8;
            Fbuffer[BUFFERSIZE + 2] = Fsize >> 16;
            Fbuffer[BUFFERSIZE + 3] = Fsize >> 24; // SEND FILE SIZE (four bytes)
        }
        else {
            ModelsFileNumber.seek(Fposition); // Move filepointer
            ShortDelay();
            ModelsFileNumber.read(Fbuffer, BUFFERSIZE); // Read part of file
            Fposition += BUFFERSIZE;
            if (Fposition > Fsize) Fposition = Fsize;
        }

        while (millis() - SentMoment < 25) {
            Radio1.flush_tx();
            Radio1.flush_rx();
        }
        SentMoment = millis();

        if (Radio1.write(&Fbuffer, BUFFERSIZE + 4)) {
            Radio1.read(&Fack, sizeof(Fack)); // ignore the ACK
        }

#ifdef DB_MODEL_EXCHANGE
        Serial.println(PacketNumber);
#endif
    }
    ModelsFileNumber.close();
#ifdef DB_MODEL_EXCHANGE
    Serial.println("ALL SENT.");
#endif
    SendValue(Progress, 100);
    DelayWithDog(750);
    NormaliseTheRadio();
    SendCommand(ProgressEnd);
    RedLedOn();
    strcpy(msg, Sent);
    strcat(msg, Str(nb1, Fsize, 0));
    strcat(msg, bytes);
    ShowFileProgress(msg);
    PlaySound(BEEPCOMPLETE);
    DelayWithDog(2000);
    SendCommand(GoModelsView);
    CurrentView = MODELSVIEW;
    CloseModelsFile();
}
/*********************************************************************************************************************************/

void SoundBank()
{
    PlaySound(BankSounds[BanksInUse[Bank - 1]]);
    ScreenTimeTimer = millis(); // reset screen counter
    if (ScreenIsOff) {
        RestoreBrightness();
        ScreenIsOff = false;
    }
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
    char bt0[]    = "bt0";
    char OnMsg[]  = "Motor ON";
    char OffMsg[] = "Motor OFF";
    if (on == 1) SendText(bt0, OnMsg);
    if (on == 0) SendText(bt0, OffMsg);
}
/*********************************************************************************************************************************/

void DoOneSwitchView(uint8_t n)
{ // re-write one day ... This works, but not very elagantly.

    char t3[]   = "t3";
    char t4[]   = "t4";
    char t5[]   = "t5";
    char t6[]   = "t6";
    char ch9[]  = "Channel 9";
    char ch10[] = "Channel 10";
    char ch11[] = "Channel 11";
    char ch12[] = "Channel 12";

    char OneSwitchView_r0[]    = "r0";     // Not used
    char OneSwitchView_r1[]    = "r1";     // Flight modes
    char OneSwitchView_r2[]    = "r2";     // Auto
    char OneSwitchView_r3[]    = "r3";     // Ch9
    char OneSwitchView_r4[]    = "r4";     // Ch10
    char OneSwitchView_r5[]    = "r5";     // Ch11
    char OneSwitchView_r6[]    = "r6";     // Ch12
    char OneSwitchView_r7[]    = "r7";     // Safety
    char OneSwitchView_r8[]    = "r8";     // Dual Rates
    char OneSwitchView_r9[]    = "r9";     // Buddy
    char OneSwitchViewc_revd[] = "c_revd"; // Reversed
    ValueSent                  = false;    // If no setting, = Not Used

    if (FMSwitch == n) SendValue(OneSwitchView_r1, 1);
    if (Autoswitch == n) SendValue(OneSwitchView_r2, 1);
    if (Channel9Switch == n) SendValue(OneSwitchView_r3, 1);
    if (Channel10Switch == n) SendValue(OneSwitchView_r4, 1);
    if (Channel11Switch == n) SendValue(OneSwitchView_r5, 1);
    if (Channel12Switch == n) SendValue(OneSwitchView_r6, 1);
    if (SafetySwitch == n) SendValue(OneSwitchView_r7, 1);
    if (DualRatesSwitch == n) SendValue(OneSwitchView_r8, 1);
    if (BuddySwitch == n) SendValue(OneSwitchView_r9, 1);

    if (!ValueSent) SendValue(OneSwitchView_r0, 1); // nothing yet, so not used
    if (SWITCH1Reversed) SendValue(OneSwitchViewc_revd, 1);
    SendText(t3, ch9);
    SendText(t4, ch10);
    SendText(t5, ch11);
    SendText(t6, ch12);

    if (strlen(ChannelNames[8]) >= 2) SendText(t3, ChannelNames[8]);   // Show EDITED channel names if they exist
    if (strlen(ChannelNames[9]) >= 2) SendText(t4, ChannelNames[9]);   // Show EDITED channel names if they exist
    if (strlen(ChannelNames[10]) >= 2) SendText(t5, ChannelNames[10]); // Show EDITED channel names if they exist
    if (strlen(ChannelNames[11]) >= 2) SendText(t6, ChannelNames[11]); // Show EDITED channel names if they exist
}
/*********************************************************************************************************************************/
void UpdateOneSwitchView()
{
    char SwNum[] = "Sw";
    SendValue(SwNum, SwitchEditNumber); // show switch number
    DoOneSwitchView(SwitchEditNumber);  // heer
}
/*********************************************************************************************************************************/
void RestoreBrightness()
{
    char cmd[20];
    char dim[] = "dim=";
    char nb[10];
    if (Brightness < 10) Brightness = 10;
    strcpy(cmd, dim);
    Str(nb, Brightness, 0);
    strcat(cmd, nb);
    SendCommand(cmd);
    ScreenTimeTimer = millis(); // reset screen counter
}
/*********************************************************************************************************************************/

void ZeroDataScreen()
{ // ZERO Those parameters that are zeroable
    TotalLostPackets   = 0;
    GapLongest         = 0;
    GapSum             = 0;
    GapAverage         = 0;
    GapCount           = 0;
    GapStart           = 0;
    RXMAXModelAltitude = 0;
    GPSMaxaltitude     = 0;
    ThisGap            = 0;
    GPSMaxDistance     = 0;
    GPSMaxSpeed        = 0;
    SavedRadioSwaps    = RadioSwaps; // Cannot easily zero these, so do a subtraction
    SavedRX1TotalTime  = RX1TotalTime;
    SavedRX2TotalTime  = RX2TotalTime;
    SavedSbusRepeats   = SbusRepeats;
    LastShowTime       = 0; // for instant redisplay
}
/***************************************************** ReadNewSwitchFunction ****************************************************************************/

void ReadNewSwitchFunction()
{
    char OneSwitchView_r1[]    = "r1"; // Flight modes
    char OneSwitchView_r2[]    = "r2"; // Auto
    char OneSwitchView_r3[]    = "r3"; // Ch9
    char OneSwitchView_r4[]    = "r4"; // Ch10
    char OneSwitchView_r5[]    = "r5"; // Ch11
    char OneSwitchView_r6[]    = "r6"; // Ch12
    char OneSwitchView_r7[]    = "r7"; // Safety
    char OneSwitchView_r8[]    = "r8"; // Dual Rates
    char OneSwitchView_r9[]    = "r9"; // Buddy
    char ProgressStart[]       = "vis Progress,1";
    char ProgressEnd[]         = "vis Progress,0";
    char PageSwitchView[]      = "page SwitchesView";
    char OneSwitchViewc_revd[] = "c_revd"; // Reversed
    char Progress[]            = "Progress";

    SendCommand(ProgressStart);
    SendValue(Progress, 10);

    if (GetValue(OneSwitchView_r1)) {
        FMSwitch = SwitchEditNumber;
    }
    else {
        if (FMSwitch == SwitchEditNumber) FMSwitch = 0;
    }

    SendValue(Progress, 15);
    if (GetValue(OneSwitchView_r2)) {
        Autoswitch = SwitchEditNumber;
    }
    else {
        if (Autoswitch == SwitchEditNumber) Autoswitch = 0;
    }
    SendValue(Progress, 25);
    if (GetValue(OneSwitchView_r3)) {
        Channel9Switch = SwitchEditNumber;
    }
    else {
        if (Channel9Switch == SwitchEditNumber) Channel9Switch = 0;
    }
    SendValue(Progress, 30);
    if (GetValue(OneSwitchView_r4)) {
        Channel10Switch = SwitchEditNumber;
    }
    else {
        if (Channel10Switch == SwitchEditNumber) Channel10Switch = 0;
    }
    SendValue(Progress, 40);
    if (GetValue(OneSwitchView_r5)) {
        Channel11Switch = SwitchEditNumber;
    }
    else {
        if (Channel11Switch == SwitchEditNumber) Channel11Switch = 0;
    }
    SendValue(Progress, 50);
    if (GetValue(OneSwitchView_r6)) {
        Channel12Switch = SwitchEditNumber;
    }
    else {
        if (Channel12Switch == SwitchEditNumber) Channel12Switch = 0;
    }
    SendValue(Progress, 60);
    if (GetValue(OneSwitchView_r7)) {
        SafetySwitch = SwitchEditNumber;
    }
    else {
        if (SafetySwitch == SwitchEditNumber) SafetySwitch = 0;
    }
    SendValue(Progress, 70);
    if (GetValue(OneSwitchView_r8)) {
        DualRatesSwitch = SwitchEditNumber;
    }
    else {
        if (DualRatesSwitch == SwitchEditNumber) DualRatesSwitch = 0;
    }
    SendValue(Progress, 80);
    if (GetValue(OneSwitchView_r9)) {
        BuddySwitch = SwitchEditNumber;
    }
    else {
        if (BuddySwitch == SwitchEditNumber) BuddySwitch = 0;
    }

    SendValue(Progress, 90);
    if (SwitchEditNumber == 1) {
        if (GetValue(OneSwitchViewc_revd)) {
            SWITCH1Reversed = true;
        }
        else {
            SWITCH1Reversed = false;
        }
    }
    if (SwitchEditNumber == 2) {
        if (GetValue(OneSwitchViewc_revd)) {
            SWITCH2Reversed = true;
        }
        else {
            SWITCH2Reversed = false;
        }
        SendValue(Progress, 95);
    }
    if (SwitchEditNumber == 3) {
        if (GetValue(OneSwitchViewc_revd)) {
            SWITCH3Reversed = true;
        }
        else {
            SWITCH3Reversed = false;
        }
    }
    if (SwitchEditNumber == 4) {
        if (GetValue(OneSwitchViewc_revd)) {
            SWITCH4Reversed = true;
        }
        else {
            SWITCH4Reversed = false;
        }
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
/***************************************************** ShowChannelName ****************************************************************************/

void ShowChannelName()
{
    char    MoveToChannel[]  = "Mch";
    char    MacrosView_chM[] = "chM";
    uint8_t ch               = GetValue(MoveToChannel);
    if (ch > 0) --ch; // no zero
    SendText(MacrosView_chM, ChannelNames[ch]);
}
/***************************************************** Populate Macros View ****************************************************************************/

void PopulateMacrosView()
{
    char    MacroNumber[]    = "Mno";
    char    TriggerChannel[] = "Tch";
    char    MoveToChannel[]  = "Mch";
    char    MoveToPosition[] = "Pos";
    char    Delay[]          = "Del";
    char    Duration[]       = "Dur";
    uint8_t n                = PreviousMacroNumber;

    if (n < 8) { // Read previous values before moveing to next
        MacrosBuffer[n][MACROTRIGGERCHANNEL] = GetValue(TriggerChannel);
        MacrosBuffer[n][MACROMOVECHANNEL]    = GetValue(MoveToChannel);
        MacrosBuffer[n][MACROMOVETOPOSITION] = GetValue(MoveToPosition);
        MacrosBuffer[n][MACROSTARTTIME]      = GetValue(Delay);
        MacrosBuffer[n][MACRODURATION]       = GetValue(Duration);
    }
    n = GetValue(MacroNumber) - 1;
    SendValue(TriggerChannel, MacrosBuffer[n][MACROTRIGGERCHANNEL]);
    SendValue(MoveToChannel, MacrosBuffer[n][MACROMOVECHANNEL]);
    SendValue(MoveToPosition, MacrosBuffer[n][MACROMOVETOPOSITION]);
    SendValue(Delay, MacrosBuffer[n][MACROSTARTTIME]);
    SendValue(Duration, MacrosBuffer[n][MACRODURATION]);
    ShowChannelName();
    PreviousMacroNumber = n;
}

/*********************************************************************************************************************************/

void ExitMacrosView()
{
    char    pRXSetupView[]               = "page RXSetupView";
    char    MacroNumber[]                = "Mno";
    char    TriggerChannel[]             = "Tch";
    char    MoveToChannel[]              = "Mch";
    char    MoveToPosition[]             = "Pos";
    char    Delay[]                      = "Del";
    char    Duration[]                   = "Dur";
    uint8_t n                            = GetValue(MacroNumber) - 1;
    MacrosBuffer[n][MACROTRIGGERCHANNEL] = GetValue(TriggerChannel);
    MacrosBuffer[n][MACROMOVECHANNEL]    = GetValue(MoveToChannel);
    MacrosBuffer[n][MACROMOVETOPOSITION] = GetValue(MoveToPosition);
    MacrosBuffer[n][MACROSTARTTIME]      = GetValue(Delay);
    MacrosBuffer[n][MACRODURATION]       = GetValue(Duration);
    UseMacros                            = true;
    SaveOneModel(ModelNumber);
    b5isGrey  = false;
    b12isGrey = false;
    SendCommand(pRXSetupView);
    CurrentView = RXSETUPVIEW;
    UpdateModelsNameEveryWhere();
}

/*********************************************************************************************************************************/

void EndReverseView()
{ // channel reverse flags are 16 individual BITs in var 'ReversedChannelBITS'
    char    fs[16][5] = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
    uint8_t i;
    char    ProgressStart[] = "vis Progress,1";
    char    pRXSetupView[]  = "page RXSetupView";
    char    Progress[]      = "Progress";
    SendCommand(ProgressStart);
    ReversedChannelBITS = 0;
    for (i = 0; i < 16; ++i) {
        SendValue(Progress, (i * (100 / 16)));
        if (GetValue(fs[i])) ReversedChannelBITS |= 1 << i; // set a BIT
    }
    SaveOneModel(ModelNumber);
    b5isGrey  = false;
    b12isGrey = false;
    SendCommand(pRXSetupView);
    CurrentView = RXSETUPVIEW;
    UpdateModelsNameEveryWhere();
}

/*********************************************************************************************************************************/

void StartReverseView()
{ // channel reverse flags are 16 individual BITs in ReversedChannelBITS
    char    pReverseView[] = "page ReverseView";
    char    fs[16][5]      = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
    uint8_t i;
    char    ProgressStart[] = "vis Progress,1";
    char    ProgressEnd[]   = "vis Progress,0";
    char    Progress[]      = "Progress";

    CurrentView = REVERSEVIEW;
    SendCommand(pReverseView);
    UpdateButtonLabels();
    SendCommand(ProgressStart);
    for (i = 0; i < 16; ++i) {
        SendValue(Progress, (i * (100 / 16)));
        if (ReversedChannelBITS & 1 << i) { // is BIT set??
            SendValue(fs[i], 1);
        }
        else {
            SendValue(fs[i], 0);
        }
    }
    SendCommand(ProgressEnd);
    UpdateModelsNameEveryWhere();
}

/*********************************************************************************************************************************/

void StartBuddyView()
{
    char BuddyM[]     = "BuddyM";
    char BuddyP[]     = "BuddyP";
    char pBuddyView[] = "page BuddyView";
    SendCommand(pBuddyView);
    CurrentView = BUDDYVIEW;
    SendValue(BuddyM, BuddyMaster);
    SendValue(BuddyP, BuddyPupilOnPPM);
}

/*********************************************************************************************************************************/

void EndBuddyView()
{
    char BuddyM[]       = "BuddyM";
    char BuddyP[]       = "BuddyP";
    char pRXSetupView[] = "page RXSetupView";
    bool OldPupil;
    bool OldMaster;
    char prompt[] = "Power off transmitter?";
    char GoBack[] = "page BuddyView";

    OldPupil        = BuddyPupilOnPPM; // save old version to detect a change
    OldMaster       = BuddyMaster;
    BuddyPupilOnPPM = GetValue(BuddyP); // Pupil
    BuddyMaster     = GetValue(BuddyM); // Master
    if ((OldPupil != BuddyPupilOnPPM) || (OldMaster != BuddyMaster)) {
        if (!GetConfirmation(GoBack, prompt)) {
            SendValue(BuddyM, OldMaster);
            SendValue(BuddyP, OldPupil);
            BuddyPupilOnPPM = OldPupil;
            BuddyMaster     = OldMaster;
        }
    }
    SaveAllParameters();
    b5isGrey  = false;
    b12isGrey = false;
    SendCommand(pRXSetupView);
    CurrentView = RXSETUPVIEW;
    UpdateModelsNameEveryWhere();
    if ((OldPupil != BuddyPupilOnPPM) || (OldMaster != BuddyMaster)) {
        digitalWrite(POWER_OFF_PIN, HIGH);
    }
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
/******************************** FUNCTIONS FOR ARRAY OF POINTERS *************************************************************/
void Blank()
{
    return;
}

/******************************************************************************************************************************/
void DoLastTimeRead()
{
    LastTimeRead = 0;
}

/******************************************************************************************************************************/
void LoadModelSelector()
{

    char MMemsp[] = "MMems.path=\"";
    char MMems[]  = "MMems";
    char crlf[]   = {13, 10, 0};
    char lb[]     = " (";
    char rb[]     = ")";
    char nb[4];
    char buf[MAXBUFFERSIZE];
    char mn[] = "modelname";

    int32_t SavedModelNumber = ModelNumber;
    for (ModelNumber = 1; ModelNumber < MAXMODELNUMBER; ++ModelNumber) {
        ReadOneModel(ModelNumber);
        if (ModelNumber == 1) {
            strcpy(buf, ModelName);
            strcat(buf, lb);
            Str(nb, ModelNumber, 0);
            strcat(buf, nb);
            strcat(buf, rb);
            strcat(buf, crlf);
        }
        else {
            strcat(buf, ModelName);
            strcat(buf, lb);
            Str(nb, ModelNumber, 0);
            strcat(buf, nb);
            strcat(buf, rb);
            strcat(buf, crlf);
        }
    }
    SendOtherText(MMemsp, buf);
    ModelNumber = SavedModelNumber;
    ReadOneModel(ModelNumber);
    SendValue(MMems, ModelNumber - 1);
    SendText(mn, ModelName);
}

/******************************************************************************************************************************/

void LoadFileSelector()
{
    char Mfilesp[] = "Mfiles.path=\"";
    char Mfiles[]  = "Mfiles";
    char crlf[]    = {13, 10, 0};
    char buf[MAXBUFFERSIZE];
    char nofiles[] = "(No files)";

    strcpy(buf, nofiles);
    for (int f = 0; f < ExportedFileCounter; ++f) {
        if (f == 0)
        {
            strcpy(buf, TheFilesList[f]);
            strcat(buf, crlf);
        }
        else {
            strcat(buf, TheFilesList[f]);
            strcat(buf, crlf);
        }
    }
    SendOtherText(Mfilesp, buf);
    FileNumberInView = GetValue(Mfiles);
    LastFileInView   = 120;
}

/******************************************************************************************************************************/
void GotoModelsView()
{
    char GoModelsView[] = "page ModelsView";

    if (ModelMatched) return; // must not change when model connected
    SaveCurrentModel();
    SendCommand(GoModelsView);
    CurrentView = MODELSVIEW;
    CurrentMode = SENDNOTHING;
    BlueLedOn();
    UpdateModelsNameEveryWhere();
    BuildDirectory();
    LoadFileSelector();
    ShowFileNumber();
    PreviousModelNumber = ModelNumber; // save number
    LoadModelSelector();
}
/******************************************************************************************************************************/
void GotoMacrosView()
{
    char pMacrosView[]  = "page MacrosView";
    PreviousMacroNumber = 200; // i.e. no usable number
    SendCommand(pMacrosView);  // Display MacroView
    CurrentView = MACROS_VIEW;
    DelayWithDog(200); // allow enough time for screen to display
    UpdateModelsNameEveryWhere();
    PopulateMacrosView();
}
/******************************************************************************************************************************/
void UpLog()
{
    if (RecentStartLine > 0 && (CurrentView == LOGVIEW)) {
        RecentStartLine -= 1;
        if (RecentStartLine < 0) RecentStartLine = 0;
        ShowLogFile(RecentStartLine);
    }
    if (RecentStartLine > 0 && (CurrentView == HELP_VIEW)) {
        RecentStartLine -= 1;
        if (RecentStartLine < 1) RecentStartLine = 0;
        ScrollHelpFile();
    }
}
/******************************************************************************************************************************/
void DownLog()
{

    if (ThereIsMoreToSee && (CurrentView == LOGVIEW)) {
        RecentStartLine += 1;
        ShowLogFile(RecentStartLine);
    }
    if (ThereIsMoreToSee && (CurrentView == HELP_VIEW)) {
        RecentStartLine += 1;
        ScrollHelpFile();
    }
}
/******************************************************************************************************************************/
void RefreshLog()
{ // refresh log screen
    if (UseLog) {
        RecentStartLine = 0;
        ShowLogFile(RecentStartLine);
        ClearText();
    }
}
/******************************************************************************************************************************/
void LogEND()
{ // close log screen
    char n0[]        = "n0";
    char c0[]        = "c0";
    char sw0[]       = "sw0";
    char pDataView[] = "page DataView";
    CurrentMode      = NORMAL;
    CurrentView      = DATAVIEW;
    LastShowTime     = 0;
    MinimumGap       = GetValue(n0);
    LogRXSwaps       = GetValue(c0);
    UseLog           = GetValue(sw0);
    SaveTransmitterParameters();
    SendCommand(pDataView);
}
/******************************************************************************************************************************/
void DelLOG()
{ // delete log and start new one

    char prompt[]   = "Delete log file?";
    char pLogView[] = "page LogView";
    if (GetConfirmation(pLogView, prompt)) {
        DeleteLogFile1();
    }
    ClearText();
}
/******************************************************************************************************************************/
void LogVIEW()
{ // Start log screen
    char n0[]       = "n0";
    char c0[]       = "c0";
    char sw0[]      = "sw0";
    char pLogView[] = "page LogView";
    SendCommand(pLogView);
    CurrentView = LOGVIEW;
    SendValue(n0, MinimumGap);
    SendValue(c0, LogRXSwaps);
    SendValue(sw0, UseLog);
    if (UseLog) {
        RecentStartLine = 0;
        ShowLogFile(RecentStartLine);
    }
}

/******************************************************************************************************************************/
void SetupViewFM()
{
    char page_RXSetupView[] = "page RXSetupView";

    SaveAllParameters();
    CurrentView = RXSETUPVIEW;
    SendCommand(page_RXSetupView);
    CurrentMode = NORMAL; // Send data again
    UpdateModelsNameEveryWhere();
}

/******************************************************************************************************************************/
void Options2End()
{ // back to setup?
    char dGMT[]           = "dGMT";
    char page_SetupView[] = "page TXSetupView";
    DeltaGMT              = GetValue(dGMT);
    SaveTransmitterParameters();
    CurrentView = TXSETUPVIEW;
    SendCommand(page_SetupView);
    UpdateModelsNameEveryWhere();
}
/******************************************************************************************************************************/

void OptionView2Start()
{
    char dGMT[] = "dGMT"; // Time zone
    char n1[]   = "n1";
    char n2[]   = "n2";
    char n3[]   = "n3";

    char OptionV2Start[] = "page OptionView2";
    char TxVCorrextion[] = "t2";

    if (CurrentView == OPTIONVIEW3) { //  TODO: And what if was Options 1??

        TxVoltageCorrection    = GetValue(TxVCorrextion);
        PowerOffWarningSeconds = GetValue(n2);
        PowerOffWarningSeconds = CheckRange(PowerOffWarningSeconds, 1, 10);

        if (LEDBrightness != GetValue(n1)) UpdateLED();
        ConnectionAssessSeconds = GetValue(n3);
        ConnectionAssessSeconds = CheckRange(ConnectionAssessSeconds, 1, 6);
        SaveAllParameters();
    }

    CurrentView  = OPTIONVIEW2;
    LastTimeRead = 0;
    SendCommand(OptionV2Start);
    DelayWithDog(100);
    SendValue(dGMT, DeltaGMT);
}

/******************************************************************************************************************************/

void OptionView3Start() /// NOT CALLED
{
    char TxVCorrextion[] = "t2";
    char n1[]            = "n1";
    char n2[]            = "n2";
    char t10[]           = "t10";
    char n3[]            = "n3";
    char lpm[]           = "c0"; // Low power mode
    char Vbuf[10];
    char OptionV3Start[] = "page OptionView3";
    char QNH[]           = "Qnh";

    CurrentView = OPTIONVIEW3;
    SendCommand(OptionV3Start);
    DelayWithDog(250);
    snprintf(Vbuf, 5, "%1.2f", StopFlyingVoltsPerCell);
    SendText(t10, Vbuf);
    SendValue(TxVCorrextion, TxVoltageCorrection);
    SendValue(n2, PowerOffWarningSeconds);
    SendValue(n3, ConnectionAssessSeconds);
    SendValue(lpm, AutoModelSelect);
    if (LEDBrightness < 15) LEDBrightness = DEFAULTLEDBRIGHTNESS;
    SendValue(n1, LEDBrightness);
    SendValue(QNH, Qnh);
}

/******************************************************************************************************************************/

void RXOptionsViewStart() // model options screen
{
    char pRXSetup1[] = "page RXOptionsView";
    char UseKill[]   = "c0";
    char Mchannel[]  = "n1";
    char Mvalue[]    = "n0";
    char t10[]       = "t10";
    char Vbuf[15];
    char RxVCorrextion[] = "n2";
    char c1[]            = "c1";
    char n3[]            = "n3";

    char n4[] = "n4"; // TimerDownwards timer minutes
    char c2[] = "c2"; // TimerDownwards timer on off
    char r0[] = "r0"; // SBUS on
    char r1[] = "r1"; // PPM on
    char n5[] = "n5"; // PPMChannelCount

    SendCommand(pRXSetup1);
    SendValue(c1, CopyTrimsToAll);
    SendValue(n3, TrimMultiplier);
    snprintf(Vbuf, 5, "%1.2f", StopFlyingVoltsPerCell);
    SendText(t10, Vbuf);
    SendValue(Mvalue, MotorChannelZero);
    SendValue(Mchannel, MotorChannel + 1);
    SendValue(UseKill, UseMotorKill);
    SendValue(RxVCorrextion, RxVoltageCorrection);
    SendValue(c2, TimerDownwards);
    SendValue(n4, TimerStartTime / 60);
    SendValue(r0, PPMdata.UseSBUSFromRX);
    SendValue(r1, !PPMdata.UseSBUSFromRX);
    SendValue(n5, PPMdata.PPMChannelCount);
    CurrentView = RXSETUPVIEW1;
    UpdateModelsNameEveryWhere();
}

/******************************************************************************************************************************/

void RXSetup1End()
{
    char UseKill[]  = "c0";
    char Mchannel[] = "n1";
    char Mvalue[]   = "n0";
    char t10[]      = "t10";
    char fbuf[16];
    char RxVCorrextion[]    = "n2";
    char c1[]               = "c1";
    char n3[]               = "n3";
    char n4[]               = "n4"; // TimerDownwards timer minutes
    char c2[]               = "c2"; // TimerDownwards timer on off
    char r0[]               = "r0"; // SBUS on
    char n5[]               = "n5"; // PPMChannelCount
    char page_RXSetupView[] = "page RXSetupView";
    char ProgressStart[]    = "vis Progress,1";
    char Progress[]         = "Progress";

    SendCommand(ProgressStart);
    CopyTrimsToAll = GetValue(c1);
    SendValue(Progress, 5);
    TrimMultiplier = GetValue(n3);
    GetText(t10, fbuf);
    StopFlyingVoltsPerCell = atof(fbuf);
    SendValue(Progress, 10);
    SFV = StopFlyingVoltsPerCell * 100; // this makes it a 16 bit value I can save easily
    SendValue(Progress, 20);
    MotorChannelZero = GetValue(Mvalue);
    SendValue(Progress, 30);
    RxVoltageCorrection = GetValue(RxVCorrextion);
    SendValue(Progress, 40);
    UseMotorKill = GetValue(UseKill);
    SendValue(Progress, 50);
    MotorChannel = GetValue(Mchannel) - 1;
    SendValue(Progress, 60);
    TimerDownwards = GetValue(c2);
    SendValue(Progress, 70);
    TimerStartTime = GetValue(n4) * 60;
    SendValue(Progress, 80);
    PPMdata.UseSBUSFromRX = GetValue(r0);
    SendValue(Progress, 90);
    PPMdata.PPMChannelCount = GetValue(n5);
    SendValue(Progress, 100);
    CurrentView = RXSETUPVIEW;
    SaveOneModel(ModelNumber);
    UpdateModelsNameEveryWhere();
    SendCommand(page_RXSetupView);
}

/******************************************************************************************************************************/

void UpdateLED()
{ // LED Brightness has changed so this ensures it is redisplayed
    char n1[]     = "n1";
    LEDBrightness = GetValue(n1);
    LEDBrightness = CheckRange(LEDBrightness, 1, 254);
    LedWasGreen   = false; // Forces a redisplay if brightness has changed
}

/******************************************************************************************************************************/

void OptionView3End() //
{
    char TxVCorrextion[]  = "t2";
    char n2[]             = "n2";
    char n3[]             = "n3";
    char n1[]             = "n1";
    char page_SetupView[] = "page TXSetupView";
    char QNH[]            = "Qnh";

    TxVoltageCorrection    = GetValue(TxVCorrextion);
    PowerOffWarningSeconds = GetValue(n2);
    PowerOffWarningSeconds = CheckRange(PowerOffWarningSeconds, 1, 10);
    Qnh                    = (uint16_t)GetValue(QNH);
    if (LEDBrightness != GetValue(n1)) UpdateLED();
    ConnectionAssessSeconds = GetValue(n3);
    ConnectionAssessSeconds = CheckRange(ConnectionAssessSeconds, 1, 6);
    SaveTransmitterParameters();
    CloseModelsFile();
    CurrentView = TXSETUPVIEW;
    SendCommand(page_SetupView);
    UpdateModelsNameEveryWhere();
}

/******************************************************************************************************************************/

void BuddyChViewStart()
{
    char page_BuddyChView[] = "page BuddyChView";
    char fs[16][5]          = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
    SendCommand(page_BuddyChView);
    CurrentView = BUDDYCHVIEW;
    UpdateButtonLabels();
    for (int i = 0; i < 16; ++i) {
        if (BuddyControlled & 1 << i) {
            SendValue(fs[i], 1);
        }
        else {
            SendValue(fs[i], 0);
        }
    }
}

/******************************************************************************************************************************/

void BuddyChViewEnd()
{
    char page_BuddyView[] = "page BuddyView";
    char Progress[]       = "Progress";
    char ProgressStart[]  = "vis Progress,1";
    char fs[16][5]        = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
    SendCommand(ProgressStart);
    BuddyControlled = 0;
    for (int i = 0; i < 16; ++i) {
        BuddyControlled |= GetValue(fs[i]) << i;
        SendValue(Progress, i * (100 / 16));
    }
    SaveOneModel(ModelNumber);
    CloseModelsFile();
    SendCommand(page_BuddyView);
    CurrentView = BUDDYVIEW;
}
/******************************************************************************************************************************/
void RudderLeftTrim() { MoveaTrim(7); }
/******************************************************************************************************************************/
void RudderRightTrim() { MoveaTrim(6); }
/******************************************************************************************************************************/
void AileronRightTrim() { MoveaTrim(0); }
/******************************************************************************************************************************/
void AileronLeftTrim() { MoveaTrim(1); }
/******************************************************************************************************************************/
void ElevatorUpTrim()
{
    uint8_t tt = 2;
    if (SticksMode == 2) tt = 5;
    MoveaTrim(tt);
}
/******************************************************************************************************************************/
void ElevatorDownTrim()
{
    uint8_t tt = 3;
    if (SticksMode == 2) tt = 4;
    MoveaTrim(tt);
}
/******************************************************************************************************************************/
void ThrottleUpTrim()
{
    uint8_t tt = 4;
    if (SticksMode == 2) tt = 3;
    MoveaTrim(tt);
}
/******************************************************************************************************************************/
void ThrottleDownTrim()
{
    uint8_t tt = 5;
    if (SticksMode == 2) tt = 2;
    MoveaTrim(tt);
}
/******************************************************************************************************************************/
void ResetTransmitterSettings()
{ // This function resets all transmitter parameters to the default state.
  // Calibration should  be done next.

    const char Tn[32] = "Unknown";

    char prompt[]         = "Delete all settings and models?!";
    int  sofar            = 0;
    char ProgressStart[]  = "vis Progress,1";
    char Progress[]       = "Progress";
    char pCalibrateView[] = "page CalibrateView";

    if (!GetConfirmation(pCalibrateView, prompt)) return;
    SendCommand(ProgressStart);
    SendValue(Progress, 2);

    BuddyPupilOnPPM    = false;
    BuddyMaster        = false;
    ModelNumber        = 1;
    ScreenTimeout      = 120;
    Inactivity_Timeout = INACTIVITYTIMEOUT;
    strcpy(TxName, Tn);
    Qnh              = 1009;
    DeltaGMT         = 0;
    BackGroundColour = 214;
    ForeGroundColour = White;
    SpecialColour    = Red;
    HighlightColour  = Yellow;
    SticksMode       = 2;
    AudioVolume      = 20;
    Brightness       = 100;
    PlayFanfare      = false;
    TrimClicks       = true;
    ButtonClicks     = false;
    SpeakingClock    = true;
    AnnounceBanks    = true;
    ResetSwitchNumbers();
    MinimumGap        = 75;
    LogRXSwaps        = true;
    UseLog            = false;
    AnnounceConnected = true;
    ResetAllTrims();
    TxVoltageCorrection     = 0;
    PowerOffWarningSeconds  = DEFAULTPOWEROFFWARNING;
    LEDBrightness           = DEFAULTLEDBRIGHTNESS;
    ConnectionAssessSeconds = 1;
    AutoModelSelect         = false;
    MotorChannel            = 15;
    MotorChannelZero        = 0;
    TimerDownwards          = false;
    PPMdata.UseTXModule     = false;
    char ProgressEnd[]      = "vis Progress,0";

    SetDS1307ToCompilerTime();
    for (int k = 1; k < 5; ++k) { // writes default four times!
        for (ModelNumber = 1; ModelNumber <= MAXMODELNUMBER; ++ModelNumber) {
            ++sofar;
            SetDefaultValues();
            SendValue(Progress, (sofar * (100 / MAXMODELNUMBER)) / 4);
            KickTheDog();
        }
    }
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
    if (InterpolationTypes[Bank][ChanneltoSet - 1] == EXPONENTIALCURVES) {
        if ((CurrentPoint == 2) || (CurrentPoint == 4)) ++CurrentPoint;
    }
}

/******************************************************************************************************************************/

void PointSelect()
{
    ++CurrentPoint;
    if (CurrentPoint > 5) CurrentPoint = 1;
    CheckInvisiblePoint();
    DisplayCurve();
}

/******************************************************************************************************************************/

void ShowDualRateChannelsName(char* nm, uint8_t n)
{
    char nu[] = "Not used";

    if (n) {
        SendText(nm, ChannelNames[n - 1]);
    }
    else {
        SendText(nm, nu);
    } // not user if zero
}

/******************************************************************************************************************************/

void DisplayDualRateValues()
{
    char rate1[]     = "rate1";
    char rate2[]     = "rate2";
    char rate3[]     = "rate3";
    char ChName1[]   = "t12";
    char ChName2[]   = "t8";
    char ChName3[]   = "t11";
    char ChName4[]   = "t13";
    char ChName5[]   = "t16";
    char ChName6[]   = "t14";
    char ChName7[]   = "t15";
    char ChName8[]   = "t17";
    char ChNumber1[] = "n2";
    char ChNumber2[] = "n0";
    char ChNumber3[] = "n1";
    char ChNumber4[] = "n3";
    char ChNumber5[] = "n6";
    char ChNumber6[] = "n4";
    char ChNumber7[] = "n5";
    char ChNumber8[] = "n7";

    SendValue(rate1, Drate1);
    SendValue(rate2, Drate2);
    SendValue(rate3, Drate3);

    SendValue(ChNumber1, DualRateChannels[0]);
    SendValue(ChNumber2, DualRateChannels[1]);
    SendValue(ChNumber3, DualRateChannels[2]);
    SendValue(ChNumber4, DualRateChannels[3]);
    SendValue(ChNumber5, DualRateChannels[4]);
    SendValue(ChNumber6, DualRateChannels[5]);
    SendValue(ChNumber7, DualRateChannels[6]);
    SendValue(ChNumber8, DualRateChannels[7]);

    ShowDualRateChannelsName(ChName1, DualRateChannels[0]);
    ShowDualRateChannelsName(ChName2, DualRateChannels[1]);
    ShowDualRateChannelsName(ChName3, DualRateChannels[2]);
    ShowDualRateChannelsName(ChName4, DualRateChannels[3]);
    ShowDualRateChannelsName(ChName5, DualRateChannels[4]);
    ShowDualRateChannelsName(ChName6, DualRateChannels[5]);
    ShowDualRateChannelsName(ChName7, DualRateChannels[6]);
    ShowDualRateChannelsName(ChName8, DualRateChannels[7]);
}

/******************************************************************************************************************************/

void DualRatesStart()
{

    char GotoDualRates[] = "page DualRatesView";

    SendCommand(GotoDualRates);
    CurrentView = DUALRATESVIEW;
    DisplayDualRateValues();
    UpdateModelsNameEveryWhere();
}

/******************************************************************************************************************************/

void ReadDualRatesValues()
{
    char rate2[]         = "rate2";
    char rate3[]         = "rate3";
    char rate1[]         = "rate1";
    char ChNumber1[]     = "n2";
    char ChNumber2[]     = "n0";
    char ChNumber3[]     = "n1";
    char ChNumber4[]     = "n3";
    char ChNumber5[]     = "n6";
    char ChNumber6[]     = "n4";
    char ChNumber7[]     = "n5";
    char ChNumber8[]     = "n7";
    char ProgressStart[] = "vis Progress,1";
    char ProgressEnd[]   = "vis Progress,0";
    char Progress[]      = "Progress";
    SendCommand(ProgressStart);
    SendValue(Progress, 10);
    DelayWithDog(10);
    Drate1 = GetValue(rate1);
    if (Drate1 > MAXDUALRATE) Drate1 = MAXDUALRATE;
    Drate2 = GetValue(rate2);
    if (Drate2 > MAXDUALRATE) Drate2 = MAXDUALRATE;
    Drate3 = GetValue(rate3);
    if (Drate3 > MAXDUALRATE) Drate3 = MAXDUALRATE;
    SendValue(Progress, 50);
    DelayWithDog(10);
    DualRateChannels[0] = CheckRange(GetValue(ChNumber1), 0, 8);
    DualRateChannels[1] = CheckRange(GetValue(ChNumber2), 0, 8);
    DualRateChannels[2] = CheckRange(GetValue(ChNumber3), 0, 8);
    SendValue(Progress, 60);
    DelayWithDog(10);
    DualRateChannels[3] = CheckRange(GetValue(ChNumber4), 0, 8);
    DualRateChannels[4] = CheckRange(GetValue(ChNumber5), 0, 8);
    DualRateChannels[5] = CheckRange(GetValue(ChNumber6), 0, 8);
    SendValue(Progress, 75);
    DelayWithDog(10);
    DualRateChannels[6] = CheckRange(GetValue(ChNumber7), 0, 8);
    DualRateChannels[7] = CheckRange(GetValue(ChNumber8), 0, 8);
    SendValue(Progress, 100);
    DelayWithDog(10);
    SendCommand(ProgressEnd);
}
/******************************************************************************************************************************/

void DualRatesEnd()
{
    char GotoSticksView[] = "page SticksView";
    ReadDualRatesValues();
    SaveOneModel(ModelNumber);
    SendCommand(GotoSticksView);
    Force_ReDisplay();
    ShowServoPos();
    CurrentView = STICKSVIEW;
}

/******************************************************************************************************************************/

void DualRatesRefresh()
{

    ReadDualRatesValues();
    DisplayDualRateValues();
}

/******************************************************************************************************************************/

void GotoGPSView()
{

    char GotoGPSView[] = "page GPSView";
    SendCommand(GotoGPSView);
    CurrentView = GPSVIEW;
}

/******************************************************************************************************************************/

void StartBankNames()
{

    char GotoBankNames[] = "page BankNameView";
    char BKS[4][4]       = {{"BK1"}, {"BK2"}, {"BK3"}, {"BK4"}};

    SendCommand(GotoBankNames);
    CurrentView = BANKSNAMESVIEW;

    for (int i = 0; i < 4; ++i) {
        SendValue(BKS[i], BanksInUse[i]);
    }
    UpdateModelsNameEveryWhere();
}

/******************************************************************************************************************************/

void EndBankNames()
{

    char BKS[4][4] = {{"BK1"}, {"BK2"}, {"BK3"}, {"BK4"}};
    for (int i = 0; i < 4; ++i) {
        BanksInUse[i] = GetValue(BKS[i]);
    }
    SaveOneModel(ModelNumber);
    StartModelSetup();
}

/******************************************************************************************************************************/

void ListenToBanks()
{
    char BKS[4][4] = {{"BK1"}, {"BK2"}, {"BK3"}, {"BK4"}};
    for (int i = 0; i < 4; ++i) {
        BanksInUse[i] = GetValue(BKS[i]);
        PlaySound(BankSounds[BanksInUse[i]]);
        DelayWithDog(1200);
    }
}
/******************************************************************************************************************************/
void StartModelSetup()
{
    char GotoModelSetup[] = "page RXSetupView";
    char fs[16][5]        = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
    char ProgressStart[]  = "vis Progress,1";
    char ProgressEnd[]    = "vis Progress,0";
    char Progress[]       = "Progress";

    if (CurrentView == FAILSAFE_VIEW) { //  read failsafe blobs
        SendCommand(ProgressStart);
        for (int i = 0; i < 16; ++i) {
            FailSafeChannel[i] = GetValue(fs[i]);
            SendValue(Progress, i * (100 / 16));
        }
        SendCommand(ProgressEnd);
    }

    if (CurrentView == MIXESVIEW) { //  read mixes
        SaveMixValues();
        SendCommand(ProgressEnd);
    }

    b5isGrey  = false;
    b12isGrey = false;
    SendCommand(GotoModelSetup);
    CurrentView = RXSETUPVIEW;
    SaveOneModel(ModelNumber);
    UpdateModelsNameEveryWhere();
    CurrentMode  = NORMAL;
    LastTimeRead = 0;
}

/******************************************************************************************************************************/
void EndModelSetup()
{

    GotoFrontView();
}

/******************************************************************************************************************************/
void StartSlowView()
{

    char GoSlowServoScreen[] = "page SlowServoView";
    char ns[16][4]           = {{"n0"}, {"n1"}, {"n2"}, {"n3"}, {"n4"}, {"n5"}, {"n6"}, {"n7"}, {"n8"}, {"n9"}, {"n10"}, {"n11"}, {"n12"}, {"n13"}, {"n14"}, {"n15"}};
    SendCommand(GoSlowServoScreen);
    CurrentView = SLOWSERVOVIEW;
    UpdateButtonLabels();
    UpdateModelsNameEveryWhere();
    for (int i = 0; i < 16; ++i) {
        SendValue(ns[i], StepSize[i]);
    }
}
/******************************************************************************************************************************/
void EndSlowView()
{
    char ns[16][4]       = {{"n0"}, {"n1"}, {"n2"}, {"n3"}, {"n4"}, {"n5"}, {"n6"}, {"n7"}, {"n8"}, {"n9"}, {"n10"}, {"n11"}, {"n12"}, {"n13"}, {"n14"}, {"n15"}};
    char ProgressStart[] = "vis Progress,1";
    char Progress[]      = "Progress";
    SendCommand(ProgressStart);
    for (int i = 0; i < 16; ++i) {
        StepSize[i] = GetValue(ns[i]);
        SendValue(Progress, i * (100 / 16));
    }
    CheckStepSizes();
    SaveOneModel(ModelNumber);
    SendValue(Progress, 100);
    StartModelSetup();
}
/******************************************************************************************************************************/
void WriteNewCurve()
{
    char CopyToAllBanks[]  = "callfm";
    char page_SticksView[] = "page SticksView";

    if (GetValue(CopyToAllBanks)) {
        for (int p = 1; p <= 4; ++p) {
            if (p != Bank) {
                MinDegrees[p][ChanneltoSet - 1]         = MinDegrees[Bank][ChanneltoSet - 1];
                MidLowDegrees[p][ChanneltoSet - 1]      = MidLowDegrees[Bank][ChanneltoSet - 1];
                CentreDegrees[p][ChanneltoSet - 1]      = CentreDegrees[Bank][ChanneltoSet - 1];
                MidHiDegrees[p][ChanneltoSet - 1]       = MidHiDegrees[Bank][ChanneltoSet - 1];
                MaxDegrees[p][ChanneltoSet - 1]         = MaxDegrees[Bank][ChanneltoSet - 1];
                InterpolationTypes[p][ChanneltoSet - 1] = InterpolationTypes[Bank][ChanneltoSet - 1];
                Exponential[p][ChanneltoSet - 1]        = Exponential[Bank][ChanneltoSet - 1];
            }
        }
    }
    ChanneltoSet = 0;
    SaveOneModel(ModelNumber);
    Force_ReDisplay();
    SendCommand(page_SticksView); // Set to SticksView
    CurrentView = STICKSVIEW;
    UpdateModelsNameEveryWhere();
    ClearText();
}

/******************************************************************************************************************************/
void StartRenameModel()
{

    char GoRenameModelScreen[] = "page RenameView";
    char NewName[]             = "NewName";
    SendCommand(GoRenameModelScreen);
    CurrentView = RENAMEMODELVIEW;
    SendText(NewName, ModelName);
}

/******************************************************************************************************************************/

void GetDefaultFilename()
{ // Build filename from ModelName as best we can using first 8 chars upper cased
    uint8_t j     = 0;
    uint8_t i     = 0;
    char    mod[] = ".MOD";
    while (i < 8) {
        if (ModelName[j] > 32) {
            SingleModelFile[i] = toUpperCase(ModelName[j]);
            ++i;
            SingleModelFile[i] = 0;
        }
        if (ModelName[j] < 32) break;
        ++j;
        if (i >= 8) break;
    }
    strcat(SingleModelFile, mod);
}

/******************************************************************************************************************************/

void FixFileName()
{
    char ModExt[] = ".MOD";
    for (uint8_t i = 0; i < strlen(SingleModelFile); ++i) SingleModelFile[i] = toUpperCase(SingleModelFile[i]);
    if (!InStrng(ModExt, SingleModelFile) && (strlen(SingleModelFile) <= 8)) strcat(SingleModelFile, ModExt);
}

/******************************************************************************************************************************/
void WriteBackup()
{

    char    ModExt[]        = ".MOD";
    uint8_t Iterations      = 4;
    char    ProgressStart[] = "vis Progress,1";
    char    ProgressEnd[]   = "vis Progress,0";
    char    Progress[]      = "Progress";
    SendValue(Progress, 1);
    FixFileName();
    if ((strlen(SingleModelFile) <= 12) && (InStrng(ModExt, SingleModelFile) > 0)) {
        SendCommand(ProgressStart);
        for (int i = 0; i < Iterations; i++) {
            CloseModelsFile();
            SingleModelFlag = true;
            SaveOneModel(1);
            SendValue(Progress, i * (100 / Iterations)); // write several times is needed!! :-)
        }
        CloseModelsFile();
        SingleModelFlag = false;
    }
    else {
        FileError = true;
    }
    if (FileError) ShowFileErrorMsg();
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
bool GetBackupFilename(char* goback, char* tt1, char* MMname, char* heading, char* pprompt)
{ // HERE THE USER CAN REPLACE DEFAULT FILENAME IF HE WANTS TO

    char GoBackupView[] = "page BackupView";
    char t0[]           = "t0";        // prompt
    char t1[]           = "t1";        // default filename
    char t3[]           = "t3";        // heading
    char Mname[]        = "Modelname"; // model name

    SendCommand(GoBackupView);

    SendText(t0, pprompt);   // prompt
    SendText(t1, tt1);       // filename
    SendText(Mname, MMname); // Model name
    SendText(t3, heading);   // heading

    Confirmed[0] = '?';
    while (Confirmed[0] == '?') { // await user response
        CheckForNextionButtonPress();
        KickTheDog();
    }
    GetText(t1, SingleModelFile);
    SendCommand(goback);
    if (Confirmed[0] == 'Y') return true;
    return false;
}
/******************************************************************************************************************************/
// Gets Windows style confirmation (FILE OVERWRITE ETC.)
// params:
// Prompt is the prompt
// goback is the command needed to return to calling page

bool GetConfirmation(char* goback, char* Prompt)
{

    char GoPopupView[] = "page PopupView";
    char Dialog[]      = "Dialog";

    SendCommand(GoPopupView);
    SendText(Dialog, Prompt);
    Confirmed[0] = '?';
    while (Confirmed[0] == '?') { // await user response
        CheckForNextionButtonPress();
        KickTheDog();
    }
    SendCommand(goback);
    LastFileInView = 120;
    if (Confirmed[0] == 'Y') return true; // tell caller OK to continue
    return false;                         // tell caller STOP!
}
/******************************************************************************************************************************/
void YesPressed() { Confirmed[0] = 'Y'; }
/******************************************************************************************************************************/
void NoPressed() { Confirmed[0] = 'N'; }
/******************************************************************************************************************************/

void SaveCurrentModel()
{
    SavedModelNumber = ModelNumber;
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
    ModelNumber     = SavedModelNumber;
    SingleModelFlag = false;
    ReadOneModel(ModelNumber);
    SaveAllParameters();
}

/******************************************************************************************************************************/

// This implements the impossible "SD card rename file" ... by reading, re-saveing under new name, then deleting old file.

void RenameFile()
{
    char ModelsView_filename[] = "filename";
    char Head[]                = "Rename this backup";
    char model[]               = "(i.e. just change its filename)";
    char prompt[]              = "New filename?";
    char Prompt[50];
    char overwr[] = "Overwrite ";
    char ques[]   = "?";
    char Deleteable[42];
    char GoModelsView[] = "page ModelsView";

    SaveCurrentModel();
    GetText(ModelsView_filename, SingleModelFile);
    strcpy(Deleteable, SingleModelFile);
    LoadModelForRenaming();
    if (GetBackupFilename(GoModelsView, SingleModelFile, model, Head, prompt)) {
        FixFileName();
        if (strcmp(Deleteable, SingleModelFile) == 0) return;
        Serial.println(SingleModelFile);
        if (CheckFileExists(SingleModelFile)) {
            strcpy(Prompt, overwr);
            strcat(Prompt, SingleModelFile);
            strcat(Prompt, ques);
            if (GetConfirmation(GoModelsView, Prompt))
            {
                WriteBackup();
                SD.remove(Deleteable);
            }
        }
        else {
            WriteBackup();
            SD.remove(Deleteable);
        }
    }
    BuildDirectory();
    LoadFileSelector();
    RestoreCurrentModel();
}

/******************************************************************************************************************************/

void ModelViewEnd()
{

    char pr[] = "Select ";
    char buf[60];
    char q[]            = "?";
    char GoModelsView[] = "page ModelsView";
    if (PreviousModelNumber != ModelNumber) {
        strcpy(buf, pr);
        strcat(buf, ModelName);
        strcat(buf, q);
        GetConfirmation(GoModelsView, buf);
        if (Confirmed[0] != 'Y') {
            ModelNumber = PreviousModelNumber;
            ReadOneModel(ModelNumber);
        }
    }
    SaveAllParameters();
    BoundFlag = false;
    GotoFrontView();
}

/******************************************************************************************************************************/

void DoMFName()
{
    DelayWithDog(100);
    CheckModelName();
}

/******************************************************************************************************************************/
// Function to display all model IDs and check for duplcates

void CheckAllModelIds()
{
    unsigned int TempModelId;
    char         Vbuf[15];
    char         MMemsp[]   = "MMems.path=\"";
    char         GoIDview[] = "page IDsView";
    char         crlf[]     = {13, 10, 0};
    char         lb[]       = "(";
    char         rb[]       = ")  ";
    char         KO[]       = ">Duplicate:";
    char         Okay[]     = " (ID OK)";
    char         n0[]       = "n0";
    char         nb[4];
    char         buf[MAXBUFFERSIZE];
    uint64_t     ModelIDs[92];
    uint8_t      DuplicatesCount  = 0;
    uint32_t     SavedModelNumber = ModelNumber;

    SendCommand(GoIDview);
    CurrentView = IDCHECKVIEW;
    CurrentMode = SENDNOTHING;

    for (ModelNumber = 1; ModelNumber < MAXMODELNUMBER - 1; ++ModelNumber) {
        ReadOneModel(ModelNumber);
        ModelIDs[ModelNumber] = ModelsMacUnionSaved.Val64;
    }
    for (ModelNumber = 1; ModelNumber < MAXMODELNUMBER - 1; ++ModelNumber) {
        ReadOneModel(ModelNumber);
        if (ModelNumber == 1) {
            strcpy(buf, lb);
        }
        else {
            strcat(buf, lb);
        }
        Str(nb, ModelNumber, 0);
        strcat(buf, nb);
        strcat(buf, rb);
        strcat(buf, ModelName);
        TempModelId = ModelsMacUnionSaved.Val32[0];
        snprintf(Vbuf, 10, "%X", TempModelId);
        if (TempModelId) {
            strcat(buf, " ");
            strcat(buf, Vbuf);
        }
        TempModelId           = ModelsMacUnionSaved.Val32[1];
        ModelIDs[ModelNumber] = ModelsMacUnionSaved.Val64;
        snprintf(Vbuf, 10, "%X", TempModelId);
        if (TempModelId) {
            strcat(buf, Vbuf);
            int p = 0;
            for (unsigned int i = 1; i < MAXMODELNUMBER - 1; ++i) {
                if ((ModelIDs[i] == ModelIDs[ModelNumber]) && (i != ModelNumber)) p = i;
            }
            if (p > 0) {
                strcat(buf, KO);
                strcat(buf, Str(nb, p, 0));
                strcat(buf, "<");
                ++DuplicatesCount;
            }
            else {
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

/******************************************************************************************************************************/

void TXModuleViewStart()
{

    char GoTXModule[] = "page TXModuleView";
    if (ModelMatched) return;

    CurrentView = TXMODULEVIEW;

    char c1[] = "c1"; // Use module
    char n3[] = "n3"; // number of channels
    char r0[] = "r0";
    char r1[] = "r1";
    char r2[] = "r2";

    SendCommand(GoTXModule);
    SendValue(c1, PPMdata.UseTXModule);
    SendValue(n3, PPMdata.PPMChannelsNumber);
    if (PPMdata.PPMOrderSelection == 1) {
        SendValue(r0, 1);
    }
    else {
        SendValue(r0, 0);
    }
    if (PPMdata.PPMOrderSelection == 2) {
        SendValue(r1, 1);
    }
    else {
        SendValue(r1, 0);
    }
    if (PPMdata.PPMOrderSelection == 3) {
        SendValue(r2, 1);
    }
    else {
        SendValue(r2, 0);
    }
}

/******************************************************************************************************************************/

void SelectChannelOrder()
{

    if (PPMdata.PPMOrderSelection == 1) PPMdata.PPMChannelOrder = PPMdata.PPMChannelOrder1;
    if (PPMdata.PPMOrderSelection == 2) PPMdata.PPMChannelOrder = PPMdata.PPMChannelOrder2;
    if (PPMdata.PPMOrderSelection == 3) PPMdata.PPMChannelOrder = PPMdata.PPMChannelOrder3;
}

/******************************************************************************************************************************/

void TXModuleViewEnd()
{

    char page_SetupView[] = "page TXSetupView";
    char GoBack[]         = "page TXModuleView";
    char c1[]             = "c1"; // Use module
    char n3[]             = "n3"; // number of channels
    char r0[]             = "r0";
    char r1[]             = "r1";
    char r2[]             = "r2";
    char prompt[]         = "Power off transmitter?";
    bool oldUseTxModule   = PPMdata.UseTXModule;
    char ProgressStart[]  = "vis Progress,1";
    char Progress[]       = "Progress";

    SendCommand(ProgressStart);
    SendValue(Progress, 10);
    PPMdata.UseTXModule = GetValue(c1);
    if (PPMdata.UseTXModule != oldUseTxModule) {
        if (!GetConfirmation(GoBack, prompt)) {
            PPMdata.UseTXModule = oldUseTxModule;
            SendValue(c1, PPMdata.UseTXModule);
            return;
        }
    }
    SendValue(Progress, 30);
    DelayWithDog(100);
    PPMdata.PPMChannelsNumber = GetValue(n3);
    SendValue(Progress, 51);
    if (GetValue(r0)) PPMdata.PPMOrderSelection = 1;
    SendValue(Progress, 63);
    DelayWithDog(10);
    if (GetValue(r1)) PPMdata.PPMOrderSelection = 2;
    SendValue(Progress, 88);
    DelayWithDog(10);
    if (GetValue(r2)) PPMdata.PPMOrderSelection = 3;
    SelectChannelOrder();
    SendValue(Progress, 99);
    DelayWithDog(10);
    SaveTransmitterParameters();
    DelayWithDog(10);
    SendCommand(page_SetupView);
    CurrentView = TXSETUPVIEW;
    DelayWithDog(10);
    if (PPMdata.UseTXModule != oldUseTxModule) {
        digitalWrite(POWER_OFF_PIN, HIGH);
    }
}

/******************************************************************************************************************************/

// ******************************** Global Array of numbered function pointers - OK up to 127 functions ... **********************************
#define LASTFUNCTION 71 // One more than final one, because first is number zero

void (*NumberedFunctions[LASTFUNCTION])() {
    Blank,                    // 0
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
    UpLog,                    // 13
    DownLog,                  // 14
    RefreshLog,               // 15
    LogEND,                   // 16
    DelLOG,                   // 17
    LogVIEW,                  // 18
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
    RXSetup1End,              // 39
    ResetTransmitterSettings, // 40
    BindNow,                  // 41
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
    EndSlowView,              // 56
    WriteNewCurve,            // 57
    StartRenameModel,         // 58
    EndRenameModel,           // 59
    YesPressed,               // 60
    NoPressed,                // 61
    RenameFile,               // 62
    CheckAllModelIds,         // 63
    Blank,                    // 64
    ReceiveModelFile,         // 65
    TXModuleViewStart,        // 66
    TXModuleViewEnd,          // 67
    DeleteModelID,            // 68
    StartPong,                // 69
    StoreModelID              // 70

}; // list will become longer ...

/*********************************************************************************************************************************
 *                          BUTTON WAS PRESSED (DEAL WITH INPUT FROM NEXTION DISPLAY)                                            *
 *********************************************************************************************************************************/
FASTRUN void ButtonWasPressed()
{
    if (strlen(TextIn) > 0) {
        StartInactvityTimeout();

        ScreenTimeTimer = millis(); // reset screen timeout counter
        if (ScreenIsOff) {
            RestoreBrightness();
            ScreenIsOff = false;
            ClearText();
            return;
        }

        union
        {
            uint8_t  First4Bytes[4];
            uint32_t FirstDWord;
        } NextionCommand;
        NextionCommand.First4Bytes[0] = TextIn[0];
        NextionCommand.First4Bytes[1] = TextIn[1];
        NextionCommand.First4Bytes[2] = 0; //   TextIn[2];
        NextionCommand.First4Bytes[3] = 0; //   TextIn[3];
        uint32_t NumberedCommand      = NextionCommand.FirstDWord - 128;

#ifdef DB_NEXTION
        if (TextIn[0] < 128) {
            Serial.print("Command WORD: -> ");
            Serial.println(TextIn);
        }
        else {
            Serial.print("Command NUMBER: -> ");
            Serial.println(NumberedCommand);
        }

#endif

        if (TextIn[0] >= 128) {                       // (First byte != printable char) indicates a numbered command...
            if (NumberedCommand < LASTFUNCTION) {     // ...so this system currently only permits 127 numbered functions to be supported.
                                                      // But that's OK! Because only about 110 still need converting, after which
                                                      // the restriction can be removed and the max will then be a full 32 bit value.
                                                      // **********************************************************************************************************************************
                NumberedFunctions[NumberedCommand](); // Call the needed function -- with a function pointer                     *
                                                      // **********************************************************************************************************************************
                b5isGrey  = false;
                b12isGrey = false;
                ClearText();
                return;
            }
        }
        int  i                         = 0;
        int  j                         = 0;
        int  p                         = 0;
        char lpm[]                     = "c0"; // Auto model selection
        char Setup[]                   = "Setup";
        char ClickX[]                  = "ClickX";
        char ClickY[]                  = "ClickY";
        char Reset[]                   = "Reset";
        char Front_View[]              = "FrontView";
        char Sticks_View[]             = "SticksView";
        char Graph_View[]              = "GraphView";
        char SetupView[]               = "MainSetup";
        char Scan_End[]                = "ScanEnd";
        char DataEnd[]                 = "DataEnd";
        char Data_View[]               = "DataView";
        char CalibrateView[]           = "CalibrateView";
        char TrimView[]                = "TrimView";
        char TRIMS50[]                 = "TRIMS50";
        char MIXES_VIEW[]              = "MIXESVIEW"; // first call
        char Mixes_View[]              = "MixesView";
        char Fhss_View[]               = "FhssView";
        char FM1[]                     = "FM 1";
        char FM2[]                     = "FM 2";
        char FM3[]                     = "FM 3";
        char FM4[]                     = "FM 4";
        char ReScan[]                  = "ReScan";
        char Delete[]                  = "Delete";
        char MixesView_MixNumber[]     = "MixNumber";
        char ModelsView_ModelNumber[]  = "ModelNumber";
        char page_SticksView[]         = "page SticksView";
        char page_GraphView[]          = "page GraphView";
        char page_SetupView[]          = "page TXSetupView";
        char page_AudioView[]          = "page AudioView";
        char page_ColoursView[]        = "page ColoursView";
        char GoSetupView[]             = "GoSetupView";
        char ColoursView[]             = "ColoursView";
        char SvT11[]                   = "t11";
        char CMsg1[]                   = "Move all controls to their full\r\nextent several times,\r\nthen press Next.";
        char SvB0[]                    = "b0";
        char CMsg2[]                   = "Next ...";
        char Cmsg3[]                   = "Centre all channels.\r\nPut edge switches fully back,\r\nor fully forward, then press Finish.";
        char Cmsg4[]                   = "Finish";
        char Cmsg5[]                   = "Repeat?";
        char Cmsg6[]                   = "Calbrate again?";
        char TypeView[]                = "TypeView";
        char CopyToAllBanks[]          = "callfm";
        char RXBAT[]                   = "RXBAT";
        char r2s[]                     = "r2s";
        char r3s[]                     = "r3s";
        char r4s[]                     = "r4s";
        char r5s[]                     = "r5s";
        char r6s[]                     = "r6s";
        char r12s[]                    = "r12s";
        char r0[]                      = "r0";
        char r1[]                      = "r1";
        char SwitchesView[]            = "SwitchesView";
        char SwitchesView1[]           = "SwitchesView1";
        char OneSwitchView[]           = "OneSwitchView";
        char PageOneSwitchView[]       = "page OneSwitchView";
        char InputsView[]              = "InputsView";
        char InputsDone[]              = "InputsDone";
        char InputStick_Labels[16][4]  = {"c1", "c2", "c3", "c4", "c5", "c6", "c7", "c8", "c9", "c10", "c11", "c12", "c13", "c14", "c15", "c16"};
        char OutputStick_Labels[16][4] = {"n4", "n5", "n6", "n7", "n8", "n9", "n10", "n11", "n12", "n13", "n14", "n15", "n16", "n17", "n18", "n19"};
        char InputTrim_labels[4][4]    = {"n0", "n1", "n2", "n3"};
        char Export[]                  = "Export";
        char Import[]                  = "Import";
        char DelFile[]                 = "DelFile";
        char ModExt[]                  = ".MOD";
        char FailSAVE[]                = "FailSAVE";
        char FailSafe[]                = "FailSafe";
        char fs[16][5]                 = {"fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15", "fs16"};
        char CH1NAME[]                 = "CH1NAME=";
        char CH2NAME[]                 = "CH2NAME=";
        char b17[]                     = "b17";
        char CH3NAME[]                 = "CH3NAME=";
        char CH4NAME[]                 = "CH4NAME=";
        char CH5NAME[]                 = "CH5NAME=";
        char CH6NAME[]                 = "CH6NAME=";
        char CH7NAME[]                 = "CH7NAME=";
        char CH8NAME[]                 = "CH8NAME=";
        char CH9NAME[]                 = "CH9NAME=";
        char CH10NAME[]                = "CH10NAME=";
        char CH11NAME[]                = "CH11NAME=";
        char CH12NAME[]                = "CH12NAME=";
        char CH13NAME[]                = "CH13NAME=";
        char CH14NAME[]                = "CH14NAME=";
        char CH15NAME[]                = "CH15NAME=";
        char CH16NAME[]                = "CH16NAME=";
        char HelpView[]                = "HelpView";
        char SendModel[]               = "SendModel";
        char OptionsViewS[]            = "OptionsViewS";
        char Pto[]                     = "Pto";
        char Tx_Name[]                 = "TxName";
        char Exrite[]                  = "Exrite";
        char ExpR[]                    = "Exp";
        char Smooth[]                  = "Smooth";
        char Lines[]                   = "Lines";
        char GOTO[]                    = "GOTO:";
        char WhichPage[]               = "page                                 "; // excessive spaces for page name
        char AddMinute[]               = "IncMinute";
        char Dec_Minute[]              = "DecMinute";
        char Dec_Hour[]                = "DecHour";
        char Inc_Hour[]                = "IncHour";
        char Inc_Year[]                = "IncYear";
        char Dec_Year[]                = "DecYear";
        char Inc_Date[]                = "IncDate";
        char Dec_Date[]                = "DecDate";
        char Inc_Month[]               = "IncMonth";
        char Dec_Month[]               = "DecMonth";
        char pDataView[]               = "page DataView";
        char pSwitchesView[]           = "page SwitchesView";
        char pInputsView[]             = "page InputsView";
        char pOptionsViewS[]           = "page OptionsView"; // TX options view
        char pMixesView[]              = "page MixesView";
        char pTypeView[]               = "page TypeView";
        char pFailSafe[]               = "page FailSafeView";
        char DataView_Clear[]          = "Clear";
        char DataView_AltZero[]        = "AltZero";
        char OptionsEnd[]              = "OptionsEnd";
        char Mark[]                    = "Mark";
        char UKRULES[]                 = "UKRULES";
        char Htext0[]                  = "HELP";
        char Htext1[]                  = "Help";
        char Bwn[]                     = "Bwn";
        char SetupCol[]                = "SetupCol";
        char b0_bco[]                  = "b0.bco";
        char b0_pco[]                  = "b0.pco";
        char High_pco[]                = "High.pco";
        char Fm_pco[]                  = "Fm.pco";
        char FrontView_BackGround[]    = "FrontView.BackGround";
        char FrontView_ForeGround[]    = "FrontView.ForeGround";
        char FrontView_Special[]       = "FrontView.Special";
        char FrontView_Highlight[]     = "FrontView.Highlight";
        char SetupAud[]                = "SetupAud";
        char n0[]                      = "n0";
        char Ex1[]                     = "Ex1";
        char Expo[]                    = "Expo";
        char AudioView[]               = "AudioView";
        char n1[]                      = "n1";
        char h0[]                      = "h0";
        char c0[]                      = "c0";
        char c1[]                      = "c1";
        char c2[]                      = "c2";
        char c3[]                      = "c3";
        char c4[]                      = "c4";
        char c5[]                      = "c5";
        char StCH[]                    = "StCH";
        char s0[]                      = "s0";
        char t2[]                      = "t2";
        char StEDIT[]                  = "StEDIT";
        char pLogView[]                = "page LogView";
        char dGMT[]                    = "dGMT";
        char TxNme[]                   = "TxName";
        char MMems[]                   = "MMems";
        char Prompt[60];
        char del[]               = "Delete ";
        char overwr[]            = "Overwrite ";
        char ques[]              = "?";
        char hhead[]             = "Create backup file for";
        char fprompt[]           = "Filename?";
        char page_RXSetupView[]  = "page RXSetupView";
        char ProgressStart[]     = "vis Progress,1";
        char ProgressEnd[]       = "vis Progress,0";
        char Progress[]          = "Progress";
        char ScreenViewTimeout[] = "Sto"; // needed for display info
        char page_FhssView[]     = "page FhssView";
        char FrontView_Hours[]   = "Hours";
        char FrontView_Mins[]    = "Mins";
        char FrontView_Secs[]    = "Secs";
        char StartBackGround[]   = "click Background,0";
        char GoModelsView[]      = "page ModelsView";
        char pCalibrateView[]    = "page CalibrateView";
        // ************************* test many input words from Nextion *****************

        if (InStrng(StCH, TextIn)) { // select sub trim channel
            SubTrimToEdit = GetValue(s0);
            SendText(t2, ChannelNames[SubTrimToEdit]);
            SendValue(n0, SubTrims[SubTrimToEdit] - 127);
            SendValue(h0, SubTrims[SubTrimToEdit]);
            ClearText();
            return;
        }

        if (InStrng(StEDIT, TextIn)) {                    // edit sub trim value
            SubTrims[SubTrimToEdit] = GetValue(n0) + 127; // 127 is mid point in 8 bit value 0 - 254
            ClearText();
            return;
        }

        if (InStrng(Delete, TextIn) > 0) {
            strcpy(Prompt, del);
            strcat(Prompt, ModelName);
            strcat(Prompt, ques);
            if (GetConfirmation(GoModelsView, Prompt)) {
                ModelNumber = GetValue(MMems) + 1;
                SetDefaultValues();
                LoadModelSelector();
            }
            ClearText();
            return;
        }

        if (InStrng(AudioView, TextIn) > 0) { // Display screen with audio options
            CurrentMode = NORMAL;
            CurrentView = AUDIOVIEW;
            SendCommand(page_AudioView);
            SendValue(n0, AudioVolume);
            SendValue(Ex1, AudioVolume);
            SendValue(n1, Brightness);
            SendValue(h0, Brightness);
            SendValue(c0, PlayFanfare);
            SendValue(c1, TrimClicks);
            SendValue(c2, ButtonClicks);
            SendValue(c3, SpeakingClock);
            SendValue(c4, AnnounceBanks);
            SendValue(c5, AnnounceConnected);
            SetAudioVolume(AudioVolume);
            RestoreBrightness();
            ClearText();
            return;
        }
        if (InStrng(SetupAud, TextIn) > 0) { // Exit from screen with audio options
            CurrentMode       = NORMAL;
            AudioVolume       = GetValue(n0);
            Brightness        = GetValue(n1);
            PlayFanfare       = GetValue(c0);
            TrimClicks        = GetValue(c1);
            ButtonClicks      = GetValue(c2);
            SpeakingClock     = GetValue(c3);
            AnnounceBanks     = GetValue(c4);
            AnnounceConnected = GetValue(c5);
            RestoreBrightness();
            SetAudioVolume(AudioVolume);
            CurrentView = TXSETUPVIEW;
            SendCommand(page_SetupView);
            LastTimeRead = 0;
            SaveTransmitterParameters();
            UpdateModelsNameEveryWhere();
            b5isGrey  = false;
            b12isGrey = false;
            ClearText();
            return;
        }
        if (InStrng(SetupView, TextIn) > 0) { //  goto main setup screen
            ClearText();
            SaveAllParameters();
            CurrentView = TXSETUPVIEW;
            SendCommand(page_SetupView);
            LastTimeRead = 0;
            CurrentMode  = NORMAL;
            CurrentView  = TXSETUPVIEW;
            b5isGrey     = false;
            b12isGrey    = false;
            ClearText();
            UpdateModelsNameEveryWhere();
            return;
        }
        if (InStrng(Mark, TextIn) > 0) {
            GPSMarkHere    = 255; // Mark this location
            GPSMaxDistance = 0;
            ClearText();
            return;
        }
        if (InStrng(UKRULES, TextIn) > 0) { // UK Offcom regulations?
            ++FHSS_data::UkRulesCounter;
            if (FHSS_data::UkRulesCounter == 1) SwapWaveBandTimer = millis();
            if (FHSS_data::UkRulesCounter == 3) {

                if ((millis() - SwapWaveBandTimer) < 5000) { // pressed three times in under 5 seconds?!
                    if (!FHSS_data::UkRules) {
                        SwapWaveBand       = 1;
                        FHSS_data::UkRules = true;
                        SendText(b17, Htext1);
                    }
                    else {
                        SwapWaveBand       = 2;
                        FHSS_data::UkRules = false;
                        SendText(b17, Htext0);
                    }
                }
                FHSS_data::UkRulesCounter = 0;
            }
            ClearText();
            return;
        }

        if (InStrng(OptionsEnd, TextIn) > 0) { // Exit from TX Options screen1
            SendCommand(ProgressStart);
            SendValue(Progress, 10);
            SticksMode = CheckRange(GetValue(n0), 1, 2);
            GetText(TxNme, TxName);
            SendValue(Progress, 30);

            SendValue(Progress, 40);
            AutoModelSelect = GetValue(lpm);
            SendValue(Progress, 50);
            LowBattery = GetValue(Bwn);
            SendValue(Progress, 60);
            ScreenTimeout = GetValue(ScreenViewTimeout);
            SendValue(Progress, 70);
            SendValue(Progress, 80);
            Inactivity_Timeout = GetValue(Pto) * TICKSPERMINUTE;
            if (Inactivity_Timeout < INACTIVITYMINIMUM) Inactivity_Timeout = INACTIVITYMINIMUM;
            if (Inactivity_Timeout > INACTIVITYMAXIMUM) Inactivity_Timeout = INACTIVITYMAXIMUM;
            SendValue(Progress, 90);
            FixDeltaGMTSign();
            if (BuddyPupilOnPPM)
            {
                Connected            = false;
                LostContactFlag      = true;
                PacketsPerSecond     = 0;
                RangeTestGoodPackets = 0;
                BlueLedOn();
            }
            SaveAllParameters();
            SendValue(Progress, 95);
            SendValue(Progress, 100);
            CurrentView = TXSETUPVIEW;
            SendCommand(page_SetupView);
            LastTimeRead = 0;
            CurrentMode  = NORMAL;
            b5isGrey     = false;
            b12isGrey    = false;
            SendCommand(ProgressEnd);
            UpdateModelsNameEveryWhere();
            ClearText();
            ConfigureStickMode();
            return;
        }

        if (InStrng(DataEnd, TextIn) > 0) { //  Exit from Data screen
            SendCommand(page_RXSetupView);
            CurrentView = RXSETUPVIEW;
            b5isGrey    = false;
            b12isGrey   = false;
            CurrentMode = NORMAL;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }
        if (InStrng(DataView_Clear, TextIn) > 0) { //  Clear Data screen
            ZeroDataScreen();
            ClearText();
            return;
        }
        if (InStrng(DataView_AltZero, TextIn) > 0) { //  Set zero altitude on data screen
            if (!GroundModelAltitude) {
                GroundModelAltitude = RXModelAltitude;
            }
            else {
                GroundModelAltitude = 0;
            }
            if (!GPSGroundAltitude) {
                GPSGroundAltitude = GPSAltitude;
            }
            else {
                GPSGroundAltitude = 0;
            }
            GPSMaxaltitude     = 0;
            RXMAXModelAltitude = 0;
            ClearText();
            return;
        }

        if (InStrng(Scan_End, TextIn) > 0) { //  goto setup screen from Scan screen
            CurrentView = TXSETUPVIEW;
            b5isGrey    = false;
            b12isGrey   = false;
            SendCommand(page_SetupView);
            LastTimeRead = 0;
            DoScanEnd();
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(HelpView, TextIn) > 0) { // Display Help screen(s)
            SavedCurrentView = CurrentView;
            CurrentView      = HELP_VIEW;
            SendHelp();
            ClearText();
            return;
        }
        if (InStrng(Dec_Minute, TextIn) > 0) {
            DecMinute();
            ClearText();
            return;
        }
        if (InStrng(AddMinute, TextIn) > 0) {
            IncMinute();
            ClearText();
            return;
        }

        if (InStrng(Dec_Hour, TextIn) > 0) {
            DecHour();
            ClearText();
            return;
        }
        if (InStrng(Inc_Hour, TextIn) > 0) {
            IncHour();
            ClearText();
            return;
        }

        if (InStrng(Dec_Year, TextIn) > 0) {
            DecYear();
            ClearText();
            return;
        }
        if (InStrng(Inc_Year, TextIn) > 0) {
            IncYear();
            ClearText();
            return;
        }

        if (InStrng(Dec_Date, TextIn) > 0) {
            DecDate();
            ClearText();
            return;
        }
        if (InStrng(Inc_Date, TextIn) > 0) {
            IncDate();
            ClearText();
            return;
        }

        if (InStrng(Dec_Month, TextIn) > 0) {
            DecMonth();
            ClearText();
            return;
        }
        if (InStrng(Inc_Month, TextIn) > 0) {
            IncMonth();
            ClearText();
            return;
        }

        if (InStrng(OptionsViewS, TextIn) > 0) { // start tx options screen 1
            FixDeltaGMTSign();
            if (CurrentView == OPTIONVIEW2) DeltaGMT = GetValue(dGMT);
            SendCommand(pOptionsViewS); // TX options view
            SendValue(n0, SticksMode);
            SendValue(ScreenViewTimeout, ScreenTimeout);
            SendValue(Pto, (Inactivity_Timeout / TICKSPERMINUTE));
            SendText(Tx_Name, TxName);
            SendValue(lpm, AutoModelSelect);
            SendValue(Bwn, LowBattery);
            CurrentView = OPTIONS_VIEW;
            CurrentMode = NORMAL;
            ClearText();
            return;
        }
        if (InStrng(GOTO, TextIn) > 0) { // Return from Help screen returns here to relevent config screen
            i = 5;
            while (uint8_t(TextIn[i]) && i < 30) {
                WhichPage[i] = TextIn[i];
                ++i;
                WhichPage[i] = 0;
            }                       // Get page name to which to return
            SendCommand(WhichPage); // this sends nextion back to last screen
                                    // Look(WhichPage); // heer ********************************
            CurrentView = SavedCurrentView;

            if (CurrentView == PONGVIEW) StartPong();

            if ((CurrentView == RXSETUPVIEW) || (CurrentView == TXSETUPVIEW)) {
                b5isGrey  = false;
                b12isGrey = false;
            }

            if (CurrentView == GRAPHVIEW) {
                DisplayCurveAndServoPos();
                SendValue(CopyToAllBanks, 0);
            }
            if (CurrentView == SWITCHES_VIEW) {
                UpdateSwitchesView();
            }
            if (CurrentView == ONE_SWITCH_VIEW) {
                UpdateOneSwitchView();
            }
            if (CurrentView == MODELSVIEW) {
                SendValue(ModelsView_ModelNumber, ModelNumber);
            }
            if (CurrentView == MIXESVIEW) {
                if (MixNumber == 0) MixNumber = 1;
                LastMixNumber = 33;                        // just to be differernt
                SendValue(MixesView_MixNumber, MixNumber); // New load of mix window
                ShowMixValues();
            }

            if (CurrentView == MACROS_VIEW) {
                // Do nothing!
            }
            if (CurrentView == CALIBRATEVIEW) {
                Force_ReDisplay();
                ShowServoPos();
            }
            if (CurrentView == REVERSEVIEW) {
                StartReverseView();
            }
            if ((CurrentView == STICKSVIEW) || (CurrentView == FRONTVIEW)) {
                Force_ReDisplay();
                ShowServoPos();
                if (CurrentView == FRONTVIEW) {
                    SendValue(FrontView_Secs, Secs);
                    SendValue(FrontView_Mins, Mins);
                    SendValue(FrontView_Hours, Hours);
                }
            }
            if (CurrentView == LOGVIEW) {

                SendCommand(pLogView);
                CurrentView     = LOGVIEW;
                RecentStartLine = 0;
                if (UseLog) ShowLogFile(RecentStartLine);
            }
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(Exrite, TextIn) > 0) { //  *******************
            if (GetValue(ExpR)) {
                InterpolationTypes[Bank][ChanneltoSet - 1] = EXPONENTIALCURVES;
            }
            if (GetValue(Smooth)) {
                InterpolationTypes[Bank][ChanneltoSet - 1] = SMOOTHEDCURVES;
            }
            if (GetValue(Lines)) {
                InterpolationTypes[Bank][ChanneltoSet - 1] = STRAIGHTLINES;
            }
            Exponential[Bank][ChanneltoSet - 1] = GetValue(Expo) + 50; // Note: Getting this value from slider was not reliable (could not return 36!)
            ClearText();
            DisplayCurveAndServoPos();
            return;
        }

        if (InStrng(SendModel, TextIn) > 0) {
            i = strlen(SendModel);
            j = 0;
            while (uint8_t(TextIn[i]) > 0) {
                SingleModelFile[j] = TextIn[i];
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            }
            SendModelFile();
            ClearText();
            return;
        }
        if (InStrng(FailSAVE, TextIn) > 0) {
            SendCommand(ProgressStart);
            for (int i = 0; i < 16; ++i) {
                FailSafeChannel[i] = GetValue(fs[i]);
                SendValue(Progress, i * (100 / 16));
            }
            SendValue(Progress, 100);
            FailSafeTimer   = millis();
            SaveFailSafeNow = true;
            ClearText();
            return;
        }
        if (InStrng(FailSafe, TextIn) > 0) {
            SendCommand(pFailSafe);
            CurrentView = FAILSAFE_VIEW;
            UpdateButtonLabels();
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(OneSwitchView, TextIn) > 0) {
            SwitchEditNumber = GetChannel(); // which switch?
            CurrentView      = ONE_SWITCH_VIEW;
            SendCommand(PageOneSwitchView); // edit one switch - could be 1-4 // heer
            UpdateOneSwitchView();
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(SwitchesView1, TextIn) > 0) { //  read switch values from screen (could be 1-4)
            ReadNewSwitchFunction();
        }

        if (InStrng(InputsView, TextIn) > 0) {
            SendCommand(pInputsView);
            CurrentView = INPUTS_VIEW;
            UpdateButtonLabels();
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(InputsDone, TextIn) > 0) {
            SendCommand(ProgressStart);
            for (int i = 0; i < 16; ++i) {
                if (ModelMatched) {
                    NewCompressNeeded = false;
                    SendData(); // To prevent failsafe, keep sending data as this takes a while
                }
                InPutStick[i]    = CheckRange((GetValue(InputStick_Labels[i]) - 1), 0, 15);
                ChannelOutPut[i] = CheckRange((GetValue(OutputStick_Labels[i]) - 1), 0, 15);
                if (i < 4) InputTrim[i] = CheckRange((GetValue(InputTrim_labels[i]) - 1), 0, 15);
                SendValue(Progress, (((i + 1) * 100) / 16) - 1);
            }
            CheckOutPutChannels();
            SendValue(Progress, 95);
            SaveOneModel(ModelNumber);
            SendValue(Progress, 100);
            CurrentMode = NORMAL;
            UpdateButtonLabels();
            CurrentView = RXSETUPVIEW;
            SendCommand(page_RXSetupView);
            b5isGrey     = false;
            b12isGrey    = false;
            LastTimeRead = 0;
            ClearText();
            return;
        }

        if (InStrng(CH1NAME, TextIn) > 0) {
            p = InStrng(CH1NAME, TextIn);
            i = p + 7;
            DoNewChannelName(1, i);
            ClearText();
            return;
        }
        if (InStrng(CH2NAME, TextIn) > 0) {
            p = InStrng(CH2NAME, TextIn);
            i = p + 7;
            DoNewChannelName(2, i);
            ClearText();
            return;
        }
        if (InStrng(CH3NAME, TextIn) > 0) {
            p = InStrng(CH3NAME, TextIn);
            i = p + 7;
            DoNewChannelName(3, i);
            ClearText();
            return;
        }
        if (InStrng(CH4NAME, TextIn) > 0) {
            p = InStrng(CH4NAME, TextIn);
            i = p + 7;
            DoNewChannelName(4, i);
            ClearText();
            return;
        }
        if (InStrng(CH5NAME, TextIn) > 0) {
            p = InStrng(CH5NAME, TextIn);
            i = p + 7;
            DoNewChannelName(5, i);
            ClearText();
            return;
        }
        if (InStrng(CH6NAME, TextIn) > 0) {
            p = InStrng(CH6NAME, TextIn);
            i = p + 7;
            DoNewChannelName(6, i);
            ClearText();
            return;
        }
        if (InStrng(CH7NAME, TextIn) > 0) {
            p = InStrng(CH7NAME, TextIn);
            i = p + 7;
            DoNewChannelName(7, i);
            ClearText();
            return;
        }
        if (InStrng(CH8NAME, TextIn) > 0) {
            p = InStrng(CH8NAME, TextIn);
            i = p + 7;
            DoNewChannelName(8, i);
            ClearText();
            return;
        }
        if (InStrng(CH9NAME, TextIn) > 0) {
            p = InStrng(CH9NAME, TextIn);
            i = p + 7;
            DoNewChannelName(9, i);
            ClearText();
            return;
        }
        if (InStrng(CH10NAME, TextIn) > 0) {
            p = InStrng(CH10NAME, TextIn);
            i = p + 8;
            DoNewChannelName(10, i);
            ClearText();
            return;
        }
        if (InStrng(CH11NAME, TextIn) > 0) {
            p = InStrng(CH11NAME, TextIn);
            i = p + 8;
            DoNewChannelName(11, i);
            ClearText();
            return;
        }
        if (InStrng(CH12NAME, TextIn) > 0) {
            p = InStrng(CH12NAME, TextIn);
            i = p + 8;
            DoNewChannelName(12, i);
            ClearText();
            return;
        }
        if (InStrng(CH13NAME, TextIn) > 0) {
            p = InStrng(CH13NAME, TextIn);
            i = p + 8;
            DoNewChannelName(13, i);
            ClearText();
            return;
        }
        if (InStrng(CH14NAME, TextIn) > 0) {
            p = InStrng(CH14NAME, TextIn);
            i = p + 8;
            DoNewChannelName(14, i);
            ClearText();
            return;
        }
        if (InStrng(CH15NAME, TextIn) > 0) {
            p = InStrng(CH15NAME, TextIn);
            i = p + 8;
            DoNewChannelName(15, i);
            ClearText();
            return;
        }
        if (InStrng(CH16NAME, TextIn) > 0) {
            p = InStrng(CH16NAME, TextIn);
            i = p + 8;
            DoNewChannelName(16, i);
            ClearText();
            return;
        }

        if (InStrng(DelFile, TextIn) > 0) { // Delete a file
            j = 0;
            p = InStrng(DelFile, TextIn);
            i = p + 6;
            while (uint8_t(TextIn[i]) > 0) {
                SingleModelFile[j] = TextIn[i];
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            }
            strcpy(Prompt, del);
            strcat(Prompt, SingleModelFile);
            strcat(Prompt, ques);
            if (GetConfirmation(GoModelsView, Prompt)) {
                SD.remove(SingleModelFile);
                BuildDirectory();
                LoadFileSelector();
                --FileNumberInView;
                ShowFileNumber();
            }
            CloseModelsFile();
            ClearText();
            return;
        }

        if (InStrng(SwitchesView, TextIn)) {
            SendCommand(pSwitchesView);
            UpdateSwitchesView(); // display saved values
            CurrentView = SWITCHES_VIEW;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(CalibrateView, TextIn)) {
            SendCommand(pCalibrateView);
            Force_ReDisplay();
            CurrentView = CALIBRATEVIEW;
            ClearText();
            return;
        }

        if (InStrng(Export, TextIn)) {
            GetDefaultFilename();
            if (GetBackupFilename(GoModelsView, SingleModelFile, ModelName, hhead, fprompt)) {
                FixFileName();
                if (CheckFileExists(SingleModelFile)) {
                    strcpy(Prompt, overwr);
                    strcat(Prompt, SingleModelFile);
                    strcat(Prompt, ques);
                    if (GetConfirmation(GoModelsView, Prompt)) {
                        WriteBackup();
                    }
                }
                else {
                    WriteBackup();
                }
            }
            BuildDirectory(); // of SD card
            LoadFileSelector();
            ClearText();
            return;
        }

        p = InStrng(Import, TextIn);
        if (p > 0) {
            j = 0;
            i = p + 5;
            while (TextIn[i] > 0) {
                SingleModelFile[j] = toUpperCase(TextIn[i]);
                ++j;
                ++i;
                SingleModelFile[j] = 0;
            }
            strcpy(Prompt, overwr);
            strcat(Prompt, ModelName);
            strcat(Prompt, ques);
            if (GetConfirmation(GoModelsView, Prompt)) {
                SendCommand(ProgressStart);
                DelayWithDog(10);
                SendValue(Progress, 5);
                DelayWithDog(10);
                if (InStrng(ModExt, SingleModelFile) == 0) strcat(SingleModelFile, ModExt);
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
                if (FileError) ShowFileErrorMsg();
                LoadModelSelector();
            }
            ClearText();
            return;
        }

        if (InStrng(GoSetupView, TextIn) > 0) {
            CurrentView = TXSETUPVIEW;
            SendCommand(page_SetupView);
            b5isGrey  = false;
            b12isGrey = false;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(ColoursView, TextIn) > 0) {
            CurrentView = COLOURS_VIEW;
            SendCommand(page_ColoursView);
            SendCommand(StartBackGround);
            ClearText();
            return;
        }

        if (InStrng(SetupCol, TextIn) > 0) { // This is the return from Colours setup
            HighlightColour  = GetOtherValue(High_pco);
            ForeGroundColour = GetOtherValue(b0_pco);
            BackGroundColour = GetOtherValue(b0_bco);
            SpecialColour    = GetOtherValue(Fm_pco);
            SendValue(FrontView_BackGround, BackGroundColour);
            SendValue(FrontView_ForeGround, ForeGroundColour);
            SendValue(FrontView_Special, SpecialColour);
            SendValue(FrontView_Highlight, HighlightColour);
            SaveTransmitterParameters();
            CurrentView = TXSETUPVIEW;
            b5isGrey    = false;
            b12isGrey   = false;
            SendCommand(page_SetupView);
            LastTimeRead = 0;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(TypeView, TextIn) > 0) {
            SendCommand(pTypeView);
            SendValue(r2s, 0); // Zero all RX batt cell count
            SendValue(r3s, 0);
            SendValue(r4s, 0);
            SendValue(r5s, 0);
            SendValue(r6s, 0);
            SendValue(r12s, 0);
            SendValue(r0, 0);
            SendValue(r1, 0);
            if (RXCellCount == 2) SendValue(r2s, 1); // Then update RX batt cell count
            if (RXCellCount == 3) SendValue(r3s, 1);
            if (RXCellCount == 4) SendValue(r4s, 1);
            if (RXCellCount == 5) SendValue(r5s, 1);
            if (RXCellCount == 6) SendValue(r6s, 1);
            if (RXCellCount == 12) SendValue(r12s, 1);
            if (TXLiPo)
                SendValue(r1, 1);
            else
                SendValue(r0, 1);

            ClearText();
            UpdateModelsNameEveryWhere();
            return;
        }

        if (InStrng(RXBAT, TextIn) > 0) { // UPdate RX batt cell count
            if (GetValue(r2s) == 1) RXCellCount = 2;
            if (GetValue(r3s) == 1) RXCellCount = 3;
            if (GetValue(r4s) == 1) RXCellCount = 4;
            if (GetValue(r5s) == 1) RXCellCount = 5;
            if (GetValue(r6s) == 1) RXCellCount = 6;
            if (GetValue(r12s) == 1) RXCellCount = 12;
            if (GetValue(r0) == 1) TXLiPo = false; // TX LIFE
            if (GetValue(r1) == 1) TXLiPo = true;  // TX LIPO
            SaveAllParameters();
            ClearText();
            return;
        }

        if (InStrng(TrimView, TextIn) > 0) { // TrimView just appeared, so update it.
            StartTrimView();
            return;
        }

        if (InStrng(TRIMS50, TextIn) > 0) {
            for (i = 0; i < 15; ++i) {
                Trims[Bank][i] = 80; // Mid value is 80
            }
            if (CopyTrimsToAll) {
                for (i = 0; i < 15; ++i) {
                    for (int fm = 1; fm < 5; ++fm) {
                        Trims[fm][i] = 80;
                    }
                }
            }
            ClearText();
            return;
        }

        if (InStrng(Setup, TextIn) > 0) { // Which channel to setup ... Goes to GraphView
            ChanneltoSet = GetChannel();
            CurrentView  = GRAPHVIEW;
            SendCommand(page_GraphView); // Set to GraphView
            DisplayCurve();
            updateInterpolationTypes();
            UpdateModelsNameEveryWhere();
            SendValue(CopyToAllBanks, 0);
            ClearText();
            return;
        }

        if (InStrng(Front_View, TextIn)) {
            CurrentView = FRONTVIEW;
            ClearText();
            PreviousBank = 250; // sure to be different
            CurrentMode  = NORMAL;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(Sticks_View, TextIn)) {
            SendCommand(page_SticksView);
            Force_ReDisplay();
            CurrentView = STICKSVIEW;
            SendCommand(page_SticksView); // Set to SticksView
            UpdateModelsNameEveryWhere();
            UpdateButtonLabels();
            ClearText();
            return;
        }

        if (InStrng(Fhss_View, TextIn)) {

            if ((!b5isGrey)) // no scan while connected!!!
            {
                if (PPMdata.UseTXModule) InitRadio(DefaultPipe); // because scan fails if radio isn't initialised
                SendCommand(page_FhssView);
                DrawFhssBox();
                DoScanInit();
                CurrentMode = SCANWAVEBAND;
                CurrentView = SCANVIEW;
                BlueLedOn();
            }
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

        if (InStrng(MIXES_VIEW, TextIn)) { //        First call to load screen
            SendCommand(pMixesView);
            CurrentView = MIXESVIEW;
            UpdateModelsNameEveryWhere();
            if (MixNumber == 0) MixNumber = 1;
            LastMixNumber = MixNumber;
            SendValue(MixesView_MixNumber, MixNumber); // New load of mix window
            ShowMixValues();
            ClearText();
            return;
        }
        if (InStrng(Mixes_View, TextIn)) { // Mix number OR a parameter has changed
            CurrentView = MIXESVIEW;
            UpdateModelsNameEveryWhere();
            uint8_t ThisMixNumber = MixNumber; // save it
            MixNumber             = GetValue(MixesView_MixNumber);
            if (LastMixNumber != MixNumber) // Did number change?
            {
                LastMixNumber = MixNumber;     // save new mix number
                MixNumber     = ThisMixNumber; // force back to old number to grab last lot before doing new one
                SaveMixValues();               // save them
                SaveOneModel(ModelNumber);
                SendCommand(ProgressEnd);
                MixNumber = LastMixNumber; // back to new one
                ShowMixValues();           // show new lot
            }
            else
            {
                FixCHNames(); // just redisplay ch names
            }
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
            CurrentMode  = NORMAL;
            CurrentView  = DATAVIEW;
            LastShowTime = 0;
            SendCommand(pDataView);
            ClearText();
            return;
        }

        if (InStrng(FM1, TextIn))
        {
            Bank         = 1;
            PreviousBank = 1;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(FM2, TextIn))
        {
            Bank         = 2;
            PreviousBank = 2;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(FM3, TextIn))
        {
            Bank         = 3;
            PreviousBank = 3;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(FM4, TextIn))
        {
            Bank         = 4;
            PreviousBank = 4;
            UpdateModelsNameEveryWhere();
            ClearText();
            return;
        }

        if (InStrng(Reset, TextIn)) // Now zeros EXPO only
        {
            Exponential[Bank][ChanneltoSet - 1]        = DEFAULT_EXPO;
            InterpolationTypes[Bank][ChanneltoSet - 1] = EXPONENTIALCURVES; // expo = default
            DisplayCurveAndServoPos();
            return;
        }

        p = (InStrng(ClickX, TextIn)); // Clicked to move point?
        if (p > 0) {
            XtouchPlace = GetNextNumber(p + 7, TextIn);
            // This drops through to get Y as well before moving the point
        }
        p = (InStrng(ClickY, TextIn)); // Clicked to move point?  // heer
        if (p > 0) {
            YtouchPlace = GetNextNumber(p + 7, TextIn);
            MovePoint();
            DisplayCurveAndServoPos();
            return;
        }
        if (CurrentMode == NORMAL) {
            if (strcmp(TextIn, "Calibrate1") == 0) {
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

        if (CurrentMode == CALIBRATELIMITS) {
            if (strcmp(TextIn, "Calibrate1") == 0) {
                CurrentMode = CENTRESTICKS;
                CurrentView = CALIBRATEVIEW;
                SendText1(SvT11, Cmsg3);
                SendText(SvB0, Cmsg4);
                ClearText();
                return;
            }
        }
        if (CurrentMode == CENTRESTICKS) {
            if (strcmp(TextIn, "Calibrate1") == 0) {
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

uint16_t MakeTwobytes(bool* f)
{                    // Pass arraypointer. Returns the two bytes
    uint16_t tb = 0; // all false is default
    for (int i = 0; i < 16; ++i) {
        if (f[15 - i] == true) {
            tb |= 1 << (i);
        } // sets a bit if true
    }
    return tb;
}

/************************************************************************************************************/

// 20 x 16bit words are sent compressed to only 15 (30 bytes)
// 4 16 bit words are vacant for other stuff (8 bytes)
// Extra data can be send using the last 10 bytes of each data packet.
// These are defined by the packet number.

void LoadPacketData()
{
    uint16_t Twobytes = 0;
    uint8_t  FS_Byte1;
    uint8_t  FS_Byte2;
    char     ProgressEnd[]       = "vis Progress,0";
    SendBuffer[CHANNELSUSED]     = PacketNumber;
    Twobytes                     = MakeTwobytes(FailSafeChannel); // 16 bool values compressed to 16 bits
    FS_Byte1                     = uint8_t(Twobytes >> 8);        // sent as two bytes
    FS_Byte2                     = uint8_t(Twobytes & 0x00FF);
    SendBuffer[CHANNELSUSED + 1] = 0;
    SendBuffer[CHANNELSUSED + 2] = 0;
    switch (PacketNumber) {
        case 0:

            //  SendBuffer[CHANNELSUSED + 2] = NOTUSEDYET;

            if (((millis() - FailSafeTimer) > 1500) && SaveFailSafeNow) {
                SendBuffer[CHANNELSUSED + 1] = SaveFailSafeNow; // FailSafeSaveMoment
                SaveFailSafeNow              = false;           // once should do it.
                SendCommand(ProgressEnd);
            }
            break;
        case 1:
            SendBuffer[CHANNELSUSED + 1] = FS_Byte2; // these are failsafe flags
            SendBuffer[CHANNELSUSED + 2] = FS_Byte1; // these are failsafe flags
            break;
        case 2:
            SendBuffer[CHANNELSUSED + 1] = Qnh >> 8;     // (HiByte)   Qnh is current atmospheric pressure at sea level here (an aviation term)
            SendBuffer[CHANNELSUSED + 2] = Qnh & 0x00ff; // (LowByte)  Qnh is current atmospheric pressure at sea level here (an aviation term)
            break;
        case 3:
            if (GPSMarkHere) {
                SendBuffer[CHANNELSUSED + 1] = 0;
                SendBuffer[CHANNELSUSED + 2] = GPSMarkHere;
                GPSMarkHere                  = 0;
            }
            break;
        case 4:
            SendBuffer[CHANNELSUSED + 1] = ModelMatched; // let receiver know whether correct model is loaded.
            SendBuffer[CHANNELSUSED + 2] = SwapWaveBand;
            if (SwapWaveBand == 2) SetTestFrequencies();
            if (SwapWaveBand == 1) SetUKFrequencies();
            SwapWaveBand = 0;
            break;

        case 5:
            SendBuffer[CHANNELSUSED + 1] = PPMdata.UseSBUSFromRX;   // 1 - 0
            SendBuffer[CHANNELSUSED + 2] = PPMdata.PPMChannelCount; //
            break;

        default: break;
    }
}

/************************************************************************************************************/

void ReadDRSwitch(bool sw1, bool sw2, bool rev) // Dual Rate Switch
{
    if ((sw1 == false) && (sw2 == false))
    {
        DualRateInUse = 2;
    }
    else {
        if (rev) {
            if (sw1) DualRateInUse = 1;
            if (sw2) DualRateInUse = 3;
        }
        else {
            if (sw1) DualRateInUse = 3;
            if (sw2) DualRateInUse = 1;
        }
    }
}

/************************************************************************************************************/

void ReadFMSwitch(bool sw1, bool sw2, bool rev) // Bank Switch
{
    if ((sw1 == false) && (sw2 == false))
    {
        Bank = 2;
    }
    else {
        if (rev) {
            if (sw1) Bank = 1;
            if (sw2) Bank = 3;
        }
        else {
            if (sw1) Bank = 3;
            if (sw2) Bank = 1;
        }
    }
}

/************************************************************************************************************/

uint8_t ReadCHSwitch(bool sw1, bool sw2, bool rev)
{
    uint8_t ttmp = 90;
    if (sw1 == false && sw2 == false) ttmp = 90;
    if (rev) {
        if (sw1) ttmp = 0;
        if (sw2) ttmp = 180;
    }
    else {
        if (sw1) ttmp = 180;
        if (sw2) ttmp = 0;
    }
    return ttmp;
}

/************************************************************************************************************/

uint8_t CheckSwitch(uint8_t swt)
{
    uint8_t rtv = 90;
    if (swt == 1) rtv = ReadCHSwitch(Switch[7], Switch[6], SWITCH1Reversed);
    if (swt == 2) rtv = ReadCHSwitch(Switch[5], Switch[4], SWITCH2Reversed);
    if (swt == 3) rtv = ReadCHSwitch(Switch[0], Switch[1], SWITCH3Reversed);
    if (swt == 4) rtv = ReadCHSwitch(Switch[2], Switch[3], SWITCH4Reversed);
    return rtv;
}

/************************************************************************************************************/

void CheckMotorOff()
{ // For Safety

    if (!UseMotorKill) return;
    ReadSwitches();
    MotorEnabled = false;
    if (Autoswitch == 1 && Switch[7] == SWITCH1Reversed) MotorEnabled = true;
    if (Autoswitch == 2 && Switch[5] == SWITCH2Reversed) MotorEnabled = true;
    if (Autoswitch == 3 && Switch[0] == SWITCH3Reversed) MotorEnabled = true;
    if (Autoswitch == 4 && Switch[2] == SWITCH4Reversed) MotorEnabled = true;
    if (SafetySwitch == 1 && Switch[7] == SWITCH1Reversed) SafetyON = true;
    if (SafetySwitch == 2 && Switch[5] == SWITCH2Reversed) SafetyON = true;
    if (SafetySwitch == 3 && Switch[0] == SWITCH3Reversed) SafetyON = true;
    if (SafetySwitch == 4 && Switch[2] == SWITCH4Reversed) SafetyON = true;
    if (SafetyON) MotorEnabled = false;
    MotorWasEnabled = MotorEnabled;
}

/************************************************************************************************************/

void GetBank()
{ //  and  motor switch and safety switch ETC ...

    char FrontView_Hours[] = "Hours";
    char FrontView_Mins[]  = "Mins";
    char FrontView_Secs[]  = "Secs";
    char WarnOff[]         = "vis Warning,0";

    if (CurrentMode != NORMAL) return; // not needed if calibrating
    SafetyON = false;
    BuddyON  = false;

    DualRateInUse = 4;

    MotorEnabled = !UseMotorKill; //  If not using motor switch then motor is always enabled.

    if (Autoswitch == 1 && Switch[7] == SWITCH1Reversed) MotorEnabled = true;
    if (Autoswitch == 2 && Switch[5] == SWITCH2Reversed) MotorEnabled = true;
    if (Autoswitch == 3 && Switch[1] == SWITCH3Reversed) MotorEnabled = true;
    if (Autoswitch == 4 && Switch[2] == SWITCH4Reversed) MotorEnabled = true;

    if (SafetySwitch == 1 && Switch[7] == SWITCH1Reversed) SafetyON = true;
    if (SafetySwitch == 2 && Switch[5] == SWITCH2Reversed) SafetyON = true;
    if (SafetySwitch == 3 && Switch[1] == SWITCH3Reversed) SafetyON = true;
    if (SafetySwitch == 4 && Switch[2] == SWITCH4Reversed) SafetyON = true;

    if (BuddySwitch == 1 && Switch[7] == SWITCH1Reversed) BuddyON = true;
    if (BuddySwitch == 2 && Switch[5] == SWITCH2Reversed) BuddyON = true;
    if (BuddySwitch == 3 && Switch[1] == SWITCH3Reversed) BuddyON = true;
    if (BuddySwitch == 4 && Switch[2] == SWITCH4Reversed) BuddyON = true;

    if (DualRatesSwitch == 4) ReadDRSwitch(Switch[2], Switch[3], SWITCH4Reversed);
    if (DualRatesSwitch == 3) ReadDRSwitch(Switch[0], Switch[1], SWITCH3Reversed);
    if (DualRatesSwitch == 2) ReadDRSwitch(Switch[4], Switch[5], SWITCH2Reversed);
    if (DualRatesSwitch == 1) ReadDRSwitch(Switch[6], Switch[7], SWITCH1Reversed);

    if (DualRateInUse == 1) DualRateValue = Drate1;
    if (DualRateInUse == 2) DualRateValue = Drate2;
    if (DualRateInUse == 3) DualRateValue = Drate3;
    if (DualRateInUse == 4) DualRateValue = 100; // Switch not in use, so use 100%

    if (PreviousDualRateInUse != DualRateInUse) {
        PreviousDualRateInUse = DualRateInUse;
        LastShowTime          = 0;
        LastTimeRead          = 0;
        RestoreBrightness();
        if (AnnounceBanks) {
            switch (DualRateInUse)
            {
                case 1:
                    PlaySound(RATE1);
                    break;
                case 2:
                    PlaySound(RATE2);
                    break;
                case 3:
                    PlaySound(RATE3);
                    break;
                default:
                    break;
            }
        }
        // if (UseLog) LogNewRATE();
    }

    if (FMSwitch == 4) ReadFMSwitch(Switch[2], Switch[3], SWITCH4Reversed);
    if (FMSwitch == 3) ReadFMSwitch(Switch[0], Switch[1], SWITCH3Reversed);
    if (FMSwitch == 2) ReadFMSwitch(Switch[4], Switch[5], SWITCH2Reversed);
    if (FMSwitch == 1) ReadFMSwitch(Switch[6], Switch[7], SWITCH1Reversed);

    if (Autoswitch == 1 && Switch[6] == SWITCH1Reversed) Bank = 4; // Flight mode 4 (Auto) overrides modes 1,2,3.
    if (Autoswitch == 2 && Switch[4] == SWITCH2Reversed) Bank = 4;
    if (Autoswitch == 3 && Switch[1] == SWITCH3Reversed) Bank = 4;
    if (Autoswitch == 4 && Switch[3] == SWITCH4Reversed) Bank = 4;

    if (SafetyWasOn != SafetyON) {
        if (SafetyON)
            ShowSafetyIsOn();
        else
            ShowSafetyIsOff();
        SafetyWasOn = SafetyON;
    }

    if (SafetyON) {
        MotorEnabled   = false;
        TimesUp        = false;
        PausedSecs     = 0;
        CountDownIndex = 0;
        if (CurrentView == FRONTVIEW) {
            if (TimerDownwards) {
                Mins = TimerStartTime / 60;
            }
            else {
                Mins = 0;
            }
            SendValue(FrontView_Secs, 0);
            SendValue(FrontView_Mins, Mins);
            SendValue(FrontView_Hours, 0);
        }
    }

    if ((MotorEnabled != MotorWasEnabled) && (UseMotorKill)) { // MotorEnabled changed ?
        if (MotorEnabled) {
            if (LedWasRed)
            {
                MotorEnabled = false;
                if (!BuddyPupilOnPPM) {
                    if ((millis() - WarningTimer) > 4000) {
                        PlaySound(PLSTURNOFF);
                        WarningTimer = millis();
                    }
                }
                return;
            }
            ShowMotor(1);
            if (AnnounceBanks) PlaySound(MOTORON); // Tell the pilot motor is on!
            if (UseLog) LogMotor(1);
            TimerMillis = millis(); // Motor ON timerpause off
        }
        else {
            if (AnnounceBanks) PlaySound(MOTOROFF);
            if (UseLog) LogMotor(0);
            SendCommand(WarnOff);
            ShowMotor(0); // Tell the pilot motor is off
            if (SendNoData) {
                SendCommand(WarnOff);
                SendNoData = false; // user turned off motor
            }
            PausedSecs = ElapsedSeconds; //    Motor OFF timerpause started
        }
        LastSeconds = 0;
        ShowMotorTimer();
    }
    Channel9SwitchValue  = CheckSwitch(Channel9Switch);
    Channel10SwitchValue = CheckSwitch(Channel10Switch);
    Channel11SwitchValue = CheckSwitch(Channel11Switch);
    Channel12SwitchValue = CheckSwitch(Channel12Switch);
    if (Bank != PreviousBank) {
        RestoreBrightness();
        LastTimeRead = 0;
        if (UseLog) LogNewBank();
        if (MotorEnabled == MotorWasEnabled) { // When turning off motor, don't sound bank too.
            if (AnnounceBanks) SoundBank();
        }
        if (CurrentView == FRONTVIEW) {
            ShowBank();
        }
        else {
            UpdateModelsNameEveryWhere();
        }
        if (CurrentView == GRAPHVIEW) DisplayCurveAndServoPos();
    }
    MotorWasEnabled = MotorEnabled; // Remember motor state
    PreviousBank    = Bank;         // Remember BANK
}

// *************************************************************************************************************

void IncTrim(uint8_t t)
{
    bool Sounded = false;
    Trims[Bank][t] += 1;
    if (Trims[Bank][t] >= 120) {
        Trims[Bank][t] = 120;
        if (TrimClicks) {

            PlaySound(BEEPCOMPLETE);
            Sounded         = true;
            TrimRepeatSpeed = DEFAULTTRIMREPEATSPEED;
        }
    }
    if (Trims[Bank][t] == 80) {
        TrimRepeatSpeed = DEFAULTTRIMREPEATSPEED; // Restore default trim repeat speed at centre
        if (TrimClicks) {
            PlaySound(BEEPMIDDLE);
            Sounded = true;
        }
    }
    if ((CurrentView == TRIM_VIEW) || (CurrentView == FRONTVIEW)) UpdateTrimView();
    if ((TrimClicks) && (!Sounded)) PlaySound(CLICKZERO);
}
// *************************************************************************************************************

void DecTrim(uint8_t t)
{

    bool Sounded = false;
    Trims[Bank][t] -= 1;
    if (Trims[Bank][t] <= 40) {
        Trims[Bank][t] = 40;
        if (TrimClicks) {
            PlaySound(BEEPCOMPLETE);
            Sounded         = true;
            TrimRepeatSpeed = DEFAULTTRIMREPEATSPEED;
        }
    }
    if (Trims[Bank][t] == 80) {
        TrimRepeatSpeed = DEFAULTTRIMREPEATSPEED; // Restore default trim repeat speed at centre
        if (TrimClicks) {
            PlaySound(BEEPMIDDLE);
            Sounded = true;
        }
    }
    if ((CurrentView == TRIM_VIEW) || (CurrentView == FRONTVIEW)) UpdateTrimView();
    if ((TrimClicks) && (!Sounded)) PlaySound(CLICKZERO);
}

// *************************************************************************************************************

void MoveaTrim(uint8_t i)
{
    uint8_t Elevator = 1;
    uint8_t Throttle = 2;

    if (SticksMode == 2) {
        Elevator = 2;
        Throttle = 1;
    }

    switch (i) {
        case 0:
            IncTrim(InputTrim[0]); // Aileron
            break;
        case 1:
            DecTrim(InputTrim[0]); // Aileron
            break;
        case 2:
            IncTrim(InputTrim[Elevator]);
            break;
        case 3:
            DecTrim(InputTrim[Elevator]);
            break;
        case 4:
            DecTrim(InputTrim[Throttle]);
            break;
        case 5:
            IncTrim(InputTrim[Throttle]);
            break;
        case 6:
            IncTrim(InputTrim[3]); // Rudder
            break;
        case 7:
            DecTrim(InputTrim[3]); // Rudder
            break;
        default:
            break;
    }
    if (ScreenIsOff) RestoreBrightness();
    if (CopyTrimsToAll) {
        for (i = 0; i < 4; ++i) {
            for (int fm = 1; fm < 5; ++fm) {
                Trims[fm][i] = Trims[Bank][i];
            }
        }
    }
}

/************************************************************************************************************/
void swap(uint8_t* a, uint8_t* b)
{ // Just swap over two bytes, a & b :-)
    uint8_t c;
    c  = *a;
    *a = *b;
    *b = c;
}
/************************************************************************************************************/
void CalibrateEdgeSwitches()
{ // This function avoids the need to rotate the four edge switches if installed backwards
    for (int i = 0; i < 8; ++i) {
        if (digitalRead(SwitchNumber[i])) {
            if (i == 0) swap(&SwitchNumber[i], &SwitchNumber[i + 1]); // swap over switches' pin number if wrongly installed
            if (i == 2) swap(&SwitchNumber[i], &SwitchNumber[i + 1]); // swap over switches' pin number if wrongly installed
            if (i == 4) swap(&SwitchNumber[i], &SwitchNumber[i + 1]); // swap over switches' pin number if wrongly installed
            if (i == 6) swap(&SwitchNumber[i], &SwitchNumber[i + 1]); // swap over switches' pin number if wrongly installed
        }
    }
}
/************************************************************************************************************/

FASTRUN void ReadSwitches() // and indeed read digital trims if these are fitted
{
    byte           flag         = 0;
    static uint8_t PreviousTrim = 255;
    for (int i = 0; i < 8; ++i) {
        Switch[i]     = !digitalRead(SwitchNumber[i]); // These are reversed because they are active low
        TrimSwitch[i] = !digitalRead(TrimNumber[i]);   // These are reversed because they are active low
        if (TrimSwitch[i]) ++flag;                     // a finger is on a trim lever...
        if ((TrimSwitch[i]) && (PreviousTrim != i)) {  // is it a new one?
            TrimTimer    = 0;                          // it IS a new one, so no delay please.
            PreviousTrim = i;                          // remember which trim it was
        }
    }
    if (flag > 1) {                               // one at a time please!!
        TrimRepeatSpeed = DEFAULTTRIMREPEATSPEED; // Restore default trim repeat speed
        for (int i = 0; i < 8; ++i) {
            (TrimSwitch[i]) = 0;
            flag            = 0;
        }
    }
    if (!flag) {
        PreviousTrim    = 254;                    // Previous trim must now match none
        TrimRepeatSpeed = DEFAULTTRIMREPEATSPEED; // Restore default trim repeat speed
    }
}
/************************************************************************************************************/

void GetRXVersionNumber()
{
    char nbuf[5];
    Str(nbuf, AckPayload.Byte1, 0);
    strcpy(ThisRadio, nbuf);
    if (LastRadio != AckPayload.Byte1) {
        LastRadio = AckPayload.Byte1;
        if (LogRXSwaps && UseLog && LastRadio <= 2 && (LastRadio)) LogThisRX();
    }
    Str(ReceiverVersionNumber, AckPayload.Byte2, 2);
    Str(nbuf, AckPayload.Byte3, 2);
    strcat(ReceiverVersionNumber, nbuf);
    Str(nbuf, AckPayload.Byte4, 0);
    strcat(ReceiverVersionNumber, nbuf);
}

/************************************************************************************************************/

FASTRUN float GetFromAckPayload()
{
    union
    {
        float   Val32;
        uint8_t Val8[4];
    } ThisUnion;
    ThisUnion.Val8[0] = AckPayload.Byte1;
    ThisUnion.Val8[1] = AckPayload.Byte2;
    ThisUnion.Val8[2] = AckPayload.Byte3;
    ThisUnion.Val8[3] = AckPayload.Byte4;
    return ThisUnion.Val32;
}
/************************************************************************************************************/
void GetTimeFromAckPayload()
{
    GPSSecs  = AckPayload.Byte1;
    GPSMins  = AckPayload.Byte2;
    GPSHours = AckPayload.Byte3;
}
/************************************************************************************************************/
void GetDateFromAckPayload()
{
    GPSDay   = AckPayload.Byte1;
    GPSMonth = AckPayload.Byte2;
    GPSYear  = AckPayload.Byte3;
}
/************************************************************************************************************/
void GetAltitude()
{
    RXModelAltitude = int(GetFromAckPayload()) - GroundModelAltitude;
    if (RXMAXModelAltitude < RXModelAltitude) RXMAXModelAltitude = RXModelAltitude;
    snprintf(Maxaltitude, 5, "%d", RXMAXModelAltitude);
    snprintf(ModelAltitude, 5, "%d", RXModelAltitude);
}
/************************************************************************************************************/
void GetTemperature()
{
    RXTemperature = GetFromAckPayload();
    snprintf(ModelTempRX, 5, "%1.2f", RXTemperature);
}
/************************************************************************************************************/
FASTRUN uint32_t GetIntFromAckPayload() // This one uses a uint32_t int
{
    union
    {
        uint32_t Val32 = 0;
        uint8_t  Val8[4];
    } ThisUnion;
    ThisUnion.Val8[0] = AckPayload.Byte1;
    ThisUnion.Val8[1] = AckPayload.Byte2;
    ThisUnion.Val8[2] = AckPayload.Byte3;
    ThisUnion.Val8[3] = AckPayload.Byte4;
    return ThisUnion.Val32;
}

/************************************************************************************************************/

void GotoFrontView()
{
    char fms[4][4]             = {{"fm1"}, {"fm2"}, {"fm3"}, {"fm4"}};
    char FrontView_Connected[] = "Connected";
    char page_FrontView[]      = "page FrontView";

    if (CurrentView != FRONTVIEW) {
        if (CurrentView == SCANVIEW) DoScanEnd();
        SendCommand(page_FrontView);
        CurrentView = FRONTVIEW;
        CurrentMode = NORMAL;
        UpdateModelsNameEveryWhere();
        SafetyWasOn ^= 1; // this forces a re-display of safety state
        ShowBank();
        LastTimeRead = 0;
        Reconnected  = false; // this is to make '** Connected! **' redisplay (in ShowComms())
        LastSeconds  = 0;     // This forces redisplay of timer...
        Force_ReDisplay();
        ShowMotorTimer();
        ClearText();
        LastShowTime = 0; // this is to make redisplay sooner (in ShowComms())
        SendText(FrontView_Connected, na);
    }
    for (int i = 0; i < 4; ++i) {
        SendText(fms[i], BankTexts[BanksInUse[i]]);
    }
    UpdateTrimView();
}
/************************************************************************************************************/

/************************************************************************************************************/
FASTRUN void ParseAckPayload()
{
    if (BuddyPupilOnPPM) return; // buddy pupil need none of this

    FHSS_data::NextChannelNumber = AckPayload.Byte5; // every packet tells of next hop destination

    if (AckPayload.Purpose & 0x80) {                                              // Hi bit is now the **HOP NOW!!** flag
        NextChannel = *(FHSS_data::FHSSChPointer + FHSS_data::NextChannelNumber); // The actual channel number pointed to.
        HopToNextChannel();
        AckPayload.Purpose &= 0x7f; // Clear the high BIT, use the remainder ...
    }

    if (!ModelMatched && !LedWasGreen) {
        GetModelsMacAddress();
        return;
    }

    switch (AckPayload.Purpose) // Only look at the low 7 BITS
    {
        case 0:
            GetRXVersionNumber();
            break;
        case 1:
            SbusRepeats = GetFromAckPayload();
            break;
        case 2:
            RadioSwaps = GetFromAckPayload();
            break;
        case 3:
            RX1TotalTime = GetFromAckPayload();
            break;
        case 4:
            RX2TotalTime = GetFromAckPayload();
            break;
        case 5:
            RXModelVolts    = GetFromAckPayload();
            RXVoltsDetected = false;
            if (RXModelVolts > 0) {
                RXVoltsDetected = true;
                if (RXCellCount == 12) RXModelVolts *= 2; // voltage divider needed !
                snprintf(ModelVolts, 5, "%1.2f", RXModelVolts);
            }
            break;
        case 6:
            GetAltitude();
            break;
        case 7:
            GetTemperature();
            break;
        case 8:
            GPSLatitude = GetFromAckPayload();
            break;
        case 9:
            GPSLongitude = GetFromAckPayload();
            break;
        case 10:
            GPSAngle = GetFromAckPayload();
            break;
        case 11:
            GPSSpeed = GetFromAckPayload();
            if (GPSMaxSpeed < GPSSpeed) GPSMaxSpeed = GPSSpeed;
            break;
        case 12:
            GpsFix = GetFromAckPayload();
            break;
        case 13:
            GPSAltitude = GetFromAckPayload() - GPSGroundAltitude;
            if (GPSAltitude < 0) GPSAltitude = 0;
            if (GPSMaxaltitude < GPSAltitude) GPSMaxaltitude = GPSAltitude;
            break;
        case 14:
            GPSDistanceTo = GetFromAckPayload();
            if (GPSMaxDistance < GPSDistanceTo) GPSMaxDistance = GPSDistanceTo;
            break;
        case 15:
            GPSCourseTo = GetFromAckPayload();
            break;
        case 16:
            GPSSatellites = (uint8_t)GetFromAckPayload();
            break;
        case 17:
            GetDateFromAckPayload();
            break;
        case 18:
            GetTimeFromAckPayload();
            ReadTheRTC();
            if (GPSDay != GmonthDay) GPSTimeSynched = false;
            if (GPSMonth != Gmonth) GPSTimeSynched = false;
            if (GPSMins != Gminute) GPSTimeSynched = false;
            if (GPSHours != Ghour) GPSTimeSynched = false;
            if (GPSSecs != Gsecond) GPSTimeSynched = false;
            if (GpsFix) SynchRTCwithGPSTime();
            break;
        default:
            break;
    }
}
/************************************************************************************************************/
FASTRUN void CheckGapsLength()
{
    if (GapStart > 0) { // when reconnected, how long was connection lost?
        ++GapCount;
        ThisGap = (millis() - GapStart); // AND in fact RX sends no data for 20 ms after reconnection
        if (ThisGap >= MinimumGap && UseLog) LogThisGap();
        if (ThisGap > GapLongest) GapLongest = ThisGap;
        GapSum += ThisGap;
        GapStart   = 0;
        GapAverage = GapSum / GapCount;
#ifdef DB_GAPS
        Serial.print("GapCount: ");
        Serial.println(GapCount);
        Serial.print("GapAverage: ");
        Serial.println(GapAverage);
        Serial.print("GapLongest: ");
        Serial.println(GapLongest);
        Serial.println(" ");
#endif
    }
}

/************************************************************************************************************/
bool CheckModelName()
{ // In ModelsView, this function checks correct name is displayed.
  // it returns true if it has changed
    char MMems[]  = "MMems";
    char Mfiles[] = "Mfiles";
    char mn[]     = "modelname";

    ModelNumber      = GetValue(MMems) + 1;
    FileNumberInView = GetValue(Mfiles);
    if (FileNumberInView != LastFileInView) {
        ShowFileNumber();
        LastFileInView = FileNumberInView;
    }
    if (LastModelLoaded != ModelNumber) {
        if ((ModelNumber >= MAXMODELNUMBER) || (ModelNumber < 1)) {
            ModelNumber = 1;
            SendValue(MMems, ModelNumber - 1);
        }
        ReadOneModel(ModelNumber);
        SendText(mn, ModelName);
        if (UseLog) LogThisModel();
        LastModelLoaded = ModelNumber;
        UpdateModelsNameEveryWhere();
        return true;
    }
    ClearText();
    return false;
}

/************************************************************************************************************/
void SimulateCloseDown()
{ // Because real closedown occurs only after button is released

    char ScreenOff[] = "dim=0";
    analogWrite(GREENLED, 0);
    analogWrite(BLUELED, 0);
    analogWrite(REDLED, 0);
    SendCommand(ScreenOff);
    SaveOneModel(ModelNumber); // in case any trim change etc wasn't saved
    DelayWithDog(2500);
    digitalWrite(POWER_OFF_PIN, HIGH);
}
/************************************************************************************************************/
void CheckPowerOffButton()
{
    static bool     PowerWarningVisible = false;
    static uint32_t PowerOffTimer       = 0;
    static uint8_t  TurnOffSecondToGo   = PowerOffWarningSeconds;
    char            PowerMsg[15];
    char            PowerPre[] = "TURN OFF?! ";
    char            nb[4];
    char            NotStillConnected[] = "vis StillConnected,0";
    char            StillConnected[]    = "vis StillConnected,1";
    char            StillConnectedBox[] = "StillConnected";

    if (!digitalRead(BUTTON_SENSE_PIN)) {
        GotoFrontView();
        if (!LedWasGreen && !PPMdata.UseTXModule)
        {
            SimulateCloseDown(); // if not connected power off immediately
        }
        else
        {
            if (!PowerOffTimer) {
                RestoreBrightness();
                PowerOffTimer     = millis(); // Start a timer for power off button down
                TurnOffSecondToGo = PowerOffWarningSeconds;
            }
        }
    }
    else {
        PowerOffTimer     = 0;
        TurnOffSecondToGo = PowerOffWarningSeconds;
    }

    if (PowerOffTimer) { // count down started?
        if (!PowerWarningVisible) {
            SendCommand(StillConnected);
            SaveOneModel(ModelNumber); // in case any trim change etc wasn't saved
            PowerWarningVisible = true;
        }
    }
    else {
        if (PowerWarningVisible) {
            SendCommand(NotStillConnected);
            PowerWarningVisible = false;
            TurnOffSecondToGo   = PowerOffWarningSeconds;
        }
        return;
    }

    if (PowerWarningVisible) {
        if ((millis() - PreviousPowerOffTimer) >= 1000) {
            strcpy(PowerMsg, PowerPre);
            Str(nb, TurnOffSecondToGo, 0);
            strcat(PowerMsg, nb);
            SendText(StillConnectedBox, PowerMsg);
            if (TurnOffSecondToGo <= 0) {  // Time's up!
                if (UseLog) LogPowerOff(); // log the event
                if (PlayFanfare) {
                    PlaySound(WHAHWHAHMSG);
                    DelayWithDog(2300);
                }
                SaveAllParameters();
                DelayWithDog(250); // wait a mo for user to see 0 and log to write to file
                SimulateCloseDown();
            }
            --TurnOffSecondToGo;
            if (TrimClicks) PlaySound(CLICKZERO);
            PreviousPowerOffTimer = millis();
        }
    }
}

/************************************************************************************************************/
void FASTRUN ManageTransmitter()
{

    uint32_t RightNow        = millis();
    uint32_t TXPacketElapsed = RightNow - LastPacketSentTime;

    KickTheDog();                 // Watchdog ... ALWAYS!
    CheckPowerOffButton();        // Pretty obvious really ...
    CheckForNextionButtonPress(); // Pretty obvious really ...
    if ((PACEMAKER - TXPacketElapsed <= TIMEFORTXMANAGMENT) && ModelMatched) {
        return; // If it's almost time to send data, then do not start some other task which might easily take longer.
    }
    if (RightNow - TransmitterLastManaged > 50) { // 20 times a second is plenty
        if (RightNow - LastTimeRead >= 1000) {    // once a second for these...
            ReadTime();                           // Do the clock
            GetStatistics();                      // Do stats
            LastTimeRead = millis();
            CheckForNextionButtonPress();
            ShowComms(); // Screen Telemetry Data
            if (CurrentView == MODELSVIEW) CheckModelName();
            if (CurrentView == FRONTVIEW) ShowMotorTimer(); // Screen Timer
            return;                                         // That's enough housekeeping this time around
        }
        if (RightNow - LastScanButtonCheck >= 100) {
            if ((CurrentView == TXSETUPVIEW) || (CurrentView == RXSETUPVIEW)) CheckScanButton();
            LastScanButtonCheck = millis();
        }
        ReadSwitches();       // Check switch positions 20 times a second
        CheckHardwareTrims(); // Trims 20 times a second
        GetBank();            // Must not call too often
        TransmitterLastManaged = millis();
    }
}
/**********************************************************************************************************/
#ifdef TXMODULESUPPORT

void SendPPM()
{ // Send a frame of PPM to Third party TX module
    if (millis() - PPMdata.LastPPMFrame < 10) return;
    PPMdata.LastPPMFrame = millis();
    for (int j = 0; j < PPMdata.PPMChannelsNumber; ++j) {
        PPMdata.PPMOutputModule.write(*(PPMdata.PPMChannelOrder + j), SendBuffer[j]);
    }
}
#endif

/************************************************************************************************************/
void FixMotorChannel()
{
    if (!MotorEnabled && !BuddyON) {
        SendBuffer[MotorChannel] = IntoHigherRes(MotorChannelZero); // If safety is on, throttle will be zero whatever was shown.
    }
}

/************************************************************************************************************/
void GetBuddyData()
{
    if (BuddyMaster) GetSlaveChannelValuesPPM(); // Get buddy data and maybe use it.
}
/************************************************************************************************************/
// LOOP
/************************************************************************************************************/

FASTRUN void loop()
{
    ManageTransmitter();   // Do the needed chores ... (if there's time)
    GetNewChannelValues(); // Load SendBuffer with new servo positions very frequently
    if (CurrentMode < 3) {
        if (UseMacros) ExecuteMacro(); // Modify it if macro is running
        if (BuddyPupilOnPPM) {
            NewCompressNeeded = false; // Fake it as Buddy does not send compressed data
            ShowServoPos();
        }
        else {                 // Skip these next lines when buddying as a slave
            GetBuddyData();    // Only if master
            FixMotorChannel(); // Maybe force it low BEFORE Binding data is added
            ShowServoPos();    // Show servo positions to user
            SendBindingPipe(); // Only if not bound yet - overwrite low throttle setting
        }
    }

    switch (CurrentMode) {
        case NORMAL: // 0
#ifdef TXMODULESUPPORT
            if (!PPMdata.UseTXModule) {
                SendData(); // local TX
            }
            else {
                SendPPM(); // for TX module
                NewCompressNeeded = false;
            }
#else
            SendData();
#endif
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
        default:
            break; // CurrentMode >= 4 for no action at all.
    }
} // end loop()

/************************************************************************************************************/
