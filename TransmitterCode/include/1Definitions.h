#ifndef Definitions_H
#define Definitions_H

// ***************************************** 1Definitions.h *************************************
//            This header file has prototypes, definitions and global variables                 *
// **********************************************************************************************

// #define USE_BTLE  // testing!

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
#include "ADC-master/ADC.h"

#ifdef USE_BTLE
#include <BTLE.h>
#endif

// *************************************************************************************
//               TX VERSION NUMBER   (May 2020 - August 2025 Malcolm Messiter)       *
//**************************************************************************************

#define TXVERSION_MAJOR 2 // first three *must* match RX but _EXTRA can be different
#define TXVERSION_MINOR 5
#define TXVERSION_MINIMUS 4
#define TXVERSION_EXTRA "D 30/08/25"

// *************************************************************************************
//          DEBUG OPTIONS (Uncomment any of these for that bit of debug info)          *
//**************************************************************************************

// #define DB_NEXTION        // Debug NEXTION
// #define DB_SD             // Debug SD card data
// #define DB_CHECKSUM       // Debug 32BIT file checksum info
// #define DB_FHSS           // Debug real time FHSS data
// #define DB_SENSORS        // Debug Sensors
// #define DB_BIND           // Debug Binding
// #define DB_SWITCHES       // Debug Switches (Obsolete)
// #define DB_MODEL_EXCHANGE // Debug MODEL EXCHANGE (by RF link)
// #define DB_GAPS           // Debug Connection Gap assessment
// #define DB_IDS            // Debug Model IDs (Obsolete)
// #define DB_Variometer     // Debug Variometer
// #define DB_PACKETDATA     // Debug Packet Data

// ************************************************************************************
//                                       General                                      *
// ************************************************************************************

#define PACEMAKER 2                       // 2 ms = 500 Hz. MINIMUM ms between sent packets of data. These brief pauses allow the receiver to poll its i2c Sensor hub, and TX to ShowComms();
#define PACEMAKER_BUDDY 5                 // 5 ms = 200 Hz. MINIMUM ms between sent packets of data. These brief pauses allow the receiver to poll its i2c Sensor hub, and TX to ShowComms();
#define PACKET_HISTORY_WINDOW 200         // For success rate calculation
#define TIMEFORTXMANAGMENT 1              // 1 is plenty. takes only 1ms or so
#define MAXRESOLUTION 4095                // 12 BIT ADC Resolution
#define CE_PIN 7                          // for SPI to nRF24L01
#define CSN_PIN 8                         // for SPI to nRF24L01
#define LOSTCONTACTCUTOFF 1               // packets to lose before declaring lost contact (only one)
#define BINDINGTIME 2000                  // 2 seconds to bind ?
#define CHANNELSUSED 16                   // 16 channels in use
#define UNCOMPRESSEDWORDS 20              // these are all bigger than needed
#define COMPRESSEDWORDS 20                // these are all bigger than needed
#define SENDBUFFERSIZE 20                 // these are all bigger than needed
#define DEFAULTPIPEADDRESS 0xB7BE3E9423LL // Pipe address for startup - any value but MUST match RX
#define MAXMIXES 32                       // 32 mixes
#define TICKSPERMINUTE 60000              // millis() = 60000 per minute
#define PROPOCHANNELS 8                   // Only 4 have knobs / 2 sticks (= 4 hall sensors)
#define BANKSWITCH 4                      // Default BANK switch
#define AUTOSWITCH 1                      // Default AUTO switch
#define BANKS_USED 4                      // Flight modes (AKA Banks)
#define LOWBATTERY 42                     // Default percent for warning (User definable)
#define DEFAULTTRIMREPEATSPEED 600
#define INACTIVITYTIMEOUT 10 * TICKSPERMINUTE // Default time after which to switch off
#define INACTIVITYMINIMUM 05 * TICKSPERMINUTE // Inactivity timeout minimum is 5 minutes
#define INACTIVITYMAXIMUM 30 * TICKSPERMINUTE // Inactivity timeout maximum is 30 minutes
#define DS1307_ADDRESS 0x68                   // I2C address for RTC

#define MAXLINES 60                 // text to load at once for help screens
#define MAXNEXTIONCOMMANDLENGTH 255 //

#define DEFAULT_EXPO 50          // = ZERO EXPO (Range is 0 - 200. Below 50 is negative Expo)
#define CHARSMAX 250             // Max length for char arrays  (was 120)
#define MAXTEXTIN 1024 * 4       // 4K buffer for incoming text from Nextion
#define DEFAULTLEDBRIGHTNESS 20  // LED brightness
#define DEFAULTPOWEROFFWARNING 2 // Default time to warn before cutting power
#define MAXDUALRATE 200
#define MAXBUFFERSIZE 1024 * 6
#define MAXMODELNUMBER 91
#define RED_LED_ON_TIME 2000   // How many ms of no connection before RED led comes on
#define LOW_VOLTAGE_TIME 10000 // How many ms to endure low voltage before announcing it. (10 seconds)

#define SHOWCOMMSDELAY 100          // ms pauses between updated info on NEXTION
#define WARMUPDELAY 300             // fails at 200 so must be >200 ...
#define SCREENCHANGEWAIT 10         // allow 10ms for screen to appear
#define BATTERY_CHECK_INTERVAL 1000 // 2 seconds between battery checks

#define POWERONOFFDELAY 1000  // Delay after power OFF before transmit stops.
#define POWERONOFFDELAY2 4000 // Delay after power ON before Off is possible....
                              // and delay after power off before power on button is active
                              // **************************************************************************
                              //                            FHSS BITS                                     *
                              //***************************************************************************

#define DATARATE RF24_250KBPS   // RF24_250KBPS or RF24_1MBPS or RF24_2MBPS
#define FASTDATARATE RF24_1MBPS // 2 MBPS = RF24_2MBPS; 1 MBPS = RF24_1MBPS >> THIS IS FOR BUDDY ONLY <<

#define RETRYCOUNT 2           // was 2. Auto retries inside nRF24L01. MAX is 15. Fails below 2.
#define RETRYWAIT 1            // was 1. 250us = Wait between retries (RetryWait+1 * 250us))
#define QUIETCHANNEL 5         // This was found to be the least busy channel in the 2.4GHz band in my house
#define STOPLISTENINGDELAY 100 // 30 // 30 seems close to ideal <<<<< *********
#define SELECTTARGETDELAY 100

// **************************************************************************
//           Channel curves box position and dimentions                     *
//***************************************************************************

#define BOXLEFT 35
#define BOXTOP 35
#define BOXWIDTH 395
#define BOXHEIGHT 395
#define BOXBOTTOM BOXTOP + BOXHEIGHT
#define BOXRIGHT BOXLEFT + BOXWIDTH

// **************************************************************************
//                            CURRENTMODE VALUES                            *
//***************************************************************************

#define NORMAL 0          // Normal = transmit as usual                 (SEND DATA!)
#define CALIBRATELIMITS 1 // Calibrate limits                           (SEND NO DATA)
#define CENTRESTICKS 2    // Calibrate Centres                          (SEND NO DATA)
#define SCANWAVEBAND 3    // Scan waveband                              (SEND NO DATA)
#define SENDNOTHING 4     // Transmission off                           (SEND NO DATA)
#define PONGMODE 5        // Play Pong                                  (SEND NO DATA)
#define LISTENMODE 6      // Listen only - for wireless buddy boxing    (SEND NO DATA)

// **************************************************************************
//                               Colours                                    *
// **************************************************************************

#define Black 0
#define Blue 31
#define Brown 48192
#define Green 2016
#define Yellow 65504
#define Red 63488
#define Gray 33840
#define SkyBlue 2047
#define Purple 39070
#define Orange 64512
#define White 65535

// **************************************************************************
//                      Parameters to sent to RX IDs                        *
// **************************************************************************

// When servo frequencies, servo centre points, PID values, failsafe data, QNH settings,
// Kalman parameters (etc.) are sent to the RX, they are sent for only 5 ms in each 100 ms period.
// This is to allow the TX and RX to exchange control data too - during the remaining 95 ms.
// It therefore takes a few seconds to send all the parameters,
// but it is necessary to allow control data to be sent too.
// Parameter transmission timing and redundancy

#define PAUSE_BEFORE_PARAMETER_SEND 3000 // ms pause before sending parameters (to allow RX to prepare)
#define PARAMETER_SEND_REPEATS 4         // Each parameter is repeated this many times (in case of packet loss)
#define PARAMETER_SEND_FREQUENCY 50      // ms between parameter send slots (was 100)
#define PARAMETER_SEND_DURATION 5        // ms duration for parameter sending (remainder used for control)
#define PARAMETER_QUEUE_MAXIMUM 250      // Maximum queued parameters allowed at once

// Parameter ID definitions. Most are used for PID stabilisation, but some are used for other purposes.
#define FAILSAFE_SETTINGS 1
#define QNH_SETTING 2
#define GPS_MARK_LOCATION 3
#define PID_VALUES 4
#define KALMAN_VALUES 5
#define SERVO_FREQUENCIES 6
#define SERVO_PULSE_WIDTHS 7
#define GEAR_RATIO 8        // Gear Ratio for RPM calculation
#define PARAMETERS_MAX_ID 9 // Max types of parameters packet to send  ... will increase.

// **************************************************************************
//                               Mixes                                      *
// **************************************************************************

#define M_MIX_OUTPUTS 0 // Offsets for Mixes array ( up to 17)
#define M_Bank 1
#define M_MasterChannel 2
#define M_SlaveChannel 3
#define M_Reversed 4
#define M_Percent 5
#define M_MIX_INPUTS 6
#define M_R2 7
#define M_ONEDIRECTION 8
#define M_OFFSET 9

// **************************************************************************
//                               Screens                                    *
// **************************************************************************

#define FRONTVIEW 0
#define STICKSVIEW 1
#define GRAPHVIEW 2
#define MIXESVIEW 3
#define SCANVIEW 4
#define MODELSVIEW 5
#define CALIBRATEVIEW 6
#define TXSETUPVIEW 7
#define SUBTRIMVIEW 8
#define DATAVIEW 9
#define TRIM_VIEW 10
#define MACROS_VIEW 11
#define SWITCHES_VIEW 12
#define ONE_SWITCH_VIEW 13
#define HELP_VIEW 14
#define OPTIONS_VIEW 15
#define INPUTS_VIEW 16
#define FAILSAFE_VIEW 17
#define COLOURS_VIEW 18
#define AUDIOVIEW 19
#define FILESVIEW 20
#define REVERSEVIEW 21
#define BUDDYVIEW 22
#define LOGVIEW 23
#define TRIMDEFVIEW 24
#define OPTIONVIEW2 25
#define OPTIONVIEW3 26
#define BUDDYCHVIEW 27
#define RXSETUPVIEW1 28
#define DUALRATESVIEW 29
#define GPSVIEW 30
#define RXSETUPVIEW 31
#define BANKSNAMESVIEW 32
#define SLOWSERVOVIEW 33
#define RENAMEMODELVIEW 34
#define FILEEXCHANGEVIEW 35
#define TXMODULEVIEW 36
#define PONGVIEW 37
#define IDCHECKVIEW 38
#define BLANKVIEW 39
#define TYPEVIEW 40
#define SERVOTYPESVIEW 41
#define LOGFILESLISTVIEW 42
#define PIDVIEW 43
#define KALMANVIEW 44

// **************************************************************************
//                          Switches' GPIOs                                 *
// **************************************************************************

#define SWITCH0 32 // EDGE SWITCHES' PIN NUMBERS ...
#define SWITCH1 31
#define SWITCH2 30
#define SWITCH3 29
#define SWITCH4 28
#define SWITCH5 27
#define SWITCH6 26
#define SWITCH7 25

// **************************************************************************
//                           TRIMS' GPIOs                                   *
// **************************************************************************

#define TRIM1A 34 // Digital trims pins
#define TRIM1B 35
#define TRIM2A 36
#define TRIM2B 37
#define TRIM3A 38
#define TRIM3B 39
#define TRIM4A 40
#define TRIM4B 41

// **************************************************************************
//                LED and Power off GPIOs                                   *
// **************************************************************************

#define REDLED 2 // COLOURED LEDS' PIN NUMBERS ...
#define GREENLED 3
#define BLUELED 4
#define POWER_OFF_PIN 5
#define BUTTON_SENSE_PIN 33

// **************************************************************************
//               Sounds                             *
//***************************************************************************

#define CLICKZERO 0
#define CLICKONE 1
#define ONEMINUTE 2
#define TWOMINUTES 3
#define THREEMINUTES 4
#define FOURMINUTES 5
#define FIVEMINUTES 6
#define SIXMINUTES 7
#define SEVENMINUTES 8
#define EIGHTMINUTES 9
#define NINEMINUTES 10
#define TENMINUTES 11
#define BANKONE 12
#define BANKTWO 13
#define BANKTHREE 14
#define BANKFOUR 15
#define BEEPMIDDLE 16
#define BEEPCOMPLETE 17
#define THEFANFARE 18
#define BATTERYISLOW 19
#define CONNECTEDMSG 20
#define DISCONNECTEDMSG 21
#define BUDDYMSG 22
#define MASTERMSG 23
#define WEAKMSG 23
#define WHAHWHAHMSG 25
#define BINDSUCCEEDED 26
#define BINDNEEDED 27
#define MMFOUND 28
#define MMMATCHED 29
#define MMNOTFOUND 30 // Model not found or 75!!
#define MOTORON 31
#define MOTOROFF 32
#define STORAGECHARGE 33
#define RATE1 34
#define RATE2 35
#define RATE3 36
#define PLSTURNOFF 37
#define AEROBATICS 38
#define AUTO 39
#define CRUISE 40
#define FLAPS 41
#define HOVER 42
#define IDLE1 43
#define IDLE2 44
#define LANDING 45
#define LAUNCH 46
#define NORMALB 47
#define SPEED 48
#define TAKEOFF 49
#define THERMAL 50
#define THRHOLD 51
#define THREEDEE 52
#define BFM1 53
#define BFM2 54
#define BFM3 55
#define BFM4 56
#define AIRBRAKES 57
#define STUNT1 58
#define STUNT2 59
#define WHEELSDOWN 60
#define WHEELSUP 61
#define TEN 62
#define NINE 63
#define EIGHT 64
#define SEVEN 65
#define SIX 66
#define FIVE 67
#define FOUR 68
#define THREE 69
#define TWO 70
#define ONE 71
#define MMSAVED 72
#define SAFEON 73
#define SAFEOFF 74
#define NOTFOUND 75 // or 30 !! :-)
#define WINDOWS1 76
#define WINDOWS2 77

#define GOINGUP1 78 // variometer sounds up...
#define GOINGUP2 79
#define GOINGUP3 80
#define GOINGUP4 81
#define GOINGUP5 82
#define GOINGUP6 83
#define GOINGUP7 84
#define GOINGUP8 85
#define GOINGUP9 86
#define GOINGUP10 87

#define GOINGDOWN1 88 // variometer sounds down...
#define GOINGDOWN2 89
#define GOINGDOWN3 90
#define GOINGDOWN4 91
#define GOINGDOWN5 92
#define GOINGDOWN6 93
#define GOINGDOWN7 94
#define GOINGDOWN8 95
#define GOINGDOWN9 96
#define GOINGDOWN10 97

#define BINDINGENABLED 98 // binding enabled sound
#define BUDDYMASTERON 99  // Buddy Master on sound
#define BUDDYPUPILON 100  // Buddy Pupil on sound
#define EXTRABUDDIES 101  // Extra Buddies found sound
#define TXNOTBOUND 102    //  TX not bound sound // not yet used
#define NUDGE_MSG 103     // buddy is in nudge mode.

// **************************************************************************
//               Three BUDDY states now possible                            *
//***************************************************************************

#define BUDDY_OFF 0   // Master has control
#define BUDDY_NUDGE 1 // Master can Nudge but buddy has mostly control.
#define BUDDY_ON 2    // Buddy has control

#define MASTER_HAS_CONTROL 0 // possible values for CurrentBuddyState
#define SLAVE_HAS_CONTROL 1
#define MASTER_CAN_NUDGE 2

// **************************************************************************
//               SDCARD MODEL MEMORY CONSTANTS                              *
//***************************************************************************

#define TXSIZE 512            // SD space reserved for transmitter (WAS  250)
#define MODELSIZE 2048        // SD space reserved for each model (WAS 1600)
#define MAXFILELEN (1024 * 3) // 3?? MAX SIZE FOR HELP AND LOG FILES
#define MAXBACKUPFILES 95

// **************************************************************************
//                            SERVO RANGE PARAMETERS                        *
//***************************************************************************

#define MINMICROS 500
#define MAXMICROS 2500
#define HALFMICROSRANGE (MAXMICROS - MINMICROS) / 2
#define MIDMICROS MINMICROS + HALFMICROSRANGE

// **************************************************************************
//                           nRF24L01 lines on off                          *
// **************************************************************************

#define CSN_ON LOW
#define CSN_OFF HIGH
#define CE_ON HIGH
#define CE_OFF LOW

// **************************************************************************
//                            Error Codes                                *
// **************************************************************************

#define NOERROR 0
#define MODELSFILENOTFOUND 1
#define CHECKSUMERROR 2
#define MOTORISON 3

// **************************************************************************
//                            Interpolations                                *
// **************************************************************************

#define STRAIGHTLINES 0
#define SMOOTHEDCURVES 1
#define EXPONENTIALCURVES 2

// ********************* Offsets within macros' buffer ***********************

#define MACROTRIGGERCHANNEL 0 // 1 - 16. 0 means dissabled.
#define MACROSTARTTIME 1      // In ** >> 10ths << ** of a second since trigger. ( = millis() * 100 ) up to 25.4 seconds
#define MACRODURATION 2       // In ** >> 10ths << ** of a second since start    ( = millis() * 100 ) up to 25.4 seconds
#define MACROMOVECHANNEL 3    // Which channel to move.
#define MACROMOVETOPOSITION 4 // Where to put said channel for said duration. (0 - 180)
#define MACRORUNNINGNOW 5     // Running flag (BIT 0 running/not running,  BIT 1 = Timer active / inactive)

// **************************************************************************
//                              Macros                                      *
// **************************************************************************

#define MAXMACROS 8     // 8 macros enough for now?
#define BYTESPERMACRO 6 // 6 bytes each

// **************************************************************************
//                          NEXTION SERIAL CONNECTION                       *
//***************************************************************************

#define NEXTION Serial1 // NEXTION is connected to Serial1

// **************************************************************************
//                            WATCHDOG                            *
//***************************************************************************

#define WATCHDOGTIMEOUT 2500 // 2.5 Seconds before reboot (32ms -> 500 seconds)
#define KICKRATE 1000        // Kick interval (must be between WATCHDOGMAXRATE and WATCHDOGTIMEOUT)
#define WATCHDOGMAXRATE 250  // 250 ms secs between kicks is max rate allowed

//***************************************************************************
//                                     PONG                                 *
//***************************************************************************

#define PONGX1 20                                    // BOX dimentions
#define PONGX2 790                                   // BOX dimentions
#define PONGY1 60                                    // BOX dimentions (was 50)
#define PONGY2 410                                   // BOX dimentions
#define PONGGOALSIZE 180                             // Size of goal
#define PONGBALLSIZE 7                               // Size of ball
#define PONGSPEED 10                                 // Frame rate
#define PONGBALLSPEED 5                              // Ball movement per frame
#define PONGCLEAR (PONGBALLSPEED + PONGBALLSIZE) + 4 // ball clearance from box when bouncing
#define GOALTOP (PONGY1 + ((PONGY2 - PONGY1) / 2)) - (PONGGOALSIZE / 2)
#define GOALBOT (PONGY1 + ((PONGY2 - PONGY1) / 2)) + (PONGGOALSIZE / 2)
#define STARTX PONGX1 + ((PONGX2 - PONGX1) / 2) // start position of ball
#define STARTY PONGY1 + ((PONGY2 - PONGY1) / 2) + 90
#define PADDLEHEIGHT 60
#define PADDLEGAP 40
#define LEFTPADDLEX PONGX1 + PADDLEGAP
#define RIGHTPADDLEX PONGX2 - PADDLEGAP
#define EXTRAPONG 38

// **************************************************************************
//                           Screenposition                                 *
//***************************************************************************

#define SCREEN_X "tch2"
#define SCREEN_Y "tch1"

// **************************************************************************
//                               MIXES                                      *
//***************************************************************************

#define MIXINPUT 0
#define MIXOUTPUT 1
#define BANK 2
#define MASTERCHANNEL 3
#define SLAVECHANNEL 4
#define ONEDIRECTION 5
#define REVERSED 6
#define OFFSET 7
#define PERCENT 8

// **************************************************************************
//                            Function Prototypes                           *
//***************************************************************************

ADC *adc = new ADC();
void KickTheDog();
void SendCommand(char *tbox);
void ReadTheSwitchesAndTrims();
void ShowComms();
void SendCharArray(char *ch0, char *ch1, char *ch2, char *ch3, char *ch4, char *ch5, char *ch6, char *ch7, char *ch8, char *ch9, char *ch10, char *ch11, char *ch12);
char *Str(char *s, int n, int comma);
void GetNewChannelValues();
void GreenLedOn();
void CheckGapsLength();
FASTRUN void ParseLongerAckPayload();
void FailedPacket();
void StartInactvityTimeout();
void ShowServoPos();
void ZeroDataScreen();
void RedLedOn();
int InStrng(char *text1, char *text2);
void ReadCheckSum32();
void ResetTransmitterSettings();
void TryToReconnect();
void FlushFifos();
FLASHMEM void SetDS1307ToCompilerTime();
int GetOtherValue(char *nbox);
void CheckInvisiblePoint();
void GotoFrontView();
void CheckDualRatesValues();
void UpdateLED();
void CheckMotorOff();
bool GetButtonPress();
void CheckPowerOffButton();
void LoadFileSelector();
void LoadModelSelector();
void PlaySound(uint16_t TheSound);
uint8_t CheckPipeNibbles(uint8_t b);
void InitRadio(uint64_t Pipe);
void SetThePipe(uint64_t WhichPipe);
void DoScanInit();
void DoScanEnd();
void HopToNextChannel();
void ScanAllChannels(bool cls);
void SendData();
void DrawFhssBox();
void SendText(char *tbox, char *NewWord); // needed a prototype or two here!
void RestoreBrightness();
void ButtonWasPressed();
void CalibrateEdgeSwitches();
void DisplayCurve();
void DrawLine(int x1, int y1, int x2, int y2, int c);
void DrawBox(int x1, int y1, int x2, int y2, int c);
void FillBox(int x1, int y1, int w, int h, int c);
void LogConnection();
void LogDisConnection();
void CloseLogFile();
void LogLongestGap();
void LogThisModel();
void Force_ReDisplay();
FASTRUN void Compress(uint16_t *compressed_buf, uint16_t *uncompressed_buf, uint8_t uncompressed_size);
FASTRUN void Decompress(uint16_t *uncompressed_buf, uint16_t *compressed_buf, uint8_t uncompressed_size);
FASTRUN void BufferTeensyMACAddPipe();
void ExecuteMacro();
void LogTimer(uint32_t Mins);
FASTRUN void LogText(char *TheText, uint16_t len, bool TimeStamp);
void LogAverageFrameRate();
void ShowBank();
void UpdateModelsNameEveryWhere();
void ResetAllTrims();
void CheckTrimValues();
void ClearSuccessRate();
int CheckRange(int v, int min, int max);
void MoveaTrim(uint8_t i);
FASTRUN void LogSafety(bool On);
void ShowMotor(int on);
void StartModelSetup();
bool GetConfirmation(char *goback, char *Prompt);
void GotoModelsView();
void SaveCurrentModel();
bool CheckModelName();
int AnalogueReed(uint8_t InputChannel);
void DelayWithDog(uint32_t HowLong);
void SaveTransmitterParameters();
void PlayPong();
void StartPong();
FASTRUN void ButtonWasPressed();
bool GetButtonPress();
void EndSend();
void ReadTheRTC();
void swap(uint8_t *a, uint8_t *b);
void SaveOneModel(uint32_t mnum);
bool ReadOneModel(uint32_t Mnum);
void SaveAllParameters();
void BindNow();
FASTRUN uint32_t GetIntFromAckPayload(); // This one uses a uint32_t int
uint32_t getvalue(char *nbox);
uint32_t GetValue(char *nbox);
void SendValue(char *nbox, int value);
void LogTotalLostPackets();
void LogTotalGoodPackets();
void LogOverallSuccessRate();
void ClearText();
void BuildDirectory();
void ShowFileNumber();
void MsgBox(char *goback, char *Prompt);
void ShowRemoteID();
void CloseModelsFile();
void ShortishDelay();
void ShortDelay();
void BlueLedOn();
void NormaliseTheRadio();
void ConfigureRadio();
uint16_t MakeTwobytes(bool *f);
void SendSpecialPacket();
void GetSpecialPacket();
void StartBuddyListen();
void RationaliseBuddy();
void ShowConnectionQuality();
void GetSlaveChannelValuesWireless();
void GetTeensyMacAddress();
FASTRUN void LogThisGap();
void GetRXVersionNumber();
void GetRXVersionNumber();
void CompareModelsIDs();
void OpenModelsFile();
uint8_t SDRead8BITS(int p_address);
short int SDRead16BITS(int p_address);
void UpdateButtonLabels();
void SDUpdate32BITS(int p_address, uint32_t p_value);
void SDUpdate8BITS(int p_address, uint8_t p_value);
uint32_t SDRead32BITS(int p_address);
void CheckSavedTrimValues();
void CheckMacrosBuffer();
void FixMotorChannel();
void SendInitialSetupParams();
void AddParameterstoQueue(uint8_t ID);
void SetDefaultValues();
FASTRUN void LogThisRX();
void CompareVersionNumbers();
void PopulateFrontView();
void PopulateDataView();
void PopulateGPSView();
void CheckScreenTime();
void CheckBatteryStates();
void ShowCurrentRate();
void ShowAMS();
void ShowTrimToAll();
FASTRUN bool CheckTXVolts();
FASTRUN bool CheckRXVolts();
bool MayBeAddZero(uint8_t nn);
void SendText1(char *tbox, char *NewWord);
void ForceDataRedisplay();
void TrimsToSubtrim();
void LogBuddyChange();
void SetUpTargetForBuddy();
FASTRUN uint16_t ReadThreePositionSwitch(uint8_t l); // This returns the input only
void UpdateSpeedScreen();
void SetNewDualRate();
void CheckSelectedRatesMode();
void GetCurveDots(uint16_t OutputChannel, uint16_t TheRate); // This for the Dual Rates function
void CheckDualRatesValues();
void ReadDualRatesValues();
void DisplayDualRateValues();
void PopulateMacrosView();
void LoadModelForRenaming();
bool GetBackupFilename(char *goback, char *tt1, char *MMname, char *heading, char *pprompt);
void FixFileName();
void WriteBackup();
void RestoreCurrentModel();
void GetYesOrNo();
uint16_t GetText(char *TextBoxName, char *TheText);
void StoreModelID();
void ResetMotorTimer();
void SpeedTest();
void StartLogFilesListScreen();
void EndLogFilesListScreen();
void LoadNewLogFile();
void DeleteThisLogFile();
void SortDirectory();
void ClearFilesList();
FASTRUN void LogModelMatched();
FASTRUN void LogModelFound();
FASTRUN void LogModelNotFound();
void SendHelp();
FASTRUN void MakeLogFileName();
void GetCommandbytes(uint8_t *C, uint8_t *C1);
void TestTheCommandByte(uint8_t C, uint8_t C1);
void LogRXVoltsPerCell();
void LogTXVoltsPerCell();
void LogStopFlyingMsg();
void LogNewRateInUse();
void LogTotalRXSwaps();
void LogTimeSinceBoot();
void LogConnectedDuration();
void LogTouched();
void RefreshDualRatesNew();
int GetIntFromTextIn(uint8_t offset);
void ShowLogFileNew(uint16_t LinesCounter);
uint16_t ReadAFewLines();
void LogVIEWNew();
File OpenTheLogFileForReading();
void StartLogFileView();
void LogTotalRXGoodPackets();
void LogTotalRXGoodPackets();
void LogTotalPacketsAttempted();
void DelaySimple(uint32_t ms);
uint8_t GetSwitchPosition(uint8_t Sw_Number);
FASTRUN void LogAverageGap();
void ReadChannelSwitches9to12();
int GetExtraParameters();
void ShowSendingParameters();
float SDReadFLOAT(int p_address);
void SDUpdateFLOAT(int p_address, float p_value);
void GetBank();
void LogRPM(uint32_t RPM);
#ifdef USE_BTLE
void SendViaBLE();
#endif
// **************************************************************************
//                            GLOBAL DATA                                   *
//***************************************************************************

RF24 Radio1(CE_PIN, CSN_PIN);
#ifdef USE_BTLE
BTLE btle(&Radio1);
#endif

/************************************************************************************************************/
/************************************************************************************************************/
// // For numeric types (int, float, double, etc.)
template <typename T>
void Look(const T &value, int format)
{
    Serial.println(value, format);
}

template <typename T>
void Look1(const T &value, int format)
{
    Serial.print(value, format);
}

// Fallback for types where a format doesn't apply (e.g., String, const char*)
template <typename T>
void Look(const T &value)
{
    Serial.println(value);
}

template <typename T>
void Look1(const T &value)
{
    Serial.print(value);
}

// ******************************************************************************************************************************************************************

WDT_T4<WDT3> TeensyWatchDog;
WDT_timings_t WatchDogConfig;
uint8_t Mixes[MAXMIXES + 1][17];                       // 17 possible elements per mix. NOTHING to do with channels count!!!
int Trims[BANKS_USED + 1][CHANNELSUSED + 1];           // Trims to store
uint8_t Exponential[BANKS_USED + 1][CHANNELSUSED + 1]; // Exponential
uint8_t InterpolationTypes[BANKS_USED + 1][CHANNELSUSED + 1];
uint8_t LastMixNumber = 1;
uint8_t MixNumber = 0;
uint8_t CurrentView = FRONTVIEW;
uint8_t SavedCurrentView = FRONTVIEW;
uint64_t DefaultPipe = DEFAULTPIPEADDRESS;      //          Default Radio pipe address
uint64_t TeensyMACAddPipe = DEFAULTPIPEADDRESS; //          New Radio pipe address for binding will come from MAC address
uint64_t BuddyMACAddPipe = DEFAULTPIPEADDRESS;  //          Buddy pipe address
char TextIn[MAXTEXTIN + 2];                     //          Spare space
uint16_t PacketsPerSecond = 0;
uint8_t PacketsHistoryBuffer[PACKET_HISTORY_WINDOW + 1]; // Here we record some history
uint32_t TotalLostPackets = 0;
uint32_t TotalGoodPackets = 0;
uint16_t PacketNumber = 0;
uint8_t GPSMarkHere = 0;
uint16_t TrimRepeatSpeed = 600;
char na[] = "";

uint8_t ServoSpeed[BANKS_USED + 1][CHANNELSUSED + 1]; //    Speed of servo movement
uint16_t CurrentPosition[SENDBUFFERSIZE + 1];         //    Position from which a slow servo started (0 = not started yet)
uint16_t SendBuffer[SENDBUFFERSIZE + 1];              //    Data to send to rx (16 words)
uint16_t BuddyBuffer[SENDBUFFERSIZE + 1];             //    Data from wireless buddy (16 words)
uint16_t ShownBuffer[SENDBUFFERSIZE + 1];             //    Data shown before
uint16_t RawDataBuffer[SENDBUFFERSIZE + 1];           //    Data as actually sent
uint16_t InputsBuffer[CHANNELSUSED + 1];              //    Data from pots
uint16_t LastBuffer[CHANNELSUSED + 1];                //    Used to spot any change
uint16_t PreMixBuffer[CHANNELSUSED + 1];              //    Data collected from sticks
uint8_t MaxDegrees[5][CHANNELSUSED + 1];              //    Max degrees (180)
uint8_t MidHiDegrees[5][CHANNELSUSED + 1];            //    MidHi degrees (135)
uint8_t CentreDegrees[5][CHANNELSUSED + 1];           //    Middle degrees (90)
uint8_t MidLowDegrees[5][CHANNELSUSED + 1];           //    MidLow Degrees (45)
uint8_t MinDegrees[5][CHANNELSUSED + 1];              //    Min Degrees (0)
uint8_t SubTrims[CHANNELSUSED + 1];                   //    Subtrims
uint8_t SubTrimToEdit = 1;
uint32_t LastPacketSentTime = 0;
uint8_t Bank = 1;
// User defined bank names zone
// ************************************** 0                  1                 2                  3                4          5           6           7          8                9        10          11       12        13             14            15          16          17      18        19           20         21     22           23         24         25          26           27        ***
char BankNames[28][14] = {{"Flight mode 1"}, {"Flight mode 2"}, {"Flight mode 3"}, {"Flight mode 4"}, {"Bank 1"}, {"Bank 2"}, {"Bank 3"}, {"Bank 4"}, {"Aerobatics"}, {"Auto"}, {"Cruise"}, {"Flaps"}, {"Hover"}, {"Idle up 1"}, {"Idle up 2"}, {"Landing"}, {"Launch"}, {"Normal"}, {"Speed"}, {"Takeoff"}, {"Thermal"}, {"Hold"}, {"3D"}, {"Brakes"}, {"Stunt 1"}, {"Stunt 2"}, {"Gear up"}, {"Gear down"}};
uint8_t BankSounds[28] = {BFM1, BFM2, BFM3, BFM4, BANKONE, BANKTWO, BANKTHREE, BANKFOUR, AEROBATICS, AUTO, CRUISE, FLAPS, HOVER, IDLE1, IDLE2, LANDING, LAUNCH, NORMALB, SPEED, TAKEOFF, THERMAL, THRHOLD, THREEDEE, AIRBRAKES, STUNT1, STUNT2, WHEELSDOWN, WHEELSUP};
uint8_t BanksInUse[4] = {0, 1, 2, 3};
uint8_t PreviousBank = 1;
// ************************************
char ChannelNames[CHANNELSUSED][11] = {{"Aileron"}, {"Elevator"}, {"Throttle"}, {"Rudder"}, {"Gear"}, {"AUX1"}, {"AUX2"}, {"AUX3"}, {"AUX4"}, {"AUX5"}, {"AUX6"}, {"AUX7"}, {"AUX8"}, {"AUX9"}, {"AUX10"}, {"AUX11"}};
uint8_t DualRateInUse = 1;
uint8_t PreviousDualRateInUse = 1;
uint16_t PreviousBuffer[SENDBUFFERSIZE + 1]; //     Used to spot any change
uint16_t ChannelMax[CHANNELSUSED + 1];       //    output of pots at max
uint16_t ChannelMidHi[CHANNELSUSED + 1];     //    output of pots at MidHi
uint16_t ChannelCentre[CHANNELSUSED + 1];    //    output of pots at Centre
uint16_t ChannelMidLow[CHANNELSUSED + 1];    //    output of pots at MidLow
uint16_t ChannelMin[CHANNELSUSED + 1];       //    output of pots at min
uint16_t ChanneltoSet = 0;
bool Connected = false;
uint16_t BuddyControlled = 0; // Flags
bool BuddyHasAllSwitches = true;
double PointsCount = 5; // This for displaying curves only
double xPoints[5];
double yPoints[5];
double xPoint = 0;
double yPoint = 0;
uint16_t ClickX;
uint16_t ClickY;
uint8_t SticksMode = 2;
uint8_t SavedSticksMode = 2;
uint16_t AnalogueInput[PROPOCHANNELS] = {A0, A1, A2, A3, A6, A7, A8, A9};                 // default definition of first 8 PROPO Channels inputs // must fix the order for mode 2
uint8_t TrimNumber[8] = {TRIM1A, TRIM1B, TRIM2A, TRIM2B, TRIM3A, TRIM3B, TRIM4A, TRIM4B}; // These too can get swapped over later
uint8_t CurrentMode = NORMAL;
uint32_t MotorStartTime = 0;
uint32_t LastSeconds = 0;
uint32_t Secs = 0;
uint32_t MotorOnSeconds = 0;
uint32_t PausedSecs = 0;
uint32_t Mins = 0;
uint32_t Hours = 0;
uint32_t ModelNumber = 1;
uint32_t SavedModelNumber = 1;
uint32_t PreviousModelNumber = 1;
uint8_t ModelDefined = 0;
uint16_t MemoryForTransmtter = 0;                                                                          // SD space for transmitter
uint16_t OneModelMemory = 0;                                                                               // SD space for every model
uint32_t SDCardAddress = 0;                                                                                // Address on SD card (offset from zero)
uint8_t SwitchNumber[8] = {SWITCH0, SWITCH1, SWITCH2, SWITCH3, SWITCH4, SWITCH5, SWITCH6, SWITCH7};        // These can get swapped over later
uint8_t DefaultSwitchNumber[8] = {SWITCH0, SWITCH1, SWITCH2, SWITCH3, SWITCH4, SWITCH5, SWITCH6, SWITCH7}; // Default values
bool DefiningTrims = false;
bool TrimDefined[4] = {true, true, true, true};
char ModelName[40] = "Untitled";
uint16_t ScreenTimeout = 120; // Screen has two minute timeout by default
int LastLinePosition = 0;
uint8_t RXCellCount = 2;
bool JustHoppedFlag = true;
bool LostContactFlag = true;
uint32_t RecentPacketsLost = 0;
uint32_t GapSum = 0;
uint32_t GapLongest = 0;
uint32_t GapStart = 0;
uint32_t ThisGap = 0;
uint32_t GapAverage = 0;
uint32_t GapCount = 0;
// *********************************** RX GPS ***************************************
float GPS_RX_Latitude = 0;
float GPS_RX_Longitude = 0;
float GPS_RX_Altitude = 0;
float GPS_RX_ANGLE = 0;
bool GPS_RX_FIX = 0;
uint8_t GPS_RX_Satellites = 0;
float GPS_RX_Speed = 0;
float GPS_RX_MaxSpeed = 0;
uint8_t GPS_RX_Hours = 0;
uint8_t GPS_RX_Mins = 0;
uint8_t GPS_RX_SECS = 0;
uint8_t GPS_RX_DAY = 0;
uint8_t GPS_RX_MONTH = 0;
uint8_t GPS_RX_YEAR = 0;
float GPS_RX_Maxaltitude = 0;
float GPS_RX_GroundAltitude = 0;
float GPS_RX_DistanceTo = 0;
float GPS_RX_CourseTo = 0;
float GPS_RX_MaxDistance = 0;
float RXModelVolts = 0;
float RXModelAltitude = 0;
float RXModelAltitudeBMP280 = 0;
float RXMAXModelAltitude = 0;
float GroundModelAltitude = 0;
float RXTemperature = 0;
float MaxAlt = 0;
char ModelTempRX[11] = {'0', 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0};
char ModelAltitude[31] = {'0', 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0};
char Maxaltitude[31] = {'0', 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0};
char ReceiverVersionNumber[20];
char TransmitterVersionNumber[20];
char ModelVolts[11] = {'0', 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0};
float RXVoltsPerCell = 0;
float TXVoltsPerCell = 0;
File ModelsFileNumber;
Adafruit_INA219 ina219;
char SingleModelFile[40];
bool SingleModelFlag = false;
bool ModelsFileOpen = false;
bool USE_INA219 = false;
bool BoundFlag = false;
bool Switch[8];
bool TrimSwitch[8];
uint8_t BankSwitch = BANKSWITCH;
uint8_t Autoswitch = Autoswitch;
uint8_t SafetySwitch = 0;
uint8_t BuddySwitch = 0;
uint8_t DualRatesSwitch = 0;

// **************************************************************************
//                Top switch Channel numbers                                   *
// **************************************************************************

#define Ch9_SW 0
#define Ch10_SW 1
#define Ch11_SW 2
#define Ch12_SW 3

uint8_t TopChannelSwitch[4] = {0, 0, 0, 0};
uint8_t TopChannelSwitchValue[4] = {0, 0, 0, 0};

bool SwitchReversed[4] = {
    false,
    false,
    false,
    false};

uint16_t StartLocation = 0;
bool ValueSent = false;
uint8_t SwitchEditNumber = 0; // number of switch being edited
uint32_t ShowServoTimer = 0;
bool LastFourOnly = false;
uint8_t InPutStick[17] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};    // User defined stick inputs
uint8_t ChannelOutPut[17] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}; // User defined channel outputs
uint8_t InputTrim[4] = {0, 1, 2, 3};                                                // User defined trim inputs
uint8_t ExportedFileCounter = 0;
char TheFilesList[150][24];
uint16_t FileNumberInView = 0;
bool FileError = false;
uint32_t RangeTestStart = 0;
uint16_t RecentGoodPacketsCount = 0;
uint8_t SaveBank = 0;
bool FailSafeChannel[CHANNELSUSED + 1];
bool SaveFailSafeNow = false;
uint32_t FailSafeTimer;
uint16_t LogLineNumber = 0;
struct CD
{
    uint16_t ChannelBitMask = 0;
    uint16_t CompressedData[COMPRESSEDWORDS + 12]; // Much Bigger than needed for safety
};
CD DataTosend;

struct CD2
{
    uint16_t ID = 0;
    uint16_t word[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};
CD2 Parameters;
uint8_t SizeOfParameters = sizeof(Parameters);

struct CD_buddy
{
    uint16_t ChannelBitMask = 0;
    uint16_t CompressedData[20]; // 40 bytes ... far too big
};
uint16_t RawDataIn[21]; //  21 x 16 BIT words

CD_buddy DataReceived;

uint16_t SizeOfDataTosend = sizeof(DataTosend);
uint8_t SizeOfCompressedData = sizeof(DataTosend.CompressedData);

uint32_t Inactivity_Timeout = INACTIVITYTIMEOUT;
uint32_t Inactivity_Start = 0;
tmElements_t tm;
char TxName[40] = "Unknown";
uint32_t LastTimeRead = 0;
uint32_t LastScanButtonCheck = 0;

uint32_t LastShowTime = 0;
uint8_t MacAddress[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t MasterMacAddress[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t BuddyMacAddress[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t ErrorState = 0;
uint16_t XtouchPlace = 0; // Clicked X
uint16_t YtouchPlace = 0; // Clicked Y

// changing these four valiables controls LED blink and speed

bool LedIsBlinking = false;
float BlinkHertz = 2;
uint32_t BlinkTimer = 0;
bool LedWasGreen = true;
bool LedWasRed = false;
char ThisRadio[6] = "0 ";
uint8_t LastRadio = 0;
uint8_t NextChannel = 0;
bool WirelessBuddy = false;
bool BuddyPupilOnWireless = false;
bool WasBuddyPupilOnWireless = false;
bool BuddyMasterOnWireless = false;
uint8_t CurrentBuddyState = 0;
float Qnh = 1009; // pressure at sea level here
uint16_t LastModelLoaded = 0;
uint16_t LastFileInView = 0;
uint8_t MinimumGap = 75;
int RecentStartLine = 0; // we need the signed version
char RecentTextFile[30];
bool LogRXSwaps = false;
bool ThereIsMoreToSee = false;
bool UseLog = false;
uint8_t Gsecond;   // = tm.Second; // 0-59
uint8_t Gminute;   // = tm.Minute; // 0-59
uint8_t Ghour;     // = tm.Hour;   // 0-23
uint8_t GweekDay;  // = tm.Wday;   // 1-7
uint8_t GmonthDay; // = tm.Day;    // 1-31
uint8_t Gmonth;    // = tm.Month;  // 1-12
uint8_t Gyear;     // = tm.Year;   // 0-99
bool GPSTimeSynched = false;
short int DeltaGMT = 0;

uint16_t TrimMultiplier = 5; // By how much to multiply trim
uint8_t DateFix = 0;
uint16_t BackGroundColour = 214;
uint16_t ForeGroundColour = White;
uint16_t HighlightColour = Yellow;
uint16_t SpecialColour = Red;
bool Reconnected = false;
uint8_t LowBattery = LOWBATTERY;
uint16_t SbusRepeats = 0;
bool RXVoltsDetected = false;
uint16_t RadioSwaps = 0;
uint16_t RX1TotalTime = 0;
uint16_t RX2TotalTime = 0;
uint8_t AudioVolume = 50;
uint32_t WarningTimer = 0;
uint32_t ScreenTimeTimer = 0;
bool ScreenIsOff = false;
uint8_t Brightness = 100;
bool UseVariometer = false;
bool PlayFanfare = true;
bool TrimClicks = true;
bool SpeakingClock = true;
bool ClockSpoken = false;
bool ClockSpoken1 = false;
bool AnnounceBanks = true;
bool AnnounceConnected = true;
bool CopyTrimsToAll = true;
uint16_t ReversedChannelBITS = 0; // 16 BIT for 16 Channels
uint16_t SavedLineX = 12345;
uint8_t ReConnectChannel = 0;
File LogFileNumber;
bool LogFileOpen = false;
bool ShowVPC = false;
short int TxVoltageCorrection = 0;
short int RxVoltageCorrection = 0;
uint16_t ServoCentrePulse[11] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // 11 channels for servo centre pulse
uint16_t ServoFrequency[11] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};       // 11 channels for servo frequency
uint8_t LEDBrightness = DEFAULTLEDBRIGHTNESS;                                     // 0-255
uint8_t PowerOffWarningSeconds = 2;
uint8_t ConnectionAssessSeconds = 1;
uint32_t PreviousPowerOffTimer = 0;
bool ModelIdentified = false;
bool ModelMatched = false;
bool AutoModelSelect = true;
uint32_t TrimTimer = 0;
uint32_t LastPassivePacketTime = 0;

union uMacReceived
{
    uint64_t Val64 = 0;
    uint32_t Val32[2];
    uint8_t Val8[8]; // Model's Mac address just obtained from model
} ModelsMacUnion;

union uMacStored
{
    uint64_t Val64 = 0;
    uint32_t Val32[2];
    uint8_t Val8[8]; // Model's Mac address that had been saved on disk
} ModelsMacUnionSaved;
bool MotorEnabled = false;
bool SendNoData = false;
bool MotorWasEnabled = false;
uint8_t MotorChannel = 2;
uint8_t MotorChannelZero = 30;
bool UseMotorKill = true;
bool SafetyON = false;
uint8_t BuddyState = BUDDY_OFF; // three possible states now. BUDDY_OFF, BUDDY_NUDGE, BUDDY_ON
bool SafetyWasOn = false;
u_int8_t WarningSound = BATTERYISLOW;
float StopFlyingVoltsPerCell = 0;
uint16_t SFV = 0; // =StopFlyingVoltsPerCell * 100
bool NewCompressNeeded = true;
uint32_t FileCheckSum = 0;
bool DoingCheckSm = false;
bool RecursedAlready = false;
bool TXLiPo = false;
uint8_t CurrentPoint = 1;
bool UseDualRates = false;
uint8_t Drate1 = 100;
uint8_t Drate2 = 75;
uint8_t Drate3 = 50;

uint8_t DualRateChannels[8] = {1, 2, 4, 0, 0, 0, 0, 0};
uint8_t DualRateRate[5];

uint8_t DualRateValue = 100;
uint16_t CurveDots[5];
char Confirmed[2];
char NewFileBuffer[MAXFILELEN];
uint16_t NewFileBufferPointer = 0;
uint8_t ReconnectionIndex = 0;
bool TimerDownwards = false;
uint16_t TimerStartTime = 5 * 60;
bool TimesUp = false;
uint8_t CountDownIndex = 0;
uint8_t MacrosBuffer[MAXMACROS][BYTESPERMACRO]; // macros' buffer
uint32_t MacroStartTime[MAXMACROS];
uint32_t MacroStopTime[MAXMACROS];
uint8_t PreviousMacroNumber = 1;
bool UseMacros = false;
uint8_t ScanSensitivity = 42;
uint8_t CurrentChannel = 0;
uint8_t PupilIsAlive = 0;
uint8_t MasterIsAlive = 0;
bool VersionsCompared = false;
uint32_t LedGreenMoment = 0;
bool BeQuiet = false;
bool ReconnectingNow = true;
uint32_t LastHopTime = 0; //  Time of last hop
uint16_t ParametersToBeSent[PARAMETER_QUEUE_MAXIMUM + 1];
uint8_t ParametersToBeSentPointer = 0;
bool UsingDefaultPipeAddress = true;
bool DontChangePipeAddress = false;
bool LastAutoModelSelect = false;
bool LastCopyTrimsToAll = false;
uint8_t OldRate = 0;
bool ForceVoltDisplay = false;
uint16_t LastPacketsPerSecond = 0;
uint16_t LastLostPackets = 0;
uint32_t LastGapLongest = 0;
uint16_t LastRadioSwaps = 0;
uint16_t LastRX1TotalTime = 0;
uint16_t LastRX2TotalTime = 0;
uint32_t LastGapAverage = 0;
float LastMaxRateOfClimb = 0;
uint32_t LastTimeSinceBoot = 0;
uint16_t LastAverageFrameRate = 0;
float MaxRateOfClimb = 0;
uint16_t LastConnectionQuality = 0;
int LastRXModelAltitude = 0;
int LastRXModelMaxAltitude = 0;
float LastRXTemperature = 0;
uint8_t RadioNumber = 0;
uint32_t LastRXReceivedPackets = 0;
// uint8_t StabilisedBank = 3;
// uint8_t LevelledBank = 3; // 3

char ParaNames[9][30] = {
    "FailSafe positions", // 1
    "QNH",                // 2
    "Mark Location",      // 3
    "PID Values",         // 4
    "Kalman Values",      // 5
    "Servo Frequencies",  // 6
    "Servo Pulse Widths", // 7
    "Gear Ratio",         // 8
};
uint16_t ScreenData[50];
uint16_t AverageFrameRate = 0;
uint64_t TotalFrameRate = 0;
uint32_t FrameRateCounter = 0;
char LogFileName[40];
char MOD[10];
char Mfiles[10];
int LastTrim[5][17];
char pFhssView[] = "page FhssView";
char pDataView[] = "page DataView";
char pSwitchesView[] = "page SwitchesView";
char pInputsView[] = "page InputsView";
char pOptionsViewS[] = "page OptionsView";
char pMixesView[] = "page MixesView";
char pTypeView[] = "page TypeView";
char pFailSafe[] = "page FailSafeView";
char pModelsView[] = "page ModelsView";
char pRXSetupView[] = "page RXSetupView";
char pCalibrateView[] = "page CalibrateView";
char pSubTrimView[] = "page SubTrimView";
char pAudioView[] = "page AudioView";
char pColoursView[] = "page ColoursView";
char pSticksView[] = "page SticksView";
char pGraphView[] = "page GraphView";
char pTXSetupView[] = "page TXSetupView";
char pBuddyChView[] = "page BuddyChView";
char pBuddyView[] = "page BuddyView";
char pOptionView2[] = "page OptionView2";
char pFrontView[] = "page FrontView";
char pPongView[] = "page PongView";
char pTXModule[] = "page TXModuleView";
char pTrimDefView[] = "page TrimDefView";
char pTrimView[] = "page TrimView";
char pLogView[] = "page LogView";
char pGPSView[] = "page GPSView";
char pPopupView[] = "page PopupView";
char pBlankView[] = "page BlankView";
char pWaitView[30]; // the saved page where wait was started
char pPIDView[] = "page PIDView";
char pKalmanView[] = "page KalmanView";
char WaitText[] = "Just a Minute! ... ";
int Previous_Current_Y = 0; // for scrolling log file
int Max_Y = 666;
uint32_t NextionReturn = 0;
bool ReadingaFile = false;
bool FirstGPSfix = true;
uint32_t RXSuccessfulPackets = 0;
uint32_t TotalPacketsAttempted = 0;
float RateOfClimb = 0;
char NextionCommand[MAXNEXTIONCOMMANDLENGTH];
char WarnNow[] = "vis Warning,1";
char WarnOff[] = "vis Warning,0";
char Warning[] = "Warning";
char err_MotorOn[] = " MOTOR IS ON! ";
uint8_t VariometerBank = 3;
uint16_t VariometerThreshold = 400; // 400 fpm
uint16_t VariometerSpacing = 125;   // 125 fpm
bool Variometer_InitDone = false;
bool BindingEnabled = false; // This is used to enable binding
uint8_t Connect_MMmsg = 0;
uint8_t Buddy_Low_Position = 0;
uint8_t Buddy_Mid_Position = 1;
uint8_t Buddy_Hi_Position = 2;

bool ParamPause = true;
bool First_RPM_Data = true;
uint32_t RotorRPM = 0;
float GearRatio = 10.3;
bool PreviousPacketFailed = false;
// **********************************************************************************************************************************
// **********************************  Area & namespace for FHSS data ************************************************************
// **********************************************************************************************************************************

namespace FHSS_data
{
    uint8_t Used_Recovery_Channels[3] = {15, 71, 82}; // channels 15, 71, 82 are used for recovery

    uint8_t FHSS_Channels[83] = {51, 28, 24, 61, 64, 55, 66, 19, 76, 21, 59, 67, 15, 71, 82, 32, 49, 69, 13, 2, 34, 47, 20, 16, 72, // UK array
                                 35, 57, 45, 29, 75, 3, 41, 62, 11, 9, 77, 37, 8, 31, 36, 18, 17, 50, 78, 73, 30, 79, 6, 23, 40,
                                 54, 12, 80, 53, 22, 1, 74, 39, 58, 63, 70, 52, 42, 25, 43, 26, 14, 38, 48, 68, 33, 27, 60, 44, 46,
                                 56, 7, 81, 5, 65, 4, 10, 1};

    uint8_t *FHSSRecoveryPointer = Used_Recovery_Channels;
    uint8_t *FHSSChPointer = FHSS_Channels; // pointer for channels array
    uint8_t NextChannelNumber = 0;
    uint8_t PaceMaker = PACEMAKER;   // now variables are used
    uint8_t RetryCount = RETRYCOUNT; // now variables are used
    uint8_t RetryWait = RETRYWAIT;   // now variables are used

} // namespace FHSS_data

/* ************************************* AckPayload structure ******************************************************

 * This first byte "Purpose" defines what all the other bytes mean, AND ...
 * the highest BIT of Purpose means ** HOP TO NEXT CHANNEL A.S.A.P. (IF ON) **
 * the lower 7 BITs then define the meaning of the remainder of the ackpayload bytes
 */

struct Payload
{
    uint8_t Purpose; // Defines meaning of the remainder
                     // Highest BIT of Purpose means HOP NOW! when ON
    uint8_t Byte1;   //
    uint8_t Byte2;   //
    uint8_t Byte3;   //
    uint8_t Byte4;   //
    uint8_t Byte5;   //
};
Payload AckPayload;

const uint8_t AckPayloadSize = sizeof(AckPayload); // i.e. 6

struct spd // Special Packet Data for Wireless Buddy functions
{
    uint8_t Command[2];
    uint64_t ModelID;
    uint8_t MasterPaceMaker; // heer
    uint8_t Channel = QUIETCHANNEL;
};
spd SpecialPacketData; // longer version

bool MasterIsInControl = true;

bool NeedToRecover = false;
uint8_t ChannelSentLastTime = 0; // The old channel number
uint8_t Index = 82;

// *********************************************** END OF GLOBAL DATA ***************************************************************

#endif
