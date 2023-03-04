
// **************************************************************************
//     This header file has all definitions and includes                    *
// **************************************************************************

#ifndef RadioFunctions_H
#define RadioFunctions_H

// **************************************************************************
//     SUPPORT FOR TX MODULE                                                *
// **************************************************************************

  #define TXMODULESUPPORT // ONLY FOR NEW VERSION  <<< *** <<<
  #define NEWPCB

// **************************************************************************
//       TX VERSION NUMBER   (May 2020 - March 2023 Malcolm Messiter)      *
//***************************************************************************

#define TXVERSION_MAJOR   2
#define TXVERSION_MINOR   1
#define TXVERSION_MINIMUS 4

// **************************************************************************
//                               Includes                                   *
// **************************************************************************

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <DS1307RTC.h>
#include <TeensyID.h>
#include <InterpolationLib.h>
#include <Watchdog_t4.h>
#include <PulsePosition.h>

// **************************************************************************
//    DEBUG OPTIONS (Uncomment any of these for that bit of debug info)     *
//***************************************************************************

// #define DB_NEXTION        // Debug NEXTION
// #define DB_SD             // Debug SD card data
// #define DB_CHECKSUM       // Debug 32BIT file checksum info
 //#define DB_FHSS           // Debug real time FHSS data
// #define DB_SENSORS        // Debug Sensors
// #define DB_BIND           // Debug Binding
// #define DB_SWITCHES       // Debug Switches
// #define DB_MODEL_EXCHANGE // Debug MODEL EXCHANGE (by RF link)
// #define DB_GAPS           // Debug Connection Gap assessment

// **************************************************************************
//                               General                                    *
// **************************************************************************

#define DEFAULTPIPEADDRESS      0xB7BE3E9423LL            // Pipe address for startup - any value but MUST match RX
#define CHANNELSUSED            16                        // 16 Channels
#define MAXMIXES                32                        // 32 mixes
#define TICKSPERMINUTE          60000                     // millis() = 60000 per minute
#define PROPOCHANNELS           8                         // Only 4 have knobs / 2 sticks (= 4 hall sensors)
#define BANKSWITCH              4                         // Default MODE switch
#define AUTOSWITCH              1                         // Default AUTO switch
#define BANKSUSED               4                         // Flight modes (AKA Banks)
#define LOWBATTERY              42                        // Default percent for warning (User definable)
#ifdef NEWPCB                                             // ***>>> red or green PCBs ... not black <<<***
  #define CE_PIN                  7                         // for SPI to nRF24L01
  #define CSN_PIN                 8                         // for SPI to nRF24L01
  #define BUDDYPPMPORT            10                        // Buddybox PPM pin
#else
  #define CE_PIN                  9                         // for SPI to nRF24L01
  #define CSN_PIN                 10                        // for SPI to nRF24L01
  #define BUDDYPPMPORT            6                         // Buddybox PPM pin
#endif

#define INACTIVITYTIMEOUT       10 * TICKSPERMINUTE       // Default time after which to switch off
#define INACTIVITYMINIMUM       05 * TICKSPERMINUTE       // Inactivity timeout minimum is 5 minutes
#define INACTIVITYMAXIMUM       30 * TICKSPERMINUTE       // Inactivity timeout maximum is 30 minutes
#define DS1307_ADDRESS          0x68                      // I2C address for RTC
#define MAXLINES                30                        // text to load at once for log and help screens
#define DEFAULT_EXPO            50                        // = ZERO EXPO (Range is 0 - 200. Below 50 is negative Expo)
#define CHARSMAX                120                       // Max length for char arrays
#define UNCOMPRESSEDWORDS       20                        // DATA TO SEND = 40  bytes
#define COMPRESSEDWORDS    UNCOMPRESSEDWORDS * 3 / 4      // COMPRESSED DATA SENT = 30  bytes
#define TIMEFORTXMANAGMENT      3                         // How many ms must remain spare between data packets before daring to undertake more trivial tasks 
#define DEFAULTLEDBRIGHTNESS    20                        // LED brightness
#define DEFAULTPOWEROFFWARNING  3                         // Default time to warn before cutting power      
#define MAXDUALRATE             200
#define MAXBUFFERSIZE           4096
#define MAXMODELNUMBER          91
#define PERFECTPACKETSPERSECOND 150                       // Flat out perfect packets per second
#define PIPES_TO_COMPARE        8
// **************************************************************************
//                            FHSS PARAMETERS                               *
//***************************************************************************

#define PACEMAKER                7    // 7 MINIMUM ms between sent packets of data. These brief pauses allow the receiver to poll its i2c Sensor hub, and TX to ShowComms();
#define RETRYCOUNT               2    // auto retries inside nRF24L01 (was 3)
#define RETRYWAIT                0    // Wait between retries is RetryWait+1 * 250us. (WAS 2) A failed packet therefore takes (RetryWait+1 * 250us) * RetryCount
#define LOSTCONTACTCUTOFF        2    // 3 How many packets to lose before reconnect triggers
#define RECONNECT_CHANNELS_COUNT 2    // How many channels to try when reconnecting
#define RECONNECT_CHANNELS_START 12   // Offset into channels' array
#define RED_LED_ON_TIME          3500 // How many ms of no connection before RED led comes on
#define LOW_VOLTAGE_TIME         3000 // How many ms to endure low voltage before announcing it. (3 seconds)

// **************************************************************************
//                            SEND MODE PARAMETERS                          *
//***************************************************************************

#define NORMAL          0 // Normal = transmit as usual
#define CALIBRATELIMITS 1 // Calibrate limits (SEND NO DATA)
#define CENTRESTICKS    2 // Calibrate Centres (SEND NO DATA)
#define SCANWAVEBAND    3 // Scan waveband (SEND NO DATA)
#define SENDNOTHING     4 // Transmission off (SEND NO DATA)

// **************************************************************************
//                               Colours                                    *
// **************************************************************************

#define Black   0
#define Blue    31
#define Brown   48192
#define Green   2016
#define Yellow  65504
#define Red     63488
#define Gray    33840
#define SkyBlue 2047
#define Purple  39070
#define Orange  64512
#define White   65535

// **************************************************************************
//                               Mixes                                      *
// **************************************************************************

#define M_Enabled       0 // Offsets for Mixes array ( up to 17)
#define M_Bank          1
#define M_MasterChannel 2
#define M_SlaveChannel  3
#define M_Reversed      4
#define M_Percent       5
#define M_R1            6
#define M_R2            7
#define M_ONEDIRECTION  8

// **************************************************************************
//                               Screens                                    *
// **************************************************************************

#define FRONTVIEW       0
#define STICKSVIEW      1
#define GRAPHVIEW       2
#define MIXESVIEW       3
#define SCANVIEW        4
#define MODELSVIEW      5
#define CALIBRATEVIEW   6
#define TXSETUPVIEW     7
#define SUBTRIMVIEW     8
#define DATAVIEW        9
#define TRIM_VIEW       10
#define MACROS_VIEW     11
#define SWITCHES_VIEW   12
#define ONE_SWITCH_VIEW 13
#define HELP_VIEW       14
#define OPTIONS_VIEW    15
#define INPUTS_VIEW     16
#define FAILSAFE_VIEW   17
#define COLOURS_VIEW    18
#define AUDIOVIEW       19
#define FILESVIEW       20
#define REVERSEVIEW     21
#define BUDDYVIEW       22
#define LOGVIEW         23
#define TRIMDEFVIEW     24
#define OPTIONVIEW2     25
#define OPTIONVIEW3     26
#define BUDDYCHVIEW     27
#define RXSETUPVIEW1    28
#define DUALRATESVIEW   29
#define GPSVIEW         30
#define RXSETUPVIEW     31
#define BANKSNAMESVIEW  32
#define SLOWSERVOVIEW   33
#define RENAMEMODELVIEW 34
#define FILEEXCHANGEVIEW 35
#define TXMODULEVIEW    36

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

#define REDLED              2 // COLOURED LEDS' PIN NUMBERS ...
#define GREENLED            3
#define BLUELED             4
#define POWER_OFF_PIN       5

#ifdef TXMODULESUPPORT
    #define BUTTON_SENSE_PIN    33
    #define PPMPORT             6
#else
    #define BUTTON_SENSE_PIN    6
#endif
// **************************************************************************
//               Sounds                             *
//***************************************************************************

#define CLICKZERO       0
#define CLICKONE        1
#define ONEMINUTE       2
#define TWOMINUTES      3
#define THREEMINUTES    4
#define FOURMINUTES     5
#define FIVEMINUTES     6
#define SIXMINUTES      7
#define SEVENMINUTES    8
#define EIGHTMINUTES    9
#define NINEMINUTES     10
#define TENMINUTES      11
#define BANKONE         12
#define BANKTWO         13
#define BANKTHREE       14
#define BANKFOUR        15
#define BEEPMIDDLE      16
#define BEEPCOMPLETE    17
#define THEFANFARE      18
#define BATTERYISLOW    19
#define CONNECTEDMSG    20
#define DISCONNECTEDMSG 21
#define BUDDYMSG        22
#define MASTERMSG       23
#define WEAKMSG         23
#define WHAHWHAHMSG     25
#define BINDSUCCEEDED   26
#define BINDNEEDED      27
#define MMFOUND         28
#define MMMATCHED       29
#define MMNOTFOUND      30
#define MOTORON         31
#define MOTOROFF        32
#define STORAGECHARGE   33
#define RATE1           34
#define RATE2           35
#define RATE3           36
#define PLSTURNOFF      37
#define AEROBATICS      38
#define AUTO            39
#define CRUISE          40
#define FLAPS           41
#define HOVER           42
#define IDLE1           43
#define IDLE2           44
#define LANDING         45
#define LAUNCH          46
#define NORMALB         47
#define SPEED           48
#define TAKEOFF         49
#define THERMAL         50
#define THRHOLD         51
#define THREEDEE        52
#define BFM1            53
#define BFM2            54
#define BFM3            55
#define BFM4            56
#define AIRBRAKES       57
#define STUNT1          58
#define STUNT2          59
#define WHEELSDOWN      60
#define WHEELSUP        61
#define TEN             62
#define NINE            63
#define EIGHT           64
#define SEVEN           65
#define SIX             66
#define FIVE            67
#define FOUR            68
#define THREE           69
#define TWO             70
#define ONE             71
#define MMSAVED         72

// **************************************************************************
//               SDCARD MODEL MEMORY CONSTANTS                              *
//***************************************************************************

#define TXSIZE            512      // SD space reserved for transmitter (WAS  250)
#define MODELSIZE         2048     // SD space reserved for each model (WAS 1600)
#define MAXFILELEN        1024 * 3 // MAX SIZE FOR HELP AND LOG FILES
#define MAXBACKUPFILES    95

// **************************************************************************
//                            SERVO RANGE PARAMETERS                        *
//***************************************************************************

#define MINMICROS       500
#define MAXMICROS       2500
#define HALFMICROSRANGE (MAXMICROS - MINMICROS) / 2
#define MIDMICROS       MINMICROS + HALFMICROSRANGE

// **************************************************************************
//                           nRF24L01 lines on off                          *
// **************************************************************************

#define CSN_ON                   LOW
#define CSN_OFF                  HIGH
#define CE_ON                    HIGH
#define CE_OFF                   LOW

// **************************************************************************
//                            Error Codes                                *
// **************************************************************************

#define NOERROR             0
#define MODELSFILENOTFOUND  1
#define CHECKSUMERROR       2
#define MOTORISON           3

// **************************************************************************
//                            Interpolations                                *
// **************************************************************************

#define STRAIGHTLINES     0
#define SMOOTHEDCURVES    1
#define EXPONENTIALCURVES 2

// **************************************************************************
//                              Macros                                      *
// **************************************************************************

#define MAXMACROS     8 // 8 macros enough for now?
#define BYTESPERMACRO 6 // 6 bytes each

// ********************* Offsets within macros' buffer ***********************

#define MACROTRIGGERCHANNEL 0 // 1 - 16. 0 means dissabled.
#define MACROSTARTTIME      1 // In ** >> 10ths << ** of a second since trigger. ( = millis() * 100 ) up to 25.4 seconds
#define MACRODURATION       2 // In ** >> 10ths << ** of a second since start    ( = millis() * 100 ) up to 25.4 seconds
#define MACROMOVECHANNEL    3 // Which channel to move.
#define MACROMOVETOPOSITION 4 // Where to put said channel for said duration. (0 - 180)
#define MACRORUNNINGNOW     5 // Running flag (BIT 0 running/not running,  BIT 1 = Timer active / inactive)

// **************************************************************************
//                            PPM PARAMETERS   (FOR BUDDY BOXING)          *
//***************************************************************************

#define PPMBUDDYFRAMERATE 10  // PPM new frame every 10 milliseconds 
#define RANGEMAX 2047         // = Frsky at 150 %
#define RANGEMIN 0            // = Frsky at 0 %

// **************************************************************************
//                          NEXTION SERIAL CONNECTION                       *
//***************************************************************************

#define NEXTION              Serial1 // NEXTION is connected to Serial1
#define MAXSHOWCOMMSSESCONDS 6       // Assess average connection quality over most recent 6 seconds continously
#define SHOWCOMMSDELAY       1000    // ms pauses between updated info on NEXTION
#define WARMUPDELAY          300     // fails at 200 so must be >200 ...
#define SCREENCHANGEWAIT     100     // allow time for screen to appear


// **************************************************************************
//                            WATCHDOG PARAMETERS                           *
//***************************************************************************

#define WATCHDOGTIMEOUT 2000 // 2 Seconds before reboot (32ms -> 500 seconds)
#define KICKRATE        500  // Kick twice a second (must be between WATCHDOGMAXRATE and WATCHDOGTIMEOUT)
#define WATCHDOGMAXRATE 250  // 250 ms secs between kicks is max rate allowed

/*********************************************************************************************************************************/
// external (global vars) needed here

extern RF24           Radio1;
extern uint8_t        CurrentMode;
extern uint8_t        NoCarrier[];
extern uint8_t        AllChannels[];
extern char           NEXTIONSleepTime[];
extern uint8_t        ScanStart;
extern uint8_t        ScanEnd;
extern uint8_t        BadChannelPointer;
extern uint8_t        BadChannels[];
extern uint8_t        NextChannel;
extern uint8_t        PacketNumber;
extern bool           JustHoppedFlag;
extern uint32_t       LastPacketSentTime;
extern bool           BoundFlag;
extern uint8_t        CurrentView;
extern uint16_t       SendBuffer[];
extern uint64_t       TeensyMACAddPipe;
extern bool           LostContactFlag;
extern uint64_t       DefaultPipe;
extern long int       RecoveryTimer;
extern bool           Connected;
extern uint16_t       CompressedData[];
extern uint8_t        FHSS_Channels[];
extern struct Payload AckPayload;
const extern uint8_t  AckPayloadSize;
extern uint8_t        SizeOfCompressedData;
extern uint16_t       RangeTestGoodPackets;
extern uint8_t        NextChannelNumber;
extern uint32_t       RecentPacketsLost;
extern uint32_t       TxOnTime;
extern uint32_t       TXTimeStamp;
extern uint32_t       HopStart;
extern char           ThisRadio[4];
extern uint32_t       GapSum;
extern uint32_t       GapStart;
extern uint8_t*       FHSSChPointer;
extern uint8_t*       FHSSRecoveryPointer;
extern bool           BuddyPupilOnPPM;
extern bool           BuddyMaster;
extern uint16_t       BackGroundColour;
extern uint16_t       HighlightColour;
extern uint16_t       ForeGroundColour;
extern uint16_t       ChannelMax[CHANNELSUSED + 1];           //    output of pots at max
extern uint16_t       ChannelMidHi[CHANNELSUSED + 1];         //    output of pots at MidHi
extern uint16_t       ChannelCentre[CHANNELSUSED + 1];        //    output of pots at Centre
extern uint16_t       ChannelMidLow[CHANNELSUSED + 1];        //    output of pots at MidLow
extern uint16_t       ChannelMin[CHANNELSUSED + 1];           //    output of pots at min
extern uint8_t        MacrosBuffer[MAXMACROS][BYTESPERMACRO]; // macros' buffer
extern uint32_t       MacroStartTime[MAXMACROS];
extern uint32_t       MacroStopTime[MAXMACROS];
extern bool           UseMacros;
extern uint32_t       TotalLostPackets;
extern bool           Reconnected;
extern bool           LedWasGreen;
extern bool           LedWasRed;
extern uint32_t       Inactivity_Timeout;
extern uint32_t       Inactivity_Start;
extern bool           UkRules;
extern bool           PreviousUkRules;
extern bool           UseLog;
extern uint32_t       ShowServoTimer;
extern uint16_t       PacketsPerSecond;
extern uint16_t       GetStickInput(uint8_t l);
extern uint8_t        PacketsHistoryBuffer[PERFECTPACKETSPERSECOND * MAXSHOWCOMMSSESCONDS];
extern uint16_t       PacketsHistoryIndex;
extern uint8_t        ConnectionAssessSeconds;
extern bool           LowPowerMode;
extern bool           NewCompressNeeded;
extern bool           ModelMatched;
extern bool           SendNoData;
extern uint8_t        ReconnectionIndex;
extern uint32_t       MostRecentHop;

// external (global) functions needed here
extern void  GetSlaveChannelValues();
extern void  KickTheDog();
extern void  SendCommand(char* tbox);
extern void  ReadSwitches();
extern void  ShowComms();
extern void  CheckTimer();
extern void  SendCharArray(char* ch0, char* ch1, char* ch2, char* ch3, char* ch4, char* ch5, char* ch6, char* ch7, char* ch8, char* ch9, char* ch10, char* ch11, char* ch12);
extern char* Str(char* s, int n, int comma);
extern void  GetNewChannelValues();
extern void  LoadPacketData();
extern void  GreenLedOn();
extern void  CheckGapsLength();
extern void  ParseAckPayload();
extern void  FailedPacket();
extern void  StartInactvityTimeout();
extern void  ShowServoPos();
extern void  SendViaPPM();
extern void  ZeroDataScreen();
extern void  RedLedOn();
extern void  ReEnableScanButton();
extern void  LogUKRules();
extern int   InStrng(char* text1, char* text2);
extern void  ReadCheckSum32();
extern void          ResetTransmitterSettings();
extern void          TryToReconnect();
extern void          FlushFifos();
extern bool          RecursedAlready;
extern FLASHMEM void SetDS1307ToCompilerTime();
extern int           GetOtherValue(char* nbox);
extern void          CheckInvisiblePoint();
extern void          GotoFrontView();
extern void          CheckDualRatesValues();
extern void          UpdateLED();
extern void          CheckMotorOff();
extern bool          GetButtonPress();
extern void          CheckPowerOffButton();
extern void          LoadFileSelector();
extern void          LoadModelSelector();
extern void          PlaySound(uint16_t TheSound);
extern uint8_t       CheckPipeNibbles(uint8_t b);

/*********************************************************************************************************************************/
// function prototypes

void         InitRadio(uint64_t Pipe);
void         SetThePipe(uint64_t WhichPipe);
void         DoScanInit();
void         DoScanEnd();
void         PreScan();
void         HopToNextChannel();
void         ScanAllChannels();
void         SendData();
void         Procrastinate(uint32_t HowLong);
void         DrawFhssBox();
void         SendText(char* tbox, char* NewWord); // needed a prototype or two here!
void         RestoreBrightness();
void         ButtonWasPressed();
void         CalibrateEdgeSwitches();
void         DisplayCurve();
void         DrawLine(int x1, int y1, int x2, int y2, int c);
void         DrawBox(int x1, int y1, int x2, int y2, int c);
void         FillBox(int x1, int y1, int w, int h, int c);
void         ReadTextFile(char* fname, char* htext, uint8_t StartLineNumber, uint8_t MaxLines);
void         LogConnection();
void         LogDisConnection();
void         CloseLogFile();
void         StartLogFile();
void         ShowLogFile(uint8_t StartLine);
void         LogThisLongGap();
void         LogThisModel();
void         Force_ReDisplay();
FASTRUN void Compress(uint16_t* compressed_buf, uint16_t* uncompressed_buf, uint8_t uncompressed_size);
FASTRUN void BufferTeensyMACAddPipe();
void         ExecuteMacro();
void         Look(int p);
void         ShowBank();
void         UpdateModelsNameEveryWhere();
void         DefineTrimsStart();
void         ResetAllTrims();
void         CheckTrimValues();
void         ClearSuccessRate();
int          CheckRange(int v, int min, int max);
void         MoveaTrim(uint8_t i);
void         SetUKFrequencies();
FASTRUN uint16_t GetStickInputInputOnly(uint8_t l);
FASTRUN void     LogSafety(bool On);
void             ShowMotor(int on);
void             StartModelSetup();
bool             GetConfirmation(char* goback, char* Prompt);
void             GotoModelsView();
void             SaveCurrentModel();
void             CheckModelName();
void             EndTrimView();
int              AnalogueReed(uint8_t InputChannel);
void             GetReturnCode();
void             SelectChannelOrder();
/*********************************************************************************************************************************/

#endif
