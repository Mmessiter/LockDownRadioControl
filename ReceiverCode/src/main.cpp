
/**************************************************** Receiver code **************************************************
  @file       main.cpp
  @brief      Main code for Lockdown Radio Control Receiver.
  @author     Malcolm Messiter 2020 - 2025
 * @page RXCODE RecieverCode
 * @section rx Features List
 * - WORKS ON TEENSY 4.0
 * - Detects and uses INA219 to read volts
 * - Detects and uses BMP280 pressure sensor for altitude and temperature and rate of climb (use default address 0x76)
 * - Detects and uses DPS310 pressure sensor for altitude and temperature and rate of climb (use default address 0x77)
 * - (DPS310 is recommended! It's better.)
 * - Binding implemented
 * - SBUS implemented
 * - Failsafe implemented
 * - RESOLUTION INCREASED TO 12 BITS
 * - Channels increased to 16. 9 or 11 PWM outputs.  SBUS can handle all.
 * - Exponential implemented (at TX end)
 * - Supports one or two tranceivers (ML01SP4s)
 * - Variometer added.
 * - Support for GPS added (Adafruit)
 * - Nexus telemetry support added (RPM etc)
 * - FHSS with 83 channels driven from RX.
 * - EEPROM storage of bind data and failsafe data
 * - Transceiver detection at startup - no more conditional compilation needed.
 * - I2C device detection at startup
 * - Watchdog timer implemented
 * - and so on...
 *
 *
 * @section rxpinout TEENSY 4.0 PINS
 * | pin number(s) | purpose |
 * |---------------|---------|

 * | 0,1   | Nexus Serial Telemetry. 1 = RX, 2 = TX. | >>> Only if Nexus is connected  <<<
 * | X...6 | PWM SERVOS Channels X - 7 |  if Nexus is used X = 2 else X = 0 (Channels 0 - 16 always available via SBUS)
 * | 7     | PWM Servo Channel 8 | Always
 * | 8     | PWM Servo Channel 9 | only when 11 PWM channels are used
 * | 9     | SPI CE1  (FOR RADIO1)  | or PWM channel 10 if 11 PWM channels are used
 * | 10    | SPI CSN1 (FOR RADIO1)  | or PWM channel 11 if 11 PWM channels are used
 * | 11    | SPI MOSI (FOR BOTH RADIOS)  |
 * | 12    | SPI MISO (FOR BOTH RADIOS)  |
 * | 13    | SPI SCK  (FOR BOTH RADIOS)  |
 * | 14    | SBUS (Serial TX3) |
 * | 15    | Can't be used. The SBUS driver takes it over as its RX3 |
 * | 16    | RED LED - This LED is ON when connected, OFF when disconnected and blinking when binding
 * | 17 << | BIND PLUG (held LOW means plug is in and Binding is on.)
 * | 18    | I2C SDA (FOR I2C) | *** --- >> BLUE WIRE   = 18 !! << --- ***
 * | 19    | I2C SCK (FOR I2C) | *** --- >> YELLOW WIRE = 19 !! << --- ***
 * | 20    | SPI CSN2 (FOR RADIO2) |
 * | 21    | SPI CE2  (FOR RADIO2) |
 * | 22    | SPI CE1  (FOR RADIO1) when 11 PWM channels are used | Otherwise unused
 * | 23    | SPI CSN1 (FOR RADIO1) when 11 PWM channels are used | Otherwise unused
 * | All exposed pins are now used but GPIOs 34 - 39 are still available as solder pads on the back of the board for extra PWM channels etc
 * @see ReceiverCode/src/main.cpp
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Adafruit_INA219.h>
#include <stdint.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "utilities/SBUS.h" // SBUS library now fixed as early version
#include <Watchdog_t4.h>

#include "utilities/1Definitions.h"

#include "utilities/radio.h"
#include "utilities/GPS.h"
#include "utilities/Binding.h"
#include "utilities/eeprom.h"
#include "utilities/Parameters.h"
#include "utilities/Nexus.h"
#include "utilities/Detect_Transceivers.h"

void DelayMillis(uint16_t ms) // This replaces any delay() calls
{
    uint32_t tt = millis();
    while (millis() - tt < ms)
    {
        KickTheDog();
    }
}

/************************************************************************************************************/

void KickTheDog()
{
    if (millis() - LastDogKick >= KICKRATE)
    {
        TeensyWatchDog.feed();
        LastDogKick = millis();
    }
}

/************************************************************************************************************/

bool CheckForCrazyValues() // Crazy values might come while binding, and should not get sent to servos.
{
    if (millis() - ReconnectedMoment > 10000)                               // crazy values are very rare after 10 seconds of connection
        return true;                                                        // go home happy
    for (int i = 0; i < 7; ++i)                                             // need only check first few as that's where bind data are (Teensy MAC address etc.)
    {                                                                       // check ...
        if ((ReceivedData[i] < MINMICROS) || (ReceivedData[i] > MAXMICROS)) // if any value is outside the range, return false
            return false;
    }
    return true;
}
//************************************************************************************************************/
inline uint8_t PwmStartIndex()
{
    return Rotorflight22Detected ? 2 : 0; // if Nexus is present,returns 2 else 0
}

/************************************************************************************************************/
// Function to get PWM value needed for given pulse length in microseconds
#ifdef USE_PWM
inline int GetPWMValue(int frequency, int length) { return static_cast<int>((static_cast<float>(length) / (1000000.0f / static_cast<float>(frequency))) * SERVO_RESOLUTION); }
#endif

/************************************************************************************************************/
void MoveServos()
{
    static uint32_t LocalTimer = 0;
    if ((millis() - LocalTimer) < 10)
        return;
    LocalTimer = millis();

    if (!CheckForCrazyValues())
    {
        TurnLedOff(); // if we have crazy values, turn the LED off and don't move the servos
        return;
    }
    else
    {
        TurnLedOn(); // if we have good values, turn the LED on and move the servos and send SBUS data
    }

#ifdef USE_SBUS
    SendSBUSData(); // Send the SBUS data
#endif

#ifdef USE_PWM
    for (int j = PwmStartIndex(); j < Servos_Used; ++j)
    {
        int PulseLength = ReceivedData[j];
        if (ServoCentrePulse[j] < 1000)
        {
            PulseLength = map(PulseLength, MINMICROS, MAXMICROS, ServoCentrePulse[j] - EXTRAAT760, ServoCentrePulse[j] + EXTRAAT760); // these lines allow for the fact that some servos don't like 760 - 2240
        }
        else
        {
            PulseLength = map(PulseLength, MINMICROS, MAXMICROS, ServoCentrePulse[j] - EXTRAAT1500, ServoCentrePulse[j] + EXTRAAT1500); // these lines allow for the fact that some servos don't like 760 - 2240
        }
        analogWrite(PWMPins[j], GetPWMValue(ServoFrequency[j], PulseLength));
    }
#endif // USE_PWM
}

/************************************************************************************************************/

/** Execute FailSafe data from EEPROM. */
void FailSafe()
{
    // Look("Entering Failsafe!");
    if (BoundFlag)
    {
        LoadFailSafeDataFromEEPROM(); // load failsafe values from EEPROM
        Connected = true;             // to force sending this data!
#ifdef USE_SBUS
        MapToSBUS();
#endif // USE_SBUS
        for (int i = 0; i < 20; i++)
        {
#ifdef USE_SBUS
            SendSBUSData(); // one at least will be sent!
#endif                      // USE_SBUS
            MoveServos();
            delay(1);
        }
        UnbindModel(); // Unbind the model so that we now only listen to default pipe
    }
    FailSafeSent = true; // Once is enough
    FailedSafe = true;
    TurnLedOff();
    MacAddressSentCounter = 0;
#ifdef DB_FAILSAFE
    Look("Fail safe activated!");
#endif
}
/************************************************************************************************************/
#ifdef USE_PWM
void SetServoFrequency()
{
    analogWriteResolution(SERVO_RES_BITS); // 12 Bits for 4096 steps
    for (uint8_t i = PwmStartIndex(); i < Servos_Used; ++i)
    {
        analogWriteFrequency(PWMPins[i], ServoFrequency[i]);
    }
}
#endif // USE_PWM
/************************************************************************************************************/
void StartSBUSandSERVOS()
{
#ifdef USE_PWM
    SetServoFrequency();
#endif // USE_PWM

#ifdef USE_SBUS
    MySbus.begin(); // AND START SBUS
#endif              // USE_SBUS
}

/************************************************************************************************************/

void TurnLedOn()
{
    if (!LedIsOn)
    {
        digitalWrite(LED_RED, HIGH);
        LedIsOn = true;
    }
}

/************************************************************************************************************/

void TurnLedOff()
{
    if (LedIsOn)
    {
        digitalWrite(LED_RED, LOW);
        LedIsOn = false;
    }
}

/************************************************************************************************************/
void RebuildFlags(bool *f, uint16_t tb)
{ // Pass arraypointer and the two bytes to be decoded. Builds the 16 bool flag bytes from 16 BITS
    for (uint8_t i = 0; i < 16; ++i)
        f[15 - i] = (tb & (1 << i)); // sets false if bit was off, or true if bit was on... delightfully short, and clear enough in my opinion!
}

// ******************************************************************************************************************************************************************

// Discover what was connected on I2C
// can use: INA219, MPU6050, ADAFRUIT GPS, BMP280, DPS310 (@ default I2c Addresses) This is a one off function at startup

FLASHMEM void ScanI2c()
{
    //  Look("scanning I2C bus for devices ...");
    for (uint8_t i = 1; i < 127; ++i)
    {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0)
        {
            if (i == 0x40)
            {
                INA219Connected = true;
                // Look1("INA219 found at address: ");
                // Look(i, HEX);
            }
            if (i == 0x68)
            {
                MPU6050Connected = true;
            }
            if (i == 0x10)
            {
                GPS_Connected = true;
            }
            if (i == BMP280Address) // first look at default address *** 0x76 ***
            {
                BMP280Connected = true;
                // Look1("BMP280 found at address: ");
                // Serial.println(i, HEX);
            }

            if (i == 0x77 || i == 0x76)
            {
                // DPS310 can be at 0x76 or 0x77, but so can BMP280.
                // To distinguish, try to read the DPS310 product ID register (0x0D, should return 0x10)
                Wire.beginTransmission(i);
                Wire.write(0x0D); // DPS310 Product ID register
                if (Wire.endTransmission(false) == 0 && Wire.requestFrom(i, (uint8_t)1) == 1)
                {
                    if (Wire.read() == 0x10)
                    {
                        DPS310Connected = true;
                        BMP280Connected = false; // If a DPS310 is found, any BMP280 will not be used
                        DPS310Address = i;
                        // Look1("DPS310 found at address: ");
                        // Serial.println(i, HEX);
                    }
                }
            }
        }
    }
}

/************************************************************************************************************/

void WatchDogCallBack()
{
    // Serial.println("RESETTING ...");
}
/************************************************************************************************************/
void S_or_O(int d1, int d2, int d3) // This function blinks the LED for S or O in Morse code
{
    for (int i = 0; i < 3; ++i)
    {
        TurnLedOn();
        delay(d1);
        TurnLedOff();
        delay(d2);
    }
    delay(d3);
}
/************************************************************************************************************/
void SOS_Led() // This function blinks the LED for SOS in Morse code
{
    uint16_t Blinkrate = 125;
    S_or_O(Blinkrate, Blinkrate, Blinkrate * 2);           // = S
    S_or_O(Blinkrate * 3, Blinkrate * 1.5, Blinkrate * 2); // = O
    S_or_O(Blinkrate, Blinkrate, Blinkrate * 7);           // = S
}
/************************************************************************************************************/

// This function never returns. It's a fatal error. Stop here and send SOS on LED !!
void Abort()
{
    for (uint32_t i = 0; i < 0xFFFFFFFF; ++i)
    {
        Look1(i);
        Look(" FATAL ERROR - PLUG IN WRONG WAY ROUND!");
        SOS_Led(); // This is a fatal error Stop here and send SOS!!
    }
}

/************************************************************************************************************/
// This function is called at statup to check that the SBUS pin is not held low (plug in wrong way round)

void TestTheSBUSPin()
{
    pinMode(SBUSPIN, OUTPUT);
    delayMicroseconds(50);
    digitalWrite(SBUSPIN, HIGH);
    delayMicroseconds(50);
    if (!digitalRead(SBUSPIN))
    {
        while (true)
        {
            Abort();
        } // This is a fatal error Stop here and send SOS!!
    }
    SBUSPORT.begin(100000); // SBUS protocol uses 100000 baud. Re initialise it since we've just used it as an output
}
/************************************************************************************************************/
// This function is called at statup to check that the no PWM pins are held low (plug in wrong way round)

void TestAllPWMPins()
{
#ifdef USE_PWM
    for (uint8_t i = PwmStartIndex(); i < Servos_Used; ++i)
    {
        pinMode(PWMPins[i], OUTPUT);
        delayMicroseconds(50);
        digitalWrite(PWMPins[i], HIGH);
        delayMicroseconds(50);
        if (!digitalRead(PWMPins[i]))
            Abort(); // is this PWM pin held low?!?!?!?!?!?!?
    }
#endif
}

/************************************************************************************************************/

void SetupPINMODES()
{
    pinMode(LED_PIN, OUTPUT);
    delay(10);
    if (Use_Second_Transceiver)
    {
        pinMode(V_Pin_Csn2, OUTPUT);
        delay(10);
        pinMode(V_Pin_Ce2, OUTPUT);
        delay(10);
    }
    pinMode(V_Pin_Ce1, OUTPUT);
    delay(10);
    pinMode(V_Pin_Csn1, OUTPUT);
    delay(10);
    pinMode(BINDPLUG_PIN, INPUT_PULLUP);
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
}

/***********************************************************************************************************/

void SetupWatchDog()
{

    WatchDogConfig.window = WATCHDOGMAXRATE;  //  = MINIMUM RATE in milli seconds, (32ms to 522.232s) must be MUCH smaller than timeout
    WatchDogConfig.timeout = WATCHDOGTIMEOUT; //  = MAX TIMEOUT in milli seconds, (32ms to 522.232s)
    WatchDogConfig.callback = WatchDogCallBack;
    TeensyWatchDog.begin(WatchDogConfig);
    LastDogKick = millis();
}
// ***************************************************************************************************************************************************
void Init_BMP280()
{
    if (!bmp.begin(BMP280Address))
    {
        Serial.println("Did not find a valid BMP280 sensor.");
        BMP280Connected = false; // This is not a fatal error but we can't use the BMP280
        return;
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */
}
/// *************************************************************************************************************/
void Init_DPS310()
{
    if (!dps310.begin_I2C())
    { // You can optionally pass an address here (default is 0x77)
        Serial.println("Did not find a valid DPS310 sensor.");
        DPS310Connected = false;
        return;
    }
    // Look("DPS310 initialized!");
    // Optional: configure oversampling and filtering
    dps310.configurePressure(DPS310_8HZ, DPS310_16SAMPLES);
    dps310.configureTemperature(DPS310_4HZ, DPS310_16SAMPLES);
    DPS310Connected = true;
}

/************************************************************************************************************/
// SETUP
/************************************************************************************************************/
FLASHMEM void setup()
{
    digitalWrite(LED_PIN, HIGH);
    delay(100); // NEEDED! (wait for power to stabilise before doing anything else)
    DetectTransceivers();
    SetupPINMODES();
    Wire.begin();
    Wire.setClock(400000); // Or 1000000, etc
                           // delay(400);// *only* needed if you want to see terminal output
    delay(300);            // Wait for I2C to settle
    ScanI2c();             // Detect what's connected
    if (BMP280Connected)
        Init_BMP280();
    if (DPS310Connected)
        Init_DPS310();
    if (INA219Connected)
        ina219.begin();
    if (GPS_Connected)
        setupGPS();
    CopyToCurrentPipe(DefaultPipe, PIPENUMBER);
    DetectRotorFlightAtBoot(); // Check for Nexus presence before we set up any PWM pins it might use

    TestTheSBUSPin(); // Check that the SBUS pin is not held low (plug in wrong way round)
    TestAllPWMPins(); // Check that the no PWM pins are held low (plug in wrong way round)
    SetupRadios();
    SetupWatchDog();
    LoadSavedPipeFromEEPROM();
    Blinking = !digitalRead(BINDPLUG_PIN); // Blinking = binding to new TX ... because bind plug is inserted
    BindPlugInserted = Blinking;           // Bind plug inserted or not
    if (BindPlugInserted)
        delay(200);
    digitalWrite(LED_PIN, LOW);
    Look1("Receiver Type Detected: ");
    Look(Receiver_Type);
}

/************************************************************************************************************/

void BlinkLed()
{
    uint16_t Blinkrate = 222;
    if (MPU6050Connected)
        Blinkrate = 666; // Blink rate is reduced if MPU6050 is connected
    if ((millis() - BlinkTimer) >= Blinkrate)
    {
        BlinkTimer = millis();
        if (BlinkValue ^= 1)
            TurnLedOn();
        else
            TurnLedOff();
    }
}
//************************************************************************************************************/

void TimeTheMainLoop()
{
    static uint32_t LastTime = 0;
    if (millis() - LastTime >= 1000)
    {
        Look1("Interations per second: ");
        Look(Interations);
        LastTime = millis();
        Interations = 0; // reset interations every second
    }
    ++Interations; // count interations per second
}

/************************************************************************************************************/
// LOOP
/************************************************************************************************************/
void loop()
{
    // TimeTheMainLoop();
    KickTheDog();
    ReceiveData();

    if (Rotorflight22Detected)
    {
        CheckMSPSerial(); // this is to read telemetry from Nexus via MSP
    }

    if (Blinking)
    {
        BlinkLed();
    }
    if (BoundFlag)
    {
        MoveServos(); // Actually do something useful at last
    }
    else
    {
        // Look1("Not Bound, attempting to get new pipe at ");
        // Look(millis());
        GetNewPipe();
    }
}