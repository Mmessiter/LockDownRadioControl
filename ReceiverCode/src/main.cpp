
// ************************************************** Receiver code **************************************************

/** @file ReceiverCode/src/main.cpp
 * // Malcolm Messiter 2020 - 2025
 * @page RXCODE RecieverCode
 *
 * @section rx Features List
 * - WORKS ON TEENSY 4.0
 * - Detects and uses INA219 to read volts
 * - Detects and uses BMP280 pressure sensor for altitude and temperature and rate of climb (use default address 0x76)
 * - Detects and uses DPS310 pressure sensor for altitude and temperature and rate of climb (use default address 0x77)
 * - (DPS310 is recommended! It's better.)
 * - Binding implemented
 * - SBUS implemented
 * - PPM Implemented on the same pin as SBUS (Serial 3 / Pin 14)
 * - Failsafe implemented
 * - RESOLUTION INCREASED TO 12 BITS
 * - Channels increased to 16. 9 or 11 PWM outputs.  SBUS can handle all. PPM Does <= 8
 * - Exponential implemented (at TX end)
 * - Supports one or two tranceivers (ML01SP4s)
 * - Variometer added.
 * - Support for GPS added (Adafruit)
 *
 *
 * @section rxpinout TEENSY 4.0 PINS
 * | pin number(s) | purpose |
 * |---------------|---------|
 * | 0...8 | PWM SERVOS Channels 1 - 9 |  (Channels 10 - 16 available via SBUS)
 * | 9     | SPI CE1  (FOR RADIO1)  | or PWM channel 10 when 11 PWM channels are used
 * | 10    | SPI CSN1 (FOR RADIO1)  | or PWM channel 11 when 11 PWM channels are used
 * | 11    | SPI MOSI (FOR BOTH RADIOS)  |
 * | 12    | SPI MISO (FOR BOTH RADIOS)  |
 * | 13    | SPI SCK  (FOR BOTH RADIOS)  |
 * | 14    | SBUS *OR PPM* output (Serial TX3) |
 * | 15    | Don't use if using SBUS. The driver takes it (RX3) |
 * | 16    | RED LED  - This LED is ON when connected, OFF when disconnected and blinking when binding
 * | 17 << | BIND PLUG (held LOW means plug is in)
 * | 18    | I2C SDA (FOR I2C) | *** --- >> BLUE WIRE   = 18 !! << --- ***
 * | 19    | I2C SCK (FOR I2C) | *** --- >> YELLOW WIRE = 19 !! << --- ***
 * | 20    | SPI CSN2 (FOR RADIO2) |
 * | 21    | SPI CE2  (FOR RADIO2) |
 * | 22    | SPI CE1  (FOR RADIO1) when 11 PWM channels are used | Otherwise unused
 * | 23    | SPI CSN1 (FOR RADIO1) when 11 PWM channels are used | Otherwise unused
 * | All exposed pins are now used but oututs 34 - 39 are still available as solder pads on the back of the board for extra PWM channels etc
 * @see ReceiverCode/src/main.cpp
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Adafruit_INA219.h>
#include <stdint.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

#include <PulsePosition.h>
#include <Watchdog_t4.h>
#include "utilities/SBUS.h" // SBUS library now fixed as early version
#include "utilities/common.h"
#include "utilities/radio.h"
#include "utilities/pid.h"
#include "utilities/GPS.h"
#include "utilities/kalman.h"

void DelayMillis(uint16_t ms) // This replaces any delay() calls
{
    uint32_t tt = millis();
    while (millis() - tt < ms)
    {
#ifdef USE_STABILISATION
        if (MPU6050Connected)
            DoStabilsation();
#endif
    }
}
/************************************************************************************************************/

void LoadFailSafeData() //
{
    uint8_t FS_Offset = FS_EEPROM_OFFSET;
    uint16_t s[CHANNELSUSED];

    for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        s[i] = map(EEPROM.read(i + FS_Offset), 0, 180, MINMICROS, MAXMICROS); // load failsafe values and simulate better resolution
    }
    FS_Offset += CHANNELSUSED;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        if (EEPROM.read(i + FS_Offset))
        {
            ReceivedData[i] = s[i];
        }
    }
    FailSafeDataLoaded = true;
#ifdef DB_FAILSAFE
    Serial.println("Fail safe settings are loaded!");
#endif
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

bool CheckCrazyValues()
{                                             // might come when binding
    if (millis() - ReconnectedMoment > 10000) // crazy values are rare after 10 seconds of connection
        return true;
    for (int i = 0; i < 7; ++i)
    {
        if ((ReceivedData[i] < MINMICROS) || (ReceivedData[i] > MAXMICROS))
            return false;
    }
    return true;
}

/************************************************************************************************************/
// Function to get PWM value needed for given pulse length in microseconds

int GetPWMValue(int frequency, int length) { return float(length / (1000000.00 / frequency)) * SERVO_RESOLUTION; } // *** >> DON'T EDIT THIS LINE!! << ***

/************************************************************************************************************/
void MoveServos()
{
    static uint32_t LocalTimer = 0;
    if ((millis() - LocalTimer) < 10)
        return;
    LocalTimer = millis();
    if (!CheckCrazyValues())
    {
        TurnLedOff();
    }
    else
    {
        TurnLedOn();
    }

    if (!UseSBUS)
    { // not SBUS = PPM
        for (int j = 0; j < PPMChannelCount; ++j)
        {
            PPMOutput.write(PPMChannelOrder[j], map(ReceivedData[j], MINMICROS, MAXMICROS, 1000, 2000));
        }
    }
    for (int j = 0; j < SERVOSUSED; ++j)
    {
        int S = ReceivedData[j];
        if (ServoCentrePulse[j] < 1000)
        {
            S = map(S, MINMICROS, MAXMICROS, ServoCentrePulse[j] - EXTRAAT760, ServoCentrePulse[j] + EXTRAAT760); // these lines allow for the fact that some servos don't like 760 - 2240
        }
        else
        {
            S = map(S, MINMICROS, MAXMICROS, ServoCentrePulse[j] - EXTRAAT1500, ServoCentrePulse[j] + EXTRAAT1500); // these lines allow for the fact that some servos don't like 760 - 2240
        }
        analogWrite(PWMPins[j], GetPWMValue(ServoFrequency[j], S));
    }
}

/************************************************************************************************************/

/** Execute FailSafe data from EEPROM. */
void FailSafe()
{
    if (BoundFlag)
    {
        LoadFailSafeData(); // load failsafe values from EEPROM
        Connected = true;   // to force sending this data!
        MapToSBUS();
        for (int i = 0; i < 20; i++)
        {
            SendSBUSData(); // one at least will be sent!
            MoveServos();
            delay(1);
        }
        Connected = false;
        BoundFlag = false;
        ModelMatched = false;
        LongAcknowledgementsCounter = 0;
        PipeSeen = false;
        pcount = 0;
        MacAddressSentCounter = 0;
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
void SetServoFrequency()
{
    analogWriteResolution(SERVO_RES_BITS); // 12 Bits for 4096 steps
    for (uint8_t i = 0; i < SERVOSUSED; ++i)
    {
        analogWriteFrequency(PWMPins[i], ServoFrequency[i]);
    }
}

/************************************************************************************************************/
void AttachServos()
{
    SetServoFrequency();
    if (UseSBUS)
    {
        MySbus.begin(); // AND START SBUS
    }
    else
    {
        PPMOutput.begin(PPMPORT); // Or PPM on same pin
    }
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
// This function binds the model using the TX supplied Pipe instead of the default one.
// If not already saved, this saves it to the eeprom too for next time.

void BindModel()
{
    // CurrentRadio->stopListening();
    // delayMicroseconds(250);
    BoundFlag = true;
    ModelMatched = true;
    Connected = true;

    if (Blinking)
    {
        SetNewPipe(); // change to bound pipe <<< ***************************************

#ifdef DB_BIND
        Serial.println("SAVING RECEIVED PIPE:");
#endif

        for (uint8_t i = 0; i < 5; ++i)
        {
            EEPROM.update(i + BIND_EEPROM_OFFSET, TheReceivedPipe[i]);

#ifdef DB_BIND
            Serial.print(TheReceivedPipe[i], HEX);
            Serial.print(" ");
#endif
        }

#ifdef DB_BIND
        Serial.println("");
        Serial.println("TX PIPE SAVED");
#endif
    }
    Blinking = false;

    if (FirstConnection)
    {
        AttachServos(); // AND START SBUS / PPM
        FirstConnection = false;
    }
    SaveNewBind = false;

#ifdef DB_BIND
    Serial.println("");
    Serial.println("DONE BINDING");
#endif
    ConnectMoment = millis();
    SuccessfulPackets = 0; // Reset the packet count
}

/************************************************************************************************************/
void ReadSavedPipe() // read only 6 bytes
{
    for (uint8_t i = 0; i < 5; ++i)
    {
        TheReceivedPipe[i] = EEPROM.read(i + BIND_EEPROM_OFFSET); // uses first 5 bytes only.
    }
    TheReceivedPipe[5] = 0;
}

/************************************************************************************************************/
void RebuildFlags(bool *f, uint16_t tb)
{ // Pass arraypointer and the two bytes to be decoded
    for (uint8_t i = 0; i < 16; ++i)
    {
        f[15 - i] = false; // false is default
        if (tb & 1 << i)
            f[15 - i] = true; // sets true if bit was on
    }
}
/************************************************************************************************************/
// For numeric types (int, float, double, etc.)
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

// Discover what was connected on I2C
// can use: INA219, MPU6050, ADAFRUIT GPS, BMP280, DPS310 (@ default I2c Addresses) This is a one off function at startup

FLASHMEM void ScanI2c()
{
    for (uint8_t i = 1; i < 127; ++i)
    {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0)
        {
            if (i == 0x40)
            {
                INA219Connected = true;
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
void SaveFailSafeData()
{
    // FailSafe data occupies EEPROM from offset FS_EEPROM_OFFSET
    uint8_t FS_Offset = FS_EEPROM_OFFSET;

    for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        EEPROM.update(i + FS_Offset, (map(ReceivedData[i], MINMICROS, MAXMICROS, 0, 180))); // save servo positions lower res: 8 bits
        DelayMillis(1);
    }
    FS_Offset += CHANNELSUSED;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        EEPROM.update(i + FS_Offset, FailSafeChannel[i]); // save flags
        DelayMillis(1);
    }
#ifdef DB_FAILSAFE
    Serial.println("Fail safe settings are saved!");
#endif
}

/************************************************************************************************************/

void WatchDogCallBack()
{
    // Serial.println("RESETTING ...");
}
/************************************************************************************************************/

void teensyMAC(uint8_t *mac)
{ // GET UNIQUE TEENSY 4.0 ID
    for (uint8_t by = 0; by < 2; by++)
        mac[by] = (HW_OCOTP_MAC1 >> ((1 - by) * 8)) & 0xFF;
    for (uint8_t by = 0; by < 4; by++)
        mac[by + 2] = (HW_OCOTP_MAC0 >> ((3 - by) * 8)) & 0xFF;
}

/************************************************************************************************************/

void ReadBindPlug()
{
    uint32_t tt = millis();
    PipePointer = DefaultPipe;
    CopyCurrentPipe(DefaultPipe, PIPENUMBER);
    if (!digitalRead(BINDPLUG_PIN))
    {                    // Bind Plug needed to bind!
        Blinking = true; // Blinking = binding to new TX
#ifdef DB_BIND
        Serial.println("Bind plug detected.");
#endif
    }
    else
    {
        Blinking = false; // Already bound
        PipePointer = TheReceivedPipe;
        CopyCurrentPipe(TheReceivedPipe, BOUNDPIPENUMBER);
        BoundFlag = true;
        Connected = true;
        SaveNewBind = false;
        while (millis() - tt < 500)
            ReceiveData();
        BindModel(); // TODO check this...
    }
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
    for (uint8_t i = 0; i < SERVOSUSED; ++i)
    {
        pinMode(PWMPins[i], OUTPUT);
        delayMicroseconds(50);
        digitalWrite(PWMPins[i], HIGH);
        delayMicroseconds(50);
        if (!digitalRead(PWMPins[i]))
            Abort(); // is this PWM pin held low?!?!?!?!?!?!?
    }
}

/************************************************************************************************************/

void SetupPINMODES()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(pinCSN1, OUTPUT);
#ifdef SECOND_TRANSCEIVER
    pinMode(pinCSN2, OUTPUT);
    pinMode(pinCE2, OUTPUT);
#endif
    pinMode(pinCE1, OUTPUT);
    pinMode(BINDPLUG_PIN, INPUT_PULLUP);
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
}
// ***************************************************************************************************************************************************
void SetupRadios()
{

    if (digitalRead(BINDPLUG_PIN))
    { // ie no bind plug, so initialise to bound pipe
        GetOldPipe();
    }
    teensyMAC(MacAddress);
    PipePointer = DefaultPipe;
    CopyCurrentPipe(DefaultPipe, PIPENUMBER);
    CurrentRadio = &Radio1;
#ifdef SECOND_TRANSCEIVER
    digitalWrite(pinCSN2, CSN_OFF);
    digitalWrite(pinCE2, CE_OFF);
#endif
    digitalWrite(pinCSN1, CSN_ON);
    digitalWrite(pinCE1, CE_ON);
    delay(4);
    InitCurrentRadio();
    ThisRadio = 1;
#ifdef SECOND_TRANSCEIVER
    CurrentRadio = &Radio2;
    digitalWrite(pinCSN1, CSN_OFF);
    digitalWrite(pinCE1, CE_OFF);
    digitalWrite(pinCSN2, CSN_ON);
    digitalWrite(pinCE2, CE_ON);
    delay(4);
    InitCurrentRadio();
    ThisRadio = 2;
#endif
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
    SetupPINMODES();
    TestTheSBUSPin(); // Check that the SBUS pin is not held low (plug in wrong way round)
    TestAllPWMPins(); // Check that the no PWM pins are held low (plug in wrong way round)
    Wire.begin();
    // delay(400); // *only* needed if you want to see terminal output
    ScanI2c(); // Detect what's connected
    if (BMP280Connected)
        Init_BMP280();
    if (DPS310Connected)
        Init_DPS310();

#ifdef USE_STABILISATION
    if (MPU6050Connected)
        InitialiseTheMPU6050();
#endif

    if (INA219Connected)
        ina219.begin();
    if (GPS_Connected)
        setupGPS();
    SetupRadios();
    SetupWatchDog();
    ReadBindPlug();
    digitalWrite(LED_PIN, LOW);
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

/************************************************************************************************************/
// LOOP
/************************************************************************************************************/

void loop() // without MPU6050 about 30000 interations per second.... EXCEPT Zero when reconnecting!!
{           // with mpu6050 only about 10000

#ifdef USE_STABILISATION
    if (MPU6050Connected)
        DoStabilsation();
#endif
    KickTheDog();
    ReceiveData();
    if (Blinking)
    {
        BlinkLed();
    }
    if (BoundFlag && ModelMatched)
    {
        SendSBUSData(); // Send the SBUS data
        MoveServos();   // Actually do something useful at last
    }
    else
    {
        GetNewPipe();
    }
}