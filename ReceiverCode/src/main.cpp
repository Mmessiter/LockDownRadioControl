#define RXVERSIONNUMBER 1 // Oct 1st 2021

// #define DEBUG
// #define DB_SENSORS
// #define DB_PID
// #define DB_BIND
// #define DB_FAILSAFE
// #define SECOND_TRANSCEIVER

#define RECEIVE_TIMEOUT 50 // 15 milliseconds was too short
#define PacketsPerHop   20
#define CHANNELSUSED    16
#define SERVOSUSED      10
#define SBUSRATE        10 // SBUS frame every 10 milliseconds
#define SBUSPORT        Serial3

bool USE_BMP280  = false; //  Pressure BMP280
bool USE_INA219  = false; //  Volts INA219
bool USE_BNO055  = false; //  Cheap BNO055 gyro
bool USE_BNO055A = false; //  Adafruit BNO055 gyro
bool USE_MPU6050 = false; //  Gyro MPU6050

#define EXTRAMICROS 500 // for extra resolution driving servos
#define MINMICROS   1000 - EXTRAMICROS
#define MAXMICROS   2000 + EXTRAMICROS

#define UNCOMPRESSEDWORDS 20                        //   16 Channels plus extra 4 16 BIT values
#define COMPRESSEDWORDS   UNCOMPRESSEDWORDS * 3 / 4 // = 16 WORDS  with no extra

/** Features List
 * - WORKS ON TEENSY 4.0
 * - Detects and uses INA219 to read volts
 * - Detects and uses uses MPU6050 gyro
 * - Detects and uses BMP280 pressure sensor for altitude
 * - Detects and uses BNO055 gyro at 28 (Adafruit) or 29 (Cheapo) hex
 * - Add Simple Kalman filter for PID.
 * - Gyro Angular velocity used for Quadcopter... a work in progress
 * - Binding implemented
 * - SBUS implemented
 * - Failsafe implemented (after two seconds)
 * - MODEL MEMORY AUTO SELECTION (REMOVED LATER)
 * - RESOLUTION INCREASED TO 12 BITS
 * - Channels incleased to 16, but only 10 PWM outputs.  SBUS can handle all.
 * - Exponential implemented (at TX end)
 */

/** TEENSY 4.0 PINS
 * | pin number(s) | purpose |
 * |---------------|---------|
 * | 0...7 | PWM SERVOS Channels 1 - 8 |
 * | 8     | PWM SERVO Channel 9 |
 * | 9     | SPI CE1 |  (FOR RADIO1)
 * | 10    | SPI CSN1 | (FOR RADIO1)
 * | 11    | SPI MOSI | (FOR BOTH RADIOS)
 * | 12    | SPI MISO | (FOR BOTH RADIOS)
 * | 13    | SPI SCK |  (FOR BOTH RADIOS)
 * | 14    | SBUS output (TX3) |
 * | 15    | N/A (RX3) |
 * | 16    | PWM SERVO Channel 10 |
 * | 18    | I2C SDA | (FOR I2C)
 * | 19    | I2C SCK | (FOR I2C)
 * | 20    | SPI CSN2 | (FOR RADIO2)
 * | 21    | SPI CE2 | (FOR RADIO2)
 * | 22    | IRQ1 | (FOR RADIO1)
 * | 23    | IRQ2 | (FOR RADIO2)
 */

#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SBUS.h>
#include <Adafruit_INA219.h>
#include <BMP280_DEV.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MPU6050_tockn.h>
#include <SimpleKalmanFilter.h>
#include <Compress.h>

#define STICKSRATE    60 * 100 // Max
#define MAXCORRECTION 15
#define DeadBand      4
#define KK            15
SimpleKalmanFilter RollRateKalman(KK, KK, 0.001);
SimpleKalmanFilter PitchRateKalman(KK, KK, 0.001);
SimpleKalmanFilter YawRateKalman(KK, KK, 0.001);

Adafruit_BNO055 BNO055_sensor[2] = {
    Adafruit_BNO055(55, BNO055_ADDRESS_A), // Adafruit version
    Adafruit_BNO055(55, BNO055_ADDRESS_B)  // Cheapo version
};
sensors_event_t orientationData, angVelocityData;
Adafruit_INA219 ina219;
BMP280_DEV      bmp280;
MPU6050         mpu6050(Wire);
Servo           MCMServo[SERVOSUSED];

uint8_t PWMPins[SERVOSUSED] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 16}; // ten now, last 6 only via sbus

#define FHSS_RESCUE_BOTTOM 118
#define FHSS_RESCUE_TOP    125

#define pinCE1  9  // NRF1
#define pinCSN1 10 // NRF1

#ifdef SECOND_TRANSCEIVER
    #define pinCSN2 20 // NRF2
    #define pinCE2  21 // NRF2
#endif

#define FAILSAFE_TIMEOUT 2000
#define LED_PIN          13

#define AEROPLANE  1
#define HELICOPTER 2
#define QUADCOPTER 3
#define HEXACOPTER 4
#define OCTOCOPTER 5

uint64_t ThisPipe = 0xBABE1E5420LL; // default startup
uint64_t NewPipe  = 0;
uint64_t OldPipe  = 0;
int      tt;

SBUS MySbus(SBUSPORT);
RF24 Radio1(pinCE1, pinCSN1);

#ifdef SECOND_TRANSCEIVER
RF24 Radio2(pinCE2, pinCSN2);
#endif

RF24* CurrentRadio = &Radio1;

Compress compress;
// ******** AckPayload Stucture using only 8 bit values for economy and better speed **********************************
struct Payload
{                                                      // Structure for data returned to transmitter.
    uint8_t volt                    = 0;               // Voltage of RX battery, if measured.
    uint8_t ReceiverFirmwareVersion = RXVERSIONNUMBER; // Firmware version number.
    uint8_t CurrentAltitude         = 0;               // Altitude, if measured.
    uint8_t ReportedPitch           = 0;
    uint8_t ReportedRoll            = 0;
    uint8_t ReportedYaw             = 0;
};
Payload AckPayload;
uint8_t AckPayloadSize = sizeof(AckPayload); // Size for later externs if needed etc.
// *****************************************************************************************************

uint8_t PacketNumber;
uint8_t NextFrequency;

uint16_t ReceivedData[UNCOMPRESSEDWORDS]; //  20  words
uint16_t PreviousData[UNCOMPRESSEDWORDS];

bool   Connected = false;
float  PacketStartTime;
int    i;
int    LastConnectionMoment = 0;
int    SearchStartTime      = 0;
int    StillSearchingTime   = 0;
double LastVoltMoment       = 0;

/**
 * Generic struct for 9 Degrees of Freedom data
 */
struct DOF9
{
    float PitchRate; /** PitchRate */
    float RollRate;  /** RollRate */
    float YawRate;   /** YawRate */
    float Pitch;     /** Pitch */
    float Roll;      /** Roll */
    float Yaw;       /** Yaw */
};

DOF9  dof9_data = DOF9(); /** The 9 DOF data. This object is used to cache and filter the data from the IMU. */
float Temperature6050;

float RollRateIntegral    = 0;
float OldRollRateIntegral = 0;
float RollRateDerivative  = 0;
float OldRollRateError    = 0;

float PitchRateIntegral    = 0;
float OldPitchRateIntegral = 0;
float PitchRateDerivative  = 0;
float OldPitchRateError    = 0;

float temperature280, pressure, altitude, StartAltitude;
bool  Swash_DisplayStarted = false;

/** Generic struct to hold component-specific PID values. */
struct PID
{
    float P = 0; /** Proportional value */
    float I = 0; /** Integral value */
    float D = 0; /** Derivative value */
};
PID SwashPID = PID(); /** Used to stabilize Roll & Pitch Rates */
PID YawPID   = PID(); /** Used to stabilize Yaw Rate */

uint8_t ModelType = 0; // 1=Aeroplane 2=Heli 3=Quadcopter 4=Hexacopter 5=Octocopter

int          LoopsPS      = 0;
int          TimeThis     = 0;
int          MainLoopTime = 0;
long int     DeltaTime    = 0;
uint8_t      BindNow      = 0;
bool         BoundFlag    = false;
uint8_t      SavedPipeAddress[8];
int          BindOKTimer             = 0;
bool         SaveNewBind             = true;
bool         ServosAttached          = false;
float        ReceiverFirmwareVersion = RXVERSIONNUMBER;
uint16_t     SbusChannels[CHANNELSUSED + 8]; // a few spare
int          SBUSTimer    = 0;
bool         FailSaveSafe = false;
bool         FailSafeChannel[17];
bool         FailSafeDataLoaded = false;
uint8_t      ModelNumber        = 0;
bool         ModelNumberSaved   = false;
uint8_t      PowerSetting       = 4;
uint8_t      DataRate           = 1;
bool         ReInit             = false;
unsigned int RX_TimeOut         = RECEIVE_TIMEOUT; // 50 MS by default
bool         FailSafeSent       = false;
uint8_t      byte1              = 0;
uint8_t      byte2              = 0;
uint8_t      ReconnectAttempts  = 0;
uint8_t      Rnumber            = 1;

/************************************************************************************************************/

uint8_t EEPROMReadByte(int p_address)
{
    uint8_t bt = EEPROM.read(p_address);
    return bt;
}

/************************************************************************************************************/

void LoadFailSafeData()
{
    uint8_t  FS_Offset = 10;
    uint16_t s[CHANNELSUSED];

    for (i = 0; i < CHANNELSUSED; i++) {
        s[i] = map(EEPROMReadByte(i + FS_Offset), 0, 180, MINMICROS, MAXMICROS); // load failsafe values and simulate better resloution
    }
    FS_Offset += CHANNELSUSED;
    for (i = 0; i < CHANNELSUSED; i++) {
        if (EEPROMReadByte(i + FS_Offset)) {
            ReceivedData[i] = s[i];
        }
    }
    FailSafeDataLoaded = true;
#ifdef DB_FAILSAFE
    Serial.println("Fail safe settings are loaded!");
#endif
}

/************************************************************************************************************/

void MapToSBUS()
{
    int RangeMax = 2047; // = Frsky at 150 %
    int RangeMin = 0;

    for (int jj = 0; jj < CHANNELSUSED; jj++) {
        SbusChannels[jj] = map(ReceivedData[jj], MINMICROS, MAXMICROS, RangeMin, RangeMax);
    }
}

/**
 * Apply a deadband to a given float value.
 * @param target_rate The actual value
 * @returns The result from applying a deadband to the @p target_rate parameter.
 * @see DEADBAND macro definition
 */
float DoDeadBand(float target_rate)
{
    if (target_rate <= DeadBand && target_rate >= -DeadBand)
        target_rate = 0;
    else if (target_rate > DeadBand)
        target_rate -= DeadBand;
    else if (target_rate < -DeadBand)
        target_rate += DeadBand;
    return target_rate;
}

/**
 * Stabilize the Yaw Rate based on a margin of error and the Yaw's proportional value
 * @param TargetYawRate The actual measured value
 * @returns The resulting "stabilized" Yaw measurement.
 */
float StabilizeYawRate(float TargetYawRate)
{
    return ((dof9_data.YawRate - TargetYawRate) * YawPID.P); // YawError * YawPID.P
}

/**
 * Calculate the Roll Rate Integral based on an ellapsed time and the previously returned value from this function.
 * @param rre The actual measured value
 * @returns The resulting integral measurement of Roll Rate.
 */
float GetRollRateIntegral(float rre)
{
    RollRateIntegral    = (rre * DeltaTime) + OldRollRateIntegral;
    OldRollRateIntegral = RollRateIntegral;
    return RollRateIntegral;
}

/**
 * Calculate the Pitch Rate Integral based on an ellapsed time and the previously returned value from this function.
 * @param rre The actual measured value
 * @returns The resulting integral measurement of Pitch Rate.
 */
float GetPitchRateIntegral(float rre)
{
    PitchRateIntegral    = (rre * DeltaTime) + OldPitchRateIntegral;
    OldPitchRateIntegral = PitchRateIntegral;
    return PitchRateIntegral;
}

/**
 * Calculate the Roll Rate Derivative based on an ellapsed time and the previously value given to this function.
 * @param rre The actual measured value
 * @returns The resulting deviation measurement of Roll Rate.
 */
float GetRollRateDerivative(float rre)
{
    float RollRateDerivative = (rre - OldRollRateError) / DeltaTime;
    OldRollRateError         = rre;
    return RollRateDerivative;
}

/**
 * Calculate the Pitch Rate Derivative based on an ellapsed time and the previously value given to this function.
 * @param rre The actual measured value
 * @returns The resulting deviation measurement of Pitch Rate.
 */
float GetPitchRateDerivative(float rre)
{
    float PitchRateDerivative = (rre - OldPitchRateError) / DeltaTime;
    OldPitchRateError         = rre;
    return PitchRateDerivative;
}

/**
 * Stabilize the Roll Rate Rate based on a margin of error and the Roll Rate's PID
 * (Proportional, Integral, and Derivative) values.
 * @param TargetRollRate The actual measured value
 * @returns The resulting "stabilized" Roll Rate measurement.
 * @see GetRollRateIntegral(), GetRollRateDerivative(), MAXCORRECTION
 */
float StabilizeRollRate(float TargetRollRate)
{
    float RollRateError      = dof9_data.RollRate - TargetRollRate;
    float RollRateCorrection = RollRateError * SwashPID.P;                   // P
    RollRateCorrection += GetRollRateIntegral(RollRateError) * SwashPID.I;   // I
    RollRateCorrection += GetRollRateDerivative(RollRateError) * SwashPID.D; // D
    return constrain(RollRateCorrection, -MAXCORRECTION, MAXCORRECTION);
}

/**
 * Stabilize the Pitch Rate based on a margin of error and the Pitch Rate's PID
 * (Proportional, Integral, and Derivative) values.
 * @param TargetPitchRate The actual measured value
 * @returns The resulting "stabilized" Pitch Rate measurement.
 * @see GetPitchRateIntegral(), GetPitchRateDerivative(), MAXCORRECTION
 */
float StabilizePitchRate(float TargetPitchRate)
{
    float PitchRateError      = (dof9_data.PitchRate - TargetPitchRate);
    float PitchRateCorrection = PitchRateError * SwashPID.P;                    // P
    PitchRateCorrection += GetPitchRateIntegral(PitchRateError) * SwashPID.I;   // I
    PitchRateCorrection += GetPitchRateDerivative(PitchRateError) * SwashPID.D; // D
    return constrain(PitchRateCorrection, -MAXCORRECTION, MAXCORRECTION);
}

/************************************************************************************************************/

void ShowPid()
{
#ifdef DB_PID
    LoopsPS++;
    if (millis() - TimeThis >= 1000) {
        // Serial.print ("LoopsPS = ");
        // Serial.println(LoopsPS);
        Serial.print("Swash_P: ");
        Serial.println(SwashPID.P);
        Serial.print("Swash_I: ");
        Serial.println(SwashPID.I, 10);
        Serial.print("Swash_D: ");
        Serial.println(SwashPID.D);
        Serial.print("RollRate: ");
        Serial.println(RollRate);
        Serial.print("DeltaTime: ");
        Serial.println(DeltaTime);
        Serial.println(" ");
        TimeThis = millis();
        LoopsPS  = 0;
    }
#endif // defined DB_PID
}

/************************************************************************************************************/

void MoveServos()
{
    int j = 0;
    int k = 0;
    MySbus.write(&SbusChannels[0]);
    if (1) { //(ModelType==AEROPLANE ){                         // !! fix Later ***************************************
        for (j = 0; j < SERVOSUSED; j++) {
            if (PreviousData[j] != ReceivedData[j]) {
                MCMServo[j].writeMicroseconds(ReceivedData[j]);
                PreviousData[j] = ReceivedData[j];
            }
        }
        return; //  !! remove  later ***************************************
    }
    if (ModelType == HELICOPTER) {
        Serial.println("HELICOPTER!");
    }
    // ************************** QUADZONE **************************************
    if (ModelType == QUADCOPTER) {
        uint8_t Throttle[4];

        float TargetRollRate  = (map((ReceivedData[0]), 0, 180, -STICKSRATE, STICKSRATE)) / 100; // get Aileron stick
        TargetRollRate        = DoDeadBand(TargetRollRate);
        float TargetPitchRate = (map((ReceivedData[1]), 0, 180, -STICKSRATE, STICKSRATE)) / 100; // get elevator stick
        TargetPitchRate       = DoDeadBand(TargetPitchRate);
        float TargetYawRate   = (map((ReceivedData[3]), 0, 180, -STICKSRATE, STICKSRATE)) / 100; // get Rudder stick
        TargetYawRate         = DoDeadBand(TargetYawRate);

        for (k = 0; k <= 3; k++) {
            Throttle[k] = ReceivedData[2];
        } // get throttle stick

        float RollRateCorrection  = StabilizeRollRate(TargetRollRate);
        float PitchRateCorrection = StabilizePitchRate(TargetPitchRate);
        float YawRateCorrection   = StabilizeYawRate(TargetYawRate);

        ShowPid();

        Throttle[0] -= YawRateCorrection;
        Throttle[1] += YawRateCorrection;
        Throttle[2] -= YawRateCorrection;
        Throttle[3] += YawRateCorrection;

        Throttle[0] += RollRateCorrection;
        Throttle[1] += RollRateCorrection;
        Throttle[2] -= RollRateCorrection;
        Throttle[3] -= RollRateCorrection;

        Throttle[0] -= PitchRateCorrection;
        Throttle[1] += PitchRateCorrection;
        Throttle[2] += PitchRateCorrection;
        Throttle[3] -= PitchRateCorrection;

        Throttle[0] = constrain(Throttle[0], 20, 180);
        Throttle[1] = constrain(Throttle[1], 20, 180);
        Throttle[2] = constrain(Throttle[2], 20, 180);
        Throttle[3] = constrain(Throttle[3], 20, 180);

        if (ReceivedData[2] < 15) { //  no throttle on very low throttle stick input
            for (k = 0; k <= 3; k++) {
                Throttle[k] = 2;
            }
            OldRollRateError  = 0;
            OldPitchRateError = 0;
        }
        for (k = 0; k <= 3; k++) {
            MCMServo[k].write(Throttle[k]);
        } //  set four throttles
    }

    // ******************************************************************************
    if (ModelType == HEXACOPTER) {
        Serial.println("HEXACOPTER!");
    }
    if (ModelType == OCTOCOPTER) {
        Serial.println("OCTOCOPTER!");
    }
}

/************************************************************************************************************/

void FailSafe()
{

    if (BoundFlag) {
        if (!FailSafeDataLoaded) {
            LoadFailSafeData();
        }
        if (!FailSafeSent) {
            MapToSBUS();
            MoveServos();
#ifdef DB_FAILSAFE
            Serial.println("FAILSAFE SENT");
#endif
            FailSafeSent = true;
        }
    }
}

/************************************************************************************************************/

void ShowHopDurationEtc()
{
    float OnePacketTime = 0;
    OnePacketTime       = (millis() - PacketStartTime) / PacketsPerHop;
    Serial.print("Hop duration: ");
    Serial.print((millis() - PacketStartTime) / 1000);
    Serial.print("s  Packets per hop: ");
    Serial.print(PacketsPerHop);
    Serial.print("  Average Time per packet: ");
    Serial.print(OnePacketTime);
    Serial.print("ms  Next channel: ");
    Serial.println(NextFrequency);
    PacketStartTime = millis();
}

/************************************************************************************************************/

void HopToNextFrequency()
{

    CurrentRadio->stopListening();
    CurrentRadio->setChannel(NextFrequency);
    CurrentRadio->startListening();
    LastConnectionMoment = millis();
#ifdef DEBUG
    ShowHopDurationEtc();
#endif
}

/************************************************************************************************************/

void InitCurrentRadio()
{
    CurrentRadio->begin();                  // sets all to defaults
    CurrentRadio->enableAckPayload();       // needed
    CurrentRadio->maskIRQ(1, 1, 1);         // no interrupts - at the moment - (line *IS* connected)
    CurrentRadio->enableDynamicPayloads();  // needed
    CurrentRadio->setCRCLength(RF24_CRC_8); // could be 16 or disabled

    switch (PowerSetting) {
        case 1:
            CurrentRadio->setPALevel(RF24_PA_MIN);
            break;
        case 2:
            CurrentRadio->setPALevel(RF24_PA_LOW);
            break;
        case 3:
            CurrentRadio->setPALevel(RF24_PA_HIGH);
            break;
        default:
            CurrentRadio->setPALevel(RF24_PA_MAX);
            break;
    }
    switch (DataRate) {
        case 2:
            CurrentRadio->setDataRate(RF24_1MBPS);
            break;
        case 3:
            CurrentRadio->setDataRate(RF24_2MBPS);
            break;
        default:
            CurrentRadio->setDataRate(RF24_250KBPS);
            break;
    }
    CurrentRadio->openReadingPipe(1, ThisPipe);
    SaveNewBind = true;
}

/************************************************************************************************************/

void Reconnect()
{
    SearchStartTime   = millis();
    ReconnectAttempts = 0;
    while (!Connected)
    {
        StillSearchingTime = millis() - SearchStartTime;
        ReconnectAttempts++;

#ifdef SECOND_TRANSCEIVER          // This part swaps to other transceiver if connection lost and two fitted
        if (ReconnectAttempts > 2) // TODO: To be checked with two ML01DP5 tranceivers...
        {
            ReconnectAttempts = 0;
            CurrentRadio->stopListening();
            if (Rnumber == 1)
            {
                Rnumber      = 2;
                CurrentRadio = &Radio2;
            }
            else
            {
                Rnumber      = 1;
                CurrentRadio = &Radio1;
            }
        }
#endif

#ifdef DEBUG
        Serial.print("Reconnection attempt: ");
        Serial.print(ReconnectAttempts);
        Serial.print("  Radio: ");
        Serial.println(Rnumber);
#endif
        i = FHSS_RESCUE_BOTTOM;
        while (!CurrentRadio->available() && i <= FHSS_RESCUE_TOP) // This loop exits as soon as connection is detected.
        {
            CurrentRadio->stopListening();
            CurrentRadio->setChannel(i);
            CurrentRadio->startListening();
            delay(3); // was 4, but 3 now seems good and is 25% faster?!
            i++;
        }
        if (CurrentRadio->available())
        {
            Connected          = true; // Connection is re-established so return, smiling!
            FailSafeSent       = false;
            ReconnectAttempts  = 0;
            StillSearchingTime = 0;
#ifdef DEBUG
            Serial.println("*****************************************************************************************************************");
#endif
        }
        else if (StillSearchingTime >= FAILSAFE_TIMEOUT)
        {
            if (!FailSafeSent)
            {
                FailSafe();
                FailSafeSent = true; // Once is enough
            }
        }
    }
}

/************************************************************************************************************/

void LoadAckPayload()
{
    // todo!
}

/************************************************************************************************************/

bool ReadData()
{
    uint16_t CompressedData[COMPRESSEDWORDS]; // 30 bytes -> 40 bytes when uncompressed
    Connected = false;
    if (CurrentRadio->available()) {
        // LoadAckPayload(); // it's now loaded by dosensors
        Connected            = true;
        LastConnectionMoment = millis();
        CurrentRadio->writeAckPayload(1, &AckPayload, AckPayloadSize);    // Send telemetry (actual length plus 0)
        CurrentRadio->read(&CompressedData, sizeof(CompressedData));      // Get Data
        compress.DeComp(ReceivedData, CompressedData, UNCOMPRESSEDWORDS); // my library to decompress !
        FailSafeDataLoaded = false;
        MapToSBUS();
    }
    return Connected;
}

/************************************************************************************************************/

void EEPROMUpdateByte(int p_address, uint8_t p_value)
{
    EEPROM.update(p_address, p_value);
}

/************************************************************************************************************/

void SaveModelNumber()
{
    // ModelNumber occupies EEPROM at offset 28
    EEPROMUpdateByte(28, ModelNumber);
}

/************************************************************************************************************/

void LoadModelNumber()
{
    ModelNumber = EEPROMReadByte(28);
}

/************************************************************************************************************/

void AttachServos()
{
    if (!ServosAttached) {
        for (i = 0; i < SERVOSUSED; i++) {
            MCMServo[i].attach(PWMPins[i]);
        }
        ServosAttached = true;
    } // now 12 SbusChannels, DEFAULT TRAVEL
    MySbus.begin();
}

/************************************************************************************************************/

/************************************************************************************************************/

void SetNewPipe()
{
    CurrentRadio->openReadingPipe(1, ThisPipe);
}

/************************************************************************************************************/

void BindModel()
{
    ThisPipe = NewPipe;
    if (SaveNewBind) {
        for (i = 0; i < 8; i++) {
            EEPROMUpdateByte(i, ReceivedData[i]);
        }
    }
    CurrentRadio->stopListening();
    SetNewPipe(); // No Longer InitCurrentRadio(); which was here
    BoundFlag   = true;
    BindNow     = 0;
    SaveNewBind = false;
    AttachServos(); // AND SBUS!!!
#ifdef DB_BIND
    Serial.println("BINDING NOW");
#endif
}

/************************************************************************************************************/

void RebuildFlags(bool* f, uint16_t tb)
{ // Pass arraypointer and the two bytes to be decoded
    for (i = 0; i < 16; i++) {
        f[15 - i] = false;                 // false is default
        if (tb & 1 << i) f[15 - i] = true; // sets true if bit was on
    }
}

/************************************************************************************************************/

void CheckParams()
{

    uint8_t  mn       = 0;
    uint16_t TwoBytes = 0;

    PacketNumber  = (ReceivedData[CHANNELSUSED + 1]);
    NextFrequency = (ReceivedData[CHANNELSUSED + 2]);

    switch (PacketNumber) {
        case 3:
            SwashPID.P = ReceivedData[CHANNELSUSED + 3];
            SwashPID.P /= 200;
            break;
        case 4:
            SwashPID.I = ReceivedData[CHANNELSUSED + 3];
            SwashPID.I /= 1000000000;
            break;
        case 5:
            SwashPID.D = ReceivedData[CHANNELSUSED + 3];
            SwashPID.D *= 900;
            break;
        case 6:
            if (BoundFlag) {
                mn = byte(ReceivedData[CHANNELSUSED + 3]);
                if (mn != ModelNumber && mn > 0) {
                    ModelNumber = mn;
                    SaveModelNumber();
                }
            }
            break;
        case 7:
            PowerSetting = ReceivedData[CHANNELSUSED + 3];
            break;
        case 8:
            DataRate = ReceivedData[CHANNELSUSED + 3];
            break;
        case 9:
            YawPID.P = ReceivedData[CHANNELSUSED + 3];
            YawPID.P /= 100;
            break;
        case 10:
            YawPID.I = ReceivedData[CHANNELSUSED + 3];
            break;
        case 11:
            YawPID.D = ReceivedData[CHANNELSUSED + 3];
            break;
        case 12:
            ModelType = ReceivedData[CHANNELSUSED + 3];
            break;
        case 13:
            BindNow = ReceivedData[CHANNELSUSED + 3];
            break;
        case 14:
            byte1 = ReceivedData[CHANNELSUSED + 3]; // These bytes are failsafe flags
            break;
        case 15:
            byte2 = ReceivedData[CHANNELSUSED + 3]; // These bytes are failsafe flags
            break;
        case 16:
            FailSaveSafe = bool(ReceivedData[CHANNELSUSED + 3]);
            if (FailSaveSafe) {
                TwoBytes = uint16_t(byte2) + uint16_t(byte1 << 8);
                RebuildFlags(FailSafeChannel, TwoBytes);
            }
            break;
        case 17:
            ReInit = bool(ReceivedData[CHANNELSUSED + 3]); // must reinitialise the port if changed settings
            if (ReInit) {
                // InitCurrentRadio(); // seems not needed - out now, but must investigate!
            }
            break;
        default:
            break; //
    }
}

/************************************************************************************************************/

#ifdef DB_SENSORS
void Sensors_Status()
{

    if (USE_BNO055 || USE_BNO055A) {
        Serial.print("Pitch=");
        Serial.print(int(Pitch));
        Serial.print(" Roll=");
        Serial.print(int(Roll));
        Serial.print(" Yaw=");
        Serial.print(int(Yaw));
        Serial.print("    PitchRate=");
        Serial.print(int(PitchRate));
        Serial.print(" RollRate=");
        Serial.print(int(RollRate));
        Serial.print(" YawRate=");
        Serial.print(int(YawRate));
    }
    if (USE_INA219) {
        Serial.print("     Volts=");
        Serial.print(volt);
    }
    if (USE_BMP280) {
        Serial.print("  Altitude=");
        Serial.print(int(CurrentAltitude * 3.28084)); // convert from meters
        Serial.print(" Temp=");
        Serial.print(int(temperature280));
    }

    Serial.println(" ");
}
#endif // defined DB_SENSORS

/************************************************************************************************************/

void DoSensors()
{
    if (USE_BMP280) {
        if (bmp280.getMeasurements(temperature280, pressure, altitude)) // heer
            AckPayload.CurrentAltitude = int(altitude - StartAltitude);
    }
    if (USE_INA219) {
        AckPayload.volt = ina219.getBusVoltage_V();
    }
#ifdef DB_SENSORS
    Sensors_Status(); // look if interested
#endif
    if ((millis() - LastVoltMoment) > 1000) {
        LastVoltMoment = millis();
    }
}

/************************************************************************************************************/

FASTRUN void ReceiveData()
{
    if (!Connected)
        if (millis() - LastConnectionMoment >= RX_TimeOut) Reconnect();
    if (ReadData()) {
        CheckParams();
        if (PacketNumber >= PacketsPerHop) {
            HopToNextFrequency();
            DoSensors();
        }
    }
}

/************************************************************************************************************/

FASTRUN void KalmanFilter()
{
    dof9_data.RollRate  = RollRateKalman.updateEstimate(dof9_data.RollRate);
    dof9_data.PitchRate = PitchRateKalman.updateEstimate(dof9_data.PitchRate);
    dof9_data.YawRate   = YawRateKalman.updateEstimate(dof9_data.YawRate);
}

/************************************************************************************************************/

void ReadBNOValues()
{
    dof9_data.Yaw       = orientationData.orientation.x;
    dof9_data.Roll      = orientationData.orientation.y;
    dof9_data.Pitch     = orientationData.orientation.z;
    dof9_data.PitchRate = -angVelocityData.gyro.x;
    dof9_data.RollRate  = -angVelocityData.gyro.y;
    dof9_data.YawRate   = -angVelocityData.gyro.z;
    KalmanFilter();
}

/************************************************************************************************************/

void Get_Mpu6050()
{
    mpu6050.update();
    dof9_data.PitchRate = mpu6050.getGyroX();
    dof9_data.RollRate  = mpu6050.getGyroY();
    dof9_data.YawRate   = mpu6050.getGyroZ();
    dof9_data.Pitch     = mpu6050.getAngleX();
    dof9_data.Roll      = mpu6050.getAngleY();
    dof9_data.Yaw       = mpu6050.getAngleZ();

    AckPayload.ReportedPitch = dof9_data.Pitch; // These values are reported to Transmitter
    AckPayload.ReportedRoll  = dof9_data.Roll;
    AckPayload.ReportedYaw   = dof9_data.Yaw;

    KalmanFilter();
}

/**
 * @brief Get orientation and angular velocity event data from a BNO055 sensor.
 * @param use_cheapo
 * - `true` selects the "cheaply" sourced BNO055 sensor.
 * - `false` selects the BNO055 sensor sourced from Adafruit.
 * @see ReadBNOValues()
 */
void Get_BNO055(const bool use_cheapo)
{
    BNO055_sensor[use_cheapo].getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    BNO055_sensor[use_cheapo].getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    ReadBNOValues();
}

/************************************************************************************************************/

void ReadSavedPipe()
{
    for (i = 0; i < 8; i++) {
        SavedPipeAddress[i] = EEPROMReadByte(i); // uses first 8 bytes only.
    }
}

/************************************************************************************************************/

void GetOldPipe()
{
    ReadSavedPipe();
    OldPipe = (uint64_t)SavedPipeAddress[0] << 56;
    OldPipe += (uint64_t)SavedPipeAddress[1] << 48;
    OldPipe += (uint64_t)SavedPipeAddress[2] << 40;
    OldPipe += (uint64_t)SavedPipeAddress[3] << 32;
    OldPipe += (uint64_t)SavedPipeAddress[4] << 24;
    OldPipe += (uint64_t)SavedPipeAddress[5] << 16;
    OldPipe += (uint64_t)SavedPipeAddress[6] << 8;
    OldPipe += (uint64_t)SavedPipeAddress[7];
}

/************************************************************************************************************/

void ScanI2c()
{
    delay(500); // allow time to wake things up
    USE_MPU6050 = false;
    USE_BMP280  = false;
    USE_INA219  = false;
    USE_BNO055  = false;
    USE_BNO055A = false;
    for (i = 1; i < 127; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
#ifdef DB_SENSORS
            Serial.print(i, HEX); // in case new one shows up
            Serial.print("   ");
            if (i == 0x68) Serial.println("MPU6050 gyro detected!");
            if (i == 0x76) Serial.println("BMP280 barometer detected!");
            if (i == 0x40) Serial.println("INA219 voltage meter detected!");
            if (i == 0x29) Serial.println("Cheapo BNO055 gyro clone detected!");
            if (i == 0x28) Serial.println("Real Adafruit BNO055 gyro detected!");
#endif

            if (i == 0x28) USE_BNO055A = true;
            if (i == 0x29) USE_BNO055 = true;
            if (i == 0x40) USE_INA219 = true;
            if (i == 0x68) USE_MPU6050 = true;
            if (i == 0x76) USE_BMP280 = true;
        }
    }
}

/************************************************************************************************************/

void GetNewPipe()
{
    NewPipe = (uint64_t)ReceivedData[0] << 56;
    NewPipe += (uint64_t)ReceivedData[1] << 48;
    NewPipe += (uint64_t)ReceivedData[2] << 40;
    NewPipe += (uint64_t)ReceivedData[3] << 32;
    NewPipe += (uint64_t)ReceivedData[4] << 24;
    NewPipe += (uint64_t)ReceivedData[5] << 16;
    NewPipe += (uint64_t)ReceivedData[6] << 8;
    NewPipe += (uint64_t)ReceivedData[7];
}

/************************************************************************************************************/
// SETUP
/************************************************************************************************************/

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    delay(1000); // Needed ! - possibly for stabilising capacitors.
    Serial.begin(9600);
    Wire.begin();
    ScanI2c(); // see what's connected
    if (USE_BNO055) BNO055_sensor[1].begin();
    if (USE_BNO055A) BNO055_sensor[0].begin();

#ifdef SECOND_TRANSCEIVER
    CurrentRadio = &Radio2;
    InitCurrentRadio(); // initialise BOTH at setup, if two.
#endif

    CurrentRadio = &Radio1;
    InitCurrentRadio();
    if (USE_BMP280) {
#ifdef DB_SENSORS
        Serial.print("Starting BMP280 ... ");
#endif

        bmp280.begin(0x76);
        bmp280.setTimeStandby(TIME_STANDBY_4000MS);
        bmp280.startNormalConversion();
        for (i = 0; i < 15000; i++) {
            bmp280.getMeasurements(temperature280, pressure, StartAltitude);
        } // warm up bmp280
        while (!bmp280.getMeasurements(temperature280, pressure, StartAltitude)) {
        }

#ifdef DB_SENSORS
        Serial.println("Done! ");
#endif
    }

    if (USE_INA219) {
        ina219.begin();
    }
    if (USE_MPU6050) {
        mpu6050.begin();
#ifdef DB_SENSORS
        mpu6050.calcGyroOffsets(true);
        Serial.println(" ");
        Serial.println(" ");
#else
        mpu6050.calcGyroOffsets(false);
#endif
    }
    DeltaTime    = 0;
    MainLoopTime = millis();
    GetOldPipe();
    digitalWrite(LED_PIN, LOW);
    LoadModelNumber();
}

/************************************************************************************************************/

void SaveFailSafeData()
{
    // FailSafe data occupies EEPROM from offset 10 to 26
    uint8_t FS_Offset = 10;
    for (i = 0; i < CHANNELSUSED; i++) {
        EEPROMUpdateByte(i + FS_Offset, (map(ReceivedData[i], MINMICROS, MAXMICROS, 0, 180))); // save servo positions lower res: 8 bits
    }
    FS_Offset += CHANNELSUSED;
    for (i = 0; i < CHANNELSUSED; i++) {
        EEPROMUpdateByte(i + FS_Offset, FailSafeChannel[i]); // save flags
    }
}

/************************************************************************************************************/

void SaveFailSafeSettings()
{
    SaveFailSafeData();
#ifdef DB_FAILSAFE
    Serial.println("Fail safe settings are saved!");
#endif
    FailSaveSafe = false;
}

/************************************************************************************************************/

void DoBinding()
{

    GetNewPipe();
#ifdef DB_BIND
    tt = NewPipe;
    Serial.println(tt);
    tt = OldPipe;
    Serial.println(tt);
#endif
    if (OldPipe == NewPipe) {
        SaveNewBind = false;

#ifdef DB_BIND
        Serial.println(millis() - BindOKTimer);
#endif

        if (BindOKTimer == 0) {
            BindOKTimer = millis();
        }
        else {
            if ((millis() - BindOKTimer) > 400) { // allow .4 of a second for the TX to bind
                BindNow = 1;
            }
        }
    }
    if (BindNow == 1 && !BoundFlag) {
#ifdef DB_BIND
        Serial.print("Binding to: ");
        tt = NewPipe;
        Serial.println(tt);
#endif
        BindModel();
    }
}

/************************************************************************************************************/
// LOOP
/************************************************************************************************************/

void loop()
{
    ReceiveData();

    if (BoundFlag) {

        if (Connected) {
            if (millis() - SBUSTimer >= SBUSRATE) {
                DeltaTime = micros() - DeltaTime;
                SBUSTimer = millis(); // timer starts before send starts....
                MoveServos();
                if (USE_BNO055A) Get_BNO055(false);
                if (USE_BNO055) Get_BNO055(true);
                if (USE_MPU6050) Get_Mpu6050();
            }
        }
        if (FailSaveSafe) SaveFailSafeSettings();
        MainLoopTime = millis();
        DeltaTime    = micros();
    }

    else {
        DoBinding();
    }
}
