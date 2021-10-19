// ************************************************** Receiver code **************************************************
#define RXVERSION_MAJOR   1 // Oct 6th 2021
#define RXVERSION_MINOR   0
#define RXVERSION_MINIMUS 3

// #define DEBUG
// #define DB_SENSORS
// #define DB_PID
// #define DB_BIND
// #define DB_FAILSAFE
  #define SECOND_TRANSCEIVER
  

#define RECEIVE_TIMEOUT 50 // 15 milliseconds was too short
#define PacketsPerHop   20
#define CHANNELSUSED    16
#define SERVOSUSED      10
#define SBUSRATE        10 // SBUS frame every 10 milliseconds
#define SBUSPORT        Serial3

bool USE_INA219  = false; //  Volts INA219
bool USE_BNO055  = false; /** Cheap BNO055 gyro */
bool USE_BNO055A = false; /** Adafruit BNO055 gyro */
bool USE_MPU6050 = false; /** Gyro MPU6050 */
bool USE_BMP280  = false; /** is BMP280 sensor connected */

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

#define STICKSRATE    60 * 100 // Max
#define MAXCORRECTION 15
#define DeadBand      4

#define KK 15 /** The value for the Kalman filter's measured and estimated parameters */

SimpleKalmanFilter RollRateKalman(KK, KK, 0.001);  /** Kalman filter for Roll */
SimpleKalmanFilter PitchRateKalman(KK, KK, 0.001); /** Kalman filter for Pitch */
SimpleKalmanFilter YawRateKalman(KK, KK, 0.001);   /** Kalman filter for Yaw */

MPU6050 mpu6050(Wire); /** instantiated obj for interfacing with the MPU6050 sensor */

/** Array off possibly connected BNO055 sensors */
Adafruit_BNO055 BNO055_sensor[2] = {
    Adafruit_BNO055(55, BNO055_ADDRESS_A), // Adafruit version
    Adafruit_BNO055(55, BNO055_ADDRESS_B)  // Cheapo version
};
sensors_event_t orientationData; /** orientation data for a sensor event */
sensors_event_t angVelocityData; /** angular velocity data for a sensor event */

Adafruit_INA219 ina219;
BMP280_DEV      bmp280; /** The object to access the BMP280 sensor */
Servo           MCMServo[SERVOSUSED];

// uint16_t BNO055_SAMPLERATE_DELAY_MS = 100; // not actually used anywhere

uint8_t PWMPins[SERVOSUSED] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 16}; // ten now, last 6 only via sbus

#define FHSS_RESCUE_BOTTOM 118
#define FHSS_RESCUE_TOP    125

#define pinCE1  9  // NRF1
#define pinCSN1 10 // NRF1

#define pinCSN2 20 // NRF2
#define pinCE2  21 // NRF2


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

SBUS MySbus(SBUSPORT);
RF24 Radio1(pinCE1, pinCSN1);
RF24 Radio2(pinCE2, pinCSN2);
RF24* CurrentRadio = &Radio1;

// ******** AckPayload Stucture ************************************************************************
struct Payload
{                                // Structure for data returned to transmitter.
    uint8_t Purpose = 0;         // This byte determines what the remainder represent.
                                 // Highest BIT of Purpose means >>IGNORE IF ON<<
    uint8_t volt            = 0; // Voltage of RX battery, if measured.
    uint8_t CurrentAltitude = 0; // Altitude, if measured.
    uint8_t ReportedPitch   = 0;
    uint8_t ReportedRoll    = 0;
    uint8_t ReportedYaw     = 0;
};
Payload AckPayload;
uint8_t AckPayloadSize = sizeof(AckPayload); // Size for later externs if needed etc.
// *****************************************************************************************************

uint8_t PacketNumber;

uint16_t ReceivedData[UNCOMPRESSEDWORDS]; //  20  words
uint16_t PreviousData[UNCOMPRESSEDWORDS]; /** Previously received data (used for servos). */

bool   Connected = false;
float  PacketStartTime;
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
    float Pitch;     /** Pitch  */
    float Roll;      /** Roll  */
    float Yaw;       /** Yaw  */
};

DOF9  dof9_data = DOF9(); /** The 9 DOF data. This object is used to cache and filter the data from the IMU. */
float Temperature6050;    /** temperature data from the MPU6050 sensor's internal temperature sensor */

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
long int     DeltaTime    = 0;     /** The ellapsed time between updates sent to the Servos */
uint8_t      BindNow      = 0;     /** indicates that the receiver should start the binding/pairing process */
bool         BoundFlag    = false; /** indicates if receiver paired with transmitter */
uint8_t      SavedPipeAddress[8];
int          BindOKTimer    = 0;
bool         SaveNewBind    = true;
bool         ServosAttached = false;
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
bool         GyroInstalled      = false;
bool         Radio1Installed    = false;
bool         Radio2Installed    = false;
byte         ThisRadio          = 1;
bool         TwoRadiosInstalled = false;

/************************************************************************************************************/

void LoadVersioNumber()
{
    AckPayload.ReportedPitch = RXVERSION_MAJOR;
    AckPayload.ReportedRoll  = RXVERSION_MINOR;
    AckPayload.ReportedYaw   = RXVERSION_MINIMUS;
}

uint8_t EEPROMReadByte(int p_address)
{
    uint8_t bt = EEPROM.read(p_address);
    return bt;
}

/** Load project defaults from EEPROM */
void LoadFailSafeData()
{
    uint8_t  FS_Offset = 10;
    uint16_t s[CHANNELSUSED];

    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        s[i] = map(EEPROMReadByte(i + FS_Offset), 0, 180, MINMICROS, MAXMICROS); // load failsafe values and simulate better resloution
    }
    FS_Offset += CHANNELSUSED;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
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

    for (int j = 0; j < CHANNELSUSED; ++j) {
        SbusChannels[j] = map(ReceivedData[j], MINMICROS, MAXMICROS, RangeMin, RangeMax);
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
        for (j = 0; j < SERVOSUSED; ++j) {
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

        for (k = 0; k < 4; ++k) {
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
            for (k = 0; k < 4; ++k) {
                Throttle[k] = 2;
            }
            OldRollRateError  = 0;
            OldPitchRateError = 0;
        }
        for (k = 0; k < 4; ++k) {
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

void ReadSavedPipe()
{
    for (uint8_t i = 0; i < 8; ++i) {
        SavedPipeAddress[i] = EEPROMReadByte(i); // uses first 8 bytes only.
    }
}

/**
 * Get pipe address from EEPROM.
 * @note Address data in EEPORM is valid only after a previous power cycle observed
 * a completed binding process (pairing was successful at least once during last flight's session).
 */
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

void FailSafe()
{

    if (BoundFlag) {
        if (!FailSafeDataLoaded) {
            LoadFailSafeData();
        }
        MapToSBUS();
        MoveServos();
    }
}

/**
 * Print out some debugging information about the channel hopping implementation
 * @param freq The next frequency to be used.
 */
void ShowHopDurationEtc(uint8_t freq)
{
    float OnePacketTime = (millis() - PacketStartTime) / PacketsPerHop;
    Serial.print("Hop duration: ");
    Serial.print((millis() - PacketStartTime) / 1000);
    Serial.print("s  Packets per hop: ");
    Serial.print(PacketsPerHop);
    Serial.print("  Average Time per packet: ");
    Serial.print(OnePacketTime);
    Serial.print("ms  Next channel: ");
    Serial.print(freq);
    if (BoundFlag) Serial.println(" Bound");
    if (!BoundFlag) Serial.println(" NOT Bound");
    PacketStartTime = millis();
}

/**
 * Make radio transceiver "hop" over to the new frequency.
 * @param freq The next frequency to use.
 */
void HopToNextFrequency(uint8_t freq)
{

    CurrentRadio->stopListening();
    CurrentRadio->setChannel(freq);
    CurrentRadio->startListening();
    LastConnectionMoment = millis();
#ifdef DEBUG
    ShowHopDurationEtc(freq);
#endif
}

/** Initialize a radio transceiver. */
void InitCurrentRadio()
{
    CurrentRadio->begin();                 
    if (CurrentRadio->isChipConnected()){
        CurrentRadio->enableAckPayload();       // needed
        CurrentRadio->maskIRQ(1, 1, 1);         // no interrupts - seems NEEDED at the moment - (line *IS* connected)
        CurrentRadio->enableDynamicPayloads();  // needed
        CurrentRadio->setCRCLength(RF24_CRC_8); // could be 16 or disabled
        CurrentRadio->setPALevel(RF24_PA_MAX);
        CurrentRadio->setDataRate(RF24_250KBPS);
        CurrentRadio->openReadingPipe(1, ThisPipe);
        SaveNewBind = true;
    }
   
}


/************************************************************************************************************/


void ProdRadio(){  // After switching radios, this prod allows EITHER to connect. Don't know why - yet!
    CurrentRadio->enableDynamicPayloads(); 
    CurrentRadio->maskIRQ(1, 1, 1);         // no interrupts - seems NEEDED at the moment - (line *IS* connected)
    CurrentRadio->setCRCLength(RF24_CRC_8); 
    CurrentRadio->setPALevel(RF24_PA_MAX);
    CurrentRadio->setDataRate(RF24_250KBPS);
    CurrentRadio->openReadingPipe(1, ThisPipe);
    delay(35); // without this short pause it sometimes hangs


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
        uint8_t i = FHSS_RESCUE_BOTTOM;
        while (!CurrentRadio->available() && i <= FHSS_RESCUE_TOP) // This loop exits as soon as connection is detected.
        {
            CurrentRadio->stopListening();
            CurrentRadio->setChannel(i);
            CurrentRadio->startListening();
            delay(4); // was 4, but 3 now seems good and is 25% faster?!
            i++;
        }
       
       if (!CurrentRadio->available()) {
            if (TwoRadiosInstalled){
                if  (ReconnectAttempts > 4){ 
                     ReconnectAttempts  = 0;
                    if (ThisRadio == 1) {
                        ThisRadio = 2; 
                        CurrentRadio = &Radio2;
                        ProdRadio();
                    } else {
                        ThisRadio = 1; 
                        CurrentRadio = &Radio1;  
                        ProdRadio();
                    }   
                } 
            }    
       }

        if (CurrentRadio->available())
        {
            Serial.print (millis());            // These lines are just to help fix this area!!
            Serial.print ("   Radio: ");        // These lines are just to help fix this area!!
            Serial.println (ThisRadio);         // These lines are just to help fix this area!!

            Connected          = true; // Connection is re-established so return, smiling!
            FailSafeSent       = false;
            ReconnectAttempts  = 0;
            StillSearchingTime = 0;
        }
        else if (StillSearchingTime >= FAILSAFE_TIMEOUT)
        {
            if (!FailSafeSent)
            {
                FailSafe();
                FailSafeSent = true; // Once is enough
#ifdef DB_FAILSAFE
                Serial.println("FailSafe sent");
#endif
                //BoundFlag = false;
            }
        }
    }
}

/************************************************************************************************************/

void LoadAckPayload()
{
    AckPayload.Purpose &= 0x7F; // Clear hi bit (=do not ignore)

    ++AckPayload.Purpose;                               // 0 =  Roll, Pitch, Yaw, Volts.
                                                        // 1 =  Version number
    if (AckPayload.Purpose > 1) AckPayload.Purpose = 0; // 1 is currently max
    if (AckPayload.Purpose == 1)
    {
        LoadVersioNumber(); // if 1 send version info
    }
}

/************************************************************************************************************/

/**
 * Decompresses uint16_t* buffer values (each with 12 bit resolution - the lower 12 bits).
 * @param uncompressed_buf[in]
 * @param compressed_buf[out] Must have allocated 3/4 the size of uncompressed_buf
 * @param uncompressed_size Size is in units of uint16_t (aka word or unsigned short)
 */
void Decompress(uint16_t* uncompressed_buf, uint16_t* compressed_buf, int uncompressed_size)
{
    int p = 0;
    for (int l = 0; l < (uncompressed_size * 3 / 4); l += 3) {
        uncompressed_buf[p] = compressed_buf[l] >> 4;
        p++;
        uncompressed_buf[p] = (compressed_buf[l] & 0xf) << 8 | compressed_buf[l + 1] >> 8;
        p++;
        uncompressed_buf[p] = (compressed_buf[l + 1] & 0xff) << 4 | compressed_buf[l + 2] >> 12;
        p++;
        uncompressed_buf[p] = compressed_buf[l + 2] & 0xfff;
        p++;
    }
}

void ClearGyroData()
{
    AckPayload.ReportedPitch = 0;
    AckPayload.ReportedRoll  = 0;
    AckPayload.ReportedYaw   = 0;
    AckPayload.Purpose |= 0x80;
}

/************************************************************************************************************/

bool ReadData()
{
    uint16_t CompressedData[COMPRESSEDWORDS]; // 30 bytes -> 40 bytes when uncompressed
    Connected = false;
    if (CurrentRadio->available()) {
        LoadAckPayload();
        Connected            = true;
        LastConnectionMoment = millis();
        CurrentRadio->writeAckPayload(1, &AckPayload, AckPayloadSize); // Send telemetry (actual length plus 0)
        CurrentRadio->read(&CompressedData, sizeof(CompressedData));   // Get Data
        if (AckPayload.Purpose == 1 && !GyroInstalled) ClearGyroData();
        Decompress(ReceivedData, CompressedData, UNCOMPRESSEDWORDS); // decompress data
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
        for (uint8_t i = 0; i < SERVOSUSED; ++i) {
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
        for (uint8_t i = 0; i < 8; ++i) {
            EEPROMUpdateByte(i, ReceivedData[i]);
        }
    }
    CurrentRadio->stopListening();
    SetNewPipe();
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
    for (uint8_t i = 0; i < 16; ++i) {
        f[15 - i] = false;                 // false is default
        if (tb & 1 << i) f[15 - i] = true; // sets true if bit was on
    }
}

/**
 * Get radio parameters from received packet.
 * @note Each packet will have a ID number at the third byte in the packet.
 * This ID number will change a different parameter (which is detirmined by the ID number).
 * @returns The next frequency that the transmitter plans to use.
 */
uint8_t CheckParams()
{

    uint8_t  mn       = 0;
    uint16_t TwoBytes = 0;

    PacketNumber = (ReceivedData[CHANNELSUSED + 1]);

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
    return ReceivedData[CHANNELSUSED + 2];
}

/************************************************************************************************************/
#ifdef DB_SENSORS
void Sensors_Status()
{
    if (USE_BNO055 || USE_BNO055A) {
        Serial.print("Pitch=");
        Serial.print(int(dof9_data.Pitch));
        Serial.print(" Roll=");
        Serial.print(int(dof9_data.Roll));
        Serial.print(" Yaw=");
        Serial.print(int(dof9_data.Yaw));
        Serial.print("    PitchRate=");
        Serial.print(int(dof9_data.PitchRate));
        Serial.print(" RollRate=");
        Serial.print(int(dof9_data.RollRate));
        Serial.print(" YawRate=");
        Serial.print(int(dof9_data.YawRate));
    }
    if (USE_INA219) {
        Serial.print("     Volts=");
        Serial.print(AckPayload.volt);
    }
    if (USE_BMP280) {
        Serial.print("  Altitude=");
        Serial.print(int(AckPayload.CurrentAltitude * 3.28084)); // convert from meters
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
            if (BoundFlag) AckPayload.CurrentAltitude = int(altitude - StartAltitude);
    }
    if (USE_INA219) {
        if (BoundFlag) AckPayload.volt = ina219.getBusVoltage_V();
    }
#ifdef DB_SENSORS
    Sensors_Status(); // does nothing if DB_SENSORS is not defined
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
        uint8_t NextFrequency = CheckParams();
        if (PacketNumber >= PacketsPerHop) {
            HopToNextFrequency(NextFrequency);
            DoSensors();
        }
    }
}

/**
 * @brief Apply a Kalman filter to the IMU's latest data
 * @see dof9_data
 */
FASTRUN void KalmanFilter()
{
    dof9_data.RollRate  = RollRateKalman.updateEstimate(dof9_data.RollRate);
    dof9_data.PitchRate = PitchRateKalman.updateEstimate(dof9_data.PitchRate);
    dof9_data.YawRate   = YawRateKalman.updateEstimate(dof9_data.YawRate);
}

/**
 * Read measurements from the BNO055 sensor's event data & apply the Kalman filter
 * @see Get_BNO055(), KalmanFilter(), orientationData, angVelocityData
 */
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

/**
 * Get updated data from the MPU6050 sensor.
 * @see dof9_data
 */

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

void ScanI2c()
{
    delay(500); // allow time to wake things up
    for (uint8_t i = 1; i < 127; ++i) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            if (i == 0x28) {
                USE_BNO055A   = true;
                GyroInstalled = true;
#ifdef DB_SENSORS
                Serial.println("Real Adafruit BNO055 gyro detected!");
#endif
            }
            else if (i == 0x29) {
                USE_BNO055    = true;
                GyroInstalled = true;
#ifdef DB_SENSORS
                Serial.println("Cheapo BNO055 gyro clone detected!");
#endif
            }
            else if (i == 0x40) {
                USE_INA219 = true;
#ifdef DB_SENSORS
                Serial.println("INA219 voltage meter detected!");
#endif
            }
            else if (i == 0x68) {
                USE_MPU6050   = true;
                GyroInstalled = true;
#ifdef DB_SENSORS
                Serial.println("MPU6050 gyro detected!");
#endif
            }
            else if (i == 0x76) {
                USE_BMP280 = true;
#ifdef DB_SENSORS
                Serial.println("BMP280 barometer detected!");)
            }
            else {
                Serial.print(i, HEX);
                Serial.print("   "); // in case some new device shows up
#endif
            }
        }
    }
}

/** Store bounded pipe address from the received pairing payload. */
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

/** Initialize the BMP280 sensor */
void InitBMP280()
{
    bmp280.begin(0x76);
    bmp280.setTimeStandby(TIME_STANDBY_4000MS);
    bmp280.startNormalConversion();
    // warm up bmp280
    for (int i = 0; i < 15000; ++i) {
        bmp280.getMeasurements(temperature280, pressure, StartAltitude);
    }
    while (!bmp280.getMeasurements(temperature280, pressure, StartAltitude)) {
        // loop infinitely until data (from the sensor) is ready
    }
}

/************************************************************************************************************/
// SETUP
/************************************************************************************************************/

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); 
    Serial.begin(9600);
    Wire.begin();
    delay(2000); // Needed ! - possibly for stabilising capacitors.
    ScanI2c();   // see what's connected
    if (USE_BNO055) BNO055_sensor[1].begin();
    if (USE_BNO055A) BNO055_sensor[0].begin();
   

 #ifdef SECOND_TRANSCEIVER
    CurrentRadio = &Radio2;
    InitCurrentRadio(); // initialise BOTH at setup, if two.
    TwoRadiosInstalled = true;
 #endif 

    CurrentRadio = &Radio1;
    InitCurrentRadio();
    ThisRadio=1;

    if (USE_BMP280) {
#ifdef DB_SENSORS
        Serial.print("Starting BMP280 ... ");
#endif
        InitBMP280();
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
        Serial.println("\n");
#else
        mpu6050.calcGyroOffsets(false);
#endif
    }
    DeltaTime    = 0;
    MainLoopTime = millis();
    GetOldPipe();
    digitalWrite(LED_PIN, LOW);
    LoadVersioNumber();
}

/************************************************************************************************************/

void SaveFailSafeData()
{
    // FailSafe data occupies EEPROM from offset 10 to 26
    uint8_t FS_Offset = 10;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        EEPROMUpdateByte(i + FS_Offset, (map(ReceivedData[i], MINMICROS, MAXMICROS, 0, 180))); // save servo positions lower res: 8 bits
    }
    FS_Offset += CHANNELSUSED;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
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
    Serial.print("NewPipe: ");
    Serial.println((int)NewPipe, HEX);

    Serial.print("OldPipe: ");
    Serial.println((int)OldPipe, HEX);
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
        Serial.println((int)NewPipe, HEX);
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
