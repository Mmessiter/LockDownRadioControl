// ************************************************** Receiver code **************************************************

#define RECEIVE_TIMEOUT 50 // 15 milliseconds was too short
#define PacketsPerHop   20
#define CHANNELSUSED    16
#define SERVOSUSED      10
#define SBUSRATE        10 // SBUS frame every 10 milliseconds
#define SBUSPORT        Serial3
#define SECOND_TRANSCEIVER       

bool USE_BMP280 = false; /** is BMP280 sensor connected */

#define EXTRAMICROS 500 // for extra resolution driving servos
#define MINMICROS   1000 - EXTRAMICROS
#define MAXMICROS   2000 + EXTRAMICROS

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

#include <Servo.h>
#include <EEPROM.h>
#include <SBUS.h>
#include <Adafruit_INA219.h>
#include <BMP280_DEV.h>
#include "utilities/radio.h"
#include "utilities/imu.h"

#define STICKSRATE 60 * 100 // Max

Adafruit_INA219 ina219;
BMP280_DEV      bmp280; /** The object to access the BMP280 sensor */
Servo           MCMServo[SERVOSUSED];

// uint16_t BNO055_SAMPLERATE_DELAY_MS = 100; // not actually used anywhere

uint8_t PWMPins[SERVOSUSED] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 16}; // ten now, last 6 only via sbus

#define LED_PIN 13

#define AEROPLANE  1
#define HELICOPTER 2
#define QUADCOPTER 3
#define HEXACOPTER 4
#define OCTOCOPTER 5

SBUS MySbus(SBUSPORT);

float  PacketStartTime;
double LastVoltMoment = 0;
float  Temperature6050; /** temperature data from the MPU6050 sensor's internal temperature sensor */

float temperature280, pressure, altitude, StartAltitude;
bool  Swash_DisplayStarted = false;

uint8_t ModelType = 0; // 1=Aeroplane 2=Heli 3=Quadcopter 4=Hexacopter 5=Octocopter

int      LoopsPS        = 0;
int      TimeThis       = 0;
int      MainLoopTime   = 0;
long int DeltaTime      = 0;     /** The ellapsed time between updates sent to the Servos */
uint8_t  BindNow        = 0;     /** indicates that the receiver should start the binding/pairing process */
bool     BoundFlag      = false; /** indicates if receiver paired with transmitter */
int      BindOKTimer    = 0;
bool     ServosAttached = false;
uint16_t SbusChannels[CHANNELSUSED + 8]; // a few spare
int      SBUSTimer = 0;
bool     FailSafeChannel[17];
bool     FailSafeDataLoaded = false;
uint8_t  ModelNumber        = 0;
bool     ModelNumberSaved   = false;
uint8_t  PowerSetting       = 4;
uint8_t  DataRate           = 1;
bool     ReInit             = false;
uint8_t  byte1              = 0;
uint8_t  byte2              = 0;
bool     GyroInstalled      = false;

/** Load project defaults from EEPROM into the ReceivedData buffer. */
void LoadFailSafeData()
{
    uint8_t  FS_Offset = 10;
    uint16_t s[CHANNELSUSED];

    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        s[i] = map(EEPROM.read(i + FS_Offset), 0, 180, MINMICROS, MAXMICROS); // load failsafe values and simulate better resloution
    }
    FS_Offset += CHANNELSUSED;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        if (EEPROM.read(i + FS_Offset)) {
            ReceivedData[i] = s[i];
        }
    }
    FailSafeDataLoaded = true;
#ifdef DB_FAILSAFE
    Serial.println("Fail safe settings are loaded!");
#endif
}

/************************************************************************************************************/

/** Map servo channels' data from ReceivedData buffer into SbusChannels buffer */
void MapToSBUS()
{
    int RangeMax = 2047; // = Frsky at 150 %
    int RangeMin = 0;

    for (int j = 0; j < CHANNELSUSED; ++j) {
        SbusChannels[j] = map(ReceivedData[j], MINMICROS, MAXMICROS, RangeMin, RangeMax);
    }
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

/** Execute FailSafe data from EEPROM. */
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

/************************************************************************************************************/

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
    Serial.println(BoundFlag ? " Bound" : " NOT Bound");
    PacketStartTime = millis();
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

void BindModel()
{
    ThisPipe = NewPipe;
    if (SaveNewBind) {
        for (uint8_t i = 0; i < 8; ++i) {
            EEPROM.update(i, ReceivedData[i]);
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
                mn = uint8_t(ReceivedData[CHANNELSUSED + 3]);
                if (mn != ModelNumber && mn > 0) {
                    ModelNumber = mn;
                    EEPROM.update(28, ModelNumber);
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
            FailSafeSave = bool(ReceivedData[CHANNELSUSED + 3]);
            if (FailSafeSave) {
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
        if (millis() - LastConnectionMoment >= RECEIVE_TIMEOUT) Reconnect();
    if (ReadData()) {
        uint8_t NextFrequency = CheckParams();
        if (PacketNumber >= PacketsPerHop) {
            HopToNextFrequency(NextFrequency);
            DoSensors();
        }
    }
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

void SaveFailSafeData()
{
    // FailSafe data occupies EEPROM from offset 10 to 26
    uint8_t FS_Offset = 10;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        EEPROM.update(i + FS_Offset, (map(ReceivedData[i], MINMICROS, MAXMICROS, 0, 180))); // save servo positions lower res: 8 bits
    }
    FS_Offset += CHANNELSUSED;
    for (uint8_t i = 0; i < CHANNELSUSED; ++i) {
        EEPROM.update(i + FS_Offset, FailSafeChannel[i]); // save flags
    }
#ifdef DB_FAILSAFE
    Serial.println("Fail safe settings are saved!");
#endif
    FailSafeSave = false;
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
        else if ((millis() - BindOKTimer) > 400) { // allow .4 of a second for the TX to bind
            BindNow = 1;
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
#endif

    CurrentRadio = &Radio1;
    InitCurrentRadio();
    ThisRadio = 1;

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
                if (USE_MPU6050) {
                    Get_Mpu6050();

                    // These values are reported to Transmitter
                    AckPayload.ReportedPitch = dof9_data.Pitch;
                    AckPayload.ReportedRoll  = dof9_data.Roll;
                    AckPayload.ReportedYaw   = dof9_data.Yaw;
                }
            }
        }
        if (FailSafeSave) SaveFailSafeData();
        MainLoopTime = millis();
        DeltaTime    = micros();
    }
    else {
        DoBinding();
    }
}
