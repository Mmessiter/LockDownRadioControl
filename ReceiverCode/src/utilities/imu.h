/** @file ReceiverCode/src/utilities/imu.h */
#ifndef _SRC_UTILITIES_IMU_H
#define _SRC_UTILITIES_IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MPU6050_tockn.h>
#include <SimpleKalmanFilter.h>

#define MAXCORRECTION 15
#define DeadBand      4
#define KK            15 /** The value for the Kalman filter's measured and estimated parameters */

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

DOF9 dof9_data = DOF9(); /** The 9 DOF data. This object is used to cache and filter the data from the IMU. */

/** Generic struct to hold component-specific PID values. */
struct PID
{
    float P = 0; /** Proportional value */
    float I = 0; /** Integral value */
    float D = 0; /** Derivative value */
};
PID SwashPID = PID(); /** Used to stabilize Roll & Pitch Rates */
PID YawPID   = PID(); /** Used to stabilize Yaw Rate */

float RollRateIntegral    = 0;
float OldRollRateIntegral = 0;
float RollRateDerivative  = 0;
float OldRollRateError    = 0;

float PitchRateIntegral    = 0;
float OldPitchRateIntegral = 0;
float PitchRateDerivative  = 0;
float OldPitchRateError    = 0;

extern long int DeltaTime; // defined in main.cpp

bool USE_INA219  = false; //  Volts INA219
bool USE_BNO055  = false; /** Cheap BNO055 gyro */
bool USE_BNO055A = false; /** Adafruit BNO055 gyro */
//bool USE_MPU6050 = false; /** Gyro MPU6050 */

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

#endif // define (_SRC_UTILITIES_IMU_H)
