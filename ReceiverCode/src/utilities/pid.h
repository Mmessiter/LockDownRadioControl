// This file contains the PID controller and the Kalman filter for the MPU6050
// FIXED VERSION: Can be calibrated at any angle and will use that as the zero reference

#ifndef _SRC_PID_H
#define _SRC_PID_H

#include <Arduino.h>
#include "utilities/1Definitions.h"

#ifdef USE_STABILISATION

// These store the sensor readings that correspond to the CALIBRATION ORIENTATION (0°)
float CalibrationRollReading = 0.0f;
float CalibrationPitchReading = 0.0f;

// ****************************************************************************************************
/// @brief Reads raw accelerometer and gyroscope data from the MPU6050 and calculates angles
void Read_MPU6050(void)
{
  constexpr uint8_t MPU6050_ADDR = 0x68;
  constexpr uint8_t ACCEL_START_REG = 0x3B;
  constexpr uint8_t READ_LENGTH = 14;
  constexpr float ACCEL_SCALE = 4096.0f; // LSB/g
  constexpr float GYRO_SCALE = 65.5f;    // LSB/(°/s)
  constexpr float RAD_TO_DEGREES = 180.0f / M_PI;

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_START_REG);
  Wire.endTransmission(false); // Use repeated start

  if (Wire.requestFrom(MPU6050_ADDR, READ_LENGTH, static_cast<bool>(true)) != READ_LENGTH)
    return;

  int16_t AccXLSB = (Wire.read() << 8) | Wire.read();
  int16_t AccYLSB = (Wire.read() << 8) | Wire.read();
  int16_t AccZLSB = (Wire.read() << 8) | Wire.read();

  Wire.read(); // Skip temperature high byte
  Wire.read(); // Skip temperature low byte

  int16_t GyroX = (Wire.read() << 8) | Wire.read();
  int16_t GyroY = (Wire.read() << 8) | Wire.read();
  int16_t GyroZ = (Wire.read() << 8) | Wire.read();

  // Convert raw gyro data to degrees per second
  RawRollRate = static_cast<float>(GyroX) / GYRO_SCALE;
  RawPitchRate = static_cast<float>(GyroY) / GYRO_SCALE;
  RawYawRate = static_cast<float>(GyroZ) / GYRO_SCALE;

  // Convert raw accelerometer data to g
  float AccX = static_cast<float>(AccXLSB) / ACCEL_SCALE;
  float AccY = static_cast<float>(AccYLSB) / ACCEL_SCALE;
  float AccZ = static_cast<float>(AccZLSB) / ACCEL_SCALE;

  // Calculate current angles using improved formula
  float currentRollReading = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * RAD_TO_DEGREES;
  float currentPitchReading = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEGREES;

  // Calculate angles relative to calibration orientation
  RawRollAngle = currentRollReading - CalibrationRollReading;
  RawPitchAngle = currentPitchReading - CalibrationPitchReading;
}

// ****************************************************************************************************
/// @brief Initialises MPU6050 and calculates gyro/angle offsets
void InitialiseTheMPU6050()
{
  constexpr int ITERATIONS = 2000;

  delay(500); // Let sensor stabilise after power-up

  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Wake up
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x08); // ±500°/s
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x08); // ±4g
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A); // CONFIG
  Wire.write(0x03); // Low-pass filter ~43Hz
  Wire.endTransmission();

  // Initialize calibration accumulators
  float rollRateSum = 0;
  float pitchRateSum = 0;
  float yawRateSum = 0;

  // Accumulators for raw accelerometer data
  float AccumulatedAccX = 0;
  float AccumulatedAccY = 0;
  float AccumulatedAccZ = 0;

  // Calibration loop - collect raw data
  for (int i = 0; i < ITERATIONS; ++i)
  {
    // Read raw sensor data directly in this loop
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    if (Wire.requestFrom(static_cast<uint8_t>(0x68), static_cast<uint8_t>(14), true) == 14)
    {
      int16_t AccXLSB = (Wire.read() << 8) | Wire.read();
      int16_t AccYLSB = (Wire.read() << 8) | Wire.read();
      int16_t AccZLSB = (Wire.read() << 8) | Wire.read();

      Wire.read();
      Wire.read(); // Skip temperature

      int16_t GyroX = (Wire.read() << 8) | Wire.read();
      int16_t GyroY = (Wire.read() << 8) | Wire.read();
      int16_t GyroZ = (Wire.read() << 8) | Wire.read();

      // Accumulate gyro rates for bias calculation
      rollRateSum += static_cast<float>(GyroX) / 65.5f;
      pitchRateSum += static_cast<float>(GyroY) / 65.5f;
      yawRateSum += static_cast<float>(GyroZ) / 65.5f;

      // Accumulate raw accelerometer values
      AccumulatedAccX += static_cast<float>(AccXLSB) / 4096.0f;
      AccumulatedAccY += static_cast<float>(AccYLSB) / 4096.0f;
      AccumulatedAccZ += static_cast<float>(AccZLSB) / 4096.0f;
    }

    delay(1);
    BlinkFast();
  }

  // Calculate average gyro rates (bias correction)
  RateCalibrationRoll = rollRateSum / ITERATIONS;
  RateCalibrationPitch = pitchRateSum / ITERATIONS;
  RateCalibrationYaw = yawRateSum / ITERATIONS;

  // Calculate average accelerometer values during calibration
  float avgAccX = AccumulatedAccX / ITERATIONS;
  float avgAccY = AccumulatedAccY / ITERATIONS;
  float avgAccZ = AccumulatedAccZ / ITERATIONS;

  // Calculate the orientation during calibration (this becomes our zero reference)
  CalibrationRollReading = atan2(avgAccY, sqrt(avgAccX * avgAccX + avgAccZ * avgAccZ)) * 180.0f / M_PI;
  CalibrationPitchReading = atan2(-avgAccX, sqrt(avgAccY * avgAccY + avgAccZ * avgAccZ)) * 180.0f / M_PI;

  // Debug output
  Serial.print("Calibration complete. Gyro biases: Roll=");
  Serial.print(RateCalibrationRoll);
  Serial.print("°/s, Pitch=");
  Serial.print(RateCalibrationPitch);
  Serial.print("°/s, Yaw=");
  Serial.print(RateCalibrationYaw);
  Serial.println("°/s");

  Serial.print("Calibration orientation: Roll=");
  Serial.print(CalibrationRollReading);
  Serial.print("°, Pitch=");
  Serial.print(CalibrationPitchReading);
  Serial.println("° (this is now 0°)");

  initKalman();
}

// [Rest of the original code remains exactly the same...]
// ******************************************************************************************************************************************************************
// This function gets the current attitude of the aircraft
void GetCurrentAttitude()
{
  static uint32_t LoopTimer;
  static uint8_t counter = 0;
  if (millis() - LoopTimer < 4) // 4ms loop time or 250Hz
    return;
  LoopTimer = millis();
  Read_MPU6050();
  RawRollRate -= RateCalibrationRoll;   // Correct for gyro calibration
  RawPitchRate -= RateCalibrationPitch; // Correct for gyro calibration
  RawYawRate -= RateCalibrationYaw;     // Correct for gyro calibration
  kalmanFilter();
  filterRatesForHelicopter();
  // The following lines are for the Serial Plotter
  if (++counter > 6)
  {
    // Print header once (this line will be ignored by the Serial Plotter's graph)
    Serial.println("RawPitch,FilteredPitch,RawRoll,FilteredRoll,RawYaw,FilteredYaw"); // heer

    Serial.print(RawPitchRate);
    Serial.print(",");
   // Serial.print(getFilteredPitchAngle());
    Serial.print(filteredPitchRate);
    Serial.print(",");

    Serial.print(RawRollRate);
    Serial.print(",");
    Serial.print(filteredRollRate);
   // Serial.print(getFilteredRollAngle());
    Serial.print(",");

    // Serial.print(RawYawRate);
    // Serial.print(",");
    // Serial.print(filteredYawRate);
    // Serial.print(",");
    Serial.println();

    //  Look(getFilteredRollAngle());
    //  Look(getFilteredPitchAngle());
    counter = 0;
  }
}

// ******************************************************************************************************************************************************************
void DoStabilsation()
{
  if (!MPU6050Connected)
  {
    return;
  }
  GetCurrentAttitude();
}

// ******************************************************************************************************************************************************************
void BlinkFast()
{ // This function blinks the LED fast to indicate that the gyro is being calibrated
  static uint32_t BlinkTimer = millis();
  if (millis() - BlinkTimer < 80)
    return; // Blink only for 80ms
  BlinkTimer = millis();
  if (LedIsOn)
    TurnLedOff();
  else
    TurnLedOn();
}

float Q_angle = 0.01f;
float Q_bias = 0.003f;
float R_measure = 0.03f;
#define USE_ANGLE_SMOOTHING

// ===================================================================================
void initKalman()
{
  static bool kalmanInitialised = false;
  if (kalmanInitialised)
    return;
  kalmanInitialised = true;

  kalmanRoll.angle = RawRollAngle;
  kalmanRoll.P[0][0] = 1;
  kalmanRoll.P[0][1] = 0;
  kalmanRoll.P[1][0] = 0;
  kalmanRoll.P[1][1] = 1;

  kalmanPitch.angle = RawPitchAngle;
  kalmanPitch.P[0][0] = 1;
  kalmanPitch.P[0][1] = 0;
  kalmanPitch.P[1][0] = 0;
  kalmanPitch.P[1][1] = 1;

  kalmanYaw.angle = 0;
  kalmanYaw.P[0][0] = 1;
  kalmanYaw.P[0][1] = 0;
  kalmanYaw.P[1][0] = 0;
  kalmanYaw.P[1][1] = 1;

  previousTime = micros();
}

// ===================================================================================
// Kalman Update Function
// ===================================================================================
float updateKalman(KalmanState &state, float accelAngle, float gyroRate, float dt)
{
  // Predict step (integrate gyro)
  state.angle += dt * (gyroRate - state.bias);

  state.P[0][0] += dt * (dt * state.P[1][1] - state.P[0][1] - state.P[1][0] + Q_angle);
  state.P[0][1] -= dt * state.P[1][1];
  state.P[1][0] -= dt * state.P[1][1];
  state.P[1][1] += Q_bias * dt;

  // Innovation
  float y = accelAngle - state.angle;

  // Kalman gain
  float S = state.P[0][0] + R_measure;
  float K[2] = {state.P[0][0] / S, state.P[1][0] / S};

  // Correction
  state.angle += K[0] * y;
  state.bias += K[1] * y;

  // Update covariance
  float P00_temp = state.P[0][0];
  float P01_temp = state.P[0][1];

  state.P[0][0] -= K[0] * P00_temp;
  state.P[0][1] -= K[0] * P01_temp;
  state.P[1][0] -= K[1] * P00_temp;
  state.P[1][1] -= K[1] * P01_temp;

  return state.angle;
}

// ===================================================================================
// Main Kalman Filter Step
// ===================================================================================
void kalmanFilter()
{
  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) / 1e6f;
  previousTime = currentTime;
  if (dt <= 0 || dt > 0.05f)
    dt = 0.01f;

#ifdef USE_ANGLE_SMOOTHING
  static float smoothedRoll = 0;
  static float smoothedPitch = 0;
  const float alpha = 0.05f;
  smoothedRoll = (1 - alpha) * smoothedRoll + alpha * RawRollAngle;
  smoothedPitch = (1 - alpha) * smoothedPitch + alpha * RawPitchAngle;
#else
  float smoothedRoll = RawRollAngle;
  float smoothedPitch = RawPitchAngle;
#endif

  filteredRoll = updateKalman(kalmanRoll, smoothedRoll, RawRollRate, dt);
  filteredPitch = updateKalman(kalmanPitch, smoothedPitch, RawPitchRate, dt);
  filteredYaw = updateKalman(kalmanYaw, 0, RawYawRate, dt);

  filteredRollRate = RawRollRate - kalmanRoll.bias;
  filteredPitchRate = RawPitchRate - kalmanPitch.bias;
  filteredYawRate = RawYawRate - kalmanYaw.bias;
}

// ===================================================================================
// Additional Low-Pass Filtering for Helicopter Control Rates
// ===================================================================================
void filterRatesForHelicopter()
{
  static float lastRollRate = 0, lastPitchRate = 0, lastYawRate = 0;
  //const float beta = 0.2f; // not enough filtering
  const float beta = 0.05f; // Adjust this value for more or less filtering
  filteredRollRate = (1 - beta) * lastRollRate + beta * filteredRollRate;
  filteredPitchRate = (1 - beta) * lastPitchRate + beta * filteredPitchRate;
  filteredYawRate = (1 - beta) * lastYawRate + beta * filteredYawRate;
  lastRollRate = filteredRollRate;
  lastPitchRate = filteredPitchRate;
  lastYawRate = filteredYawRate;
}

// ===================================================================================
// Helper Functions for Angle Tracking
// ===================================================================================
bool IsInverted()
{
  return (getFilteredRollAngle() < -135.0f || getFilteredRollAngle() > 135.0f);
}

float getFilteredRollAngle()
{
  return kalmanRoll.angle;
}

float getFilteredPitchAngle()
{
  return kalmanPitch.angle;
}

#endif // USE_STABILISATION
#endif // _SRC_PID_H
       // End of file: src/utilities/pid.h
