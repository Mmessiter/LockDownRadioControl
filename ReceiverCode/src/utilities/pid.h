

// This file contains the PID controller and the Kalman filter for the MPU6050
// UPDATED VERSION: Can be calibrated at any angle and will use that as the zero reference

#ifndef _SRC_PID_H
#define _SRC_PID_H

#include <Arduino.h>
#include "utilities/1Definitions.h"

#ifdef USE_STABILISATION

// Store the reference gravity vector at calibration
float CalibX = 0.0f;
float CalibY = 0.0f;
float CalibZ = 0.0f;

// ****************************************************************************************************
/// @brief Reads raw accelerometer and gyroscope data from the MPU6050 and calculates angles
void Read_MPU6050(void)
{
  constexpr uint8_t MPU6050_ADDR = 0x68;
  constexpr uint8_t ACCEL_START_REG = 0x3B;
  constexpr uint8_t READ_LENGTH = 14;
  constexpr float ACCEL_SCALE = 4096.0f;
  constexpr float GYRO_SCALE = 65.5f;
  constexpr float RAD_TO_DEGREES = 180.0f / M_PI;

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_START_REG);
  Wire.endTransmission(false);

  if (Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)READ_LENGTH, true) != READ_LENGTH)
    return;

  int16_t AccXLSB = (Wire.read() << 8) | Wire.read();
  int16_t AccYLSB = (Wire.read() << 8) | Wire.read();
  int16_t AccZLSB = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read(); // skip temp

  int16_t GyroX = (Wire.read() << 8) | Wire.read();
  int16_t GyroY = (Wire.read() << 8) | Wire.read();
  int16_t GyroZ = (Wire.read() << 8) | Wire.read();

  RawRollRate = static_cast<float>(GyroX) / GYRO_SCALE;
  RawPitchRate = static_cast<float>(GyroY) / GYRO_SCALE;
  RawYawRate = static_cast<float>(GyroZ) / GYRO_SCALE;

  float AccX = static_cast<float>(AccXLSB) / ACCEL_SCALE;
  float AccY = static_cast<float>(AccYLSB) / ACCEL_SCALE;
  float AccZ = static_cast<float>(AccZLSB) / ACCEL_SCALE;

  // Use dot products to calculate roll and pitch relative to calibration vector
  float normRef = sqrt(CalibX * CalibX + CalibY * CalibY + CalibZ * CalibZ);
  float normNow = sqrt(AccX * AccX + AccY * AccY + AccZ * AccZ);

  float dot = (CalibX * AccX + CalibY * AccY + CalibZ * AccZ) / (normRef * normNow);
  dot = constrain(dot, -1.0f, 1.0f);
  float angleDiff = acos(dot) * RAD_TO_DEGREES;

  float crossX = CalibY * AccZ - CalibZ * AccY;
  float crossY = CalibZ * AccX - CalibX * AccZ;

  RawRollAngle = angleDiff * (crossX > 0 ? 1 : -1);
  RawPitchAngle = angleDiff * (crossY > 0 ? 1 : -1);
}

// ****************************************************************************************************
/// @brief Initialises MPU6050 and calculates gyro/angle offsets
void InitialiseTheMPU6050()
{
  constexpr int ITERATIONS = 2000;
  constexpr uint8_t MPU_ADDR = 0x68;
  constexpr uint8_t BYTES_TO_READ = 14;

  delay(500);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  float rollRateSum = 0, pitchRateSum = 0, yawRateSum = 0;
  float AccumX = 0, AccumY = 0, AccumZ = 0;

  for (int i = 0; i < ITERATIONS; ++i)
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)BYTES_TO_READ, true) == BYTES_TO_READ)
    {
      int16_t ax = (Wire.read() << 8) | Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();
      Wire.read();
      Wire.read();
      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();

      rollRateSum += gx / 65.5f;
      pitchRateSum += gy / 65.5f;
      yawRateSum += gz / 65.5f;

      AccumX += ax / 4096.0f;
      AccumY += ay / 4096.0f;
      AccumZ += az / 4096.0f;
    }
    delay(1);
    BlinkFast();
  }

  RateCalibrationRoll = rollRateSum / ITERATIONS;
  RateCalibrationPitch = pitchRateSum / ITERATIONS;
  RateCalibrationYaw = yawRateSum / ITERATIONS;

  CalibX = AccumX / ITERATIONS;
  CalibY = AccumY / ITERATIONS;
  CalibZ = AccumZ / ITERATIONS;

  Read_MPU6050();

  Serial.print("Calibration complete. Gyro biases: Roll=");
  Serial.print(RateCalibrationRoll);
  Serial.print(", Pitch=");
  Serial.print(RateCalibrationPitch);
  Serial.print(", Yaw=");
  Serial.println(RateCalibrationYaw);
  Serial.print("Calibrated gravity vector: ");
  Serial.print(CalibX);
  Serial.print(", ");
  Serial.print(CalibY);
  Serial.print(", ");
  Serial.println(CalibZ);
  Serial.print("Initial roll/pitch: ");
  Serial.print(RawRollAngle);
  Serial.print(", ");
  Serial.println(RawPitchAngle);

  initKalman();
}


// ******************************************************************************************************************************************************************
// This function is used to time the loop and print the loop rate to the Serial Monitor for debugging purposes only
void TimeTheLoop()
{
  static uint32_t previousTime = 0;
  static uint16_t Counter = 0;
  if (millis() - previousTime > 1000)
  {
    Serial.print("Loop rate: ");
    Serial.print(Counter);
    Serial.println(" Hz");
    Counter = 0;
    previousTime = millis();
  }
  ++Counter;
}

// ******************************************************************************************************************************************************************
// This function gets the current attitude of the aircraft
void GetCurrentAttitude()
{
  static uint32_t LoopTimer;
  uint32_t Now = millis(); // Get the current time in milliseconds with a single call to save time.
  static uint8_t counter = 0;
  if (Now - LoopTimer  < 2 ) // 2ms loop time or 500 Hz !!!!!!!!!!!!!!!!!
  {
    return;
  }
  LoopTimer = Now;
  //TimeTheLoop(); // Print loop rate to Serial Monitor
  Read_MPU6050();
  RawRollRate -= RateCalibrationRoll;   // Correct for gyro calibration
  RawPitchRate -= RateCalibrationPitch; // Correct for gyro calibration
  RawYawRate -= RateCalibrationYaw;     // Correct for gyro calibration
  kalmanFilter();
  filterRatesForHelicopter();
  // The following lines are for the Serial Plotter
  if (++counter > 12)
  {
    // Print header once (this line will be ignored by the Serial Plotter's graph)
    Serial.println("RawPitch,FilteredPitch,RawRoll,FilteredRoll,RawYaw,FilteredYaw"); // heer

    Serial.print(RawPitchAngle);
    Serial.print(",");
     Serial.print(getFilteredPitchAngle());
    //Serial.print(filteredPitchRate);
    Serial.print(",");

    Serial.print(RawRollAngle);
    Serial.print(",");
    //Serial.print(filteredRollRate);
     Serial.print(getFilteredRollAngle());
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
  // const float beta = 0.2f; // not enough filtering
  const float beta = 0.01f; // very aggressive filtering
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
