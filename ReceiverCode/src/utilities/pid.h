// This file contains the PID controller and the Kalman filter for the MPU6050 ****************************************************************************************

#ifndef _SRC_PID_H
#define _SRC_PID_H

#include <Arduino.h>
#include "utilities/1Definitions.h"

#ifdef USE_STABILISATION

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

  // Set register pointer to start of burst read
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_START_REG);
  Wire.endTransmission(false); // Use repeated start for efficiency

  // Avoid overload ambiguity by explicitly casting to bool
  if (Wire.requestFrom(MPU6050_ADDR, READ_LENGTH, static_cast<bool>(true)) != READ_LENGTH)
    return; // Exit if not all bytes are received

  int16_t AccXLSB = (Wire.read() << 8) | Wire.read();
  int16_t AccYLSB = (Wire.read() << 8) | Wire.read();
  int16_t AccZLSB = (Wire.read() << 8) | Wire.read(); /// 

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
  AccX = static_cast<float>(AccXLSB) / ACCEL_SCALE;
  AccY = static_cast<float>(AccYLSB) / ACCEL_SCALE;
  AccZ = static_cast<float>(AccZLSB) / ACCEL_SCALE;

  // Estimate roll and pitch angles from accelerometer data
  RawRollAngle = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * RAD_TO_DEGREES;
  RawPitchAngle = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEGREES;
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
// ******************************************************************************************************************************************************************
// Initializes and calibrates the MPU6050 — assumes Wire.begin() already called
void InitialiseTheMPU6050()
{
  constexpr int ITERATIONS = 1000;

  delay(250); // Let sensor stabilize

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

  RateCalibrationRoll = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw = 0;

  for (int i = 0; i < ITERATIONS; ++i)
  {
    Read_MPU6050();
    RateCalibrationRoll += RawRollRate;
    RateCalibrationPitch += RawPitchRate;
    RateCalibrationYaw += RawYawRate;
    delay(1);
    BlinkFast();
  }

  RateCalibrationRoll /= ITERATIONS;
  RateCalibrationPitch /= ITERATIONS;
  RateCalibrationYaw /= ITERATIONS;

  initKalman();
}
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
    Serial.println("RawPitch,FilteredPitch,RawRoll,FilteredRoll,RawYaw,FilteredYaw");

    Serial.print(RawPitchAngle);
    Serial.print(",");
    Serial.print(filteredPitch);
    Serial.print(",");

    Serial.print(RawRollAngle);
    Serial.print(",");
    Serial.print(filteredRoll);
    Serial.print(",");

    // Serial.print(RawYawRate);
    // Serial.print(",");
    // Serial.print(filteredYawRate);
    // Serial.print(",");
    Serial.println();
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
#endif // USE_STABILISATION
#endif // _SRC_PID_H
// End of file: src/utilities/pid.h
