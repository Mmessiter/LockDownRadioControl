// This file contains the PID controller and the Kalman filter for the MPU6050 ****************************************************************************************

#ifndef _SRC_PID_H
#define _SRC_PID_H

#include <Arduino.h>
#include "utilities/common.h"

#ifdef USE_STABILISATION

// This function reads the MPU6050 registers  frequently to get the gyro and accelerometer signals
void Read_MPU6050(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  
  Wire.beginTransmission(0x68); //
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
 
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
 
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RawRollRate = (float)GyroX / 65.5;
  RawPitchRate = (float)GyroY / 65.5;
  RawYawRate = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
  RawRollAngle = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  RawPitchAngle = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}
// ******************************************************************************************************************************************************************
/// @brief // This function reads the MPU6050 registers frequently to get the gyro and accelerometer signals
void Read_MPU6050FASTER(void)
{
  Wire.beginTransmission(0x68);                                                   // Start the transmission
  Wire.write(0x3B);                                                               // Starting register for Accel data
  Wire.endTransmission();                                                         // End the transmission
  Wire.requestFrom(0x68, 14);                                                     // Request 14 bytes from the MPU-6050
  int16_t AccXLSB = (Wire.read() << 8) | Wire.read();                             // Combine the two 8-bit registers into one 16-bit number
  int16_t AccYLSB = (Wire.read() << 8) | Wire.read();                             // Combine the two 8-bit registers into one 16-bit number
  int16_t AccZLSB = (Wire.read() << 8) | Wire.read();                             // Combine the two 8-bit registers into one 16-bit number
  Wire.read();                                                                    // Skip the temperature data
  Wire.read();                                                                    // Skip the temperature data
  int16_t GyroX = (Wire.read() << 8) | Wire.read();                               // Combine the two 8-bit registers into one 16-bit number
  int16_t GyroY = (Wire.read() << 8) | Wire.read();                               // Combine the two 8-bit registers into one 16-bit number
  int16_t GyroZ = (Wire.read() << 8) | Wire.read();                               // Combine the two 8-bit registers into one 16-bit number
  RawRollRate = (float)GyroX / 65.5;                                              // Convert to degrees per second
  RawPitchRate = (float)GyroY / 65.5;                                             // Convert to degrees per second
  RawYawRate = (float)GyroZ / 65.5;                                               // Convert to degrees per second
  AccX = (float)AccXLSB / 4096;                                                   // Convert to g
  AccY = (float)AccYLSB / 4096;                                                   // Convert to g
  AccZ = (float)AccZLSB / 4096;                                                   // Convert to g
  RawRollAngle = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / M_PI);   // Convert to degrees
  RawPitchAngle = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / M_PI); // Convert to degrees
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

// This function initialises the MPU6050 and calibrates the gyro

void InitialiseTheMPU6050()
{
#define ITERATIONS 1000 // make bigger for better calibration after tests
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x08); // 0x08 for ±500°/s, 0x10 for ±1000°/s
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x08); // 0x08 for ±4g
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A); // CONFIG register
  Wire.write(0x03); // ~43Hz bandwidth for both gyro and accel
  Wire.endTransmission();

  for (int i = 0; i < ITERATIONS; ++i)
  {
    Read_MPU6050FASTER();
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
  Read_MPU6050FASTER();
  RawRollRate -= RateCalibrationRoll;   // Correct for gyro calibration
  RawPitchRate -= RateCalibrationPitch; // Correct for gyro calibration
  RawYawRate -= RateCalibrationYaw;     // Correct for gyro calibration
  kalmanFilter();
  filterRatesForHelicopter();
  ++counter;
  if (counter > 5)
  {
    // Print header once (this line will be ignored by the Serial Plotter's graph)
    Serial.println("Raw,Filtered");

    // Then stream only numeric data:
    Serial.print(RawRollAngle);
    Serial.print(",");
    Serial.println(filteredRoll);
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
