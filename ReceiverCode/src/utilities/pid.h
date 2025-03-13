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
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
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

  int Iterations = 1000; // make bigger for better calibration after tests

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (int i = 0; i < Iterations; ++i)
  {
    Read_MPU6050();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
    BlinkFast();
  }
  RateCalibrationRoll /= Iterations;
  RateCalibrationPitch /= Iterations;
  RateCalibrationYaw /= Iterations;
}

// ******************************************************************************************************************************************************************
// This function is the Kalman filter for 1D signals
void Kalman_Filter_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// ******************************************************************************************************************************************************************
// This function gets the current attitude of the aircraft

void GetCurrentAttitude()
{

  static uint32_t LoopTimer;
  static uint32_t LoopTimer2;
  static uint32_t IterationCounter = 0;
  if (millis() - LoopTimer < 4)
    return;
  LoopTimer = millis();

  // if (millis() - LoopTimer2 < 1000)
  //   ++IterationCounter;
  // else
  // {
  //   Look(IterationCounter);
  //   IterationCounter = 0;
  //   LoopTimer2 = millis();
  // }

  Read_MPU6050(); // Read the gyro and accelerometer

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  Kalman_Filter_1d(CurrentRollAngle, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll); // Kalman filter for roll
  CurrentRollAngle = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  Kalman_Filter_1d(CurrentPitchAngle, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch); // Kalman filter for pitch
  CurrentPitchAngle = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Serial.print("Roll Angle [°] ");
  // Serial.println(CurrentRollAngle);
  // Serial.print(" Pitch Angle [°] ");
  //  Serial.println(CurrentPitchAngle);
  // Serial.println(RateYaw);
}

void DoStabilsation()
{
  GetCurrentAttitude();
  return;
}
#endif // USE_STABILISATION

#endif // _SRC_PID_H

// End of file: src/utilities/pid.h
