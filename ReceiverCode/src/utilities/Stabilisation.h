// This file contains the PID controller and the Kalman filter for the MPU6050
// FIXED VERSION: Can be calibrated at any angle and will use that as the zero reference

#ifndef STABILISATION_CODE
#define STABILISATION_CODE

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
  float currentRollReading = atan2(AccY, AccZ) * RAD_TO_DEGREES;
  float currentPitchReading = atan2(-AccX, AccZ) * RAD_TO_DEGREES;

  // Calculate angles relative to calibration orientation
  RawRollAngle = currentRollReading - CalibrationRollReading;
  RawPitchAngle = currentPitchReading - CalibrationPitchReading;
}

// ****************************************************************************************************
void PerformMPU6050Calibration() // Calibrate and save result
{
  constexpr int ITERATIONS = 2000;
  // Initialize calibration accumulators
  float rollRateSum = 0;
  float pitchRateSum = 0;
  float yawRateSum = 0;

  // Accumulators for raw accelerometer data
  float AccumulatedAccX = 0;
  float AccumulatedAccY = 0;
  float AccumulatedAccZ = 0;

  uint32_t Aileron_Sum = 0, Elevator_Sum = 0, Rudder_Sum = 0, Throttle_Sum = 0;

  KickTheDog();
  // Calibration loop - sensor must be flat. Calibration is needed because no calibration data was found on the EEPROM
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

      ReceiveData();
      Aileron_Sum += ReceivedData[0];  // Aileron
      Elevator_Sum += ReceivedData[1]; // Elevator
      Throttle_Sum += ReceivedData[2]; // Throttle
      Rudder_Sum += ReceivedData[3];   // Rudder
    }
    BlinkFast();
    KickTheDog();
  }
  // Calculate average gyro rates (bias correction)
  RateCalibrationRoll = rollRateSum / ITERATIONS;
  RateCalibrationPitch = pitchRateSum / ITERATIONS;
  RateCalibrationYaw = yawRateSum / ITERATIONS;

  // Calculate average control surface positions
  Aileron_Centre = Aileron_Sum / ITERATIONS; 
  Elevator_Centre = Elevator_Sum / ITERATIONS;
  Rudder_Centre = Rudder_Sum / ITERATIONS;
  Throttle_Centre = Throttle_Sum / ITERATIONS;

  // Calculate average accelerometer values during calibration
  float avgAccX = AccumulatedAccX / ITERATIONS;
  float avgAccY = AccumulatedAccY / ITERATIONS;
  float avgAccZ = AccumulatedAccZ / ITERATIONS;

  // Calculate the orientation during calibration (this becomes our zero reference)
  CalibrationRollReading = atan2(avgAccY, sqrt(avgAccX * avgAccX + avgAccZ * avgAccZ)) * 180.0f / M_PI;
  CalibrationPitchReading = atan2(-avgAccX, sqrt(avgAccY * avgAccY + avgAccZ * avgAccZ)) * 180.0f / M_PI;
  SaveMPU6050CalibrationDataToEEPROM();       // Save calibration data to EEPROM
  delay(1000);                                // Brief pause to ensure EEPROM flash write completes before re-read
  if (LoadMPU6050CalibrationDataFromEEPROM()) // check by reading it back.
  {
    TurnLedOff(); // Turn off the LED after calibration. This indicates success.
    while (true)
      delay(50); // reboot by rather seriously upsetting the watchdog
  }
  else //  read failed so we have an error
  {
    TurnLedOn();                                               // Turn on the LED to indicate an error
    Look("ERROR: Failed to save calibration data to EEPROM."); // only useful when debugging at the computer
    CalibrationStatus = CALIBRATION_STATUS_FAILED;
  }
}
// ****************************************************************************************************
void DebugCentres()
{
  Look1("Aileron_Centre: ");
  Look(Aileron_Centre);
  Look1(" Elevator_Centre: ");
  Look(Elevator_Centre);
  Look1(" Rudder_Centre: ");
  Look(Rudder_Centre);
  Look1(" Throttle_Centre: ");
  Look(Throttle_Centre);
}

// ****************************************************************************************************
// This initialisation function wakes up the sensor and sets various parameters:
// Gyro: ±500°/s | Accel: ±4g | Low-pass filter ~43Hz
// Then it looks to see if calibration data had been previously saved in the EEPROM.
// If it had not, then the sensor is calibrated and the results saved in the EEPROM for next time.
// NOTA BENE: Calibration *MUST* be performed with the model precisely horizontal.

void InitialiseTheMPU6050()
{
  delay(250); // Let sensor stabilise after power-up
  // wake up the sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configure Gyro ±500°/s
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // configure accelerometer ±4g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();

  // configure low pass filter  ~43Hz
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  delay(100);
  if (!LoadMPU6050CalibrationDataFromEEPROM())
  {
    CalibrationStatus = CALIBRATION_STATUS_FAILED;
  }
  else
  {
    CalibrationStatus = CALIBRATION_STATUS_SUCCEEDED;
   // DebugCentres();
  }

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
    Look1("Loop rate: ");
    Look1(Counter);
    Look(" Hz");
    Counter = 0;
    previousTime = millis();
  }
  ++Counter;
}
// ******************************************************************************************************************************************************************
// This function filters the roll, pitch and yaw rates for helicopters using a low-pass filter
void PlotRates()
{
  // Print header once (this line will be ignored by the Serial Plotter's graph)
  Look("RawRollRate,FilteredRollRate,RawPitchRate,FilteredPitchRate,RawYawRate,FilteredYawRate");

  Look1(RawRollRate);
  Look1(",");
  Look1(filteredRollRate);
  Look1(",");
  Look1(RawPitchRate);
  Look1(",");
  Look1(filteredPitchRate);
  Look1(",");
  Look1(RawYawRate);
  Look1(",");
  Look(filteredYawRate);
}
// ******************************************************************************************************************************************************************
void PlotAttitude()
{
  // Print header once (this line will be ignored by the Serial Plotter's graph)
  Look("RawPitch,FilteredPitch,RawRoll,FilteredRoll,RawYaw,FilteredYaw"); //

  Look1(RawPitchAngle);
  Look1(",");
  Look1(filteredPitch);
  Look1(",");
  Look1(RawRollAngle);
  Look1(",");
  Look1(filteredRoll);
  Look(",");
}

// ******************************************************************************************************************************************************************
// This function gets the current attitude of the aircraft
void GetCurrentAttitude()
{
  static uint32_t LoopTimer;
  uint32_t Now = millis(); // Get the current time in milliseconds with a single call to save time.
  static uint8_t counter = 0;
  if (Now - LoopTimer < 2) // 2ms loop time or 500 Hz
  {
    return;
  }
  LoopTimer = Now;
  // TimeTheLoop(); // Print loop rate to Serial Monitor
  Read_MPU6050();
  RawRollRate -= RateCalibrationRoll;   // Correct for gyro calibration
  RawPitchRate -= RateCalibrationPitch; // Correct for gyro calibration
  RawYawRate -= RateCalibrationYaw;     // Correct for gyro calibration

  if (UseKalmanFilter)
  {
    kalmanFilter(); // heer
  }
  else
  {
    filteredRollRate = RawRollRate;
    filteredPitchRate = RawPitchRate;
    filteredYawRate = RawYawRate;
    filteredRoll = RawRollAngle;
    filteredPitch = RawPitchAngle;
  }
  if (UseRateLFP)
  {
    filterRatesForHelicopter();
  }
  if (++counter > 6)
  {
    // PlotRates();
     PlotAttitude(); // Print the attitude to the Serial Plotter
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

  state.P[0][0] += dt * (dt * state.P[1][1] - state.P[0][1] - state.P[1][0] + Kalman_Q_angle);
  state.P[0][1] -= dt * state.P[1][1];
  state.P[1][0] -= dt * state.P[1][1];
  state.P[1][1] += Kalman_Q_bias * dt;

  // Innovation
  float y = accelAngle - state.angle;

  // Kalman gain
  float S = state.P[0][0] + Kalman_R_measure;
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
  uint32_t currentTime = micros();
  float dt = (currentTime - previousTime) / 1e6f;
  previousTime = currentTime;
  if (dt <= 0 || dt > 0.05f)
    dt = 0.01f;

#ifdef USE_ANGLE_SMOOTHING
  static float smoothedRoll = 0;
  static float smoothedPitch = 0;

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
