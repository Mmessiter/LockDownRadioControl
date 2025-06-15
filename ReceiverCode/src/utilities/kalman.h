// *** This file contains Kalman filter for the MPU6050 data ****************************************************************************************

#include "utilities/1Definitions.h"
#ifdef USE_STABILISATION
#ifndef KALMAN_H
#define KALMAN_H

// ======================================================
// KALMAN FILTER IMPLEMENTATION
// ======================================================
// TUNING PARAMETERS
/*
  ===============================================
  KALMAN FILTER — QUICK MISBEHAVIOUR TUNING GUIDE
  ===============================================

  Symptoms & Suggested Fixes:

  ▸ Feels sluggish, slow to respond
      → Increase Q_angle slightly

  ▸ Filter is shaky or jittery during vibration
      → Increase R_measure

  ▸ Filter reacts too sharply or is twitchy
      → Decrease Q_angle, or increase R_measure

  ▸ Filter drifts slowly over time
      → Increase Q_bias

  ▸ Output looks clean but servos flutter or overreact
      → Enable or increase USE_ANGLE_SMOOTHING
      → Increase R_measure

  ▸ Filter becomes unstable or erratic during flight
      → Ensure dt is clamped to safe range
      → Lower Q_angle and increase R_measure

  NOTES:
  - Q_angle governs how quickly the filter responds to changes (responsiveness vs. smoothness)
  - Q_bias affects long-term gyro stability (drift correction)
  - R_measure controls how much trust we place in the accelerometer (noise rejection)
*/

float Q_angle = 0.01f;
float Q_bias = 0.003f;
float R_measure = 0.03f;
#define USE_ANGLE_SMOOTHING

// ===================================================================================
// Kalman Filter State Initialisation
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
    const float beta = 0.2f;
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

#endif // KALMAN_H
#endif // USE_STABILISATION