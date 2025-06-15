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

// Q_angle: Trust in the system model (gyro integration)
// - Higher value = more responsive to fast attitude changes (less lag), but may introduce wobble from noise
// - Lower value = smoother output, but slower to respond to real motion
// - If the aircraft feels slow to respond or delayed after you move it, try increasing this
float Q_angle = 0.0003f;
// float Q_angle = 1.5f;
// Q_bias: How quickly we allow the filter to believe that the gyro's zero has drifted
// - Higher value = allows bias to change quickly (useful if the gyro drifts a lot)
// - Lower value = assumes gyro bias is stable
// - Rarely needs adjustment unless your gyro warms up and drifts — in that case, increase slightly
float Q_bias = 0.002f;

// R_measure: Trust in the accelerometer
// - Higher value = assumes the accelerometer is noisy → more filtering, smoother output
// - Lower value = assumes accelerometer is clean → more responsive, but more susceptible to vibration
// - If you see wobbles or flutter from vibration, increase this
// - If you want snappier correction and your model is smooth, decrease this a little
float R_measure = 0.07f;

// USE_ANGLE_SMOOTHING: Pre-smooth accel-based roll and pitch angles before feeding them into Kalman
// - Helps reduce high-frequency noise in noisy sensors or vibrating airframes
// - If disabled, the Kalman sees more raw "jumps" in angle from accel
// - Leave enabled for helicopters, foamies, and anything not butter-smooth!
#define USE_ANGLE_SMOOTHING
// ===================================================================================
// Kalman Filter State Initialisation
// ===================================================================================
void initKalman()
{
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
float updateKalman(KalmanState &state, float newAngle, float newRate, float dt)
{
    // Predict step
    state.angle += dt * (newRate - state.bias);

    state.P[0][0] += dt * (dt * state.P[1][1] - state.P[0][1] - state.P[1][0] + Q_angle);
    state.P[0][1] -= dt * state.P[1][1];
    state.P[1][0] -= dt * state.P[1][1];
    state.P[1][1] += Q_bias * dt;

    // Update step
    float S = state.P[0][0] + R_measure;
    float K[2];
    K[0] = state.P[0][0] / S;
    K[1] = state.P[1][0] / S;

    float y = newAngle - state.angle;
    state.angle += K[0] * y;
    state.bias += K[1] * y;

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
    float dt = (currentTime - previousTime) / 1e6f; // convert to seconds
    previousTime = currentTime;

    if (dt <= 0 || dt > 0.05f)
        dt = 0.01f; // Clamp dt to avoid large jumps

#ifdef USE_ANGLE_SMOOTHING
    static float smoothedRollAngle = 0;
    static float smoothedPitchAngle = 0;
    const float accelAlpha = 0.05f; // More smoothing now

    smoothedRollAngle = (1 - accelAlpha) * smoothedRollAngle + accelAlpha * RawRollAngle;
    smoothedPitchAngle = (1 - accelAlpha) * smoothedPitchAngle + accelAlpha * RawPitchAngle;

    filteredRoll = updateKalman(kalmanRoll, smoothedRollAngle, RawRollRate, dt);
    filteredPitch = updateKalman(kalmanPitch, smoothedPitchAngle, RawPitchRate, dt);
#else
    filteredRoll = updateKalman(kalmanRoll, RawRollAngle, RawRollRate, dt);
    filteredPitch = updateKalman(kalmanPitch, RawPitchAngle, RawPitchRate, dt);
#endif

    // Yaw: no external reference, just integrate
    filteredYaw = updateKalman(kalmanYaw, kalmanYaw.angle, RawYawRate, dt);

    // Corrected gyro rates
    filteredRollRate = RawRollRate - kalmanRoll.bias;
    filteredPitchRate = RawPitchRate - kalmanPitch.bias;
    filteredYawRate = RawYawRate - kalmanYaw.bias;
}

// ===================================================================================
// Additional Low-Pass Filtering for Helicopter Control Rates
// ===================================================================================
void filterRatesForHelicopter()
{
    static float lastFilteredRollRate = 0;
    static float lastFilteredPitchRate = 0;
    static float lastFilteredYawRate = 0;

    const float beta = 0.2f; // good balance for helicopter control

    filteredRollRate = (1 - beta) * lastFilteredRollRate + beta * (RawRollRate - kalmanRoll.bias);
    filteredPitchRate = (1 - beta) * lastFilteredPitchRate + beta * (RawPitchRate - kalmanPitch.bias);
    filteredYawRate = (1 - beta) * lastFilteredYawRate + beta * (RawYawRate - kalmanYaw.bias);

    lastFilteredRollRate = filteredRollRate;
    lastFilteredPitchRate = filteredPitchRate;
    lastFilteredYawRate = filteredYawRate;
}

#endif // KALMAN_H
#endif // USE_STABILISATION