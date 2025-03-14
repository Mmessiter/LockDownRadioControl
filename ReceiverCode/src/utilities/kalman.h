// *** This file contains Kalman filter for the MPU6050 data ****************************************************************************************

#include "utilities/common.h"
#ifdef USE_STABILISATION
#ifndef KALMAN_H
#define KALMAN_H

// ======================================================
// KALMAN FILTER IMPLEMENTATION
// ======================================================
// Kalman filter state variables for Roll, Pitch, Yaw

// Kalman filter tuning parameters
const float Q_angle = 0.001;  // Process noise for the angle (smaller = smoother but slower response)
const float Q_bias = 0.003;   // Process noise for the bias estimate
const float R_measure = 0.03; // Measurement noise (larger = less trust in accelerometer)

// Initialize Kalman filters
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
// ********************************************************************************************************************
float updateKalman(KalmanState &state, float newAngle, float newRate, float dt)
{
    state.angle += dt * (newRate - state.bias);
    state.P[0][0] += dt * (dt * state.P[1][1] - state.P[0][1] - state.P[1][0] + Q_angle);
    state.P[0][1] -= dt * state.P[1][1];
    state.P[1][0] -= dt * state.P[1][1];
    state.P[1][1] += Q_bias * dt;
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
// ********************************************************************************************************************

// Main Kalman filter function to call from your loop
void kalmanFilter()
{
    unsigned long currentTime = micros();
    float dt = (currentTime - previousTime) / 1000000.0; // Convert to seconds
    previousTime = currentTime;
    if (dt <= 0 || dt > 0.2)
        dt = 0.01;
     filteredRoll = updateKalman(kalmanRoll, RawRollAngle, RawRollRate, dt);
     filteredPitch = updateKalman(kalmanPitch, RawPitchAngle, RawPitchRate, dt);
     filteredYaw = updateKalman(kalmanYaw, kalmanYaw.angle, RawYawRate, dt); // Note: Using previous estimate as no absolute yaw reference
     filteredRollRate = RawRollRate - kalmanRoll.bias;
     filteredPitchRate = RawPitchRate - kalmanPitch.bias;
     filteredYawRate = RawYawRate - kalmanYaw.bias;
}
// ********************************************************************************************************************

void filterRatesForHelicopter()
{

    // Low-pass filter coefficient (0-1): lower = more filtering
    // For helicopters, use 0.1-0.3 for smooth control without too much delay

    static float lastFilteredRollRate = 0;
    static float lastFilteredPitchRate = 0;
    static float lastFilteredYawRate = 0;
    float beta = 0.2;

    filteredRollRate = (1 - beta) * lastFilteredRollRate + beta * (RawRollRate - kalmanRoll.bias);
    filteredPitchRate = (1 - beta) * lastFilteredPitchRate + beta * (RawPitchRate - kalmanPitch.bias);
    filteredYawRate = (1 - beta) * lastFilteredYawRate + beta * (RawYawRate - kalmanYaw.bias);

    lastFilteredRollRate = filteredRollRate;
    lastFilteredPitchRate = filteredPitchRate;
    lastFilteredYawRate = filteredYawRate;
}

// ======================================================
// TUNING RECOMMENDATIONS FOR RC AIRCRAFT
// ======================================================
/*

Kalman Filter Tuning:
- Q_angle: Increase for more responsive attitude estimation (0.001-0.01)
- Q_bias: Adjust based on gyro drift characteristics (0.001-0.005)
- R_measure: Increase in high-vibration environments (0.03-0.1)

For quadcopters, start with:
Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03

For helicopters, try:
Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.05

For fixed-wing aircraft, try:
Q_angle = 0.0005, Q_bias = 0.003, R_measure = 0.01
*/

#endif // KALMAN_H
#endif // USE_STABILISATION