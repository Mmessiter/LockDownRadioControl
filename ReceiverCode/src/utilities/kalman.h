// *** This file contains Kalman filter for the MPU6050 data ****************************************************************************************

#include "utilities/common.h"
#ifdef USE_STABILISATION
#ifndef KALMAN_H
#define KALMAN_H



// ======================================================
// KALMAN FILTER IMPLEMENTATION
// ======================================================
// Kalman filter state variables for Roll, Pitch, Yaw
/*

## Q_angle (Process Noise for the Angle)
This parameter represents the uncertainty in how the angle changes between measurements. It models the "process noise" in your angle state.

- **Lower values** (e.g., 0.001): Assumes the angle changes very smoothly and predictably. This results in smoother output but slower response to actual changes. The filter will be more resistant to noise but might miss rapid movements.
- **Higher values** (e.g., 0.1): Assumes larger unpredictable changes in angle are possible. This makes the filter more responsive to new measurements but potentially more susceptible to noise.
- **Use case impact**: In applications like drone stabilization, too low a value might cause sluggish correction to wind gusts, while too high might make the drone jittery.

## Q_bias (Process Noise for the Bias)
This models the uncertainty in how the gyroscope bias drifts over time.

- **Lower values**: Assumes the bias is very stable and changes very slowly. This works well with high-quality sensors that have minimal drift.
- **Higher values**: Assumes the bias might change more significantly between measurements. This is more appropriate for lower-quality sensors or environments with temperature fluctuations that affect bias.
- **Use case impact**: In a VR headset, properly tuned Q_bias helps maintain accurate orientation tracking during extended use despite sensor warming.

## R_measure (Measurement Noise)
This represents the uncertainty or noise in your measurements (typically from an accelerometer).

- **Lower values**: Indicates high trust in the raw sensor measurements. The filter will quickly adjust based on new measurements.
- **Higher values**: Indicates less trust in raw measurements. The filter will rely more on its internal model and predictions.
- **Use case impact**: In a robot navigating rough terrain, high R_measure might be appropriate when accelerometer readings are affected by vibrations and impacts.

These parameters work together to create an optimal balance. For example, in an IMU-based orientation estimation:
- Too low Q_angle + low R_measure: Will follow noisy measurements too closely
- Too high Q_angle + high R_measure: Will be unresponsive to actual changes in orientation
- Balanced values: Will filter out noise while maintaining responsiveness to real movements

The art of Kalman filter tuning is finding the right combination for your specific application and sensor characteristics.

*/
// For quadcopters, start with:
// const float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;

// For helicopters, try:
// const float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.05;

// For fixed-wing aircraft, try:
const float Q_angle = 0.0005, Q_bias = 0.003, R_measure = 0.01;

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

#endif // KALMAN_H
#endif // USE_STABILISATION