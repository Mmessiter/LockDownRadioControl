// RC Helicopter Throttle PID Controller with Feed-Forward Compensation
// For Teensy 4.0 controlling a 700-size helicopter with Scorpion Tribunus ESC

#include <Arduino.h>
#include "utilities/1Definitions.h"

// PID Controller State Structure
struct PIDState
{
    float integral = 0.0f;
    float lastError = 0.0f;
    float lastDerivative = 0.0f;
    unsigned long lastTime = 0;
    bool initialized = false;
};

// Global PID state (declare this globally in your main code)
PIDState throttlePIDState;

/**
 * PID Controller for RC Helicopter Throttle/Governor
 *
 * @param currentRPM        Current rotor head RPM (measured)
 * @param desiredRPM        Target rotor head RPM
 * @param currentThrottle   Current throttle channel value (1000-2000 μs PWM)
 * @param collectivePitch   Current collective pitch value (-100 to +100, 0 = neutral)
 * @param Kp                Proportional gain
 * @param Ki                Integral gain
 * @param Kd                Derivative gain
 * @param ffGain            Feed-forward gain for collective compensation (0.0-2.0 typical)
 * @param pidState          Reference to PID state structure
 *
 * @return New throttle channel value (1000-2000 μs PWM)
 */
int calculateThrottlePID(
    float currentRPM,
    float desiredRPM,
    int currentThrottle,
    float collectivePitch,
    float Kp,
    float Ki,
    float Kd,
    float ffGain,
    PIDState &pidState)
{
    // Constants
    const float THROTTLE_MAX = 2000.0f;   // Maximum throttle PWM (μs)
    const float THROTTLE_IDLE = 1100.0f;  // Idle/minimum throttle (adjust for your ESC)
    const float INTEGRAL_LIMIT = 200.0f;  // Anti-windup limit
    const float DERIVATIVE_FILTER = 0.7f; // Low-pass filter coefficient (0-1)
   // const float FF_PITCH_SCALE = 2.0f;    // Scale factor for collective pitch influence
    const float RPM_DEADBAND = 20.0f;     // RPM deadband to prevent hunting

    // Get current time
    unsigned long currentTime = micros();

    // Initialize on first run
    if (!pidState.initialized)
    {
        pidState.lastTime = currentTime;
        pidState.lastError = 0;
        pidState.integral = 0;
        pidState.lastDerivative = 0;
        pidState.initialized = true;
        return currentThrottle;
    }

    // Calculate time delta in seconds
    float dt = (currentTime - pidState.lastTime) / 1000000.0f;

    // Prevent division by zero and handle timer overflow
    if (dt <= 0 || dt > 1.0f)
    {
        pidState.lastTime = currentTime;
        return currentThrottle;
    }

    // Calculate RPM error
    float error = desiredRPM - currentRPM;

    // Apply deadband to reduce hunting
    if (abs(error) < RPM_DEADBAND)
    {
        error = 0;
    }

    // --- PROPORTIONAL TERM ---
    float pTerm = Kp * error;

    // --- INTEGRAL TERM ---
    pidState.integral += error * dt;

    // Anti-windup: Limit integral accumulation
    pidState.integral = constrain(pidState.integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    // If we're at throttle limits and error would increase integral, stop integrating
    if ((currentThrottle >= THROTTLE_MAX - 10 && error > 0) ||
        (currentThrottle <= THROTTLE_IDLE + 10 && error < 0))
    {
        pidState.integral -= error * dt; // Remove the integration we just did
    }

    float iTerm = Ki * pidState.integral;

    // --- DERIVATIVE TERM ---
    float derivative = 0;
    if (dt > 0)
    {
        derivative = (error - pidState.lastError) / dt;
    }

    // Apply low-pass filter to derivative to reduce noise
    derivative = (DERIVATIVE_FILTER * pidState.lastDerivative) +
                 ((1.0f - DERIVATIVE_FILTER) * derivative);
    pidState.lastDerivative = derivative;

    float dTerm = Kd * derivative;

    // --- FEED-FORWARD COMPENSATION ---
    // Anticipate load changes from collective pitch
    // Positive collective = more load = need more throttle
    // This helps prevent RPM droop during collective inputs
    float collectiveCompensation = 0;

    // Calculate collective pitch change rate (approximation)
    static float lastCollective = 0;
    float collectiveRate = (collectivePitch - lastCollective) / dt;
    lastCollective = collectivePitch;

    // Feed-forward based on absolute collective position
    // More positive or negative collective = more load
    //collectiveCompensation = ffGain * abs(collectivePitch) * FF_PITCH_SCALE;   // NO!!

    // Additional compensation for rapid collective changes
    if (collectiveRate > 10.0f)
    { // Rapid positive collective change
        collectiveCompensation = ffGain * collectiveRate * 0.5f;
    }

    // --- CALCULATE TOTAL OUTPUT ---
    float pidOutput = pTerm + iTerm + dTerm + collectiveCompensation;

    // Apply output to current throttle
    float newThrottle = currentThrottle + pidOutput;

    // Constrain throttle to valid range
    newThrottle = constrain(newThrottle, THROTTLE_IDLE, THROTTLE_MAX);

    // Soft-start protection: Limit rate of throttle change
    const float MAX_THROTTLE_CHANGE = 50.0f; // Max change per update (μs)
    float throttleChange = newThrottle - currentThrottle;
    if (abs(throttleChange) > MAX_THROTTLE_CHANGE)
    {
        newThrottle = currentThrottle + (throttleChange > 0 ? MAX_THROTTLE_CHANGE : -MAX_THROTTLE_CHANGE);
    }

    // Update state for next iteration
    pidState.lastError = error;
    pidState.lastTime = currentTime;

    return (int)newThrottle;
}

// --- RECOMMENDED PID VALUES FOR 700-SIZE HELICOPTER ---
// These are starting points - fine-tune based on your specific setup

struct PIDTuningValues
{
    float Kp;     // Proportional gain
    float Ki;     // Integral gain
    float Kd;     // Derivative gain
    float ffGain; // Feed-forward gain
    const char *description;
};

bool GovernorEnabled = false;       // Governor on/off switch
uint8_t ThrottleChannel = 5;        // This get a '-1' later :-)
uint8_t CollectivePitchChannel = 3; // Collective pitch channel (1-indexed too ...)
uint16_t GovernedThrottle = 0;      // Governed throttle value
float desiredRPM = 1300.0f;         // Desired rotor RPM
float ffGain = 0.0f;                // Feed-forward gain (0.0-2.0 typical)
float Kp = 0.004;                   // was 0.45 Proportional gain
float Ki = 0.0009;                  // was 0.22 Integral gain
float Kd = 0.08f;                   // Derivative gain

// **************************************************************************************************************************
// Use governor to modify throttle
void UpdateGovernor()
{
    static bool firstRun = true;

    if (!GovernorEnabled || RotorRPM < 1000)
    {
        // Reset PID when governor is disabled
        throttlePIDState.integral = 0;
        throttlePIDState.initialized = false;
        GovernedThrottle = 0;
        firstRun = true; // Reset firstRun flag when disabled
        return;
    }

    if (firstRun)
    {
        throttlePIDState.initialized = false;
        throttlePIDState.integral = 0;
        throttlePIDState.lastError = 0;
        throttlePIDState.lastDerivative = 0;
        throttlePIDState.lastTime = micros();
        firstRun = false;
    }

    int newThrottle = calculateThrottlePID(
        RotorRPM,
        desiredRPM,
        ReceivedData[ThrottleChannel - 1],
        map(ReceivedData[CollectivePitchChannel - 1], 1000, 2000, -100, 100),
        Kp, Ki, Kd, ffGain,
        throttlePIDState);
    GovernedThrottle = constrain(newThrottle, 1100, 1950);
}

// **************************************************************************************************************************

// --- TUNING GUIDE ---
/*
 * PID Tuning Procedure for RC Helicopter Governor:
 *
 * 1. Start with all gains at 0
 * 2. Increase Kp until the system responds to RPM changes (start with 0.3)
 *    - If oscillations occur, reduce Kp by 20-30%
 * 3. Add Ki to eliminate steady-state error (start with Kp * 0.5)
 *    - If RPM hunts or is unstable, reduce Ki
 * 4. Add small amount of Kd to reduce overshoot (start with Kp * 0.15)
 *    - Too much Kd causes noise and vibration
 * 5. Tune ffGain (feed-forward) to prevent RPM droop during collective changes
 *    - Start with 0.8, increase if RPM drops during collective inputs
 *    - Typical range: 0.5-1.5
 *
 * Symptoms and Solutions:
 * - Slow response: Increase Kp
 * - Oscillation: Decrease Kp and/or Kd
 * - RPM droop under load: Increase Ki and/or ffGain
 * - Overshoot: Increase Kd (carefully)
 * - Hunting at steady state: Decrease Ki, increase deadband
 *
 * Safety Notes:
 * - Always bench test first without main blades
 * - Use throttle hold for emergency stops
 * - Monitor ESC and motor temperatures
 * - Ensure proper RPM sensor signal quality
 */