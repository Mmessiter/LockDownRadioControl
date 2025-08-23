// Helicopter RPM PID Controller for Teensy 4.0
// Controls throttle output based on RPM feedback
// Designed for 700-size helicopter with Scorpion Tribunus ESC

#include <Arduino.h>

class HeliRPM_PID
{
private:
    float kP, kI, kD;
    float integral;
    float previousError;
    unsigned long lastTime;
    float integralMax;
    bool firstRun;

    // Throttle output limits (microseconds for RC servo/ESC)
    static const int THROTTLE_MIN = 1000;
    static const int THROTTLE_MAX = 2000;

public:
    HeliRPM_PID(float p, float i, float d)
    {
        kP = p;
        kI = i;
        kD = d;
        integral = 0.0;
        previousError = 0.0;
        lastTime = 0;
        integralMax = 200.0; // Limit integral windup
        firstRun = true;
    }

    // Reset PID controller (call when switching modes or starting)
    void reset()
    {
        integral = 0.0;
        previousError = 0.0;
        firstRun = true;
    }

    // Set new PID gains
    void setGains(float p, float i, float d)
    {
        kP = p;
        kI = i;
        kD = d;
    }

    // Set integral windup limit
    void setIntegralLimit(float limit)
    {
        integralMax = limit;
    }
};

// Main PID control function
// Parameters:
//   currentRPM: Current measured RPM from sensor
//   desiredRPM: Target RPM setpoint
//   currentThrottle: Current throttle channel value (microseconds)
//   kP, kI, kD: PID gains
// Returns: New throttle value (microseconds, clamped to 1000-2000)
int helicopterRPM_PID(float currentRPM, float desiredRPM, int currentThrottle,
                      float kP, float kI, float kD)
{

    static float integral = 0.0;
    static float previousError = 0.0;
    static unsigned long lastTime = 0;
    static bool firstRun = true;

    // Get current time
    unsigned long currentTime = millis();

    // Calculate time delta (convert to seconds)
    float deltaTime;
    if (firstRun)
    {
        deltaTime = 0.05; // Assume 50ms for first run (20Hz)
        firstRun = false;
    }
    else
    {
        deltaTime = (currentTime - lastTime) / 1000.0;
        // Protect against division by zero or unrealistic time deltas
        if (deltaTime <= 0 || deltaTime > 0.2)
        {
            deltaTime = 0.05; // Default to 50ms
        }
    }

    // Calculate error (desired - actual)
    float error = desiredRPM - currentRPM;

    // Proportional term
    float pTerm = kP * error;

    // Integral term with windup protection
    integral += error * deltaTime;
    // Clamp integral to prevent windup
    float integralMax = 200.0; // Adjust based on system characteristics
    if (integral > integralMax)
        integral = integralMax;
    if (integral < -integralMax)
        integral = -integralMax;
    float iTerm = kI * integral;

    // Derivative term
    float derivative = (error - previousError) / deltaTime;
    float dTerm = kD * derivative;

    // Calculate PID output
    float pidOutput = pTerm + iTerm + dTerm;

    // Convert PID output to throttle adjustment
    // Scale the PID output appropriately for throttle range
    int throttleAdjustment = (int)pidOutput;
    int newThrottle = currentThrottle + throttleAdjustment;

    // Clamp throttle output to valid RC range
    if (newThrottle > 2000)
        newThrottle = 2000;
    if (newThrottle < 1000)
        newThrottle = 1000;

    // Reset integral if we're at throttle limits to prevent windup
    if ((newThrottle >= 2000 && pidOutput > 0) ||
        (newThrottle <= 1000 && pidOutput < 0))
    {
        integral *= 0.9; // Reduce integral gradually
    }

    // Store values for next iteration
    previousError = error;
    lastTime = currentTime;

    return newThrottle;
}

// Enhanced version with additional features
class AdvancedHeliPID
{
private:
    float kP, kI, kD;
    float integral;
    float previousError;
    unsigned long lastTime;
    float integralMax;
    bool firstRun;

    // Additional features
    float outputFilter; // Low-pass filter coefficient for output smoothing
    float lastOutput;

public:
    AdvancedHeliPID(float p, float i, float d, float integralLimit = 200.0)
    {
        kP = p;
        kI = i;
        kD = d;
        integral = 0.0;
        previousError = 0.0;
        integralMax = integralLimit;
        firstRun = true;
        outputFilter = 0.8; // Smooth output changes
        lastOutput = 1000;
        lastTime = 0;
    }

    int compute(float currentRPM, float desiredRPM, int currentThrottle)
    {
        unsigned long currentTime = millis();

        float deltaTime;
        if (firstRun)
        {
            deltaTime = 0.05;
            firstRun = false;
        }
        else
        {
            deltaTime = (currentTime - lastTime) / 1000.0;
            if (deltaTime <= 0 || deltaTime > 0.2)
                deltaTime = 0.05;
        }

        float error = desiredRPM - currentRPM;

        // PID calculations
        float pTerm = kP * error;

        integral += error * deltaTime;
        integral = constrain(integral, -integralMax, integralMax);
        float iTerm = kI * integral;

        float derivative = (error - previousError) / deltaTime;
        float dTerm = kD * derivative;

        float pidOutput = pTerm + iTerm + dTerm;
        int newThrottle = currentThrottle + (int)pidOutput;

        // Apply output filtering for smoother control
        newThrottle = (int)(outputFilter * lastOutput + (1.0 - outputFilter) * newThrottle);

        // Clamp and handle windup
        newThrottle = constrain(newThrottle, 1000, 2000);

        if ((newThrottle >= 2000 && pidOutput > 0) ||
            (newThrottle <= 1000 && pidOutput < 0))
        {
            integral *= 0.85;
        }

        previousError = error;
        lastTime = currentTime;
        lastOutput = newThrottle;

        return newThrottle;
    }

    void reset()
    {
        integral = 0.0;
        previousError = 0.0;
        firstRun = true;
    }
};

/* USAGE EXAMPLE:

// Create PID controller instance
AdvancedHeliPID rpmController(0.8, 0.05, 0.1);

void setup() {
    Serial.begin(115200);
    // Initialize your RPM sensor and ESC here
}

void loop() {
    // Read current RPM from sensor
    float currentRPM = readRPMSensor();

    // Set desired RPM (could be from pilot input or flight mode)
    float desiredRPM = 2000;

    // Get current throttle value
    int currentThrottle = getCurrentThrottle();

    // Compute new throttle value
    int newThrottle = rpmController.compute(currentRPM, desiredRPM, currentThrottle);

    // Output to ESC
    setThrottleOutput(newThrottle);

    delay(50); // Run at ~20Hz
}

*/