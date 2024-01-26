#ifndef _SRC_PID_H
#define _SRC_PID_H

#include <Arduino.h>
#include <MPU6050_tockn.h>
#include "utilities/common.h"


#ifdef DOSTABILISATION
 
// ************************************************************************************************************
// PID control
// Not used yet but will be needed for PID control




class KalmanFilter {
public:
    KalmanFilter() {
        Q = 0.001f;  // Process noise covariance
        R = 0.01f;   // Measurement noise covariance
        x = 0.0f;    // Initial state (estimate)
        P = 1.0f;    // Initial estimate uncertainty
    }

    float filter(float z) {
        // Prediction
        float x_pred = x;
        float P_pred = P + Q;

        // Update
        float K = P_pred / (P_pred + R);
        x = x_pred + K * (z - x_pred);
        P = (1 - K) * P_pred;

        return x;
    }

private:
    float Q;  // Process noise covariance
    float R;  // Measurement noise covariance
    float x;  // State (estimate)
    float P;  // Estimate uncertainty
};

KalmanFilter kalman;

// *****************************************************************************************************************
class LowLatencyFilter { 
  private: 
   
    float ReturnedValue;
    float FilterFactor = 0; 
    float UsedFilterFactor;

// *******************************************************************************
float Method2(float NewDataPoint){
     static float OldDataPoints[8];
     int count = 6;
     ReturnedValue = 0;
     OldDataPoints[count-1] = NewDataPoint;
     for (int i = 1; i < count; ++i){  
        ReturnedValue += OldDataPoints[i];
        OldDataPoints[i-1] = OldDataPoints[i];
      }
    return ReturnedValue / (count-1);
   }

  public:   
  // *****************************************************************************************************************
      void SetFilterFactor(float f){
      FilterFactor = f;
    }
  // *****************************************************************************************************************   
    float Filter(float NewDataPoint){
    
    ReturnedValue = Method2(NewDataPoint);
 
    return ReturnedValue;
  }
  // *****************************************************************************************************************   
};

LowLatencyFilter XF;
LowLatencyFilter YF;



// *****************************************************************************************************************
void ReadMPU6050(){

  }
/************************************************************************************************************/

void PIDEntryPoint(){
    static uint32_t LastTime = 0;
    static uint32_t lcount = 0;
  
    if (millis()-LastTime >= 1000){
        LastTime = millis();
        Serial.print("Loop speed: ");
        Serial.print(lcount);
        Serial.print(" Hz  ");
        Serial.println(millis());
        lcount = 0;
   }
   ++lcount;
   
   ReadMPU6050();

}

/************************************************************************************************************/

void DoStabilsation(){  // This is called from the main loop and from all DelayMillis() loops
    
    if (!MPU6050Connected) {
      //  Look("No MPU6050 connected");
        return;         
    }
    static uint32_t LastTime = 0;
    if (micros()-LastTime >= 3996){ // 4000 =250 Hz
        LastTime = micros();
        PIDEntryPoint();            // here we can call a timed stabilisation event at exactly 250 Hz
   }
}

#else

void DoStabilsation(){ // no stabilisation
    return;
}

#endif // DOSTABILISATION

#endif // _SRC_PID_H

// End of file: src/utilities/pid.h

