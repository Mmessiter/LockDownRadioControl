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

float Xaccel = 0;
float Yaccel = 0;
float Zaccel = 0;

float XAngularaccel = 0;
float YAngularaccel = 0;
float ZAngularaccel = 0;

float xPos = 0;
float yPos = 0;
float zPos = 0;

float MPU6050Temp = 0;

void ReadMPU6050(){

        mpu6050.update();

        Xaccel = mpu6050.getAccX();               //   Accelerations
        Yaccel = mpu6050.getAccY();               //   Accelerations
        Zaccel = mpu6050.getAccZ();               //   Accelerations

        XAngularaccel = mpu6050.getGyroX();       //   Gyro Angular accelerations
        YAngularaccel = mpu6050.getGyroY();       //   Gyro Angular accelerations
        ZAngularaccel = mpu6050.getGyroZ();       //   Gyro Angular accelerations

        xPos   = mpu6050.getAngleX();             //  Angular positions computed by integration of the angular accelerations
        yPos   = mpu6050.getAngleY();             //  Angular positions computed by integration of the angular accelerations
        zPos   = mpu6050.getAngleZ();             //  Angular positions computed by integration of the angular accelerations

        MPU6050Temp = mpu6050.getTemp(); //  Temperature in degrees C

        // Look1("Xaccel: ");
        // Look(Xaccel);
        // Look1("Yaccel: ");
        // Look(Yaccel);
        // Look1("Zaccel: ");
        // Look(Zaccel);

        // Look1("XAngularaccel: ");
        // Look(XAngularaccel);
        // Look1("YAngularaccel: ");
        // Look(YAngularaccel);
        // Look1("ZAngularaccel: ");
        // Look(ZAngularaccel);

      //   Look1("xPos: ");
      //   Look(xPos);
        // Look1("yPos: ");
        // Look(yPos);
        // Look1("zPos: ");
        // Look(zPos);

        // Look1("MPU6050Temp: ");
        // Look(MPU6050Temp);
        

  }
/************************************************************************************************************/

void PIDEntryPoint(){

  //   static uint32_t LastTime = 0;
  //   static uint32_t lcount = 0;
  
  //   if (millis()-LastTime >= 1000){
  //       LastTime = millis();
  //       Serial.print("Loop speed: ");
  //       Serial.print(lcount);
  //       Serial.print(" Hz  ");
  //       Serial.println(millis());
  //       lcount = 0;
  //  }
  //  ++lcount;
   
   ReadMPU6050();

}

/************************************************************************************************************/

void DoStabilsation(){  // This is called from the main loop and from all DelayMillis() loops
    
    if (!MPU6050Connected) {
      //  Look("No MPU6050 connected");
        return;         
    }
    static uint32_t LastTime = 0;
    if (millis()-LastTime >= 5){ // 5000 = 200 Hz
        LastTime = millis();
        PIDEntryPoint();            // here we can call a timed stabilisation event at exactly 250 Hz
   }
}

#else

//void DoStabilsation(){ // dummy function for no stabilisation
//    return;
//}

#endif // DOSTABILISATION

#endif // _SRC_PID_H

// End of file: src/utilities/pid.h

