#ifndef _SRC_PID_H
#define _SRC_PID_H

#include <Arduino.h>
#include <MPU6050_tockn.h>
#include "utilities/common.h"


#ifdef DOSTABILISATION
 
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

        Xaccel        = mpu6050.getAccX();                //   Accelerations
        Yaccel        = mpu6050.getAccY();                //   Accelerations
        Zaccel        = mpu6050.getAccZ();                //   Accelerations
        XAngularaccel = mpu6050.getGyroX();               //   Gyro Angular accelerations
        YAngularaccel = mpu6050.getGyroY();               //   Gyro Angular accelerations
        ZAngularaccel = mpu6050.getGyroZ();               //   Gyro Angular accelerations
        xPos          = mpu6050.getAngleX();              //   Angular positions computed by integration of the angular accelerations
        yPos          = mpu6050.getAngleY();              //   Angular positions computed by integration of the angular accelerations
        zPos          = mpu6050.getAngleZ();              //   Angular positions computed by integration of the angular accelerations

        MPU6050Temp = mpu6050.getTemp(); //  Temperature in degrees C

        // Look1("\tXaccel: ");
        // Look1(Xaccel);
        // Look1("\tYaccel: ");
        // Look1(Yaccel);
        // Look1("\tZaccel: ");
        // Look1(Zaccel);



        // Look1("\tXAngularaccel: ");
        // Look1(XAngularaccel);
        // Look1("\tYAngularaccel: ");
        // Look1(YAngularaccel);
        // Look1("\tZAngularaccel: ");
        // Look1(ZAngularaccel);

       
        Look1("xPos: ");
        Look1(xPos);
        Look1("\tyPos: ");
        Look(yPos);
       // Look1("\tzPos: ");
       // Look1(zPos);

        // Look1("MPU6050Temp: ");
        // Look(MPU6050Temp);
        



       //  Look("\t");

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
       
    static uint32_t LastTime = 0;
    if (millis()-LastTime >= 5){ // 5000 = 200 Hz
        LastTime = millis();
        PIDEntryPoint();            // here we can call a timed stabilisation event at exactly 250 Hz
   }
  // Look (millis());
}

#else

//void DoStabilsation(){ // dummy function for no stabilisation
//    return;
//}

#endif // DOSTABILISATION

#endif // _SRC_PID_H

// End of file: src/utilities/pid.h

