/** @file ReceiverCode/src/utilities/common.h */

//**************************************************************************************************************************
   //     #define SECOND_TRANSCEIVER // >>>>>>>>>>>>>>>> ******* DON'T FORGET TO SET THIS ONE !!! ******* <<<<<<<<<<<<<<<<<<<<<<<<< **
//**************************************************************************************************************************

    #ifndef _SRC_UTILITIES_COMMON_H
    #define _SRC_UTILITIES_COMMON_H
    #include <Adafruit_INA219.h> 
    #include <stdint.h>
    #include <EEPROM.h>

    #define RXVERSION_MAJOR            1
    #define RXVERSION_MINOR            5
    #define RXVERSION_MINIMUS          3               // February 3rd 2022
    
    #define SENSOR_HUB_I2C_ADDRESS     8


    #define HOPTIME         30 // ms between channel changes (= 4 packets per hop)
    #define FREQUENCYSCOUNT 82 // use 82 different channels

 //  #define DEBUG         // for FHSS
// #define DB_SENSORS
// #define DB_BIND
// #define DB_FAILSAFE

uint32_t  LastPacketArrivalTime = 0;
bool FailSafeSave         = false;
bool FailSafeSent         = false;
bool INA219_CONNECTED     = false;                   //  Volts from INA219 ?

#define RECONNECT_CH 83
uint8_t FHSS_Channels[84] = {28, 24, 61, 64, 28, 55, 66, 19, 76, 21, 59, 67, 15, 71, 82, 32, 49, 69, 13, 2, 34, 47, 20, 34, 69, 
                             16, 2, 72, 35, 76, 35, 57, 45, 29, 76, 75, 49, 59, 3, 57, 20, 16, 41, 59, 62, 59, 67, 11, 3, 9, 77, 
                             37, 8, 31, 36, 34, 18, 75, 17, 9, 50, 78, 77, 73, 30, 50, 79, 6, 36, 20, 23, 79, 40, 54, 51, 19, 69, 
                             12, 18, 80, 53, 41, 24};

/************************************************************************************************************/

#endif // defined (_SRC_UTILITIES_COMMON_H)
