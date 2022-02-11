/** @file ReceiverCode/src/utilities/common.h */
    #ifndef _SRC_UTILITIES_COMMON_H
    #define _SRC_UTILITIES_COMMON_H
    
    #include <SPI.h>
    #include <RF24.h>
    #include <Adafruit_INA219.h> 
    #include <stdint.h>
    #include <EEPROM.h>
    #include <Wire.h>
    #include <Servo.h> 
    #include <EEPROM.h>
    #include <SBUS.h>
    #include "utilities/radio.h"

//**************************************************************************************************************************
    #define SECOND_TRANSCEIVER // >>>>>>>>>>>>>>>> ******* DON'T FORGET TO SET THIS ONE !!! ******* <<<<<<<<<<<<<<<<<<<<< **
//**************************************************************************************************************************


    #define RXVERSION_MAJOR             1
    #define RXVERSION_MINOR             5
    #define RXVERSION_MINIMUS           5    // February 11th 2022
    #define SENSOR_HUB_I2C_ADDRESS      8
    #define HOPTIME                     50   // ms between channel changes ( >5 packets per hop)
    #define LISTEN_PERIOD               15   // ms to listen for TX in Reconnect() before switching over to try the other transceiver
    #define FREQUENCYSCOUNT             82   // use 82 different channels
    #define RECEIVE_TIMEOUT             25   // 25 milliseconds seems an optimal value
    #define CHANNELSUSED                16
    #define SERVOSUSED                  9    // All 16 are available via SBUS
    #define SBUSRATE                    10   // SBUS frame every 10 milliseconds
    #define SBUSPORT                    Serial3
    #define RECONNECTGAP                20   // Send no data to servos for 20 ms after a reconnect (10 was not quite enough)
    #define MINMICROS                   500
    #define MAXMICROS                   2500
    #define LED_PIN                     LED_BUILTIN
    #define RANGEMAX                    2047 // = Frsky at 150 %
    #define RANGEMIN                    0
    #define pinCE1                      9    // NRF1
    #define pinCSN1                     10   // NRF1
    #define pinCSN2                     20   // NRF2
    #define pinCE2                      21   // NRF2
    #define FAILSAFE_TIMEOUT            2000 // two seconds until failsafe

    // #define DEBUG         // for FHSS
    // #define DB_SENSORS
    // #define DB_BIND
    // #define DB_FAILSAFE

uint32_t  LastPacketArrivalTime = 0;
bool FailSafeSave               = false;
bool FailSafeSent               = false;
bool INA219_CONNECTED           = false;                   //  Volts from INA219 ?

#define RECONNECT_CH 83
uint8_t FHSS_Channels[84] = {28, 24, 61, 64, 28, 55, 66, 19, 76, 21, 59, 67, 15, 71, 82, 32, 49, 69, 13, 2, 34, 47, 20, 34, 69, 
                             16, 2, 72, 35, 76, 35, 57, 45, 29, 76, 75, 49, 59, 3, 57, 20, 16, 41, 59, 62, 59, 67, 11, 3, 9, 77, 
                             37, 8, 31, 36, 34, 18, 75, 17, 9, 50, 78, 77, 73, 30, 50, 79, 6, 36, 20, 23, 79, 40, 54, 51, 19, 69, 
                             12, 18, 80, 53, 41, 24};

/************************************************************************************************************/

#endif // defined (_SRC_UTILITIES_COMMON_H)
