/** @file ReceiverCode/src/utilities/common.h */
// Malcolm Messiter 2022
    #ifndef _SRC_UTILITIES_COMMON_H
    #define _SRC_UTILITIES_COMMON_H
    
    #include <SPI.h>
    #include <RF24.h>
    #include <Adafruit_INA219.h> 
    #include <stdint.h>
    #include <EEPROM.h>
    #include <Wire.h>
    #include <Servo.h> 
    #include <SBUS.h>
    #include "utilities/radio.h"
    
    #define RXVERSION_MAJOR             1
    #define RXVERSION_MINOR             8
    #define RXVERSION_MINIMUS           1   // July 3rd 2022

//**************************************************************************************************************************
    #define SECOND_TRANSCEIVER // >>>>>>>>>>>>>>>> ******* DON'T FORGET TO SET THIS ONE !!! ******* <<<<<<<<<<<<<<<<<<<<< **
//**************************************************************************************************************************

    #define SENSOR_HUB_I2C_ADDRESS      8
    
 
    #define HOPTIME                     97   // >= ms between channel changes (10 packets per hop)
    
    #define LISTEN_PERIOD               10   // 10 How many ms to listen for TX in Reconnect()
    #define RECEIVE_TIMEOUT             11   // <=9 fails, >=11 OK ... Reduced here

    #define FREQUENCYSCOUNT             82   // use 82 different channels
    #define FREQUENCYSCOUNT1            41   // use 41 different test channels
   
    #define CHANNELSUSED                16   
    #define SERVOSUSED                  9    // But all 16 are available via SBUS
    #define SBUSRATE                    10   // SBUS frame every 10 milliseconds
    #define SBUSPORT                    Serial3
    #define RECONNECTGAP                25   // Send no data to servos for 25 ms after a reconnect (10 was not quite enough)
    #define MINMICROS                   500
    #define MAXMICROS                   2500
    #define LED_PIN                     LED_BUILTIN
    #define RANGEMAX                    2047   // = Frsky at 150 %
    #define RANGEMIN                    0
    #define pinCE1                      9    // NRF1
    #define pinCSN1                     10   // NRF1
    #define pinCSN2                     20   // NRF2
    #define pinCE2                      21   // NRF2
    #define FAILSAFE_TIMEOUT            2000 // two seconds until failsafe
    #define RECONNECT_CHANNELS_COUNT    3    // How many channels to try when reconnecting
    #define RECONNECT_CHANNELS_START    12   // Offset in the array to begin getting reconnect channels
    #define UNCOMPRESSEDWORDS           20              //   16 Channels plus extra 4 16 BIT values
    #define COMPRESSEDWORDS   UNCOMPRESSEDWORDS * 3 / 4 // = 16 WORDS  with no extra
    #define CSN_ON  LOW
    #define CSN_OFF HIGH
    #define CE_ON   HIGH
    #define CE_OFF  LOW


    // #define DB_FHSS        
    // #define DB_SENSORS
    // #define DB_BIND
    // #define DB_FAILSAFE
    // #define DB_RXTIMERS

uint32_t  LastPacketArrivalTime = 0;
bool FailSafeSave               = false;
bool INA219_CONNECTED           = false;                   //  Volts from INA219 ?

uint8_t * FHSSChPointer;                                   // Pointer for FHSS channels' array
uint8_t FrequencyCount = FREQUENCYSCOUNT;
uint8_t FHSS_Channels[83] = {51,28,24,61,64,55,66,19,76,21,59,67,15,71,82,32,49,69,13,2,34,47,20,16,72,    // These are good for UK
35,57,45,29,75,3,41,62,11,9,77,37,8,31,36,18,17,50,78,73,30,79,6,23,40,
54,12,80,53,22,1,74,39,58,63,70,52,42,25,43,26,14,38,48,68,33,27,60,44,46,
56,7,81,5,65,4,10};

uint8_t FHSS_Channels1[42] = {93,111,107,103,106,97,108,102,118,104,101,109,98,           // These are good for tests where WiFi is too overpowering
113,124,115,91,96,85,117,89,99,114,87,112,
86,94,92,119,120,100,121,123,95,122,105,84,116,90,110,88};

/************************************************************************************************************/

#endif // defined (_SRC_UTILITIES_COMMON_H)
