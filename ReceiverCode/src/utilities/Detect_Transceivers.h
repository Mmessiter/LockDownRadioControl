#ifndef _SRC_DETECT_TRANSCEIVERS_H
#define _SRC_DETECT_TRANSCEIVERS_H
#include <Arduino.h>
#include "utilities/1Definitions.h"
#include <SPI.h>
#include <RF24.h>
//************************************************************************************************************/
void DetectTransceivers()
{
    RF24 tempRadio1(9, 10);
    RF24 tempRadio2(22, 23);
    RF24 tempRadio3(21, 20);
    bool RadioAt_9_10 = false;
    bool RadioAt_22_23 = false;
    bool RadioAt_21_20 = false;
    SPI.begin();
    delay(100); // let SPI settle
    tempRadio1.begin();
    tempRadio2.begin();
    tempRadio3.begin();
    delay(5); // let radios settle
    if (tempRadio1.isChipConnected())
        RadioAt_9_10 = true;
    if (tempRadio2.isChipConnected())
        RadioAt_22_23 = true;
    if (tempRadio3.isChipConnected())
        RadioAt_21_20 = true;

    if (RadioAt_9_10) // old rxs with only 8 pwm outputs
    {
        // Look1("Only 8 PWMs and ");
        Use_eleven_PWM_Outputs = false;
        Servos_Used = 9;
        V_Pin_Ce1 = PINA_CE1;   // 9
        V_Pin_Csn1 = PINA_CSN1; // 10
        Receiver_Type = 2;      // Old type, possibly 2 transceivers ... (but might be type 1 if no 2nd transceiver ... see below)
    }
    if (RadioAt_22_23) // all rxs with 2nd transceiver
    {
        // Look1("All 11 PWMs and ");
        Use_eleven_PWM_Outputs = true;
        Servos_Used = 11;
        V_Pin_Ce1 = PIN_CE1;   // 22
        V_Pin_Csn1 = PIN_CSN1; // 23
        Receiver_Type = 3;     // New dual transceiver type with All 11 PWMs
    }
    
    if (RadioAt_21_20)
    {
        // Look("2 transceivers.");
        Use_Second_Transceiver = true;
        V_Pin_Ce2 = PIN_CE2;   // 21
        V_Pin_Csn2 = PIN_CSN2; // 20
    }
    else
    {
        // Look("only 1 transceiver.");
        Use_Second_Transceiver = false;
        Use_eleven_PWM_Outputs = false;
        Servos_Used = 9;
        V_Pin_Ce1 = PINA_CE1;   // 9
        V_Pin_Csn1 = PINA_CSN1; // 10
        Receiver_Type = 1; // single transceiver type
    }
}
#endif // _SRC_DETECT_TRANSCEIVERS_H