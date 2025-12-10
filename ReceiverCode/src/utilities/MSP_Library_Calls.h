
// ** MSP_Library_Calls.h **

#ifndef MSP_Library_Calls_H
#define MSP_Library_Calls_H
#include "utilities/1Definitions.h"
#include <Arduino.h>

/************************************************************************************************************/

#include "utilities/msp/ReefwingMSP.h"
#define MAX_CMD_SIZE 32
#define RETURN '\r'
#define NEW_LINE '\n'
#define NULL_CHAR '\0'
ReefwingMSP msp;


//************************************************************************************************************/

void InitMsp()
{
    msp.begin(MSP_UART); // timeout uses default 500 ms
}
// ************************************************************************************************************/
#endif // USE_MSP_LIBRARY
