// *************************************** Calibrate.h *****************************************

// This is the code for calibrating the sticks

#include <Arduino.h>
#include "1Definitions.h"

#ifndef CALIBRATE_H
#define CALIBRATE_H

/*********************************************************************************************************************************/

void ReduceLimits()
{ // Get things setup for sticks calibration
    for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        ChannelMax[i] = MAXRESOLUTION / 2;
        ChannelMin[i] = MAXRESOLUTION / 2;
    }
    for (uint8_t i = 0; i < CHANNELSUSED; ++i)
    {
        MaxDegrees[Bank][i] = 180;
        CentreDegrees[Bank][i] = 90;
        MinDegrees[Bank][i] = 0;
    }
}
/*********************************************************************************************************************************/
void CalibrateSticks() // This discovers end of travel place for sticks etc.
{
    uint16_t p;
    for (uint8_t i = 0; i < PROPOCHANNELS; ++i)
    {
        p = adc->analogRead(AnalogueInput[i]);
        if (ChannelMax[i] < p)
            ChannelMax[i] = p;
        if (ChannelMin[i] > p)
            ChannelMin[i] = p;
    }
    NewCompressNeeded = false; // fake it as we are not sending data
    GetAllInputs();
    DualRateValue = 100;
    CalculateAllOutputs();
    ShowServoPos();
}

/*********************************************************************************************************************************/
/** @brief Get centre as 90 degrees */
void ChannelCentres()
{
    for (int i = 0; i < PROPOCHANNELS; ++i)
    {
        ChannelCentre[i] = AnalogueReed(i);
        ChannelMidHi[i] = ChannelCentre[i] + ((ChannelMax[i] - ChannelCentre[i]) / 2);
        ChannelMidLow[i] = ChannelMin[i] + ((ChannelCentre[i] - ChannelMin[i]) / 2);
    }
    for (int i = PROPOCHANNELS; i < CHANNELSUSED; ++i)
    {
        ChannelMin[i] = 500;
        ChannelCentre[i] = 1500;
        ChannelMax[i] = 2500;
    }
    NewCompressNeeded = false; // fake it as we are not sending data
    GetNewChannelValues();
    ShowServoPos();
    CalibrateEdgeSwitches(); // These are now calibrated too in case some are reversed.
}

/*********************************************************************************************************************************/
/** @brief STICKS CALIBRATION */
FLASHMEM void InitMaxMin()
{
    for (int i = 0; i < CHANNELSUSED; ++i)
    {
        ChannelMax[i] = MAXRESOLUTION;
        ChannelMidHi[i] = MAXRESOLUTION * 3 / 4;
        ChannelCentre[i] = MAXRESOLUTION / 2;
        ChannelMidLow[i] = MAXRESOLUTION / 4;
        ChannelMin[i] = 0;
    }
}
#endif