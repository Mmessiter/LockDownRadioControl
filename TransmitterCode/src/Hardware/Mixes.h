// *************************************** Mixes.h  *****************************************
#include <Arduino.h>
#include "Hardware/1Definitions.h"

#ifndef MIXES_H
    #define MIXES_H
    
/*********************************************************************************************************************************/
//   INPUT Mixes
/*********************************************************************************************************************************/
FASTRUN void MixInputs() 
{
    for (short MixNumber = 1; MixNumber < MAXMIXES; ++MixNumber)
    {
        if (Mixes[MixNumber][M_MIX_INPUTS])//
        {
            if (Mixes[MixNumber][M_Bank] == Bank || (!Mixes[MixNumber][M_Bank]))
            {
                for (short MasterChannel = 0; MasterChannel < CHANNELSUSED; ++MasterChannel)
                {
                    if ((Mixes[MixNumber][M_MasterChannel] - 1) == MasterChannel)
                    {
                        short SlaveChannel  =   (Mixes[MixNumber][M_SlaveChannel] - 1) ;
                        short max           =   ChannelMax[SlaveChannel]; 
                        short mid           =   ChannelCentre[SlaveChannel];
                        short min           =   ChannelMin[SlaveChannel];
                        short midl          =   mid - min;
                        short midh          =   max - mid;
                        short MappedInput;
                        short MixValue;
                        
                        if (InputsBuffer[MasterChannel] < ChannelCentre[MasterChannel]) {
                            MappedInput = map (InputsBuffer[MasterChannel], ChannelMin[MasterChannel], ChannelCentre[MasterChannel], ChannelMin[SlaveChannel], ChannelCentre[SlaveChannel]);
                        } else {
                            MappedInput = map (InputsBuffer[MasterChannel], ChannelCentre[MasterChannel], ChannelMax[MasterChannel], ChannelCentre[SlaveChannel], ChannelMax[SlaveChannel]);
                        }

                        if (MappedInput < mid) {
                            MixValue = map (MappedInput,min,mid,-midl,0) * (short)Mixes[MixNumber][M_Percent] / 100;
                        } else {
                            MixValue = map (MappedInput,mid,max,0,midh)  * (short)Mixes[MixNumber][M_Percent] / 100;
                        }
                      
                        if (Mixes[MixNumber][M_ONEDIRECTION])
                        {
                            if (Mixes[MixNumber][M_Reversed])
                            {
                                if (MixValue > 0) MixValue = -MixValue;
                            }
                            else
                            {
                                if (MixValue < 0) MixValue = -MixValue;
                            }
                        }
                        else
                        {
                            if (Mixes[MixNumber][M_Reversed]) MixValue = -MixValue;
                        }
                        MixValue += (Mixes[MixNumber][M_OFFSET] - 127) * 8;                 // add offset
                        MixValue += InputsBuffer[SlaveChannel] ;                            // This is the actual mix moment! (MixValue is now the mixed value
                        if (ChannelMin[SlaveChannel] > ChannelMax[SlaveChannel])
                        {
                           InputsBuffer[SlaveChannel] = constrain(MixValue, ChannelMax[SlaveChannel],  ChannelMin[SlaveChannel]);
                        }
                        else
                        {
                            InputsBuffer[SlaveChannel] = constrain(MixValue,  ChannelMin[SlaveChannel], ChannelMax[SlaveChannel]);
                        }
                    }
                }
            }
        }
    }
}

/*********************************************************************************************************************************/
//   OUTPUT Mixes
/*********************************************************************************************************************************/
FASTRUN void MixOutputs()
{
    for (short MixNumber = 1; MixNumber < MAXMIXES; ++MixNumber)
    {
        if (Mixes[MixNumber][M_MIX_OUTPUTS])
        {
            if (Mixes[MixNumber][M_Bank] == Bank || (!Mixes[MixNumber][M_Bank]))
            {
                for (short ChannelNumber = 0; ChannelNumber < CHANNELSUSED; ++ChannelNumber)
                {
                    if ((Mixes[MixNumber][M_MasterChannel] - 1) == ChannelNumber)
                    {
                        short MixValue = (map(PreMixBuffer[ChannelNumber], MINMICROS, MAXMICROS, -HALFMICROSRANGE, HALFMICROSRANGE)) * (short)Mixes[MixNumber][M_Percent] / 100;

                        if (Mixes[MixNumber][M_ONEDIRECTION])
                        {
                            if (Mixes[MixNumber][M_Reversed])
                            {
                                if (MixValue > 0) MixValue = -MixValue;
                            }
                            else
                            {
                                if (MixValue < 0) MixValue = -MixValue;
                            }
                        }
                        else
                        {
                            if (Mixes[MixNumber][M_Reversed]) MixValue = -MixValue;
                        }
                        MixValue += SendBuffer[(Mixes[MixNumber][M_SlaveChannel]) - 1]; // This is the actual mix moment! (MixValue is now the mixed value)
                        MixValue += (Mixes[MixNumber][M_OFFSET] - 127) * 8;
                        short MinimumDeg = IntoHigherRes(MinDegrees[Bank][(Mixes[MixNumber][M_SlaveChannel]) - 1]);
                        short MaximumDeg = IntoHigherRes(MaxDegrees[Bank][(Mixes[MixNumber][M_SlaveChannel]) - 1]);
                        if (MinimumDeg > MaximumDeg)
                        {
                            SendBuffer[(Mixes[MixNumber][M_SlaveChannel]) - 1] = constrain(MixValue, MaximumDeg, MinimumDeg);
                        }
                        else
                        {
                            SendBuffer[(Mixes[MixNumber][M_SlaveChannel]) - 1] = constrain(MixValue, MinimumDeg, MaximumDeg);
                        }
                    }
                }
            }
        }
    }
}

    
    #endif

    