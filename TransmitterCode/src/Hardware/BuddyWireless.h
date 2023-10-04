// *************************************** BuddyWireless.h  *****************************************
#include <Arduino.h>
#include "Hardware/1Definitions.h"
#include "Hardware/StructsEtc.h"
#ifndef BUDDYWIRELESS_H
    #define BUDDYWIRELESS_H
//*************************************************************************************************************************
void Setup_Solo()
{
    PaceMaker              = PACEMAKER;
    RetryCount             = RETRYCOUNT;
    RetryWait              = RETRYWAIT;
    LostContactCutOff      = LOSTCONTACTCUTOFF;
    ReconnectChannelsCount = RECONNECT_CHANNELS_COUNT;
    ReconnectChannelsStart = RECONNECT_CHANNELS_START;
}
//*************************************************************************************************************************
void Setup_Master()
{
    PaceMaker              = PACEMAKER_MASTER;
    RetryCount             = RETRYCOUNT_MASTER;
    RetryWait              = RETRYWAIT_MASTER;
    LostContactCutOff      = LOSTCONTACTCUTOFF_MASTER;
    ReconnectChannelsCount = RECONNECT_CHANNELS_COUNT_MASTER;
    ReconnectChannelsStart = RECONNECT_CHANNELS_START_MASTER;
}
//*************************************************************************************************************************
void Setup_Pupil()
{
    PaceMaker              = PACEMAKER_PUPIL;
    RetryCount             = RETRYCOUNT_PUPIL;
    RetryWait              = RETRYWAIT_PUPIL;
    LostContactCutOff      = LOSTCONTACTCUTOFF_PUPIL;
    ReconnectChannelsCount = RECONNECT_CHANNELS_COUNT_PUPIL;
    ReconnectChannelsStart = RECONNECT_CHANNELS_START_PUPIL;
}

//*************************************************************************************************************************
#endif
