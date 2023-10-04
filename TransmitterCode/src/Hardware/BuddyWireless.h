// *************************************** BuddyWireless.h  *****************************************
#include <Arduino.h>
#include "Hardware/1Definitions.h"
#include "Hardware/StructsEtc.h"
#ifndef BUDDYWIRELESS_H
    #define BUDDYWIRELESS_H
//*************************************************************************************************************************
void Setup_Solo()
{
    FHSS_data::PaceMaker              = PACEMAKER;
    FHSS_data::RetryCount             = RETRYCOUNT;
    FHSS_data::RetryWait              = RETRYWAIT;
    FHSS_data::LostContactCutOff      = LOSTCONTACTCUTOFF;
    FHSS_data::ReconnectChannelsCount = RECONNECT_CHANNELS_COUNT;
    FHSS_data::ReconnectChannelsStart = RECONNECT_CHANNELS_START;
}
//*************************************************************************************************************************
void Setup_Master()
{
    FHSS_data::PaceMaker              = PACEMAKER_MASTER;
    FHSS_data::RetryCount             = RETRYCOUNT_MASTER;
    FHSS_data::RetryWait              = RETRYWAIT_MASTER;
    FHSS_data::LostContactCutOff      = LOSTCONTACTCUTOFF_MASTER;
    FHSS_data::ReconnectChannelsCount = RECONNECT_CHANNELS_COUNT_MASTER;
    FHSS_data::ReconnectChannelsStart = RECONNECT_CHANNELS_START_MASTER;
}
//*************************************************************************************************************************
void Setup_Pupil()
{
    FHSS_data::PaceMaker              = PACEMAKER_PUPIL;
    FHSS_data::RetryCount             = RETRYCOUNT_PUPIL;
    FHSS_data::RetryWait              = RETRYWAIT_PUPIL;
    FHSS_data::LostContactCutOff      = LOSTCONTACTCUTOFF_PUPIL;
    FHSS_data::ReconnectChannelsCount = RECONNECT_CHANNELS_COUNT_PUPIL;
    FHSS_data::ReconnectChannelsStart = RECONNECT_CHANNELS_START_PUPIL;
}

//*************************************************************************************************************************
#endif
