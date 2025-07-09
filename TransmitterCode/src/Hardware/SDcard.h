// ********************** SDcard.h ***********************************************
// This file contains  the functions for reading and writing to the SD card for MODEL MEMORIES and TRANSMITTER PARAMETERS

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#include <SD.h>
#include <SPI.h>

#ifndef SD_CARD_H
#define SD_CARD_H

/*********************************************************************************************************************************/
void SaveCheckSum32()
{ // uses 5 bytes. Last one is indicator of use.

    DoingCheckSm = true;
    SDUpdate32BITS(SDCardAddress, FileCheckSum); //
    SDCardAddress += 4;
    SDUpdate8BITS(SDCardAddress, 0xFF); // indicator
    SDCardAddress++;
    DoingCheckSm = false;

#ifdef DB_CHECKSUM
    Serial.print("Writing: ");
    Serial.println(FileCheckSum);
#endif
}

/*********************************************************************************************************************************/
void ReadCheckSum32()
{ // uses 5 bytes. Last one is indicator of use.
  // This function sets ErrorState to a non zero value if there'a a file error

    bool UseCheckSm = false;
    DoingCheckSm = true;
    uint32_t ch;
    ch = SDRead32BITS(SDCardAddress);
    SDCardAddress += 4;
    if (SDRead8BITS(SDCardAddress) == 0xFF)
        UseCheckSm = true; // if this byte isn't 0xFF then checksum was never written here.
    ++SDCardAddress;
    DoingCheckSm = false;
    if (UseCheckSm)
    {
        if (ch != FileCheckSum)
        {
            ErrorState = CHECKSUMERROR;
        }
#ifdef DB_CHECKSUM
        Serial.print("Read from file: ");
        Serial.println(ch);
        if (ch == FileCheckSum)
        {
            Serial.println("FILE CHECKSUM IS GOOD :-) ! ");
        }
        else
        {
            Serial.print("FILE CHECKSUM WAS ");
            Serial.println(FileCheckSum);
        }
#endif
    }
}

/*********************************************************************************************************************************/
bool CheckDuplicate(uint8_t ch, int8_t j)
{
    for (int i = 0; i < j; ++i)
    {
        if (ch == ChannelOutPut[i])
            return false; // no duplicate output channels allowed
    }
    return true;
}

/*********************************************************************************************************************************/

void CheckOutPutChannels() // This function checks for bad or duplicate output channels and resets them if found
{
    bool resetit = false;
    for (int i = 0; i < 16; ++i)
    {
        if ((ChannelOutPut[i] > 15) || (!CheckDuplicate(ChannelOutPut[i], i)))
        {
            resetit = true;
            break;
        }
    }
    if (resetit)
    {
        for (int i = 0; i < 16; ++i)
        {
            ChannelOutPut[i] = i;
        }
    }
}

/*********************************************************************************************************************************/
void CheckServoSpeeds()
{ // for slow servos

    bool flag = false;
    for (int j = 0; j < BANKS_USED + 1; ++j)
    {
        for (int i = 0; i < CHANNELSUSED + 1; ++i)
        {
            if ((ServoSpeed[j][i] > 100) || (!ServoSpeed[j][i]))
            { // out of range or zero means the list is corrupt and needs resetting
                flag = true;
                break;
            }
        }
    }
    if (flag)
    {
        for (int j = 0; j < BANKS_USED + 1; ++j)
        {
            for (int i = 0; i < CHANNELSUSED + 1; ++i)
            {
                ServoSpeed[j][i] = 100;
            }
        }
    }
}
/*********************************************************************************************************************************/
void CheckBanksInUse()
{
    for (int i = 0; i < 4; ++i)
    {
        if (BanksInUse[i] > 23)
            BanksInUse[i] = i;
    }
}
/*********************************************************************************************************************************/

void CheckServoType()
{
    for (int i = 0; i < 11; ++i)
    {
        if (ServoFrequency[i] > 1000)
            ServoFrequency[i] = 50;
        if (ServoFrequency[i] < 50)
            ServoFrequency[i] = 50;
        if (ServoCentrePulse[i] > 2500)
            ServoCentrePulse[i] = 1500;
        if (ServoCentrePulse[i] < 300)
            ServoCentrePulse[i] = 760;
    }
}
///*********************************************************************************************************************************/
void CheckStabilistionParameters()
{

    if (ActiveSettings->Marker != PID_MARKER_VALUE)
    {
        ActiveSettings = &RateSettings; // set the active settings to the rate settings
        RateSettings = FactoryPlaneRate;
        SelfLevelSettings = FactoryPlaneLevelling;
        SelfLevellingOn = false; // defaults are always without self-levelling
                                 // MsgBox(pFrontView, (char*)"PIDs etc. -> defaults!");
    }
}

/************************************************************************************************************************************************/
/**********************************  READ A MODEL ***********************************************************************************************/
/************************************************************************************************************************************************/

bool ReadOneModel(uint32_t Mnum)
{
    uint16_t j;
    uint16_t i;
    char NoModelYet[] = "File error?";

    FileCheckSum = 0;
    MixNumber = 0;
    if ((ModelNumber > 90) || (ModelNumber <= 0))
        ModelNumber = 1;

    OpenModelsFile();

    if (!ModelsFileOpen)
    {
        DelayWithDog(300);
        OpenModelsFile();
    }

    if (!ModelsFileOpen)
    {
        strcpy(ModelName, NoModelYet); // indicator of error or no model
        return false;
    }

    SDCardAddress = TXSIZE;
    if (SingleModelFlag)
        SDCardAddress = 0; // MODELOFFSET;   // Changed from 250 to 0
    SDCardAddress += ((Mnum - 1) * MODELSIZE);
    StartLocation = SDCardAddress;
    ModelDefined = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    if (ModelDefined != 42)
        return false;
    for (j = 0; j < 30; ++j)
    {
        ModelName[j] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }

    for (i = 0; i < CHANNELSUSED; ++i)
    {
        for (j = 1; j <= 4; ++j)
        {
            MaxDegrees[j][i] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
            MidHiDegrees[j][i] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
            CentreDegrees[j][i] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
            MidLowDegrees[j][i] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
            MinDegrees[j][i] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < MAXMIXES; ++j)
    {
        for (i = 0; i < CHANNELSUSED + 1; ++i)
        {
            Mixes[j][i] = SDRead8BITS(SDCardAddress); // Read mixes
            ++SDCardAddress;
        }
    }

    for (j = 0; j < BANKS_USED + 1; ++j)
    {
        for (i = 0; i < CHANNELSUSED + 1; ++i)
        {
            Trims[j][i] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < BANKS_USED + 1; ++j)
    {
        for (i = 0; i < CHANNELSUSED + 1; ++i)
        {
            ServoSpeed[j][i] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
        }
    }
    CheckServoSpeeds();
    RXCellCount = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    TrimMultiplier = SDRead16BITS(SDCardAddress);
    TrimMultiplier = CheckRange(TrimMultiplier, 0, 25);
    ++SDCardAddress;
    ++SDCardAddress;
    LowBattery = SDRead8BITS(SDCardAddress);
    if (LowBattery > 100)
        LowBattery = LOWBATTERY;
    if (LowBattery < 10)
        LowBattery = LOWBATTERY;
    ++SDCardAddress;
    CopyTrimsToAll = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;

    for (i = 0; i < CHANNELSUSED; ++i)
    {
        SubTrims[i] = SDRead8BITS(SDCardAddress);
        if ((SubTrims[i] < 10) || (SubTrims[i] > 244))
            SubTrims[i] = 127; // centre if undefined or zero
        ++SDCardAddress;
    }
    ReversedChannelBITS = SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    for (i = 0; i < 4; ++i)
    {
        InputTrim[i] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    RxVoltageCorrection = SDRead16BITS(SDCardAddress);

    ++SDCardAddress;
    ++SDCardAddress;
    CheckSavedTrimValues();
    BuddyControlled = SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    SavedSticksMode = SDRead8BITS(SDCardAddress); // save sticks mode in case of rf transfer
    ++SDCardAddress;

    for (i = 0; i < CHANNELSUSED; ++i)
    {
        InPutStick[i] = SDRead8BITS(SDCardAddress);
        if (InPutStick[i] > 16)
            InPutStick[i] = i; // reset if nothing was saved!
        ++SDCardAddress;
    }

    for (i = 0; i < 4; ++i)
    {
        DualRateRate[i] = SDRead8BITS(SDCardAddress);
        if (DualRateRate[i] > 3)
            DualRateRate[i] = 0;
        ++SDCardAddress;
    }

    BuddyHasAllSwitches = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;

    ++SDCardAddress; // spare bytes
    ++SDCardAddress;
    ++SDCardAddress;
    ++SDCardAddress;
    ++SDCardAddress;

    for (i = 0; i < CHANNELSUSED; ++i)
    {
        FailSafeChannel[i] = bool(SDRead8BITS(SDCardAddress));
        if (int(FailSafeChannel[i]) > 1)
            FailSafeChannel[i] = 0;
        ++SDCardAddress;
    }
    for (i = 0; i < CHANNELSUSED; ++i)
    {
        for (j = 0; j < 10; ++j)
        {
            ChannelNames[i][j] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
        }
    }

    for (j = 0; j < BANKS_USED + 1; ++j)
    {
        for (i = 0; i < CHANNELSUSED + 1; ++i)
        {
            Exponential[j][i] = SDRead8BITS(SDCardAddress);
            if (Exponential[j][i] >= 201 || Exponential[j][i] == 0)
            {
                Exponential[j][i] = DEFAULT_EXPO;
            }
            ++SDCardAddress;
        }
    }
    for (j = 0; j < BANKS_USED + 1; ++j)
    {
        for (i = 0; i < CHANNELSUSED + 1; ++i)
        {
            InterpolationTypes[j][i] = SDRead8BITS(SDCardAddress);
            if (InterpolationTypes[j][i] < 0 || InterpolationTypes[j][i] > 2)
            {
                InterpolationTypes[j][i] = EXPONENTIALCURVES;
            }
            ++SDCardAddress;
        }
    }
    for (j = 0; j < BYTESPERMACRO; ++j)
    {
        for (i = 0; i < MAXMACROS; ++i)
        {
            MacrosBuffer[i][j] = SDRead8BITS(SDCardAddress);
            ++SDCardAddress;
        }
    }
    for (i = 0; i < 8; ++i)
    {
        ModelsMacUnionSaved.Val8[i] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    UseMotorKill = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    MotorChannelZero = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    MotorChannel = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;

    // TREX
    ++SDCardAddress;

    SFV = SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    StopFlyingVoltsPerCell = float(SFV) / 100;
    if (StopFlyingVoltsPerCell < 3 || StopFlyingVoltsPerCell > 4)
        StopFlyingVoltsPerCell = 3.50; // a useful default stop time?!
    Drate2 = SDRead8BITS(SDCardAddress);
    if ((Drate2 < 10) || (Drate2 > 200))
        Drate2 = 100;
    ++SDCardAddress;
    Drate3 = SDRead8BITS(SDCardAddress);
    if ((Drate3 < 10) || (Drate3 > 200))
        Drate3 = 100;
    ++SDCardAddress;
    Drate1 = SDRead8BITS(SDCardAddress);
    if ((Drate1 < 10) || (Drate1 > 200))
        Drate1 = 100;
    ++SDCardAddress;
    for (int i = 0; i < 8; ++i)
    {
        DualRateChannels[i] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    CheckDualRatesValues();

    ++SDCardAddress; // Spare byte
    ++SDCardAddress; // Spare byte

    for (i = 0; i < 4; ++i)
    {
        BanksInUse[i] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    CheckBanksInUse();

    for (i = 0; i < 12; ++i)
    {
        // 12 Spare bytes was 16
        ++SDCardAddress;
    }
    ServoCentrePulse[0] = SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    ServoFrequency[0] = SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;

    TimerDownwards = (bool)SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    TimerStartTime = SDRead16BITS(SDCardAddress);
    if (TimerStartTime > 120 * 60)
        TimerStartTime = 5 * 60;
    ++SDCardAddress;
    ++SDCardAddress;
    StabilisedBank = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    LevelledBank = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;

    for (i = 0; i < 16; ++i)
    {
        ChannelOutPut[i] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    for (int i = 0; i < 11; ++i)
    {
        ServoFrequency[i] = SDRead16BITS(SDCardAddress);
        ++SDCardAddress;
        ++SDCardAddress;
        ServoCentrePulse[i] = SDRead16BITS(SDCardAddress);
        ++SDCardAddress;
        ++SDCardAddress;
    }

    // Stabiliasation and Self Levelling *****************

    StabilisationOn = (bool)SDRead8BITS(SDCardAddress); // 3
    ++SDCardAddress;
    SelfLevellingOn = (bool)SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    SavedActiveSettings = ActiveSettings; // save current settings pointer
    ActiveSettings = &RateSettings;

    ActiveSettings->PID_P = SDReadFLOAT(SDCardAddress); // heer 1
    SDCardAddress += 4;
    ActiveSettings->PID_I = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->PID_D = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;

    ActiveSettings->Tail_PID_P = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->Tail_PID_I = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->Tail_PID_D = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;

    ActiveSettings->Kalman_Q_angle = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->Kalman_Q_bias = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->Kalman_R_measure = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->alpha = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->beta = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->UseKalmanFilter = (bool)SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    ActiveSettings->UseRateLPF = (bool)SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    ActiveSettings->Marker = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;

    ActiveSettings = &SelfLevelSettings;

    ActiveSettings->PID_P = SDReadFLOAT(SDCardAddress); // heer 2
    SDCardAddress += 4;
    ActiveSettings->PID_I = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->PID_D = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;

    ActiveSettings->Tail_PID_P = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->Tail_PID_I = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->Tail_PID_D = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;

    ActiveSettings->Kalman_Q_angle = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->Kalman_Q_bias = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->Kalman_R_measure = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->alpha = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->beta = SDReadFLOAT(SDCardAddress);
    SDCardAddress += 4;
    ActiveSettings->UseKalmanFilter = (bool)SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    ActiveSettings->UseRateLPF = (bool)SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    ActiveSettings->Marker = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;

    ActiveSettings = SavedActiveSettings;

    CheckOutPutChannels();
    CheckServoType();
    CheckStabilistionParameters();

    // **************************************

    ReadCheckSum32();
    OneModelMemory = SDCardAddress - StartLocation;

#ifdef DB_SD
    Serial.print(MemoryForTransmtter);
    Serial.println(" bytes were  used for TX data.");
    Serial.print(TXSIZE);
    Serial.println(" bytes had been reserved.");
    Serial.print("So ");
    Serial.print(TXSIZE - MemoryForTransmtter);
    Serial.println(" spare bytes still remain for TX params.");
    Serial.print("Loaded model number: ");
    Serial.println(ModelNumber);
    Serial.print("Model name: ");
    Serial.println(ModelName);
    Serial.println(" ");
    Serial.print(OneModelMemory);
    Serial.println(" bytes used per model.");
    Serial.print(MODELSIZE - OneModelMemory);
    Serial.println(" spare bytes per model.");
    Serial.print(MODELSIZE);
    Serial.println(" bytes reserved per model.)");
    Serial.println(" ");
#endif // defined DB_SD

#ifdef DB_CHECKSUM
    Serial.print("Read Model: ");
    Serial.print(ModelName);
    Serial.print(" Calculated model checksum: ");
    Serial.println(FileCheckSum);
    Serial.println(" ");
#endif

    UpdateButtonLabels();
    CheckMacrosBuffer();
    return true;
}

/*********************************************************************************************************************************/

void CloseModelsFile()
{
    if (ModelsFileOpen)
    {
        ModelsFileNumber.close();
        ModelsFileOpen = false;
        delayMicroseconds(500);
    }
}

/*********************************************************************************************************************************/

bool CheckFileExists(char *fl)
{
    CloseModelsFile();
    bool exists = false;
    File t;
    t = SD.open(fl, FILE_READ);
    if (t)
        exists = true;
    t.close();
    return exists;
}

/*********************************************************************************************************************************/

void ShortDelay()
{
    delayMicroseconds(10);
}

void ShortishDelay()
{
    delayMicroseconds(1750);
}

/*********************************************************************************************************************************/

void OpenModelsFile()
{

    char ModelsFile[] = "models.dat";

    if (!ModelsFileOpen)
    {
        if (SingleModelFlag)
        {
            ModelsFileNumber = SD.open(SingleModelFile, FILE_WRITE);
            DelayWithDog(100);
        }
        else
        {
            ModelsFileNumber = SD.open(ModelsFile, FILE_WRITE);
            DelayWithDog(100);
        }
        if (ModelsFileNumber == 0)
        {
            FileError = true;
        }
        else
        {
            ModelsFileOpen = true;
        }
    }
}
/*********************************************************************************************************************************/

void BuildCheckSum(int p_address, short int p_value)
{
    if (!DoingCheckSm)
        FileCheckSum += (p_value * (p_address + 1)); // don't include checksum in its own calculation
}

/*********************************************************************************************************************************/
void SDUpdateFLOAT(int p_address, float p_value)
{
    union
    {
        float f;
        uint8_t b[4];
        uint32_t u32;
    } converter;
    converter.f = p_value;
    BuildCheckSum(p_address, converter.u32);
    ModelsFileNumber.seek(p_address);
    ShortDelay();
    for (int i = 0; i < 4; ++i)
    {
        ModelsFileNumber.write(converter.b[i]);
        ShortDelay();
    }
}
//*********************************************************************************************************************************/
float SDReadFLOAT(int p_address)
{
    union
    {
        float f;
        uint8_t b[4];
        uint32_t u32;
    } converter;
    ModelsFileNumber.seek(p_address);
    ShortDelay();
    for (int i = 0; i < 4; ++i)
    {
        converter.b[i] = ModelsFileNumber.read();
        ShortDelay();
    }
    BuildCheckSum(p_address, converter.u32);
    return converter.f;
}
/*********************************************************************************************************************************/

void SDUpdate32BITS(int p_address, uint32_t p_value)
{
    BuildCheckSum(p_address, p_value);
    ModelsFileNumber.seek(p_address);
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value));
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value >> 8));
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value >> 16));
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value >> 24));
    ShortDelay();
}

/*********************************************************************************************************************************/

uint32_t SDRead32BITS(int p_address)
{
    ModelsFileNumber.seek(p_address);
    uint32_t r = ModelsFileNumber.read();
    r += ModelsFileNumber.read() << 8;
    r += ModelsFileNumber.read() << 16;
    r += ModelsFileNumber.read() << 24;
    return r;
}
/*********************************************************************************************************************************/

void SDUpdate16BITS(int p_address, short int p_value)
{
    BuildCheckSum(p_address, p_value);
    ModelsFileNumber.seek(p_address);
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value));
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value >> 8));
    ShortDelay();
}
/*********************************************************************************************************************************/

void SDUpdate8BITS(int p_address, uint8_t p_value)
{
    BuildCheckSum(p_address, p_value);
    ModelsFileNumber.seek(p_address);
    ShortDelay();
    ModelsFileNumber.write(uint8_t(p_value));
    ShortDelay();
}

/*********************************************************************************************************************************/

short int SDRead16BITS(int p_address)
{

    ModelsFileNumber.seek(p_address);
    short int r = ModelsFileNumber.read();
    r += ModelsFileNumber.read() << 8;
    BuildCheckSum(p_address, r);
    return r;
}

/*********************************************************************************************************************************/

uint8_t SDRead8BITS(int p_address)
{
    ModelsFileNumber.seek(p_address);
    uint8_t r = ModelsFileNumber.read();
    BuildCheckSum(p_address, r);
    return r;
}

/*********************************************************************************************************************************/
/******************************************* LOAD ALL PARAMS *********************************************************************/
/*********************************************************************************************************************************/

// NB This starts with transmitter params and then calls the model params load function

bool LoadAllParameters()
{
    int j = 0;
    int i = 0;
    FileCheckSum = 0;
    if (!ModelsFileOpen)
        OpenModelsFile();
    if (!ModelsFileOpen)
        return false;
    SDCardAddress = 0;
    if ((SDRead16BITS(SDCardAddress)) != 12345)
        return false; // not a good file?!
    SDCardAddress += 2;
    for (i = 0; i < CHANNELSUSED; ++i)
    {
        ChannelMin[i] = SDRead16BITS(SDCardAddress);
        SDCardAddress += 2;
        ChannelMidLow[i] = SDRead16BITS(SDCardAddress);
        SDCardAddress += 2;
        ChannelCentre[i] = SDRead16BITS(SDCardAddress);
        SDCardAddress += 2;
        ChannelMidHi[i] = SDRead16BITS(SDCardAddress);
        SDCardAddress += 2;
        ChannelMax[i] = SDRead16BITS(SDCardAddress);
        SDCardAddress += 2;
    }
    ++SDCardAddress;
    ++SDCardAddress;
    ModelNumber = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    ScreenTimeout = SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    Inactivity_Timeout = SDRead8BITS(SDCardAddress) * TICKSPERMINUTE;
    if (Inactivity_Timeout < INACTIVITYMINIMUM)
        Inactivity_Timeout = INACTIVITYMINIMUM;
    if (Inactivity_Timeout > INACTIVITYMAXIMUM)
        Inactivity_Timeout = INACTIVITYMAXIMUM;
    ++SDCardAddress;
    for (j = 0; j < 30; ++j)
    {
        TxName[j] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    Qnh = (float)SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    DeltaGMT = SDRead16BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    BackGroundColour = SDRead16BITS(SDCardAddress);
    if (BackGroundColour == 0)
        BackGroundColour = 214;
    ++SDCardAddress;
    ++SDCardAddress;
    ForeGroundColour = SDRead16BITS(SDCardAddress);
    if (ForeGroundColour == 0)
        ForeGroundColour = 65535;
    ++SDCardAddress;
    ++SDCardAddress;
    SpecialColour = SDRead16BITS(SDCardAddress);
    if (SpecialColour == 0)
        SpecialColour = Red;
    ++SDCardAddress;
    ++SDCardAddress;
    HighlightColour = SDRead16BITS(SDCardAddress);
    if (HighlightColour == 0)
        HighlightColour = Yellow;
    ++SDCardAddress;
    ++SDCardAddress;
    SticksMode = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    AudioVolume = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    Brightness = SDRead8BITS(SDCardAddress);
    if (Brightness < 10)
        Brightness = 10;
    ++SDCardAddress;
    PlayFanfare = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    TrimClicks = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    UseVariometer = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    SpeakingClock = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    AnnounceBanks = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    for (i = 0; i < 8; ++i)
    {
        j = SDRead8BITS(SDCardAddress);
        if ((j >= SWITCH7) && (j <= SWITCH0))
        {
            SwitchNumber[i] = j;
        }
        ++SDCardAddress;
    }
    ScanSensitivity = SDRead8BITS(SDCardAddress);
    ScanSensitivity = CheckRange(ScanSensitivity, 1, 255);
    ++SDCardAddress;
    MinimumGap = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    LogRXSwaps = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    UseLog = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    AnnounceConnected = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    for (j = 0; j < 8; ++j)
    {
        TrimNumber[j] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    TxVoltageCorrection = SDRead16BITS(SDCardAddress);
    if ((TxVoltageCorrection > 20) || (TxVoltageCorrection < 0))
        TxVoltageCorrection = 0;
    ++SDCardAddress;
    ++SDCardAddress;
    PowerOffWarningSeconds = SDRead8BITS(SDCardAddress);
    PowerOffWarningSeconds = CheckRange(PowerOffWarningSeconds, 1, 30);
    ++SDCardAddress;
    LEDBrightness = SDRead8BITS(SDCardAddress);
    LEDBrightness = CheckRange(LEDBrightness, 1, 254);
    ++SDCardAddress;
    WirelessBuddy = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    ConnectionAssessSeconds = SDRead8BITS(SDCardAddress);
    ConnectionAssessSeconds = CheckRange(ConnectionAssessSeconds, 1, 6);
    ++SDCardAddress;
    AutoModelSelect = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    TXLiPo = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    ++SDCardAddress;
    BuddyPupilOnWireless = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    ++SDCardAddress;
    for (int q = 0; q < 5; ++q)
    {
        BuddyMacAddress[q] = SDRead8BITS(SDCardAddress);
        ++SDCardAddress;
    }
    BuddyMasterOnWireless = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    // ************************** READ THE SWITCHES NOW PER TX !*********************************
    BankSwitch = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    Autoswitch = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    TopChannelSwitch[Ch9_SW] = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    TopChannelSwitch[Ch10_SW] = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    TopChannelSwitch[Ch11_SW] = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    TopChannelSwitch[Ch12_SW] = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    SwitchReversed[0] = bool(SDRead8BITS(SDCardAddress));
    ++SDCardAddress;
    SwitchReversed[1] = bool(SDRead8BITS(SDCardAddress));
    ++SDCardAddress;
    SwitchReversed[2] = bool(SDRead8BITS(SDCardAddress));
    ++SDCardAddress;
    SwitchReversed[3] = bool(SDRead8BITS(SDCardAddress));
    ++SDCardAddress;
    BuddySwitch = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    DualRatesSwitch = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    SafetySwitch = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    VariometerBank = SDRead8BITS(SDCardAddress);
    VariometerBank = CheckRange(VariometerBank, 0, 3);
    ++SDCardAddress;
    VariometerSpacing = SDRead16BITS(SDCardAddress);
    VariometerSpacing = CheckRange(VariometerSpacing, 50, 1000);
    ++SDCardAddress;
    ++SDCardAddress;
    VariometerThreshold = SDRead16BITS(SDCardAddress);
    VariometerThreshold = CheckRange(VariometerThreshold, 0, 1000);
    ++SDCardAddress;
    ++SDCardAddress;
    Buddy_Low_Position = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    Buddy_Mid_Position = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;
    Buddy_Hi_Position = SDRead8BITS(SDCardAddress);
    ++SDCardAddress;

    ReadCheckSum32();
    CheckTrimValues();
    MemoryForTransmtter = SDCardAddress;
    if ((ModelNumber < 1) || (ModelNumber > 99))
        ModelNumber = 1;
    ReadOneModel(ModelNumber);
    return true;
}

// uint8_t Buddy_Low_Position = 0;
// uint8_t Buddy_Mid_Position = 1;
// uint8_t Buddy_Hi_Position = 2;

/*********************************************************************************************************************************/
/******************************************** SAVE ONLY THE TRANSMITTER PARAMS ****************************************************/
/*********************************************************************************************************************************/

void SaveTransmitterParameters()
{
    bool EON = false;
    int j = 0;
    int i = 0;
    if (!ModelsFileOpen)
        OpenModelsFile();

    SDCardAddress = 0;
    FileCheckSum = 0;

    SDUpdate16BITS(SDCardAddress, 12345); // marker that file exists!

    SDCardAddress += 2;
    for (i = 0; i < CHANNELSUSED; ++i)
    {
        SDUpdate16BITS(SDCardAddress, ChannelMin[i]); // Stick min output of pot
        SDCardAddress += 2;
        SDUpdate16BITS(SDCardAddress, ChannelMidLow[i]); //
        SDCardAddress += 2;
        SDUpdate16BITS(SDCardAddress, ChannelCentre[i]); // Stick Centre output of pot
        SDCardAddress += 2;
        SDUpdate16BITS(SDCardAddress, ChannelMidHi[i]); //
        SDCardAddress += 2;
        SDUpdate16BITS(SDCardAddress, ChannelMax[i]); // Stick max output of pot
        SDCardAddress += 2;
    }
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, ModelNumber);
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, ScreenTimeout);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, (Inactivity_Timeout / TICKSPERMINUTE));
    ++SDCardAddress;
    for (j = 0; j < 30; ++j)
    {
        if (EON)
            TxName[j] = 0;
        SDUpdate8BITS(SDCardAddress, TxName[j]);
        if (TxName[j] == 0)
            EON = true;
        ++SDCardAddress;
    }
    SDUpdate16BITS(SDCardAddress, (uint16_t)Qnh);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, DeltaGMT);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, BackGroundColour);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, ForeGroundColour);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, SpecialColour);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, HighlightColour);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SticksMode);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, AudioVolume);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Brightness);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, PlayFanfare);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, TrimClicks);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, UseVariometer);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SpeakingClock);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, AnnounceBanks);
    ++SDCardAddress;
    for (i = 0; i < 8; ++i)
    {
        SDUpdate8BITS(SDCardAddress, SwitchNumber[i]);
        ++SDCardAddress;
    }
    SDUpdate8BITS(SDCardAddress, ScanSensitivity);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, MinimumGap);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, LogRXSwaps);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, UseLog);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, AnnounceConnected);
    ++SDCardAddress;
    for (j = 0; j < 8; ++j)
    {
        SDUpdate8BITS(SDCardAddress, TrimNumber[j]);
        ++SDCardAddress;
    }
    SDUpdate16BITS(SDCardAddress, TxVoltageCorrection);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, PowerOffWarningSeconds);
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, LEDBrightness);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, WirelessBuddy);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, ConnectionAssessSeconds);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, AutoModelSelect);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, TXLiPo);
    ++SDCardAddress;
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, BuddyPupilOnWireless);
    ++SDCardAddress;
    ++SDCardAddress;
    for (i = 0; i < 5; ++i)
    {
        SDUpdate8BITS(SDCardAddress, BuddyMacAddress[i]);
        ++SDCardAddress;
    }
    SDUpdate8BITS(SDCardAddress, BuddyMasterOnWireless);
    ++SDCardAddress;

    // WRITE THE SWITCHES NOW PER TX ***

    SDUpdate8BITS(SDCardAddress, BankSwitch);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Autoswitch);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, TopChannelSwitch[Ch9_SW]);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, TopChannelSwitch[Ch10_SW]);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, TopChannelSwitch[Ch11_SW]);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, TopChannelSwitch[Ch12_SW]);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SwitchReversed[0]);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SwitchReversed[1]);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SwitchReversed[2]);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SwitchReversed[3]);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, BuddySwitch);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, DualRatesSwitch);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SafetySwitch);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, VariometerBank);
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, VariometerSpacing);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, VariometerThreshold);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Buddy_Low_Position);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Buddy_Mid_Position);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Buddy_Hi_Position);
    ++SDCardAddress;

    SaveCheckSum32(); // Save the Transmitter parametres checksm
    CloseModelsFile();
}

/*********************************************************************************************************************************/
/****************************************** END OF SAVE TRANSMITTER PARAMS   *****************************************************/
/*********************************************************************************************************************************/

/*********************************************************************************************************************************/
/******************************************  SAVE ALL PARAMS   *******************************************************************/
/*********************************************************************************************************************************/

void SaveAllParameters()
{
    if (!ModelsFileOpen)
        OpenModelsFile();
    SaveTransmitterParameters();
    MemoryForTransmtter = SDCardAddress - 2;
    SaveOneModel(ModelNumber);
#ifdef DB_SD
    Serial.println(" ");
    Serial.print(MemoryForTransmtter);
    Serial.println(" bytes written to SD CARD FOR TX.");
    Serial.print(TXSIZE);
    Serial.println(" bytes reserved for TX.");
    Serial.print(TXSIZE - MemoryForTransmtter);
    Serial.println(" Spare bytes still for any new TX params.");
    Serial.print("Saved model: ");
    Serial.print(ModelNumber);
    Serial.print(" (");
    Serial.print(ModelName);
    Serial.println(")");
#endif // defined DB_SD
}

/*********************************************************************************************************************************/
/******************************************  END OF SAVE ALL PARAMS   ************************************************************/
/*********************************************************************************************************************************/

/*********************************************************************************************************************************/
/******************************************  SAVE MODEL PARAMS   *****************************************************************/
/*********************************************************************************************************************************/

/** MODEL Specific */
void SaveOneModel(uint32_t mnum)
{
    uint32_t j;
    uint32_t i;
    bool EndOfName = false;
    FileCheckSum = 0;
    if ((mnum < 1) || (mnum > MAXMODELNUMBER))
        return; // There is no model zero!
    if (!ModelsFileOpen)
        OpenModelsFile();
    SDCardAddress = TXSIZE;                  //  spare bytes for TX stuff
    SDCardAddress += (mnum - 1) * MODELSIZE; //  spare bytes for Model params
    if (SingleModelFlag)
        SDCardAddress = 0;
    StartLocation = SDCardAddress;
    ModelDefined = 42;
    SDUpdate8BITS(SDCardAddress, ModelDefined);
    ++SDCardAddress;
    for (j = 0; j < 30; ++j)
    {
        if (EndOfName)
            ModelName[j] = 0;
        SDUpdate8BITS(SDCardAddress, ModelName[j]);
        if (ModelName[j] == 0)
            EndOfName = true;
        ++SDCardAddress;
    }

    for (i = 0; i < CHANNELSUSED; ++i)
    {
        for (j = 1; j <= 4; ++j)
        {
            SDUpdate8BITS(SDCardAddress, MaxDegrees[j][i]); // Max requested in degrees (180)
            ++SDCardAddress;
            SDUpdate8BITS(SDCardAddress, MidHiDegrees[j][i]); // MidHi requested in degrees (135)
            ++SDCardAddress;
            SDUpdate8BITS(SDCardAddress, CentreDegrees[j][i]); // Centre requested in degrees (90)
            ++SDCardAddress;
            SDUpdate8BITS(SDCardAddress, MidLowDegrees[j][i]); // MidLo requested in degrees (45)
            ++SDCardAddress;
            SDUpdate8BITS(SDCardAddress, MinDegrees[j][i]); // Min requested in degrees (0)
            ++SDCardAddress;
        }
    }
    for (j = 0; j < MAXMIXES; ++j)
    {
        for (i = 0; i < 17; ++i)
        {
            SDUpdate8BITS(SDCardAddress, Mixes[j][i]); // Save mixes
            ++SDCardAddress;
        }
    }
    for (j = 0; j < BANKS_USED + 1; ++j)
    {
        for (i = 0; i < CHANNELSUSED + 1; ++i)
        {
            SDUpdate8BITS(SDCardAddress, Trims[j][i]);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < BANKS_USED + 1; ++j)
    {
        for (i = 0; i < CHANNELSUSED + 1; ++i)
        {
            SDUpdate8BITS(SDCardAddress, ServoSpeed[j][i]); // Save servo speeds
            ++SDCardAddress;
        }
    }
    SDUpdate8BITS(SDCardAddress, RXCellCount);
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, TrimMultiplier);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, LowBattery);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, CopyTrimsToAll);
    ++SDCardAddress;
    for (i = 0; i < CHANNELSUSED; ++i)
    {
        SDUpdate8BITS(SDCardAddress, SubTrims[i]);
        ++SDCardAddress;
    }
    SDUpdate16BITS(SDCardAddress, ReversedChannelBITS);
    ++SDCardAddress;
    ++SDCardAddress;
    for (i = 0; i < 4; ++i)
    {
        SDUpdate8BITS(SDCardAddress, InputTrim[i]);
        ++SDCardAddress;
    }
    SDUpdate16BITS(SDCardAddress, RxVoltageCorrection);
    ++SDCardAddress;
    ++SDCardAddress;

    SDUpdate16BITS(SDCardAddress, BuddyControlled);
    ++SDCardAddress;
    ++SDCardAddress;

    SDUpdate8BITS(SDCardAddress, SticksMode); // save sticks mode in case of rf transfer

    SDCardAddress += 1;

    for (i = 0; i < CHANNELSUSED; ++i)
    {
        SDUpdate8BITS(SDCardAddress, InPutStick[i]);
        ++SDCardAddress;
    }

    for (i = 0; i < 4; ++i)
    {
        SDUpdate8BITS(SDCardAddress, DualRateRate[i]);
        ++SDCardAddress;
    }

    SDUpdate8BITS(SDCardAddress, BuddyHasAllSwitches);
    ++SDCardAddress;

    ++SDCardAddress;
    ++SDCardAddress;
    ++SDCardAddress;
    ++SDCardAddress;
    ++SDCardAddress;

    // *********************************************************************************

    for (i = 0; i < CHANNELSUSED; ++i)
    {
        SDUpdate8BITS(SDCardAddress, FailSafeChannel[i]);
        ++SDCardAddress;
    }
    for (i = 0; i < CHANNELSUSED; ++i)
    {
        for (j = 0; j < 10; ++j)
        {
            SDUpdate8BITS(SDCardAddress, ChannelNames[i][j]);
            ++SDCardAddress;
        }
    }
    for (j = 0; j < BANKS_USED + 1; ++j)
    {
        for (i = 0; i < CHANNELSUSED + 1; ++i)
        {
            SDUpdate8BITS(SDCardAddress, Exponential[j][i]);
            ++SDCardAddress;
        }
    }

    for (j = 0; j < BANKS_USED + 1; ++j)
    {
        for (i = 0; i < CHANNELSUSED + 1; ++i)
        {
            SDUpdate8BITS(SDCardAddress, InterpolationTypes[j][i]);
            ++SDCardAddress;
        }
    }

    for (j = 0; j < BYTESPERMACRO; ++j)
    {
        for (i = 0; i < MAXMACROS; ++i)
        {
            SDUpdate8BITS(SDCardAddress, MacrosBuffer[i][j]);
            ++SDCardAddress;
        }
    }
    for (i = 0; i < 8; ++i)
    {
        SDUpdate8BITS(SDCardAddress, ModelsMacUnionSaved.Val8[i]);
        ++SDCardAddress;
    }

    SDUpdate8BITS(SDCardAddress, UseMotorKill);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, MotorChannelZero);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, MotorChannel);
    if (MotorChannel > 15)
        MotorChannel = 15;
    ++SDCardAddress;
    ++SDCardAddress; // Spare byte

    SDUpdate16BITS(SDCardAddress, SFV);
    ++SDCardAddress;
    ++SDCardAddress;

    SDUpdate8BITS(SDCardAddress, Drate2);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Drate3);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, Drate1);
    ++SDCardAddress;

    for (int i = 0; i < 8; ++i)
    {
        SDUpdate8BITS(SDCardAddress, DualRateChannels[i]);
        ++SDCardAddress;
    }
    ++SDCardAddress; // Spare byte
    ++SDCardAddress; // Spare byte

    for (i = 0; i < 4; ++i)
    {
        SDUpdate8BITS(SDCardAddress, BanksInUse[i]);
        ++SDCardAddress;
    }

    for (i = 0; i < 12; ++i)
    {
        // 12 Spare bytes was 16
        ++SDCardAddress;
    }
    CheckServoType();
    SDUpdate16BITS(SDCardAddress, ServoCentrePulse[0]);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, ServoFrequency[0]);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, TimerDownwards);
    ++SDCardAddress;
    SDUpdate16BITS(SDCardAddress, TimerStartTime);
    ++SDCardAddress;
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, StabilisedBank);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, LevelledBank);
    ++SDCardAddress;
    ++SDCardAddress;
    for (i = 0; i < 16; ++i)
    {
        SDUpdate8BITS(SDCardAddress, ChannelOutPut[i]);
        ++SDCardAddress;
    }
    for (int i = 0; i < 11; ++i)
    {
        SDUpdate16BITS(SDCardAddress, ServoFrequency[i]);
        ++SDCardAddress;
        ++SDCardAddress;
        SDUpdate16BITS(SDCardAddress, ServoCentrePulse[i]);
        ++SDCardAddress;
        ++SDCardAddress;
    }

    // Stabilisation and Self Levelling *****************

    SDUpdate8BITS(SDCardAddress, StabilisationOn); // 1
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, SelfLevellingOn);
    ++SDCardAddress;
    SavedActiveSettings = ActiveSettings; // save state of active settings

    ActiveSettings = &RateSettings;

    SDUpdateFLOAT(SDCardAddress, ActiveSettings->PID_P); // heer 3
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->PID_I);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->PID_D);
    SDCardAddress += 4;

    SDUpdateFLOAT(SDCardAddress, ActiveSettings->Tail_PID_P);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->Tail_PID_I);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->Tail_PID_D);
    SDCardAddress += 4;

    SDUpdateFLOAT(SDCardAddress, ActiveSettings->Kalman_Q_angle);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->Kalman_Q_bias);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->Kalman_R_measure);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->alpha);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->beta);
    SDCardAddress += 4;
    SDUpdate8BITS(SDCardAddress, ActiveSettings->UseKalmanFilter);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, ActiveSettings->UseRateLPF);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, ActiveSettings->Marker);
    ++SDCardAddress;

    ActiveSettings = &SelfLevelSettings;

    SDUpdateFLOAT(SDCardAddress, ActiveSettings->PID_P); // heer 4
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->PID_I);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->PID_D);
    SDCardAddress += 4;

    SDUpdateFLOAT(SDCardAddress, ActiveSettings->Tail_PID_P);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->Tail_PID_I);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->Tail_PID_D);
    SDCardAddress += 4;

    SDUpdateFLOAT(SDCardAddress, ActiveSettings->Kalman_Q_angle);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->Kalman_Q_bias);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->Kalman_R_measure);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->alpha);
    SDCardAddress += 4;
    SDUpdateFLOAT(SDCardAddress, ActiveSettings->beta);
    SDCardAddress += 4;
    SDUpdate8BITS(SDCardAddress, ActiveSettings->UseKalmanFilter);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, ActiveSettings->UseRateLPF);
    ++SDCardAddress;
    SDUpdate8BITS(SDCardAddress, ActiveSettings->Marker);
    ++SDCardAddress;

    ActiveSettings = SavedActiveSettings; // restore active settings
    SaveCheckSum32();                     // Save the Model parametres checksm

    // ********************** Add more

    OneModelMemory = SDCardAddress - StartLocation;
#ifdef DB_SD
    Serial.print("Saved model: ");
    Serial.println(ModelName);
    Serial.println(" ");
    Serial.print(OneModelMemory);
    Serial.println(" bytes used per model.");
    Serial.print(MODELSIZE - OneModelMemory);
    Serial.println(" spare bytes per model.");
    Serial.print(MODELSIZE);
    Serial.println(" bytes reserved per model.)");
    Serial.print("Write Model ");
    Serial.print(ModelNumber);
    Serial.print(" Checksum: ");
    Serial.println(FileCheckSum);
    Serial.println(" ");
#endif // defined DB_SD

#ifdef DB_CHECKSUM
    Serial.print("Write Model: ");
    Serial.print(ModelName);
    Serial.print(" Checksum: ");
    Serial.println(FileCheckSum);
    Serial.println(" ");
#endif

    CloseModelsFile();
}

/*********************************************************************************************************************************/
/******************************************  END OF SAVE MODEL PARAMS   **********************************************************/
/*********************************************************************************************************************************/

#endif