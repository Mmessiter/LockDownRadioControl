// ********************** SDFiles.h ***********************************************
// This file contains  the functions for opening files to the SD card 

#include <Arduino.h>
#include "1Definitions.h"
#include <SD.h>
#include <SPI.h>

#ifndef SD_FILES_H  
#define SD_FILES_H

/************************************************************************************************************/
FASTRUN void OpenLogFileW()
{
    if (!LogFileOpen)
    {
        LogFileNumber = SD.open(LogFileName, FILE_WRITE);
        LogFileOpen = true;
    }
}
/****************************************************************************************************************************/
File OpenTheLogFileForReading()
{
    char SearchFile[40];
    strcpy(SearchFile, "/");
    strcat(SearchFile, LogFileName);
    File fnumber = SD.open(SearchFile, FILE_READ);
    if (fnumber)
        LogFileOpen = true;
    return fnumber;
}

/******************************************************************************************************************************/

uint32_t GetFileSize(char *filename)
{
    File file = SD.open(filename, FILE_READ);
    if (!file)
        return 0;
    uint32_t size = file.size();
    file.close();
    return size;
}

/*********************************************************************************************************************************/

void WriteEntireBuffer()
{
    ModelsFileNumber.close();
    CloseModelsFile();
    for (int i = 0; i < 3; ++i)
    {
        ModelsFileNumber = SD.open(SingleModelFile, FILE_WRITE);
        ModelsFileNumber.seek(0);
        ShortishDelay();
        ModelsFileNumber.write(NewFileBuffer, NewFileBufferPointer);
        ShortishDelay();
        ModelsFileNumber.close();
        ShortishDelay();
    }
}

#endif // SD_FILES_H