// ********************** SDFiles.h ***********************************************
// This file contains  the functions for opening files to the SD card

#include <Arduino.h>
#include "1Definitions.h"
#include <SD.h>
#include <SPI.h>

#ifndef SD_FILES_H
#define SD_FILES_H

#define TINYCARD SD





// **********************************************************************************************************************************/
// This function adds the needed path and creates 'SearchFile' but doesn't alter the filename.
// It looks at the filename extension to determine which directory to look in.  
// This is because we have different directories for help files, log files, image files and model files.

void AddPath(char *filename)
{
    strcpy(SearchFile, "/");
    if (InStrng((char *)".TXT", filename))
        strcat(SearchFile, "help/");
    if (InStrng((char *)".LOG", filename))
        strcat(SearchFile, "log/");
    if (InStrng((char *)".MOD", filename))
        strcat(SearchFile, "mod/");
    if (InStrng((char *)".jpg", filename))
        strcat(SearchFile, "images/");
    strcat(SearchFile, filename);
}
/****************************************************************************************************************************/
File OpenTextFileForReading()
{
    AddPath(TextFileName);
    File fnumber = TINYCARD.open(SearchFile, FILE_READ);
    if (fnumber)
        LogFileOpen = true;
    return fnumber;
}
/************************************************************************************************************/
FASTRUN void OpenLogFileW()
{
    if (!LogFileOpen)
    {
        AddPath(TextFileName);
        LogFileNumber = TINYCARD.open(SearchFile, FILE_WRITE);
        LogFileOpen = true;
    }
}
/************************************************************************************************************/
FASTRUN void DeleteLogFile()
{
    AddPath(TextFileName);
    TINYCARD.remove(SearchFile);
}
/******************************************************************************************************************************/

void DeleteThisLogFile()
{
    char FileBox[] = "FilesBox";
    char prompt[] = "Delete ";
    char pLogFiles[] = "page LogFiles";
    char Query[] = "?";
    char pprompt[80];
    strcpy(TextFileName, TheFilesList[GetValue(FileBox)]);
    strcpy(pprompt, prompt);
    strcat(pprompt, TextFileName);
    strcat(pprompt, Query);
    if (GetConfirmation(pLogFiles, pprompt))
    {
        AddPath(TextFileName);
        TINYCARD.remove(SearchFile);
        strcpy(MOD, ".LOG");
        BuildDirectory();
        strcpy(Mfiles, "FilesBox");
        LoadFileSelector();
    }
}
// ******************************************************************************************************************************/
/******************************************************************************************************************************/

uint32_t GetFileSize(char *filename)
{

    AddPath(filename);
    File file = TINYCARD.open(SearchFile, FILE_READ);
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
        AddPath(SingleModelFile);
        ModelsFileNumber = TINYCARD.open(SearchFile, FILE_WRITE);
        ModelsFileNumber.seek(0);
        ShortishDelay();
        ModelsFileNumber.write(NewFileBuffer, NewFileBufferPointer);
        ShortishDelay();
        ModelsFileNumber.close();
        ShortishDelay();
    }
}

/******************************************************************************************************************************/

void ShowFreeSpaceEtc()
{

    float FreeSpaceOnSD = ((float)TINYCARD.totalSize() - (float)TINYCARD.usedSize()) / ((float)(1024 * 1024 * 1024));
    float UsedSpaceOnSD = (float)TINYCARD.usedSize() / ((float)(1024 * 1024));

    char t4[] = "t4";
    char t5[] = "t5";
    char t6[] = "t6";
    char NB[20];
    char Gbytes[] = " GB";
    char Mbytes[] = " MB";
    char *Bbytes = Gbytes;

    dtostrf((float)TINYCARD.totalSize() / (float)(1024 * 1024 * 1024), 2, 2, NB);
    AddSpacesBefore(NB, 5);
    strcat(NB, Gbytes);
    SendText(t4, NB);
    dtostrf(FreeSpaceOnSD, 2, 2, NB);
    AddSpacesBefore(NB, 5);
    strcat(NB, Gbytes);
    SendText(t6, NB);

    if (UsedSpaceOnSD >= 100) // less than 100 MB?
    {
        UsedSpaceOnSD /= (float)1024.00;
    }
    else
    {
        Bbytes = Mbytes; // use MB not GB
    }
    dtostrf(UsedSpaceOnSD, 2, 2, NB);
    AddSpacesBefore(NB, 5);
    strcat(NB, Bbytes);
    SendText(t5, NB);
}

/******************************************************************************************************************************/

// This implements the impossible "TINYCARD card rename file" ... by reading, re-saveing under new name, then deleting old file.

void RenameFile()
{
    char ModelsView_filename[] = "filename";
    char Head[] = "Rename this backup";
    char model[] = "(i.e. just change its filename)";
    char prompt[] = "New filename?";
    char Prompt[50];
    char overwr[] = "Overwrite ";
    char ques[] = "?";
    char Deleteable[42];

    SaveCurrentModel();
    GetText(ModelsView_filename, SingleModelFile);
    strcpy(Deleteable, SingleModelFile);
    LoadModelForRenaming();
    if (GetBackupFilename(pModelsView, SingleModelFile, model, Head, prompt))
    {
        FixFileName();
        if (strcmp(Deleteable, SingleModelFile) == 0)
            return;
        Serial.println(SingleModelFile);
        if (CheckFileExists(SingleModelFile))
        {
            strcpy(Prompt, overwr);
            strcat(Prompt, SingleModelFile);
            strcat(Prompt, ques);
            if (GetConfirmation(pModelsView, Prompt))
            {
                WriteBackup();
                TINYCARD.remove(Deleteable);
            }
        }
        else
        {
            WriteBackup();
            TINYCARD.remove(Deleteable);
        }
    }
    strcpy(MOD, ".MOD");
    BuildDirectory();
    strcpy(Mfiles, "Mfiles");
    LoadFileSelector();
    RestoreCurrentModel();
}

/*********************************************************************************************************************************/

bool CheckFileExists(char *fl)
{
    CloseModelsFile();
    bool exists = false;
    File t;
    AddPath(fl);
    t = TINYCARD.open(SearchFile, FILE_READ);
    if (t)
        exists = true;
    t.close();
    return exists;
}
/*********************************************************************************************************************************/

void OpenModelsFile()
{

    char ModelsFile[] = "models.dat";

    if (!ModelsFileOpen)
    {
        if (SingleModelFlag)
        {
            AddPath(SingleModelFile);
            ModelsFileNumber = TINYCARD.open(SearchFile, FILE_WRITE);
            DelayWithDog(100);
        }
        else
        {
            ModelsFileNumber = TINYCARD.open(ModelsFile, FILE_WRITE);
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
// *********************************************************************************************************************************/
void CheckSDCard()
{
    char err_404[] = "SD card error!";
    char err_405[] = "or not found!";
    bool SDCARDOK = TINYCARD.begin(BUILTIN_SDCARD); // MUST return true or SD card is not working
    if (!SDCARDOK)
    {
        delay(1000); // allow screen to warm up
        SendCommand(pFrontView);
        delay(70);
        RestoreBrightness();
        delay(70);
        SendCommand(WarnNow);
        for (uint8_t i = 0; i < 4; ++i)
        {
            SendText(Warning, err_404);
            delay(1000);
            SendText(Warning, err_405);
            delay(1000);
            PlaySound(WHAHWHAHMSG);
        }
        digitalWrite(POWER_OFF_PIN, HIGH); // turn off power now!
    }
}
// *********************************************************************************************************************************/
void DeleteMODfile(int p)
{
    int j = 0;
    char prompt[60];
    int i = p + 6;
    char ques[] = "?";
    char del[] = "Delete ";
    while (uint8_t(TextIn[i]) > 0)
    {
        SingleModelFile[j] = TextIn[i];
        ++j;
        ++i;
        SingleModelFile[j] = 0;
    }
    strcpy(prompt, del);
    strcat(prompt, SingleModelFile);
    strcat(prompt, ques);
    if (GetConfirmation(pModelsView, prompt))
    {
        AddPath(SingleModelFile);
        TINYCARD.remove(SearchFile);
        strcpy(MOD, ".MOD");
        BuildDirectory();
        strcpy(Mfiles, "Mfiles");
        LoadFileSelector();
        --FileNumberInView;
        ShowFileNumber();
    }
    CloseModelsFile();
    ClearText();
}
/*********************************************************************************************************************************/
void BuildDirectory() // HEER
{
    char Entry1[30];
    char fn[20];
    int i = 0;
    File dir;

    if ((strcmp(MOD, ".LOG") == 0))
        dir = TINYCARD.open("/log/");
    if ((strcmp(MOD, ".TXT") == 0))
        dir = TINYCARD.open("/help/");
    if ((strcmp(MOD, ".MOD") == 0))
        dir = TINYCARD.open("/mod/");

    ExportedFileCounter = 0;
    while (true)
    {
        File entry = dir.openNextFile();
        if (!entry || ExportedFileCounter > MAXBACKUPFILES)
            break;
        strcpy(Entry1, entry.name());
        if ((InStrng(MOD, Entry1) > 0) && (!InStrng((char *)"._", Entry1)))
        {
            strcpy(fn, entry.name());
            for (i = 0; i < 12; ++i)
            {
                TheFilesList[ExportedFileCounter][i] = fn[i];
            }
            ExportedFileCounter++;
        }
        entry.close();
    }
    SortDirectory();
}
#endif // SD_FILES_H