// This file contains the code for the LogFilesList screen which is used to display the list of log files and to delete them.

#ifndef LOGFILESLIST_H
#define LOGFILESLIST_H
#include <Arduino.h>
#include "1Definitions.h"

/******************************************************************************************************************************/

void ClearFilesList()
{
    for (int i = 0; i < 99; i++)
    {
        strcpy(TheFilesList[i], "");
    }
}
/******************************************************************************************************************************/
void StartLogFilesListScreen()
{
    // This function shows a directory of all log files OR all help files.
    // It allows deleting or viewing log files
    // it allows viewing of any help file.
    
    
    char pLogFiles[] = "page LogFiles";
    char prompt[] = "Please disconnect first!";
    char txt[] = ".TXT";
    char b15off[] = "vis b15,0";
    char b1off[] = "vis b1,0";
    char t0[] = "t0";
    char LogfilesTitle[] = "All log files";
    char HelpFilesTitle[] = "All help files";

    if (LedWasGreen)
    {                             // Check if the device is connected
        MsgBox(pLogView, prompt); // Display a message box and return
        return;
    }
    SendCommand(pLogFiles);         // Go to the LogFiles screen
    CurrentView = LOGFILESLISTVIEW; // Set the current view to LogFilesList
    DelayWithDog(70);

    if (InStrng(txt, LogFileName))
    {
        strcpy(MOD, ".TXT");
        SendCommand(b15off);  // no deleting of help files!
        SendCommand(b1off);   // no help if viewing help already 
        SendText(t0, HelpFilesTitle);
    }
    else
    {
        strcpy(MOD, ".LOG"); // Set the file extension to .LOG
        SendText(t0, LogfilesTitle);
    }
    BuildDirectory();           // Build the directory
    strcpy(Mfiles, "FilesBox"); // Set the file box name
    SavedCurrentView = LOGVIEW; // to stop it going round for ever!
    LoadFileSelector();         // Load the file selector
}
/******************************************************************************************************************************/

void EndLogFilesListScreen()
{
    LogVIEWNew();
}

/******************************************************************************************************************************/
void LoadNewLogFile()
{
    char FileBox[] = "FilesBox";
    strcpy(LogFileName, TheFilesList[GetValue(FileBox)]); // Get the selected file name
    ReadingaFile = true;
    LogVIEWNew(); // Start the log screen
}
/******************************************************************************************************************************/

void DeleteThisLogFile()
{
    char FileBox[] = "FilesBox";
    char prompt[] = "Delete ";
    char pLogFiles[] = "page LogFiles";
    char Query[] = "?";
    char pprompt[80];
    strcpy(LogFileName, TheFilesList[GetValue(FileBox)]);
    strcpy(pprompt, prompt);
    strcat(pprompt, LogFileName);
    strcat(pprompt, Query);
    if (GetConfirmation(pLogFiles, pprompt))
    {
        SD.remove(LogFileName);
        strcpy(MOD, ".LOG");
        BuildDirectory();
        strcpy(Mfiles, "FilesBox");
        LoadFileSelector();
    }
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

/******************************************************************************************************************************/

char *AddSizeToFilename(int ff, char *size)
{
    char nb[20];
    char ThisFile[20];
    strcpy(ThisFile, TheFilesList[ff]);
    while (strlen(TheFilesList[ff]) < 12)
        strcat(TheFilesList[ff], " ");
    float s = (float)GetFileSize(ThisFile) / (float)1024.00;
    dtostrf(s, 2, 2, nb);
    while ((strlen(size) + strlen(nb)) < 7)
        strcat(size, " ");
    strcat(size, nb);
    strcat(size, " KB");
    return size;
}

/******************************************************************************************************************************/

char *AddSpacesBefore(char *s, uint8_t n)
{
    char temp[20];
    strcpy(temp, "");
    while ((strlen(temp) + strlen(s)) < n)
        strcat(temp, " ");
    strcat(temp, s);
    strcpy(s, temp);
    return s;
}
/******************************************************************************************************************************/

void ShowFreeSpaceEtc()
{

    float FreeSpaceOnSD = ((float)SD.totalSize() - (float)SD.usedSize()) / ((float)(1024 * 1024 * 1024));
    float UsedSpaceOnSD = (float)SD.usedSize() / ((float)(1024 * 1024));

    char t4[] = "t4";
    char t5[] = "t5";
    char t6[] = "t6";
    char NB[20];
    char Gbytes[] = " GB";
    char Mbytes[] = " MB";
    char *Bbytes = Gbytes;

    dtostrf((float)SD.totalSize() / (float)(1024 * 1024 * 1024), 2, 2, NB);
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

void LoadFileSelector()
{
    char Mfilesp[20];
    char crlf[] = {13, 10, 0};
    char buf[MAXBUFFERSIZE];
    char nofiles[] = "(No files)";
    char SizeBuf[20];

    strcpy(Mfilesp, Mfiles);
    strcat(Mfilesp, ".path=\"");
    strcpy(buf, nofiles);
    for (int f = 0; f < ExportedFileCounter; ++f)
    {
        strcpy(SizeBuf, "");
        if (strcmp(MOD, ".MOD"))
            AddSizeToFilename(f, SizeBuf);
        if (!f)
            strcpy(buf, TheFilesList[f]);
        else
            strcat(buf, TheFilesList[f]); // first one? if not, concatenate
        if (strcmp(MOD, ".MOD"))
            strcat(buf, SizeBuf);
        strcat(buf, crlf);
    }
    SendOtherText(Mfilesp, buf);
    FileNumberInView = GetValue(Mfiles);
    LastFileInView = 120;
    if (strcmp(MOD, ".MOD"))
        ShowFreeSpaceEtc(); // show free space etc if log files are being viewed
}

#endif // LOGFILESLIST_H
