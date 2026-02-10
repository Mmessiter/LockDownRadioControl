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

    if (InStrng(txt, TextFileName))
    {
        strcpy(MOD, ".TXT");
        SendCommand(b15off); // no deleting of help files!
        SendCommand(b1off);  // no help if viewing help already
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
    strcpy(TextFileName, TheFilesList[GetValue(FileBox)]); // Get the selected file name
    ReadingaFile = true;
    LogVIEWNew(); // Start the log screen
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
