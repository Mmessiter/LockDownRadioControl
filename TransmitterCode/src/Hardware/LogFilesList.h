// This file contains the code for the LogFilesList screen which is used to display the list of log files and to delete them.

#ifndef LOGFILESLIST_H
#define LOGFILESLIST_H
#include <Arduino.h>
#include "Hardware/1Definitions.h"


/******************************************************************************************************************************/

void ClearFilesList(){
    for (int i = 0; i < 99; i++) {
        strcpy(TheFilesList[i], "");
    }
}
/******************************************************************************************************************************/
void StartLogFilesListScreen(){
    char pLogFiles[] = "page LogFiles";//
    char prompt[]    = "Please disconnect first!";
    if (LedWasGreen) {                  // Check if the device is connected
        MsgBox(pLogView, prompt);       // Display a message box and return
       return;
    }
    SendCommand(pLogFiles);             // Go to the LogFiles screen
    CurrentView = LOGFILESLISTVIEW;     // Set the current view to LogFilesList
    strcpy (MOD,".LOG");                // Set the file extension to .LOG
    BuildDirectory();                   // Build the directory
    strcpy(Mfiles,"FilesBox");          // Set the file box name
    LoadFileSelector();                 // Load the file selector
}
/******************************************************************************************************************************/

void EndLogFilesListScreen(){
     LogVIEW();
}
/******************************************************************************************************************************/
void LoadNewLogFile(){
    char FileBox[] = "FilesBox";
    char sw0[] = "sw0";
    strcpy (LogFileName,TheFilesList[GetValue(FileBox)]);       // Get the selected file name
    SendCommand(pLogView);
    SendValue(sw0, UseLog);
    CurrentView = LOGVIEW;
    ShowLogFile(0);                                             // Display the selected log file         
}
/******************************************************************************************************************************/

void DeleteThisLogFile(){
    char FileBox[] = "FilesBox";
    char prompt[]   = "Delete ";
    char pLogFiles[] = "page LogFiles";
    char Query[] = "?";
    char pprompt[80];
    strcpy (LogFileName,TheFilesList[GetValue(FileBox)]);
    strcpy(pprompt,prompt);
    strcat(pprompt,LogFileName);
    strcat(pprompt,Query);
    if (GetConfirmation(pLogFiles, pprompt)) {
        SD.remove(LogFileName);
        strcpy (MOD,".LOG");
        BuildDirectory(); // heer
        strcpy(Mfiles,"FilesBox");
        LoadFileSelector();
    }
}


/******************************************************************************************************************************/

uint32_t GetFileSize(char * filename)
{
    File file = SD.open(filename, FILE_READ);
    if (!file) return 0;
    uint32_t size = file.size();
    file.close();
    return size;
}

/******************************************************************************************************************************/

char * AddSizeToFilename(int ff, char * size){
    char nb[20];
    if (strcmp(MOD, ".LOG") != 0) 
    return size;
    float s = (float) GetFileSize(TheFilesList[ff]) /  (float)1024.00;
    dtostrf(s, 2, 2, nb);
    while ((strlen(size) + strlen (nb)) < 7 ) strcat(size, " ");
    strcat(size, nb); 
    strcat(size, " KB");
    return size;
}

/******************************************************************************************************************************/

char * AddSpacesBefore(char * s, uint8_t n){
    char temp[20];
    strcpy(temp, "");
    while ((strlen(temp) + strlen(s)) < n) strcat(temp, " ");
    strcat(temp, s);
    strcpy(s, temp);
    return s;
}
/******************************************************************************************************************************/

void   ShowFreeSpaceEtc(){

    float FreeSpaceOnSD = (SD.totalSize() - SD.usedSize()) / ((float) (1024 * 1024 * 1024));
    float UsedSpaceOnSD = SD.usedSize() / ((float) (1024 * 1024 * 1024));
    
    char t4[] = "t4";
    char t5[] = "t5";
    char t6[] = "t6";
    char NB[20];
    char Gbytes[] = " GB";

    dtostrf(SD.totalSize() / (float) (1024 * 1024 * 1024), 2, 2, NB);
    AddSpacesBefore(NB, 6);
    strcat(NB, Gbytes);
    SendText(t4, NB);
    dtostrf(FreeSpaceOnSD, 2, 2, NB);
    AddSpacesBefore(NB, 6);
    strcat(NB, Gbytes);
    SendText(t6, NB);  
    dtostrf(UsedSpaceOnSD, 2, 2, NB);
    AddSpacesBefore(NB, 6);
    strcat(NB, Gbytes);
    SendText(t5, NB);
}

/******************************************************************************************************************************/

void LoadFileSelector()
{
    char Mfilesp[20];
    char crlf[]    = {13, 10, 0};
    char buf[MAXBUFFERSIZE];
    char nofiles[] = "(No files)";
    char SizeBuf[20];

    strcpy(Mfilesp, Mfiles);
    strcat(Mfilesp, ".path=\"");
    strcpy(buf, nofiles);
    for (int f = 0; f < ExportedFileCounter; ++f) {
        strcpy(SizeBuf, "");
        if (!strcmp(MOD, ".LOG")) AddSizeToFilename(f, SizeBuf);
        if (!f) strcpy(buf, TheFilesList[f]); else strcat(buf, TheFilesList[f]); // first one? if not, concatenate
        if (!strcmp(MOD, ".LOG")) strcat(buf, SizeBuf);
        strcat(buf, crlf);
    }
    SendOtherText(Mfilesp, buf);
    FileNumberInView = GetValue(Mfiles);
    LastFileInView   = 120;
    if (!strcmp(MOD, ".LOG")) ShowFreeSpaceEtc(); // show free space etc if log files are being viewed
}


#endif // LOGFILESLIST_H
