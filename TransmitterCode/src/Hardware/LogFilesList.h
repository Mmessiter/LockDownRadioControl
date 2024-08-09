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
    UseLog = false;                     // Set the UseLog flag to false
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
    UseLog = false;                                             // Set the UseLog flag to false
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
#endif // LOGFILESLIST_H
