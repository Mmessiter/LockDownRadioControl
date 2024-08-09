
// This file displays the help screens and log screens
// (Log files use the same functions)

#ifndef HELP_H
#define HELP_H

#include <Arduino.h>
#include "Hardware/1Definitions.h"



/******************************************************************************************************************************/
void UpLog()
{
    if (RecentStartLine > 0 && (CurrentView == LOGVIEW)) {
        RecentStartLine -= ScrollAmount;
        if (RecentStartLine < 0) RecentStartLine = 0;
            MakeLogFileName();  
            ShowLogFile(RecentStartLine);
    }
    if (RecentStartLine > 0 && (CurrentView == HELP_VIEW)) {
        RecentStartLine -= ScrollAmount;
        if (RecentStartLine < 1) RecentStartLine = 0;
        ScrollHelpFile();//
    }
}
/******************************************************************************************************************************/
void DownLog()
{

    if (ThereIsMoreToSee && (CurrentView == LOGVIEW)) {
        RecentStartLine += ScrollAmount;
            MakeLogFileName();  
            ShowLogFile(RecentStartLine);
    }
    if (ThereIsMoreToSee && (CurrentView == HELP_VIEW)) {
        RecentStartLine += ScrollAmount;
        ScrollHelpFile();
    }
}




/*********************************************************************************************************************************/
void ScrollHelpFile()
{                                   // redisplays help file to scroll it ... maybe from top of file
    char HelpText[MAXFILELEN + 10]; // MAX = 3K or so
    char HelpView[] = "HelpText";
    ReadTextFile(RecentTextFile, HelpText, RecentStartLine, MAXLINES); // Then load help text
    SendText1(HelpView, HelpText);                                     // Then send it
}
/*********************************************************************************************************************************/
void SendHelp()
{ // load new help file
   
    char HelpFile[20];
    int  i = 9;
    int  j = 0;
    SendCommand(pHelpView); // first load the right screen
    while (TextIn[i] != 0 && j < 19) {
        HelpFile[j] = TextIn[i];
        ++i;
        ++j;
        HelpFile[j] = 0;
    }
    strcpy(RecentTextFile, HelpFile);
    RecentStartLine = 0;
    ScrollHelpFile();
}
#endif
