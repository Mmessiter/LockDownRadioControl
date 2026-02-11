// ********************** ChooseImage.h ***********************************************
// This file contains the functions to Choose the model image
// ******************************************************************************

#include <Arduino.h>
#include "1Definitions.h"
#include <SPI.h>

#ifndef CHOOSEIMAGE_H
#define CHOOSEIMAGE_H
// *********************************************************************************************************************************/
void ShowCurrentlySelectedImage()
{
    char newfname[20];
    FileNumberInView = GetValue((char *)"MMems");
    for (int i = 0; i < 12; ++i)
    {
        newfname[i] = TheFilesList[FileNumberInView][i];
        if (TheFilesList[FileNumberInView][i + 1] == '.')
        {
            newfname[i + 1] = 0;
            break;
        }
    }
    strcpy(ModelImageFileName, newfname);
    CheckModelImageFileName();
    DisplayModelImage();
}

// *************************************************************************************************************************/

void ImageScrollStop() // finger lifted from the screen, so stop scrolling and show the currently selected image
{
    for (int i = 0; i < 5; ++i) // display the image repeatedly for a second as it's often slow to respond.
    {
        ShowCurrentlySelectedImage();
        DelayWithDog(10);
    }
}

// *************************************************************************************************************************/
uint16_t GetThisImageNumber()
{
    char temp[30];
    for (int j = 0; j < 89; ++j)
    {
        for (int i = 0; i < 12; ++i)
        {
            temp[i] = TheFilesList[j][i];
            if (TheFilesList[j][i + 1] == '.')
            {
                temp[i + 1] = 0;
                break;
            }
        }
        if (strcmp(temp, ModelImageFileName) == 0)
            return j;
    }
    return 0;
}

// *************************************************************************************************************************/
void StartChooseImage()
{
    SendCommand((char *)"page ImageView");
    CurrentView = CHOOSEIMAGEVIEW;
    strcpy(Mfiles, "MMems"); // Set the file box name
    strcpy(MOD, ".jpg");
    BuildDirectory();
    LoadFileSelector();
    SendText((char *)"t0", ModelName);
    SendValue((char *)"MMems", GetThisImageNumber());
    ImageScrollStop();
}
// *********************************************************************************************************************************/
void EndChooseImage()
{
    SaveOneModel(ModelNumber);
    RXOptionsViewStart();
}

#endif
