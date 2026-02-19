// ********************** ChooseImage.h ***********************************************
// This file contains the functions to Choose the model image
// ******************************************************************************

#include <Arduino.h>
#include "1Definitions.h"
#include <SPI.h>

#ifndef CHOOSEIMAGE_H
#define CHOOSEIMAGE_H
// *********************************************************************************************************************************/
char *GetModelImageFileName(char *fname)
{
    FileNumberInView = GetValue((char *)"MMems");
    for (int i = 0; i < 12; ++i)
    {
        fname[i] = TheFilesList[FileNumberInView][i];
        if (TheFilesList[FileNumberInView][i + 1] == '.')
        {
            fname[i + 1] = 0;
            break;
        }
    }
    return fname;
}

// *************************************************************************************************************************/

void ImageScrollStop() // finger lifted from the screen, so stop scrolling and show the currently selected image
{
    DisplayModelImage();        // show the currently selected image
    for (int i = 0; i < 5; ++i) // display the image repeatedly for a second as it's often slow to respond.
    {
        DelayWithDog(100);                         // delay for a 0.1 second while kicking the dog and checking for power off button
        GetModelImageFileName(ModelImageFileName); // get the currently selected image file name
        DisplayModelImage();                       // show the currently selected image
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
