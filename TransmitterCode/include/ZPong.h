
#include <Arduino.h>
#include "1Definitions.h"
#ifndef ZPONG_H
    #define ZPONG_H
   
    
/*******************************************************************************************************************************/
//                                                  ZPONG.h
/*******************************************************************************************************************************/

bool GameEnd(){
        PlaySound(THEFANFARE);
        DelayWithDog(6000); 
        return true; // = play again ...//
}
/*******************************************************************************************************************************/

void StartPong()
{
    if (LedWasGreen) { // if connected to model, don't start pong
        char no[] = "Please disconnect first!";
        MsgBox(pFrontView,no);
        GotoFrontView();
        return;
    }
    SendCommand(pPongView);
    SetDefaultValues();
    CurrentView = PONGVIEW;
    CurrentMode = PONGMODE;
    BlueLedOn();
    DrawBox(PONGX1, PONGY1, PONGX2, PONGY2, ForeGroundColour);    // Draw court
    DrawLine(PONGX1, GOALTOP, PONGX1, GOALBOT, BackGroundColour); // Make goal openings
    DrawLine(PONGX2, GOALTOP, PONGX2, GOALBOT, BackGroundColour); // Make goal openings
}
/*********************************************************************************************************************************/
void MoveBall(int x, int y)
{
    static int lastx = 0;
    static int lasty = 0;
    DrawDot(lastx, lasty, PONGBALLSIZE + 1, BackGroundColour); // Clear last position
    DrawDot(x, y, PONGBALLSIZE, HighlightColour);              // Draw new position
    lastx = x;                                                 // save last position
    lasty = y;                                                 // save last position
}
/*********************************************************************************************************************************/
void MovePaddles(int ly, int ry)
{
    DrawLine(LEFTPADDLEX, PONGY1 + 3, LEFTPADDLEX, PONGY2 - 3, BackGroundColour);
    DrawLine(LEFTPADDLEX, ly - (PADDLEHEIGHT / 2), LEFTPADDLEX, ly + (PADDLEHEIGHT / 2), ForeGroundColour);
    DrawLine(RIGHTPADDLEX, PONGY1 + 3, RIGHTPADDLEX, PONGY2 - 3, BackGroundColour);
    DrawLine(RIGHTPADDLEX, ry - (PADDLEHEIGHT / 2), RIGHTPADDLEX, ry + (PADDLEHEIGHT / 2), ForeGroundColour);
}
/*********************************************************************************************************************************/

void PlayPong()
{ // called 100 times per second
    static uint32_t  Ponged          = 0;
    static short int x               = STARTX;
    static short int y               = STARTY;
    static short int LeftPaddlY      = STARTY;
    static short int RightPaddlY     = STARTY;
    static short int PrevLeftPaddlY  = STARTY;
    static short int PrevRightPaddlY = STARTY;

    static short int incy       = PONGBALLSPEED;
    static short int incx       = PONGBALLSPEED;
    static short int LeftScore  = 0;
    static short int RightScore = 0;
    char             n0[]       = "n0"; // score labels
    char             n1[]       = "n1"; // score labels
    uint8_t          lstk       = 1;    // left stick input
    uint8_t          rstk       = 2;    // right stick input

    if (CurrentView == HELP_VIEW || CurrentView == LOGVIEW) return;
    if ((millis() - Ponged) < PONGSPEED) return;
    Ponged = millis();
    y += incy;
    x += incx;
    if (SticksMode == 2) swap(&lstk, &rstk);
    LeftPaddlY  = map(PreMixBuffer[lstk], MINMICROS, MAXMICROS, PONGY2 + EXTRAPONG, PONGY1 - EXTRAPONG);
    RightPaddlY = map(PreMixBuffer[rstk], MINMICROS, MAXMICROS, PONGY2 + EXTRAPONG, PONGY1 - EXTRAPONG);
    MovePaddles(LeftPaddlY, RightPaddlY);
    if ((x <= LEFTPADDLEX + PONGCLEAR) && (x >= LEFTPADDLEX - PONGCLEAR)) { // hit left paddle?
        if ((y >= (LeftPaddlY - (PADDLEHEIGHT / 2))) && (y <= (LeftPaddlY + (PADDLEHEIGHT / 2)))) {
            incx = -incx;
            x    = LEFTPADDLEX + PONGCLEAR + 10;
            randomSeed(micros());
            if (PrevLeftPaddlY > LeftPaddlY) incy = -random(PONGBALLSPEED);
            if (PrevLeftPaddlY < LeftPaddlY) incy = random(PONGBALLSPEED);
            PlaySound(BEEPMIDDLE);
        }
    }
    PrevLeftPaddlY = LeftPaddlY;
    if ((x <= (RIGHTPADDLEX + PONGCLEAR)) && (x >= (RIGHTPADDLEX - PONGCLEAR))) { // hit right paddle?
        if ((y >= (RightPaddlY - (PADDLEHEIGHT / 2))) && (y <= (RightPaddlY + (PADDLEHEIGHT / 2)))) {
            incx = -incx;
            x    = (RIGHTPADDLEX - PONGCLEAR) - 10;
            randomSeed(micros());
            if (PrevRightPaddlY > RightPaddlY) incy = -random(PONGBALLSPEED);
            if (PrevRightPaddlY < RightPaddlY) incy = random(PONGBALLSPEED);
            PlaySound(BEEPMIDDLE);
        }
    }
    PrevRightPaddlY = RightPaddlY;
    if ((y + PONGCLEAR) >= PONGY2) { // bounce off bottom
        incy = -incy;
        y    = (PONGY2 - PONGCLEAR) - 10;
        PlaySound(CLICKZERO);
    }
    if (y <= (PONGY1 + PONGCLEAR)) { // bounce off top
        incy = -incy;
        y    = PONGY1 + PONGCLEAR + 10;
        PlaySound(CLICKZERO);
    }
    if (((x + PONGCLEAR) >= PONGX2) && (x)) {
        if ((y < GOALBOT - 2) && (y > GOALTOP + 2)) { // scored on the right?
            ++RightScore;
            SendValue(n0, RightScore);
            if (RightScore >= 10) {
               if (GameEnd()) {
                    RightScore = 0;
                    LeftScore  = 0;
                    SendValue(n0, RightScore);
                    SendValue(n1, LeftScore);
                } else {
                    GotoFrontView();
                }
                
            }
            x = PONGX2 + 5;
            MoveBall(x, y);
            PlaySound(BEEPCOMPLETE);
            DelayWithDog(500);
            x = -STARTX;
            y = -STARTY;
            randomSeed(micros());
            incx = -PONGBALLSPEED + random(PONGBALLSPEED * 2);
            while (abs(incx) < 3) incx = -PONGBALLSPEED + random(PONGBALLSPEED * 2);
            incy = PONGBALLSPEED - random(PONGBALLSPEED * 2);
        }
        else {
            incx = -incx;
            PlaySound(CLICKZERO);
        }
    }
    if ((x <= (PONGX1 + PONGCLEAR)) && (x)) { // scored on the left?
        if ((y < GOALBOT - 2) && (y > GOALTOP + 2)) {
            ++LeftScore;
            x = PONGX1 - 5;
            MoveBall(x, y);
            PlaySound(BEEPCOMPLETE);
            SendValue(n1, LeftScore);
            if (LeftScore >= 10) {
                if (GameEnd()) {
                    RightScore = 0;
                    LeftScore  = 0;
                    SendValue(n0, RightScore);
                    SendValue(n1, LeftScore);
                } else {
                     GotoFrontView();
                }
                   

            }
            DelayWithDog(500);
            x = -STARTX;
            y = -STARTY;
            randomSeed(micros());
            incx = PONGBALLSPEED - random(PONGBALLSPEED * 2);
            while (abs(incx) < 3) incx = PONGBALLSPEED - random(PONGBALLSPEED * 2);
            incy = PONGBALLSPEED - random(PONGBALLSPEED * 2);
        }
        else {
            incx = -incx;
            PlaySound(CLICKZERO);
        }
    }
    MoveBall(x, y);
    NewCompressNeeded = false;
    SendValue(n0, RightScore);
    SendValue(n1, LeftScore);
    DrawLine(PONGX1 + ((PONGX2 - PONGX1) / 2), PONGY1, PONGX1 + ((PONGX2 - PONGX1) / 2), PONGY2, Gray);
}

/****************************************************************************************************************/
#endif