
// *************************************** STABILISATION.h  *****************************************



#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifndef STABILISATION_H
#define STABILISATION_H

#ifdef USE_STABILISATION

#define GAINS_PIN 24 // the only free pin with ADC
/******************************************************************************************************************************/
void init_gains_pin()
{
    pinMode(GAINS_PIN, INPUT); // this is the pin used for adjusting gains.
}
/*******************************************************************************************************************************/
void ReadGainsKnob(){
  //  uint16_t G = adc->analogRead(GAINS_PIN);
   //   Look(G);
}
// *******************************************************************************************************************************/
// This function is called from main.cpp to check the state of the stabilisation and self-
void CheckStabilisationAndSelf_levelling(){
  static bool PreviousStabilisedState;
  static bool PreviousLevelledState;

  if (Bank == StabilisedBank || StabilisedBank == 0)
  {
    {                               // turn on stabilisation
      if (!PreviousStabilisedState) // unless already on
      {
        PreviousStabilisedState = true;
        SwitchStabilisation(PreviousStabilisedState);
      }
      {
        PreviousStabilisedState = true;
        SwitchStabilisation(PreviousStabilisedState);
      }
    }
  }
  else
  {
    if (PreviousStabilisedState) // turn off stabilisation if it was on
    {
      PreviousStabilisedState = false;
      SwitchStabilisation(PreviousStabilisedState);
    }
  }

  if (Bank == LevelledBank || LevelledBank == 0)
  {                             // turn on levelling
    if (!PreviousLevelledState) // unless already on
    {
      PreviousLevelledState = true;
      SwitchLevelling(PreviousLevelledState);
    }
  }
  else
  { // turn off levelling if it was on
    if (PreviousLevelledState)
    {
      PreviousLevelledState = false;
      SwitchLevelling(PreviousLevelledState);
    }
  }
}

/*******************************************************************************************************************************/
#endif // USE_STABILISATION
#endif // STABILISATION_H
