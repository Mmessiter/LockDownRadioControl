
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

/*******************************************************************************************************************************/
#endif // USE_STABILISATION
#endif // STABILISATION_H
