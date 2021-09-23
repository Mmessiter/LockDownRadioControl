
#ifndef Compress_h
#define Compress_h
#include "Arduino.h"

class Compress                       
{
	public:
		void DeComp(uint16_t *d, uint16_t *c,int cc);
		void Comp(uint16_t *c, uint16_t *o, int cc);
	private:
  		
};
#endif