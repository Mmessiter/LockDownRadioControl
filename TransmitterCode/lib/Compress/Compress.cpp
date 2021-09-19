
#include "Arduino.h"
#include "Compress.h"


// **** Decompresses cc*3/4 x 16 BIT values up to cc loading only their low 12 BITS cc must be divisble by 4! ******************
void Compress::DeComp(uint16_t *d, uint16_t *c,int cc){ 
int p=0,l=0;for (l=0;l<(cc*3/4);l+=3){d[p]=c[l]>>4;p++;d[p]=(c[l]&0xf)<<8|c[l+1]>>8;p++;d[p]=(c[l+1]&0xff)<<4|c[l+2]>>12;p++;d[p]=c[l+2]&0xfff;p++;}}


// **** Compresses cc x 16 BIT 'o' values down to only cc*3/4 'c' values by using only their low 12 BITS... cc *must* be divisible by 4! ********
void Compress::Comp(uint16_t *c, uint16_t *o, int cc){ 
int p=0,l=0;for (l=0;l<(cc*3/4);l+=3){c[l]=o[p]<<4|o[p+1]>>8;p++;c[l+1]=o[p]<<8|o[p+1]>>4;p++;c[l+2]=o[p]<<12|o[p+1];p++;p++;}}
