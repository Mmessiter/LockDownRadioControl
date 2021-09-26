
#ifndef _TRANSMITTERCODE_LIB_COMPRESS_H
#define _TRANSMITTERCODE_LIB_COMPRESS_H
#include <stdint.h>

class Compress {
public:
    void DeComp(uint16_t* d, uint16_t* c, int cc);
    void Comp(uint16_t* c, uint16_t* o, int cc);

private:
};
#endif
