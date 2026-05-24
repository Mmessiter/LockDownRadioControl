// LockDownRadioControl — RXV2  ::  Channels.h
//
// Channel decompression + decode. Mirrors v1's Decompress() +
// RearrangeTheChannels() pipeline. The 16 channel values land in
// channelMicros[] (declared in 1Defs.h) as 12-bit microsecond counts
// (500..2500); the output frame builders in Output.h map to their respective
// wire formats from there.
//
//*********************************************************************

#ifndef _SRC_CHANNELS_H
#define _SRC_CHANNELS_H

#include "1Defs.h"

//*********************************************************************
//  3:4 packed decompression
//*********************************************************************
// Mirrors v1's Decompress(): 3 uint16_t input → 4 12-bit output.

inline void decompress(uint16_t* out, const uint16_t* in, uint8_t outSize) {
    uint8_t compressedSize = (outSize * 3) / 4;
    uint8_t p = 0;
    for (uint8_t i = 0; i + 2 < compressedSize; i += 3) {
        uint16_t w0 = in[i], w1 = in[i + 1], w2 = in[i + 2];
        out[p++] = w0 >> 4;
        out[p++] = ((w0 & 0x0F) << 8) | (w1 >> 8);
        out[p++] = ((w1 & 0xFF) << 4) | (w2 >> 12);
        out[p++] = w2 & 0x0FFF;
    }
}

//*********************************************************************
//  Number of 12-bit values to expect from a given raw payload size
//*********************************************************************
// Mirrors v1 GetDecompressedSize(). Payload = ChannelBitMask (2 bytes) + 3:4
// packed channel data.

inline uint8_t decompressedSize(uint8_t payloadBytes) {
    if (payloadBytes <= 2) return 0;
    uint8_t ds = (((uint16_t)(payloadBytes - 2) * 4) / 3) / 2;
    while (ds % 4) ++ds;
    return ds;
}

//*********************************************************************
//  Decode a received packet into channelMicros[]
//*********************************************************************
// Mirrors v1's Decompress() + RearrangeTheChannels() (no MapToSBUS — that
// happens at frame build time in Output.h).

inline void decodeChannelData(const uint8_t* payload, uint8_t size) {
    if (size < 4) return;
    uint16_t mask = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    if (mask == 0) return;                               // parameter packet, not channel data

    uint16_t compressed[16] = {0};
    uint8_t  nData = size - 2;
    for (uint8_t i = 0; i + 1 < nData; i += 2) {
        compressed[i / 2] = (uint16_t)payload[2 + i] | ((uint16_t)payload[2 + i + 1] << 8);
    }
    uint16_t raw[24] = {0};
    decompress(raw, compressed, decompressedSize(size));

    uint8_t p = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        if (mask & (1u << i)) {
            channelMicros[i] = raw[p++];
        }
    }
    lastChannelDataMs = millis();
}

#endif // _SRC_CHANNELS_H
