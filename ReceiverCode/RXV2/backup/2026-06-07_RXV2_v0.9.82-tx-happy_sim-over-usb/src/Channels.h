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

    // "Being flown" detection (drives the model-ID broadcast in loadNextAck).
    // Each channel's baseline is the FIRST value seen for it on this connection;
    // a later move past the deadband is a genuine stick input → the model is
    // being flown. First-sighting baselines are key: the TX only sends *changed*
    // channels and resends low-priority ones every few seconds, so a channel can
    // first appear well into the connection — that initial value must set the
    // baseline, not look like a movement (which is what broke the fixed-snapshot
    // approach). A >500 ms gap marks a fresh connection and re-arms everything.
    static bool     chSeen[16];
    static uint16_t chBase[16];
    static uint8_t  chMoveCount[16];
    const uint32_t  nowc = millis();
    if (lastChannelDataMs == 0 || (uint32_t)(nowc - lastChannelDataMs) > 500) {
        for (uint8_t i = 0; i < 16; ++i) { chSeen[i] = false; chMoveCount[i] = 0; }
        beingFlown = false;
    }

    uint8_t p = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        if (mask & (1u << i)) {
            uint16_t v      = raw[p++];
            channelMicros[i] = v;
            if (!chSeen[i]) { chBase[i] = v; chSeen[i] = true; }   // first sighting → baseline
            else if (!beingFlown) {
                int d = (int)v - (int)chBase[i];
                if (d < 0) d = -d;
                if (d > MAC_STICK_DEADBAND) {
                    // Require the SAME channel out of band on several packets so a
                    // single corrupted packet on a marginal link can't trip it.
                    if (++chMoveCount[i] >= MAC_MOVE_CONFIRM) {
                        beingFlown = true;   // sustained real stick move
                    }
                } else {
                    chMoveCount[i] = 0;       // back in band → not a real move
                }
            }
        }
    }
    lastChannelDataMs = nowc;
}

#endif // _SRC_CHANNELS_H
