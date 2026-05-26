// LockDownRadioControl — RXV2  ::  Radio.h
//
// nRF24L01+ control: self-test, dual-radio detection + swap-on-loss, v1
// listen configuration, bind, ack-payload rotation, and the radio polling
// loop. Mirrors v1's ReceiverCode/src/utilities/radio.h pattern.
//
//*********************************************************************

#ifndef _SRC_RADIO_H
#define _SRC_RADIO_H

#include "1Defs.h"
#include "Storage.h"        // saveBindToNvs()
#include "Channels.h"       // decompress(), decompressedSize(), decodeChannelData()

//*********************************************************************
//  Data-rate name (for the self-test report)
//*********************************************************************

inline const char* dataRateName(rf24_datarate_e d) {
    switch (d) {
        case RF24_1MBPS:   return "1 Mbps";
        case RF24_2MBPS:   return "2 Mbps";
        case RF24_250KBPS: return "250 kbps";
        default:           return "?";
    }
}

//*********************************************************************
//  Radio1 self-test (runs once at boot)
//*********************************************************************
// Confirms SPI wiring, chip presence, and register read/write. Result lands
// in the rfTest global so /diagnostics can show "PASS" / "FAIL" with detail.

inline void runRadioSelfTest() {
    rfTest.ran = true;

    pinMode(PIN_NRF_CSN, OUTPUT);
    digitalWrite(PIN_NRF_CSN, HIGH);
    pinMode(PIN_NRF_CE,  OUTPUT);
    digitalWrite(PIN_NRF_CE,  LOW);

    // Override variant defaults — route MISO off the BOOT-strap pin.
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
    delay(50);

    rfTest.beginOk = radio1.begin();
    if (rfTest.beginOk) {
        radio1.printDetails();
    } else {
        rfTest.verdict = "radio1.begin() failed — see wiring checklist";
        Serial.println("[rf] radio1.begin() FAILED — chip not responding on SPI");
        return;
    }

    rfTest.chipConnected = radio1.isChipConnected();

    radio1.setChannel(76);
    delayMicroseconds(150);
    rfTest.channelRead = radio1.getChannel();
    rfTest.channelOk   = (rfTest.channelRead == 76);

    radio1.setDataRate(RF24_250KBPS);
    delayMicroseconds(150);
    rf24_datarate_e dr = radio1.getDataRate();
    rfTest.dataRateRead = static_cast<uint8_t>(dr);
    rfTest.dataRateOk   = (dr == RF24_250KBPS);

    bool allOk = rfTest.beginOk && rfTest.chipConnected
              && rfTest.channelOk && rfTest.dataRateOk;
    rfTest.verdict = allOk
        ? "PASS — radio responds, registers writable, ready to receive"
        : "FAIL — see details (likely a wiring fault)";

    Serial.printf("[rf] radio1 begin=%d chip=%d ch=%d dr=%d  %s\n",
                  rfTest.beginOk, rfTest.chipConnected,
                  rfTest.channelOk, rfTest.dataRateOk,
                  rfTest.verdict);
}

//*********************************************************************
//  Probe each of the three radio slots (single / dual / triple PCB)
//*********************************************************************
// Probes slots 2 and 3 (slot 1 already covered by runRadioSelfTest). Each
// slot's presence is recorded independently — if slot 1 has failed but
// slot 3 still answers, swap-on-loss can use whichever radios actually
// respond. Slots that are absent are simply skipped by swapRadios().

inline bool probeRadioSlot(RF24& r, uint8_t cePin, uint8_t csnPin, uint8_t idxForLog) {
    pinMode(cePin,  OUTPUT);
    pinMode(csnPin, OUTPUT);
    digitalWrite(cePin,  LOW);
    digitalWrite(csnPin, HIGH);
    delay(5);
    if (!r.begin() || !r.isChipConnected()) {
        Serial.printf("[rf] radio%u not present\n", idxForLog);
        return false;
    }
    r.setChannel(76);
    delayMicroseconds(150);
    bool channelOk = (r.getChannel() == 76);
    Serial.printf("[rf] radio%u DETECTED, channel r/w %s\n",
                  idxForLog, channelOk ? "ok" : "FAIL");
    return channelOk;
}

inline void detectAllRadios() {
    // Slot 1 is the SPI-bench from runRadioSelfTest — read its verdict.
    radioPresent[0] = rfTest.beginOk && rfTest.chipConnected;
    radioPresent[1] = probeRadioSlot(radio2, PIN_NRF_CE2, PIN_NRF_CSN2, 2);
    radioPresent[2] = probeRadioSlot(radio3, PIN_NRF_CE3, PIN_NRF_CSN3, 3);
    numRadiosPresent = (uint8_t)(radioPresent[0] + radioPresent[1] + radioPresent[2]);

    // currentRadio defaults to radio1; if it's missing, pick the first
    // slot that did respond so the receiver still works.
    if (!radioPresent[0]) {
        for (uint8_t i = 1; i < 3; i++) {
            if (radioPresent[i]) { currentRadio = radios[i]; activeRadioIdx = i + 1; break; }
        }
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "Radios detected: %u (1:%s 2:%s 3:%s)",
             numRadiosPresent,
             radioPresent[0] ? "ok" : "-",
             radioPresent[1] ? "ok" : "-",
             radioPresent[2] ? "ok" : "-");
    events.add(buf);
    Serial.printf("[rf] %s\n", buf);
}

//*********************************************************************
//  Swap active radio (CE-line swap) — rotates through present slots only
//*********************************************************************
// Skips absent slots so a 3-slot PCB with one dead chip keeps cycling
// only the working two. With one radio there's nothing to swap to.

inline void swapRadios() {
    if (numRadiosPresent < 2) return;

    uint8_t startIdx = activeRadioIdx - 1;   // 0-based
    uint8_t nextIdx  = startIdx;
    for (uint8_t step = 0; step < 3; step++) {
        nextIdx = (nextIdx + 1) % 3;
        if (radioPresent[nextIdx]) break;
    }
    if (nextIdx == startIdx) return;         // nothing else present

    // Accrue the time we've spent on the outgoing radio before switching.
    uint32_t now = millis();
    radioActiveMs[startIdx] += (now - radioActiveStartMs);
    radioActiveStartMs       = now;

    currentRadio->stopListening();
    delayMicroseconds(150);
    currentRadio   = radios[nextIdx];
    activeRadioIdx = nextIdx + 1;
    currentRadio->startListening();
    delayMicroseconds(150);
    // Re-prime the ack-payload FIFO on the new active radio — the previous
    // one's queued payloads stay with it.
    loadNextAck();
    loadNextAck();
    loadNextAck();

    radioSwaps++;
    lastRadioSwapMs = millis();
    char buf[60];
    snprintf(buf, sizeof(buf), "Swapped to radio %u", activeRadioIdx);
    events.add(buf);
    Serial.printf("[rf] %s\n", buf);
}

//*********************************************************************
//  Build the next ack payload
//*********************************************************************
// Mirrors v1's LoadAckPayload(). First MAC_ACK_THRESHOLD acks carry our
// 8-byte board ID (so TX can identify us); after that we rotate items 0..35.
// Items we don't have sensors for return zeros — v1 ignores zero values
// cleanly. byte[0] also carries the FHSS HOP flag (high bit) when it's time
// to switch channel; byte[5] carries the FHSS table index so the TX knows
// where we're going.

// Active seconds for radio slot idx (0..2). For the currently-active radio
// this includes the live counter; for others it's the frozen accumulator.
inline uint32_t radioElapsedSec(uint8_t idx) {
    uint32_t acc = radioActiveMs[idx];
    if (idx == (uint8_t)(activeRadioIdx - 1)) acc += (millis() - radioActiveStartMs);
    return acc / 1000;
}

inline void loadNextAck() {
    uint8_t ack[ACK_PAYLOAD_BYTES] = {0};

    // FHSS hop decision — gated on fhssEnabled while we're still proving the
    // link on a fixed channel.
    bool hopThisAck = false;
    if (fhssEnabled && macAcksSent >= MAC_ACK_THRESHOLD &&
        (uint32_t)(millis() - lastHopMs) >= HOP_TIME_MS) {
        nextChannelIdx = (nextChannelIdx + 1) % 83;
        hopThisAck = true;
    }

    if (macAcksSent < MAC_ACK_THRESHOLD) {
        // MAC ack format (v1 SendMacAddress)
        ackByteZero = ackByteZero ? 0 : 1;        // toggles 0,1,0,1 → which MAC half
        ack[0] = ackByteZero;                     // HOP bit stays clear during MAC phase
        uint8_t base = ackByteZero ? 4 : 0;
        ack[1] = boardMac[base + 0];
        ack[2] = boardMac[base + 1];
        ack[3] = boardMac[base + 2];
        ack[4] = boardMac[base + 3];
        macAcksSent++;
    } else {
        // Telemetry rotation (v1 LoadAckPayload switch, simplified)
        if (++telemetryItem > MAX_TELEMETRY_ITEM) telemetryItem = 0;
        ack[0] = telemetryItem;
        bool versionCase = false;
        switch (telemetryItem) {
            case 0:
                // Slot 0 ALWAYS carries the real chip MAC's first half. The
                // v1 TX's pre-match parser (ParseAckPayload → GetModelsMacAddress
                // while !ModelMatched && !LedWasGreen) reads slot 0 into
                // ModelsMacUnion.Val32[0] on every packet — sending anything
                // else here clobbers the MAC and corrupts the saved model ID.
                // RX firmware version lives on slot 25; the v1 TX UI shows
                // garbage there but the bind handshake stays rock solid.
                ack[1] = boardMac[0];
                ack[2] = boardMac[1];
                ack[3] = boardMac[2];
                ack[4] = boardMac[3];
                break;
            case 1:
                // Slot 1 — second half of the real chip MAC, same rationale as slot 0.
                ack[1] = boardMac[4];
                ack[2] = boardMac[5];
                ack[3] = boardMac[6];
                ack[4] = boardMac[7];
                break;
            case 2:   packU32(ack, radioSwaps);                     break;  // RadioSwaps
            case 3:   packU32(ack, radioElapsedSec(0));             break;  // Transceiver 1 active time (sec)
            case 4:   packU32(ack, radioElapsedSec(1));             break;  // Transceiver 2 active time (sec)
            case 36:  packU32(ack, radioElapsedSec(2));             break;  // Transceiver 3 active time (sec) — matches TX's RX3TotalTime handler
            case 5: {
                // Battery voltage. Prefer FC's reported voltage; fall back to 0 if
                // no CRSF telemetry. v1 TX has a backward-compat quirk: if it
                // receives above 6S max it assumes 12S pre-halved and ×2. We pre-halve.
                constexpr float V_6S_MAX = 25.2f;        // 6 × 4.2 V
                float v = (fcTelem.valid && fcTelem.fcBattVolts > 0.1f) ? fcTelem.fcBattVolts : 0.0f;
                float vTx = (v > V_6S_MAX) ? (v * 0.5f) : v;
                packF32(ack, vTx);
                break;
            }
            case 6:   packF32(ack, 0.0f);                           break;  // baro altitude
            case 7:   packF32(ack, 0.0f);                           break;  // baro temperature
            case 19:  packF32(ack, 0.0f);                           break;  // rate of climb
            case 21:
                // Battery current (Rotorflight). Forward FC's measured current.
                if (fcTelem.valid) packF32(ack, fcTelem.fcBattAmps);
                break;
            case 22:
                // Battery capacity used (mAh, Rotorflight).
                if (fcTelem.valid) packF32(ack, (float)fcTelem.fcBattMah);
                break;
            case 23:
                // Receiver type index into the TX's Rx_type[] table:
                //   0=Unknown, 1=TRX:1 PWM:8, 2=TRX:2 PWM:8, 3=TRX:2 PWM:11,
                //   4=TRX:1 V2,  5=TRX:2 V2,  6=TRX:3 V2
                // RXV2 reports 4 / 5 / 6 based on radios actually detected
                // (e.g. a 3-slot PCB with one dead chip reports as 2-radio).
                ack[1] = (uint8_t)(3 + numRadiosPresent);  // 1→4, 2→5, 3→6
                break;
            case 24:  packF32(ack, 0.0f);                           break;  // ESC temp
            case 25:
                // RXV2 firmware version. Sent on slot 25 instead of slot 0 so
                // that slot 0 can stay as a stable real-MAC carrier (the v1 TX
                // reads slot 0 into ModelsMacUnion every packet during pre-match).
                // The v1 TX displays "garbage" in its RX firmware field because
                // it reads version only from slot 0 — accepted trade-off until
                // the TX has a slot-25 reader or a different version channel.
                ack[1] = activeRadioIdx;
                ack[2] = RXV2_V_MAJOR;
                ack[3] = RXV2_V_MINOR;
                ack[4] = RXV2_V_MINIMUS;
                ack[5] = (uint8_t)RXV2_V_EXTRA;
                versionCase = true;
                break;
            case 31:
                // Rotorflight version flag. Set when FC telemetry is active so v1 TX
                // picks up the Rotorflight-specific slots (cases 20-22).
                packU32(ack, fcTelem.valid ? 1u : 0u);
                break;
            case 35:  packU32(ack, buildDays);                      break;  // BuildAge in days since 2020-01-01
            default:  break;
        }
        if (hopThisAck) {
            ack[0] |= 0x80;
            hopPending = true;
        }
        if (!versionCase) {
            ack[5] = nextChannelIdx;
        }
    }

    if (currentRadio->writeAckPayload(V1_PIPE_NUMBER, ack, ACK_PAYLOAD_BYTES)) {
        rx.acksWritten++;
    }
}

//*********************************************************************
//  Try to extract a bind from an incoming packet
//*********************************************************************
// Mirrors v1's GetNewPipe(). When unbound and the packet looks like a bind
// frame (channels 1..5 carry the new 5-byte pipe address), extract the pipe
// and re-open the reading pipe on both radios.

inline void tryBind(const uint8_t* payload, uint8_t size) {
    if (bindState.bound)   return;
    if (size < 10)    return;
    uint16_t mask = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    if ((mask & 0x003E) != 0x003E) return;       // channels 1..5 must all be flagged

    bindState.attempts++;

    uint16_t compressed[16] = {0};
    uint8_t  nDataBytes = size - 2;
    for (uint8_t i = 0; i + 1 < nDataBytes; i += 2) {
        compressed[i / 2] = (uint16_t)payload[2 + i] | ((uint16_t)payload[2 + i + 1] << 8);
    }

    uint16_t raw[24] = {0};
    decompress(raw, compressed, decompressedSize(size));

    uint16_t channel[16] = {0};
    uint8_t p = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        if (mask & (1u << i)) {
            channel[i] = raw[p++];
        }
    }

    // v1 GetNewPipe reverses the byte order when copying into the pipe address.
    for (uint8_t i = 0; i < 5; ++i) {
        bindState.pipe[4 - i] = (uint8_t)(channel[i + 1] & 0xff);
    }

    // Switch the radio's reading pipe to the new address. Stay on channel 82.
    currentRadio->stopListening();
    delayMicroseconds(150);
    currentRadio->flush_tx();
    currentRadio->flush_rx();
    currentRadio->openReadingPipe(V1_PIPE_NUMBER, bindState.pipe);
    currentRadio->startListening();
    delayMicroseconds(150);
    // Mirror the new pipe to every OTHER present radio so any subsequent
    // swap lands on a slot already configured for this bind.
    for (uint8_t i = 0; i < 3; i++) {
        if (!radioPresent[i] || radios[i] == currentRadio) continue;
        radios[i]->stopListening();
        delayMicroseconds(150);
        radios[i]->flush_tx();
        radios[i]->flush_rx();
        radios[i]->openReadingPipe(V1_PIPE_NUMBER, bindState.pipe);
        // Other slots stay in standby (CE low) — only currentRadio has CE high.
    }

    loadNextAck();
    loadNextAck();
    loadNextAck();

    bindState.bound       = true;
    bindState.boundMillis = millis();
    saveBindToNvs();

    char buf[80];
    snprintf(buf, sizeof(buf), "Bound to TX, pipe %02X %02X %02X %02X %02X",
             bindState.pipe[0], bindState.pipe[1], bindState.pipe[2], bindState.pipe[3], bindState.pipe[4]);
    events.add(buf);

    Serial.printf("[bind] new pipe %02X %02X %02X %02X %02X (after %u attempts)\n",
                  bindState.pipe[0], bindState.pipe[1], bindState.pipe[2], bindState.pipe[3], bindState.pipe[4],
                  (unsigned)bindState.attempts);
}

//*********************************************************************
//  Configure radio(s) for v1-compatible listen
//*********************************************************************
// Mirrors v1's ConfigureRadio(). Both radios get identical settings — Radio1
// active (CE high), Radio2 in standby (CE low) so only one is receiving at
// any moment.

inline void radioBeginListenV1() {
    // Was guarded on rfTest.beginOk (radio1-specific). That gated out the
    // valid case where slot 1 is empty but slot 2 or 3 has a working chip
    // — nothing got configured and the TX couldn't connect. Now we only
    // bail when literally zero radios are present.
    if (numRadiosPresent == 0) return;

    auto configureOne = [](RF24& r, const uint8_t* pipe) {
        r.setPALevel(RF24_PA_MAX);
        r.setDataRate(RF24_250KBPS);
        r.enableAckPayload();
        r.setRetries(2, 2);
        r.enableDynamicPayloads();
        r.setAddressWidth(5);
        r.setCRCLength(RF24_CRC_16);
        r.setAutoAck(true);
        r.maskIRQ(1, 1, 1);
        r.setChannel(V1_RECOVERY_CH);
        r.openReadingPipe(V1_PIPE_NUMBER, pipe);
    };
    const uint8_t* pipe = bindState.bound ? bindState.pipe : V1_DEFAULT_PIPE;
    for (uint8_t i = 0; i < 3; i++) {
        if (radioPresent[i]) configureOne(*radios[i], pipe);
    }
    // Start listening on the first present slot — usually radio1, but if it
    // was missing at boot the active pointer was already pointed elsewhere
    // by detectAllRadios(), so honour that.
    if (!radioPresent[0]) {
        for (uint8_t i = 0; i < 3; i++) {
            if (radioPresent[i]) { currentRadio = radios[i]; activeRadioIdx = i + 1; break; }
        }
    } else {
        currentRadio = &radio1;
        activeRadioIdx = 1;
    }
    currentRadio->startListening();
    radioActiveStartMs = millis();   // start counting time on the initial active radio
    Serial.printf("[rf] %s pipe %02X %02X %02X %02X %02X on channel %u\n",
                  bindState.bound ? "bound" : "default",
                  pipe[0], pipe[1], pipe[2], pipe[3], pipe[4], V1_RECOVERY_CH);
    (void)V1_DEFAULT_PIPE;

    // Pre-load ack-payload FIFO so the very first incoming packet's ACK
    // already carries telemetry bytes.
    loadNextAck();
    loadNextAck();
    loadNextAck();

    lastHopMs = millis();
}

//*********************************************************************
//  Per-loop poll for incoming packets
//*********************************************************************

inline void radioPoll() {
    uint8_t pipe = 0;
    if (currentRadio->available(&pipe)) {
        uint8_t size = currentRadio->getDynamicPayloadSize();
        if (size > 0 && size <= 32) {
            currentRadio->read(rx.lastBytes, size);
            rx.lastPayload = size;
            if (size > rx.maxPayload) {
                rx.maxPayload = size;
                memcpy(rx.maxBytes, rx.lastBytes, size);
            }
        } else {
            currentRadio->flush_rx();
            rx.lastPayload = 0;
        }
        rx.packets   += 1;
        rx.lastMillis = millis();

        // tryBind switches pipes and pre-loads acks itself if it fires.
        if (!bindState.bound) {
            tryBind(rx.lastBytes, size);
        } else {
            decodeChannelData(rx.lastBytes, size);
        }

        loadNextAck();

        // If we just told the TX to hop, hop ourselves now so we're both on
        // the new channel when the next packet flies (v1 UseReceivedData).
        if (hopPending) {
            hopPending = false;
            currentRadio->stopListening();
            delayMicroseconds(100);
            currentRadio->setChannel(FHSS_CHANNELS[nextChannelIdx]);
            currentRadio->startListening();
            delayMicroseconds(100);
            lastHopMs = millis();
        }
    }
}

#endif // _SRC_RADIO_H
