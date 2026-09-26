// SquachWatch-CYD — the LoRa sniffer. See include/lora_sniffer.h.
//
// One radio, so one profile at a time, and the whole design is about which
// one to be on next. FOCUS parks on one profile and hears everything there.
// SURVEY walks the enabled profiles with a channel-activity check on each:
// a CAD is blind to the sync word and takes a few symbols, so a round over a
// dozen profiles fits inside a LongFast preamble; on a hit the chip drops
// into reception on its own and, once a frame lands, the survey lingers
// there a moment, because traffic clusters (a rebroadcast follows a packet).
//
// The radio lives in its own task on the other core. loop() only ever reads
// the ring and the counters; the task only ever writes them, under one mutex.
#if SQUACH_LORA
#include "lora_sniffer.h"
#include "lora_radio.h"
#include "lora_profiles.h"
#include "lora_classify.h"
#include "lora_meshcore.h"
#include "lora_nodes.h"
#include "lora_meshtastic.h"
#include "lora_channels.h"
#include "clock.h"
#include "settings.h"
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>

namespace Lora {

const char* modeName(Mode m) {
    switch (m) {
        case Mode::FOCUS:  return "FOCUS";
        case Mode::SURVEY: return "SURVEY";
        case Mode::SWEEP:  return "SWEEP";
        default:           return "OFF";
    }
}

namespace {

bool               s_present = false;
LoraRadio::BringUp s_up;
TaskHandle_t       s_task = nullptr;
SemaphoreHandle_t  s_lock = nullptr;

// What loop() asks of the task. Read at every step of the task's loop;
// written from loop() only, so plain volatile is enough.
volatile Mode     s_mode   = Mode::SURVEY;
volatile uint8_t  s_focus  = 0;
volatile uint64_t s_survey = 0;
volatile uint8_t  s_current = 0;
#if defined(LORA_PROBE)
volatile bool     s_dumpHex = true;    // the probe build shows every byte
#else
volatile bool     s_dumpHex = false;   // the console's LORA HEX switch
#endif

volatile bool     s_tap = false;

// The spectrum, from SWEEP: 863.0 to 870.0 MHz in 50 kHz steps.
uint8_t  s_specLive[SPECTRUM_BINS];
uint8_t  s_specHold[SPECTRUM_BINS];
uint32_t s_sweeps = 0;

// The ring. PSRAM when there is any, written one whole record at a time,
// which is the sequential kind of write the panel tolerates.
Packet*  s_ring = nullptr;
uint16_t s_cap = 0, s_head = 0, s_count = 0;
uint32_t s_total = 0;
uint32_t s_printed = 0;
Stats    s_stats;
uint32_t s_listenStart = 0;

// How long a CAD of four symbols takes on a profile, with the retune on
// top, so the survey's wait has a bound.
uint32_t cadMs(const Profile& p) {
    return (symbolUs(p.sf, p.bwKhz10) * 5) / 1000 + 4;
}
// How long the longest frame on a profile could take, for the receive timeout
// after a CAD hit: a full preamble and 255 bytes.
uint32_t maxFrameMs(const Profile& p) {
    return timeOnAirUs(p.sf, p.bwKhz10, 8, p.preamble, 255, true, false, (p.flags & PF_LDRO) != 0) / 1000 + 50;
}

void push(Packet& pk) {
    pk.ms = millis();
    pk.epoch = Clock::trusted() ? Clock::nowEpoch() : 0;
    classify(pk);
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(50)) != pdTRUE) return;
    if (s_ring && s_cap) {
        memcpy(&s_ring[s_head], &pk, sizeof pk);
        s_head = (uint16_t)((s_head + 1) % s_cap);
        if (s_count < s_cap) s_count++;
    }
    s_total++;
    s_stats.packets++;
    if (pk.flags & PK_CRC_ERR) s_stats.crcErrors++;
    s_stats.airtimeMs += pk.toaUs / 1000;
    if (pk.profile < 64) s_stats.byProfile[pk.profile]++;
    s_stats.byProto[(int)pk.proto]++;
    Nodes::note(pk, pk.ms, pk.epoch);
    xSemaphoreGive(s_lock);
}

// One IRQ's worth of bookkeeping. True when a frame was read.
bool handleIrq(uint8_t profileIdx) {
    const uint16_t f = LoraRadio::irq();
    if (!f) return false;
    if (f & LoraRadio::IRQ_RX_DONE) {
        Packet pk;
        memset(&pk, 0, sizeof pk);
        pk.profile = profileIdx;
        if (LoraRadio::readPacket(pk)) push(pk);
        return true;
    }
    // Not a frame: count what it was and clear only that, so the RX_DONE
    // that may still be coming keeps its own flags for readData's CRC check.
    uint16_t clear = 0;
    if (f & LoraRadio::IRQ_PREAMBLE)  { s_stats.preambles++;    clear |= LoraRadio::IRQ_PREAMBLE; }
    if (f & LoraRadio::IRQ_HDR_ERR)   { s_stats.headerErrors++; clear |= LoraRadio::IRQ_HDR_ERR; }
    if (f & LoraRadio::IRQ_HDR_VALID) { clear |= LoraRadio::IRQ_HDR_VALID; }
    if (f & LoraRadio::IRQ_TIMEOUT)   { clear |= LoraRadio::IRQ_TIMEOUT; }
    if (clear) LoraRadio::clearIrq(clear);
    return false;
}

uint8_t nextSurveyProfile(uint8_t from) {
    const uint8_t n = profileCount();
    const uint64_t mask = s_survey;
    for (uint8_t k = 1; k <= n; k++) {
        const uint8_t i = (uint8_t)((from + k) % n);
        if (i < 64 && (mask & (1ull << i))) return i;
    }
    return from;
}

void noiseSample() {
    const int16_t r = LoraRadio::rssiNow();
    // A slow floor: drop at once, rise slowly.
    if (r < s_stats.noiseDbm || s_stats.noiseDbm == 0) s_stats.noiseDbm = r;
    else if (r > s_stats.noiseDbm + 1) s_stats.noiseDbm++;
}

void task(void*) {
    Mode    onMode  = Mode::OFF;
    uint8_t onIdx   = 0xFF;
    bool    listening = false;
    uint32_t lastNoise = 0;

    for (;;) {
        const Mode want = s_mode;
        if (want == Mode::OFF) {
            if (listening) { LoraRadio::standby(); listening = false; }
            onMode = Mode::OFF;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (want == Mode::SWEEP) {
            // One pass over the band: tune, let the front end settle, read
            // the instantaneous RSSI twice and keep the louder. A pass takes
            // about a second; the hold decays a notch per pass.
            if (onMode != Mode::SWEEP) {
                LoraRadio::apply(profile(s_focus));
                listening = LoraRadio::startReceive();
                onMode = Mode::SWEEP;
            }
            for (uint8_t i = 0; i < SPECTRUM_BINS && s_mode == Mode::SWEEP; i++) {
                LoraRadio::tuneHz(863000000u + 50000u * i);
                vTaskDelay(pdMS_TO_TICKS(2));
                int16_t best = LoraRadio::rssiNow();
                vTaskDelay(1);
                const int16_t r2 = LoraRadio::rssiNow();
                if (r2 > best) best = r2;
                int v = best + 150; if (v < 1) v = 1; if (v > 120) v = 120;
                s_specLive[i] = (uint8_t)v;
                if (s_specLive[i] > s_specHold[i]) s_specHold[i] = s_specLive[i];
                else if (s_specHold[i] > 1) s_specHold[i]--;
            }
            s_sweeps++;
            continue;
        }

        if (want == Mode::FOCUS) {
            const uint8_t idx = s_focus;
            if (onMode != Mode::FOCUS || idx != onIdx || !listening) {
                LoraRadio::apply(profile(idx));
                listening = LoraRadio::startReceive();
                onMode = Mode::FOCUS; onIdx = idx; s_current = idx;
            }
            if (LoraRadio::waitIrq(500)) handleIrq(idx);
            if (millis() - lastNoise > 1000) { lastNoise = millis(); noiseSample(); }
            continue;
        }

        // SURVEY. A CAD on the next profile; on a hit the chip is already in
        // RX, so wait out the frame, then linger for whatever follows it.
        onMode = Mode::SURVEY;
        const uint8_t idx = nextSurveyProfile(onIdx == 0xFF ? profileCount() - 1 : onIdx);
        onIdx = idx; s_current = idx;
        const Profile& p = profile(idx);
        if (!LoraRadio::apply(p)) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        s_stats.cadRounds++;
        const uint32_t frameMs = maxFrameMs(p);
        if (!LoraRadio::startCad(4, true, frameMs)) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        listening = true;
        if (!LoraRadio::waitIrq(cadMs(p) + 30)) { LoraRadio::standby(); listening = false; continue; }
        uint16_t f = LoraRadio::irq();
        if (!(f & LoraRadio::IRQ_CAD_HIT)) {
            // CAD_DONE alone: nothing there. On to the next.
            LoraRadio::clearIrq(f);
            if (millis() - lastNoise > 1000) { lastNoise = millis(); noiseSample(); }
            continue;
        }
        s_stats.cadHits++;
        LoraRadio::clearIrq((uint16_t)(LoraRadio::IRQ_CAD_HIT | LoraRadio::IRQ_CAD_DONE));
        // The chip is in RX now, for frameMs at most.
        bool got = false;
        const uint32_t t0 = millis();
        while (millis() - t0 < frameMs + 50) {
            if (!LoraRadio::waitIrq(frameMs + 50 - (millis() - t0))) break;
            f = LoraRadio::irq();
            if (f & LoraRadio::IRQ_TIMEOUT) { LoraRadio::clearIrq(f); break; }
            if (handleIrq(idx)) { got = true; break; }
        }
        if (!got) continue;
        // Linger: a frame came, so its answers are about to. Two seconds of
        // plain reception here, or until the mode changes under us.
        LoraRadio::startReceive();
        const uint32_t l0 = millis();
        while (millis() - l0 < 2000 && s_mode == Mode::SURVEY) {
            if (LoraRadio::waitIrq(200)) {
                if (handleIrq(idx)) { /* keep lingering */ }
            }
        }
        LoraRadio::standby();
        listening = false;
    }
}

}

bool begin() {
    s_lock = xSemaphoreCreateMutex();
    memset(&s_stats, 0, sizeof s_stats);

    // The channel keys come back BEFORE the radio is asked anything, and they
    // come back even when the radio never answers. They belong to the
    // decoders, not to the module, and `LORA CHAN` is deliberately reachable
    // with no module for that reason - which is exactly what makes the order
    // matter. Restore it after the early return and the no-module case reads
    // an empty list, and then the first `CHAN` write encodes that empty list
    // straight over the stored one. With K1 on the card slot a user loses
    // every key they ever entered, by running the command the documentation
    // tells them still works. That is the shape of the IgnoreList::save() bug
    // this file already cites; it does not get to happen twice.
    const uint8_t chans = Chan::restore();

    s_present = LoraRadio::begin(s_up);
    if (!s_present) {
        Serial.printf("[lora] no module; %u channel%s restored anyway - the keys are the decoders'\n",
                      (unsigned)chans, chans == 1 ? "" : "s");
        return false;
    }

    // The ring: a few hundred records in PSRAM, a handful in internal RAM
    // when there is none.
    s_cap = 256;
    s_ring = (Packet*)heap_caps_malloc(sizeof(Packet) * s_cap, MALLOC_CAP_SPIRAM);
    if (!s_ring) {
        s_cap = 24;
        s_ring = (Packet*)heap_caps_malloc(sizeof(Packet) * s_cap, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!s_ring) s_cap = 0;

    // The survey's default: everything in the module's own band. The 433
    // rows are one command away for whoever wants to know what leaks in.
    uint64_t mask = 0;
    for (uint8_t i = 0; i < profileCount() && i < 64; i++)
        if (!(profile(i).flags & PF_433)) mask |= 1ull << i;
    s_survey = mask;
    s_focus = Settings::loraFocus() < profileCount() ? Settings::loraFocus() : defaultProfile();
    s_mode = (Mode)Settings::loraMode();
#if defined(LORA_PROBE)
    s_mode = Mode::FOCUS;   // the bench parks; the survey is for the field
#endif

    // Chan::restore() ran at the top of this function, which is also before
    // xTaskCreatePinnedToCore below - the task is the only thing that feeds
    // frames to the decoders, so the very first frame after a boot is already
    // tried against every key. Settings::load() ran near the top of setup()
    // (main.cpp), hundreds of milliseconds before this, so the store was open.

    Serial.printf("[lora] up: %s, TCXO %s, ring of %u in %s, %u channel%s restored\n", s_up.version,
                  s_up.tcxoDeci ? "on" : "off (crystal)", (unsigned)s_cap,
                  s_cap > 64 ? "PSRAM" : "internal RAM",
                  (unsigned)chans, chans == 1 ? "" : "s");
    xTaskCreatePinnedToCore(task, "lora", 6144, nullptr, 2, &s_task, 0);
    return true;
}

bool present() { return s_present; }

void statusLine(char* out, size_t cap) {
    if (!s_present) {
        snprintf(out, cap, "no module (%s)", s_up.chipFound ? "modem failed" : s_up.busyLow ? "busy fell, no chip" : "K1 not on WM?");
        return;
    }
    char mhz[12];
    formatMHz(profile(s_current).freqHz, mhz, sizeof mhz);
    snprintf(out, cap, "%s tcxo %u.%u  %s %s %s  %lu pkts", s_up.version,
             (unsigned)(s_up.tcxoDeci / 10), (unsigned)(s_up.tcxoDeci % 10),
             modeName(s_mode), profile(s_current).name, mhz, (unsigned long)s_total);
}

void tick(uint32_t now) {
    (void)now;
    if (!s_present) return;
    // Print what arrived since the last pass, a few per pass at most so a
    // burst never stalls a frame.
    uint8_t budget = 4;
    while (s_printed < s_total && budget--) {
        Packet pk;
        const uint32_t behind = s_total - s_printed;
        if (behind > s_count) { s_printed = s_total - s_count; continue; }
        if (!packetAt((uint16_t)(behind - 1), pk)) break;
        s_printed++;
        char line[96], mhz[12], bw[8];
        summary(pk, line, sizeof line);
        formatMHz(pk.freqHz, mhz, sizeof mhz);
        formatBw(pk.bwKhz10, bw, sizeof bw);
        Serial.printf("[lora] %s SF%u/%s sync %02x  %d dBm  snr %d.%02d  ferr %+ld  %s%s  %lu ms  %s\n",
                      mhz, (unsigned)pk.sf, bw, (unsigned)pk.sync, (int)pk.rssi,
                      (int)(pk.snr4 / 4), (int)abs(pk.snr4 % 4) * 25, (long)pk.ferrHz,
                      (pk.flags & PK_CRC_ERR) ? "CRC ERR" : (pk.flags & PK_CRC_OK) ? "crc ok" : "no crc",
                      (pk.flags & PK_IMPLICIT) ? " implicit" : "",
                      (unsigned long)(pk.toaUs / 1000), line);
        if (s_dumpHex) {
            char hex[255 * 3 + 32];
            hexDump(pk, 16, hex, sizeof hex);
            Serial.println(hex);
        }
        if (s_tap) {
            // LoRaTap version 0: fifteen bytes, big-endian, then the frame.
            // RSSI fields are dBm + 139 as the format wants; SNR is x4 in
            // two's complement, which is how the record already holds it.
            uint8_t h[15];
            h[0] = 0; h[1] = 0; h[2] = 0; h[3] = 15;
            h[4] = (uint8_t)(pk.freqHz >> 24); h[5] = (uint8_t)(pk.freqHz >> 16); h[6] = (uint8_t)(pk.freqHz >> 8); h[7] = (uint8_t)pk.freqHz;
            h[8] = (uint8_t)(pk.bwKhz10 / 1250);
            h[9] = pk.sf;
            const int r = pk.rssi + 139;
            h[10] = (uint8_t)(r < 0 ? 0 : r > 255 ? 255 : r);
            h[11] = h[10];
            const int c = s_stats.noiseDbm + 139;
            h[12] = (uint8_t)(c < 0 ? 0 : c > 255 ? 255 : c);
            h[13] = (uint8_t)pk.snr4;
            h[14] = pk.sync;
            char out[2 * (15 + 255) + 1];
            char* o = out;
            static const char* D = "0123456789abcdef";
            for (int i = 0; i < 15; i++) { *o++ = D[h[i] >> 4]; *o++ = D[h[i] & 15]; }
            for (int i = 0; i < pk.len; i++) { *o++ = D[pk.data[i] >> 4]; *o++ = D[pk.data[i] & 15]; }
            *o = '\0';
            Serial.printf("[tap] %s\n", out);
        }
    }
}

void setMode(Mode m) { if (m == Mode::SWEEP) { memset(s_specLive, 0, sizeof s_specLive); memset(s_specHold, 0, sizeof s_specHold); s_sweeps = 0; } s_mode = m; }
uint8_t spectrum(uint8_t* live, uint8_t* hold, uint8_t cap) {
    const uint8_t n = cap < SPECTRUM_BINS ? cap : SPECTRUM_BINS;
    if (live) memcpy(live, (const void*)s_specLive, n);
    if (hold) memcpy(hold, (const void*)s_specHold, n);
    return n;
}
uint32_t spectrumSweeps() { return s_sweeps; }
void setTap(bool on) { s_tap = on; }
bool tap() { return s_tap; }
Mode mode() { return s_mode; }
void setFocus(uint8_t i) { if (i < profileCount()) s_focus = i; }
uint8_t focus() { return s_focus; }
void setSurveyMask(uint64_t m) { s_survey = m; }
uint64_t surveyMask() { return s_survey; }
uint8_t currentProfile() { return s_current; }

uint32_t packetTotal() { return s_total; }
uint16_t packetCount() { return s_count; }

bool packetAt(uint16_t idx, Packet& out) {
    if (!s_ring || !s_lock) return false;
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    bool ok = idx < s_count;
    if (ok) {
        const uint16_t slot = (uint16_t)((s_head + s_cap - 1 - idx) % s_cap);
        memcpy(&out, &s_ring[slot], sizeof out);
    }
    xSemaphoreGive(s_lock);
    return ok;
}

const Stats& stats() { return s_stats; }

uint8_t nodeCount() { return Nodes::count(); }
uint8_t nodeOrder(uint8_t* idx, uint8_t cap) {
    if (!s_lock || xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) return 0;
    const uint8_t n = Nodes::order(idx, cap);
    xSemaphoreGive(s_lock);
    return n;
}
bool nodeAt(uint8_t i, Nodes::Node& out) {
    if (!s_lock || xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    const Nodes::Node* n = Nodes::at(i);
    if (n) memcpy(&out, n, sizeof out);
    xSemaphoreGive(s_lock);
    return n != nullptr;
}

// ---- the channel keys ------------------------------------------------------
// These run on loop()'s task while the radio task is decoding on the other
// core, and there is no mutex over the channel tables. That is deliberate and
// it is safe rather than lucky: channel(i) clamps its index against the live
// count, so a table shrinking under a decode can never be read out of bounds.
// The worst a mute or a drop can cost is one frame that does not decrypt while
// the entries shift -- against a mutex on the decoders' hot path, which every
// frame would pay for. The NVS write that follows is a few milliseconds on
// this task, which is a frame of the screen, not of the radio.
namespace {

// Every change to the list goes through here, so a store that refused the
// write is said out loud exactly once. Silence would put the board back where
// it started: a list that is there until the power goes.
void saveOrWarn() {
    if (!Chan::save())
        Serial.println("[lora] the channel list could NOT be stored -- it holds until the next boot and no further");
}

// Where a Meshtastic preset stops being built in.
uint8_t mtUserFirst() { return (uint8_t)(Meshtastic::channelCount() - Meshtastic::userChannelCount()); }

// The filter the CHANNELS view draws through: every MeshCore channel, every
// Meshtastic channel of the user's, and a Meshtastic PRESET only once it has
// opened a frame. Fourteen presets times two key modes is twenty-eight rows of
// decoder capability rather than twenty-eight channels somebody chose, and
// listing all of them would bury the thirteen that were typed in by hand. The
// count of the ones left out is on screen, so nothing is quietly hidden.
// MeshCore's one built-in stays: Public is the channel everything is on.
bool rowAt(uint8_t row, bool& isMc, uint8_t& idx) {
    uint8_t seen = 0;
    for (uint8_t i = 0; i < MeshCore::channelCount(); i++)
        if (seen++ == row) { isMc = true; idx = i; return true; }
    const uint8_t first = mtUserFirst();
    for (uint8_t i = 0; i < Meshtastic::channelCount(); i++) {
        if (i < first && !Meshtastic::channelFrames(i)) continue;
        if (seen++ == row) { isMc = false; idx = i; return true; }
    }
    return false;
}

}  // namespace

uint8_t channelRowCount() {
    uint8_t n = MeshCore::channelCount();
    const uint8_t first = mtUserFirst();
    for (uint8_t i = 0; i < Meshtastic::channelCount(); i++)
        if (i >= first || Meshtastic::channelFrames(i)) n++;
    return n;
}

bool channelRow(uint8_t row, ChannelRow& out) {
    bool isMc = false; uint8_t i = 0;
    if (!rowAt(row, isMc, i)) return false;
    memset(&out, 0, sizeof out);
    if (isMc) {
        const MeshCore::Channel c = MeshCore::channel(i);
        out.proto = Proto::MESHCORE;
        strncpy(out.name, c.name, sizeof out.name - 1);
        out.hash = c.hash;
        out.frames = MeshCore::channelFrames(i);
        out.lastMs = MeshCore::channelLastMs(i);
        out.builtIn = i < (uint8_t)(MeshCore::channelCount() - MeshCore::userChannelCount());
        out.enabled = c.enabled;
        out.derived = c.derived;
        out.keyBits = 128;      // MeshCore is AES-128 throughout
    } else {
        const Meshtastic::Channel c = Meshtastic::channel(i);
        out.proto = Proto::MESHTASTIC;
        strncpy(out.name, c.name, sizeof out.name - 1);
        out.hash = c.hash;
        out.frames = Meshtastic::channelFrames(i);
        out.lastMs = Meshtastic::channelLastMs(i);
        out.builtIn = i < mtUserFirst();
        out.enabled = c.enabled;
        out.derived = false;
        out.keyBits = (uint16_t)(c.keyLen * 8);
    }
    return true;
}

bool toggleChannelRow(uint8_t row) {
    ChannelRow r;
    if (!channelRow(row, r) || r.builtIn) return false;
    bool isMc = false; uint8_t i = 0;
    if (!rowAt(row, isMc, i)) return false;
    if (isMc) MeshCore::setChannelEnabled(i, !r.enabled);
    else      Meshtastic::setChannelEnabled(i, !r.enabled);
    saveOrWarn();      // a switch that forgot itself at the next boot is not a switch
    return true;
}

uint8_t channelsQuietBuiltIn() {
    uint8_t n = 0;
    const uint8_t first = mtUserFirst();
    for (uint8_t i = 0; i < first; i++) if (!Meshtastic::channelFrames(i)) n++;
    return n;
}

void channelCapacity(uint8_t& mcUsed, uint8_t& mcMax, uint8_t& mtUsed, uint8_t& mtMax) {
    mcUsed = MeshCore::userChannelCount(); mcMax = MeshCore::maxUserChannels();
    mtUsed = Meshtastic::userChannelCount(); mtMax = Meshtastic::maxUserChannels();
}

// ---- LORA CHAN: the keys, on the console -----------------------------------
// Keys the decoders hold. Without one a group message is read down to its
// type, its route and its channel hash and no further, which is what every
// frame said before this existed -- there was no way in.
//
// A MeshCore hashtag channel needs no key at all: the key IS
// SHA256("#name")[0..15] and anyone may join, so naming the tag is enough.
// That is why `LORA CHAN MC #test` takes no second argument.
//
// Every command that changes the list writes it back to the settings store
// before it returns. That is the whole point of this being here.
static void channelList() {
    for (uint8_t i = 0; i < MeshCore::channelCount(); i++) {
        const MeshCore::Channel c = MeshCore::channel(i);
        const bool builtIn = i < (uint8_t)(MeshCore::channelCount() - MeshCore::userChannelCount());
        Serial.printf("[lora] MC %2u %-20s hash %02x  %-9s%s  %5lu frames%s\n", (unsigned)i, c.name, (unsigned)c.hash,
                      builtIn ? "built in" : c.derived ? "tag" : "key",
                      c.enabled ? "      " : " MUTED", (unsigned long)MeshCore::channelFrames(i),
                      MeshCore::channelLastMs(i) ? "" : ", never heard");
    }
    const uint8_t mtFirst = (uint8_t)(Meshtastic::channelCount() - Meshtastic::userChannelCount());
    for (uint8_t i = 0; i < Meshtastic::channelCount(); i++) {
        const Meshtastic::Channel c = Meshtastic::channel(i);
        // The twenty-eight presets are two keys, not twenty-eight: the
        // published default and ham mode, over fourteen radio profiles. Only
        // the ones that have opened something are printed, with the rest
        // counted at the end, or they bury the user's own.
        if (i < mtFirst && !Meshtastic::channelFrames(i)) continue;
        Serial.printf("[lora] MT %2u %-20s hash %02x  %-9s%s  %5lu frames%s\n", (unsigned)i, c.name, (unsigned)c.hash,
                      i < mtFirst ? "built in" : c.keyLen ? "key" : "plaintext",
                      c.enabled ? "      " : " MUTED", (unsigned long)Meshtastic::channelFrames(i),
                      Meshtastic::channelLastMs(i) ? "" : ", never heard");
    }
    uint8_t mcU, mcM, mtU, mtM;
    channelCapacity(mcU, mcM, mtU, mtM);
    Serial.printf("[lora] %u of %u MeshCore, %u of %u Meshtastic; %u built-in preset keys not listed (nothing heard on them)\n",
                  (unsigned)mcU, (unsigned)mcM, (unsigned)mtU, (unsigned)mtM, (unsigned)channelsQuietBuiltIn());
    Serial.print("[lora] LORA CHAN MC #tag | MC|MT <name> <key> | DROP MC|MT <name> | MUTE MC|MT <name> | CLEAR | GROUP");
    for (uint8_t g = 0; g < Chan::groupCount(); g++) Serial.printf(" %s", Chan::groupName(g));
    Serial.println();
}

// "MC <rest>" / "MT <rest>": which decoder, and what is left after it.
static bool channelWhich(const char*& r, bool& mc) {
    mc = strncasecmp(r, "MC", 2) == 0;
    const bool mt = strncasecmp(r, "MT", 2) == 0;
    if (!mc && !mt) return false;
    r += 2;
    while (*r == ' ') r++;
    return *r != '\0';
}

static bool channelConsole(const char* r) {
    while (*r == ' ') r++;
    if (!*r) { channelList(); return true; }

    if (strcasecmp(r, "CLEAR") == 0) {
        MeshCore::clearUserChannels();
        Meshtastic::clearUserChannels();
        // The stored copy too, or the next boot puts every one of them back
        // and CLEAR would only have cleared the list until the power went.
        saveOrWarn();
        Serial.println("[lora] channels: back to the built-in ones, stored copy included");
        return true;
    }

    if (strncasecmp(r, "GROUP", 5) == 0) {
        // A named list of open hashtag channels in one command. The eleven the
        // owner typed in one at a time on 2026-09-26 are the NRW group; every
        // one derives its own key from its tag, so there is no key to type and
        // nothing secret in the table.
        const char* g = r + 5;
        while (*g == ' ') g++;
        if (!*g) {
            for (uint8_t i = 0; i < Chan::groupCount(); i++) {
                Serial.printf("[lora] GROUP %-6s %u channels:", Chan::groupName(i), (unsigned)Chan::groupSize(i));
                for (uint8_t k = 0; k < Chan::groupSize(i); k++) Serial.printf(" %s", Chan::groupTag(i, k));
                Serial.println();
            }
            Serial.println("[lora] LORA CHAN GROUP <name> adds one; already-held channels are left alone");
            return true;
        }
        const int ix = Chan::findGroup(g);
        if (ix < 0) { Serial.printf("[lora] no group \"%s\" -- LORA CHAN GROUP lists them\n", g); return true; }
        const uint8_t added = Chan::addGroup((uint8_t)ix);
        if (added) saveOrWarn();
        Serial.printf("[lora] GROUP %s: %u of %u added and stored%s\n", Chan::groupName((uint8_t)ix),
                      (unsigned)added, (unsigned)Chan::groupSize((uint8_t)ix),
                      added == Chan::groupSize((uint8_t)ix) ? "" : " (the rest were already held, or the table is full)");
        return true;
    }

    if (strncasecmp(r, "DROP", 4) == 0 || strncasecmp(r, "MUTE", 4) == 0) {
        const bool drop = strncasecmp(r, "DROP", 4) == 0;
        const char* a2 = r + 4;
        bool mc = false;
        if (!channelWhich(a2, mc)) {
            Serial.printf("[lora] LORA CHAN %s MC|MT <name> -- the name as LORA CHAN prints it\n", drop ? "DROP" : "MUTE");
            return true;
        }
        const int ix = mc ? MeshCore::findChannel(a2) : Meshtastic::findChannel(a2);
        if (ix < 0) { Serial.printf("[lora] %s \"%s\": not in the list\n", mc ? "MC" : "MT", a2); return true; }
        const uint8_t i = (uint8_t)ix;
        if (drop) {
            if (!(mc ? MeshCore::removeChannel(i) : Meshtastic::removeChannel(i))) {
                Serial.printf("[lora] \"%s\" is built in -- it cannot be dropped, only muted\n", a2);
                return true;
            }
            saveOrWarn();
            Serial.printf("[lora] %s \"%s\" dropped, stored copy included\n", mc ? "MC" : "MT", a2);
            return true;
        }
        const bool was = mc ? MeshCore::channel(i).enabled : Meshtastic::channel(i).enabled;
        const bool builtIn = mc ? i < (uint8_t)(MeshCore::channelCount() - MeshCore::userChannelCount())
                                : i < (uint8_t)(Meshtastic::channelCount() - Meshtastic::userChannelCount());
        if (builtIn) {
            Serial.printf("[lora] \"%s\" is built in and stays on -- muting it would make this board hear less than a stock node\n", a2);
            return true;
        }
        if (mc) MeshCore::setChannelEnabled(i, !was);
        else    Meshtastic::setChannelEnabled(i, !was);
        saveOrWarn();
        Serial.printf("[lora] %s \"%s\" %s\n", mc ? "MC" : "MT", a2,
                      was ? "muted -- still in the list, no longer tried" : "back on");
        return true;
    }

    bool mc = false;
    const char* a2 = r;
    if (!channelWhich(a2, mc)) {
        Serial.println("[lora] LORA CHAN MC|MT <name> [key] -- MC #tag needs no key");
        return true;
    }
    char name[32] = {0}, key[80] = {0};
    const char* sp = strchr(a2, ' ');
    if (sp) {
        size_t n = (size_t)(sp - a2); if (n >= sizeof name) n = sizeof name - 1;
        memcpy(name, a2, n);
        const char* k = sp; while (*k == ' ') k++;
        strncpy(key, k, sizeof key - 1);
    } else {
        strncpy(name, a2, sizeof name - 1);
    }
    if (!name[0]) { Serial.println("[lora] a channel needs a name"); return true; }
    if ((mc ? MeshCore::findChannel(name) : Meshtastic::findChannel(name)) >= 0) {
        Serial.printf("[lora] %s \"%s\" is already in the list\n", mc ? "MC" : "MT", name);
        return true;
    }
    const bool ok = mc ? MeshCore::addChannel(name, key[0] ? key : nullptr)
                       : Meshtastic::addChannel(name, key);
    if (!ok) {
        Serial.printf("[lora] %s \"%s\": refused - table full, or the key is neither 16 nor 32 bytes of base64 or hex\n",
                      mc ? "MC" : "MT", name);
        return true;
    }
    saveOrWarn();
    if (mc) {
        const uint8_t last = (uint8_t)(MeshCore::channelCount() - 1);
        const MeshCore::Channel c = MeshCore::channel(last);
        // The hash is ONE byte of SHA256, so two channels can share it and
        // in a real list they do - #bochum and #rheine both come out 6c.
        // That is not a clash to resolve: a frame carries the hash only as
        // a hint, and the two-byte HMAC in front of the ciphertext decides
        // which key was right. Say it anyway, so a quiet channel is not
        // mistaken for a broken one.
        // Copied out, not pointed at: channel(0) rebuilds Public in a static
        // of its own on every call, so a name borrowed from it is only good
        // until the next one.
        char twin[24] = {0};
        for (uint8_t i = 0; i < last; i++)
            if (MeshCore::channel(i).hash == c.hash) { strncpy(twin, MeshCore::channel(i).name, sizeof twin - 1); break; }
        Serial.printf("[lora] MC \"%s\" added and stored, hash %02x - frames whose channel hash matches will now decrypt%s%s%s\n",
                      c.name, (unsigned)c.hash,
                      twin[0] ? " (same hash as \"" : "", twin,
                      twin[0] ? "\"; the HMAC tells them apart)" : "");
    } else {
        Serial.printf("[lora] MT \"%s\" added and stored\n", name);
    }
    return true;
}

bool console(const char* line) {
    if (strncasecmp(line, "LORA", 4) != 0) return false;
    const char* a = line + 4;
    while (*a == ' ') a++;
    // The channel list belongs to the decoders, not to the radio: it can be
    // read and edited with K1 on the card slot and no module in the board at
    // all, and the keys will be waiting when one arrives. So CHAN comes before
    // the no-module gate.
    if (strncasecmp(a, "CHAN", 4) == 0) return channelConsole(a + 4);
    if (!s_present) { Serial.println("[lora] no module answered at boot (LORA CHAN still works: the keys are not the radio's)"); return true; }
    if (*a == '\0' || strcasecmp(a, "STATUS") == 0) {
        char s[96];
        statusLine(s, sizeof s);
        Serial.printf("[lora] %s\n", s);
        Serial.printf("[lora] %lu frames, %lu crc errors, %lu header errors, %lu stray preambles; cad %lu rounds %lu hits; air %lu ms; noise %d dBm\n",
                      (unsigned long)s_stats.packets, (unsigned long)s_stats.crcErrors, (unsigned long)s_stats.headerErrors,
                      (unsigned long)s_stats.preambles, (unsigned long)s_stats.cadRounds, (unsigned long)s_stats.cadHits,
                      (unsigned long)s_stats.airtimeMs, (int)s_stats.noiseDbm);
        return true;
    }
    if (strcasecmp(a, "LIST") == 0) {
        for (uint8_t i = 0; i < profileCount(); i++) {
            const Profile& p = profile(i);
            char mhz[12], bw[8];
            formatMHz(p.freqHz, mhz, sizeof mhz); formatBw(p.bwKhz10, bw, sizeof bw);
            Serial.printf("[lora] %2u %c %-14s %s SF%-2u %s sync %02x pre %u%s%s  %u\n", (unsigned)i,
                          (s_survey & (1ull << i)) ? '*' : ' ', p.name, mhz, (unsigned)p.sf, bw,
                          (unsigned)p.sync, (unsigned)p.preamble,
                          (p.flags & PF_INVERT) ? " iq-inv" : "", (p.flags & PF_IMPLICIT) ? " implicit" : "",
                          (unsigned)s_stats.byProfile[i < 64 ? i : 0]);
        }
        return true;
    }
    if (strncasecmp(a, "FOCUS", 5) == 0) {
        const int n = atoi(a + 5);
        if (n >= 0 && n < profileCount()) { setFocus((uint8_t)n); setMode(Mode::FOCUS); Serial.printf("[lora] focus on %u %s\n", n, profile((uint8_t)n).name); }
        else Serial.println("[lora] LORA FOCUS <n> -- see LORA LIST");
        return true;
    }
    if (strcasecmp(a, "SURVEY") == 0) { setMode(Mode::SURVEY); Serial.println("[lora] survey"); return true; }
    if (strcasecmp(a, "SWEEPING") == 0 || strcasecmp(a, "SPECTRUM") == 0) { setMode(Mode::SWEEP); Serial.println("[lora] sweeping 863-870 MHz; the STATS view draws it"); return true; }
    if (strcasecmp(a, "TAP") == 0) { s_tap = !s_tap; Serial.printf("[lora] LoRaTap lines %s\n", s_tap ? "on: tools/loratap2pcap.py turns the log into a pcap" : "off"); return true; }
    if (strcasecmp(a, "OFF") == 0)    { setMode(Mode::OFF); Serial.println("[lora] off"); return true; }
    if (strcasecmp(a, "HEX") == 0)    { s_dumpHex = !s_dumpHex; Serial.printf("[lora] hex dumps %s\n", s_dumpHex ? "on" : "off"); return true; }
    if (strcasecmp(a, "ALL") == 0) {
        uint64_t m = 0;
        for (uint8_t i = 0; i < profileCount() && i < 64; i++) m |= 1ull << i;
        s_survey = m; Serial.println("[lora] survey: every profile, 433 included"); return true;
    }
    if (strncasecmp(a, "MASK", 4) == 0) {
        s_survey = strtoull(a + 4, nullptr, 16); Serial.printf("[lora] survey mask %016llx\n", (unsigned long long)s_survey); return true;
    }
    if (strncasecmp(a, "SWEEP", 5) == 0) {
        // A quick look at the band: park the survey, tune across it and read
        // the instantaneous RSSI. Blocking, on the console's own terms.
        const Mode was = s_mode;
        setMode(Mode::OFF);
        vTaskDelay(pdMS_TO_TICKS(300));
        uint32_t from = 863000000, to = 870000000, step = 100000;
        unsigned f0, f1, st;
        if (sscanf(a + 5, "%u %u %u", &f0, &f1, &st) == 3) { from = f0 * 1000u; to = f1 * 1000u; step = st * 1000u; }
        LoraRadio::apply(profile(s_focus));
        LoraRadio::startReceive();
        for (uint32_t hz = from; hz <= to; hz += step) {
            LoraRadio::tuneHz(hz);
            vTaskDelay(pdMS_TO_TICKS(3));
            int16_t best = -200;
            for (int k = 0; k < 4; k++) { const int16_t r = LoraRadio::rssiNow(); if (r > best) best = r; vTaskDelay(1); }
            char mhz[12]; formatMHz(hz, mhz, sizeof mhz);
            int bars = (best + 130) / 3; if (bars < 0) bars = 0; if (bars > 40) bars = 40;
            Serial.printf("[sweep] %s %4d  %.*s\n", mhz, (int)best, bars, "########################################");
        }
        LoraRadio::standby();
        setMode(was);
        return true;
    }
    if (strcasecmp(a, "NODES") == 0) {
        // The same table the NODES screen draws, on the console, because bench
        // work happens over the serial line and reading a node list off a photo
        // of the panel is no way to work.
        uint8_t idx[64];
        const uint8_t n = Nodes::order(idx, (uint8_t)(sizeof idx));
        if (!n) { Serial.println("[lora] no nodes heard yet"); return true; }
        const uint32_t now = millis();
        for (uint8_t i = 0; i < n; i++) {
            const Nodes::Node* nd = Nodes::at(idx[i]);
            if (!nd) continue;
            char flags[32] = {0};
            Nodes::flagsText(*nd, flags, sizeof flags);
            char pos[28] = "-";
            if (nd->hasPos)
                snprintf(pos, sizeof pos, "%.4f,%.4f", nd->latE7 / 1e7, nd->lonE7 / 1e7);
            Serial.printf("[lora] %-11s %-10s %-22s %-9s %4d dBm snr %5.2f  %3u pkt  %2u.%01u%%  %lus ago  %s %s\n",
                          protoName(nd->proto), nd->tag, nd->name[0] ? nd->name : "-",
                          Nodes::roleText(*nd), (int)nd->rssi, nd->snr4 / 4.0,
                          (unsigned)nd->packets,
                          (unsigned)(Nodes::dutyPermille(*nd, now) / 10),
                          (unsigned)(Nodes::dutyPermille(*nd, now) % 10),
                          (unsigned long)((now - nd->lastMs) / 1000), pos, flags);
        }
        Serial.printf("[lora] %u node(s)\n", (unsigned)n);
        return true;
    }
    Serial.println("[lora] LORA | LIST | FOCUS <n> | SURVEY | SPECTRUM | OFF | ALL | MASK <hex> | HEX | TAP | CHAN | NODES | SWEEP [kHz from to step]");
    Serial.println("[lora] LORA CHAN on its own lists the keys and the rest of its words");
    return true;
}

}
#endif
