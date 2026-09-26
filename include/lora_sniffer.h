// SquachWatch-CYD — the LoRa sniffer: what main.cpp, the console and the
// screens see of the SX1262. The radio itself is in lora_radio.h.
//
// Only the CrowPanel 7 has the slot (-DSQUACH_LORA). Everywhere else the
// calls below are inline no-ops, so the emulator and the other boards include
// this header and never link the radio.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "lora_pkt.h"

namespace Lora {

enum class Mode : uint8_t { OFF = 0, FOCUS = 1, SURVEY = 2, COUNT = 3 };
const char* modeName(Mode m);

// Counters since boot, for the CHANNEL screen and the console.
struct Stats {
    uint32_t packets;       // frames read, good or not
    uint32_t crcErrors;     // frames with a failed payload CRC
    uint32_t headerErrors;  // a header that did not check: another sync word, or a collision
    uint32_t preambles;     // preambles that never became a header
    uint32_t cadRounds;     // CAD checks run
    uint32_t cadHits;       // ...that heard something
    uint32_t airtimeMs;     // sum of the time on air of every frame read
    uint32_t listenMs;      // how long the radio has been in RX or CAD in total
    int16_t  noiseDbm;      // the quietest recent instantaneous RSSI reading
    uint16_t byProfile[64]; // frames per profile index
    uint16_t byProto[(int)Proto::COUNT];
};

#if SQUACH_LORA
// After the WiFi and Bluetooth radios are up. True when a module answered.
bool begin();
bool present();
// One line for DIAGNOSTICS: the chip, the oscillator, the mode.
void statusLine(char* out, size_t cap);
// Each pass of loop(): prints new frames to the console and keeps the
// counters. Cheap when nothing arrived.
void tick(uint32_t now);

void setMode(Mode m);
Mode mode();
void setFocus(uint8_t profileIdx);
uint8_t focus();
// Which profiles the survey rotates through, as a bitmask over the table
// (bit i = profile i). Defaults to everything in the module's own band.
void setSurveyMask(uint64_t mask);
uint64_t surveyMask();
// Which profile the radio is on right now.
uint8_t currentProfile();

// The ring: newest first. packetAt copies, so the caller may hold it across
// frames; false past the end.
uint32_t packetTotal();
uint16_t packetCount();
bool     packetAt(uint16_t idxFromNewest, Packet& out);
const Stats& stats();

// The "LORA ..." console commands; true if the line was one.
bool console(const char* line);
#else
inline bool begin() { return false; }
inline bool present() { return false; }
inline void statusLine(char* out, size_t cap) { if (cap) out[0] = '\0'; }
inline void tick(uint32_t) {}
inline void setMode(Mode) {}
inline Mode mode() { return Mode::OFF; }
inline void setFocus(uint8_t) {}
inline uint8_t focus() { return 0; }
inline void setSurveyMask(uint64_t) {}
inline uint64_t surveyMask() { return 0; }
inline uint8_t currentProfile() { return 0; }
inline uint32_t packetTotal() { return 0; }
inline uint16_t packetCount() { return 0; }
inline bool packetAt(uint16_t, Packet&) { return false; }
inline const Stats& stats() { static Stats s = {}; return s; }
inline bool console(const char*) { return false; }
#endif

}
