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
#include "lora_nodes.h"

namespace Lora {

// SWEEP is the spectrum: the radio walks the band reading the instantaneous
// RSSI and hears nothing else meanwhile. Not a saved setting.
enum class Mode : uint8_t { OFF = 0, FOCUS = 1, SURVEY = 2, SWEEP = 3, COUNT = 4 };
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

// The node table, copied out under the sniffer's lock: how many, the order
// newest first, and one row.
uint8_t nodeCount();
uint8_t nodeOrder(uint8_t* idx, uint8_t cap);
bool    nodeAt(uint8_t i, Nodes::Node& out);

// ---- the channel keys, for the CHANNELS view --------------------------------
// One row per key worth showing. The screens go through here rather than
// calling MeshCore:: and Meshtastic:: themselves, the same way they do for
// nodes: those namespaces are compiled on every board, this slot exists on
// one, and the no-op branch below is what keeps ui_lora.cpp board-agnostic.
struct ChannelRow {
    Proto    proto;
    char     name[24];
    uint8_t  hash;
    uint32_t frames;    // frames this key has opened since boot
    uint32_t lastMs;    // when the last one landed; 0 = never
    bool     builtIn;   // Public, or a Meshtastic preset: cannot be muted or dropped
    bool     enabled;
    bool     derived;   // MeshCore hashtag: the key comes from the name
    uint16_t keyBits;   // 128 or 256 -- and 256 does not fit in a byte; 0 = no encryption (ham mode)
};
uint8_t channelRowCount();
bool    channelRow(uint8_t i, ChannelRow& out);
// Mute or unmute the key on that row and write the list back to the store.
// A built-in row is left alone; returns false when nothing changed.
bool    toggleChannelRow(uint8_t i);
// Built-in keys that are NOT listed because nothing has arrived on them --
// twenty-eight Meshtastic presets on a quiet band. Counted, not hidden: the
// view says how many there are.
uint8_t channelsQuietBuiltIn();
// How many of each table are the user's, and the room there is. For the one
// line on screen that says whether the list can still grow.
void    channelCapacity(uint8_t& mcUsed, uint8_t& mcMax, uint8_t& mtUsed, uint8_t& mtMax);

// The spectrum from SWEEP: one value per bin from 863.0 to 870.0 MHz in
// 50 kHz steps, as dBm + 150 (0 = nothing yet). `hold` keeps the loudest
// reading with a slow decay. Returns the bin count.
static const uint8_t SPECTRUM_BINS = 141;
uint8_t spectrum(uint8_t* live, uint8_t* hold, uint8_t cap);
uint32_t spectrumSweeps();

// LoRaTap over the console: every frame as one "[tap] <hex>" line, which
// tools/loratap2pcap.py turns into a capture Wireshark opens.
void setTap(bool on);
bool tap();

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
inline uint8_t nodeCount() { return 0; }
inline uint8_t nodeOrder(uint8_t*, uint8_t) { return 0; }
inline bool nodeAt(uint8_t, Nodes::Node&) { return false; }
struct ChannelRow {
    Proto    proto;
    char     name[24];
    uint8_t  hash;
    uint32_t frames, lastMs;
    bool     builtIn, enabled, derived;
    uint16_t keyBits;
};
inline uint8_t channelRowCount() { return 0; }
inline bool channelRow(uint8_t, ChannelRow&) { return false; }
inline bool toggleChannelRow(uint8_t) { return false; }
inline uint8_t channelsQuietBuiltIn() { return 0; }
inline void channelCapacity(uint8_t& a, uint8_t& b, uint8_t& c, uint8_t& d) { a = b = c = d = 0; }
static const uint8_t SPECTRUM_BINS = 141;
inline uint8_t spectrum(uint8_t*, uint8_t*, uint8_t) { return 0; }
inline uint32_t spectrumSweeps() { return 0; }
inline void setTap(bool) {}
inline bool tap() { return false; }
inline bool console(const char*) { return false; }
#endif

}
