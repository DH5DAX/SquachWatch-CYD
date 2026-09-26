// SquachWatch-CYD — the receive profiles: where each LoRa network lives.
//
// One radio, one profile at a time. The table is the listening plan in
// docs/LORA.md section 4, most valuable first, so "the first N" is always a
// sensible survey. Standalone, for the host tests.
#pragma once
#include "lora_pkt.h"

namespace Lora {

// The table and its length. Indices are stable within a build and are what
// a Packet's `profile` field holds, so nothing persists them.
const Profile* profiles();
uint8_t        profileCount();
const Profile& profile(uint8_t i);   // clamped

// The index the survey starts on, and what FOCUS mode falls back to: the
// German Meshtastic default.
uint8_t defaultProfile();

// Meshtastic's frequency-slot rule (RadioInterface.cpp), for a channel with a
// name that is not a preset's: the slot is a djb2 hash of the name. Region
// edges in Hz, bandwidth in units of 100 Hz. Returns the centre frequency.
uint32_t meshtasticSlotHz(uint32_t startHz, uint32_t endHz, uint16_t bwKhz10,
                          uint32_t spacingHz, uint32_t paddingHz, const char* name);

// A short label for a frequency, "869.525", into a 10-byte buffer.
void formatMHz(uint32_t hz, char* out, size_t cap);
// "250k", "62.5k", "7.8k"
void formatBw(uint16_t bwKhz10, char* out, size_t cap);

}
