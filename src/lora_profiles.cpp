// SquachWatch-CYD — the receive profiles. See include/lora_profiles.h and
// docs/LORA.md section 4 for where each number comes from.
#include "lora_profiles.h"
#include "lora_crypto.h"
#include <stdio.h>

namespace Lora {

// name, freq, bw(x100Hz), sf, cr, sync, preamble, flags, implicitLen, hint, group
static const Profile PROFILES[] = {
    // ---- Meshtastic and MeshCore on 869 MHz: what Germany actually runs ----
    // Every 250 kHz Meshtastic preset in EU_868 lands on 869.525; the SF is
    // what tells LongFast, MediumFast and ShortSlow apart.
    { "MT LongFast",    869525000, 2500, 11, 5, 0x2B, 16, PF_CRC,          0, Proto::MESHTASTIC, PG_MESH_EU },
    { "MT MediumFast",  869525000, 2500,  9, 5, 0x2B, 16, PF_CRC,          0, Proto::MESHTASTIC, PG_MESH_EU },
    { "MT ShortSlow",   869525000, 2500,  8, 5, 0x2B, 16, PF_CRC,          0, Proto::MESHTASTIC, PG_MESH_EU },
    // MeshCore's EU/UK Narrow: SF8 at 62.5 kHz, 32-symbol preamble since 1.16.
    { "MC EU Narrow",   869618000,  625,  8, 8, 0x12, 32, PF_CRC,          0, Proto::MESHCORE,   PG_MESH_EU },
    // The old MeshCore EU preset: the same PHY as LongFast, only the sync word differs.
    { "MC EU old",      869525000, 2500, 11, 5, 0x12, 16, PF_CRC,          0, Proto::MESHCORE,   PG_MESH_EU },
    { "MT NarrowSlow",  869442000,  625,  8, 6, 0x2B, 16, PF_CRC,          0, Proto::MESHTASTIC, PG_MESH_EU },
    { "MT LongSlow",    869462500, 1250, 12, 8, 0x2B, 16, PF_CRC,          0, Proto::MESHTASTIC, PG_MESH_EU },
    { "MT LongMod",     869587500, 1250, 11, 8, 0x2B, 16, PF_CRC,          0, Proto::MESHTASTIC, PG_MESH_EU },
    { "MT MediumSlow",  869525000, 2500, 10, 5, 0x2B, 16, PF_CRC,          0, Proto::MESHTASTIC, PG_MESH_EU },
    { "MT ShortFast",   869525000, 2500,  7, 5, 0x2B, 16, PF_CRC,          0, Proto::MESHTASTIC, PG_MESH_EU },
    { "MC CZ Narrow",   869432000,  625,  7, 5, 0x12, 32, PF_CRC,          0, Proto::MESHCORE,   PG_MESH_EU },

    // ---- LoRaWAN EU868 ------------------------------------------------------
    // Uplinks: sync 0x34, CRC on, normal IQ. One channel and one SF at a
    // time; the three mandatory channels first, the TTN five after.
    { "WAN 868.1 SF7",  868100000, 1250,  7, 5, 0x34,  8, PF_CRC,          0, Proto::LORAWAN,    PG_LORAWAN },
    { "WAN 868.1 SF12", 868100000, 1250, 12, 5, 0x34,  8, PF_CRC,          0, Proto::LORAWAN,    PG_LORAWAN },
    { "WAN 868.3 SF7",  868300000, 1250,  7, 5, 0x34,  8, PF_CRC,          0, Proto::LORAWAN,    PG_LORAWAN },
    { "WAN 868.5 SF7",  868500000, 1250,  7, 5, 0x34,  8, PF_CRC,          0, Proto::LORAWAN,    PG_LORAWAN },
    { "WAN 868.1 SF9",  868100000, 1250,  9, 5, 0x34,  8, PF_CRC,          0, Proto::LORAWAN,    PG_LORAWAN },
    { "WAN 868.3 SF12", 868300000, 1250, 12, 5, 0x34,  8, PF_CRC,          0, Proto::LORAWAN,    PG_LORAWAN },
    { "WAN 868.5 SF12", 868500000, 1250, 12, 5, 0x34,  8, PF_CRC,          0, Proto::LORAWAN,    PG_LORAWAN },
    { "WAN 867.1 SF7",  867100000, 1250,  7, 5, 0x34,  8, PF_CRC,          0, Proto::LORAWAN,    PG_LORAWAN },
    { "WAN 867.3 SF7",  867300000, 1250,  7, 5, 0x34,  8, PF_CRC,          0, Proto::LORAWAN,    PG_LORAWAN },
    { "WAN 867.5 SF7",  867500000, 1250,  7, 5, 0x34,  8, PF_CRC,          0, Proto::LORAWAN,    PG_LORAWAN },
    { "WAN 867.7 SF7",  867700000, 1250,  7, 5, 0x34,  8, PF_CRC,          0, Proto::LORAWAN,    PG_LORAWAN },
    { "WAN 867.9 SF7",  867900000, 1250,  7, 5, 0x34,  8, PF_CRC,          0, Proto::LORAWAN,    PG_LORAWAN },
    // RX2 downlinks: inverted IQ, no CRC, 10-symbol preamble. TTN uses SF9
    // here, the spec (and Helium) SF12.
    { "WAN RX2 SF9",    869525000, 1250,  9, 5, 0x34, 10, PF_INVERT,       0, Proto::LORAWAN,    PG_LORAWAN },
    { "WAN RX2 SF12",   869525000, 1250, 12, 5, 0x34, 10, PF_INVERT,       0, Proto::LORAWAN,    PG_LORAWAN },
    // The Class B beacon: implicit header, 17 bytes, no CRC, every 128 s.
    { "WAN Beacon",     869525000, 1250,  9, 5, 0x34, 10, PF_IMPLICIT,    17, Proto::LORAWAN,    PG_LORAWAN },
    { "WAN 868.3 SF7B", 868300000, 2500,  7, 5, 0x34,  8, PF_CRC,          0, Proto::LORAWAN,    PG_LORAWAN }, // DR6

    // ---- Aviation -------------------------------------------------------------
    { "FANET",          868200000, 2500,  7, 5, 0xF1,  8, PF_CRC,          0, Proto::FANET,      PG_AIR },

    // ---- 433 MHz: the amateur allocations and the 433 mesh presets ---------
    // Below the module's band. It hears strong signals here and nothing else,
    // and it must never transmit here; see docs/LORA.md section 1.
    { "APRS 433.775",   433775000, 1250, 12, 5, 0x12,  8, PF_CRC|PF_HAM|PF_433, 0, Proto::APRS,    PG_HAM_433 },
    { "APRS 433.900",   433900000, 1250, 12, 5, 0x12,  8, PF_CRC|PF_HAM|PF_433, 0, Proto::APRS,    PG_HAM_433 },
    { "MeshCom",        433175000, 2500, 11, 6, 0x2B, 32, PF_CRC|PF_HAM|PF_433, 0, Proto::MESHCOM, PG_HAM_433 },
    { "MT EU433 LF",    433875000, 2500, 11, 5, 0x2B, 16, PF_CRC|PF_433,   0, Proto::MESHTASTIC, PG_HAM_433 },
    { "MT 70cm Narrow", 433650000,  625,  8, 6, 0x2B, 16, PF_CRC|PF_HAM|PF_433, 0, Proto::MESHTASTIC, PG_HAM_433 },
    { "MC EU433 Nrw",   433650000,  625,  8, 8, 0x12, 32, PF_CRC|PF_433,   0, Proto::MESHCORE,   PG_HAM_433 },
    { "MC EU433 LR",    433650000, 2500, 11, 5, 0x12, 16, PF_CRC|PF_433,   0, Proto::MESHCORE,   PG_HAM_433 },
    { "MT 434.100",     434100000, 1250, 11, 8, 0x2B, 16, PF_CRC|PF_HAM|PF_433, 0, Proto::MESHTASTIC, PG_HAM_433 }, // DARC band plan

    // ---- Other ---------------------------------------------------------------
    { "RadioLib dflt",  434000000, 1250,  9, 7, 0x12,  8, PF_CRC|PF_433,   0, Proto::UNKNOWN,    PG_OTHER },
    { "Heltec demo",    868000000, 1250,  7, 5, 0x12,  8, PF_CRC,          0, Proto::UNKNOWN,    PG_OTHER },
    { "LoRaMesher",     869900000, 1250,  7, 5, 0x14,  8, PF_CRC,          0, Proto::UNKNOWN,    PG_OTHER },
    { "RNode 867.2",    867200000, 1250,  8, 5, 0x12, 18, PF_CRC,          0, Proto::RETICULUM,  PG_OTHER },
    { "MT LiteFast",    866300000, 1250,  9, 5, 0x2B, 16, PF_CRC,          0, Proto::MESHTASTIC, PG_OTHER },
    { "TS011 WOR",      865100000, 1250,  9, 5, 0x34,  8, PF_CRC|PF_INVERT, 0, Proto::LORAWAN,   PG_OTHER },
};

const Profile* profiles()     { return PROFILES; }
uint8_t        profileCount() { return (uint8_t)(sizeof(PROFILES) / sizeof(PROFILES[0])); }
const Profile& profile(uint8_t i) { return PROFILES[i < profileCount() ? i : 0]; }
uint8_t        defaultProfile() { return 0; }

uint32_t meshtasticSlotHz(uint32_t startHz, uint32_t endHz, uint16_t bwKhz10,
                          uint32_t spacingHz, uint32_t paddingHz, const char* name) {
    const uint32_t bwHz = (uint32_t)bwKhz10 * 100u;
    const uint32_t slotWidth = spacingHz + 2 * paddingHz + bwHz;
    if (!slotWidth) return startHz;
    // round((end - start + spacing) / slotWidth)
    const uint32_t span = endHz - startHz + spacingHz;
    uint32_t numSlots = (span + slotWidth / 2) / slotWidth;
    if (!numSlots) numSlots = 1;
    const uint32_t slot = LoraCrypto::djb2(name) % numSlots;
    return startHz + bwHz / 2 + paddingHz + slot * slotWidth;
}

void formatMHz(uint32_t hz, char* out, size_t cap) {
    const uint32_t khz = (hz + 500) / 1000;
    snprintf(out, cap, "%lu.%03lu", (unsigned long)(khz / 1000), (unsigned long)(khz % 1000));
}

void formatBw(uint16_t bwKhz10, char* out, size_t cap) {
    if (bwKhz10 % 10 == 0) snprintf(out, cap, "%uk", (unsigned)(bwKhz10 / 10));
    else                   snprintf(out, cap, "%u.%uk", (unsigned)(bwKhz10 / 10), (unsigned)(bwKhz10 % 10));
}

}
