// SquachWatch-CYD — who is on the air: one row per LoRa transmitter.
//
// Every decoder names its sender somehow -- a node number, a public key's
// first bytes, a DevAddr, a callsign, a FANET address -- and this keeps one
// row per (network, name) with what has been learned about it: the name it
// gave, its role, its last position, how loud, how often, how much airtime,
// and the things a sysop would want flagged. Standalone, for the host tests;
// the sniffer feeds it and the NODES view reads it.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "lora_pkt.h"

namespace Lora {
namespace Nodes {

enum NodeFlags : uint8_t {
    NF_HOPS_HIGH  = 0x01,   // Meshtastic: a hop limit above the default 3
    NF_DEPRECATED = 0x02,   // Meshtastic: ROUTER_CLIENT or REPEATER
    NF_MQTT       = 0x04,   // Meshtastic: relaying MQTT traffic onto the air
    NF_CHATTY     = 0x08,   // telemetry or positions more often than the defaults
    NF_DUTY       = 0x10,   // over 10 % of the last hour on the air
    NF_BAD_CLOCK  = 0x20,   // MeshCore: an advert timestamp far from our clock
    NF_RESETS     = 0x40,   // LoRaWAN: the frame counter went backwards
    NF_OLD_FW     = 0x80,   // Meshtastic: no hop_start (before 2.3) or no relay byte (before 2.6)
};

struct Node {
    Proto    proto;
    uint64_t id;
    char     tag[12];       // the short handle: "!3b1c9a2e", "40 DL Rep", "26011b2c", "DH5DAX-7", "11:A3F0"
    char     name[24];      // what it called itself, or who runs it
    uint8_t  role;          // the network's own role byte
    bool     hasPos;
    int32_t  latE7, lonE7;
    int16_t  rssi;
    int8_t   snr4;
    uint16_t packets;
    uint32_t firstMs, lastMs;
    uint32_t airtimeMs;     // total time on air heard from it
    uint32_t hourAirMs;     // in the current hour bucket
    uint32_t hourStartMs;
    uint8_t  hops;          // last hops away / path length
    uint8_t  hopLimit;      // last hop limit seen (Meshtastic)
    uint16_t counter;       // LoRaWAN FCnt; MeshCore advert count
    uint16_t lost;          // LoRaWAN: frames the counter skipped
    uint32_t lastTelemMs;   // for NF_CHATTY
    uint8_t  flags;
};

// Feed a classified packet in. `now` is millis(); `epoch` the wall clock
// or 0. Decodes what it needs itself.
void note(const Packet& pk, uint32_t now, uint32_t epoch);

uint8_t     count();
const Node* at(uint8_t i);
// Indices sorted newest-heard first, into idx (cap entries). Returns how many.
uint8_t     order(uint8_t* idx, uint8_t cap);
void        clear();
// Airtime over the last hour as a per-mille of the hour so far.
uint16_t    dutyPermille(const Node& n, uint32_t now);
const char* roleText(const Node& n);    // the network's word for its role
const char* flagsText(const Node& n, char* out, size_t cap);   // "hops mqtt duty"

}
}
