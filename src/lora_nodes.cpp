// SquachWatch-CYD — the node table. See include/lora_nodes.h.
#include "lora_nodes.h"
#include "lora_meshtastic.h"
#include "lora_meshcore.h"
#include "lora_lorawan.h"
#include "lora_aprs.h"
#include "lora_fanet.h"
#include <stdio.h>
#include <string.h>

namespace Lora {
namespace Nodes {

static const uint8_t CAP = 96;
static Node    s_nodes[CAP];
static uint8_t s_n = 0;

uint8_t     count() { return s_n; }
const Node* at(uint8_t i) { return i < s_n ? &s_nodes[i] : nullptr; }
void        clear() { s_n = 0; }

static Node* find(Proto p, uint64_t id) {
    for (uint8_t i = 0; i < s_n; i++) if (s_nodes[i].proto == p && s_nodes[i].id == id) return &s_nodes[i];
    return nullptr;
}

// A row for this sender, new or found. When the table is full the one
// unheard for longest gives way.
static Node* get(Proto p, uint64_t id, uint32_t now) {
    Node* n = find(p, id);
    if (n) return n;
    if (s_n < CAP) n = &s_nodes[s_n++];
    else {
        n = &s_nodes[0];
        for (uint8_t i = 1; i < CAP; i++) if (s_nodes[i].lastMs < n->lastMs) n = &s_nodes[i];
    }
    memset(n, 0, sizeof *n);
    n->proto = p; n->id = id; n->firstMs = now; n->hourStartMs = now;
    n->rssi = -200;
    return n;
}

static void heard(Node& n, const Packet& pk, uint32_t now) {
    n.lastMs = now;
    n.rssi = pk.rssi; n.snr4 = pk.snr4;
    n.packets++;
    n.airtimeMs += pk.toaUs / 1000;
    if (now - n.hourStartMs >= 3600000u) { n.hourStartMs = now; n.hourAirMs = 0; }
    n.hourAirMs += pk.toaUs / 1000;
    if (dutyPermille(n, now) > 100 && now - n.hourStartMs > 300000u) n.flags |= NF_DUTY; else n.flags &= (uint8_t)~NF_DUTY;
}

uint16_t dutyPermille(const Node& n, uint32_t now) {
    const uint32_t span = now - n.hourStartMs;
    if (span < 1000) return 0;
    return (uint16_t)(((uint64_t)n.hourAirMs * 1000ull) / span);
}

static void noteMeshtastic(const Packet& pk, uint32_t now) {
    Meshtastic::Decoded d;
    if (!Meshtastic::decode(pk, d)) return;
    Node& n = *get(Proto::MESHTASTIC, d.hdr.from, now);
    if (!n.tag[0]) Meshtastic::nodeId(d.hdr.from, n.tag);
    heard(n, pk, now);
    n.hopLimit = d.hdr.hopLimit;
    n.hops = Meshtastic::hopsAway(d.hdr);
    if (d.hdr.hopLimit > 3 || d.hdr.hopStart > 3) n.flags |= NF_HOPS_HIGH;
    if (d.hdr.viaMqtt) n.flags |= NF_MQTT;
    if (!d.hdr.hopStart || !d.hdr.relayNode) n.flags |= NF_OLD_FW;
    if (!d.haveData) return;
    switch (d.data.portnum) {
        case Meshtastic::PORT_NODEINFO:
            if (d.user.longName[0]) strncpy(n.name, d.user.longName, sizeof n.name - 1);
            if (d.user.shortName[0]) { snprintf(n.tag, sizeof n.tag, "%s", d.user.shortName); }
            n.role = d.user.role;
            if (d.user.role == 3 || d.user.role == 4) n.flags |= NF_DEPRECATED;
            break;
        case Meshtastic::PORT_POSITION:
            if (d.pos.hasLatLon) { n.hasPos = true; n.latE7 = d.pos.latI; n.lonE7 = d.pos.lonI; }
            if (n.lastTelemMs && now - n.lastTelemMs < 5 * 60000u) n.flags |= NF_CHATTY;
            n.lastTelemMs = now;
            break;
        case Meshtastic::PORT_TELEMETRY:
            // The default is an hour; under thirty minutes is the floor the
            // firmware enforces on the default channel.
            if (n.lastTelemMs && now - n.lastTelemMs < 30 * 60000u) n.flags |= NF_CHATTY;
            n.lastTelemMs = now;
            break;
        default: break;
    }
}

static void noteMeshCore(const Packet& pk, uint32_t now, uint32_t epoch) {
    MeshCore::Decoded d;
    if (!MeshCore::decode(pk, d)) return;
    const MeshCore::Frame& f = d.f;
    // Adverts name their sender in full. Everything else names only the
    // last relay (the path's newest hash) or a destination, which is not
    // the sender -- so those count against the relay's row, as relayed.
    if (d.haveAdvert) {
        uint64_t id = 0;
        for (int i = 0; i < 8; i++) id = (id << 8) | d.adv.pubkey[i];
        Node& n = *get(Proto::MESHCORE, id, now);
        heard(n, pk, now);
        snprintf(n.tag, sizeof n.tag, "%02x%02x%02x", d.adv.pubkey[0], d.adv.pubkey[1], d.adv.pubkey[2]);
        if (d.adv.hasName) strncpy(n.name, d.adv.name, sizeof n.name - 1);
        n.role = d.adv.nodeType;
        n.hops = f.hops;
        n.counter++;
        if (d.adv.hasLatLon) { n.hasPos = true; n.latE7 = d.adv.latE6 * 10; n.lonE7 = d.adv.lonE6 * 10; }
        if (epoch) {
            const uint32_t diff = d.adv.timestamp > epoch ? d.adv.timestamp - epoch : epoch - d.adv.timestamp;
            if (diff > 86400u) n.flags |= NF_BAD_CLOCK;
        }
        return;
    }
    if (f.hops && f.hashSize == 1 && (f.route == MeshCore::ROUTE_FLOOD || f.route == MeshCore::ROUTE_TRANSPORT_FLOOD)) {
        // The newest path byte is the repeater that just sent this copy.
        const uint8_t h = f.path[f.hops - 1];
        // Match it to an advertised key when one is known; else its own row.
        Node* n = nullptr;
        for (uint8_t i = 0; i < s_n; i++)
            if (s_nodes[i].proto == Proto::MESHCORE && (uint8_t)(s_nodes[i].id >> 56) == h && s_nodes[i].name[0]) { n = &s_nodes[i]; break; }
        if (!n) {
            n = get(Proto::MESHCORE, 0x0100000000000000ull | h, now);
            if (!n->tag[0]) snprintf(n->tag, sizeof n->tag, "rpt %02x", h);
        }
        heard(*n, pk, now);
        n->hops = f.hops;
    }
}

static void noteLoRaWAN(const Packet& pk, uint32_t now) {
    LoRaWAN::Decoded d;
    if (!LoRaWAN::decode(pk, d) || d.isBeacon) return;
    const LoRaWAN::Frame& f = d.f;
    if (f.mtype == LoRaWAN::JOIN_REQUEST) {
        uint64_t id = 0;
        for (int i = 0; i < 8; i++) id = (id << 8) | f.devEui[i];
        Node& n = *get(Proto::LORAWAN, id, now);
        heard(n, pk, now);
        snprintf(n.tag, sizeof n.tag, "%02x%02x..%02x%02x", f.devEui[0], f.devEui[1], f.devEui[6], f.devEui[7]);
        strncpy(n.name, d.maker ? d.maker : "join request", sizeof n.name - 1);
        n.role = 1;
        n.counter++;
        if (n.counter > 3 && now - n.firstMs < 10 * 60000u) n.flags |= NF_CHATTY;   // joining over and over
        return;
    }
    if (f.mtype == LoRaWAN::JOIN_ACCEPT || f.mtype == LoRaWAN::PROPRIETARY) return;
    Node& n = *get(Proto::LORAWAN, f.devAddr, now);
    const bool fresh = n.packets == 0;
    heard(n, pk, now);
    snprintf(n.tag, sizeof n.tag, "%08lx", (unsigned long)f.devAddr);
    if (d.net.op) strncpy(n.name, d.net.op, sizeof n.name - 1);
    n.role = f.uplink ? 0 : 2;
    if (f.uplink) {
        if (!fresh) {
            if (f.fCnt > n.counter + 1) n.lost = (uint16_t)(n.lost + (f.fCnt - n.counter - 1));
            else if (f.fCnt < n.counter) n.flags |= NF_RESETS;
        }
        n.counter = f.fCnt;
    }
}

static uint64_t textId(const char* s) {
    // FNV-1a over a callsign: stable, and a collision needs two hams to
    // share a hash, which the screen would show as one row.
    uint64_t h = 0xcbf29ce484222325ull;
    for (; *s; s++) { h ^= (uint8_t)*s; h *= 0x100000001b3ull; }
    return h;
}

static void noteAprs(const Packet& pk, uint32_t now) {
    Aprs::Frame f;
    if (!Aprs::parse(pk.data, pk.len, f)) return;
    Node& n = *get(Proto::APRS, textId(f.src), now);
    heard(n, pk, now);
    strncpy(n.tag, f.src, sizeof n.tag - 1);
    if (f.info.hasPos) { n.hasPos = true; n.latE7 = f.info.latE7; n.lonE7 = f.info.lonE7; }
    n.hops = f.digipeated;
    if (f.info.kind == Aprs::INFO_STATUS && f.info.text[0]) strncpy(n.name, f.info.text, sizeof n.name - 1);
}

static void noteMeshCom(const Packet& pk, uint32_t now) {
    MeshCom::Frame f;
    if (!MeshCom::parse(pk.data, pk.len, f) || f.type == 'A') return;
    Node& n = *get(Proto::MESHCOM, textId(f.src), now);
    heard(n, pk, now);
    strncpy(n.tag, f.src, sizeof n.tag - 1);
    const char* hw = MeshCom::hwName(f.hwId);
    if (hw) snprintf(n.name, sizeof n.name, "%s fw%u", hw, (unsigned)f.fwVersion);
    else snprintf(n.name, sizeof n.name, "hw%u fw%u", (unsigned)f.hwId, (unsigned)f.fwVersion);
    if (f.hasInfo) { n.hasPos = true; n.latE7 = f.info.latE7; n.lonE7 = f.info.lonE7; }
    n.hops = f.hopsLeft;
    n.role = f.viaServer ? 1 : 0;
}

static void noteFanet(const Packet& pk, uint32_t now) {
    Fanet::Frame f;
    if (!Fanet::parse(pk.data, pk.len, f)) return;
    Node& n = *get(Proto::FANET, ((uint64_t)f.manufacturer << 16) | f.id, now);
    heard(n, pk, now);
    Fanet::addressText(f.manufacturer, f.id, n.tag);
    if (f.type == Fanet::T_NAME && f.text[0]) strncpy(n.name, f.text, sizeof n.name - 1);
    else if (!n.name[0]) { const char* m = Fanet::manufacturerName(f.manufacturer); if (m) strncpy(n.name, m, sizeof n.name - 1); }
    if (f.hasPos) { n.hasPos = true; n.latE7 = f.latE7; n.lonE7 = f.lonE7; }
    if (f.type == Fanet::T_TRACKING) n.role = (uint8_t)(0x10 | f.aircraft);
    else if (f.type == Fanet::T_GROUND) n.role = (uint8_t)(0x20 | f.groundType);
    else if (f.type == Fanet::T_SERVICE) n.role = 0x30;
}

void note(const Packet& pk, uint32_t now, uint32_t epoch) {
    if (pk.flags & PK_CRC_ERR) return;
    switch (pk.proto) {
        case Proto::MESHTASTIC: noteMeshtastic(pk, now); break;
        case Proto::MESHCORE:   noteMeshCore(pk, now, epoch); break;
        case Proto::LORAWAN:    noteLoRaWAN(pk, now); break;
        case Proto::APRS:       noteAprs(pk, now); break;
        case Proto::MESHCOM:    noteMeshCom(pk, now); break;
        case Proto::FANET:      noteFanet(pk, now); break;
        default: break;
    }
}

uint8_t order(uint8_t* idx, uint8_t cap) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < s_n && n < cap; i++) idx[n++] = i;
    // Insertion sort by lastMs, newest first: the table is small.
    for (uint8_t i = 1; i < n; i++) {
        const uint8_t v = idx[i];
        int j = i - 1;
        while (j >= 0 && s_nodes[idx[j]].lastMs < s_nodes[v].lastMs) { idx[j + 1] = idx[j]; j--; }
        idx[j + 1] = v;
    }
    return n;
}

const char* roleText(const Node& n) {
    switch (n.proto) {
        case Proto::MESHTASTIC: return Meshtastic::roleName(n.role);
        case Proto::MESHCORE:   return n.role ? MeshCore::nodeTypeName(n.role) : "relay";
        case Proto::LORAWAN:    return n.role == 1 ? "joining" : n.role == 2 ? "downlink" : "device";
        case Proto::APRS:       return n.hops ? "digipeated" : "direct";
        case Proto::MESHCOM:    return n.role ? "via server" : "rf";
        case Proto::FANET:
            if ((n.role & 0xF0) == 0x10) return Fanet::aircraftName(n.role & 0x0F);
            if ((n.role & 0xF0) == 0x20) return "ground";
            if (n.role == 0x30) return "station";
            return "";
        default: return "";
    }
}

const char* flagsText(const Node& n, char* out, size_t cap) {
    size_t o = 0;
    if (cap) out[0] = '\0';
    struct { uint8_t f; const char* s; } names[] = {
        { NF_HOPS_HIGH, "hops" }, { NF_DEPRECATED, "oldrole" }, { NF_MQTT, "mqtt" }, { NF_CHATTY, "chatty" },
        { NF_DUTY, "duty" }, { NF_BAD_CLOCK, "clock" }, { NF_RESETS, "reset" }, { NF_OLD_FW, "oldfw" },
    };
    for (size_t i = 0; i < sizeof names / sizeof names[0]; i++)
        if ((n.flags & names[i].f) && o + strlen(names[i].s) + 2 < cap)
            o += (size_t)snprintf(out + o, cap - o, "%s%s", o ? " " : "", names[i].s);
    return out;
}

}
}
