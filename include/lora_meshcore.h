// SquachWatch-CYD — MeshCore, off the air.
//
// Adverts are plaintext and signed; the relay path is in the clear on every
// packet; the Public channel and any hashtag channel have derivable keys.
// Direct messages are between two nodes' keys and stay shut. docs/LORA.md
// section 3.2 has the layout and the sources.
//
// Standalone, for the host tests.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "lora_pkt.h"

namespace MeshCore {

enum Route : uint8_t { ROUTE_TRANSPORT_FLOOD = 0, ROUTE_FLOOD = 1, ROUTE_DIRECT = 2, ROUTE_TRANSPORT_DIRECT = 3 };
enum Type : uint8_t {
    TYPE_REQ = 0, TYPE_RESPONSE = 1, TYPE_TXT_MSG = 2, TYPE_ACK = 3, TYPE_ADVERT = 4, TYPE_GRP_TXT = 5,
    TYPE_GRP_DATA = 6, TYPE_ANON_REQ = 7, TYPE_PATH = 8, TYPE_TRACE = 9, TYPE_MULTIPART = 10,
    TYPE_CONTROL = 11, TYPE_RAW_CUSTOM = 15
};
const char* typeName(uint8_t t);
const char* routeName(uint8_t r);

struct Frame {
    uint8_t        route, type, version;
    bool           hasTransport;
    uint16_t       transport1, transport2;
    uint8_t        hashSize;       // bytes per path entry, 1 or 2
    uint8_t        hops;           // entries in the path
    const uint8_t* path;
    const uint8_t* payload;
    uint8_t        payloadLen;
};
bool parse(const uint8_t* d, uint8_t len, Frame& f);

// ---- adverts ---------------------------------------------------------------
enum NodeType : uint8_t { NODE_CHAT = 1, NODE_REPEATER = 2, NODE_ROOM = 3, NODE_SENSOR = 4 };
const char* nodeTypeName(uint8_t t);

struct Advert {
    uint8_t  pubkey[32];
    uint32_t timestamp;
    uint8_t  signature[64];
    uint8_t  nodeType;
    bool     hasLatLon;
    int32_t  latE6, lonE6;
    bool     hasName;
    char     name[33];
};
// The layout only; the Ed25519 signature is carried, not checked.
bool parseAdvert(const Frame& f, Advert& a);

// ---- group channels ---------------------------------------------------------
struct Channel {
    char    name[24];
    uint8_t key[16];
    uint8_t hash;        // SHA256(key)[0]
};
// SHA256("#name")[:16]: a hashtag channel's key is its name.
void    hashtagKey(const char* nameWithHash, uint8_t key[16]);
uint8_t channelHash(const uint8_t key[16]);
// The Public channel is built in; the user may add hashtags and keys.
uint8_t        channelCount();
const Channel& channel(uint8_t i);
bool           addChannel(const char* name, const char* keyTextOrNull);   // no key: derived from "#name"
void           clearUserChannels();

// AES-128-ECB with a 2-byte HMAC-SHA256 in front, the way every MeshCore
// channel and message is sealed. The MAC is checked before anything is
// decrypted, so a wrong key is a clean "no" and never a wrong answer.
bool openSealed(const uint8_t key[16], const uint8_t* mac2, const uint8_t* cipher, uint8_t cipherLen,
                uint8_t* plain, uint8_t& plainLen);

struct GroupText {
    uint32_t timestamp;
    uint8_t  flags;
    char     sender[40];   // whatever came before ": " -- claimed, not verified
    char     text[160];
};
bool parseGroupText(const uint8_t* plain, uint8_t len, GroupText& g);

// ---- trace and control ----------------------------------------------------
struct Trace {
    uint32_t tag, auth;
    uint8_t  flags;
    uint8_t  nHashes;
    uint8_t  hashes[32];   // the repeaters asked to relay, hashSize bytes each
    uint8_t  nSnr;
    int8_t   snr4[32];     // SNR x 4 per hop so far, from the path
};
bool parseTrace(const Frame& f, Trace& t);

// ---- the whole thing ----------------------------------------------------------
struct Decoded {
    Frame     f;
    bool      haveAdvert;
    Advert    adv;
    bool      haveGroup;
    uint8_t   channelIdx;
    GroupText grp;
    bool      haveTrace;
    Trace     trace;
    uint8_t   plain[192];
    // For the sealed types: the one-byte destination and source hashes.
    uint8_t   destHash, srcHash;
    uint8_t   ackCrc[4];
};
bool decode(const Lora::Packet& pk, Decoded& out);
void summary(const Lora::Packet& pk, char* out, size_t cap);

}
