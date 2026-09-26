// SquachWatch-CYD — the online half of node enrichment. See include/lora_enrich.h
// for the five rules this file exists to obey; they are not repeated here.
//
// WHAT IS PURE AND WHAT IS NOT. Everything below except the two functions at
// the very bottom is arithmetic and string work over buffers: the queue, the
// order, the back-off, the rate-limit stall, the cache, the log and both
// response parsers. Only the transport touches HTTPClient, it sits behind a
// pointer, and the host test installs a canned responder in its place -- so a
// desktop checks the policy that decides what leaves the device, and the board
// is left to prove only that a socket works. There is no WiFi shim in sim/,
// which is why that split is mandatory rather than tidy.
#include "lora_enrich.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if SQUACH_LORA
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#endif

namespace Lora {
namespace Enrich {
namespace {

// ---- the sources -------------------------------------------------------------
struct SourceDef {
    const char* name;
    const char* host;
    const char* path;       // one %s, the identifier
    uint32_t    spacingMs;
};
const SourceDef SOURCES[SRC_COUNT] = {
    // Verified 2026-09-26: 200 over plain http, no redirect, 934 B on a hit,
    // 367 B on a "NOT_FOUND" miss, 404 with 30 B on the other kind of miss.
    // Three response shapes on one route, and the parser handles all three.
    { "callsign", "hamrig.com",         "/api/public/callsign-db/%s",     1000 },
    // Verified 2026-09-26: 200 over plain http, 148 B, and the body is wrapped
    // in {"devices":[ ... ]} -- the schema's envelope, which a single-device
    // query keeps.
    { "aircraft", "ddb.glidernet.org",  "/download/?j=1&device_id=%s",   12000 },
};

// ---- state -------------------------------------------------------------------
struct Entry {
    uint64_t key;
    Proto    proto;
    uint8_t  source;
    uint8_t  tries;
    uint32_t nextMs;        // not before this
    char     text[16];
    // The order, kept per entry rather than recomputed: the snapshot the scan
    // read is gone by the time the request goes out.
    bool     direct;
    uint16_t packets;
    uint32_t lastMs;
    bool     busy;          // out on the wire: not picked twice, not displaced
};

Entry    s_q[QUEUE_MAX];
uint8_t  s_qn = 0;
uint32_t s_srcNextMs[SRC_COUNT] = { 0, 0 };

Record*  s_cache = nullptr;         // PSRAM; see begin()
char*    s_body  = nullptr;         // PSRAM, beside it
size_t   s_bodyCap = 0;
uint8_t  s_cn = 0;
uint32_t s_writes = 0;

LogRow   s_log[LOG_MAX];
uint8_t  s_ln = 0;                  // rows used
uint8_t  s_lhead = 0;               // next slot

bool s_master = false, s_ham = false, s_ogn = false;
uint8_t s_sent = 0, s_hit = 0, s_miss = 0, s_noAnswer = 0;

Fetch s_fetch = nullptr;
NetUp s_netUp = nullptr;

// The cache and the log are written by the worker task and read by loop() for
// the screen, so they are guarded -- the same shape as the sniffer's own lock,
// and compiled away on a desktop where there is one thread.
#if SQUACH_LORA
SemaphoreHandle_t s_lock = nullptr;
inline void lock()   { if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY); }
inline void unlock() { if (s_lock) xSemaphoreGive(s_lock); }
#else
inline void lock()   {}
inline void unlock() {}
#endif

// ---- a JSON scanner, not a JSON library --------------------------------------
// src/ota_wifi.cpp:238 establishes the pattern and the reason: a scanner reads
// the handful of keys it wants and cannot be surprised by the rest of the
// document. Here that is the whole defence -- a key this does not name is a
// key that never reaches RAM this side of the socket buffer.

// The value of "key" as a string, searched only between from and to.
bool jsonStr(const char* from, const char* to, const char* key, char* out, size_t cap) {
    if (!out || !cap) return false;
    out[0] = '\0';
    char needle[24];
    snprintf(needle, sizeof needle, "\"%s\"", key);
    const size_t nl = strlen(needle);
    const char* p = from;
    while (p + nl <= to) {
        if (memcmp(p, needle, nl) == 0) break;
        p++;
    }
    if (p + nl > to) return false;
    p += nl;
    while (p < to && (*p == ' ' || *p == ':')) p++;
    if (p >= to || *p != '"') return false;      // null, a number or an object: not a string
    p++;
    size_t o = 0;
    while (p < to && *p != '"') {
        char c = *p++;
        if (c == '\\' && p < to) {
            const char e = *p++;
            // Escapes are read, not copied: \" would otherwise end the value
            // early and scramble everything after it. The fonts are ASCII, so
            // anything this cannot spell becomes a question mark.
            c = (e == '"' || e == '\\' || e == '/') ? e : '?';
            if (e == 'u') for (int i = 0; i < 4 && p < to; i++) p++;
        }
        if ((unsigned char)c >= 0x80) c = '?';
        if (o + 1 < cap) out[o++] = c;
    }
    out[o] = '\0';
    return o != 0;
}

// The object that "key" names, as [begin,end). Brace counting, string-aware,
// because the four keys this reads all live inside one object and identically
// named keys live in the siblings: hamrig's envelope carries a `source` both
// inside the callsign object and outside it, and its `hamrig_user` block is a
// sibling that could carry a `city` of its own.
bool jsonObj(const char* from, const char* to, const char* key,
             const char*& begin, const char*& end) {
    char needle[24];
    snprintf(needle, sizeof needle, "\"%s\"", key);
    const size_t nl = strlen(needle);
    const char* p = from;
    while (p + nl <= to && memcmp(p, needle, nl) != 0) p++;
    if (p + nl > to) return false;
    p += nl;
    while (p < to && (*p == ' ' || *p == ':')) p++;
    if (p >= to || *p != '{') return false;
    begin = p;
    int depth = 0;
    bool inStr = false;
    for (; p < to; p++) {
        if (inStr) {
            if (*p == '\\') p++;
            else if (*p == '"') inStr = false;
            continue;
        }
        if (*p == '"') inStr = true;
        else if (*p == '{') depth++;
        else if (*p == '}' && --depth == 0) { end = p + 1; return true; }
    }
    return false;
}

bool contains(const char* from, const char* to, const char* what) {
    const size_t n = strlen(what);
    for (const char* p = from; p + n <= to; p++) if (memcmp(p, what, n) == 0) return true;
    return false;
}

void put(char* dst, size_t cap, const char* src) {
    if (!src || !src[0]) { dst[0] = '\0'; return; }
    strncpy(dst, src, cap - 1);
    dst[cap - 1] = '\0';
}

}  // namespace

const char* sourceName(uint8_t s) { return s < SRC_COUNT ? SOURCES[s].name : "?"; }
const char* sourceHost(uint8_t s) { return s < SRC_COUNT ? SOURCES[s].host : "?"; }
const char* sourcePath(uint8_t s) { return s < SRC_COUNT ? SOURCES[s].path : ""; }
uint32_t sourceSpacingMs(uint8_t s) { return s < SRC_COUNT ? SOURCES[s].spacingMs : 0; }

const char* answerText(uint8_t a) {
    switch (a) {
        case ANS_WAIT:     return "asking";
        case ANS_HIT:      return "known";
        // Two different facts, said differently on purpose: "not in the
        // registry" is an answer, "we never managed to ask" is not.
        case ANS_MISS:     return "not listed";
        case ANS_NOANSWER: return "no answer";
        default:           return "";
    }
}

// ---- what would be sent ------------------------------------------------------
bool plan(const Nodes::Node& n, bool allowHam, bool allowOgn, Plan& out) {
    memset(&out, 0, sizeof out);
    switch (n.proto) {
        case Proto::APRS:
        case Proto::MESHCOM: {
            // The tag is the source callsign verbatim, straight off the air
            // (src/lora_nodes.cpp). Station identification, not a guess.
            if (!allowHam || !n.tag[0]) return false;
            out.source = SRC_HAM;
            // Without the SSID: -7 is which of an operator's stations it is,
            // and the licence is held by the operator.
            size_t i = 0;
            while (n.tag[i] && n.tag[i] != '-' && i + 1 < sizeof out.text) {
                out.text[i] = n.tag[i];
                i++;
            }
            out.text[i] = '\0';
            return i >= 3;      // shorter than three characters is not a callsign
        }
        case Proto::FANET: {
            // The row's id IS the 24-bit OGN address: manufacturer in the high
            // byte, then the 16-bit id (src/lora_nodes.cpp's noteFanet). The
            // database wants it as six hex digits, and a bare device_id query
            // is unambiguous -- no device_id in the whole dump appears under
            // more than one device_type, so the type letter never has to be
            // guessed. It is F for FANET registrations, not O.
            if (!allowOgn) return false;
            out.source = SRC_OGN;
            snprintf(out.text, sizeof out.text, "%02X%04X",
                     (unsigned)((n.id >> 16) & 0xFF), (unsigned)(n.id & 0xFFFF));
            return true;
        }
        // MESHTASTIC and MESHCORE: a callsign here is a substring of a name a
        // stranger typed. Rule 3 in the header, and it is the reason this
        // switch has no default that guesses.
        // LORAWAN: a DevAddr names its operator from a table in flash and no
        // public service maps one to anything else. RETICULUM and UNKNOWN:
        // there is no identifier to send.
        default:
            return false;
    }
}

// ---- the parsers -------------------------------------------------------------
Answer parseHamrig(const char* body, size_t len, int code, Record& out) {
    out.answer = ANS_NONE;
    out.what[0] = out.where[0] = out.grid[0] = out.extra[0] = '\0';
    // 404 with {"error":"Callsign not found"}: the first of this route's two
    // miss shapes. Nothing to retry.
    if (code == 404) { out.answer = ANS_MISS; return ANS_MISS; }
    if (code != 200 || !body || !len) { out.answer = ANS_NOANSWER; return ANS_NOANSWER; }
    const char* b = body;
    const char* e = body + len;
    // The second miss shape: 200, success true, and every field the literal
    // string "NOT_FOUND" with source "hamdb". Its field list includes
    // `address`, which is why this is checked before anything is read.
    if (contains(b, e, "\"NOT_FOUND\"") || contains(b, e, "\"error\"")) {
        out.answer = ANS_MISS;
        return ANS_MISS;
    }
    const char* ob = nullptr;
    const char* oe = nullptr;
    if (!jsonObj(b, e, "callsign", ob, oe)) {
        // We asked and something came back that this cannot read. Asking again
        // returns the same bytes, so it is remembered rather than retried --
        // and the record stays empty, so nothing false reaches a screen.
        out.answer = ANS_MISS;
        return ANS_MISS;
    }
    char v[40];
    if (jsonStr(ob, oe, "country", v, sizeof v))       put(out.what,  sizeof out.what,  v);
    if (jsonStr(ob, oe, "city", v, sizeof v))          put(out.where, sizeof out.where, v);
    if (jsonStr(ob, oe, "grid_square", v, sizeof v))   put(out.grid,  sizeof out.grid,  v);
    if (jsonStr(ob, oe, "license_class", v, sizeof v)) put(out.extra, sizeof out.extra, v);
    // Four keys read, and that is the end of this body's life. No name, no
    // photo URL, no coordinates, and no address under any of its three
    // spellings.
    const bool any = out.what[0] || out.where[0] || out.grid[0] || out.extra[0];
    out.answer = any ? ANS_HIT : ANS_MISS;
    return (Answer)out.answer;
}

Answer parseOgn(const char* body, size_t len, int code, Record& out) {
    out.answer = ANS_NONE;
    out.what[0] = out.where[0] = out.grid[0] = out.extra[0] = '\0';
    // 429 is what this server says when asked twice quickly, and it is a
    // failure to ask rather than an answer: see step().
    if (code != 200 || !body || !len) { out.answer = ANS_NOANSWER; return ANS_NOANSWER; }
    const char* b = body;
    const char* e = body + len;
    // {"devices":[]} -- the address is not registered. A real miss.
    if (contains(b, e, "\"devices\":[]")) { out.answer = ANS_MISS; return ANS_MISS; }
    char v[40];
    // tracked=N is the pilot's own opt-out, and the only place in any of these
    // feeds where a dataset hands the device an explicit consent signal. The
    // server already blanks registration, cn and aircraft_model on all 482
    // such rows, so there is nothing to suppress -- what a client can still get
    // wrong is showing the address of a device that asked not to be tracked, so
    // the record says so in words and carries no identity at all.
    const bool optedOut = jsonStr(b, e, "tracked", v, sizeof v) && v[0] == 'N';
    if (optedOut) {
        put(out.where, sizeof out.where, "opted out");
        out.answer = ANS_HIT;
        return ANS_HIT;
    }
    if (jsonStr(b, e, "aircraft_model", v, sizeof v)) put(out.what, sizeof out.what, v);
    char reg[24] = {0}, cn[8] = {0};
    jsonStr(b, e, "registration", reg, sizeof reg);
    jsonStr(b, e, "cn", cn, sizeof cn);
    if (reg[0] || cn[0]) snprintf(out.extra, sizeof out.extra, "%s%s%s", reg, (reg[0] && cn[0]) ? " " : "", cn);
    const bool any = out.what[0] || out.extra[0];
    out.answer = any ? ANS_HIT : ANS_MISS;
    return (Answer)out.answer;
}

// ---- the cache ---------------------------------------------------------------
namespace {
// EVERYTHING BELOW REQUIRES THE LOCK, and none of it takes the lock itself.
// That is the rule that keeps this deadlock-free, because the mutex is not
// recursive: a public function takes the lock once and calls only these.
//
// The state is genuinely shared. scan() runs on loop()'s task and step() runs
// on the worker task, both over the same queue, so an append racing a removal
// would walk off the end of it. The one thing that must NOT happen under the
// lock is the request itself: an eight-second HTTP timeout with loop() blocked
// behind it is eight seconds of lost frames, so step() copies the entry out,
// drops the lock, asks, and takes the lock again to record the answer.
Record* findLocked(Proto p, uint64_t id) {
    if (!s_cache) return nullptr;
    for (uint8_t i = 0; i < s_cn; i++)
        if (s_cache[i].proto == p && s_cache[i].key == id) return &s_cache[i];
    return nullptr;
}

// One write, whole, at the moment a response is parsed -- which is during a
// WiFi window, when the backlight is already down and nothing is being drawn.
// Never on the frame path.
void rememberLocked(const Record& r) {
    if (!s_cache) return;
    Record* slot = findLocked(r.proto, r.key);
    if (!slot) {
        if (s_cn < CACHE_MAX) slot = &s_cache[s_cn++];
        else {
            slot = &s_cache[0];
            for (uint8_t i = 1; i < CACHE_MAX; i++)
                if (s_cache[i].whenMs < slot->whenMs) slot = &s_cache[i];
        }
    }
    *slot = r;
    s_writes++;
}

void logLocked(uint8_t source, const char* what, int code, size_t bytes, uint32_t now) {
    LogRow& l = s_log[s_lhead];
    l.ms = now;
    l.source = source;
    l.code = (int16_t)(code > 32767 ? 32767 : code < -32768 ? -32768 : code);
    l.bytes = (uint16_t)(bytes > 65535 ? 65535 : bytes);
    put(l.what, sizeof l.what, what);
    s_lhead = (uint8_t)((s_lhead + 1) % LOG_MAX);
    if (s_ln < LOG_MAX) s_ln++;
}

void dropLocked(uint8_t i) {
    if (i >= s_qn) return;
    for (uint8_t j = i; j + 1 < s_qn; j++) s_q[j] = s_q[j + 1];
    s_qn--;
}

// The order: directly-heard first, then most packets, then most recently heard.
// Not newest-first -- spending a twelve-second rate-limit slot on a node heard
// once a minute ago is the worst available use of it.
bool better(const Entry& a, const Entry& b) {
    if (a.direct != b.direct) return a.direct;
    if (a.packets != b.packets) return a.packets > b.packets;
    return a.lastMs > b.lastMs;
}

int findEntryLocked(Proto p, uint64_t id) {
    for (uint8_t i = 0; i < s_qn; i++) if (s_q[i].proto == p && s_q[i].key == id) return i;
    return -1;
}
}  // namespace

bool cached(Proto p, uint64_t id, Record& out) {
    lock();
    const Record* r = findLocked(p, id);
    if (r) out = *r;
    unlock();
    return r != nullptr;
}

uint32_t cacheWrites() { return s_writes; }

void begin() {
#if SQUACH_LORA
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
#endif
    if (s_cache) return;
    const size_t need = sizeof(Record) * CACHE_MAX;
    const size_t bodyCap = 2048;        // a hit measured 934 B; 148 B for OGN
#if SQUACH_LORA
    // PSRAM: 8,448 bytes of cache (88 a record, 96 of them) and 2 kB of body
    // buffer, against 7.6 MB available and about 57 kB of CONTIGUOUS internal
    // RAM that this deliberately does not touch. What stays internal is the
    // queue and the log alone -- 16 x 40 plus 16 x 28, about 1.1 kB. A record
    // is written whole, so this never becomes the scattered-write pattern that
    // starves the panel's DMA.
    s_cache = (Record*)heap_caps_malloc(need, MALLOC_CAP_SPIRAM);
    s_body  = (char*)heap_caps_malloc(bodyCap, MALLOC_CAP_SPIRAM);
#else
    s_cache = (Record*)malloc(need);
    s_body  = (char*)malloc(bodyCap);
#endif
    if (s_cache) memset(s_cache, 0, need);
    if (s_body) { memset(s_body, 0, bodyCap); s_bodyCap = bodyCap; }
}

void setSources(bool master, bool ham, bool ogn) {
    // Called from loop() every pass, so it does nothing at all until something
    // actually changes: a mutex acquisition per frame to re-learn the same
    // three booleans is a frame's worth of nothing.
    if (master == s_master && ham == s_ham && ogn == s_ogn) return;
    s_master = master; s_ham = ham; s_ogn = ogn;
    if (armed()) return;
    // Switched off means the queue goes, not just that nothing new is added --
    // except an entry already out on the wire, whose reply has a pointer into
    // this array waiting for it.
    lock();
    uint8_t w = 0;
    for (uint8_t i = 0; i < s_qn; i++) if (s_q[i].busy) s_q[w++] = s_q[i];
    s_qn = w;
    unlock();
}
bool armed() { return s_master && (s_ham || s_ogn); }

namespace {
// Asked again at the moment of the request, not only when the row was queued:
// a switch turned off has to stop what is already in the queue, or "off" would
// mean "off from now on for rows nobody has seen yet".
bool allowed(uint8_t source) {
    if (!s_master) return false;
    return source == SRC_HAM ? s_ham : source == SRC_OGN ? s_ogn : false;
}
}  // namespace

void clearAll() {
    lock();
    s_qn = s_cn = s_ln = s_lhead = 0;
    s_sent = s_hit = s_miss = s_noAnswer = 0;
    s_writes = 0;
    s_srcNextMs[SRC_HAM] = s_srcNextMs[SRC_OGN] = 0;
    if (s_cache) memset(s_cache, 0, sizeof(Record) * CACHE_MAX);
    unlock();
}

uint8_t scan(const Nodes::Node* rows, uint8_t n, uint32_t now) {
    if (!armed() || !rows) return s_qn;
    lock();
    for (uint8_t i = 0; i < n; i++) {
        const Nodes::Node& nd = rows[i];
        Plan p;
        if (!plan(nd, s_ham, s_ogn, p)) continue;
        // Asked once, and the answer -- including a miss and including a
        // give-up -- stands for the life of the boot. A callsign's country does
        // not change, a registry row changes slowly, and a structurally valid
        // callsign absent from every database is absent permanently: three of
        // thirty tokens in a live check were exactly that.
        if (findLocked(nd.proto, nd.id)) continue;
        if (findEntryLocked(nd.proto, nd.id) >= 0) continue;
        Entry e;
        memset(&e, 0, sizeof e);
        e.key = nd.id; e.proto = nd.proto; e.source = p.source;
        e.nextMs = now;
        e.direct = nd.directPackets > 0;
        e.packets = nd.packets;
        e.lastMs = nd.lastMs;
        put(e.text, sizeof e.text, p.text);
        if (s_qn < QUEUE_MAX) { s_q[s_qn++] = e; continue; }
        // Full: the new row takes the worst entry's place only if it beats it,
        // and never an entry that is out on the wire right now.
        int worst = -1;
        for (uint8_t j = 0; j < QUEUE_MAX; j++) {
            if (s_q[j].busy) continue;
            if (worst < 0 || better(s_q[worst], s_q[j])) worst = j;
        }
        if (worst >= 0 && better(e, s_q[worst])) s_q[worst] = e;
    }
    const uint8_t depth = s_qn;
    unlock();
    return depth;
}

bool step(uint32_t now) {
    if (!armed() || !s_fetch || !s_cache || !s_body) return false;
    if (s_netUp && !s_netUp()) return false;

    // Under the lock: the best entry that is due, whose source is not inside
    // its spacing and is still switched on. Copied out, because the queue may
    // be rearranged by a scan while the request is in flight.
    lock();
    int best = -1;
    for (uint8_t i = 0; i < s_qn; i++) {
        const Entry& e = s_q[i];
        if (e.busy) continue;
        if (!allowed(e.source)) continue;
        if ((int32_t)(now - e.nextMs) < 0) continue;
        if ((int32_t)(now - s_srcNextMs[e.source]) < 0) continue;
        if (best < 0 || better(e, s_q[best])) best = i;
    }
    if (best < 0) { unlock(); return false; }
    s_q[best].busy = true;
    const Entry e = s_q[best];
    // The source's next slot is claimed before the request goes out, not after
    // it comes back: two workers must not both find the source free.
    s_srcNextMs[e.source] = now + SOURCES[e.source].spacingMs;
    unlock();

    size_t len = 0;
    const int code = s_fetch(e.source, e.text, s_body, s_bodyCap, len);

    lock();
    // Logged before the result is interpreted, so the log is a record of what
    // left the device rather than of what the device made of the reply. It goes
    // in even if the entry has since been dropped: the request happened.
    logLocked(e.source, e.text, code, len, now);
    s_sent++;
    const int ix = findEntryLocked(e.proto, e.key);
    if (ix < 0) { unlock(); return true; }       // dropped mid-flight; the log still has it
    s_q[ix].busy = false;

    // 429 stalls the SOURCE, not just this entry, and does not consume a try:
    // being told to slow down is not a failed lookup. Measured on OGN -- one
    // request served, the next two refused immediately, still refused four
    // seconds later.
    if (code == 429) {
        s_q[ix].nextMs = now + SOURCES[e.source].spacingMs;
        unlock();
        return true;
    }

    Record r;
    memset(&r, 0, sizeof r);
    r.key = e.key; r.proto = e.proto; r.source = e.source; r.whenMs = now;
    const Answer a = e.source == SRC_HAM ? parseHamrig(s_body, len, code, r)
                                         : parseOgn(s_body, len, code, r);
    if (a == ANS_HIT || a == ANS_MISS) {
        if (a == ANS_HIT) s_hit++; else s_miss++;
        rememberLocked(r);
        dropLocked((uint8_t)ix);
        unlock();
        return true;
    }
    // A failure to ask: a timeout, a DNS miss, no route, a 5xx. Four goes,
    // backing off 60, 120 and 240 seconds, and then the row says "no answer"
    // rather than "not listed", because those are different facts.
    s_q[ix].tries++;
    if (s_q[ix].tries >= 4) {
        r.answer = ANS_NOANSWER;
        s_noAnswer++;
        rememberLocked(r);
        dropLocked((uint8_t)ix);
        unlock();
        return true;
    }
    uint32_t back = 60000u << (s_q[ix].tries - 1);
    if (back > 900000u) back = 900000u;
    s_q[ix].nextMs = now + back;
    unlock();
    return true;
}

void progress(Progress& out, uint32_t now) {
    memset(&out, 0, sizeof out);
    lock();
    out.queued = s_qn;
    out.sent = s_sent;
    out.hit = s_hit; out.miss = s_miss; out.noAnswer = s_noAnswer;
    for (uint8_t s = 0; s < SRC_COUNT; s++)
        if (s_srcNextMs[s] && (int32_t)(now - s_srcNextMs[s]) < 0) out.stalled++;
    unlock();
}

uint8_t logCount() { return s_ln; }
bool logAt(uint8_t i, LogRow& out) {
    lock();
    const bool ok = i < s_ln;
    if (ok) out = s_log[(uint8_t)((s_lhead + LOG_MAX - 1 - i) % LOG_MAX)];
    unlock();
    return ok;
}

void setTransport(Fetch f, NetUp up) { s_fetch = f; s_netUp = up; }

// ---- the transport, and the only part a desktop cannot check -----------------
#if SQUACH_LORA

namespace {
bool wifiUp() { return WiFi.status() == WL_CONNECTED; }

int httpGet(uint8_t source, const char* text, char* body, size_t cap, size_t& len) {
    len = 0;
    if (source >= SRC_COUNT) return -1;
    char path[96];
    // The format string is a literal in SOURCES above with exactly one %s in
    // it, and `text` came out of plan() bounded to fifteen characters -- so this
    // is a table lookup wearing a format string's clothes, not user input
    // reaching a formatter.
    snprintf(path, sizeof path, SOURCES[source].path, text);
    char url[160];
    // Plain HTTP, for the reason src/ota_wifi.cpp:192 gives for the firmware
    // image itself: this framework's mbedTLS allocates its two 16,717-byte
    // record buffers from MALLOC_CAP_INTERNAL, measured in the shipped
    // library, so PSRAM cannot serve them (see docs/LORA.md). Both hosts
    // answer over plain HTTP, verified, and neither reply is a secret -- a
    // registry row about a station that just broadcast its own identity.
    snprintf(url, sizeof url, "http://%s%s", SOURCES[source].host, path);
    WiFiClient client;
    HTTPClient http;
    if (!http.begin(client, url)) return -1;
    http.setTimeout(8000);
    http.setConnectTimeout(5000);
    // Identified, because the OGN operator's own convention is that a client
    // says who it is so it can be throttled per application rather than
    // blocked outright.
    http.setUserAgent("SquachWatch/lora-enrich");
    const int code = http.GET();
    if (code == 200) {
        WiFiClient* s = http.getStreamPtr();
        const int total = http.getSize();
        uint32_t t0 = millis();
        while (len < cap - 1 && (total < 0 || (int)len < total) && millis() - t0 < 8000) {
            const int avail = s->available();
            if (avail > 0) {
                const int r = s->read((uint8_t*)body + len, (size_t)avail < cap - 1 - len ? (size_t)avail : cap - 1 - len);
                if (r > 0) { len += (size_t)r; t0 = millis(); }
            } else if (!http.connected()) {
                break;
            } else {
                delay(5);
            }
        }
        body[len] = '\0';
    }
    http.end();
    return code;
}

TaskHandle_t s_task = nullptr;

void worker(void*) {
    // Spends what is due and leaves. Not a long-lived task: it exists only
    // while WiFi is up for another reason, and the moment that stops being
    // true it has no business running.
    while (armed() && wifiUp() && s_qn) {
        if (!step(millis())) vTaskDelay(pdMS_TO_TICKS(250));
        else vTaskDelay(pdMS_TO_TICKS(20));
    }
    s_task = nullptr;
    vTaskDelete(nullptr);
}
}  // namespace

bool running() { return s_task != nullptr; }

void tick(uint32_t) {
    if (!s_fetch) setTransport(httpGet, wifiUp);
    if (s_task || !armed() || !s_qn || !wifiUp()) return;
    // 4 kB measured against ota_wifi.cpp's own plain-HTTP task, which uses
    // 3.2 kB of its 8 kB for the same HTTPClient shape plus an update's
    // bookkeeping. The body buffer is in PSRAM, not on this stack.
    if (xTaskCreatePinnedToCore(worker, "loraenrich", 5120, nullptr, 1, &s_task, 1) != pdPASS)
        s_task = nullptr;
}

#else
bool running() { return false; }
void tick(uint32_t) {}
#endif

}
}
