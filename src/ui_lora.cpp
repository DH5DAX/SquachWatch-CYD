// SquachWatch-CYD — the LORA screen. See include/ui_lora.h.
#include "ui_lora.h"
#include "ui_scroll.h"
#include "theme.h"
#include "lora_sniffer.h"
#include "lora_profiles.h"
#include "lora_classify.h"
#include "lora_nodes.h"
#include "lora_ident.h"
#include "lora_enrich.h"
#include "settings.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>

namespace {

LoraView s_view = LoraView::LIST;
int      s_scroll[5] = { 0, 0, 0, 0, 0 };   // one per LoraView, PACKET's unused
uint16_t s_open = 0;          // PACKET: which frame, newest first
uint32_t s_openTotal = 0;     // ...at which packetTotal(), so it tracks as new ones arrive

const int BODY_TOP = 16;

uint16_t protoColor(Lora::Proto p) {
    switch (p) {
        case Lora::Proto::MESHTASTIC: return Theme::GREEN;
        case Lora::Proto::MESHCORE:   return Theme::CYAN;
        case Lora::Proto::LORAWAN:    return Theme::AMBER;
        case Lora::Proto::APRS:       return Theme::VAPOR_PINK;
        case Lora::Proto::MESHCOM:    return Theme::VAPOR_PURPLE;
        case Lora::Proto::FANET:      return Theme::WHITE;
        case Lora::Proto::RETICULUM:  return Theme::PURPLE;
        default:                      return Theme::RED;
    }
}

void ageText(uint32_t now, uint32_t ms, char* out, size_t cap) {
    const uint32_t s = (now - ms) / 1000;
    if (s < 60) snprintf(out, cap, "%lus", (unsigned long)s);
    else if (s < 3600) snprintf(out, cap, "%lum", (unsigned long)(s / 60));
    else snprintf(out, cap, "%luh", (unsigned long)(s / 3600));
}

// Three buttons where every other screen has its bar: BACK and the two
// views that are not this one (PACKET has < and > instead).
struct Bar { int y, h, x[3], w; };
Bar bar(int screenW, int screenH) {
    Theme::ButtonBarGeom g = Theme::computeButtonBar(screenW, screenH);
    Bar b;
    b.y = g.y; b.h = g.h;
    const int margin = 8, gap = 8;
    b.w = (screenW - 2 * margin - 2 * gap) / 3;
    b.x[0] = margin; b.x[1] = margin + b.w + gap; b.x[2] = margin + 2 * (b.w + gap);
    return b;
}

// Four destinations and two buttons, so the pair names the NEIGHBOURS in the
// cycle LIST -> NODES -> STATS -> CHANS -> LIST rather than "the other two":
// every view is then one or two taps from every other, and a button always
// says where it lands. PACKET keeps its own < and >.
const LoraView CYCLE[4] = { LoraView::LIST, LoraView::NODES, LoraView::STATS, LoraView::CHANS };

int cycleSlot(LoraView v) {
    for (int i = 0; i < 4; i++) if (CYCLE[i] == v) return i;
    return 0;
}
LoraView cycleStep(LoraView v, int delta) { return CYCLE[(cycleSlot(v) + 4 + delta) % 4]; }

const char* viewLabel(LoraView v) {
    switch (v) {
        case LoraView::NODES: return "[ NODES ]";
        case LoraView::STATS: return "[ STATS ]";
        case LoraView::CHANS: return "[ CHANS ]";
        default:              return "[ LIST ]";
    }
}

void barLabels(const char* l[3]) {
    l[0] = "[ BACK ]";
    if (s_view == LoraView::PACKET) { l[1] = "[ < ]"; l[2] = "[ > ]"; return; }
    l[1] = viewLabel(cycleStep(s_view, -1));
    l[2] = viewLabel(cycleStep(s_view, +1));
}

int rowH(TFT_eSPI& t) { t.setTextSize(1); return t.fontHeight() + 6; }

void drawBar(TFT_eSPI& t, int w, int h) {
    const Bar b = bar(w, h);
    const char* l[3]; barLabels(l);
    for (int i = 0; i < 3; i++) Theme::drawButton(t, b.x[i], b.y, b.w, b.h, l[i], false);
}

// One line under the title: the mode, where the radio is, and the numbers
// that say whether it is hearing anything.
int drawStatus(TFT_eSPI& t, int w, int y) {
    t.setTextSize(1);
    t.setTextWrap(false);
    char line[96];
    if (!Lora::present()) {
        snprintf(line, sizeof line, "NO MODULE: set K1 to the wireless position and restart");
        t.setTextColor(Theme::AMBER, Theme::BG);
    } else {
        const Lora::Profile& p = Lora::profile(Lora::currentProfile());
        char mhz[12]; Lora::formatMHz(p.freqHz, mhz, sizeof mhz);
        const Lora::Stats& s = Lora::stats();
        snprintf(line, sizeof line, "%s %s %s SF%u  %lu pkts  noise %d", Lora::modeName(Lora::mode()), p.name, mhz,
                 (unsigned)p.sf, (unsigned long)Lora::packetTotal(), (int)s.noiseDbm);
        t.setTextColor(Theme::CYAN, Theme::BG);
    }
    t.fillRect(0, y, w, t.fontHeight() + 2, Theme::BG);
    t.setCursor(4, y + 1);
    t.print(line);
    return y + t.fontHeight() + 3;
}

void drawList(TFT_eSPI& t, uint32_t now, int w, int top, int bottom) {
    const int rh = rowH(t);
    const uint16_t count = Lora::packetCount();
    uiClampScroll(s_scroll[0], count, bottom - top, rh);
    if (!count) {
        t.setTextColor(Theme::CYAN, Theme::BG);
        t.setCursor(8, top + 20);
        t.print(Lora::present() ? "nothing heard yet" : "");
        return;
    }
    int y = top;
    Lora::Packet pk;
    for (int idx = s_scroll[0]; idx < count && y + rh <= bottom; idx++, y += rh) {
        if (!Lora::packetAt((uint16_t)idx, pk)) break;
        Theme::drawListRowPanel(t, w, y, rh);
        const uint16_t col = (pk.flags & Lora::PK_CRC_ERR) ? Theme::RED : protoColor(pk.proto);
        t.fillRect(4, y + 3, 4, rh - 6, col);
        char age[8]; ageText(now, pk.ms, age, sizeof age);
        char line[128], sum[96];
        Lora::summary(pk, sum, sizeof sum);
        snprintf(line, sizeof line, "%-4s %4d %4s %s", Lora::protoShort(pk.proto), (int)pk.rssi, age,
                 (pk.flags & Lora::PK_CRC_ERR) ? "CRC ERR" : sum);
        t.setTextColor(col, Theme::BG);
        t.setCursor(12, y + 3);
        t.print(line);
    }
    Theme::drawScrollbar(t, w - 6, top, bottom - top, count, (bottom - top) / rh, s_scroll[0]);
}

void drawPacket(TFT_eSPI& t, uint32_t now, int w, int top, int bottom) {
    // The frame keeps its place as newer ones land above it.
    const uint32_t total = Lora::packetTotal();
    uint32_t idx = s_open + (total - s_openTotal);
    if (idx >= Lora::packetCount()) idx = Lora::packetCount() ? Lora::packetCount() - 1 : 0;
    Lora::Packet pk;
    if (!Lora::packetAt((uint16_t)idx, pk)) {
        t.setTextColor(Theme::CYAN, Theme::BG); t.setCursor(8, top + 20); t.print("no frame");
        return;
    }
    t.setTextSize(1);
    const int lh = t.fontHeight() + 2;
    int y = top + 2;
    char mhz[12], bw[8], age[8], line[100];
    Lora::formatMHz(pk.freqHz, mhz, sizeof mhz); Lora::formatBw(pk.bwKhz10, bw, sizeof bw); ageText(now, pk.ms, age, sizeof age);
    t.setTextColor(protoColor(pk.proto), Theme::BG);
    snprintf(line, sizeof line, "%s  #%lu  %s ago", Lora::protoName(pk.proto), (unsigned long)(total - idx), age);
    t.setCursor(4, y); t.print(line); y += lh;
    t.setTextColor(Theme::WHITE, Theme::BG);
    snprintf(line, sizeof line, "%s MHz SF%u %s sync %02x%s%s  %s", mhz, (unsigned)pk.sf, bw, (unsigned)pk.sync,
             (pk.pflags & Lora::PF_INVERT) ? " iq-inv" : "", (pk.flags & Lora::PK_IMPLICIT) ? " implicit" : "",
             pk.profile < Lora::profileCount() ? Lora::profile(pk.profile).name : "custom");
    t.setCursor(4, y); t.print(line); y += lh;
    snprintf(line, sizeof line, "%d dBm  snr %d.%02d  ferr %+ld Hz  %u B  cr4/%u  %lu ms  %s", (int)pk.rssi,
             (int)(pk.snr4 / 4), (int)abs(pk.snr4 % 4) * 25, (long)pk.ferrHz, (unsigned)pk.len, (unsigned)pk.cr,
             (unsigned long)(pk.toaUs / 1000),
             (pk.flags & Lora::PK_CRC_ERR) ? "CRC ERR" : (pk.flags & Lora::PK_CRC_OK) ? "crc ok" : "no crc");
    t.setCursor(4, y); t.print(line); y += lh + 2;
    // The decoded line, wrapped by hand: it is one string and the panel is
    // narrow, and Theme::wrapText's 48-column lines are for bubbles.
    char sum[160];
    Lora::summary(pk, sum, sizeof sum);
    const int cols = (w - 8) / 6;
    t.setTextColor(protoColor(pk.proto), Theme::BG);
    for (const char* p = sum; *p && y + lh <= bottom; ) {
        int n = (int)strlen(p); if (n > cols) n = cols;
        if (n == cols) { int k = n; while (k > cols / 2 && p[k] != ' ') k--; if (k > cols / 2) n = k; }
        char seg[100]; memcpy(seg, p, (size_t)n); seg[n] = '\0';
        t.setCursor(4, y); t.print(seg); y += lh;
        p += n; while (*p == ' ') p++;
    }
    y += 2;
    // The bytes, sixteen a row, as far as the screen goes.
    t.setTextColor(Theme::CYAN, Theme::BG);
    const int per = cols >= 56 ? 16 : 8;
    for (uint8_t i = 0; i < pk.len && y + lh <= bottom; i = (uint8_t)(i + per)) {
        size_t o = (size_t)snprintf(line, sizeof line, "%02x: ", (unsigned)i);
        for (uint8_t j = i; j < pk.len && j < (uint8_t)(i + per) && o + 3 < sizeof line; j++)
            o += (size_t)snprintf(line + o, sizeof line - o, "%02x ", pk.data[j]);
        t.setCursor(4, y); t.print(line); y += lh;
    }
}

void drawNodes(TFT_eSPI& t, uint32_t now, int w, int top, int bottom) {
    const int rh = rowH(t);
    uint8_t idx[96];
    const uint8_t count = Lora::nodeOrder(idx, sizeof idx);
    uiClampScroll(s_scroll[2], count, bottom - top, rh);
    if (!count) {
        t.setTextColor(Theme::CYAN, Theme::BG); t.setCursor(8, top + 20); t.print("no transmitters named yet");
        return;
    }
    int y = top;
    Lora::Nodes::Node n;
    for (int i = s_scroll[2]; i < count && y + rh <= bottom; i++, y += rh) {
        if (!Lora::nodeAt(idx[i], n)) break;
        Theme::drawListRowPanel(t, w, y, rh);
        const uint16_t col = protoColor(n.proto);
        t.fillRect(4, y + 3, 4, rh - 6, col);
        char age[8], flags[48], line[128];
        ageText(now, n.lastMs, age, sizeof age);
        Lora::Nodes::flagsText(n, flags, sizeof flags);
        // The signal column is a link measurement only for a frame this node
        // transmitted itself. A row heard only through repeaters gets the
        // relay's figure with a v in front of it: a bare "-73" in this column
        // reads as a distance from here, and for a node eleven hops out it is
        // the distance to the last repeater instead.
        // The grid square takes the place of the bare "@" this column used to
        // hold: six characters where two were, displacing the tail of the
        // flags text, which is the only thing on the line with slack in it.
        // Worth the trade -- "@" said a position had been decoded, a locator
        // says where, in the unit the operators of these networks speak, and it
        // is the node's OWN advertised position and never a licensee's.
        char where[8] = "";
        if (n.hasPos) Lora::Ident::grid(n.latE7, n.lonE7, where, sizeof where);
        // One character for an online answer, beside the one the flags use: a
        // dot when a lookup answered, a dash when it was asked and there was
        // nothing to find or nobody to answer. Blank means nothing was asked,
        // which is what every row says until a switch is turned on.
        Lora::Enrich::Record er;
        const bool asked = Lora::Enrich::cached(n.proto, n.id, er);
        const char* look = !asked ? "" : er.answer == Lora::Enrich::ANS_HIT ? "." : "-";
        char sig[10] = "--";
        if (n.directPackets)   snprintf(sig, sizeof sig, "%d", (int)n.rssi);
        else if (n.viaPackets) snprintf(sig, sizeof sig, "v%d", (int)n.viaRssi);
        // The hour bucket tumbles, so a single 0.7 s frame 1.5 s into a fresh
        // one is 46.6 % of it. Under five minutes there is no figure to show --
        // which is the same floor the NF_DUTY flag has always had under it.
        char dut[10] = "--";
        if (Lora::Nodes::dutyKnown(n, now)) {
            const uint16_t duty = Lora::Nodes::dutyPermille(n, now);
            snprintf(dut, sizeof dut, "%u.%u%%", duty / 10, duty % 10);
        }
        snprintf(line, sizeof line, "%-4s %-10.10s %-14.14s %5s %3u %4s %-5s %-6s %s%s%s", Lora::protoShort(n.proto), n.tag,
                 n.name[0] ? n.name : Lora::Nodes::roleText(n), sig, (unsigned)n.packets, age,
                 dut, where, look, flags[0] ? "!" : "", flags);
        t.setTextColor(n.flags ? Theme::AMBER : col, Theme::BG);
        t.setCursor(12, y + 3);
        t.print(line);
    }
    Theme::drawScrollbar(t, w - 6, top, bottom - top, count, (bottom - top) / rh, s_scroll[2]);
}

// The keys the decoders hold: which are known, which have been heard, and
// which are switched off. A tap mutes one, and that is deliberately all a
// finger can do -- see the header.
void drawChans(TFT_eSPI& t, uint32_t now, int w, int top, int bottom) {
    t.setTextSize(1);
    const int rh = rowH(t);
    const int lh = t.fontHeight() + 2;
    uint8_t mcU = 0, mcM = 0, mtU = 0, mtM = 0;
    Lora::channelCapacity(mcU, mcM, mtU, mtM);
    char head[80];
    // The presets are counted rather than listed: fourteen radio profiles
    // times two key modes is twenty-eight rows of decoder capability, not
    // twenty-eight channels anybody chose. One appears in the list as soon as
    // it opens a frame.
    snprintf(head, sizeof head, "keys: %u/%u MC, %u/%u MT, %u presets quiet",
             (unsigned)mcU, (unsigned)mcM, (unsigned)mtU, (unsigned)mtM, (unsigned)Lora::channelsQuietBuiltIn());
    t.setTextColor(Theme::CYAN, Theme::BG);
    t.setCursor(6, top);
    t.print(head);

    const int listTop = top + lh + 2;
    const int listBottom = bottom - lh - 2;
    const uint8_t count = Lora::channelRowCount();
    uiClampScroll(s_scroll[(int)LoraView::CHANS], count, listBottom - listTop, rh);
    int y = listTop;
    Lora::ChannelRow r;
    for (int i = s_scroll[(int)LoraView::CHANS]; i < (int)count && y + rh <= listBottom; i++, y += rh) {
        if (!Lora::channelRow((uint8_t)i, r)) break;
        Theme::drawListRowPanel(t, w, y, rh);
        // Dim for muted, the network's colour once it has opened something,
        // white for a key that has never been used: "known but never heard" is
        // exactly the state a sysop is looking for on this screen.
        const uint16_t col = !r.enabled ? Theme::VAPOR_PURPLE : r.frames ? protoColor(r.proto) : Theme::WHITE;
        t.fillRect(4, y + 3, 4, rh - 6, col);
        char age[8] = "-";
        if (r.lastMs) ageText(now, r.lastMs, age, sizeof age);
        char line[128];
        snprintf(line, sizeof line, "%-4s %-16.16s %02x %-8s %5lu %4s %s", Lora::protoShort(r.proto), r.name,
                 (unsigned)r.hash, r.builtIn ? "built in" : r.derived ? "tag" : r.keyBits ? "key" : "plain",
                 (unsigned long)r.frames, age, r.enabled ? "" : "MUTED");
        t.setTextColor(col, Theme::BG);
        t.setCursor(12, y + 3);
        t.print(line);
    }
    Theme::drawScrollbar(t, w - 6, listTop, listBottom - listTop, count, (listBottom - listTop) / rh,
                         s_scroll[(int)LoraView::CHANS]);
    t.setTextColor(Theme::AMBER, Theme::BG);
    t.setCursor(6, bottom - lh);
    // With nothing of one's own in the list, the useful sentence is the one
    // that fills it; after that, the one that says what a finger can do.
    t.print(mcU + mtU ? "tap to mute; the console adds them: LORA CHAN"
                      : "no keys of your own: LORA CHAN GROUP NRW adds eleven");
}

int statLine(TFT_eSPI& t, int y, uint16_t col, const char* label, const char* text) {
    t.setTextColor(col, Theme::BG);
    t.setCursor(6, y);
    t.print(label);
    t.setTextColor(Theme::WHITE, Theme::BG);
    t.setCursor(6 + t.textWidth(label) + 6, y);
    t.print(text);
    return y + t.fontHeight() + 2;
}

// The spectrum from a SWEEP, when there has been one: the hold in the dim
// colour, the live reading bright, 863 to 870 MHz across the width.
int drawSpectrum(TFT_eSPI& t, int w, int y, int bottom) {
    uint8_t live[Lora::SPECTRUM_BINS], hold[Lora::SPECTRUM_BINS];
    const uint8_t n = Lora::spectrum(live, hold, Lora::SPECTRUM_BINS);
    if (!n || !Lora::spectrumSweeps()) return y;
    const int gh = bottom - y - 12;
    if (gh < 30) return y;
    const int x0 = 24, gw = w - x0 - 6;
    // The floor of the scale is -140 dBm (value 10), the top -60 (value 90).
    auto bar = [&](uint8_t v) { int h = ((int)v - 10) * gh / 80; return h < 0 ? 0 : h > gh ? gh : h; };
    t.drawRect(x0 - 1, y, gw + 2, gh + 2, Theme::PURPLE);
    for (uint8_t i = 0; i < n; i++) {
        const int x = x0 + (int)((long)i * gw / n);
        const int bw = (int)((long)(i + 1) * gw / n) - (int)((long)i * gw / n);
        const int hh = bar(hold[i]), lh = bar(live[i]);
        if (hh) t.fillRect(x, y + 1 + gh - hh, bw > 1 ? bw - 1 : 1, hh, Theme::VAPOR_PURPLE);
        if (lh) t.fillRect(x, y + 1 + gh - lh, bw > 1 ? bw - 1 : 1, lh, Theme::CYAN);
    }
    t.setTextSize(1);
    t.setTextColor(Theme::CYAN, Theme::BG);
    t.setCursor(0, y); t.print("-60");
    t.setCursor(0, y + gh - t.fontHeight()); t.print("-140");
    const int ly = y + gh + 3;
    for (int mhz = 863; mhz <= 870; mhz++) {
        const int x = x0 + (mhz - 863) * gw / 7;
        t.drawFastVLine(x, y + gh - 3, 3, Theme::WHITE);
        if (mhz % 2 == 1 || mhz == 870) {
            char l[8]; snprintf(l, sizeof l, "%d", mhz);
            t.setCursor(x - (mhz == 870 ? t.textWidth(l) : t.textWidth(l) / 2), ly); t.print(l);
        }
    }
    return bottom;
}

void drawStats(TFT_eSPI& t, uint32_t now, int w, int top, int bottom) {
    (void)now;
    t.setTextSize(1);
    const Lora::Stats& s = Lora::stats();
    char b[96];
    int y = top + 2;
    if (!Lora::present()) {
        char st[96]; Lora::statusLine(st, sizeof st);
        y = statLine(t, y, Theme::AMBER, "SLOT:", st);
        return;
    }
    Lora::statusLine(b, sizeof b);
    y = statLine(t, y, Theme::CYAN, "RADIO:", b);
    snprintf(b, sizeof b, "%lu frames, %lu crc err, %lu hdr err, %lu stray preambles",
             (unsigned long)s.packets, (unsigned long)s.crcErrors, (unsigned long)s.headerErrors, (unsigned long)s.preambles);
    y = statLine(t, y, Theme::CYAN, "HEARD:", b);
    snprintf(b, sizeof b, "%lu rounds, %lu hits", (unsigned long)s.cadRounds, (unsigned long)s.cadHits);
    y = statLine(t, y, Theme::CYAN, "CAD:", b);
    const uint32_t up = millis() / 1000;
    snprintf(b, sizeof b, "%lu.%lu s of time on air heard; noise %d dBm",
             (unsigned long)(s.airtimeMs / 1000), (unsigned long)((s.airtimeMs / 100) % 10), (int)s.noiseDbm);
    y = statLine(t, y, Theme::CYAN, "AIR:", b);
    // Airtime over the time the receiver was actually on a profile -- which is
    // the occupancy of that air while we were listening to it. Dividing by
    // uptime instead, as this line used to, divides by the wall clock the
    // survey spent on the other thirty-odd profiles and answers nothing.
    if (s.listenMs) {
        const uint32_t pct100 = (uint32_t)(((uint64_t)s.airtimeMs * 10000ull) / s.listenMs);
        snprintf(b, sizeof b, "%lu s receiving, %lu%% of uptime; %lu.%02lu%% of it was frames",
                 (unsigned long)(s.listenMs / 1000), (unsigned long)(up ? s.listenMs / 10 / up : 0),
                 (unsigned long)(pct100 / 100), (unsigned long)(pct100 % 100));
    } else {
        snprintf(b, sizeof b, "the receiver has not been on yet");
    }
    y = statLine(t, y, Theme::CYAN, "LISTEN:", b);
    size_t o = 0;
    for (int p = 1; p < (int)Lora::Proto::COUNT && o + 12 < sizeof b; p++)
        if (s.byProto[p]) o += (size_t)snprintf(b + o, sizeof b - o, "%s%s %u", o ? "  " : "", Lora::protoShort((Lora::Proto)p), (unsigned)s.byProto[p]);
    if (s.byProto[0]) snprintf(b + o, sizeof b - o, "%s?? %u", o ? "  " : "", (unsigned)s.byProto[0]);
    y = statLine(t, y, Theme::CYAN, "BY NET:", o || s.byProto[0] ? b : "-");
    // What has left this board, and what may. Amber when anything may leave,
    // cyan when nothing can: the unusual state is the one that gets the colour,
    // and here the unusual state is being allowed to talk about other people.
    // The full log -- every identifier, every host, every status -- is LORA
    // LOOKUPS on the console; there is no room for sixteen rows here and no
    // screen to put them on yet.
    {
        Lora::Enrich::Progress pr;
        Lora::Enrich::progress(pr, now);
        if (!Settings::loraLookups()) {
            snprintf(b, sizeof b, "off: nothing about a node leaves this board");
        } else {
            snprintf(b, sizeof b, "%s%s%s: %u sent, %u known, %u not listed, %u no answer%s",
                     Settings::loraLookupCall() ? "hamrig.com" : "",
                     (Settings::loraLookupCall() && Settings::loraLookupOgn()) ? " + " : "",
                     Settings::loraLookupOgn() ? "glidernet.org" : "",
                     (unsigned)pr.sent, (unsigned)pr.hit, (unsigned)pr.miss, (unsigned)pr.noAnswer,
                     pr.queued ? ", more waiting for WiFi" : "");
            if (!Settings::loraLookupCall() && !Settings::loraLookupOgn())
                snprintf(b, sizeof b, "on, but every source is off: nothing leaves");
        }
        y = statLine(t, y, Settings::loraLookups() ? Theme::AMBER : Theme::CYAN, "LOOKUP:", b);
    }
    y += 4;
    if (Lora::mode() == Lora::Mode::SWEEP || Lora::spectrumSweeps()) {
        snprintf(b, sizeof b, "%lu passes over 863-870 MHz%s", (unsigned long)Lora::spectrumSweeps(),
                 Lora::mode() == Lora::Mode::SWEEP ? "; tap the status line to stop" : "");
        y = statLine(t, y, Theme::CYAN, "SWEEP:", b);
        drawSpectrum(t, w, y + 2, bottom);
        return;
    }
    // The busiest profiles as bars.
    uint8_t top8[8]; uint8_t n8 = 0;
    for (uint8_t i = 0; i < Lora::profileCount() && i < 64; i++) {
        if (!s.byProfile[i]) continue;
        uint8_t k = n8 < 8 ? n8++ : 7;
        while (k > 0 && s.byProfile[top8[k - 1]] < s.byProfile[i]) { top8[k] = top8[k - 1]; k--; }
        if (k < 8) top8[k] = i;
    }
    uint16_t most = n8 ? s.byProfile[top8[0]] : 1;
    const int lh = t.fontHeight() + 2;
    for (uint8_t i = 0; i < n8 && y + lh <= bottom; i++) {
        const Lora::Profile& p = Lora::profile(top8[i]);
        snprintf(b, sizeof b, "%-14s %4u", p.name, (unsigned)s.byProfile[top8[i]]);
        t.setTextColor(Theme::WHITE, Theme::BG); t.setCursor(6, y); t.print(b);
        const int bx = 6 + 20 * 6, bw = w - bx - 12;
        t.fillRect(bx, y + 1, (int)((long)bw * s.byProfile[top8[i]] / most), t.fontHeight() - 2, Theme::CYAN);
        y += lh;
    }
}

}

void uiLoraInit(TFT_eSPI& t, LoraView v) {
    t.fillRect(0, 0, t.width(), t.height(), Theme::BG);
    s_view = v == LoraView::PACKET ? LoraView::LIST : v;   // PACKET needs a frame chosen first
}

LoraView uiLoraView() { return s_view; }

void uiLoraScroll(int delta) {
    int& s = s_scroll[(int)s_view];
    s += delta;
    if (s < 0) s = 0;
}

void uiLoraTick(TFT_eSPI& t, uint32_t now, const DetectionEngine& eng, bool advance) {
    (void)eng; (void)advance;
    const int w = t.width(), h = t.height();
    const Bar b = bar(w, h);
    const int bottom = b.y - 4;
    t.fillRect(0, BODY_TOP, w, bottom - BODY_TOP, Theme::BG);
    char title[32];
    switch (s_view) {
        case LoraView::LIST:   snprintf(title, sizeof title, ">> LORA  (%u) <<", (unsigned)Lora::packetCount()); break;
        case LoraView::PACKET: snprintf(title, sizeof title, ">> LORA FRAME <<"); break;
        case LoraView::NODES:  snprintf(title, sizeof title, ">> LORA NODES  (%u) <<", (unsigned)Lora::nodeCount()); break;
        case LoraView::CHANS:  snprintf(title, sizeof title, ">> LORA CHANNELS  (%u) <<", (unsigned)Lora::channelRowCount()); break;
        // STATS was titled "LORA CHANNEL" -- the radio channel. With a view
        // about crypto channels next to it that title was a trap, and the
        // button has always said STATS.
        default:               snprintf(title, sizeof title, ">> LORA STATS <<"); break;
    }
    Theme::drawTitleBar(t, title);
    int top = drawStatus(t, w, BODY_TOP + 1);
    switch (s_view) {
        case LoraView::LIST:   drawList(t, now, w, top, bottom); break;
        case LoraView::PACKET: drawPacket(t, now, w, top, bottom); break;
        case LoraView::NODES:  drawNodes(t, now, w, top, bottom); break;
        case LoraView::CHANS:  drawChans(t, now, w, top, bottom); break;
        default:               drawStats(t, now, w, top, bottom); break;
    }
    drawBar(t, w, h);
    Theme::drawToast(t, now);
}

LoraTap uiLoraTap(TFT_eSPI& t, int x, int y, int screenW, int screenH) {
    const Bar b = bar(screenW, screenH);
    if (y >= b.y && y <= b.y + b.h) {
        int which = -1;
        for (int i = 0; i < 3; i++) if (x >= b.x[i] && x <= b.x[i] + b.w) which = i;
        if (which < 0) return LoraTap::NONE;
        if (which == 0) {
            if (s_view == LoraView::PACKET) { s_view = LoraView::LIST; return LoraTap::HANDLED; }
            return LoraTap::BACK;
        }
        switch (s_view) {
            case LoraView::PACKET: {
                // < is towards the newer frame, > the older, in the list's order.
                const uint32_t total = Lora::packetTotal();
                uint32_t idx = s_open + (total - s_openTotal);
                if (which == 1 && idx > 0) idx--;
                if (which == 2 && idx + 1 < Lora::packetCount()) idx++;
                s_open = (uint16_t)idx; s_openTotal = total;
                break;
            }
            default: s_view = cycleStep(s_view, which == 1 ? -1 : +1); break;
        }
        return LoraTap::HANDLED;
    }
    // The status line cycles the mode: OFF, FOCUS, SURVEY, SWEEP. The
    // first three are the setting; SWEEP is for now and is not kept.
    t.setTextSize(1);
    if (Lora::present() && y >= BODY_TOP && y < BODY_TOP + 1 + t.fontHeight() + 3) {
        const uint8_t next = (uint8_t)(((uint8_t)Lora::mode() + 1) % (uint8_t)Lora::Mode::COUNT);
        Lora::setMode((Lora::Mode)next);
        if (next < 3) { while (Settings::loraMode() != next) Settings::cycleLoraMode(); }
        return LoraTap::HANDLED;
    }
    if (s_view == LoraView::CHANS && y >= BODY_TOP) {
        const int rh = rowH(t);
        t.setTextSize(1);
        const int lh = t.fontHeight() + 2;
        // The same arithmetic drawChans() lays the rows out with: the status
        // line, then the count line, then the list.
        const int listTop = BODY_TOP + 1 + t.fontHeight() + 3 + lh + 2;
        if (y < listTop) return LoraTap::NONE;
        const int row = s_scroll[(int)LoraView::CHANS] + (y - listTop) / rh;
        Lora::ChannelRow r;
        if (row >= 0 && Lora::channelRow((uint8_t)row, r)) {
            if (Lora::toggleChannelRow((uint8_t)row))
                Theme::showToast(r.enabled ? "MUTED" : "LISTENING", r.name,
                                 r.enabled ? Theme::VAPOR_PURPLE : Theme::CYAN);
            else
                Theme::showToast("BUILT IN", "stays on -- yours go in over LORA CHAN", Theme::AMBER);
            return LoraTap::HANDLED;
        }
        return LoraTap::NONE;
    }
    if (s_view == LoraView::LIST && y >= BODY_TOP) {
        const int rh = rowH(t);
        t.setTextSize(1);
        const int top = BODY_TOP + 1 + t.fontHeight() + 3;
        if (y < top) return LoraTap::NONE;
        const int row = s_scroll[0] + (y - top) / rh;
        if (row < (int)Lora::packetCount()) {
            s_open = (uint16_t)row; s_openTotal = Lora::packetTotal();
            s_view = LoraView::PACKET;
            return LoraTap::HANDLED;
        }
    }
    return LoraTap::NONE;
}
