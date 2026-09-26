// SquachWatch-CYD — the LORA screen. See include/ui_lora.h.
#include "ui_lora.h"
#include "ui_scroll.h"
#include "theme.h"
#include "lora_sniffer.h"
#include "lora_profiles.h"
#include "lora_classify.h"
#include "lora_nodes.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>

namespace {

LoraView s_view = LoraView::LIST;
int      s_scroll[4] = { 0, 0, 0, 0 };
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

void barLabels(const char* l[3]) {
    l[0] = "[ BACK ]";
    switch (s_view) {
        case LoraView::LIST:   l[1] = "[ NODES ]"; l[2] = "[ STATS ]"; break;
        case LoraView::NODES:  l[1] = "[ LIST ]";  l[2] = "[ STATS ]"; break;
        case LoraView::STATS:  l[1] = "[ LIST ]";  l[2] = "[ NODES ]"; break;
        default:               l[1] = "[ < ]";     l[2] = "[ > ]";     break;
    }
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
        const uint16_t duty = Lora::Nodes::dutyPermille(n, now);
        snprintf(line, sizeof line, "%-4s %-10.10s %-14.14s %4d %3u %4s %u.%u%% %s%s%s", Lora::protoShort(n.proto), n.tag,
                 n.name[0] ? n.name : Lora::Nodes::roleText(n), (int)n.rssi, (unsigned)n.packets, age,
                 duty / 10, duty % 10, n.hasPos ? "@ " : "", flags[0] ? "!" : "", flags);
        t.setTextColor(n.flags ? Theme::AMBER : col, Theme::BG);
        t.setCursor(12, y + 3);
        t.print(line);
    }
    Theme::drawScrollbar(t, w - 6, top, bottom - top, count, (bottom - top) / rh, s_scroll[2]);
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
    snprintf(b, sizeof b, "%lu.%lu s on the air heard, %lu.%02lu%% of uptime; noise %d dBm",
             (unsigned long)(s.airtimeMs / 1000), (unsigned long)((s.airtimeMs / 100) % 10),
             (unsigned long)(up ? s.airtimeMs / 10 / up : 0), (unsigned long)(up ? (s.airtimeMs * 10 / up) % 100 : 0), (int)s.noiseDbm);
    y = statLine(t, y, Theme::CYAN, "AIR:", b);
    size_t o = 0;
    for (int p = 1; p < (int)Lora::Proto::COUNT && o + 12 < sizeof b; p++)
        if (s.byProto[p]) o += (size_t)snprintf(b + o, sizeof b - o, "%s%s %u", o ? "  " : "", Lora::protoShort((Lora::Proto)p), (unsigned)s.byProto[p]);
    if (s.byProto[0]) snprintf(b + o, sizeof b - o, "%s?? %u", o ? "  " : "", (unsigned)s.byProto[0]);
    y = statLine(t, y, Theme::CYAN, "BY NET:", o || s.byProto[0] ? b : "-");
    y += 4;
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

void uiLoraInit(TFT_eSPI& t) {
    t.fillRect(0, 0, t.width(), t.height(), Theme::BG);
    s_view = LoraView::LIST;
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
        default:               snprintf(title, sizeof title, ">> LORA CHANNEL <<"); break;
    }
    Theme::drawTitleBar(t, title);
    int top = drawStatus(t, w, BODY_TOP + 1);
    switch (s_view) {
        case LoraView::LIST:   drawList(t, now, w, top, bottom); break;
        case LoraView::PACKET: drawPacket(t, now, w, top, bottom); break;
        case LoraView::NODES:  drawNodes(t, now, w, top, bottom); break;
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
            case LoraView::LIST:   s_view = which == 1 ? LoraView::NODES : LoraView::STATS; break;
            case LoraView::NODES:  s_view = which == 1 ? LoraView::LIST  : LoraView::STATS; break;
            case LoraView::STATS:  s_view = which == 1 ? LoraView::LIST  : LoraView::NODES; break;
            case LoraView::PACKET: {
                // < is towards the newer frame, > the older, in the list's order.
                const uint32_t total = Lora::packetTotal();
                uint32_t idx = s_open + (total - s_openTotal);
                if (which == 1 && idx > 0) idx--;
                if (which == 2 && idx + 1 < Lora::packetCount()) idx++;
                s_open = (uint16_t)idx; s_openTotal = total;
                break;
            }
        }
        return LoraTap::HANDLED;
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
