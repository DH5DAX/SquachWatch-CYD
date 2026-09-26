// SquachWatch-CYD — the LORA screen: what the wireless slot hears.
//
// Five views on one screen, the way the DEX has its index and its cards:
// LIST is every frame newest first, one line each; a tap opens PACKET, the
// bytes and the decoded line; NODES is one row per transmitter; STATS is the
// radio channel -- counters, airtime, the noise floor, which profiles are
// busy; CHANS is the crypto channels, the keys the decoders hold. Plain text
// at size 1 on purpose: this is the maintenance tool, and a sysop wants the
// numbers, not the mascot. Reached from Settings' SYSTEM page; on boards
// without the slot the rows are never offered.
//
// The CHANS view can mute a channel and nothing else. A panel with no
// keyboard cannot type `#gelsenkirchen`, and pretending otherwise with an
// on-screen keyboard would be a worse lie than saying so: adding and dropping
// are `LORA CHAN` on the console, and the view says as much at its foot.
#pragma once
#include <TFT_eSPI.h>
#include <stdint.h>

class DetectionEngine;

enum class LoraView : uint8_t { LIST = 0, PACKET, NODES, STATS, CHANS };

// Opens on `v`; the SYSTEM page's LORA CHANNELS row comes straight to CHANS.
void uiLoraInit(TFT_eSPI& t, LoraView v = LoraView::LIST);
void uiLoraTick(TFT_eSPI& t, uint32_t now, const DetectionEngine& eng, bool advance = true);
void uiLoraScroll(int delta);                // positive = down
LoraView uiLoraView();

enum class LoraTap : uint8_t { NONE, HANDLED, BACK };
// Handles the bar and the rows itself; BACK means leave the screen.
LoraTap uiLoraTap(TFT_eSPI& t, int x, int y, int screenW, int screenH);
