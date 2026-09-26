// SquachWatch-CYD — the LORA screen: what the wireless slot hears.
//
// Four views on one screen, the way the DEX has its index and its cards:
// LIST is every frame newest first, one line each; a tap opens PACKET, the
// bytes and the decoded line; NODES is one row per transmitter; STATS is
// the channel -- counters, airtime, the noise floor, which profiles are
// busy. Plain text at size 1 on purpose: this is the maintenance tool, and
// a sysop wants the numbers, not the mascot. Reached from Settings' SYSTEM
// page; on boards without the slot the row is never offered.
#pragma once
#include <TFT_eSPI.h>
#include <stdint.h>

class DetectionEngine;

enum class LoraView : uint8_t { LIST = 0, PACKET, NODES, STATS };

void uiLoraInit(TFT_eSPI& t);
void uiLoraTick(TFT_eSPI& t, uint32_t now, const DetectionEngine& eng, bool advance = true);
void uiLoraScroll(int delta);                // positive = down
LoraView uiLoraView();

enum class LoraTap : uint8_t { NONE, HANDLED, BACK };
// Handles the bar and the rows itself; BACK means leave the screen.
LoraTap uiLoraTap(TFT_eSPI& t, int x, int y, int screenW, int screenH);
