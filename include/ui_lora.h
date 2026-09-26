// SquachWatch-CYD — the LORA screen: what the wireless slot hears, and which
// antenna hears it best.
//
// Ten views on one screen. Six are the maintenance tool: LIST is every frame
// newest first and a tap opens PACKET, the bytes and the decoded line; the same
// ring filtered to MeshCore adverts is a seventh nothing extra is stored for;
// NODES is one row per transmitter; TRAFFIC is the rate and the occupancy of the
// air as they move; CHANS is the crypto channels, and a tap on one opens
// CHANMSG, what actually came through that key; STATS is the radio channel.
// Two are the antenna survey -- SURVEY, the stations heard first-hand with the
// trend of each one's signal, and SURVEYCMP, two runs of it against each other,
// which is where the measurement actually happens. PICK is the grid that
// reaches every one of them in one tap.
//
// HOW THEY ARE REACHED, because four views used to sit in a ring with the two
// bar buttons naming their neighbours and ten cannot. A ring of ten is five taps
// wide, and a button that says [ TRAFFIC ] tells you nothing about where the
// other eight are. So the bar is [ BACK ] [ VIEWS ] [ SURVEY ] on every view:
// the picker puts all ten on the glass at once with a live figure under each, so
// every view is two taps from every other, and the survey -- the reason this
// phase exists -- is one tap from all of them and never behind the picker. What
// a view needs beyond that it draws for itself, big, in its own body: the
// survey's START/STOP is a 392-pixel button because it is pressed with a thumb
// while the other hand holds an antenna.
//
// Plain text at size 1 for the dense lists on purpose: this is the maintenance
// tool and a sysop wants the numbers, not the mascot. The SURVEY view is the
// deliberate exception and src/ui_lora.cpp says why -- it is read at arm's
// length, outdoors, one-handed.
//
// What a finger can do here is limited by design, and the views say so at their
// foot rather than pretending otherwise: a panel with no keyboard cannot type
// `#gelsenkirchen` or `dipole at the balcony rail`, so adding and dropping
// channel keys is `LORA CHAN` and naming a survey run is `LORA SURVEY LABEL`.
#pragma once
#include <TFT_eSPI.h>
#include <stdint.h>

class DetectionEngine;

enum class LoraView : uint8_t {
    LIST = 0,     // every frame, newest first (ADVERTS is this one filtered)
    PACKET,       // one frame: the fields, the decode, the bytes
    NODES,        // one row per transmitter
    STATS,        // the radio channel: counters, airtime, noise, the spectrum
    CHANS,        // the crypto channels and their keys
    CHANMSG,      // what came through one channel's key -- or through all of them
    TRAFFIC,      // the rate and the occupancy of the air, as they move
    SURVEY,       // the antenna survey: the stations heard first-hand, live
    SURVEYCMP,    // two survey runs against each other: the measurement
    PICK,         // the grid that reaches all of the above
    COUNT
};

// Opens on `v`; the SYSTEM page's LORA CHANNELS row comes straight to CHANS.
// The views that need a subject chosen first (PACKET, CHANMSG) and the picker
// itself open on LIST instead.
void uiLoraInit(TFT_eSPI& t, LoraView v = LoraView::LIST);
void uiLoraTick(TFT_eSPI& t, uint32_t now, const DetectionEngine& eng, bool advance = true);
void uiLoraScroll(int delta);                // positive = down
// How far a finger must travel for one row of scroll: the height of a row in
// whichever view is showing. main.cpp's drag compared against a flat 10 px,
// which on a finger-sized row scrolled the list two rows for every row of
// finger travel -- it slipped out from under the thumb -- and called a 12 px
// wobble during a tap a drag.
int  uiLoraDragStep(TFT_eSPI& t);
LoraView uiLoraView();

enum class LoraTap : uint8_t { NONE, HANDLED, BACK };
// Handles the bar, the rows and the in-body buttons itself; BACK means leave
// the screen, and the views that sit under another one (PACKET under LIST,
// CHANMSG under CHANS, the picker over whatever opened it) answer HANDLED and
// go up one level instead.
LoraTap uiLoraTap(TFT_eSPI& t, int x, int y, int screenW, int screenH);
