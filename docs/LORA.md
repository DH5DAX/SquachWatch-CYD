# LoRa on the CrowPanel 7

What the Elecrow LoRa module can hear, decode and send, and how it could make
SquachWatch a LoRa pocket knife for detection, classification, decoding and
for the people who run LoRa networks.

This is research, not code. It was written on 2026-09-26 against what was
current that day: Meshtastic firmware 2.8.x, MeshCore 1.17.1, RadioLib 7.7.1,
LoRaWAN Regional Parameters RP002-1.0.5 and MeshCom 4.35. Where a fact comes
from a project's source code rather than its docs, the source was read. The
values checked by recomputing them are marked **checked**. Anything that could
not be confirmed from a primary source is marked **unverified**. Some questions
only the board itself can answer; those are collected in
[Measure first](#10-measure-first).

## What is built

The research below became code on 2026-09-26, in the `crowpanel7` build
(`-DSQUACH_LORA`), and none of it has met the board yet: it compiles, the
decoders pass their desktop tests against published vectors, and
[Measure first](#10-measure-first) is still the list of what only the
hardware can answer.

- **Bring-up** (`lora_radio.cpp`): a reset pulse and a raw read of the
  version register before RadioLib, then the TCXO voltages tried in the
  datasheet's order. `LORA` on the console says what it found;
  DIAGNOSTICS has a LORA line.
- **The sniffer** (`lora_sniffer.cpp`): its own task, a ring in PSRAM,
  FOCUS and SURVEY (CAD rotation with a two-second linger after a hit), a
  SWEEP that draws 863–870 MHz, and `LORA TAP` for Wireshark through
  `tools/loratap2pcap.py`. Profiles are section 4's table, in
  `lora_profiles.cpp`.
- **Decoders**, each standalone with a host test: Meshtastic, MeshCore,
  LoRaWAN, LoRa APRS, MeshCom and FANET. The keys the user holds go in
  through `Meshtastic::addChannel`, `MeshCore::addChannel` and
  `LoRaWAN::addSession`; a settings screen for them is still to do.
- **The LORA screen** (`ui_lora.cpp`): LIST, FRAME, NODES with the sysop
  flags, CHANNEL with the counters and the spectrum. LORA MODE and LORA
  PROFILE on the SYSTEM page.
- **Not built**: the FSK profiles (OGN, ADS-L, wM-Bus), transmitting, the
  `LORA_TRACKER` detection, and the LoRaWAN downlink chase and beacon
  scheduling; the profiles for RX2 and the beacon exist and FOCUS can park
  on them.

## The short version

- **The module is a Semtech SX1262** with a TCXO and an RF switch driven by
  DIO2, sold for 868 or 915 MHz. It has one receiver, which listens on one
  frequency, one bandwidth, one spreading factor, one sync word and one IQ
  polarity at a time. A LoRaWAN gateway hears 8 channels at every spreading
  factor at once; this board hears a slice. Everything below is designed
  around that.
- **LoRa and the SD card cannot be used at the same time.** The DIP switch K1
  routes GPIO 4/5/6 to one or the other. Captures have to go to flash, which
  has about 7.4 MB spare on this board.
- **What it can decode with no secret at all:**
  - Meshtastic: every header, plus every channel on the default key.
  - MeshCore: adverts, relay paths, trace SNRs, and the Public and hashtag
    channels.
  - Amateur and aviation systems, which are plaintext by design: LoRa APRS,
    MeshCom, FANET, OGN and ADS-L.
  - LoRaWAN headers, join requests and Class B beacons.
- **What it can decode with keys the user holds:** LoRaWAN payloads from the
  user's own devices, and private Meshtastic and MeshCore channels.
- **What it cannot decode:** Meshtastic PKI direct messages, MeshCore direct
  messages, LR-FHSS, Sigfox uplinks, mioty, Amazon Sidewalk, and the content
  of FLARM.
- **433 MHz is marginal.** LoRa APRS, MeshCom, the 433 Meshtastic and MeshCore
  presets and most LoRa satellites all live there. The 868 module will only
  hear strong local signals on 433, and it should never transmit there.
- **German law limits what may be decoded by default.** Receiving a message
  that was not meant for the public, for radio amateurs or for the operator is
  prohibited under § 5 TDDDG (see [section 8](#8-the-law)).

---

## 1. The hardware

### The module

| | |
|---|---|
| Product | "Wireless module for CrowPanel Advanced Series", variant SX1262 (LoRa). The SKU is DAC0010, which covers every variant. It is a +$6.55 add-on on the 7.0 product page |
| Chip | Semtech SX1262, per Elecrow. No module schematic is published, and the part is **unverified** beyond Elecrow's word; reading the chip's version string at boot settles it |
| Bands | Elecrow lists only 868 MHz and 915 MHz versions. There is no 433 MHz version |
| RF switch | DIO2: low means receive, high means transmit. RadioLib's `begin()` already sets `setDio2AsRfSwitch(true)` |
| Oscillator | A TCXO powered from DIO3. Elecrow's ESP32-S3 examples and Meshtastic both use 3.3 V, while Elecrow's ESP32-P4 example uses 1.6 V. The datasheet wants VDD above VTCXO + 200 mV, so 3.3 V cannot be fully met on a 3.3 V rail. Measure it |
| TX power | +22 dBm is the chip maximum. No module figure is published |
| Antenna | IPEX-1 (U.FL) connector, 10 cm pigtail, 3.5 dBi antenna |

### Wiring on this board

The board has the STC8 helper at 0x30, so it is a 7.0 **V1.2 or later**. V1.0
used a GPIO expander there, and its LoRa chip select is on GPIO 0 instead
of 8.

| Module pin | ESP32-S3 GPIO | Path |
|---|---|---|
| SCK / MISO / MOSI | 5 / 4 / 6 | CH486F mux U11 (shared with the SD card and I2S) |
| NSS | **8** (V1.2+; V1.0 used 0) | direct |
| DIO1 (IRQ) | 20 | CH486F mux U9 |
| NRESET | 19 | CH486F mux U9 |
| BUSY | 2 | direct on V1.3+, through U9 on V1.2 |

This is Elecrow's own code for V1.2 to V1.5:

```cpp
SPI.begin(5, 4, 6, 8);
SX1262 radio = new Module(8 /*NSS*/, 20 /*DIO1*/, 19 /*NRST*/, 2 /*BUSY*/, SPI);
```

Watch out for the net names on the 7.0 V1.2 schematic: `IO2_W_CS` and
`IO8_BUSY` are the wrong way round. The 4.3" schematics and Elecrow's code both
say NSS is 8 and BUSY is 2.

### The DIP switch, and why there is no SD card

K1 is a two-position DIP switch. Its two lines select a channel on both CH486F
muxes, and no GPIO can change it. It works like this:

| S1 | S0 | 7.0 V1.2 | 7.0 V1.3 / 1.4 / 1.5 |
|---|---|---|---|
| 0 | 0 | speaker and mic | speaker and PDM mic |
| 0 | **1** | **wireless module** (and UART1 on J12) | **wireless module** (and UART1 on J12) |
| 1 | 0 | SD card | nothing |
| 1 | 1 | SD card and mic | SD card and mic |

Elecrow staff say the same on their forum: LoRa and the SD card cannot coexist.

What that means for the firmware:

- **Probe at boot for whichever one is connected.** The SD card answers a raw
  CMD0 with 0x01, as `crowpanel7_probe.cpp` already does. An SX126x answers a
  register read of 0x0740/0x0741 with 0x14 0x24 straight after reset (its
  default sync word). Show the result in DIAGNOSTICS so a wrongly set switch is
  obvious.
- **Log to flash instead of the card.** The 16 MB part is empty after the
  coredump partition (0x840000 to the end), so there is roughly 7.4 MB for a
  capture ring.
- **The switch has side effects.** In the wireless position the speaker is off,
  but the buzzer still works because it sits behind the STC8. The J12 UART
  header must stay empty, because it shares GPIO 19/20 with the module.

### Board-specific catches

- **GPIO 19/20 are the ESP32-S3's USB-JTAG pins.** That is harmless here,
  because USB goes through the CH340K and USB CDC stays off.
- **Scattered CPU writes into PSRAM make the RGB panel twitch** (see
  `crowpanel7_board.h`). Packet rings and node tables therefore belong in
  internal RAM, or have to be written to PSRAM in large sequential blocks.
  Flash writes should be batched too; the `crowpanel7-paneltest` build
  measures what they do to the picture.
- **Off-the-shelf Meshtastic does not run on this board.** Its `elecrow_panel`
  build hard-codes chip select 0, which only fits V1.0; Meshtastic issue
  #11393, still open, asks for a V1.2+ target. MeshCore has no CrowPanel
  Advance port at all.
- **RadioLib 7.7.1 on arduino-esp32 2.0.14 is untested.** Elecrow's V1.5
  examples use 7.7.1, but RadioLib's CI builds against the 3.x core. It has to
  be compiled before anything else.

---

## 2. What the SX1262 can and cannot do

| Capability | Status |
|---|---|
| LoRa receive | Yes: SF5–SF12, BW 7.8–500 kHz, explicit or implicit header, CRC on or off, IQ normal or inverted. Sensitivity at BW125 is −124 dBm at SF7 and −137 dBm at SF12 |
| Coding rate | Does not need to be set. In explicit-header mode, CR, payload length and the CRC-present flag all arrive in the header |
| Sync word | A hardware filter on 2 bytes (register 0x0740). RadioLib maps a 1-byte word `sw` to `(sw&0xF0)\|4, (sw<<4)\|4`, so 0x12 becomes 0x1424, 0x34 becomes 0x3444, 0x2B becomes 0x24B4 and 0xF1 becomes 0xF414. There is no promiscuous mode; foreign sync words sometimes get through, but not reliably (RadioLib discussion #1135) |
| CAD (channel activity detection) | Yes, for one SF and BW at a time. By how it works it ignores the sync word, and it detects preamble or data symbols. A 4-symbol CAD takes about 4.6 ms at SF7/125, 9 ms at SF9/250, 18 ms at SF9/125, 37 ms at SF11/250 and 147 ms at SF12/125. RadioLib: `scanChannel()` / `startChannelScan()` |
| Signal measurements | Per-packet RSSI and SNR; instantaneous RSSI; `getFrequencyError()`, which RadioLib marks as undocumented on SX126x (use with care); and the despread signal RSSI, which is only reachable by subclassing |
| Spectral scan | Needs a binary patch uploaded on every power-up (`SX126x_patch_scan.h`). It returns a 33-bin RSSI histogram per frequency and is marked experimental in RadioLib |
| GFSK / FSK | Yes: 0.6–300 kb/s, sync word up to 8 bytes, CRC with custom polynomial and seed, whitening, 255-byte packets. There is no hardware Manchester decoding and no AFC |
| OOK receive | **No.** This rules out the rtl_433 world of 433 MHz weather stations and remotes |
| LR-FHSS | **Transmit only.** The host builds the frames; receiving needs an SX1302/SX1303 gateway |
| BPSK | **Transmit only** (100 or 600 bps). So Sigfox uplinks cannot be received |
| Frequency range | 150–960 MHz, but the module's matching network is tuned for one band. Off-band loss is **unverified**; a guess is 10–20+ dB |

**What has to match for a packet to arrive.** Frequency, bandwidth, spreading
factor, sync word, IQ polarity and LDRO must all match the transmitter. The
receiver's preamble setting must be at least the transmitter's, so program
the longest expected (Semtech's advice). A mismatched LDRO produces a valid
header followed by a CRC error, and that symptom is itself worth reporting.

**Compared with a gateway.** An SX1302 decodes 8 channels at every spreading
factor, plus one fixed-SF channel and one FSK channel, up to 16 packets in
parallel. The SX1262 does one of each. A sniffer built on it is statistical,
and it should say so on screen.

---

## 3. The catalogue

### At a glance

| System | Where in DE | PHY | Sync | Readable with no key | Readable with a public key | Needs a private key | Transmit |
|---|---|---|---|---|---|---|---|
| Meshtastic | 869.525 MHz, BW250, SF11/9/8 (by region) | LoRa | 0x2B | whole 16-byte header: who, to whom, packet id, hops, relayer | default-key channels: text, names, hardware, positions, telemetry, traceroutes | private channels, PKI DMs, admin | yes, with an identity of its own |
| MeshCore | 869.618 MHz, BW62.5, SF8 | LoRa | 0x12 | adverts (key, name, role, GPS), relay path, trace SNR per hop | Public channel, #hashtag channels | DMs, repeater requests, private channels | yes, with an identity of its own |
| LoRaWAN (TTN, Helium, operators) | 868.1–868.5 and 867.1–867.9 MHz up, 869.525 MHz RX2 | LoRa, one FSK rate | 0x34 | DevAddr (so the operator), FCnt, flags, join EUIs (so the manufacturer), 1.0.x MAC commands, Class B beacons | – | payloads (AppSKey), MAC on port 0 | only as the user's own registered device |
| LoRa APRS | 433.775 MHz, BW125, SF12 | LoRa | 0x12 | everything (plaintext TNC2) | – | – | licensed amateurs, but not with this module |
| MeshCom 4 | 433.175 MHz, BW250, SF11 | LoRa | 0x2B | everything (plaintext) | – | – | licensed amateurs, but not with this module |
| FANET | 868.2 MHz, BW250, SF7 | LoRa | 0xF1 | everything | – | – | possible, but it is aviation safety traffic: keep out |
| OGN tracker | 868.2 / 868.4 MHz | GFSK 100 kcps Manchester | 8 bytes | everything (the whitening key is zero) | – | – | no |
| ADS-L (EASA) | 868.2 / 868.4, 869.525 MHz | GFSK | 2 bytes | everything (the scrambling key is zero) | – | – | no |
| FLARM | 868.2 / 868.4 MHz | GFSK | – | that a burst happened | – | proprietary: do not decode | no |
| Reticulum / RNode | user-set (example 867.2 MHz) | LoRa | 0x12 | header, hashes, announces (names) | – | links, data | not in scope |
| Wireless M-Bus T1/C1 | 868.95 MHz | GFSK 100 k | 0x543D | manufacturer, meter ID, type, version | – | readings (AES) | no |
| UKHAS balloons | 434.x MHz | LoRa, BW20.8–250 | 0x12 | everything | – | – | no |
| LoRa satellites | mostly 400–470 MHz, a few 863–870 | LoRa | mostly 0x12 | per satellite, mostly plaintext | – | – | no |

### 3.1 Meshtastic

**Radio settings.** The frame uses sync word 0x2B, a 16-symbol preamble, an
explicit header, CRC on, normal IQ, and is at most 255 bytes. The main
presets are:

| Preset | SF | BW (kHz) | CR |
|---|---|---|---|
| ShortFast | 7 | 250 | 4/5 |
| ShortSlow | 8 | 250 | 4/5 |
| MediumFast | 9 | 250 | 4/5 |
| MediumSlow | 10 | 250 | 4/5 |
| LongFast (default) | 11 | 250 | 4/5 |
| LongMod | 11 | 125 | 4/8 |
| LongSlow | 12 | 125 | 4/8 |

- **Not allowed in EU_868:** the Turbo presets (ShortTurbo, MediumTurbo,
  LongTurbo, all at BW500).
- **New in 2.8:** NarrowFast and NarrowSlow (SF7/8 at BW62.5, CR 4/6) for
  EU_N_868 and the amateur 70 cm band, and LiteFast and LiteSlow for EU_866.

**Frequency.** It follows from the region and a djb2 hash of the channel name
(the preset name if the name is empty):

```
slotWidth = spacing + 2*padding + bw
numSlots  = round((end - start + spacing) / slotWidth)
slot      = djb2(name) % numSlots          // h = 5381; h = h*33 + c
freq      = start + bw/2 + padding + slot*slotWidth
```

EU_868 (869.4–869.65 MHz) has room for one 250 kHz slot, so every 250 kHz
preset lands on **869.525 MHz**. The rest are:

- LongSlow: 869.4625 MHz. LongMod: 869.5875 MHz.
- EU_N_868 (NarrowSlow): 869.442 MHz.
- EU_433: LongFast on 433.875 MHz.
- US: LongFast on 906.875 MHz.

All of these are **checked**.

**What German communities actually run.** They sit on 869.525 MHz and differ
only by SF: some still run LongFast, Kiel and Berlin moved to MediumFast, and
Hessen and Rheinland moved to ShortSlow. So in Germany, cycling SF11, SF9 and
SF8 on one frequency covers nearly everyone.

**The 16-byte header, always in the clear** (little-endian):

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | `to`; 0xFFFFFFFF means broadcast |
| 4 | 4 | `from` (node number, shown as `!%08x`) |
| 8 | 4 | packet id |
| 12 | 1 | bits 0–2 hop_limit, bit 3 want_ack, bit 4 via_mqtt, bits 5–7 hop_start |
| 13 | 1 | channel hash; 0x00 means a PKI DM |
| 14 | 1 | next_hop: last byte of the next hop (2.6+) |
| 15 | 1 | relay_node: last byte of whoever sent this copy (2.6+) |
| 16 | ≤239 | encrypted `Data` protobuf |

Hops so far are `hop_start − hop_limit`. A hop_start of 0 means firmware
older than 2.3, and a relay_node of 0 means firmware older than 2.6. From
2.8, the node number is CRC32 of the node's public key.

**Channel hash and key.** The hash is XOR(name bytes) XOR XOR(key bytes).
The default PSK `AQ==` expands to `d4f1bb3a20290759f0bcffabcf4e6901`.
Default-key hashes are LongFast 0x08, MediumFast 0x1F, ShortSlow 0x77,
ShortFast 0x70, MediumSlow 0x18, LongSlow 0x0F, LongMod 0x6E and NarrowSlow
0x12 (all **checked**). A frame with one of those hashes on the matching
preset can be decrypted with the published key. In ham mode the PSK is
stripped and the payload is plain protobuf.

**Encryption.** Channel traffic is AES-CTR (AES-128 for 16-byte keys) with the
nonce `id (u64 LE) ‖ from (u32 LE) ‖ 0`. There is no MAC, so a decryption is
judged by whether the protobuf parses. PKI DMs (2.5+) use X25519 and AES-CCM
and cannot be read by a third party. 2.8 adds XEdDSA signatures inside `Data`
(field 10), which can be verified against the public key from NodeInfo.

**What is worth decoding.** A small hand-written protobuf reader is enough; the
firmware does not need all of nanopb. These are the ports that matter:

| Port | Number | Contents |
|---|---|---|
| TEXT_MESSAGE | 1 | text |
| POSITION | 3 | lat/lon ×1e-7, altitude, precision |
| NODEINFO | 4 | long and short name, hardware model, role, public key, licensed flag |
| ROUTING | 5 | ACK/NAK and error codes |
| TELEMETRY | 67 | battery, voltage, channel utilisation, air_util_tx, uptime; plus environment and local stats |
| TRACEROUTE | 70 | the route both ways, with SNR×4 per hop |
| NEIGHBORINFO | 71 | neighbours with SNR |
| RANGE_TEST | 66 | range test |
| WAYPOINT | 8 | waypoint |
| PAXCOUNTER | 34 | paxcounter |

**Flooding.** A copy counts as a duplicate when (from, id) repeats. The
rebroadcast delay depends on SNR, so distant nodes go first. CLIENT nodes
cancel their copy when they hear someone else relay it; ROUTER and
ROUTER_LATE never cancel. Since 2.6, DMs use next-hop routing once a route
has been learned.

The roles are:

| Role | Number |
|---|---|
| CLIENT | 0 |
| CLIENT_MUTE | 1 |
| ROUTER | 2 |
| ROUTER_CLIENT (deprecated) | 3 |
| REPEATER (deprecated) | 4 |
| TRACKER | 5 |
| SENSOR | 6 |
| TAK | 7 |
| CLIENT_HIDDEN | 8 |
| LOST_AND_FOUND | 9 |
| TAK_TRACKER | 10 |
| ROUTER_LATE | 11 |
| CLIENT_BASE | 12 |

The default intervals are:

| Message | Default interval |
|---|---|
| NodeInfo | 3 h |
| Telemetry | 60 min |
| Position | 60 min |
| NeighborInfo | 6 h, and never on the default channel |

### 3.2 MeshCore

**Radio settings.**

| Preset | Frequency | BW | SF | CR | Status |
|---|---|---|---|---|---|
| EU/UK Narrow | 869.618 MHz | 62.5 | 8 | 4/8 | Germany's choice since 2025 |
| EU deprecated | 869.525 MHz | 250 | 11 | 4/5 | old |
| Czech | 869.432 MHz | 62.5 | 7 | – | |
| EU 433 | 433.650 MHz | 250 or 62.5 | – | – | |
| US | 910.525 MHz | 62.5 | 7 | – | |

- **Framing:** sync word 0x12, CRC on, explicit header, normal IQ.
- **Preamble:** 32 symbols at SF ≤ 8, otherwise 16 (1.16+).
- **Maximum frame size:** 255 bytes.

The deprecated EU preset is physically identical to Meshtastic LongFast, and
only the sync word tells the two apart.

**Packet layout:**

```
header(1) [transport codes(4), route types 0 and 3 only] path_len(1) path(n) payload(≤184)
header   = VV PPPP RR   route: 0 transport-flood, 1 flood, 2 direct, 3 transport-direct
path_len = bits 0-5 hop count, bits 6-7 hash size − 1 (1, 2 or 3 bytes per hop)
```

The payload types are:

| Type | Name | Type | Name |
|---|---|---|---|
| 0x00 | REQ | 0x07 | ANON_REQ |
| 0x01 | RESPONSE | 0x08 | PATH |
| 0x02 | TXT_MSG | 0x09 | TRACE |
| 0x03 | ACK | 0x0A | MULTIPART |
| 0x04 | ADVERT | 0x0B | CONTROL |
| 0x05 | GRP_TXT | 0x0F | RAW_CUSTOM |
| 0x06 | GRP_DATA | | |

**How the path works.** On a flood, every repeater appends its hash to the
path, so the sniffer sees the whole relay chain. On a direct packet the path
holds the hops still to come. Only repeaters and room servers relay;
companions never do.

**ADVERT, always plaintext and always signed:**

| Offset | Size | Field |
|---|---|---|
| 0 | 32 | Ed25519 public key; its first byte is the node's hash |
| 32 | 4 | timestamp (unix, LE) |
| 36 | 64 | signature over key ‖ timestamp ‖ appdata |
| 100 | 1 | flags: low nibble is the type (1 chat, 2 repeater, 3 room, 4 sensor); 0x10 lat/lon present, 0x80 name present |
| … | 4+4 | lat, lon as int32 ×1e6 |
| … | rest | name |

**Crypto.**

- **Cipher and MAC:** AES-128-ECB, followed by a 2-byte truncated
  HMAC-SHA256.
- **Channel hash:** SHA256(key)[0].
- **Public channel:** key `8b3387e9c5cdea6ac9e5edbaa115cd72`, hash 0x11
  (**checked**).
- **Hashtag channels:** the key is SHA256("#name")[:16]; for example `#test`
  gives hash 0xD9 (**checked**). Anyone who knows the name can read the
  channel, and the user can type in the names they use.
- **Private traffic:** DMs, requests and paths use X25519 between the two
  nodes and cannot be read by a third party.

**Other things in the clear:**

- **TRACE** collects SNR×4 per hop in the path field. It is a ready-made link
  quality report.
- **CONTROL** discovery responses carry SNR and public keys.
- **Transport codes** (regions) are a 2-byte HMAC with key
  SHA256("#region")[:16]. A region with a public name can be recognised by
  trying the known names.
- **Duplicate key:** SHA256(type ‖ payload)[:8]. It stays the same across
  hops, which groups the copies of one packet together.

### 3.3 LoRaWAN: TTN, Helium and the operators

**EU868 radio.**

- **Uplinks** use 868.1, 868.3 and 868.5 MHz (mandatory), plus 867.1–867.9
  MHz on TTN, with DR0–5 (SF12 to SF7 at BW125).
- **Other data rates:** DR6 is SF7/BW250 on 868.3 MHz. DR7 is FSK 50 kbps on
  868.8 MHz with sync 0xC194C1, which the SX1262 can receive.
- **Data rates the SX1262 cannot receive:** DR8–11 are LR-FHSS. DR12/13
  (SF6/SF5) are new in RP002-1.0.5 and are still rare; they use sync 0x12
  instead of 0x34.

**Uplink, downlink and beacon use different radio settings.** This is the big
catch for a single radio:

| | Sync | Preamble | Header | CRC | IQ |
|---|---|---|---|---|---|
| Uplink | 0x34 | 8 | explicit | **on** | normal |
| Downlink | 0x34 | 10 | explicit | **off** | **inverted** |
| Class B beacon | 0x34 | 10 | **implicit**, 17 bytes | **off** | normal |

**Timing.**

| | RX1 | RX2 |
|---|---|---|
| Data (spec default) | 1 s after the uplink, same frequency and DR | 2 s after the uplink |
| Data (The Things Stack) | 5 s | 6 s |
| Join-accept | 5 s | 6 s |

RX2 is 869.525 MHz at SF12 by the spec, but SF9 on TTN. Helium keeps the spec
value; try both.

**The frame.** Everything is little-endian:

```
MHDR(1) = FType(3) RFU(3) Major(2)
  000 join-request   001 join-accept   010/100 unconfirmed/confirmed up
  011/101 unconfirmed/confirmed down   111 proprietary
Data:         DevAddr(4) FCtrl(1) FCnt(2) FOpts(0-15) [FPort(1) FRMPayload] MIC(4)
Join-request: JoinEUI(8) DevEUI(8) DevNonce(2) MIC(4)   <- all in the clear
```

FCtrl on an uplink carries ADR, ADRACKReq, ACK, ClassB and FOptsLen; on a
downlink the ClassB bit is FPending instead. In LoRaWAN 1.0.x the FOpts MAC
commands are plaintext, and 1.1 encrypts them. The useful 1.0.x commands are:

- LinkCheckAns: margin and gateway count
- LinkADRReq: DR, power and channel mask
- DevStatusAns: battery and SNR
- DeviceTimeAns
- NewChannelReq and RXParamSetupReq

**Who runs the network, from the DevAddr alone.** A DevAddr starts with a
unary type prefix and a NwkID, which map to a NetID and then to an operator.
The official list is the LoRa Alliance NetID allocation, revision R111 (2025-09).
The operators most likely to be seen in Germany are:

| DevAddr | Operator |
|---|---|
| `26`/`27`xxxxxx | The Things Network (NetID 0x000013); `260B`xxxx is TTS eu1 |
| `78`/`79`xxxxxx | Helium (0x00003C); also `E05A`xxxx (0x60002D) and `FC014C`xx (0xC00053); the legacy `48` range |
| `74`/`75`xxxxxx | Minol ZENNER Connect (0x00003A) |
| `6C`/`6D`xxxxxx | Netze BW (0x000036) |
| `30`/`31`xxxxxx | Loriot (0x000018) |
| `1E`/`1F`xxxxxx | Orange (0x00000F) |
| `04`/`05`xxxxxx | Actility (0x000002) |
| `08`/`09`xxxxxx | Swisscom (0x000004) |
| `16`/`17`xxxxxx | Everynet (0x00000B) |
| `76`/`77`xxxxxx | Semtech (0x00003B) |
| `FC0160`xx | TTN Foundation, type 6 (0xC00058) |

Types 3–7 share prefixes between several NetIDs, so there the answer can be
"one of these".

**Who made the device, from a join.** The DevEUI and JoinEUI are EUI-64s, and
their OUI names the maker:

| OUI | Maker |
|---|---|
| 24:E1:24 | Milesight |
| A8:40:41 | Dragino |
| AC:1F:09 | RAKwireless |
| 00:16:16 | Browan |
| 64:7F:DA | Tektelic |
| 20:63:5F | Abeeway |
| 04:B6:48 | Zenner |
| 2C:F7:F1 | Seeed |
| 60:81:F9 | Helium Systems |

The 70:B3:D5 range holds many small LoRa vendors.

Some JoinEUIs identify the join server:

- `70B3D57ED0000000` is The Things Join Server.
- All zeros is the usual hobbyist setting.
- `0016C001FFFE0001` is Semtech LoRa Cloud, which was switched off on
  2025-07-31, so it only turns up on devices nobody has reconfigured.

**What else can be derived without keys:**

- per-device message rate and FCnt (lost packets, resets, rejoins)
- confirmed versus unconfirmed traffic, and ADR state
- SF distribution and airtime against the duty cycle
- downlinks paired with their uplinks by timing
- gateway positions from Class B beacons:
  - The beacon is 17 bytes: RFU, Param, 4-byte GPS time, CRC, 7 gateway-specific
    bytes, CRC.
  - InfoDesc values 0–2 carry the gateway's latitude and longitude, and 3
    carries its NetID and gateway ID.
  - Beacons go out every 128 s, aligned to the GPS epoch.

**With keys (sysop mode).** The MIC is AES-CMAC over the B0 block and the
message; the payload is AES-CTR-like with the A_i blocks. OTAA 1.0.x session
keys come from the join-accept and the AppKey. The algorithm was **checked**
against the lora-packet test vector: frame
`40F17DBE4900020001954378762B11FF0D` gives DevAddr 49BE7DF1, FCnt 2, FPort 1,
payload `test`, and MIC 2b11ff0d matches. RadioLib and the mbedTLS in the
core already provide AES and CMAC.

For decoding payloads:

- Cayenne LPP is small and worth building in.
- Vendor codecs are JavaScript in TheThingsNetwork/lorawan-devices. For the
  device, a handful should be ported by hand (Elsys TLV, Milesight
  channel-typed, Dragino fixed structs).

**Helium in 2026.** On the air it is plain LoRaWAN, recognisable only by its
DevAddr ranges.

- **Beacons are going away.** Proof-of-Coverage beacons were proprietary-MType
  frames of 51 random bytes on the uplink channels, sent at most every 6 h.
  HIP-149 (approved, starting 2026-06) retires Proof-of-Coverage on IoT, so
  don't count on seeing them.
- **Data-only hotspots** simply forward traffic.

**LoRaWAN Relay (TS011).** Wake-on-radio frames go out on 865.1 and 865.5 MHz,
with ACKs on 865.3 and 865.9 MHz, all with inverted IQ. Relayed uplinks reach
the network on FPort 226.

### 3.4 LoRa APRS

- **Radio settings:** 433.775 MHz, BW125, SF12, CR 4/5, sync 0x12,
  preamble 8, CRC on.
- **Downlink:** 433.900 MHz carries gateway-to-node traffic (DARC band plan,
  May 2025).
- **Other countries:** Poland uses 434.855 MHz at SF9, CR 4/7; the UK uses
  439.9125 MHz.

**Frame format.** The payload is `3C FF 01` followed by an ASCII TNC2 line,
`SRC>DEST,PATH:info`, all plaintext. The decoder needs APRS uncompressed,
base91-compressed (the CA2RXU tracker's default) and Mic-E position formats,
plus messages, status and telemetry.

**Path conventions.** The default path is `WIDE1-1`. A digipeater writes
`CALL*` into the path, and an iGate adds `qAR`/`qAO` when it uploads. A binary
AX.25 variant exists (sh123/esp32_loraprs, CR 4/7) but is incompatible with
the rest.

**An SF12 packet takes 1–3 s of airtime**; 80 bytes is about 3.3 s
(**checked**).

### 3.5 MeshCom 4

MeshCom is the OE1KBC / icssw.org project (MIT licence, source at
icssw-org/MeshCom-Firmware).

- **Radio settings:** 433.175 MHz, BW250, SF11, CR 4/6, **sync 0x2B**,
  **preamble 32**, CRC on.
- **Shared sync word:** MeshCom and Meshtastic use the same sync word.
  Frequency and payload tell them apart.
- **Presets:** there is an 868 preset on 869.525 MHz.
- **Mirror on LoRa APRS:** MeshCom also sends positions on 433.775 MHz as LoRa
  APRS, with destination `APRSMC`.

The frame is plaintext:

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | type: `:` text, `!` position/telemetry, `@` HEY beacon, `A` ACK |
| 1 | 4 | message id (LE) |
| 5 | 1 | bits 0–3 hops left (default 5); 0x80 already via server; 0x40 record the relay path |
| 6 | … | ASCII `SRC[,RELAY…]>DEST` + type char + payload, then 0x00 |
| … | 1+1 | hardware id, modem id |
| … | 2 | checksum: a 16-bit arithmetic sum, big-endian |
| … | 1+1+1 | firmware version, last-hop hardware id, sub-version |
| … | 1 | 0x7E |

An ACK is 12 bytes: `41`, its own id (4), `0x80|hops`, the id being
acknowledged (4), `01`, `00`. HEY frames collect `CALL,RSSI,SNR;` per relay,
which is a signal report for the whole route.

### 3.6 FANET

FANET is the paragliding network, with an open spec in `3s1d/fanet-stm32`
`protocol.txt`.

- **Radio settings:** 868.2 MHz, BW250, SF7, CR 4/5–4/8, sync 0xF1 (older
  devices use 0x12), 14 dBm, under 1 % duty cycle.
- **Header:** byte 0 is the extended-header flag, forward flag and type (6
  bits), followed by the manufacturer (1 byte) and the unique ID (2 bytes).
  An optional extended header carries ACK, unicast and signature flags.
- **Types:**
  - 1 tracking: position, altitude, speed, climb, aircraft type
  - 2 name
  - 3 message
  - 4 weather and service
  - 5 landmarks
  - 7 ground tracking
  - 9 thermal
- **Coordinates:** 24-bit values, lat = raw/93206 and lon = raw/46603.
- **Manufacturers:** 0x01 Skytraxx, 0x07 SoftRF, 0x11 FANET+, 0xE0 OGN
  tracker, 0xFB ESP32 base station.

It is all plaintext. Receive it, but never transmit into it: this is air
safety traffic.

### 3.7 OGN, ADS-L and FLARM (FSK)

- **OGN tracker:** 868.2 and 868.4 MHz, GFSK BT0.5, ±50 kHz, 100 kchip/s
  Manchester. The sync is `AA 66 55 A5 96 99 96 5A`. The packet is 26 bytes:
  20 of data and 6 of LDPC parity. It carries a 24-bit address, address type,
  relay count, position, altitude, speed, heading and climb. The whitening is
  TEA with an all-zero key, so it is readable by design.
- **ADS-L, EASA's open standard** (SRD-860 Issue 2, 2025-12-01). Its M-band
  uses the same physical layer as OGN, with Manchester-coded sync 0x724B and
  CRC-24. Its scrambling is XXTEA with a zero key, so it is readable.
  The O-band on 869.525 MHz runs at 38.4 kb/s and 200 kb/s GMSK.
- **SoftRF** already receives OGN, ADS-L and FANET+ on SX1262 hardware.
  Manchester decoding, the CRC and the descrambling are done in software. The
  board hears only one of 868.2 and 868.4 MHz at a time, so it hears roughly
  half of the alternating traffic.
- **FLARM** uses the same physical layer, but its protocol is proprietary and
  obfuscated. Count its bursts and leave their content alone.

### 3.8 Reticulum / RNode

- **Radio settings:** set by the user; there is no standard channel. The
  manual's example is 867.2 MHz, BW125, SF8. Sync is 0x12, the preamble is at
  least 18 symbols, CRC is on.
- **RNode framing:** a 1-byte header whose upper nibble is a sequence number
  and whose bit 0 marks a split packet. Split packets carry the same header
  byte in both halves, which is how the receiver pairs them.
- **Reticulum packet:** a flags byte, a hops byte, an optional transport ID,
  a 16-byte destination hash, a context byte and the data.
- **What is readable:** announces are plaintext and signed: a public key and,
  often, an LXMF display name. Everything else is encrypted.
- **IFAC:** traffic protected by an interface access code looks random without
  the passphrase.

### 3.9 Wireless M-Bus (FSK)

- **Modes:** T1 is 868.95 MHz, 100 kcps, 3-of-6 coded. C1 is 868.95 MHz,
  100 kbps NRZ. Both use sync 0x543D.
- **Receiving it on an SX1262:** Kustonium/esphome-wmbus-bridge-rawonly
  receives T1 and C1 on an SX1262. It needs the wide receive filter, because
  the chip has no FSK AFC, and a long-packet workaround for T1 frames that
  exceed 255 bytes on air. S1 on 868.3 MHz is poor on this chip.
- **What is readable:** the link header is always plaintext: manufacturer
  (three letters), meter ID, version and device type. The readings are AES
  (OMS modes 5 and 7).
- **What to show:** which meters are around and how often they send. Never
  decrypt them.

### 3.10 Balloons and satellites

- **UKHAS / PITS balloon trackers:** on 434.x MHz, typically mode 0 (SF11,
  BW20.8, CR 4/8, LDRO) or mode 2 (SF8, BW62.5), sync 0x12.
  - **Sentence:** `$$PAYLOAD,counter,time,lat,lon,alt,…*CRC16`, where the
    CRC is CCITT, polynomial 0x1021, seed 0xFFFF.
  - **Other first bytes:** SSDV image packets start with 0x66–0x69, and
    HABpack with 0x80–0x8F.
  - **Narrow modes need care:** BW20.8 needs a stable TCXO and possibly
    frequency stepping.
- **LoRa satellites (TinyGS):**
  - **On 868:** almost all are between 400 and 470 MHz, and the TinyGS FAQ
    says a band-matched board will not hear them. The 868 exceptions are
    ConnectaIoT-4 on 869.525 MHz, 6GSTARLAB on 864.97 MHz and Kosar 1.5 on
    863.4 MHz.
  - **Parameters are pushed:** TinyGS sends each station the parameters before
    a pass, as JSON over MQTT.

### 3.11 Fingerprints of the usual defaults

An unknown packet can often be named by its settings alone:

| Source | Frequency | SF / BW / CR | Sync | Preamble | Payload clue |
|---|---|---|---|---|---|
| RadioLib `begin()` default | 434.0 MHz | 9 / 125 / 4/7 | 0x12 | 8 | anything |
| arduino-LoRa default | user-set | 7 / 125 / 4/5 | 0x12 | 8 | **CRC off** |
| Heltec factory test | 868.0 MHz | 7 / 125 / 4/5 | private | 8 | `hello N,Rssi:` |
| LilyGo factory | 868.0 MHz | 10 / 125 / 4/6 | 0x12 | 15 | CRC off |
| RAK RUI3 P2P | 868.0 MHz | 7 / 125 / 4/5 | private | 10 | |
| LoRaMesher | 869.9 MHz | 7 / 125 / 4/5 | 0x14 | 8 | |

Sync 0x12 is shared by RadioLib, arduino-LoRa, LoRa APRS, RNode, UKHAS,
MeshCore and most satellites. Classify by payload, not by sync word alone.

### 3.12 Out of reach

These are listed so the screen can say "something is here" without
pretending to decode it:

- **LR-FHSS:** the SX1262 can only transmit it, so it shows up as energy at
  most.
- **Sigfox uplinks:** DBPSK at 100 Hz bandwidth is below anything the SX1262
  can receive.
- **mioty:** it is not LoRa.
- **Amazon Sidewalk:** encrypted and US-only.
- **433 MHz OOK remotes and weather stations:** the chip has no OOK receive.

---

## 4. A listening plan for Germany

With one receiver the question is always which profile to listen on next. These
are the profiles worth having, most valuable first:

| # | Profile | Frequency | BW | SF | Sync | Other | Catches |
|---|---|---|---|---|---|---|---|
| 1 | Meshtastic EU | 869.525 MHz | 250 | 11 / 9 / 8 | 0x2B | pre 16 | LongFast, MediumFast, ShortSlow |
| 2 | MeshCore EU | 869.618 MHz | 62.5 | 8 | 0x12 | pre 32 | MeshCore Narrow |
| 3 | LoRaWAN up | 868.1/.3/.5, 867.1–.9 MHz | 125 | 7–12 | 0x34 | CRC on | uplinks, joins |
| 4 | LoRaWAN RX2 | 869.525 MHz | 125 | 9 (TTN), 12 | 0x34 | IQ inverted, CRC off, pre 10 | downlinks, join-accepts |
| 5 | Class B beacon | 869.525 MHz | 125 | 9 | 0x34 | implicit 17 B, CRC off | gateway positions |
| 6 | Meshtastic Narrow | 869.442 MHz | 62.5 | 8 | 0x2B | | 2.8 EU_N_868 |
| 7 | FANET | 868.2 MHz | 250 | 7 | 0xF1 | | paragliders, ground stations |
| 8 | OGN / ADS-L | 868.2 / 868.4 MHz | GFSK 100 k | | 8-byte sync | Manchester in software | gliders, drones |
| 9 | wM-Bus | 868.95 MHz | GFSK 100 k | | 0x543D | | meters |
| 10 | LoRaWAN FSK | 868.8 MHz | GFSK 50 k | | 0xC194C1 | | DR7 |
| 11 | 433 (strong only) | 433.775 / 433.175 / 433.875 / 433.650 MHz | | | 0x12 / 0x2B | | APRS, MeshCom, Meshtastic, MeshCore |

869.525 MHz is the busiest spot in the band. Meshtastic, LoRaWAN RX2 and Class
B, the old MeshCore preset, ADS-L's O-band, the Sigfox downlink, a MeshCom
preset and one satellite all use it.

**How to schedule.**

- **Focus mode: park on one profile.** This is what a sysop wants: every
  packet of the one network they run.
- **Survey mode: rotate profiles by CAD.** CAD is blind to sync words, so it
  answers "is there LoRa at this SF and BW?" quickly. On a hit the radio
  parks in RX; on a header error it tries the other sync word next time.
  - **What the numbers allow:**
    - A LongFast preamble lasts about 165 ms, so a CAD round of SF8, SF9 and
      SF11 on 869.525 MHz (about 50 ms plus retune time) catches almost every
      LongFast start.
    - A ShortSlow preamble lasts only about 21 ms, so most ShortSlow starts
      will be missed.
  - **What survey mode reports:** statistics, not complete traffic.
- **Chase the downlink.** A LoRaWAN downlink comes at a known time after its
  uplink. After catching an uplink, the radio can switch to IQ-inverted, CRC
  off, and listen:
  - at +1 s (or +5 s on The Things Stack) on the same frequency (RX1)
  - at +2 s (+6 s) on 869.525 MHz (RX2)
  - at +5 and +6 s after a join
  
  One radio can do this because it all happens in sequence.
- **Schedule the beacon.** Class B beacons are aligned to GPS time every 128 s.
  With the clock set (UTC + 18 s leap seconds = GPS), the radio needs to sit
  on profile 5 for only a few hundred milliseconds every two minutes.

---

## 5. Tools for the people who run the networks

### For any LoRa network

- **A live packet list** with per-packet RSSI, SNR, frequency error, SF, BW,
  CR, length and time on air.
- **A packet view** showing the raw bytes and the decoded fields as a tree.
- **Time on air, from Semtech's formula** (checked):

  ```
  n   = 8*PL + (crc?16:0) - 4*SF + (implicit?0:20) + (SF>6?8:0)    (n ≥ 0)
  den = (SF>6 && LDRO) ? 4*(SF-2) : 4*SF
  sym = ceil(n/den)*(CR+4) + preamble + 12 (+2 if SF ≤ 6)
  ToA = (4*sym + 1) * 2^(SF-2) / BW
  ```

  Some examples:

  | Packet | Time on air |
  |---|---|
  | Meshtastic LongFast, 50-byte payload | 0.72 s |
  | MeshCore Narrow, 60 bytes | 0.71 s |
  | LoRaWAN SF12, 23 bytes | 1.48 s |
  | LoRaWAN SF7, 23 bytes | 62 ms |
  | LoRa APRS, 80 bytes | 3.3 s |
  | MeshCom, 80 bytes | 1.1 s |

- **Duty cycle per transmitter**, checked against the sub-band limit in
  [section 8](#8-the-law). A MeshCore repeater at its default airtime setting
  can exceed the 10 % that 869.4–869.65 MHz allows. A Meshtastic node sending
  telemetry every few minutes shows up the same way.
- **Channel utilisation**, from airtime heard plus RSSI sampling between
  packets.
- **A noise floor and a spectrum or waterfall**, from RSSI sweeps or the
  spectral-scan patch.
- **SF and DR histograms.**
- **Collision and misfit counters:**
  - A preamble with no valid header means another sync word or colliding
    packets.
  - A header followed by a CRC error means an LDRO mismatch or interference.
- **Export to Wireshark.** LoRaTap (LINKTYPE 270) pcap files over the 2 Mbaud
  serial line or WiFi. Wireshark's `lorawan` dissector reads them, and can
  decrypt with keys the user supplies. Semtech `rxpk` JSON is the other format
  worth writing.

### Meshtastic

- **A node table** built from headers and NodeInfo: name, hardware, role,
  firmware era (from hop_start and relay_node), last heard, RSSI and SNR.
- **Relay chains.** Group copies by (from, id), then list who relayed each
  copy (relay_node), in what order, and after what delay. Count redundant
  relays.
- **Hop histogram** (hop_start − hop_limit).
- **Misconfiguration flags:**
  - hop limit above 3
  - deprecated roles (REPEATER, ROUTER_CLIENT)
  - ROUTER on a node that is plainly mobile or badly sited
  - intervals below the defaults
  - NeighborInfo on the default channel
  - via_mqtt floods
  - exact positions on the default key
  - 2.8 only: a node number that is not CRC32(pubkey), or one node number
    seen with two keys
- **Link quality** from traceroutes and NeighborInfo (SNR per hop), next to
  what the board itself measures from each relayer.
- **Telemetry decoded:** battery, channel utilisation and air_util_tx as each
  node reports it.

### MeshCore

- **A repeater table** from adverts: name, role, position, the clock it
  claims, advert interval.
- **Relay paths**, straight from the path field; repeated hashes mean a loop.
- **Hash collisions.** Two repeaters with the same first key byte confuse
  1-byte paths.
- **Mixed hash sizes.** Multi-byte hashes in a mesh that still has
  pre-1.14 repeaters.
- **Trace results** with SNR per hop.
- **Duty-cycle violations**, **bad clocks** and **flood adverts more often
  than every 3 h**.

### LoRaWAN

- **Operators heard**, from DevAddr to NetID to name, with counts and SF mix.
- **Devices heard** as a DevAddr table: rate, FCnt gaps (packet loss), FCnt
  resets (rejoins), confirmed share, ADR state, airtime per day against TTN's
  fair-use 30 s.
- **Joins** with DevEUI and JoinEUI resolved to maker and join server; a
  DevEUI joining over and over is a device in trouble.
- **Downlink pairing and timing**, and MAC commands in 1.0.x FOpts:
  LinkADRReq, DevStatusAns battery and SNR, LinkCheckAns.
- **Gateways from Class B beacons**, with GPS position or NetID and gateway
  ID.
- **Sysop mode.** The user types in the session keys or AppKey of their own
  devices. The board then verifies the MIC, decrypts, decodes Cayenne LPP or a
  built-in vendor codec, and shows the full 32-bit FCnt.

### LoRa APRS and MeshCom

- **The heard list**: callsign, last position, RSSI and SNR.
- **Paths**: who digipeated, and whether it went through an iGate or server.
- **Traffic on the right channel**: 433.775 versus 433.900 MHz.
- **For MeshCom**, also:
  - duplicate message IDs, and each ACK matched to its message
  - HEY signal-report chains
  - checksum failures
  - nodes on the wrong preset or preamble

This only works if the module hears 433 MHz at all; see
[Measure first](#10-measure-first).

### FANET, OGN, ADS-L

- A neighbour table of aircraft and ground stations, and weather stations from
  FANET type 4.
- Forwarding and relay use, and the load on 868.2 MHz.

---

## 6. The SquachWatch angle: LoRa trackers that follow you

SquachWatch already warns about AirTags. LoRa has trackers too:

- LoRaWAN GPS trackers from Abeeway, Milesight, Dragino, Browan and Digital
  Matter.
- Meshtastic nodes in TRACKER role.
- FANET and OGN trackers.

A tracker hidden on a car or in a bag keeps transmitting the same identity
wherever its owner's victim goes. The identity is a DevAddr, a Meshtastic
node number or a FANET address.

This detector needs no content at all, only the header fields:

- the same identifier heard in several places, or over hours of moving
- with RSSI that stays strong
- plus a maker hint when a join is caught: the DevEUI's OUI belonging to a
  tracker vendor

It fits the existing detection model (`Detection`, alerts, the DEX) as a new
type, say `LORA_TRACKER`, alongside `LORA_NODE` for plain sightings. Keep it
local, as everything else is (see [section 8](#8-the-law)). A DevAddr can
change on rejoin, so the check has to tolerate gaps.

---

## 7. Transmitting

Sending is optional, and the rules below keep it clean:

- Send under an identity of the board's own.
- Never reuse or imitate another node's identity.
- Enforce the duty cycle in firmware.

| What | Where | Needs | Why a sysop wants it |
|---|---|---|---|
| Meshtastic NodeInfo, text, traceroute, range test | 869.525 MHz, 10 % | Its own X25519 keypair and node number (from 2.8, CRC32 of the public key), kept across reboots; the default channel key. Hop limit low, traceroutes rate-limited | a route with SNR per hop to any node, on demand |
| MeshCore advert, Public channel message, TRACE | 869.618 MHz, 10 % | Its own Ed25519 key for adverts; none for the Public channel | SNR per hop along a chosen repeater path |
| MeshCore repeater stats | 869.618 MHz | The repeater's guest or admin password, i.e. its owner's | noise floor, counters and airtime from the repeater itself |
| LoRaWAN test device | 868.1–868.5 MHz, 1 % | A device registered on the user's own TTN or Helium account (RadioLib's LoRaWAN node, class A) | LinkCheckReq returns the margin in dB and the number of gateways that heard it: a coverage meter. DeviceTimeReq returns GPS time |
| LoRa APRS, MeshCom | 433 MHz, amateur licence | A licence (DH5DAX has one), the callsign in every frame, no encryption | **Not with this module:** an 868-matched output stage should not transmit at 433 MHz. A 433 module or a second board would be needed |

Test with hop limit 0 (Meshtastic) or zero-hop (MeshCore) where that is
enough. Keep scripted beacons slower than each firmware's own minimum
intervals.

---

## 8. The law

This is not legal advice; it is what shaped the recommendations above.

**What may be listened to: § 5 TDDDG** (formerly § 89 TKG):

> Mit einer Funkanlage dürfen nur Nachrichten, die für den Betreiber der
> Funkanlage, für Funkamateure im Sinne des § 2 Nummer 1 des
> Amateurfunkgesetzes, für die Allgemeinheit oder für einen unbestimmten
> Personenkreis bestimmt sind, abgehört werden.

§ 5(2) also forbids passing on the content, or even the fact, of messages that
were received without being meant for the receiver. § 27 makes a breach of
§ 5(1) a criminal offence.

What follows for the defaults:

- **Decoded by default:**
  - Amateur traffic: LoRa APRS and MeshCom.
  - Aviation broadcasts meant for everyone: FANET, OGN and ADS-L.
  - Channels whose keys are published for anyone to use: the Meshtastic
    default key, and the MeshCore Public and hashtag channels. Reading these
    as "für einen unbestimmten Personenkreis" is this document's
    interpretation, not settled law.
- **Headers only by default:** LoRaWAN, wM-Bus, and private Meshtastic and
  MeshCore traffic. Whether header metadata counts as a "Nachricht" is a grey
  area, so keep it local and aggregated.
- **Payloads only with keys the user enters:** keys to their own devices and
  networks, entered by hand. The firmware does no key guessing of any kind.
- **Nothing goes out:** no upload, no sharing of third-party content, and
  exports only at the user's hand.
- **Positions of people** (Meshtastic, APRS) are personal data. Private use is
  outside the GDPR's scope, but publishing them is not.

**Transmitting on the amateur bands: AFuV.**

- § 11 requires the callsign at least every 10 minutes; APRS and MeshCom carry
  it in every frame.
- § 16(8) forbids encryption that obscures content, so no encrypted Meshtastic
  or MeshCore on 70 cm.
- The DARC 70 cm band plan (May 2025) names:
  - 433.175 MHz for MeshCom
  - 433.775 MHz (RX) and 433.900 MHz (TX) for LoRa APRS
  - 434.100 MHz for Meshtastic (125 kHz)

**Transmitting on 868 MHz: SRD rules.** A general licence, BNetzA Vfg.
91/2025 (November 2025, replacing 133/2019), allows transmitting on 868 MHz.
The amateur licence adds nothing there. The limits are:

| Band (MHz) | Power (ERP) | Duty cycle |
|---|---|---|
| 863–865 | 25 mW | 0.1 % |
| 865–868 | 25 mW | 1 % |
| 865.6–867.6 (four 200 kHz network channels) | 500 mW, APC | 2.5 % (10 % for network access points) |
| 868.0–868.6 | 25 mW | 1 % |
| 868.7–869.2 | 25 mW | 0.1 % |
| **869.4–869.65** | **500 mW** | **10 %** |
| 869.7–870 | 5 mW / 25 mW | none / 1 % |
| 433.05–434.79 | 1 mW / 10 mW | none / 10 % |

---

## 9. How it would fit into SquachWatch

**Build.**

- **A build flag:** `SQUACH_LORA`, set only for `env:crowpanel7`. No other
  board has the slot.
- **RadioLib 7.7.1 in `lib_deps`.** It has to be test-compiled against
  arduino-esp32 2.0.14 first.
- **Crypto:** mbedTLS in the core already covers AES, SHA-256, HMAC, CCM and
  X25519, and `meshcrypto.cpp` uses it. RadioLib brings AES-CMAC. Ed25519
  signature checks (MeshCore adverts, Meshtastic 2.8 XEdDSA) are optional and
  would need a small library.

**Modules.**

- **`lora_radio`:** owns the SX1262 on FSPI 5/4/6/8, with the IRQ on
  GPIO 20. It runs in its own task; the DIO1 interrupt only sets a flag.
- **`lora_sched`:** focus and survey modes, CAD rounds, downlink chasing,
  beacon slots.
- **`lora_pkt`:** one record per packet (time, frequency, profile, RSSI, SNR,
  frequency error, CR and CRC from the header, bytes) in a ring in internal
  RAM, because of the PSRAM twitch.
- **Decoders, one file each:** Meshtastic, MeshCore, LoRaWAN, APRS, MeshCom,
  FANET, and later the FSK ones. Each exposes the same three calls: "is this
  mine?", "decode", and "which node is it?".
- **Tables in flash:** the NetID registry subset, a LoRa-relevant OUI subset,
  and the Meshtastic preset and channel-hash table.
- **Storage:** a capture partition in the unused 7.4 MB of flash, written in
  batches.

**Screens** (at the 400×240 logical canvas):

| Screen | Shows |
|---|---|
| LORA | the live packet list, each row with a protocol colour chip |
| PACKET | raw bytes and decoded fields |
| NODES | per-network node tables |
| CHANNEL | utilisation, airtime per node against the limit, SF histogram, noise floor |
| SPECTRUM | the waterfall |

DEX cards for the protocol types, and alerts only for `LORA_TRACKER`.

**Phases.** Each one is usable on its own:

1. **Bring-up.** A `crowpanel7-loraprobe` build reads the chip version,
   decides between SD and LoRa, tries TCXO voltages, and parks on 869.525 MHz
   SF11 sync 0x2B to print Meshtastic headers over serial. This answers most
   of [Measure first](#10-measure-first).
2. **The passive sniffer.** Profiles, focus mode, the packet list, time on
   air, duty cycle, and LoRaTap export.
3. **Meshtastic and MeshCore decoding, and node tables.** For most users in
   Germany this is the payoff.
4. **LoRaWAN.** Operator and maker lookup, FCnt tracking, downlink chasing,
   Class B beacons, then sysop mode with the user's keys.
5. **FANET, then FSK** (OGN, ADS-L, wM-Bus headers) through SoftRF's approach.
6. **The spectrum screen** and the CAD survey map.
7. **Transmitting:** Meshtastic traceroute, MeshCore trace, and the LoRaWAN
   coverage tester.
8. **`LORA_TRACKER` detection.**

---

## 10. Measure first

1. **K1 on this board:** which switch is S0 and which is S1, and which way is
   ON. Check against the silkscreen and with a continuity test on the P5/P21
   pads.
2. **Board revision:** V1.2 (I2S mic, BUSY through the mux) or V1.3+ (PDM
   mic, BUSY direct).
3. **The module:** is it really an SX1262 (version string)? Is it matched for
   868 or 915?
4. **TCXO voltage:** does `begin()` succeed at 1.6, 1.8 and 3.3 V? What
   frequency offset does it show against a known transmitter?
5. **Off-band receive:** how many dB are lost at 433 MHz?
6. **Sync words:** which of 0x12, 0x34, 0x2B and 0xF1 leak through a filter
   set to another?
7. **CAD tuning:** detPeak and detMin against false alarms, and the real
   retune-plus-CAD time per profile.
8. **Spectral scan:** does the patch take on this chip, and how fast does it
   sweep?
9. **Power:** how far does the 3.3 V rail droop at +22 dBm (about 120 mA)
   with the panel running?
10. **The interrupt on GPIO 20:** what is its latency, and does it lose packets
    while the RGB panel refreshes?
11. **The build:** does RadioLib 7.7.1 build and run on arduino-esp32 2.0.14?

---

## Sources

**Hardware**

- Elecrow wireless module: https://www.elecrow.com/wireless-module-for-crowpanel-advanced-series.html
- CrowPanel Advance 7.0 repo (examples lesson-07, `Eagle_SCH&PCB/version1.5/readme.md`): https://github.com/Elecrow-RD/CrowPanel-Advance-7-HMI-ESP32-S3-AI-Powered-IPS-Touch-Screen-800x480
- Elecrow wiki, 7.0 Advance: https://www.elecrow.com/pub/wiki/ESP32_Display-7.0_inch(Advance_Series)wiki.html
- Elecrow forum, LoRa and TF card: https://forum.elecrow.com/discussion/26219
- Meshtastic `elecrow_panel` variant: https://github.com/meshtastic/firmware/blob/master/variants/esp32s3/elecrow_panel/variant.h
- Meshtastic issue #11393: https://github.com/meshtastic/firmware/issues/11393
- SX1261/2 datasheet Rev 2.2: https://www.semtech.com/products/wireless-rf/lora-connect/sx1262
- RadioLib: https://github.com/jgromes/RadioLib (discussion #1135 on sync words)

**Meshtastic and MeshCore**

- Meshtastic firmware (`RadioInterface.*`, `MeshRadio.h`, `Channels.cpp`, `CryptoEngine.cpp`, `FloodingRouter.cpp`, `Default.h`): https://github.com/meshtastic/firmware
- Meshtastic protobufs: https://github.com/meshtastic/protobufs
- https://meshtastic.org/docs/overview/mesh-algo/ , https://meshtastic.org/docs/overview/encryption/
- MeshCore (`docs/packet_format.md`, `docs/payloads.md`, `src/Packet.h`, `src/Mesh.cpp`, `src/Utils.cpp`): https://github.com/meshcore-dev/MeshCore
- German preset moves: https://meshdresden.eu/meshtastic-preset-umstellungen-in-deutschland/ , https://www.meshrheinland.de/meshtastic/grundeinstellungen
- Decoders: https://github.com/michaelhart/meshcore-decoder , https://github.com/rightup/pyMC_core , https://github.com/ErikDorstel/meshShark

**LoRaWAN**

- RP002-1.0.5: https://resources.lora-alliance.org/technical-specifications/rp002-1-0-5-lorawan-regional-parameters
- NetID allocation R111: https://lora-alliance.org/wp-content/uploads/2025/09/LoRa-Alliance-NetID-Allocation_R111.pdf
- TTN: https://www.thethingsnetwork.org/docs/lorawan/frequency-plans/ , https://www.thethingsnetwork.org/docs/lorawan/prefix-assignments/ , https://www.thethingsnetwork.org/docs/lorawan/duty-cycle/
- The Things Stack MAC settings (Rx1Delay 5 s): https://www.thethingsindustries.com/docs/hardware/devices/configuring-devices/mac-settings/
- Helium: https://docs.helium.com/iot/run-an-lns/buy-an-oui , https://github.com/helium/HIP (HIP-149)
- lora-packet (test vector): https://github.com/anthonykirby/lora-packet
- ChirpStack `lrwn`: https://github.com/chirpstack/chirpstack
- Device codecs: https://github.com/TheThingsNetwork/lorawan-devices
- LoRaTap: https://github.com/eriknl/LoRaTap

**Amateur radio and others**

- LoRa APRS iGate / tracker: https://github.com/richonguzman/LoRa_APRS_iGate
- MeshCom: https://github.com/icssw-org/MeshCom-Firmware
- FANET: https://github.com/3s1d/fanet-stm32/blob/master/Src/fanet/radio/protocol.txt
- OGN tracking protocol: http://wiki.glidernet.org/ogn-tracking-protocol
- ADS-L Issue 2: https://www.easa.europa.eu/en/document-library/agency-decisions/ed-decision-2022024r
- SoftRF: https://github.com/lyusupov/SoftRF
- TinyGS: https://github.com/tinygs/tinyGS/wiki
- PITS LoRa gateway: https://github.com/PiInTheSky/lora-gateway
- RNode / Reticulum: https://github.com/markqvist/RNode_Firmware , https://github.com/markqvist/Reticulum
- wM-Bus on SX1262: https://github.com/Kustonium/esphome-wmbus-bridge-rawonly , https://github.com/wmbusmeters/wmbusmeters
- Spectrum: https://github.com/Genaker/LoraSA

**Law**

- § 5 TDDDG: https://dejure.org/gesetze/TDDDG/5.html , § 27: https://dejure.org/gesetze/TDDDG/27.html
- AFuV: https://www.gesetze-im-internet.de/afuv_2005/
- BNetzA Vfg. 91/2025: https://www.bundesnetzagentur.de/DE/Fachthemen/Telekommunikation/Frequenzen/Allgemeinzuteilungen/_DL/vfg91_2025.pdf
- DARC 70 cm band plan (May 2025): https://www.darc.de/fileadmin/filemounts/referate/vus/bandplaene/UHF_Bandplan_70_cm_Mai_2025.pdf
