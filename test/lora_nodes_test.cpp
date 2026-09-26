// The node table -- src/lora_nodes.cpp -- fed constructed frames from three
// networks, and read back the way the NODES view reads it: one row per
// transmitter, the newest first, with the flags a sysop would want.
#include "lora_nodes.h"
#include "lora_meshtastic.h"
#include "lora_classify.h"
#include "lora_crypto.h"
#include "test_util.h"
#include <cstring>
#include <cstdio>

using namespace Lora;

static Packet pk;

static void mtFrame(uint32_t from, uint8_t flags, const uint8_t* data, uint8_t dlen, uint32_t id) {
    memset(&pk, 0, sizeof pk);
    pk.sync = 0x2B; pk.sf = 11; pk.bwKhz10 = 2500; pk.freqHz = 869525000; pk.cr = 5;
    pk.flags = PK_CRC_PRESENT | PK_CRC_OK;
    LoraCrypto::wr32le(pk.data, Meshtastic::BROADCAST); LoraCrypto::wr32le(pk.data + 4, from); LoraCrypto::wr32le(pk.data + 8, id);
    pk.data[12] = flags; pk.data[13] = 0x08; pk.data[14] = 0; pk.data[15] = (uint8_t)from;
    memcpy(pk.data + 16, data, dlen);
    pk.len = (uint8_t)(16 + dlen);
    Meshtastic::Header h; Meshtastic::parseHeader(pk.data, pk.len, h);
    Meshtastic::Channel c = Meshtastic::channel(0);
    Meshtastic::decrypt(h, c, pk.data + 16, dlen);
    pk.toaUs = timeOnAirUs(11, 2500, 5, 16, pk.len, true, false, false);
    classify(pk);
}

int main() {
    Nodes::clear();

    suite("A Meshtastic node, named by its NodeInfo");
    {
        const uint8_t text[] = { 0x08, 0x01, 0x12, 0x02, 'h', 'i' };
        mtFrame(0x3b1c9a2e, 0x63, text, sizeof text, 1);
        ck("classified as Meshtastic", pk.proto == Proto::MESHTASTIC);
        Nodes::note(pk, 1000, 0);
        ck("one node", Nodes::count() == 1);
        const Nodes::Node* n = Nodes::at(0);
        ck("tag is the node id", n && strcmp(n->tag, "!3b1c9a2e") == 0);
        ck("no flags on a default-hop text", n && n->flags == 0 && n->hops == 0);
        const uint8_t user[] = { 0x0A, 9, '!', '3','b','1','c','9','a','2','e', 0x12, 3, 'B','o','b', 0x1A, 3, 'B','O','B', 0x38, 4 };
        uint8_t data[40] = { 0x08, 0x04, 0x12, (uint8_t)sizeof user };
        memcpy(data + 4, user, sizeof user);
        mtFrame(0x3b1c9a2e, 0xE7, data, (uint8_t)(4 + sizeof user), 2);   // hop_start 7, hop_limit 7: too many
        Nodes::note(pk, 2000, 0);
        n = Nodes::at(0);
        ck("still one node", Nodes::count() == 1 && n->packets == 2);
        ck("named Bob, tagged BOB", strcmp(n->name, "Bob") == 0 && strcmp(n->tag, "BOB") == 0);
        ck("REPEATER is a deprecated role", strcmp(Nodes::roleText(*n), "REPEATER") == 0 && (n->flags & Nodes::NF_DEPRECATED));
        ck("a hop limit of 7 is flagged", n->flags & Nodes::NF_HOPS_HIGH);
        char f[48]; Nodes::flagsText(*n, f, sizeof f);
        ck("flags text", strcmp(f, "hops oldrole") == 0);
        ck("airtime counted", n->airtimeMs > 0);
    }

    suite("A LoRaWAN device counts its frames");
    {
        auto wan = [](uint16_t fcnt) {
            memset(&pk, 0, sizeof pk);
            pk.sync = 0x34; pk.sf = 7; pk.bwKhz10 = 1250; pk.freqHz = 868100000; pk.cr = 5;
            pk.flags = PK_CRC_PRESENT | PK_CRC_OK;
            const uint8_t d[] = { 0x40, 0x2C, 0x1B, 0x01, 0x26, 0x80, (uint8_t)fcnt, (uint8_t)(fcnt >> 8), 0x01, 0xAA, 0xBB, 0x11, 0x22, 0x33, 0x44 };
            memcpy(pk.data, d, sizeof d); pk.len = sizeof d;
            pk.toaUs = 60000;
            classify(pk);
        };
        wan(10); ck("classified as LoRaWAN", pk.proto == Proto::LORAWAN); Nodes::note(pk, 3000, 0);
        wan(11); Nodes::note(pk, 4000, 0);
        wan(14); Nodes::note(pk, 5000, 0);
        ck("two nodes now", Nodes::count() == 2);
        const Nodes::Node* n = Nodes::at(1);
        ck("tagged by DevAddr, named by operator", strcmp(n->tag, "26011b2c") == 0 && strcmp(n->name, "The Things Network") == 0);
        ck("two frames lost between 11 and 14", n->lost == 2 && n->counter == 14);
        wan(3); Nodes::note(pk, 6000, 0);
        ck("a counter going backwards is a reset", Nodes::at(1)->flags & Nodes::NF_RESETS);
    }

    suite("Order and duty");
    {
        uint8_t idx[8];
        ck("two in order", Nodes::order(idx, 8) == 2);
        ck("the LoRaWAN device, heard last, is first", Nodes::at(idx[0])->proto == Proto::LORAWAN);
        const Nodes::Node* n = Nodes::at(0);
        // 2 frames of about 0.3 s each in the first 2 s: well over 10 % of that window
        ck("duty per mille is airtime over the window", Nodes::dutyPermille(*n, 3000) > 100);
        Nodes::clear();
        ck("cleared", Nodes::count() == 0);
    }

    return report();
}
