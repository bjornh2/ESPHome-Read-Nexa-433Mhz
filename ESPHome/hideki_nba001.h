#pragma once
/**
 * hideki_nba001.h
 * ---------------
 * Decoder for the Nexa NBA-001 temperature/humidity sensor (and identical
 * Hideki TS04 / TFA TS34C hardware).
 *
 * RF parameters
 *   Frequency  : 433.92 MHz
 *   Modulation : OOK
 *   Encoding   : OOK_PULSE_DMC (Differential Manchester)
 *     short pulse/gap ≈ 520 µs  (half bit period → encodes "1" when paired)
 *     long  pulse/gap ≈ 1040 µs (full bit period → encodes "0")
 *
 * Packet structure (10 bytes raw, 9 bytes after removing the sync byte)
 *   After receiving, the pipeline is:
 *     1. DMC → raw bit stream
 *     2. Find sync byte 0x0D in raw stream
 *     3. Unstuff parity bits (every 9th bit per byte)
 *     4. Invert each byte  (^0xFF)
 *     5. XOR check  (XOR of bytes 0–7 must be 0)
 *     6. CRC-8 check (poly 0x07, init 0x00 over all 9 bytes must be 0)
 *     7. Reflect each byte (bit-reverse, LSB↔MSB)
 *     8. Extract:
 *          packet[0]  = channel (bits[7:5]) + rolling-code/RC (bits[3:0])
 *          packet[1]  = frame length
 *          packet[2]  = sequence number / type
 *          packet[3]  = temperature ones + tenths  (BCD nibbles)
 *          packet[4]  = temperature hundreds (BCD) + sign bit[7] + battery bit[6]
 *          packet[5]  = humidity tens + ones (BCD nibbles)
 *          packet[6]  = humidity check
 *          packet[7]  = XOR check byte
 *          packet[8]  = CRC-8 byte
 *
 * Sensor ID (RFXcom convention)  sensor_id = (packet[0] << 8) | 0x0E
 *   This matches what RFXcom/Home Assistant uses for the TH7-Cresta sub-type.
 *   NOTE: The rolling-code changes on every battery replacement, so after
 *         swapping batteries you must update the sensor_X_id substitution.
 *
 * Protocol reference: https://github.com/merbanan/rtl_433/blob/master/src/devices/hideki.c
 * Verified against captured Nexa NBA-001 data from rtl_433 issue #803.
 */

#include "esphome.h"
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <map>
#include <vector>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// Low-level helpers (matching rtl_433/hideki.c names/behaviour)
// ─────────────────────────────────────────────────────────────────────────────

static inline uint8_t nba_reflect8(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

static inline uint8_t nba_parity8(uint8_t b)
{
    b ^= b >> 4;
    b ^= b >> 2;
    b ^= b >> 1;
    return b & 1;
}

/** CRC-8, polynomial 0x07, initial value 0x00.
 *  Returns 0x00 when a valid message including CRC byte is fed. */
static uint8_t nba_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 1: Convert DMC pulse stream → raw bit array
//   Two consecutive short (≈520 µs) durations = bit "1"
//   One long (≈1040 µs) duration               = bit "0"
// ─────────────────────────────────────────────────────────────────────────────
static void nba_dmc_to_bits(const std::vector<int32_t> &raw, std::vector<uint8_t> &bits)
{
    const int32_t HALF_US = 520;
    const int32_t TOL     = 230;  // ±230 µs tolerance

    bool pending_short = false;

    for (size_t i = 0; i < raw.size(); i++) {
        int32_t d = raw[i] < 0 ? -raw[i] : raw[i];  // absolute duration

        bool is_short = (d >= HALF_US - TOL && d <= HALF_US + TOL);
        bool is_long  = (d >= 2 * HALF_US - TOL && d <= 2 * HALF_US + TOL);

        if (is_short) {
            if (pending_short) {
                bits.push_back(1);   // two consecutive shorts → bit "1"
                pending_short = false;
            } else {
                pending_short = true;
            }
        } else if (is_long) {
            if (pending_short) {
                // short + long: emit 0 and reset (clock jitter / first short was an edge)
                bits.push_back(0);
                pending_short = false;
            } else {
                bits.push_back(0);   // single long → bit "0"
            }
        } else {
            // Duration outside expected range: treat as idle / reset
            pending_short = false;
        }
    }
    // If the stream ends with an unmatched short (idle timeout fires before the
    // trailing space is recorded) emit a '1' bit – the matching half is implied.
    if (pending_short)
        bits.push_back(1);
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 2: Locate the Hideki sync byte (0x0D) in the raw bit stream, unstuff
//         parity bits, validate XOR + CRC, reflect bytes, extract sensor data.
// ─────────────────────────────────────────────────────────────────────────────

struct NexaPacket {
    bool     valid        = false;
    uint16_t sensor_id    = 0;      // (packet[0] << 8) | 0x0E  (RFXcom convention)
    int      channel      = 0;      // physical channel (1-based)
    int      rc           = 0;      // 4-bit rolling code (changes on battery swap)
    float    temperature  = 0.0f;   // °C, one decimal
    int      humidity     = 0;      // % RH, integer
    bool     battery_ok   = true;
};

/**
 * Try to decode one Nexa NBA-001 / Hideki TS04 packet from @p bits,
 * trying every alignment starting at @p start_offset.
 */
static NexaPacket nba_decode_from_bits(const std::vector<uint8_t> &bits)
{
    NexaPacket result;

    // Need at least: 10 stuffed bytes (9 bits each) = 90 bits after sync
    // plus we try multiple alignments, so require ≥ 94 bits total.
    if (bits.size() < 88) {
        ESP_LOGD("nba001", "  decode: only %d bits, need ≥88 – skip", (int)bits.size());
        return result;
    }

    // Scan for sync byte 0x0D = 0b00001101 in the raw (pre-inversion) bit stream.
    // The preamble ends with a stray bit that shifts the stream by 1, so we scan
    // all positions p from 0 upward.  Allow 1 bit of slack (+1) so that p=1 is
    // reachable even when bits.size() == 90 exactly.

    // Need p + 8 (sync) + 9*9 (9 data bytes of 9 bits each, but we read 8+skip1
    // per byte so last bit is at p+8+8*9+7 = p+87).
    for (size_t p = 0; p + 88 <= bits.size(); p++) {
        // Read 8 bits to check for sync
        uint8_t s = 0;
        for (int i = 0; i < 8; i++)
            s = (s << 1) | bits[p + i];
        if (s != 0x0D)
            continue;

        // The sync word is exactly 8 bits (no parity bit follows it in the
        // protocol – rtl_433 confirms startpos = 9 - i when sync found at i=0).
        // Data bytes start immediately at bit p+8.

        // ── Unstuff 9 data bytes (each group of 9 bits: 8 data + 1 parity) ──
        // If the last bit(s) are beyond the buffer (stream truncated by idle
        // timeout), try both 0 and 1 for the missing bit via two decode passes.
        const int  NBYTES     = 9;
        size_t     data_start = p + 8;  // sync word is 8 bits, no trailing parity bit

        // Determine how many bits are OOB so we know how many passes to try.
        size_t last_data_idx = data_start + (size_t)(NBYTES - 1) * 9 + 7;
        int    tries         = (last_data_idx < bits.size()) ? 1 : 2;

        NexaPacket trial_result;
        for (int fill = 0; fill < tries; fill++) {
            uint8_t    packet[9]  = {};

            for (int i = 0; i < NBYTES; i++) {
                uint8_t byte = 0;
                for (int b = 0; b < 8; b++) {
                    size_t idx = data_start + (size_t)i * 9 + (size_t)b;
                    // OOB: use fill value (0 first pass, 1 second pass)
                    byte = (byte << 1) | (idx < bits.size() ? bits[idx] : (uint8_t)fill);
                }

                // Invert byte (protocol transmits inverted bits).
                // Parity check omitted: parity bit boundaries are ambiguous when
                // sync lands off a 9-bit group boundary.  XOR + CRC-8 together
                // give 1-in-65536 false-positive rate, which is sufficient.
                packet[i] = byte ^ 0xFF;
            }

            // ── XOR check: XOR of bytes 0–7 must equal 0 ──
            uint8_t xr = 0;
            for (int i = 0; i < 8; i++) xr ^= packet[i];
            if (xr != 0) {
                ESP_LOGD("nba001", "  @%d fill=%d: XOR fail (xr=0x%02X)", (int)p, fill, xr);
                continue;
            }

            // ── CRC-8 check: poly=0x07 init=0x00 over all 9 bytes == 0 ──
            if (nba_crc8(packet, NBYTES) != 0) {
                ESP_LOGD("nba001", "  @%d fill=%d: CRC fail", (int)p, fill);
                continue;
            }

            // ── Reflect bytes ──
            for (int i = 0; i < NBYTES; i++)
                packet[i] = nba_reflect8(packet[i]);

            // ── Validate packet length field ──
            int pkt_len = (packet[1] >> 1) & 0x1F;
            if (pkt_len + 2 != NBYTES) {
                ESP_LOGD("nba001", "  @%d fill=%d: length fail (pkt_len=%d)", (int)p, fill, pkt_len);
                continue;
            }

            // ── Extract sensor data ──
            int channel = (packet[0] >> 5) & 0x07;
            if (channel >= 5) channel -= 1;
            int rc = packet[0] & 0x0F;

            int raw_temp = (packet[4] & 0x0F) * 100
                         + ((packet[3] & 0xF0) >> 4) * 10
                         + (packet[3] & 0x0F);
            bool positive   = ((packet[4] >> 7) & 1) != 0;
            bool battery_ok = ((packet[4] >> 6) & 1) != 0;
            int  humidity   = ((packet[5] & 0xF0) >> 4) * 10 + (packet[5] & 0x0F);

            trial_result.valid       = true;
            trial_result.sensor_id   = ((uint16_t)packet[0] << 8) | 0x0E;
            trial_result.channel     = channel + 1;
            trial_result.rc          = rc;
            trial_result.temperature = (positive ? 1.0f : -1.0f) * raw_temp * 0.1f;
            trial_result.humidity    = humidity;
            trial_result.battery_ok  = battery_ok;
            break;  // Valid decode found
        }  // end fill loop

        if (trial_result.valid) {
            result = trial_result;
            break;  // Found a valid packet at this p; stop scanning
        }
    }

    if (!result.valid)
        ESP_LOGD("nba001", "  no valid packet found in %d bits", (int)bits.size());

    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Public entry point – called from ESPHome remote_receiver on_raw lambda
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Decode a Nexa NBA-001 packet from ESPHome remote_receiver raw pulse data.
 *
 * Usage in YAML on_raw lambda:
 *   auto pkt = nba001_decode(x);
 *   if (pkt.valid) { ... pkt.sensor_id ... pkt.temperature ... }
 */
static NexaPacket nba001_decode(const std::vector<int32_t> &raw)
{
    // Static buffer: avoids a heap alloc/free on every on_raw callback
    // (which fires for all 433 MHz RF noise, not just Nexa packets).
    // Safe because ESPHome runs single-threaded in the main loop.
    static std::vector<uint8_t> bits;
    bits.clear();
    bits.reserve(300);
    nba_dmc_to_bits(raw, bits);
    return nba_decode_from_bits(bits);
}

/**
 * Build a compact JSON string for a decoded packet.
 * e.g.  {"id":"0x340E","ch":1,"t":25.5,"h":48,"batt":true}
 */
static std::string nba001_to_json(const NexaPacket &pkt)
{
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"id\":\"0x%04X\",\"ch\":%d,\"rc\":%d,\"t\":%.1f,\"h\":%d,\"batt\":%s}",
             pkt.sensor_id,
             pkt.channel,
             pkt.rc,
             pkt.temperature,
             pkt.humidity,
             pkt.battery_ok ? "true" : "false");
    return std::string(buf);
}

// ─────────────────────────────────────────────────────────────────────────────
// Nexa FS-558 RF smoke alarm decoder
//
// RF parameters
//   Frequency  : 433.92 MHz
//   Modulation : OOK
//   Encoding   : simple pulse-width, one pulse+gap pair per bit
//     Long pulse  (≈1190 µs) + Short gap (≈490 µs)  = bit "1"
//     Short pulse (≈ 430 µs) + Long  gap (≈1270 µs)  = bit "0"
//
// Packet structure (24 data bits + end marker)
//   bits[23:1]  23-bit device address, unique per unit, never changes
//   bit[0]      state bit (0 = observed when test-button pressed;
//               verify against real alarm to confirm polarity)
//   End marker  2 × (short pulse + short gap) + trailing pulse
//
// NOTE: The FS-558 does NOT use ESPHome’s built-in Nexa decoder.
//       Its timing (T≈430 µs) differs from the Nexa decoder’s expected
//       sync gap (>2000 µs), so on_nexa will never fire for this device.
//       Use on_raw + fs558_decode() instead.
// ─────────────────────────────────────────────────────────────────────────────

struct Fs558Packet {
    bool     valid    = false;
    uint32_t device   = 0;     // 23-bit device address (bits[23:1])
    bool     state    = false; // bit[0] of the packet
};

/**
 * Decode one Nexa FS-558 packet from ESPHome remote_receiver raw pulse data.
 *
 * The packet-size check (49–70 values) prevents NBA-001 packets (~130+
 * values) from being fed into this decoder.
 */
static Fs558Packet fs558_decode(const std::vector<int32_t> &raw)
{
    Fs558Packet result;

    // FS-558 packets: 24 data pairs + 2 end-marker pairs + 1 trailing pulse
    // = 49–53 values.  NBA-001 packets are 130+ values—skip them.
    if (raw.size() < 49 || raw.size() > 70) return result;

    const int32_t SHORT_US = 430;
    const int32_t LONG_US  = 1190;
    const int32_t TOL      = 180;

    auto is_short = [&](int32_t v) -> bool {
        return (v >= SHORT_US - TOL && v <= SHORT_US + TOL);
    };
    auto is_long = [&](int32_t v) -> bool {
        return (v >= LONG_US - TOL && v <= LONG_US + TOL);
    };

    uint32_t bits = 0;
    for (int i = 0; i < 24; i++) {
        int32_t pulse = raw[i * 2];
        int32_t gap   = raw[i * 2 + 1] < 0 ? -raw[i * 2 + 1] : raw[i * 2 + 1];

        if (is_long(pulse) && is_short(gap)) {
            bits = (bits << 1) | 1u;
        } else if (is_short(pulse) && is_long(gap)) {
            bits = (bits << 1) | 0u;
        } else {
            return result;   // symbol out of tolerance → not FS-558
        }
    }

    // Verify end marker: pair at index 24 (values 48,49) must be short+short
    if ((int)raw.size() < 50) return result;
    int32_t em_pulse = raw[48];
    int32_t em_gap   = raw[49] < 0 ? -raw[49] : raw[49];
    if (!is_short(em_pulse) || !is_short(em_gap)) return result;

    result.valid  = true;
    result.device = bits >> 1;          // upper 23 bits = device address
    result.state  = (bits & 1u) != 0;  // bit[0] = state
    return result;
}

/**
 * Build a compact JSON string for a decoded FS-558 packet.
 * e.g.  {"id":"0x680005","state":false}
 */
static std::string fs558_to_json(const Fs558Packet &pkt)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "{\"id\":\"0x%06X\",\"state\":%s}",
             (unsigned)pkt.device, pkt.state ? "true" : "false");
    return std::string(buf);
}
