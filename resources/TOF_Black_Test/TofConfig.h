#pragma once
// ============================================================================
// TofConfig.h — the winning M60F ToF configuration for the TAXI mini-sumo.
//
// Found with TOF_Black_Test (sensor 3, MID) against a MATTE BLACK opponent:
//   * grid sweep of B0 x B5 x B6 at 300..500 mm
//   * empty-space noise sweep of B4 x B5
//
// Verified with these values:
//   empty space (>1.5 m) : valid = 0     -> no false positives
//   black @ 300 mm       : valid = 100%  -> perfect detection
//   black @ 400 mm       : valid = 95%   -> solid
//
// Edit the four values below, then send 'A' in TOF_Black_Test to push them to
// all five sensors (write + verify + save + restart, one sensor at a time).
// ============================================================================

// Sampling time (reg 0xB0), milliseconds. Longer = more photons collected on a
// black target, at the cost of a staler reading. 45 was the best detection /
// latency trade at the 300 mm engage distance. Must stay below the firmware's
// TOF_TIMEOUT_MS (55 in TAXI_Firmware/Config.h).
#define TOF_CFG_B0  45

// Range checks (reg 0xB4). 0xFE = 254 enables ALL checks, including the
// wrap/phase pair (bits 1-2) that the factory default 0xF8 = 248 leaves off.
// Those two checks are what removed the large-distance phantom readings on
// empty space: 23 false positives at 248 dropped to 1 harmless one at 254.
#define TOF_CFG_B4  254

// Signal threshold (reg 0xB5). Lowest value that stayed clean on empty space
// once B4 = 254 was in force. NOTE: this is NOT compared against register 0x02
// (a row can show sigMax below B5 and still pass) — trust the valid bit.
#define TOF_CFG_B5  30

// Sigma threshold (reg 0xB6), millimetres. Must be high: the black return is
// noisy (sigma ~220-330 mm), so a low value rejects the target itself.
#define TOF_CFG_B6  300
