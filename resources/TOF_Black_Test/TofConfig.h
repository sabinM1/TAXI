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
//   black @ 200 mm       : valid = 100%  -> perfect detection
//   black @ 300 mm       : valid = 96%   -> solid
//
// Edit the values below, then send 'A' in TOF_Black_Test to push them to
// all five sensors (write + verify + save + restart, one sensor at a time).
// ============================================================================

// Sampling time (reg 0xB0), milliseconds. Longer = more photons collected on a
// black target, at the cost of a staler reading. 50 was the best detection /
// latency trade at the 300 mm engage distance. Must stay below the firmware's
// TOF_TIMEOUT_MS (55 in TAXI_Firmware/Config.h).
#define TOF_CFG_B0  50

// Range checks (reg 0xB4). 0xFE = 254 enables ALL checks, including the
// wrap/phase pair (bits 1-2) that the factory default 0xF8 = 248 leaves off.
// Those two checks are what removed the large-distance phantom readings on
// empty space: 23 false positives at 248 dropped to 1 harmless one at 254.
#define TOF_CFG_B4  254

// Signal threshold (reg 0xB5). The black return is weak (sigAvg ~4 at 300 mm),
// so this must stay LOW or the target itself is rejected. 30 keeps black
// detection while trimming the weakest phantoms; higher values kill black.
// NOTE: this is NOT compared against register 0x02 (a row can show sigMax below
// B5 and still pass) — trust the valid bit.
#define TOF_CFG_B5  30

// Sigma threshold (reg 0xB6), millimetres. Must be high: the black return is
// noisy (sigma ~220-330 mm), so a low value rejects the target itself.
#define TOF_CFG_B6  300

// Max distance (reg 0xB8), millimetres. The empty-space phantoms are LONG
// readings (600-800 mm), so this is the surgical fix: anything past 500 mm is
// rejected by the distance-max check without touching the signal threshold.
// Used because bit 7 of B4 (0xFE = 254) enables the distance-max check.
#define TOF_CFG_B8  500

// Crosstalk correction (reg 0xB2). Left at 0: the sweep showed B2 does NOT
// change the signal the sensor sees (sigAvg flat at 4-6 across 0..200), it only
// tightens acceptance until black falls out too. No usable value exists.
#define TOF_CFG_B2  0
