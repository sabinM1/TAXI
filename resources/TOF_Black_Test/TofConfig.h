#pragma once
// ============================================================================
// TofConfig.h — the winning M60F ToF configuration for the TAXI mini-sumo.
//
// Found with TOF_Black_Test (sensor 3, MID) against a MATTE BLACK opponent:
//   * grid sweep of B0 x B5 x B6 at 300..500 mm
//   * empty-space noise sweep of B4 x B5
//   * stability sweep ('Z') of B4 x B5 x B6, which counts distinct
//     MEASUREMENTS and reports the phantom events surviving the firmware's
//     consecutive-valid filter (ev3)
//
// Verified with these values:
//   empty space (>1.5 m) : valid = 0     -> no false positives
//   black @ 200 mm       : valid = 100%  -> perfect detection
//   black @ 300 mm       : valid = 96%   -> solid
//   empty space, 'Z'     : ev3 = 0       -> nothing survives the N=3 filter
//
// Edit the values below, then send 'A' in TOF_Black_Test to push them to
// all five sensors (write + verify + save + restart, one sensor at a time).
// ============================================================================

// Sampling time (reg 0xB0), milliseconds. Longer = more photons collected on a
// black target, at the cost of a staler reading. 50 was the best detection /
// latency trade at the 300 mm engage distance. Must stay below the firmware's
// TOF_TIMEOUT_MS (55 in TAXI_Firmware/Config.h).
#define TOF_CFG_B0  50

// Range checks (reg 0xB4). 0xFE = 254 turns EVERY check on, including the
// wrap/phase pair (bits 1-2) and the distance-max check (bit 7).
//
// This is the value the black target was actually verified against (100% at
// 200 mm, 96% at 300 mm). An earlier session dropped it to 248 on the strength
// of a sensor-2 empty-space sweep, but that disabled the wrap/phase checks and
// was never re-verified against black. The 'Z' sweep now shows 254 is also the
// cleaner of the two on empty space, so there is no reason to keep 248.
#define TOF_CFG_B4  254

// Signal threshold (reg 0xB5). The black return is weak (sigAvg ~4 at 300 mm),
// so this must stay LOW or the target itself is rejected — but not so low that
// ambient noise passes.
//
// 30 is where the four empty-space runs put the floor. Across 4 runs of ~26
// measurements each, 254/30/200 let through ONE phantom measurement total —
// exactly the same as 254/40/300. Raising B5 to 40 therefore buys no extra
// rejection while costing 6% of black detection (95.6% -> 89.6%), so 30 is the
// better trade. The lone phantom is a single measurement, so the firmware's
// N=3 filter still drops it.
// NOTE: this is NOT compared against register 0x02 (a row can show sigMax below
// B5 and still pass) — trust the valid bit.
#define TOF_CFG_B5  30

// Sigma threshold (reg 0xB6), millimetres. Lower = stricter.
//
// 200 pairs with B5=30 for the cleanest empty-space result of any config:
// 254/30/200 let through one phantom measurement across four runs, and it
// keeps the black target at 95.6% valid (min 91% over seven runs). B6=300 at
// the same B5 leaks 4 phantoms; B6=100 is too strict and the black target
// itself flickers (64-80%).
#define TOF_CFG_B6  200

// Max distance (reg 0xB8), millimetres. The empty-space phantoms are LONG
// readings (600-800 mm), so this is the surgical fix: anything past 500 mm is
// rejected by the distance-max check without touching the signal threshold.
// Used because bit 7 of B4 (0xFE = 254) enables the distance-max check.
#define TOF_CFG_B8  500

// Crosstalk correction (reg 0xB2). Left at 0: the sweep showed B2 does NOT
// change the signal the sensor sees (sigAvg flat at 4-6 across 0..200), it only
// tightens acceptance until black falls out too. No usable value exists.
#define TOF_CFG_B2  0
