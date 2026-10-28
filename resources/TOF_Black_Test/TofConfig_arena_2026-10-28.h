#pragma once
// ============================================================================
// TofConfig_arena_2026-10-28.h — TEMPORARY arena-session config (NOT the
// shipping TofConfig.h). Copy over TofConfig.h to use it, or keep as a record.
//
// Derived from the 2026-10-28 arena session: 'Z' stability sweeps of
// B4 x B5 x B6 on ALL FIVE sensors (6 reps on sensor 4, 1 rep each on 1/2/3/5),
// plus the B4 x B5 noise sweep and the B7 / B0 / B1-B2 sweeps on sensor 4.
//
// NO BLACK TARGET was available this session. The detection side is therefore
// UNVERIFIED — only the phantom (false-positive) side was measured. The B5=20
// choice is validated against empty-space phantoms only.
//
// What the data showed:
//   * B4=254, B5=20, B6=200 -> ev3 = 0 on ALL FIVE sensors. This is the only
//     combo in the whole sweep with zero predicted phantom events everywhere.
//   * B4=248 is measurably worse than 254 at low B5 on every sensor.
//   * B5=0 leaks ev3 = 2..4 on sensors 1, 2 and 4 -> not viable.
//   * B6 (100/200/300) barely matters at B5=20; all give ev3=0.
//   * The empty-space phantom band sits at 375-399 mm (the far wall). B8=500
//     rejects that band outright; B8=700 lets it through the distance-max
//     check. 500 mm is still far past any sumo engagement distance.
//
// KNOWN UNRESOLVED ISSUE:
//   Sensor 1 has a REAL ~200 mm reflector in its cone (dMean=194, longest=21,
//   tight 150-250 mm cluster — not the 375-399 mm wall band the others show).
//   It is a full-authority side sensor (TOF_WEIGHT[0]=+4), so this is a
//   hardware problem, not a register problem. Find and remove/blank the
//   reflector, then re-run 'Z empty 500 6 1'.
//
// TO APPLY: send 'A' in TOF_Black_Test to push these to all five sensors
// (write + verify + save + restart, one sensor at a time), then 'g 1'..'g 5'.
//
// NOTE: 'A' does NOT write B7. Sensor 3 was at B7=2 and sensor 4 at B7=9 —
// set any min-distance floor per sensor with 'w B7 <v>' -> 'S' -> 'r'.
// ============================================================================

// Sampling time (reg 0xB0), milliseconds. Unchanged. Must stay below the
// firmware's TOF_TIMEOUT_MS (55 in TAXI_Firmware/Config.h).
#define TOF_CFG_B0  50

// Range checks (reg 0xB4). 0xFE = 254 turns EVERY check on. Confirmed better
// than 248 at low B5 on all five sensors.
#define TOF_CFG_B4  254

// Signal threshold (reg 0xB5). 20 is the only value with ev3=0 on all five
// sensors. 0 leaks 2..4 phantom events; 10 is marginal. NOT compared against
// register 0x02 — trust the valid bit.
#define TOF_CFG_B5  20

// Sigma threshold (reg 0xB6), millimetres. 100/200/300 all give ev3=0 at
// B5=20; 200 kept as the safe middle.
#define TOF_CFG_B6  200

// Max distance (reg 0xB8), millimetres. CHANGED from 700 to 500 this session:
// the empty-space phantom band is 375-399 mm and 500 rejects it outright.
// Keep in sync with TOF_MAX_VALID_MM in TAXI_Firmware/Config.h (set to 500).
#define TOF_CFG_B8  500

// Crosstalk correction (reg 0xB2). Left at 0: the B1/B2 sweep showed any
// B2 >= 100 rejects everything (valid -> 0). No usable value exists.
#define TOF_CFG_B2  0
