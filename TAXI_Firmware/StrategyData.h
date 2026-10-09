    /*
 * StrategyData.h — opening tactic data, managed by tools/strategy-designer.html
 *
 * Included by Strategies.h AFTER the TacticStep / Strategy / SearchStyle /
 * StartPose types are defined. Do NOT include this file directly.
 *
 * The strategy designer reads and writes THIS file: import it, edit the
 * openings on the dohyo, then export it back over this one. There is no other
 * copy of the strategies anywhere — this file is the single source of truth.
 *
 * Dohyo 77 cm | robot 10x10 cm
 * Calibration: 60 cm/s straight @ 100% duty, 750 deg/s @ full differential.
 *
 * Each STRATEGIES row carries the start pose that opening assumes:
 *   { x, y, heading }  — x/y in cm from centre (+x right, +y toward the
 *   opponent), heading in degrees (0 = up, + = clockwise). The opening is
 *   open-loop, so this is placement metadata, not a steering input.
 *
 * Placements sit in the BOTTOM half (operator's side, y < 0). The shikiri-sen
 * is at y = -5 cm, so y = -10 cm is the most forward legal centre for a 10 cm
 * robot; deeper and offset poses give the openings room to turn or flank.
 */
#ifndef STRATEGY_DATA_H
#define STRATEGY_DATA_H

// slot 0: Rush (dash)
static const TacticStep s0[] = { {100,100,250} };
// slot 1: Turn-right rush
static const TacticStep s1[] = { {100,-100,120}, {100,100,400} };
// slot 2: Turn-left rush
static const TacticStep s2[] = { {-100,100,120}, {100,100,400} };
// slot 3: Retreat-then-attack (fake)
static const TacticStep s3[] = { {-100,-100,200}, {100,100,450} };
// slot 4: Angle-45 flank (right)
static const TacticStep s4[] = { {100,-100,60}, {100,100,400} };
// slot 5: Angle-90 side attack (right)
static const TacticStep s5[] = { {100,-100,120}, {100,100,400} };
// slot 6: Hit-spin (scan then rush)
static const TacticStep s6[] = { {90,-90,240}, {100,100,300} };
// slot 7: Wait-then-rush (deceptive)
static const TacticStep s7[] = { {0,0,300}, {100,100,400} };

static const Strategy STRATEGIES[8] = {
  { s0, 1, SEARCH_SPIN, { 0, -10, 0 } },    // 0: Rush (dash) — front edge on the line
  { s1, 2, SEARCH_ARC, { 0, -15, 0 } },    // 1: Turn-right rush
  { s2, 2, SEARCH_ARC, { 0, -15, 0 } },    // 2: Turn-left rush
  { s3, 2, SEARCH_SPIN, { 0, -22, 0 } },    // 3: Retreat-then-attack (fake)
  { s4, 2, SEARCH_ARC, { 8, -15, 0 } },    // 4: Angle-45 flank (right)
  { s5, 2, SEARCH_ARC, { 8, -15, 0 } },    // 5: Angle-90 side attack (right)
  { s6, 2, SEARCH_SWEEP, { 0, -15, 0 } },    // 6: Hit-spin (scan then rush)
  { s7, 2, SEARCH_SPIN, { 0, -22, 0 } },    // 7: Wait-then-rush (deceptive)
};

#if DEBUG_VERBOSE
// Human-readable opening names, index-aligned with STRATEGIES[].
static const char* const STRATEGY_NAMES[8] = {
  "Rush (dash)",
  "Turn-right rush",
  "Turn-left rush",
  "Retreat-then-attack (fake)",
  "Angle-45 flank (right)",
  "Angle-90 side attack (right)",
  "Hit-spin (scan then rush)",
  "Wait-then-rush (deceptive)"
};
#endif

#endif // STRATEGY_DATA_H
