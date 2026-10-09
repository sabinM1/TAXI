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
 * Placements sit in the BOTTOM half (operator's side, y < 0), pushed back
 * toward the outer edge. The shikiri-sen is at y = -5 cm, so y = -10 cm is the
 * most forward legal centre for a 10 cm robot; the tawara is part of the dohyo,
 * so y = -33.5 cm is the most rearward legal centre (back edge on the border).
 * Every opening starts deep (y = -25 to -31 cm) to leave runway to accelerate,
 * turn or flank.
 *
 * Openings are tuned against the measured kinematics (60 cm/s straight, 750 deg/s
 * full differential) so each one stays inside the ring even if the opponent is
 * not where it was expected: no step list may drive the robot over the border on
 * its own. The line-edge override would catch it, but an opening that relies on
 * that has already lost the round.
 *
 * The set, and what each is for:
 *   0 Rush (dash)              straight bull rush, reaches past mid-ring
 *   1/2 Turn-right/left rush   30 deg turn then a long angled rush
 *   3 Retreat-then-attack      short reverse to bait, then rush
 *   4/5 Angle-45 flank        45 deg turn up a side lane, to hit a corner
 *   6 Hit-spin                 full 360 deg sweep, then rush
 *   7 Wait-then-rush           pause to bait the opponent in, then rush
 */
#ifndef STRATEGY_DATA_H
#define STRATEGY_DATA_H

// slot 0: Rush (dash)
static const TacticStep s0[] = { {100,100,600} };
// slot 1: Turn-right rush
static const TacticStep s1[] = { {100,-100,40}, {100,100,550} };
// slot 2: Turn-left rush
static const TacticStep s2[] = { {-100,100,40}, {100,100,550} };
// slot 3: Retreat-then-attack (fake)
static const TacticStep s3[] = { {-100,-100,100}, {100,100,700} };
// slot 4: Angle-45 flank (right)
static const TacticStep s4[] = { {100,-100,60}, {100,100,400} };
// slot 5: Angle-45 flank (left)
static const TacticStep s5[] = { {-100,100,60}, {100,100,400} };
// slot 6: Hit-spin (scan then rush)
static const TacticStep s6[] = { {100,-100,480}, {100,100,400} };
// slot 7: Wait-then-rush (deceptive)
static const TacticStep s7[] = { {0,0,300}, {100,100,600} };

static const Strategy STRATEGIES[8] = {
  { s0, 1, SEARCH_SPIN, { 0, -25, 0 } },    // 0: Rush (dash)
  { s1, 2, SEARCH_ARC, { 0, -25, 0 } },    // 1: Turn-right rush
  { s2, 2, SEARCH_ARC, { 0, -25, 0 } },    // 2: Turn-left rush
  { s3, 2, SEARCH_SPIN, { 0, -25, 0 } },    // 3: Retreat-then-attack (fake)
  { s4, 2, SEARCH_ARC, { 8, -25, 0 } },    // 4: Angle-45 flank (right)
  { s5, 2, SEARCH_ARC, { -8, -25, 0 } },    // 5: Angle-45 flank (left)
  { s6, 2, SEARCH_SWEEP, { 0, -25, 0 } },    // 6: Hit-spin (scan then rush)
  { s7, 2, SEARCH_SPIN, { 0, -31, 0 } },    // 7: Wait-then-rush (deceptive)
};

#if DEBUG_VERBOSE
// Human-readable opening names, index-aligned with STRATEGIES[].
static const char* const STRATEGY_NAMES[8] = {
  "Rush (dash)",
  "Turn-right rush",
  "Turn-left rush",
  "Retreat-then-attack (fake)",
  "Angle-45 flank (right)",
  "Angle-45 flank (left)",
  "Hit-spin (scan then rush)",
  "Wait-then-rush (deceptive)"
};
#endif

#endif // STRATEGY_DATA_H
