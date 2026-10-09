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
 * Every opening starts deep (y = -25 cm) to leave runway to accelerate, turn or
 * flank.
 *
 * Openings are tuned against the measured kinematics (60 cm/s straight, 750 deg/s
 * full differential) so each one stays inside the ring even if the opponent is
 * not where it was expected: no step list may drive the robot over the border on
 * its own. The line-edge override would catch it, but an opening that relies on
 * that has already lost the round.
 *
 * Openings that would otherwise turn to a heading now START at that heading
 * instead: the operator places the robot already angled, so no time or precision
 * is spent pivoting. Slots 1/2 start at +/-30 deg, slots 3/4 at +/-45 deg, and
 * the edge flanks (5/6) at +/-90 deg.
 *
 * The last field of each row is handoffMs: 65% of the opening's total time,
 * computed by the designer at export. After that point a close opponent outranks
 * the committed move (see ST_TACTIC) — the opening is open-loop, but it must not
 * drive past a target it can already see. A value of 0 means NEVER hand off: the
 * opening runs fully blind, ignoring the ToF for its whole duration. The edge
 * flanks (5/6) use 0, since their sweep must complete to end up behind the
 * opponent.
 *
 * The set, and what each is for (right/left pairs sit next to each other):
 *   0 Rush (dash)              straight bull rush, reaches past mid-ring
 *   1/2 Angled rush            30 deg start, long angled rush
 *   3/4 Flank                  45 deg start, rush out, then turn onto the middle
 *   5/6 Edge flank             90 deg start, wide 200 deg arc hugging the tawara, blind
 *   7 Bait-and-flank           short reverse to bait, then flank right
 */
#ifndef STRATEGY_DATA_H
#define STRATEGY_DATA_H

// slot 0: Rush (dash)
static const TacticStep s0[] = { {100,100,600} };
// slot 1: Angled rush (right)
static const TacticStep s1[] = { {100,100,550} };
// slot 2: Angled rush (left)
static const TacticStep s2[] = { {100,100,550} };
// slot 3: Flank (right)
static const TacticStep s3[] = { {100,100,200}, {-100,100,120} };
// slot 4: Flank (left)
static const TacticStep s4[] = { {100,100,200}, {100,-100,120} };
// slot 5: Edge flank (right)
static const TacticStep s5[] = { {67,100,1616}, {-100,100,90} };
// slot 6: Edge flank (left)
static const TacticStep s6[] = { {100,67,1616}, {100,-100,90} };
// slot 7: Bait-and-flank (right)
static const TacticStep s7[] = { {-100,-100,80}, {100,100,600} };

static const Strategy STRATEGIES[8] = {
  { s0, 1, SEARCH_SPIN,  { 0, -25, 0 },    390 },    // 0: Rush (dash)
  { s1, 1, SEARCH_ARC,   { 0, -25, 30 },   358 },    // 1: Angled rush (right)
  { s2, 1, SEARCH_ARC,   { 0, -25, -30 },  358 },    // 2: Angled rush (left)
  { s3, 2, SEARCH_ARC,   { 8, -25, 45 },   208 },    // 3: Flank (right)
  { s4, 2, SEARCH_ARC,   { -8, -25, -45 }, 208 },    // 4: Flank (left)
  { s5, 2, SEARCH_ARC,   { 8, -25, 90 },   0 },      // 5: Edge flank (right) — blind
  { s6, 2, SEARCH_ARC,   { -8, -25, -90 }, 0 },      // 6: Edge flank (left) — blind
  { s7, 2, SEARCH_ARC,   { 0, -25, 30 },   442 },    // 7: Bait-and-flank (right)
};

#if DEBUG_VERBOSE
// Human-readable opening names, index-aligned with STRATEGIES[].
static const char* const STRATEGY_NAMES[8] = {
  "Rush (dash)",
  "Angled rush (right)",
  "Angled rush (left)",
  "Flank (right)",
  "Flank (left)",
  "Edge flank (right)",
  "Edge flank (left)",
  "Bait-and-flank (right)"
};
#endif

#endif // STRATEGY_DATA_H
