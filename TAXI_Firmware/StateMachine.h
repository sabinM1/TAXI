/*
 * StateMachine.h — combat flow.
 *
 * States:  IDLE -> TACTIC -> SEARCH -> ATTACK, with EVADE and BACKOFF as
 *          overrides from any combat state, and STOP when Start/Stop drops.
 *
 * SEARCH vs ATTACK — the ONLY thing that separates them is how far away the
 * opponent is (Config: TARGET_ENGAGE_MM):
 *   SEARCH  nothing close enough to commit to. Runs the DIP-selected search
 *           pattern at SEARCH_SPEED_PCT, and if a far sighting turns up, arcs
 *           slowly onto it instead of running the pattern blind. It never
 *           charges on a distance reading it cannot trust.
 *   ATTACK  the opponent is inside TARGET_ENGAGE_MM -> drive through it. Speed
 *           ramps with distance, steering is one proportional correction
 *           (steerToward()).
 *
 * There is no separate "pivot to re-center" state any more. The correction is
 * scaled by the base speed, so a hard bearing at attack speed already
 * counter-rotates the inner wheel and swings the robot round, while the same
 * bearing at search speed is only a gentle arc. One law, no threshold to
 * chatter across, and never a moment where the robot pivots in place without
 * making progress (which is how the old FLANK state used to lose a target).
 *
 * Start module (A0, INPUT_PULLUP): HIGH = fight, LOW = armed/stopped
 * (NanoLabs module, active-high per official docs; the Full_Robot_Test
 * sketch guessed the opposite polarity — it was wrong).
 *
 * Priority each loop: stop signal > evade > side backoff > state body.
 */
#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Sensors.h"
#include "Strategies.h"

enum RobotState : uint8_t { ST_IDLE, ST_TACTIC, ST_SEARCH, ST_ATTACK, ST_BACKOFF, ST_EVADE, ST_STOP };
static RobotState state = ST_IDLE;

// Start/stop signal latch. The NanoLabs module holds its state in
// non-volatile memory and drives PIN_START as a clean logic level, so the pin
// is read raw every loop — no debounce, no filtering, no averaging. Any filter
// here would be pure added latency on the referee's kill command.
// MUST start LOW (armed): initialising it HIGH would fake an instant start
// before the pin has ever been read.
static bool startSignal = LOW;

// Search bookkeeping
static uint32_t searchModeStart = 0;
static int8_t   searchDir = 1;          // +1 turn right, -1 turn left
static uint32_t lastSeenMs = 0;

// Evade bookkeeping. One program runs as two timed phases, FLEE -> TURN, and
// evadeFrom tracks the edge the flee is currently escaping from — that can
// change mid-program if a different sensor goes over white (see evadeUpdate()).
enum EvadePhase : uint8_t { EV_FLEE, EV_TURN };
static uint32_t   evadeStart = 0;        // start of the whole program
static uint32_t   evadePhaseStart = 0;   // start of the current phase
static EvadePhase evadePhase = EV_FLEE;
static int8_t     evadeFrom = 0;

// Side-backoff bookkeeping. backoffStart is the millis() of the last backoff
// trigger; it doubles as the cooldown anchor (see SIDE_BACKOFF_COOLDOWN_MS).
// Seeded so the first backoff is armed immediately.
static uint32_t backoffStart = (uint32_t)0 - (SIDE_BACKOFF_MS + SIDE_BACKOFF_COOLDOWN_MS);

// How long the "opponent alongside" condition has held, and whether it is
// holding right now. The hold restarts from zero any loop the condition is false
// (reading back out of range, or a side sensor that is not being polled), so the
// backoff only fires on a reading that STAYS close.
static uint32_t sideCloseSince  = 0;
static bool     sideCloseActive = false;

/**
 * Continuous milliseconds either SIDE ToF has been reporting an opponent at
 * contact range. 0 when the condition is not currently true. Evaluated every
 * loop so the hold keeps accumulating while the robot is reversing.
 */
static uint32_t sideCloseHeldMs() {
  if (!sensorsSideClose(SIDE_BACKOFF_DIST_MM)) { sideCloseActive = false; return 0; }
  uint32_t now = millis();
  if (!sideCloseActive) { sideCloseActive = true; sideCloseSince = now; }
  return now - sideCloseSince;
}

static bool isStarted() { return startSignal == HIGH; }

#if DEBUG_VERBOSE
static const char* stateName(RobotState s) {
  switch (s) {
    case ST_IDLE:   return "IDLE";
    case ST_TACTIC: return "TACTIC";
    case ST_SEARCH: return "SEARCH";
    case ST_ATTACK: return "ATTACK";
    case ST_BACKOFF: return "BACKOFF";
    case ST_EVADE:  return "EVADE";
    case ST_STOP:   return "STOP";
    default:        return "?";
  }
}
// Which half of the evade program is running, for the debug table.
static const char* evadePhaseName() {
  return (evadePhase == EV_FLEE) ? "FLEE" : "TURN";
}
// All state assignments go through here so every transition is logged.
static void gotoState(RobotState s) {
  if (s == state) return;
  Serial.print(F("[SM] ")); Serial.print(stateName(state));
  Serial.print(F(" -> "));   Serial.print(stateName(s));
  Serial.print(F("  t=")); Serial.print(millis());
  Serial.print(F("ms"));
  if (s == ST_EVADE) { Serial.print(F("  edgeFrom=")); Serial.print(evadeFrom); }
  Serial.println();
  state = s;
}
#else
static void gotoState(RobotState s) { state = s; }
#endif

/**
 * Sample the Start/Stop module. HIGH = fight, LOW = armed/stopped.
 *
 * RAW read on purpose (Config.h): the module output is a latched logic level,
 * so this is a single digitalRead() with no debounce and no state — it must
 * never be slowed down, because this value decides whether the robot is
 * allowed to keep driving.
 *
 * Call every loop() BEFORE anything that uses isStarted().
 */
static void startModuleUpdate() {
  startSignal = digitalRead(PIN_START);
}

// ------------- target bearing + last-seen direction -------------------------

// Direction the opponent was last seen in: a LIVE sighting at/inside
// TARGET_ENGAGE_MM. Used whenever the sensors have only a far or empty picture.
// This is NOT a filter — a close reading always wins outright, so it can never
// lag a pivot the way the old bearing memory did.
static int8_t lastSeenBearing = 0;
static bool   lastSeenValid   = false;
// Did the most recent targetBearing() call have an actual sensor reading (however
// far)? Distinguishes "steering on a live reading" from "steering on the memory"
// — only the former may hold ATTACK against the lost-target timeout.
static bool   bearingRawSeen = false;

/**
 * The bearing the robot steers on.
 *
 *   close sighting (<= TARGET_ENGAGE_MM) -> the live reading, and it becomes the
 *                                           remembered direction
 *   far sighting, or nothing             -> the remembered direction, if any
 *   nothing remembered yet               -> the far reading as-is, or 127
 *
 * The far case is the point of the memory: at long range the ToF bearing is weak
 * and jumpy, so steering on where the opponent was last actually seen is a better
 * bet than on a faint far reading.
 *
 * No smoothing anywhere: a lagging bearing makes the robot keep pivoting after it
 * has already swept past the opponent, and with the correction scaled by the base
 * speed the steering is smooth by construction — there is no band edge to chatter
 * across.
 *
 * 127 ("nothing visible") is only returned when there is no memory to fall back on.
 */
static int8_t targetBearing() {
  int8_t raw = sensorsOpponent();
  bool ok; uint16_t mm = sensorsClosestMm(&ok);
  bearingRawSeen = (raw != 127);

  // Close enough to be a real, trustworthy sighting: remember it, steer on it.
  if (raw != 127 && ok && mm <= TARGET_ENGAGE_MM) {
    lastSeenBearing = raw;
    lastSeenValid   = true;
    return raw;
  }

  // Far away, or nothing visible: prioritise the remembered direction.
  if (lastSeenValid) return lastSeenBearing;

  return raw;      // nothing remembered yet: use the far reading as-is
}

// ---------------- contact latch ----------------

// millis() of the last time the MIDDLE sensor read blade contact. Seeded so that
// "was contact seen within CONTACT_LATCH_MS?" is false before anything happens.
static uint32_t contactStamp = (uint32_t)0 - CONTACT_LATCH_MS;

/**
 * Blade contact, latched.
 *
 * sensorsContact() is a single-frame test on the MIDDLE sensor, which is exactly
 * the sensor that gets blinded at impact: the blade ends up inside the M60F's
 * 9 mm minimum range, so the reading goes invalid right when we are in the shove.
 * A bare single-frame test therefore drops the robot out of the full-power push
 * at the worst moment — back to the steering push (one wheel slowed) or even the
 * cautious blind push.
 *
 * Latching contact for CONTACT_LATCH_MS makes the commitment stick: once the
 * robot has decided it is in a shove, a bad or missing frame cannot interrupt it.
 * The latch is stamped fresh every frame that real contact is seen, so a
 * sustained shove keeps it armed; it expires naturally if the opponent gets away.
 */
static bool targetInContact() {
  if (sensorsContact()) { contactStamp = millis(); return true; }
  return (uint32_t)(millis() - contactStamp) < CONTACT_LATCH_MS;
}

// ------------------------- state helpers ------------------------------------

/**
 * Begin a search. The pattern's first leg turns toward the side the opponent was
 * last seen on, so losing sight does not drop the robot into a blind sweep away
 * from where the opponent actually is — this is the "prioritise the remembered
 * direction" rule.
 *
 * Only the DIRECTION of the first leg is biased: searchUpdate() still flips
 * searchDir every SEARCH_PATTERN_TIMEOUT_MS, so the robot can never spiral one way
 * forever. This is also why the memory is a direction and not a servo target — a
 * remembered bearing does not change as the robot turns, so steering on it
 * directly would spin the robot endlessly.
 */
static void enterSearch() {
  searchModeStart = millis();
  if (lastSeenValid) searchDir = (lastSeenBearing >= 0) ? 1 : -1;
}

/**
 * Steer the robot while hunting. Three patterns:
 *   SPIN  — rotate in place (fastest sensor sweep, never leaves the ring)
 *   ARC   — wide arcs, covers ground while scanning
 *   SWEEP — alternating shallow arcs (figure-8-ish), most area covered
 * After SEARCH_PATTERN_TIMEOUT_MS without seeing anything, the pattern
 * mirrors (searchDir flips) so we never spiral forever in one direction.
 */
static void searchUpdate(SearchStyle style) {
  int sp = SEARCH_SPEED_PCT;
  if (millis() - searchModeStart > SEARCH_PATTERN_TIMEOUT_MS) {
    searchModeStart = millis();
    searchDir = -searchDir;
  }

  switch (style) {
    case SEARCH_SPIN:   // in-place rotation, fastest sensor sweep
      motorsSet(searchDir > 0 ? sp : -sp,
                searchDir > 0 ? -sp : sp);
      break;
    case SEARCH_ARC:    // wide arcs, covers ground while moving
      motorsSet(searchDir > 0 ? sp : sp / 2,
                searchDir > 0 ? sp / 2 : sp);
      break;
    case SEARCH_SWEEP:  // figure-8-ish: alternate arcs every cycle
    default:
      motorsSet(searchDir > 0 ? sp : sp / 3,
                searchDir > 0 ? sp / 3 : sp);
      break;
  }
}

/**
 * Base push speed for the attack, read out of the ATTACK_SPEED_RAMP table in
 * Config.h and linearly interpolated between its anchors.
 *
 * The window runs from ATTACK_SPEED_FAR_PCT at TARGET_ENGAGE_MM (the distance the
 * robot commits from) up to ATTACK_SPEED_CLOSE_PCT at ATTACK_CLOSE_DIST_MM, flat
 * outside it. It is the TABLE, not a single slope, that shapes the approach: slow
 * while the reading could still be anything, speed building as the range closes,
 * everything inside the last ATTACK_CLOSE_DIST_MM. See the table's comment.
 *
 * Falls back to the far value (the ramp's first anchor) when the closest reading
 * is invalid — which is also the speed used for the blind push.
 */
static int attackBaseSpeed(uint16_t mm, bool ok) {
  if (!ok || mm >= ATTACK_SPEED_RAMP[0].mm) return ATTACK_SPEED_RAMP[0].pct;

  // Anchors run far -> near. Find the segment this distance falls in and
  // interpolate between that anchor and the one before it (farther, slower).
  for (uint8_t i = 1; i < ATTACK_SPEED_RAMP_POINTS; i++) {
    if (mm < ATTACK_SPEED_RAMP[i].mm) continue;
    return (int)map((long)mm,
                    ATTACK_SPEED_RAMP[i].mm,     ATTACK_SPEED_RAMP[i - 1].mm,
                    ATTACK_SPEED_RAMP[i].pct,    ATTACK_SPEED_RAMP[i - 1].pct);
  }
  return ATTACK_SPEED_RAMP[ATTACK_SPEED_RAMP_POINTS - 1].pct;   // closest zone
}

/**
 * The one steering law — used by the attack, and by SEARCH's approach leg.
 *
 * Both wheels drive forward at `base`; a differential proportional to
 * `bearing` is then shifted between them:
 *
 *   bearing > 0  opponent to the right -> right wheel slows, left speeds up
 *   bearing < 0  opponent to the left  -> mirrored
 *   bearing = 0  dead ahead            -> both wheels equal, straight line
 *
 * The correction scales with BOTH the bearing and `base` (via
 * ATTACK_STEER_GAIN_PCT), which is what lets this single function replace the
 * old attack-steer + flank-pivot pair: at attack speed a full bearing swings the
 * robot round onto the target, while at SEARCH_SPEED_PCT the same bearing is only
 * a gentle arc. Because the outer wheel is always driving forward, the robot
 * never stalls or spins in place while trying to line up.
 *
 * `gainPct` is the steering authority at full bearing, as a percent of `base`
 * (see ATTACK_STEER_GAIN_PCT); it defaults to the configured value. ST_ATTACK
 * passes a REDUCED gain close in, to let the yaw settle before the blade lands —
 * see straightenGainPct(). SEARCH's approach leg always uses the default.
 *
 * What the gain means for the inner wheel: at gain 100 a full bearing stops it
 * (corr == base); the counter-rotation that swings the robot round onto the
 * target only starts once the gain is above 100.
 *
 * bearing = 127 (nothing visible) is not a direction: it drives straight.
 * `base` is expected >= 0; both outputs are clamped to -100..100.
 */
static void steerToward(int base, int8_t bearing, int gainPct = ATTACK_STEER_GAIN_PCT) {
  if (bearing == 127) { motorsSet(base, base); return; }
  int corr = (int)((int32_t)bearing * base * gainPct / 10000);
  motorsSet(constrain(base + corr, -100, 100),
            constrain(base - corr, -100, 100));
}

/**
 * Steering authority for the chase at the current range, faded out over the last
 * stretch so the blade arrives square instead of mid-turn. See
 * STRAIGHTEN_DIST_MM in Config.h for why this exists and why it is a separate
 * distance from the contact latch.
 *
 * Only the MIDDLE sensor arms the taper, and that is the safety property, not an
 * implementation detail: the M60F's 18-22 degree cone is narrower than the
 * steering dead zone, so if the middle sensor can see the opponent at this range
 * then the opponent really is in front of the blade. A target being turned onto
 * from a corner never arms it, so it always keeps the full correction and is
 * never abandoned halfway through the turn.
 */
static int straightenGainPct() {
  if (!tofValid[2] || tofMm[2] >= STRAIGHTEN_DIST_MM) return ATTACK_STEER_GAIN_PCT;
  if (tofMm[2] <= PUSH_CONTACT_DIST_MM) return 0;
  return (int)map((long)tofMm[2], PUSH_CONTACT_DIST_MM, STRAIGHTEN_DIST_MM,
                  0, ATTACK_STEER_GAIN_PCT);
}

/**
 * One loop-tick of the chase (ST_ATTACK body). Speed from the closest reading,
 * steering from `bearing` — passed in rather than re-fetched, so the caller can
 * stamp lastSeenMs from the same sample it is acting on.
 *
 * NOT used at blade contact: the ST_ATTACK body intercepts that case first, so
 * this always runs with at least some standoff (see PUSH_CONTACT_DIST_MM).
 */
static void attackUpdate(int8_t bearing) {
  bool ok; uint16_t mm = sensorsClosestMm(&ok);
  steerToward(attackBaseSpeed(mm, ok), bearing, straightenGainPct());
}

/**
 * Start a fresh evade program for the edge that just tripped.
 * Called by the edge override in stateMachineUpdate().
 */
static void evadeBegin(int8_t edge) {
  evadeFrom       = edge;
  evadeStart      = millis();
  evadePhaseStart = evadeStart;
  evadePhase      = EV_FLEE;
}

/**
 * The escape burst itself: one motor command that gets the robot away from a
 * specific edge.
 *
 * @param src  the edge driving this burst: 0 = left, 1 = rear, 2 = right.
 *
 * Rear edge -> drive FORWARD (reversing would push us further off the dohyo).
 * Front edge -> reverse hard, straight back onto the black.
 *
 * The edge is a parameter rather than reading `evadeFrom` directly because the
 * edge we must escape from is not necessarily the one EVADE started on (see
 * evadeUpdate()).
 */
static void evadeEscapeBurst(int8_t src) {
  if (src == 1) {
    // Rear edge: NEVER reverse further — drive forward with a gentle bias.
    if (searchDir > 0) motorsSet(80, 55); else motorsSet(55, 80);
  } else {
    motorsSet(EVADE_REVERSE_SPEED_PCT, EVADE_REVERSE_SPEED_PCT);
  }
}

/**
 * The pivot away from the edge: TURN phase of the evade program.
 *
 * Left edge  -> left wheel forward, right wheel back  -> rotates right.
 * Right edge -> mirrored -> rotates left.
 * Rear edge  -> spin in place (either way leads back towards the middle).
 * Uses the most recent edge seen, so a program that started on the left but
 * ended up over the rear pivots according to where the danger actually is.
 */
static void evadeTurnAway() {
  if (evadeFrom == 0)      motorsSet(EVADE_TURN_OUTER_PCT, -EVADE_TURN_INNER_PCT);
  else if (evadeFrom == 2) motorsSet(-EVADE_TURN_INNER_PCT, EVADE_TURN_OUTER_PCT);
  else motorsSet(searchDir > 0 ? EVADE_TURN_SPIN_PCT : -EVADE_TURN_SPIN_PCT,
                 searchDir > 0 ? -EVADE_TURN_SPIN_PCT : EVADE_TURN_SPIN_PCT);
}

/**
 * EVADE program. Two timed phases, entered exclusively via the edge override in
 * stateMachineUpdate():
 *
 *   FLEE  get off the white. At least EVADE_ESCAPE_MIN_MS of blind burst (the
 *         edge can flicker off while we are still rolling off the dohyo), then
 *         keep fleeing while a sensor still reads white, up to EVADE_FLEE_MAX_MS
 *         — past that the program gives up instead of pivoting along the edge it
 *         is sitting on. The rear edge always flees FORWARD — reversing would
 *         push us further out.
 *
 *   TURN  pivot away from the edge for exactly EVADE_TURN_MS. This is the phase
 *         that decides how far the robot rotates, so it is a fixed duration: the
 *         old code pivoted for whatever was left of EVADE_TOTAL_MAX_MS once the
 *         white cleared, which made the overshoot different every run and
 *         impossible to tune.
 *
 *   done  stop, hand over to SEARCH.
 *
 * Two escape hatches keep this safe:
 *   - the edge is re-read live in both phases, so the flee always follows the
 *     sensor that is actually white now, not the one that started the program;
 *   - the TURN only ever starts from black, and aborts back to FLEE if the pivot
 *     itself finds white again (bad geometry, wrong turn direction, short ring).
 *     EVADE_TOTAL_MAX_MS is the hard stop, so a stuck sensor can never spin the
 *     robot forever.
 */
static void evadeUpdate() {
  // Hard stop for the whole program. A safety net, not a tuning knob — normal
  // escapes finish long before this.
  if (millis() - evadeStart >= EVADE_TOTAL_MAX_MS) {
    motorsStop();
    gotoState(ST_SEARCH);
    enterSearch();
    return;
  }

  int8_t   live = sensorsEdgeIndex();          // -1 = every sensor on black
  uint32_t pt   = millis() - evadePhaseStart;  // time in the current phase

  switch (evadePhase) {
    case EV_FLEE:
      if (live >= 0) evadeFrom = live;         // fleeing from a stale edge can
      evadeEscapeBurst(evadeFrom);             // drive us further off the ring

      if (pt >= EVADE_FLEE_MAX_MS) {
        // Still over the white after the full flee budget. Pivoting now would
        // rotate the robot along the edge it is sitting on, so stop the program
        // instead: SEARCH will re-enter EVADE with a fresh flee if the white is
        // still there. Reached when the white will not clear — a wheel over the
        // line, or a line sensor stuck reading white.
        motorsStop();
        gotoState(ST_SEARCH);
        enterSearch();
      } else if (live < 0 && pt >= EVADE_ESCAPE_MIN_MS) {
        // Clear, and past the mandatory blind burst -> time to turn.
        evadePhase = EV_TURN;
        evadePhaseStart = millis();
      }
      break;

    case EV_TURN:
      if (live >= 0) {
        // The pivot itself pushed a sensor back over the white. Fleeing again is
        // safer than continuing to rotate over the edge.
        evadeFrom = live;
        evadePhase = EV_FLEE;
        evadePhaseStart = millis();
        break;
      }
      evadeTurnAway();
      if (pt >= EVADE_TURN_MS) {
        motorsStop();
        gotoState(ST_SEARCH);
        enterSearch();
      }
      break;
  }
}

// ------------------------- main entry ---------------------------------------

/**
 * Combat brain. Call once per loop() AFTER sensorsUpdate(); drives motors
 * and mutates `state`. Priority each call:
 *   1. start module killed   -> STOP   (unconditional, terminal, motors cut
 *                              every loop from then on — nothing can override
 *                              or out-prioritise it, see the latching note in
 *                              the ST_STOP case)
 *   2. line edge             -> EVADE (from any fighting state, incl. TACTIC)
 *   3. state body
 *
 * This function is also the ONLY place motors are ever commanded, so once the
 * pin reads LOW the robot cannot be driven again without a power cycle.
 */
void stateMachineUpdate() {
  startModuleUpdate();

  // Global stop, checked before every other rule in this function. The wake-up
  // case (state == ST_IDLE) is excluded so the pre-match state does not jump
  // straight to STOP; ST_STOP is excluded because it re-runs motorsStop() in
  // its own state body anyway.
  if (!isStarted() && state != ST_IDLE && state != ST_STOP) {
    gotoState(ST_STOP); motorsStop();
    return;
  }

  // EVADE override: any edge while fighting (including the opening tactic)
  if (sensorsAnyEdge() && state != ST_EVADE &&
      state != ST_IDLE && state != ST_STOP) {
    evadeBegin(sensorsEdgeIndex());
    gotoState(ST_EVADE);
  }

  // Side-sensor backoff. An opponent alongside the robot (a SIDE ToF at contact
  // range) cannot be acted on by the front sensors, so reverse hard to open the
  // gap. The condition must HOLD for SIDE_BACKOFF_CONFIRM_MS first, so a single
  // grazing frame or a stale reading cannot fire the reverse. Checked AFTER the
  // line-edge override above, which therefore always wins: the reverse can never
  // bypass the edge check. The cooldown stops a target that stays alongside from
  // making the robot back up forever.
  uint32_t sideHeldMs = sideCloseHeldMs();
  if (state != ST_BACKOFF && state != ST_IDLE && state != ST_STOP &&
      sideHeldMs >= SIDE_BACKOFF_CONFIRM_MS &&
      (uint32_t)(millis() - backoffStart) >= SIDE_BACKOFF_MS + SIDE_BACKOFF_COOLDOWN_MS) {
    backoffStart    = millis();
    sideCloseActive = false;      // the next backoff needs its own fresh hold
    gotoState(ST_BACKOFF);
  }

  // Degraded fallback: too many ToF sensors are not ANSWERING. This only ever
  // changes the BLIND search/push (see the ST_SEARCH and ST_ATTACK cases) — a
  // visible target is always acted on, no matter what this says.
  bool degraded = sensorsHealthyCount() < TOF_MIN_HEALTHY_SENSORS;

  // Every ToF sensor is polled on every sweep, in every state — there is no
  // front-only scan any more. The chase needs to be able to turn onto an opponent
  // that is off to a side, and BACKOFF needs the side sensors live, so narrowing
  // the sweep to the front trio would blind the robot to the very readings it acts
  // on. A full five-sensor round-robin is ~10 ms, fast enough for all of it.

  switch (state) {
    case ST_IDLE:
      motorsStop();
      if (isStarted()) {
        // Match start: forget anything seen during bench testing, so the
        // remembered direction is only ever populated by this round.
        lastSeenValid = false;
        tacticReset();
        gotoState(ST_TACTIC);
      }
      break;

    case ST_TACTIC:
      // The opening runs to completion, with ONE exception besides the line-edge
      // override above: once handoffMs (65% of the opening, baked in by the
      // designer) has elapsed, a close opponent outranks the committed move. The
      // opening is open-loop, but it must not drive past a target it can already
      // see — this is the same sighting + range test ST_SEARCH commits on, so the
      // handoff lands in ST_ATTACK exactly as a normal sighting would.
      //
      // handoffMs == 0 means the opening runs fully blind (no ToF handoff at all),
      // used by the edge flanks whose sweep must complete.
      if (STRATEGIES[strategyIndex].handoffMs != 0 &&
          tacticElapsedMs() >= STRATEGIES[strategyIndex].handoffMs) {
        int8_t sighted = sensorsOpponent();
        bool ok; uint16_t mm = sensorsClosestMm(&ok);
        if (sighted != 127 && ok && mm <= TARGET_ENGAGE_MM) {
          lastSeenMs = millis();
          gotoState(ST_ATTACK);
          break;
        }
      }
      if (tacticRun()) {
        gotoState(ST_SEARCH);
        enterSearch();
      }
      break;

    case ST_SEARCH: {
      // The RAW reading, at any range: a visible opponent must be chased, so this
      // deliberately does NOT go through targetBearing() — that prefers the
      // remembered direction, and a stale guess must never start an attack.
      int8_t sighted = sensorsOpponent();

      if (sighted != 127) {
        lastSeenMs = millis();
        bool ok; uint16_t mm = sensorsClosestMm(&ok);

        if (ok && mm <= TARGET_ENGAGE_MM) {
          // Close enough to commit: drive through it.
          gotoState(ST_ATTACK);
          break;
        }
        // Visible but still outside the engagement range: turn slowly onto it and
        // close in, so a distant sighting is not lost without committing to a
        // charge on a bearing that is not trustworthy yet.
        steerToward(SEARCH_SPEED_PCT, sighted);
        break;
      }

      // Nothing visible. If too many ToF sensors are not answering, blind spin is
      // the safest way to stumble back onto the opponent and it can never drive
      // off the dohyo. Otherwise run the selected search pattern — whose first leg
      // enterSearch() turns toward the last-seen direction.
      if (degraded) {
        motorsSet(SEARCH_SPEED_PCT, -SEARCH_SPEED_PCT);
        break;
      }
      searchUpdate(STRATEGIES[strategyIndex].search);
      break;
    }

    case ST_ATTACK: {
      // Blade contact beats everything else in this state: both wheels full
      // ahead, no steering trim, no turn-in. lastSeenMs is stamped because the
      // opponent is very much visible — without it the lost-target timeout would
      // eventually pull the robot out of its own shove. Latched (targetInContact)
      // so one invalid frame at impact cannot break the shove.
      if (targetInContact()) {
        motorsSet(PUSH_CONTACT_SPEED_PCT, PUSH_CONTACT_SPEED_PCT);
        lastSeenMs = millis();
        break;
      }

      int8_t bearing = targetBearing();
      if (bearingRawSeen) lastSeenMs = millis();

      if ((millis() - lastSeenMs) > ATTACK_TARGET_LOST_MS) {
        // No LIVE sighting for the whole grace window, so the bearing (if any) is
        // only a remembered direction: stop attacking on it and go search.
        gotoState(ST_SEARCH); enterSearch();
      } else if (bearing == 127) {
        // Push on blind momentum. Degraded means too few sensors are ANSWERING,
        // so keep it deliberately cautious.
        int push = degraded ? DEGRADED_PUSH_SPEED_PCT : ATTACK_SPEED_FAR_PCT;
        motorsSet(push, push);
      } else {
        attackUpdate(bearing);
      }
      break;
    }

    case ST_EVADE:
      evadeUpdate();
      break;

    // Side-sensor backoff: reverse flat out for SIDE_BACKOFF_MS, then hand back
    // to SEARCH, which turns to bring the opponent onto the front sensors. The
    // line-edge override runs before this every loop, so the reverse can never
    // drive the robot over the border — the line check is never bypassed.
    case ST_BACKOFF:
      if ((uint32_t)(millis() - backoffStart) >= SIDE_BACKOFF_MS) {
        motorsStop();
        gotoState(ST_SEARCH);
        enterSearch();
      } else {
        motorsSet(SIDE_BACKOFF_SPEED_PCT, SIDE_BACKOFF_SPEED_PCT);
      }
      break;

    // Latching terminal state: the referee's stop ends the match, so the only
    // exit is a power cycle (matches the NanoLabs reference flow, which parks
    // the robot in a locked loop). motorsStop() re-runs every loop, which is
    // cheap and guarantees nothing here can be pre-empted by a stale command.
    case ST_STOP:
    default:
      motorsStop();
      break;
  }
}

/** Current state for the ring/status display. */
uint8_t stateMachineState() { return (uint8_t)state; }

#endif // STATEMACHINE_H
