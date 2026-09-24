/*
 * StateMachine.h — combat flow.
 *
 * States:  IDLE -> TACTIC -> SEARCH <-> ATTACK/FLANK, with EVADE override
 *          from any combat state, and STOP when the Start/Stop module drops.
 *
 * ATTACK vs FLANK — both mean "the opponent is on a ToF sensor", they only
 * differ in how far off-axis it is (split at +/-TARGET_ON_AXIS_BAND bearing
 * units, see Config.h):
 *   ATTACK  target is roughly ahead -> ram it. Speed comes from the distance
 *           (fast up close, cautious far), steering is a smooth proportional
 *           correction (see attackUpdate()).
 *   FLANK   target is off to a side -> bang-bang pivot at a fixed speed until
 *           it is back in front, then hand over to ATTACK (see flankUpdate()).
 *           It is a re-centering turn, not an orbit around the opponent.
 * SEARCH picks the right one on sight; the two also swap automatically while
 * fighting if the bearing drifts in or out of the band.
 *
 * Start module (A0, INPUT_PULLUP): HIGH = fight, LOW = armed/stopped
 * (NanoLabs module, active-high per official docs; the Full_Robot_Test
 * sketch guessed the opposite polarity — it was wrong).
 *
 * Priority each loop: stop signal > evade > track (attack/flank) > search.
 */
#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Sensors.h"
#include "Strategies.h"

enum RobotState : uint8_t { ST_IDLE, ST_TACTIC, ST_SEARCH, ST_ATTACK, ST_FLANK, ST_EVADE, ST_STOP };
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

static bool isStarted() { return startSignal == HIGH; }

#if DEBUG_VERBOSE
static const char* stateName(RobotState s) {
  switch (s) {
    case ST_IDLE:   return "IDLE";
    case ST_TACTIC: return "TACTIC";
    case ST_SEARCH: return "SEARCH";
    case ST_ATTACK: return "ATTACK";
    case ST_FLANK:  return "FLANK";
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

// ---------------- target bearing: live reading + short memory ---------------

static int8_t   memBearing  = 0;    // direction the opponent was last seen in
static uint8_t  memStrength = 0;    // how strong that sighting was (0..100)
static uint32_t memStamp    = 0;    // when it was taken

/**
 * How much a sighting taken `age` ms ago still biases the bearing, in percent.
 * Linear fade to zero across TARGET_MEMORY_MS.
 */
static uint8_t targetMemoryWeight(uint32_t age) {
  if (age >= TARGET_MEMORY_MS) return 0;
  uint32_t v = (uint32_t)memStrength * (TARGET_MEMORY_MS - age) / TARGET_MEMORY_MS;
  return (uint8_t)(v * TARGET_MEMORY_MAX_PCT / 100);
}

/**
 * The bearing the robot actually steers on.
 *
 * The live reading from sensorsOpponent(), blended towards the direction the
 * opponent was last seen in by a weight that fades out over TARGET_MEMORY_MS.
 * This is what stops a single frame from reversing the turn: a reading that
 * contradicts the remembered direction only gets the remaining say, so the robot
 * finishes the correction it started and re-orients after the memory expires.
 *
 * 127 ("nothing visible") is passed straight through — the memory must never
 * fake a target, or target acquisition and the lost-target timeout break.
 *
 * Refresh rule: the memory is retaken from the live reading when it does not
 * contradict what is stored (same side of centre) and is at least as confident.
 * A sudden opposite reading therefore does NOT overwrite the memory — if it did,
 * there would be nothing left to damp the turn with.
 */
static int8_t targetBearing() {
  int8_t raw = sensorsOpponent();
  if (raw == 127) return 127;

  bool ok; uint16_t mm = sensorsClosestMm(&ok);

  // Confidence 0..100: how close the opponent was seen. Full at
  // TARGET_MEMORY_NEAR_MM, nothing at ATTACK_FAR_DIST_MM.
  uint8_t conf = 0;
  if (ok && mm <= TARGET_MEMORY_NEAR_MM) {
    conf = 100;
  } else if (ok && mm < ATTACK_FAR_DIST_MM) {
    conf = 100 - (uint8_t)((uint32_t)(mm - TARGET_MEMORY_NEAR_MM) * 100 /
                           (ATTACK_FAR_DIST_MM - TARGET_MEMORY_NEAR_MM));
  }

  uint8_t w = targetMemoryWeight(millis() - memStamp);
  bool agrees = (raw >= 0 && memBearing >= 0) || (raw <= 0 && memBearing <= 0);

  if (w == 0 || (agrees && conf >= memStrength)) {
    memBearing  = raw;
    memStrength = conf;
    memStamp    = millis();
    return raw;                     // nothing to damp: steer on the live reading
  }

  return (int8_t)constrain((int32_t)raw * (100 - w) / 100 +
                           (int32_t)memBearing * w / 100, -100, 100);
}

// ------------------------- state helpers ------------------------------------

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
 * Base push speed for the attack: full power at ATTACK_CLOSE_DIST_MM,
 * tapering linearly to ATTACK_SPEED_FAR_PCT at ATTACK_FAR_DIST_MM.
 * Falls back to the far speed when the closest reading is invalid.
 */
static int attackBaseSpeed(uint16_t mm, bool ok) {
  if (!ok || mm >= ATTACK_FAR_DIST_MM) return ATTACK_SPEED_FAR_PCT;
  if (mm <= ATTACK_CLOSE_DIST_MM)      return ATTACK_SPEED_CLOSE_PCT;
  return (int)map((long)mm, ATTACK_CLOSE_DIST_MM, ATTACK_FAR_DIST_MM,
                  ATTACK_SPEED_CLOSE_PCT, ATTACK_SPEED_FAR_PCT);
}

/**
 * Chase + push. Used while the opponent is centred (|bearing| <= TARGET_ON_AXIS_BAND).
 * Speed: attackBaseSpeed() tapers with distance — full power at
 * ATTACK_CLOSE_DIST_MM, ATTACK_SPEED_FAR_PCT at ATTACK_FAR_DIST_MM.
 * Steering: one smooth proportional term — bearing > 0 (opponent to the
 * right) slows the right motor and vice-versa, scaled by ATTACK_STEER_GAIN_PCT.
 * Stamps lastSeenMs so the caller's lost-target timeout stays armed.
 *
 * NOT used at blade contact: the ST_ATTACK body intercepts that case first, so
 * this always runs with at least some standoff (see PUSH_CONTACT_DIST_MM).
 */
static void attackUpdate() {
  int8_t bearing = targetBearing();
  bool ok; uint16_t mm = sensorsClosestMm(&ok);
  lastSeenMs = millis();

  int base = attackBaseSpeed(mm, ok);

  // Proportional steer: bearing >0 = opponent right -> slow right motor.
  int corr = (int)((int32_t)bearing * base * ATTACK_STEER_GAIN_PCT / 10000);
  motorsSet(constrain(base + corr, -100, 100),
            constrain(base - corr, -100, 100));
}

/**
 * Re-centering pivot. Used while the opponent is off-axis
 * (|bearing| > TARGET_FLANK_BAND), i.e. only a side or outer front sensor has it.
 *
 * The pivot speed scales with how far off-axis the target is: a nudge at the
 * entry threshold, a full FLANK_TURN_SPEED_PCT at |bearing| = 100, the inner wheel
 * counterspinning at half of that. Bang-bang (one speed for every bearing) is why
 * a target barely outside the band got the same full-scale pivot as one 90 degrees
 * off, which overshot and then lost it.
 *
 * 127 means lost — do nothing and let the caller's timeout drop us back to SEARCH.
 * Unlike ATTACK this never pushes blind: a pivot in the wrong direction is worse
 * than standing still.
 */
static void flankUpdate() {
  int8_t bearing = targetBearing();
  if (bearing == 127) return;
  lastSeenMs = millis();

  uint8_t mag = (uint8_t)abs((int)bearing);          // 1..100
  uint8_t outer = (uint8_t)constrain(map((long)mag, TARGET_ON_AXIS_BAND, 100,
                                         FLANK_TURN_MIN_PCT, FLANK_TURN_SPEED_PCT),
                                     FLANK_TURN_MIN_PCT, FLANK_TURN_SPEED_PCT);
  uint8_t inner = outer / 2;                         // counterspin, as before

  if (bearing > 0) motorsSet(outer, -inner);
  else             motorsSet(-inner, outer);
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
    searchModeStart = millis();
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
        searchModeStart = millis();
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
        searchModeStart = millis();
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

  // Degraded fallback: too many ToF dead -> stay in safe spin-search.
  bool degraded = sensorsHealthyCount() < TOF_MIN_HEALTHY_SENSORS;

  // ToF front-priority scan only while pushing. FLANK deliberately keeps the
  // full round-robin: it exists to chase a target that is off to the side, so it
  // needs the side sensors live — capping it to the front trio would leave it
  // blind to the very reading that sent it there.
  sensorsSetAttackScan(state == ST_ATTACK);

  switch (state) {
    case ST_IDLE:
      motorsStop();
      if (isStarted()) { tacticReset(); gotoState(ST_TACTIC); }
      break;

    case ST_TACTIC:
      if (tacticRun()) {
        gotoState(ST_SEARCH);
        searchModeStart = millis();
      }
      break;

    case ST_SEARCH: {
      // Degraded (too many ToF dead): blind spin-search — in-place rotation is
      // the safest way to stumble back onto the opponent, and it can never
      // drive off the dohyo.
      if (degraded) {
        motorsSet(SEARCH_SPEED_PCT, -SEARCH_SPEED_PCT);
        break;
      }
      int8_t bearing = targetBearing();
      if (bearing != 127) {
        lastSeenMs = millis();
        gotoState((bearing >= -TARGET_ON_AXIS_BAND && bearing <= TARGET_ON_AXIS_BAND)
                      ? ST_ATTACK : ST_FLANK);
        break;
      }
      searchUpdate(STRATEGIES[strategyIndex].search);
      break;
    }

    case ST_ATTACK: {
      // Blade contact beats everything else in this state: both wheels full
      // ahead, no steering trim, and no switch to FLANK. lastSeenMs is stamped
      // because the opponent is very much visible — without it the lost-target
      // timeout would eventually pull the robot out of its own shove.
      if (sensorsContact()) {
        motorsSet(PUSH_CONTACT_SPEED_PCT, PUSH_CONTACT_SPEED_PCT);
        lastSeenMs = millis();
        break;
      }

      int8_t bearing = degraded ? 127 : targetBearing();
      if (bearing == 127) {
        if (millis() - lastSeenMs > ATTACK_TARGET_LOST_MS) {
          gotoState(ST_SEARCH); searchModeStart = millis();
        } else {
          // Push on blind momentum: degraded means the sensor bus is
          // untrustworthy, so use a deliberately cautious 40%.
          int push = degraded ? 40 : attackBaseSpeed(0xFFFF, false);
          motorsSet(push, push);
        }
      } else if (bearing < -TARGET_FLANK_BAND || bearing > TARGET_FLANK_BAND) {
        // Off by more than the outer threshold: pivot to re-center. Anything
        // inside TARGET_FLANK_BAND stays in ATTACK (that dead zone is the
        // hysteresis against FLANK).
        gotoState(ST_FLANK);
      } else {
        attackUpdate();
      }
      break;
    }

    case ST_FLANK: {
      if (degraded) { gotoState(ST_SEARCH); searchModeStart = millis(); break; }
      int8_t bearing = targetBearing();
      if (bearing == 127) {
        if (millis() - lastSeenMs > ATTACK_TARGET_LOST_MS) {
          gotoState(ST_SEARCH); searchModeStart = millis();
        } else {
          motorsStop();
        }
      } else if (bearing >= -TARGET_ON_AXIS_BAND && bearing <= TARGET_ON_AXIS_BAND) {
        gotoState(ST_ATTACK);
      } else {
        flankUpdate();
      }
      break;
    }

    case ST_EVADE:
      evadeUpdate();
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
