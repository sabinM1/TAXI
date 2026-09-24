/*
 * StateMachine.h — combat flow.
 *
 * States:  IDLE -> TACTIC -> SEARCH <-> ATTACK/FLANK, with EVADE override
 *          from any combat state, and STOP when the Start/Stop module drops.
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

// Start/stop debounce state. IMPORTANT: must start LOW (armed) — HIGH would
// fake an instant start before the debounce ever reads the real pin.
static bool    startRaw = LOW, startStable = LOW;
static uint32_t startChangeMs = 0;

// Search bookkeeping
static uint32_t searchModeStart = 0;
static int8_t   searchDir = 1;          // +1 turn right, -1 turn left
static uint32_t lastSeenMs = 0;

// Evade bookkeeping
static uint32_t evadeStart = 0;
static int8_t   evadeFrom = 0;

static bool isStarted() { return startStable == HIGH; }

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
 * Debounce the Start/Stop module. HIGH = fight, LOW = armed/stopped.
 * Call every loop() BEFORE anything that uses isStarted().
 */
static void startModuleUpdate() {
  bool raw = digitalRead(PIN_START);
  if (raw != startRaw) { startRaw = raw; startChangeMs = millis(); }
  if (raw != startStable && (millis() - startChangeMs) >= START_SIGNAL_DEBOUNCE_MS)
    startStable = raw;
}

// ------------------------- state helpers ------------------------------------

/**
 * Steer the robot while hunting. Three patterns:
 *   SPIN  — rotate in place (fastest sensor sweep, never leaves the ring)
 *   ARC   — wide arcs, covers ground while scanning
 *   SWEEP — alternating shallow arcs (figure-8-ish), most area covered
 * After SEARCH_PATTERN_TIMEOUT_MS without seeing anything, the pattern
 * mirrors (searchDir flips) so we never spiral forever in one direction.
 * Scaled by the trimpot master.
 */
static void searchUpdate(SearchStyle style) {
  uint8_t scale = trimpotScale();
  int sp = (SEARCH_SPEED_PCT * scale) / 100;
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
 * Finally scaled by the trimpot master.
 */
static int attackBaseSpeed(uint16_t mm, bool ok) {
  int base;
  if (!ok || mm >= ATTACK_FAR_DIST_MM)  base = ATTACK_SPEED_FAR_PCT;
  else if (mm <= ATTACK_CLOSE_DIST_MM)  base = ATTACK_SPEED_CLOSE_PCT;
  else base = (int)map((long)mm, ATTACK_CLOSE_DIST_MM, ATTACK_FAR_DIST_MM,
                       ATTACK_SPEED_CLOSE_PCT, ATTACK_SPEED_FAR_PCT);
  return (base * trimpotScale()) / 100;
}

/**
 * Chase + push: base speed from distance, plus proportional steering.
 * Bearing > 0 (opponent right) slows the right motor, and vice-versa.
 * Stamps lastSeenMs so the caller's lost-target timeout stays armed.
 */
static void attackUpdate() {
  int8_t bearing = sensorsOpponent();
  bool ok; uint16_t mm = sensorsClosestMm(&ok);
  lastSeenMs = millis();

  int base = attackBaseSpeed(mm, ok);

  // Proportional steer: bearing >0 = opponent right -> slow right motor.
  int corr = (int)((int32_t)bearing * base * ATTACK_STEER_GAIN_PCT / 10000);
  motorsSet(constrain(base + corr, -100, 100),
            constrain(base - corr, -100, 100));
}

/**
 * Opponent registered off-center: pivot toward the bearing until it is
 * re-centered (±15), then shove forward. 127 means lost — do nothing and
 * let the caller's timeout drop us back to SEARCH.
 */
static void flankUpdate() {
  // Opponent on a side sensor: turn toward it until it is in front.
  int8_t bearing = sensorsOpponent();
  if (bearing == 127) return;
  lastSeenMs = millis();

  if (bearing >= -15 && bearing <= 15) {
    motorsSet(FLANK_TURN_SPEED_PCT, FLANK_TURN_SPEED_PCT);   // lined up: push
  } else if (bearing > 0) {
    motorsSet(FLANK_TURN_SPEED_PCT, -FLANK_TURN_SPEED_PCT / 2);
  } else {
    motorsSet(-FLANK_TURN_SPEED_PCT / 2, FLANK_TURN_SPEED_PCT);
  }
}

/** Phase 1 of EVADE: the blind escape burst.
 *  Rear edge -> drive FORWARD (reversing would push us further off).
 *  Any front edge -> reverse hard.
 */
static void evadeEscapeBurst() {
  if (evadeFrom == 1) {
    // Rear edge: NEVER reverse further — drive forward with a gentle bias.
    if (searchDir > 0) motorsSet(80, 55); else motorsSet(55, 80);
  } else {
    motorsSet(EVADE_REVERSE_SPEED_PCT, EVADE_REVERSE_SPEED_PCT);
  }
}

/** Phase 2 of EVADE: once sensors confirm we left the edge, pivot away
 *  from whichever side tripped so we re-enter facing the ring centre.
 */
static void evadeTurnAway() {
  if (evadeFrom == 0)      motorsSet(70, -50);    // left edge  -> spin right
  else if (evadeFrom == 2) motorsSet(-50, 70);    // right edge -> spin left
  else motorsSet(searchDir > 0 ? 60 : -60,
                 searchDir > 0 ? -60 : 60);       // rear -> pivot home side
}

/**
 * EVADE program: EVADE_ESCAPE_MIN_MS blind burst, then up to
 * EVADE_TOTAL_MAX_MS of edge-aware escape/pivot, then hand control to
 * SEARCH. Entered exclusively via the edge override in stateMachineUpdate().
 */
static void evadeUpdate() {
  uint32_t t = millis() - evadeStart;

  // Phase 1: minimum blind escape burst (ignores sensors — can still be
  // rolling off the dohyo while the edge flickers on/off).
  if (t < EVADE_ESCAPE_MIN_MS) { evadeEscapeBurst(); return; }

  // Phase 2: if the edge persists keep escaping, else pivot back in-bounds.
  if (t < EVADE_TOTAL_MAX_MS) {
    if (sensorsAnyEdge()) evadeEscapeBurst();
    else                  evadeTurnAway();
    return;
  }

  motorsStop();
  gotoState(ST_SEARCH);
  searchModeStart = millis();
}

// ------------------------- main entry ---------------------------------------

/**
 * Combat brain. Call once per loop() AFTER sensorsUpdate(); drives motors
 * and mutates `state`. Priority each call:
 *   1. start module killed   -> STOP
 *   2. line edge             -> EVADE (from any fighting state, incl. TACTIC)
 *   3. state body
 */
void stateMachineUpdate() {
  startModuleUpdate();

  // Global stop: module released while fighting
  if (!isStarted() && state != ST_IDLE && state != ST_STOP) {
    gotoState(ST_STOP); motorsStop();
    return;
  }

  // EVADE override: any edge while fighting (including the opening tactic)
  if (sensorsAnyEdge() && state != ST_EVADE &&
      state != ST_IDLE && state != ST_STOP) {
    evadeFrom = sensorsEdgeIndex();
    evadeStart = millis();
    gotoState(ST_EVADE);
  }

  // Degraded fallback: too many ToF dead -> stay in safe spin-search.
  bool degraded = sensorsHealthyCount() < TOF_MIN_HEALTHY_SENSORS;

  // ToF front-priority scan only while pushing a target.
  sensorsSetAttackScan(state == ST_ATTACK || state == ST_FLANK);

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
      if (degraded) { motorsSet(50, -50); break; }   // blind spin-search
      int8_t bearing = sensorsOpponent();
      if (bearing != 127) {
        lastSeenMs = millis();
        gotoState((bearing >= -15 && bearing <= 15) ? ST_ATTACK : ST_FLANK);
        break;
      }
      searchUpdate(STRATEGIES[strategyIndex].search);
      break;
    }

    case ST_ATTACK: {
      int8_t bearing = degraded ? 127 : sensorsOpponent();
      if (bearing == 127) {
        if (millis() - lastSeenMs > ATTACK_TARGET_LOST_MS) {
          gotoState(ST_SEARCH); searchModeStart = millis();
        } else {
          int push = degraded ? 40
                              : attackBaseSpeed(0xFFFF, false);
          motorsSet(push, push);   // push on blind momentum
        }
      } else if (bearing < -15 || bearing > 15) {
        gotoState(ST_FLANK);        // opponent slid off center
      } else {
        attackUpdate();
      }
      break;
    }

    case ST_FLANK: {
      if (degraded) { gotoState(ST_SEARCH); searchModeStart = millis(); break; }
      int8_t bearing = sensorsOpponent();
      if (bearing == 127) {
        if (millis() - lastSeenMs > ATTACK_TARGET_LOST_MS) {
          gotoState(ST_SEARCH); searchModeStart = millis();
        } else {
          motorsStop();
        }
      } else if (bearing >= -15 && bearing <= 15) {
        gotoState(ST_ATTACK);
      } else {
        flankUpdate();
      }
      break;
    }

    case ST_EVADE:
      evadeUpdate();
      break;

    case ST_STOP:
    default:
      motorsStop();
      break;
  }
}

/** Current state for the ring/status display. */
uint8_t stateMachineState() { return (uint8_t)state; }

#endif // STATEMACHINE_H
