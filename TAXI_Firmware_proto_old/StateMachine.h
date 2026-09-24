/**
 * StateMachine.h — Combat State Machine
 * 
 * Implements the state machine described in docs/SYSTEM_ARCHITECTURE.md:
 *   IDLE → TACTIC → SEARCH ↔ ATTACK
 *                      ↕        ↓
 *                    FLANK    EVADE (overrides all)
 * 
 * Priority chain (highest to lowest):
 *   1. EVADE  — line sensor triggered (safety override)
 *   2. ATTACK — front opponent detected (PD tracking)
 *   3. FLANK  — side opponent only (arc toward)
 *   4. SEARCH — no opponent (sweep-search)
 * 
 * WHY THIS DESIGN (from Phase 3 decisions):
 *   - Q5: Multiple strategies via DIP switches (some reactive, some active search)
 *   - Q6: Curved re-entry paths for edge evasion
 *   - Q12: 3 front + 2 side M60F (no rear sensor)
 */

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "PIDController.h"
#include "Tactics.h"

// ============================================================================
// STATE ENUM
// ============================================================================

enum RobotState : uint8_t {
    STATE_IDLE = 0,     // Waiting for start signal
    STATE_TACTIC,       // Executing opening tactic
    STATE_SEARCH,       // Searching for opponent
    STATE_ATTACK,       // PD-controlled attack
    STATE_FLANK,        // Side opponent tracking
    STATE_EVADE,        // Edge escape (overrides everything)
    STATE_STOP          // Emergency stop / match end
};

// ============================================================================
// STATE MACHINE GLOBALS
// ============================================================================

static RobotState currentState = STATE_IDLE;
static RobotState previousState = STATE_IDLE;
static uint32_t stateEntryTime = 0;       // When current state was entered
static uint32_t searchStartTime = 0;      // When SEARCH began (for timeout)
static uint32_t lastOpponentSeen = 0;     // Last time any opponent was detected
static int8_t lastKnownDirection = 0;     // -1=left, 0=forward, 1=right
static uint8_t evadeTriggerSensor = 0;    // Which QRE triggered EVADE
static uint32_t evadeStartTime = 0;       // When EVADE began
static uint8_t evadePhase = 0;            // 0=reverse, 1=curve/pivot

// ============================================================================
// STATE TRANSITIONS
// ============================================================================

/**
 * Transition to a new state. Records entry time, resets phase counters.
 */
void stateTransition(RobotState newState) {
    if (newState == currentState) return;
    
    previousState = currentState;
    currentState = newState;
    stateEntryTime = millis();
    evadePhase = 0;
    
    #if SERIAL_DEBUG
    const char* stateNames[] = {"IDLE","TACTIC","SEARCH","ATTACK","FLANK","EVADE","STOP"};
    Serial.print(F("[State] "));
    Serial.print(stateNames[previousState]);
    Serial.print(F(" → "));
    Serial.println(stateNames[currentState]);
    #endif
}

// ============================================================================
// STATE HANDLERS
// ============================================================================

/**
 * IDLE state: Wait for START-STOP module to trigger.
 * 
 * Hardware check:
 *   - START-STOP on A0 (per user Q7 requirement)
 *   - Reads HIGH when competition start signal detected
 *   - Debounced for START_SIGNAL_DEBOUNCE_MS
 */
void stateIdle() {
    static uint32_t signalStable = 0;
    static bool lastSignal = false;
    
    bool signal = digitalRead(START_STOP_PIN);
    
    if (signal && !lastSignal) {
        signalStable = millis();
    }
    
    if (signal && (millis() - signalStable) > START_SIGNAL_DEBOUNCE_MS) {
        // Start signal confirmed — enter TACTIC
        stateTransition(STATE_TACTIC);
    }
    
    lastSignal = signal;
    
    // Show sensor health on LED ring while waiting
    // (handled by LEDRing update — called from main loop)
}

/**
 * TACTIC state: Execute DIP-switch-selected opening pattern.
 * 
 * Calls the tactic handler from Tactics.h which runs the programmed sequence.
 * When tactic completes (returns true), transition to SEARCH.
 * 
 * Tactic implementation is in Tactics.h — separated for the HTML strategy GUI
 * to eventually generate this file.
 */
void stateTactic() {
    bool tacticDone = tacticExecute();
    
    if (tacticDone) {
        searchStartTime = millis();
        stateTransition(STATE_SEARCH);
    }
    
    // Still allow edge detection during tactic
    if (qreAnyEdge() >= 0) {
        evadeTriggerSensor = qreAnyEdge();
        stateTransition(STATE_EVADE);
    }
}

/**
 * SEARCH state: Sweep-search for opponent.
 * 
 * Uses distance-adaptive arc width:
 *   - If opponent was recently seen (<800ms ago): tight arcs toward last direction
 *   - If opponent was seen longer ago: wider arcs
 *   - If never seen: spin-search (rotate in place)
 * 
 * Distance-gated (Q5 Option B strategy): robot actively moves and searches.
 * For Option A strategies (purely reactive), SEARCH would spin in place.
 * Strategy selection determines search behavior.
 */
void stateSearch() {
    // Check if opponent appeared during search
    uint8_t oppType = pidOpponentType();
    if (oppType == 1) {
        stateTransition(STATE_ATTACK);
        return;
    }
    if (oppType == 2) {
        stateTransition(STATE_FLANK);
        return;
    }
    
    // Determine search speed from trimpot
    uint8_t searchSpeed = map(xmotion.Trimpot(), 0, 1023, 30, 100);
    searchSpeed = constrain(searchSpeed, 30, 100);
    
    uint32_t now = millis();
    uint32_t searchElapsed = now - searchStartTime;
    uint32_t sinceLastSeen = now - lastOpponentSeen;
    
    // If we've been searching too long, escalate to spin-search
    if (searchElapsed > SEARCH_TIMEOUT_MS) {
        // Rotate in place — full sweep
        motorsPivotRight(searchSpeed);
        return;
    }
    
    // Use last known direction memory
    if (sinceLastSeen < LAST_KNOWN_TIMEOUT_MS) {
        // Recent contact — arc toward last known direction
        if (lastKnownDirection < 0) {
            // Was going left — arc left
            motorsArc(searchSpeed * 0.5, searchSpeed);
        } else if (lastKnownDirection > 0) {
            // Was going right — arc right
            motorsArc(searchSpeed, searchSpeed * 0.5);
        } else {
            // Was going forward — continue forward
            motorsForward(searchSpeed);
        }
    } else {
        // No recent contact — wide sweep pattern
        // Alternate direction every SEARCH_ARC_DURATION_MS
        uint32_t phase = (searchElapsed / SEARCH_ARC_DURATION_MS) % 4;
        switch (phase) {
            case 0: motorsArc(searchSpeed, searchSpeed * SEARCH_ARC_RATIO); break;  // Arc right
            case 1: motorsForward(searchSpeed); break;                              // Forward
            case 2: motorsArc(searchSpeed * SEARCH_ARC_RATIO, searchSpeed); break;  // Arc left
            case 3: motorsForward(searchSpeed); break;                              // Forward
        }
    }
}

/**
 * ATTACK state: PD-controlled opponent tracking.
 * 
 * Calls pidUpdate() which handles:
 *   - Weighted error calculation
 *   - Anti-flag protection
 *   - Hybrid gain scheduling
 *   - Distance-gated attack zones
 *   - Motor output
 * 
 * Records last known direction for SEARCH memory.
 */
void stateAttack() {
    bool tracking = pidUpdate();
    
    if (!tracking) {
        // Lost opponent — check if we should keep attacking or search
        if (millis() - lastOpponentSeen > 500) {
            // Lost for >500ms → back to SEARCH
            pidReset();
            stateTransition(STATE_SEARCH);
            return;
        }
        // Briefly lost — hold last known behavior for a moment
        return;
    }
    
    // Update last known direction
    if (pidState.error < -1.0f) {
        lastKnownDirection = -1;  // Turning left
    } else if (pidState.error > 1.0f) {
        lastKnownDirection = 1;   // Turning right
    } else {
        lastKnownDirection = 0;   // Centered
    }
    
    lastOpponentSeen = millis();
}

/**
 * FLANK state: Side opponent detected, arc toward them.
 * 
 * Uses side M60F sensors (indices 3=Left, 4=Right) to determine arc direction.
 * Front sensors are checked as well — if front opponent appears, upgrade to ATTACK.
 */
void stateFlank() {
    // Check if front opponent appeared (upgrade to ATTACK)
    uint8_t oppType = pidOpponentType();
    if (oppType == 1) {
        stateTransition(STATE_ATTACK);
        return;
    }
    if (oppType == 0) {
        // Lost opponent entirely
        lastOpponentSeen = millis() - 600;  // Will trigger SEARCH transition
        stateTransition(STATE_SEARCH);
        return;
    }
    
    // Still only side opponent — arc toward it
    bool leftSeen = (m60fHealth[3] >= HEALTH_FAIL_THRESHOLD &&
                     m60fDistances[3] != M60F_INVALID_DIST &&
                     m60fDistances[3] <= ZONE_GREEN_MAX);
    bool rightSeen = (m60fHealth[4] >= HEALTH_FAIL_THRESHOLD &&
                      m60fDistances[4] != M60F_INVALID_DIST &&
                      m60fDistances[4] <= ZONE_GREEN_MAX);
    
    if (leftSeen && !rightSeen) {
        // Opponent on left — arc left
        motorsArc(PID_BASE_SPEED_FLANK * 0.4, PID_BASE_SPEED_FLANK);
        lastKnownDirection = -1;
    } else if (rightSeen && !leftSeen) {
        // Opponent on right — arc right
        motorsArc(PID_BASE_SPEED_FLANK, PID_BASE_SPEED_FLANK * 0.4);
        lastKnownDirection = 1;
    } else {
        // Both sides or ambiguous — forward
        motorsForward(PID_BASE_SPEED_FLANK);
    }
    
    lastOpponentSeen = millis();
}

/**
 * EVADE state: Edge escape with curved re-entry.
 * 
 * Implements the user's "curbă" (curve) preference from Q6.
 * Phase 0: Reverse briefly
 * Phase 1: Execute curved re-entry path
 * 
 * If opponent appears during evasion, can transition to ATTACK mid-curve.
 * If both front and rear sensors trigger, both-front response takes priority.
 */
void stateEvade() {
    uint32_t evElapsed = millis() - evadeStartTime;
    
    // Determine which sensors are triggering
    int8_t trigger = -2;  // -2 = no trigger, -1 = both front, 0-3 = specific
    
    if (qreBothFrontEdge()) {
        trigger = -1;  // Both front
    } else if (qreIsEdge(0)) {
        trigger = 0;   // Front-Left
    } else if (qreIsEdge(1)) {
        trigger = 1;   // Front-Right
    } else if (qreIsEdge(2)) {
        trigger = 2;   // Rear-Left
    } else if (qreIsEdge(3)) {
        trigger = 3;   // Rear-Right
    }
    
    // If all sensors are clear, go back to SEARCH
    if (trigger == -2 && evElapsed > EVADE_CURVE_DURATION_MS + EVADE_REVERSE_MS) {
        stateTransition(STATE_SEARCH);
        searchStartTime = millis();
        return;
    }
    
    // Phase 0: Reverse
    if (evadePhase == 0 && evElapsed < EVADE_REVERSE_MS) {
        motorsBackward(EVADE_REVERSE_SPEED);
        return;
    }
    
    // Transition to Phase 1
    if (evadePhase == 0) {
        evadePhase = 1;
    }
    
    // Phase 1: Curved re-entry (or pivot for both-front)
    if (evadePhase == 1 && evElapsed < EVADE_REVERSE_MS + EVADE_CURVE_DURATION_MS) {
        if (trigger >= -1) {
            motorsEvadeCurve(trigger);
        } else {
            // Lost trigger — default to forward
            motorsForward(EVADE_REVERSE_SPEED);
        }
        return;
    }
    
    // Phase 1 complete — check if opponent appeared during evasion
    if (pidOpponentType() == 1) {
        stateTransition(STATE_ATTACK);
        return;
    }
    
    // Otherwise go to SEARCH
    stateTransition(STATE_SEARCH);
    searchStartTime = millis();
}

/**
 * STOP state: Motors off. Entered on critical failure or match timeout.
 * Requires power cycle to exit.
 */
void stateStop() {
    motorsStop();
}

// ============================================================================
// MAIN STATE MACHINE UPDATE
// ============================================================================

/**
 * Run one iteration of the state machine.
 * Called every main loop iteration. Never blocks.
 * 
 * Priority: EVADE overrides everything (checked first).
 * Then delegates to the current state's handler.
 */
void stateMachineUpdate() {
    // ---- Highest priority: Edge detection (overrides everything) ----
    int8_t edgeSensor = qreAnyEdge();
    if (edgeSensor >= 0 && currentState != STATE_EVADE && currentState != STATE_IDLE) {
        evadeTriggerSensor = edgeSensor;
        evadeStartTime = millis();
        evadePhase = 0;
        pidReset();  // Stop attack if we were attacking
        stateTransition(STATE_EVADE);
    }
    
    // ---- Failsafe: Check degradation level ----
    uint8_t healthyM60F = m60fHealthyCount();
    if (healthyM60F <= 2 && currentState != STATE_IDLE && currentState != STATE_STOP) {
        // Degraded-2: too many sensor failures → safe search only
        if (currentState == STATE_ATTACK || currentState == STATE_FLANK) {
            stateTransition(STATE_SEARCH);
        }
    }
    
    // ---- Delegate to current state ----
    switch (currentState) {
        case STATE_IDLE:   stateIdle();   break;
        case STATE_TACTIC: stateTactic(); break;
        case STATE_SEARCH: stateSearch(); break;
        case STATE_ATTACK: stateAttack(); break;
        case STATE_FLANK:  stateFlank();  break;
        case STATE_EVADE:  stateEvade();  break;
        case STATE_STOP:   stateStop();   break;
    }
}

/**
 * Initialize the state machine.
 * Called once from setup() after sensor init.
 */
void stateMachineInit() {
    currentState = STATE_IDLE;
    stateEntryTime = millis();
    
    #if SERIAL_DEBUG
    Serial.println(F("[StateMachine] Initialized — waiting for start signal"));
    #endif
}

#endif // STATEMACHINE_H
