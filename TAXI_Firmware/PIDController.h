/**
 * PIDController.h — PD Controller with Hybrid Gain Scheduling
 * 
 * Adapted from Black Magic PID v2 (Soft) and v3 (Sharp) with improvements:
 *   - Distance-based weighting instead of binary 0/1
 *   - Hybrid gain scheduling: aggressive lock-on → smooth tracking
 *   - Anti-flag protection (dynamic outer weight reduction)
 *   - Distance-gated attack zones (red/yellow/green)
 * 
 * WHY PD NOT PID (from Phase 2/3 analysis):
 *   Integral windup is a known problem when opponent suddenly disappears.
 *   The "last known position" memory pattern handles sustained tracking
 *   better than an I-term would in mini sumo's dynamic engagements.
 * 
 * REFERENCE VALUES (from Black Magic PID):
 *   Soft:  Kp=28.5, Kd=15.0  → smooth, minimal oscillation
 *   Sharp: Kp=35.0, Kd=10.0  → aggressive, fast lock-on
 *   Our hybrid: both, switched by engagement duration
 */

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"

// ============================================================================
// PID STATE
// ============================================================================

struct PIDState {
    float kp;               // Current proportional gain
    float kd;               // Current derivative gain
    float error;            // Current weighted error
    float lastError;        // Previous error (for derivative)
    float pidOutput;        // Computed output (motor speed delta)
    uint32_t detectTime;    // When opponent was first detected (ms)
    bool opponentDetected;  // True if opponent was seen this cycle
};

static PIDState pidState = {
    PID_KP_LOCK,            // Start aggressive
    PID_KD_LOCK,
    0.0f, 0.0f, 0.0f,
    0, false
};

// ============================================================================
// PID COMPUTATION
// ============================================================================

/**
 * Compute PD output and update motor speeds.
 * 
 * Algorithm:
 *   1. Calculate weighted error from all healthy M60F sensors
 *   2. Apply anti-flag weight reduction if front sensors active
 *   3. Determine attack zone from closest front sensor distance
 *   4. Apply hybrid gain scheduling (lock-on vs tracking)
 *   5. Compute PD output: Kp*error + Kd*(error - lastError)
 *   6. Apply base speed + pidOutput to motors
 * 
 * @return true if opponent is actively being tracked
 */
bool pidUpdate() {
    // ---- Step 1: Read sensor values ----
    // M60F sensor layout: [0]=Front-Center, [1]=Front-Left 25°, [2]=Front-Right 25°
    //                     [3]=Left 90°, [4]=Right 90°
    
    float error = 0.0f;
    float totalWeight = 0.0f;
    bool frontActive = false;
    uint16_t closestFront = M60F_INVALID_DIST;
    
    // Direction weights: left-to-right spatial positioning
    // M3(Left)=-4, M1(FL-25°)=-2, M0(Center)=0, M2(FR-25°)=+2, M4(Right)=+4
    const float dirWeight[5] = M60F_WEIGHT_DIRECTION;
    
    // Determine if front sensors see opponent (for anti-flag)
    if (m60fDistances[0] != M60F_INVALID_DIST && m60fDistances[0] < ZONE_GREEN_MAX) {
        frontActive = true;
        closestFront = min(closestFront, m60fDistances[0]);
    }
    if (m60fDistances[1] != M60F_INVALID_DIST && m60fDistances[1] < ZONE_GREEN_MAX) {
        frontActive = true;
        closestFront = min(closestFront, m60fDistances[1]);
    }
    if (m60fDistances[2] != M60F_INVALID_DIST && m60fDistances[2] < ZONE_GREEN_MAX) {
        frontActive = true;
        closestFront = min(closestFront, m60fDistances[2]);
    }
    
    // ---- Step 2: Calculate weighted error ----
    for (uint8_t i = 0; i < M60F_COUNT; i++) {
        // Skip unhealthy or out-of-range sensors
        if (m60fHealth[i] < HEALTH_FAIL_THRESHOLD) continue;
        if (m60fDistances[i] == M60F_INVALID_DIST) continue;
        if (m60fDistances[i] > ZONE_GREEN_MAX) continue;
        
        // Determine sensor weight (anti-flag logic)
        float sensorWeight;
        if (i == 0 || i == 1 || i == 2) {
            // Front sensors: always inner weight
            sensorWeight = M60F_WEIGHT_INNER;
        } else {
            // Side sensors (indices 3, 4): reduce weight if front active
            sensorWeight = frontActive ? 
                M60F_WEIGHT_OUTER_REDUCED : M60F_WEIGHT_OUTER_NORMAL;
        }
        
        // Inverse distance weighting: closer = more influence
        // Clamp distance to avoid division by zero
        float dist = max((float)m60fDistances[i], 20.0f);
        float distWeight = 300.0f / dist;  // 1.0 at 300mm, 3.0 at 100mm
        
        error += dirWeight[i] * sensorWeight * distWeight;
        totalWeight += sensorWeight * distWeight;
    }
    
    // Normalize error
    if (totalWeight > 0.0f) {
        error /= totalWeight;
    }
    
    // ---- Step 3: Determine if opponent is detected ----
    bool opponentDetected = (totalWeight > 0.0f);
    
    if (!opponentDetected) {
        // No opponent — reset PID state
        pidState.opponentDetected = false;
        pidState.lastError = 0.0f;
        pidState.error = 0.0f;
        pidState.pidOutput = 0.0f;
        return false;
    }
    
    // ---- Step 4: Gain scheduling ----
    uint32_t now = millis();
    
    if (!pidState.opponentDetected) {
        // First detection — enter lock-on phase
        pidState.opponentDetected = true;
        pidState.detectTime = now;
        pidState.kp = PID_KP_LOCK;
        pidState.kd = PID_KD_LOCK;
    } else {
        // Already tracking — transition from lock-on to tracking
        uint32_t elapsed = now - pidState.detectTime;
        
        if (elapsed < PID_LOCK_DURATION_MS) {
            // Still in lock-on phase
            pidState.kp = PID_KP_LOCK;
            pidState.kd = PID_KD_LOCK;
        } else if (elapsed < PID_LOCK_DURATION_MS + PID_FADE_DURATION_MS) {
            // Transitioning: linear interpolation
            float fadeProgress = (float)(elapsed - PID_LOCK_DURATION_MS) / PID_FADE_DURATION_MS;
            pidState.kp = PID_KP_LOCK + (PID_KP_TRACK - PID_KP_LOCK) * fadeProgress;
            pidState.kd = PID_KD_LOCK + (PID_KD_TRACK - PID_KD_LOCK) * fadeProgress;
        } else {
            // Tracking phase
            pidState.kp = PID_KP_TRACK;
            pidState.kd = PID_KD_TRACK;
        }
    }
    
    // ---- Step 5: PD formula ----
    pidState.error = error;
    pidState.pidOutput = (pidState.kp * error) + (pidState.kd * (error - pidState.lastError));
    pidState.lastError = error;
    
    // ---- Step 6: Determine base speed from attack zone ----
    uint8_t baseSpeed;
    if (closestFront <= ZONE_RED_MAX) {
        baseSpeed = ZONE_SPEED_RED;       // Full attack
    } else if (closestFront <= ZONE_YELLOW_MAX) {
        baseSpeed = ZONE_SPEED_YELLOW;    // Approach
    } else {
        baseSpeed = ZONE_SPEED_GREEN;     // Cautious
    }
    
    // ---- Step 7: Apply to motors ----
    int16_t leftSpeed  = (int16_t)baseSpeed + (int16_t)pidState.pidOutput;
    int16_t rightSpeed = (int16_t)baseSpeed - (int16_t)pidState.pidOutput;
    
    // Constrain and send
    leftSpeed  = constrain(leftSpeed,  PID_MIN_SPEED, PID_MAX_SPEED);
    rightSpeed = constrain(rightSpeed, PID_MIN_SPEED, PID_MAX_SPEED);
    
    motorsSet((int8_t)leftSpeed, (int8_t)rightSpeed);
    
    return true;
}

/**
 * Reset PID state when opponent is lost.
 * Called when transitioning from ATTACK to SEARCH.
 */
void pidReset() {
    pidState.opponentDetected = false;
    pidState.lastError = 0.0f;
    pidState.error = 0.0f;
    pidState.pidOutput = 0.0f;
}

/**
 * Check if any opponent sensor is detecting (front or side).
 * Used to determine if we should be in ATTACK/FLANK vs SEARCH.
 * 
 * @return 0 = no opponent, 1 = front opponent, 2 = side only (flank)
 */
uint8_t pidOpponentType() {
    bool frontSeen = false;
    bool sideSeen = false;
    
    // Check front sensors (indices 0, 1, 2)
    for (uint8_t i = 0; i <= 2; i++) {
        if (m60fHealth[i] >= HEALTH_FAIL_THRESHOLD &&
            m60fDistances[i] != M60F_INVALID_DIST &&
            m60fDistances[i] <= ZONE_GREEN_MAX) {
            frontSeen = true;
            break;
        }
    }
    
    // Check side sensors (indices 3, 4)
    for (uint8_t i = 3; i <= 4; i++) {
        if (m60fHealth[i] >= HEALTH_FAIL_THRESHOLD &&
            m60fDistances[i] != M60F_INVALID_DIST &&
            m60fDistances[i] <= ZONE_GREEN_MAX) {
            sideSeen = true;
            break;
        }
    }
    
    if (frontSeen) return 1;    // Front opponent → full ATTACK
    if (sideSeen)  return 2;    // Side only → FLANK
    return 0;                    // No opponent
}

#endif // PIDCONTROLLER_H
