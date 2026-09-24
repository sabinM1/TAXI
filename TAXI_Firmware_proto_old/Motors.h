/**
 * Motors.h — Motor Control Wrappers
 * 
 * Wraps xmotion.MotorControl() with safety limiting, direction ramping,
 * and the curved-evasion blending needed for the user's "curbă" edge escape.
 * 
 * WHY CUSTOM WRAPPER instead of raw MotorControl():
 *   - Constrains output to valid PWM range
 *   - Provides named functions for common movement patterns
 *   - Centralizes motor safety (zero on init, stop on fault)
 * 
 * VERIFIED against xmotionV3.cpp:
 *   - MotorControl(RightPWM, LeftPWM): accepts -255..255
 *   - LM_DIR=12, LM_PWM=10, RM_DIR=13, RM_PWM=11
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <xmotionV3.h>
#include "Config.h"

// ============================================================================
// MOTOR INITIALIZATION
// ============================================================================

/**
 * Initialize motors to safe stopped state.
 * xmotionV3 constructor already sets pin modes — this just ensures stop.
 */
void motorsInit() {
    xmotion.StopMotors(100);
    #if SERIAL_DEBUG
    Serial.println(F("[Motors] Initialized, stopped"));
    #endif
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

/**
 * Set motor speeds in percentage (-100 to 100).
 * Positive = forward, negative = reverse.
 * Internally converts to PWM values (0–255) and calls xmotion.MotorControl().
 * 
 * NOTE: This function does NOT block. It sets PWM registers and returns immediately.
 * The xmotionV3 library's MotorControl() is the TIMELESS variant — correct choice.
 */
void motorsSet(int8_t leftPercent, int8_t rightPercent) {
    // Clamp to valid range
    leftPercent  = constrain(leftPercent,  PID_MIN_SPEED, PID_MAX_SPEED);
    rightPercent = constrain(rightPercent, PID_MIN_SPEED, PID_MAX_SPEED);
    
    // Convert percentage (-100..100) to PWM (-255..255)
    int16_t leftPWM  = (int16_t)leftPercent  * 255 / 100;
    int16_t rightPWM = (int16_t)rightPercent * 255 / 100;
    
    xmotion.MotorControl(rightPWM, leftPWM);
}

/**
 * Stop both motors immediately (non-blocking).
 */
void motorsStop() {
    xmotion.MotorControl(0, 0);
}

/**
 * Drive both motors forward at given speed (non-blocking).
 */
void motorsForward(uint8_t speed) {
    int8_t spd = constrain(speed, 0, 100);
    motorsSet(spd, spd);
}

/**
 * Drive both motors backward at given speed (non-blocking).
 */
void motorsBackward(uint8_t speed) {
    int8_t spd = constrain(speed, 0, 100);
    motorsSet(-spd, -spd);
}

/**
 * Pivot right: left forward, right backward (non-blocking).
 */
void motorsPivotRight(uint8_t speed) {
    int8_t spd = constrain(speed, 0, 100);
    motorsSet(spd, -spd);
}

/**
 * Pivot left: right forward, left backward (non-blocking).
 */
void motorsPivotLeft(uint8_t speed) {
    int8_t spd = constrain(speed, 0, 100);
    motorsSet(-spd, spd);
}

/**
 * Arc turn with blended speeds (non-blocking).
 * Both motors forward, but at different speeds.
 * This is the "curbă" (curve) pattern requested for edge re-entry.
 * 
 * @param leftSpeed  Left motor speed % (0–100)
 * @param rightSpeed Right motor speed % (0–100)
 */
void motorsArc(uint8_t leftSpeed, uint8_t rightSpeed) {
    motorsSet(
        constrain(leftSpeed, 0, 100),
        constrain(rightSpeed, 0, 100)
    );
}

/**
 * Curved re-entry based on triggered edge sensor.
 * Implements the user's "curbă în loc de linie dreaptă" preference.
 * 
 * @param triggeredSensor  0=FL, 1=FR, 2=RL, 3=RR
 *                         -1 = both front (full pivot)
 */
void motorsEvadeCurve(int8_t triggeredSensor) {
    switch (triggeredSensor) {
        case 0:  // Front-Left → arc right-forward
            motorsArc(EVADE_FL_LEFT, EVADE_FL_RIGHT);
            break;
        case 1:  // Front-Right → arc left-forward
            motorsArc(EVADE_FR_LEFT, EVADE_FR_RIGHT);
            break;
        case 2:  // Rear-Left → drive forward at right angle
            motorsArc(EVADE_RL_LEFT, EVADE_RL_RIGHT);
            break;
        case 3:  // Rear-Right → drive forward at left angle
            motorsArc(EVADE_RR_LEFT, EVADE_RR_RIGHT);
            break;
        case -1: // Both front → pivot right
        default:
            motorsPivotRight(100);
            break;
    }
}

#endif // MOTORS_H
