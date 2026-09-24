/*
 * Motor.h — minimal, percent-based, left/right-correct motor driver.
 *
 * Talks to the DRV8833-style PWM+DIR driver directly (no xmotionV3 calls):
 * positive speed = robot forward, negative = reverse. Hardware quirks
 * (the left motor mounted upside-down, the optional bench power cap)
 * live ONLY here so nobody has to think about them further up.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Config.h"

// Last commanded speeds, for the debug dump (not readback — just what we sent). 
static int motorLastL = 0, motorLastR = 0;

/**
 * Configure the 4 driver pins (left+right PWM and DIR) as outputs.
 * Does NOT move the robot: call motorsStop() right after from setup().
 */
static inline void motorInit() {
  pinMode(PIN_LM_PWM, OUTPUT);
  pinMode(PIN_LM_DIR, OUTPUT);
  pinMode(PIN_RM_PWM, OUTPUT);
  pinMode(PIN_RM_DIR, OUTPUT);
}

/**
 * Drive one motor.
 * @param pwmPin  speed pin
 * @param dirPin  direction pin
 * @param invert  true flips the DIR level (mounted-backwards motors)
 * @param p       signed percent -100..100, + = logical forward.
 *                Clamped; with MOTOR_BENCH_LIMIT_ENABLED it is clamped
 *                tighter for safe bench tests.
 */
static inline void motorWrite(uint8_t pwmPin, uint8_t dirPin, bool invert, int p) {
#if MOTOR_BENCH_LIMIT_ENABLED
  p = constrain(p, -MOTOR_BENCH_MAX_PCT, MOTOR_BENCH_MAX_PCT);
#else
  p = constrain(p, -100, 100);
#endif
  bool fwd = (p >= 0) ^ invert;     // logical forward after mounting flip
  digitalWrite(dirPin, fwd ? HIGH : LOW);
  analogWrite(pwmPin, map(abs(p), 0, 100, 0, 255));
}

/**
 * Drive both motors. leftPct/rightPct in -100..100, + = robot forward.
 * Non-blocking: sets PWM registers and returns immediately.
 */
static inline void motorsSet(int leftPct, int rightPct) {
  motorLastL = leftPct;
  motorLastR = rightPct;
  motorWrite(PIN_LM_PWM, PIN_LM_DIR, LM_INVERTED, leftPct);
  motorWrite(PIN_RM_PWM, PIN_RM_DIR, RM_INVERTED, rightPct);
}

/** Cut drive to both motors (coast). Safe default everywhere. */
static inline void motorsStop() {
  motorLastL = 0; motorLastR = 0;
  analogWrite(PIN_LM_PWM, 0);
  analogWrite(PIN_RM_PWM, 0);
}

/** Last command issued (for the debug dump) — clamping happens inside. */
static inline void motorsGetLast(int* l, int* r) { *l = motorLastL; *r = motorLastR; }

#endif // MOTOR_H
