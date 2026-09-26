/*
 * Motor.h — minimal, percent-based, left/right-correct motor driver.
 *
 * Driver wiring (XMotion V3): each channel is one PWM pin plus one DIR pin.
 *   LEFT  : PWM = D10, DIR = D12
 *   RIGHT : PWM = D11, DIR = D13
 * Positive speed = robot forward, negative = reverse. Hardware quirks (the
 * left motor mounted upside-down, the left channel's PWM timer) live ONLY
 * here so nobody has to think about them anywhere else.
 *
 * ---------------------------------------------------------------------------
 * WHY THE LEFT CHANNEL DOES NOT USE analogWrite()
 * ---------------------------------------------------------------------------
 * The M60F ToF bus runs on SingleWireSerial, a half-duplex software UART that
 * times its bits with a hardware timer and the input-capture pin. On the
 * ATmega32U4 the only capture pin wired up is D4 (ICP1), and that forces the
 * library onto Timer1 (SingleWireSerial_config.h: ICTIMER 1 -> TCCR1A/B,
 * OCR1A, ICR1, TIMER1_CAPT_vect). SingleWireSerial::begin() therefore does
 * `TCCR1A = 0` and re-arms Timer1 in CTC/Normal mode with its own prescaler.
 *
 * D10 is OC1B — Timer1's other compare output. As soon as the sensor bus is
 * begun, analogWrite(10, x) writes COM1B1 and OCR1B into a timer that is no
 * longer in a PWM mode, so the OC1B pin never toggles and the LEFT motor gets
 * no drive. (The RIGHT motor is on D11 = OC0A = Timer0, which the bus never
 * touches — that is why only the left side was dead.)
 *
 * Fix: drive D10 from Timer4 channel B instead. PB6 is multiplexed as both
 * OC1B (Timer1) and OC4B (Timer4) — ATmega32U4 datasheet, Port B alternate
 * functions — and Timer4's other outputs are D13/OC4A and D6/OC4D, neither of
 * which is a PWM output in this robot (D13 is the right-motor DIR, D6 is a
 * DIP switch). So channel B gives true hardware PWM on the existing wire, with
 * no interference with the ToF bus, no software PWM and no extra CPU load.
 * Timer4 is not used by anything else in this firmware.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Config.h"

#if !defined(__AVR_ATmega32U4__)
#error "Motor.h's left-channel PWM is written for the ATmega32U4 (XMotion V3 / Arduino Leonardo)."
#endif

// ---------------------------------------------------------------------------
// LEFT channel PWM: Timer4 channel B (OC4B) on PB6 = D10.
//
// PB6 is hard-wired to that one timer output, so the whole block only makes
// sense while PIN_LM_PWM stays on D10.
// ---------------------------------------------------------------------------
#if PIN_LM_PWM != 10
#error "Motor.h drives the left channel with Timer4 OC4B on D10 (PB6); rework this block if PIN_LM_PWM changes."
#endif

#define LM_PWM_TOP        255         // TOP is held in OCR4C: 8-bit counter -> 256 speed steps
                                      // (8-bit is what analogWrite() gave here anyway, and it
                                      //  works whether or not Timer4's 10-bit mode is enabled)
#define LM_PWM_CLKSEL     (_BV(CS42) | _BV(CS41) | _BV(CS40))
                                      // TCCR4B CS43:CS40 = 0111 = /64
                                      // -> 16 MHz / 64 = 250 kHz timer clock
                                      // -> PWM frequency = 250 kHz / 256 = 977 Hz
                                      // Same rate as the right channel (Timer0 /64,
                                      // for millis()) so both motors make the same
                                      // torque at part duty.

// Last commanded speeds, for the debug dump (not readback — just what we sent).
static int motorLastL = 0, motorLastR = 0;

/**
 * Configure the 4 driver pins and the left channel's Timer4 PWM hardware.
 * Does NOT move the robot: call motorsStop() right after from setup().
 */
static inline void motorInit() {
  pinMode(PIN_LM_PWM, OUTPUT);
  pinMode(PIN_LM_DIR, OUTPUT);
  pinMode(PIN_RM_PWM, OUTPUT);
  pinMode(PIN_RM_DIR, OUTPUT);

  // Left channel: take Timer4 over for channel B (OC4B -> D10).
  // Order matters: silence every Timer4 output and park the TOP register
  // before the pin is connected, so D10/D13/D6 never glitch on the way up.
  // Timer4 is left clocked from clkIO: the ATmega32U4 has no PLL-select bit for
  // it (PLLCSR only has PLOCK/PLLE, and those belong to the USB PLL — do not
  // touch them), so CS43:CS40 alone sets the PWM frequency.
  TCCR4A  = 0;                            // all Timer4 compare outputs disconnected
  TCCR4C  = 0;                            // channel D off -> D6 stays a plain input
  digitalWrite(PIN_LM_PWM, LOW);          // park the pin low before handing it to the timer
  TC4H    = 0;                            // 10-bit registers: keep the two MSBs clear
  OCR4C   = LM_PWM_TOP;                   // OCR4C holds the Fast PWM TOP value
  TCCR4B  = LM_PWM_CLKSEL;                // prescaler; dead-time bits stay 0
  TCCR4D  = 0;                            // mode 00 = Fast PWM, TOP = OCR4C
  TCCR4A  = _BV(PWM4B) | _BV(COM4B1);     // connect OC4B, non-inverting (clear on match, set at BOTTOM)
}

/**
 * Write the left channel's duty.
 * @param duty 0..LM_PWM_TOP, 0 = off, LM_PWM_TOP = full speed.
 * Timer4 latches OCR4B at the next TOP, so the output changes glitch-free.
 */
static inline void motorWriteLeftDuty(uint16_t duty) {
  if (duty > LM_PWM_TOP) duty = LM_PWM_TOP;
  TC4H  = 0;                              // 10-bit registers: MSBs clear, low byte follows
  OCR4B = (uint8_t)duty;
}

/**
 * Drive one motor.
 * @param pwmPin  speed pin. PIN_LM_PWM is served by Timer4 (see the file
 *                header); any other value must be an analogWrite()-capable pin.
 * @param dirPin  direction pin
 * @param invert  true flips the DIR level (mounted-backwards motors)
 * @param p       signed percent -100..100, + = logical forward.
 *                Clamped; with MOTOR_BENCH_LIMIT_ENABLED it is clamped
 *                tighter for safe bench tests.
 */
static inline void motorWrite(uint8_t pwmPin, uint8_t dirPin, bool invert, int p, uint8_t trimPct) {
#if MOTOR_BENCH_LIMIT_ENABLED
  p = constrain(p, -MOTOR_BENCH_MAX_PCT, MOTOR_BENCH_MAX_PCT);
#else
  p = constrain(p, -100, 100);
#endif
  bool fwd = (p >= 0) ^ invert;
  digitalWrite(dirPin, fwd ? HIGH : LOW);

  uint16_t duty = (uint16_t)map(abs(p), 0, 100, 0, 255);
  if (trimPct != 100) duty = (uint16_t)((uint32_t)duty * trimPct / 100);

  if (pwmPin == PIN_LM_PWM) motorWriteLeftDuty(duty);
  else                      analogWrite(pwmPin, duty);
}

/**
 * Drive both motors. leftPct/rightPct in -100..100, + = robot forward.
 * Non-blocking: sets PWM registers and returns immediately.
 */
static inline void motorsSet(int leftPct, int rightPct) {
  motorLastL = leftPct;
  motorLastR = rightPct;
  motorWrite(PIN_LM_PWM, PIN_LM_DIR, LM_INVERTED, leftPct, LM_TRIM_PCT);
  motorWrite(PIN_RM_PWM, PIN_RM_DIR, RM_INVERTED, rightPct, RM_TRIM_PCT);
}

/** Cut drive to both motors (coast). Safe default everywhere. */
static inline void motorsStop() {
  motorLastL = 0; motorLastR = 0;
  motorWriteLeftDuty(0);
  analogWrite(PIN_RM_PWM, 0);
}

/** Last command issued (for the debug dump) — clamping happens inside. */
static inline void motorsGetLast(int* l, int* r) { *l = motorLastL; *r = motorLastR; }

#endif // MOTOR_H
