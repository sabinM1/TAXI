/*
 * Strategies.h — DIP-switch selected opening tactics: types + executor.
 *
 * DIP on D5/D6/D7 (internal pull-ups; switch DOWN reads 1 — see
 * xmotion_basics_101.md). index = (D7<<2)|(D6<<1)|D5, read once at start.
 *
 * The opening DATA (the TacticStep arrays, the STRATEGIES[8] table — including
 * each opening's assumed start pose — and the debug names) lives in
 * StrategyData.h, which is included below. That file is the single source of
 * truth and is what tools/strategy-designer.html reads and writes — so the
 * designer never has to touch the executor logic here.
 *
 * ---------------------------------------------------------------------------
 * HOW AN OPENING WORKS
 * ---------------------------------------------------------------------------
 * Each opening is a timed step list (left%, right%, ms) played the instant the
 * start signal goes HIGH, then combat hands off to SEARCH. The openings are
 * OPEN-LOOP by design: at the start line the ToF picture is often empty or
 * unreliable, so a committed move is more predictable than a reactive one.
 *
 * The opening ALWAYS runs to completion. The only thing that can interrupt it
 * is the line-edge override (EVADE) in stateMachineUpdate(), which outranks
 * every state body — a clipped border aborts the opening, a detected opponent
 * does not.
 *
 * The step model is fully general for a differential drive:
 *   {100,100,ms}   straight forward
 *   {-100,-100,ms} straight reverse
 *   {100,-100,ms}  spin in place (right)
 *   {100, 30,ms}   arc right (outer wheel faster)
 *   {0,0,ms}       pause
 * so any researched opening (rush, turn-and-dash, flank, fake-out, hit-spin)
 * is just a list of these.
 *
 * ---------------------------------------------------------------------------
 * DESIGNING / TUNING THE OPENINGS
 * ---------------------------------------------------------------------------
 * Open tools/strategy-designer.html, import StrategyData.h, draw each opening
 * on a 77 cm dohyo and export it back. The designer has TWO calibration
 * numbers, both measured on the real ring:
 *   - straight speed (cm/s @ 100 % duty), and
 *   - turn rate (deg/s @ full differential, i.e. left +100 / right -100).
 * A robot's pivot rate is usually lower than its straight-line speed.
 */
#ifndef STRATEGIES_H
#define STRATEGIES_H

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"

enum SearchStyle : uint8_t { SEARCH_SPIN, SEARCH_ARC, SEARCH_SWEEP };

/**
 * One timed step of an opening tactic.
 * leftPct/rightPct in -100..100 (+ = forward), held for ms milliseconds.
 */
struct TacticStep { int8_t leftPct, rightPct; uint16_t ms; };

/**
 * Where the robot is placed on the dohyo before the start signal.
 * x/y in cm from the dohyo centre (+x right, +y toward the opponent),
 * heading in degrees (0 = up, + = clockwise).
 *
 * The opening is open-loop, so this does not steer the robot at run time —
 * it records the placement each opening assumes. It is printed at boot so
 * the operator knows where to set the robot down for the selected DIP code.
 */
struct StartPose { float x, y, heading; };

/**
 * A full opening: a short burst of timed motor steps executed right after
 * the start signal, then combat hands off to SEARCH. The search field
 * also picks which search pattern this opening leads into, and start
 * records the placement the opening assumes.
 *
 * handoffMs is 65% of the opening's total time, computed by the strategy
 * designer at export. Once that much of the opening has run, a close opponent
 * outranks the committed move: the opening is open-loop, but it must not drive
 * past a target it can already see (see the ST_TACTIC case).
 *
 * handoffMs == 0 is a sentinel meaning NEVER hand off: the opening runs fully
 * blind, ignoring the ToF for its whole duration. The edge flanks use it, since
 * their sweep must complete to end up behind the opponent.
 */
struct Strategy {
  const TacticStep* steps;
  uint8_t stepCount;
  SearchStyle search;
  StartPose start;
  uint16_t handoffMs;
};

// The opening data: TacticStep arrays, STRATEGIES[8], STRATEGY_NAMES[8].
#include "StrategyData.h"

static uint8_t strategyIndex = 0;

/**
 * Read the DIP: ON = bit set (1). OFF = bit clear (0).
 *
 * Hardware quirk (xmotion_basics_101.md): switch DOWN = pin reads 1, and
 * DOWN is the OFF position. So digitalRead() returns 1 for OFF — we invert
 * so the resulting index matches the printed markings (000 = strategy 0).
 *
 * Bit order: the switch markings read MSB-first (leftmost switch = bit 2), but
 * the pins are wired the other way round — PIN_DIP1 is the RIGHTMOST switch.
 * So PIN_DIP1 maps to bit 2 and PIN_DIP3 to bit 0, which makes the printed
 * marking (e.g. 100) match the resulting index (4), not its mirror (1).
 */
static uint8_t dipRead() {
  return ((!digitalRead(PIN_DIP1)) ? 4 : 0)
       | ((!digitalRead(PIN_DIP2)) ? 2 : 0)
       | ((!digitalRead(PIN_DIP3)) ? 1 : 0);
}

/**
 * Call once in setup(): arm the DIP pins with pull-ups and latch the
 * selected strategy. Changing the DIP mid-match has no effect.
 */
void strategiesInit() {
  pinMode(PIN_DIP1, INPUT_PULLUP);
  pinMode(PIN_DIP2, INPUT_PULLUP);
  pinMode(PIN_DIP3, INPUT_PULLUP);
  delay(2);
  strategyIndex = dipRead() & 7;
#if SERIAL_DEBUG
  Serial.print(F("[DIP] strategy ")); Serial.print(strategyIndex);
#if DEBUG_VERBOSE
  Serial.print(F(" (")); Serial.print(STRATEGY_NAMES[strategyIndex]); Serial.print(')');
#endif
  const StartPose& sp = STRATEGIES[strategyIndex].start;
  // Cast to int: whole-cm guidance is plenty, and it avoids pulling the
  // float-formatting code into the binary.
  Serial.print(F(" start (")); Serial.print((int)sp.x); Serial.print(F(", ")); Serial.print((int)sp.y);
  Serial.print(F(") cm ")); Serial.print((int)sp.heading); Serial.println(F(" deg"));
#endif
}

// Non-blocking opening executor ----------------------------------------
static uint8_t  tacticStep = 0;
static uint32_t tacticStepStart = 0;
static uint32_t tacticStartMs = 0;
static bool     tacticFirst = true;

/** Rewind the executor. Called by the state machine on entering ST_TACTIC. */
void tacticReset() { tacticStep = 0; tacticStepStart = millis(); tacticStartMs = millis(); tacticFirst = true; }

/**
 * Milliseconds since the opening actually started. 0 until the first
 * tacticRun() call, so a caller that checks before the opening has ticked
 * never sees a stale elapsed time.
 */
uint32_t tacticElapsedMs() { return tacticFirst ? 0 : (millis() - tacticStartMs); }

/**
 * Run one loop-tick of the opening. Holds the current step's motor values
 * until its duration expires, then advances. Motors keep their last command
 * between calls (PWM keeps running by itself).
 * @return true  when the whole sequence is done and the robot may hunt.
 */
bool tacticRun() {
  const Strategy& s = STRATEGIES[strategyIndex];
  if (tacticStep >= s.stepCount) { motorsStop(); return true; }

  const TacticStep& st = s.steps[tacticStep];
  if (tacticFirst) { tacticFirst = false; tacticStepStart = millis(); tacticStartMs = millis(); }
  motorsSet(st.leftPct, st.rightPct);

  if (millis() - tacticStepStart >= st.ms) {
    tacticStep++;
    tacticStepStart = millis();
  }
  return false;
}

#endif // STRATEGIES_H
