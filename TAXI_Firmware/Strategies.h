/*
 * Strategies.h — DIP-switch selected opening tactics + combat search styles.
 *
 * DIP on D5/D6/D7 (internal pull-ups; switch DOWN reads 1 — see
 * xmotion_basics_101.md). index = (D7<<2)|(D6<<1)|D5, read once at start.
 *
 * Each opening is a timed step list (left%, right%, ms), then combat mode
 * takes over. Each strategy also picks its SEARCH style so openings pair
 * with a sensible hunt pattern.
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
 * A full opening: a short burst of timed motor steps executed right after
 * the start signal, then combat hands off to SEARCH. The search field
 * also picks which search pattern this opening leads into.
 */
struct Strategy {
  const TacticStep* steps;
  uint8_t stepCount;
  SearchStyle search;
};

// The 8 openings. Timings (ms) are calibrated on the bench; keep them
// short — sensors take over the moment the sequence finishes.
static const TacticStep s0[] = { {100,100,250} };                        // rush
static const TacticStep s1[] = { {100,-100,120}, {90,90,400} };          // turn right -> rush
static const TacticStep s2[] = { {-100,100,120}, {90,90,400} };          // turn left -> rush
static const TacticStep s3[] = { {-80,80,300} };                         // spin away (safe)
static const TacticStep s4[] = { {45,80,350}, {80,45,350} };             // arc right sweep
static const TacticStep s5[] = { {80,45,350}, {45,80,350} };             // arc left sweep
static const TacticStep s6[] = { {0,0,300}, {100,100,400} };             // wait -> rush (deceptive)
static const TacticStep s7[] = { {100,30,200}, {30,100,200}, {90,90,250} }; // wiggle trick

static const Strategy STRATEGIES[8] = {
  { s0, 1, SEARCH_SPIN  },   // 000
  { s1, 2, SEARCH_ARC   },   // 001
  { s2, 2, SEARCH_ARC   },   // 010
  { s3, 1, SEARCH_ARC   },   // 011
  { s4, 2, SEARCH_SWEEP },   // 100
  { s5, 2, SEARCH_SWEEP },   // 101
  { s6, 2, SEARCH_SPIN  },   // 110
  { s7, 3, SEARCH_SWEEP },   // 111
};

static uint8_t strategyIndex = 0;

/**
 * Read the DIP: ON = bit set (1). OFF = bit clear (0).
 *
 * Hardware quirk (xmotion_basics_101.md): switch DOWN = pin reads 1, and
 * DOWN is the OFF position. So digitalRead() returns 1 for OFF — we invert
 * so the resulting index matches the printed markings (000 = strategy 0).
 */
static uint8_t dipRead() {
  return ((!digitalRead(PIN_DIP1)) ? 1 : 0)
       | ((!digitalRead(PIN_DIP2)) ? 2 : 0)
       | ((!digitalRead(PIN_DIP3)) ? 4 : 0);
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
  Serial.print(F("[DIP] strategy ")); Serial.println(strategyIndex);
#endif
}

// Non-blocking opening executor ----------------------------------------
static uint8_t  tacticStep = 0;
static uint32_t tacticStepStart = 0;
static bool     tacticFirst = true;

/** Rewind the executor. Called by the state machine on entering ST_TACTIC. */
void tacticReset() { tacticStep = 0; tacticStepStart = millis(); tacticFirst = true; }

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
  if (tacticFirst) { tacticFirst = false; tacticStepStart = millis(); }
  motorsSet(st.leftPct, st.rightPct);

  if (millis() - tacticStepStart >= st.ms) {
    tacticStep++;
    tacticStepStart = millis();
  }
  return false;
}

#endif // STRATEGIES_H
