/*
 * Sensors.h — M60F ToF bus + QRE1113 line sensors (header-only).
 *
 * ToF: round-robin, ONE sensor read per sensorsUpdate() call so the combat
 * loop never blocks longer than ~2 ms on the shared bus.
 * All 5 channels are normalised to millimetres in tofMm[] (ID3 is mm,
 * the rest cm -> x10). 0 = invalid reading.
 *
 * Line: three QRE1113 modules (left A5, rear A4, right A1). Black interior
 * reads HIGH, white border reads LOW (see Config.h). Edge = raw BELOW the
 * sensor's white threshold; a Schmitt trigger latches it and only releases
 * once clearly back above the black threshold. Debounced: LINE_DEBOUNCE_SAMPLES
 * consecutive white reads confirm the edge.
 *
 * IMPORTANT (from Full_Robot_Test): finish all ToF reads BEFORE
 * FastLED.show() — the ring bit-bang disables interrupts and would corrupt
 * the serial bus. The state machine reads sensors first, updates the ring
 * last.
 */
#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <SingleWireSerial.h>
#include "Config.h"
#include "DistanceSensor.h"
#include "LineSensor.h"

// ---------------------------------------------------------------- ToF -------
// Index layout: [0]=id1 side right, [1]=id2 front right, [2]=id3 center,
//               [3]=id4 front left,  [4]=id5 side left.
uint16_t tofMm[TOF_SENSOR_COUNT];
bool     tofValid[TOF_SENSOR_COUNT];

// Bearing weights (left negative, right positive) matching the layout above.
static const int8_t TOF_WEIGHT[TOF_SENSOR_COUNT] = { 4, 2, 0, -2, -4 };

// The three FRONT sensors (ids 2, 3, 4) are indices 1..3. Only these are polled
// in attack-scan mode; everything else uses the full round-robin.
static inline bool tofIsFrontIndex(uint8_t i) { return i >= 1 && i <= 3; }

static SingleWireSerial tofBus(false);           // bus on D4 (Config: PIN_TOF_BUS)
static DistanceSensor tofSensors[TOF_SENSOR_COUNT] = {
  DistanceSensor(&tofBus, 1), DistanceSensor(&tofBus, 2),
  DistanceSensor(&tofBus, 3), DistanceSensor(&tofBus, 4),
  DistanceSensor(&tofBus, 5)
};
static uint8_t tofCursor = 0;
static bool    tofAttackScan = false;            // true -> only front 3 ids
static uint8_t frontCursor = 0;                  // cycles front idx 1,2,3

// Simple health counters per sensor (0..TOF_HEALTH_MAX).
static uint8_t tofHealth[TOF_SENSOR_COUNT];

// Number of sensors currently considered alive. A sensor is "healthy" when
// its health counter is at least half full — i.e. it has been answering
// consistently for the last few polls.
uint8_t sensorsHealthyCount() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++)
    if (tofHealth[i] >= TOF_HEALTH_MAX / 2) n++;
  return n;
}

/**
 * Switch the ToF poll pattern. In attack mode only the three front sensors
 * (ids 2, 3, 4) are cycled -> ~3x faster updates on the sensors that decide
 * the fight.
 *
 * Entering attack scan also INVALIDATES the two side readings. They are the
 * last values measured before the switch, and because sensorsOpponent()
 * weights every reading by inverse distance, one close side reading taken a
 * moment earlier would keep outvoting all three front sensors — dragging the
 * bearing off-centre and pushing ATTACK into a FLANK pivot against data that is
 * no longer being sampled. A reading nobody is refreshing must not steer.
 *
 * They become valid again on their next real poll (the round-robin refreshes
 * all five within ~5 loops ≈ 10 ms), and their health counters are untouched,
 * so the degraded-mode check is unaffected.
 */
void sensorsSetAttackScan(bool on) {
  if (on == tofAttackScan) return;            // only act on the transition
  tofAttackScan = on;
  if (!on) return;
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++)
    if (!tofIsFrontIndex(i)) tofValid[i] = false;
}

// --------------------------------------------------------------- Line -------
// Three QRE1113 modules. Raw ADC in lineRaw[]; latched edge flags below.
// Edge = white border = LOW reading, per-sensor thresholds in Config.h.
LineSensor lineSensL(PIN_LINE_L, LINE_LEFT_WHITE_THRESHOLD);
LineSensor lineSensR(PIN_LINE_R, LINE_RIGHT_WHITE_THRESHOLD);
LineSensor lineSensB(PIN_LINE_B, LINE_REAR_WHITE_THRESHOLD);

bool lineLeft = false, lineRear = false, lineRight = false;   // latched edge flags
uint16_t lineRaw[3];                                          // 0=L 1=rear 2=R
static uint8_t lineEdgeCount[3];                              // white-hit debounce

// -------------------------------------------------------------- Logic -------
// One-time hardware bring-up: bus down, baud set, line pins as inputs,
// all readings invalidated until the first real poll.
void sensorsInit() {
  pinMode(PIN_TOF_BUS, INPUT);          // release SIG; pull-up holds idle high
  tofBus.begin(TOF_BUS_BAUD);
  tofBus.setTimeout(TOF_TIMEOUT_MS);    // must exceed the sensors' sampling budget
                                        // (reg 0xB0 = 12 ms on this robot, see Config.h)
  pinMode(PIN_TRIMPOT, INPUT);
  lineSensL.begin();
  lineSensR.begin();
  lineSensB.begin();
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    tofMm[i] = 0; tofValid[i] = false; tofHealth[i] = TOF_HEALTH_MAX;
  }
}

// Live master speed, percent between TRIMPOT_MIN_SCALE_PCT and 100.
// Called by the state machine where speed is derived.
uint8_t trimpotScale() {
  return (uint8_t)map(analogRead(PIN_TRIMPOT), 0, 1023, TRIMPOT_MIN_SCALE_PCT, 100);
}

// Read only the ToF sensor at the active cursor (round-robin), normalise mm.
static void tofReadOne() {
  // Pick the sensor index this call: full round-robin, or front trio
  // (indices 1,2,3 = ids 2,3,4) when in attack scan mode.
  uint8_t i;
  if (tofAttackScan) {
    i = 1 + frontCursor;                         // cycles 1,2,3
    frontCursor = (frontCursor + 1) % 3;
  } else {
    i = tofCursor;
    tofCursor = (tofCursor + 1) % TOF_SENSOR_COUNT;
  }
  DistanceSensor& s = tofSensors[i];

  bool ok; uint16_t mm;
  if (s.mDeviceIndex == 3) {
    uint16_t raw = s.readDistance_mm();
    ok = (raw != TOF_INVALID_MM && raw >= TOF_MIN_VALID_MM && raw <= TOF_MAX_VALID_MM);
    mm = raw;
  } else {
    uint16_t raw = s.readDistance_cm();
    mm = (uint16_t)raw * 10;
    ok = (raw != TOF_INVALID_CM && mm >= TOF_MIN_VALID_MM && mm <= TOF_MAX_VALID_MM);
  }
  tofValid[i] = ok;
  tofMm[i]    = ok ? mm : 0;
  if (ok) { if (tofHealth[i] < TOF_HEALTH_MAX) tofHealth[i]++; }
  else    { if (tofHealth[i] > 0)              tofHealth[i]--; }
}

static void lineReadAll() {
  lineRaw[0] = lineSensL.readRaw();
  lineRaw[1] = lineSensB.readRaw();
  lineRaw[2] = lineSensR.readRaw();

  // Per-sensor Schmitt-trigger thresholds, indexed like lineRaw[]
  // (0 = left, 1 = rear, 2 = right) and the debug print order.
  static const uint16_t WHITE_THR[3] = {
    LINE_LEFT_WHITE_THRESHOLD, LINE_REAR_WHITE_THRESHOLD, LINE_RIGHT_WHITE_THRESHOLD
  };
  static const uint16_t BLACK_THR[3] = {
    LINE_LEFT_BLACK_THRESHOLD, LINE_REAR_BLACK_THRESHOLD, LINE_RIGHT_BLACK_THRESHOLD
  };
  bool* flags[3] = { &lineLeft, &lineRear, &lineRight };

  for (uint8_t i = 0; i < 3; i++) {
    if (*flags[i]) {
      // Already on edge: hold it until clearly back on black (no debounce on
      // recovery — we only need an unambiguous return to the interior).
      if (lineRaw[i] > BLACK_THR[i]) *flags[i] = false;
    } else {
      // Off edge: a run of white reads arms the debounce, then commits.
      if (lineRaw[i] < WHITE_THR[i]) {
        if (lineEdgeCount[i] < LINE_DEBOUNCE_SAMPLES) lineEdgeCount[i]++;
        if (lineEdgeCount[i] >= LINE_DEBOUNCE_SAMPLES) *flags[i] = true;
      } else {
        lineEdgeCount[i] = 0;
      }
    }
  }
}

/** One ToF read + one line pass. Call exactly once per loop(). */
void sensorsUpdate() {
  tofReadOne();
  lineReadAll();
}

/** Any debounced edge active right now? Used by the EVADE override. */
bool sensorsAnyEdge() { return lineLeft || lineRear || lineRight; }

/**
 * Which edge sensor tripped: 0=left, 1=rear, 2=right, -1 if none.
 * Priority rear > left > right (rear is the most dangerous: we are being
 * pushed from behind).
 */
int8_t sensorsEdgeIndex() {
  if (lineRear)  return 1;
  if (lineLeft)  return 0;
  if (lineRight) return 2;
  return -1;
}

/**
 * Opponent bearing from the ToF array: -100 = hard left, 0 = dead ahead,
 * +100 = hard right, 127 = nothing visible. Weighted vote where each valid
 * sensor contributes its geometric weight scaled by inverse distance —
 * close reflections dominate, far/uncertain ones barely count.
 */
int8_t sensorsOpponent() {
  int32_t wsum = 0, wtot = 0;
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    if (!tofValid[i]) continue;
    int32_t w = (int32_t)TOF_MAX_VALID_MM + 1 - tofMm[i];   // near -> heavy
    wsum += (int32_t)TOF_WEIGHT[i] * 25 * w;
    wtot += w;
  }
  if (wtot == 0) return 127;
  int32_t b = wsum / wtot;
  return (int8_t)constrain(b, -100, 100);
}

/** Smallest currently-valid ToF distance in mm; ok=false when none. */
uint16_t sensorsClosestMm(bool* ok) {
  uint16_t best = 0xFFFF;
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++)
    if (tofValid[i] && tofMm[i] < best) best = tofMm[i];
  *ok = (best != 0xFFFF);
  return *ok ? best : 0;
}

#endif // SENSORS_H
