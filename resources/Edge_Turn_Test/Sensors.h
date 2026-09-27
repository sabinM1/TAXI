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
// TOF_WEIGHT_SCALE turns a weight into bearing units (weight * 25), so the
// largest weight (+/-4, the 90-degree side sensors) reaches the full +/-100
// "hard over" and everything else is read as a fraction of that.
//
// These weights are STEERING AUTHORITY, not a measured angle. The sensors sit at
// 0, +/-25 and +/-90 degrees, and a weight proportional to the angle would make
// the front pair far too quiet: a target seen ONLY by a front-outer sensor is a
// target the robot is about to drive straight past, and it has to be turned onto
// almost as hard as one that is already alongside. At the old +/-2 (roughly
// angle-proportional) a front-left-only sighting produced a bearing of 50 — half
// the turn rate of a side sensor — so the robot under-turned and overshot every
// time the opponent sat off a front corner while the sides stayed quiet. +/-3
// keeps the sides loudest (4) but lifts the front pair to 75, which commands a
// hard turn-in without quite asking for a full pivot on a ~25-degree error.
//
// The front trio outranking the sides (see sensorsOpponent()) is what stops a
// side reading from interfering while the front can see the target, so this gap
// only ever decides the case where a side sensor is the ONLY thing looking at
// the opponent. If a front-corner sighting still under-turns, go to +/-4, which
// makes a front-outer reading command exactly the same turn as a side one.
static const int8_t TOF_WEIGHT[TOF_SENSOR_COUNT] = { 4, 3, 0, -3, -4 };
#define TOF_WEIGHT_SCALE 25

// The three FRONT sensors (ids 2, 3, 4) are indices 1..3. sensorsOpponent() uses
// this to let the front trio outrank the two side sensors — see the note there.
static inline bool tofIsFrontIndex(uint8_t i) { return i >= 1 && i <= 3; }

static SingleWireSerial tofBus(false);           // bus on D4 (Config: PIN_TOF_BUS)
static DistanceSensor tofSensors[TOF_SENSOR_COUNT] = {
  DistanceSensor(&tofBus, 1), DistanceSensor(&tofBus, 2),
  DistanceSensor(&tofBus, 3), DistanceSensor(&tofBus, 4),
  DistanceSensor(&tofBus, 5)
};
static uint8_t tofCursor = 0;                    // round-robin position

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
  lineSensL.begin();
  lineSensR.begin();
  lineSensB.begin();
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    tofMm[i] = 0; tofValid[i] = false; tofHealth[i] = TOF_HEALTH_MAX;
  }
}

// NOTE: there is no trimpotScale() here any more. The onboard trimpot is not read
// (see Config.h): every speed is an absolute percent constant, so a run is
// reproducible and no knob position can change the behaviour.

// Read only the ToF sensor at the active cursor (round-robin), normalise mm.
//
// Every sensor is polled, in every state: the chase has to be able to turn onto
// an opponent that is off to a side, and the side-backoff needs the side sensors
// live, so there is no front-only scan any more. A full five-sensor sweep costs
// ~10 ms, which is fast enough for all of it.
static void tofReadOne() {
  uint8_t i = tofCursor;
  tofCursor = (tofCursor + 1) % TOF_SENSOR_COUNT;
  DistanceSensor& s = tofSensors[i];

  // Read the register directly so we can tell apart the two very different
  // reasons a reading is unusable:
  //   responded == false  -> the sensor did not ANSWER at all (bus/sensor fault)
  //   responded == true   -> it answered, it just has no target in range
  // Only the first is a fault; the second is the normal "nothing in front" case.
  bool responded, ok;
  uint16_t raw = 0, mm = 0;      // zero-initialised: a failed read leaves them unwritten
  if (s.mDeviceIndex == 3) {
    responded = s.readRegister(0x01, &raw);      // id 3 reports millimetres
    mm = raw;
    ok = responded && raw != TOF_INVALID_MM && raw >= TOF_MIN_VALID_MM && raw <= TOF_MAX_VALID_MM;
  } else {
    responded = s.readRegister(0x06, &raw);      // the rest report centimetres
    mm = (uint16_t)raw * 10;
    ok = responded && raw != TOF_INVALID_CM && mm >= TOF_MIN_VALID_MM && mm <= TOF_MAX_VALID_MM;
  }
  tofValid[i] = ok;
  tofMm[i]    = ok ? mm : 0;

  // Health tracks the BUS, not the target. A sensor that answers is alive even
  // when it reports "nothing in range" — which is the NORMAL state for every
  // sensor that is not currently pointing at the opponent. Counting that as a
  // fault made the robot declare itself degraded (and blind-spin) as soon as the
  // opponent was visible to only one or two sensors.
  if (responded) { if (tofHealth[i] < TOF_HEALTH_MAX) tofHealth[i]++; }
  else           { if (tofHealth[i] > 0)              tofHealth[i]--; }
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
 * sensor contributes its geometric weight scaled by INVERSE DISTANCE SQUARED —
 * see the WEIGHT note in the loop body for why the falloff is squared.
 *
 * The three FRONT sensors outrank the two side ones: as soon as any of the front
 * trio has a reading, the sides are ignored. Their geometric weight is the
 * largest (+/-4), so a single close side reflection — the opponent's blade edge,
 * a wall, a hand — would otherwise outvote the sensors that are actually looking
 * at the target and throw the heading off. The sides still decide on their own
 * when nothing in front can see anything, which is exactly the "opponent is beside
 * me" case the side-backoff state exists for.
 */
int8_t sensorsOpponent() {
  bool frontSeen = tofValid[1] || tofValid[2] || tofValid[3];
  int32_t wsum = 0, wtot = 0;
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    if (!tofValid[i]) continue;
    if (frontSeen && !tofIsFrontIndex(i)) continue;

    // WEIGHT: linear distance (the old `TOF_MAX_VALID_MM + 1 - mm`) gave a
    // 500 mm reading roughly half the say of a 170 mm one, so a far reflection on
    // ONE side cancelled a close one on the other and the vote came out
    // "dead ahead" — the robot drove straight past an opponent it could clearly
    // see (e.g. front-left at 170 mm against front-right at 500 mm averaged to a
    // bearing of ~-17, inside the old ATTACK dead zone, so the robot drove on
    // past). Squaring the same term makes the weight collapse far faster with
    // distance: the same pair now reads ~-31, i.e. a real steering correction. The
    // closest sensor therefore decides the heading, which is what makes the robot
    // commit to what it is about to hit instead of averaging that away against
    // something further off.
    //
    // Integer-safe: w <= (800+1)^2 = 641601 and the largest term is
    // 4 * 25 * 641601 ~= 6.4e7, so the 5-sensor sum stays far inside int32.
    // It is a pure ratio, so the absolute scale cancels in wsum / wtot.
    int32_t w = (int32_t)TOF_MAX_VALID_MM + 1 - tofMm[i];   // near -> heavy
    w *= w;                                                 // squared falloff
    wsum += (int32_t)TOF_WEIGHT[i] * TOF_WEIGHT_SCALE * w;
    wtot += w;
  }
  if (wtot == 0) return 127;
  int32_t b = wsum / wtot;
  return (int8_t)constrain(b, -100, 100);
}

/**
 * Blade-to-blade contact: the MIDDLE ToF sensor (id 3, straight ahead) reading
 * PUSH_CONTACT_DIST_MM or closer. Only the middle sensor is used on purpose —
 * the outer front sensors can see the opponent's corner from well off to the side,
 * and "contact" must mean the opponent is actually in front of the blade.
 */
bool sensorsContact() {
  return tofValid[2] && tofMm[2] <= PUSH_CONTACT_DIST_MM;
}

/**
 * Either SIDE ToF at or inside `mm` — index 0 = ID1 side right, index 4 = ID5
 * side left. These look perpendicular to the robot, so this means "the opponent
 * is alongside", the one case the front sensors cannot act on.
 *
 * Only true for a reading that is currently valid, i.e. one that has been
 * refreshed this sweep: every sensor is polled in every state, so a false here
 * means "nothing alongside", not "not looked at".
 */
bool sensorsSideClose(uint16_t mm) {
  return (tofValid[0] && tofMm[0] <= mm) || (tofValid[4] && tofMm[4] <= mm);
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
