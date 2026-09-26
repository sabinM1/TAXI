/*
 * Edge_Turn_Test — autonomous evade-turn tuner, started by the Start/Stop module.
 * Runs on battery, no serial, no PC.
 *
 * WHAT IT DOES
 *   1. Power on. The LED pulses slowly: armed, waiting for the referee/remote
 *      START signal (NanoLabs module -> HIGH). Put the robot in the middle of the
 *      dohyo while it waits.
 *   2. START signal -> it drives FORWARD at the firmware's hunt speed until a line
 *      sensor sees the white border.
 *   3. It runs the firmware's evade reflex on that edge:
 *        FLEE  reverse away from the edge until the sensors are clear
 *              (at least FLEE_MS, at most FLEE_MAX_MS of it)
 *        TURN  pivot away from the edge for exactly TURN_MS
 *   4. It stops and holds still. The LED blinks three times: measure the result.
 *   5. STOP signal -> back to armed (step 1). Place it again and send START for
 *      the next run: no reset and no power cycle needed.
 *
 * HOW TO ITERATE
 *   1. Change TURN_MS below (that is the knob you are tuning), re-flash.
 *   2. Place the robot in the middle facing away from you, send START.
 *   3. Watch where it ends up, measure the rotation, then scale:
 *          new TURN_MS = TURN_MS x (wanted degrees / measured degrees)
 *   How to measure:
 *     - tape a mark on the floor along the robot's starting heading and a matching
 *       mark beside the blade; the angle between the two marks is the rotation;
 *     - or stand a phone on the chassis with a compass/level app and read the
 *       heading before and after.
 *   Turning slower is easier to measure and less likely to slip: lower
 *   TURN_OUTER_PCT/TURN_INNER_PCT and raise TURN_MS proportionally.
 *
 *   The ring LED that lights up tells you which sensor saw the line, so the
 *   approach geometry is confirmed from across the room.
 *
 * LED MEANINGS (onboard LED1 = D8, LED2 = D9)
 *   slow pulse        armed, waiting for the START signal
 *   solid on          driving / fleeing / turning
 *   three slow blinks run finished, robot holding still -> measure now
 *   fast blink        the run is not measurable (send STOP, fix, START again):
 *                       - a sensor was already on white when START arrived
 *                       - no line found within CRUISE_MAX_MS
 *                       - the flee could not clear the edge
 *   both LEDs on, no  the module was already latched ON at power-up. Send STOP
 *   blinking, no      from the remote, then power-cycle the robot (same rule as
 *   movement          the match firmware).
 *
 * RING (WS2812, 7 LEDs) — visual confirmation of the edge detection
 *   The moment a line sensor sees the white, the ring LED that represents THAT
 *   sensor lights up solid white and stays lit until STOP: front-left = LED 3,
 *   back = LED 1, front-right = LED 5 (the same mapping the match firmware uses
 *   for its line overlay). It tells you at a glance that the detection fired, and
 *   which sensor caught the line — so at the end of the run you can also tell
 *   whether the approach angle was what you expected.
 *
 * START/STOP POLARITY
 *   HIGH = fight/run, LOW = armed/stopped (NanoLabs module, active high). The pin
 *   is read raw every loop with no debounce and no filtering, exactly like the
 *   match firmware does, and like there it is the first thing checked: the moment
 *   it goes LOW the motors are cut, whatever the run was doing.
 *   The one deliberate difference from the match firmware: a STOP here re-arms the
 *   robot instead of latching until a power cycle, because this is a test rig and
 *   you want to run it again immediately.
 *
 * WHAT IT DELIBERATELY DOES NOT DO
 *   - No ToF bus: sensorsInit() is never called, so SingleWireSerial never takes
 *     Timer1 and the loop stays fast. Only the 3 QRE1113 line sensors are read,
 *     through the firmware's own lineReadAll() — same thresholds, same Schmitt
 *     trigger, same debounce as the real firmware.
 *   - The ring only ever shows the line-confirmation LED: no telemetry, no
 *     heartbeat, nothing that costs a FastLED.show() while the robot is moving.
 *   - No start/stop module handling beyond the raw signal, no state machine, no
 *     strategies.
 *   - No serial by default (SERIAL_LOG 0). A battery-run robot has no host, so
 *     there is nothing to print to, and this keeps the run timing clean.
 *
 * SHARED HEADERS
 *   Config.h / Motor.h / Sensors.h / LineSensor.h / DistanceSensor.h /
 *   RingInfoDisplay.h are copies of the ones next to TAXI_Firmware.ino (a sketch
 *   cannot reliably include files from a sibling folder). If you change the line
 *   thresholds, the motor polarity or the start module polarity in the firmware,
 *   copy those headers over again or this test tunes against stale numbers.
 */

#include "Config.h"
#include "Motor.h"
#include "Sensors.h"     // line sensors + edge flags only; ToF bus left unstarted
#include "RingInfoDisplay.h"   // only used to light one LED on the line detection

// ============================================================================
//                    EDIT THESE — everything is compile-time
// ============================================================================

#define TURN_MS         80   // *** THE KNOB *** pivot duration in ms.
                              // -> EVADE_TURN_MS in TAXI_Firmware/Config.h

#define TURN_OUTER_PCT   70   // pivot: outer wheel forward   -> EVADE_TURN_OUTER_PCT
#define TURN_INNER_PCT   50   // pivot: inner wheel reversed  -> EVADE_TURN_INNER_PCT

#define FLEE_MS          15   // flee floor (blind burst)     -> EVADE_ESCAPE_MIN_MS
#define FLEE_MAX_MS     300   // flee ceiling; past this the run is invalid
                              //                              -> EVADE_FLEE_MAX_MS
#define FLEE_SPEED_PCT  -80   // flee speed, negative = reverse
                              //                              -> EVADE_REVERSE_SPEED_PCT
#define REAR_FWD_PCT     80   // flee when the REAR edge trips (drive forward!)
#define REAR_BIAS_PCT    55   //   ..and the bias on the other wheel
                              //   both -> evadeEscapeBurst() in StateMachine.h

// Approach speed, fixed. Use SEARCH_SPEED_PCT (18) to match the firmware's hunt
// speed, or ATTACK_SPEED_FAR_PCT (60) to match its cautious push. The evade
// flee/turn constants below are absolute too, so nothing here depends on any knob
// position: same numbers, same behaviour, every run.
#define CRUISE_PCT       SEARCH_SPEED_PCT
#define CRUISE_MAX_MS    5000   // safety: if no line is found in this time, stop

#define SERIAL_LOG        0   // 1 = also print the run on USB Serial.
                              // Leave 0 for battery runs: with no host attached
                              // there is nothing to read, and this guarantees
                              // zero serial cost in the timing you are tuning.

// ============================================================================

enum RunState : uint8_t {
  R_WAIT,        // armed, motors off, waiting for the START signal
  R_CRUISE,      // driving forward looking for the line
  R_FLEE,        // escaping the edge
  R_TURN,        // pivoting away from the edge
  R_DONE,        // finished, holding still (measure now), until STOP
  R_BADSTART,    // a sensor was already on white when START arrived
  R_NOLINE,      // CRUISE_MAX_MS elapsed without finding the line
  R_STUCK        // flee hit FLEE_MAX_MS with white still under a sensor
};

static RunState rs = R_WAIT;
static uint32_t phaseStart = 0;
static int8_t   edgeSrc = 0;      // edge that tripped this run
static bool     prevEdge[3] = { false, false, false };

/**
 * Is the run allowed right now? Raw read, no debounce — the module holds a clean
 * latched level and this value decides whether the motors may drive at all.
 */
static bool startSignal() { return digitalRead(PIN_START) == HIGH; }

/** Non-blocking LED pattern since `t0`: `count` blinks of `period` ms, or a
 *  continuous blink when count == 0. */
static void ledBlink(uint32_t t0, uint16_t period, uint8_t count) {
  uint32_t t = millis() - t0;
  if (count && (t / period) >= count) { digitalWrite(PIN_LED1, LOW); return; }
  digitalWrite(PIN_LED1, ((t % period) < (period / 2)) ? HIGH : LOW);
}

/** Armed heartbeat: a short pulse once a second. */
static void ledWait(uint32_t t0) {
  digitalWrite(PIN_LED2, LOW);
  digitalWrite(PIN_LED1, ((millis() - t0) % 1000) < 100 ? HIGH : LOW);
}

/**
 * Ring LED that represents each line sensor (same mapping the match firmware uses
 * for its line overlay), indexed like the edge flags: 0 = left, 1 = rear, 2 = right.
 * The ring layout is LED0 = centre, 1 = back, 2 = left-back, 3 = left-front,
 * 4 = front, 5 = right-front, 6 = right-back.
 */
static const uint8_t LINE_RING_LED[3] = { 3, 1, 5 };

/**
 * Visual confirmation of the edge detection: light the one ring LED that matches
 * the sensor which tripped, solid white. Latched by the WS2812, so it stays lit
 * with no further FastLED work until it is cleared on STOP.
 */
static void ringShowLine(int8_t src) {
  if (src < 0 || src > 2) return;
  RingClear();
  RingSetLEDColor(LINE_RING_LED[src], CRGB::White);
  RingShow();
}

/** Flee command for one edge. Rear -> forward (reversing would push us out). */
static void flee(int8_t src) {
  if (src == 1) motorsSet(REAR_FWD_PCT, REAR_BIAS_PCT);
  else          motorsSet(FLEE_SPEED_PCT, FLEE_SPEED_PCT);
}

/** Pivot away from the edge. Left edge -> rotate right, and vice versa. */
static void pivot(int8_t src) {
  if (src == 0)      motorsSet(TURN_OUTER_PCT, -TURN_INNER_PCT);
  else if (src == 2) motorsSet(-TURN_INNER_PCT, TURN_OUTER_PCT);
  else               motorsSet(TURN_OUTER_PCT, -TURN_OUTER_PCT);   // rear: spin
}

static void enter(RunState s) {
  rs = s;
  phaseStart = millis();
#if SERIAL_LOG
  Serial.print(F("[run] t=")); Serial.print(millis());
  Serial.print(F("ms -> state ")); Serial.println((uint8_t)s);
#endif
}

#if SERIAL_LOG
static const char* edgeName(int8_t s) {
  return (s == 0) ? "LEFT" : (s == 1) ? "REAR" : "RIGHT";
}
#endif

/** Rising edge of any line flag. Schmitt + debounce already done by lineReadAll(). */
static int8_t newEdge() {
  bool cur[3] = { lineLeft, lineRear, lineRight };
  int8_t found = -1;
  for (uint8_t i = 0; i < 3; i++)
    if (!prevEdge[i] && cur[i]) { found = (int8_t)i; break; }
  for (uint8_t i = 0; i < 3; i++) prevEdge[i] = cur[i];
  return found;
}

void setup() {
  motorInit();
  motorsStop();

  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_START, INPUT_PULLUP);

  // Line sensors only. sensorsInit() is deliberately NOT called: it would start
  // the ToF bus (Timer1) and slow the loop down for no benefit here.
  lineSensL.begin();
  lineSensR.begin();
  lineSensB.begin();

  // Ring: initialized blank, and only ever used to light the one LED that confirms
  // an edge detection (see ringShowLine()).
  RingInit();

#if SERIAL_LOG
  Serial.begin(MONITOR_BAUD);
  Serial.println(F("[Edge_Turn_Test] autonomous run, started by the Start/Stop module"));
  Serial.print(F("[run] TURN_MS=")); Serial.print(TURN_MS);
  Serial.print(F("  FLEE=")); Serial.print(FLEE_MS); Serial.print(F(".."));
  Serial.print(FLEE_MAX_MS); Serial.print(F("ms  cruise="));
  Serial.print(CRUISE_PCT); Serial.println('%');
#endif

  // Same rule as the match firmware: the module must be OFF (LOW) at power-up. If
  // it is still latched ON from a previous run, running it now would launch the
  // robot the instant it is placed. Hold with both LEDs lit until a power cycle.
  if (startSignal()) {
#if SERIAL_LOG
    Serial.println(F("[run] FATAL: start pin HIGH at boot - send STOP and power cycle"));
#endif
    digitalWrite(PIN_LED1, HIGH);
    digitalWrite(PIN_LED2, HIGH);
    while (true) motorsStop();       // parked, no way out but a power cycle
  }

  enter(R_WAIT);
}

void loop() {
  lineReadAll();          // same Schmitt + debounce as the firmware
  int8_t hit = newEdge(); // -1 = nothing new
  bool start = startSignal();

  // STOP has top priority, exactly like the match firmware: the instant the pin
  // drops, no motor command is allowed. A running state abandons its run, a
  // finished/error state re-arms so the next START works without a reset.
  if (!start && rs != R_WAIT) {
    motorsStop();
    RingClear(); RingShow();     // blank the line LED for the next run
    enter(R_WAIT);
  }

  switch (rs) {
    case R_WAIT:                            // armed, waiting for the referee
      motorsStop();
      ledWait(phaseStart);
      if (start) {
        if (sensorsAnyEdge()) {
          ringShowLine(sensorsEdgeIndex());   // also shows WHICH sensor is on white
          enter(R_BADSTART);                // started sitting on the white
        } else {
          enter(R_CRUISE);
        }
      }
      break;

    case R_CRUISE:                          // drive forward until the line
      motorsSet(CRUISE_PCT, CRUISE_PCT);
      digitalWrite(PIN_LED1, HIGH);
      if (hit >= 0) {
        edgeSrc = hit;
        ringShowLine(edgeSrc);              // visual confirmation, which sensor
#if SERIAL_LOG
        Serial.print(F("[run] edge ")); Serial.print(edgeName(edgeSrc));
        Serial.print(F("  raw L/B/R="));
        Serial.print(lineRaw[0]); Serial.print('/');
        Serial.print(lineRaw[1]); Serial.print('/');
        Serial.println(lineRaw[2]);
#endif
        enter(R_FLEE);
      } else if (millis() - phaseStart >= CRUISE_MAX_MS) {
        motorsStop();
        enter(R_NOLINE);
      }
      break;

    case R_FLEE:
      flee(edgeSrc);
      digitalWrite(PIN_LED1, HIGH);
      // Advance once the sensors are clear, but never before the mandatory blind
      // burst (the edge can flicker off while still rolling off the dohyo) and
      // never past FLEE_MAX_MS — past that the run is not measurable, because a
      // pivot started over white is pivoting along the line, not on black.
      if (millis() - phaseStart >= FLEE_MAX_MS) {
        motorsStop();
        enter(R_STUCK);
      } else if (!sensorsAnyEdge() && millis() - phaseStart >= FLEE_MS) {
        enter(R_TURN);
      }
      break;

    case R_TURN:
      pivot(edgeSrc);
      digitalWrite(PIN_LED1, HIGH);
      if (millis() - phaseStart >= TURN_MS) {
        motorsStop();
        enter(R_DONE);                      // three blinks: measure now
      }
      break;

    case R_DONE:
      motorsStop();
      ledBlink(phaseStart, 500, 3);
      break;

    case R_BADSTART:
    case R_NOLINE:
    case R_STUCK:
      motorsStop();
      ledBlink(phaseStart, 150, 0);         // fast blink: see the header
      break;
  }
}
