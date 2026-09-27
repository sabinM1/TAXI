/*
 * Straight_Line_Test — drive straight until a line sensor sees the white border.
 * Started by the Start/Stop module. Runs on battery, no serial, no PC.
 *
 * WHAT IT DOES
 *   1. Power on. 3 s settle delay, then LED1 pulses once a second: armed, waiting
 *      for the referee / remote START signal. Put the robot in the middle of the
 *      dohyo facing where you want while it waits.
 *   2. START -> both wheels drive FORWARD at DRIVE_PCT. Nothing else: no
 *      steering, no ToF, no strategies, no state machine beyond the run phases.
 *   3. The instant any of the 3 line sensors sees the white border, both motors
 *      are cut and the ring LED for that sensor lights solid white. LED1 blinks
 *      three times: the run is over, look at which ring LED is lit.
 *   4. STOP -> the robot stops for good. Same rule as the match firmware: a stop
 *      is terminal, power-cycle to run again.
 *
 * START/STOP — THE SAME PATH AS TAXI_Firmware
 *   StateMachine.h in TAXI_Firmware is the reference. Copied here with the same
 *   names so there is one start/stop behaviour in the whole project:
 *     - startSignal / startModuleUpdate() / isStarted(): one RAW digitalRead() of
 *       PIN_START, HIGH = fight, LOW = armed/stopped (NanoLabs module). No
 *       debounce, no filtering, no averaging.
 *     - Sampled ONCE per loop, FIRST, before anything that can move the robot.
 *     - Stop has the highest priority: the moment the pin reads LOW the motors are
 *       cut and the state jumps to ST_STOP, which is terminal.
 *     - The pin MUST read LOW at power-on, checked AFTER the 3 s settle delay
 *       exactly like the firmware's delay(3000) — never before it.
 *
 * WHY THE 3 s DELAY IS NOT OPTIONAL
 *   The Start/Stop module is its own MCU and boots at the same time as the robot.
 *   Until it has booted, its output is high-Z, so the robot's INPUT_PULLUP reads
 *   that as HIGH — a fake "module is ON". Sampling PIN_START in that window trips
 *   the power-on check and the robot never moves. Sampling happens only after
 *   delay(STARTUP_SETTLE_MS). Do not sample earlier.
 *
 * LED MEANINGS (onboard LED1 = D8, LED2 = D9)
 *   slow pulse        armed, waiting for the START signal
 *   solid on          driving
 *   three slow blinks line found, motors cut, holding still
 *   fast blink        the run is not valid: a sensor was already on white when
 *                     START arrived, or no line was found within DRIVE_MAX_MS
 *   LED1 blinks fast  the module was already latched ON at power-up. Send STOP and
 *   forever, no       power-cycle the robot (identical to the match firmware).
 *   movement
 *
 * RING (WS2812, 7 LEDs)
 *   Used only to confirm the detection: the moment a line sensor sees the white,
 *   the ring LED that represents THAT sensor lights solid white and stays lit, so
 *   you can read the result from across the room once the robot stops.
 *   front-left = LED 3, back = LED 1, front-right = LED 5 — the same mapping the
 *   match firmware uses for its line overlay.
 *
 * WHAT YOU ARE TESTING
 *   Whether the robot tracks a true straight line under power. It drives through
 *   the same Motor.h, polarity and LM_TRIM_PCT / RM_TRIM_PCT as the match
 *   firmware, so any curve here is drift the firmware has too. The robot turns
 *   TOWARD THE SLOWER SIDE: pulls LEFT -> the RIGHT side is doing more work, so
 *   step RM_TRIM_PCT down (98, 95, ...); pulls RIGHT -> step LM_TRIM_PCT down.
 *   Only ever trim the faster side DOWN.
 *
 * WHAT IT DELIBERATELY DOES NOT DO
 *   - No ToF bus: sensorsInit() is never called, so SingleWireSerial never takes
 *     Timer1 and the loop stays fast. Only the 3 QRE1113 line sensors are read,
 *     through the firmware's own lineReadAll() — same thresholds, same Schmitt
 *     trigger, same debounce as the real firmware.
 *   - No steering, no attack/evade logic, no tactics, no DIP reading.
 *
 * SHARED HEADERS
 *   Config.h / Motor.h / Sensors.h / LineSensor.h / DistanceSensor.h /
 *   RingInfoDisplay.h are byte-for-byte copies of the ones next to
 *   TAXI_Firmware.ino (a sketch cannot reliably include files from a sibling
 *   folder). If the line thresholds, the motor polarity or the start pin change in
 *   the firmware, copy those headers over again or this test runs on stale numbers.
 */

#include "Config.h"
#include "Motor.h"
#include "Sensors.h"           // line sensors + edge flags only; ToF bus left unstarted
#include "RingInfoDisplay.h"   // only used to light one LED on the line detection

// ============================================================================
//                    EDIT THESE — everything is compile-time
// ============================================================================

#define DRIVE_PCT       25   // forward speed, percent. Both wheels get exactly
                             // this — no trim beyond LM_TRIM_PCT/RM_TRIM_PCT,
                             // no steering correction of any kind.

#define STARTUP_SETTLE_MS 3000
                             // Settling time before PIN_START is first read, the
                             // same as the firmware's delay(3000). MUST stay long
                             // enough for the Start/Stop module to boot, or the
                             // power-on check reads the module's floating output
                             // as "ON". Do not shorten it.

#define DRIVE_MAX_MS  10000  // safety stop: if no line is found in this time the
                             // robot is off the dohyo (or a sensor is dead), so
                             // cut the motors and fast-blink instead of driving
                             // away. Normal runs finish in well under a second.

#define SERIAL_LOG       0   // 1 = also print the run on USB Serial.
                             // Leave 0 for battery runs: with no host attached
                             // there is nothing to read, and this keeps the run
                             // timing clean.

// ============================================================================

enum RunState : uint8_t {
  ST_IDLE,      // armed, motors off, waiting for the START signal
  ST_DRIVE,     // driving straight forward, looking for the line
  ST_FOUND,     // line detected, motors cut, holding still
  ST_BADSTART,  // a sensor was already on white when START arrived
  ST_NOLINE,    // DRIVE_MAX_MS elapsed without finding the line
  ST_STOP       // stopped by the module or the referee — terminal
};

static RunState state = ST_IDLE;
static uint32_t phaseStart = 0;

// ---- Start/stop signal -----------------------------------------------------
// The NanoLabs module holds its state in non-volatile memory and drives
// PIN_START as a clean logic level, so the pin is read raw every loop — no
// debounce, no filtering, no averaging (see StateMachine.h in TAXI_Firmware).
// MUST start LOW (armed): initialising it HIGH would fake an instant start
// before the pin has ever been read.
static bool startSignal = LOW;

/** Sample the Start/Stop module. HIGH = fight, LOW = armed/stopped.
 *  Call once per loop() BEFORE anything that uses isStarted(). */
static void startModuleUpdate() {
  startSignal = digitalRead(PIN_START);
}

static bool isStarted() { return startSignal == HIGH; }

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
 * Ring LED that represents each line sensor, indexed like the edge flags
 * (0 = left, 1 = rear, 2 = right) and the serial print order. The ring layout is
 * LED0 = centre, 1 = back, 2 = left-back, 3 = left-front, 4 = front,
 * 5 = right-front, 6 = right-back — the same mapping the match firmware uses.
 */
static const uint8_t LINE_RING_LED[3] = { 3, 1, 5 };

/**
 * Visual confirmation of the detection: light the one ring LED that matches the
 * sensor which tripped, solid white. Latched by the WS2812, so it stays lit with
 * no further FastLED work.
 */
static void ringShowLine(int8_t src) {
  if (src < 0 || src > 2) return;
  RingClear();
  RingSetLEDColor(LINE_RING_LED[src], CRGB::White);
  RingShow();
}

static void gotoState(RunState s) {
  state = s;
  phaseStart = millis();
#if SERIAL_LOG
  Serial.print(F("[run] t=")); Serial.print(millis());
  Serial.print(F("ms -> state ")); Serial.println((uint8_t)s);
#endif
}

void setup() {
#if SERIAL_LOG
  // Serial first: the settle delay below is the natural window to watch the boot.
  Serial.begin(MONITOR_BAUD);
  Serial.println(F("[LineTest] boot - settling"));
#endif

  // Claim the motor pins before the delay. From power-on until here they float
  // in input mode, which lets the driver chatter the motors.
  motorInit();
  motorsStop();

  // Settling period before the start pin is first read — see the header.
  delay(STARTUP_SETTLE_MS);

  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);

  // NanoLabs start module rule, identical to the match firmware: the pin MUST
  // read LOW at power-on (module armed, waiting for the referee). If it reads
  // HIGH the module still holds the previous run's "on" state — the referee must
  // send stop and the robot must be power-cycled. Lock hard and blink LED1 so
  // this can never be mistaken for a ready robot.
  startModuleUpdate();
  if (isStarted()) {
#if SERIAL_LOG
    Serial.println(F("[run] FATAL: start pin HIGH at boot - power cycle!"));
#endif
    while (true) {
      digitalWrite(PIN_LED1, HIGH); delay(100);
      digitalWrite(PIN_LED1, LOW);  delay(100);
    }
  }

  // Line sensors only. sensorsInit() is deliberately NOT called: it would start
  // the ToF bus (Timer1) and slow the loop down for no benefit here.
  lineSensL.begin();
  lineSensR.begin();
  lineSensB.begin();

  // Ring: initialized blank, only ever used to light the one LED that confirms a
  // detection (see ringShowLine()).
  RingInit();

  gotoState(ST_IDLE);
}

void loop() {
  // Start/stop sampled ONCE per loop, FIRST — exactly like the match firmware.
  startModuleUpdate();
  lineReadAll();            // same Schmitt + debounce as the firmware

  // Global stop, checked before every other rule below. ST_IDLE is excluded so
  // the pre-match state does not jump straight to STOP; ST_STOP is excluded
  // because it re-runs motorsStop() in its own state body anyway.
  if (!isStarted() && state != ST_IDLE && state != ST_STOP) {
    gotoState(ST_STOP); motorsStop();
    return;
  }

#if SERIAL_LOG
  static uint32_t lastLog = 0;
  if (millis() - lastLog >= 500) {
    lastLog = millis();
    Serial.print(F("[run] st=")); Serial.print((uint8_t)state);
    Serial.print(F(" start=")); Serial.print(isStarted() ? 1 : 0);
    Serial.print(F(" line L/B/R="));
    Serial.print(lineRaw[0]); Serial.print('/');
    Serial.print(lineRaw[1]); Serial.print('/');
    Serial.print(lineRaw[2]);
    Serial.println();
  }
#endif

  switch (state) {
    case ST_IDLE:                           // armed, waiting for the referee
      motorsStop();
      ledWait(phaseStart);
      if (isStarted()) {
        if (sensorsAnyEdge()) {
          ringShowLine(sensorsEdgeIndex());   // also shows WHICH sensor is on white
          gotoState(ST_BADSTART);             // started sitting on the white
        } else {
          gotoState(ST_DRIVE);
        }
      }
      break;

    case ST_DRIVE:                          // straight, flat out, until the line
      motorsSet(DRIVE_PCT, DRIVE_PCT);
      digitalWrite(PIN_LED1, HIGH);
      if (sensorsAnyEdge()) {
        motorsStop();
        ringShowLine(sensorsEdgeIndex());   // visual confirmation, which sensor
#if SERIAL_LOG
        Serial.print(F("[run] line at t=")); Serial.print(millis());
        Serial.print(F("ms  raw L/B/R="));
        Serial.print(lineRaw[0]); Serial.print('/');
        Serial.print(lineRaw[1]); Serial.print('/');
        Serial.println(lineRaw[2]);
#endif
        gotoState(ST_FOUND);
      } else if (millis() - phaseStart >= DRIVE_MAX_MS) {
        motorsStop();
        gotoState(ST_NOLINE);
      }
      break;

    case ST_FOUND:
      motorsStop();
      ledBlink(phaseStart, 500, 3);         // three blinks: read the ring LED
      break;

    case ST_BADSTART:
    case ST_NOLINE:
      motorsStop();
      ledBlink(phaseStart, 150, 0);         // fast blink: see the header
      break;

    case ST_STOP:
    default:
      motorsStop();                         // terminal: power-cycle to run again
      break;
  }
}
