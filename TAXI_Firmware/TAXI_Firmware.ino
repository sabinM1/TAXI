/*
 * TAXI_Firmware.ino — main program for the TAXI mini sumo.
 *
 * Hardware (verified, see Config.h):
 *   5x M60F ToF on D4 (ids 1..5; 1,2,4,5 in cm — 3 in mm)
 *   3x QRE1113 line sensors (A1 right, A4 rear, A5 left)
 *   WS2812 7-LED ring on A2  /  Start-Stop module on A0 (HIGH = fight)
 *   DIP strategy select on D5/D6/D7  /  dual motors (D10 D12 | D11 D13)
 *
 * Loop order matters: ToF serial reads always finish before FastLED.show()
 * (the WS2812 bit-bang blocks interrupts and would garble the sensor bus).
 *
 * Libraries to install (Arduino Library Manager): FastLED, SingleWireSerial.
 */

#include "Config.h"
#include "RingInfoDisplay.h"
#include "Sensors.h"
#include "Strategies.h"
#include "StateMachine.h"
#include <avr/wdt.h>   // watchdog

// ---------------------------------------------------------------------------
// Watchdog — armed for DEBUG_VERBOSE (bench) builds only, on purpose.
//
// A watchdog reset mid-match is not a recovery, it is the end of the round: on
// the way back up the start module still reads HIGH (the fight is on), so
// setup() trips the boot lock and parks the robot blinking until someone
// power-cycles it. Without a watchdog a hung loop simply keeps the last motor
// command, which in sumo frequently still finishes the push it started.
//
// On the bench the trade runs the other way: a hang means a 100%-duty motor
// turning next to your hands, and a reset there recovers cleanly (the module
// reads LOW, so the robot just re-arms into IDLE). Bench builds also hold the
// only unbounded blocking call in the firmware — the ~50 ms serial table.
//
// 250 ms is ~100x a normal loop (~2 ms) and far above the longest
// interrupt-disabled section (~210 us in FastLED.show()), so it can only ever
// fire on a genuine lock-up, never on ordinary timing.
//
// Rule that follows from this: set DEBUG_VERBOSE false for a real match (that
// is what disarms the watchdog and stops the serial dumps).
#define WATCHDOG_ENABLED  (DEBUG_VERBOSE)

// Render all 5 ToF sensors as a green->red distance gradient on the outer
// ring (uses RingInfoDisplay.h helpers). Far (>= RING_DIST_GREEN_CM) = green,
// very close = red; brightness also fades with distance (near = bright).
// Invalid readings stay blank.
static void renderToFGradient() {
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    if (!tofValid[i]) continue;
    uint16_t cm = tofMm[i] / 10u;      // tofMm[] holds mm for every sensor
    CRGB c = RingDistanceColor(cm);
    ringLeds[RING_TOF_LED[i + 1]] = RingScaleColor(c, RingDistanceBrightness(cm));
  }
}

// Ring behaviour, driven by the state machine:
//   IDLE  — live pit telemetry: 5-sensor ToF green->red gradient + white line
//           overlay + green heartbeat on the centre LED, refreshed every 50 ms.
//   FIGHT — ring is OFF (TACTIC/SEARCH/ATTACK/FLANK/EVADE). It is blanked once
//           and FastLED.show() stops being called, so the WS2812 bit-bang never
//           steals loop time or interrupts the ToF bus while fighting.
//           EXCEPTION — DEBUG_VERBOSE builds keep the exact IDLE telemetry
//           running through the fight: same ToF gradient, same line overlay,
//           same heartbeat. That is the whole point of a debug build (watch
//           what the sensors see while the robot actually drives) and it is
//           still safe for the ToF bus, because loop() finishes every sensor
//           read before this function runs FastLED.show().
//   STOP  — match over: whole ring filled solid white (shown once, latched).
//           This wins over the debug telemetry, so the end of the match is
//           unmistakable.
static void ringStatusUpdate() {
  uint8_t st = stateMachineState();

  // Do we want the live telemetry view right now?
#if DEBUG_VERBOSE
  const bool showTelemetry = (st != ST_STOP);   // idle + every fight state
#else
  const bool showTelemetry = (st == ST_IDLE);   // idle only, fight is dark
#endif

  // Fight in a match build -> keep the ring dark and cheap. The blanking
  // happens once here; after RingDisable() every RingShow() is a no-op.
  if (!showTelemetry && st != ST_STOP) {
    if (RingIsEnabled()) RingDisable();
    return;
  }

  if (!RingIsEnabled()) RingEnable();

  // Match over -> solid white, once (LEDs latch their state).
  if (st == ST_STOP) {
    static bool stopLit = false;
    if (stopLit) return;
    RingFill(CRGB::White);
    RingShow();
    stopLit = true;
    return;
  }

  // IDLE (plus every fight state in a debug build) -> live telemetry, 50 ms.
  static uint32_t blinkT = 0;
  static uint32_t hbT = 0;
  static bool blinkOn = false;
  if (millis() - blinkT < 50) return;
  blinkT = millis();
  // Heartbeat toggles slower (250 ms) so it stays a readable blink.
  if (millis() - hbT >= 250) { hbT = millis(); blinkOn = !blinkOn; }

  FastLED.clear();
  renderToFGradient();
  if (lineLeft)    ringLeds[3] = CRGB::White;   // left edge  (front-left)
  if (lineRear)    ringLeds[1] = CRGB::White;   // rear edge  (back)
  if (lineRight)   ringLeds[5] = CRGB::White;   // right edge (front-right)
  if (blinkOn)     ringLeds[0] = CRGB::Green;   // heartbeat (center)
  RingShow();
}

// ============================= DEBUG TELEMETRY =============================
// Prints a labelled table every 400 ms when DEBUG_VERBOSE is on.
// ToF values are the normalised millimetre distances in tofMm[]; "---"
// marks a sensor with no valid reading this pass.

// Loop timing telemetry (µs), measured in loop() and shown in the table.
uint32_t gSensorUs = 0;   // sensorsUpdate() duration (ToF read + line pass)
uint32_t gLoopUs   = 0;   // core loop: sensors + state machine + ring update

#if DEBUG_VERBOSE

static void debugSep() {
  Serial.println(F("--------------------------------------------------------------"));
}

// One ToF value, left-aligned in an 8-char column so the header/value rows
// line up. Invalid sensors print "---".
static void tofCell(uint8_t i) {
  if (tofValid[i]) {
    uint16_t v = tofMm[i];
    Serial.print(v);
    uint8_t d = (v >= 100) ? 3 : ((v >= 10) ? 2 : 1);
    for (uint8_t s = d; s < 8; s++) Serial.print(' ');
  } else {
    Serial.print(F("---     "));   // 3 dashes + 5 spaces = 8 chars
  }
}

static void debugDump() {
  static uint32_t last = 0;
  if (millis() - last < 400) return;
  last = millis();

  int l, r; motorsGetLast(&l, &r);
  bool ok; uint16_t mm = sensorsClosestMm(&ok);

  debugSep();
  Serial.print(F(" t=")); Serial.print(millis());
  Serial.print(F("    State=")); Serial.print(stateName((RobotState)stateMachineState()));
  // In EVADE, say which half of the program is running (flee vs pivot).
  if (stateMachineState() == ST_EVADE) {
    Serial.print(F(" (")); Serial.print(evadePhaseName()); Serial.print(')');
  }
  Serial.print(F("   Strategy=")); Serial.print(strategyIndex);
  Serial.print(F("  Start=")); Serial.println(digitalRead(PIN_START));
  Serial.print(F(" Bearing=")); Serial.print(sensorsOpponent());
  Serial.print(F("   Closest=")); if (ok) Serial.print(mm); else Serial.print(F("---"));
  Serial.print(F("mm  Health=")); Serial.print(sensorsHealthyCount());
  Serial.print('/'); Serial.print(TOF_SENSOR_COUNT);
  Serial.print(F("  Trim=")); Serial.print(trimpotScale()); Serial.println('%');
  Serial.print(F(" Loop=")); Serial.print(gLoopUs); Serial.print(F("us"));
  Serial.print(F("   Sensors=")); Serial.print(gSensorUs); Serial.println(F("us"));
  debugSep();

  // ToF distances (mm), left -> right across the robot.
  // Array index order is 0=SR 1=FR 2=MID 3=FL 4=SL.
  Serial.println(F(" ToF(mm)     SL      FL      MID     FR      SR"));
  Serial.print(F("             "));
  tofCell(4);   // SL  side-left
  tofCell(3);   // FL  front-left
  tofCell(2);   // MID middle
  tofCell(1);   // FR  front-right
  tofCell(0);   // SR  side-right
  Serial.println();
  debugSep();

  Serial.println(F(" Line        Left(A5)  Back(A4)  Right(A1)"));
  Serial.print(F(" Value       "));
  Serial.print(lineRaw[0]); Serial.print(F("        "));
  Serial.print(lineRaw[1]); Serial.print(F("        "));
  Serial.println(lineRaw[2]);
  Serial.print(F(" Edge        "));
  Serial.print(lineLeft); Serial.print(F("          "));
  Serial.print(lineRear); Serial.print(F("          "));
  Serial.println(lineRight);
  debugSep();

  Serial.print(F(" Motor       L=")); Serial.print(l);
  Serial.print(F("       R=")); Serial.println(r);
  debugSep();
}
#endif

void setup() {
  // Serial first: the 3 s startup delay below is the natural window to watch
  // the board come up, and anything printed before Serial.begin() is lost.
  Serial.begin(MONITOR_BAUD);

  // Watchdog housekeeping before anything slow. A reset caused by the watchdog
  // leaves WDRF set in MCUSR with the watchdog STILL ARMED on its shortest
  // timeout, so it must be cleared and switched off before the 3 s delay —
  // otherwise the board would reset in a loop and never finish booting.
  MCUSR = 0;
  wdt_disable();

  // Claim the motor pins next. From power-on until now they float in input
  // mode, which lets the driver chatter the motors (part of the ~0.5 s
  // right-motor spin is the bootloader blinking D13/RM_DIR — hardware, not us).
  motorInit();
  motorsStop();

#if SERIAL_DEBUG
  Serial.println(F("[TAXI] boot - 3s settle"));
#endif

  // Startup grace period: 3 s before anything else happens. Lets the
  // operator step back after power-on. Motors are already held stopped.
  delay(3000);

  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);

  // NanoLabs start module rule: the pin MUST read LOW at power-on
  // (module armed, waiting for the referee start command). If it reads
  // HIGH here, the module still holds the previous match's "on" state —
  // the referee must send stop and the robot must be power-cycled.
  // Lock hard and blink UL1 so this can never be mistaken for a ready robot.
  // Sampled through the same raw read the loop uses — no debounce anywhere
  // in the start/stop path (see StateMachine.h).
  startModuleUpdate();
  if (isStarted()) {
#if SERIAL_DEBUG
    Serial.println(F("[TAXI] FATAL: start pin HIGH at boot - power cycle!"));
#endif
    while (true) {
      digitalWrite(PIN_LED1, HIGH); delay(100);
      digitalWrite(PIN_LED1, LOW);  delay(100);
    }
  }

  sensorsInit();
  strategiesInit();
  RingInit();

#if SERIAL_DEBUG
  Serial.println(F("[TAXI] firmware up"));
#endif
  delay(500);   // give M60F sensors time to boot (10..50 ms each, generous)

  // Arm the watchdog last, when every subsystem is up and the slow startup
  // work (3 s settle + 500 ms sensor boot) is out of the way. See
  // WATCHDOG_ENABLED at the top of this file.
#if WATCHDOG_ENABLED
  wdt_enable(WDTO_250MS);
#endif
}

void loop() {
#if WATCHDOG_ENABLED
  wdt_reset();              // every path through loop() must reach this
#endif
#if DEBUG_VERBOSE
  uint32_t t0 = micros();
#endif
  sensorsUpdate();          // 1 ToF read + line pass (all serial I/O here)
#if DEBUG_VERBOSE
  uint32_t t1 = micros();
#endif
  stateMachineUpdate();     // drives motors, no serial I/O
  ringStatusUpdate();       // FastLED.show() LAST — blocks interrupts
#if DEBUG_VERBOSE
  gSensorUs = t1 - t0;
  gLoopUs   = micros() - t0;   // excludes the debugDump() serial cost below
  debugDump();
#endif
}
