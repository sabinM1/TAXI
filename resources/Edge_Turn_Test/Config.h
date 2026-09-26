/*
 * Config.h — every pin, threshold and tunable for the TAXI mini sumo.
 *
 * VERIFIED against:
 *   - resources/Full_Robot_Test  (motor/ring/line/start-stop wiring on THIS board)
 *   - resources/ToF_Ring_Test    (M60F bus shares D4, ids 1..5 @115200 baud)
 *   - resources/xmotion_basics_101.md (DIP switches on D5/D6/D7, DOWN reads 1)
 *
 * The old TAXI_Firmware_proto_old/ pin map (Serial1 bus, 4 line sensors,
 * ring on D4) does NOT match this robot — it was never flashed. Do not copy it.
 *
 * Naming scheme:
 *   PIN_*            physical Arduino pin numbers
 *   *_PCT            dimensionless percent 0..100 (speeds, gains)
 *   *_MM / *_MS      physical units
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// PINS
// ============================================================================

// Motor driver (XMotion V3 board labels: MOT1 = left, MOT2 = right).
// Verified against the official XMotionV3-library: LM_PWM = D10, NOT D3.
// D3 is not wired to the left driver channel on this board.
//
// D10/PB6 is multiplexed as OC1B (Timer1) AND OC4B (Timer4). It is driven from
// Timer4 on purpose: the ToF bus (D4) needs SingleWireSerial, which claims
// Timer1 on the 32U4, and that makes analogWrite()/OC1B dead. See Motor.h.
#define PIN_LM_PWM   10     // left motor PWM  (MOT1, Timer4 OC4B), do not change
#define PIN_LM_DIR   12     // left motor direction
#define PIN_RM_PWM   11     // right motor PWM (MOT2)
#define PIN_RM_DIR   13     // right motor direction

#define PIN_TOF_BUS  4      // M60F shared half-duplex bus (SIG lines tied here,
                            // 4.7k pull-up to 5V on the harness)
#define PIN_RING     A2     // WS2812 7-LED ring data-in (through ~330R)
#define PIN_START    A0     // Start/Stop module signal. HIGH = fight, LOW =
                            // armed/stopped (NanoLabs docs). Read raw, every
                            // loop, with no debounce/filter — see
                            // StateMachine.h. Must read LOW at power-up.
#define PIN_LINE_L   A5     // QRE1113 line sensor, LEFT
#define PIN_LINE_R   A1     // QRE1113 line sensor, RIGHT
#define PIN_LINE_B   A4     // QRE1113 line sensor, REAR
#define PIN_DIP1     5      // strategy bit 0 (LSB); switch ON = bit set
#define PIN_DIP2     6      // strategy bit 1
#define PIN_DIP3     7      // strategy bit 2 (MSB)
#define PIN_LED1     8      // onboard user LED 1 (currently unused at runtime)
#define PIN_LED2     9      // onboard user LED 2 (currently unused at runtime)
#define PIN_TRIMPOT  A3     // onboard trimpot — RESERVED, deliberately not read
                            // by this firmware (see the TRIMPOT section below)

// ============================================================================
// MOTOR POLARITY / SAFETY
// ============================================================================

// DIR HIGH drives a motor forward on both ports, but the LEFT motor is
// wired/mounted opposite on this chassis, so its logical direction is
// flipped here. If the robot spins instead of driving straight, touch
// ONLY these two flags — never patch call sites.
#define LM_INVERTED  true
#define RM_INVERTED  false

// Per-side straight-line trim (percent of duty), see TAXI_Firmware/Motor.h.
#define LM_TRIM_PCT   100
#define RM_TRIM_PCT   100

// Bench-safety clamp: while true, no motor command exceeds
// MOTOR_BENCH_MAX_PCT anywhere (attack, evade, tactics included).
// Set false only for a real match on the dohyo.
#define MOTOR_BENCH_LIMIT_ENABLED   false
#define MOTOR_BENCH_MAX_PCT         25

// ============================================================================
// ToF SENSORS (M60F)
// ============================================================================

#define TOF_SENSOR_COUNT   5        // sensors on the shared bus
#define TOF_BUS_BAUD       115200   // must match sensor register 0xB0
// Per-read timeout for the M60F bus. The read returns the last stored value
// almost instantly (~0.75 ms measured), but under ACTIVE ranging the sensor
// MCU can be busy measuring for its full sampling/timing budget (reg 0xB0,
// currently 12 ms on this robot), delaying a read that lands mid-measurement.
// This must stay above that budget + margin or a late reply leaks into the
// next sensor's read (the old "shifted by one" bug). 15 = 12 ms budget + margin.
#define TOF_TIMEOUT_MS     15
#define MONITOR_BAUD       115200   // USB Serial debug baud

// Valid measurement window (sensor config: 9..800 mm). Anything outside
// (or the no-response sentinel) is treated as "invalid", not as distance.
#define TOF_MIN_VALID_MM   9
#define TOF_MAX_VALID_MM   800
#define TOF_INVALID_CM     0xFF     // sentinel returned by cm sensors on timeout
#define TOF_INVALID_MM     0xFFF    // sentinel returned by the mm sensor (id 3)

// ============================================================================
// LINE SENSORS (QRE1113)
// ============================================================================
//
// Three QRE1113 reflective sensors tell the BLACK dohyo interior apart from
// the WHITE border we must not cross. Raw ADC (0-1023) reads:
//   BLACK interior (weak reflection)  -> HIGH value
//   WHITE border  (strong reflection) -> LOW value
// So an "edge" (white border) is a LOW reading: raw BELOW the white threshold.
//
// Each sensor is a Schmitt trigger with two thresholds (see Sensors.h): it
// latches to "edge" below its WHITE threshold (debounced) and only releases
// once it clearly recovers ABOVE its BLACK threshold. The deadband between
// the two absorbs sensor noise.
//
// Debug print order (lineRaw[0..2]): 0 = LEFT, 1 = REAR, 2 = RIGHT.
// Calibration (verified on the robot, raw ADC 0-1023):
#define LINE_LEFT_WHITE_THRESHOLD   400   // LEFT  (A5) raw BELOW this = white border
#define LINE_LEFT_BLACK_THRESHOLD   800   // LEFT  (A5) raw ABOVE this = black interior
#define LINE_REAR_WHITE_THRESHOLD   765   // REAR  (A4) raw BELOW this = white border
#define LINE_REAR_BLACK_THRESHOLD   820   // REAR  (A4) raw ABOVE this = black interior
#define LINE_RIGHT_WHITE_THRESHOLD  400   // RIGHT (A1) raw BELOW this = white border
#define LINE_RIGHT_BLACK_THRESHOLD  800   // RIGHT (A1) raw ABOVE this = black interior
#define LINE_DEBOUNCE_SAMPLES       2     // consecutive white reads to confirm edge

// ============================================================================
// TIMING
// ============================================================================

#define SERIAL_DEBUG      true   // boot/strategy messages (cheap, few lines)
// Full telemetry mode: state transitions + a 400 ms status table with sensors,
// bearing, motors, edge flags and loop timing. Also the switch that keeps the
// ring telemetry alive during a fight, and the one that arms the watchdog.
// Disable for matches: the serial table blocks the loop for ~50 ms per dump
// (USB CDC at 115200), which is wasted time and skews sensor freshness.
#define DEBUG_VERBOSE     true
#define SEARCH_PATTERN_TIMEOUT_MS  3000   // no opponent for this long in
                                            // SEARCH -> flip search direction
#define ATTACK_TARGET_LOST_MS      400    // target invisible this long in
                                            // ATTACK/FLANK -> back to SEARCH
// NOTE: the Start/Stop module deliberately has NO debounce constant here.
// Its output is a latched, clean logic level (the NanoLabs module stores its
// state in non-volatile memory), so StateMachine.h reads the pin directly and
// reacts on the same loop. Never add filtering to that path: it would only
// delay the referee's kill command.

// ============================================================================
// COMBAT SPEEDS
// ============================================================================

// Attack base speed ramps from ATTACK_SPEED_CLOSE_PCT (point-blank) down to
// ATTACK_SPEED_FAR_PCT (at ATTACK_FAR_DIST_MM), linearly in between.
#define ATTACK_SPEED_CLOSE_PCT  100
#define ATTACK_SPEED_FAR_PCT    75
#define ATTACK_CLOSE_DIST_MM    250   // <= this: full push
#define ATTACK_FAR_DIST_MM      400   // >= this: ATTACK_SPEED_FAR_PCT

// Blade-to-blade contact. The MIDDLE ToF (id 3, looking straight ahead) reading
// this close means the opponent is on the blade. From there both wheels go to
// PUSH_CONTACT_SPEED_PCT straight ahead and the steering trim is skipped: a
// pushing contest is won by whoever still has both wheels driving, and at that
// range a correction would only give one of them away. It also outranks the
// switch to FLANK — contact is worth more than re-centering first.
#define PUSH_CONTACT_DIST_MM     30   // MIDDLE sensor distance that means contact
#define PUSH_CONTACT_SPEED_PCT  100   // both wheels, no differential
#define ATTACK_STEER_GAIN_PCT  100    // % of base speed applied as full
                                      // differential at bearing = +/-100.
                                      // Higher = steers onto the opponent harder
                                      // while pushing (more aggressive tracking).

// ----- which way the robot thinks the opponent is ---------------------------
// Bearing units, the scale of sensorsOpponent(): -100 hard left, 0 dead ahead,
// +100 hard right.
//
// ATTACK (push, steering with the gain above) and FLANK (pivot to re-center) are
// separated by TWO thresholds, not one. With a single threshold the command
// jumped from "a ~10% differential" to "a full pivot" the moment the bearing
// crossed it, so one extra sensor lighting up was enough to spin the robot over a
// target that was nearly straight ahead. The dead zone between the two numbers
// removes that cliff and stops the two states chattering.
#define TARGET_ON_AXIS_BAND     18   // |bearing| <= this -> ATTACK
#define TARGET_FLANK_BAND       26   // |bearing| >  this -> FLANK
                                     // in between: stay in the state we are in

// There is deliberately NO bearing memory/filter: the robot steers on the live
// reading. Any smoothing lagged the bearing during a pivot and turned corrections
// into spins.

#define SEARCH_SPEED_PCT        20    // base speed of all search patterns
#define FLANK_TURN_SPEED_PCT   100    // pivot speed at |bearing| = 100 (inner
                                      // wheel counterspins at half this)
#define FLANK_TURN_MIN_PCT      56    // pivot speed at the FLANK entry threshold,
                                      // scaled linearly up to FLANK_TURN_SPEED_PCT
                                      // above it: full power for a big correction,
                                      // a nudge for a small one. One value for both
                                      // meant every entry pivoted flat out and
                                      // overshot the target
#define EVADE_REVERSE_SPEED_PCT -80   // escape burst away from an edge

// ============================================================================
// TRIMPOT — NOT USED
// ============================================================================
// The onboard trimpot is deliberately not read anywhere in this firmware: every
// speed below is an absolute percent constant, so the same DIP selection gives
// the same run every time, with no knob to drift between matches or to be left
// in the wrong position by accident. PIN_TRIMPOT stays defined above so the pin
// remains documented and reserved (A3 is not used for anything else).

// ============================================================================
// ToF HEALTH FAILSAFE
// ============================================================================

#define TOF_HEALTH_MAX            8   // per-sensor counter ceiling
#define TOF_MIN_HEALTHY_SENSORS   3   // fewer healthy sensors than this ->
                                      // degraded mode (safe blind spin-search)

// ============================================================================
// EVADE TIMING
// ============================================================================
//
// One evade program is FLEE -> TURN -> hand over to SEARCH. Both phases are
// timed, so the same edge always produces the same escape. See evadeUpdate().

#define EVADE_ESCAPE_MIN_MS   15    // FLEE floor: the mandatory blind burst.
                                    // Short on purpose — it only exists so one
                                    // flickering sample cannot end the flee. The
                                    // moment the sensors read black the phase ends
                                    // anyway, so a small floor means the robot
                                    // turns as soon as it is actually clear.
                                    // Tuned on the robot with Edge_Turn_Test.
#define EVADE_FLEE_MAX_MS     300   // FLEE ceiling: once this long has passed
                                    // with a sensor STILL reading white, stop
                                    // fleeing anyway — that run cannot be
                                    // measured, and pivoting over white would be
                                    // pivoting along the edge.
#define EVADE_TURN_MS         80    // *** OVERSHOOT KNOB *** duration of the
                                    // pivot away from the edge. Tuned on the
                                    // robot with resources/Edge_Turn_Test
                                    // (see its GUIDE).
// Pivot speeds used by the TURN phase (evadeTurnAway()). These set the pivot
// RATE; together with EVADE_TURN_MS they set how far the robot rotates.
#define EVADE_TURN_OUTER_PCT   70   // outer wheel, forward
#define EVADE_TURN_INNER_PCT   50   // inner wheel, applied reversed
#define EVADE_TURN_SPIN_PCT    60   // both wheels, when the REAR edge tripped
                                    // (spin in place instead of an arc)
// Safety net for one whole evade program — NOT a tuning knob, and deliberately
// generous. It only fires if FLEE and TURN keep re-arming each other, which
// needs a line sensor stuck reading white (dead or disconnected). The escape
// duration itself is set by the two phase constants above.
#define EVADE_TOTAL_MAX_MS    800

#endif // CONFIG_H
