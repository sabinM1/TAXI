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
#define PIN_LM_PWM   3      // left motor PWM  (MOT1)
#define PIN_LM_DIR   12     // left motor direction
#define PIN_RM_PWM   11     // right motor PWM (MOT2)
#define PIN_RM_DIR   13     // right motor direction

#define PIN_TOF_BUS  4      // M60F shared half-duplex bus (SIG lines tied here,
                            // 4.7k pull-up to 5V on the harness)
#define PIN_RING     A2     // WS2812 7-LED ring data-in (through ~330R)
#define PIN_START    A0     // Start/Stop module signal (HIGH = fight, see
                            // NanoLabs docs; must read LOW at power-up)
#define PIN_LINE_L   A5     // QRE1113 line sensor, LEFT
#define PIN_LINE_R   A1     // QRE1113 line sensor, RIGHT
#define PIN_LINE_B   A4     // QRE1113 line sensor, REAR
#define PIN_DIP1     5      // strategy bit 0 (LSB); switch ON = bit set
#define PIN_DIP2     6      // strategy bit 1
#define PIN_DIP3     7      // strategy bit 2 (MSB)
#define PIN_LED1     8      // onboard user LED 1 (currently unused at runtime)
#define PIN_LED2     9      // onboard user LED 2 (currently unused at runtime)
#define PIN_TRIMPOT  A3     // onboard trimpot = live master-speed knob

// ============================================================================
// MOTOR POLARITY / SAFETY
// ============================================================================

// DIR HIGH drives a motor forward on both ports, but the LEFT motor is
// wired/mounted opposite on this chassis, so its logical direction is
// flipped here. If the robot spins instead of driving straight, touch
// ONLY these two flags — never patch call sites.
#define LM_INVERTED  true
#define RM_INVERTED  false

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
#define LINE_LEFT_WHITE_THRESHOLD   460   // LEFT  (A5) raw BELOW this = white border
#define LINE_LEFT_BLACK_THRESHOLD   920   // LEFT  (A5) raw ABOVE this = black interior
#define LINE_REAR_WHITE_THRESHOLD   850   // REAR  (A4) raw BELOW this = white border
#define LINE_REAR_BLACK_THRESHOLD   960   // REAR  (A4) raw ABOVE this = black interior
#define LINE_RIGHT_WHITE_THRESHOLD  460   // RIGHT (A1) raw BELOW this = white border
#define LINE_RIGHT_BLACK_THRESHOLD  920   // RIGHT (A1) raw ABOVE this = black interior
#define LINE_DEBOUNCE_SAMPLES       2     // consecutive white reads to confirm edge

// ============================================================================
// TIMING
// ============================================================================

#define SERIAL_DEBUG      true   // boot/strategy messages (cheap, few lines)
// Full telemetry mode: state transitions + a 200 ms status dump with
// sensors, bearing, motors, edge flags. Disable for matches (Serial costs
// loop time on the 32u4).
#define DEBUG_VERBOSE     true
#define SEARCH_PATTERN_TIMEOUT_MS  3000   // no opponent for this long in
                                            // SEARCH -> flip search direction
#define ATTACK_TARGET_LOST_MS      400    // target invisible this long in
                                            // ATTACK/FLANK -> back to SEARCH
#define START_SIGNAL_DEBOUNCE_MS   50     // start module chatter filter
                                            // (NanoLabs latches state internally)

// ============================================================================
// COMBAT SPEEDS
// ============================================================================

// Attack base speed ramps from ATTACK_SPEED_CLOSE_PCT (point-blank) down to
// ATTACK_SPEED_FAR_PCT (at ATTACK_FAR_DIST_MM), linearly in between.
#define ATTACK_SPEED_CLOSE_PCT  100
#define ATTACK_SPEED_FAR_PCT    60
#define ATTACK_CLOSE_DIST_MM    150   // <= this: full push
#define ATTACK_FAR_DIST_MM      400   // >= this: cautious push
#define ATTACK_STEER_GAIN_PCT   50    // % of base speed applied as full
                                      // differential at bearing = +/-100

#define SEARCH_SPEED_PCT        45    // base speed of all search patterns
#define FLANK_TURN_SPEED_PCT    60    // pivot push while re-centering a target
#define EVADE_REVERSE_SPEED_PCT -80   // escape burst away from an edge

// ============================================================================
// TRIMPOT (live master-speed knob)
// ============================================================================

// Trimpot scales attack/flank/search speeds linearly from
// TRIMPOT_MIN_SCALE_PCT (knob fully CCW) to 100% (fully CW).
#define TRIMPOT_MIN_SCALE_PCT   50

// ============================================================================
// ToF HEALTH FAILSAFE
// ============================================================================

#define TOF_HEALTH_MAX            8   // per-sensor counter ceiling
#define TOF_MIN_HEALTHY_SENSORS   3   // fewer healthy sensors than this ->
                                      // degraded mode (safe blind spin-search)

// ============================================================================
// EVADE TIMING
// ============================================================================

#define EVADE_ESCAPE_MIN_MS   60    // mandatory blind escape burst — the robot
                                    // may still be rolling off the dohyo even
                                    // if the edge briefly stops reading
#define EVADE_TOTAL_MAX_MS    400   // max time in EVADE before giving control
                                    // back to SEARCH

#endif // CONFIG_H
