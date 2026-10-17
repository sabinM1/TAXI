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
#define PIN_DIP1     5      // strategy bit 2 (MSB); switch ON = bit set
#define PIN_DIP2     6      // strategy bit 1
#define PIN_DIP3     7      // strategy bit 0 (LSB)
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

// ============================================================================
// DRIVE TRIM (straight-line calibration)
// ============================================================================
// No two motors, gearboxes or wheel fits are identical, so the same command to
// both sides makes the robot curve. These scale each side's PWM duty to
// compensate, applied in motorWrite() after the percent -> duty conversion.
//
// 100 = no change. Only ever trim the FASTER side DOWN — duty cannot exceed 255,
// so there is no headroom to trim a slow side up.
//
// Calibrate on the floor: command a straight line, see which way it curves. The
// robot turns TOWARD THE SLOWER SIDE, so if it curves LEFT the RIGHT side is
// doing more work and RM_TRIM_PCT is the knob (and vice versa). Step it down a
// few points (98, 95, ...) until it tracks straight. 1-3 % is usually enough;
// the cost is only that the trimmed side's top speed drops by the same few
// percent.
//
// Sanity check for the direction, because it is easy to get backwards: on equal
// commands a differential drive turns left exactly when the RIGHT wheel covers
// more ground (v_right - v_left = yaw rate x track width). "Curves left" therefore
// means the right is faster, not the left.
//
// NOTE: the two PWM channels were given the same frequency on purpose (see
// Motor.h) so this trim only has to cover mechanical differences, not an
// electrical one.
#define LM_TRIM_PCT   100   // left  side duty scale (%)
#define RM_TRIM_PCT    98   // right side duty scale (%)

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
// Per-read timeout for the M60F bus. This is a MAXIMUM, not a fixed wait: the
// read returns the instant the sensor's reply terminator arrives (readBytesUntil
// stops on '\n'), so a healthy sensor costs ~0.75 ms regardless of this value.
// It only bounds the failure case. Under ACTIVE ranging the sensor MCU can be
// busy measuring for its full sampling/timing budget (reg 0xB0, now 45 ms on
// this robot), delaying a read that lands mid-measurement. This must stay above
// that budget + margin or a late reply leaks into the next sensor's read (the
// old "shifted by one" bug). 55 = 45 ms budget + margin.
#define TOF_TIMEOUT_MS     55
#define MONITOR_BAUD       115200   // USB Serial debug baud. NOMINAL ONLY: on the
                                    // ATmega32U4 Serial is USB CDC, not a UART, so
                                    // this does not change the transfer rate (the
                                    // host just uses it to pick a port speed). The
                                    // real readout-rate knob is DEBUG_DUMP_MS.

// Valid measurement window (sensor config: 9..800 mm). Anything outside
// (or the no-response sentinel) is treated as "invalid", not as distance.
#define TOF_MIN_VALID_MM   9
#define TOF_MAX_VALID_MM   800
#define TOF_INVALID_CM     0xFF     // sentinel returned by cm sensors on timeout
#define TOF_INVALID_MM     0xFFF    // sentinel returned by the mm sensor (id 3)

// ----- phantom filter -------------------------------------------------------
// The M60F black-target return is weak (signal ~4-15 at 200-300 mm), the same
// magnitude as the sensor's own crosstalk/ambient noise. No register setting
// separates them (B5, B8 and B2 were all swept and all kill black at the same
// point they kill the phantoms). The phantoms ARE filterable in time, though:
// a real target reads valid on consecutive polls, a phantom is a stray frame.
//
// A sensor's reading is only published to tofValid[]/tofMm[] after this many
// CONSECUTIVE valid reads. 3 was chosen from the bench data: black reads ~96%
// valid at 200 mm (0.96^3 = 88% still detected) while empty-space phantoms read
// ~7-13% valid (0.13^3 = 0.2% false positives). Set to 1 to disable the filter.
#define TOF_CONSECUTIVE_VALID  3

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
#define LINE_LEFT_WHITE_THRESHOLD   600   // LEFT  (A5) raw BELOW this = white border
#define LINE_LEFT_BLACK_THRESHOLD   750   // LEFT  (A5) raw ABOVE this = black interior
#define LINE_REAR_WHITE_THRESHOLD   780   // REAR  (A4) raw BELOW this = white border
#define LINE_REAR_BLACK_THRESHOLD   810   // REAR  (A4) raw ABOVE this = black interior
#define LINE_RIGHT_WHITE_THRESHOLD  600   // RIGHT (A1) raw BELOW this = white border
#define LINE_RIGHT_BLACK_THRESHOLD  750   // RIGHT (A1) raw ABOVE this = black interior
#define LINE_DEBOUNCE_SAMPLES       2     // consecutive white reads to confirm edge

// ============================================================================
// TIMING
// ============================================================================

#define SERIAL_DEBUG      true   // boot/strategy messages (cheap, few lines)

// ----- strategy test mode ---------------------------------------------------
// Run ONLY the opening tactic, then stop. After the DIP-selected opening
// finishes, the robot halts and holds instead of handing off to SEARCH, so a
// single opening can be watched in isolation (does it end up where the designer
// says? does it clear the border? how far does it travel?).
//
// The line-edge override is DISABLED in this mode, so the robot will not evade
// the border — the line sensors are ignored entirely. The ToF handoff inside the
// opening is also disabled: the opening runs fully open-loop for its whole
// duration, which is what you want when measuring the move itself.
//
// When the opening finishes the robot stops and STAYS stopped; the only way out
// is a power cycle. Set false for normal combat.
#define STRATEGY_TEST     false
// Full telemetry mode: state transitions + a 400 ms status table with sensors,
// bearing, motors, edge flags and loop timing. Also the switch that keeps the
// ring telemetry alive during a fight, and the one that arms the watchdog.
// Disable for matches: the serial table blocks the loop for ~50 ms per dump
// (USB CDC at 115200), which is wasted time and skews sensor freshness.
#define DEBUG_VERBOSE     true
// How often the status table is printed, in ms — the debug READOUT RATE.
//
// The table is ~580 bytes, and Serial.print() BLOCKS whenever the 32U4's USB CDC
// TX buffer (64 B) is full, so each dump costs roughly 5-20 ms of loop time
// depending on how fast the host drains it. That cost is the floor on this knob:
// the ToF bus is read ONE sensor per loop (round-robin), so once the dump eats a
// large share of the loop the full five-sensor sweep stretches out and the
// printed Bearing/ToF values start describing a moment that has already passed.
//
//   Loop= / Sensors= stay accurate at ANY rate — they are sampled in loop()
//   before debugDump() runs, so the serial cost is excluded from them.
//
//   400   the old hard-coded value: 4+ loop ticks between rows, very easy to read
//   100   4x faster than the old value, still readable, roughly 10 % loop overhead
//    50   CURRENT: impact-level resolution, ~20-40 % overhead — expect some
//         staleness between five-sensor sweeps
//   <50   the serial cost dominates the loop; treat the numbers as indicative
#define DEBUG_DUMP_MS      50
// Live status format.
//   true  = a labelled column HEADER plus one value ROW, redrawn in place with
//           a carriage return. No ANSI needed, so it works in any monitor, and
//           the row never scrolls. The header is reprinted above EVERY row, so
//           each value is always labelled.
//   false = the original multi-line table, which scrolls the terminal every
//           dump (that scrolling is what made the output flicker).
// The row is DEBUG_LINE_WIDTH chars wide; keep the terminal at least that wide
// or it wraps and the redraw smears.
#define DEBUG_LIVE_LINE   true
#define DEBUG_LINE_WIDTH  111
#define SEARCH_PATTERN_TIMEOUT_MS  3000   // no opponent for this long in
                                            // SEARCH -> flip search direction
#define ATTACK_TARGET_LOST_MS      400    // target invisible this long in
                                            // ATTACK -> back to SEARCH
// NOTE: the Start/Stop module deliberately has NO debounce constant here.
// Its output is a latched, clean logic level (the NanoLabs module stores its
// state in non-volatile memory), so StateMachine.h reads the pin directly and
// reacts on the same loop. Never add filtering to that path: it would only
// delay the referee's kill command.

// ============================================================================
// COMBAT SPEEDS
// ============================================================================

// ----- when does the robot commit to the attack? -----------------------------
// ONE distance decides it, and it is used in two places that must agree:
//   - SEARCH hands over to ATTACK at/inside TARGET_ENGAGE_MM;
//   - a sighting at/inside it is also the one the robot TRUSTS and remembers as
//     "where the opponent was last seen" (see targetBearing()).
// Farther out the ToF bearing is weak and jumpy, so the robot stays in SEARCH and
// keeps hunting — turning onto whatever it sees at SEARCH_SPEED_PCT, which is also
// the speed of the whole approach beyond this distance — instead of committing to
// a charge on a ghost.
//
// This replaced the old pair of bearing thresholds (TARGET_ON_AXIS_BAND /
// TARGET_FLANK_BAND) plus the separate LAST_SEEN_MAX_MM: the mode switch is now
// "how far away is it", not "how far off-axis is it", which is both easier to
// reason about and impossible to chatter (a distance is monotonic; a bearing
// bouncing either side of a threshold is not).
//
// It is also the FAR anchor of the attack speed ramp below, so the speed the robot
// approaches at and the distance it is willing to engage from stay one decision.
//
// Lower it to make the robot hunt longer before engaging; raise it to commit
// earlier and from further out.
#define TARGET_ENGAGE_MM       300

// ----------------------------------------------------------------------------
// ATTACK SPEED PROFILE
// ----------------------------------------------------------------------------
// The base speed as a function of the CLOSEST opponent reading. It is NOT a
// straight line: it is a small table of (distance, speed) anchors, linearly
// interpolated between them. The anchors are the shape, so the profile can be
// whatever is wanted without fighting the code:
//
//   closest mm :  >300   300   250   200   150   100    80    60   <60
//   base speed :    30    35    40    45    52    60    80   100   100
//                   ^     |                      |     |
//    SEARCH_SPEED --+     |                      |     +-- ATTACK_CLOSE_DIST_MM:
//    (hunting; no         |                      |         full power from here in
//     attack yet)         |                      +-------- 60 % at 100 mm
//                         +-- ATTACK_SPEED_FAR_PCT at TARGET_ENGAGE_MM
//                             (the distance SEARCH hands over at)
//
// The shape is the whole point. The robot creeps in at a third of its speed
// while the reading could still be anything, builds speed as the range closes,
// and only commits everything inside the last 60 mm — where there is no room left
// to correct and the only thing that matters is the shove. Carrying speed at
// range is what makes the robot arrive at an off-centre opponent before it has
// finished turning onto it, sail past and have to re-acquire.
//
// The last anchor deliberately coincides with PUSH_CONTACT_DIST_MM, so the speed
// is already at PUSH_CONTACT_SPEED_PCT when the contact latch takes over: no step
// in the command at the moment of impact.
//
// 250 mm is not listed as an anchor because it falls on the 300 -> 200 segment
// anyway (35 -> 45 passes through 40 there). Add anchors, in FAR -> NEAR order,
// to reshape any part of the curve.
#define ATTACK_SPEED_CLOSE_PCT  100   // anchor value at ATTACK_CLOSE_DIST_MM
#define ATTACK_SPEED_FAR_PCT     35   // anchor value at TARGET_ENGAGE_MM, and
                                      // the blind push speed when nothing is
                                      // visible at all
#define ATTACK_CLOSE_DIST_MM     60   // full power from here in

// Anchors must be ordered far -> near (non-increasing mm).
static const struct AttackSpeedPoint { uint16_t mm; uint8_t pct; } ATTACK_SPEED_RAMP[] = {
  { TARGET_ENGAGE_MM,     ATTACK_SPEED_FAR_PCT   },   // 300 mm -> 35 %
  { 200,                  45                    },
  { 100,                  60                    },
  { ATTACK_CLOSE_DIST_MM, ATTACK_SPEED_CLOSE_PCT },   //  60 mm -> 100 %
};
#define ATTACK_SPEED_RAMP_POINTS \
  (sizeof(ATTACK_SPEED_RAMP) / sizeof(ATTACK_SPEED_RAMP[0]))

// Blade-to-blade contact. The MIDDLE ToF (id 3, looking straight ahead) reading
// this close means the opponent is on the blade. From there both wheels go to
// PUSH_CONTACT_SPEED_PCT straight ahead and the steering trim is skipped: a
// pushing contest is won by whoever still has both wheels driving, and at that
// range a correction would only give one of them away. It outranks everything
// else in ST_ATTACK — contact is worth more than re-centering first.
//
// DISTANCE is deliberately generous rather than 30 mm. A single M60F reading can
// be up to its 12 ms sampling budget old, and at impact the blade goes inside the
// sensor's 9 mm minimum so the reading can go INVALID altogether — with a tight
// threshold the robot reacts late or not at all and stops shoving at the worst
// moment. Triggering the full-power shove earlier costs nothing on a head-on.
#define PUSH_CONTACT_DIST_MM     60   // MIDDLE sensor distance that means contact
#define PUSH_CONTACT_SPEED_PCT  100   // both wheels, no differential
// Contact LATENCY: once contact is seen, keep treating it as contact for this
// long even if the reading goes invalid or drifts (the blade inside the sensor's
// minimum range) — a shove must not be interrupted by one bad frame. This is what
// makes the robot commit instantly on a head-on instead of hesitating mid-push.
#define CONTACT_LATCH_MS        250

// STRAIGHTEN BEFORE IMPACT — the anti-lever fix.
//
// A blade that arrives mid-turn hits with a corner, and a corner is exactly what
// the opponent's wedge gets its leading edge under. The steering law is
// proportional-only, so it keeps correcting right up to contact, and whatever yaw
// rate the robot has at 60 mm it still has at impact.
//
// So inside STRAIGHTEN_DIST_MM the steering correction is faded out linearly,
// reaching zero at PUSH_CONTACT_DIST_MM, which lets the yaw rate decay and the
// plow land square. Only the MIDDLE ToF arms the taper: its 18-22 degree cone is
// narrower than the steering dead zone, so a target it can see at this range is
// already effectively dead ahead, and anything off to a side does not arm it — a
// target being turned onto is never abandoned mid-turn.
//
// Deliberately a SEPARATE constant from PUSH_CONTACT_DIST_MM rather than just
// raising that one: PUSH_CONTACT_DIST_MM is also the near anchor of
// ATTACK_SPEED_RAMP, so moving it would jump the base speed from ~53 % to 100 %
// 60 mm earlier as well as giving up the steering earlier. Here the speed ramp is
// untouched, so the robot still accelerates into the shove while it straightens.
//
// Lower it to keep steering deeper into the engagement; raise it to straighten
// sooner (and give up more last-instant correction).
#define STRAIGHTEN_DIST_MM     120

// How hard the robot turns onto the opponent while chasing. The differential
// applied at bearing +/-100 is this many percent of the CURRENT BASE SPEED (not
// a fixed pivot rate). Scaling the correction by the speed is what lets one law
// do both jobs:
//   - at attack speed (35..100 %) a hard bearing counter-rotates the inner wheel
//     and swings the robot round onto the target;
//   - at the slow search/approach speed the exact same bearing is only a gentle
//     arc, so the robot never stalls or spins on the spot while hunting.
// That is why the separate ATTACK (arc) and FLANK (pivot) states are gone: there
// is no longer a bearing threshold at which the robot stops making forward
// progress. See steerToward() in StateMachine.h.
//
// 100 means the correction at full bearing exactly cancels the base speed: the
// inner wheel stops but never reverses, which is the slowest possible pivoting
// turn. That is why the robot was still rotating when it reached the opponent —
// it had the authority of a stopped wheel and the whole approach to use it. 120
// pushes the inner wheel into a mild counter-rotation at full bearing while
// leaving small corrections proportional, so the turn-in finishes during the
// approach instead of during the impact.
//
// Raise it further for a tighter turn-in, until the robot weaves or oscillates —
// with a proportional-only law and no smoothing, oscillation is the failure mode.
// Step it up ~20 at a time and re-test; the next step from here is 140.
#define ATTACK_STEER_GAIN_PCT  120

// SEARCH usually approaches a far sighting gently, so its base speed (30) with
// the normal 120% gain yields only a small correction. A live, side-dominant
// opponent needs a real pivot even while outside TARGET_ENGAGE_MM. At base 30,
// 200% gives a correction of 60 at full bearing: the inner wheel runs at -30%
// while the outer reaches +90%, so the robot turns hard without switching to a
// blind in-place search pattern. Front-led approaches keep the normal gain.
#define SEARCH_SIDE_STEER_GAIN_PCT 200

#define SEARCH_SPEED_PCT        30   // base speed of every search pattern, and of
                                     // the approach leg when a far sighting is
                                     // being turned onto — i.e. the speed
                                     // everywhere outside TARGET_ENGAGE_MM. Fast
                                     // enough to cover the dohyo and to have a
                                     // real turn rate (the steering correction is
                                     // scaled by this), slow enough not to
                                     // overshoot a target it has not finished
                                     // turning onto yet.
#define EVADE_REVERSE_SPEED_PCT -80   // escape burst away from an edge

// ============================================================================
// SIDE-SENSOR BACKOFF
// ============================================================================
//
// The two SIDE ToF sensors (ID1 side right, ID5 side left) look perpendicular to
// the robot, so an opponent they see at close range is ALONGSIDE. The bearing
// vote (inverse-distance-squared) naturally lets side sensors dominate when the
// opponent is alongside, so the robot turns aggressively onto it. At very close
// range (SIDE_BACKOFF_DIST_MM) the side-backoff triggers a hard reverse to open
// the gap, then SEARCH turns to bring the opponent onto the front sensors.
//
// NOTE: the sides are always polled (there is no front-only scan any more), so
// this can fire from normal combat states, ATTACK included. It deliberately
// cannot interrupt EVADE.
#define SIDE_BACKOFF_DIST_MM      20   // side ToF distance that triggers the
                                       // reverse. Tightened from 30 mm: the side
                                       // sensors are perpendicular, so a slightly
                                       // angled opponent (or a dohyo wall) can
                                       // cross their 30 mm band without ever
                                       // being alongside. 20 mm means "alongside
                                       // and touching", which is the only case
                                       // this state exists for.
#define SIDE_BACKOFF_CONFIRM_MS  350   // the side reading must HOLD at/inside
                                       // DIST_MM for this long before the reverse
                                       // fires — one grazing frame or a stale
                                       // reading must not trigger it. Cleared the
                                       // moment the reading goes back out.
                                       // Long (350 ms) on purpose: the reverse is
                                       // a full-power move away from the opponent,
                                       // so it must only ever fire on sustained
                                       // contact, never on a brush past the sensor.
#define SIDE_BACKOFF_MS          100   // reverse duration (the "temporary" part)
#define SIDE_BACKOFF_SPEED_PCT  -100   // full-speed reverse
#define SIDE_BACKOFF_COOLDOWN_MS 300   // minimum gap between backoffs, so a target
                                       // that stays alongside cannot make the
                                       // robot back up forever

// ============================================================================
// RING SPEED INDICATOR
// ============================================================================
// Ring LED 1 (the BACK led, normally only the rear line indicator) also shows the
// robot's current drive effort: cyan at 100 % speed fading to green at 0 %.
#define RING_SPEED_LED_BRIGHTNESS 102   // 0..255 -> 40 % of normal brightness

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
#define TOF_MIN_HEALTHY_SENSORS   3   // fewer ANSWERING sensors than this ->
                                      // degraded mode (safe blind spin-search).
                                      // "Answering", not "seeing something": a
                                      // sensor with no object in range is still
                                      // healthy. See tofReadOne().
#define DEGRADED_PUSH_SPEED_PCT  20   // blind push speed in ATTACK while the bus
                                      // is degraded (below), instead of the normal
                                      // ATTACK_SPEED_FAR_PCT. Deliberately the
                                      // slowest number in the file: the robot is
                                      // blind AND the target has been lost, so it
                                      // only creeps forward on momentum.

// ============================================================================
// EVADE TIMING
// ============================================================================
//
// One evade program is FLEE -> TURN -> hand over to SEARCH. Both phases are
// timed, so the same edge always produces the same escape. See evadeUpdate().

#define EVADE_ESCAPE_MIN_MS   35    // FLEE floor: the mandatory blind reverse
                                    // burst, executed before the robot is allowed
                                    // to look at the line sensors again. It is
                                    // what guarantees a line clip actually gets
                                    // backed away from instead of just being
                                    // twitched clear — a very short floor let the
                                    // robot clip the border, register it, and
                                    // drive straight back over the same spot.
                                    // Long enough to move a real distance at
                                    // EVADE_REVERSE_SPEED_PCT (~15 mm), short
                                    // enough that it cannot carry the robot off
                                    // the dohyo on its own.
                                    // Tuned on the robot with Edge_Turn_Test.
#define EVADE_FLEE_MAX_MS     300   // FLEE ceiling: once this long has passed
                                    // with a sensor STILL reading white, stop
                                    // fleeing anyway — that run cannot be
                                    // measured, and pivoting over white would be
                                    // pivoting along the edge.
#define EVADE_TURN_MS        120    // *** OVERSHOOT KNOB *** duration of the
                                    // pivot away from the edge, i.e. how far the
                                    // robot rotates: angle = this x the pivot
                                    // rate below. Raised so the robot is left
                                    // pointing decisively back into the ring
                                    // instead of marginally off the border.
                                    // Tuned on the robot with
                                    // resources/Edge_Turn_Test (see its GUIDE).
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
