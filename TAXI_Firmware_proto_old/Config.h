/**
 * TAXI Firmware — Configuration Constants
 * 
 * ALL tunable values, pin assignments, thresholds, and magic numbers
 * are defined exactly once in this file. Change them here, not in code.
 * 
 * VERIFIED against:
 *   - xmotionV3.h/.cpp (JSUMO library — hardware pin definitions)
 *   - M60F-TOF-distance-sensor.pdf (register map, timing, protocol)
 *   - Black Magic PID v2/v3 (proven PD controller patterns)
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// 1. PIN ASSIGNMENTS — VERIFIED against xmotionV3.cpp
// ============================================================================

// --- Motor Control (hardcoded in xmotionV3.cpp — DO NOT CHANGE) ---
// LM_PWM = 10, LM_DIR = 12, RM_PWM = 11, RM_DIR = 13
// These are used via xmotion.MotorControl() — no direct pin access needed.

// --- M60F ToF Sensor Bus (half-duplex UART) ---
#define M60F_SERIAL         Serial1         // Hardware Serial1 (D0=RX, D1=TX)
#define M60F_RX_PIN         0               // D0 / PD2 — Serial1 RX
#define M60F_TX_PIN         1               // D1 / PD3 — Serial1 TX

// --- ICM20602 IMU (I²C) ---
#define IMU_SDA_PIN         2               // D2 / PD1
#define IMU_SCL_PIN         3               // D3 / PD0
#define IMU_I2C_ADDRESS     0x68            // AD0 = GND
#define IMU_ENABLED         true            // Set false to disable IMU

// --- WS2812 LED Ring ---
#define LED_RING_PIN        4               // D4 / PD4
#define LED_RING_COUNT      7               // 7-bit ring

// --- START-STOP Module ---
#define START_STOP_PIN      A0              // Per user requirement — use A0 header
// NOTE: A0 was the voltage divider in stock XMotion config.
// User explicitly said "NO VOLTAGE DIVIDER" — A0 is repurposed.

// --- QRE1113 Line Sensors (analog) ---
#define QRE_FL_PIN          A1              // Front-Left corner
#define QRE_FR_PIN          A2              // Front-Right corner
#define QRE_RL_PIN          A4              // Rear-Left corner
#define QRE_RR_PIN          A5              // Rear-Right corner
#define QRE_COUNT           4

// --- DIP Switches (strategy selection) ---
#define DIP1_PIN            5               // PC6 — strategy bit 0 (LSB)
#define DIP2_PIN            6               // PD7 — strategy bit 1
#define DIP3_PIN            7               // PE6 — strategy bit 2 (MSB)

// --- Onboard Trimpot ---
#define TRIMPOT_PIN         A3              // Used via xmotion.Trimpot()

// --- Onboard User LEDs ---
#define USER_LED1_PIN       8               // PB4 — attack indicator
#define USER_LED2_PIN       9               // PB5 — sensor health indicator

// ============================================================================
// 2. M60F SENSOR CONFIGURATION — VERIFIED against M60F datasheet
// ============================================================================

#define M60F_COUNT          5               // Total sensors on bus
#define M60F_BAUD_RATE      250000          // 0% error on 16MHz ATmega32U4 (UBRR=3)
#define M60F_TIMEOUT_US     2000            // Max wait for sensor response (microseconds)
#define M60F_RESPONSE_MAX   8               // Max RX buffer size ("FFF\n" = 4 + newline)
#define M60F_INVALID_DIST   0xFFF           // Returned when sensor fails to respond

// M60F Register addresses (from datasheet)
#define M60F_REG_STATUS     0x00            // Ranging status (read-only)
#define M60F_REG_DIST_MM    0x01            // Distance in mm (read-only)
#define M60F_REG_DIST_CM    0x06            // Distance in cm (read-only)
#define M60F_REG_DETECT     0x07            // Detection output (read-only)
#define M60F_REG_IO_MODE    0x80            // 0=Serial, 1=Digital, 2=PWM
#define M60F_REG_SERIAL_ID  0x81            // 0x00–0xFF unique per sensor
#define M60F_REG_BAUD       0x82            // Baud rate selector
#define M60F_REG_SAMPLE_MS  0xB0            // Sampling time (5–50ms)
#define M60F_REG_DETECT_MODE 0xBA           // Detection mode (0/1/2)
#define M60F_REG_THRESH_LOW 0xBC            // Lower threshold mm
#define M60F_REG_THRESH_HIGH 0xBD           // Upper threshold mm

// Reliable minimum sample period (from datasheet: "Operation below 10ms not guaranteed")
#define M60F_SAMPLE_PERIOD_MS 10

// ============================================================================
// 3. QRE1113 LINE SENSOR CONFIGURATION
// ============================================================================

// Default threshold: values ABOVE this = edge detected (black surface).
// Calibrated per sensor during calibration routine.
// Typical: white ring = 150-300, black edge = 700-900.
#define QRE_DEFAULT_THRESHOLD   650

// Number of consecutive samples above threshold required to trigger EVADE.
// Prevents false triggers from surface scratches.
#define QRE_DEBOUNCE_COUNT      3

// ADC quirk: first analogRead() after channel switch returns residual charge.
// Always discard first reading, use second.
#define QRE_ADC_DISCARD_FIRST   true

// ============================================================================
// 4. PID CONTROLLER CONFIGURATION
// ============================================================================

// --- Lock-on phase (first ~200ms after opponent detection) ---
#define PID_KP_LOCK             35.0f       // Aggressive proportional gain
#define PID_KD_LOCK             8.0f        // Reduced derivative (fast response)
#define PID_LOCK_DURATION_MS    200         // How long lock-on phase lasts

// --- Tracking phase (sustained opponent contact) ---
#define PID_KP_TRACK            28.5f       // Smooth proportional gain  
#define PID_KD_TRACK            15.0f       // High derivative (damping)
#define PID_FADE_DURATION_MS    50          // Transition duration between phases

// --- Motor limits ---
#define PID_BASE_SPEED_ATTACK   80          // Base attack speed (% of max PWM)
#define PID_BASE_SPEED_FLANK    60          // Base flank speed (%)
#define PID_MIN_SPEED           -100        // Minimum motor output %
#define PID_MAX_SPEED           100         // Maximum motor output %

// --- Distance attack zones (mm) ---
#define ZONE_RED_MAX            100         // Full attack speed
#define ZONE_YELLOW_MAX         300         // Approach speed
#define ZONE_GREEN_MAX          500         // Cautious approach
#define ZONE_SPEED_RED          100         // Speed % in red zone
#define ZONE_SPEED_YELLOW       70          // Speed % in yellow zone
#define ZONE_SPEED_GREEN        40          // Speed % in green zone

// --- Sensor weights for PID error calculation ---
// Spatial weights: left-to-right directionality
// M3 (Left 90°) = -4, M1 (Front-Left 25°) = -2, M0 (Center 0°) = 0,
// M2 (Front-Right 25°) = +2, M4 (Right 90°) = +4
#define M60F_WEIGHT_DIRECTION { -4.0f, -2.0f, 0.0f, 2.0f, 4.0f }

// Anti-flag: when front sensors (indices 0,1,2) detect opponent,
// reduce outer sensor weights to prevent wild spinning
#define M60F_WEIGHT_OUTER_NORMAL   5.0f    // Outer sensor weight (normal)
#define M60F_WEIGHT_OUTER_REDUCED  1.5f    // Outer sensor weight (anti-flag)
#define M60F_WEIGHT_INNER          2.0f    // Inner sensor weight (always)

// ============================================================================
// 5. SEARCH BEHAVIOR CONFIGURATION
// ============================================================================

#define SEARCH_SPEED_DEFAULT    60          // Default search speed %
#define SEARCH_ARC_DURATION_MS  200         // Duration of each search arc
#define SEARCH_ARC_RATIO        0.35f       // Speed ratio for arc turns (e.g., 35% diff)
#define SEARCH_TIMEOUT_MS       3000        // Max search before changing pattern

// Last-known-position memory: how long to favor last direction (ms)
#define LAST_KNOWN_TIMEOUT_MS   800

// ============================================================================
// 6. EDGE EVASION CONFIGURATION
// ============================================================================

// Curved re-entry — per sensor trigger (matches user's "curbă" preference)
#define EVADE_REVERSE_MS        80          // Reverse duration before re-entry
#define EVADE_REVERSE_SPEED     100         // Reverse speed %
#define EVADE_CURVE_DURATION_MS 300         // Arc duration for curved re-entry
#define EVADE_PIVOT_DURATION_MS 300         // Pivot duration for both-front trigger

// Arc parameters: {left_motor%, right_motor%} for each triggered sensor
// Front-left → arc right-forward
#define EVADE_FL_LEFT           70
#define EVADE_FL_RIGHT          20
// Front-right → arc left-forward  
#define EVADE_FR_LEFT           20
#define EVADE_FR_RIGHT          70
// Rear-left → drive forward at angle
#define EVADE_RL_LEFT           40
#define EVADE_RL_RIGHT          100
// Rear-right → drive forward at angle
#define EVADE_RR_LEFT           100
#define EVADE_RR_RIGHT          40

// ============================================================================
// 7. STRATEGY SYSTEM
// ============================================================================

#define STRATEGY_COUNT          8           // 3 DIP switches = 8 strategies

// Tactics.json structure will be loaded at compile time from strategies.h
// DIP pins are pulled HIGH internally; switch DOWN = reads 1
// Strategy index = (DIP3<<2) | (DIP2<<1) | DIP1

// ============================================================================
// 8. LED RING PATTERNS
// ============================================================================

#define LED_BRIGHTNESS          64          // 0-255, keep low for battery life
#define LED_COLOR_IDLE          CRGB::Green
#define LED_COLOR_SEARCH        CRGB::Blue
#define LED_COLOR_ATTACK        CRGB::Red
#define LED_COLOR_FLANK_LEFT    CRGB::Yellow
#define LED_COLOR_FLANK_RIGHT   CRGB::Orange
#define LED_COLOR_EVADE         CRGB::White
#define LED_COLOR_ERROR         CRGB::Red
#define LED_COLOR_OK            CRGB::Green

// ============================================================================
// 9. TIMING CONSTANTS
// ============================================================================

#define LOOP_SAFETY_US          100         // Min loop time guard (not needed, just safety)
#define SENSOR_CHECK_INTERVAL   1           // Run sensor update every N loop iterations (1 = every)
#define START_SIGNAL_DEBOUNCE_MS 50         // Debounce the START-STOP module

// ============================================================================
// 10. FAILSAFE CONFIGURATION
// ============================================================================

#define HEALTH_WINDOW           10          // Max health counter value
#define HEALTH_DECREMENT        2           // Penalty per timeout
#define HEALTH_INCREMENT        1           // Reward per successful read
#define HEALTH_FAIL_THRESHOLD   3           // Counter below this = sensor flagged
#define DEGRADED_MAX_TIME_MS    10000       // Max degraded operation before stop

// ============================================================================
// 11. SERIAL DEBUG (disable for competition to save cycles)
// ============================================================================

#define SERIAL_DEBUG            true        // Enable Serial debug output
#define SERIAL_BAUD             9600        // Debug serial baud (USB — not M60F bus)

#endif // CONFIG_H
