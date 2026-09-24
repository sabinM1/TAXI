/**
 * RingInfoDisplay.h — WS2812 7-LED Ring status display library
 *
 * Self-contained header (single include) that drives a 7-LED WS2812 ring
 * using FastLED. Designed as a "library / header" so the rest of your
 * firmware just calls a few clear functions — it does NOT read any sensors
 * itself. You feed it distance values and line-sensor states; it renders
 * them onto the ring.
 *
 * Features:
 *   - Per-LED RGB control (set any LED to any 8-bit R/G/B color)
 *   - Global non-destructive brightness (luminosity) scaling
 *   - 7-LED ring layout helpers (outer 1..6 clockwise, LED 0 = center)
 *   - Clear the whole ring (all blank)
 *   - Disable / enable the ring (no-op show() when disabled) — call once
 *     the match starts and you no longer need the visual feedback
 *   - High-level "useful info" renderers for ToF ranges and line detection
 *
 * Physical ring layout (verified against hardware):
 *   LED 0 = center
 *   LED 1 = back      (6 o'clock)
 *   LED 2 = left back (~8 o'clock)
 *   LED 3 = left front (~10 o'clock)
 *   LED 4 = front     (12 o'clock)
 *   LED 5 = right front (~2 o'clock)
 *   LED 6 = right back (~4 o'clock)
 *
 * Sensor layout assumptions (documented so you can re-map in one place):
 *   ToF (5 sensors, configured range 9 mm .. 800 mm; cm EXCEPT ID3 which is mm):
 *     ID1 = side right      -> LED 6  (right back)
 *     ID2 = right front     -> LED 5  (right front)
 *     ID3 = middle (mm)     -> LED 4  (front)
 *     ID4 = front left      -> LED 3  (left front)
 *     ID5 = side left       -> LED 2  (left back)
 *   Line (3 sensors):
 *     LINE0 = left  -> LED 3
 *     LINE1 = center-> LED 4
 *     LINE2 = right -> LED 5
 *   Center LED 0 = aggregate "attention" indicator.
 *
 * Requires: FastLED library (Arduino Library Manager -> "FastLED").
 */

#ifndef RING_INFO_DISPLAY_H
#define RING_INFO_DISPLAY_H

#include <Arduino.h>
#define FASTLED_INTERNAL          // suppress FastLED version banner
#include <FastLED.h>

// ============================================================================
// CONFIGURATION (tune here)
// ============================================================================

#define RING_PIN         A2      // WS2812 DIN pin
#define RING_LED_COUNT   7       // 1 center + 6 outer


#define RING_DEFAULT_BRIGHTNESS 128   // 0..255, non-destructive

// Distance thresholds (cm). Raw sensor values are converted to cm internally.
//
// Sensor physical range (as configured): 9 mm (min) .. 800 mm (max).
//   -> cm sensors  report 1..80 cm
//   -> mm sensor   (ID3) reports 9..800 mm
//
// RING_DIST_GREEN_CM  — far end of the continuous color gradient. A distance at
//                       or above this (80 cm = 800 mm) renders fully green; it
//                       fades to red as distance approaches the minimum.
// RING_DIST_YELLOW_CM — "close" threshold for the center attention LED and the
//                       brightness layer (not a color boundary). At/below this
//                       the LED is at full brightness and the center LED fires.
#define RING_DIST_GREEN_CM    80    // 800 mm max range -> fully green
#define RING_DIST_YELLOW_CM   25

// Valid distance range (as configured). Any reading outside this is treated as
// invalid and blanks the LED (in addition to the "no response" sentinel).
//   cm sensors: 1..80 cm   (min 9 mm rounds up to 1 cm)
//   mm sensor : 9..800 mm
#define RING_DIST_MIN_CM    1
#define RING_DIST_MAX_CM    80
#define RING_DIST_MIN_MM    9
#define RING_DIST_MAX_MM    800

// M60F "no reading" sentinels (matches DistanceSensor.h / test5.ino):
//   - cm sensors (ID1,2,4,5) return 0xFF   on no response
//   - mm sensor  (ID3)       returns 0xFFF on no response
#define RING_INVALID_CM   0xFF
#define RING_INVALID_MM   0xFFF

// Brightness scaling for distance LEDs (0..255). Nearer = brighter, so the
// ring gives a continuous intensity cue on top of the color gradient. At/below
// YELLOW_CM the LED is at full brightness; at/above GREEN_CM it is dimmed to
// RING_DIST_FAR_DIM; in between it fades linearly.
#define RING_DIST_FAR_DIM     60      // brightness (0..255) for far/"green" LEDs

// ============================================================================
// INTERNAL STATE
// ============================================================================

static CRGB  ringLeds[RING_LED_COUNT];
static bool  ringEnabled   = true;   // false = show() becomes no-op
static bool  ringInitialized = false;

// ============================================================================
// LOW-LEVEL CORE
// ============================================================================

/**
 * Initialize the ring. Call once in setup().
 * Turns on the strip, applies brightness, and clears to blank.
 */
void RingInit() {
    FastLED.addLeds<NEOPIXEL, RING_PIN>(ringLeds, RING_LED_COUNT);
    FastLED.setBrightness(RING_DEFAULT_BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
    ringInitialized = true;
    ringEnabled     = true;
}

/**
 * Set global brightness (luminosity) 0..255 without changing stored colors.
 */
void RingSetBrightness(uint8_t level) {
    FastLED.setBrightness(level);
}

/**
 * Set a single LED by index (0..6) to an 8-bit RGB color.
 * Bounds-checked; safe to call before init.
 */
void RingSetLED(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (!ringInitialized || index >= RING_LED_COUNT) return;
    ringLeds[index] = CRGB(r, g, b);
}

/**
 * Set a single LED by index to a CRGB color.
 */
void RingSetLEDColor(uint8_t index, CRGB color) {
    if (!ringInitialized || index >= RING_LED_COUNT) return;
    ringLeds[index] = color;
}

/**
 * Fill the entire ring with one 8-bit RGB color.
 */
void RingFillRGB(uint8_t r, uint8_t g, uint8_t b) {
    if (!ringInitialized) return;
    for (uint8_t i = 0; i < RING_LED_COUNT; i++) {
        ringLeds[i] = CRGB(r, g, b);
    }
}

/**
 * Fill the entire ring with a CRGB color.
 */
void RingFill(CRGB color) {
    if (!ringInitialized) return;
    for (uint8_t i = 0; i < RING_LED_COUNT; i++) {
        ringLeds[i] = color;
    }
}

/**
 * Clear the ring — set every LED to black (blank).
 * Does NOT call show(); call RingShow() to push the change.
 */
void RingClear() {
    if (!ringInitialized) return;
    FastLED.clear();
}

/**
 * Push the internal buffer to the physical LEDs.
 * No-op when the ring is disabled.
 */
void RingShow() {
    if (!ringInitialized || !ringEnabled) return;
    FastLED.show();
}

/**
 * Disable the ring — clear it and make all subsequent RingShow() calls no-ops.
 * Call once the match starts (or whenever visual feedback is no longer needed).
 */
void RingDisable() {
    if (ringInitialized) {
        FastLED.clear();
        FastLED.show();   // force blank out one final time
    }
    ringEnabled = false;
}

/**
 * Re-enable a previously disabled ring.
 */
void RingEnable() {
    ringEnabled = true;
}

/**
 * Query whether the ring is currently enabled.
 */
bool RingIsEnabled() {
    return ringEnabled;
}

// ============================================================================
// HELPERS
// ============================================================================

/**
 * Map a distance (already in cm) to a continuous green->red color.
 *
 *   distCm >= RING_DIST_GREEN_CM  -> green  (safe / opponent far)
 *   distCm == 0                   -> red    (very close)
 *   between                       -> hue sweeps green -> yellow -> orange -> red
 *
 * Uses FastLED's HSV hue (HUE_GREEN=96 down to HUE_RED=0) so the transition is
 * smooth instead of 3 hard color bands. Integer math only (no float).
 */
CRGB RingDistanceColor(uint16_t distCm) {
    uint16_t clamped = (distCm > RING_DIST_GREEN_CM) ? RING_DIST_GREEN_CM : distCm;
    uint16_t d = RING_DIST_GREEN_CM - clamped;          // 0..GREEN_CM (0 = far)
    uint8_t  hue = (uint8_t)(HUE_GREEN - ((uint32_t)HUE_GREEN * d) / RING_DIST_GREEN_CM);
    return CHSV(hue, 255, 255);
}

/**
 * Map a distance (already in cm) to a brightness scale factor 0..255.
 * Used to layer intensity onto the traffic-light color: closer objects glow
 * brighter, far objects are dim. This is independent of the global ring
 * brightness and is applied per-LED.
 *
 *   <= YELLOW_CM  -> 255 (full)
 *   >= GREEN_CM   -> RING_DIST_FAR_DIM
 *   between       -> linear fade
 */
uint8_t RingDistanceBrightness(uint16_t distCm) {
    if (distCm <= RING_DIST_YELLOW_CM) return 255;
    if (distCm >= RING_DIST_GREEN_CM)  return RING_DIST_FAR_DIM;

    // Linear interpolation between the yellow and green thresholds.
    uint32_t span = RING_DIST_GREEN_CM - RING_DIST_YELLOW_CM;
    uint32_t t    = distCm - RING_DIST_YELLOW_CM;   // 0..span
    uint32_t b    = 255 - ((255 - RING_DIST_FAR_DIM) * t) / span;
    return (uint8_t)b;
}

// ============================================================================
// HIGH-LEVEL "USEFUL INFO" RENDERERS
// ============================================================================

/**
 * Scale a CRGB color by a 0..255 brightness factor and return the result.
 * Uses video-safe scaling (nscale8_video) so hues stay correct as they dim.
 */
CRGB RingScaleColor(CRGB color, uint8_t brightness) {
    color.nscale8_video(brightness);
    return color;
}

/**
 * Return true if a raw distance value should be treated as invalid (blank LED).
 * Invalid means either the "no reading" sentinel OR an out-of-range value.
 * ID3 is mm, all others are cm.
 */
bool RingIsInvalidDistance(uint8_t id, uint16_t raw) {
    if (id == 3) {
        // mm sensor: sentinel, or outside 9..800 mm.
        if (raw == RING_INVALID_MM) return true;
        return (raw < RING_DIST_MIN_MM || raw > RING_DIST_MAX_MM);
    }
    // cm sensors: sentinel, or outside 1..80 cm.
    if (raw == RING_INVALID_CM) return true;
    return (raw < RING_DIST_MIN_CM || raw > RING_DIST_MAX_CM);
}

// ToF sensor -> ring LED index mapping (outer ring, front = LED 4).
// Index here is the ToF ID (1..5). ID3 = middle is mm and maps to front.
static const uint8_t RING_TOF_LED[6] = {
    0,          // [0] unused
    6,          // ID1 side right     -> right back
    5,          // ID2 right front    -> right front
    4,          // ID3 middle (mm)    -> front
    3,          // ID4 front left     -> left front
    2,          // ID5 side left      -> left back
};

/**
 * Convert a raw ToF reading to centimeters. ID3 (middle) is mm, so it is
 * divided by 10; all other sensors already report cm.
 */
static inline uint16_t RingToFToCm(uint8_t id, uint16_t raw) {
    return (id == 3) ? (raw / 10u) : raw;
}

/**
 * Render the 5 ToF sensors onto the outer ring (green->red, brightness-scaled).
 * Returns true if any sensor is in the "close" zone (<= YELLOW_CM).
 */
static bool RingRenderToF(const uint16_t* distances) {
    bool anyClose = false;
    for (uint8_t id = 1; id <= 5; id++) {
        uint16_t raw = distances[id - 1];
        if (RingIsInvalidDistance(id, raw)) continue;   // dead sensor = blank
        uint16_t cm = RingToFToCm(id, raw);
        CRGB c = RingDistanceColor(cm);
        ringLeds[RING_TOF_LED[id]] = RingScaleColor(c, RingDistanceBrightness(cm));
        if (cm <= RING_DIST_YELLOW_CM) anyClose = true;
    }
    return anyClose;
}

/**
 * Show the 5 ToF sensor ranges as a traffic-light ring.
 *
 * @param distances  Array of 5 raw distances, indexed by ToF ID:
 *                   distances[0] = ID1 side right  (cm)
 *                   distances[1] = ID2 right front (cm)
 *                   distances[2] = ID3 middle      (mm! converted here)
 *                   distances[3] = ID4 front left  (cm)
 *                   distances[4] = ID5 side left   (cm)
 *
 * A reading equal to the M60F "no response" sentinel (0xFF for cm, 0xFFF for
 * mm) OR any value outside the configured 9..800 mm range leaves that LED
 * blank so a dead/out-of-range sensor is obvious.
 */
void RingShowToFRanges(const uint16_t* distances) {
    if (!ringInitialized) return;
    FastLED.clear();

    bool anyClose = RingRenderToF(distances);

    // Center LED = attention if ANY sensor is in the close zone.
    ringLeds[0] = anyClose ? CRGB::Red : CRGB::Black;

    RingShow();
}

/**
 * Show 3 line sensors (left / center / right) on the ring.
 *
 * @param line  Array of 3 booleans: line[0]=left, line[1]=center, line[2]=right.
 *              true  = line/edge detected (LED lights white).
 *              false = no line (LED stays blank).
 */
void RingShowLine(const bool* line) {
    if (!ringInitialized) return;
    FastLED.clear();

    // Front arc: left -> LED3, center -> LED4, right -> LED5
    if (line[0]) ringLeds[3] = CRGB::White;
    if (line[1]) ringLeds[4] = CRGB::White;
    if (line[2]) ringLeds[5] = CRGB::White;

    // Center LED = aggregate: any line detected -> red attention
    ringLeds[0] = (line[0] || line[1] || line[2]) ? CRGB::Red : CRGB::Black;

    RingShow();
}

/**
 * Combined view: ToF ranges on the outer ring + line state on the center LED.
 * Convenience wrapper so the main loop has one call for full status.
 */
void RingShowAll(const uint16_t* distances, const bool* line) {
    if (!ringInitialized) return;

    // LINE DETECTION HAS PRIORITY: if any line sensor is tripped, render the
    // line view (white front arc + red center) and ignore distance info
    // entirely — a line/edge is more urgent than opponent distance.
    if (line[0] || line[1] || line[2]) {
        RingShowLine(line);
        return;
    }

    // No line detected -> show ToF distance info.
    FastLED.clear();

    bool anyClose = RingRenderToF(distances);

    // Center LED: amber if opponent close.
    ringLeds[0] = anyClose ? CRGB::Orange : CRGB::Black;

    RingShow();
}

#endif // RING_INFO_DISPLAY_H
