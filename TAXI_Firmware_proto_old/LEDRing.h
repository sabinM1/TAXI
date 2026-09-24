/**
 * LEDRing.h — WS2812 RGB LED Ring Controller
 * 
 * Provides 7-bit addressable RGB ring control using FastLED.
 * Visual feedback for: state indication, sensor health, attack direction.
 * 
 * MAPPING (7 LEDs, 12-o'clock = front, clockwise):
 *   LED 0 = 12 o'clock (front)
 *   LED 1 = ~2 o'clock
 *   LED 2 = ~4 o'clock (front-right corner)
 *   LED 3 = 6 o'clock (rear)
 *   LED 4 = ~8 o'clock (rear-left corner)
 *   LED 5 = ~10 o'clock (front-left corner)
 *   LED 6 = center/reserved
 * 
 * WHY: User's idei.txt: "Led indicator de direcție" (direction indicator LED).
 * Phase 3 decision: integrated visual feedback with specific patterns per state.
 */

#ifndef LEDRING_H
#define LEDRING_H

#include <Arduino.h>
#define FASTLED_INTERNAL  // Suppress FastLED version warning
#include <FastLED.h>
#include "Config.h"

// ============================================================================
// FASTLED CONFIGURATION
// ============================================================================

static CRGB ledRing[LED_RING_COUNT];

// ============================================================================
// INITIALIZATION
// ============================================================================

/**
 * Initialize the WS2812 LED ring.
 * Called once from setup().
 */
void ledRingInit() {
    FastLED.addLeds<NEOPIXEL, LED_RING_PIN>(ledRing, LED_RING_COUNT);
    FastLED.setBrightness(LED_BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
    
    #if SERIAL_DEBUG
    Serial.println(F("[LEDRing] WS2812 initialized, 7 LEDs on D4"));
    #endif
}

// ============================================================================
// PATTERN HELPERS
// ============================================================================

/**
 * Fill entire ring with one color.
 */
void ledRingFillAll(CRGB color) {
    for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
        ledRing[i] = color;
    }
}

/**
 * Set specific LED by position index.
 * Position 0 = front (12 o'clock), goes clockwise.
 */
void ledRingSetPosition(uint8_t pos, CRGB color) {
    if (pos < LED_RING_COUNT) {
        ledRing[pos] = color;
    }
}

/**
 * Pulse brightness using a sine wave.
 * Returns a dimmed version of the input color based on millis() phase.
 */
CRGB ledRingPulse(CRGB color, uint8_t speed_ms) {
    uint8_t brightness = sin8(millis() / speed_ms * 255 / 1000);
    return color.nscale8_video(brightness);
}

/**
 * Create a sweeping/rotating pattern.
 * Lights one LED at 'position' (0-6) and dims the rest.
 */
void ledRingSweep(uint8_t position, CRGB color) {
    FastLED.clear();
    position = position % LED_RING_COUNT;
    ledRing[position] = color;
    // Dim neighbors
    uint8_t prev = (position + LED_RING_COUNT - 1) % LED_RING_COUNT;
    uint8_t next = (position + 1) % LED_RING_COUNT;
    ledRing[prev] = color;
    ledRing[next] = color;
    ledRing[prev].nscale8_video(64);
    ledRing[next].nscale8_video(64);
}

// ============================================================================
// STATE-BASED PATTERNS
// ============================================================================

/**
 * Update LED ring based on current robot state.
 * Called once per main loop iteration.
 * 
 * @param state        Current robot state enum
 * @param attackDir    Attack direction (-1=left, 0=center, 1=right)
 * @param sensorHealth Array of 5 sensor health statuses
 */
void ledRingUpdate(uint8_t state, int8_t attackDir, uint8_t* sensorHealth) {
    FastLED.clear();
    
    switch (state) {
        case 0: // IDLE — show sensor health
            for (uint8_t i = 0; i < M60F_COUNT && i < LED_RING_COUNT; i++) {
                ledRing[i] = (sensorHealth[i] >= HEALTH_FAIL_THRESHOLD) ? 
                    LED_COLOR_OK : LED_COLOR_ERROR;
            }
            // Rear sensors (indices 5,6 if available) show QRE status
            break;
            
        case 1: // TACTIC — rapid green blink
            ledRingFillAll((millis() % 200 < 100) ? LED_COLOR_OK : CRGB::Black);
            break;
            
        case 2: // SEARCH — pulsing blue
            ledRingFillAll(ledRingPulse(LED_COLOR_SEARCH, 1));
            break;
            
        case 3: // ATTACK — direction indicator
            if (attackDir < 0) {
                // Attacking left — light left half
                ledRing[5] = LED_COLOR_ATTACK;
                ledRing[0] = LED_COLOR_ATTACK;
                ledRing[1] = LED_COLOR_ATTACK;
            } else if (attackDir > 0) {
                // Attacking right — light right half
                ledRing[1] = LED_COLOR_ATTACK;
                ledRing[2] = LED_COLOR_ATTACK;
                ledRing[3] = LED_COLOR_ATTACK;
            } else {
                // Centered — all red
                ledRingFillAll(LED_COLOR_ATTACK);
            }
            break;
            
        case 4: // FLANK — yellow/orange arc toward opponent
            if (attackDir < 0) {
                // Flanking left
                ledRing[5] = LED_COLOR_FLANK_LEFT;
                ledRing[0] = LED_COLOR_FLANK_LEFT;
            } else {
                // Flanking right
                ledRing[1] = LED_COLOR_FLANK_RIGHT;
                ledRing[2] = LED_COLOR_FLANK_RIGHT;
            }
            break;
            
        case 5: // EVADE — white flash
            ledRingFillAll((millis() % 150 < 75) ? LED_COLOR_EVADE : CRGB::Black);
            break;
            
        case 6: // STOP — solid red
        default:
            ledRingFillAll(LED_COLOR_ERROR);
            break;
    }
    
    FastLED.show();
}

/**
 * Startup animation: rainbow sweep.
 * Called during power-on to confirm LEDs are working.
 */
void ledRingStartupAnimation() {
    for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
        ledRing[i] = CHSV(i * 255 / LED_RING_COUNT, 255, 255);
        FastLED.show();
        delay(80);
    }
    delay(300);
    FastLED.clear();
    FastLED.show();
}

#endif // LEDRING_H
