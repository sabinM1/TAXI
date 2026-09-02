/*
  WS2812 7-LED Ring — Advanced FastLED Demo
  --------------------------------------------
  Board: Any Arduino (Uno/Nano/ESP32/etc.)
  Library: FastLED (Arduino IDE Library Manager -> "FastLED")
  Wiring: DIN -> D6 (via 300-500 ohm resistor), 5V -> 5V, GND -> GND

  Demonstrates:
    - Non-destructive brightness scaling
    - Power limiting (mA budget)
    - HSV rainbow generation
    - CRGBPalette16 smooth color cycling
    - Perlin noise ("fire") animation
    - beatsin8 non-blocking breathing effect
    - EVERY_N_MILLISECONDS non-blocking scheduler
*/

#include <FastLED.h>

#define LED_PIN     6
#define NUM_LEDS    7
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define VOLTS       5
#define MAX_MA      500   // current budget in milliamps

CRGB leds[NUM_LEDS];

// Custom palette: dark blue -> cyan -> white -> orange (used by paletteCycle)
DEFINE_GRADIENT_PALETTE(icefire_gp) {
    0,   0,   0,  80,
   64,   0, 180, 220,
  128, 255, 255, 255,
  192, 255, 120,   0,
  255, 120,   0,   0
};
CRGBPalette16 icefirePalette = icefire_gp;

uint8_t effectIndex = 0;
uint8_t hueOffset = 0;
uint8_t paletteIndex = 0;

void setup() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(140);                       // 0-255, non-destructive
  FastLED.setMaxPowerInVoltsAndMilliamps(VOLTS, MAX_MA); // auto current limiting
  FastLED.clear();
  FastLED.show();
}

void loop() {
  // Non-blocking effect scheduler: rotate effect every 8 seconds
  EVERY_N_MILLISECONDS(8000) {
    effectIndex = (effectIndex + 1) % 6;
    FastLED.clear();
  }

  switch (effectIndex) {
    case 0: colorWipe(CRGB::Red, 60);        break;
    case 1: theaterChase(CRGB::Blue, 100);   break;
    case 2: rainbowCycle();                  break;
    case 3: breathingEffect(CRGB::Cyan);     break;
    case 4: paletteCycle();                  break;
    case 5: fireEffect();                    break;
  }

  FastLED.show();
}

// --- Effect 1: Classic sequential color wipe ---
void colorWipe(CRGB color, uint8_t wait_ms) {
  static uint8_t pos = 0;
  EVERY_N_MILLISECONDS(wait_ms) {
    leds[pos] = color;
    pos = (pos + 1) % NUM_LEDS;
    if (pos == 0) FastLED.clear();
  }
}

// --- Effect 2: Theater chase (every 3rd LED lit, rotating) ---
void theaterChase(CRGB color, uint8_t wait_ms) {
  static uint8_t offset = 0;
  EVERY_N_MILLISECONDS(wait_ms) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = ((i + offset) % 3 == 0) ? color : CRGB::Black;
    }
    offset = (offset + 1) % 3;
  }
}

// --- Effect 3: HSV rainbow rotating across all 7 pixels ---
void rainbowCycle() {
  EVERY_N_MILLISECONDS(20) {
    for (int i = 0; i < NUM_LEDS; i++) {
      uint8_t hue = hueOffset + (i * 255 / NUM_LEDS);
      leds[i] = CHSV(hue, 255, 255);
    }
    hueOffset++;
  }
}

// --- Effect 4: Non-blocking breathing brightness pulse via beatsin8 ---
void breathingEffect(CRGB baseColor) {
  uint8_t breath = beatsin8(20, 30, 255); // 20 BPM, min 30, max 255
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = baseColor;
    leds[i].nscale8(breath);              // scale this pixel's brightness
  }
}

// --- Effect 5: Smooth palette-based color cycling ---
void paletteCycle() {
  EVERY_N_MILLISECONDS(30) {
    for (int i = 0; i < NUM_LEDS; i++) {
      uint8_t index = paletteIndex + (i * 255 / NUM_LEDS);
      leds[i] = ColorFromPalette(icefirePalette, index, 255, LINEARBLEND);
    }
    paletteIndex++;
  }
}

// --- Effect 6: Perlin-noise-driven "fire" simulation ---
void fireEffect() {
  static uint16_t noiseTime = 0;
  EVERY_N_MILLISECONDS(30) {
    for (int i = 0; i < NUM_LEDS; i++) {
      uint8_t n = inoise8(i * 40, noiseTime);
      leds[i] = ColorFromPalette(icefirePalette, n, 255, LINEARBLEND);
    }
    noiseTime += 20;
  }
}
