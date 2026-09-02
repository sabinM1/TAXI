
# WS2812 7‑LED Ring — Technical Documentation & Arduino Implementation

## 1. Hardware Overview

The **WS2812 7‑LED Ring** (sold as "NeoPixel Jewel" by Adafruit, and under generic names by Botland, DIYElectronics, etc.) is a small PCB with **7 individually addressable RGB LEDs**: 6 LEDs (5050 package) arranged in a circle plus 1 LED in the center. Each LED integrates its own WS2812 driver chip, so a single microcontroller data pin can control all 7 pixels independently over a one-wire serial protocol [web:5][web:15].

| Parameter | Value |
|---|---|
| LED count | 7 (6 outer + 1 center) |
| LED package | 5050 RGB with integrated WS2812 driver |
| Outer diameter | 23 mm |
| Supply voltage | 5 V |
| Current per LED | ~18 mA typical (datasheet "typical" figure; full-white peak can reach ~50–60 mA per LED, so budget up to ~350–400 mA worst case for all 7) [web:5][web:7] |
| Total typical current | ~130 mA (all LEDs, moderate colors) [web:5][web:10] |
| Pins | GND, 5V, DIN (Data In), DOUT (Data Out, for chaining more rings/strips) [web:5][web:7] |
| Color depth | 24-bit (8 bits per R/G/B channel), 16.7 million colors [web:15] |
| Protocol | Single-wire, self-clocked, ~800 kHz (WS2812/NeoPixel timing) |

### Wiring

- **GND** → Arduino GND
- **5V** → external 5V supply or Arduino 5V (fine for a single 7-LED ring at moderate brightness)
- **DIN** → Arduino digital pin (e.g. D6), ideally through a 300–500 Ω resistor to protect the first LED's input
- **DOUT** → leave unconnected, or chain to DIN of the next ring/strip

Recommended good practice (from Adafruit's official best practices):
- Add a 300–500 Ω resistor in series on the data line.
- Add a 100–1000 µF capacitor across 5V/GND near the ring if powered externally.
- Never hot-plug the ring while powered.

## 2. Library Choice

Two mainstream libraries drive WS2812 rings on Arduino:

| Library | Strengths | Best for |
|---|---|---|
| **Adafruit NeoPixel** | Simple API, small footprint, well documented [web:9][web:13] | Beginners, simple color-setting projects |
| **FastLED** | Faster, includes 8-bit math, color palettes, HSV, noise, dithering, multi-strip/multi-pin support, non-destructive brightness scaling [web:1][web:3][web:6][web:11] | Advanced animations, multiple effects, performance-critical projects |

Given the request for **advanced functions**, this documentation uses **FastLED** (install via Arduino IDE Library Manager: search "FastLED", by the FastLED community — currently maintained at github.com/FastLED/FastLED [web:3]). Adafruit_NeoPixel equivalents are noted in comments where relevant.

## 3. Core API Used

```cpp
#include <FastLED.h>

#define LED_PIN     6
#define NUM_LEDS    7
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
FastLED.show();                 // push buffer to LEDs
FastLED.setBrightness(128);     // 0-255, non-destructive global scaling
FastLED.setMaxPowerInVoltsAndMilliamps(5, 500); // power limiter
```

Key advanced features documented and demonstrated below:

1. **Non-destructive brightness scaling** — `FastLED.setBrightness()` scales output without altering the stored `CRGB` values [web:11].
2. **Power management** — `FastLED.setMaxPowerInVoltsAndMilliamps()` auto-limits current draw, protecting USB ports [web:11].
3. **HSV color space & rainbow generation** — `CHSV`, `fill_rainbow()`.
4. **Color palettes** — `CRGBPalette16`, `ColorFromPalette()` for smooth gradient transitions.
5. **Perlin noise animation** — `inoise8()` for organic "fire"/"plasma" effects.
6. **Sine-wave timing (`beatsin8`)** — smooth breathing/pulsing without blocking `delay()`.
7. **Non-blocking multitask timing** — `EVERY_N_MILLISECONDS` macro for scheduling effects.
8. **8-bit math helpers** — `qadd8`, `scale8`, `blend()` for fast, safe color arithmetic.

## 4. Code Overview

The accompanying `.ino` file implements:

- `colorWipe()` — sequential fill (classic demo effect)
- `theaterChase()` — theater-marquee chase
- `rainbowCycle()` — HSV-based rotating rainbow across all 7 LEDs
- `breathingEffect()` — brightness pulsing via `beatsin8` (non-blocking)
- `paletteCycle()` — smooth color transitions using a `CRGBPalette16`
- `fireEffect()` — Perlin-noise-driven flame simulation
- `setPower()` — demonstrates power limiting and brightness API
- A `switch`-based non-blocking scheduler in `loop()` using `EVERY_N_MILLISECONDS` to rotate between effects automatically every 8 seconds.

## 5. Practical Notes

- With only 7 LEDs, current draw is modest; the ring can usually run directly off the Arduino's 5V pin, but if brightness is set near 255 across all channels on all LEDs, add an external 5V supply.
- Use `COLOR_ORDER GRB` — this is standard for WS2812/WS2812B; if colors look swapped (e.g., red shows as green), try `RGB` or `BRG`.
- FastLED's `show()` call disables interrupts briefly while sending data; avoid calling it inside interrupt-heavy code.
- For products sold in Romania/EU (e.g., Botland, eMAG-listed WS2812 rings), the specs above match the commonly sold 7×WS2812 5050 ring (23 mm OD) [web:5][web:7][web:10].
