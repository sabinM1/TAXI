# RingInfoDisplay — WS2812 7-LED Ring Info Display

Self-contained header + demo for driving a 7-LED WS2812 ring with FastLED. It
renders **sensor status** (ToF distances + line detection) onto the ring and
provides low-level RGB / brightness / clear / disable controls.

The library never reads hardware itself — you feed it values, it paints LEDs.

---

## 1. Physical LED layout

The ring is wired in a fixed physical order. Index `0` is the **center**, then
the outer ring runs clockwise:

```
                4 (front)
            3            5
        (left front)   (right front)

         2                6
      (left back)     (right back)

               1 (back)

       [ 0 ]  = center (aggregate / attention)
```

| LED index | Position   | Clock position |
|-----------|------------|----------------|
| 0         | center     | —              |
| 1         | back       | 6 o'clock      |
| 2         | left back  | ~8 o'clock     |
| 3         | left front | ~10 o'clock    |
| 4         | front      | 12 o'clock     |
| 5         | right front| ~2 o'clock     |
| 6         | right back | ~4 o'clock     |

---

## 2. Sensor → LED mapping

### ToF distance sensors (5×)

All sensors report **cm** except **ID3 (middle)** which reports **mm**.

| ToF ID | Name        | Units | LED index | Ring position |
|--------|-------------|-------|-----------|---------------|
| ID1    | side right  | cm    | 6         | right back    |
| ID2    | right front | cm    | 5         | right front   |
| ID3    | middle      | mm    | 4         | front         |
| ID4    | front left  | cm    | 3         | left front    |
| ID5    | side left   | cm    | 2         | left back     |

### Line sensors (3×)

| Line index | Name   | LED index | Ring position |
|------------|--------|-----------|---------------|
| 0          | left   | 3         | left front    |
| 1          | center | 4         | front         |
| 2          | right  | 5         | right front   |

### Center LED (0)

The center LED is the **aggregate / attention** indicator:

| Condition                        | Center LED color |
|----------------------------------|------------------|
| Any ToF sensor ≤ 25 cm (ToF view)| Red              |
| Any line sensor tripped          | Red              |
| Opponent ≤ 25 cm (combined view) | Orange           |
| Otherwise                        | Off / black      |

---

## 3. Color meaning

### ToF distance (continuous green → red)

Each ToF LED's color is a **continuous hue gradient** from green (far/safe) to
red (close), passing through yellow and orange in between. There are no hard
color bands.

The gradient is driven by `RING_DIST_GREEN_CM`: any distance at or above that
threshold is fully green, and it fades toward red as the distance drops to the
sensor minimum.

Sensors are configured for a **9 mm .. 800 mm** range (0 .. 80 cm).

| Distance (cm)                | Color                        | Meaning             |
|------------------------------|------------------------------|---------------------|
| `>= RING_DIST_GREEN_CM` (80) | Green                        | Safe / opponent far (800 mm) |
| between (80 .. 0)            | Green → yellow → orange → red | Continuous approach |
| `~0` (9 mm)                  | Red                          | Very close          |

`RING_DIST_YELLOW_CM` is still used by the **brightness** layer below (and the
center-attention LED), but is no longer a color-band boundary.

### Distance brightness (luminosity)

On top of the traffic-light color, each ToF LED's **brightness** is scaled by
how far the object is — nearer objects glow brighter, far objects are dimmed.
This gives a continuous intensity cue between the discrete color bands.

| Distance (cm)        | Brightness                                   |
|----------------------|----------------------------------------------|
| `<= RING_DIST_YELLOW_CM` (25) | Full (255)                            |
| `>= RING_DIST_GREEN_CM` (80)  | Dim (`RING_DIST_FAR_DIM`, default 60)  |
| between (25–80)      | Linear fade between full and dim              |

This per-LED intensity is **independent** of the global ring brightness
(`RingSetBrightness`), and uses video-safe scaling so hues stay correct as
they dim. Tune `RING_DIST_FAR_DIM` in `RingInfoDisplay.h` to set how dim "far"
LEDs get (0 = off, 255 = no dimming).

### Line sensors

| State                    | Color | Meaning                       |
|--------------------------|-------|-------------------------------|
| Line detected (`true`)   | White | Line/edge under that sensor   |
| No line (`false`)        | Off   | No line                       |

### Priority

**Line detection overrides distance.** In the combined view (`RingShowAll`),
if any line sensor is tripped, the ring shows the line view and ignores ToF
data. This is because running over the dohyo edge is more urgent than opponent
distance.

---

## 4. Programming / integration

### 4.1 Include the header

Add at the top of your sketch:

```cpp
#include "RingInfoDisplay.h"
```

Copy `RingInfoDisplay.h` into your sketch folder (or add its folder to the
include path). Requires the **FastLED** library.

### 4.2 Configure (optional)

Defaults are in `RingInfoDisplay.h`. Override before including, or edit the
header directly:

```cpp
#define RING_PIN                 A2    // WS2812 DIN pin
#define RING_LED_COUNT           7     // 1 center + 6 outer
#define RING_DEFAULT_BRIGHTNESS  128   // 0..255
#define RING_DIST_GREEN_CM       80    // >= this = green (far end of gradient, 800 mm)
#define RING_DIST_YELLOW_CM      25    // brightness/attention threshold (still used)
#define RING_DIST_FAR_DIM        60    // brightness (0..255) for far/"green" LEDs
#define RING_DIST_MIN_CM         1     // readings below this (cm) are invalid/blank
#define RING_DIST_MAX_CM         80    // readings above this (cm) are invalid/blank
#define RING_DIST_MIN_MM         9     // readings below this (mm, ID3) are invalid
#define RING_DIST_MAX_MM         800   // readings above this (mm, ID3) are invalid
```

> `RING_PIN` is `A2`.

### 4.3 Setup

```cpp
void setup() {
    Serial.begin(115200);
    RingInit();        // init + clear + apply brightness
}
```

### 4.4 Main loop

Feed your real sensor readings into one high-level call. You need two arrays:

```cpp
// ToF distances, indexed by ID: [ID1, ID2, ID3, ID4, ID5]
// ID3 is in mm, the rest in cm.
uint16_t tofDistances[5];

// Line sensors: [left, center, right]
bool lineSensors[3];
```

Then, each loop iteration:

```cpp
void loop() {
    // ... read your sensors and fill tofDistances[] / lineSensors[] ...

    RingShowAll(tofDistances, lineSensors);
}
```

The library handles "no reading" sentinels and out-of-range values internally
(see §6), so you don't need to pass an invalid value — just feed raw sensor
output.

`RingShowAll` renders everything with the priority rule (line > distance).

### 4.5 Other high-level functions

| Function | What it does |
|----------|--------------|
| `RingShowToFRanges(dist)`      | ToF traffic-light view only |
| `RingShowLine(line)`           | Line view only (white arc + red center) |
| `RingShowAll(dist, line)`      | Combined view (line priority) |

### 4.6 Low-level API

| Function | What it does |
|----------|--------------|
| `RingInit()`                          | Init the strip, apply brightness, clear |
| `RingSetBrightness(uint8_t level)`    | Set global brightness 0..255 |
| `RingDistanceColor(cm)`                | Continuous green→red CRGB for a distance |
| `RingDistanceBrightness(cm)`           | Per-LED intensity factor 0..255 for a distance |
| `RingScaleColor(color, brightness)`    | Video-safe dim a CRGB by a factor |
| `RingIsInvalidDistance(id, raw)`       | True if `raw` is the sentinel or out-of-range (blank LED) |
| `RingSetLED(i, r, g, b)`              | Set LED `i` to RGB color |
| `RingSetLEDColor(i, CRGB)`            | Set LED `i` to a CRGB color |
| `RingFillRGB(r, g, b)`                | Fill whole ring one color |
| `RingFill(CRGB)`                      | Fill whole ring one CRGB |
| `RingClear()`                         | Set all LEDs black (no show) |
| `RingShow()`                          | Push buffer to LEDs (no-op if disabled) |
| `RingDisable()`                       | Clear + make `RingShow()` a no-op |
| `RingEnable()`                        | Re-enable a disabled ring |
| `RingIsEnabled()`                     | Query enabled state |

### 4.7 Disable the ring after the match starts

Call once when visual feedback is no longer needed:

```cpp
RingDisable();
```

`RingShow()` becomes a no-op and the ring is blanked. To restore later:

```cpp
RingEnable();
```

---

## 5. Testing the LED numbering

`RingInfoDisplay.ino` has a compile-time switch at the top:

```cpp
#define ENABLE_NUMBERING_MODE   1
```

- `1` → runs **only** the numbering test: lights one LED at a time (`0..6`,
  2 s each) and prints the index to serial, so you can verify physical
  ordering.
- `0` → runs the full demo (ToF / line / clear / disable rotation).

---

## 6. Notes / caveats

- **ToF units & range**: the library treats all sensors as **cm except ID3
  (mm)** and converts ID3 via `/10`. Sensors are configured for **9 mm ..
  800 mm**. The gradient maps 800 mm → green and ~9 mm → red. Confirm your
  actual sensor output matches, otherwise the colors will be wrong.
- **"No reading" sentinels & out-of-range both blank the LED.** The M60F
  sensors report `0xFF` (cm, ID1/2/4/5) or `0xFFF` (mm, ID3) on no response.
  `RingIsInvalidDistance()` also treats any reading **outside the configured
  9..800 mm range** (1..80 cm) as invalid. Either way the LED stays off.
  These constants are `RING_INVALID_CM` / `RING_INVALID_MM` and
  `RING_DIST_MIN_*` / `RING_DIST_MAX_*` in `RingInfoDisplay.h`.
- **Line sensor count**: this library assumes **3** line sensors. The existing
  TAXI firmware (`Sensors.h`) uses **4× QRE1113** — reconcile before wiring.
- **Name collision**: `RingInfoDisplay.h` defines its own `static` state and
  function names (`Ring*`). It is intentionally separate from the TAXI
  `LEDRing.h` (`ledRing*`) to avoid clashes — don't include both without
  renaming.
