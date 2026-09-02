/*
 * RingInfoDisplay.ino
 * -------------------
 * Demo for RingInfoDisplay.h — drives a 7-LED WS2812 ring to show:
 *   - Opponent simulation (enter right -> sweep -> circle -> charge -> retreat)
 *   - 5x ToF sensor ranges (traffic-light colored, ID3 in mm)
 *   - 3x line sensors (white LEDs + center attention LED)
 *   - Brightness fade / Clear / disable / enable control
 *
 * Wiring: DIN -> A2 (via 300-500 ohm resistor), 5V -> 5V, GND -> GND.
 * Library: FastLED (Arduino Library Manager -> "FastLED").
 *
 * NOTE: this example FAKES sensor data (no real ToF/line hardware attached).
 * Replace the fake arrays with your own sensor readings.
 */

#include "RingInfoDisplay.h"

// ============================================================================
// LED-numbering test mode.
//   1 = run ONLY the numbering test (lights one LED at a time, 0..6)
//   0 = run the full demo (ToF / line / clear / disable rotation)
// ============================================================================
#define ENABLE_NUMBERING_MODE   0

#if !ENABLE_NUMBERING_MODE

// ============================================================================
// FULL DEMO (normal operation)
// ============================================================================

// --- Fake sensor data (replace with real readings) ---
// ToF: [ID1 side-right cm, ID2 right-front cm, ID3 middle mm, ID4 front-left cm, ID5 side-left cm]
uint16_t tofDistances[5] = { 45, 12, 300, 70, 80 };

// Line: [left, center, right] booleans
bool lineSensors[3] = { false, false, false };

// "No reading" sentinels are handled inside the header per-sensor:
//   cm sensors (ID1,2,4,5) -> 0xFF, mm sensor (ID3) -> 0xFFF.
// See RING_INVALID_CM / RING_INVALID_MM in RingInfoDisplay.h.

// Simple demo state machine
enum DemoMode {
    MODE_SIMULATION = 0,
    MODE_SHOW_LINE,
    MODE_BRIGHTNESS,
    MODE_CLEAR,
    MODE_DISABLE,
    MODE_COUNT
};
uint8_t mode = MODE_SIMULATION;
uint32_t lastSwitchMs = 0;
uint32_t lastBrightLogMs = 0;
uint32_t lastToFLogMs = 0;
uint32_t lastLineLogMs = 0;
uint32_t simStartMs = 0;      // anchored start of the current simulation pass
uint32_t modeChangedMs = 0;   // timestamp of the last mode change (for blank gap)

// Brief blank period between modes so each demo starts from a clean (off) ring.
#define TRANSITION_MS   350

// Duration each demo mode stays active before advancing (ms).
#define MODE_DWELL_MS          8000
#define BRIGHTNESS_DWELL_MS    16000   // white fade (8s) + spectrum (8s)
#define SIMULATION_DWELL_MS    22000   // simulation is longer (full narrative)

// Track last-printed values so we only log actual changes.
uint16_t lastToF[5] = { 0 };
bool     lastLine[3] = { 0 };
uint8_t  lastMode   = 0xFF;

void setup() {
    Serial.begin(115200);
    delay(5000);

    RingInit();               // init + clear
    Serial.println(F("RingInfoDisplay demo"));
    Serial.println(F("A2, 7 LEDs, brightness 128"));
    Serial.println(F("range 9..800mm; green(far)->red(close)  (ID3 mm)"));
    Serial.println();
}

// Log one ToF line only if a value changed, throttled to ~4 Hz.
void logToF() {
    if (millis() - lastToFLogMs < 250) return;   // max ~4 prints/sec
    if (tofDistances[0] == lastToF[0] && tofDistances[1] == lastToF[1] &&
        tofDistances[2] == lastToF[2] && tofDistances[3] == lastToF[3] &&
        tofDistances[4] == lastToF[4]) return;
    lastToFLogMs = millis();
    for (uint8_t i = 0; i < 5; i++) lastToF[i] = tofDistances[i];

    Serial.print(F("  ToF: SR="));
    Serial.print(tofDistances[0]);
    Serial.print(F("cm RF="));
    Serial.print(tofDistances[1]);
    Serial.print(F("cm MID="));
    Serial.print(tofDistances[2]);
    Serial.print(F("mm FL="));
    Serial.print(tofDistances[3]);
    Serial.print(F("cm SL="));
    Serial.print(tofDistances[4]);
    Serial.println(F("cm"));
}

// Log the line state only if it changed.
void logLine() {
    if (lineSensors[0] == lastLine[0] && lineSensors[1] == lastLine[1] &&
        lineSensors[2] == lastLine[2]) return;
    for (uint8_t i = 0; i < 3; i++) lastLine[i] = lineSensors[i];

    Serial.print(F("  Line L/C/R: "));
    Serial.print(lineSensors[0] ? '1' : '0');
    Serial.print('/');
    Serial.print(lineSensors[1] ? '1' : '0');
    Serial.print('/');
    Serial.println(lineSensors[2] ? '1' : '0');
}

// Log a mode change.
void logMode() {
    if (mode == lastMode) return;
    lastMode = mode;

    Serial.println();
    Serial.print(F("["));
    Serial.print(mode + 1);
    Serial.print('/');
    Serial.print(MODE_COUNT);
    Serial.print(F("] "));
    switch (mode) {
        case MODE_SIMULATION:
            Serial.println(F("SIMULATION (opponent narrative)"));
            Serial.println(F("  expect: enter right, sweep L/R, circle, charge middle, retreat"));
            break;
        case MODE_SHOW_LINE:
            Serial.println(F("LINE"));
            Serial.println(F("  expect: white front arc + red center"));
            break;
        case MODE_BRIGHTNESS:
            Serial.println(F("BRIGHTNESS (white fade + RGB spectrum)"));
            Serial.println(F("  expect: white fades in/out, then full hue cycle"));
            break;
        case MODE_CLEAR:
            Serial.println(F("CLEAR"));
            Serial.println(F("  expect: all LEDs off"));
            break;
        case MODE_DISABLE:
            Serial.println(F("DISABLE"));
            Serial.println(F("  expect: ring blank, RingShow is no-op"));
            break;
        default:
            Serial.println(F("?"));
            break;
    }
}

void loop() {
    // Advance to the next demo mode. Simulation and brightness stay longer so
    // their multi-phase sequences can play out.
    uint32_t dwell = MODE_DWELL_MS;
    if (mode == MODE_SIMULATION) dwell = SIMULATION_DWELL_MS;
    else if (mode == MODE_BRIGHTNESS) dwell = BRIGHTNESS_DWELL_MS;
    if (millis() - lastSwitchMs > dwell) {
        lastSwitchMs = millis();
        mode = (mode + 1) % MODE_COUNT;
        simStartMs    = millis();   // anchor the simulation on entry
        modeChangedMs = millis();   // anchor the blank gap on entry
        logMode();

        if (mode == MODE_CLEAR) {
            RingClear();
            RingShow();
        }
        if (mode == MODE_DISABLE) {
            RingDisable();
        } else if (!RingIsEnabled()) {
            RingEnable();
        }

        // Leaving the brightness demo restores the default brightness.
        if (mode != MODE_BRIGHTNESS) {
            RingSetBrightness(RING_DEFAULT_BRIGHTNESS);
        }
    }

    // During the short blank gap after any mode change, clear the ring and
    // skip the per-mode rendering so each demo visibly starts from all-off.
    if (millis() - modeChangedMs < TRANSITION_MS) {
        RingClear();
        RingShow();
        return;
    }

    switch (mode) {
        case MODE_SIMULATION: {
            // Opponent simulation (full narrative):
            //   1. enter from the right   -> 2. sweep right -> left
            //   3. circle back to front   -> 4. drive straight at the middle
            //   5. pause close            -> 6. retreat and vanish
            // Sensor range as configured: 9 mm .. 800 mm (0..80 cm).
            const uint16_t FAR_CM   = 80;    // far cm (green zone, 800 mm)
            const uint16_t FAR_MM   = 800;   // far mm for ID3 (green zone)
            const uint16_t CLOSE_CM = 1;     // close cm (red zone, ~9 mm)
            const uint16_t CLOSE_MM = 9;     // close mm for ID3 (red zone, min)
            uint32_t t = millis() - simStartMs;

            // Phase boundaries (ms). Total = SIMULATION_DWELL_MS.
            const uint32_t T_ENTER    = 1800;   // enter right
            const uint32_t T_SWEEP    = 5600;   // right -> left sweep
            const uint32_t T_CIRCLE   = 9800;   // circle back around to front
            const uint32_t T_APPROACH = 16000;  // drive at middle
            const uint32_t T_PAUSE    = 19000;  // hold close
            const uint32_t T_RETREAT  = SIMULATION_DWELL_MS; // fade out

            // Default: everyone far (green), no line.
            tofDistances[0] = FAR_CM;
            tofDistances[1] = FAR_CM;
            tofDistances[2] = FAR_MM;
            tofDistances[3] = FAR_CM;
            tofDistances[4] = FAR_CM;
            lineSensors[0] = lineSensors[1] = lineSensors[2] = false;

            // Linear ramp helper: returns a value lerp'd from a->b over the
            // given [start,end) window, clamped to the ends. Uses signed math
            // so descending ramps (a > b) don't underflow.
            auto ramp = [](uint32_t now, uint32_t start, uint32_t end,
                           int32_t a, int32_t b) -> int32_t {
                if (now <= start) return a;
                if (now >= end)   return b;
                return a + (b - a) * (int32_t)(now - start) / (int32_t)(end - start);
            };

            // 1. Enter from the right: ID1/ID2 (right) fade in from far to close.
            if (t < T_ENTER) {
                tofDistances[0] = (uint16_t)ramp(t, 0, T_ENTER, FAR_CM, CLOSE_CM);
                tofDistances[1] = (uint16_t)ramp(t, 0, T_ENTER, FAR_CM, CLOSE_CM);
            }
            // 2. Sweep right -> left: the close window moves across the ring.
            else if (t < T_SWEEP) {
                uint32_t p = t - T_ENTER;
                const uint32_t S = T_SWEEP - T_ENTER;   // sweep span

                // Right sensors fade out over the first half.
                tofDistances[0] = (uint16_t)ramp(p, 0, S / 2, CLOSE_CM, FAR_CM);
                tofDistances[1] = (uint16_t)ramp(p, 0, S / 2, CLOSE_CM, FAR_CM);

                // Middle: close in the middle of the sweep.
                tofDistances[2] = (uint16_t)ramp(p, S / 4, 3 * S / 4, FAR_MM, CLOSE_MM);
                tofDistances[2] = (uint16_t)ramp(p, 3 * S / 4, S, CLOSE_MM, FAR_MM);

                // Left sensors fade in over the second half.
                tofDistances[3] = (uint16_t)ramp(p, S / 2, S, FAR_CM, CLOSE_CM);
                tofDistances[4] = (uint16_t)ramp(p, S / 2, S, FAR_CM, CLOSE_CM);
            }
            // 3. Circle back: left fades out, right fades back in, then middle.
            else if (t < T_CIRCLE) {
                uint32_t p = t - T_SWEEP;
                const uint32_t C = T_CIRCLE - T_SWEEP;

                tofDistances[3] = (uint16_t)ramp(p, 0, C / 3, CLOSE_CM, FAR_CM);
                tofDistances[4] = (uint16_t)ramp(p, 0, C / 3, CLOSE_CM, FAR_CM);
                tofDistances[0] = (uint16_t)ramp(p, C / 3, 2 * C / 3, FAR_CM, CLOSE_CM);
                tofDistances[1] = (uint16_t)ramp(p, C / 3, 2 * C / 3, FAR_CM, CLOSE_CM);
                tofDistances[2] = (uint16_t)ramp(p, 2 * C / 3, C, FAR_MM, CLOSE_MM);
            }
            // 4. Drive at the middle: ID3 ramps far -> close (green -> red).
            else if (t < T_APPROACH) {
                uint32_t p = t - T_CIRCLE;
                const uint32_t A = T_APPROACH - T_CIRCLE;
                tofDistances[2] = (uint16_t)ramp(p, 0, A, FAR_MM, CLOSE_MM);
            }
            // 5. Pause close on the middle.
            else if (t < T_PAUSE) {
                tofDistances[2] = CLOSE_MM;
            }
            // 6. Retreat: everything fades back to far (opponent vanishes).
            else {
                uint32_t p = t - T_PAUSE;
                const uint32_t R = T_RETREAT - T_PAUSE;
                uint32_t cm = ramp(p, 0, R, CLOSE_CM, FAR_CM);
                uint32_t mm = ramp(p, 0, R, CLOSE_MM, FAR_MM);
                tofDistances[0] = (uint16_t)cm;
                tofDistances[1] = (uint16_t)cm;
                tofDistances[2] = (uint16_t)mm;
                tofDistances[3] = (uint16_t)cm;
                tofDistances[4] = (uint16_t)cm;
            }

            RingShowToFRanges(tofDistances);
            logToF();
            break;
        }

        case MODE_SHOW_LINE:
            lineSensors[0] = ((millis() / 800) % 2) == 0;   // left toggles
            lineSensors[1] = ((millis() / 1400) % 2) == 0;  // center toggles
            lineSensors[2] = ((millis() / 1200) % 2) == 0;  // right toggles
            RingShowLine(lineSensors);
            logLine();
            break;

        case MODE_BRIGHTNESS: {
            // Alternates every 8s between two brightness demos:
            //   1. WHITE  — ramp global brightness 0..255..0 on white LEDs
            //   2. SPECTRUM — cycle the full RGB hue wheel at full brightness
            uint32_t seg = (millis() - modeChangedMs) / 8000;   // 0,1,2,...
            bool spectrum = (seg % 2) == 1;                      // odd = spectrum

            if (!spectrum) {
                // White fade: triangle wave on global brightness.
                uint32_t t = (millis() - modeChangedMs) % 4000;
                uint8_t b = (t < 2000) ? (uint8_t)(t * 255 / 2000)
                                       : (uint8_t)((4000 - t) * 255 / 2000);
                RingSetBrightness(b);
                RingFillRGB(255, 255, 255);
                RingShow();

                if (millis() - lastBrightLogMs > 1000) {
                    lastBrightLogMs = millis();
                    Serial.print(F("  [white] brightness = "));
                    Serial.println(b);
                }
            } else {
                // Full RGB spectrum cycle: hue sweeps 0..255, all LEDs same hue.
                uint8_t hue = (uint8_t)((millis() / 20) % 256);
                RingSetBrightness(RING_DEFAULT_BRIGHTNESS);
                RingFill(CHSV(hue, 255, 255));
                RingShow();

                if (millis() - lastBrightLogMs > 1000) {
                    lastBrightLogMs = millis();
                    Serial.print(F("  [spectrum] hue = "));
                    Serial.println(hue);
                }
            }
            break;
        }

        case MODE_CLEAR:
            // Ring stays blank here.
            break;

        case MODE_DISABLE:
            // Ring is disabled — RingShow() is a no-op, LEDs stay blank
            // until the next mode switch re-enables it.
            break;
    }
}

#else

// ============================================================================
// LED NUMBERING TEST MODE (ENABLE_NUMBERING_MODE == 1)
// ============================================================================

static uint8_t lastPrinted = 0xFF;

void setup() {
    Serial.begin(115200);
    delay(5000);

    RingInit();               // init + clear
    Serial.println(F("LED numbering test mode"));
    Serial.println(F("RING_PIN=A2  LED_COUNT=7"));
    Serial.println(F("Lights one LED at a time (0..6) to verify ordering."));
    Serial.println();
}

void loop() {
    // Light exactly one LED at a time so the physical position of each
    // index (0..6) can be visually verified. Every 800ms the lit LED
    // advances; the current index is also printed to serial.
    uint8_t led = (millis() / 2000) % RING_LED_COUNT;
    RingClear();
    RingSetLED(led, 255, 255, 255);
    RingShow();

    if (led != lastPrinted) {
        lastPrinted = led;
        Serial.print(F("Numbering: LED "));
        Serial.println(led);
    }
}

#endif // ENABLE_NUMBERING_MODE
