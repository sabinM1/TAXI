# TAXI — Mini Sumo Robot Firmware

Autonomous mini-sumo (500 g class) combat firmware for a 5× ToF-sensor, 3-line-sensor robot running on the JSUMO XMotion V3 controller, flashed as **Arduino Leonardo (ATmega32U4)**.

---

## 1. Project Overview

TAXI is an autonomous mini sumo robot: it waits for the competition start signal, executes a DIP-switch-selectable opening move, then hunts, tracks and pushes opponents using an array of five M60F laser ToF sensors. Three QRE1113 reflectance sensors guard the white dohyo edge and trigger an escape maneuver from any state. A 7-LED WS2812 ring shows live ToF/line telemetry in the pit (and, in debug builds, during the match too).

Architecture: `QRE1113 edge sensors + M60F ToF array → ATmega32U4 (XMotion V3) → dual H-bridge → 2× JSUMO Core 6V 400 RPM motors`, with `DIP switches → configuration input` and `WS2812 ring + USB serial → status output`. All speeds are compile-time constants (no trimpot master knob, see §7).

The combat flow is a finite state machine: `IDLE → TACTIC → SEARCH → ATTACK`, with `BACKOFF` (opponent alongside) and `EVADE` (edge) as overrides from any combat state, and `STOP` on loss of the start signal. SEARCH and ATTACK are separated by a single distance (`TARGET_ENGAGE_MM`), and both use the same steering law — there is no separate pivot state.

---

## 2. Hardware / Bill of Materials (BOM)

| Component | Model/Part No. | Qty | Voltage/Rating | Notes |
|---|---|---|---|---|
| Controller board | JSUMO XMotion V3 | 1 | 7–28 V input, 1.6 A 5 V rail, 2×6 A motor driver | ATmega32U4, flashed as Arduino Leonardo (confirmed) |
| Drive motors | JSUMO Core 6V 400 RPM (48:1) | 2 | 6 V nominal, 3.2 A stall each | Wired to onboard H-bridges (MOT1/MOT2) |
| Wheels | JSUMO JS2622S (steel hub, 26.5 × 22 mm, 20 Shore A silicone) | 1 set (2) | N/A | 3 mm D-shaft fit; spare tires (JS2622 "Rezerve") on hand |
| Opponent sensors | SoBotz M60F ToF | 5 | 2.8–5.5 V, 25 mA typ / 30 mA max each | OneWire serial mode, daisy-chained; FOV 18–22°, start-up 10–50 ms, sampling 5–50 ms |
| Edge sensors | QRE1113 IR reflective | 3 | 5 V, analog out | Left / rear / right of blade |
| Status display | WS2812B 5050 7-LED ring | 1 | 5 V, ~126 mA max all-white | Driven via FastLED |
| Start module | NanoLabs MicroStart (JSUMO MicroStart type) | 1 | 5 V/3.3 V, digital out | Active-HIGH latching; must power up LOW (armed) — firmware enforces this |
| IMU | TDK InvenSense ICM20602 | 1 | 1.71–3.6 V, I2C/SPI | Present in BOM, **not wired in this firmware** (see §14) |
| Blade | Kanzawa ShadowEdge, 100 × 15 × 0.25 mm | 1 (+9 spares) | N/A | Mounted with 3M 9448A tape |
| Battery | TATTU 3S 11.4 V 300 mAh 75 C LiHV | 1 (+4 spares) | 13.05 V full charge (LiHV) | Main power |
| Chassis material | PolyMax Tough PC (3D printed) | — | N/A | Structure/mounts |
| Fasteners/misc | M3 brass inserts, M3×35/50 Torx screws, 3M 9448A tape, 26/28 AWG silicone wire | — | N/A | Assembly |

---

## 3. Wiring & Pinout

All pin numbers match `Config.h` exactly. Motors are driven directly by the onboard H-bridges; only signal wiring is external.

| Arduino Pin | Function | Connected To | Pin Mode | Notes |
|---|---|---|---|---|
| D10 | Left motor PWM (`PIN_LM_PWM`) | XMotion MOT1 PWM (onboard) | OUTPUT | Timer4 channel B (`OC4B`) hardware PWM, 8-bit at ~977 Hz — rate matched to the right channel; **not** `analogWrite()`, see §13 |
| D12 | Left motor DIR (`PIN_LM_DIR`) | XMotion MOT1 DIR (onboard) | OUTPUT | HIGH = forward before `LM_INVERTED` flip |
| D11 | Right motor PWM (`PIN_RM_PWM`) | XMotion MOT2 PWM (onboard) | OUTPUT | Timer0 PWM via `analogWrite()` |
| D13 | Right motor DIR (`PIN_RM_DIR`) | XMotion MOT2 DIR (onboard) | OUTPUT | HIGH = forward |
| D4 | ToF bus (`PIN_TOF_BUS`) | SIG of all 5 M60F, tied together | INPUT (idle) | Half-duplex single wire; 4.7 kΩ pull-up to 5 V on the harness; driven by `SingleWireSerial` |
| A2 | LED ring (`PIN_RING`) | WS2812 DIN via ~330–500 Ω series resistor | OUTPUT (FastLED) | `FastLED.show()` blocks interrupts — must run AFTER all ToF reads (see `loop()`) |
| A0 | Start/stop (`PIN_START`) | NanoLabs MicroStart out | INPUT_PULLUP | HIGH = fight, LOW = armed/stopped; **read raw every loop, no debounce**; must read LOW at boot |
| A1 | Right line (`PIN_LINE_R`) | QRE1113 out (U9) | INPUT (analog) | Edge = LOW (white border); black ≳ 800 / white ≲ 400 |
| A4 | Rear line (`PIN_LINE_B`) | QRE1113 out (U8) | INPUT (analog) | Edge = LOW (white border); black ≳ 820 / white ≲ 765 |
| A5 | Left line (`PIN_LINE_L`) | QRE1113 out (U7) | INPUT (analog) | Edge = LOW (white border); black ≳ 800 / white ≲ 400 |
| D5 | DIP bit 0 (`PIN_DIP1`) | Onboard DIP switch | INPUT_PULLUP | Switch ON = bit set (firmware inverts; pin reads 1 when switch is DOWN/OFF) |
| D6 | DIP bit 1 (`PIN_DIP2`) | Onboard DIP switch | INPUT_PULLUP | Same |
| D7 | DIP bit 2 (`PIN_DIP3`) | Onboard DIP switch | INPUT_PULLUP | Same |
| A3 | Trimpot (`PIN_TRIMPOT`) | Onboard trimpot wiper | Not configured | Reserved — the firmware never reads it (no master-speed knob, see §7) |
| D8, D9 | User LEDs (`PIN_LED1/2`) | Onboard | OUTPUT | Reserved; currently unused at runtime |

**ToF geometry (verified in `Full_Robot_Test`/`RingInfoDisplay.h`):**

```
   id4(-25°)   id3(0°, mm!)   id2(+25°)     front = blade side
        \          |          /
   id5 (-90°) — [ ROBOT ] — id1 (+90°)
```

- Sensors 1, 2, 4, 5 answer register `0x06` in **cm**; sensor 3 answers register `0x01` in **mm**.
- No-response sentinels: `0xFF` (cm), `0xFFF` (mm); valid window 9–800 mm.
- Steering authority per sensor (`TOF_WEIGHT` in `Sensors.h`, left→right): **4 / 3 / 0 / 3 / 4**, scaled by `TOF_WEIGHT_SCALE = 25`. A sighting on a side sensor alone therefore commands a full ±100 bearing, a front-outer sensor alone ±75, and the middle sensor alone 0 (dead ahead, but it only sets the *direction*, never the strength).

**Reserved / do-not-touch pins:** D0/D1 (Serial1 — free but avoid conflict with USB debugging assumptions), D2 (Leonardo SDA, free), plus anything the USB stack owns. No level shifting anywhere: all external modules run at 5 V logic.

---

## 4. Power Requirements

| Load | Voltage | Peak current | Notes |
|---|---|---|---|
| 2× Core 6V motors | via onboard driver from battery | 2 × 3.2 A (stall) | Motors confirmed tolerant of the full 13.05 V LiHV charge — no duty cap required for matches |
| M60F ×5 | 5 V rail | 5 × 30 mA max = 150 mA | Datasheet: 25 mA typ, 30 mA max each |
| QRE1113 ×3 | 5 V rail | 3 × ~2 mA LED + pull-up | Analog modules |
| WS2812 ring | 5 V rail | ≤126 mA (all white) | Brightness-limited in software |
| XMotion logic | 5 V rail | ~50 mA | ATmega32U4 + peripherals |

Total worst-case is dominated by motor stalls (~6.4 A burst); the battery (75 C, 22.5 A continuous) is not the limit — the driver (6 A/channel) is. Brownout risk: motor stalls drag the shared battery rail; the board's onboard switching regulator isolates the 5 V logic rail, but avoid prolonged stalls. Both motors tolerate the full LiHV 13.05 V, so no software voltage cap is needed for matches (the bench cap exists only for desk safety).

---

## 5. Software Dependencies & Setup

- Toolchain used: **arduino-cli 1.4.1** (or Arduino IDE 2.x equivalent)
- Board core: **arduino:avr 1.8.8**
- Board (exact menu name): **Arduino Leonardo**

| Library | Version pinned | Purpose |
|---|---|---|
| FastLED | 3.10.5 | WS2812 ring |
| SingleWireSerial | 1.2.0 | M60F half-duplex bus |

`DistanceSensor.h`, `LineSensor.h`, `RingInfoDisplay.h` ship inside the sketch folder (copied from the verified `resources/Full_Robot_Test` project) — no installation needed.

---

## 6. Firmware Structure & Logic

`TAXI_Firmware.ino` (sketch root) + headers; header-only modules, single translation unit.

```
setup(): claim motor pins + stop, 3 s startup delay, Serial begin, pins, DIP
         latch, hard-lock if A0 reads HIGH at boot (module still latched on),
         sensors init (ToF bus down @115200, line pins), ring init, 500 ms ToF
         boot delay.
loop():  sensorsUpdate()      // 1 ToF read + 1 line pass — ALL serial I/O
         stateMachineUpdate() // motor decisions (stop check first)
         ringStatusUpdate()   // FastLED.show() LAST (blocks interrupts)
```

**State machine** (`StateMachine.h`):

| State | Behavior |
|---|---|
| `ST_IDLE` | Motors stopped; waits for start signal HIGH (read raw each loop) |
| `ST_TACTIC` | Plays the DIP-selected opening timed-step sequence, then → SEARCH. Runs to completion; only the line-edge override (EVADE) can interrupt it |
| `ST_SEARCH` | Spin/arc/sweep pattern at `SEARCH_SPEED_PCT`; flips direction every 3 s if blind. A sighting **beyond** `TARGET_ENGAGE_MM` is turned onto slowly (same steering law, search speed) without committing; inside `TARGET_ENGAGE_MM` → ATTACK |
| `ST_ATTACK` | Opponent inside `TARGET_ENGAGE_MM`: speed from the `ATTACK_SPEED_RAMP` table (35 % at the engage distance, 100 % inside `ATTACK_CLOSE_DIST_MM`) + one proportional steering term (`steerToward()`, scaled by base speed, so a hard bearing counter-rotates the inner wheel). All 5 ToF sensors stay polled. At blade contact (≤ `PUSH_CONTACT_DIST_MM` on the middle sensor) both wheels go flat out and the steering trim is skipped |
| `ST_BACKOFF` | Opponent alongside (a SIDE ToF at ≤ `SIDE_BACKOFF_DIST_MM` **held** for `SIDE_BACKOFF_CONFIRM_MS`): reverse flat out for `SIDE_BACKOFF_MS`, then → SEARCH. Rate-limited by `SIDE_BACKOFF_COOLDOWN_MS`; the line-edge override still outranks it |
| `ST_EVADE` | Edge override, two timed phases: FLEE (≥35 ms blind reverse, keeps fleeing while any sensor reads white, ≤300 ms; rear edge drives **forward**) → TURN (fixed 120 ms pivot away from the edge) → SEARCH |
| `ST_STOP` | Start signal lost → motors off, latched until power cycle |

**Key functions:**

| File | Function | Purpose |
|---|---|---|
| `Motor.h` | `motorsSet(l, r)` | Signed-percent drive, applies bench cap + left-motor inversion |
| | `motorsStop()` | Coast both motors |
| `Sensors.h` | `sensorsUpdate()` | One ToF poll + one line pass per loop |
| | `sensorsOpponent()` | Bearing −100..+100 (127 = nothing), weighted vote: per-sensor steering authority `TOF_WEIGHT` `{4, 3, 0, −3, −4}` (side, front-outer, middle, front-outer, side) × inverse distance **squared**, so the closest sensor dominates; the front trio outranks the side sensors |
| `StateMachine.h` | `targetBearing()` | The bearing actually steered on: the live reading, or the remembered last-seen direction when the sensors only see something far away or nothing |
| | `enterSearch()` | Begins a search; biases the first leg toward the last-seen direction |
| | `sensorsClosestMm()` | Nearest valid distance |
| | `sensorsContact()` | True when the MIDDLE sensor reads blade contact (<`PUSH_CONTACT_DIST_MM`) |
| | `sensorsAnyEdge()` / `sensorsEdgeIndex()` | Debounced edge flags, priority rear>left>right |
| | `sensorsHealthyCount()` | ToF sensors currently **answering** on the bus (a sensor with nothing in range is still healthy) |
| `Strategies.h` | `strategiesInit()` | Read DIP, latch strategy 0–7 |
| | `tacticRun()` | Non-blocking opening executor |
| `StrategyData.h` | `STRATEGIES[8]` | The opening data (TacticStep arrays + table + per-opening start pose + debug names). Single source of truth, read/written by `tools/strategy-designer.html` |
| `StateMachine.h` | `stateMachineUpdate()` | Priority: stop signal > edge > side backoff > state body (sole motor command site) |
| | `startModuleUpdate()` | Raw start/stop sample — no debounce, by design |
| | `steerToward(base, bearing)` | **The** steering law: drive at `base`, shift a bearing-scaled differential between the wheels. Used by the attack and by SEARCH's approach leg |
| | `attackUpdate(bearing)` | One chase tick: speed from the closest reading, steering from `steerToward()` |

**Protocols:** ToF bus = printable ASCII-hex (`R<id><reg>\n` query, hex value + `\n` reply) over `SingleWireSerial` @ 115200 on D4. LEDs = WS2812 800 kbps via FastLED. Debug = USB `Serial` @ 115200.

---

## 7. Configuration Constants

All in `Config.h` (pins listed in §3 are omitted here).

| Constant | Default | Meaning | Safe Range |
|---|---|---|---|
| `LM_INVERTED` / `RM_INVERTED` | true / false | Per-side logical-direction flip for mounting | true/false |
| `LM_TRIM_PCT` / `RM_TRIM_PCT` | 100 / 100 | Per-side duty trim for straight-line calibration (100 = no change; only trim the faster side DOWN) | 80–100 |
| `MOTOR_BENCH_LIMIT_ENABLED` | false | Clamp all motor commands to `MOTOR_BENCH_MAX_PCT` | true (bench) / false (match) |
| `MOTOR_BENCH_MAX_PCT` | 25 | Bench power cap % | 10–50 |
| `TOF_MIN_VALID_MM` / `TOF_MAX_VALID_MM` | 9 / 800 | Accepted measurement window of the M60F config | per sensor config |
| `LINE_LEFT_WHITE_THRESHOLD` / `LINE_LEFT_BLACK_THRESHOLD` | 400 / 800 | Left edge (raw below) / black (raw above) | recalibrate on venue |
| `LINE_REAR_WHITE_THRESHOLD` / `LINE_REAR_BLACK_THRESHOLD` | 765 / 820 | Rear edge (raw below) / black (raw above) | recalibrate on venue |
| `LINE_RIGHT_WHITE_THRESHOLD` / `LINE_RIGHT_BLACK_THRESHOLD` | 400 / 800 | Right edge (raw below) / black (raw above) | recalibrate on venue |
| `LINE_DEBOUNCE_SAMPLES` | 2 | Consecutive hits to confirm an edge | 1–5 |
| `SEARCH_PATTERN_TIMEOUT_MS` | 3000 | Blind-search time before mirroring the pattern | 1000–6000 |
| `ATTACK_TARGET_LOST_MS` | 400 | Target-loss grace before falling back to SEARCH | 200–1000 |
| `ATTACK_SPEED_CLOSE_PCT` | 100 | Base speed at `ATTACK_CLOSE_DIST_MM` (the near anchor of the ramp table) | 40–100 |
| `ATTACK_SPEED_FAR_PCT` | 35 | Base speed at `TARGET_ENGAGE_MM` (the far anchor), and the blind-push speed when nothing is visible | 20–100 |
| `ATTACK_CLOSE_DIST_MM` | 60 | Full power from here in. Deliberately equal to `PUSH_CONTACT_DIST_MM`, so the speed is already at `PUSH_CONTACT_SPEED_PCT` when the contact latch takes over | 20–800 |
| `ATTACK_SPEED_RAMP` (table, `Config.h`) | 300→35, 200→45, 100→60, 60→100 | **The attack speed profile** — (distance, speed) anchors interpolated linearly in the closest ToF reading. Gives >300 mm `SEARCH_SPEED_PCT`, then 300→35 %, 250→40 %, 200→45 %, 150→53 %, 100→60 %, 80→80 %, ≤60→100 %. Far → near order | anchors 20–800 mm, 0–100 % |
| `PUSH_CONTACT_DIST_MM` / `PUSH_CONTACT_SPEED_PCT` / `CONTACT_LATCH_MS` | 60 / 100 / 250 | MIDDLE ToF contact distance, the flat speed used there (both wheels, no steering trim), and how long contact stays latched after the last sighting | 20–120 / 60–100 / 100–500 |
| `ATTACK_STEER_GAIN_PCT` | 100 | The turn-in: differential at bearing ±100 as a % of the current base speed. At attack speed this counter-rotates the inner wheel; at search speed the same bearing is a gentle arc | 50–150 |
| `TARGET_ENGAGE_MM` | 300 | **The** mode switch. SEARCH commits to ATTACK inside this distance, and a sighting inside it is also the one trusted/remembered as "last seen" | 100–800 |
| `SEARCH_SPEED_PCT` | 30 | Base speed of every search pattern and of the far-sighting approach leg — the speed everywhere outside `TARGET_ENGAGE_MM` | 5–80 |
| `EVADE_REVERSE_SPEED_PCT` | −80 | FLEE speed for front edges (negative = reverse) | −100..−50 |
| `SIDE_BACKOFF_DIST_MM` / `SIDE_BACKOFF_CONFIRM_MS` / `SIDE_BACKOFF_MS` / `SIDE_BACKOFF_SPEED_PCT` / `SIDE_BACKOFF_COOLDOWN_MS` | 20 / 350 / 100 / −100 / 300 | Opponent-alongside backoff: side-ToF trigger distance, how long the reading must HOLD before it fires, reverse duration, reverse speed, and minimum gap between backoffs | 10–60 / 50–400 / 50–300 / −100..−50 / 100–800 |
| `RING_SPEED_LED_BRIGHTNESS` | 102 | Brightness (0–255) of the LED-1 drive-speed indicator (40 %) | 0–255 |
| `TOF_HEALTH_MAX` | 8 | Per-sensor health counter ceiling | — |
| `TOF_MIN_HEALTHY_SENSORS` | 3 | Fewer ToF sensors *answering* than this → degraded blind spin-search | 1–5 |
| `DEGRADED_PUSH_SPEED_PCT` | 20 | Blind push speed in ATTACK while the bus is degraded, instead of `ATTACK_SPEED_FAR_PCT` | 10–35 |
| `EVADE_ESCAPE_MIN_MS` | 35 | FLEE floor: mandatory blind reverse burst before the line sensors are allowed to end the flee | 10–150 |
| `EVADE_FLEE_MAX_MS` | 300 | FLEE ceiling when a sensor will not clear | 150–500 |
| `EVADE_TURN_MS` | 120 | TURN: pivot duration — the overshoot knob (rotation = this × the pivot speed) | 30–500 |
| `EVADE_TURN_OUTER_PCT` / `EVADE_TURN_INNER_PCT` / `EVADE_TURN_SPIN_PCT` | 70 / 50 / 60 | TURN pivot speeds: outer wheel forward, inner reversed, both wheels for a rear spin | 30–100 |
| `EVADE_TOTAL_MAX_MS` | 800 | Safety net for one whole evade program (not a tuning knob) | 500–1500 |

\* The 6 V motors are running off a 13 V rail — treat 100 % duty cautiously (see §4).

---

## 8. Build & Upload

1. Install **Arduino IDE 2.x** (or `arduino-cli` 1.4+).
2. Boards Manager → install **"Arduino AVR Boards" 1.8.8**.
3. Library Manager → install **FastLED 3.10.5** and **SingleWireSerial 1.2.0**.
4. Open `TAXI/TAXI_Firmware/TAXI_Firmware.ino`.
5. Select board **Arduino Leonardo** and the robot's COM port.
6. Verify, then Upload. No bootloader button dance needed on Leonardo (double-tap reset only if a corrupt sketch blocks USB).

CLI equivalent:

```
arduino-cli compile --fqbn arduino:avr:leonardo TAXI_Firmware
arduino-cli upload  --fqbn arduino:avr:leonardo -p <PORT> TAXI_Firmware
```

Expected sketch size: ~12.7 KB flash (44 %), ~0.66 KB RAM — verified compiling this revision.

---

## 9. Usage / Operation

**Match-day arming (NanoLabs module rules):**

1. Set the DIP switches (D5/D6/D7) to pick the opening strategy **before power-on** (latched once at boot):

| DIP (D5 D6 D7) | Strategy | Opening | Search style after |
|---|---|---|---|
| 000 | 0 | Rush (dash) 250 ms | Spin |
| 001 | 1 | Turn-right 120 ms → rush | Arc |
| 010 | 2 | Turn-left 120 ms → rush | Arc |
| 011 | 3 | Retreat 200 ms → rush (fake-out) | Spin |
| 100 | 4 | Angle-45 flank right → rush | Arc |
| 101 | 5 | Angle-90 side attack → rush | Arc |
| 110 | 6 | Hit-spin 200 ms → rush | Sweep |
| 111 | 7 | Wait 300 ms → rush (deceptive) | Spin |

These are the "best of" the researched mini-sumo openings (rush, turn-and-dash, flank, fake-out, hit-spin, deceptive wait). The opening data lives in **`StrategyData.h`** (the single source of truth). Each opening also stores the **start pose** it assumes — `{ x, y, heading }` in cm/deg from the dohyo centre — so a strategy can be designed for a specific placement (centred on the shikiri-sen, off to one side, angled, …). Design and tune it visually with **[`tools/strategy-designer.html`](tools/strategy-designer.html)**: import `StrategyData.h`, pick a slot, drag the robot to that opening's start pose, draw the steps, simulate it with your measured speeds, and export it back over the same file.

2. All speeds are fixed constants in `Config.h` (§7) — the robot has no master-speed knob, so the same DIP selection gives the same run every time.
3. **Power on with the module in its armed state.** The module saves state in non-volatile memory — if the robot was previously started and never received a stop, the pin will already be HIGH at boot. The firmware detects this, locks in place, and blinks the onboard LED (D8) rapidly: ask the referee for a stop command, then power-cycle.
4. Ring shows: **live pit telemetry** — the 5 ToF sensors as a green→red distance gradient on the outer LEDs (green = far, red = very close, blank = no reading), any line sensor over white overlaid solid white, a green heartbeat blinking on the centre LED, and the **back LED (1) doubling as a drive-speed indicator**: cyan at 100 % speed fading to green at 0 %, at 40 % brightness (`RING_SPEED_LED_BRIGHTNESS`). A rear edge still overrides it to white.
5. When the referee sends start, the pin goes HIGH: the robot runs its opening, then hunts. The ring goes dark for the whole fight — except in a `DEBUG_VERBOSE` build, where it keeps showing the telemetry below.
6. The referee's stop command drops the pin LOW. The pin is read raw on every loop, so the stop is registered on the very next `stateMachineUpdate()` — motors cut within one loop (~2 ms typical, ~15 ms worst case if a dead ToF sensor has to time out first). The robot then stays frozen, ring solid white, until power cycle, as required.

Ring during the match: **off** (TACTIC/SEARCH/ATTACK/BACKOFF/EVADE) — it is blanked once on entering the fight and `FastLED.show()` stops being called, so it costs no loop time and cannot disturb the ToF bus. IDLE = live ToF gradient + white line overlay + speed indicator on LED 1 + green heartbeat. STOP = solid white, latched once.

**Debug exception:** with `DEBUG_VERBOSE true` the IDLE telemetry (ToF gradient + line overlay + heartbeat) keeps running through the fight as well, so you can watch what the sensors see while the robot actually drives. It is safe for the ToF bus — `loop()` always finishes the sensor read before the ring update calls `FastLED.show()`. STOP still overrides it with solid white. Set `DEBUG_VERBOSE false` for a real match: that is what makes the ring go dark and silent while fighting, and it is also what disarms the watchdog (§12).

Two details worth knowing while watching that fight-time telemetry:

- Every ToF sensor is polled on every sweep, in every state. There is no front-only "attack scan" any more: the chase has to be able to turn onto an opponent that is off to a side, and BACKOFF needs the side sensors live. The full five-sensor round-robin costs ~10 ms, which is fast enough for all of it.
- The line overlay works normally in a fight, which makes EVADE easy to watch: a white LED on the ring is the exact sensor that triggered the escape.

Serial monitor @115200 prints `[TAXI] firmware up` and the latched DIP strategy at boot.

---

## 10. Calibration & Tuning

**Line sensors**
- Raw ADC is HIGH over black (interior) and LOW over white (border); an edge is a LOW reading.
- Per-sensor thresholds live in `Config.h`: left/right white <400 (black >800), rear white <765 (black >820). These are the verified on-robot values.
- To recalibrate: run `IR_Line_Test` and note the black and white raw bands for each of L / B / R, then set each sensor's white threshold just above its white band and its black threshold just below its black band.

**Straight line (drive trim)**
- Both motors now run at the same PWM frequency (977 Hz) so they make the same torque at the same duty — the remaining curve, if any, is mechanical (motor, gearbox, wheel fit).
- Calibrate on the floor: command a straight line and watch which way it curves. If it curves **left**, the left side is faster, so step `LM_TRIM_PCT` down (97, 94, …) and re-flash until it tracks straight. 1–3 % is usually enough.
- Only ever trim the **faster** side down — duty cannot exceed 255, so there is no headroom to trim a slow side up. The cost of the trim is that the trimmed side's top speed drops by the same few percent.

**Speeds**
- No master knob: every speed is an absolute `*_PCT` constant in `Config.h` (§7). If a speed is wrong for the venue, change the constant and re-flash.
- Indoor testing: set `MOTOR_BENCH_LIMIT_ENABLED` to `true` (default is `false`, i.e. full match power) — that clamps every motor command, evade and openings included.

**Attack aggression (how hard it goes for the opponent)**
- `TARGET_ENGAGE_MM` — **the** aggression/latency knob, and the one to change first. It is the single distance at which SEARCH commits to ATTACK. Lower it and the robot hunts longer before charging (safer, slower to engage); raise it and it starts charging from further out, on a weaker bearing.
- `ATTACK_SPEED_RAMP` (the table in `Config.h`) — **the** shape of the approach, and the knob to reach for first. Add or move anchors (far → near) to put speed exactly where you want it; the code interpolates linearly between them and clamps outside the first and last. Current profile:

  | closest ToF | base speed |
  |---|---|
  | > 300 mm | `SEARCH_SPEED_PCT` (30 %, still SEARCH) |
  | 300 mm | 35 % |
  | 250 mm | 40 % |
  | 200 mm | 45 % |
  | 150 mm | 53 % |
  | 100 mm | 60 % |
  | 80 mm | 80 % |
  | ≤ 60 mm | 100 % |

  The first anchor's distance is `TARGET_ENGAGE_MM` and its value is `ATTACK_SPEED_FAR_PCT`; the last anchor's distance is `ATTACK_CLOSE_DIST_MM` and its value is `ATTACK_SPEED_CLOSE_PCT`. Moving those four constants moves the endpoints — edit the middle anchors to reshape the curve between them.
- `ATTACK_STEER_GAIN_PCT` — how hard it turns onto the opponent. Raise until it weaves/oscillates, then back off.
- `SEARCH_SPEED_PCT` — how fast it hunts, how fast it closes on a far sighting, and how fast it turns onto one (the steering correction is scaled by base speed). 30 % is a compromise: enough to cover the dohyo and keep a usable turn rate while approaching, without carrying so much speed that it arrives at an off-centre opponent before finishing the turn.
- `PUSH_CONTACT_DIST_MM` / `CONTACT_LATCH_MS` — the head-on collision response: the MIDDLE sensor distance that triggers the full-power both-wheels shove (no steering trim), and how long that commitment sticks if the reading goes invalid (the blade ends up inside the sensor's 9 mm minimum). Raise the distance if it still reacts late on a head-on; the latch only ever *extends* an already-detected shove.

**Target tracking (combat turning)**
- There is only one steering law now — `steerToward()` in `StateMachine.h`, used at both attack and search speed. There is no ATTACK/FLANK threshold to tune or to chatter across, and no state in which the robot pivots on the spot without making forward progress.
- Tuning, in order of effect:
  1. `TOF_WEIGHT` (in `Sensors.h`, not `Config.h`) — how hard each sensor asks the robot to turn. The front pair at ±3 commands 75 of the ±100 the side sensors command; set them to ±4 to make a front-corner sighting turn exactly as hard as an alongside one. This is the knob for "it sees it off to a corner but does not come round onto it".
  2. `ATTACK_STEER_GAIN_PCT` — the turn-in at every speed. If the robot sweeps past the opponent, lower it; if it under-turns and slides by, raise it.
  3. `TARGET_ENGAGE_MM` — see the aggression section above.
- Read the debug table: `Contact=1` means the middle sensor is inside `PUSH_CONTACT_DIST_MM`, i.e. the robot is in a shoving contest; `Closest=` is the distance the speed ramp is reading.
- **If the robot evades or spins while the ring clearly shows the opponent, check the `Edge` row first.** `Edge 1 1 1` on the black interior means the line thresholds are wrong for this surface — see §12.

**Opening timings**
- Each 100 % pivot step ≈ how many ms = 90° on YOUR robot must be measured (`s1`/`s2` use 120 ms as a first guess). Film a run at 120 fps or mark-and-count turns; adjust the step durations in `StrategyData.h`.
- Fastest way to do this: open **[`tools/strategy-designer.html`](tools/strategy-designer.html)**, import `StrategyData.h`, set the two calibrations (**straight speed** in cm/s @ 100 % and **turn rate** in deg/s @ full differential) to your measured values, then drag the robot to its real start position and read the simulated heading / edge-exit time for each opening. The **start pose is per slot** — each opening remembers where it is placed, and that pose is saved back into `StrategyData.h` with the steps. The tool's **probe** readout answers questions like "placed at the bottom middle, how long until it reaches the left edge?" directly — set the probe heading and read the ms.
- The opening always runs to completion; the only thing that can interrupt it is the line-edge override (EVADE), which outranks every state body.

**Attack steering**
- If the robot over-corrects and oscillates around the opponent, lower `ATTACK_STEER_GAIN_PCT`; if it drifts off-line while pushing, raise it.

**Evade turn (the edge reflex)**
- The FLEE phase is the reverse: it runs for at least `EVADE_ESCAPE_MIN_MS` (the blind burst, which is what actually backs the robot off a clipped border) and then for as long as any sensor still reads white, up to `EVADE_FLEE_MAX_MS`. Raise `EVADE_ESCAPE_MIN_MS` if a line clip is registered and then driven straight back over.
- How far the robot rotates away from an edge is set by `EVADE_TURN_MS` (duration) times the pivot speeds `EVADE_TURN_OUTER_PCT` / `EVADE_TURN_INNER_PCT`. Change the duration first — it is the cleanest knob.
- Procedure: flash `resources/Edge_Turn_Test`, put the robot on the dohyo with one line sensor just over the white, run one event (`g`, or arm with `a` and nudge the robot over the edge), mark the heading before and after, then scale `EVADE_TURN_MS` by (wanted degrees ÷ measured degrees). Full command list, log format and the numbers-to-constants mapping: **[`resources/Edge_Turn_Test/GUIDE.md`](../resources/Edge_Turn_Test/GUIDE.md)**.
- Watch the `STILL WHITE: flee is too short` warning in that log. If it appears, the robot started pivoting while still over the white: raise `EVADE_ESCAPE_MIN_MS` or `EVADE_FLEE_MAX_MS` before touching the turn.

---

## 11. Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| Robot spins in place instead of driving straight | Left wiring/mounting flip not accounted for | Toggle `LM_INVERTED` in `Config.h` |
| Robot spins a full circle while the ring shows ToF detections | Only 1–2 sensors see the opponent; the rest read "nothing in range" and were counted as faults, so `sensorsHealthyCount()` fell below `TOF_MIN_HEALTHY_SENSORS` and the robot went degraded (blind spin, bearing ignored) | Fixed in `tofReadOne()`: health now tracks whether a sensor *answers*, not whether it sees a target. A visible target also now always beats the degraded fallback |
| ToF values all invalid, red ring blink | Bus baud mismatch / bus pin left floating / sensor ids not 1–5 | Re-run `Full_Robot_Test` mode `1`; check 4.7 kΩ pull-up; reconfigure ids (per SoBotz tool) |
| Random resets when motors start | Battery sag on stall | Charge/replace battery; verify the driver is not overloaded; bench-test with the power cap on |
| Robot evades while still on black interior (false edge) | White threshold too high — black reads dip below it | Recalibrate white/black thresholds with `IR_Line_Test` (§10) |
| No reaction to start signal | Module still latched on from a previous match, or wiring | If the onboard LED blinks rapidly at power-on: ask referee for stop command, then power-cycle. A0 must read LOW before the fight. |
| Ring frozen / ToF stutter together | `FastLED.show()` interrupt blocking during a sensor read | Keep loop order: sensors → motors → ring (already enforced) |
| Robot hunts for a long time before engaging | `TARGET_ENGAGE_MM` is small for the venue, or a ToF sensor is degraded so the closest reading is missing | Raise `TARGET_ENGAGE_MM`; check the `Health=` field in the debug table |
| Ring solid white and robot unresponsive after a reset | Any reset mid-round is caught by the boot lock — the module still reports fight | Ask the referee for stop, then power-cycle. Unavoidable by design (§13) |
| Robot restarts every ~250 ms on the bench (motors cut, then boot again) | `DEBUG_VERBOSE` build arms a 250 ms watchdog and something blocked the loop (long serial dump / host stopped reading the port) | Keep the Serial Monitor open, or set `DEBUG_VERBOSE false` and re-flash |
| Compiles but USB port disappears mid-upload on Leonardo | Known 32U4 USB-resets | Double-tap reset, then upload within the 8 s bootloader window |

---

## 12. Safety Notes

- **LiHV battery:** charge ONLY with a LiHV-capable charger (4.35 V/cell), balance-charge always, charge/store in a fireproof LiPo bag, never below 3.0 V/cell or above 4.35 V/cell; discard swollen packs.
- **Watchdog:** bench builds (`DEBUG_VERBOSE true`) arm a 250 ms watchdog, so if the loop ever locks up while you are holding the robot the board resets and the motors cut instead of running on. Match builds deliberately do **not** arm it — see §13.
- **Blade:** the Kanzawa blade is a honed steel edge — handle the robot by the chassis, never by the front.
- **Motors:** wheels can grab fingers/cable at full speed — bench-test with the robot on blocks and `MOTOR_BENCH_LIMIT_ENABLED = true`.
- Spun-up robot falling off a desk will not stop: always bench-test with wheels free-spinning.
- Lead-tin balancing strips: wash hands after handling, wear a mask if soldering/cutting.

---

## 13. Known Limitations / Future Work

- The ICM20602 IMU in the BOM is **not wired in this firmware** (its default I²C pins D2/D3 are unused by the motor driver, but the sensor is still absent from the current wiring; SPI remap possible — open task).
- **Timer1 belongs to the ToF bus.** `SingleWireSerial` (D4 = `ICP1`) needs Timer1 for bit timing, so `analogWrite()` is dead on D9 and D10 (`OC1A`/`OC1B`). The left motor therefore runs off Timer4 channel B (`OC4B`, which is also on PB6/D10) — see `Motor.h`. Consequences for future work: never call `analogWrite(10, …)`; Timer4 is taken, so D13 (`OC4A`) and D6 (`OC4D`) can't be PWM either; and any library that grabs Timer1 will break either the left motor or the sensor bus. Full timer map and reasoning: **[`TIMERS.md`](TIMERS.md)**.
- No PID — the chase steering is a proportional correction on the live bearing only (a PD on bearing rate is the next tuning step).
- QRE thresholds are fixed at compile time; no on-venue auto-calibration routine yet.
- Tactic step durations are estimates pending on-dohyo calibration (§11).
- M60F reads return the last stored value in ~0.75 ms, but under active ranging the sensor MCU can hold the bus for its full sampling budget (reg `0xB0` = 12 ms); `TOF_TIMEOUT_MS` is 15 to cover that. Only one sensor is polled per loop, so a dead sensor bounds the loop at ~15 ms, not 5 × 15 ms.
- **No watchdog in a match build** (`DEBUG_VERBOSE false`), on purpose: a hung loop keeps the last motor command (which in sumo often still finishes the push) instead of rebooting the robot. Bench builds do arm it at 250 ms — the full reasoning is at the top of `TAXI_Firmware.ino`.
- **Any reset during a round ends that round.** On the way back up the start module still reads HIGH, so `setup()` hits the boot lock and the robot blinks until someone power-cycles it. That is the same rule that catches a latched-on module before a match, so it is intentional — but it means a brownout or EMI reset has no in-match recovery path.
- `DEBUG_VERBOSE` builds are not timing-equivalent to match builds: the 400 ms serial table blocks the loop for ~50 ms per dump, sensor readings go correspondingly staler, and the watchdog is armed. Use them to observe, not to tune feel.
- All five ToF sensors are polled in every state, so a full sweep takes ~10 ms. If a future change needs faster front-sensor updates, the round-robin in `tofReadOne()` is the place to reintroduce a scan mode — but note that BACKOFF and the chase both depend on the side sensors being live.

---

## 14. License & Credits

- Project firmware: unspecified — confirm with author.
- XMotion V3 library and motor naming: **JSUMO**.
- `DistanceSensor.h` / ring display approach: adapted from the project's `Full_Robot_Test`/`ToF_Ring_Test` sketches and the SoBotz M60F register documentation.
- Third-party libraries: **FastLED** (Daniel Garcia et al.), **SingleWireSerial** (JSUMO community), Arduino AVR core.
