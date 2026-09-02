# TAXI — Mini Sumo Robot: System Architecture

**Version:** 1.0  
**Date:** 2026-07-03  
**Platform:** JSUMO XMotion V3 (ATmega32U4 @ 16MHz)  
**Class:** Mini Sumo — 500g max, 10cm × 10cm footprint, 77cm dohyo

---

## 1. Hardware Overview

### 1.1 Controller Board

**JSUMO XMotion V3** — All-in-one Arduino Leonardo-compatible board integrating:

| Subsystem | Specification |
|-----------|--------------|
| MCU | ATmega32U4 @ 16MHz, 32KB Flash, 2.5KB SRAM |
| Motor drivers | 2× 6A H-bridge (LM: DIR=12, PWM=10; RM: DIR=13, PWM=11) |
| Voltage regulator | Switching, 7–24V input, 1.6A @ 5V output |
| User I/O | 8 protected pins (D0–D4, A0–A5), DIP switches (D5–D7), Trimpot (A3) |
| LEDs | User LED1 = D8, User LED2 = D9 |
| USB | Micro USB, appears as Arduino Leonardo COM port |

### 1.2 Sensors

#### M60F ToF Distance Sensors (5× SoBotz)
- **Type:** 940nm Time-of-Flight laser ranging
- **Range:** 5mm–600mm (configurable to 1000mm)
- **FOV:** 18–22° cone
- **Dimensions:** 7×7×9mm (smallest mini sumo sensor available)
- **Sample rate:** 10ms minimum reliable sampling period
- **Communication:** Half-duplex UART, ASCII hex protocol
- **Bus topology:** 5 sensors share one SIG wire (open-drain) with 2.4kΩ pull-up to 5V
- **Baud rate:** 250,000 bps (0% error on ATmega32U4 @ 16MHz: UBRR = 3)
- **Placement:**
  - M0: Front-Center, facing 0° (straight forward)
  - M1: Front-Left, angled 25° left of center
  - M2: Front-Right, angled 25° right of center
  - M3: Left side, facing 90° left
  - M4: Right side, facing 90° right

#### QRE1113 Analog Line Sensors (4×)
- **Type:** IR reflective optocoupler (IR LED + phototransistor)
- **Output:** Analog voltage 0–5V → ADC 0–1023
- **Principle:** LOWER value = MORE reflective (white ring), HIGHER = darker (black edge)
- **Typical values:** 150–300 on white ring, 700–900 on black edge
- **Placement:** Four corners of chassis underside
  - Q0: Front-Left corner (A1)
  - Q1: Front-Right corner (A2)
  - Q2: Rear-Left corner (A4)
  - Q3: Rear-Right corner (A5)

#### ICM20602 6-Axis IMU
- **Type:** 3-axis gyroscope + 3-axis accelerometer, I²C
- **Address:** 0x68 (AD0 = GND) or 0x69 (AD0 = VCC)
- **Purpose:** Gyro-based turn control, acceleration-based edge/push detection, experimental dead-reckoning positioning
- **I²C pins:** SDA = D2, SCL = D3 (no conflict with Serial1 on D0/D1 — different AVR ports)

#### START-STOP NanoLabs Module
- **Pin:** A0 (per user requirement — uses the onboard header)
- **Function:** Detects official competition IR start signal
- **Output:** HIGH when start signal is detected

### 1.3 Actuators

#### JSUMO Core 6V Motors (2× 400 RPM)
- **Stall current:** 3.2A @ 6V (within XMotion's 6A/channel limit)
- **Shaft:** 3mm D-shaft, 8mm exposed
- **Overvolting:** Run at 11.4V nominal (3S LiHV) for ~700+ RPM effective
- **Gear ratio:** 48:1

#### JS2622S Steel-Silicone Wheels
- **Diameter:** 26.5mm
- **Width:** 22mm (19mm silicone contact)
- **Weight:** 25g each (steel hub — adds useful low ballast)
- **Hardness:** 20 Shore A (very soft, maximum dohyo grip)
- **Theoretical speed @ 400RPM:** ~555 mm/s (≈ dohyo diameter in 1.4s)
- **Theoretical speed @ 700RPM (overvolted):** ~970 mm/s

### 1.4 Outputs

#### WS2812 RGB LED Ring (7-bit)
- **Pin:** D4
- **Library:** FastLED
- **Patterns:** Direction indicator, sensor health, state display

#### Kanzawa ShadowEdge Blade
- **Material:** Hardened steel, black finish
- **Dimensions:** 100mm × 15mm × 0.25mm
- **Mounting:** 3M 9448A double-sided tape + reference wall alignment + height adjustment screw

### 1.5 Power

- **Battery:** TATTU 11.4V 3S LiHV, 300mAh, 75C
- **Connector:** XT30
- **Full charge:** 13.05V (4.35V/cell)
- **Runtime estimate:** ~7.5 min (300mAh / ~2.4A typical draw)

---

## 2. Non-Blocking Sensor Architecture

### 2.1 M60F Half-Duplex UART Wiring

```
                    5V
                     │
                    [2.4kΩ]   ← Single pull-up for entire bus
                     │
    ┌────────────────┼────────────────┬────────────┬────────────┬────────────┐
    │                │                │            │            │            │
  [M60F #0]      [M60F #1]       [M60F #2]     [M60F #3]    [M60F #4]
  ID=0x00        ID=0x01         ID=0x02       ID=0x03      ID=0x04
    │                │                │            │            │            │
    └────────────────┼────────────────┴────────────┴────────────┴────────────┘
                     │
              ┌──────┴──────┐
              │             │
           D0/RX        [100Ω]
         (Serial1)         │
                        D1/TX
                      (Serial1)
```

- **All 5 sensors share one SIG wire** connected to both D0 (RX) and D1 (TX)
- **100Ω resistor on TX** prevents bus contention when TX drives against a sensor pulling SIG low
- **2.4kΩ pull-up to 5V** provides fast rising edges at 250k baud
- **TX echo is discarded** in firmware — every transmitted byte appears on RX due to the bridge

### 2.2 Async Serial State Machine

The sensor sweep uses a cooperative round-robin state machine that NEVER blocks:

```
State: IDLE → select sensor N → transmit "RNN01\n" → discard echo →
State: WAITING → collect RX bytes until '\n' or 2ms timeout →
  ├─ Got response → parse hex → store distance[N] → N++ → IDLE
  └─ Timeout → distance[N] = 0xFFF → N++ → IDLE
```

**Timing budget @ 250kbaud (5 sensors):**

| Step | Time |
|------|------|
| TX "R0001\n" (6 bytes) | ~240 µs |
| Echo discard (6 bytes) | ~240 µs |
| RX response (≤5 bytes) | ~200 µs |
| **Per sensor total** | **~680 µs** |
| **Full 5-sensor sweep** | **~3.4 ms** |

Against the M60F's 10ms sampling window, this is 3.4ms polling vs 10ms update — zero data loss.

### 2.3 QRE1113 Analog Fast Read

Each QRE sensor is read with the double-read pattern required by ATmega32U4's ADC:

```
analogRead(pin);       // discard — ADC mux settling
value = analogRead(pin);  // use this reading
```

All 4 sensors read sequentially: ~100µs × 2 reads × 4 sensors = **~800 µs total**.

---

## 3. State Machine Design

### 3.1 States

```
                    ┌─────────┐
          ┌────────►│  IDLE   │◄────────┐
          │         └────┬─────┘         │
          │              │ Start signal  │ Stop button
          │         ┌────▼─────┐         │
          │    ┌───►│  TACTIC  │───┐     │
          │    │    └────┬─────┘   │     │
          │    │         │ Tactic │     │
          │    │         │  done  │     │
          │    │    ┌────▼─────┐   │     │
          │    │    │  SEARCH  │◄──┘     │
          │    │    └──┬───┬───┘         │
          │    │   Opp │   │ No opp      │
          │    │  seen │   │ + timeout   │
          │    │  ┌────▼──┐│             │
          │    │  │ATTACK ││             │
          │    │  └───┬───┘│             │
          │    │ Edge │    │             │
          │    │  ┌───▼────▼──┐          │
          │    └──┤   EVADE   │          │
          │       └─────┬─────┘          │
          │             │ Evasion        │
          │             │ complete       │
          └─────────────┘                │
                    ┌────────────────────┘
                    │  (Power off / stop)
                    ▼
                 ┌──────┐
                 │ STOP │
                 └──────┘
```

### 3.2 State Descriptions

| State | Trigger | Behavior | Exit Condition |
|-------|---------|----------|----------------|
| **IDLE** | Power-on | Sensor health check, LED ring startup sequence, wait for DIP read | START-STOP signal on A0 |
| **TACTIC** | Start signal | Execute DIP-switch-selected opening pattern (zigzag, dash, spin, etc.) | Tactic timer expires |
| **SEARCH** | Tactic complete | Sweep-search with distance-adaptive arc width. Uses last-known-position memory. | Any M60F detects opponent within threshold |
| **ATTACK** | Opponent detected | PD-controlled tracking. Hybrid gain scheduling: aggressive Kp=35 lock-on (200ms) → smooth Kp=28 tracking | All M60F lose opponent for >500ms, OR edge detected |
| **EVADE** | QRE line sensor triggered | Curved re-entry path based on which corner sensor triggered. Front-left → arc right, Front-right → arc left, Both → reverse + 180° pivot, Rear corners → drive forward at angle | Evasion path complete, sensors clear |
| **STOP** | Power-off or critical failure | Motors off, LED ring red | Power cycle |

### 3.3 Priority Chain

Within the main loop, states are evaluated in priority order:

1. **EVADE** (highest) — Safety-critical. Any QRE sensor above threshold triggers immediate response, overriding all other states.
2. **ATTACK** — Any M60F front sensor detects opponent within configured range.
3. **FLANK** (sub-state of ATTACK) — Side M60F sensors detect opponent but front sensors are clear.
4. **SEARCH** — Default state when no opponent and no edge.

---

## 4. PID Controller Design

### 4.1 Architecture

**Type:** PD (Proportional-Derivative) — no integral term to prevent windup when opponent disappears.

**Input:** Weighted error from 5 M60F distance sensors, using inverse-distance weighting.

```
error = Σ( weight[sensor] × direction[sensor] × (1.0 / distance[sensor]) )
```

Where:
- `direction[] = {-4, -2, 0, +2, +4}` (left-to-right spatial weights)
- `weight[]` is dynamically adjusted for anti-flag protection
- Distances in mm, clamped to [20, 600]; 0xFFF = invalid → sensor excluded

**Output:** Differential motor speeds via `MotorControl()`.

### 4.2 Hybrid Gain Scheduling

| Phase | Duration | Kp | Kd | Behavior |
|-------|----------|----|----|----------|
| **Lock-on** | 0–200ms after first detection | 35.0 | 8.0 | Aggressive centering — snaps to opponent |
| **Tracking** | 200ms+ sustained contact | 28.5 | 15.0 | Smooth tracking — minimal oscillation |

Gain transition is linear over 50ms between phases:
```
Kp = 35.0 - (7.0 × min(elapsed_since_detect / 250.0, 1.0))
Kd =  8.0 + (7.0 × min(elapsed_since_detect / 250.0, 1.0))
```

### 4.3 Anti-Flag Protection

When front sensors (M0, M1, M2) detect opponent, outer sensor weights are reduced:
- Normal: outer weight = 5.0, inner weight = 2.0
- Anti-flag: outer weight = 1.5, inner weight = 2.0

This prevents the robot from spinning 90° when it sees the opponent's broad front as a "far left + far right" signal.

### 4.4 Distance-Gated Attack Zones

| Zone | Distance | Behavior |
|------|----------|----------|
| **Red zone** | < 100mm | Full attack speed (100% base) |
| **Yellow zone** | 100–300mm | Approach speed (70% base) |
| **Green zone** | 300–500mm | Cautious approach (40% base) |
| **Out of range** | > 500mm or 0xFFF | No opponent — transition to SEARCH |

---

## 5. IMU Integration (ICM20602)

### 5.1 Use Cases

1. **Turn rate control:** Gyro Z-axis provides yaw rate. During EVADE, the curved re-entry can be executed as "turn at 200°/s until heading = target" instead of timed arcs. This is immune to battery voltage affecting motor speed.

2. **Edge/push detection:** Accelerometer X/Y axes detect sudden lateral acceleration (being pushed by opponent). If acceleration exceeds threshold while not actively driving sideways, robot applies counter-torque to resist being pushed out.

3. **Dead-reckoning positioning (experimental):** Integrate gyro Z for heading, integrate accel X/Y (double-integrated) for position estimate. Useful for "return to center" during SEARCH and for the HTML strategy GUI's path programming feature.

### 5.2 I²C Wiring

```
ICM20602          XMotion V3
  VCC  ────────── 5V
  GND  ────────── GND
  SDA  ────────── D2 (PD1)
  SCL  ────────── D3 (PD0)
  AD0  ────────── GND (address 0x68)
```

---

## 6. Strategy System

### 6.1 DIP Switch Configuration (D5–D7, 8 strategies)

| D3 | D2 | D1 | Strategy |
|----|----|----|----------|
| 0 | 0 | 0 | Strategy 0: Immediate forward rush (reactive — Option A) |
| 0 | 0 | 1 | Strategy 1: 90° right turn → forward rush |
| 0 | 1 | 0 | Strategy 2: 90° left turn → forward rush |
| 0 | 1 | 1 | Strategy 3: Zigzag right-to-left cross → search |
| 1 | 0 | 0 | Strategy 4: Zigzag left-to-right cross → search |
| 1 | 0 | 1 | Strategy 5: 180° spin → forward (for rear-position starts) |
| 1 | 1 | 0 | Strategy 6: Wait-200ms-then-rush (deceptive timing) |
| 1 | 1 | 1 | Strategy 7: Curve-to-right sweep (aggressive search, Option B) |

**Note:** DIP switches read 1 when in DOWN position (internal pull-up active).

### 6.2 Strategy Definition Format

Strategies are defined as JSON for the HTML strategy GUI. Each strategy contains:
- Opening sequence (series of motor commands with timing)
- Search behavior parameters (arc width, speed, timeout)
- Attack parameters (PID gains, distance zones)
- Edge behavior parameters (evasion curve radius, reverse duration)

The HTML GUI exports a `strategies.h` file that can be directly included in the firmware.

---

## 7. LED Ring Patterns

| Pattern | Meaning |
|---------|---------|
| Green sweep clockwise | Sensor calibration in progress |
| All green solid | All sensors OK, waiting for start |
| Blue pulsing | SEARCH mode |
| Red rapid flash | ATTACK mode — opponent locked |
| Yellow left-half | Opponent detected on LEFT sensors |
| Yellow right-half | Opponent detected on RIGHT sensors |
| White flash + red | EVADE — edge detected |
| Single red LED (position) | Indicates which sensor failed health check |
| Rainbow cycle | Post-match celebration / idle |

---

## 8. Calibration Procedures

### 8.1 QRE1113 Line Sensor Calibration

1. Place robot on dohyo, white ring surface
2. Power on with DIP set to all-UP (strategy 7 enters calibration mode)
3. Robot rotates in place for 2 seconds, recording min/max analog values for each QRE
4. Threshold = min + (max - min) × 0.5 (midpoint between white and black)
5. Values stored in EEPROM for persistence
6. LED ring shows green on success, red on failure (insufficient contrast)

### 8.2 M60F Sensor Verification

1. On boot, firmware queries all 5 M60F sensors
2. Each sensor must respond within 500ms
3. LED ring indicates: position 0 = 12-o'clock LED, position 1 = 2-o'clock, etc.
4. Green LED = sensor OK, Red LED = sensor failed
5. If any sensor fails, robot enters degraded mode (see Section 10)

---

## 9. Edge Evasion — Curved Re-entry

Based on user's specific design request ("curbă în loc de linie dreaptă"):

| Trigger | Response |
|---------|----------|
| Front-Left QRE (Q0) | Reverse 80ms → ArcTurn right-forward (L=70%, R=20%) for 300ms |
| Front-Right QRE (Q1) | Reverse 80ms → ArcTurn left-forward (L=20%, R=70%) for 300ms |
| Both Front QREs | Reverse 150ms → 180° pivot right → resume SEARCH |
| Rear-Left QRE (Q2) | Drive forward at 45° right angle for 200ms |
| Rear-Right QRE (Q3) | Drive forward at 45° left angle for 200ms |
| Both Rear QREs | Full forward 100% for 300ms → resume SEARCH |

The curved path uses `MotorControl()` with blended speeds (no `delay()`) so the main loop continues running sensor checks during evasion. If a front sensor detects opponent mid-evasion, the robot transitions to ATTACK immediately.

---

## 10. Failsafe & Degraded Operation

### 10.1 Sensor Health Tracking

Each M60F sensor maintains a health counter:
- Successful read: counter = min(counter + 1, 10)
- Timeout/failure: counter = max(counter - 2, 0)
- `counter < 3` → sensor flagged as failed

### 10.2 Degradation Levels

| Level | Condition | Behavior |
|-------|-----------|----------|
| **Full** | All sensors healthy | Normal operation |
| **Degraded-1** | 1 M60F failed | Exclude sensor from PID calculation, redistribute weights. LED ring shows yellow at failed position. |
| **Degraded-2** | 2+ M60F failed OR 1 QRE failed | Enter SAFE_SEARCH mode: slow forward-only (30% speed), aggressive edge checking. |
| **Critical** | All front M60F failed OR 2+ QRE failed | STOP all motors. LED ring solid red. Requires power cycle. |

### 10.3 Timeout Limits

- Maximum degraded operation: 10 seconds, then STOP
- M60F response timeout: 2ms per poll
- QRE out-of-range timeout (value stuck at 0 or 1023 for >500ms): flag as failed

---

## 11. Missing Parts — Complete Requisition List

Based on BOM validation (Phase 2), the following items are needed to complete the build:

| # | Item | Quantity | Purpose | Approx. Cost |
|---|------|----------|---------|-------------|
| 1 | **2.4kΩ resistor** (1/4W, through-hole) | 1 | Pull-up for M60F SIG bus — REQUIRED. Without this, the open-drain bus cannot rise to logic HIGH. Solder between 5V and SIG bus junction. | <$0.10 |
| 2 | **100Ω resistor** (1/4W, through-hole) | 1 | TX-RX loopback bridge for half-duplex UART — REQUIRED. Solder between D1 (TX) and SIG bus junction. Limits current when TX and sensor both drive the line. | <$0.10 |
| 3 | **XT30 male connector** (battery side) | 3 | Solder to TATTU battery leads. Male on battery, female on XMotion V3 power input. Buy extras for spare batteries. | ~$1 each |
| 4 | **XT30 female connector** (board side) | 2 | Solder to XMotion V3 power input terminals. One installed, one spare. | ~$1 each |
| 5 | **Mini slide switch** (3A+, SPDT) | 1 | Power on/off switch in line between battery and XMotion V3. Solder in series with positive battery lead. Rated for at least 3A to handle stall current. | ~$2 |
| 6 | **M3 stainless steel hex nuts** | 50 | For through-hole fastening where heat-set inserts aren't practical (motor mounts, aluminum plate). | ~$3 |
| 7 | **M3 stainless steel flat washers** | 50 | Distribute screw pressure on 3D printed parts and aluminum plate. Prevents cracking PC filament. | ~$3 |
| 8 | **Female header pin strip** (2.54mm, 1×40) | 2 | Solder to XMotion V3 I/O pins for removable sensor connections. Allows sensor replacement without desoldering. | ~$2 |
| 9 | **Heat shrink tubing kit** (assorted, 2:1) | 1 kit | Insulate all solder joints — especially the M60F bus junction and battery connector. Prevents shorts in tight chassis. | ~$5 |
| 10 | **Flux pen** (no-clean) | 1 | Apply before soldering lead strips to perfboard or wire joints. Ensures proper wetting with lead-based solder. | ~$4 |
| 11 | **Nylon M3 standoffs** (6mm, 10mm, 15mm) | 4 each | Mount XMotion V3 to chassis at correct height. 10mm recommended for clearing sensor wiring underneath. | ~$5 |
| 12 | **Velcro battery strap** (15×150mm) | 2 | Secure TATTU LiHV pack inside chassis. Allows quick battery swaps between rounds. | ~$3 |
| 13 | **Small perfboard** (~30×50mm, 2.54mm pitch) | 1 | Solder the M60F bus junction (5 sensor wires + 2.4kΩ + 100Ω + D0 + D1). Keeps wiring clean and strain-relieved. | ~$1 |

**Total estimated additional cost: ~$30–35**

---

## 12. Known Limitations

1. **ATmega32U4 ADC quirk:** First `analogRead()` after channel switch returns residual charge. Mitigated by double-read pattern.
2. **No I-term in PID:** Intentional — prevents windup when opponent disappears. If opponent is stationary and resisting push, the robot won't ramp up force. Acceptable trade-off for mini sumo's dynamic engagements.
3. **Dead-reckoning drift:** ICM20602 position estimate will drift ~10–30cm over a 30-second match without external reference. Useful for short-term path following only.
4. **M60F minimum range:** 5mm minimum. Blade-mounted opponent at contact range may read as 0xFFF (invalid). Handled by last-known-position memory.
5. **Overvolted motor lifespan:** Running 6V motors at 13V reduces brush life. With 5 batteries in rotation, this is acceptable for competition use. Expect to replace motors after 50–100 full-power matches.
6. **Steel wheel inertia:** JS2622S steel hubs (25g) provide excellent traction and low CoG but increase rotational inertia. Direction changes require more current. The XMotion's 6A/channel driver handles this with margin.

---

## 13. Future Improvement Ideas

1. **HTML Strategy GUI:** Standalone tool to visually program paths and export `strategies.h` (per user requirement Q11). Already planned.
2. **PID Tuning Tool:** Companion program that guides through one-parameter-at-a-time tuning with real-time Serial feedback.
3. **EEPROM configuration:** Store calibrated QRE thresholds, strategy parameters, and PID gains in EEPROM for persistence across power cycles.
4. **Bluetooth telemetry:** HC-05 module on D10 (freed pin) for wireless debug output during testing.
5. **6th M60F sensor:** Add rear-facing sensor if flank ambushes become a problem in competition.
6. **Current sensing:** Monitor motor current draw to detect stall conditions and adjust PID output accordingly.
7. **Dual IMU fusion:** Add a second IMU for vibration rejection and more accurate yaw estimation.
