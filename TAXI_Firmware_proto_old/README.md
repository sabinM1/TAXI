# TAXI — Mini Sumo Robot Firmware

**Platform:** JSUMO XMotion V3 (ATmega32U4, Arduino Leonardo)  
**Class:** Mini Sumo (500g, 10cm × 10cm)  
**Author:** sabinM1@users.noreply.github.com  
**Date:** 2026-07-03

---

## Quick Start

1. Open `TAXI_Firmware.ino` in Arduino IDE
2. Select board: **Arduino Leonardo**
3. Select the correct COM port
4. Install required libraries:
   - **FastLED** by Daniel Garcia (for WS2812 LED ring)
   - **ICM20602** or generic MPU6050 library (for IMU)
   - **xmotionV3** (JSUMO — included in `lib/`)
5. Upload (`Ctrl+U`)

---

## Project Structure

```
TAXI_Firmware/
├── TAXI_Firmware.ino          # Main sketch — setup() and loop()
├── Config.h                    # ALL tunable constants, pin assignments, thresholds
├── Sensors.h                   # M60F ToF async state machine + QRE1113 analog reads
├── Motors.h                    # Motor control wrappers, ramp limiting
├── StateMachine.h              # Combat state machine (search/attack/evade/flank)
├── PIDController.h             # PD controller with gain scheduling
├── Tactics.h                   # DIP-switch selectable opening strategies
├── LEDRing.h                   # WS2812 LED ring patterns
├── IMU.h                       # ICM20602 gyro/accel positioning (experimental)
├── lib/
│   └── xmotionV3/              # JSUMO XMotion V3 library (local copy)
├── docs/
│   └── SYSTEM_ARCHITECTURE.md  # Full system documentation
├── tools/
│   └── PID_Tuner.html          # Standalone PID tuning tool
└── strategies/
    └── strategy_template.json  # Strategy export format for the GUI
```

---

## Pin Assignment (Final, Verified)

| Function | XMotion Label | AVR Port | Arduino Pin | Notes |
|----------|--------------|----------|-------------|-------|
| M60F Bus RX | D0 | PD2 | 0 | Serial1 RX — shared sensor bus |
| M60F Bus TX | D1 | PD3 | 1 | Serial1 TX — bridged to RX via 100Ω |
| ICM20602 SDA | D2 | PD1 | 2 | I²C data |
| ICM20602 SCL | D3 | PD0 | 3 | I²C clock |
| WS2812 LED Ring | D4 | PD4 | 4 | FastLED data pin |
| DIP Switch 1 | D5 | PC6 | 5 | Strategy bit 0 (LSB) |
| DIP Switch 2 | D6 | PD7 | 6 | Strategy bit 1 |
| DIP Switch 3 | D7 | PE6 | 7 | Strategy bit 2 (MSB) |
| User LED 1 | D8 | PB4 | 8 | Onboard — attack indicator |
| User LED 2 | D9 | PB5 | 9 | Onboard — sensor health |
| START-STOP Module | A0 | PF7 | A0 | Competition start signal |
| QRE Line FL | A1 | PF6 | A1 | Front-Left edge sensor |
| QRE Line FR | A2 | PF5 | A2 | Front-Right edge sensor |
| Trimpot | A3 | PF4 | A3 | Speed adjustment (0–1023) |
| QRE Line RL | A4 | PF1 | A4 | Rear-Left edge sensor |
| QRE Line RR | A5 | PF0 | A5 | Rear-Right edge sensor |
| LM_PWM | — | — | 10 | Left motor PWM (library) |
| RM_PWM | — | — | 11 | Right motor PWM (library) |
| LM_DIR | — | — | 12 | Left motor direction (library) |
| RM_DIR | — | — | 13 | Right motor direction (library) |

---

## Sensor Layout

```
          TOP VIEW — Robot Front (blade side) is UP
          ┌─────────────────────────┐
          │  ░░░░  BLADE  ░░░░      │
          │                         │
          │   M1 ◄──► M0 ◄──► M2   │  M0 = Front-Center (0°)
          │    -25°    0°    +25°   │  M1 = Front-Left (-25°)
          │                         │  M2 = Front-Right (+25°)
     M3 ◄─┤                   ├─► M4  M3 = Left side (90°)
          │   ████████████████ │      M4 = Right side (90°)
     Q0 ──┤                   ├── Q1  Q0 = QRE Front-Left
          │                   │      Q1 = QRE Front-Right
          │                   │      Q2 = QRE Rear-Left
     Q2 ──┤                   ├── Q3  Q3 = QRE Rear-Right
          │      WHEELS       │
          └─────────────────────────┘
```

---

## Competition Day Checklist

1. Charge all 5 TATTU LiHV batteries to 13.05V (LiHV mode)
2. Verify all 5 M60F sensors respond on power-up (LED ring indicates)
3. Calibrate QRE line thresholds (see `docs/SYSTEM_ARCHITECTURE.md`)
4. Inspect blade edge — replace if worn (10 spares)
5. Clean silicone tires with isopropyl alcohol
6. Verify DIP switch selects desired strategy
7. Set trimpot to mid-range (~512) for first match
8. Test START-STOP module on A0 with competition start signal
9. Ensure robot fits in 10cm×10cm inspection box
10. Verify weight is under 500g on competition scale
