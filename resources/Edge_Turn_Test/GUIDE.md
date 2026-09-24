# Edge_Turn_Test — quick guide

Autonomous tuner for the evade turn, started by the **Start/Stop module**. Battery
powered, no serial, no PC. Send START → drives forward → hits the white border →
flees + pivots → stops and holds so you can measure.

```
arduino-cli compile --fqbn arduino:avr:leonardo resources/Edge_Turn_Test
arduino-cli upload  --fqbn arduino:avr:leonardo -p COMx resources/Edge_Turn_Test
```

---

## Sequence

| Step | Behaviour |
|---|---|
| Power on | Armed, motors off, waiting for START. Place the robot in the middle |
| **START** | Drives forward at `CRUISE_PCT` until a line sensor sees white |
| Edge found | **FLEE** (reverse off the edge) then **TURN** (pivot away) for exactly `TURN_MS`. The ring lights the LED of the sensor that caught the line |
| Done | Stops and holds still → measure |
| **STOP** | Back to armed; START again for another run. No reset, no re-flash |

The module must be OFF at power-up (HIGH at boot = latched from before → STOP +
power-cycle). STOP cuts the motors instantly. Pin read raw, no debounce.

## LEDs (D8/D9) + ring

| Pattern | Meaning |
|---|---|
| Pulse once a second | Armed, waiting for START |
| Solid | Driving / fleeing / turning |
| 3 slow blinks | Finished → **measure now** |
| Fast blink | Run invalid: started on white, no line in 5 s, or flee could not clear the edge (send STOP, fix, START) |
| Both solid, no motion | Module latched at boot → STOP + power-cycle |
| Ring: one white LED | That sensor saw the line — front-left = LED 3, back = LED 1, front-right = LED 5. Stays lit until STOP |

## Knobs (top of the .ino, compile-time)

| Constant | Default | What it does |
|---|---|---|
| `TURN_MS` | 80 | **The knob** — pivot duration = rotation |
| `TURN_OUTER_PCT` / `TURN_INNER_PCT` | 70 / 50 | Pivot speeds |
| `FLEE_MS` / `FLEE_MAX_MS` | 15 / 300 | Flee floor / ceiling |
| `FLEE_SPEED_PCT` | −80 | Flee speed |
| `CRUISE_PCT` | `SEARCH_SPEED_PCT` (45) | Approach speed — matches the firmware's hunt speed |
| `CRUISE_MAX_MS` | 5000 | Give up if no line found |
| `SERIAL_LOG` | 0 | 1 = print runs to USB (tether only) |

## Tuning loop

1. Run once, measure the rotation: tape a mark on the floor along the starting
   heading + a matching mark on the chassis; the angle between them is the rotation.
2. `new TURN_MS = TURN_MS × (wanted ÷ measured)`.
3. Re-flash, START, repeat — 2–3 passes is usually enough.
4. If the result varies between runs, the pivot is slipping: drop the pivot speeds
   (e.g. 60/40) and raise `TURN_MS` by the same ratio.
5. Fast blink after the flee = fix that first (raise `FLEE_MS` in 20 ms steps); a
   pivot started over white measures nothing.

## Back into the firmware

| Here | `TAXI_Firmware/Config.h` |
|---|---|
| `TURN_MS` | `EVADE_TURN_MS` |
| `TURN_OUTER_PCT` / `TURN_INNER_PCT` | `EVADE_TURN_OUTER_PCT` / `EVADE_TURN_INNER_PCT` |
| `FLEE_MS` / `FLEE_MAX_MS` | `EVADE_ESCAPE_MIN_MS` / `EVADE_FLEE_MAX_MS` |
| `FLEE_SPEED_PCT` | `EVADE_REVERSE_SPEED_PCT` |

## Safety

- It drives off whatever it is on — first run: hold the robot with the wheels free.
- Keep clear of table edges and the blade; send STOP for anything unexpected.
