# TAXI — Timer / PWM Resource Map (ATmega32U4)

Why the left motor is driven from Timer4 instead of `analogWrite()`, and what
must not be changed without thinking about timers first.

> Applies to the XMotion V3 board, flashed as **Arduino Leonardo**
> (`arduino:avr:leonardo`, ATmega32U4 @ 16 MHz).

---

## 1. The short version

Motors need PWM to run at partial speed. PWM is produced by the chip's timers,
and each timer is permanently wired to specific pins. Two pieces of this robot
both needed the **same** timer:

- the **left motor speed pin (D10)** → Timer1
- the **ToF sensor bus (D4)** → also has to use Timer1

The sensor library won that fight at startup and reconfigured Timer1 for its
own bit timing. The left motor's speed pin then stopped producing any PWM, so
the motor received no speed command. The right motor was unaffected because it
sits on a different timer.

**Fix:** D10 is wired to *two* timers on this chip (Timer1 `OC1B` and Timer4
`OC4B`). It is now driven from **Timer4**, which nothing else uses. No rewiring,
no software PWM, no behaviour change anywhere else.

---

## 2. Which pin is on which timer

Arduino's own pin→timer table for the Leonardo (`variants/leonardo/pins_arduino.h`,
`digital_pin_to_timer_PGM[]`):

| Pin | Timer channel | Used in this firmware for |
|---|---|---|
| D3 | Timer0 `OC0B` | free |
| D5 | Timer3 `OC3A` | DIP switch 1 (plain digital input) |
| D6 | Timer4 `OC4D` | DIP switch 2 (plain digital input) — **left disconnected** |
| D9 | Timer1 `OC1A` | free — **but `analogWrite()` does not work here** |
| D10 | Timer1 `OC1B` / **Timer4 `OC4B`** | left motor PWM — **driven from Timer4** |
| D11 | Timer0 `OC0A` (**and** Timer1 `OC1C`) | right motor PWM — works because Timer1's output stays disconnected |
| D13 | Timer4 `OC4A` | right motor DIR (plain digital output) — **left disconnected** |

## 3. Who owns what

| Timer | Owner | Notes |
|---|---|---|
| Timer0 | Arduino core (`millis()`) + right motor PWM on D11 | Reconfigured by nothing else in this project. |
| Timer1 | `SingleWireSerial` on D4 (`ICP1`) | **Hijacked by the ToF bus.** Kills PWM on D9 and D10. |
| Timer3 | nobody | The core only pre-configures it for `analogWrite(5, …)`; D5 is a DIP input here. |
| Timer4 | **left motor PWM on D10 (`OC4B`)** | Channels A (D13) and D (D6) deliberately disconnected. |

---

## 4. The conflict, in detail

### 4.1 The sensor library has no choice

`SingleWireSerial` is a half-duplex software UART that measures incoming bits
with the timer's **input capture** unit. The ATmega32U4 has exactly one capture
pin, `ICP1` = **D4**, and only one timer can serve it. From the library's own
config header (`SingleWireSerial/src/SingleWireSerial_config.h`, `ICTIMER 1`):

```c
#define TCCRA TCCR1A
#define TCCRB TCCR1B
#define OCRA  OCR1A
#define ICR   ICR1
#define TIMSK TIMSK1
#define TIMER_CAPT_vect TIMER1_CAPT_vect
...
#define ICArduinoPin 4
```

### 4.2 Starting the bus rewires Timer1

`SingleWireSerial::begin()` (called from `sensorsInit()` in `Sensors.h`):

```c
TCCRA = 0;              // TCCR1A = 0
TCCRC = 0;              // TCCR1C = 0
setRxIntMsk(true);      // TCCR1B = ICNC1|prescaler ; TIMSK1 |= ICIE1
```

`TCCRC = 0` is what keeps the **right** motor alive. D11 is `PB7`, which is
multiplexed as `OC0A` (Timer0 — what the core uses for `analogWrite(11)`) *and*
`OC1C` (Timer1). On these dual-output pins both timer outputs reach the pin, so
if Timer1's `COM1C` were ever enabled, the right motor's PWM would be mixed with
a Timer1 compare output. The library zeroes `TCCR1C`, so `OC1C` stays
disconnected and D11 is safe — but it is safe only because of that line.

and every transmitted byte sets it up again for bit timing:

```c
TCCRA = 0;
TCCRB = _setCTC;        // WGM12 + prescaler  -> CTC mode, TOP = OCR1A
OCRA  = _bitDelay-1;
```

### 4.3 Why that kills the left motor

The Arduino core configures Timer1 **once**, in `wiring.c`:

```c
TCCR1B = 0; sbi(TCCR1B, CS11); sbi(TCCR1B, CS10);   // prescaler 64
sbi(TCCR1A, WGM10);                                 // 8-bit phase-correct PWM mode
```

`analogWrite()` only ever touches the compare bits — `wiring_analog.c`,
`case TIMER1B:` does `sbi(TCCR1A, COM1B1); OCR1B = val;`. It assumes the timer
is still in the PWM mode the core set up.

So after the bus starts, `analogWrite(10, x)` writes a duty value into a timer
that is no longer generating PWM and whose output pin has been disconnected
(`TCCR1A = 0` also clears `COM1B1`). Result: **no PWM on D10**. The DIR pin
still toggled, so the left motor was left uncontrolled rather than cleanly off,
which is why it twitched briefly and then sat still.

The right motor is on D11 = Timer0, which `SingleWireSerial` never touches —
hence "right motor seems fine" was the exact fingerprint of this bug.

---

## 5. The fix

D10 is `PB6`, and `PB6` is multiplexed as **`OC1B` (Timer1) *or* `OC4B`
(Timer4)** — ATmega32U4 datasheet, "Alternate Functions of Port B". Timer4 was
completely free, so the left motor now uses it:

```c
TCCR4A  = 0;                            // disconnect every Timer4 output first
TCCR4C  = 0;                            // channel D off -> D6 stays a plain input
digitalWrite(PIN_LM_PWM, LOW);          // park the pin low before handing it over
TC4H    = 0;                            // 10-bit registers: keep the two MSBs clear
OCR4C   = 255;                          // OCR4C holds the Fast PWM TOP value
TCCR4B  = _BV(CS42)|_BV(CS41)|_BV(CS40); // prescaler /64
TCCR4D  = 0;                            // WGM41:WGM40 = 00 -> Fast PWM, TOP = OCR4C
TCCR4A  = _BV(PWM4B) | _BV(COM4B1);     // connect OC4B, non-inverting (0x21)
```

- PWM frequency: 16 MHz / 64 / 256 = **977 Hz**, deliberately the same as the
  right channel's `analogWrite()` rate (Timer0 is also /64, for `millis()`).
  Matching matters: at part duty the slower channel has more current ripple and
  therefore makes less torque, so two different rates made the robot curve
  steadily towards the faster side. 977 Hz is also what the XMotion library's
  own `MotorControl()` produces on both motors.
- Resolution: **8-bit**. `TCCR4B` bit 7 (`PWM4X`) selects 8- vs 10-bit mode and
  the core leaves it 0, so 8-bit values are correct either way — and 8-bit is
  what `analogWrite()` would have given here anyway.
- Timer4's `OC4A` (D13) and `OC4D` (D6) are explicitly left disconnected, so
  D13 stays an ordinary output for `RM_DIR` and D6 stays an ordinary input for
  the DIP switch.
- Timer4 is clocked from `clkIO`. `PLLCSR` only defines `PLOCK` and `PLLE` on
  this chip (both belong to the USB PLL) — **do not touch `PLLCSR`.**

Duty is written as a 10-bit-safe register pair, which is the documented access
order for Timer4's `OCR4B`:

```c
TC4H  = 0;              // high bits first...
OCR4B = (uint8_t)duty;  // ...then the low byte
```

The whole thing lives in `Motor.h`; nothing above it changed, since
`motorsSet()` keeps its `-100..100` percent interface.

---

## 6. How this was verified

`SingleWireSerial`'s timer usage was read out of the library source (not
assumed), and the generated machine code was inspected to confirm the writes
actually happen:

```
arduino-cli compile --fqbn arduino:avr:leonardo --output-dir build_check TAXI_Firmware
avr-objdump -d build_check/TAXI_Firmware.ino.elf | grep -i "00bf\|00c0\|00c1\|00c2\|00c3\|00d0\|00d1"
```

Observed in `motorInit()`:

| Register | Value | Meaning |
|---|---|---|
| `TCCR4A` (0xC0) | `0x21` | `PWM4B` + `COM4B1` → `OC4B` connected, non-inverting |
| `TCCR4B` (0xC1) | `0x04` | prescaler /8 |
| `TCCR4C` (0xC2) | `0x00` | channel D disconnected (D6 unaffected) |
| `TCCR4D` (0xC3) | `0x00` | Fast PWM, TOP = `OCR4C` |
| `OCR4C` (0xD1) | `0xFF` | TOP = 255 |
| `TC4H` (0xBF) | `0x00` | high bits clear |

---

## 7. Rules for future work

- Never call `analogWrite(10, …)` — D10 is Timer4's channel B now. Use
  `motorWriteLeftDuty()` / `motorsSet()`.
- D9 cannot do `analogWrite()` either; Timer1 belongs to the ToF bus.
- **D11 is the same landmine one pin over.** It is `OC0A` (Timer0, what the core
  uses) *and* `OC1C` (Timer1). Today `SingleWireSerial` clears `TCCR1C`, so
  `OC1C` never drives the pin and the right motor works. If that ever changes —
  a different library version, or anything else enabling `COM1C` on Timer1 — the
  right motor breaks exactly the way the left one did before the Timer4 fix.
  Symptom to remember: one motor fine, the other uncontrolled.
- Timer4 is taken. D13 and D6 are therefore not available as PWM outputs.
- Timer3 is the only genuinely spare timer that has an output pin (D5) — and
  D5 is used as a DIP switch input.
- Any library added later that claims **Timer1** will break either the ToF bus
  or the left motor. Check `SingleWireSerial_config.h`-style config headers
  before adding soft-serial/`tone()`/`Servo`-type code.
- `millis()`/`micros()` depend on Timer0, which also carries the right motor's
  PWM. Nothing here may change Timer0's `WGM`/prescaler or both `millis()` and
  the right motor break together.

---

## 8. Related files

| File | Relevance |
|---|---|
| `TAXI_Firmware/Motor.h` | The fix; file header explains the whole story. |
| `TAXI_Firmware/Config.h` | `PIN_LM_PWM` / `PIN_TOF_BUS` and the comment on why D10 is Timer4-driven. |
| `TAXI_Firmware/Sensors.h` | `sensorsInit()` — where `SingleWireSerial::begin()` claims Timer1. |
| `TAXI_Firmware/README.md` | Wiring table (§3) and the matching entry in Known Limitations (§13). |
