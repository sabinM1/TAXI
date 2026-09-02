# XMotion V3 Sensor System: Complete Technical Reference
## Mini Sumo Robot — M60F ToF + QRE1113 Analog + Non-Blocking Architecture

---

## 1. Hardware Platform: JSumo XMotion V3

The XMotion V3 is an all-in-one Arduino-compatible robot controller built around the **ATmega32U4** (8-bit AVR, 16MHz, same core as the Arduino Leonardo/Micro). It integrates dual 6A motor drivers, a switching regulator accepting 7–24V input, 8 protected I/O pins, DIP switches, a trimpot, Bluetooth port, I2C port, optional encoder ports, and two user LEDs — eliminating the need for external motor driver boards in a competition mini sumo build.

### Pin Map — Relevant to Sensors

| XMotion Label | AVR Port | Interrupt      | Notes                            |
|---------------|----------|----------------|----------------------------------|
| **D0**        | PD2      | ✅ INT2        | Hardware RX — Serial1            |
| **D1**        | PD3      | ✅ INT3        | Hardware TX — Serial1            |
| **D2**        | PD1      | ✅ INT1        | SDA if I2C needed                |
| **D3**        | PD0      | ✅ INT0        | SCL if I2C needed                |
| **D4**        | PD4      | ❌ None        | General GPIO, poll only          |
| **D8**        | PB4      | ✅ PCINT4      | User LED 1 — avoid for sensors   |
| **A0**        | PF7      | ❌ None        | Analog ADC input                 |
| **A1**        | PF6      | ❌ None        | Analog ADC input                 |
| **A2**        | PF5      | ❌ None        | Analog ADC input                 |
| **A4**        | PF1      | ❌ None        | Analog ADC input                 |
| **A5**        | PF0      | ❌ None        | Analog ADC input                 |

**D0 and D1 are the hardware Serial1 UART pins** used for the M60F sensor bus. Port F pins (A0–A5) have no pin-change interrupt capability on the ATmega32U4 — they are analog ADC inputs only.

---

## 2. Sobotz M60F ToF Sensor

### Overview

The M60F is a purpose-built mini sumo Time-of-Flight distance sensor from SoBotz, measuring **7×7×9mm** — the smallest mini sumo distance sensor available. It operates at 940nm wavelength with a detection FOV of 18–22° and a maximum practical range of **up to 1000mm**. Communication is via a single open-drain SIG pin supporting three I/O modes: Serial (half-duplex UART), Digital 0/1, and PWM.

### Electrical Characteristics

| Parameter            | Min   | Typical | Max           |
|----------------------|-------|---------|---------------|
| Operating voltage    | 2.8V  | 5V      | 5.5V          |
| Supply current       | —     | 25mA    | 30mA          |
| Sampling delay       | 5ms   | 33ms    | 50ms          |
| Reliable sampling    | 10ms  | —       | —             |
| Start-up time        | 10ms  | —       | 50ms          |
| Max range            | —     | 1000mm  | 1200mm@50ms   |
| Wavelength           | —     | 940nm   | —             |
| Field of view        | 18°   | —       | 22°           |
| Dimensions           | —     | 7×7×9mm | —             |

### SIG Pin Behaviour

The SIG pin is open-drain — it can only pull LOW, never push HIGH. Therefore:

- A pull-up resistor to VCC is **mandatory** on every SIG bus (2.4kΩ–10kΩ)
- Use **2.4kΩ** for 250000 baud — lower resistance gives faster rising edges
- Multiple sensors can share one SIG bus wire, each addressed by its serial ID

---

## 3. M60F Operating Modes

### Mode 0 — Serial (Half-Duplex UART)

Full register access. All distance values and configuration available. Protocol is ASCII hex, newline-terminated:

```
Read:    R<id><reg>\n      →  <hex value>\n
Write:   W<id><reg><val>\n →  A\n (ok) or F\n (fail)
Save:    S<id>\n           →  A\n
Restart: U<id>\n           →  A\n
Reset:   Z<id>\n           →  A\n
```

### Full Register Map

| Reg  | Name                  | Default | R/W | Notes                                  |
|------|-----------------------|---------|-----|----------------------------------------|
| `00` | Ranging status        | —       | R   | Bit 0 = measurement valid              |
| `01` | Distance mm           | —       | R   | `FFF` = invalid/out of range           |
| `06` | Distance cm           | —       | R   | `FF` = invalid                         |
| `07` | Detection output      | —       | R   | Result of register BA logic            |
| `80` | IO mode               | 1       | W   | 0=Serial, 1=Digital, 2=PWM            |
| `81` | Serial ID             | 0       | W   | 0x00–0xFF, unique per sensor           |
| `82` | Serial baud           | 0       | W   | See baud table below                   |
| `B0` | Sampling time ms      | 33      | W   | Min 5ms; reliable operation ≥10ms     |
| `B4` | Range checks          | F8      | W   | Bitmask of validity checks             |
| `B5` | Signal threshold      | 1024    | W   | Min photon count for valid reading     |
| `B6` | Sigma threshold       | 15      | W   | Max std deviation (mm)                 |
| `B7` | Min distance mm       | 1       | W   | —                                      |
| `B8` | Max distance mm       | 500     | W   | —                                      |
| `BA` | Detection mode        | 0       | W   | 0=valid only, 1=>BC, 2=BC–BD          |
| `BB` | Output invert         | 0       | W   | 1 = flip digital output polarity       |
| `BC` | Lower threshold mm    | 0       | W   | Detection modes 1 & 2                  |
| `BD` | Upper threshold mm    | 0       | W   | Detection mode 2 only                  |

### Baud Rate Register `82`

| Value | Baud   | ATmega32U4 @ 16MHz Error | Use?              |
|-------|--------|--------------------------|-------------------|
| 0     | 9600   | 0.0%                     | ✅ Config/debug   |
| 5     | 115200 | −3.5%                    | ⚠️ Avoid          |
| 6     | 230400 | +8.5%                    | ❌ Never use      |
| **7** | **250000** | **0.0%**            | **✅ Use this**   |

250000 baud divides exactly from 16MHz (UBRR = 3), giving zero error. By contrast, 115200 has a well-known −3.5% error on AVR at 16MHz and can cause framing errors on longer messages. 230400 has +8.5% error and is unreliable.

### Mode 1 — Digital 0/1

SIG pin outputs HIGH or LOW based on the detection logic configured in register `BA`. No UART overhead — the MCU only reads a GPIO pin. Limitation: binary output only, no distance value. Best suited for side sensors where only opponent presence (not distance) is required.

### Mode 2 — PWM

20ms fixed period, 1ms–2ms pulse mapped to the configured min–max distance. Reading requires `pulseIn()`, which **blocks the CPU for up to 22ms per call**. Do not use in competition firmware.

---

## 4. M60F on ATmega32U4: Half-Duplex UART Wiring

The ATmega32U4 has full-duplex UART only (separate RX and TX pins). Since the M60F SIG is half-duplex, bridge D0 and D1 together with a **100Ω series resistor on TX** to prevent bus contention:

```
5V ──── 2.4kΩ ────┬──── SIG (Sensor 0, ID 0x00)
                   ├──── SIG (Sensor 1, ID 0x01)
                   ├──── SIG (Sensor 2, ID 0x02)
                   ├──── SIG (Sensor 3, ID 0x03)
                   ├──── SIG (Sensor 4, ID 0x04)
                   │
                   ├──── D0 / RX (PD2)  ← Serial1
                   └──[100Ω]── D1 / TX (PD3)  ← Serial1

GND ──── GND (all sensors)
5V  ──── VIN (all sensors)
```

One pull-up resistor for the entire shared bus is sufficient. Because TX is looped back to RX, the firmware must **discard the echo** of its own transmitted bytes as they arrive on RX.

---

## 5. Multi-Sensor Serial Bus: 5× M60F on One Wire

All five sensors share one SIG wire and are each assigned a unique serial ID (register `81`, values 0x00–0x04). Sensors measure continuously and independently in the background — only the serial read-out is sequential.

### One-Time Configuration (Per Sensor, Done Individually Before Joining the Bus)

If the sensor defaults to Digital mode, first force it into serial mode by pulling SIG LOW before applying power, then releasing after 50ms. The sensor will boot at 9600 baud with ID 0x00. Then configure:

```
W0082 7     → baud = 250000
W0081 NN    → unique ID: 00, 01, 02, 03, or 04
W00B0 A     → sampling time = 10ms (0x0A hex)
W0080 0     → IO mode = Serial
S00         → save configuration to flash
U00         → restart sensor
```

After this, the sensor permanently boots at 250000 baud with its assigned ID. No MCU intervention is required at power-up.

---

## 6. Non-Blocking Serial Read: Async State Machine

The provided `DistanceSensor.h` library calls `readBytesUntil('\n', ...)`, which **blocks the CPU for the entire UART transaction** — up to 10ms at 9600 baud. The correct approach is a round-robin async state machine that collects bytes as they arrive and never waits:

```cpp
#define NUM_SENSORS  5
#define BAUD_RATE    250000
#define TIMEOUT_US   2000       // 2ms safety timeout per request

volatile uint16_t distances[NUM_SENSORS]; // 0xFFF = invalid/no reading

static uint8_t  currentSensor = 0;
static char     rxBuf[8];
static uint8_t  rxLen       = 0;
static bool     waitingResp = false;
static uint32_t requestTime = 0;

void requestDistance(uint8_t id) {
    while (Serial1.available()) Serial1.read(); // flush stale RX/echo
    char buf[8];
    sprintf(buf, "R%02X01\n", id);              // read register 01 = mm
    Serial1.print(buf);

    // Discard TX echo from loopback bridge
    uint32_t t = micros();
    uint8_t echoLen = strlen(buf), got = 0;
    while (got < echoLen && micros() - t < 500)
        if (Serial1.available()) { Serial1.read(); got++; }

    rxLen = 0;
    waitingResp = true;
    requestTime = micros();
}

void updateSensors() {
    if (!waitingResp) {
        requestDistance(currentSensor);
        return;
    }
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n') {
            rxBuf[rxLen] = 0;
            distances[currentSensor] = (uint16_t)strtol(rxBuf, nullptr, 16);
            rxLen = 0;
            waitingResp = false;
            currentSensor = (currentSensor + 1) % NUM_SENSORS;
            requestDistance(currentSensor);     // chain immediately
            return;
        }
        if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
    }
    // Timeout guard — skip sensor if no response arrives
    if (micros() - requestTime > TIMEOUT_US) {
        distances[currentSensor] = 0xFFF;
        rxLen = 0; waitingResp = false;
        currentSensor = (currentSensor + 1) % NUM_SENSORS;
        requestDistance(currentSensor);
    }
}

void setup() {
    Serial1.begin(BAUD_RATE);
    delay(100);
    requestDistance(0); // kick off the async chain
}

void loop() {
    updateSensors(); // call every iteration — never blocks
    // distances[0..4] always contain the last valid reading in mm
    // 0xFFF = invalid or out of range
}
```

### Timing Budget @ 250kbaud, 5 Sensors

| Step                              | Time     |
|-----------------------------------|----------|
| TX `R0001\n` (6 chars)            | ~240µs   |
| Echo discard                      | ~240µs   |
| RX response (max 4 hex + `\n`)    | ~200µs   |
| **Per sensor total**              | **~680µs** |
| **All 5 sensors**                 | **~3.4ms** |

A complete 5-sensor sweep finishes in ~3.4ms against the 10ms sensor sample window — with zero CPU blocking.

---

## 7. QRE1113 Analog Module

### What It Is

The QRE1113 **analog** module outputs a **continuous analog voltage** on its signal pin, proportional to the intensity of IR light reflected from the surface below. Unlike the digital/capacitor-discharge version, it requires no timing and no special GPIO mode switching — the output is read directly with `analogRead()`.

The module contains:
- QRE1113 IR emitter + phototransistor pair
- A pull-up resistor on the signal pin
- 3-pin interface: **VCC, GND, OUT**
- Operating voltage: 5V (3.3V compatible)
- Optimal sensing distance: ~3mm from surface

### How the Analog Output Works

The phototransistor conducts more current when more IR light is reflected. With a pull-up resistor on the signal line, this creates a **voltage divider effect**:

- **High reflectance** (white surface / ring interior) → phototransistor conducts heavily → signal pin pulled LOW → **low ADC value** (close to 0)
- **Low reflectance** (black tape, edge, no surface) → phototransistor conducts less → signal pin pulled HIGH by resistor → **high ADC value** (close to 1023)

This is read with a single `analogRead()` call:

```cpp
int QRE1113_Pin = A0;

void setup() {
    Serial.begin(9600);
}

void loop() {
    int value = analogRead(QRE1113_Pin);
    // Lower value = more reflective (white/ring)
    // Higher value = less reflective (black/edge/no surface)
    Serial.println(value);
}
```

### ADC Characteristics on ATmega32U4

The ATmega32U4 ADC is 10-bit, returning values from **0 to 1023**. One important quirk: when switching ADC channels between reads, the first reading on a new channel can contain residual charge from the previous channel. To avoid this, **read each pin twice and discard the first result**:

```cpp
analogRead(pin); // discard — clears residual charge
int value = analogRead(pin); // use this value
```

This is especially important when multiplexing several QRE sensors on adjacent ADC pins in the same loop iteration.

### Fast Parallel Analog Reading on Port F

All analog pins on the XMotion V3 are on **Port F** (A0=PF7, A1=PF6, A2=PF5, A4=PF1, A5=PF0). Port F has **no pin-change interrupt capability** on the ATmega32U4. The analog approach, however, does not need interrupts — `analogRead()` completes in approximately **100µs** per channel (including ADC conversion at default 125kHz clock), which is far faster than the M60F's 10ms sample window.

For 4 QRE sensors read sequentially, total ADC time is **~400–800µs** (two reads per pin for stability), completing well within any reasonable loop period.

```cpp
// QRE sensors on A1, A2, A4, A5
const uint8_t qrePins[4] = {A1, A2, A4, A5};
uint16_t qreValues[4]    = {0};

void readQRE_all() {
    for (uint8_t i = 0; i < 4; i++) {
        analogRead(qrePins[i]);             // discard first (ADC mux settle)
        qreValues[i] = analogRead(qrePins[i]); // use second reading
    }
}

bool isEdge(uint8_t sensor) {
    return qreValues[sensor] > 700; // tune to your mat (0–1023 scale)
}
```

Call `readQRE_all()` once per main loop iteration. Since there is no timeout or blocking wait, the function completes in deterministic time every call.

### Calibration

The exact threshold between "ring surface" (white) and "edge/outside" (black or no surface) depends on:
- Surface reflectance of your specific mat
- IR LED current (determined by module resistor)
- Mounting height (optimal: 3mm)
- Ambient lighting conditions

A reliable calibration approach:
1. Print representative `qreValues[]` over Serial during a test run
2. Observe values on the white ring surface and on the black border/edge
3. Set your `isEdge()` threshold midway between the two clusters

Typical values on a standard mini sumo mat: **~150–300 on white**, **~700–900 on black/edge**. Values above 1000 generally indicate the sensor is too far from the surface or nothing is underneath.

---

## 8. Sensor Pin Assignment Summary

| Sensor     | Role         | Pin          | AVR    | Read Method           |
|------------|--------------|--------------|--------|-----------------------|
| M60F #0    | Front        | Shared bus   | PD2/3  | Serial1 async 250kbaud|
| M60F #1    | Front-Left   | Shared bus   | PD2/3  | Serial1 async 250kbaud|
| M60F #2    | Front-Right  | Shared bus   | PD2/3  | Serial1 async 250kbaud|
| M60F #3    | Left         | Shared bus   | PD2/3  | Serial1 async 250kbaud|
| M60F #4    | Right        | Shared bus   | PD2/3  | Serial1 async 250kbaud|
| QRE #0     | Edge FL      | A1           | PF6    | analogRead (×2)       |
| QRE #1     | Edge FR      | A2           | PF5    | analogRead (×2)       |
| QRE #2     | Edge RL      | A4           | PF1    | analogRead (×2)       |
| QRE #3     | Edge RR      | A5           | PF0    | analogRead (×2)       |

---

## 9. Complete Robot Loop Architecture

```
setup():
  ├─ Serial1.begin(250000)          → M60F bus init
  ├─ requestDistance(0)             → kick off async ToF chain
  └─ readQRE_all()                  → initial QRE baseline read

loop() [runs every ~500µs–1ms, never blocks]:
  ├─ updateSensors()                → M60F async state machine
  │     └─ distances[0..4] always current (mm, 0xFFF = invalid)
  ├─ readQRE_all()                  → ~800µs, sequential ADC reads
  │     └─ qreValues[0..3] always current (0–1023)
  └─ stateMachine():
        ├─ SEARCH    → no opponent in range, no edge
        ├─ ATTACK    → opponent in front (distances[0] < threshold), no edge
        ├─ EVADE     → edge detected (isEdge() true on any sensor)
        └─ FLANK     → opponent on side sensors only
```

Nothing in the loop uses `delay()`, `pulseIn()`, `readBytesUntil()`, or any blocking wait.

---

## 10. Mode and Method Comparison

| Sensor      | Method               | Blocking? | Update Rate       | Output             |
|-------------|----------------------|-----------|-------------------|--------------------|
| M60F ×5     | Serial 250k async    | ❌ Never  | ~10ms per sensor  | Distance in mm     |
| QRE1113 ×4  | analogRead (×2/pin)  | ❌ Never  | ~800µs for all 4  | 0–1023 ADC value   |

---

## 11. Does MCU Speed Matter?

For this sensor combination, the ATmega32U4 is not the bottleneck. The physical limits are:

- **M60F minimum sample period: 10ms** — a hardware physics constraint no MCU can change
- **QRE1113 analog ADC: ~100µs per channel** — fast enough that even a 16MHz AVR reads all 4 in under 1ms
- **Decision logic (comparisons, state machine): <5µs** on ATmega32U4

An ESP32S3 at 240MHz would execute the same logic ~15× faster but then idle waiting for the same sensor updates. The XMotion V3's ATmega32U4 is entirely adequate — the performance gain comes from **non-blocking sensor architecture**, not raw CPU frequency.
