# Full Robot Test Suite

Comprehensive test sketch for the mini sumo robot covering all hardware functions.

## Hardware Configuration

| Pin | Function | Component |
|-----|----------|-----------|
| A0  | Digital Input | Start/Stop Module (U10) |
| A1  | Analog Input | QRE1113 Right Line Sensor (U9) |
| A2  | Digital Output | WS2812 7-LED Ring DIN (U11) |
| A4  | Analog Input | QRE1113 Rear Line Sensor (U8) |
| A5  | Analog Input | QRE1113 Left Line Sensor (U7) |
| D3  | PWM Output | MOT1 PWM |
| D11 | PWM Output | MOT2 PWM |
| D12 | Digital Output | MOT1 DIR |
| D13 | Digital Output | MOT2 DIR |
| D4  | Half-Duplex Serial | M60F ToF Sensors SIG Bus (U2-U6) |

## Required Libraries

Install via Arduino Library Manager:
- **FastLED** (for WS2812 ring)
- **SingleWireSerial** (for M60F ToF bus)

## Test Modes

Send commands via Serial Monitor (115200 baud):

| Key | Mode | Description |
|-----|------|-------------|
| `0` | Idle | Blinking center LED, motors stopped |
| `1` | ToF Test | Read all 5 ToF sensors, display on ring & serial |
| `2` | Line Test | Read 3 line sensors, display on ring & serial |
| `3` | Ring Test | Cycle through LED ring effects |
| `4` | Motor Test | Sequence through all motor movements |
| `5` | Start/Stop Test | Monitor Start/Stop module state |
| `6` | All Sensors | Combined ToF + Line + Start/Stop view |
| `7` | XMotion Test | Exercise XMotion library functions |
| `a` | Auto Mode | Toggle 5s auto-cycling (motors **skipped** by default) |
| `r` | Reset ToF | Restart all ToF sensors, verify each answers |
| `s` | Save ToF Config | Save settings, verify each ACKs |
| `h` | Help | Print command list |

## Auto Mode & Safety

Auto-cycle runs: ToF → Line → Ring → Start/Stop → All → XMotion. **The motor test
is excluded by default** (`AUTO_CYCLE_MOTORS false` in the sketch) so an
unattended desk robot can't drive off. Set the flag to `true` or press `4`
manually to run motors.

## ToF Sensor Details

- **IDs 1,2,4,5**: Report distance in **cm** (register 0x06)
- **ID 3 (middle)**: Reports distance in **mm** (register 0x01)
- **Range**: 9mm - 800mm (configured)
- **No response**: 0xFF (cm) or 0xFFF (mm)
- **Baud rate**: 115200 (must match sensor register 0xB0)

## Ring LED Mapping

```
        LED 4 (Front)
    LED 3         LED 5
LED 2               LED 6
        LED 0 (Center)
        LED 1 (Back)
```

**ToF → LED:**
- ID1 (Side Right)     → LED 6 (Right Back)
- ID2 (Right Front)    → LED 5 (Right Front)
- ID3 (Middle, mm)     → LED 4 (Front)
- ID4 (Front Left)     → LED 3 (Left Front)
- ID5 (Side Left)      → LED 2 (Left Back)

**Line → LED (actual physical layout: left / REAR / right):**
- Left (A5)   → LED 3 (left front)
- Rear (A4)   → LED 1 (back, 6 o'clock)
- Right (A1)  → LED 5 (right front)

Note: the stock `RingShowLine()` in `RingInfoDisplay.h` assumes a
left/center/right-front layout, so the sketch renders line state with its own
`renderLineRing()` to place the rear sensor on the back LED.

## Line Sensor Calibration

The QRE1113 sensors output:
- **Low values** (~100-300) = White/reflective surface
- **High values** (~700-900) = Black line/edge

Default threshold: 500. Adjust via `setThreshold()` or use `calibrate(3000)` while moving over both surfaces.

## Motor Control (XMotion Library)

**All motor output is capped at 15% of full power** via `MOTOR_MAX_PCT` in
`xmotionV3.cpp`:

```cpp
#define MOTOR_MAX_PCT 15   // percent of full power, applies to ALL motor calls
```

The cap is enforced inside the library, so both the percent API
(`Forward`, `Backward`, `Right0`, `Left0`, `ArcTurn`) and the raw PWM API
(`MotorControl`) are clamped. The motor test intentionally requests full power
so you can verify the cap is working (wheels should turn slowly).
Available xmotion calls:
- `Forward/Backward/Right0/Left0(speed, time)` - speed 0-100% (clamped to cap)
- `ArcTurn(leftSpeed, rightSpeed, time)` - differential drive
- `MotorControl(rightPWM, leftPWM)` - raw PWM, clamped to cap equivalent
- `StopMotors(time)`
- `Trimpot()` - Read trimpot (A3)
- `UserLed1/2(time)` - Blink user LEDs
- `ToggleLeds(time)` - Alternate LEDs
- `CounterLeds(time, count)` - Blink N times

## Usage

1. Open `Full_Robot_Test.ino` in Arduino IDE
2. Select board: **Arduino Leonardo** (or your XMotion board)
3. Install required libraries
4. Upload to robot
5. Open Serial Monitor at 115200 baud
6. Send commands to test each subsystem

## Files

- `Full_Robot_Test.ino` - Main test sketch
- `DistanceSensor.h` - M60F ToF sensor driver
- `RingInfoDisplay.h` - WS2812 7-LED ring display library
- `LineSensor.h` - QRE1113 line sensor driver
- `xmotionV3.h` / `xmotionV3.cpp` - XMotion V3 motor/IO library