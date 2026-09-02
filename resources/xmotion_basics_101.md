# XMotion Basics – XMotion 101

*Source: [JSumo Blog – XMotion Basics](https://blog.jsumo.com/xmotion-basics-xmotion-101/)*
*Author: Firat Dede*

## What Is XMotion?

XMotion is an **Arduino-compatible, all-in-one robot controller** designed for robotics, IoT, and maker projects. It integrates motor drivers, a switching-mode regulator, and interface circuits into a single board.

With XMotion you can build:

- Mini sumo robot (500 g, 2–4 motors)
- Midi sumo robot (1 kg, 2–4 motors)
- Line follower robot
- Obstacle avoiding / maze robot
- Mini explorer robot (up to 3 kg)
- Bluetooth-controlled robot
- Light arm robot

---

## 1. Getting Started

- XMotion has **8 spare I/O pins**: 4 digital, 4 analog (analog pins can also act as digital I/O).
- Connects via **micro USB**; the board is recognized as an **Arduino Leonardo**.
- In Arduino IDE, select the correct COM port and set board type to **Leonardo**.

## 2. First Code – Blink LED

XMotion has two onboard user LEDs: **D8 (Led1)** and **D9 (Led2)**.

```cpp
int UserLed1 = 9; // Led Pin is D9

void setup() {
  pinMode(UserLed1, OUTPUT);
}

void loop() {
  digitalWrite(UserLed1, HIGH);
  delay(500);
  digitalWrite(UserLed1, LOW);
  delay(500);
}
```

Upload with **Ctrl+U**. Minor USB interruptions during upload are normal.

## 3. Adding the XMotion Library

1. Download `xmotion.zip`.
2. In Arduino IDE: **Sketch → Include Library → Add ZIP Library**.
3. Restart Arduino IDE.
4. Examples appear under **File → Examples → XMotion**.

## 4. Toggling LEDs Example

Path: `File/Examples/XMotion/Basics/Blink_2_Leds`

```cpp
#include <xmotion.h>

void setup() {}

void loop() {
  xmotion.ToggleLeds(1000);
}
```

Both user LEDs toggle continuously at 1-second intervals.

## 5. Motor Control Functions Example

Connect two DC motors to the left/right green terminals, then run:

```cpp
#include <xmotion.h>

void setup() {
  xmotion.BlinkDelay(100); // Blink both LEDs every 100 ms
}

void loop() {
  xmotion.Forward(100, 100);        // 100% speed, both motors forward, 100ms
  xmotion.StopMotors(100);          // Stop both motors, 100ms
  xmotion.Backward(70, 250);        // 70% speed, both motors backward, 250ms
  xmotion.Right0(51, 1000);         // 51% speed, 1s right pivot turn
  xmotion.Left0(20, 3500);          // 20% speed, 3.5s left pivot turn
  xmotion.ArcTurn(20, 60, 250);     // 250ms, 20% left / 60% right motor
  xmotion.MotorControl(-150, 190);  // Timeless: -150/255 left, 190/255 right
  delay(100);                       // Delay for last MotorControl statement
}
```

## 6. XMotion Function Reference

**Basics:**

- `xmotion.UserLed1(time_ms);`
- `xmotion.UserLed2(time_ms);`
- `xmotion.ToggleLeds(time_ms);`
- `xmotion.CounterLeds(time_ms, int Multiplier);`
- `xmotion.Trimpot();`
- `xmotion.VoltageIn();`
- `xmotion.LipoCutOff(int Cell);`

**Motor Control:**

- `xmotion.Forward(%Percent, Time_ms);`
- `xmotion.StopMotors(Time_ms);`
- `xmotion.Backward(%Percent, Time_ms);`
- `xmotion.Right0(%Percent, Time_ms);`
- `xmotion.Left0(%Percent, Time_ms);`
- `xmotion.ArcTurn(%LeftPercent, %RightPercent, Time_ms);`
- `xmotion.MotorControl(LeftValue, RightValue);`

## 7. Built-in Button & DIP Switch

- Button is tied to **D10** (Start pin).
- 3-position DIP switch tied to **D5, D6, D7**.

```cpp
#define DipSwitch1 5
#define DipSwitch2 6
#define DipSwitch3 7
#define Start 10

void setup() {
  pinMode(DipSwitch1, INPUT);
  pinMode(DipSwitch2, INPUT);
  pinMode(DipSwitch3, INPUT);
  pinMode(Start, INPUT);
  digitalWrite(DipSwitch1, HIGH); // Pull-up
  digitalWrite(DipSwitch2, HIGH);
  digitalWrite(DipSwitch3, HIGH);
  Serial.begin(9600);
}

void loop() {
  Serial.print("Button State:");
  Serial.print(digitalRead(Start));
  Serial.print(" Dipswitch Inputs:");
  Serial.print(digitalRead(DipSwitch1));
  Serial.print(digitalRead(DipSwitch2));
  Serial.println(digitalRead(DipSwitch3));
  delay(100);
}
```

- Button reads logic 0 when not pressed.
- DIP switch inputs read logic 1 when set to the "down" position.
- Common usage patterns:
  - `if (digitalRead(DipSwitch1)==1) {Kp=0.30;} else {Kp=0.35;}`
  - `while(digitalRead(Start)==0) {MotorControl(0,0); digitalWrite(Led1, HIGH); digitalWrite(Led2, HIGH);}`

DIP switches are typically used for mini sumo starting-mode selection and on-board development.

## 8. Analog Input Elements — Trimpot & Voltage Sensor

- Onboard trimpot connected to **A3**.
- Voltage divider (battery sensor) connected to **A0**.

```cpp
#include <xmotion.h>

void setup() {}

void loop() {
  xmotion.BlinkDelay(xmotion.Trimpot()); // 0–1023 ms range
}
```

- `xmotion.Trimpot();` returns a value from 0–1023.
- `xmotion.VoltageIn();` reads input/battery voltage — useful for LiPo protection.

## 9. Motor Selection for XMotion

A motor is compatible if **stall current < 6 A** and **working voltage < 24 V** (e.g., a 12V/1A motor works fine; a 24V/5A motor does not).

Tested motor models:

- JSumo 15mm Core Series
- JSumo 16mm Profast Series
- JSumo 12mm All Series
- Fingertech 16mm Spark Series
- Pololu 12mm HP & normal series
- Pololu 20mm and 25mm Gearmotor series
- JSumo 25mm DC Gearmotor series
- JSumo Titan 37mm Series (HP variants work but aren't recommended due to high stall current)

If a motor draws excessive stall current at startup, the motor driver may shut off protectively — cycle the power switch to reset.

## 10. Battery Selection for XMotion

- Recommended: **2S to 4S LiPo batteries**, typically 350–1000 mAh at 3S.
- Voltage reference: 2S full charge = 8.4V (20% remaining ≈ 7.4V); 3S full charge = 12.6V (20% remaining ≈ 11.1V).
- Use `xmotion.VoltageIn();` and `xmotion.LipoCutOff(int Cell);` for low-voltage protection (triggers below 3.6V per cell).
- Battery voltage feeds directly into the motor driver — increasing voltage boosts speed/torque (common "overvolting" practice).
- Battery **capacity (mAh)** only affects total current available, not board safety — high-capacity batteries won't damage the board.

## 11. Choosing Sensors & Other Circuits

- Any sensor compatible with Arduino Uno/Leonardo works with XMotion.
- XMotion supplies **500 mA** via switching regulator (vs. Uno's 200 mA linear regulator), enough for servos.
- I/O terminals output 5V; sensors requiring other voltages must connect directly to the battery input.
- MCU (ATmega32u4) inputs are protected with 220-ohm resistors.
- Compatible add-ons: ultrasonic sensors, RF modules, Bluetooth modules, extra motor drivers, breakout boards.

## 12. Soldering Parts to XMotion

- Solder pads on the underside use standard **0.1 inch (2.54 mm)** spacing.
- For swappable setups, solder male headers and use female jumper cables.
- Strip only **1 mm of insulation** to avoid short circuits.
- Always verify sensor polarity before soldering.

## 13. Developing Further Projects

- Use the trimpot to control speed on a line follower robot.
- Use the 3-position DIP switch for up to 8 starting-mode combinations on mini sumo robots.
- Attach a Bluetooth module to the bottom pins for phone-based debug/control.
- Use the two user LEDs for sensor testing and debug routines.
- Use the voltage divider input to correlate robot speed with battery voltage.
- Start the robot via the built-in button or an external start module.

**Author's advice:** build up incrementally — test LEDs, then motor control, then conditional logic — before combining everything into a full robot program.
