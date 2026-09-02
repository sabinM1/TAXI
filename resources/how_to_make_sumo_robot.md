# How to Design & Make a Good Sumo Robot

*Source: [JSumo Blog – How to Make Sumo Robot](https://blog.jsumo.com/how-to-make-sumo-robot/)*
*Author: Firat Dede*

A sumo robot can be divided into three subsystems:

1. **Mechanic System** – Structure
2. **Electronic System** – Organs
3. **Software System** – Robot's spirit

---

## Mechanics

### DC Motors for Sumo Robots

- Used for the robot's drive; at least two motors are needed.
- High-quality brushed DC motors (e.g., **Faulhaber**, **Maxon**) perform best, though pricier.
- Suitable range: **50–150 W**, **12–24 V**.
- Recommended: **Maxon RE35 series** — lightweight with good torque and speed.
- Smaller gear modules give higher precision but less strength; bigger modules give more strength but less precision.

### Robot Gears

- Small motors often come with an integrated gearbox.
- Larger motors require a custom gear system.
  - Example 1: Fusion Sumo Robot — 7.2:1 reduction, 1 module gears.
  - Example 2: Another robot — 5:1 reduction, 0.6 module gears.
- **Gear module** = ratio between gear tooth number and circumference. Smaller module = lighter/smaller design.
- Suggested modules: **0.6 or 0.8** (0.8 eases metric/imperial pitch conversion).
- Suggested reduction ratio: **4:1 to 10:1**
  - Lower ratio → more speed
  - Higher ratio → more pushing force
- Design principle: keep the drivetrain simple — fewer gears mean less weight, less inertia, and higher response.

### Robot Wheels

- Wheel diameter: **35–55 mm**; smaller wheels preferred for lower inertia.
- Custom wheels should be made from hard rubber or polyurethane.
- Wheel rims should be machined from aluminum.

### Robot Body

- Common materials: Lexan PVC, aluminum, carbon fiber, or steel.
- Aluminum is preferred — easy to tap and machine.
- Recommended thickness: **5–6 mm**.
- Use high-strength aluminum alloys (JSumo uses **60XX and 70XX series**) to avoid overly soft material.

### Blades & Front Plates

| Material | Characteristics |
|---|---|
| Spring steel | Flexible, but wears out and needs frequent replacement |
| HSS (High-Speed Steel) | Very rigid and sharp, but more fragile under hard impacts |

- Thin blades: attached with strong double-sided tape (e.g., 3M).
- Thick blades: attached with M3 or M2.5 machine screws.
- Design blades to be easily changeable/replaceable.

---

## Electronics

### Robot Controllers (PIC, MBED, Arduino…)

- No major practical difference between microcontroller brands (Microchip, Atmel, STM32).
- MBED & Renesas microcontrollers are popular among Japanese teams.
- Author's favorites:
  - **PIC16F877A** — cheap, 33 I/O pins.
  - **Arduino Nano** — USB connectivity speeds up programming/debugging.

### Edge Sensors

- Detect the dohyo's edge via color contrast (white outer edge, black inner surface).
- Popular sensor models:
  - QRD1114
  - TCRT5000 (beware of common fakes)
  - QTR1A (based on QRE1113GR optocoupler — author's favorite)
  - ML1 (smallest design used)
- SMD sensors may falsely trigger on surface scratches.
- Placement: 4 corners for standard sumo; front edges suffice for mini sumo.

### Opponent Sensors

- Act as the robot's "eyes"; options include ultrasonic, infrared, or modulated visible light sensors.
- Infrared/modulated light preferred for faster response than ultrasonic.
- Use multiple sensors to avoid blind spots (author used 7 Pepperl+Fuchs sensors; one Romanian robot "Flex" used 11).
- Sensing range comparison (lowest to highest):
  - MZ80 – MR45 (+)
  - Pepperl+Fuchs MLT1000 Series (++)
  - Omron E3Z D62–D82 Series (+++)
  - Keyence PZG41 Series (++++, pricier but high quality)

### DC Motor Drivers

- Must match motor power requirements.
- Recommended brands: Roboclaw, Dimension Engineering.
- Selection tips:
  - Deliver at least **25A continuously**.
  - Support higher voltage than motor rating (e.g., a 24V motor needs 30–36V driver capability).
  - Easy to interface.
- MCU-based drivers (e.g., Sabertooth) introduce response delay — author prefers **custom-built MOSFET motor drivers**.
- L298-type drivers suitable only for non-magnetic (lightweight) sumo designs.

### Sumo Robot Programming

- Recommended languages:
  - MikroC (for PIC controllers)
  - Arduino Language (for Arduino-based boards)
  - PICBasic (beginner-friendly, PIC controllers)
- C-based languages are broadly similar and easy to learn.

### Voltage Regulators

- Critical for high-voltage systems (24V+).
- Switching-mode regulators (e.g., LM2596, LM2576) are more efficient than linear regulators.
- Trade-off: switching regulators generate more electrical noise — mitigate with additional capacitors and careful design.

### RF or IR Remote Control

- Needed for fast sumo robots that can't be stopped by hand.
- Custom-built IR transmitter/receiver circuits are commonly used.
- Note: many competitions prohibit RF remote controls.

### Cables, Wires & Connectors

- Use high-quality wires: thicker/soft wires for power, thinner wires for data/signal lines.
- Recommended connectors: **Deans (T-Plug)** or **XT60**.

---

## Summary Table

| Subsystem | Key Components |
|---|---|
| Mechanics | DC motors, gears, wheels, chassis, blades |
| Electronics | Microcontroller, edge sensors, opponent sensors, motor drivers, voltage regulators, remote control, wiring |
| Software | Firmware in C-based languages (MikroC, Arduino, PICBasic) |
