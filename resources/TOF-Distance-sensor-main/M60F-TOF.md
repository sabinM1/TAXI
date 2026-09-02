# M60F TOF Distance Sensor — Datasheet

## Device Pinout

![Sensor pinout diagram: a small rectangular sensor module with a 3-pin header labeled VIN (red wire), GND (black wire), and SIG (white wire), plus a photo of the physical module showing the lens window.]

| Pin | Function |
|-----|----------|
| VIN | Connect to power supply |
| GND | Connect to main ground |
| SIG | Sensor communication, open-drain input/output |

## Typical Application

The sensor operates from 2.8V to 5.5V, and the power supply must handle up to 30mA.

![Typical application circuit: the sensor block (labeled SENSOR) with VIN, GND, and SIG pins. SIG connects through a pull-up resistor R1 to VREF, and also connects onward to MCU_PIN. VIN is tied to +5V.]

The SIG pin is open-drain, so a pull-up resistor (typically 2.4k–10k) is required — lower resistance increases power loss but supports higher serial speeds, and the pull-up voltage should be >= 2.8V (usually tied to the MCU supply).

In serial mode, SIG acts as a half-duplex UART line. Since some MCUs (like AVR) only support full-duplex UART, you can either tie RX and TX together or use software serial; USB-to-UART adapters should also have RX and TX bridged.

### Electrical Characteristics

| Parameter | Conditions | Min | Typical | Max | Unit |
|-----------|-----------|-----|---------|-----|------|
| Maximum allowed voltage | — | -0.3 | — | 7 | V |
| Operating voltage | At 25°C | 2.8 | 5 | 5.5 | V |
| Supply current | — | — | 25 | 30 | mA |

### Sensor Characteristics

| Parameter | Conditions | Min | Typical | Max | Unit |
|-----------|-----------|-----|---------|-----|------|
| Sampling delay | — | 5 | 33 | 50 | ms |
| Maximum range | Sampling time is 50ms | — | 1200 | — | mm |
| Wavelength | — | — | 940 | — | nm |
| Detection field of view | — | 18 | — | 22 | ° |
| Start-up time | — | 10 | — | 50 | ms |

## Device Operation

Measurement begins as soon as the sensor starts (after start-up time), and distance can be read in millimeters or centimeters. The sensor can also be configured to detect a range condition (e.g. distance above a threshold). Depending on the I/O configuration, the SIG pin outputs a digital 0/1, PWM, or serial signal:

- **Serial** — Half-duplex UART used to read sensor registers and change configuration.
- **Digital 0/1** — SIG outputs the detection state: LOW if false, HIGH when true. Detection mode and threshold are configurable.
- **PWM** — SIG outputs a servo-style PWM signal. Period is 20ms, pulse is 1ms-2ms. Pulse width scales with measured distance (1ms = minimum distance, 2ms = maximum distance; both configurable). If the measurement is invalid, the pulse is 2ms.

### Device Configuration

In serial mode, all sensor parameters can be changed and take effect immediately, except I/O configuration, which only applies after a sensor restart. After reset, configuration registers are loaded from the sensor's internal memory; the "save configuration" command persists the current registers to memory, and "reset configuration" restores factory defaults.

An easy way to configure the sensor is using a USB to UART adapter with any serial terminal (e.g. PuTTY, or the Arduino IDE's integrated terminal) — see the "Serial protocol" section for message formatting.

### Device Restart

The sensor can be restarted either by disconnecting and reconnecting the power supply, or by using the "restart sensor" command.

### Entering Serial Mode

If the sensor is configured in digital or PWM mode, you can switch to serial mode by: disconnecting the power supply, shorting the SIG pin to GND (or driving it LOW from an MCU), reconnecting power, waiting at least 50ms for boot, then removing the short from SIG. The sensor will then be in serial mode at 9600 baud with id 0. A subsequent reset returns the sensor to the mode configured in its registers.

## Serial Protocol

The serial protocol uses ASCII characters; numbers are in hexadecimal format. Each packet starts with the command letter and ends with a newline symbol 0x0A (`\n`). Available commands:

| Command | Send | Receive |
|---------|------|---------|
| Write register | `W` `<serial id>` `<register>` `<value>` `\n` (0xA) | `A` or `F` `\n` (0xA) |
| Read register | `R` `<serial id>` `<register>` `\n` (0xA) | `<register value>` `\n` (0xA) |
| Save configuration | `S` `<serial id>` `\n` (0xA) | `A` `\n` (0xA) |
| Restart sensor | `U` `<serial id>` `\n` (0xA) | `A` `\n` (0xA) |
| Reset configuration | `Z` `<serial id>` `\n` (0xA) | `A` `\n` (0xA) |

**Notes:**
- Serial id and register number are sent as two hex characters each.
- Value is hexadecimal, up to 4 characters, leading zeros optional (e.g. `5` == `0005`).
- Write returns `A` on success, `F` on failure (e.g. writing to a read-only register).
- Restart executes ~50ms after the sensor sends its response; all registers reload from saved configuration afterward.

### Example Messages (C strings)

```c
// Write register: set register B0 of sensor 01 to 0x21
Send    -> "W01B021\n"
Receive <- "A\n"                  // success

// Write register: set register 00 of sensor 00 to 0xF65 (invalid: register 00 is read-only)
Send    -> "W0000F65\n"
Receive <- "F\n"                  // failure

// Read register: read register 01 (distance mm) of sensor A6
Send    -> "RA601\n"
Receive <- "12C\n"                // distance = 0x12C = 300 mm
```

## Registers

### Read-Only Registers

| Index | Name | Min | Max | Description |
|-------|------|-----|-----|--------------|
| 00 | Ranging status | 0 | FF | Status of measurement (see bitfield below) |
| 01 | Distance mm | 0 | FFF | Measured distance in mm. Value is `FFF` if invalid |
| 02 | Signal rate | 0 | FFFF | Amount of signal returned in last measurement |
| 03 | Ambient rate | 0 | FFFF | Amount of ambient light in last measurement |
| 04 | Sigma | 0 | FFFF | Measurement standard deviation in mm |
| 05 | SPAD count | 0 | FFFF | Number of photon detectors used in last measurement |
| 06 | Distance cm | 0 | FF | Measured distance in cm. Value is `FF` if invalid |
| 07 | Detection output | 0 | 1 | Output of detection logic configured in register BA |

**Ranging status (register 00) bitfield:**

| Bit | Meaning |
|-----|---------|
| 0 | Measurement is valid |
| 1 | Range wraparound check not passed |
| 2 | Range phase check not passed |
| 3 | Signal minimum check not passed |
| 4 | Signal threshold check not passed |
| 5 | Sigma threshold check not passed |
| 6 | Distance min check not passed |
| 7 | Distance max check not passed |

### Configuration Registers

| Index | Name | Min | Max | Default | Notes |
|-------|------|-----|-----|---------|-------|
| 80 | IO mode | 0 | 2 | 1 | 0 = Serial, 1 = Digital 0/1, 2 = PWM. Takes effect after restart |
| 81 | Serial id | 0 | FF | 0 | Used when IO mode = 0 |
| B0 | Serial baud | 0 | 7 | 0 | 0=9600, 1=19200, 2=38400, 3=57600, 4=74880, 5=115200, 6=230400, 7=250000 |
| 83 | — | — | — | — | *(unused/reserved in source datasheet)* |
| 84 | — | — | — | — | *(unused/reserved in source datasheet)* |
| B0 | Sampling time (ms) | 5 | 50 | 33 | Operation below 10ms not guaranteed *(shares index B0 with Serial baud in source — likely a typo in the original datasheet)* |
| B1 | Offset correction | 0 | FFFF | 0 | Measurement offset correction value |
| B2 | Crosstalk correction | 0 | FFFF | 0 | Measurement crosstalk correction value |
| B3 | Linear correction | 0 | FFFF | 8000 | D = Draw x correction / 0x8000 |
| B4 | Range checks | 0 | FF | F8 | Bitfield enabling validity checks (see below) |
| B5 | Signal threshold | 0 | FFFF | 1024 | Min photon count for a valid distance |
| B6 | Sigma threshold | 0 | FFFF | 15 | Max std. deviation (mm) for a valid distance |
| B7 | Min distance mm | 0 | FFF | 1 | Used if bit 6 of B4 = 1 |
| B8 | Max distance mm | 0 | FFF | 500 | Used if bit 7 of B4 = 1 |
| BA | Detection mode | 0 | 2 | 0 | 0 = valid measurement, 1 = above lower threshold (BC), 2 = within range (BC-BD) |
| BB | Detection output invert | 0 | 1 | 0 | 0 = normal, 1 = inverted |
| BC | Detection lower threshold mm | 0 | FFF | 0 | For detection modes 1 and 2 |
| BD | Detection upper threshold mm | 0 | FFF | 0 | For detection mode 2 |

**Range checks (register B4) bitfield** — each bit enables one validity check; all checks always run and report status to register 00, but only enabled bits (here) determine overall validity:

| Bit | Meaning |
|-----|---------|
| 0 | Not used |
| 1 | Range wraparound check |
| 2 | Range phase check |
| 3 | Signal minimum check |
| 4 | Signal threshold check |
| 5 | Sigma threshold check |
| 6 | Distance min check |
| 7 | Distance max check |
