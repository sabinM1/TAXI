# Arduino Project Documentation Guideline (for AI Documentation Agents)

## Purpose

This document defines the standard an AI assistant must follow when generating documentation for an Arduino-based hardware/software project. It applies to personal robotics projects, sensor integrations, combat robot electronics, and general embedded prototypes. Follow this structure exactly unless the user explicitly requests a different format. Precision over verbosity: every claim must be verifiable from the code, schematic, or BOM provided — never invent pin numbers, part names, or values.

---

## 0. Before Writing Anything

1. Inventory all available inputs: source code (`.ino`/`.cpp`/`.h`), wiring diagrams/schematics, BOM/parts list, photos, datasheets, and any user notes.
2. If a required fact (pinout, voltage, library version, sensor model) is missing or ambiguous, flag it explicitly with `> ⚠️ TODO: confirm <fact>` rather than guessing.
3. Identify the target audience (the user themself for future reference, a teammate, or public/GitHub readers) and adjust depth accordingly, but never omit safety-critical or wiring-critical details for brevity.
4. Determine the microcontroller/board variant (e.g., Arduino Uno R3, Nano, ESP32, Mega2560) since pin capabilities, voltage levels, and library compatibility differ.

---

## 1. Required File Structure

Produce a single `README.md` (or `DOCS.md` for larger projects) with this section order. Omit a section only if genuinely not applicable, but note the omission.

```
1. Title & One-Line Summary
2. Project Overview
3. Hardware / Bill of Materials (BOM)
4. Wiring & Pinout
5. Power Requirements
6. Software Dependencies & Setup
7. Firmware Structure & Logic
8. Configuration Constants
9. Build & Upload Instructions
10. Usage / Operation
11. Calibration & Tuning (if applicable)
12. Troubleshooting
13. Safety Notes
14. Known Limitations / Future Work
15. License & Credits
```

---

## 2. Section-by-Section Requirements

### 1. Title & One-Line Summary
- Project name as H1.
- One sentence describing what it does and what board it runs on.

### 2. Project Overview
- 2–4 sentences: goal, key features, what problem it solves.
- Mention the high-level architecture (e.g., "sensor → microcontroller → motor driver → actuator").
- Include a block-diagram description in words if no image is available; if an image exists, reference it with a relative path (`![Block Diagram](docs/block_diagram.png)`).

### 3. Hardware / Bill of Materials
Present as a Markdown table. Required columns:

| Component | Model/Part No. | Qty | Voltage/Rating | Notes |
|---|---|---|---|---|

- Include every part referenced in the code or schematic (resistors, capacitors, connectors included if functionally relevant).
- Specify exact part numbers when known (e.g., "MG995 servo," not "a servo"); mark unknowns with the TODO flag.
- Note substitutions/tolerances only if the user has confirmed them.

### 4. Wiring & Pinout
This is the highest-precision section — errors here can damage hardware.

- Provide a pin-mapping table:

| Arduino Pin | Function | Connected To | Pin Mode | Notes |
|---|---|---|---|---|

- State pin numbering exactly as used in code (`D2`, `A0`, `GPIO4`, etc.) — do not normalize or rename.
- Explicitly note: digital vs analog, INPUT/INPUT_PULLUP/OUTPUT, PWM-capable pins, interrupt-capable pins if used.
- Call out any level-shifting requirements (e.g., 5V logic to 3.3V sensor).
- If a schematic image/Fritzing diagram exists, embed/reference it; otherwise generate an ASCII wiring list grouped by component.
- Flag any pins that must NOT be used for other purposes (e.g., reserved for I2C, SPI, or bootloader).

### 5. Power Requirements
- Supply voltage and current draw per component and total estimated peak/average current.
- Power source used (USB, battery pack, external regulator) and its rating.
- Note any regulators, fuses, or reverse-polarity protection in the design.
- Flag brownout risks (e.g., servos/motors drawing current spikes on the same rail as the MCU).

### 6. Software Dependencies & Setup
- Arduino IDE version or PlatformIO config used.
- Board package/core required (e.g., `esp32` by Espressif, version).
- Every included library with exact name as it appears in Library Manager and version if pinned (from `library.properties` or `platformio.ini` if available).
- Any external tools (Python scripts, serial monitors, companion apps).

### 7. Firmware Structure & Logic
- Summarize `setup()` responsibilities and `loop()` flow in plain language, not a line-by-line code dump.
- Describe state machines, interrupt handlers, or timing logic (e.g., `millis()`-based non-blocking delays) if present.
- List custom functions with a one-line purpose each in a table: `| Function | Purpose |`.
- Note communication protocols used (Serial, I2C, SPI, Bluetooth, ESP-NOW, Wi-Fi) and their baud rate/addresses.

### 8. Configuration Constants
- Table of all `#define` / `const` tunables a user might need to change (pins already covered in Section 4 — focus here on thresholds, timings, calibration offsets, PID gains, etc.).

| Constant | Default Value | Meaning | Safe Range |
|---|---|---|---|

### 9. Build & Upload Instructions
Numbered, reproducible steps:
1. IDE/board manager setup.
2. Board and port selection (exact board name in menu).
3. Compile flags or partition scheme if relevant (ESP32/ESP8266).
4. Upload procedure, including any manual bootloader steps (e.g., hold BOOT button).

### 10. Usage / Operation
- How to power on and what to expect (LED states, serial output, startup sequence).
- Control interface: buttons, serial commands, app, remote control — list exact command syntax if serial-controlled.
- Expected normal behavior described observably (not just internally).

### 11. Calibration & Tuning (if applicable)
- Step-by-step calibration procedure (e.g., sensor zeroing, PID tuning, ESC arming).
- Expected value ranges to confirm correct calibration.

### 12. Troubleshooting
Table format:

| Symptom | Likely Cause | Fix |
|---|---|---|

- Include only issues plausible given the actual hardware/code (e.g., brownout resets, floating pins, wrong COM port, missing pull-up resistor) — do not pad with generic filler.

### 13. Safety Notes
- Battery handling (LiPo charging/storage precautions if used).
- Moving parts, sharp components, high-current motor warnings — relevant for combat robot or motor-driven builds.
- ESD precautions during assembly if sensitive components are involved.

### 14. Known Limitations / Future Work
- Honest list of current shortcomings and planned improvements, sourced from user notes or code comments (e.g., `// TODO` in source).

### 15. License & Credits
- License type if specified; otherwise state "unspecified — confirm with author."
- Credit third-party libraries, tutorials, or code snippets referenced/adapted.

---

## 3. Formatting Rules

- Use GitHub-Flavored Markdown: headers (`##`), tables, fenced code blocks with language tags (` ```cpp `, ` ```ini `), and task lists (`- [ ]`) where appropriate.
- Keep code excerpts minimal and purposeful — only show snippets that illustrate a specific point (e.g., a tricky interrupt handler), never paste the entire sketch into the README.
- Use consistent units (V, mA, mm, Hz) throughout; never mix without labeling.
- Use exact identifiers from the source (variable names, pin macros) verbatim — do not paraphrase code symbols.
- Number all sequential procedures; use bullets for non-ordered facts.
- Every table must have a header row; no empty cells — use "N/A" explicitly instead of leaving blank.

## 4. Accuracy & Verification Rules

- Never state a voltage, current, pin assignment, or library version without a source (code, datasheet, or explicit user statement). If uncertain, mark with `⚠️ TODO: confirm`.
- Cross-check the BOM against the wiring table and the code's pin definitions for consistency before finalizing — flag any mismatch instead of silently resolving it.
- If the code uses a library function whose exact electrical behavior matters (e.g., `analogWrite` PWM frequency, `Servo.attach` pulse range), state the assumed default only if not overridden in code.
- Do not fabricate performance figures (e.g., "high accuracy," "long battery life") — only state measured or documented values, or omit the claim.

## 5. Output Delivery

- Deliver as one complete `.md` file, self-contained, ready to drop into the project's repository root.
- Do not split into multiple files unless the project explicitly has multi-module firmware requiring separate docs (in which case, cross-link them and note this limitation in Section 14).
