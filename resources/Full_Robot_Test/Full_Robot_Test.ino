/*
 * Full_Robot_Test.ino
 * -------------------
 * Comprehensive test for the mini sumo robot covering all hardware functions:
 *   - 5x M60F ToF sensors (single-wire bus on D4)
 *   - 3x QRE1113 line/edge sensors (A1=right, A4=rear, A5=left)
 *   - Start/Stop module (A0, used as digital input)
 *   - WS2812 7-LED ring (A2)
 *   - Dual JSUMO 400RPM motors (capped to MOTOR_MAX_PCT in xmotionV3.cpp)
 *   - XMotion V3 library functions
 *
 * Wiring per hardware config:
 *   A0 -> Start/Stop module (U10) - digital read
 *   A1 -> QRE1113 right line sensor (U9)
 *   A2 -> WS2812 DIN (U11)
 *   A4 -> QRE1113 rear line sensor (U8)
 *   A5 -> QRE1113 left line sensor (U7)
 *   D3/D12  -> MOT1 PWM/DIR (left)   - driven inside xmotionV3.cpp
 *   D11/D13 -> MOT2 PWM/DIR (right)  - driven inside xmotionV3.cpp
 *   D4  -> M60F SIG bus (5 sensors, 4.7k pull-up to 5V)
 *
 * Serial commands (115200 baud):
 *   0..7 - select test mode   a - toggle auto-cycle
 *   r    - restart ToF        s - save ToF config  h - help
 *
 * SAFETY: all motor output is clamped to MOTOR_MAX_PCT percent of full power
 * (defined in xmotionV3.cpp, currently 15%). Change it there, not here.
 */

#include <SingleWireSerial.h>
#include <FastLED.h>
#include "DistanceSensor.h"
#include "RingInfoDisplay.h"
#include "xmotionV3.h"
#include "LineSensor.h"

// ============================================================================
// CONFIG
// ============================================================================
#define PIN_START_STOP   A0     // Start/Stop module (digital)
#define PIN_LINE_RIGHT   A1     // QRE1113 right (U9)
#define PIN_LINE_REAR    A4     // QRE1113 rear (U8)
#define PIN_LINE_LEFT    A5     // QRE1113 left (U7)
#define PIN_TOF_BUS      4      // M60F shared SIG bus

#define SENSOR_COUNT     5
#define MONITOR_BAUD     115200
#define SENSOR_BAUD      115200
#define LINE_THRESHOLD   500    // analog threshold; values above = line (dark)

#define TEST_DURATION    5000   // ms per test in auto-cycle mode

// When false, auto-cycle skips the motor test (desk safety).
#define AUTO_CYCLE_MOTORS  false

// ============================================================================
// TYPES & GLOBALS
// ============================================================================
enum TestMode : uint8_t {
  TEST_IDLE, TEST_TOF, TEST_LINE, TEST_RING,
  TEST_MOTORS, TEST_START_STOP, TEST_ALL, TEST_XMOTION
};

SingleWireSerial oneWire(false);
DistanceSensor sensors[SENSOR_COUNT] = {
  DistanceSensor(&oneWire, 1), DistanceSensor(&oneWire, 2),
  DistanceSensor(&oneWire, 3), DistanceSensor(&oneWire, 4),
  DistanceSensor(&oneWire, 5)
};

LineSensor lineRight(PIN_LINE_RIGHT, LINE_THRESHOLD);
LineSensor lineRear (PIN_LINE_REAR,  LINE_THRESHOLD);
LineSensor lineLeft (PIN_LINE_LEFT,  LINE_THRESHOLD);

uint16_t tofDistances[SENSOR_COUNT];
uint16_t lineRaw[3];      // [0]=left [1]=rear [2]=right - one ADC sample each
bool     lineStates[3];   // derived from lineRaw in the same pass

TestMode currentTest = TEST_IDLE;
unsigned long lastTestSwitch = 0;
bool autoMode = true;

#if AUTO_CYCLE_MOTORS
const TestMode AUTO_SEQUENCE[] = { TEST_TOF, TEST_LINE, TEST_RING, TEST_MOTORS,
                                   TEST_START_STOP, TEST_ALL, TEST_XMOTION };
#else
const TestMode AUTO_SEQUENCE[] = { TEST_TOF, TEST_LINE, TEST_RING,
                                   TEST_START_STOP, TEST_ALL, TEST_XMOTION };
#endif
#define AUTO_LEN  (sizeof(AUTO_SEQUENCE) / sizeof(AUTO_SEQUENCE[0]))

// ============================================================================
// PROGMEM NAME TABLES (test modes, motor phases, xmotion steps)
// ============================================================================
const char N0[] PROGMEM = "IDLE";        const char N1[] PROGMEM = "TOF";
const char N2[] PROGMEM = "LINE";        const char N3[] PROGMEM = "RING";
const char N4[] PROGMEM = "MOTORS";      const char N5[] PROGMEM = "START/STOP";
const char N6[] PROGMEM = "ALL SENSORS"; const char N7[] PROGMEM = "XMOTION";
const char* const MODE_NAMES[] PROGMEM = { N0, N1, N2, N3, N4, N5, N6, N7 };

const char M0[] PROGMEM = "Forward";       const char M1[] PROGMEM = "Backward";
const char M2[] PROGMEM = "Right spin";    const char M3[] PROGMEM = "Left spin";
const char M4[] PROGMEM = "Arc L fast";    const char M5[] PROGMEM = "Arc R fast";
const char M6[] PROGMEM = "Raw R>L fwd";   const char M7[] PROGMEM = "Raw R>L rev";
const char* const MOTOR_PHASES[] PROGMEM = { M0, M1, M2, M3, M4, M5, M6, M7 };

const char X0[] PROGMEM = "Trimpot read";  const char X1[] PROGMEM = "UserLed1";
const char X2[] PROGMEM = "UserLed2";      const char X3[] PROGMEM = "ToggleLeds";
const char X4[] PROGMEM = "CounterLeds";   const char X5[] PROGMEM = "Forward";
const char X6[] PROGMEM = "Backward";      const char X7[] PROGMEM = "Right0";
const char X8[] PROGMEM = "Left0";         const char X9[] PROGMEM = "StopMotors";
const char* const XMOTION_STEPS[] PROGMEM =
  { X0, X1, X2, X3, X4, X5, X6, X7, X8, X9 };

// Print name[i] from a PROGMEM string table.
void printPName(const char* const* table, uint8_t i) {
  Serial.print((const __FlashStringHelper*)pgm_read_ptr(&table[i]));
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
void setup() {
  pinMode(PIN_START_STOP, INPUT_PULLUP);
  pinMode(PIN_TOF_BUS, INPUT);          // release SIG; pull-up holds idle high

  Serial.begin(MONITOR_BAUD);
  oneWire.begin(SENSOR_BAUD);
  oneWire.setTimeout(2);                // 2ms: ~4x a full reply @115200

  RingInit();
  lineLeft.begin(); lineRear.begin(); lineRight.begin();
  xmotion.SETUP();
  xmotion.StopMotors(0);

  Serial.println(F("=== FULL ROBOT TEST ==="));
  Serial.println(F("HW: A0=Start A1=LineR A2=Ring A4=LineRear A5=LineL D4=ToF"));
  Serial.println(F("Motors capped at MOTOR_MAX_PCT (see xmotionV3.cpp)"));
  printHelp();
  Serial.print(F("Auto mode: ON (5s/test, motors "));
  Serial.println(AUTO_CYCLE_MOTORS ? F("INCLUDED)") : F("SKIPPED - press 4)"));
  Serial.println();

  delay(100);  // sensor start-up (10-50ms each)
}

void loop() {
  handleSerialCommands();

  if (autoMode && (millis() - lastTestSwitch > TEST_DURATION)) {
    uint8_t idx = AUTO_LEN - 1;                     // unknown mode -> wrap to 0
    for (uint8_t i = 0; i < AUTO_LEN; i++)
      if (AUTO_SEQUENCE[i] == currentTest) { idx = i; break; }
    setTestMode(AUTO_SEQUENCE[(idx + 1) % AUTO_LEN]);   // keeps autoMode on
  }

  switch (currentTest) {
    case TEST_TOF:        runToFTest();       break;
    case TEST_LINE:       runLineTest();      break;
    case TEST_RING:       runRingTest();      break;
    case TEST_MOTORS:     runMotorTest();     break;
    case TEST_START_STOP: runStartStopTest(); break;
    case TEST_ALL:        runAllTest();       break;
    case TEST_XMOTION:    runXMotionTest();   break;
    default:              runIdle();          break;
  }
}

// ============================================================================
// MODE / COMMAND HANDLING
// ============================================================================
void handleSerialCommands() {
  if (!Serial.available()) return;
  char cmd = Serial.read();

  if (cmd >= '0' && cmd <= '7') {
    setTestMode((TestMode)(cmd - '0'));
    autoMode = false;                     // manual selection stops the cycle
    return;
  }
  switch (cmd) {
    case 'a': case 'A':
      autoMode = !autoMode;
      lastTestSwitch = millis();
      Serial.print(F("Auto mode: ")); Serial.println(autoMode ? F("ON") : F("OFF"));
      break;
    case 'r': case 'R': resetToFSensors(); break;
    case 's': case 'S': saveToFConfig();   break;
    case 'h': case 'H': case '?': printHelp(); break;
    default: break;                       // swallow '\n', '\r', junk
  }
}

void printHelp() {
  Serial.println(F("Cmds: 0=Idle 1=ToF 2=Line 3=Ring 4=Motors 5=Start/Stop"));
  Serial.println(F("      6=All 7=XMotion a=Auto r=ToF-reset s=ToF-save h=Help"));
}

void setTestMode(TestMode mode) {
  currentTest = mode;
  lastTestSwitch = millis();
  Serial.print(F("Test mode: ")); printPName(MODE_NAMES, (uint8_t)mode);
  Serial.println();
  xmotion.StopMotors(0);
  RingClear();
  RingShow();
}

// ============================================================================
// SHARED HELPERS
// ============================================================================

// Print one ToF reading; no dynamic allocation (no String on AVR).
void printTofReading(uint8_t id, uint16_t value) {
  if (id == 3) {
    if (value == 0xFFF) Serial.print(F("---"));
    else { Serial.print(value); Serial.print(F("mm")); }
  } else {
    if (value == 0xFF) Serial.print(F("---"));
    else { Serial.print(value); Serial.print(F("cm")); }
  }
}

// Full ToF sweep (ID3 in mm, others cm) into tofDistances[].
void readToFSensors() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    tofDistances[i] = (sensors[i].mDeviceIndex == 3)
                      ? sensors[i].readDistance_mm()
                      : sensors[i].readDistance_cm();
  }
}

// Single ADC pass: fill lineRaw[] and lineStates[] from the same samples.
void readLineSensors() {
  lineRaw[0] = lineLeft.readRaw();
  lineRaw[1] = lineRear.readRaw();
  lineRaw[2] = lineRight.readRaw();
  lineStates[0] = lineLeft.evaluate(lineRaw[0]);
  lineStates[1] = lineRear.evaluate(lineRaw[1]);
  lineStates[2] = lineRight.evaluate(lineRaw[2]);
}

// Render line sensors with the ACTUAL layout (left / REAR / right):
//   left (A5) -> LED 3 (left front)   rear (A4) -> LED 1 (back, 6 o'clock)
//   right (A1) -> LED 5 (right front) center LED 0 = red if ANY line tripped
// Does NOT call RingShow(); caller may overlay more info first.
void renderLineRing() {
  FastLED.clear();
  if (lineStates[0]) ringLeds[3] = CRGB::White;
  if (lineStates[1]) ringLeds[1] = CRGB::White;   // rear sensor at the back LED
  if (lineStates[2]) ringLeds[5] = CRGB::White;
  ringLeds[0] = (lineStates[0] || lineStates[1] || lineStates[2])
                ? CRGB::Red : CRGB::Black;
}

// ============================================================================
// TESTS
// ============================================================================
void runToFTest() {
  static unsigned long lastPrint = 0;
  readToFSensors();
  RingShowToFRanges(tofDistances);

  if (millis() - lastPrint < 200) return;
  lastPrint = millis();
  Serial.print(F("TOF: "));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print('S'); Serial.print(i + 1); Serial.print('=');
    printTofReading(sensors[i].mDeviceIndex, tofDistances[i]);
    if (i < SENSOR_COUNT - 1) Serial.print(F(" | "));
  }
  Serial.println();
}

void runLineTest() {
  static unsigned long lastPrint = 0;
  readLineSensors();
  renderLineRing();
  RingShow();

  if (millis() - lastPrint < 200) return;
  lastPrint = millis();
  Serial.print(F("LINE L="));  Serial.print(lineStates[0] ? F("ON ") : F("off"));
  Serial.print(F(" REAR="));   Serial.print(lineStates[1] ? F("ON ") : F("off"));
  Serial.print(F(" R="));      Serial.print(lineStates[2] ? F("ON ") : F("off"));
  Serial.print(F("  raw ["));  Serial.print(lineRaw[0]);
  Serial.print(',');           Serial.print(lineRaw[1]);
  Serial.print(',');           Serial.print(lineRaw[2]);
  Serial.println(']');
}

void runRingTest() {
  static uint8_t effect = 0, pos = 0, hue = 0;
  static unsigned long lastEffect = 0;

  if (millis() - lastEffect > 3000) {
    effect = (effect + 1) % 6;
    lastEffect = millis();
    pos = 0; hue = 0;
    FastLED.clear(); RingShow();
    Serial.print(F("Ring effect: ")); Serial.println(effect);
  }

  switch (effect) {
    case 0:  // color wipe - red
      EVERY_N_MILLISECONDS(100) {
        if (pos == 0) FastLED.clear();
        RingSetLED(pos, 255, 0, 0);
        RingShow();
        pos = (pos + 1) % RING_LED_COUNT;
      }
      break;
    case 1:  // theater chase - blue
      EVERY_N_MILLISECONDS(150) {
        FastLED.clear();
        for (uint8_t i = 0; i < RING_LED_COUNT; i++)
          if ((i + pos) % 3 == 0) RingSetLED(i, 0, 0, 255);
        RingShow();
        pos = (pos + 1) % 3;
      }
      break;
    case 2:  // rainbow
      EVERY_N_MILLISECONDS(30) {
        for (uint8_t i = 0; i < RING_LED_COUNT; i++)
          RingSetLEDColor(i, CHSV(hue + i * 255 / RING_LED_COUNT, 255, 255));
        RingShow();
        hue++;
      }
      break;
    case 3:  // breathing cyan (scaled copy, no internal buffer poking)
      {
        CRGB c(0, 255, 255);
        c.nscale8_video(beatsin8(20, 30, 255));
        RingFill(c);
        RingShow();
      }
      break;
    case 4: runToFTest();  return;   // live distance display
    case 5: runLineTest(); return;   // live line display
  }
}

void runMotorTest() {
  static uint8_t phase = 0xFF;             // 0xFF -> announce on first entry
  static unsigned long t0 = 0;

  if (millis() - t0 > 2000) {
    t0 = millis();
    phase = (phase == 0xFF) ? 0 : (phase + 1) % 8;
    xmotion.StopMotors(0);
    Serial.print(F("Motor ")); Serial.print(phase); Serial.print(F(": "));
    printPName(MOTOR_PHASES, phase); Serial.println();
  }

  // Requests intentionally exceed the cap so MOTOR_MAX_PCT is exercised.
  switch (phase) {
    case 0: xmotion.Forward(100, 0);          break;
    case 1: xmotion.Backward(100, 0);         break;
    case 2: xmotion.Right0(100, 0);           break;
    case 3: xmotion.Left0(100, 0);            break;
    case 4: xmotion.ArcTurn(100, 50, 0);      break;
    case 5: xmotion.ArcTurn(50, 100, 0);      break;
    case 6: xmotion.MotorControl(255, 128);   break;
    case 7: xmotion.MotorControl(-128, -255); break;
  }
}

void runStartStopTest() {
  static unsigned long lastPrint = 0, lastChange = 0;
  static bool lastStable = HIGH, lastRaw = HIGH;
  const unsigned long DEBOUNCE_MS = 30;

  bool raw = digitalRead(PIN_START_STOP);
  if (raw != lastRaw) { lastRaw = raw; lastChange = millis(); }

  if (raw != lastStable && (millis() - lastChange) >= DEBOUNCE_MS) {
    lastStable = raw;
    bool started = (raw == LOW);
    Serial.println(started ? F("Start/Stop: START") : F("Start/Stop: STOP"));
    RingFill(started ? CRGB::Green : CRGB::Red);
    RingShow();
  }

  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print(F("Start/Stop: "));
    Serial.println(lastStable == LOW ? F("LOW (start)") : F("HIGH (idle)"));
  }
}

void runAllTest() {
  static unsigned long lastPrint = 0;
  readToFSensors();
  readLineSensors();
  bool started = (digitalRead(PIN_START_STOP) == LOW);

  // IMPORTANT: all single-wire ToF reads finish BEFORE FastLED.show()
  // disables interrupts for the WS2812 bit-bang.
  if (lineStates[0] || lineStates[1] || lineStates[2]) {
    renderLineRing();               // line has priority - more urgent
  } else {
    FastLED.clear();
    bool anyClose = RingRenderToF(tofDistances);
    ringLeds[0] = anyClose ? CRGB::Orange : CRGB::Black;
  }
  RingShow();

  if (millis() - lastPrint < 300) return;
  lastPrint = millis();
  Serial.print(F("ALL: TOF=["));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    printTofReading(sensors[i].mDeviceIndex, tofDistances[i]);
    if (i < SENSOR_COUNT - 1) Serial.print(',');
  }
  Serial.print(F("] LINE="));
  Serial.print(lineStates[0] ? '1' : '0');
  Serial.print(lineStates[1] ? '1' : '0');
  Serial.print(lineStates[2] ? '1' : '0');
  Serial.print(F(" START="));
  Serial.println(started ? F("ON") : F("OFF"));
}

void runXMotionTest() {
  static uint8_t step = 0xFF;
  static unsigned long t0 = 0;

  if (millis() - t0 > 1500) {
    t0 = millis();
    step = (step == 0xFF) ? 0 : (step + 1) % 10;
    xmotion.StopMotors(0);
    Serial.print(F("XMotion ")); Serial.print(step); Serial.print(F(": "));
    printPName(XMOTION_STEPS, step); Serial.println();
  }

  switch (step) {
    case 0: {   // trimpot, throttled print
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 300) {
        lastPrint = millis();
        Serial.print(F("Trimpot: ")); Serial.println(xmotion.Trimpot());
      }
    } break;
    case 1: xmotion.UserLed1(200);       break;   // blocking ~400ms (test only)
    case 2: xmotion.UserLed2(200);       break;
    case 3: xmotion.ToggleLeds(200);     break;
    case 4: xmotion.CounterLeds(100, 3); break;
    case 5: xmotion.Forward(100, 0);     break;   // capped by MOTOR_MAX_PCT
    case 6: xmotion.Backward(100, 0);    break;
    case 7: xmotion.Right0(100, 0);      break;
    case 8: xmotion.Left0(100, 0);       break;
    case 9: xmotion.StopMotors(0);       break;
  }
}

void runIdle() {
  static unsigned long lastBlink = 0;
  static bool on = false;
  if (millis() - lastBlink > 1000) {
    lastBlink = millis();
    on = !on;
    RingSetLED(0, 0, on ? 32 : 0, on ? 0 : 32);   // dim green/blue blink
    RingShow();
  }
}

// ============================================================================
// TOF MAINTENANCE (serial commands r / s)
// ============================================================================
void resetToFSensors() {
  Serial.println(F("Restarting ToF sensors..."));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) sensors[i].restartSensor();
  delay(100);
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(F("  ID")); Serial.print(sensors[i].mDeviceIndex);
    Serial.println(sensors[i].waitForBoot(300) ? F(" OK") : F(" NO RESPONSE"));
  }
}

void saveToFConfig() {
  Serial.println(F("Saving ToF config..."));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(F("  ID")); Serial.print(sensors[i].mDeviceIndex);
    Serial.println(sensors[i].saveConfiguration() ? F(" saved") : F(" NO ACK"));
  }
}