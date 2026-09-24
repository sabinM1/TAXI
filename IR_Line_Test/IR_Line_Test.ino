/*
 * IR_Line_Test — bench test for the three QRE1113 line sensors.
 *
 * Prints raw ADC values (0..1023) and the debounced edge state of each
 * sensor to the Serial Monitor @115200, evaluated exactly like TAXI_Firmware.
 *
 *   Pins:      LEFT = A5, REAR = A4, RIGHT = A1   (Config.h)
 *   Logic:     black interior = HIGH, white border = LOW
 *              EDGE = raw BELOW the white threshold (Schmitt + debounce).
 *   Thresholds (Config.h): left/right white <400 (black >800),
 *                          rear   white <765 (black >820).
 *
 * Expected Serial output (on the dohyo):
 *   L: 690 [ OK  ] | B: 705 [ OK  ] | R: 688 [ OK  ]    <- black interior
 *   L:  41 [EDGE ] | B: 700 [ OK  ] | R: 690 [ OK  ]    <- left on white border
 */

#include "Config.h"
#include "LineSensor.h"

// Same instances/wiring as the combat firmware (Sensors.h).
LineSensor lineSensL(PIN_LINE_L, LINE_LEFT_WHITE_THRESHOLD);
LineSensor lineSensB(PIN_LINE_B, LINE_REAR_WHITE_THRESHOLD);
LineSensor lineSensR(PIN_LINE_R, LINE_RIGHT_WHITE_THRESHOLD);

static uint8_t edgeCount[3];          // 0=L 1=B 2=R white-hit debounce counters
static bool    state[3];              // latched edge flags (true = EDGE)

static const uint16_t PRINT_PERIOD_MS = 100;

// Mirrors Sensors.h lineReadAll(): Schmitt trigger + debounce.
static bool evalDebounced(uint8_t i, uint16_t raw) {
  static const uint16_t WHITE_THR[3] = {
    LINE_LEFT_WHITE_THRESHOLD, LINE_REAR_WHITE_THRESHOLD, LINE_RIGHT_WHITE_THRESHOLD
  };
  static const uint16_t BLACK_THR[3] = {
    LINE_LEFT_BLACK_THRESHOLD, LINE_REAR_BLACK_THRESHOLD, LINE_RIGHT_BLACK_THRESHOLD
  };

  if (state[i]) {
    if (raw > BLACK_THR[i]) state[i] = false;   // clearly back on black
  } else {
    if (raw < WHITE_THR[i]) {
      if (edgeCount[i] < LINE_DEBOUNCE_SAMPLES) edgeCount[i]++;
      if (edgeCount[i] >= LINE_DEBOUNCE_SAMPLES) state[i] = true;
    } else {
      edgeCount[i] = 0;
    }
  }
  return state[i];
}

void setup() {
  Serial.begin(MONITOR_BAUD);
  while (!Serial && millis() < 3000) {}   // native USB (32u4): wait briefly

  lineSensL.begin();
  lineSensB.begin();
  lineSensR.begin();

  Serial.println(F("=== IR_Line_Test — QRE1113 bench test ==="));
  Serial.println(F("black = HIGH, white = LOW; raw < white threshold => EDGE"));
  Serial.println(F("L/B/R white thr: 400/765/400, black thr: 800/820/800\n"));
}

void loop() {
  uint16_t rawL = lineSensL.readRaw();
  uint16_t rawB = lineSensB.readRaw();
  uint16_t rawR = lineSensR.readRaw();

  evalDebounced(0, rawL);
  evalDebounced(1, rawB);
  evalDebounced(2, rawR);

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = millis();
    Serial.print(F("L: ")); Serial.print(rawL);
    Serial.print(state[0] ? F(" [EDGE]") : F(" [ OK ]"));
    Serial.print(F(" | B: ")); Serial.print(rawB);
    Serial.print(state[1] ? F(" [EDGE]") : F(" [ OK ]"));
    Serial.print(F(" | R: ")); Serial.print(rawR);
    Serial.println(state[2] ? F(" [EDGE]") : F(" [ OK ]"));
  }
}
