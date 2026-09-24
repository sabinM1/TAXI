/*
 * TAXI_Firmware.ino — main program for the TAXI mini sumo.
 *
 * Hardware (verified, see Config.h):
 *   5x M60F ToF on D4 (ids 1..5; 1,2,4,5 in cm — 3 in mm)
 *   3x QRE1113 line sensors (A1 right, A4 rear, A5 left)
 *   WS2812 7-LED ring on A2  /  Start-Stop module on A0 (HIGH = fight)
 *   DIP strategy select on D5/D6/D7  /  dual motors (D3 D12 | D11 D13)
 *
 * Loop order matters: ToF serial reads always finish before FastLED.show()
 * (the WS2812 bit-bang blocks interrupts and would garble the sensor bus).
 *
 * Libraries to install (Arduino Library Manager): FastLED, SingleWireSerial.
 */

#include "Config.h"
#include "RingInfoDisplay.h"
#include "Sensors.h"
#include "Strategies.h"
#include "StateMachine.h"

static const CRGB STATE_COLOR[] = {
  CRGB::Green,    // IDLE    — waiting for start
  CRGB::Purple,   // TACTIC  — opening move
  CRGB::Blue,     // SEARCH
  CRGB::Red,      // ATTACK
  CRGB::Orange,   // FLANK
  CRGB::White,    // EVADE
  CRGB::Black     // STOP
};

// Render all 5 ToF sensors as a green->red distance gradient on the outer
// ring (uses RingInfoDisplay.h helpers). Far (>= RING_DIST_GREEN_CM) = green,
// very close = red; brightness also fades with distance (near = bright).
// Invalid readings stay blank.
static void renderToFGradient() {
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    if (!tofValid[i]) continue;
    uint16_t cm = tofMm[i] / 10u;      // tofMm[] holds mm for every sensor
    CRGB c = RingDistanceColor(cm);
    ringLeds[RING_TOF_LED[i + 1]] = RingScaleColor(c, RingDistanceBrightness(cm));
  }
}

static void ringStatusUpdate() {
  static uint32_t blinkT = 0;
  static bool blinkOn = false;

  if (millis() - blinkT < 250) return;
  blinkT = millis();
  blinkOn = !blinkOn;
  FastLED.clear();

  switch ((RobotState)stateMachineState()) {
    case ST_IDLE: {
      // Pit view: 5-sensor ToF distance gradient (green far -> red close)
      // + line-edge overlay (white = white border seen). Strategy number is
      // echoed on Serial at boot, so the ring is free for a live sensor check.
      renderToFGradient();
      if (lineLeft)    ringLeds[3] = CRGB::White;   // left edge  (front-left)
      if (lineRear)    ringLeds[1] = CRGB::White;   // rear edge  (back)
      if (lineRight)   ringLeds[5] = CRGB::White;   // right edge (front-right)
      if (blinkOn)     ringLeds[0] = CRGB::Green;   // heartbeat (center)
      break;
    }

    case ST_SEARCH: {
      // Live front-arc ToF view: LED3=id4 L, LED4=id3 C, LED5=id2 R.
      if (tofValid[3]) ringLeds[3] = CRGB::Green;
      if (tofValid[2]) ringLeds[4] = CRGB::Green;
      if (tofValid[1]) ringLeds[5] = CRGB::Green;
      bool ok; uint16_t mm = sensorsClosestMm(&ok);
      if (ok && mm <= 250) ringLeds[0] = CRGB(255, 191, 0);   // amber
      // Degraded sensor bus: blink center red.
      if (sensorsHealthyCount() < TOF_MIN_HEALTHY_SENSORS && blinkOn)
        ringLeds[0] = CRGB::Red;
      break;
    }

    default:
      RingFill(STATE_COLOR[stateMachineState()]);
      break;
  }
  RingShow();
}

// ============================= DEBUG TELEMETRY =============================
// Prints one snapshot every 200 ms when DEBUG_VERBOSE is on:
//   state, DipCP strategy, start pin, opponent bearing + closest mm,
//   all 5 ToF (mm, '-' = invalid), line raw + debounced flags, health count,
//   trimpot scale, last motor command.
#if DEBUG_VERBOSE
static void debugDump() {
  static uint32_t last = 0;
  if (millis() - last < 200) return;
  last = millis();

  int l, r; motorsGetLast(&l, &r);
  bool ok; uint16_t mm = sensorsClosestMm(&ok);

  Serial.print(F("[DBG] t=")); Serial.print(millis());
  Serial.print(F(" st="));  Serial.print(stateName((RobotState)stateMachineState()));
  Serial.print(F(" dip="));  Serial.print(strategyIndex);
  Serial.print(F(" start="));Serial.print(digitalRead(PIN_START));
  Serial.print(F(" brg="));  Serial.print(sensorsOpponent());
  Serial.print(F(" mm="));   if (ok) Serial.print(mm); else Serial.print('-');
  Serial.print(F(" tof=["));
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    if (tofValid[i]) Serial.print(tofMm[i]); else Serial.print('-');
    if (i < TOF_SENSOR_COUNT - 1) Serial.print(',');
  }
  Serial.print(F("] h="));    Serial.print(sensorsHealthyCount());
  Serial.print(F(" line=[")); for (uint8_t i = 0; i < 3; i++) {
    Serial.print(lineRaw[i]); if (i < 2) Serial.print(',');
  }
  Serial.print(F("] edge="));
  Serial.print(lineLeft); Serial.print(lineRear); Serial.print(lineRight);
  Serial.print(F(" tri=")); Serial.print(trimpotScale());
  Serial.print(F(" mot=")); Serial.print(l); Serial.print('/'); Serial.println(r);
}
#endif

void setup() {
  // FIRST: claim the motor pins. From power-on until now they float in
  // input mode, which lets the driver chatter the motors (the ~0.5 s
  // right-motor spin some of it is the bootloader blinking D13/RM_DIR).
  motorInit();
  motorsStop();

  // Startup grace period: 3 s before anything else happens. Lets the
  // operator step back after power-on. Motors are already held stopped.
  delay(3000);

  Serial.begin(MONITOR_BAUD);
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);

  // NanoLabs start module rule: the pin MUST read LOW at power-on
  // (module armed, waiting for the referee start command). If it reads
  // HIGH here, the module still holds the previous match's "on" state —
  // the referee must send stop and the robot must be power-cycled.
  // Lock hard and blink UL1 so this can never be mistaken for a ready robot.
  if (digitalRead(PIN_START) == HIGH) {
#if SERIAL_DEBUG
    Serial.println(F("[TAXI] FATAL: start pin HIGH at boot - power cycle!"));
#endif
    while (true) {
      digitalWrite(PIN_LED1, HIGH); delay(100);
      digitalWrite(PIN_LED1, LOW);  delay(100);
    }
  }

  sensorsInit();
  strategiesInit();
  RingInit();

#if SERIAL_DEBUG
  Serial.println(F("[TAXI] firmware up"));
#endif
  delay(500);   // give M60F sensors time to boot (10..50 ms each, generous)
}

void loop() {
  sensorsUpdate();          // 1 ToF read + line pass (all serial I/O here)
  stateMachineUpdate();     // drives motors, no serial I/O
  ringStatusUpdate();       // FastLED.show() LAST — blocks interrupts
#if DEBUG_VERBOSE
  debugDump();
#endif
}
