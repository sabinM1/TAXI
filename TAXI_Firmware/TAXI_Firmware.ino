/**
 * ============================================================================
 * TAXI — Mini Sumo Robot Firmware
 * Platform: JSUMO XMotion V3 (ATmega32U4, Arduino Leonardo)
 * Class: Mini Sumo — 500g, 10cm × 10cm, 77cm dohyo
 * ============================================================================
 * 
 * ARCHITECTURE: Non-blocking modular state machine
 *   - All sensor reads are async or sub-millisecond
 *   - No delay(), no pulseIn(), no blocking readBytesUntil()
 *   - Main loop runs at full speed (~1.5ms per iteration)
 * 
 * MODULES:
 *   Config.h        — All constants, pins, thresholds (tune here)
 *   Sensors.h       — M60F async UART + QRE1113 analog fast read
 *   Motors.h        — Motor control wrappers (non-blocking)
 *   PIDController.h — PD controller with hybrid gain scheduling
 *   StateMachine.h  — Combat state machine (IDLE→TACTIC→SEARCH↔ATTACK↔FLANK→EVADE)
 *   Tactics.h       — DIP-switch selectable opening strategies
 *   LEDRing.h       — WS2812 LED ring visual feedback
 *   IMU.h           — ICM20602 gyro/accel (experimental)
 * 
 * DECISIONS (from Phase 3 user answers):
 *   Q1: 3 front M60F (center + 25° left/right) + 2 side (90°) — Option A
 *   Q2: All M60F in Serial mode (distance mm) — Option A
 *   Q3: ICM20602 full integration with positioning — Option A
 *   Q4: 500g / 10cm×10cm standard mini sumo
 *   Q5: Multiple strategies via DIP (reactive + active search)
 *   Q6: Curved re-entry edge evasion
 *   Q7: START-STOP on A0
 *   Q8: PID gains hardcoded + companion tuning tool
 *   Q9: Missing parts included in BOM
 *   Q10: XT30 battery connector
 *   Q11: Standalone HTML GUI for strategy programming
 *   Q12: 3 front + 2 side, NO rear sensor
 */

#include <xmotionV3.h>
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "PIDController.h"
#include "StateMachine.h"
#include "Tactics.h"
#include "LEDRing.h"
#include "IMU.h"

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // ---- Serial debug output ----
    #if SERIAL_DEBUG
    Serial.begin(SERIAL_BAUD);
    delay(500);  // Let USB serial stabilize
    Serial.println(F("\n\n╔══════════════════════════════════╗"));
    Serial.println(F("║     TAXI Mini Sumo Firmware     ║"));
    Serial.println(F("║     v1.0 — XMotion V3           ║"));
    Serial.println(F("╚══════════════════════════════════╝"));
    #endif
    
    // ---- Critical: Stop motors immediately ----
    // xmotionV3 constructor sets motor pins as outputs, but doesn't set values.
    // StopMotors ensures motors are OFF during initialization.
    xmotion.StopMotors(100);
    
    // ---- Initialize sensors ----
    #if SERIAL_DEBUG
    Serial.println(F("\n--- Sensor Init ---"));
    #endif
    
    m60fInit();     // Start M60F async polling chain (Serial1 @ 250kbaud)
    qreInit();      // Initialize QRE1113 pins
    qreReadAll();   // Prime first readings
    
    // ---- Initialize actuators ----
    #if SERIAL_DEBUG
    Serial.println(F("\n--- Actuator Init ---"));
    #endif
    
    motorsInit();   // Ensure motors stopped
    ledRingInit();  // Initialize WS2812 ring
    
    // ---- Initialize controls ----
    #if SERIAL_DEBUG
    Serial.println(F("\n--- Control Init ---"));
    #endif
    
    // START-STOP module on A0
    pinMode(START_STOP_PIN, INPUT);
    
    // DIP switches (with internal pull-ups)
    tacticPinsInit();
    
    // ---- Initialize IMU (optional — firmware continues if unavailable) ----
    #if SERIAL_DEBUG
    Serial.println(F("\n--- IMU Init ---"));
    #endif
    
    imuInit();
    
    // ---- Initialize state machine ----
    stateMachineInit();
    
    // ---- Startup visual confirmation ----
    ledRingStartupAnimation();
    
    // ---- Display selected strategy ----
    uint8_t strategy = tacticReadDIP();
    #if SERIAL_DEBUG
    Serial.print(F("\n--- READY --- Strategy: "));
    Serial.print(strategy);
    Serial.print(F(" — "));
    Serial.println(tacticRegistry[strategy]->name);
    Serial.println(F("Waiting for START-STOP signal on A0...\n"));
    #endif
    
    // ---- Indicate readiness on LED ring ----
    ledRingFillAll(LED_COLOR_IDLE);
    FastLED.show();
}

// ============================================================================
// MAIN LOOP — Non-blocking, full-speed execution
// ============================================================================

void loop() {
    // ---- 1. Update M60F sensors (async state machine, ~680µs per call) ----
    // Only progresses one step per call — never blocks.
    // Adds new distance data to m60fDistances[] as responses arrive.
    m60fUpdateSensors();
    
    // ---- 2. Read QRE1113 line sensors (~800µs for all 4) ----
    // Double-read pattern handles ATmega32U4 ADC mux settling.
    qreReadAll();
    
    // ---- 3. Read IMU (if enabled, ~200µs I²C) ----
    imuRead();
    
    // ---- 4. Run state machine decision engine ----
    // Evaluates current state, checks sensor inputs, transitions as needed.
    // Sets motor outputs directly (non-blocking).
    stateMachineUpdate();
    
    // ---- 5. Update LED ring visual feedback ----
    // Shows current state, sensor health, attack direction.
    // Uses currentState from StateMachine and pidState from PIDController.
    ledRingUpdate(
        (uint8_t)currentState,
        pidState.opponentDetected ? 
            (pidState.error < -1.0f ? -1 : (pidState.error > 1.0f ? 1 : 0)) 
            : 0,
        m60fHealth
    );
    
    // ---- 6. Debug output (disable for competition to save cycles) ----
    #if SERIAL_DEBUG
    static uint32_t lastDebugMs = 0;
    if (millis() - lastDebugMs > 500) {  // Every 500ms
        lastDebugMs = millis();
        
        // Compact one-line sensor summary
        Serial.print(F("["));
        Serial.print(millis());
        Serial.print(F("ms] State:"));
        
        const char* states[] = {"IDLE","TACTIC","SEARCH","ATTACK","FLANK","EVADE","STOP"};
        Serial.print(states[currentState]);
        
        Serial.print(F(" M60F:"));
        for (uint8_t i = 0; i < M60F_COUNT; i++) {
            if (m60fDistances[i] == M60F_INVALID_DIST) {
                Serial.print(F("--- "));
            } else {
                Serial.print(m60fDistances[i]);
                Serial.print(F("mm "));
            }
        }
        
        Serial.print(F(" QRE:"));
        for (uint8_t i = 0; i < QRE_COUNT; i++) {
            Serial.print(qreValues[i]);
            Serial.print(qreIsEdge(i) ? F("! ") : F("  "));
        }
        
        Serial.print(F(" PID:Kp="));
        Serial.print(pidState.kp, 1);
        Serial.print(F(" Kd="));
        Serial.print(pidState.kd, 1);
        Serial.print(F(" Err="));
        Serial.print(pidState.error, 1);
        
        Serial.print(F(" Health:"));
        for (uint8_t i = 0; i < M60F_COUNT; i++) {
            Serial.print(m60fHealth[i]);
            Serial.print(F(" "));
        }
        
        Serial.println();
    }
    #endif
    
    // ---- TOTAL LOOP TIME: ~1.5–2.0ms ----
    // Well within the 10ms M60F sensor sampling window.
    // No delay() calls anywhere — the loop runs at maximum speed.
}

// ============================================================================
// END OF FIRMWARE
// ============================================================================
// 
// COMPETITION DAY QUICK REFERENCE:
//   1. Charge batteries to 13.05V (LiHV mode)
//   2. Power on — verify LED startup animation plays
//   3. Check Serial monitor for sensor health (if debug enabled)
//   4. Set DIP switches for desired strategy
//   5. Place robot on dohyo, facing correct direction
//   6. Wait for green ring = ready
//   7. START-STOP module will trigger on competition signal
//   8. Between rounds: check tires, blade edge, sensor lenses
//
// TROUBLESHOOTING:
//   - Red LED at position N = M60F sensor N failed health check
//   - Robot spins continuously = QRE threshold too low (recalibrate)
//   - Robot charges off dohyo = QRE threshold too high or sensor detached
//   - Robot doesn't react to opponent = check M60F serial bus wiring
//   - Motors don't move = check battery voltage, XT30 connection
// ============================================================================
