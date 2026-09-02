/**
 * Sensors.h — M60F ToF Async State Machine & QRE1113 Analog Reader
 * 
 * M60F: Non-blocking round-robin polling of 5 sensors on a shared half-duplex
 *       UART bus. Uses Serial1 (D0=RX, D1=TX) at 250kbaud.
 *       Implements the async state machine from ideas_and_proposal.md.
 * 
 * QRE1113: Fast analog reading with ADC double-sample pattern to avoid
 *          ATmega32U4 mux settling issues.
 * 
 * WHY THIS DESIGN (from Phase 3 decision Q2 = Option A):
 *   - Distance values enable zone-gated attack (closer = faster)
 *   - Distance-adaptive search (tight arcs for near opponent, wide for far)
 *   - Zero CPU blocking — 3.4ms sweep vs 10ms sensor window
 *   - All sensors on one pin pair preserves I/O for IMU and LED ring
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "Config.h"

// ============================================================================
// M60F GLOBAL STATE (volatile — accessed in main loop only, but declared here)
// ============================================================================

// Distance array: distances[0..4] in mm. 0xFFF = invalid/no reading.
volatile uint16_t m60fDistances[M60F_COUNT] = {M60F_INVALID_DIST};

// Health tracking for failsafe degradation
uint8_t m60fHealth[M60F_COUNT] = {HEALTH_WINDOW};

// Async state machine variables
static uint8_t  m60fCurrentSensor = 0;
static char     m60fRxBuf[M60F_RESPONSE_MAX];
static uint8_t  m60fRxLen = 0;
static bool     m60fWaitingResp = false;
static uint32_t m60fRequestTime = 0;

// ============================================================================
// QRE1113 GLOBAL STATE
// ============================================================================

// Last analog readings (0–1023). Higher = darker = more likely edge.
uint16_t qreValues[QRE_COUNT] = {0};

// Debounce counters for edge detection
uint8_t qreEdgeCounter[QRE_COUNT] = {0};

// Calibrated thresholds (set by calibration routine)
uint16_t qreThresholds[QRE_COUNT] = {
    QRE_DEFAULT_THRESHOLD,
    QRE_DEFAULT_THRESHOLD,
    QRE_DEFAULT_THRESHOLD,
    QRE_DEFAULT_THRESHOLD
};

// ============================================================================
// M60F ASYNC STATE MACHINE
// ============================================================================

/**
 * Send a distance request to a specific M60F sensor.
 * Format: "R<id_hex><register_hex>\n" e.g., "R0001\n" = read sensor 0, register 01 (mm).
 * 
 * After sending, discards TX echo (bytes appear on RX due to D0-D1 bridge).
 * Sets waitingResp = true so updateSensors() knows to collect the response.
 */
void m60fRequestDistance(uint8_t sensorId) {
    // Flush any stale RX data
    while (M60F_SERIAL.available()) {
        M60F_SERIAL.read();
    }
    
    // Build command: "R<ID><REG>\n"
    char cmd[8];
    sprintf(cmd, "R%02X%02X\n", sensorId, M60F_REG_DIST_MM);
    
    // Transmit
    M60F_SERIAL.print(cmd);
    
    // Discard TX echo (bridge loops TX back to RX)
    uint32_t tStart = micros();
    uint8_t echoLen = strlen(cmd);
    uint8_t echoGot = 0;
    while (echoGot < echoLen && (micros() - tStart) < 500) {
        if (M60F_SERIAL.available()) {
            M60F_SERIAL.read();
            echoGot++;
        }
    }
    
    // Reset RX buffer and arm timeout
    m60fRxLen = 0;
    m60fWaitingResp = true;
    m60fRequestTime = micros();
}

/**
 * Non-blocking M60F sensor update.
 * Call once per main loop iteration. Manages the round-robin polling chain.
 * 
 * Two modes:
 *   1. WAITING: Collects RX bytes until '\n' or timeout
 *   2. IDLE: Sends next request
 * 
 * On response: parses hex distance, updates m60fDistances[], advances to next sensor.
 * On timeout: stores 0xFFF (invalid), penalizes health counter, advances.
 */
void m60fUpdateSensors() {
    if (!m60fWaitingResp) {
        // IDLE: kick off next request
        m60fRequestDistance(m60fCurrentSensor);
        return;
    }
    
    // WAITING: collect response bytes
    while (M60F_SERIAL.available()) {
        char c = M60F_SERIAL.read();
        
        if (c == '\n') {
            // End of response — parse
            m60fRxBuf[m60fRxLen] = '\0';
            
            if (m60fRxLen > 0) {
                m60fDistances[m60fCurrentSensor] = 
                    (uint16_t)strtol(m60fRxBuf, nullptr, 16);
                
                // Validate: 0xFFF means sensor reported invalid reading
                if (m60fDistances[m60fCurrentSensor] != M60F_INVALID_DIST) {
                    // Successful read — boost health
                    m60fHealth[m60fCurrentSensor] = 
                        min(m60fHealth[m60fCurrentSensor] + HEALTH_INCREMENT, 
                            (uint8_t)HEALTH_WINDOW);
                }
            } else {
                // Empty response = failure
                m60fDistances[m60fCurrentSensor] = M60F_INVALID_DIST;
            }
            
            // Advance to next sensor
            m60fRxLen = 0;
            m60fWaitingResp = false;
            m60fCurrentSensor = (m60fCurrentSensor + 1) % M60F_COUNT;
            
            // Chain immediately to next sensor
            m60fRequestDistance(m60fCurrentSensor);
            return;
        }
        
        // Not newline — buffer the byte
        if (m60fRxLen < M60F_RESPONSE_MAX - 1) {
            m60fRxBuf[m60fRxLen++] = c;
        }
    }
    
    // Timeout guard — if sensor doesn't respond within TIMEOUT_US
    if ((micros() - m60fRequestTime) > M60F_TIMEOUT_US) {
        m60fDistances[m60fCurrentSensor] = M60F_INVALID_DIST;
        
        // Penalize health on timeout
        if (m60fHealth[m60fCurrentSensor] >= HEALTH_DECREMENT) {
            m60fHealth[m60fCurrentSensor] -= HEALTH_DECREMENT;
        } else {
            m60fHealth[m60fCurrentSensor] = 0;
        }
        
        #if SERIAL_DEBUG
        Serial.print(F("[M60F] Timeout sensor "));
        Serial.println(m60fCurrentSensor);
        #endif
        
        // Skip to next sensor
        m60fRxLen = 0;
        m60fWaitingResp = false;
        m60fCurrentSensor = (m60fCurrentSensor + 1) % M60F_COUNT;
        m60fRequestDistance(m60fCurrentSensor);
    }
}

/**
 * Initialize M60F serial bus and kick off the async chain.
 * Called once from setup().
 */
void m60fInit() {
    M60F_SERIAL.begin(M60F_BAUD_RATE);
    
    // Allow sensors to boot (start-up time: 10–50ms per datasheet)
    delay(100);
    
    // Prime the async chain with first request
    m60fRequestDistance(0);
    
    #if SERIAL_DEBUG
    Serial.println(F("[M60F] Initialized at 250000 baud, 5 sensors"));
    #endif
}

/**
 * Check if all M60F sensors are responding.
 * Used during pre-fight calibration/health check.
 * Returns true only when every sensor has returned at least one valid reading.
 */
bool m60fAllHealthy() {
    for (uint8_t i = 0; i < M60F_COUNT; i++) {
        if (m60fHealth[i] < HEALTH_FAIL_THRESHOLD) return false;
    }
    return true;
}

/**
 * Count healthy M60F sensors for degradation level assessment.
 */
uint8_t m60fHealthyCount() {
    uint8_t count = 0;
    for (uint8_t i = 0; i < M60F_COUNT; i++) {
        if (m60fHealth[i] >= HEALTH_FAIL_THRESHOLD) count++;
    }
    return count;
}

// ============================================================================
// QRE1113 ANALOG FAST READ
// ============================================================================

/**
 * Read all 4 QRE1113 line sensors.
 * Uses double-read pattern: first read discards ADC mux settling charge,
 * second read provides the actual value.
 * 
 * Total time: ~800µs for all 4 sensors (2 reads × 4 sensors × ~100µs each).
 * Call once per main loop iteration.
 */
void qreReadAll() {
    const uint8_t pins[QRE_COUNT] = {QRE_FL_PIN, QRE_FR_PIN, QRE_RL_PIN, QRE_RR_PIN};
    
    for (uint8_t i = 0; i < QRE_COUNT; i++) {
        #if QRE_ADC_DISCARD_FIRST
        analogRead(pins[i]);                // Discard — clears mux charge
        #endif
        qreValues[i] = analogRead(pins[i]); // Actual reading (0–1023)
    }
}

/**
 * Check if a specific QRE sensor is detecting the edge.
 * Uses debouncing: value must exceed threshold for QRE_DEBOUNCE_COUNT
 * consecutive reads before reporting true.
 * 
 * @param sensorIndex  0=FL, 1=FR, 2=RL, 3=RR
 * @return true if edge is detected (debounced)
 */
bool qreIsEdge(uint8_t sensorIndex) {
    if (sensorIndex >= QRE_COUNT) return false;
    
    if (qreValues[sensorIndex] > qreThresholds[sensorIndex]) {
        qreEdgeCounter[sensorIndex]++;
        if (qreEdgeCounter[sensorIndex] >= QRE_DEBOUNCE_COUNT) {
            qreEdgeCounter[sensorIndex] = QRE_DEBOUNCE_COUNT; // Clamp
            return true;
        }
    } else {
        qreEdgeCounter[sensorIndex] = 0;
    }
    return false;
}

/**
 * Check if ANY QRE sensor detects the edge.
 * Returns which sensor triggered (0-3) or -1 if none.
 */
int8_t qreAnyEdge() {
    if (qreIsEdge(0)) return 0;  // Front-Left
    if (qreIsEdge(1)) return 1;  // Front-Right
    if (qreIsEdge(2)) return 2;  // Rear-Left
    if (qreIsEdge(3)) return 3;  // Rear-Right
    return -1;                    // No edge
}

/**
 * Check if both front QRE sensors detect edge (robot is fully over line).
 */
bool qreBothFrontEdge() {
    return qreIsEdge(0) && qreIsEdge(1);
}

/**
 * Initialize QRE pins. Called once from setup().
 * Pins default to INPUT (analogRead handles this), but explicit for clarity.
 */
void qreInit() {
    pinMode(QRE_FL_PIN, INPUT);
    pinMode(QRE_FR_PIN, INPUT);
    pinMode(QRE_RL_PIN, INPUT);
    pinMode(QRE_RR_PIN, INPUT);
    
    // Prime the readings
    qreReadAll();
    
    #if SERIAL_DEBUG
    Serial.println(F("[QRE] 4 sensors initialized on A1, A2, A4, A5"));
    #endif
}

// ============================================================================
// CALIBRATION HELPERS
// ============================================================================

/**
 * Run QRE calibration: robot rotates in place, records min/max per sensor,
 * sets threshold at midpoint. Stores in qreThresholds[].
 * 
 * Call this during pre-fight with DIP set to calibration mode (all UP = strategy 7).
 */
void qreCalibrate() {
    uint16_t qreMin[QRE_COUNT] = {1023, 1023, 1023, 1023};
    uint16_t qreMax[QRE_COUNT] = {0, 0, 0, 0};
    
    // Rotate in place for 3 seconds, sampling continuously
    uint32_t tStart = millis();
    while (millis() - tStart < 3000) {
        // Rotate slowly
        xmotion.MotorControl(60, -60);  // Right motor forward, left backward
        
        qreReadAll();
        for (uint8_t i = 0; i < QRE_COUNT; i++) {
            if (qreValues[i] < qreMin[i]) qreMin[i] = qreValues[i];
            if (qreValues[i] > qreMax[i]) qreMax[i] = qreValues[i];
        }
        delay(10);  // Blocking is OK during calibration (not combat)
    }
    
    // Stop
    xmotion.MotorControl(0, 0);
    
    // Set thresholds at 60% between min and max (conservative — avoid false positives)
    for (uint8_t i = 0; i < QRE_COUNT; i++) {
        uint16_t range = qreMax[i] - qreMin[i];
        if (range > 50) {  // Sufficient contrast — valid calibration
            qreThresholds[i] = qreMin[i] + (uint16_t)(range * 0.6f);
        } else {
            // Low contrast — use default
            qreThresholds[i] = QRE_DEFAULT_THRESHOLD;
        }
        
        #if SERIAL_DEBUG
        Serial.print(F("[QRE Cal] Sensor "));
        Serial.print(i);
        Serial.print(F(": min="));
        Serial.print(qreMin[i]);
        Serial.print(F(", max="));
        Serial.print(qreMax[i]);
        Serial.print(F(", thr="));
        Serial.println(qreThresholds[i]);
        #endif
    }
}

#endif // SENSORS_H
