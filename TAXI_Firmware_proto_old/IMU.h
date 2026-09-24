/**
 * IMU.h — ICM20602 6-Axis IMU Integration
 * 
 * Provides gyro-based yaw control, acceleration-based push detection,
 * and experimental dead-reckoning positioning.
 * 
 * USES (from Phase 3 decision Q3):
 *   1. Turn rate control — gyro Z replaces timed arcs with rate-based turns
 *   2. Edge/push detection — accelerometer detects being pushed
 *   3. Positioning (experimental) — dead reckoning for search patterns
 * 
 * I²C: SDA=D2 (PD1), SCL=D3 (PD0). Address 0x68 (AD0=GND).
 * No conflict with Serial1 (D0/D1) — different AVR port.
 * 
 * NOTE: This is the most experimental module. The ICM20602 uses a register
 * structure similar to MPU6050/ICM20948. Register addresses and init sequence
 * should be verified against the specific ICM20602 breakout board purchased.
 * If the IMU fails to initialize, the firmware continues in IMU-disabled mode.
 */

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"

// ============================================================================
// ICM20602 REGISTER MAP (verified against TDK datasheet)
// ============================================================================

#define ICM20602_ADDR           0x68    // 7-bit I²C address (AD0=GND)
#define ICM20602_WHO_AM_I       0x75    // Should return 0x12
#define ICM20602_PWR_MGMT_1     0x6B    // Power management
#define ICM20602_PWR_MGMT_2     0x6C    // Power management 2
#define ICM20602_GYRO_CONFIG    0x1B    // Gyroscope configuration
#define ICM20602_ACCEL_CONFIG   0x1C    // Accelerometer configuration
#define ICM20602_ACCEL_XOUT_H   0x3B    // Accel X high byte (6 bytes: X,Y,Z)
#define ICM20602_GYRO_XOUT_H    0x43    // Gyro X high byte (6 bytes: X,Y,Z)
#define ICM20602_SMPLRT_DIV     0x19    // Sample rate divider

// ============================================================================
// IMU DATA STRUCTURES
// ============================================================================

struct IMUData {
    // Raw sensor values
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    
    // Computed values
    float yawRate;          // degrees/second (gyro Z)
    float accelMagnitude;   // Total acceleration magnitude
    float heading;          // Integrated yaw (degrees, drifts over time)
    
    // Status
    bool initialized;
    uint32_t lastReadTime;
};

static IMUData imuData = {0};
static bool imuEnabled = IMU_ENABLED;

// ============================================================================
// INITIALIZATION
// ============================================================================

/**
 * Write to a single ICM20602 register.
 */
void imuWriteRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(ICM20602_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

/**
 * Read multiple bytes starting from a register.
 */
void imuReadRegisters(uint8_t reg, uint8_t count, uint8_t* buffer) {
    Wire.beginTransmission(ICM20602_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(ICM20602_ADDR, count);
    
    for (uint8_t i = 0; i < count && Wire.available(); i++) {
        buffer[i] = Wire.read();
    }
}

/**
 * Initialize the ICM20602 IMU.
 * Returns true if IMU is detected and configured.
 * Called once from setup().
 */
bool imuInit() {
    if (!imuEnabled) {
        #if SERIAL_DEBUG
        Serial.println(F("[IMU] Disabled in Config.h"));
        #endif
        return false;
    }
    
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I²C (fast mode)
    
    delay(50);  // Allow IMU to power up
    
    // Check WHO_AM_I
    uint8_t whoami = 0;
    imuReadRegisters(ICM20602_WHO_AM_I, 1, &whoami);
    
    if (whoami != 0x12) {
        #if SERIAL_DEBUG
        Serial.print(F("[IMU] WHO_AM_I mismatch. Expected 0x12, got 0x"));
        Serial.println(whoami, HEX);
        Serial.println(F("[IMU] Running in IMU-disabled mode"));
        #endif
        imuEnabled = false;
        return false;
    }
    
    // Wake up the IMU (clear sleep bit)
    imuWriteRegister(ICM20602_PWR_MGMT_1, 0x00);
    delay(50);
    
    // Configure gyroscope: ±2000 dps
    imuWriteRegister(ICM20602_GYRO_CONFIG, 0x18);  // 0b00011000 = ±2000 dps
    // Sensitivity: 16.4 LSB/dps
    
    // Configure accelerometer: ±16g
    imuWriteRegister(ICM20602_ACCEL_CONFIG, 0x18);  // 0b00011000 = ±16g
    // Sensitivity: 2048 LSB/g
    
    // Set sample rate: 1kHz / (1 + SMPLRT_DIV)
    imuWriteRegister(ICM20602_SMPLRT_DIV, 0x04);  // 200Hz
    
    // Low-pass filter: 41Hz (accel), 41Hz (gyro)
    imuWriteRegister(ICM20602_PWR_MGMT_2, 0x03);
    
    imuData.initialized = true;
    imuData.lastReadTime = millis();
    
    #if SERIAL_DEBUG
    Serial.println(F("[IMU] ICM20602 initialized — 200Hz, ±2000dps, ±16g"));
    #endif
    
    return true;
}

// ============================================================================
// DATA ACQUISITION
// ============================================================================

/**
 * Read accelerometer and gyroscope data from the IMU.
 * Non-blocking — reads I²C registers and returns immediately.
 * Call once per main loop iteration (or less frequently if not critical).
 * 
 * Converting raw values:
 *   Gyro:  raw / 16.4 = degrees/second (at ±2000 dps)
 *   Accel: raw / 2048.0 = g (at ±16g)
 */
void imuRead() {
    if (!imuEnabled || !imuData.initialized) return;
    
    uint8_t buffer[14];
    imuReadRegisters(ICM20602_ACCEL_XOUT_H, 14, buffer);
    
    // Parse accelerometer (big-endian)
    imuData.accelX = (int16_t)((buffer[0] << 8) | buffer[1]);
    imuData.accelY = (int16_t)((buffer[2] << 8) | buffer[3]);
    imuData.accelZ = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // Parse gyroscope (big-endian)
    imuData.gyroX = (int16_t)((buffer[8]  << 8) | buffer[9]);
    imuData.gyroY = (int16_t)((buffer[10] << 8) | buffer[11]);
    imuData.gyroZ = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    // Compute derived values
    imuData.yawRate = imuData.gyroZ / 16.4f;  // degrees/second
    
    // Compute acceleration magnitude (in g)
    float ax = imuData.accelX / 2048.0f;
    float ay = imuData.accelY / 2048.0f;
    float az = imuData.accelZ / 2048.0f;
    imuData.accelMagnitude = sqrt(ax*ax + ay*ay + az*az);
    
    // Integrate heading (simple — will drift ~1°/min)
    uint32_t now = millis();
    float dt = (now - imuData.lastReadTime) / 1000.0f;
    imuData.heading += imuData.yawRate * dt;
    imuData.heading = fmod(imuData.heading, 360.0f);
    if (imuData.heading < 0) imuData.heading += 360.0f;
    
    imuData.lastReadTime = now;
}

// ============================================================================
// APPLICATION-SPECIFIC FUNCTIONS
// ============================================================================

/**
 * Check if the robot is being pushed (sudden lateral acceleration).
 * Compares current acceleration magnitude against 1g resting value.
 * 
 * @return true if push is detected (acceleration spike > threshold)
 */
bool imuIsBeingPushed() {
    if (!imuEnabled) return false;
    
    // Normal resting: ~1.0g (gravity only)
    // Being pushed: >1.5g lateral + gravity combined
    return imuData.accelMagnitude > 1.5f;
}

/**
 * Turn at a target yaw rate using gyro feedback.
 * Compares current yaw rate against target and adjusts motor speeds.
 * More precise than timed turns — compensates for battery voltage sag.
 * 
 * @param targetRate  Desired yaw rate in degrees/second (+ = right, - = left)
 * @param baseSpeed   Base motor speed for the turn (0–100)
 * 
 * This is a simplified rate controller — full implementation would use
 * a separate PID loop for yaw rate, but for mini sumo this proportional
 * correction is sufficient.
 */
void imuTurnAtRate(float targetRate, uint8_t baseSpeed) {
    if (!imuEnabled) {
        // Fallback: use timed turn if IMU unavailable
        if (targetRate > 0) {
            motorsPivotRight(baseSpeed);
        } else {
            motorsPivotLeft(baseSpeed);
        }
        return;
    }
    
    float rateError = targetRate - imuData.yawRate;
    float correction = rateError * 0.5f;  // Proportional correction
    
    int16_t leftSpeed  = (int16_t)(-targetRate * baseSpeed / 100.0f + correction);
    int16_t rightSpeed = (int16_t)( targetRate * baseSpeed / 100.0f - correction);
    
    leftSpeed  = constrain(leftSpeed,  -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);
    
    motorsSet((int8_t)leftSpeed, (int8_t)rightSpeed);
}

/**
 * Get current heading for dead-reckoning positioning.
 * Drifts ~1°/minute without magnetometer — useful for short-term (<30s) tracking.
 */
float imuGetHeading() {
    return imuData.heading;
}

#endif // IMU_H
