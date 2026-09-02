#include <SingleWireSerial.h>
#include "DistanceSensor.h"

// Leonardo (ATmega32u4): input capture pin is digital pin 4 (ICP1 = PD4)
#define SENSOR_PIN 4
// Unique id to assign to THIS sensor (0..255). Change per sensor.
#define NEW_ID 0

SingleWireSerial oneWire(false);
DistanceSensor sensor0(&oneWire, 0);   // talks to id 0 during setup

void setup() {
  // Give you time to open the Serial Monitor before anything runs.
  Serial.begin(9600);
  Serial.println("=== M60F Configuration ===");
  Serial.println("Wiring: VIN -> 5V (with jumper), GND -> GND, SIG -> pin4 + 4.7k to 5V.");
  Serial.println("Open Serial Monitor now. Starting in 5s...");
  delay(5000);

  // 0. Release SIG, check whether the sensor is ALREADY in serial mode.
  pinMode(SENSOR_PIN, INPUT);
  oneWire.begin(9600);
  oneWire.setTimeout(10);

  if (sensor0.waitForBoot()) {
    Serial.println("0) Sensor is ALREADY in serial mode (id 0).");
  } else {
    Serial.println("0) Sensor not in serial mode yet. Forcing entry...");

    // Disable the bus so driving SIG low doesn't fire a spurious input capture.
    oneWire.end();

    // 1. Force serial mode: hold SIG low, then YOU power-cycle the sensor.
    pinMode(SENSOR_PIN, OUTPUT);
    digitalWrite(SENSOR_PIN, 0);

    Serial.println();
    Serial.println("1) SIG is held LOW.");
    Serial.println("   NOW disconnect VIN, wait 1s, reconnect it.");
    Serial.println("   Waiting 10s for you to power-cycle...");
    delay(10000);

    pinMode(SENSOR_PIN, INPUT);   // release SIG
    oneWire.begin(9600);
    oneWire.setTimeout(10);

    if (!sensor0.waitForBoot()) {
      Serial.println("Sensor not responding!");
      while(1);
    }
    Serial.println("1) Sensor is in serial mode (id 0).");
  }

  // 2. Reset to factory defaults. The default IO mode is digital, so the sensor
  //    will need a reboot afterward (matching the vendor example).
  if (!sensor0.resetConfiguration()) {
    Serial.println("2) FAILED to reset config!");
    while(1);
  }
  Serial.println("2) Configuration reset to factory defaults.");

  // 3. Reboot the sensor so the reset takes effect (IO mode loads from flash).
  oneWire.end();
  pinMode(SENSOR_PIN, OUTPUT);
  digitalWrite(SENSOR_PIN, 0);

  Serial.println();
  Serial.println("3) NOW disconnect VIN, wait 1s, reconnect it (reset apply).");
  Serial.println("   Waiting 10s for you to power-cycle...");
  delay(10000);

  pinMode(SENSOR_PIN, INPUT);
  oneWire.begin(9600);
  oneWire.setTimeout(10);

  if (!sensor0.waitForBoot()) {
    Serial.println("Sensor not responding after reset!");
    while(1);
  }
  Serial.println("3) Sensor is back in serial mode (id 0).");

  // 4. Set IO mode = Serial (0x80 = 0).
  if (!sensor0.writeRegister(0x80, 0)) {
    Serial.println("4) FAILED to write register 80");
    while(1);
  }
  Serial.println("4) IO mode set to Serial (0x80 = 0).");

  // 5. Assign the unique id (0x81 = NEW_ID).
  if (!sensor0.writeRegister(0x81, NEW_ID)) {
    Serial.println("5) FAILED to write register 81");
    while(1);
  }
  Serial.print("5) Serial id set to ");
  Serial.println(NEW_ID);

  // 6. SAVE to flash. Save against the NEW id first, fall back to id 0.
  DistanceSensor sensorNew(&oneWire, NEW_ID);
  bool saved = sensorNew.saveConfiguration();
  if (!saved) saved = sensor0.saveConfiguration();
  if (!saved) {
    Serial.println("6) FAILED to save configuration!");
    while(1);
  }
  Serial.println("6) Configuration saved to sensor flash.");

  // 7. Reboot to apply the saved IO-mode change.
  oneWire.end();
  pinMode(SENSOR_PIN, OUTPUT);
  digitalWrite(SENSOR_PIN, 0);

  Serial.println();
  Serial.println("7) NOW disconnect VIN, wait 1s, reconnect it (apply save).");
  Serial.println("   (do NOT hold SIG low this time — let it boot from saved config)");
  Serial.println("   Waiting 10s for you to power-cycle...");
  delay(10000);

  pinMode(SENSOR_PIN, INPUT);
  oneWire.begin(9600);
  oneWire.setTimeout(10);

  // 8. Verify.
  DistanceSensor sensorVerify(&oneWire, NEW_ID);
  if (!sensorVerify.waitForBoot()) {
    Serial.println("8) Sensor not responding after power-cycle!");
    Serial.println("   -> It likely reverted to DIGITAL mode. Save did not persist.");
    while(1);
  }

  uint16_t ioMode = 0xFFFF, serialId = 0xFFFF;
  sensorVerify.readRegister(0x80, &ioMode);
  sensorVerify.readRegister(0x81, &serialId);
  Serial.print("8) Verify IO mode (0x80) = "); Serial.println(ioMode, HEX);
  Serial.print("   Verify serial id (0x81) = "); Serial.println(serialId, HEX);

  if (ioMode == 0 && serialId == NEW_ID) {
    Serial.println();
    Serial.println("SUCCESS. Sensor is permanently in serial mode.");
  } else {
    Serial.println();
    Serial.println("WARNING: values don't match. Re-run and check.");
  }
}

void loop() {
  // One-shot configuration; nothing to repeat.
}
