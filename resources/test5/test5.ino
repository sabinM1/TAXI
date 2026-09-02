#include <SingleWireSerial.h>
#include "DistanceSensor.h"

// Single-wire half-duplex bus on digital pin 4 (SIG of all 5 M60F sensors tied
// together here, each with a pull-up resistor to 5V).
#define SENSOR_PIN 4

// Number of sensors on the bus. Their serial ids are 1..5 (not 0..4).
#define SENSOR_COUNT 5

// Serial monitor (USB) baud — this is what you set in the IDE's Serial Monitor.
#define MONITOR_BAUD 115200

// M60F BUS baud. This MUST match the sensors' configured baud (register 0xB0).
// Factory default is 9600. If you reconfigured them (e.g. to 115200 => 0xB0 = 5),
// change this to match, otherwise the sensors will not answer.
#define SENSOR_BAUD 115200

// One shared half-duplex bus; every sensor's SIG line ties onto SENSOR_PIN.
SingleWireSerial oneWire(false);

// One handle per sensor. The device id is the sensor's serial id (0x81),
// which this test expects to already be programmed to 1..5.
DistanceSensor sensors[SENSOR_COUNT] = {
  DistanceSensor(&oneWire, 1),
  DistanceSensor(&oneWire, 2),
  DistanceSensor(&oneWire, 3),
  DistanceSensor(&oneWire, 4),
  DistanceSensor(&oneWire, 5)
};

void setup() {
  pinMode(SENSOR_PIN, INPUT);       // release SIG; pull-up handles idle high

  Serial.begin(MONITOR_BAUD);
  oneWire.begin(SENSOR_BAUD);
  oneWire.setTimeout(2);

  Serial.println(F("=== M60F 5-Sensor Test ==="));
  Serial.print(F("Bus: D4 single-wire @ "));
  Serial.print(SENSOR_BAUD);
  Serial.println(F(" baud, ids 1..5"));
  Serial.print(F("Monitor @ "));
  Serial.print(MONITOR_BAUD);
  Serial.println(F(" baud"));
  Serial.println(F("Sensors 1,2,4,5 = cm | Sensor 3 = mm"));
  Serial.println(F("Output: id | distance | sweep time"));
  Serial.println();

  delay(100);                       // sensor start-up time (10-50ms each)
}

void loop() {
  uint32_t sweepStart = micros();

  // Print happens here (inside the timed region? no — see sweep capture below).
  // We read all sensors first, then print, so print time is not measured.
  uint16_t val[SENSOR_COUNT];

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint8_t id = sensors[i].mDeviceIndex;
    // Every sensor reads cm, except id 3 which reads mm.
    if (id == 3) {
      val[i] = sensors[i].readDistance_mm();
    } else {
      val[i] = sensors[i].readDistance_cm();
    }
  }

  uint32_t sweepUs = micros() - sweepStart;   // pure sensor-poll time

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint8_t id = sensors[i].mDeviceIndex;
    Serial.print(id);
    Serial.print(F("  "));

    if (id == 3) {
      if (val[i] == 0xFFF) {
        Serial.print(F("NO RESPONSE"));
      } else {
        Serial.print(val[i]);
        Serial.print(F("mm"));
      }
    } else {
      if (val[i] == 0xFF) {
        Serial.print(F("NO RESPONSE"));
      } else {
        Serial.print(val[i]);
        Serial.print(F("cm"));
      }
    }
    Serial.println();
  }

  Serial.print(F("---- sweep: "));
  Serial.print(sweepUs);
  Serial.print(F(" us / "));
  Serial.print(sweepUs / 1000.0, 3);
  Serial.println(F(" ms"));

  delay(100);
}
