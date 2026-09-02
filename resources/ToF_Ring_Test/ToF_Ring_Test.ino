/*
 * ToF_Ring_Test.ino
 * -----------------
 * Merged test: read 5x M60F ToF sensors (single-wire bus) and render them on a
 * 7-LED WS2812 ring via RingInfoDisplay.h.
 *
 * No line sensors yet — this only exercises the ToF -> ring path.
 *
 * Wiring:
 *   - M60F SIG line -> D4 (shared half-duplex, pull-up to 5V)
 *   - WS2812 DIN     -> A2 (via 300-500 ohm resistor), 5V -> 5V, GND -> GND
 *
 * Library: FastLED (for the ring) + SingleWireSerial (for the sensor bus).
 *
 * NOTE: sensor ids are 1..5; ID3 (middle) reports mm, the rest report cm.
 * The ring library converts ID3 to cm internally and clamps to the
 * configured 9..800 mm range. No-response (0xFF/0xFFF) blanks that LED.
 */

#include <SingleWireSerial.h>
#include "DistanceSensor.h"
#include "RingInfoDisplay.h"

// Single-wire half-duplex bus on digital pin 4 (SIG of all 5 M60F sensors tied
// together here, each with a pull-up resistor to 5V).
#define SENSOR_PIN 4

// Number of sensors on the bus. Their serial ids are 1..5 (not 0..4).
#define SENSOR_COUNT 5

// Serial monitor (USB) baud.
#define MONITOR_BAUD 115200

// M60F BUS baud. MUST match the sensors' configured baud (register 0xB0).
#define SENSOR_BAUD 115200

// One shared half-duplex bus; every sensor's SIG line ties onto SENSOR_PIN.
SingleWireSerial oneWire(false);

// One handle per sensor. The device id is the sensor's serial id (0x81),
// expected to already be programmed to 1..5.
DistanceSensor sensors[SENSOR_COUNT] = {
  DistanceSensor(&oneWire, 1),
  DistanceSensor(&oneWire, 2),
  DistanceSensor(&oneWire, 3),
  DistanceSensor(&oneWire, 4),
  DistanceSensor(&oneWire, 5)
};

// Raw distances, indexed by ToF id (0..4):
//   [0] ID1 side right (cm), [1] ID2 right front (cm), [2] ID3 middle (mm),
//   [3] ID4 front left (cm), [4] ID5 side left (cm).
uint16_t tofDistances[SENSOR_COUNT];

void setup() {
  pinMode(SENSOR_PIN, INPUT);       // release SIG; pull-up handles idle high

  Serial.begin(MONITOR_BAUD);
  oneWire.begin(SENSOR_BAUD);
  oneWire.setTimeout(2);

  RingInit();                       // init + clear the ring

  Serial.println(F("=== ToF + 7-LED Ring Test ==="));
  Serial.print(F("Bus: D4 single-wire @ "));
  Serial.print(SENSOR_BAUD);
  Serial.println(F(" baud, ids 1..5"));
  Serial.print(F("Ring: A2, "));
  Serial.print(RING_LED_COUNT);
  Serial.println(F(" LEDs"));
  Serial.println(F("Sensors 1,2,4,5 = cm | Sensor 3 = mm"));
  Serial.println(F("Range 9..800 mm; green(far) -> red(close); invalid = LED off"));
  Serial.println();

  delay(100);                       // sensor start-up time (10-50ms each)
}

void loop() {
  // Read all sensors (ID3 in mm, others in cm).
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint8_t id = sensors[i].mDeviceIndex;
    if (id == 3) {
      tofDistances[i] = sensors[i].readDistance_mm();
    } else {
      tofDistances[i] = sensors[i].readDistance_cm();
    }
  }

  // Render distances onto the ring.
  RingShowToFRanges(tofDistances);

  // Print a compact one-line summary.
  Serial.print(F("SR="));
  Serial.print(tofDistances[0]);
  Serial.print(F("cm RF="));
  Serial.print(tofDistances[1]);
  Serial.print(F("cm MID="));
  Serial.print(tofDistances[2]);
  Serial.print(F("mm FL="));
  Serial.print(tofDistances[3]);
  Serial.print(F("cm SL="));
  Serial.print(tofDistances[4]);
  Serial.println(F("cm"));

  delay(100);
}
