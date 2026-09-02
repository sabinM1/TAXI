#include <SingleWireSerial.h>

// Transparent half-duplex bridge:
//   USB CDC (Serial)  <->  SingleWireSerial (M60F on D4)
// No "TX " prefix, no echo, no filtering — the web tool sends raw M60F
// commands ("R0001\n") and expects raw replies ("12C\n").

SingleWireSerial bus;   // single-wire mode, pin fixed to D4 on Leonardo

void setup() {
  Serial.begin(115200);   // USB CDC to PC (baud is ignored for CDC, cosmetic)
  bus.begin(115200);        // MUST match the M60F serial baud (default 9600)
  bus.setTimeout(10);
}

void loop() {
  // PC -> sensor
  while (Serial.available()) {
    int c = Serial.read();
    if (c >= 0) bus.write((uint8_t)c);
  }

  // sensor -> PC
  while (bus.available()) {
    int c = bus.read();
    if (c >= 0) Serial.write((uint8_t)c);
  }
}
