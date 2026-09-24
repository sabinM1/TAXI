/*
 * TOF_Timing_Probe.ino — measure M60F ToF response latency to pick the minimum
 * safe TOF_TIMEOUT_MS for TAXI_Firmware.
 *
 * READ-ONLY: this sketch only ever issues 'R' (read) commands. It does NOT
 * write, save, restart, or reset any sensor — the sensor configuration is
 * left exactly as you programmed it.
 *
 * What it measures:
 *   For each sensor id 1..5 it issues a distance read (id 3 -> reg 0x01 mm,
 *   the rest -> reg 0x06 cm) and times, with micros(), how long it takes from
 *   the start of the command until the FIRST response byte arrives. That gap
 *   is dominated by the sensor's sampling time, and it is the only span the
 *   per-byte SingleWireSerial timeout has to cover — so it is the number we
 *   need. It also reads register 0xB0 once per sensor as a cross-check.
 *
 * Output:
 *   Per-sensor min / avg / max / p99 first-byte latency, the 0xB0 value, and
 *   a final recommended TOF_TIMEOUT_MS = ceil((max_ms + 5) / 5) * 5.
 *
 * Hardware: 5x M60F on D4 (shared half-duplex bus), 115200 baud.
 * Library:  SingleWireSerial (Arduino Library Manager).
 *
 * Runtime: ~300 samples x 5 sensors x ~33 ms ≈ 50 s. Progress prints as it goes.
 */

#include <Arduino.h>
#include <SingleWireSerial.h>

#define PIN_TOF_BUS   4
#define TOF_BUS_BAUD  115200
#define USB_BAUD      115200

#define PROBE_SAMPLES 300         // reads per sensor
#define CEILING_US    200000UL    // 200 ms hard per-read ceiling (never truncate)

// Sampling-time cross-check register. NOTE: the M60F datasheet lists 0xB0 as
// BOTH "serial baud" and "sampling time" (an internal doc typo). On these
// sensors it is very likely the baud setting (5 = 115200). We still print it,
// but the empirical latency is the authoritative number.
#define REG_SAMPLING  0xB0

static SingleWireSerial tofBus(false);

// "HEX" would collide with Arduino's Print.h #define HEX, so we use HEXDIG.
static const char HEXDIG[16] = "0123456789ABCDEF";

// Build "R%02X%02X\n" (6 bytes, no NUL terminator) into buf.
static void buildReadCmd(char* buf, uint8_t id, uint8_t reg) {
  buf[0] = 'R';
  buf[1] = HEXDIG[(id >> 4) & 0xF];
  buf[2] = HEXDIG[id & 0xF];
  buf[3] = HEXDIG[(reg >> 4) & 0xF];
  buf[4] = HEXDIG[reg & 0xF];
  buf[5] = '\n';
}

// SingleWireSerial only exposes write(uint8_t) (its override hides the
// inherited buffer overload), so commands go out one byte at a time.
static void writeCmd(const char* buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) tofBus.write(buf[i]);
}

// Drain the current response line up to and including '\n', so the next
// command starts on a clean byte boundary.
static void drainLine() {
  uint32_t start = micros();
  while ((uint32_t)(micros() - start) < 5000UL) {   // 5 ms >> a 4-hex reply
    if (tofBus.available()) {
      if (tofBus.read() == '\n') return;
    }
  }
}

// Time from command start to the FIRST response byte, in microseconds.
// Sets *ok=false (returns 0) if no byte arrives within CEILING_US.
static uint32_t measureFirstByte(uint8_t id, uint8_t reg, bool* ok) {
  char cmd[6];
  buildReadCmd(cmd, id, reg);

  uint32_t t0 = micros();
  writeCmd(cmd, 6);
  tofBus.flush();                                   // ensure TX done before polling

  while (!tofBus.available()) {
    if ((uint32_t)(micros() - t0) >= CEILING_US) { *ok = false; return 0; }
  }
  uint32_t us = (uint32_t)(micros() - t0);
  drainLine();
  *ok = true;
  return us;
}

// Read a register value (0xB0 cross-check). Returns the decoded value.
static uint16_t readRegValue(uint8_t id, uint8_t reg, bool* ok) {
  char cmd[6];
  buildReadCmd(cmd, id, reg);
  writeCmd(cmd, 6);
  tofBus.flush();

  char resp[4];
  uint8_t len = 0;
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < 100000UL && len < 4) {
    if (tofBus.available()) {
      char c = tofBus.read();
      if (c == '\n') break;
      resp[len++] = c;
    }
  }
  if (len == 0) { *ok = false; return 0; }

  uint16_t v = 0;
  for (uint8_t i = 0; i < len; i++) {
    char c = resp[i];
    uint8_t d = (c <= '9') ? (c - '0') : ((c & 0x1F) + 9);
    v = (uint16_t)(v << 4) | d;
  }
  *ok = true;
  return v;
}

// Distance register per id: id 3 = mm (0x01), others = cm (0x06).
static uint8_t distReg(uint8_t id) { return (id == 3) ? 0x01 : 0x06; }

// Insertion sort ascending (small array, run once per sensor).
static void sortAsc(uint32_t* a, uint16_t n) {
  for (uint16_t i = 1; i < n; i++) {
    uint32_t v = a[i];
    int16_t j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
}

static uint32_t samples[PROBE_SAMPLES];
static bool runDone = false;

void setup() {
  Serial.begin(USB_BAUD);
  while (!Serial && millis() < 5000);

  pinMode(PIN_TOF_BUS, INPUT);
  tofBus.begin(TOF_BUS_BAUD);
  tofBus.setTimeout(CEILING_US / 1000UL);

  delay(500);   // let sensors boot

  Serial.println(F("TOF Timing Probe — read-only, does NOT reprogram sensors"));
  Serial.print(F("Samples per sensor: ")); Serial.println(PROBE_SAMPLES);
  Serial.println(F("Expect ~50 s total. Do not move the robot or block sensors."));
  Serial.println(F("--------------------------------------------------------------"));
}

void loop() {
  if (runDone) return;   // idle after the single pass

  uint32_t globalMaxUs = 0;

  for (uint8_t id = 1; id <= 5; id++) {
    uint8_t reg = distReg(id);
    uint32_t minUs = 0xFFFFFFFFUL, maxUs = 0, sumUs = 0;
    uint16_t okCount = 0;

    Serial.print(F("Sensor id ")); Serial.print(id);
    Serial.print(F(" (reg 0x")); Serial.print(HEXDIG[(reg >> 4) & 0xF]);
    Serial.print(HEXDIG[reg & 0xF]); Serial.print(F(") ... "));

    for (uint16_t i = 0; i < PROBE_SAMPLES; i++) {
      bool ok;
      uint32_t us = measureFirstByte(id, reg, &ok);
      if (ok) {
        samples[okCount] = us;
        if (us < minUs) minUs = us;
        if (us > maxUs) maxUs = us;
        sumUs += us;
        okCount++;
      }
    }

    if (okCount == 0) {
      Serial.println(F("NO RESPONSE"));
      continue;
    }

    sortAsc(samples, okCount);
    uint32_t p99 = samples[(uint16_t)((uint32_t)okCount * 99 / 100)];

    // Cross-check register 0xB0.
    bool b0ok;
    uint16_t b0 = readRegValue(id, REG_SAMPLING, &b0ok);

    Serial.print(okCount); Serial.print('/'); Serial.print(PROBE_SAMPLES);
    Serial.println(F(" responded"));
    Serial.print(F("   min=")); Serial.print(minUs);
    Serial.print(F("us  avg=")); Serial.print(sumUs / okCount);
    Serial.print(F("us  max=")); Serial.print(maxUs);
    Serial.print(F("us  p99=")); Serial.print(p99); Serial.println(F("us"));
    Serial.print(F("   reg 0xB0 = "));
    if (b0ok) { Serial.print(b0); Serial.print(F(" (0x")); Serial.print(HEXDIG[(b0 >> 4) & 0xF]); Serial.print(HEXDIG[b0 & 0xF]); Serial.println(')'); }
    else      { Serial.println(F("no response")); }

    if (maxUs > globalMaxUs) globalMaxUs = maxUs;
  }

  Serial.println(F("--------------------------------------------------------------"));
  uint32_t maxMs = globalMaxUs / 1000UL;
  if ((globalMaxUs % 1000UL) != 0) maxMs++;   // round UP to whole ms
  uint32_t recommended = ((maxMs + 5 + 4) / 5) * 5;   // ceil((max + 5)/5)*5
  Serial.print(F("Overall max first-byte latency: "));
  Serial.print(globalMaxUs); Serial.print(F("us (~")); Serial.print(maxMs); Serial.println(F("ms)"));
  Serial.print(F("Recommended TOF_TIMEOUT_MS = ")); Serial.println(recommended);
  Serial.println(F("DONE — power off and flash TAXI_Firmware."));

  runDone = true;
}
