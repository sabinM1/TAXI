/*
 * Bench_Test — the all-in-one bench diagnostic for the TAXI mini sumo.
 *
 * Everything runs on the assembled robot: no sensor has to be unplugged, no
 * header has to be edited, nothing has to be re-flashed between tests. Plug in
 * USB, open the Serial Monitor at 115200, and drive it from the command line.
 *
 * WHY A MENU AND NOT A SCRIPT
 *   A bench test that prints everything at once is unreadable, and a bench test
 *   that does one thing needs re-flashing between questions. This one keeps a
 *   live view you can leave running while you move a target around, plus one-shot
 *   commands for the deep dives. Nothing blocks except the two commands that are
 *   explicitly time-based ('s' and 'b').
 *
 * HOW TO USE IT
 *   'h' prints the menu. 'p' runs the GUIDED black-vs-white tuning procedure
 *   step by step (it tells you what to place where, waits for a key, then runs
 *   the matching measurement). The three commands that answer most questions:
 *
 *     l      LIVE table: every ToF sensor, the line sensors, the motors, the
 *            bearing the MATCH firmware would compute from this exact scene, and
 *            the loop timing. Leave it running and move the target.
 *     n      add the ToF SNR lines to the live table: signal / ambient / sigma /
 *            SPAD / ranging status, and the sensor's own signal threshold.
 *     s      ToF sensitivity stats over a 2 s window, per sensor:
 *            SigMin/SigAvg/SigMax and Max/Thr%. THIS is the black-vs-white test:
 *            park the target at a fixed distance, run 's', swap black for white,
 *            run 's' again, compare Max/Thr%. If Max/Thr% is under 100 %, the
 *            sensor is rejecting its own return and no amount of steering code
 *            can help.
 *
 * THE DISTINCTION THAT MATTERS
 *   "Answered" and "valid" are different failures. A sensor that ANSWERS but
 *   reports a distance of 0xFF/0xFFF has seen the target and rejected the return
 *   (too weak, too noisy, out of range). A sensor that does not answer at all has
 *   a bus or wiring problem. The live table shows both: the Answered row, and
 *   "---" for the distances. Health=5/5 in the match firmware while every value
 *   reads "---" means the second kind of failure, which is why this tool prints
 *   the signal level next to the threshold.
 *
 * SAFETY
 *   - Every motor command is clamped to BENCH_MAX_PCT, for every command, always.
 *   - Motors auto-stop if no command arrives for MOTOR_SAFETY_MS, so unplugging
 *     the USB (or closing the monitor) cannot leave the robot running.
 *   - 'z' is the panic button: stop everything and blank the ring.
 *   - The Start/Stop module is only REPORTED, never obeyed. This tool does not
 *     arm from the start pin and cannot be locked out by it, unlike the match
 *     firmware. Nothing here drives the robot on its own.
 *   - Register writes ('W'/'S'/'Z'/'U') change the sensors' non-volatile config.
 *     'W' alone does not survive a power cycle; 'S' commits it. 'Z' then 'S'
 *     restores factory defaults on that sensor. The defaults are printed under
 *     'c' if you need to get back.
 *
 * WHAT IT DOES NOT DO
 *   - No state machine, no strategies, no DIP-selected opening, no timing that
 *     behaves like a match. This is a measurement instrument; the match firmware
 *     is the thing being measured.
 *   - No ToF writes outside the explicit 'W' / 'S' / 'Z' / 'U' commands.
 *
 * SHARED HEADERS
 *   Config.h / Motor.h / Sensors.h / LineSensor.h / DistanceSensor.h /
 *   RingInfoDisplay.h are copies of the ones next to TAXI_Firmware.ino (a sketch
 *   cannot reliably include files from a sibling folder). This matters MORE here
 *   than anywhere else: the ToF register decoding, the valid-distance window and
 *   the line thresholds must be the firmware's own values, or the bench numbers
 *   do not describe what the robot does on the dohyo. Re-copy them whenever the
 *   firmware's copies change.
 */

#include "Config.h"
#include "Motor.h"
#include "Sensors.h"           // ToF bus + line sensors + the firmware's own readers
#include "RingInfoDisplay.h"

// Forward declaration: the full struct is defined lower down, but Arduino
// inserts its auto-generated function prototypes right after the includes, so
// the type has to be visible here or those prototypes fail to compile.
struct SensorStats;

// ============================================================================
//                             BENCH SETTINGS
// ============================================================================

#define BENCH_MAX_PCT       35   // hard ceiling on EVERY motor command issued by
                                 // this tool, whatever the command asks for.
                                 // Enough to see both wheels turn and to check
                                 // direction; not enough to launch off the bench.
#define MOTOR_SAFETY_MS   5000   // motors auto-stop after this long with no
                                 // command received from the host. Protects
                                 // against a runaway after the USB is unplugged.
#define STATS_WINDOW_MS   2000   // 's' integration window
#define BUS_HEALTH_READS    50   // reads per sensor for 'b'
#define LIVE_DEFAULT_MS    250   // live refresh interval
#define LIVE_MIN_MS         50   // below this the serial cost dominates the loop
#define SERIAL_WAIT_MS    3000   // max wait for a USB host at boot

// ============================================================================
//                               STATE
// ============================================================================

// Per-sensor ToF detail, filled by readSensors() when detail is requested.
static uint16_t tofSignal[TOF_SENSOR_COUNT];
static uint16_t tofAmbient[TOF_SENSOR_COUNT];
static uint16_t tofSigma[TOF_SENSOR_COUNT];
static uint16_t tofSpad[TOF_SENSOR_COUNT];
static uint16_t tofStatus[TOF_SENSOR_COUNT];
static bool     tofAnswered[TOF_SENSOR_COUNT];

// Cached signal threshold (reg 0xB5) per sensor — read at boot and after any
// write, so the live view can show signal-against-threshold without paying a
// config read every sweep.
static uint16_t tofThr[TOF_SENSOR_COUNT];

static uint32_t gSweepUs     = 0;   // duration of the last full sensor sweep
static uint32_t gLoopUs      = 0;   // duration of the last loop() iteration
static uint8_t  gAnswered    = 0;   // sensors that answered the last sweep

static bool     liveOn       = false;
static bool     snrOn        = false;
static uint16_t liveMs       = LIVE_DEFAULT_MS;

static uint32_t lastHostMs   = 0;   // last byte received from the host
static int8_t   motorCmd     = 0;   // 0 = stopped, see cmdMotor()

static char lineBuf[28];
static uint8_t lineLen = 0;

// ============================================================================
//                          PRINTING HELPERS
// ============================================================================

static void sep() {
  Serial.println(F("--------------------------------------------------------------"));
}

/** Left-aligned decimal in a fixed-width column, so every table lines up. */
static void pad(uint32_t v, uint8_t w) {
  Serial.print(v);
  uint8_t d = 1;
  for (uint32_t t = v; t >= 10; t /= 10) d++;
  for (uint8_t i = d; i < w; i++) Serial.print(' ');
}

/** Left-aligned string in a fixed-width column. */
static void padStr(const char* s, uint8_t w) {
  Serial.print(s);
  for (uint8_t i = (uint8_t)strlen(s); i < w; i++) Serial.print(' ');
}

/** "---" padded to the same column width (no valid reading). */
static void padDash(uint8_t w) {
  Serial.print(F("---"));
  for (uint8_t i = 3; i < w; i++) Serial.print(' ');
}

/** Hex with a 0x prefix and a fixed digit count. */
static void hexN(uint16_t v, uint8_t digits) {
  static const char HX[] = "0123456789ABCDEF";
  Serial.print(F("0x"));
  for (int8_t i = digits - 1; i >= 0; i--) Serial.print(HX[(v >> (i * 4)) & 0xF]);
}

/** Hex in a fixed-width column, so a hex row lines up with the decimal rows. */
static void padHex(uint16_t v, uint8_t digits, uint8_t w) {
  hexN(v, digits);
  for (int8_t i = digits + 2; i < (int8_t)w; i++) Serial.print(' ');
}

/** Sensor index -> the short name used in every table. Index order matches the
 *  firmware: [0]=id1 side right, [1]=id2 front right, [2]=id3 middle (mm),
 *  [3]=id4 front left, [4]=id5 side left. */
static const char* sensorName(uint8_t i) {
  switch (i) {
    case 0:  return "SR";
    case 1:  return "FR";
    case 2:  return "MID";
    case 3:  return "FL";
    default: return "SL";
  }
}

/** The register each sensor reports distance in: id 3 is millimetres, the rest
 *  centimetres. Same split the match firmware uses. */
static uint8_t distReg(uint8_t i) {
  return (tofSensors[i].mDeviceIndex == 3) ? 0x01 : 0x06;
}

/** Decode the ranging-status bitfield (register 0x00) in words. Bit 0 set means
 *  the measurement is valid; bits 1..7 are the checks that FAILED. */
static void printStatusBits(uint16_t st) {
  if (st == 0xFF) { Serial.print(F("no response")); return; }
  Serial.print((st & 0x01) ? F("valid") : F("INVALID"));
  if (st & 0x02) Serial.print(F(" +wrap"));
  if (st & 0x04) Serial.print(F(" +phase"));
  if (st & 0x08) Serial.print(F(" +sigMin"));
  if (st & 0x10) Serial.print(F(" +sigThr"));
  if (st & 0x20) Serial.print(F(" +sigma"));
  if (st & 0x40) Serial.print(F(" +distMin"));
  if (st & 0x80) Serial.print(F(" +distMax"));
}

/** Decode the range-check enable bitfield (register 0xB4). */
static void printRangeChecks(uint16_t v) {
  bool any = false;
  if (v & 0x02) { Serial.print(F("wrap"));   any = true; }
  if (v & 0x04) { if (any) Serial.print(','); Serial.print(F("phase"));  any = true; }
  if (v & 0x08) { if (any) Serial.print(','); Serial.print(F("sigMin")); any = true; }
  if (v & 0x10) { if (any) Serial.print(','); Serial.print(F("sigThr")); any = true; }
  if (v & 0x20) { if (any) Serial.print(','); Serial.print(F("sigma"));  any = true; }
  if (v & 0x40) { if (any) Serial.print(','); Serial.print(F("dMin"));   any = true; }
  if (v & 0x80) { if (any) Serial.print(','); Serial.print(F("dMax"));   any = true; }
  if (!any) Serial.print(F("none"));
}

// ============================================================================
//                          SENSOR READING
// ============================================================================

/**
 * Read every ToF sensor into the firmware's own tofMm[]/tofValid[], optionally
 * also pulling the SNR registers.
 *
 * tofMm[]/tofValid[] are filled exactly the way the match firmware fills them
 * (same sentinels, same 9..800 mm window, same mm/cm split), so sensorsOpponent()
 * and sensorsClosestMm() can be called afterwards and the bench agrees with what
 * the robot would have done. That is the point of using the firmware's own
 * validity rule here rather than a copy of it.
 *
 * @param detail  also read status/signal/ambient/sigma/SPAD (5 extra reads per
 *                sensor, so a sweep costs ~6x more). A sensor that does not
 *                answer is skipped entirely, so one dead sensor cannot turn a
 *                sweep into a timeout avalanche.
 */
static void readSensors(bool detail) {
  uint32_t t0 = micros();
  gAnswered = 0;

  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    DistanceSensor& s = tofSensors[i];

    // Zero the detail slots first: readRegister() only writes on success, so a
    // failed read leaves 0 behind instead of the previous sweep's value.
    tofSignal[i] = tofAmbient[i] = tofSigma[i] = tofSpad[i] = tofStatus[i] = 0;

    uint16_t raw = 0;
    tofAnswered[i] = s.readRegister(distReg(i), &raw);

    if (!tofAnswered[i]) {
      tofValid[i] = false;
      tofMm[i]    = 0;
      continue;
    }
    gAnswered++;

    // Normalise to millimetres and apply the firmware's valid window.
    uint16_t mm;
    bool ok;
    if (s.mDeviceIndex == 3) {
      mm = raw;                                   // id 3 already reports millimetres
      ok = (raw != TOF_INVALID_MM) && raw >= TOF_MIN_VALID_MM && raw <= TOF_MAX_VALID_MM;
    } else {
      mm = (uint16_t)raw * 10;                    // the rest report centimetres
      ok = (raw != TOF_INVALID_CM) && mm >= TOF_MIN_VALID_MM && mm <= TOF_MAX_VALID_MM;
    }
    tofValid[i] = ok;
    tofMm[i]    = ok ? mm : 0;

    if (!detail) continue;
    s.readRegister(0x00, &tofStatus[i]);          // ranging status bitfield
    s.readRegister(0x02, &tofSignal[i]);          // signal rate  (the money number)
    s.readRegister(0x03, &tofAmbient[i]);         // ambient rate
    s.readRegister(0x04, &tofSigma[i]);           // measurement std deviation, mm
    s.readRegister(0x05, &tofSpad[i]);            // photon detectors used
  }

  gSweepUs = micros() - t0;
}

/** Refresh the cached signal thresholds (reg 0xB5) for all sensors. */
static void readThresholds() {
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    uint16_t v = 0;
    tofThr[i] = tofSensors[i].readRegister(0xB5, &v) ? v : 0;
  }
}

// ============================================================================
//                            LIVE READOUT
// ============================================================================

/**
 * The live table. Layout deliberately mirrors the match firmware's debug table
 * (same sensor column order, same ToF(mm) header) so the two can be read side by
 * side, plus the two rows the firmware does not print: Answered, and — under 'n'
 * — the SNR detail.
 */
static void printLive() {
  sep();
  Serial.print(F(" t=")); Serial.print(millis());

  // What the MATCH firmware would make of this exact scene.
  int8_t bearing = sensorsOpponent();
  bool ok; uint16_t closest = sensorsClosestMm(&ok);

  Serial.print(F("   Bearing=")); Serial.print(bearing);
  Serial.print(F("   Closest="));
  if (ok) { Serial.print(closest); Serial.print(F("mm")); } else Serial.print(F("---"));
  Serial.print(F("   Answered=")); Serial.print(gAnswered);
  Serial.print('/'); Serial.print(TOF_SENSOR_COUNT);
  Serial.print(F("   Sweep=")); Serial.print(gSweepUs); Serial.println(F("us"));
  sep();

  Serial.println(F(" ToF(mm)     SL      FL      MID     FR      SR"));
  Serial.print(F("             "));
  for (int8_t i = 4; i >= 0; i--) {
    if (tofValid[i]) pad(tofMm[i], 8); else padDash(8);
  }
  Serial.println();

  // Answered != valid. A sensor that answers with the invalid sentinel has seen
  // the target and rejected it; this row is what separates that from a dead bus.
  Serial.print(F(" Answered    "));
  for (int8_t i = 4; i >= 0; i--) pad(tofAnswered[i] ? 1 : 0, 8);
  Serial.println();

  if (snrOn) {
    Serial.print(F(" Signal      ")); for (int8_t i = 4; i >= 0; i--) pad(tofSignal[i], 8);   Serial.println();
    Serial.print(F(" Thr(0xB5)   ")); for (int8_t i = 4; i >= 0; i--) pad(tofThr[i], 8);      Serial.println();
    Serial.print(F(" Ambient     ")); for (int8_t i = 4; i >= 0; i--) pad(tofAmbient[i], 8);  Serial.println();
    Serial.print(F(" Sigma       ")); for (int8_t i = 4; i >= 0; i--) pad(tofSigma[i], 8);    Serial.println();
    Serial.print(F(" SPAD        ")); for (int8_t i = 4; i >= 0; i--) pad(tofSpad[i], 8);     Serial.println();
    Serial.print(F(" Status      "));
    for (int8_t i = 4; i >= 0; i--) {
      if (tofAnswered[i]) padHex(tofStatus[i], 2, 8); else padDash(8);
    }
    Serial.println();
  }
  sep();

  Serial.println(F(" Line        Left(A5)  Back(A4)  Right(A1)"));
  Serial.print(F(" Value       "));
  pad(lineRaw[0], 10); pad(lineRaw[1], 10); Serial.println(lineRaw[2]);
  Serial.print(F(" Edge        "));
  pad(lineLeft, 11); pad(lineRear, 11); Serial.println(lineRight);
  sep();

  int l, r; motorsGetLast(&l, &r);
  Serial.print(F(" Motor       L=")); pad(l, 9);
  Serial.print(F(" R=")); pad(r, 4);
  Serial.print(F("  (bench cap ")); Serial.print(BENCH_MAX_PCT); Serial.println(F("%)"));
  sep();
}

// ============================================================================
//                       ONE-SHOT DIAGNOSTIC COMMANDS
// ============================================================================

/**
 * 's' — ToF sensitivity statistics over STATS_WINDOW_MS.
 *
 * The black-vs-white experiment: hold the target still at a known distance, run
 * 's', swap the target, run 's' again. Max/Thr% is the whole answer — it is the
 * strongest return the sensor saw, as a percentage of the threshold at which it
 * stops believing the measurement. Under 100 % means the sensor is throwing
 * everything away at that distance and no firmware change can recover it.
 */
static void cmdStats() {
  uint32_t sigMin[TOF_SENSOR_COUNT], sigMax[TOF_SENSOR_COUNT], sigSum[TOF_SENSOR_COUNT];
  uint32_t sigCnt[TOF_SENSOR_COUNT];
  uint32_t sgmSum[TOF_SENSOR_COUNT];
  uint16_t dMin[TOF_SENSOR_COUNT], dMax[TOF_SENSOR_COUNT];
  uint16_t validCnt[TOF_SENSOR_COUNT];
  uint16_t sweeps = 0;

  readThresholds();

  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    sigMin[i] = 0xFFFFFFFF; sigMax[i] = 0; sigSum[i] = 0; sigCnt[i] = 0;
    sgmSum[i] = 0; dMin[i] = 0xFFFF; dMax[i] = 0; validCnt[i] = 0;
  }

  Serial.print(F("[stats] sampling ")); Serial.print(STATS_WINDOW_MS);
  Serial.println(F(" ms, hold the target still..."));
  uint32_t t0 = millis();
  while (millis() - t0 < STATS_WINDOW_MS) {
    readSensors(true);
    sweeps++;
    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
      if (!tofAnswered[i]) continue;
      uint16_t sg = tofSignal[i];
      if (sg < sigMin[i]) sigMin[i] = sg;
      if (sg > sigMax[i]) sigMax[i] = sg;
      sigSum[i] += sg;
      sgmSum[i] += tofSigma[i];
      sigCnt[i]++;
      if (tofValid[i]) {
        validCnt[i]++;
        if (tofMm[i] < dMin[i]) dMin[i] = tofMm[i];
        if (tofMm[i] > dMax[i]) dMax[i] = tofMm[i];
      }
    }
  }

  sep();
  Serial.print(F(" [stats] ")); Serial.print(sweeps); Serial.println(F(" sweeps"));
  Serial.print(F("       "));
  padStr("Thr", 7);   padStr("SigMin", 8);  padStr("SigAvg", 8);
  padStr("SigMax", 8); padStr("Max/Thr%", 10); padStr("SigmaAvg", 9);
  Serial.println(F("Valid   Dist mm"));

  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    Serial.print(' ');
    Serial.print(tofSensors[i].mDeviceIndex);
    Serial.print(' ');
    padStr(sensorName(i), 4);

    if (sigCnt[i] == 0) { Serial.println(F("no response to any read")); continue; }

    uint32_t ratio = tofThr[i] ? (sigMax[i] * 100UL) / tofThr[i] : 0;
    pad(tofThr[i], 7);
    pad(sigMin[i], 8);
    pad(sigSum[i] / sigCnt[i], 8);
    pad(sigMax[i], 8);
    pad(ratio, 10);
    pad(sgmSum[i] / sigCnt[i], 9);
    pad(validCnt[i], 8);
    if (validCnt[i]) { Serial.print(dMin[i]); Serial.print(F("..")); Serial.print(dMax[i]); }
    else             { Serial.print(F("---")); }
    Serial.println();
  }
  Serial.println(F(" Id order: 1=SR 2=FR 3=MID 4=FL 5=SL.  Max/Thr% < 100 => sensor"));
  Serial.println(F(" rejects its own return at this distance/target."));
  sep();
}

/**
 * 'b' — bus health. Reads each sensor BUS_HEALTH_READS times and reports the
 * response rate and latency. A failed read costs the full TOF_TIMEOUT_MS, so a
 * dead or intermittent sensor shows up as both a low count and a huge avg.
 */
static void cmdBus() {
  sep();
  Serial.print(F(" [bus] ")); Serial.print(BUS_HEALTH_READS);
  Serial.println(F(" reads per sensor"));
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    uint16_t okCnt = 0, raw = 0;
    uint32_t sumUs = 0, maxUs = 0;
    for (uint16_t n = 0; n < BUS_HEALTH_READS; n++) {
      uint32_t t0 = micros();
      bool ok = tofSensors[i].readRegister(distReg(i), &raw);
      uint32_t dt = micros() - t0;
      sumUs += dt;
      if (dt > maxUs) maxUs = dt;
      if (ok) okCnt++;
    }
    Serial.print(F(" id ")); Serial.print(tofSensors[i].mDeviceIndex);
    Serial.print(' '); Serial.print(sensorName(i));
    pad(okCnt, 4); Serial.print('/'); Serial.print(BUS_HEALTH_READS);
    Serial.print(F("  avg ")); pad(sumUs / BUS_HEALTH_READS, 7);
    Serial.print(F("us  max ")); pad(maxUs, 7); Serial.println(F("us"));
  }
  Serial.print(F(" (a miss costs the full ")); Serial.print(TOF_TIMEOUT_MS);
  Serial.println(F(" ms timeout)"));
  sep();
}

/** Read-only registers 0x00..0x07 for one sensor, decoded. */
static void cmdRegs(uint8_t id) {
  DistanceSensor& s = tofSensors[id - 1];
  uint16_t v = 0;
  sep();
  Serial.print(F(" [regs] sensor ")); Serial.print(id);
  Serial.print(' '); Serial.print(sensorName(id - 1)); Serial.println();
  if (!s.readRegister(0x00, &v)) { Serial.println(F(" no response")); sep(); return; }
  Serial.print(F("   00 Status      ")); hexN(v, 2); Serial.print(F("  ")); printStatusBits(v); Serial.println();
  if (s.readRegister(0x01, &v)) { Serial.print(F("   01 Dist mm     ")); hexN(v, 3); Serial.print(F("  ")); Serial.println(v); }
  if (s.readRegister(0x02, &v)) { Serial.print(F("   02 Signal      ")); hexN(v, 4); Serial.print(F("  ")); Serial.println(v); }
  if (s.readRegister(0x03, &v)) { Serial.print(F("   03 Ambient     ")); hexN(v, 4); Serial.print(F("  ")); Serial.println(v); }
  if (s.readRegister(0x04, &v)) { Serial.print(F("   04 Sigma mm    ")); hexN(v, 4); Serial.print(F("  ")); Serial.println(v); }
  if (s.readRegister(0x05, &v)) { Serial.print(F("   05 SPAD count  ")); hexN(v, 4); Serial.print(F("  ")); Serial.println(v); }
  if (s.readRegister(0x06, &v)) { Serial.print(F("   06 Dist cm     ")); hexN(v, 2); Serial.print(F("  ")); Serial.println(v); }
  if (s.readRegister(0x07, &v)) { Serial.print(F("   07 Detection   ")); hexN(v, 1); Serial.print(F("  ")); Serial.println(v); }
  sep();
}

/** Config registers 0x80..0xBD for one sensor — the same view the SoBotz tool
 *  prints, so the sensor can be verified without leaving this sketch. */
static void cmdConfig(uint8_t id) {
  DistanceSensor& s = tofSensors[id - 1];
  static const struct { uint8_t reg; const char* name; } R[] = {
    { 0x80, "I/O mode" }, { 0x81, "Serial id" },
    { 0x82, "Serial baud" }, { 0x83, "reserved" },
    { 0x84, "reserved" }, { 0xB0, "Sampling ms" },
    { 0xB1, "Offset corr" }, { 0xB2, "Crosstalk corr" },
    { 0xB3, "Linear corr" }, { 0xB4, "Range checks" },
    { 0xB5, "Signal thr" }, { 0xB6, "Sigma thr mm" },
    { 0xB7, "Min dist mm" }, { 0xB8, "Max dist mm" },
    { 0xBA, "Detection mode" }, { 0xBB, "Det invert" },
    { 0xBC, "Det lower mm" }, { 0xBD, "Det upper mm" },
  };

  sep();
  Serial.print(F(" [config] sensor ")); Serial.print(id);
  Serial.print(' '); Serial.print(sensorName(id - 1)); Serial.println();

  uint16_t probe = 0;
  if (!s.readRegister(0xB0, &probe)) { Serial.println(F(" no response")); sep(); return; }

  for (uint8_t i = 0; i < sizeof(R) / sizeof(R[0]); i++) {
    uint16_t v = 0;
    if (!s.readRegister(R[i].reg, &v)) continue;
    Serial.print(F("   ")); hexN(R[i].reg, 2);
    Serial.print(' '); padStr(R[i].name, 14); Serial.print(' ');
    hexN(v, (v > 0xFF) ? 4 : 2);
    Serial.print(F("  ("));
    if (R[i].reg == 0xB4) printRangeChecks(v);
    else                  Serial.print(v);
    Serial.println(')');
  }
  sep();
}

/** 'c' — the config view for all five sensors, one after another. */
static void cmdConfigAll() {
  for (uint8_t id = 1; id <= TOF_SENSOR_COUNT; id++) cmdConfig(id);
}

/** 'i' — pins, wiring assumptions and the two timing numbers that matter. */
static void cmdInfo() {
  sep();
  Serial.println(F(" [info]"));
  Serial.print(F("   ToF D")); Serial.print(PIN_TOF_BUS);
  Serial.print(F(" @ ")); Serial.print(TOF_BUS_BAUD);
  Serial.print(F(" baud, timeout ")); Serial.print(TOF_TIMEOUT_MS);
  Serial.print(F(" ms, ")); Serial.print(TOF_SENSOR_COUNT);
  Serial.println(F(" sensors (id3=mm 0x01, rest=cm 0x06)"));
  Serial.print(F("   valid ")); Serial.print(TOF_MIN_VALID_MM);
  Serial.print(F("..")); Serial.print(TOF_MAX_VALID_MM);
  Serial.println(F(" mm (beyond this reads ---)"));
  Serial.print(F("   motor LM_PWM=D")); Serial.print(PIN_LM_PWM);
  Serial.print(F(" LM_DIR=D")); Serial.print(PIN_LM_DIR);
  Serial.print(F(" RM_PWM=D")); Serial.print(PIN_RM_PWM);
  Serial.print(F(" RM_DIR=D")); Serial.print(PIN_RM_DIR);
  Serial.print(F(" inv L/R=")); Serial.print(LM_INVERTED);
  Serial.print('/'); Serial.println(RM_INVERTED);
  Serial.print(F("   trim LM=")); Serial.print(LM_TRIM_PCT);
  Serial.print(F("% RM=")); Serial.print(RM_TRIM_PCT);
  Serial.print(F("%  bench cap ")); Serial.print(BENCH_MAX_PCT); Serial.println('%');
  Serial.print(F("   line L=A")); Serial.print(PIN_LINE_L - A0);
  Serial.print(F("(")); Serial.print(LINE_LEFT_WHITE_THRESHOLD);
  Serial.print(F(") R=A")); Serial.print(PIN_LINE_R - A0);
  Serial.print(F("(")); Serial.print(LINE_RIGHT_WHITE_THRESHOLD);
  Serial.print(F(") rear=A")); Serial.print(PIN_LINE_B - A0);
  Serial.print(F("(")); Serial.print(LINE_REAR_WHITE_THRESHOLD);
  Serial.println(F(")  edge = raw BELOW thr"));
  Serial.print(F("   ring A2, 7 LEDs  LED1/2=D"));
  Serial.print(PIN_LED1); Serial.print("/D"); Serial.println(PIN_LED2);

  // DIP: the pin reads 1 for the OFF (down) position, so the strategy index is
  // the INVERSE of the raw pins. Printing both makes a wiring/polarity mistake
  // obvious instead of silently changing which opening runs.
  uint8_t d1 = digitalRead(PIN_DIP1), d2 = digitalRead(PIN_DIP2), d3 = digitalRead(PIN_DIP3);
  Serial.print(F("   DIP raw D5,D6,D7="));
  Serial.print(d1); Serial.print(','); Serial.print(d2); Serial.print(','); Serial.print(d3);
  uint8_t idx = ((d1 ? 0 : 1)) | ((d2 ? 0 : 2)) | ((d3 ? 0 : 4));
  Serial.print(F(" -> strategy ")); Serial.println(idx);

  Serial.print(F("   start A0=")); Serial.print(digitalRead(PIN_START));
  Serial.println(F(" (1=FIGHT, 0=armed) - NOT obeyed here"));
  Serial.print(F("   loop ")); Serial.print(gLoopUs);
  Serial.print(F(" us, sweep ")); Serial.print(gSweepUs);
  Serial.println(F(" us"));
  sep();
}

// ============================================================================
//              COMPACT CSV CHARACTERISATION  (ToF tuning support)
// ============================================================================
//
// These commands exist so a tuning session produces a few short, machine-
// readable lines per step instead of a wall of human tables. Every line starts
// with a tag (X / K / A / g) and is comma-separated, so a whole run can be
// pasted back verbatim.
//
//   g <id>                 one CSV line: the config registers 0x80..0xBD
//   x <id> [ms]            one CSV line: signal/sigma/distance stats for ONE
//                          sensor over [ms] (default id 3, 1500 ms). This is the
//                          black-vs-white workhorse: park the target, run 'x 3',
//                          swap black<->white, run 'x 3' again, compare.
//   K <id> <reg> <v..> [w<ms>] [keep]
//                          write <reg> through a list of values, sampling after
//                          each and printing one CSV row per value. Restores the
//                          original value at the end unless 'keep'. This is how
//                          sampling time (B0), signal threshold (B5) and sigma
//                          threshold (B6) get swept in a single command.
//   A <reg> <val> [<reg> <val> ..]
//                          apply the winning register/value pairs to ALL five
//                          sensors and save each one (non-volatile).
//
// CSV columns (after the tag/id/reg/val prefix):
//   samples, ans, valid, sigMin, sigAvg, sigMax, thr, ratio%, ambAvg, sigmaAvg,
//   dMin, dMax, dAvg, dStd, readUsAvg, readUsMax
//     ans    = reads that got a reply (bus health)
//     valid  = readings the sensor's own check AND the firmware window accept
//     thr    = signal threshold reg 0xB5 currently in force
//     ratio% = sigMax as a percentage of thr  (>100 = comfortably detected)
//     dStd   = distance standard deviation in mm over the valid samples
//     readUs = per-read latency on the shared bus (avg / worst case)

#define SWEEP_DEFAULT_MS  800    // 'K' sampling window per value
#define SWEEP_MAX_VALUES  16     // max values in one 'K' sweep

struct SensorStats {
  uint16_t samples;
  uint16_t ans;
  uint16_t valid;
  uint16_t sigMin, sigMax;
  uint32_t sigSum;
  uint32_t ambSum, sigmaSum;
  uint16_t dMin, dMax;
  uint32_t dSum, dSumSq;
  uint32_t readUsSum, readUsMax;
};

/** Sample ONE sensor for windowMs, filling st. Index-based (0..4), matching
 *  tofSensors[] and distReg(). */
static void sampleOne(uint8_t i, uint16_t windowMs, SensorStats& st) {
  DistanceSensor& s = tofSensors[i];
  st.samples = st.ans = st.valid = 0;
  st.sigMin = 0xFFFF; st.sigMax = 0; st.sigSum = 0;
  st.ambSum = st.sigmaSum = 0;
  st.dMin = 0xFFFF; st.dMax = 0; st.dSum = 0; st.dSumSq = 0;
  st.readUsSum = 0; st.readUsMax = 0;

  uint32_t t0 = millis();
  while (millis() - t0 < windowMs) {
    uint32_t r0 = micros();
    uint16_t raw = 0;
    bool ans = s.readRegister(distReg(i), &raw);
    uint32_t dt = micros() - r0;
    st.readUsSum += dt;
    if (dt > st.readUsMax) st.readUsMax = dt;
    st.samples++;
    if (!ans) continue;
    st.ans++;

    // Same measurement, pulled apart: signal is the money number, sigma is the
    // noise, status bit 0 is the sensor's own accept/reject decision.
    uint16_t sig = 0, amb = 0, sgm = 0, status = 0;
    s.readRegister(0x02, &sig);
    s.readRegister(0x03, &amb);
    s.readRegister(0x04, &sgm);
    s.readRegister(0x00, &status);

    if (sig < st.sigMin) st.sigMin = sig;
    if (sig > st.sigMax) st.sigMax = sig;
    st.sigSum += sig;
    st.ambSum += amb;
    st.sigmaSum += sgm;

    uint16_t mm; bool ok;
    if (s.mDeviceIndex == 3) {
      mm = raw;
      ok = (raw != TOF_INVALID_MM) && raw >= TOF_MIN_VALID_MM && raw <= TOF_MAX_VALID_MM;
    } else {
      mm = (uint16_t)raw * 10;
      ok = (raw != TOF_INVALID_CM) && mm >= TOF_MIN_VALID_MM && mm <= TOF_MAX_VALID_MM;
    }
    if (ok && (status & 0x01)) {
      st.valid++;
      if (mm < st.dMin) st.dMin = mm;
      if (mm > st.dMax) st.dMax = mm;
      st.dSum += mm;
      st.dSumSq += (uint32_t)mm * mm;
    }
  }
}

/** Print the shared CSV tail (everything after the tag/id prefix). No newline. */
static void csvBody(uint8_t i, const SensorStats& st) {
  uint16_t thr = tofThr[i];
  uint32_t sigAvg = st.ans   ? st.sigSum   / st.ans   : 0;
  uint32_t ambAvg = st.ans   ? st.ambSum   / st.ans   : 0;
  uint32_t sgmAvg = st.ans   ? st.sigmaSum / st.ans   : 0;
  uint32_t dAvg   = st.valid ? st.dSum     / st.valid : 0;
  uint32_t ratio  = thr ? (st.sigMax * 100UL) / thr : 0;

  // Integer standard deviation of the accepted distances (reliability metric).
  uint32_t dStd = 0;
  if (st.valid > 1) {
    int32_t var = (int32_t)(st.dSumSq / st.valid) - (int32_t)(dAvg * dAvg);
    if (var > 0) dStd = (uint32_t)sqrt((double)var);
  }

  Serial.print(st.samples);                Serial.print(',');
  Serial.print(st.ans);                    Serial.print(',');
  Serial.print(st.valid);                  Serial.print(',');
  Serial.print(st.sigMin == 0xFFFF ? 0 : st.sigMin); Serial.print(',');
  Serial.print(sigAvg);                    Serial.print(',');
  Serial.print(st.sigMax);                 Serial.print(',');
  Serial.print(thr);                       Serial.print(',');
  Serial.print(ratio);                     Serial.print(',');
  Serial.print(ambAvg);                    Serial.print(',');
  Serial.print(sgmAvg);                    Serial.print(',');
  Serial.print(st.valid ? st.dMin : 0);    Serial.print(',');
  Serial.print(st.valid ? st.dMax : 0);    Serial.print(',');
  Serial.print(dAvg);                      Serial.print(',');
  Serial.print(dStd);                      Serial.print(',');
  Serial.print(st.samples ? st.readUsSum / st.samples : 0); Serial.print(',');
  Serial.print(st.readUsMax);
}

static void refreshThr(uint8_t i) {
  uint16_t v = 0;
  tofThr[i] = tofSensors[i].readRegister(0xB5, &v) ? v : 0;
}

/** 'g <id>' - compact config dump, one CSV line. */
static void cmdConfigCsv(uint8_t id) {
  static const uint8_t REGS[] = {
    0x80,0x81,0x82,0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xBA,0xBB,0xBC,0xBD
  };
  DistanceSensor& s = tofSensors[id - 1];
  Serial.println(F("g,id,80,81,82,B0,B1,B2,B3,B4,B5,B6,B7,B8,BA,BB,BC,BD"));
  Serial.print(F("g,")); Serial.print(id);
  for (uint8_t k = 0; k < sizeof(REGS); k++) {
    uint16_t v = 0;
    Serial.print(',');
    if (s.readRegister(REGS[k], &v)) Serial.print(v); else Serial.print('?');
  }
  Serial.println();
}

/** 'x <id> [ms]' - compact stats for ONE sensor. */
static void cmdCsvStats(uint8_t id, uint16_t ms) {
  static bool hdr = false;
  if (!hdr) {
    Serial.println(F("X,id,samples,ans,valid,sigMin,sigAvg,sigMax,thr,ratio%,ambAvg,sigmaAvg,dMin,dMax,dAvg,dStd,readUsAvg,readUsMax"));
    hdr = true;
  }
  refreshThr(id - 1);
  SensorStats st;
  sampleOne(id - 1, ms, st);
  Serial.print(F("X,")); Serial.print(id); Serial.print(',');
  csvBody(id - 1, st);
  Serial.println();
}

/** 'K <id> <reg> <v...> [w<ms>] [keep]' - register sweep. */
static void cmdSweep(uint8_t id, uint8_t reg, const uint16_t* vals, uint8_t n,
                     bool keep, uint16_t ms) {
  DistanceSensor& s = tofSensors[id - 1];
  uint16_t orig = 0;
  bool haveOrig = s.readRegister(reg, &orig);
  Serial.println(F("K,id,reg,val,samples,ans,valid,sigMin,sigAvg,sigMax,thr,ratio%,ambAvg,sigmaAvg,dMin,dMax,dAvg,dStd,readUsAvg,readUsMax"));
  for (uint8_t k = 0; k < n; k++) {
    s.writeRegister(reg, vals[k]);
    delay(200);                              // let the setting take effect + settle
    refreshThr(id - 1);
    SensorStats st;
    sampleOne(id - 1, ms, st);
    Serial.print(F("K,")); Serial.print(id); Serial.print(',');
    hexN(reg, 2); Serial.print(','); Serial.print(vals[k]); Serial.print(',');
    csvBody(id - 1, st);
    Serial.println();
  }
  if (haveOrig && !keep) s.writeRegister(reg, orig);
  refreshThr(id - 1);
  Serial.print(F("K,restore,")); Serial.println(keep ? F("kept") : F("original"));
}

/** 'A <reg> <val> ...' - apply pairs to every sensor and save. */
static void cmdApplyAll(const uint16_t* regs, const uint16_t* vals, uint8_t n) {
  Serial.println(F("A,id,reg,val,write,save"));
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    DistanceSensor& s = tofSensors[i];
    for (uint8_t k = 0; k < n; k++) {
      bool w = s.writeRegister((uint8_t)regs[k], vals[k]);
      Serial.print(F("A,")); Serial.print(s.mDeviceIndex); Serial.print(',');
      hexN(regs[k], 2); Serial.print(','); Serial.print(vals[k]); Serial.print(',');
      Serial.println(w ? F("ok") : F("FAIL"));
    }
    bool sv = s.saveConfiguration();
    Serial.print(F("A,")); Serial.print(s.mDeviceIndex);
    Serial.println(sv ? F(",save,ok") : F(",save,FAIL"));
  }
  readThresholds();
}

// ============================================================================
//                    GUIDED TUNING PROCEDURE  ('p')
// ============================================================================
// A blocking, step-by-step walk-through of the black-vs-white procedure in
// GUIDE.md: it prints what to place where, waits for a key, then runs the
// matching measurement. Nothing is saved to the sensors until the last step,
// so 'q' can abort at any point. Keys: g (or any key) = go, s = skip, q = quit.
// 'g' is advertised because some serial monitors do not send CR/LF on Enter.
//
// It blocks the main loop (like 's' and 'b' already do), so the live view is
// switched off and the motors are stopped on entry.

static bool gQuit;                       // set by gAsk() when the operator quits

static void gFlush() { while (Serial.available()) Serial.read(); }

/** One key, or 0 on timeout. A trailing CR/LF is swallowed so one Enter = one key. */
static char gKey(uint32_t to) {
  uint32_t t0 = millis();
  while (!Serial.available()) if (millis() - t0 > to) return 0;
  char c = (char)Serial.read();
  delay(2);
  while (Serial.available() && (Serial.peek() == '\r' || Serial.peek() == '\n')) Serial.read();
  return c;
}

/** Print a step prompt and wait. Returns true to run the step, false to skip;
 *  sets gQuit on 'q' or timeout. */
static bool gAsk(const __FlashStringHelper* title) {
  sep();
  Serial.print(F("== ")); Serial.println(title);
  Serial.println(F("   [g]=go  s=skip  q=quit"));
  char c = gKey(120000);
  if (c == 0 || c == 'q' || c == 'Q') { gQuit = true; return false; }
  return !(c == 's' || c == 'S');
}

/** Prompt for a number; empty input keeps the default. */
static uint16_t gNum(const __FlashStringHelper* label, uint16_t def) {
  Serial.print(F("   ")); Serial.print(label);
  Serial.print(F(" [")); Serial.print(def); Serial.print(F("]: "));
  char buf[12]; uint8_t n = 0; uint32_t t0 = millis();
  for (;;) {
    if (millis() - t0 > 120000) break;
    if (!Serial.available()) continue;
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') break;   // CR, LF or CRLF all end the entry
    if (n < sizeof(buf) - 1) buf[n++] = c;
  }
  buf[n] = '\0';
  delay(2);
  while (Serial.available() && (Serial.peek() == '\r' || Serial.peek() == '\n')) Serial.read();
  return (n == 0) ? def : (uint16_t)strtol(buf, NULL, 10);
}

static void cmdGuide() {
  motorCmd = 0; motorsStop(); liveOn = false; gFlush(); gQuit = false;

  gAsk(F("TAXI ToF guided tuning. Need black+white plates + ruler. Sensor 3 (MID)."));
  if (gQuit) return;

  if (gAsk(F("Step 0: clear area (>1.5 m). Reads config + bus health.")))
    { cmdConfigCsv(3); cmdBus(); }
  if (gQuit) return;

  static const uint16_t dists[] = {100, 200, 300, 400, 500};
  for (uint8_t d = 0; d < 5; d++) {
    sep();
    Serial.print(F("== Step 1: black/white @ ")); Serial.print(dists[d]); Serial.println(F(" mm"));
    Serial.println(F("   WHITE plate. [g]=go  s=skip  q=quit"));
    char c = gKey(120000);
    if (c == 0 || c == 'q' || c == 'Q') return;
    if (c != 's' && c != 'S') cmdCsvStats(3, 1500);
    Serial.println(F("   Now BLACK, same distance. [g]=go  s=skip  q=quit"));
    c = gKey(120000);
    if (c == 0 || c == 'q' || c == 'Q') return;
    if (c != 's' && c != 'S') cmdCsvStats(3, 1500);
    gFlush();
  }

  if (gAsk(F("Step 2: no targets (>1.5 m). sigMax = noise floor, valid must be 0.")))
    cmdCsvStats(3, 1500);
  if (gQuit) return;

  if (gAsk(F("Step 3: BLACK@300. Sweep sampling B0 = 5..50 ms.")))
    { static const uint16_t v[] = {5,8,10,12,15,20,25,33,40,50}; cmdSweep(3,0xB0,v,10,false,800); }
  if (gQuit) return;

  if (gAsk(F("Step 4: BLACK@300. Sweep signal thr B5 = 100..2000.")))
    { static const uint16_t v[] = {100,200,300,400,600,800,1024,1500,2000}; cmdSweep(3,0xB5,v,9,false,800); }
  if (gQuit) return;

  if (gAsk(F("Step 5 (opt): BLACK@300. Sweep sigma thr B6 = 10..100.")))
    { static const uint16_t v[] = {10,15,20,30,40,60,100}; cmdSweep(3,0xB6,v,7,false,800); }
  if (gQuit) return;

  if (gAsk(F("Step 6 (opt): sweep max range B8 = 500..800 mm.")))
    { static const uint16_t v[] = {500,600,700,800}; cmdSweep(3,0xB8,v,4,false,800); }
  if (gQuit) return;

  if (gAsk(F("Step 7: re-apply winners (W 3 B0/B5/B6/B8), verify BLACK at each distance.")))
    cmdCsvStats(3, 1500);
  if (gQuit) return;

  sep();
  Serial.println(F("== Step 8: apply winners to all 5 + save."));
  Serial.println(F("   Type values; Enter keeps current value."));
  uint16_t c0 = 0, c5 = 0, c6 = 0, c8 = 0;
  tofSensors[2].readRegister(0xB0, &c0);
  tofSensors[2].readRegister(0xB5, &c5);
  tofSensors[2].readRegister(0xB6, &c6);
  tofSensors[2].readRegister(0xB8, &c8);
  uint16_t b0 = gNum(F("B0 sampling ms"), c0);
  uint16_t b5 = gNum(F("B5 signal thr"), c5);
  uint16_t b6 = gNum(F("B6 sigma thr"), c6);
  uint16_t b8 = gNum(F("B8 max dist mm"), c8);
  Serial.print(F("   Apply B0=")); Serial.print(b0); Serial.print(F(" B5=")); Serial.print(b5);
  Serial.print(F(" B6=")); Serial.print(b6); Serial.print(F(" B8=")); Serial.print(b8);
  Serial.println(F(" ?  [g]=yes  q=no"));
  {
    char c = gKey(120000);
    if (c != 0 && c != 'q' && c != 'Q') {
      uint16_t regs[4] = {0xB0, 0xB5, 0xB6, 0xB8};
      uint16_t vals[4] = {b0, b5, b6, b8};
      cmdApplyAll(regs, vals, 4);
    } else {
      Serial.println(F("   (not applied)"));
    }
  }
  sep();
  Serial.println(F("== Done. Set TOF_TIMEOUT_MS > sampling in Config.h."));
  sep();
}

// ============================================================================
//                          ACTUATOR COMMANDS
// ============================================================================

/**
 * 'm <n>' — drive the motors at a fixed, clamped duty until changed.
 * Wheels off the ground (or the robot on blocks) for anything but '1'.
 */
static void cmdMotor(uint8_t n) {
  int l = 0, r = 0;
  switch (n) {
    case 1: l = +BENCH_MAX_PCT; r = 0;               break;   // left forward
    case 2: l = 0;              r = +BENCH_MAX_PCT;  break;   // right forward
    case 3: l = +BENCH_MAX_PCT; r = +BENCH_MAX_PCT;  break;   // both forward
    case 4: l = -BENCH_MAX_PCT; r = 0;               break;   // left reverse
    case 5: l = 0;              r = -BENCH_MAX_PCT;  break;   // right reverse
    case 6: l = -BENCH_MAX_PCT; r = -BENCH_MAX_PCT;  break;   // both reverse
    case 7: l = -BENCH_MAX_PCT; r = +BENCH_MAX_PCT;  break;   // spin left
    case 8: l = +BENCH_MAX_PCT; r = -BENCH_MAX_PCT;  break;   // spin right
    default: n = 0; break;                                     // stop
  }
  motorCmd = (int8_t)n;
  if (n == 0) motorsStop();
  else        motorsSet(l, r);
  Serial.print(F("[motor] "));
  if (n == 0) Serial.println(F("stopped"));
  else { Serial.print(F("L=")); Serial.print(l); Serial.print(F(" R=")); Serial.println(r); }
}

/**
 * 'q <n>' — ring test. 0..6 light the single LED with that index, 7 lights all,
 * 8 blanks. Confirms the LED order that all the firmware's mappings assume:
 * LED0 = centre, 1 = back, 2 = left-back, 3 = left-front, 4 = front,
 * 5 = right-front, 6 = right-back.
 */
static void cmdRing(uint8_t n) {
  RingClear();
  if (n <= 6)      RingSetLEDColor(n, CRGB::White);
  else if (n == 7) RingFill(CRGB::White);
  RingShow();
  Serial.print(F("[ring] "));
  if (n == 7)      Serial.println(F("all white"));
  else if (n <= 6) { Serial.print(F("LED ")); Serial.println(n); }
  else             Serial.println(F("blank"));
}

/** 'z' — panic: everything off. */
static void cmdPanic() {
  motorCmd = 0;
  motorsStop();
  RingClear(); RingShow();
  liveOn = false;
  Serial.println(F("[z] motors stopped, ring blank, live view off"));
}

// ============================================================================
//                            COMMAND PARSER
// ============================================================================

/** Split the next whitespace-delimited token out of *p, or NULL at the end. */
static char* tok(char** p) {
  while (**p == ' ') (*p)++;
  if (**p == '\0') return NULL;
  char* start = *p;
  while (**p && **p != ' ') (*p)++;
  if (**p) { **p = '\0'; (*p)++; }
  return start;
}

static long argDec(char** p, long def) {
  char* t = tok(p);
  return t ? strtol(t, NULL, 10) : def;
}

static long argHex(char** p, long def) {
  char* t = tok(p);
  return t ? strtol(t, NULL, 16) : def;
}

static void printHelp() {
  sep();
  Serial.println(F(" Bench_Test - TAXI mini sumo diagnostics"));
  sep();
  Serial.println(F(" h menu   l live   n SNR   r <ms> rate   i info   z PANIC"));
  Serial.println(F(" p GUIDED black/white tuning procedure (step by step)"));
  Serial.println(F(" s 2s ToF stats   b bus health   1..5 regs 0x00-07   c config all"));
  Serial.println(F(" m <n> motors 0-8 (35% cap)   q <n> ring 0-6 LED,7 all,8 blank"));
  Serial.println(F(" W <id> <regHex> <val>  S <id> save  Z <id> reset  U <id> restart"));
  Serial.println(F("   regs: B0 sampling ms | B4 checks | B5 sig thr | B6 sigma thr"));
  Serial.println(F("         B8 max mm | B1 offset | B2 crosstalk | B3 linear (0..FFF)"));
  Serial.println(F(" CSV: g <id> config | x <id> [ms] stats | K <id> <reg> <v..> sweep"));
  Serial.println(F("      A <reg> <val> .. apply to all + save (cols in GUIDE.md)"));
  sep();
}

static void handleLine(char* s) {
  char* p = s;
  char* verb = tok(&p);
  if (!verb) return;

  switch (verb[0]) {
    case 'h': case 'H': case '?': printHelp(); break;

    case 'p': case 'P': cmdGuide(); break;

    case 'l': case 'L':
      liveOn = !liveOn;
      Serial.print(F("[live] ")); Serial.println(liveOn ? F("on") : F("off"));
      break;

    case 'n': case 'N':
      snrOn = !snrOn;
      Serial.print(F("[snr] ")); Serial.println(snrOn ? F("on") : F("off"));
      break;

    case 'r': case 'R': {
      long ms = argDec(&p, 0);
      if (ms < LIVE_MIN_MS) ms = LIVE_MIN_MS;
      liveMs = (uint16_t)ms;
      Serial.print(F("[rate] ")); Serial.print(liveMs); Serial.println(F(" ms"));
      break;
    }

    case 's': case 'S': {
      // 'S' with an argument is a sensor save; bare 's' is the stats run.
      char* t = tok(&p);
      if (t) {
        uint8_t id = (uint8_t)strtol(t, NULL, 10);
        if (id < 1 || id > TOF_SENSOR_COUNT) { Serial.println(F("ERR: id 1..5")); break; }
        Serial.print(F("[save] sensor "));
        Serial.println(tofSensors[id - 1].saveConfiguration() ? F("ok") : F("FAILED"));
        readThresholds();
      } else {
        cmdStats();
      }
      break;
    }

    case 'b': case 'B': cmdBus(); break;
    case 'c': case 'C': cmdConfigAll(); break;
    case 'i': case 'I': cmdInfo(); break;
    case 'z': case 'Z': {
      // 'Z <id>' is a factory reset; bare 'z' is the panic button.
      char* t = tok(&p);
      if (t && verb[0] == 'Z') {
        uint8_t id = (uint8_t)strtol(t, NULL, 10);
        if (id < 1 || id > TOF_SENSOR_COUNT) { Serial.println(F("ERR: id 1..5")); break; }
        Serial.print(F("[reset] sensor "));
        Serial.print(tofSensors[id - 1].resetConfiguration() ? F("ok") : F("FAILED"));
        Serial.println(F("  (send S <id> to keep it)"));
        readThresholds();
      } else {
        cmdPanic();
      }
      break;
    }

    case 'm': case 'M': cmdMotor((uint8_t)argDec(&p, 0)); break;
    case 'q': case 'Q': cmdRing((uint8_t)argDec(&p, 8)); break;

    case 'W': {
      uint8_t id  = (uint8_t)argDec(&p, 0);
      long reg    = argHex(&p, -1);
      long val    = argDec(&p, -1);
      if (id < 1 || id > TOF_SENSOR_COUNT || reg < 0 || val < 0) {
        Serial.println(F("ERR: W <id> <regHex> <valDec>,  e.g. W 1 B0 33"));
        break;
      }
      if (val > 0xFFF) { Serial.println(F("ERR: value is 0..FFF (12 bits)")); break; }
      Serial.print(F("[write] sensor ")); Serial.print(id);
      Serial.print(F(" reg ")); hexN((uint16_t)reg, 2);
      Serial.print(F(" <- ")); Serial.print(val);
      Serial.println(tofSensors[id - 1].writeRegister((uint8_t)reg, (uint16_t)val)
                     ? F("  ok") : F("  FAILED"));
      readThresholds();
      break;
    }

    case 'U': {
      uint8_t id = (uint8_t)argDec(&p, 0);
      if (id < 1 || id > TOF_SENSOR_COUNT) { Serial.println(F("ERR: id 1..5")); break; }
      Serial.print(F("[restart] sensor "));
      Serial.println(tofSensors[id - 1].restartSensor() ? F("ok") : F("FAILED"));
      break;
    }

    // A bare digit selects the read-only register dump for that sensor id.
    case '0': case '1': case '2': case '3': case '4': case '5': case '6':
    case '7': case '8': case '9': {
      uint8_t id = (uint8_t)strtol(verb, NULL, 10);
      if (id < 1 || id > TOF_SENSOR_COUNT) Serial.println(F("ERR: id 1..5"));
      else cmdRegs(id);
      break;
    }

    // --- compact CSV tuning helpers ----------------------------------------
    case 'g': case 'G': {
      uint8_t id = (uint8_t)argDec(&p, 3);
      if (id < 1 || id > TOF_SENSOR_COUNT) { Serial.println(F("ERR: id 1..5")); break; }
      cmdConfigCsv(id);
      break;
    }

    case 'x': case 'X': {
      uint8_t id = (uint8_t)argDec(&p, 3);
      long ms = argDec(&p, 1500);
      if (id < 1 || id > TOF_SENSOR_COUNT) { Serial.println(F("ERR: id 1..5")); break; }
      if (ms < 200) ms = 200;
      cmdCsvStats(id, (uint16_t)ms);
      break;
    }

    case 'K': {
      uint8_t id = (uint8_t)argDec(&p, 3);
      long reg = argHex(&p, -1);
      if (id < 1 || id > TOF_SENSOR_COUNT || reg < 0) {
        Serial.println(F("ERR: K <id> <regHex> <v...> [w<ms>] [keep]")); break;
      }
      uint16_t vals[SWEEP_MAX_VALUES];
      uint8_t n = 0;
      bool keep = false;
      uint16_t ms = SWEEP_DEFAULT_MS;
      for (;;) {
        char* t = tok(&p);
        if (!t) break;
        if (!strcmp(t, "keep")) { keep = true; continue; }
        if (t[0] == 'w' && t[1]) { ms = (uint16_t)strtol(t + 1, NULL, 10); continue; }
        if (n < SWEEP_MAX_VALUES) vals[n++] = (uint16_t)strtol(t, NULL, 10);
      }
      if (n == 0) { Serial.println(F("ERR: K needs at least one value")); break; }
      if (ms < 100) ms = 100;
      cmdSweep(id, (uint8_t)reg, vals, n, keep, ms);
      break;
    }

    case 'A': {
      uint16_t regs[8], vals[8];
      uint8_t n = 0;
      for (;;) {
        char* rt = tok(&p);
        if (!rt) break;
        char* vt = tok(&p);
        if (!vt || n >= 8) break;
        regs[n] = (uint16_t)strtol(rt, NULL, 16);
        vals[n] = (uint16_t)strtol(vt, NULL, 10);
        n++;
      }
      if (n == 0) { Serial.println(F("ERR: A <regHex> <valDec> [<regHex> <valDec> ...]")); break; }
      cmdApplyAll(regs, vals, n);
      break;
    }

    default:
      Serial.println(F("ERR: unknown command, 'h' for the menu"));
      break;
  }
}

// ============================================================================
//                              SETUP / LOOP
// ============================================================================

void setup() {
  Serial.begin(MONITOR_BAUD);
  // Bounded wait: on battery with no host this must not hang for ever.
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < SERIAL_WAIT_MS) {}

  // Claim the motor pins before anything else moves. Note this does NOT lock the
  // start pin out - see the header.
  motorInit();
  motorsStop();

  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_DIP1, INPUT_PULLUP);
  pinMode(PIN_DIP2, INPUT_PULLUP);
  pinMode(PIN_DIP3, INPUT_PULLUP);

  // sensorsInit() is the firmware's own bring-up: pins, bus baud, timeout, line
  // sensors. Using it (rather than a copy) means the bench exercises the exact
  // bus configuration the match firmware runs, so the latency numbers transfer.
  sensorsInit();

  RingInit();
  RingClear();
  RingShow();

  readThresholds();

  Serial.println();
  Serial.println(F("=== TAXI Bench_Test ==="));
  Serial.print(F("Serial ")); Serial.print(MONITOR_BAUD);
  Serial.println(F("  -  'h' menu, 'l' live, 'p' guided tuning"));
  printHelp();
}

void loop() {
  uint32_t loopStart = micros();

  // --- host input (also the motor-safety heartbeat) -------------------------
  while (Serial.available()) {
    char c = Serial.read();
    lastHostMs = millis();
    if (c == '\n' || c == '\r') {
      if (lineLen) { lineBuf[lineLen] = '\0'; handleLine(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                              // overflow: drop the line
      Serial.println(F("ERR: line too long"));
    }
  }

  // --- motor safety ---------------------------------------------------------
  if (motorCmd != 0 && millis() - lastHostMs > MOTOR_SAFETY_MS) {
    motorCmd = 0;
    motorsStop();
    Serial.println(F("[safety] motors stopped - no host activity"));
  }

  // --- live readout ---------------------------------------------------------
  static uint32_t lastLive = 0;
  if (liveOn && millis() - lastLive >= liveMs) {
    lastLive = millis();
    readSensors(snrOn);          // SNR detail only when it is actually displayed
    printLive();
  }

  gLoopUs = micros() - loopStart;
}
