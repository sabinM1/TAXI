/*
 * TOF_Black_Test — M60F black-target tuning test (sensor id 3, MID).
 *
 * Two things this sketch does that Bench_Test did not:
 *   1. VERIFIES every write by reading the register back.
 *   2. Saves + restarts after each config, because the sensor LATCHES its
 *      configuration at boot (a live write changes the readback but not the
 *      behaviour). This was the bug behind every "the setting does nothing".
 *
 * It also prints the FULL ranging-status byte (which check failed), not just
 * bit 0.
 *
 * Hardware: sensor id 3 on D4, 4.7k pull-up to 5V, Serial Monitor 115200.
 * Target:   matte BLACK plate, perpendicular, filling the cone.
 *
 * Commands:
 *   h              help
 *   c              read + print the full config
 *   s [ms]         sample (default 1500 ms): stats + status histogram
 *   w <reg> <v>    write reg (decimal or 0x.. hex), read back, report
 *   r              restart sensor (reloads registers from saved config)
 *   S              save current config to sensor memory
 *   t              quick sweep of B0 and B5 (no restart)
 *   T              saved sweep of B0 and B5 (write + save + restart)
 *   g <dist>       GRID: every B0 x B5 x B6 combo at <dist> mm, averaged
 *   v <dist>       VERIFY: current config at <dist> mm, averaged
 *   n              EMPTY-SPACE noise sweep: B4 x B5, 10 reps each
 *   Z [tag] [id]   STABILITY sweep: B4 x B5 x B6, raw burst/flicker analysis
 *   P              PER-SENSOR phantom survey: current config, all 5 sensors
 *   M              MIN-DISTANCE (B7) sweep on empty space
 *   B              SAMPLING-TIME (B0) sweep on empty space
 *   A              apply TofConfig.h values to ALL 5 sensors (verify + save)
 *
 * Grid output is CSV, one row per combo:
 *   G,dist,b0,b5,b6,reps,samples,ans,valid,validPct,sigMin,sigAvg,sigMax,
 *   sigmaAvg,ambAvg,dAvg,dMin,dMax,dStd,status
 */

#include <Arduino.h>
#include <string.h>
#include <math.h>
#include <SingleWireSerial.h>
#include "DistanceSensor.h"
#include "TofConfig.h"

// ---- hardware (matches Config.h) -------------------------------------------
#define PIN_TOF_BUS    4
#define TOF_BUS_BAUD   115200
#define TOF_TIMEOUT_MS 15
#define SENSOR_ID      3

// Which sensor the 'n' empty-space noise sweep and the 'Z' stability sweep run
// on. 3 = MID, the reference sensor for this tuning session.
#define NOISE_SENSOR_ID 3

SingleWireSerial bus(false);
DistanceSensor sensor(&bus, SENSOR_ID);

// All five sensors, used by the 'A' apply-to-all command.
DistanceSensor allSensors[5] = {
  DistanceSensor(&bus, 1), DistanceSensor(&bus, 2),
  DistanceSensor(&bus, 3), DistanceSensor(&bus, 4),
  DistanceSensor(&bus, 5)
};

// ---- grid definition -------------------------------------------------------
static const uint16_t GRID_B0[] = {25, 33, 40, 50};
static const uint16_t GRID_B5[] = {0, 20, 40, 60};
static const uint16_t GRID_B6[] = {100, 300};
#define GRID_REPS       3
#define GRID_SAMPLE_MS  500

// ---- structs (declared early: Arduino inserts prototypes above) ------------
struct Stats {
  uint16_t samples, ans, valid, dN;
  uint16_t sigMin, sigMax; uint32_t sigSum;
  uint32_t ambSum, sigmaSum, spadSum;
  uint16_t dMin, dMax; uint32_t dSum, dSumSq;
  uint16_t statusHist[8];
};

struct Acc {
  uint32_t reps;
  uint32_t samples, ans, valid, dN;
  uint32_t sigMin, sigMax, sigSum;
  uint32_t ambSum, sigmaSum, spadSum;
  uint32_t dMin, dMax, dSum, dSumSq;
  uint32_t statusHist[8];
};

// ---- stability-sweep sizing + result ---------------------------------------
// Declared here, with the other structs, because the Arduino preprocessor
// hoists function prototypes above everything: a struct defined next to its
// first use would be referenced by the prototype before it exists.
#define STAB_READS      1200   // raw reads per config (~1.5 s at ~800 reads/s)
#define STAB_MAX_RUNS   64     // run-length histogram buckets
#define STAB_MAX_DIST   16     // distance histogram buckets (25 mm each)
#define STAB_MAX_N      10     // largest consecutive-valid N to evaluate

struct StabResult {
  uint16_t reads, meas, valid, measValid;
  uint16_t runs, longest;
  uint16_t ev[STAB_MAX_N + 1];
  uint16_t dMin, dMax, dMean;
  uint16_t runHist[STAB_MAX_RUNS];
  uint16_t distHist[STAB_MAX_DIST];
};

// ---- helpers ---------------------------------------------------------------
static void hex2(uint8_t v) {
  static const char* D = "0123456789ABCDEF";
  Serial.print(D[(v >> 4) & 0xF]);
  Serial.print(D[v & 0xF]);
}

static uint16_t rd(uint8_t reg) {
  uint16_t v = 0;
  sensor.readRegister(reg, &v);
  return v;
}

static const char* regName(uint8_t r) {
  switch (r) {
    case 0x80: return "IO mode";
    case 0x81: return "serial id";
    case 0x82: return "baud";
    case 0xB0: return "sampling ms";
    case 0xB1: return "offset";
    case 0xB2: return "crosstalk";
    case 0xB3: return "linear";
    case 0xB4: return "checks";
    case 0xB5: return "signal thr";
    case 0xB6: return "sigma thr";
    case 0xB7: return "min mm";
    case 0xB8: return "max mm";
    case 0xBA: return "det mode";
    case 0xBB: return "det invert";
    case 0xBC: return "det lo mm";
    case 0xBD: return "det hi mm";
    default:   return "?";
  }
}

static const char* statusName(uint8_t b) {
  switch (b) {
    case 0: return "valid";
    case 1: return "wrap";
    case 2: return "phase";
    case 3: return "sigMIN";
    case 4: return "sigTHR";
    case 5: return "sigma";
    case 6: return "distMIN";
    case 7: return "distMAX";
    default: return "?";
  }
}

// ---- sampling --------------------------------------------------------------
static void sample(uint16_t windowMs, Stats& st) {
  memset(&st, 0, sizeof(st));
  st.sigMin = 0xFFFF;
  st.dMin   = 0xFFFF;

  uint32_t t0 = millis();
  while (millis() - t0 < windowMs) {
    uint16_t raw = 0, sig = 0, amb = 0, sgm = 0, spad = 0, status = 0;
    st.samples++;
    if (!sensor.readRegister(0x01, &raw)) continue;
    st.ans++;
    sensor.readRegister(0x02, &sig);
    sensor.readRegister(0x03, &amb);
    sensor.readRegister(0x04, &sgm);
    sensor.readRegister(0x05, &spad);
    sensor.readRegister(0x00, &status);

    if (sig < st.sigMin) st.sigMin = sig;
    if (sig > st.sigMax) st.sigMax = sig;
    st.sigSum += sig; st.ambSum += amb; st.sigmaSum += sgm; st.spadSum += spad;

    for (uint8_t b = 0; b < 8; b++)
      if (status & (1 << b)) st.statusHist[b]++;

    if (status & 0x01) {
      st.valid++;
      if (raw != 0xFFF) {
        st.dN++;
        if (raw < st.dMin) st.dMin = raw;
        if (raw > st.dMax) st.dMax = raw;
        st.dSum += raw;
        st.dSumSq += (uint32_t)raw * raw;
      }
    }
  }
}

// ---- raw per-read capture (for the host-side filter simulator) -------------
// Reads the sensor as fast as the bus allows and prints ONE VALUE per read:
//   the distance in mm when the read is valid, or -1 when it is not.
// One line per read, wrapped in RAW-BEGIN / RAW-END. The point is to see not
// just HOW LONG the phantom bursts are but WHAT DISTANCE they report: if the
// phantoms cluster at one distance and the target sits elsewhere, a distance
// window separates them and no time filter is needed.
//
// Usage: point at empty space, send 'X <id> <ms>' (default id 3, 20000 ms).
// 'Y <ms>' captures all five sensors in sequence with NO headers, so the whole
// output can be pasted straight into a file for the host simulator. Paste the
// blocks back; the simulator replays them through the firmware filter logic.
//
// The valid window matches TAXI_Firmware/Config.h (TOF_MIN_VALID_MM..
// TOF_MAX_VALID_MM) so the capture reflects what the firmware would accept.
#define RAW_MIN_VALID_MM 9
#define RAW_MAX_VALID_MM 800

static void rawOne(DistanceSensor& s, uint8_t id, uint16_t ms, bool quiet) {
  if (!quiet) {
    Serial.println();
    Serial.print(F("=== RAW CAPTURE id ")); Serial.print(id);
    Serial.print(F(" @ ")); Serial.print(rd(0xB0));
    Serial.print(F(" ms sampling, ")); Serial.print(ms); Serial.println(F(" ms ==="));
    Serial.println(F("One line per read: distance mm, or -1 if invalid."));
  }
  Serial.print(F("RAW-BEGIN ")); Serial.println(id);

  uint32_t t0 = millis();
  uint16_t n = 0, valid = 0;
  while (millis() - t0 < ms) {
    uint16_t raw = 0, status = 0;
    bool ok = s.readRegister(0x01, &raw) && s.readRegister(0x00, &status);
    bool v = ok && (status & 0x01) && raw != 0xFFF &&
             raw >= RAW_MIN_VALID_MM && raw <= RAW_MAX_VALID_MM;
    if (v) { Serial.println(raw); valid++; }
    else   { Serial.println(F("-1")); }
    n++;
  }
  Serial.println(F("RAW-END"));
  if (!quiet) {
    Serial.print(F("# id=")); Serial.print(id);
    Serial.print(F(" reads=")); Serial.print(n);
    Serial.print(F(" valid=")); Serial.print(valid);
    Serial.print(F(" pct=")); Serial.println(n ? (uint32_t)valid * 100 / n : 0);
    Serial.println(F("=== raw capture done ==="));
  }
}

static void runRaw(uint8_t id, uint16_t ms) {
  if (id < 1 || id > 5) { Serial.println(F("ERR: id 1..5")); return; }
  rawOne(allSensors[id - 1], id, ms, false);
}

static void runRawAll(uint16_t ms) {
  for (uint8_t id = 1; id <= 5; id++) {
    rawOne(allSensors[id - 1], id, ms, true);
    delay(200);
  }
}

static void printStats(const Stats& st) {
  uint32_t sigAvg  = st.ans   ? st.sigSum   / st.ans   : 0;
  uint32_t sgmAvg  = st.ans   ? st.sigmaSum / st.ans   : 0;
  uint32_t spadAvg = st.ans   ? st.spadSum  / st.ans   : 0;
  uint32_t ambAvg  = st.ans   ? st.ambSum   / st.ans   : 0;
  uint32_t dAvg    = st.dN    ? st.dSum     / st.dN    : 0;

  Serial.print(F("   n="));    Serial.print(st.samples);
  Serial.print(F(" ans="));    Serial.print(st.ans);
  Serial.print(F(" valid="));  Serial.print(st.valid);
  Serial.print(F(" sig="));    Serial.print(st.sigMin == 0xFFFF ? 0 : st.sigMin);
  Serial.print('/');           Serial.print(sigAvg);
  Serial.print('/');           Serial.print(st.sigMax);
  Serial.print(F(" spad="));   Serial.print(spadAvg);
  Serial.print(F(" amb="));    Serial.print(ambAvg);
  Serial.print(F(" sigma="));  Serial.print(sgmAvg);
  Serial.print(F(" d="));      Serial.print(dAvg);
  Serial.print(F(" ["));       Serial.print(st.dN ? st.dMin : 0);
  Serial.print('-');           Serial.print(st.dN ? st.dMax : 0);
  Serial.println(']');

  Serial.print(F("   status:"));
  bool any = false;
  for (uint8_t b = 0; b < 8; b++) {
    if (st.statusHist[b]) {
      Serial.print(' '); Serial.print(statusName(b));
      Serial.print('='); Serial.print(st.statusHist[b]);
      any = true;
    }
  }
  if (!any) Serial.print(F(" (none)"));
  Serial.println();
}

// ---- accumulate + print averaged -------------------------------------------
static void accReset(Acc& a) {
  memset(&a, 0, sizeof(a));
  a.sigMin = 0xFFFF;
  a.dMin   = 0xFFFF;
}

static void accAdd(Acc& a, const Stats& st) {
  a.reps++;
  a.samples += st.samples;
  a.ans     += st.ans;
  a.valid   += st.valid;
  a.dN      += st.dN;
  if (st.sigMin < a.sigMin) a.sigMin = st.sigMin;
  if (st.sigMax > a.sigMax) a.sigMax = st.sigMax;
  a.sigSum   += st.sigSum;
  a.ambSum   += st.ambSum;
  a.sigmaSum += st.sigmaSum;
  a.spadSum  += st.spadSum;
  a.dSum     += st.dSum;
  a.dSumSq   += st.dSumSq;
  if (st.dN) {
    if (st.dMin < a.dMin) a.dMin = st.dMin;
    if (st.dMax > a.dMax) a.dMax = st.dMax;
  }
  for (uint8_t b = 0; b < 8; b++) a.statusHist[b] += st.statusHist[b];
}

// Shared CSV body (everything after the tag + config columns).
static void accBody(const Acc& a) {
  uint32_t sigAvg = a.ans ? a.sigSum / a.ans : 0;
  uint32_t sgmAvg = a.ans ? a.sigmaSum / a.ans : 0;
  uint32_t ambAvg = a.ans ? a.ambSum / a.ans : 0;
  uint32_t dAvg   = a.dN ? a.dSum / a.dN : 0;
  uint32_t dStd   = 0;
  if (a.dN > 1) {
    int32_t var = (int32_t)(a.dSumSq / a.dN) - (int32_t)(dAvg * dAvg);
    if (var > 0) dStd = (uint32_t)sqrt((double)var);
  }
  uint32_t validPct = a.samples ? (a.valid * 100UL) / a.samples : 0;

  Serial.print(a.reps);
  Serial.print(','); Serial.print(a.samples);
  Serial.print(','); Serial.print(a.ans);
  Serial.print(','); Serial.print(a.valid);
  Serial.print(','); Serial.print(validPct);
  Serial.print(','); Serial.print(a.sigMin == 0xFFFF ? 0 : a.sigMin);
  Serial.print(','); Serial.print(sigAvg);
  Serial.print(','); Serial.print(a.sigMax);
  Serial.print(','); Serial.print(sgmAvg);
  Serial.print(','); Serial.print(ambAvg);
  Serial.print(','); Serial.print(dAvg);
  Serial.print(','); Serial.print(a.dN ? a.dMin : 0);
  Serial.print(','); Serial.print(a.dN ? a.dMax : 0);
  Serial.print(','); Serial.print(dStd);
  Serial.print(',');
  bool any = false;
  for (uint8_t b = 0; b < 8; b++) {
    if (a.statusHist[b]) { if (any) Serial.print('+'); Serial.print(statusName(b)); any = true; }
  }
  if (!any) Serial.print('-');
  Serial.println();
}

static void accPrint(const Acc& a, uint16_t dist, uint16_t b0, uint16_t b5, uint16_t b6) {
  Serial.print(F("G,")); Serial.print(dist);
  Serial.print(','); Serial.print(b0);
  Serial.print(','); Serial.print(b5);
  Serial.print(','); Serial.print(b6);
  Serial.print(',');
  accBody(a);
}

// ---- config set (write + save + restart) -----------------------------------
static void setConfig(uint16_t b0, uint16_t b5, uint16_t b6) {
  sensor.writeRegister(0xB0, b0);
  sensor.writeRegister(0xB5, b5);
  sensor.writeRegister(0xB6, b6);
  sensor.saveConfiguration();
  sensor.restartSensor();
  delay(400);                       // restart ~50 ms + boot 10-50 ms
}

// ---- grid + verify ---------------------------------------------------------
static void runGrid(uint16_t dist) {
  Serial.println();
  Serial.print(F("=== GRID @ ")); Serial.print(dist); Serial.println(F(" mm black ==="));
  Serial.print(F("B0 x B5 x B6 = "));
  Serial.print(sizeof(GRID_B0)/2); Serial.print('x');
  Serial.print(sizeof(GRID_B5)/2); Serial.print('x');
  Serial.print(sizeof(GRID_B6)/2);
  Serial.print(F(" combos, ")); Serial.print(GRID_REPS);
  Serial.print(F(" reps x ")); Serial.print(GRID_SAMPLE_MS); Serial.println(F(" ms"));
  Serial.println(F("G,dist,b0,b5,b6,reps,samples,ans,valid,validPct,sigMin,sigAvg,sigMax,sigmaAvg,ambAvg,dAvg,dMin,dMax,dStd,status"));

  for (uint8_t i = 0; i < sizeof(GRID_B0)/2; i++)
  for (uint8_t j = 0; j < sizeof(GRID_B5)/2; j++)
  for (uint8_t k = 0; k < sizeof(GRID_B6)/2; k++) {
    uint16_t b0 = GRID_B0[i], b5 = GRID_B5[j], b6 = GRID_B6[k];
    setConfig(b0, b5, b6);
    Acc acc; accReset(acc);
    for (uint8_t r = 0; r < GRID_REPS; r++) {
      Stats st; sample(GRID_SAMPLE_MS, st); accAdd(acc, st);
    }
    accPrint(acc, dist, b0, b5, b6);
  }
  Serial.println(F("=== grid done ==="));
}

// ---- B2 crosstalk sweep at a fixed distance (black target) ------------------
// Sweeps B2 (crosstalk correction) finely while the BLACK target sits at a
// fixed distance, saved+restarted so each value takes effect. The point is to
// find whether ANY B2 keeps the black target alive while trimming phantoms.
// Watch 'valid' (want high) and 'sigAvg' (the black return).
static const uint16_t B2_SWEEP[] = { 0, 10, 20, 30, 40, 50, 75, 100, 150, 200 };
#define B2_REPS      5
#define B2_SAMPLE_MS 400

static void runB2(uint16_t dist) {
  uint16_t b0 = rd(0xB0), b4 = rd(0xB4), b5 = rd(0xB5), b6 = rd(0xB6);
  uint16_t o2 = rd(0xB2);
  Serial.println();
  Serial.print(F("=== B2 CROSSTALK SWEEP @ ")); Serial.print(dist); Serial.println(F(" mm black ==="));
  Serial.println(F("Put the BLACK plate at that distance, perpendicular."));
  Serial.print(F("B2 = ")); Serial.print(sizeof(B2_SWEEP)/2);
  Serial.print(F(" values, ")); Serial.print(B2_REPS);
  Serial.print(F(" reps x ")); Serial.print(B2_SAMPLE_MS);
  Serial.print(F(" ms, B0=")); Serial.print(b0);
  Serial.print(F(" B4=")); Serial.print(b4);
  Serial.print(F(" B5=")); Serial.print(b5);
  Serial.print(F(" B6=")); Serial.println(b6);
  Serial.println(F("D,dist,b2,reps,samples,ans,valid,validPct,sigMin,sigAvg,sigMax,sigmaAvg,ambAvg,dAvg,dMin,dMax,dStd,status"));

  for (uint8_t i = 0; i < sizeof(B2_SWEEP)/2; i++) {
    uint16_t b2 = B2_SWEEP[i];
    sensor.writeRegister(0xB2, b2);
    sensor.saveConfiguration();
    sensor.restartSensor();
    delay(400);
    Acc acc; accReset(acc);
    for (uint8_t r = 0; r < B2_REPS; r++) {
      Stats st; sample(B2_SAMPLE_MS, st); accAdd(acc, st);
    }
    Serial.print(F("D,")); Serial.print(dist);
    Serial.print(','); Serial.print(b2);
    Serial.print(',');
    accBody(acc);
  }

  sensor.writeRegister(0xB2, o2);
  sensor.saveConfiguration();
  sensor.restartSensor();
  delay(400);
  Serial.print(F("  (restored B2=")); Serial.print(o2); Serial.println(')');
  Serial.println(F("=== B2 sweep done ==="));
}

static void runVerify(uint16_t dist) {
  Serial.println();
  Serial.print(F("=== VERIFY current config @ ")); Serial.print(dist); Serial.println(F(" mm black ==="));
  Serial.println(F("G,dist,b0,b5,b6,reps,samples,ans,valid,validPct,sigMin,sigAvg,sigMax,sigmaAvg,ambAvg,dAvg,dMin,dMax,dStd,status"));
  uint16_t b0 = rd(0xB0), b5 = rd(0xB5), b6 = rd(0xB6);
  Acc acc; accReset(acc);
  for (uint8_t r = 0; r < GRID_REPS; r++) {
    Stats st; sample(GRID_SAMPLE_MS, st); accAdd(acc, st);
  }
  accPrint(acc, dist, b0, b5, b6);
  Serial.println(F("=== verify done ==="));
}

// ---- empty-space noise-floor sweep -----------------------------------------
// Sweeps B4 (checks) x B5 (signal threshold) at the CURRENT B0/B6, averaging
// NOISE_REPS samples per config. Point the sensor at >1.5 m of EMPTY SPACE
// first (no target). A config is safe only if valid == 0 across every rep.
static const uint16_t NOISE_B4[] = { 248, 254 };
static const uint16_t NOISE_B5[] = { 0, 5, 10, 15, 20, 25, 30, 40, 60 };
#define NOISE_REPS      10
#define NOISE_SAMPLE_MS 400

static void runNoise() {
  DistanceSensor& s = allSensors[NOISE_SENSOR_ID - 1];
  uint16_t b0 = rd(0xB0), b6 = rd(0xB6);
  Serial.println();
  Serial.println(F("=== EMPTY-SPACE NOISE FLOOR ==="));
  Serial.println(F("Point the sensor at >1.5 m of empty space (no target)."));
  Serial.print(F("sensor id ")); Serial.println(NOISE_SENSOR_ID);
  Serial.print(F("B4 x B5 = "));
  Serial.print(sizeof(NOISE_B4)/2); Serial.print('x');
  Serial.print(sizeof(NOISE_B5)/2);
  Serial.print(F(" combos, ")); Serial.print(NOISE_REPS);
  Serial.print(F(" reps x ")); Serial.print(NOISE_SAMPLE_MS);
  Serial.print(F(" ms, B0=")); Serial.print(b0);
  Serial.print(F(" B6=")); Serial.println(b6);
  Serial.println(F("N,b4,b5,reps,samples,ans,valid,validPct,sigMin,sigAvg,sigMax,sigmaAvg,ambAvg,dAvg,dMin,dMax,dStd,status"));

  for (uint8_t i = 0; i < sizeof(NOISE_B4)/2; i++)
  for (uint8_t j = 0; j < sizeof(NOISE_B5)/2; j++) {
    uint16_t b4 = NOISE_B4[i], b5 = NOISE_B5[j];
    s.writeRegister(0xB4, b4);
    s.writeRegister(0xB5, b5);
    s.saveConfiguration();
    s.restartSensor();
    delay(400);
    Acc acc; accReset(acc);
    for (uint8_t r = 0; r < NOISE_REPS; r++) {
      Stats st; sample(NOISE_SAMPLE_MS, st); accAdd(acc, st);
    }
    Serial.print(F("N,")); Serial.print(b4);
    Serial.print(','); Serial.print(b5);
    Serial.print(',');
    accBody(acc);
  }
  Serial.println(F("=== noise sweep done ==="));
}

// ---- B1/B2 correction sweep (empty space) ----------------------------------
// Sweeps B1 (offset) x B2 (crosstalk) at the CURRENT B0/B4/B5/B6, saved and
// restarted so each value actually takes effect. Point at >1.5 m of EMPTY SPACE.
// Crosstalk (B2) cancels internal reflection from the sensor window, a known
// phantom source; offset (B1) shifts the reported distance. Watch 'valid': a
// value that drops it toward 0 is cancelling the phantoms.
static const uint16_t CORR_B1[] = { 0, 100, 300, 600 };
static const uint16_t CORR_B2[] = { 0, 50, 100, 200, 400 };
#define CORR_REPS      6
#define CORR_SAMPLE_MS 400

static void runCorr() {
  uint16_t b0 = rd(0xB0), b4 = rd(0xB4), b5 = rd(0xB5), b6 = rd(0xB6);
  uint16_t o1 = rd(0xB1), o2 = rd(0xB2);
  Serial.println();
  Serial.println(F("=== B1/B2 CORRECTION SWEEP (empty space) ==="));
  Serial.println(F("Point the sensor at >1.5 m of empty space (no target)."));
  Serial.print(F("B1 x B2 = "));
  Serial.print(sizeof(CORR_B1)/2); Serial.print('x');
  Serial.print(sizeof(CORR_B2)/2);
  Serial.print(F(" combos, ")); Serial.print(CORR_REPS);
  Serial.print(F(" reps x ")); Serial.print(CORR_SAMPLE_MS);
  Serial.print(F(" ms, B0=")); Serial.print(b0);
  Serial.print(F(" B4=")); Serial.print(b4);
  Serial.print(F(" B5=")); Serial.print(b5);
  Serial.print(F(" B6=")); Serial.println(b6);
  Serial.println(F("C,b1,b2,reps,samples,ans,valid,validPct,sigMin,sigAvg,sigMax,sigmaAvg,ambAvg,dAvg,dMin,dMax,dStd,status"));

  for (uint8_t i = 0; i < sizeof(CORR_B1)/2; i++)
  for (uint8_t j = 0; j < sizeof(CORR_B2)/2; j++) {
    uint16_t b1 = CORR_B1[i], b2 = CORR_B2[j];
    sensor.writeRegister(0xB1, b1);
    sensor.writeRegister(0xB2, b2);
    sensor.saveConfiguration();
    sensor.restartSensor();
    delay(400);
    Acc acc; accReset(acc);
    for (uint8_t r = 0; r < CORR_REPS; r++) {
      Stats st; sample(CORR_SAMPLE_MS, st); accAdd(acc, st);
    }
    Serial.print(F("C,")); Serial.print(b1);
    Serial.print(','); Serial.print(b2);
    Serial.print(',');
    accBody(acc);
  }

  sensor.writeRegister(0xB1, o1);
  sensor.writeRegister(0xB2, o2);
  sensor.saveConfiguration();
  sensor.restartSensor();
  delay(400);
  Serial.print(F("  (restored B1=")); Serial.print(o1);
  Serial.print(F(" B2=")); Serial.print(o2); Serial.println(')');
  Serial.println(F("=== correction sweep done ==="));
}

// ---- stability sweep (the hysteresis / flicker test) -----------------------
// The averaged sweeps ('g', 'v', 'n') answer "how many reads were valid". They
// CANNOT answer the question that actually matters for the phantom problem:
// are the valid reads a SUSTAINED run (a real target) or a flicker (a phantom)?
//
// This routine captures the RAW per-read stream and reports its structure:
//   * valid-run length histogram  -> burst shape (flicker vs sustained)
//   * distance histogram          -> where the phantoms sit
//   * consecutive-valid filter    -> phantom events surviving at N = 1..10
//
// It runs that analysis for EVERY config in the sweep, so one 'Z' answers
// "which config turns the flicker into either nothing or a solid run".
//
// Point the sensor at EMPTY SPACE for the phantom test, or at the BLACK plate
// for the detection test. The output is one CSV row per config:
//   Z,tag,b4,b5,b6,reads,meas,valid,validPct,runs,longest,ev1,ev3,ev5,ev10,dMin,dMax,dMean
// where `meas` is the number of DISTINCT measurements (reads collapse to this),
// validPct is the share of MEASUREMENTS that were valid, and evN = phantom
// EVENTS surviving a consecutive-valid filter of N measurements.
// A good empty-space config has validPct low AND ev3 == 0.
// A good black-target config has validPct high AND longest large.
//
// The sizing constants and the StabResult struct live near the top of the file
// with the other structs (see the note there).

// One config's raw capture + analysis, split into a MEASURE step (fills a
// result struct) and PRINT steps, so the same analysis is reused by the
// per-sensor survey ('P') and the single-register sweeps ('M', 'B') without
// duplicating the capture loop.
//
// Everything is computed in a SINGLE streaming pass — no per-read buffer. The
// ATmega32U4 only has 2 KB of RAM and the sketch already uses most of it, so
// storing 1200 reads (or even 1200 bits) is not affordable. The consecutive-
// valid filter is evaluated for all N at once by keeping one streak counter
// per N and one "published" flag per N.
//
// CRITICAL: the analysis counts distinct MEASUREMENTS, not polls. The bus is
// polled far faster than the sensor refreshes (B0 ms), so the SAME measurement
// is read ~27 times in a row. Counting those repeats as separate valid reads is
// what made the old output useless: a single-measurement phantom looked like a
// 27-long sustained run and survived every N. A repeat of the previous raw
// value is recognised and skipped, exactly like the firmware's tofReadOne().
static void stabMeasure(DistanceSensor& s, StabResult& r) {
  memset(&r, 0, sizeof(r));
  r.dMin = 0xFFFF;

  uint16_t streak[STAB_MAX_N + 1];   // per-N consecutive-valid counter
  bool     pub[STAB_MAX_N + 1];      // per-N published flag
  for (uint8_t n = 0; n <= STAB_MAX_N; n++) { streak[n] = 0; pub[n] = false; }

  uint16_t curRun = 0;
  uint32_t dSum = 0;
  uint16_t lastRaw = 0; bool haveLast = false;
  uint32_t t0 = millis();

  while (r.reads < STAB_READS) {
    uint16_t raw = 0, status = 0;
    bool ok = s.readRegister(0x01, &raw) && s.readRegister(0x00, &status);
    bool v = ok && (status & 0x01) && raw != 0xFFF &&
             raw >= RAW_MIN_VALID_MM && raw <= RAW_MAX_VALID_MM;
    r.reads++;

    // Skip a repeat of the previous raw value: it is the SAME measurement, not
    // a new one. This is the whole point of the routine.
    bool newMeas = !haveLast || (raw != lastRaw);
    lastRaw = raw; haveLast = true;
    if (!newMeas) {
      if (millis() - t0 > 8000) break;
      continue;
    }
    r.meas++;

    if (v) {
      r.valid++;
      r.measValid++;
      curRun++;
      dSum += raw;
      if (raw < r.dMin) r.dMin = raw;
      if (raw > r.dMax) r.dMax = raw;
      uint8_t bucket = raw / 25;
      if (bucket >= STAB_MAX_DIST) bucket = STAB_MAX_DIST - 1;
      r.distHist[bucket]++;
    } else {
      if (curRun) {
        r.runs++;
        if (curRun > r.longest) r.longest = curRun;
        uint8_t bucket = (curRun >= STAB_MAX_RUNS) ? (STAB_MAX_RUNS - 1) : (uint8_t)curRun;
        r.runHist[bucket]++;
        curRun = 0;
      }
    }

    // Consecutive-valid filter, all N in one pass, over MEASUREMENTS.
    for (uint8_t n = 1; n <= STAB_MAX_N; n++) {
      if (v) { if (streak[n] < n) streak[n]++; }
      else streak[n] = 0;
      bool p = (streak[n] >= n);
      if (p && !pub[n]) r.ev[n]++;
      pub[n] = p;
    }

    // Safety: never spin longer than 8 s even if the bus stalls.
    if (millis() - t0 > 8000) break;
  }

  // Flush a run that was still open when the capture ended.
  if (curRun) {
    r.runs++;
    if (curRun > r.longest) r.longest = curRun;
    uint8_t bucket = (curRun >= STAB_MAX_RUNS) ? (STAB_MAX_RUNS - 1) : (uint8_t)curRun;
    r.runHist[bucket]++;
  }

  r.dMean = r.valid ? dSum / r.valid : 0;
}

// Full CSV row + the burst/distance histograms (the 'Z' sweep).
static void stabPrintCsv(const char* tag, uint16_t b4, uint16_t b5, uint16_t b6,
                         const StabResult& r, bool verbose) {
  Serial.print(F("Z,")); Serial.print(tag);
  Serial.print(','); Serial.print(b4);
  Serial.print(','); Serial.print(b5);
  Serial.print(','); Serial.print(b6);
  Serial.print(','); Serial.print(r.reads);
  Serial.print(','); Serial.print(r.meas);
  Serial.print(','); Serial.print(r.valid);
  Serial.print(','); Serial.print(r.meas ? (uint32_t)r.measValid * 100 / r.meas : 0);
  Serial.print(','); Serial.print(r.runs);
  Serial.print(','); Serial.print(r.longest);
  Serial.print(','); Serial.print(r.ev[1]);
  Serial.print(','); Serial.print(r.ev[3]);
  Serial.print(','); Serial.print(r.ev[5]);
  Serial.print(','); Serial.print(r.ev[10]);
  Serial.print(','); Serial.print(r.valid ? r.dMin : 0);
  Serial.print(','); Serial.print(r.valid ? r.dMax : 0);
  Serial.print(','); Serial.print(r.dMean);
  Serial.println();

  if (verbose && r.runs) {
    Serial.print(F("#   runs:"));
    for (uint8_t b = 1; b < STAB_MAX_RUNS; b++)
      if (r.runHist[b]) { Serial.print(' '); Serial.print(b); Serial.print('x'); Serial.print(r.runHist[b]); }
    Serial.println();
    Serial.print(F("#   dist:"));
    for (uint8_t b = 0; b < STAB_MAX_DIST; b++)
      if (r.distHist[b]) { Serial.print(' '); Serial.print(b * 25); Serial.print('-'); Serial.print(b * 25 + 24); Serial.print('x'); Serial.print(r.distHist[b]); }
    Serial.println();
  }
}

// Compact one-row-per-value form used by the per-sensor survey and the
// single-register sweeps: tag,value,meas,validPct,runs,longest,ev1,ev3,ev5,
// ev10,dMin,dMax,dMean.
static void stabPrintCompact(const char* tag, uint16_t value, const StabResult& r) {
  Serial.print(tag); Serial.print(',');
  Serial.print(value);
  Serial.print(','); Serial.print(r.meas);
  Serial.print(','); Serial.print(r.meas ? (uint32_t)r.measValid * 100 / r.meas : 0);
  Serial.print(','); Serial.print(r.runs);
  Serial.print(','); Serial.print(r.longest);
  Serial.print(','); Serial.print(r.ev[1]);
  Serial.print(','); Serial.print(r.ev[3]);
  Serial.print(','); Serial.print(r.ev[5]);
  Serial.print(','); Serial.print(r.ev[10]);
  Serial.print(','); Serial.print(r.valid ? r.dMin : 0);
  Serial.print(','); Serial.print(r.valid ? r.dMax : 0);
  Serial.print(','); Serial.print(r.dMean);
  Serial.println();
}

static void stabConfig(DistanceSensor& s, const char* tag,
                       uint16_t b4, uint16_t b5, uint16_t b6) {
  StabResult r;
  stabMeasure(s, r);
  stabPrintCsv(tag, b4, b5, b6, r, true);
}

// The sweep itself: B4 x B5 x B6, saved+restarted so each value takes effect.
// B6 is swept at three points because the empty-space data showed 100 beating
// 300 by a wide margin; the middle point shows where the knee is.
static const uint16_t STAB_B4[] = { 248, 254 };
static const uint16_t STAB_B5[] = { 0, 10, 20, 30, 40 };
static const uint16_t STAB_B6[] = { 100, 200, 300 };

static void runStab(const char* tag, uint8_t id) {
  if (id < 1 || id > 5) { Serial.println(F("ERR: id 1..5")); return; }
  DistanceSensor& s = allSensors[id - 1];
  uint16_t o4 = rd(0xB4), o5 = rd(0xB5), o6 = rd(0xB6);
  Serial.println();
  Serial.println(F("=== STABILITY SWEEP (flicker / hysteresis) ==="));
  Serial.print(F("sensor id ")); Serial.println(id);
  Serial.print(F("tag=")); Serial.println(tag);
  Serial.print(F("B4 x B5 x B6 = "));
  Serial.print(sizeof(STAB_B4)/2); Serial.print('x');
  Serial.print(sizeof(STAB_B5)/2); Serial.print('x');
  Serial.print(sizeof(STAB_B6)/2);
  Serial.print(F(" combos, ")); Serial.print(STAB_READS);
  Serial.println(F(" raw reads each"));
  Serial.println(F("Z,tag,b4,b5,b6,reads,meas,valid,validPct,runs,longest,ev1,ev3,ev5,ev10,dMin,dMax,dMean"));

  for (uint8_t i = 0; i < sizeof(STAB_B4)/2; i++)
  for (uint8_t j = 0; j < sizeof(STAB_B5)/2; j++)
  for (uint8_t k = 0; k < sizeof(STAB_B6)/2; k++) {
    uint16_t b4 = STAB_B4[i], b5 = STAB_B5[j], b6 = STAB_B6[k];
    s.writeRegister(0xB4, b4);
    s.writeRegister(0xB5, b5);
    s.writeRegister(0xB6, b6);
    s.saveConfiguration();
    s.restartSensor();
    delay(400);
    stabConfig(s, tag, b4, b5, b6);
  }

  s.writeRegister(0xB4, o4);
  s.writeRegister(0xB5, o5);
  s.writeRegister(0xB6, o6);
  s.saveConfiguration();
  s.restartSensor();
  delay(400);
  Serial.print(F("  (restored B4=")); Serial.print(o4);
  Serial.print(F(" B5=")); Serial.print(o5);
  Serial.print(F(" B6=")); Serial.print(o6); Serial.println(')');
  Serial.println(F("=== stability sweep done ==="));
}

// ---- per-sensor phantom survey ('P') ---------------------------------------
// Runs the CURRENT config on all five sensors and reports the burst analysis for
// each. Answers "which sensors are inherently noisy": a sensor with a high
// validPct here will produce phantoms no matter what the shared config is, and
// is the one to clean or replace. No target needed.
static void runPhantomSurvey() {
  uint16_t b4 = rd(0xB4), b5 = rd(0xB5), b6 = rd(0xB6);
  Serial.println();
  Serial.println(F("=== PER-SENSOR PHANTOM SURVEY (empty space) ==="));
  Serial.println(F("Point every sensor at >1.5 m of empty space (no target)."));
  Serial.print(F("current config: B4=")); Serial.print(b4);
  Serial.print(F(" B5=")); Serial.print(b5);
  Serial.print(F(" B6=")); Serial.println(b6);
  Serial.println(F("P,id,meas,validPct,runs,longest,ev1,ev3,ev5,ev10,dMin,dMax,dMean"));

  for (uint8_t id = 1; id <= 5; id++) {
    StabResult r;
    stabMeasure(allSensors[id - 1], r);
    stabPrintCompact("P", id, r);
    delay(200);
  }
  Serial.println(F("=== phantom survey done ==="));
}

// ---- min-distance (B7) sweep ('M') -----------------------------------------
// Sweeps the sensor's own minimum-distance window on empty space. Phantoms that
// sit at short range (the 0-50 mm cluster in the histograms) are killed by a
// floor without touching the signal threshold, so this is a surgical lever the
// B5/B6 sweeps cannot reach. Watch validPct and ev3 fall. No target needed.
static const uint16_t MIN_SWEEP[] = { 0, 25, 50, 75, 100, 150, 200 };

static void runMinSweep() {
  DistanceSensor& s = allSensors[NOISE_SENSOR_ID - 1];
  uint16_t o7 = rd(0xB7);
  uint16_t b4 = rd(0xB4), b5 = rd(0xB5), b6 = rd(0xB6);
  Serial.println();
  Serial.println(F("=== MIN-DISTANCE (B7) SWEEP (empty space) ==="));
  Serial.println(F("Point the sensor at >1.5 m of empty space (no target)."));
  Serial.print(F("sensor id ")); Serial.println(NOISE_SENSOR_ID);
  Serial.print(F("B4=")); Serial.print(b4);
  Serial.print(F(" B5=")); Serial.print(b5);
  Serial.print(F(" B6=")); Serial.println(b6);
  Serial.println(F("M,b7,meas,validPct,runs,longest,ev1,ev3,ev5,ev10,dMin,dMax,dMean"));

  for (uint8_t i = 0; i < sizeof(MIN_SWEEP)/2; i++) {
    uint16_t b7 = MIN_SWEEP[i];
    s.writeRegister(0xB7, b7);
    s.saveConfiguration();
    s.restartSensor();
    delay(400);
    StabResult r;
    stabMeasure(s, r);
    stabPrintCompact("M", b7, r);
  }

  s.writeRegister(0xB7, o7);
  s.saveConfiguration();
  s.restartSensor();
  delay(400);
  Serial.print(F("  (restored B7=")); Serial.print(o7); Serial.println(')');
  Serial.println(F("=== min-distance sweep done ==="));
}

// ---- sampling-time (B0) sweep ('B') ----------------------------------------
// Sweeps the integration time on empty space. Longer integration collects more
// photons, which helps the black target but also gives ambient noise more time
// to trip the validity checks. If a SHORT B0 is clean here AND still sees black,
// it is strictly better: fresher readings and fewer phantoms. No target needed.
static const uint16_t B0_SWEEP[] = { 10, 15, 20, 25, 33, 40, 50 };

static void runB0Sweep() {
  DistanceSensor& s = allSensors[NOISE_SENSOR_ID - 1];
  uint16_t o0 = rd(0xB0);
  uint16_t b4 = rd(0xB4), b5 = rd(0xB5), b6 = rd(0xB6);
  Serial.println();
  Serial.println(F("=== SAMPLING-TIME (B0) SWEEP (empty space) ==="));
  Serial.println(F("Point the sensor at >1.5 m of empty space (no target)."));
  Serial.print(F("sensor id ")); Serial.println(NOISE_SENSOR_ID);
  Serial.print(F("B4=")); Serial.print(b4);
  Serial.print(F(" B5=")); Serial.print(b5);
  Serial.print(F(" B6=")); Serial.println(b6);
  Serial.println(F("B,b0,meas,validPct,runs,longest,ev1,ev3,ev5,ev10,dMin,dMax,dMean"));

  for (uint8_t i = 0; i < sizeof(B0_SWEEP)/2; i++) {
    uint16_t b0 = B0_SWEEP[i];
    s.writeRegister(0xB0, b0);
    s.saveConfiguration();
    s.restartSensor();
    delay(400);
    StabResult r;
    stabMeasure(s, r);
    stabPrintCompact("B", b0, r);
  }

  s.writeRegister(0xB0, o0);
  s.saveConfiguration();
  s.restartSensor();
  delay(400);
  Serial.print(F("  (restored B0=")); Serial.print(o0); Serial.println(')');
  Serial.println(F("=== sampling-time sweep done ==="));
}

// ---- apply the winning config to all five sensors --------------------------
// Values come from TofConfig.h. Writes each register, reads it back, saves and
// restarts, ONE SENSOR AT A TIME, with delays so the shared bus settles between
// steps. Prints a per-register ok/BAD and a per-sensor verdict.
static void applyToAll() {
  static const uint8_t  REGS[6] = { 0xB0, 0xB2, 0xB4, 0xB5, 0xB6, 0xB8 };
  static const uint16_t VALS[6] = { TOF_CFG_B0, TOF_CFG_B2, TOF_CFG_B4, TOF_CFG_B5, TOF_CFG_B6, TOF_CFG_B8 };

  Serial.println();
  Serial.println(F("=== APPLY WINNING CONFIG TO ALL 5 SENSORS ==="));
  Serial.print(F("B0=")); Serial.print(TOF_CFG_B0);
  Serial.print(F(" B2=")); Serial.print(TOF_CFG_B2);
  Serial.print(F(" B4=")); Serial.print(TOF_CFG_B4);
  Serial.print(F(" B5=")); Serial.print(TOF_CFG_B5);
  Serial.print(F(" B6=")); Serial.print(TOF_CFG_B6);
  Serial.print(F(" B8=")); Serial.println(TOF_CFG_B8);

  bool allOk = true;
  for (uint8_t id = 1; id <= 5; id++) {
    DistanceSensor& s = allSensors[id - 1];
    Serial.print(F("id ")); Serial.print(id); Serial.print(F(": "));
    bool ok = true;
    for (uint8_t k = 0; k < 6; k++) {
      bool w = s.writeRegister(REGS[k], VALS[k]);
      delay(20);
      uint16_t rb = 0;
      bool r = s.readRegister(REGS[k], &rb);
      bool good = w && r && rb == VALS[k];
      ok = ok && good;
      hex2(REGS[k]);
      Serial.print('='); Serial.print(rb);
      Serial.print(good ? F(" ok ") : F(" BAD "));
      delay(20);
    }
    bool saved = s.saveConfiguration();
    delay(50);
    s.restartSensor();
    delay(400);
    Serial.println(ok && saved ? F("-> saved+restarted") : F("-> FAILED"));
    allOk = allOk && ok && saved;
    delay(100);
  }
  Serial.println(allOk ? F("=== apply done: all 5 OK ===")
                       : F("=== apply done: SOME FAILED ==="));
}

// ---- config dump -----------------------------------------------------------
static void printConfig() {
  static const uint8_t regs[] = {
    0x80,0x81,0x82,0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xBA,0xBB,0xBC,0xBD
  };
  Serial.println(F("reg  name          val"));
  for (uint8_t i = 0; i < sizeof(regs); i++) {
    uint16_t v = 0;
    bool ok = sensor.readRegister(regs[i], &v);
    Serial.print(F("  ")); hex2(regs[i]); Serial.print(F("  "));
    const char* n = regName(regs[i]);
    Serial.print(n);
    for (uint8_t p = strlen(n); p < 14; p++) Serial.print(' ');
    if (ok) Serial.println(v); else Serial.println(F("?"));
  }
}

// ---- write with verification -----------------------------------------------
static bool writeVerified(uint8_t reg, uint16_t val) {
  bool w = sensor.writeRegister(reg, val);
  uint16_t rb = rd(reg);
  Serial.print(F("  w ")); hex2(reg);
  Serial.print(F(" <- ")); Serial.print(val);
  Serial.print(w ? F("  ack=ok") : F("  ack=FAIL"));
  Serial.print(F("  readback=")); Serial.print(rb);
  Serial.println(rb == val ? F("  OK") : F("  MISMATCH"));
  return w && rb == val;
}

// ---- quick sweeps ----------------------------------------------------------
static void sweepReg(uint8_t reg, const uint16_t* vals, uint8_t n, bool saved) {
  uint16_t orig = rd(reg);
  Serial.println();
  Serial.print(saved ? F("-- SAVED sweep ") : F("-- sweep ")); hex2(reg);
  Serial.print(F(" (")); Serial.print(regName(reg));
  Serial.print(F("), orig=")); Serial.print(orig); Serial.println(F(" --"));
  for (uint8_t k = 0; k < n; k++) {
    if (saved) {
      sensor.writeRegister(reg, vals[k]);
      sensor.saveConfiguration();
      sensor.restartSensor();
      delay(400);
      uint16_t rb = rd(reg);
      Serial.print(F("  ")); hex2(reg); Serial.print(F(" <- ")); Serial.print(vals[k]);
      Serial.print(F("  readback=")); Serial.print(rb);
      Serial.println(rb == vals[k] ? F("  OK") : F("  MISMATCH"));
    } else {
      writeVerified(reg, vals[k]);
    }
    Stats st; sample(1200, st); printStats(st);
  }
  if (saved) {
    sensor.writeRegister(reg, orig);
    sensor.saveConfiguration();
    sensor.restartSensor();
    delay(400);
  } else {
    sensor.writeRegister(reg, orig);
  }
  Serial.print(F("  (restored ")); Serial.print(orig); Serial.println(')');
}

static void runSweep(bool saved) {
  Serial.println();
  Serial.println(saved ? F("=== SAVED SWEEP (write + save + restart) ===")
                       : F("=== QUICK SWEEP (no restart) ==="));
  static const uint16_t b0[] = {5, 10, 15, 20, 25, 33, 40, 50};
  static const uint16_t b5[] = {0, 10, 20, 30, 40, 60, 100, 200, 512};
  sweepReg(0xB0, b0, 8, saved);
  sweepReg(0xB5, b5, 9, saved);
  Serial.println(F("=== done ==="));
}

// ---- command line ----------------------------------------------------------
static char lineBuf[40];
static uint8_t lineLen = 0;

static void printHelp() {
  Serial.println(F("--- TOF_Black_Test (sensor 3, MID) ---"));
  Serial.println(F(" h            help"));
  Serial.println(F(" c            read + print full config"));
  Serial.println(F(" s [ms]       sample (default 1500): stats + status histogram"));
  Serial.println(F(" w <reg> <v>  write reg (dec or 0x..), read back, report"));
  Serial.println(F(" r            restart (reloads registers from SAVED config)"));
  Serial.println(F(" S            save current config to sensor memory"));
  Serial.println(F(" t            quick sweep B0/B5 (no restart)"));
  Serial.println(F(" T            saved sweep B0/B5 (write+save+restart)"));
  Serial.println(F(" g <dist>     GRID: all B0 x B5 x B6 combos at <dist> mm, averaged"));
  Serial.println(F(" v <dist>     VERIFY current config at <dist> mm, averaged"));
  Serial.println(F(" n            EMPTY-SPACE noise sweep: B4 x B5, 10 reps each"));
  Serial.println(F(" Z [tag] [id] STABILITY sweep: B4 x B5 x B6, raw burst analysis"));
  Serial.println(F(" P            PER-SENSOR phantom survey: current config, all 5"));
  Serial.println(F(" M            MIN-DISTANCE (B7) sweep on empty space"));
  Serial.println(F(" B            SAMPLING-TIME (B0) sweep on empty space"));
  Serial.println(F(" C            B1/B2 correction sweep on empty space (offset x crosstalk)"));
  Serial.println(F(" D <dist>     B2 crosstalk sweep at <dist> mm with the BLACK target"));
  Serial.println(F(" X <id> [ms]  RAW per-read capture from sensor <id> (default 3)"));
  Serial.println(F(" Y [ms]       RAW capture from ALL five sensors in sequence"));
  Serial.println(F(" A            APPLY TofConfig.h values to ALL 5 sensors (verify+save)"));
  Serial.println(F("--------------------------------------"));
}

static long parseVal(const char* s) {
  if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) return strtol(s + 2, NULL, 16);
  return strtol(s, NULL, 10);
}

static void handleLine(char* s) {
  while (*s == ' ') s++;
  char verb = *s;
  if (!verb) return;
  while (*s && *s != ' ') s++;
  while (*s == ' ') s++;

  switch (verb) {
    case 'h': case 'H': case '?': printHelp(); break;

    case 'c': printConfig(); break;

    case 's': case 'S': {
      if (verb == 'S') {
        Serial.println(sensor.saveConfiguration() ? F("[save] ok") : F("[save] FAILED"));
        break;
      }
      uint16_t ms = *s ? (uint16_t)strtol(s, NULL, 10) : 1500;
      if (ms < 200) ms = 200;
      Stats st;
      sample(ms, st);
      printStats(st);
      break;
    }

    case 'w': case 'W': {
      char* regTok = s;
      while (*s && *s != ' ') s++;
      if (*s) *s++ = '\0';
      while (*s == ' ') s++;
      if (!*regTok || !*s) { Serial.println(F("ERR: w <reg> <val>")); break; }
      uint8_t reg = (uint8_t)strtol(regTok, NULL, 16);   // registers are hex
      uint16_t val = (uint16_t)parseVal(s);              // value: dec or 0x.. hex
      writeVerified(reg, val);
      break;
    }

    case 'r': case 'R':
      Serial.println(sensor.restartSensor() ? F("[restart] ok (registers reloaded from saved config)")
                                            : F("[restart] FAILED"));
      delay(120);
      break;

    case 't': runSweep(false); break;
    case 'T': runSweep(true); break;

    case 'g': case 'G': {
      uint16_t dist = *s ? (uint16_t)strtol(s, NULL, 10) : 250;
      runGrid(dist);
      break;
    }

    case 'v': case 'V': {
      uint16_t dist = *s ? (uint16_t)strtol(s, NULL, 10) : 250;
      runVerify(dist);
      break;
    }

    case 'n': case 'N': runNoise(); break;

    case 'C': runCorr(); break;

    case 'Z': {
      // 'Z [tag] [id]' — stability sweep. Tag defaults to 'empty'; the optional
      // id (1..5) picks the sensor, default NOISE_SENSOR_ID. A bare number as
      // the first token is taken as the id, so 'Z 3' and 'Z empty 3' both work.
      char* tok = s;
      while (*s && *s != ' ') s++;
      if (*s) *s++ = '\0';
      while (*s == ' ') s++;
      const char* tag = "empty";
      uint8_t id = NOISE_SENSOR_ID;
      if (*tok) {
        bool numeric = true;
        for (char* p = tok; *p; p++) if (*p < '0' || *p > '9') { numeric = false; break; }
        if (numeric) id = (uint8_t)strtol(tok, NULL, 10);
        else tag = tok;
      }
      if (*s) id = (uint8_t)strtol(s, NULL, 10);
      runStab(tag, id);
      break;
    }

    case 'P': runPhantomSurvey(); break;
    case 'M': runMinSweep(); break;
    case 'B': runB0Sweep(); break;

    case 'D': {
      uint16_t dist = *s ? (uint16_t)strtol(s, NULL, 10) : 300;
      runB2(dist);
      break;
    }

    case 'X': {
      // 'X <id> [ms]' — capture from one sensor (default id 3).
      uint8_t id = 3;
      uint16_t ms = 20000;
      if (*s) {
        id = (uint8_t)strtol(s, NULL, 10);
        while (*s && *s != ' ') s++;
        while (*s == ' ') s++;
        if (*s) ms = (uint16_t)strtol(s, NULL, 10);
      }
      runRaw(id, ms);
      break;
    }

    case 'Y': {
      // 'Y [ms]' — capture from ALL five sensors in sequence. Separate verb
      // because the dispatcher strips the verb token before we see it, so
      // 'XALL' cannot be told apart from 'X' here.
      uint16_t ms = *s ? (uint16_t)strtol(s, NULL, 10) : 20000;
      runRawAll(ms);
      break;
    }

    case 'a': case 'A': applyToAll(); break;

    default:
      Serial.println(F("ERR: unknown, 'h' for help"));
      break;
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  pinMode(PIN_TOF_BUS, INPUT);
  bus.begin(TOF_BUS_BAUD);
  bus.setTimeout(TOF_TIMEOUT_MS);

  Serial.println();
  Serial.println(F("=== TOF_Black_Test ==="));
  Serial.println(F("Sensor 3 (MID) on D4. Put a matte BLACK plate in front,"));
  Serial.println(F("perpendicular, filling the cone."));
  Serial.println(F("Grid: 'g <dist>'   Verify: 'v <dist>'   Help: 'h'"));
  printHelp();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) { lineBuf[lineLen] = '\0'; handleLine(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
      Serial.println(F("ERR: line too long"));
    }
  }
}
