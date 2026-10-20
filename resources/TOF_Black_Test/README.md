# TOF_Black_Test — M60F black-target tuning (sensor 3, MID)

A self-contained sketch for finding the best M60F settings for a **matte black
target**, at several distances, with every write verified and every result
averaged over repeats.

It is separate from `Bench_Test` because `Bench_Test` had three blind spots that
made the earlier results unreliable:

1. Its `K` sweep **never read the register back**, so a failed or ignored write
   looked exactly like "the setting doesn't help".
2. It only looked at **status bit 0**. The status byte has 8 bits that say
   *which* check failed — that is the whole diagnosis, and it was discarded.
3. It did not account for the fact that the sensor **latches its configuration
   at boot** (see below).

---

## The big finding: config is latched at boot

The datasheet says settings "take effect immediately". **On these units they do
not.** A live `w` changes the register you read back, but the sensor keeps using
the boot-time value until you **save + restart**. Proven directly:

| `B5` | applied how | `valid` @ 250 mm black |
|---|---|---|
| 0 | live `w` only | **0** (`sigTHR-fail`) |
| 0 | `w` + `S` + `r` | **174** |

Same value, same target — only the saved+restarted one took effect. **This is
why every earlier sweep showed no change: they were all measuring the boot-time
config, not the value being swept.**

So: to make a value take effect → `w <reg> <v>` → `S` → `r` → measure.

---

## Wiring / setup

- Sensor **id 3 (MID)** on **D4**, 4.7k pull-up to 5V.
- Serial Monitor at **115200**.
- Target: a **matte black** plate, **perpendicular**, wide enough to fill the
  18–22° cone (≥150 mm at 250 mm).
- Robot stationary; motors irrelevant.

---

## Steps

### 1. Flash and open the Serial Monitor

```
=== TOF_Black_Test ===
Sensor 3 (MID) on D4. Put a matte BLACK plate in front,
perpendicular, filling the cone.
Grid: 'g <dist>'   Verify: 'v <dist>'   Help: 'h'
```

### 2. Confirm the config — send `c`

Prints every register with its value.

### 3. Baseline sample — send `s`

```
   n=200 ans=200 valid=0 sig=0/18/49 spad=65535 amb=43 sigma=262 d=0 [0-0]
   status: sigTHR-fail=200
```

The **`status:`** line names exactly which check is rejecting the reading.

### 4. Run the grid at each distance

Put the black plate at the distance, then:

```
g 150
g 200
g 250
g 300
```

Each `g` runs **every `B0 × B5 × B6` combination**, saves+restarts each one,
takes **3 samples of 500 ms**, and prints one averaged CSV row per combo.

Default grid:

| Register | Values |
|---|---|
| `B0` sampling | 25, 33, 40, 50 |
| `B5` signal thr | 0, 20, 40, 60 |
| `B6` sigma thr | 100, 300 |

= 32 combos × 4 distances = 128 configs. About **1 minute per distance**.

### 5. Verify the winner — send `v <dist>`

Set the winning config, then:

```
v 150
v 200
v 250
v 300
```

`v` re-measures the **current** config at that distance, averaged, so you can
confirm it holds up at every range.

### 6. Empty-space noise floor — send `n`

Point the sensor at **>1.5 m of empty space** (no target, no wall, no hand),
then send `n`. It sweeps `B4 × B5` (checks × signal threshold) at the current
`B0`/`B6`, taking **10 averaged samples per config**, and prints one `N,...` CSV
row each.

A config is **safe only if `valid` is 0 across every rep** — any non-zero
`valid` here is a phantom opponent. Use it to pick the lowest `B5` that is
consistently clean, and to see whether `B4=254` (all checks on) beats the
default `248`.

```
N,b4,b5,reps,samples,ans,valid,validPct,sigMin,sigAvg,sigMax,sigmaAvg,ambAvg,dAvg,dMin,dMax,dStd,status
```

Default sweep: `B4` = {248, 254}, `B5` = {0, 20, 25, 30, 35, 40, 45, 50, 60}.

> Note: the signal threshold does **not** compare against register `0x02`. A row
> can show `sigMax` below `B5` yet still pass — trust `valid`, not `sigMax`.

### 7. Stability sweep — send `Z [tag] [maxmm] [reps] [id]`

The averaged sweeps (`g`, `v`, `n`) answer *"how many reads were valid"*. They
**cannot** answer the question that actually matters for the phantom problem:
are the valid reads a **sustained run** (a real target) or a **flicker** (a
phantom)? A config can show `validPct = 20%` as either one solid burst or two
hundred single-frame blips, and those are completely different robots.

`Z` captures the **raw per-read stream** and reports its structure:

```
Z,tag,b4,b5,b6,reads,meas,valid,validPct,runs,longest,ev1,ev3,ev5,ev10,dMin,dMax,dMean
```

| Column | Meaning |
|---|---|
| `reads` | raw bus reads (the sensor is polled far faster than it refreshes) |
| `meas` | **distinct measurements** — repeats of the same raw value collapse |
| `validPct` | share of **measurements** that were valid |
| `runs` | number of valid bursts (fewer = less flicker) |
| `longest` | longest single burst in measurements (bigger = more sustained) |
| `ev1`/`ev3`/`ev5`/`ev10` | **phantom events surviving** a consecutive-valid filter of N |
| `dMin`/`dMax`/`dMean` | distance spread of the valid measurements |

> **Why `meas` matters.** The bus is polled far faster than the sensor refreshes
> (`B0` ms), so the *same* measurement is read ~27 times in a row. Counting those
> repeats as separate valid reads makes a single-measurement phantom look like a
> 27-long sustained run that survives every N. `Z` collapses repeats exactly like
> the firmware's `tofReadOne()` does, so `ev3` is a true prediction of what the
> robot will publish.

`evN` is the number of times the published value would flip from nothing to a
reading. A sustained burst of length `L` yields exactly **one** event for any
`N <= L`, and **zero** for `N > L`. So `ev3` is literally "how many phantom
sightings the firmware's `TOF_CONSECUTIVE_VALID=3` filter would let through".

**How to use it:**

- **Empty space** → `Z empty`. Want `validPct` low **and** `ev3 == 0`.
- **Black plate** → `Z black`. Want `validPct` high **and** `longest` large.
- **Another sensor** → `Z empty 4` (tag then id), or just `Z 4`.

**Optional arguments.** After the tag you may add up to three numbers. They are
classified by magnitude, so the order does not matter:

| Value | Meaning |
|---|---|
| `>= 100` | **temporary `B8` max distance** (mm). Written before the sweep and **restored** when it ends — the saved config is untouched. Use it to test a tighter distance window on empty space without editing `TofConfig.h`. |
| `6 .. 99` | **repeat count** — runs the whole sweep that many times back to back. |
| `1 .. 5` | **sensor id** (default `3`). |

```
Z empty 500 6     # temporary B8=500 mm, repeat the sweep 6 times
Z empty 500       # temporary B8=500 mm, one run
Z black 10        # 10 runs, no B8 change
Z empty 500 6 2   # B8=500, 6 runs, sensor 2
Z 3               # sensor 3, one run (bare id still works)
```

Each repeat is separated by a `# --- rep N/M ---` comment line. The temporary
`B8` is set once before the first repeat and restored after the last, so every
repeat sees the same window.

> Note: because `1..5` is reserved for the sensor id, a repeat count of 2–5 is
> not expressible — use `6` or more, or just run the command again.

A config that is clean on empty space *and* sustained on black is the winner.
The `#   runs:` and `#   dist:` lines below each row give the full histograms
when you need to see the shape rather than the summary.

Default sweep: `B4` = {248, 254}, `B5` = {0, 10, 20, 30, 40}, `B6` = {100, 200,
300} = 30 combos × 1200 raw reads each. About **1 minute** per run.

### 8. Other empty-space tests (no target needed)

These all run with the sensor pointed at **>1.5 m of empty space**. They answer
the questions the `Z` sweep cannot.

**`P` — per-sensor phantom survey.** Runs the *current* config on all five
sensors and reports the burst analysis for each:

```
P,id,meas,validPct,runs,longest,ev1,ev3,ev5,ev10,dMin,dMax,dMean
```

A sensor with a high `validPct` here is **inherently noisy** — it will produce
phantoms no matter what the shared config is, and is the one to clean or
replace. This is how you tell a bad sensor from a bad config.

**`M` — min-distance (`B7`) sweep.** Sweeps the sensor's own minimum-distance
window. Phantoms that sit at short range (the 0–50 mm cluster in the
histograms) are killed by a floor without touching the signal threshold — a
surgical lever the `B5`/`B6` sweeps cannot reach.

```
M,b7,meas,validPct,runs,longest,ev1,ev3,ev5,ev10,dMin,dMax,dMean
```

**`B` — sampling-time (`B0`) sweep.** Longer integration collects more photons,
which helps the black target but also gives ambient noise more time to trip the
validity checks. If a **short** `B0` is clean here *and* still sees black, it is
strictly better: fresher readings and fewer phantoms.

```
B,b0,meas,validPct,runs,longest,ev1,ev3,ev5,ev10,dMin,dMax,dMean
```

All three restore the swept register when they finish.

### 10. Paste everything back

`c`, `s`, the `g` rows, the `v` rows, and the `n` rows.

### 11. Apply the winner to all five sensors — send `A`

Edit `TofConfig.h` (same folder) to hold the winning values, then send `A`. It
writes each register to **all five sensors one at a time**, reads every write
back, saves, restarts, and prints a per-register `ok`/`BAD` plus a per-sensor
verdict. Confirm with `g 1` … `g 5`.

The current winner is already in `TofConfig.h`:

| Reg | Value | Why |
|---|---|---|
| `B0` | 45 | sampling ms — best black signal vs latency |
| `B4` | 254 | all checks on; kills the empty-space phantoms |
| `B5` | 30 | lowest signal threshold clean on empty space |
| `B6` | 300 | sigma must be high; the black return is noisy |

Verified with these values: empty space `valid=0`, black @ 300 mm `valid=100%`,
black @ 400 mm `valid=95%`.

---

## How to read the CSV

```
G,dist,b0,b5,b6,reps,samples,ans,valid,validPct,sigMin,sigAvg,sigMax,
sigmaAvg,ambAvg,dAvg,dMin,dMax,dStd,status
```

| Column | Meaning |
|---|---|
| `valid` / `validPct` | reads the sensor accepted (want high) |
| `sigMin/Avg/Max` | returned light (compare with the noise floor) |
| `sigmaAvg` | measurement noise in mm (lower = better) |
| `dAvg` / `dMin` / `dMax` | distance; want `dAvg` near the true distance and a **tight** `dMin..dMax` |
| `dStd` | distance standard deviation (lower = more reliable) |
| `status` | which checks failed (e.g. `sigTHR+sigma`) |

**Pick the row with high `validPct`, low `sigmaAvg`, and a tight `dMin..dMax`
centred on the true distance.**

---

## Commands

| Command | What it does |
|---|---|
| `h` | help |
| `c` | read + print the full config |
| `s [ms]` | sample (default 1500 ms): stats + status histogram |
| `w <reg> <v>` | write reg, read back, report (`v` decimal, or `0x..` hex) |
| `r` | restart (reloads registers from saved config) |
| `S` | save current config to sensor memory |
| `t` | quick sweep `B0`/`B5`, **no restart** (shows boot-time behaviour) |
| `T` | saved sweep `B0`/`B5` (write + save + restart) |
| `g <dist>` | **grid**: all `B0 × B5 × B6` combos at `<dist>` mm, averaged |
| `v <dist>` | **verify**: current config at `<dist>` mm, averaged |
| `n` | **empty-space noise sweep**: `B4 × B5`, 10 reps each |
| `Z [tag] [maxmm] [reps] [id]` | **stability sweep**: `B4 × B5 × B6`, raw burst/flicker analysis. `maxmm >= 100` = temporary `B8` (restored); `reps 6..99` = repeat; `id 1..5` = sensor |
| `P` | **per-sensor phantom survey**: current config, all 5 sensors |
| `M` | **min-distance (`B7`) sweep** on empty space |
| `B` | **sampling-time (`B0`) sweep** on empty space |
| `A` | **apply** `TofConfig.h` values to all 5 sensors (verify + save) |

Note: `w` takes the **register in hex** (`B4`) and the **value in decimal or
`0x`-hex** (`248` or `0xF8`). (The old `Bench_Test` `W` only accepted decimal,
which is why `W 3 B4 F8` wrote 0.)

---

## Findings so far

- **Writes apply, but config is latched at boot** — only `S` + `r` makes a value
  take effect.
- **`B5` is the signal threshold** and works once saved+restarted (`valid` falls
  123 → 0 as `B5` rises 0 → 512).
- **`B0` (sampling) works** — longer sampling gives more valid reads and tighter
  distances. `B0=50` was best: `valid ≈ 184/200 (92%)`, range `238–360` for a
  250 mm plate.
- Best combo so far: **`B0=50`, `B5=30–60`, `B6=100`**.
- Still marginal: `sigma ≈ 240`, signal `~16–25`, and the distance reads
  ~280 mm for a 250 mm plate (a calibration offset — check `B1`/`B3`).
- `spad=65535` on every read is unexplained.
- **The averaged sweeps hide the real failure mode.** A live log on the robot
  showed the MID sensor flickering `---` → `106` → `---` → `292` while a side
  sensor sat frozen at exactly `230` for 30 consecutive rows. Averaging those
  into one `validPct` number loses the fact that they are *flicker* and *stuck*,
  not a target. Use `Z` to see the burst structure instead.

## Firmware note

If you adopt `B0=50`, the sensor can be busy measuring for up to 50 ms, so raise
`TOF_TIMEOUT_MS` in `Config.h` from 15 to ~60 ms, or a late reply leaks into the
next sensor's read.
