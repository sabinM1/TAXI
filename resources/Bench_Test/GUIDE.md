# M60F ToF tuning guide — finding the best configuration for sensor ID 3

Goal, in priority order:

1. **See black opponents well.** A matte-black target returns far less IR, so it
   is the hardest thing the sensor has to detect. Everything else is secondary
   to this.
2. **Be reliable.** No false positives (seeing a target when there is none) and
   no flicker (valid/invalid on a real target).
3. **Low latency.** The reading must be as fresh as possible so the match
   firmware reacts fast.

These three fight each other. A ToF sensor measures distance from the light it
gets back, and black returns much less light, so the two levers that recover a
black target both cost something:

| Lever | Register | Effect | Cost |
|-------|----------|--------|------|
| Longer sampling (integration) time | `0xB0` (5..50 ms, default 33) | More light collected -> higher signal on black | Higher latency |
| Lower signal threshold | `0xB5` (default 1024) | Accepts weaker returns -> black passes | Closer to the noise floor -> false positives |
| Higher sigma threshold | `0xB6` (default 15 mm) | Tolerates noisier returns | Wild/jittery readings pass |

So the job is not "crank everything up". It is: **find the shortest sampling
time that lifts the black return comfortably above the noise floor, then set the
signal threshold between the two with margin.** Sampling time is the latency
knob, so we push it down last.

> Datasheet quirk: the M60F doc lists register `0xB0` as BOTH "serial baud" and
> "sampling time". On these units `0x82` is the baud and `0xB0` is the sampling
> time — confirm with `g 3` (see Step 0). If a `0xB0` sweep does not change the
> read latency at all, `0xB0` is not the sampling register on your units.

---

## Registers that matter (from the datasheet)

| Reg | Name | Default | Range | Role here |
|-----|------|---------|-------|-----------|
| `0x00` | Ranging status | – | – | Bit 0 = valid; bits 1..7 = which check failed |
| `0x01` | Distance mm | – | – | **ID 3** reports here (mm) |
| `0x02` | Signal rate | – | – | Returned light — the money number |
| `0x03` | Ambient rate | – | – | Background light (a false-positive driver) |
| `0x04` | Sigma | – | – | Measurement std-dev in mm (noise) |
| `0x05` | SPAD count | – | – | Photon detectors used |
| `0x06` | Distance cm | – | – | IDs 1,2,4,5 report here (cm) |
| `0xB0` | Sampling time (ms) | 33 | 5..50 | Latency vs signal |
| `0xB4` | Range checks | 0xF8 | – | Which checks gate validity |
| `0xB5` | Signal threshold | 1024 | 0..FFF | Weakest return still accepted |
| `0xB6` | Sigma threshold (mm) | 15 | 0..FFF | Noisiest return still accepted |
| `0xB7` | Min distance (mm) | 1 | 0..FFF | Keep small (firmware min is 9) |
| `0xB8` | Max distance (mm) | 500 | 0..FFF | Raise to 800 to match the firmware window |

**ID 3 is the MIDDLE front sensor** (`sensorName` = `MID`). It looks straight
ahead, so it is the right one to tune for "seeing the opponent".

---

## Guided procedure — the easy way (recommended)

Flash `Bench_Test.ino`, open the Serial Monitor at **115200**, and send:

```
p
```

The sketch then walks you through the whole procedure one step at a time. At
each step it prints exactly what to place where and waits for you:

```
== Step 3: BLACK@300. Sweep sampling B0 = 5..50 ms.
   [g]=go  s=skip  q=quit
```

- **g** (or any key) — run the step / continue.
- **s** — skip this step.
- **q** — quit the procedure.

> Some serial monitors do not send CR/LF when you press Enter, so the wizard
> advertises **`g`** as the "go" key. Any key works, but use `g` if Enter seems
> to do nothing.

It runs the same commands the manual procedure below uses (`g 3`, `b`, `x 3`,
`K ...`) and prints the same CSV lines, so you can still paste the output back.
The last step asks for the winning values and applies them to all five sensors.

**Nothing is written to non-volatile memory until that last step**, so you can
abort with `q` at any point without changing the sensors. The motors are stopped
and the live view is switched off while the procedure runs.

### What the guided run does

| Step | It asks you to | It runs |
|------|----------------|---------|
| 0 | clear the area (>1.5 m) | `g 3` + `b` — baseline config + bus health |
| 1 | WHITE, then BLACK, at 100/200/300/400/500 mm | `x 3` twice per distance |
| 2 | remove all targets (>1.5 m) | `x 3` — the noise floor |
| 3 | hold BLACK at 300 mm | `K 3 B0 5 8 10 12 15 20 25 33 40 50` |
| 4 | keep BLACK at 300 mm | `K 3 B5 100 200 300 400 600 800 1024 1500 2000` |
| 5 | keep BLACK at 300 mm (optional) | `K 3 B6 10 15 20 30 40 60 100` |
| 6 | optional | `K 3 B8 500 600 700 800` |
| 7 | re-apply your winners | `x 3` — verify |
| 8 | type the winning values | `A B0 .. B5 .. B6 .. B8 ..` — all five, saved |

At Step 8, pressing Enter on a prompt keeps the sensor's **current** value, so
you only type the registers you actually changed. (Enter, CR, LF and CRLF all
end an entry, so any serial-monitor line-ending setting works.)

### Reading the output

Every measurement prints one CSV line. The columns that decide the tuning:

- `valid` — readings the sensor's own check AND the firmware 9..800 mm window
  accept. Black target: want `valid` ≈ `ans`. No target: want `valid` = 0.
- `ratio%` — `sigMax` as a percentage of the signal threshold in force.
  **>100 means the sensor can accept this target; <100 means it is throwing the
  return away.**
- `dStd` — distance standard deviation in mm (jitter; single digits are good).
- `readUsMax` — worst-case per-read latency; must stay under `TOF_TIMEOUT_MS`.

The manual steps below explain *why* each number matters; the guided mode just
runs them in order.

---

## New bench commands (added to Bench_Test.ino)

All output is short comma-separated lines prefixed with a tag, so a whole run can
be pasted back verbatim.

| Command | What it does |
|---------|--------------|
| `p` | **Guided procedure** — walks the steps below, prompting for each. |
| `g <id>` | One CSV line: config registers `0x80..0xBD`. Default id 3. |
| `x <id> [ms]` | One CSV line of stats for ONE sensor over `[ms]` (default 1500). |
| `K <id> <reg> <v..> [w<ms>] [keep]` | Write `reg` through a list of values, sampling after each; one CSV row per value. Restores the original value unless `keep`. |
| `A <reg> <val> [<reg> <val> ..]` | Apply register/value pairs to **all five** sensors and save (non-volatile). |

CSV columns (after the `tag,id,...` prefix):

```
samples, ans, valid, sigMin, sigAvg, sigMax, thr, ratio%, ambAvg, sigmaAvg,
dMin, dMax, dAvg, dStd, readUsAvg, readUsMax
```

- `ans` — reads that got a reply (bus health; should equal `samples`).
- `valid` — readings the sensor's own check AND the firmware 9..800 mm window accept.
- `thr` — signal threshold (`0xB5`) in force at the time.
- `ratio%` — `sigMax` as a percentage of `thr`. **>100 means the sensor can
  accept this target; <100 means it is throwing the return away.**
- `dStd` — distance standard deviation in mm (jitter; lower = more reliable).
- `readUsAvg/readUsMax` — per-read latency on the shared bus.

The existing `l` (live), `n` (SNR rows), `s` (all-sensor stats) and `b` (bus
health) commands are still there for eyeballing.

---

## Physical setup

- Robot **stationary**, motors irrelevant — this is a sensor test. Wheels can be
  on the bench.
- **Target:** two flat plates, identical size/geometry, one **matte black**, one
  **white** (or the real opponent's material/colour). A plate is better than a
  curved/glossy object because it gives a repeatable return.
- **Size:** the sensor cone is 18-22 deg, so at 300 mm it covers ~116 mm across.
  Make the target **at least 150 mm wide** so it fills the cone; wider for 400-500 mm.
- **Mount:** hold the plate **perpendicular** to the sensor axis, centred on it,
  at a **measured** distance from the sensor's front face. A tape/ruler on the
  bench or a simple jig beats holding it by hand.
- **Lighting:** test under the same lighting as the real arena. Ambient IR
  matters and shows up in `ambAvg`.
- Keep the existing wiring and pull-up. Do not unplug sensors — every sensor is
  polled on the shared bus.

Serial Monitor at **115200**. Send `h` any time for the menu, `p` for the guided
procedure.

---

## Manual procedure (what the guided mode runs for you)

Use this if you prefer to drive each command yourself, or to understand what the
guided mode is doing.

### Step 0 — Baseline and sanity

```
g 3        <- record the current config of sensor 3 (paste this back)
b          <- bus health: sensor 3 must answer ~50/50 with low latency
```

Check the `g 3` line: note `0x82` (should be `5` = 115200 baud) and `0xB0`
(should be the sampling time, e.g. 33 or 12). Note `0xB5`, `0xB6`, `0xB8` too.
This is your "before" picture and your restore point.

### Step 1 — Black vs white at fixed distances (the core experiment)

This measures how much weaker black is. Do it at the distances you care about
(the match firmware commits to attack at `TARGET_ENGAGE_MM` = 300 mm, so 300 is
the key number; add 100/200/400/500 for the full picture).

For each distance D in {100, 200, 300, 400, 500} mm:

```
x 3        <- white target at D, held still
x 3        <- black target at D, held still   (swap the plate, same distance)
```

Record `sigAvg`, `sigMax`, `ratio%`, `valid`, `dStd` for each. Expect the black
`sigMax` to be well below the white one. **If black `ratio%` < 100 at D, the
sensor is rejecting black at that distance** — Steps 3-5 are how you fix it.

### Step 2 — Noise floor (the reliability anchor)

Point the sensor at **empty space** (no target, nothing within ~1.5 m):

```
x 3
```

Record `sigMax` — this is the **noise floor**. `valid` must be `0` here; if it is
not, the thresholds are already too loose. The signal threshold you choose in
Step 4 must stay **above** this noise floor or you get false positives.

### Step 3 — Sampling-time sweep (black signal vs latency)

Put the **black** target at the required detection distance (300 mm). Sweep the
sampling time:

```
K 3 B0 5 8 10 12 15 20 25 33 40 50
```

Each row shows what that sampling time buys: `sigAvg`/`sigMax` rise with it, and
`readUsAvg`/`readUsMax` show the latency it costs.

**Pick the SMALLEST sampling time** at which the black `sigMax` sits comfortably
above the Step-2 noise floor (a factor of ~2+ is a good target) — that is the
latency/black tradeoff point.

- If `readUs` does not move across the sweep, `0xB0` is not sampling on your
  units; check `0x82` vs `0xB0` in Step 0.
- Remember the chosen sampling time: `TOF_TIMEOUT_MS` in `Config.h` (currently
  15 ms) must exceed it plus margin, or the old "reply leaks into the next
  sensor's read" bug comes back. If you pick 33 ms, `TOF_TIMEOUT_MS` needs to be
  ~40 ms.

### Step 4 — Signal-threshold sweep (acceptance vs false positives)

Keep the **black** target at 300 mm. Sweep `0xB5`:

```
K 3 B5 100 200 300 400 600 800 1024 1500 2000
```

Watch `valid` and `ratio%`: `valid` jumps from 0 to N once `thr` drops below the
black signal.

Choose `thr` **below the black `sigMin`** (so even the weakest black sample
passes) but **above the Step-2 noise floor `sigMax`**, with margin (~2x on each
side). A value that only just passes black is fragile; a value near the noise
floor is a false-positive generator.

### Step 5 — Sigma threshold (jitter), only if needed

If valid black readings show high `dStd`, or the reading flickers valid/invalid,
relax `0xB6`:

```
K 3 B6 10 15 20 30 40 60 100
```

Raise it until black stops being rejected for noise, but not so high that
obviously wrong distances pass. Prefer fixing jitter with sampling time first;
`0xB6` is a fallback.

### Step 6 — Range window

If you need the sensor to accept targets out to 800 mm (the firmware window),
raise the max distance:

```
K 3 B8 500 600 700 800
```

Keep `0xB7` (min distance) small. Note the default `0xB8` = 500 mm is *narrower*
than the firmware's 800 mm window — a target between 500 and 800 mm is currently
rejected by the sensor itself.

### Step 7 — Verify (reliability + latency)

With the winning config applied to sensor 3 (re-run the winning `K` with `keep`,
or just `W 3 B0 <v>` etc.), redo the black/white comparison at every distance:

```
x 3        <- black at 100/200/300/400/500, and with no target
```

Success criteria:

- Black target: `valid` ≈ `ans` (accepts nearly every read), `ratio%` > 100,
  `dStd` small (single-digit mm is good).
- No target: `valid` = 0 (no false positives).
- `readUsMax` < `TOF_TIMEOUT_MS`.

### Step 8 — Save and apply to all five

Save sensor 3 (if you did not use `keep`):

```
S 3
```

Apply the winning values to every sensor and save them all in one shot:

```
A B0 <sampling> B5 <thr> B6 <sigma> B8 <maxdist>
```

Verify:

```
g 1
g 2
g 3
g 4
g 5
```

Finally, mirror any changed timing back into the firmware:

- `Config.h`: `TOF_TIMEOUT_MS` must exceed the new sampling time + margin.
- `Config.h`: `TOF_MAX_VALID_MM` if you changed the usable range.
- Re-run `resources/TOF_Timing_Probe` if you want an independent latency check.

---

## Worked session (copy-paste order)

Guided (recommended):

```
p                       walk the whole procedure, prompting at each step
```

Manual equivalent:

```
h                       menu
g 3                     Step 0 baseline (paste back)
b                       bus health

x 3                     Step 1 white @ 300
<swap to black, 300>
x 3                     Step 1 black @ 300
<no target>
x 3                     Step 2 noise floor

<black @ 300>
K 3 B0 5 8 10 12 15 20 25 33 40 50     Step 3 sampling sweep
K 3 B5 100 200 300 400 600 800 1024    Step 4 threshold sweep (at chosen B0)
K 3 B6 10 15 20 30 40 60 100           Step 5 sigma (only if jittery)
K 3 B8 500 600 700 800                 Step 6 range

<re-apply winners with keep, then re-verify>
x 3                                     Step 7 verify black @ each distance
x 3                                     Step 7 verify no-target -> valid 0

A B0 <s> B5 <t> B6 <g> B8 <m>           Step 8 apply to all + save
g 3                                     confirm
```

Paste the `g 3` line from Step 0, the `K` sweep tables, and the final `x 3`
lines back for interpretation.
