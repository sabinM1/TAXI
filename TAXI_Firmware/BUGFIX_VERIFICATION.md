# Side-Sensor Steering Verification

## Main defect confirmed

`sensorsOpponent()` previously ignored both side ToF sensors whenever any front
sensor (including the zero-weight center sensor) had a valid reading. With an
opponent beside the robot, a medium-distance center/front reading could silence
the close side reading and dilute or erase the steering command. The corrected
vote includes every valid sensor and weights each by `(801 - mm)^2`, so the
closest sensor dominates while sensors on the same side preserve full authority.

Validated combinations use `STEER_GAIN = 120%`:

| Scenario | Bearing | Approx. base | Motor L/R |
|---|---:|---:|---:|
| Side-only at 150 mm | -100 | 45 | -9 / 99 |
| Side 150 mm + center 400 mm | -72 | 45 | 7 / 83 |
| Side 150 mm + same-side front 300 mm | -100 | 45 | -9 / 99 |
| Side 150 mm + opposite front 400 mm | -44 | 45 | 22 / 68 |
| Side-only at 350 mm (SEARCH approach) | -100 | 30 | -6 / 66 |
| Front-left-only at 150 mm | -100 | 45 | -9 / 99 |
| Both sides at 200 mm | 0 | 45 | 45 / 45 |

## Issues found during review

1. **Far side-only SEARCH approach remained gentle.** A side sensor reading just
   beyond `TARGET_ENGAGE_MM` can produce bearing ±100, but `steerToward()` used
   `SEARCH_SPEED_PCT` as the base. At full 120% gain this only yields about
   `-6 / +66`, which is intentionally an arc and does not match the required
   aggressive side turn.

2. **ATTACK bearing memory could bypass the corrected vote.** `targetBearing()`
   prefers `lastSeenBearing` whenever the current opponent reading is beyond
   `TARGET_ENGAGE_MM`. A live side-only vote can therefore be discarded if an
   older frontal close reading remains remembered. In addition, `bearingRawSeen`
   marks any-range sightings as fresh, allowing a far sighting to keep ATTACK
   alive while steering on that older memory.

3. **Side-backoff could interrupt EVADE's turn phase.** The backoff gate excluded
   BACKOFF, IDLE and STOP, but not EVADE. Once the line sensor cleared and EVADE
   reached TURN, a sufficiently old side-contact hold could replace the pivot
   with straight reverse, risking motion back toward the border being escaped.

These were behavioral/control-flow findings, not syntax failures: the sketch
compiled successfully for `arduino:avr:leonardo` after the vote fix.

## Corrections applied after documentation

1. `SEARCH` now uses `SEARCH_SIDE_STEER_GAIN_PCT` (200%) when
   `sensorsSideDominant()` is true: the closer side sensor is no farther than
   every valid front reading and its direction agrees with the vote. Comparing
   distances avoids missing a side-led vote diluted below ±100 by a zero-weight
   center reading, while avoiding a gain boost when an opposite front sensor is
   actually closer. At the 30% search base this changes a far side-only approach
   from approximately `-6 / +66` to `-30 / +90`. A front-led approach still uses
   `ATTACK_STEER_GAIN_PCT`, preserving the front turn behavior.

2. `targetBearing()` now treats a far side-dominant reading as authoritative and
   updates the remembered side. `bearingRawSeen` now means a close live sighting,
   so an opponent held beyond `TARGET_ENGAGE_MM` cannot refresh ATTACK
   indefinitely while the robot steers on older memory.

3. The side-backoff gate now excludes `ST_EVADE`, so the edge-escape pivot cannot
   be replaced by a straight reverse after the line sensor clears.

Final scenario results:

| Scenario | Bearing | Base | Gain | Motor L/R |
|---|---:|---:|---:|---:|
| Side-only at 150 mm | -100 | 45 | 120 | -9 / 99 |
| Side 150 mm + center 400 mm | -72 | 45 | 120 | 7 / 83 |
| Side 150 mm + same-side front 300 mm | -100 | 45 | 120 | -9 / 99 |
| Side 150 mm + opposite front 400 mm | -44 | 45 | 120 | 22 / 68 |
| Side-only at 350 mm (SEARCH approach) | -100 | 30 | 200 | -30 / 90 |
| Side 350 mm + center 500 mm (SEARCH) | -69 | 30 | 200 | -11 / 71 |
| Front-left-only at 150 mm | -100 | 45 | 120 | -9 / 99 |
| Both sides at 200 mm | 0 | 45 | 120 | 45 / 45 |

## Exhaustive behavioral cases

- **No valid ToF:** bearing 127; SEARCH uses its selected pattern, ATTACK falls
  back to the push/lost-target rules.
- **Front-only sighting:** unchanged. Front sensors receive their existing full
  authority, far front approaches retain `ATTACK_STEER_GAIN_PCT`.
- **Side-only at 21..300 mm:** enters ATTACK and receives full ±100 bearing; at
  150 mm the command is approximately -9/+99.
- **Side-dominant vote beyond 300 mm:** remains in SEARCH and uses 200% gain;
  side-only at 350 mm commands approximately -30/+90. It also overrides older
  frontal memory during the short ATTACK grace window without refreshing the
  ATTACK lost-target timer.
- **Side plus same-side front:** signs agree, so the vote remains full authority;
  behavior is equivalent to the original strong front-corner turn.
- **Side plus opposite-side front:** the closer reflection has squared-weight
  authority; mixed votes are intentionally intermediate instead of false hard
  turns.
- **Side plus center:** center cannot change the sign because its steering weight
  is 0, but it can dilute magnitude. Distance dominance still identifies the
  side-led case for far-SEARCH gain and attack-memory override.
- **Both side sensors:** equal/simultaneous side votes cancel toward 0 rather
  than choosing an arbitrary side.
- **Side at or below 20 mm for the configured hold:** side-backoff wins over the
  normal state body and reverses to open the gap; it cannot replace an EVADE
  pivot once EVADE is active.
- **Middle-sensor blade contact:** contact latch and full-speed push still win
  inside ATTACK before `steerToward()` is called.
- **Line edge:** EVADE is still checked before opponents/backoff and cannot be
  bypassed by ToF behavior.

Final validation: `arduino-cli compile --fqbn arduino:avr:leonardo .` passes
(17,136 bytes flash, 59%; 1,034 bytes RAM, 40%), and `git diff --check` reports
no whitespace errors.
