# Guided procedure (`p`) — verification notes

Scope: the interactive guided black-vs-white procedure added to `Bench_Test.ino`
(command `p`) and the `GUIDE.md` rewrite that documents it.

## What changed

- `Bench_Test.ino`: new `p` command — a blocking, step-by-step walk-through of
  the GUIDE.md procedure. It prints what to place where, waits for a key, runs
  the matching measurement (`g 3`, `b`, `x 3`, `K ...`), and at the end applies
  the winning values to all five sensors. Helpers: `gKey`, `gAsk`, `gNum`,
  `cmdGuide`.
- `GUIDE.md`: leads with the guided mode; the manual command sequence is kept as
  the reference / fallback.

## Flash budget (important)

The sketch was already at **97 % (27,912 / 28,672 bytes)** before this change,
because of the CSV tuning helpers. The guided mode needs ~2.9 KB, so room had to
be made. The following pre-existing text was compacted — same information,
terser wording, no behaviour change:

| Item | Change |
|---|---|
| `printHelp()` | verbose per-command descriptions collapsed to one line each; the "Black-vs-white procedure" block removed (the guided mode replaces it) |
| `cmdInfo()` | long labels shortened |
| `cmdBus()` | trailing explanation shortened |
| `cmdConfig()` | register-name table no longer stores trailing padding; `padStr(name, 14)` keeps the column aligned |

Final: **28,602 bytes (99 %), 70 bytes of headroom**, 1,052 bytes RAM (41 %).
The headroom is small — any future addition should start by freeing space.

## Issues found during verification

### 1. Intro `s` aborted the whole procedure (minor UX)
`if (!gAsk(intro)) return;` treats "skip" the same as "quit", so pressing `s`
on the intro screen exited the wizard instead of continuing to Step 0.

### 2. Step 8 defaults were hardcoded, not the sensor's current values (correctness)
The prompt said "Enter keeps the [default]", but the bracketed values were fixed
constants (33 / 1024 / 15 / 800). If the operator just pressed Enter, those
values were written **and saved** to all five sensors — silently overwriting a
sensor whose real config differed (e.g. a sampling time of 12). The bracket value
must be the sensor's *current* register value.

### 3. Header comment promised an `h` help key that does not exist (doc mismatch)
The block comment listed "h = help"; the compact wizard has no help handler and
`gAsk` treats `h` as "continue".

### 4. Duplicate `if (gQuit) return;` after the intro (dead code)
The intro fix left two identical guards back to back. Harmless but wasteful;
removed.

### 5. Flash headroom is only ~70 bytes (build fragility)
Not a logic bug, but the sketch now sits at 99 % of the Leonardo's 28,672 bytes.
Recorded so the next change starts by freeing space rather than by debugging a
"sketch too big" error.

### 6. Enter did not advance the wizard on some serial monitors (UX)
The prompts advertised only `[Enter]=go`. `gAsk` already treats *any* key as
"go", so the logic was fine — but if the monitor does not emit CR/LF on Enter
(or emits only CR), pressing Enter sends nothing and the prompt just sits there
until the 120 s timeout. The operator has no way to know a letter key works.

### 7. `gNum` (Step 8) only ended on LF (correctness)
`gNum` read characters and broke only on `'\n'`, ignoring `'\r'`:

- **CR-only terminal** (line ending = "Carriage return"): Enter sends `\r`, which
  was ignored, so the entry never ended and the prompt hung for the full 120 s
  before falling back to the default.
- **CRLF terminal**: the entry ended on `\r` but the trailing `\n` was left in the
  buffer, so the *next* `gNum` read it as an empty line and silently took its
  default — the operator's typed value was skipped.

## Fixes applied

1. Intro now uses `gAsk(...); if (gQuit) return;` so `s` continues and only `q`
   aborts.
2. Step 8 reads sensor 3's current `0xB0/0xB5/0xB6/0xB8` and uses those as the
   bracket defaults, so Enter truly keeps the existing value.
3. Comment corrected (no `h` key).
4. Duplicate guard removed.
5. Documented above; no code change.
6. Every prompt now advertises **`g`** (`[g]=go  s=skip  q=quit`); any key still
   works, so `g` is just the documented "go" key for monitors that swallow Enter.
7. `gNum` now breaks on `'\r'` **or** `'\n'` and then drains any trailing CR/LF,
   so CR-only, LF-only and CRLF line endings all submit exactly one entry.
   The boot banner was trimmed to keep the build inside the flash budget
   (28,608 / 28,672 bytes, 64 bytes headroom).

## Behaviour walk-through (all paths)

| Situation | Result |
|---|---|
| `p` pressed | motors stopped, live view off, input flushed, intro shown |
| `g` (or any key) at a prompt | step runs |
| Enter at a prompt (monitor sends CR/LF) | step runs |
| Enter at a prompt (monitor sends nothing) | use `g` instead |
| `s` at a step | step skipped, procedure continues |
| `q` (or 2 min with no key) | procedure aborts, nothing saved |
| Step 1, `s` on white | white skipped, black still offered |
| Step 1, `q` | abort |
| Step 3–6 sweep | register written per value, original restored at the end |
| Step 7 | single verify measurement |
| Step 8, Enter on all prompts | sensor 3's current values re-applied to all 5 and saved |
| Step 8, CR-only / LF-only / CRLF line ending | entry submitted exactly once |
| Step 8, `q` | nothing applied |
| Any measurement | keys typed during it are flushed before the next prompt |

## Not verified

- Bench test on hardware (no robot attached during this change).
- The `K` sweep timing feel at 800 ms/value (~10 s for the B0 sweep).
