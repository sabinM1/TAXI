#!/usr/bin/env python3
"""
filter_sim.py — replay raw ToF captures through the firmware's phantom filter.

The firmware (TAXI_Firmware/Sensors.h, tofReadOne) only publishes a reading
after TOF_CONSECUTIVE_VALID valid reads in a row. This script replays real
captures from the bench sketch's 'X' / 'XALL' command and reports, per sensor:

  1. the raw burst-length histogram (how long each valid run lasts),
  2. the DISTANCE histogram of the valid reads, and
  3. for every candidate streak length N, how many phantom events survive.

Input: one or more RAW-BEGIN [id] ... RAW-END blocks pasted from the Serial
Monitor. Each line is a distance in mm (valid) or -1 (invalid). The older
'V'/'.' character format is still accepted.

Usage:
    python filter_sim.py capture.txt
    python filter_sim.py < capture.txt
"""

import sys
from collections import Counter


def load_captures(text):
    """Return a list of (id, reads) where reads is a list of ints (mm or -1)."""
    captures = []
    cur_id = None
    cur = []
    in_block = False

    for ln in text.splitlines():
        s = ln.strip()
        if s.startswith("RAW-BEGIN"):
            in_block = True
            cur = []
            parts = s.split()
            cur_id = parts[1] if len(parts) > 1 else "?"
            continue
        if s == "RAW-END":
            if in_block:
                captures.append((cur_id, cur))
            in_block = False
            cur = []
            continue
        if not s:
            continue
        if not in_block:
            continue
        if set(s) <= set("V."):
            cur.extend(0 if c == "V" else -1 for c in s)
            continue
        try:
            cur.append(int(s))
        except ValueError:
            continue

    if in_block and cur:
        captures.append((cur_id, cur))
    return captures


def replay(reads, n):
    """Mirror Sensors.h tofReadOne(): publish only after n valid reads in a row."""
    streak = 0
    published = []
    for d in reads:
        if d >= 0:
            if streak < n:
                streak += 1
        else:
            streak = 0
        published.append(streak >= n)
    return published


def runs(flags):
    lengths = []
    run = 0
    for f in flags:
        if f:
            run += 1
        elif run:
            lengths.append(run)
            run = 0
    if run:
        lengths.append(run)
    return lengths


def analyse(cid, reads):
    total = len(reads)
    valid = [d for d in reads if d >= 0]
    print("=" * 60)
    print(f"SENSOR id {cid}: {total} reads, {len(valid)} valid "
          f"({len(valid) * 100.0 / total:.1f}%)")
    print("=" * 60)
    if total == 0:
        print("  (no data)\n")
        return

    # burst lengths
    bl = runs([d >= 0 for d in reads])
    print("raw valid-run lengths (bursts before filtering):")
    if bl:
        for k in sorted(Counter(bl)):
            print(f"  {k:5d} x{Counter(bl)[k]:5d}  "
                  f"{'#' * min(Counter(bl)[k], 50)}")
    else:
        print("  (none)")
    print()

    # distance histogram
    if valid:
        buckets = Counter((d // 25) * 25 for d in valid)
        print("valid-read distance histogram (25 mm buckets):")
        for k in sorted(buckets):
            print(f"  {k:4d}-{k + 24:4d} mm x{buckets[k]:5d}  "
                  f"{'#' * min(buckets[k], 50)}")
        print(f"valid distance: min={min(valid)} max={max(valid)} "
              f"mean={sum(valid) / len(valid):.0f} mm")
        print()

    # filter sweep
    print(f"{'N':>3} {'published':>10} {'pub%':>6} "
          f"{'events':>7} {'ev/1k':>7} {'longest':>8}")
    print("-" * 48)
    best = None
    for n in range(1, 11):
        pub = replay(reads, n)
        npub = sum(pub)
        lengths = runs(pub)
        nevents = len(lengths)
        longest = max(lengths) if lengths else 0
        print(f"{n:>3} {npub:>10} {npub * 100.0 / total:>5.1f}% "
              f"{nevents:>7} {nevents * 1000.0 / total:>7.1f} {longest:>8}")
        if nevents == 0 and (best is None or npub > best[1]):
            best = (n, npub)

    print()
    if best:
        print(f"==> cleanest: N={best[0]} (0 phantom events, "
              f"{best[1]} frames published)")
    else:
        print("==> no N up to 10 gives 0 events: phantoms are SUSTAINED.")
    print()


def main():
    if len(sys.argv) > 1:
        with open(sys.argv[1], "r", encoding="utf-8", errors="replace") as f:
            text = f.read()
    else:
        text = sys.stdin.read()

    captures = load_captures(text)
    if not captures:
        print("No RAW-BEGIN..RAW-END data found.")
        return 1

    for cid, reads in captures:
        analyse(cid, reads)

    # summary across sensors
    if len(captures) > 1:
        print("=" * 60)
        print("SUMMARY (valid% on empty space — lower is healthier)")
        print("=" * 60)
        for cid, reads in captures:
            if not reads:
                continue
            v = sum(1 for d in reads if d >= 0)
            pct = v * 100.0 / len(reads)
            flag = "  <-- FAULTY" if pct > 5 else "  ok"
            print(f"  id {cid}: {pct:5.1f}% valid{flag}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
