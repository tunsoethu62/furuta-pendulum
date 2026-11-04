"""Compare balance runs (gain set A vs B) for the handout's task 6. Standard library only.

Usage: python analysis/compare_runs.py data/<balance log>.csv [more logs ...]
Each BALANCE segment (mode 2) is summarised: duration, pendulum d rms/max, arm theta range, control effort.
Disturbances: every time |d| exceeds 2 deg after being settled, the peak tilt and the time to get back inside
+-1 deg (and stay there 0.5 s) are reported.
"""
import csv, math, sys

def segments(path):
    rows = []
    for r in csv.reader(open(path, encoding="utf-8", errors="replace")):
        if len(r) >= 6 and r[0].strip().isdigit():
            gains = r[6].strip() if len(r) > 6 else "A"   # logs before the A/B update are gain set A
            rows.append((int(r[0]) / 1000, float(r[1]), float(r[2]), float(r[3]), int(r[5]), gains))
    seg, out = [], []
    for row in rows:
        if row[4] == 2: seg.append(row)
        elif seg: out.append(seg); seg = []
    if seg: out.append(seg)
    return out

def disturbances(s):
    res, i = [], 0
    settled_after = s[0][0] + 2.0
    while i < len(s):
        t, _, d = s[i][0], s[i][1], s[i][2]
        if t > settled_after and abs(d) > 2.0:
            start, peak, j = t, abs(d), i
            while j < len(s) and not (abs(s[j][2]) < 1.0 and all(abs(x[2]) < 1.0 for x in s[j:j + 50])):
                peak = max(peak, abs(s[j][2])); j += 1
            rec = s[j][0] - start if j < len(s) else None
            arm = max(abs(x[1]) for x in s[i:j + 1])
            res.append((start - s[0][0], peak, rec, arm))
            i = j + 50; settled_after = s[min(i, len(s) - 1)][0]
        else: i += 1
    return res

def rms(v): return math.sqrt(sum(x * x for x in v) / len(v)) if v else float("nan")

for path in sys.argv[1:]:
    for s in segments(path):
        calm = [x for x in s if x[0] > s[0][0] + 2.0] or s
        print(f"{path} | gains {s[0][5]} | {s[-1][0] - s[0][0]:.1f} s | d rms {rms([x[2] for x in calm]):.2f} deg, "
              f"max {max(abs(x[2]) for x in s):.2f} | theta {min(x[1] for x in s):.1f}..{max(x[1] for x in s):.1f} deg | "
              f"u rms {rms([x[3] for x in calm]):.1f} rad/s^2, max {max(abs(x[3]) for x in s):.0f}")
        for t0, peak, rec, arm in disturbances(s):
            print(f"    disturbance at {t0:.1f} s: peak tilt {peak:.1f} deg, recovered in "
                  f"{'%.2f s' % rec if rec is not None else 'not within log'}, max |theta| {arm:.1f} deg")
