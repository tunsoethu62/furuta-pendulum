"""Free-decay analysis for the pendulum (arm held still). Usage: python analysis/decay_analysis.py data/<log>.csv
Prints the period, the peaks, the logarithmic decrements (viscous model) and the amplitude drop per half
cycle (dry/Coulomb friction model). Standard library only."""
import csv, math, sys

def load(path):
    rows = [r for r in csv.reader(open(path, encoding="utf-8", errors="replace")) if r and r[0].strip().isdigit()]
    return [int(r[0]) / 1e6 for r in rows], [int(r[2]) for r in rows]

def main(path):
    t, u = load(path)
    rest = sum(u[-50:]) / 50                                   # final rest position (counts)
    rel = next(i for i in range(len(u)) if abs(u[i] - u[0]) > 30)  # start of the manual lift
    fast = next(i for i in range(rel, len(u) - 1) if abs(u[i + 1] - u[i]) > 40)   # first fast sample = falling after release
    lo = fast
    while lo > rel and (u[lo] - u[lo - 1]) * (u[fast + 1] - u[fast]) > 0: lo -= 1  # walk back to the release point
    ext = [lo]
    for i in range(lo + 1, len(u) - 1):
        if (u[i] - u[i - 1]) * (u[i + 1] - u[i]) <= 0 and abs(u[i] - u[ext[-1]]) > 20 and (u[i] - rest) * (u[ext[-1]] - rest) < 0:
            ext.append(i)
    deg = lambda c: c * 360 / 4096
    print(f"samples {len(t)}, dt {(t[-1]-t[0])/(len(t)-1)*1000:.1f} ms, rest {rest:.0f} counts")
    peaks = [(t[i], deg(u[i] - rest)) for i in ext if abs(deg(u[i] - rest)) > 2]  # ignore the final stop
    for tp, a in peaks: print(f"  t={tp:5.2f} s  amplitude {a:+6.1f} deg")
    half = [b[0] - a[0] for a, b in zip(peaks, peaks[1:])]
    T = 2 * sum(half[1:]) / len(half[1:]) if len(half) > 1 else float("nan")
    print(f"period T ~ {T:.3f} s  -> omega_d ~ {2*math.pi/T:.2f} rad/s")
    pp = [abs(a[1]) + abs(b[1]) for a, b in zip(peaks, peaks[1:])]
    print("peak-to-peak per half cycle [deg]:", [round(x, 1) for x in pp])
    drops = [a - b for a, b in zip(pp, pp[1:])]
    print("drop of peak-to-peak per half cycle [deg]:", [round(x, 1) for x in drops])
    if drops:
        d = sum(drops) / len(drops) / 2   # linear (Coulomb) decay: amplitude falls by d each half cycle
        print(f"Coulomb model: amplitude drop ~{d:.1f} deg per half cycle -> T_c / (m_p g l_p) ~ {math.radians(d)/2:.3f}")
    for side in (1, -1):
        a = [abs(x[1]) for x in peaks if x[1] * side > 0]
        dl = [math.log(p / q) for p, q in zip(a, a[1:]) if q > 2]
        print(f"side {'+' if side > 0 else '-'}: amplitudes {[round(x, 1) for x in a]}, log decrements {[round(d, 3) for d in dl]}, "
              f"zeta {[round(d / math.sqrt(4*math.pi**2 + d*d), 3) for d in dl]}")

if __name__ == "__main__": main(sys.argv[1])
