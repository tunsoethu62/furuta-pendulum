"""Build and execute host tests of firmware controller.h; no hardware access.

uv run --no-project --with ziglang python analysis/run_swingup_checks.py
Generated binaries/CSV are under ignored .pio/swingup-host/ (not measured data).
Alternatively compile analysis/swingup_sim.cpp with a local C++11 compiler.
"""
from pathlib import Path
import subprocess
import sys
import os

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / ".pio" / "swingup-host"
OUT.mkdir(parents=True, exist_ok=True)
binary = OUT / ("checks.exe" if sys.platform == "win32" else "checks")
compiler = [sys.executable, "-m", "ziglang", "c++", "-std=c++11", "-O2", "-UNDEBUG",
            "-nostdlib++", "-fno-exceptions", "-fno-rtti"]
qualification = OUT / ("qualification.exe" if os.name == "nt" else "qualification")
subprocess.run(compiler + [str(ROOT / "tests" / "turn_qualification.cpp"),
                         "-o", str(qualification)], check=True)
subprocess.run([str(qualification)], check=True)
parity = OUT / ("parity.exe" if os.name == "nt" else "parity")
subprocess.run(compiler + ["-I", str(ROOT / "tests" / "swingup_stubs"),
               str(ROOT / "tests" / "balance_parity.cpp"), "-o", str(parity)], check=True)
subprocess.run([str(parity)], check=True)
subprocess.run(compiler + [str(ROOT / "analysis" / "swingup_sim.cpp"), "-o", str(binary)], check=True)
subprocess.run([str(binary), str(OUT)], check=True)
adapter = OUT / ("adapter.exe" if os.name == "nt" else "adapter")
subprocess.run(compiler + ["-I", str(ROOT / "tests" / "swingup_stubs"),
               str(ROOT / "tests" / "swingup_adapter.cpp"), "-o", str(adapter)], check=True)
subprocess.run([str(adapter)], check=True)
diagnostics = OUT / ("diagnostics.exe" if os.name == "nt" else "diagnostics")
subprocess.run(compiler + [str(ROOT / "tests" / "swingup_diagnostics.cpp"),
                         "-o", str(diagnostics)], check=True)
fixture = OUT / "diagnostic_fixture.log"
subprocess.run([str(diagnostics),str(fixture)], check=True)
sys.path.insert(0,str(ROOT / "analysis"))
from decode_swingup_diagnostics import decode, HEADER
rows, malformed = decode(fixture.read_text().splitlines())
assert rows and not malformed and all(len(row)==len(HEADER) for row in rows)
assert decode(['#D,broken'])[1] == 1
print('PASS: generated diagnostic frames decode with exact schema; malformed frame counted')
# Historical diagnostic logs remain readable after appending turn_status.
legacy = ROOT / 'data' / 'device-monitor-261104-160607.log'
if legacy.exists():
    old_rows, bad = decode(legacy.read_text().splitlines())
    assert len(old_rows)==469 and bad==0 and all(len(row)==len(HEADER) for row in old_rows)
    print('PASS: 469 legacy diagnostic records still decode')
    import csv
    trace=OUT/'turn_trace.txt'
    # Original decoded CSV remains a measured artifact; replay fixture is derived.
    with (ROOT/'data'/'swingup_diagnostic_2026-11-04_1606.csv').open() as source:
        samples=list(csv.DictReader(source))
    trace.write_text(''.join(f"{r['t_ms']} {r['kind']} {r['alpha_rad']} {r['peak_rad']} {r['rate_rad_s']}\n"
                            for r in samples if r['mode']=='3' and r['kind']!='M'))
    replay=OUT/'turn_trace_replay.exe'
    subprocess.run(compiler+[str(ROOT/'tests'/'turn_trace_replay.cpp'),'-o',str(replay)],check=True)
    subprocess.run([str(replay),str(trace)],check=True)
