"""Offline 15/18 degree comparison. Does not create/upload an 18-degree firmware.

uv run --no-project --with ziglang python analysis/evaluate_capture_windows.py
An 18-degree rollout failure is a reported evaluation result, not suppressed.
"""
from pathlib import Path
import json
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'.pio'/'capture-window-evaluation'
OUT.mkdir(parents=True,exist_ok=True)
compiler=[sys.executable,'-m','ziglang','c++','-std=c++11','-O2','-UNDEBUG',
          '-nostdlib++','-fno-exceptions','-fno-rtti']
results={}
for window in (15,18):
    args=[f'-DSWING_CAPTURE_DEG={window}.0f']
    results[window]={}
    for name,source in [('grid','capture_window_eval.cpp'),('full','swingup_sim.cpp')]:
        binary=OUT/f'{name}{window}.exe'
        subprocess.run(compiler+args+[str(ROOT/'analysis'/source),'-o',str(binary)],check=True)
        run_dir=OUT/f'{name}{window}'
        run_dir.mkdir(exist_ok=True)
        run=subprocess.run([str(binary),str(run_dir)],capture_output=True,text=True)
        output=run.stdout+run.stderr
        (OUT/f'{name}{window}.txt').write_text(output)
        results[window][name]={'exit_code':run.returncode,'output':output}
        print(f'--- {window} degrees, {name}, exit={run.returncode} ---\n{output}')
(OUT/'results.json').write_text(json.dumps(results,indent=2))
if results[15]['grid']['exit_code'] or results[15]['full']['exit_code']:
    raise SystemExit('Default 15-degree validation failed.')
if results[18]['grid']['exit_code']:
    raise SystemExit('18-degree grid evaluation could not run.')
if results[18]['full']['exit_code']:
    raise SystemExit('18-degree full swing-up validation failed (v2.3 firmware default is 18).')
print('15 and 18 degree results are retained in results.json; v2.3 firmware default is 18 degrees.')
