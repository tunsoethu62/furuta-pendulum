"""Decode v2.1-diag #D lines into named CSV; never accesses hardware.

python analysis/decode_swingup_diagnostics.py path/to/device-monitor.log out.csv
Non-diagnostic startup text is ignored; malformed diagnostic lines are counted.
"""
import csv
import sys
from collections import Counter
from pathlib import Path

FIELDS = [
    'alpha_rad', 'peak_rad', 'energy_before', 'energy_after', 'loss',
    'planned_vp_rad_s', 'target_raw_rad_s', 'target_clipped_rad_s',
    'v_before_rad_s', 'v_after_rad_s', 'z_rad_s', 'raw_a_rad_s2',
    'work_lower_positive', 'work_lower_negative', 'work_upper_positive',
    'work_upper_negative', 'reversal_delta_v_rad_s', 'theta_rad',
    'rate_rad_s', 'trim_rad', 'u_rad_s2', 'turn_status',
]
SCALES = [1000]*10 + [100,10] + [1]*4 + [1000,1000,100,1000,10,1]
HEADER = ['t_ms','kind','direction','mode','reason','reject_mask','dropped','events','message_code']+FIELDS

def decode(lines):
    rows=[]; malformed=0
    for line in lines:
        if not line.startswith('#D,'): continue
        parts=line.strip().split(',')
        if len(parts) not in (30,31):
            malformed+=1; continue
        try:
            ms=int(parts[1]); kind=parts[2]
            meta=list(map(int,parts[3:9])); raw=list(map(int,parts[9:]))
            if len(raw)==21: raw.append(None) # legacy v2.1 diagnostic schema
            values=[None if x is None or x==-32768 else x/s for x,s in zip(raw,SCALES)]
            message=raw[0] if kind=='M' else ''
            if kind=='M': values=['']*22
            rows.append([ms,kind,*meta,message,*values])
        except ValueError:
            malformed+=1
    return rows,malformed

def main():
    source, output=map(Path,sys.argv[1:3])
    if source.resolve()==output.resolve(): raise SystemExit('Output must differ from input.')
    rows,bad=decode(source.read_text(encoding='utf-8',errors='replace').splitlines())
    if not rows: raise SystemExit('No valid #D records; source left unchanged.')
    # Refuse to overwrite an existing analysis/user file.
    with output.open('x',newline='',encoding='utf-8') as f:
        writer=csv.writer(f);writer.writerow(HEADER);writer.writerows(rows)
    print(f'{len(rows)} records; malformed={bad}; dropped={max(r[6] for r in rows)}; events={dict(Counter(r[1] for r in rows))}')
if __name__=='__main__': main()
