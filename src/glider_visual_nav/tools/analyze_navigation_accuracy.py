#!/usr/bin/env python3
"""Offline accuracy decomposition; consumes evaluation logs only."""
import argparse
import json
import math
from pathlib import Path

def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))

def analyze(path):
    rows=[]
    with path.open() as stream:
        for line in stream:
            item=json.loads(line)
            if 'nav' in item and 'truth' in item:
                rows.append(item)
    def rmse(values):
        return math.sqrt(sum(v*v for v in values)/len(values)) if values else None
    result={'source':str(path),'samples':len(rows)}
    for label, subset in [('all',rows),('banked',[r for r in rows if abs(r['truth']['roll'])>.15])]:
        result[label]=dict(samples=len(subset),
            roll_rmse_deg=rmse([math.degrees(wrap(r['nav']['roll']-r['truth']['roll'])) for r in subset]),
            yaw_rmse_deg=rmse([math.degrees(wrap(r['nav']['yaw']-r['truth']['yaw'])) for r in subset]),
            vx_rmse=rmse([r['nav']['vx']-r['truth']['vx'] for r in subset]),
            vy_rmse=rmse([r['nav']['vy']-r['truth']['vy'] for r in subset]))
    result['note']='Recorded observation window, not full flight. Roll and velocity errors are distinct; no time offset is inferred from aggregate RMSE.'
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=analyze(args.input)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2))
