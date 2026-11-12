#!/usr/bin/env python3
"""Offline comparison of MAVLink emission stamps, FDM, and independent evaluator."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pymavlink import DFReader
T=np.array([[0,1,0],[1,0,0],[0,0,-1]])
S=np.diag([1,-1,-1])
def analyze(run,evaluation):
    packets=[json.loads(s) for s in (run/'mavlink_packets.jsonl').read_text().splitlines()]
    truth=[json.loads(s) for s in evaluation.read_text().splitlines() if json.loads(s)['kind']=='truth']
    t=np.array([s['t'] for s in truth])
    v=np.array([np.linalg.norm(s['v']) for s in truth])
    angles=np.unwrap(Rotation.from_quat([s['q'] for s in truth]).as_euler('xyz'),axis=0)
    out={'run':str(run),'evaluation':str(evaluation)}
    for kind in ('RAW_IMU','ATTITUDE','SCALED_PRESSURE'):
        rows=[s for s in packets if s['mavpackettype']==kind and np.interp(s['clock'],t,v)>5]
        if not rows:continue
        boot=np.array([s['time_usec']*1e-6 if kind=='RAW_IMU' else s['time_boot_ms']*.001 for s in rows])
        clock=np.array([s['clock'] for s in rows]);age=clock-boot
        stats=dict(samples=len(rows),emission_age_max_s=float(max(age)),emission_age_p99_s=float(np.percentile(age,99)),
                   emission_age_median_s=float(np.median(age)))
        if kind=='ATTITUDE':
            rot=Rotation.from_matrix(T@Rotation.from_euler('xyz',[[s['roll'],s['pitch'],s['yaw']] for s in rows]).as_matrix()@S)
            pred=np.column_stack([np.interp(clock,t,angles[:,j]) for j in range(3)])
            e=rot.as_euler('xyz')-pred;e=np.arctan2(np.sin(e),np.cos(e))
            stats['angle_rmse_at_receipt_deg']=np.degrees(np.sqrt(np.mean(e**2,axis=0))).tolist()
        out[kind]=stats
    files=list((run/'logs').glob('*.BIN'))
    if files:
        reader=DFReader.DFReader_binary(str(files[0]));offsets=[]
        while True:
            m=reader.recv_match(type=['JSN1'])
            if m is None:break
            offsets.append(m.TStamp-m.TimeUS*1e-6)
        if offsets:
            out['jsn_clock_offset_s']=dict(samples=len(offsets),min=float(min(offsets)),max=float(max(offsets)))
    out['note']='MAVLink emission age includes HAL/physics clock offset; it is not gyro filter group delay.'
    return out
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('evaluation',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=analyze(a.run,a.evaluation);a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
