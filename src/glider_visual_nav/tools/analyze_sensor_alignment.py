#!/usr/bin/env python3
"""Offline evaluation only: compare independent truth at acquisition timestamps."""
import argparse, json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def analyze(path):
    data = [json.loads(s) for s in path.read_text().splitlines()]
    groups = {k: [r for r in data if r['kind'] == k] for k in ('truth', 'imu', 'nav', 'height', 'flow', 'direct_imu')}
    gt = groups['truth']
    t = np.array([r['t'] for r in gt])
    values = {k: np.array([r[k] for r in gt]) for k in ('v', 'w', 'p')}
    angles = np.unwrap(Rotation.from_quat([r['q'] for r in gt]).as_euler('xyz'), axis=0)

    def interp(a, ts):
        return np.column_stack([np.interp(ts, t, a[:, j]) for j in range(a.shape[1])])
    out = {'source': str(path), 'counts': {k: len(v) for k, v in groups.items()}}
    for kind in ('imu', 'nav', 'flow', 'height', 'direct_imu'):
        rows = groups[kind]
        if not rows:
            continue
        ts = np.array([r['t'] for r in rows])
        speed = np.linalg.norm(interp(values['v'], ts), axis=1)
        valid = (ts >= t[0]) & (ts <= t[-1]) & (speed > 5)
        rows = [r for r, ok in zip(rows, valid) if ok]
        ts = ts[valid]
        if not rows:
            continue
        age = np.array([r['rx'] - r['t'] for r in rows])
        errors = {'delivery_age_max_s': float(np.max(age)), 'delivery_age_min_s': float(np.min(age)), 'delivery_age_p99_s': float(np.percentile(age, 99))}
        if kind in ('imu', 'nav'):
            e = Rotation.from_quat([r['q'] for r in rows]).as_euler('xyz') - interp(angles, ts)
            e = np.arctan2(np.sin(e), np.cos(e))
            errors['angle_rmse_deg'] = np.sqrt(np.mean(e ** 2, axis=0)).tolist()
            errors['angle_rmse_deg'] = (np.array(errors['angle_rmse_deg']) * 180 / np.pi).tolist()
        if kind in ('imu', 'direct_imu'):
            w = np.array([r['w'] for r in rows])
            errors['gyro_rmse_rad_s'] = np.sqrt(np.mean((w - interp(values['w'], ts)) ** 2, axis=0)).tolist()
            scan = []
            for shift in np.linspace(-2.0, 2.0, 401):
                e = w - interp(values['w'], ts + shift)
                scan.append((float(np.mean(e ** 2)), float(shift)))
            errors['gyro_best_shift_s'] = min(scan)[1]
        if kind in ('nav', 'flow'):
            errors['velocity_rmse_m_s'] = np.sqrt(np.mean((np.array([r['v'] for r in rows]) - interp(values['v'], ts)) ** 2, axis=0)).tolist()
        if kind == 'height':
            e = np.array([r['z'] for r in rows]) - interp(values['p'], ts)[:, 2]
            errors['body_height_rmse_m'] = float(np.sqrt(np.mean(e ** 2)))
            errors['body_height_max_abs_m'] = float(np.max(abs(e)))
        out[kind] = dict(samples=len(rows), **errors)
    nav = out.get('nav', {})
    out['accuracy_passed'] = bool(nav.get('samples', 0) >= 100 and max(nav.get('velocity_rmse_m_s', [float('inf')])[:2]) < 0.5 and (max(nav.get('angle_rmse_deg', [float('inf')])[::2]) < 2.5) and (out.get('height', {}).get('body_height_max_abs_m', float('inf')) < 0.5) and (out.get('imu', {}).get('delivery_age_max_s', float('inf')) < 0.025) and (out.get('height', {}).get('delivery_age_max_s', float('inf')) < 0.025))
    out['note'] = 'Flight smoke passed is separate from accuracy_passed. Offsets are offline diagnostics, never truth input.'
    return out
if __name__ == '__main__':
    a = argparse.ArgumentParser()
    a.add_argument('input', type=Path)
    a.add_argument('--output', type=Path, required=True)
    args = a.parse_args()
    out = analyze(args.input)
    args.output.write_text(json.dumps(out, indent=2) + '\n')
    print(json.dumps(out, indent=2))
