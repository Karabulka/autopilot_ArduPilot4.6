#!/usr/bin/env python3
"""Combine independent results without changing thresholds or source flags."""
import argparse,json
from pathlib import Path
def combine(flight,accuracy,transport):
    checks={
        'flight':bool(flight.get('passed') and flight.get('airborne_seconds',0)>=90),
        'navigation_accuracy':bool(accuracy.get('accuracy_passed')),
        'arduplane_roll_pitch':max(transport.get('ATTITUDE',{}).get('angle_rmse_at_receipt_deg',[float('inf')])[:2])<1.5,
        'mavlink_imu_emission_age':transport.get('RAW_IMU',{}).get('emission_age_max_s',float('inf'))<.025,
        'mavlink_pressure_emission_age':transport.get('SCALED_PRESSURE',{}).get('emission_age_max_s',float('inf'))<.025,
    }
    return dict(passed=all(checks.values()),checks=checks,flight=flight,accuracy={**accuracy, 'passed':bool(accuracy.get('accuracy_passed'))},transport=transport,
                sensor_source='direct FLU IMU + static pressure; ArduPlane control via MAVLink',
                limitations=['Ideal Gazebo sensor orientation, flat ground, zero wind; not hardware validation',
                             'ROS /clock can lag sensor delivery; negative delivery ages are retained'])
if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('flight','accuracy','transport','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    r=combine(json.loads(a.flight.read_text()),json.loads(a.accuracy.read_text()),json.loads(a.transport.read_text()))
    a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r['checks'],indent=2));print('passed=',r['passed'])
