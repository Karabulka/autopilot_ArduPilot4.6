"""Export standalone scientific plots and copy completed PX4 logs beside evaluator telemetry."""
from pathlib import Path
import json
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parents[3]
logs=root/'log_visual_nav'
for name in ['cruise_final','glide_02','polarity_final','fallback_final']:
    report_path=logs/(name+'.json')
    report=json.loads(report_path.read_text())
    directory=root/'build_visual_nav'/('flight_'+name)/'log'
    candidates=list(directory.rglob('*.ulg')) if directory.exists() else []
    if candidates:
        destination=logs/(name+'.ulg')
        shutil.copy2(max(candidates,key=lambda path:path.stat().st_mtime),destination)
        report['px4_ulog']=str(destination)
    if name=='polarity_final':
        for key in ['cross_track_rmse_straight_m','cross_track_rmse_waypoint_change_m']:
            report['metrics'][key]=None  # a predefined step is not a waypoint-route trial
    report_path.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')

plt.rcParams.update({'font.size':9,'axes.grid':True,'grid.alpha':.25})
fig,axes=plt.subplots(2,2,figsize=(12,9),constrained_layout=True)
errors,error_axes=plt.subplots(2,3,figsize=(15,7),constrained_layout=True)
energy,energy_axes=plt.subplots(2,2,figsize=(12,7),constrained_layout=True)
for row,(name,label) in enumerate([('cruise_final','Cruise, powered'),('glide_02','Unpowered glide')]):
    report=json.loads((logs/(name+'.json')).read_text())
    records=[json.loads(line) for line in (logs/(name+'.jsonl')).read_text().splitlines()]
    matched=[a for a in records if a['kind']=='matched']
    truth=[a for a in records if a['kind']=='truth' and a['t']>=report['launch_time']]
    time=np.array([a['t']-report['launch_time'] for a in matched])
    true_time=np.array([a['t']-report['launch_time'] for a in truth])
    actual=np.array([a['p_truth'] for a in matched]); estimated=np.array([a['p_nav'] for a in matched])
    ax=axes[row,0]
    ax.plot([0,100,100,0,0],[0,0,100,100,0],'k--',alpha=.5,label='Waypoint route')
    ax.plot(actual[:,0],actual[:,1],label='Physics truth')
    ax.plot(estimated[:,0],estimated[:,1],label='Visual/magnetic odometry')
    ax.scatter([0,100,100,0],[0,0,100,100],c='k',s=18)
    ax.set(title=label,xlabel='East [m]',ylabel='North [m]',aspect='equal'); ax.legend(fontsize=8)
    ax=axes[row,1]
    ax.plot(time,actual[:,2],label='Physics base_link height')
    ax.plot(time,estimated[:,2],label='Barometric estimate')
    if row==0: ax.axhline(40.25,color='k',linestyle='--',label='Nominal body target (camera h=40 m)')
    ax.set(title=label+' altitude',xlabel='Time after release [s]',ylabel='Height [m]'); ax.legend(fontsize=8)
    v_error=np.array([a['velocity_error'] for a in matched])
    ax=error_axes[row,0]; ax.plot(time,v_error[:,0],label='Vx'); ax.plot(time,v_error[:,1],label='Vy')
    ax.set(title=label+' body velocity error',xlabel='Time [s]',ylabel='Error [m/s]'); ax.legend()
    ax=error_axes[row,1]; ax.plot(time,[a['yaw_error_deg'] for a in matched],color='purple')
    ax.set(title=label+' magnetic yaw error',xlabel='Time [s]',ylabel='Error [deg]')
    ax=error_axes[row,2]
    ax.plot(time,[np.linalg.norm(a['drift_xy']) for a in matched],label='XY drift magnitude')
    ax.plot(time,[a['cross_track'] for a in matched],label='Signed cross-track')
    ax.set(title=label+' position/path error',xlabel='Time [s]',ylabel='Error [m]'); ax.legend(fontsize=8)
    ax=energy_axes[row,0]
    ax.plot(true_time,[np.linalg.norm(a['v_body']) for a in truth],label='Physics speed')
    ax.axhline(12 if row==0 else 8.5,color='k',linestyle='--',label='Speed target')
    ax.set(title=label+' speed',xlabel='Time after release [s]',ylabel='Speed [m/s]'); ax.legend()
    commands=[a for a in records if a['kind']=='px4_setpoint' and a['t']>=report['launch_time']]
    ax=energy_axes[row,1]
    ax.plot([a['t']-report['launch_time'] for a in commands],[a['thrust'][0] for a in commands],label='PX4 normalized thrust setpoint')
    motor=[a for a in records if a['kind']=='gazebo_motor_command' and a['t']>=report['launch_time']]
    ax.plot([a['t']-report['launch_time'] for a in motor],[a['rad_s']/1000 for a in motor],label='Gazebo rotor command / 1000',alpha=.7)
    ax.set(title=label+' actuation',xlabel='Time [s]',ylabel='Normalized command',ylim=(-.05,1.05)); ax.legend(fontsize=8)
fig.savefig(logs/'sprint3_trajectories.png',dpi=160)
errors.savefig(logs/'sprint3_errors.png',dpi=160)
energy.savefig(logs/'sprint3_height_speed_actuation.png',dpi=160)
print('Exported plots and completed PX4 ULogs into',logs)
