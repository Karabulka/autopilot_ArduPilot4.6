"""Independent closed-loop evaluator. Truth is consumed ONLY here, never republished to control.

Writes raw acquisition-stamped telemetry (JSONL) and aggregate metrics, including failed runs.
The optional polarity stimulus is a predefined attitude step, independent of truth feedback.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import shutil
import time
import xml.etree.ElementTree as ET
import numpy as np


def rotation(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--regime',choices=['cruise_const_alt','unpowered_glide'],default='cruise_const_alt')
    parser.add_argument('--duration',type=float,default=55)
    parser.add_argument('--initial-height',type=float,default=40)
    parser.add_argument('--glide-speed',type=float,default=8.5)
    parser.add_argument('--output',required=True)
    parser.add_argument('--polarity',action='store_true')
    parser.add_argument('--drop-visual-at',type=float,default=0,help='fault test: terminate only the owned optical-flow child')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[3]
    output=Path(args.output); output.parent.mkdir(parents=True,exist_ok=True)
    directory=root/'build_visual_nav'/('flight_'+output.stem)
    directory.mkdir(parents=True,exist_ok=True)
    world=ET.parse(root/'Tools/simulation/gz/worlds/visual_nav_flight.sdf')
    world.getroot().find("world/include[name='rc_cessna_visual']/pose").text=f'0 0 {args.initial_height+.25} 0 0 0'
    world_path=directory/'world.sdf'; world.write(world_path,encoding='unicode')
    os.environ.update(ROS_DOMAIN_ID='66',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',ROS_LOCALHOST_ONLY='0',
                      ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT',RMW_FASTRTPS_USE_QOS_FROM_XML='1',
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(root/'src/glider_visual_nav/config/fastdds_wsl.xml'),
                      FASTDDS_DEFAULT_PROFILES_FILE=str(root/'src/glider_visual_nav/config/fastdds_wsl.xml'),
                      GZ_IP='127.0.0.1',GZ_PARTITION='glider_flight_'+str(os.getpid()))
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String,Float64
    from rosgraph_msgs.msg import Clock
    from px4_msgs.msg import VehicleStatus,VehicleAttitudeSetpoint,ActuatorMotors
    log=output.with_suffix('.process.log').open('w')
    raw=output.with_suffix('.jsonl').open('w')
    report={'regime':args.regime,'initial_optical_height':args.initial_height,'passed':False,'metrics':{},'states':[]}
    command=['ros2','launch','glider_visual_nav','flight.launch.py','flight_regime:='+args.regime,
             'world_file:='+str(world_path),'px4_workdir:='+str(directory),'glide_speed:='+str(args.glide_speed)]
    if args.polarity:
        command.append('guidance_enabled:=false')
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(); node=rclpy.create_node('independent_flight_evaluator')
    truth=deque(maxlen=10000); pending=[]; records=[]; clock=[0.0]; status=[None]; state=['']; launch_time=[None]
    goal=[(100.,0.)]; goal_change=[0.0]; origin=[None]; path=[0.0]; last_path=[None]; targets=[]; thrust=[]; truth_rpy=[]; motors=[]; motor_commands=[]
    def stamp(msg): return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    def vector(v): return np.array([v.x,v.y,v.z])
    def quaternion(q): return np.array([q.x,q.y,q.z,q.w])
    def rpy(q):
        r=rotation(q)
        return math.atan2(r[2,1],r[2,2]),math.asin(np.clip(-r[2,0],-1,1)),math.atan2(r[1,0],r[0,0])
    def clean(value):
        if isinstance(value,dict): return {key:clean(item) for key,item in value.items()}
        if isinstance(value,(tuple,list)): return [clean(item) for item in value]
        if hasattr(value,'item'): return clean(value.item())
        if isinstance(value,float) and not math.isfinite(value): return None
        return value
    def write(kind,data): raw.write(json.dumps(clean({'kind':kind,**data}),allow_nan=False)+'\n')
    def on_truth(msg):
        t=stamp(msg); p=vector(msg.pose.pose.position); q=quaternion(msg.pose.pose.orientation)
        v=vector(msg.twist.twist.linear); a=rpy(q)
        truth.append((t,p,q,v))
        if origin[0] is None: origin[0]=p.copy()
        if launch_time[0] is not None:
            if last_path[0] is not None: path[0]+=float(np.linalg.norm(p[:2]-last_path[0][:2]))
            last_path[0]=p.copy(); truth_rpy.append((t,*a,p[2],float(np.linalg.norm(v))))
        write('truth',dict(t=t,p=p.tolist(),q=q.tolist(),v_body=v.tolist()))
    def on_odom(msg):
        pending.append((stamp(msg),vector(msg.pose.pose.position),quaternion(msg.pose.pose.orientation),vector(msg.twist.twist.linear),goal[0],goal_change[0]))
    def on_status(msg):
        status[0]=msg
        write('px4_status',dict(t=clock[0],arming_state=int(msg.arming_state),nav_state=int(msg.nav_state)))
    def on_state(msg):
        if msg.data!=state[0]:
            state[0]=msg.data; report['states'].append(dict(t=clock[0],state=msg.data)); print('state',clock[0],msg.data,flush=True)
    def on_target(msg):
        next_goal=(msg.pose.position.x,msg.pose.position.y)
        if next_goal!=goal[0]: goal_change[0]=stamp(msg)
        goal[0]=next_goal
        targets.append((stamp(msg),goal[0])); write('target',dict(t=stamp(msg),goal=goal[0],rpy=rpy(quaternion(msg.pose.orientation))))
    def on_setpoint(msg):
        thrust.append((clock[0],float(msg.thrust_body[0])))
        write('px4_setpoint',dict(t=clock[0],q=list(msg.q_d),thrust=list(msg.thrust_body)))
    def on_motor(msg):
        if launch_time[0] is not None:
            # PX4 ActuatorMotors defines NaN as motor stop (not an unknown thrust).
            motors.append(float(msg.control[0]) if math.isfinite(msg.control[0]) else 0.0)
            write('actuator_motor',dict(t=clock[0],control=float(msg.control[0])))
    def on_motor_command(msg):
        if launch_time[0] is not None and math.isfinite(msg.data):
            motor_commands.append(msg.data); write('gazebo_motor_command',dict(t=clock[0],rad_s=msg.data))
    subscriptions=[node.create_subscription(typ,topic,callback,qos) for typ,topic,callback,qos in [
        (Odometry,'/evaluation/ground_truth',on_truth,qos_profile_sensor_data),
        (Odometry,'/glider/odometry',on_odom,10),
        (VehicleStatus,'/fmu/out/vehicle_status_v1',on_status,qos_profile_sensor_data),
        (String,'/glider/interface_state',on_state,10),
        (PoseStamped,'/glider/target_pose',on_target,10),
        (VehicleAttitudeSetpoint,'/fmu/in/vehicle_attitude_setpoint_v1',on_setpoint,10),
        (ActuatorMotors,'/fmu/out/actuator_motors',on_motor,qos_profile_sensor_data),
        (Float64,'/evaluation/motor_speed_cmd',on_motor_command,qos_profile_sensor_data),
        (Clock,'/clock',lambda msg:clock.__setitem__(0,msg.clock.sec+msg.clock.nanosec*1e-9),qos_profile_sensor_data)]]
    stimulus=None
    def match():
        while pending and truth and pending[0][0]<truth[-1][0]-.03:
            t,p,q,v,g,changed=pending.pop(0)
            if launch_time[0] is None or t<launch_time[0]+1: continue
            samples=list(truth)
            index=next((i for i,a in enumerate(samples) if a[0]>=t),None)
            if index is None or index==0: continue
            a,b=samples[index-1],samples[index]
            if t-a[0]>.1 or b[0]-t>.1: continue
            alpha=(t-a[0])/(b[0]-a[0]); tp=a[1]*(1-alpha)+b[1]*alpha
            tq=a[2]*(1-alpha)+(b[2] if np.dot(a[2],b[2])>=0 else -b[2])*alpha; tq/=np.linalg.norm(tq)
            tv=a[3]*(1-alpha)+b[3]*alpha
            yaw=rpy(q)[2]; tyaw=rpy(tq)[2]; dyaw=math.atan2(math.sin(yaw-tyaw),math.cos(yaw-tyaw))
            wp=[(0.,0.),(100.,0.),(100.,100.),(0.,100.)]; end=min(range(4),key=lambda i:np.linalg.norm(np.array(wp[i])-np.array(g)))
            start=np.array(wp[(end-1)%4]); line=np.array(wp[end])-start
            local=tp[:2]-origin[0][:2]; cross=float(np.cross(line,local-start)/np.linalg.norm(line))
            record=dict(t=t,p_nav=p.tolist(),p_truth=tp.tolist(),v_nav=v.tolist(),v_truth=tv.tolist(),
                        velocity_error=(v-tv).tolist(),yaw_error_deg=math.degrees(dyaw),
                        drift_xy=(p[:2]-local).tolist(),cross_track=cross,goal=g,truth_roll=rpy(tq)[0],height=tp[2],
                        waypoint_transition=t-changed<4.0,
                        roll_error_deg=math.degrees(rpy(q)[0]-rpy(tq)[0]),pitch_error_deg=math.degrees(rpy(q)[1]-rpy(tq)[1]))
            records.append(record); write('matched',record)
    def spin_until(predicate,wall_timeout):
        end=time.monotonic()+wall_timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02); match()
            if process.poll() is not None: raise RuntimeError('launch exited')
            if predicate(): return
        raise RuntimeError('timeout: state='+state[0]+', truth='+str(len(truth))+', matched='+str(len(records)))
    try:
        spin_until(lambda:state[0]=='ACTIVE' and status[0] is not None and status[0].arming_state==2 and status[0].nav_state==14 and bool(truth),100)
        report['gps_parameters']={}
        import re
        for parameter in ['SYS_HAS_GPS','SIM_GPS_USED','EKF2_GPS_CTRL']:
            result=subprocess.run([str(root/'build/px4_sitl_default/bin/px4-param'),'show',parameter],capture_output=True,text=True,timeout=5)
            report['gps_parameters'][parameter]=result.stdout.strip()
            if not re.search(re.escape(parameter)+r'.*:\s*0\s*$',result.stdout,flags=re.M):
                raise RuntimeError('GPS disabling parameter was not zero: '+result.stdout)
        launch_time[0]=clock[0]
        reply=subprocess.run(['gz','service','-s','/evaluation/launch','--reqtype','gz.msgs.Vector3d',
                              '--reptype','gz.msgs.Boolean','--timeout','5000','--req','x:16 y:0 z:0'],capture_output=True,text=True,timeout=8)
        if reply.returncode or 'true' not in reply.stdout: raise RuntimeError('airlaunch failed: '+reply.stdout+reply.stderr)
        if args.polarity:
            stimulus=subprocess.Popen([sys.executable,str(root/'src/glider_visual_nav/tools/polarity_stimulus.py'),
                                       '--start-time',str(launch_time[0])],stdout=log,stderr=subprocess.STDOUT)
        wall_end=time.monotonic()+args.duration*5+60
        last_print=-1; fault_injected=False
        while clock[0]-launch_time[0]<args.duration:
            rclpy.spin_once(node,timeout_sec=.01); match()
            elapsed=clock[0]-launch_time[0]
            if args.drop_visual_at and elapsed>=args.drop_visual_at and not fault_injected:
                children=Path(f'/proc/{process.pid}/task/{process.pid}/children').read_text().split()
                for child in children:
                    commandline=Path(f'/proc/{child}/cmdline').read_bytes().split(b'\0')[0].decode()
                    if commandline==str(root/'install_visual_nav/glider_visual_nav/lib/glider_visual_nav/optical_flow_node'):
                        os.kill(int(child),signal.SIGTERM); fault_injected=True
                if not fault_injected: raise RuntimeError('owned optical child not found for fault injection')
            if int(elapsed)//5!=last_print:
                last_print=int(elapsed)//5
                print('flight',round(elapsed,2),'pose',truth[-1][1].tolist(),'state',state[0],flush=True)
            if truth[-1][1][2]<3: raise RuntimeError('flight reached ground safety boundary')
            if process.poll() is not None or time.monotonic()>wall_end: raise RuntimeError('flight stopped/timed out')
        report['passed']=True
    except Exception as error:
        report['error']=str(error)
        print('FAILED',error,flush=True)
    finally:
        match()
        if records:
            velocity=np.array([a['velocity_error'] for a in records]); heading=np.array([a['yaw_error_deg'] for a in records])
            cross=np.array([a['cross_track'] for a in records]); bank=np.array([abs(a['truth_roll'])>.12 for a in records])
            report['metrics']=dict(samples=len(records),velocity_rmse_xy=np.sqrt(np.mean(velocity[:,:2]**2,axis=0)).tolist(),
                                   yaw_rmse_deg=float(np.sqrt(np.mean(heading**2))),
                                   yaw_rmse_banked_deg=float(np.sqrt(np.mean(heading[bank]**2))) if any(bank) else None,
                                   final_drift_m=float(np.linalg.norm(records[-1]['drift_xy'])),path_m=path[0],
                                   drift_percent=100*float(np.linalg.norm(records[-1]['drift_xy']))/max(path[0],1),
                                   cross_track_rmse_m=float(np.sqrt(np.mean(cross**2))),max_cross_track_m=float(max(abs(cross))),
                                   achieved_waypoint_targets=list(dict.fromkeys(tuple(a[1]) for a in targets)),
                                   min_height=min(a[4] for a in truth_rpy),max_height=max(a[4] for a in truth_rpy),
                                   max_thrust=max((a[1] for a in thrust),default=None))
            report['metrics']['actual_motor_max']=max(motors) if motors else None
            report['metrics']['gazebo_motor_command_max_rad_s']=max(motor_commands) if motor_commands else None
            report['metrics']['height_rmse_40m']=float(np.sqrt(np.mean([(a['height']-40.25)**2 for a in records])))
            if args.regime=='unpowered_glide': report['metrics']['height_rmse_40m']=None
            transition=np.array([a['waypoint_transition'] for a in records])
            for label,mask in [('straight',~transition),('waypoint_change',transition)]:
                report['metrics']['cross_track_rmse_'+label+'_m']=float(np.sqrt(np.mean(cross[mask]**2))) if any(mask) else None
            report['metrics']['velocity_rmse_banked_xy']=np.sqrt(np.mean(velocity[bank,:2]**2,axis=0)).tolist() if any(bank) else None
            report['metrics']['longest_navigation_gap_s']=float(max(np.diff([a['t'] for a in records]))) if len(records)>1 else None
            report['metrics']['roll_rmse_deg']=float(np.sqrt(np.mean([a['roll_error_deg']**2 for a in records])))
            report['metrics']['pitch_rmse_deg']=float(np.sqrt(np.mean([a['pitch_error_deg']**2 for a in records])))
            report['metrics']['setpoint_received_rate_hz']=(len(thrust)-1)/max(thrust[-1][0]-thrust[0][0],.001) if len(thrust)>1 else 0
            steady_speeds=[a[5] for a in truth_rpy if a[0]>launch_time[0]+5]
            if args.regime=='unpowered_glide' and steady_speeds:
                report['metrics']['glide_target_speed_m_s']=args.glide_speed
                report['metrics']['glide_speed_rmse_after_5s_m_s']=float(np.sqrt(np.mean((np.array(steady_speeds)-args.glide_speed)**2)))
                if report['metrics']['glide_speed_rmse_after_5s_m_s']>1.5:
                    report['passed']=False; report['error']='glide speed RMSE exceeds 1.5 m/s'
            if args.polarity:
                def mean_angle(start,end,index):
                    values=[a[index] for a in truth_rpy if start<a[0]-launch_time[0]<end]
                    return float(np.mean(values)) if values else float('nan')
                roll_step=mean_angle(3,4,1)-mean_angle(1,2,1)
                pitch_up_step=-(mean_angle(7,8,2)-mean_angle(5,6,2))
                report['metrics']['roll_step_response_rad']=roll_step
                report['metrics']['pitch_up_step_response_rad']=pitch_up_step
                report['metrics']['polarity_passed']=roll_step>.05 and pitch_up_step>.025
                report['metrics']['cross_track_rmse_m']=None
                report['metrics']['max_cross_track_m']=None
                report['metrics']['cross_track_rmse_straight_m']=None
                report['metrics']['cross_track_rmse_waypoint_change_m']=None
                if not report['metrics']['polarity_passed']: report['passed']=False; report['error']='control polarity check failed'
            elif args.regime=='cruise_const_alt' and not args.drop_visual_at and len(report['metrics']['achieved_waypoint_targets'])<3:
                report['passed']=False; report['error']='fewer than three route segments commanded'
            if args.regime=='cruise_const_alt' and not args.polarity and not args.drop_visual_at and report['metrics']['height_rmse_40m']>3:
                report['passed']=False; report['error']='cruise height RMSE exceeds 3 m'
            degraded=any(a['state']=='DEGRADED_WINGS_LEVEL' for a in report['states'])
            if degraded and not args.drop_visual_at:
                report['passed']=False; report['error']='visual fallback activated during flight'
            if args.drop_visual_at:
                report['fault_injection_passed']=fault_injected and degraded
                report['metrics']['fallback_final_roll_deg']=math.degrees(truth_rpy[-1][1])
                report['metrics']['final_px4_nav_state']=int(status[0].nav_state) if status[0] else None
                report['metrics']['fallback_final_thrust']=thrust[-1][1] if thrust else None
                if abs(truth_rpy[-1][1])>.12 or status[0].nav_state!=14: report['fault_injection_passed']=False
                if not report['fault_injection_passed']: report['passed']=False; report['error']='fallback fault test failed'
            if args.regime=='unpowered_glide' and report['metrics']['max_thrust']!=0: report['passed']=False; report['error']='nonzero glide thrust'
            if args.regime=='unpowered_glide' and (not motors or max(motors)>1e-6):
                report['passed']=False; report['error']='actual motor output not verified zero'
            if args.regime=='unpowered_glide' and (not motor_commands or max(motor_commands)>1e-6):
                report['passed']=False; report['error']='Gazebo motor command not verified zero'
        report['launch_time']=launch_time[0]; report['end_time']=clock[0]
        output.write_text(json.dumps(clean(report),indent=2,allow_nan=False)+'\n')
        raw.close(); node.destroy_node(); rclpy.shutdown()
        if stimulus and stimulus.poll() is None:
            stimulus.send_signal(signal.SIGINT)
            stimulus.wait(timeout=5)
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM); process.wait(timeout=6)
        log.close()
        ulogs=list((directory/'log').rglob('*.ulg')) if (directory/'log').exists() else []
        if ulogs:
            ulog=max(ulogs,key=lambda item:item.stat().st_mtime)
            destination=output.with_suffix('.ulg'); shutil.copy2(ulog,destination)
            report['px4_ulog']=str(destination)
            output.write_text(json.dumps(clean(report),indent=2,allow_nan=False)+'\n')
    if not report['passed']: raise SystemExit(1)


if __name__=='__main__': main()
