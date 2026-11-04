#!/usr/bin/env python3
"""Isolated Gazebo sensor and passive-glide smoke test; never feeds truth to control."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, MagneticField
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=75)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--arduplane', action='store_true')
    parser.add_argument('--guided', action='store_true', help='SITL only: request GUIDED and normal arming')
    args = parser.parse_args()
    if args.guided and not args.arduplane:
        parser.error('--guided requires --arduplane')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID='67', GZ_PARTITION='glider_arduplane', ARDUPLANE_ENABLE_CONTROL='true' if args.guided else 'false')
    rclpy.init()
    node = rclpy.create_node('glider_smoke_evaluator')
    processes = []
    streams = []
    raw = args.output.with_suffix('.jsonl').open('w')
    result = {'passed': False, 'test': 'passive glide, no guided/armed control',
              'guided_requested': args.guided, 'active_samples': 0, 'frames': 0, 'imu': 0, 'magnetometer': 0, 'barometer': 0, 'odometry': 0}
    gcs = None
    truth = None
    start_stamp = None
    airborne_until = None
    first_airborne = None
    airborne_samples = 0
    airspeed = 0.
    minimum_height = math.inf

    def start(name, command):
        stream = args.output.with_suffix('.'+name+'.log').open('w')
        streams.append(stream)
        processes.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))

    def on_truth(msg):
        nonlocal truth, airborne_until, airspeed, minimum_height, first_airborne, airborne_samples
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        truth = stamp, p.x, p.y, p.z
        speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
        raw.write(json.dumps(dict(t=stamp, xyz=[p.x,p.y,p.z], speed=speed))+'\n')
        if start_stamp is not None:
            minimum_height = min(minimum_height, p.z)
            airspeed = max(airspeed, speed)
            if p.z > 2. and speed > 5.:
                airborne_until = stamp
                if first_airborne is None:
                    first_airborne = stamp
                airborne_samples += 1

    def on_camera(msg):
        if msg.width == 640 and msg.height == 480 and len(msg.data) >= 640*480:
            result['frames'] += 1

    def count(key):
        return lambda _: result.__setitem__(key, result[key]+1)

    def on_state(msg):
        if msg.data == 'ACTIVE':
            result['active_samples'] += 1

    subscriptions = [
        node.create_subscription(String, '/glider/interface_state', on_state, qos_profile_sensor_data),
        node.create_subscription(Odometry, '/evaluation/ground_truth', on_truth, qos_profile_sensor_data),
        node.create_subscription(Image, '/camera/image_raw', on_camera, qos_profile_sensor_data),
        node.create_subscription(Imu, '/glider/imu', count('imu'), qos_profile_sensor_data),
        node.create_subscription(MagneticField, '/glider/magnetic_field', count('magnetometer'), qos_profile_sensor_data),
        node.create_subscription(PointStamped, '/glider/barometric_height', count('barometer'), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/glider/odometry', count('odometry'), qos_profile_sensor_data),
    ]
    script = str(ROOT/'src/glider_visual_nav/tools/arduplane_sim.sh')
    try:
        if args.guided:
            os.environ['MAVLINK20'] = '1'
            from pymavlink import mavutil
            gcs = mavutil.mavlink_connection('udpin:127.0.0.1:14551', source_system=255)
        start('sim', ['bash', script, 'run' if args.arduplane else 'passive'])
        if not args.arduplane:
            start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                             '/evaluation/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                             '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'])
        end = time.monotonic() + 60
        while truth is None or result['frames'] < 5 or (args.arduplane and not result['odometry']):
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() > end or any(p.poll() is not None for p in processes):
                raise RuntimeError('fixture/camera/navigation did not become ready')
        if args.guided:
            mode_confirmed = False
            armed = False
            last_send = -math.inf
            deadline = time.monotonic()+100
            while not armed and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.03)
                packet = gcs.recv_match(blocking=False)
                if not packet:
                    continue
                if packet.get_type() == 'STATUSTEXT':
                    print(packet.text, flush=True)
                if packet.get_type() == 'COMMAND_ACK':
                    print(f'ACK command={packet.command} result={packet.result}', flush=True)
                if packet.get_type() != 'HEARTBEAT' or packet.get_srcSystem() != 1:
                    continue
                mode_confirmed = packet.custom_mode == 15
                armed = mode_confirmed and bool(packet.base_mode & 128)
                now = time.monotonic()
                if armed or now-last_send < 2:
                    continue
                gcs.mav.heartbeat_send(6, 8, 0, 0, 4)
                if not mode_confirmed:
                    gcs.mav.command_long_send(1, 1, 176, 0, 1, 15, 0, 0, 0, 0, 0)
                else:
                    # Fixed test reference, not a measured GNSS fix.
                    gcs.mav.command_int_send(1, 1, 0, 179, 0, 0, 0, 0, 0, 0,
                                             473977420, 85455940, 0.)
                    gcs.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
                last_send = now
            result.update(guided_confirmed=mode_confirmed, armed=armed)
            if not armed:
                raise RuntimeError('SITL did not confirm GUIDED and normal arming')
            result['test'] = 'SITL GUIDED flight with normal arming'
        start_stamp = truth[0]
        release = subprocess.run(['bash', script, 'fly'], capture_output=True, text=True, timeout=12)
        if release.returncode or 'data: true' not in release.stdout:
            raise RuntimeError('fixture did not release: '+release.stdout+release.stderr)
        end = time.monotonic()+args.duration*6+20
        while truth[0]-start_stamp < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() > end or any(p.poll() is not None for p in processes):
                raise RuntimeError('simulator stopped or timed out')
        result.update(sim_seconds=truth[0]-start_stamp,
                      airborne_seconds=max(0., (airborne_until or start_stamp)-start_stamp),
                      final_position=truth[1:], minimum_height=minimum_height, peak_speed=airspeed)
        result['first_airborne_sim_time'] = first_airborne
        result['airborne_samples'] = airborne_samples
        result['passed'] = airspeed > 8. and result['airborne_seconds'] >= 5. and result['frames'] > 30 and (not args.guided or
            (result['active_samples'] > 10 and minimum_height > 2. and
             airborne_until is not None and truth[0]-airborne_until < .5 and
             first_airborne is not None and airborne_until-first_airborne >= args.duration-2.))
    except Exception as error:
        result['error'] = str(error)
    finally:
        if gcs is not None:
            gcs.close()
        raw.close()
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
        for proc in processes:
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        for stream in streams:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(json.dumps(result), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
