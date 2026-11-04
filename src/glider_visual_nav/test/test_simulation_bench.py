"""Opt-in real Gazebo/bridge/multi-process qualification. No truth enters navigation.

Run after sourcing ROS and the isolated install. The evaluator alone issues pose
commands; yaw and velocity under test come from navigation's ROS outputs.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import re
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo-root', default='/home/nikol/autopilot')
    parser.add_argument('--output', required=True)
    parser.add_argument('--px4', action='store_true', help='also qualify rebuilt PX4 sensor_mag uORB output')
    args = parser.parse_args()
    root = Path(args.repo_root)
    os.environ.update(FASTRTPS_DEFAULT_PROFILES_FILE=str(root / 'src/glider_visual_nav/config/fastdds_wsl.xml'),
                      FASTDDS_DEFAULT_PROFILES_FILE=str(root / 'src/glider_visual_nav/config/fastdds_wsl.xml'),
                      ROS_DOMAIN_ID='65', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                      RMW_FASTRTPS_USE_QOS_FROM_XML='1',
                      ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT', GZ_IP='127.0.0.1',
                      GZ_PARTITION='glider_qualification_' + str(os.getpid()))
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo, MagneticField
    from geometry_msgs.msg import TwistWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    report = {'stages': [], 'passed': False}
    log_path = Path(args.output).with_suffix('.log')
    log = log_path.open('w')
    process = subprocess.Popen(['ros2', 'launch', 'glider_visual_nav', 'simulation.launch.py',
                                'repo_root:=' + str(root)], stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('sensor_bench_evaluator')
    counts = dict(image=0, info=0, mag=0, velocity=0, odom=0, clock=0)
    latest = {}
    samples = []
    timestamps = {'image': [], 'mag': []}
    px4 = None
    px4_log = None
    def receive(key, message):
        counts[key] += 1
        latest[key] = message
        if key in timestamps:
            timestamps[key].append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
        if key == 'image':
            image = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)[:, :message.width].copy()
            corners = cv2.goodFeaturesToTrack(image, 500, 0.01, 7.0)
            latest['corners'] = 0 if corners is None else len(corners)
            latest['gray'] = image
            samples.append(latest['corners'])
    subscriptions = [node.create_subscription(kind, topic, lambda msg, key=key: receive(key, msg), qos)
                     for key, kind, topic, qos in [
                         ('image', Image, '/camera/image_raw', qos_profile_sensor_data),
                         ('info', CameraInfo, '/camera/camera_info', qos_profile_sensor_data),
                         ('mag', MagneticField, '/glider/magnetic_field', qos_profile_sensor_data),
                         ('velocity', TwistWithCovarianceStamped, '/glider/visual_velocity', 10),
                         ('odom', Odometry, '/glider/odometry', 10),
                         ('clock', Clock, '/clock', qos_profile_sensor_data)]]
    def wait(predicate, seconds=50):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if process.poll() is not None:
                raise RuntimeError('launch exited; see ' + str(log_path))
            if predicate():
                return
        raise RuntimeError('timed out: counts=' + repr(counts))
    def pose(height, yaw):
        request = (f'name: "rc_cessna_visual" position {{ x: 0 y: 0 z: {height + .25} }} '
                   f'orientation {{ x: 0 y: 0 z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)} }}')
        result = subprocess.run(['gz', 'service', '-s', '/world/visual_nav_flat/set_pose',
                                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                                 '--timeout', '5000', '--req', request], capture_output=True, text=True, timeout=8)
        if result.returncode or 'true' not in result.stdout:
            raise RuntimeError('pose service failed: ' + result.stdout + result.stderr)
    try:
        wait(lambda: all(counts[key] >= 3 for key in counts), 90)
        if args.px4:
            directory = root / 'build_visual_nav/px4_bench'
            directory.mkdir(parents=True, exist_ok=True)
            binary_dir = root / 'build/px4_sitl_default/bin'
            startup = directory / 'rc.sensor_bench'
            startup.write_text(f'#!/bin/sh\n{binary_dir}/px4-gz_bridge --instance 7 start -w visual_nav_flat -n rc_cessna_visual\n')
            px4_log = Path(args.output).with_suffix('.px4.log').open('w')
            px4 = subprocess.Popen([str(binary_dir / 'px4'), '-d', '-i', '7', '-w', str(directory),
                                    '-s', str(startup), str(root / 'build/px4_sitl_default')],
                                   stdout=px4_log, stderr=subprocess.STDOUT, start_new_session=True)
            settle = time.monotonic() + 3
            wait(lambda: time.monotonic() >= settle)
        report['camera'] = {'width': latest['info'].width, 'height': latest['info'].height,
                            'K': list(latest['info'].k), 'frame': latest['info'].header.frame_id}
        assert latest['image'].header.frame_id == 'camera_optical_frame'
        assert latest['mag'].header.frame_id == 'base_link'
        for height, degrees in [(40, 0), (40, 90), (40, 180), (40, 270), (30, 0), (60, 0)]:
            yaw = math.radians(degrees)
            pose(height, yaw)
            settle = time.monotonic() + 3
            wait(lambda: time.monotonic() >= settle)
            samples.clear()
            initial_image, initial_odom = counts['image'], counts['odom']
            wait(lambda: counts['image'] >= initial_image + 15 and counts['odom'] >= initial_odom + 10)
            q = latest['odom'].pose.pose.orientation
            measured = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            error = math.atan2(math.sin(measured-yaw), math.cos(measured-yaw))
            v = latest['velocity'].twist.twist.linear
            speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            b = latest['mag'].magnetic_field
            expected = [math.cos(yaw)*25e-6*math.sin(.2) + math.sin(yaw)*25e-6*math.cos(.2),
                        -math.sin(yaw)*25e-6*math.sin(.2) + math.cos(yaw)*25e-6*math.cos(.2), -42e-6]
            field_error = max(abs(a-c) for a, c in zip([b.x,b.y,b.z], expected))
            stage = dict(height=height, yaw_degrees=degrees, measured_yaw_degrees=math.degrees(measured),
                         yaw_error_degrees=math.degrees(error), speed=speed,
                         min_corners=min(samples), max_corners=max(samples),
                         magnetic_field=[b.x,b.y,b.z], field_error_tesla=field_error)
            if args.px4:
                output = subprocess.run([str(binary_dir / 'px4-listener'), '--instance', '7', 'sensor_mag', '-n', '1'],
                                        capture_output=True, text=True, timeout=8)
                fields = re.findall(r'^\s*([xyz]):\s*([-+0-9.eE]+)', output.stdout, flags=re.M)
                values = dict((key, float(value)) for key, value in fields)
                if len(values) != 3:
                    raise RuntimeError('PX4 sensor_mag missing: ' + output.stdout + output.stderr)
                actual = [values[key] for key in 'xyz']
                expected_px4 = [1e4*expected[0], -1e4*expected[1], -1e4*expected[2]]
                stage['px4_sensor_mag_gauss_frd'] = actual
                stage['px4_error_gauss'] = max(abs(a-b) for a,b in zip(actual, expected_px4))
                assert stage['px4_error_gauss'] < 1e-5, stage
            report['stages'].append(stage)
            cv2.imwrite(str(Path(args.output).with_name(f'camera_h{height}_yaw{degrees}.png')), latest['gray'])
            print(json.dumps(stage), flush=True)
            assert min(samples) >= 100, stage
            assert speed < .05 and abs(error) < math.radians(.5) and field_error < 1e-9, stage
        report['passed'] = True
        report['received_rate_sim_hz'] = {
            key: float((len(values)-1) / (values[-1]-values[0])) for key, values in timestamps.items()}
        report['median_stamp_interval_seconds'] = {
            key: float(np.median(np.diff(values))) for key, values in timestamps.items()}
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        report['counts'] = counts
        Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
        if px4:
            px4.send_signal(signal.SIGINT)
            try:
                px4.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(px4.pid, signal.SIGTERM)
                px4.wait(timeout=5)
            px4_log.close()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    main()
