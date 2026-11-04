"""Actual OS-process DDS probe; this parent deliberately never imports rclpy."""
import argparse
import json
import os
import subprocess
import tempfile
import time
from pathlib import Path


def probe(binary, profile, domain):
    env = os.environ.copy()
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'ROS_LOCALHOST_ONLY', 'ROS_DISCOVERY_SERVER', 'ROS_STATIC_PEERS'):
        env.pop(key, None)
    env.update(ROS_DOMAIN_ID=str(domain), RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
               ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT', RMW_FASTRTPS_USE_QOS_FROM_XML='1')
    if profile:
        env['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
        env['FASTDDS_DEFAULT_PROFILES_FILE'] = str(profile)
    sub = subprocess.Popen([binary, 'sub'], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    pub = None
    try:
        time.sleep(0.7)
        pub = subprocess.Popen([binary, 'pub'], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        pub_out = pub.communicate(timeout=25)[0]
        sub_out = sub.communicate(timeout=26)[0]
        def result(output):
            return next(json.loads(line) for line in output.splitlines() if line.startswith('{'))
        return {'publisher': result(pub_out), 'subscriber': result(sub_out),
                'passed': pub.returncode == 0 and sub.returncode == 0,
                'logs': pub_out + sub_out}
    finally:
        for process in (pub, sub):
            if process and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', required=True)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--diagnose', action='store_true')
    parser.add_argument('--output')
    parser.add_argument('--repeat', type=int, default=1)
    args = parser.parse_args()
    results = {}
    profiles = [('loopback_unicast', args.profile)]
    with tempfile.TemporaryDirectory(prefix='glider_dds_matrix_') as directory:
        if args.diagnose:
            template = '''<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
<transport_descriptors><transport_descriptor><transport_id>udp</transport_id><type>UDPv4</type>{whitelist}</transport_descriptor></transport_descriptors>
<participant profile_name="probe" is_default_profile="true"><rtps><useBuiltinTransports>false</useBuiltinTransports><userTransports><transport_id>udp</transport_id></userTransports></rtps></participant></profiles>'''
            profiles = [('default', None)]
            for name, whitelist in [('udp_all_interfaces', ''), ('udp_loopback_multicast', '<interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>')]:
                path = Path(directory) / (name + '.xml')
                path.write_text(template.format(whitelist=whitelist))
                profiles.append((name, path))
            profiles.append(('loopback_unicast', args.profile))
            path = Path(directory) / 'loopback_large.xml'
            path.write_text(Path(args.profile).read_text().replace('<maxMessageSize>1200</maxMessageSize>', '<maxMessageSize>65000</maxMessageSize>'))
            profiles.insert(-1, ('loopback_large_datagrams', path))
        elif args.repeat > 1:
            profiles = [('loopback_unicast_' + str(i), args.profile) for i in range(args.repeat)]
        for index, (name, profile) in enumerate(profiles):
            results[name] = probe(args.binary, profile, (20 if args.diagnose else 80) + index)
            print(name, json.dumps(results[name]), flush=True)
    if args.output:
        Path(args.output).write_text(json.dumps(results, indent=2) + '\n')
    selected = [value for name, value in results.items() if name.startswith('loopback_unicast')]
    assert selected and all(value['passed'] for value in selected), 'separate-process DDS delivery failed'
