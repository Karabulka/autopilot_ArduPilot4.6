"""Generate a local px4_msgs dependency from this exact PX4 checkout, never a moving branch.
Build with CMake in build_visual_nav/deps; colcon discovery remains package-isolated.
"""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

root = Path(__file__).resolve().parents[3]
source = root / 'build_visual_nav/deps/px4_msgs'
(source / 'msg').mkdir(parents=True, exist_ok=True)
names = ['VehicleAttitudeSetpoint', 'OffboardControlMode', 'VehicleCommand', 'VehicleCommandAck',
         'VehicleStatus', 'VehicleAttitude', 'VehicleAngularVelocity', 'VehicleAirData',
         'AirspeedValidated', 'VehicleControlMode', 'ActuatorMotors', 'ActuatorServos']
manifest = {}
for name in names:
    path = root / 'msg/versioned' / (name + '.msg')
    if not path.exists():
        path = root / 'msg' / (name + '.msg')
    shutil.copyfile(path, source / 'msg' / path.name)
    manifest[name] = {'source': str(path.relative_to(root)), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
(source / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(px4_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
''' + ''.join(f'  "msg/{name}.msg"\n' for name in names) + ''')
ament_export_dependencies(rosidl_default_runtime)
install(FILES message_manifest.json DESTINATION share/${PROJECT_NAME})
ament_package()
''')
(source / 'package.xml').write_text('''<package format="3"><name>px4_msgs</name><version>0.0.1</version>
<description>Exact-checkout PX4 messages for the isolated visual-navigation test</description>
<maintainer email="nikolyan14052005@gmail.com">nikol</maintainer><license>BSD-3-Clause</license>
<buildtool_depend>ament_cmake</buildtool_depend><build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend><member_of_group>rosidl_interface_packages</member_of_group>
<export><build_type>ament_cmake</build_type></export></package>''')
(source / 'message_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
subprocess.run(['cmake', '-S', str(source), '-B', str(root / 'build_visual_nav/deps/messages_build'),
                '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=OFF', '-DPython3_EXECUTABLE=/usr/bin/python3',
                '-DCMAKE_INSTALL_PREFIX=' + str(root / 'install_visual_nav/px4_msgs')], check=True)
subprocess.run(['cmake', '--build', str(root / 'build_visual_nav/deps/messages_build'), '-j4'], check=True)
subprocess.run(['cmake', '--install', str(root / 'build_visual_nav/deps/messages_build')], check=True)
prefix = root / 'install_visual_nav/px4_msgs'
wrapper = f'export AMENT_CURRENT_PREFIX="{prefix}"\nsource "{prefix}/share/px4_msgs/local_setup.bash"\nexport COLCON_PREFIX_PATH="{prefix}:${{COLCON_PREFIX_PATH:-}}"\n'
(prefix / 'local_setup.bash').write_text(wrapper)
(prefix / 'setup.bash').write_text(wrapper)
