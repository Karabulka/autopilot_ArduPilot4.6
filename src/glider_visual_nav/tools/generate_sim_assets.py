"""Reproducible local bench assets; no Fuel/network dependencies."""
from pathlib import Path
import math
import re
import xml.etree.ElementTree as ET
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
GZ = ROOT / 'Tools/simulation/gz'


def generate():
    ground = GZ / 'models/visual_nav_ground'
    texture = ground / 'materials/textures/ground.png'
    texture.parent.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(20261020)
    tiles = rng.choice(np.array([24, 88, 168, 232], dtype=np.uint8), (512, 512))
    cv2.imwrite(str(texture), np.repeat(np.repeat(tiles, 8, axis=0), 8, axis=1))
    def config(name):
        return f'<model><name>{name}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><author><name>glider_visual_nav</name></author><description>Deterministic visual navigation bench</description></model>\n'
    (ground / 'model.config').write_text(config('visual_nav_ground'))
    (ground / 'model.sdf').write_text('''<sdf version="1.9"><model name="visual_nav_ground"><static>true</static><link name="ground">
<collision name="ground"><geometry><plane><normal>0 0 1</normal><size>512 512</size></plane></geometry></collision>
<visual name="ground"><cast_shadows>false</cast_shadows><geometry><plane><normal>0 0 1</normal><size>512 512</size></plane></geometry>
<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular><pbr><metal><albedo_map>model://visual_nav_ground/materials/textures/ground.png</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr></material>
</visual></link></model></sdf>\n''')
    target = GZ / 'models/rc_cessna_visual'
    target.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(GZ / 'models/rc_cessna/model.sdf')
    model = tree.getroot().find('model')
    model.set('name', 'rc_cessna_visual')
    if model.find('pose') is not None:
        model.find('pose').text = '0 0 0 0 0 0'
    body = model.find("link[@name='base_link']")
    for sensor in list(body.findall('sensor')):
        if sensor.get('type') == 'navsat':
            body.remove(sensor)
    for noise in model.findall(".//sensor[@type='magnetometer']//noise"):
        noise.clear()
        noise.set('type', 'none')
    camera = ET.fromstring('''<sensor name="nadir_camera" type="camera">
<gz_frame_id>camera_optical_frame</gz_frame_id><pose>0 0 -0.25 0 1.5707963267948966 0</pose>
<topic>/camera/image_raw</topic><always_on>true</always_on><update_rate>30</update_rate>
<camera><horizontal_fov>1.74</horizontal_fov><image><width>640</width><height>480</height><format>L8</format></image>
<camera_info_topic>/camera/camera_info</camera_info_topic><clip><near>0.05</near><far>1000</far></clip></camera></sensor>''')
    body.append(camera)
    for sensor in model.findall(".//sensor[@type='magnetometer']"):
        sensor.find('update_rate').text = '100'
        frame = sensor.find('gz_frame_id')
        if frame is None:
            frame = ET.SubElement(sensor, 'gz_frame_id')
        frame.text = 'base_link'
    ET.indent(tree, space='  ')
    tree.write(target / 'model.sdf', encoding='unicode', xml_declaration=True)
    (target / 'model.config').write_text(config('rc_cessna_visual'))
    # Existing models were tuned to the legacy Gauss output. Correct only mag noise.
    for path in (GZ / 'models').glob('*/model.sdf'):
        bom = path.read_bytes().startswith(b'\xef\xbb\xbf')
        text = path.read_text(encoding='utf-8-sig')
        def correct(match):
            return match[0].replace('<stddev>0.0001</stddev>', '<stddev>3e-7</stddev>')
        updated = re.sub(r'<sensor\b[^>]*type="magnetometer"[^>]*>.*?</sensor>', correct, text, flags=re.S)
        if updated != text:
            path.write_text(updated, encoding='utf-8-sig' if bom else 'utf-8')
    server = GZ / 'server.config'
    text = server.read_text()
    text = text.replace('name="gz::sim::systems::Magnetometer"/>', 'name="gz::sim::systems::Magnetometer"><use_units_gauss>false</use_units_gauss><use_earth_frame_ned>false</use_earth_frame_ned></plugin>')
    text = re.sub(r'(<use_units_gauss>)true(</use_units_gauss>)', r'\g<1>false\2', text)
    text = re.sub(r'(<use_earth_frame_ned>)true(</use_earth_frame_ned>)', r'\g<1>false\2', text)
    server.write_text(text)
    east, north = 25e-6 * math.sin(0.2), 25e-6 * math.cos(0.2)
    (GZ / 'worlds/visual_nav_flat.sdf').write_text(f'''<sdf version="1.9"><world name="visual_nav_flat">
<physics name="physics" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1</real_time_factor></physics>
<gravity>0 0 -9.80665</gravity><wind><linear_velocity>0 0 0</linear_velocity></wind>
<magnetic_field>{east:.12g} {north:.12g} -0.000042</magnetic_field>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
<plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
<plugin filename="libFixedMagneticFieldSystem.so" name="glider_visual_nav::FixedMagneticFieldSystem"/>
<scene><ambient>0.8 0.8 0.8 1</ambient><background>0.6 0.7 0.8 1</background><shadows>false</shadows></scene>
<light name="sun" type="directional"><pose>0 0 100 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><specular>0 0 0 1</specular><direction>0 0 -1</direction></light>
<include><uri>model://visual_nav_ground</uri></include>
<!-- Static overrides the flight model only in this sensor qualification world. -->
<include><uri>model://rc_cessna_visual</uri><name>rc_cessna_visual</name><pose>0 0 40.25 0 0 0</pose><static>true</static></include>
</world></sdf>\n''')


if __name__ == '__main__':
    generate()
    flight = ET.parse(GZ / 'worlds/visual_nav_flat.sdf')
    world = flight.getroot().find('world')
    world.set('name', 'visual_nav_flight')
    world.find("include[name='rc_cessna_visual']/static").text = 'false'
    world.append(ET.fromstring('<model name="launch_anchor"><static>true</static><link name="anchor"/></model>'))
    world.append(ET.fromstring('<plugin filename="libFlightFixtureSystem.so" name="glider_visual_nav::FlightFixtureSystem"/>'))
    ET.indent(flight)
    flight.write(GZ / 'worlds/visual_nav_flight.sdf', encoding='unicode')
