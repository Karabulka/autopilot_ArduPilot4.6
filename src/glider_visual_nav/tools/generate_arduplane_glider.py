#!/usr/bin/env python3
"""Generate a Harmonic model from the classic glider's mass and airfoil polar."""
from pathlib import Path
import copy
import xml.etree.ElementTree as E

root = Path(__file__).resolve().parents[3]
pkg = root/'src/glider_visual_nav'
classic = E.parse(root/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/glider/glider.sdf').find('model')
sdf = E.Element('sdf', version='1.9')
model = E.SubElement(sdf, 'model', name='glider_visual')


def node(parent, tag, value=None, **attributes):
    child = E.SubElement(parent, tag, attributes)
    if value is not None:
        child.text = str(value)
    return child


def box(parent, name, pose, size, visual=True):
    element = node(parent, 'visual' if visual else 'collision', name=name)
    node(element, 'pose', pose)
    node(node(node(element, 'geometry'), 'box'), 'size', size)
    if visual:
        node(node(element, 'material'), 'diffuse', '0.83 0.88 0.96 1')


base = node(model, 'link', name='base_link')
base.append(copy.deepcopy(classic.find('link/inertial')))
box(base, 'fuselage', '0 0 0 0 0 0', '1.55 0.11 0.12')
box(base, 'contact', '0 0 0 0 0 0', '1.55 0.11 0.12', False)
box(base, 'wing', '0.04 0 0.05 0 0 0', '0.20 3.20 0.035')
box(base, 'tail', '-0.65 0 0.04 0 0 0', '0.16 0.56 0.024')
box(base, 'fin', '-0.63 0 0.14 0 0 0', '0.20 0.02 0.20')
surfaces = [('aileron_left', '-0.07 0.99 0.05', '0.075 0.9 0.02', '0 1 0'),
            ('aileron_right', '-0.07 -0.99 0.05', '0.075 0.9 0.02', '0 1 0'),
            ('elevator', '-0.74 0 0.04', '0.065 0.48 0.018', '0 1 0'),
            ('rudder', '-0.75 0 0.16', '0.065 0.02 0.15', '0 0 1')]
for name, position, size, axis in surfaces:
    link = node(model, 'link', name=name)
    node(link, 'pose', position+' 0 0 0')
    inertia = node(link, 'inertial')
    node(inertia, 'mass', .025)
    tensor = node(inertia, 'inertia')
    for key in ('ixx', 'iyy', 'izz'):
        node(tensor, key, .0001)
    box(link, name, '-0.025 0 0 0 0 0', size)
    joint = node(model, 'joint', name=name+'_joint', type='revolute')
    node(joint, 'parent', 'base_link')
    node(joint, 'child', name)
    a = node(joint, 'axis')
    node(a, 'xyz', axis)
    limit = node(a, 'limit')
    node(limit, 'lower', -.45)
    node(limit, 'upper', .45)
    node(node(a, 'dynamics'), 'damping', .12)

for source_name, joint, cp, area, gain in [
    ('left_wing', 'aileron_left', '-0.01 .82 .05', .32, -.65),
    ('right_wing', 'aileron_right', '-0.01 -.82 .05', .32, -.65),
    ('elevator', 'elevator', '-.69 0 .04', .09, -.80),
    ('rudder', 'rudder', '-.69 0 .15', .045, -.70),
]:
    source = classic.find(f"plugin[@name='{source_name}']")
    aero = node(model, 'plugin', filename='gz-sim-lift-drag-system',
                name='gz::sim::systems::LiftDrag')
    for key in ('cla', 'cda', 'cma', 'alpha_stall', 'cla_stall', 'cda_stall', 'cma_stall',
                'air_density', 'forward', 'upward', 'link_name'):
        aero.append(copy.deepcopy(source.find(key)))
    for key, value in dict(a0=.025 if 'wing' in source_name else 0, cp=cp, area=area,
                           control_joint_name=joint+'_joint', control_joint_rad_to_cl=gain).items():
        node(aero, key, value)

cessna = E.parse(root/'Tools/simulation/gz/models/rc_cessna_visual/model.sdf')
for sensor in cessna.findall('.//sensor'):
    if sensor.get('type') in ('camera', 'magnetometer', 'imu', 'air_pressure'):
        base.append(copy.deepcopy(sensor))
imu_name = base.find("sensor[@type='imu']").get('name')

folder = pkg/'simulation/models/glider_visual'
folder.mkdir(parents=True, exist_ok=True)


def write(tree, path):
    E.indent(tree, space='  ')
    E.ElementTree(tree).write(path, encoding='utf-8', xml_declaration=True)


write(sdf, folder/'model.sdf')
plugin = node(model, 'plugin', name='ArduPilotPlugin', filename='ArduPilotPlugin')
for key, value in dict(fdm_addr='127.0.0.1', fdm_port_in=9002, lock_step=0,
                       imuName='base_link::'+imu_name).items():
    node(plugin, key, value)
node(plugin, 'modelXYZToAirplaneXForwardZDown', '0 0 0 180 0 0', degrees='true')
node(plugin, 'gazeboXYZToNED', '0 0 0 180 0 90', degrees='true')
for channel, (name, *_rest) in zip((0, 4, 1, 3), surfaces):
    control = node(plugin, 'control', channel=str(channel))
    for key, value in dict(jointName=name+'_joint', type='POSITION', useForce=1,
                           multiplier=-.9 if name == 'aileron_right' else .9,
                           offset=-.5, servo_min=1000, servo_max=2000,
                           p_gain=2, d_gain=.02, cmd_max=1, cmd_min=-1).items():
        node(control, key, value)
node(base.find("sensor[@type='imu']"), 'pose', '0 0 0 180 0 0', degrees='true')
write(sdf, folder/'arduplane.sdf')
(folder/'model.config').write_text('<model><name>glider_visual</name><version>0.1</version>'
                                  '<sdf version="1.9">model.sdf</sdf></model>\n')
world_tree = E.parse(root/'Tools/simulation/gz/worlds/visual_nav_flight.sdf')
world = world_tree.find('world')
world.set('name', 'glider_visual_flight')
include = next(x for x in world.findall('include') if x.findtext('name') == 'rc_cessna_visual')
include.find('uri').text = 'model://glider_visual'
include.find('name').text = 'glider_visual'
node(world.find("plugin[@filename='libFlightFixtureSystem.so']"), 'model_name', 'glider_visual')
write(world_tree.getroot(), pkg/'simulation/worlds/glider_visual.sdf')
include.find("uri").text = 'model://glider_visual/arduplane.sdf'
include.find("pose").text = "0 0 145.25 0 0 0"
write(world_tree.getroot(), pkg/'simulation/worlds/arduplane_glider_visual.sdf')
