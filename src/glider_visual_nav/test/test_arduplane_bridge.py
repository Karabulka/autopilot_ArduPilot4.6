"""Coordinate and MAVLink v2 wire contract tests."""
import io
import math
from pathlib import Path
import runpy
import unittest

bridge = runpy.run_path(str(Path(__file__).resolve().parents[1]/'tools/ardupilot_mavlink_bridge_node'))
mav = bridge['mavutil'].mavlink


class ArduPlaneBridgeTest(unittest.TestCase):
    def test_boot_clock_removes_variable_transport_delay(self):
        c=bridge['BootClockMapper']()
        self.assertAlmostEqual(c.map(10.,110.),110.)
        self.assertAlmostEqual(c.map(10.02,110.12),110.02)
        self.assertAlmostEqual(c.map(10.04,110.14),110.04)
        self.assertAlmostEqual(c.map(.01,2.),2.)
        with self.assertRaises(ValueError):
            c.map(float('nan'),2.)

    def test_tilt_and_body_rates_roundtrip(self):
        import numpy as np
        r=bridge['Rotation'].from_euler('xyz',[.3,-.2,1.1]).as_matrix()
        ned=bridge['T']@r@bridge['S']
        np.testing.assert_allclose(bridge['T'].T@ned@bridge['S'].T,r,atol=1e-12)
        np.testing.assert_allclose(bridge['S']@np.array([100.,200.,300.])*.001,[.1,-.2,-.3])

    def test_optical_pose_contract(self):
        import xml.etree.ElementTree as ET
        import numpy as np
        root=Path(__file__).resolve().parents[1]
        for name in ('model.sdf','arduplane.sdf'):
            tree=ET.parse(root/'simulation/models/glider_visual'/name)
            pose=tree.find(".//sensor[@name='nadir_camera']/pose")
            a=[float(v) for v in pose.text.split()]
            # Gazebo camera: forward +X, left +Y, up +Z.
            optical=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
            actual=bridge['Rotation'].from_euler('xyz',a[3:]).as_matrix()@optical
            np.testing.assert_allclose(actual,[[0,-1,0],[-1,0,0],[0,0,-1]],atol=1e-12)
            self.assertAlmostEqual(a[2],-.25)

    def test_udp_datagram_all_frames_decoded(self):
        buffer = io.BytesIO()
        writer=mav.MAVLink(buffer,srcSystem=1,srcComponent=1)
        for n in range(100):
            writer.attitude_send(n*20,0.,0.,0.,.1,.2,.3)
        messages=mav.MAVLink(None).parse_buffer(buffer.getvalue())
        self.assertEqual(len(messages),100)
        self.assertEqual(messages[-1].time_boot_ms,1980)

    def test_east_heading_to_ned(self):
        q = bridge['ned_attitude']([0., 0., 0., 1.])
        self.assertAlmostEqual(abs(q[0]), math.sqrt(.5))
        self.assertAlmostEqual(q[3]/q[0], 1.)
        self.assertAlmostEqual(q[1], 0.)
        self.assertAlmostEqual(q[2], 0.)

    def test_gazebo_imu_specific_force_is_frd(self):
        import xml.etree.ElementTree as ET
        root = Path(__file__).resolve().parents[1]
        tree = ET.parse(root/'simulation/models/glider_visual/arduplane.sdf')
        pose = tree.find(".//sensor[@type='imu']/pose")
        self.assertIsNotNone(pose)
        angles = [float(x) for x in pose.text.split()][3:]
        sensor = bridge['Rotation'].from_euler('xyz', angles, degrees=True)
        force = sensor.inv().apply([0., 0., 9.80665])
        self.assertAlmostEqual(force[2], -9.80665)
        self.assertAlmostEqual(force[1], 0., places=10)

    def test_heading_conversion_all_cardinal_directions(self):
        for yaw in (0., math.pi/2, math.pi, -math.pi/2):
            q = bridge['Rotation'].from_euler('z', yaw).as_quat()
            w, x, y, z = bridge['ned_attitude'](q)
            actual = bridge['Rotation'].from_quat([x,y,z,w]).as_euler('xyz')[2]
            expected = math.pi/2-yaw
            self.assertAlmostEqual(math.atan2(math.sin(actual-expected), math.cos(actual-expected)), 0.)

    def test_pressure_agl(self):
        self.assertEqual(bridge['pressure_height'](1013.25, 1013.25), 0.)
        p = 1013.25 * (1.-40./44330.)**(1./.190295)
        self.assertAlmostEqual(bridge['pressure_height'](p, 1013.25), 40.)
        with self.assertRaises(ValueError):
            bridge['pressure_height'](float('nan'), 1013.25)

    def test_mavlink2_attitude_frame(self):
        buffer = io.BytesIO()
        writer = mav.MAVLink(buffer, srcSystem=42, srcComponent=191)
        writer.set_attitude_target_send(1000, 1, 1, 7, bridge['ned_attitude']([0., 0., 0., 1.]),
                                        0., 0., 0., 0.)
        wire = buffer.getvalue()
        self.assertEqual(wire[0], 0xfd)
        parsed = mav.MAVLink(None).parse_buffer(wire)[0]
        self.assertEqual(parsed.get_type(), 'SET_ATTITUDE_TARGET')
        self.assertEqual(parsed.type_mask, 7)
        self.assertEqual(parsed.thrust, 0.)
        self.assertEqual(parsed.get_srcSystem(), 42)


if __name__ == '__main__':
    unittest.main()
