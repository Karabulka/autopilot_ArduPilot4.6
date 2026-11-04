"""Coordinate and MAVLink v2 wire contract tests."""
import io
import math
from pathlib import Path
import runpy
import unittest

bridge = runpy.run_path(str(Path(__file__).resolve().parents[1]/'tools/ardupilot_mavlink_bridge_node'))
mav = bridge['mavutil'].mavlink


class ArduPlaneBridgeTest(unittest.TestCase):
    def test_east_heading_to_ned(self):
        q = bridge['ned_attitude']([0., 0., 0., 1.])
        self.assertAlmostEqual(abs(q[0]), math.sqrt(.5))
        self.assertAlmostEqual(q[3]/q[0], 1.)
        self.assertAlmostEqual(q[1], 0.)
        self.assertAlmostEqual(q[2], 0.)

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
