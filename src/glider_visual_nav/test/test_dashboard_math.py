"""Independent display/evaluator math; no simulator/control inputs are modified."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from flight_dashboard import cross_track, interpolate, world_velocity, wrap
from dashboard_plots import Timeline, command_at, line_runs


class DashboardMath(unittest.TestCase):
    def test_left_right_cross_track(self):
        self.assertEqual(cross_track((0, 0), (100, 0), (40, 7)), 7)
        self.assertEqual(cross_track((0, 0), (100, 0), (40, -7)), -7)
        self.assertEqual(cross_track((0, 0), (0, 100), (-7, 40)), 7)

    def test_body_velocity_north_heading(self):
        q = SimpleNamespace(x=0., y=0., z=math.sqrt(.5), w=math.sqrt(.5))
        v = SimpleNamespace(x=12., y=2., z=0.)
        east, north = world_velocity(q, v)
        self.assertAlmostEqual(east, -2.)
        self.assertAlmostEqual(north, 12.)

    def test_truth_timestamp_and_angle_wrap(self):
        a = dict(t=1., x=0., y=0., z=40., yaw=math.radians(179), roll=0., vx=10., vy=0., speed=10.)
        b = dict(a, t=1.1, x=1., yaw=math.radians(-179))
        middle = interpolate([a, b], 1.05)
        self.assertAlmostEqual(middle['x'], .5)
        self.assertAlmostEqual(abs(wrap(middle['yaw'])), math.pi)
        self.assertIsNone(interpolate([a, b], 1.2))
        self.assertIsNone(interpolate([a, dict(b, t=2.)], 1.5))

    def test_course_plot_does_not_join_angle_wrap(self):
        runs = line_runs([(0., 179.), (.1, -179.), (.2, -175.)], 0., 1., angular=True)
        self.assertEqual([len(run) for run in runs], [1, 2])

    def test_missing_samples_and_gaps_remain_visible(self):
        points = [(0., 1.), (.1, None), (.2, 2.), (2., 3.)]
        self.assertEqual(len(line_runs(points, 0., 3.)), 3)
        self.assertEqual(line_runs(points, .3, 1.), [])

    def test_command_alignment_no_future_or_stale_command(self):
        commands = [dict(t=1., segment=0), dict(t=2., segment=1)]
        self.assertEqual(command_at(commands, 1.1)['segment'], 0)
        self.assertEqual(command_at(commands, 2.)['segment'], 1)
        self.assertIsNone(command_at(commands, .9))
        self.assertIsNone(command_at(commands, 1.8))

    def test_bounded_history_no_duplicate_or_reordered_samples(self):
        data = Timeline()
        for i in range(1000):
            data.add('height', float(i), 40.)
        self.assertLessEqual(len(data.channels['height']), 306)
        data.add('height', 999., 41.)
        data.add('height', 998., 99.)
        self.assertEqual(data.channels['height'][-1], (999., 41.))
        data.add('height', 1000., float('nan'))
        self.assertEqual(data.latest, 999.)

    def test_csv_keeps_missing_value_and_waypoint_event(self):
        import csv
        data = Timeline()
        data.add('course', 1., None)
        data.add('course', 1.1, 90.)
        data.event(1.1, 1, 'WP3')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'charts.csv'
            data.export_csv(path)
            with path.open() as stream:
                rows = list(csv.reader(stream))
        self.assertEqual(rows[1][-1], '')
        self.assertEqual(rows[-1][2:], ['waypoint_event', 'WP3'])


if __name__ == '__main__':
    unittest.main()
