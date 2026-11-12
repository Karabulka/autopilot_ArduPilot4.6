#!/usr/bin/env python3
"""Independent evaluator: records only, never publishes navigation inputs."""
import argparse, json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, TwistWithCovarianceStamped

def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-09

def vec(v):
    return [v.x, v.y, v.z]

class Logger(Node):

    def __init__(self, path):
        super().__init__('sensor_alignment_evaluator')
        self.declare_parameter('use_sim_time', True) if not self.has_parameter('use_sim_time') else None
        self.f = path.open('w')
        self.subs = []
        for topic, typ, kind in [('/evaluation/ground_truth', Odometry, 'truth'), ('/glider/odometry', Odometry, 'nav'), ('/glider/imu', Imu, 'imu'), ('/evaluation/sensor_imu', Imu, 'direct_imu'), ('/glider/barometric_height', PointStamped, 'height'), ('/glider/visual_velocity', TwistWithCovarianceStamped, 'flow')]:
            self.subs.append(self.create_subscription(typ, topic, lambda m, k=kind: self.record(m, k), Q))

    def record(self, m, k):
        d = dict(kind=k, t=stamp(m), rx=self.get_clock().now().nanoseconds * 1e-09)
        if k in ('truth', 'nav'):
            q = m.pose.pose.orientation
            d.update(q=[q.x, q.y, q.z, q.w], p=vec(m.pose.pose.position), v=vec(m.twist.twist.linear), w=vec(m.twist.twist.angular))
        elif k in ('imu', 'direct_imu'):
            q = m.orientation
            d.update(q=[q.x, q.y, q.z, q.w], w=vec(m.angular_velocity))
            if k == 'direct_imu':
                d['w'] = [d['w'][0], -d['w'][1], -d['w'][2]]
        elif k == 'height':
            d['z'] = m.point.z
        else:
            d['v'] = vec(m.twist.twist.linear)
        self.f.write(json.dumps(d) + '\n')
        self.f.flush()
if __name__ == '__main__':
    a = argparse.ArgumentParser()
    a.add_argument('output', type=Path)
    args, extra = a.parse_known_args()
    rclpy.init(args=extra)
    n = Logger(args.output)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.f.close()
        n.destroy_node()
        rclpy.shutdown()
