#!/usr/bin/env python3
"""Direct onboard IMU + static pressure adapter; no simulator pose subscription."""
import math
from statistics import median
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import Imu, FluidPressure
from geometry_msgs.msg import PointStamped

class Sensors(Node):

    def __init__(self):
        super().__init__('onboard_sensor_node')
        self.declare_parameter('initial_body_height_m', -1.0)
        if self.get_parameter('initial_body_height_m').value < 0:
            raise ValueError('initial measured/reference AGL is required')
        self.declare_parameter('reference_pressure_pa', 101325.0)
        self.initial = []
        self.offset = None
        self.ip = self.create_publisher(Imu, 'glider/imu', Q)
        self.hp = self.create_publisher(PointStamped, 'glider/barometric_height', Q)
        self.create_subscription(Imu, 'sensors/navigation_imu', self.imu, Q)
        self.create_subscription(FluidPressure, 'sensors/static_pressure', self.pressure, Q)

    def imu(self, m):
        if m.header.frame_id != 'base_link':
            return
        self.ip.publish(m)

    def pressure(self, m):
        if not math.isfinite(m.fluid_pressure) or m.fluid_pressure <= 0:
            return
        h = 44330.0 * (1.0 - (m.fluid_pressure / self.get_parameter('reference_pressure_pa').value) ** 0.190295)
        if self.offset is None:
            self.initial.append(h)
            if len(self.initial) < 25:
                return
            self.offset = self.get_parameter('initial_body_height_m').value - median(self.initial)
        out = PointStamped()
        out.header = m.header
        out.header.frame_id = 'odom'
        out.point.z = h + self.offset
        self.hp.publish(out)
if __name__ == '__main__':
    rclpy.init()
    n = Sensors()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
