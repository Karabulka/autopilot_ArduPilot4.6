"""Direct sensor adapter contracts: preserve acquisition time and reject bad data."""
import os
from pathlib import Path
import runpy
import unittest
root=Path(__file__).resolve().parents[1]
os.environ['ROS_DOMAIN_ID']='91'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(root/'config/fastdds_wsl.xml')
os.environ['FASTDDS_DEFAULT_PROFILES_FILE']=os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']
module=runpy.run_path(str(root/'tools/onboard_sensor_node.py'))
rclpy=module['rclpy']
class Sink:
    def __init__(self):self.messages=[]
    def publish(self,message):self.messages.append(message)
class SensorContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args','-p','initial_body_height_m:=145.25'])
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.node=module['Sensors']();self.node.ip=Sink();self.node.hp=Sink()
    def tearDown(self):self.node.destroy_node()
    def test_imu_preserves_acquisition_stamp_and_flu(self):
        m=module['Imu']();m.header.frame_id='base_link';m.header.stamp.sec=42
        m.orientation.w=1.;m.angular_velocity.y=-.2
        self.node.imu(m);self.assertIs(self.node.ip.messages[0],m)
        self.assertEqual(self.node.ip.messages[0].header.stamp.sec,42)
        m.header.frame_id='frd';self.node.imu(m);self.assertEqual(len(self.node.ip.messages),1)
    def test_pressure_startup_and_height_change(self):
        m=module['FluidPressure']();m.fluid_pressure=101325.
        for _ in range(25):self.node.pressure(m)
        self.assertAlmostEqual(self.node.hp.messages[-1].point.z,145.25)
        m.header.stamp.sec=7;m.fluid_pressure=101325.*(1.-40./44330.)**(1./.190295)
        self.node.pressure(m)
        self.assertAlmostEqual(self.node.hp.messages[-1].point.z,185.25,places=7)
        self.assertEqual(self.node.hp.messages[-1].header.stamp.sec,7)
    def test_acceptance_fails_closed_without_transport(self):
        combine=runpy.run_path(str(root/'tools/build_acceptance_report.py'))['combine']
        report=combine({'passed':True,'airborne_seconds':90},{'accuracy_passed':True},{})
        self.assertFalse(report['passed'])
        self.assertFalse(report['checks']['arduplane_roll_pitch'])

    def test_invalid_pressure_not_published(self):
        m=module['FluidPressure']();m.fluid_pressure=float('nan')
        self.node.pressure(m);self.assertFalse(self.node.hp.messages)
if __name__=='__main__':unittest.main()
