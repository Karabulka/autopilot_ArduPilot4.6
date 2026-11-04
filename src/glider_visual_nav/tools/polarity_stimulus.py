"""Predefined test commands only. This process has NO ground-truth subscription."""
import argparse
import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped

parser=argparse.ArgumentParser()
parser.add_argument('--start-time',type=float,required=True)
args=parser.parse_args()
rclpy.init()
node=rclpy.create_node('predefined_polarity_stimulus',parameter_overrides=[Parameter('use_sim_time',value=True)])
publisher=node.create_publisher(PoseStamped,'/glider/target_pose',10)


def tick():
    elapsed=node.get_clock().now().nanoseconds*1e-9-args.start_time
    roll=.15 if 2<elapsed<4 else 0
    pitch_up=.12 if 6<elapsed<8 else .04
    c,s=math.cos,math.sin
    msg=PoseStamped(); msg.header.frame_id='odom'; msg.header.stamp=node.get_clock().now().to_msg()
    msg.pose.orientation.x=s(roll/2)*c(-pitch_up/2)
    msg.pose.orientation.y=c(roll/2)*s(-pitch_up/2)
    msg.pose.orientation.z=-s(roll/2)*s(-pitch_up/2)
    msg.pose.orientation.w=c(roll/2)*c(-pitch_up/2)
    publisher.publish(msg)


timer=node.create_timer(.02,tick)
try: rclpy.spin(node)
except (KeyboardInterrupt,ExternalShutdownException): pass
finally:
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
