#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import FullState
from scipy.spatial.transform import Rotation as R
import numpy as np

class HoverAboveJackalNode(Node):
    def __init__(self):
        super().__init__('hover_above_jackal')
        self.declare_parameter('cf_name', 'cf1')
        self.declare_parameter('jackal_name', 'jackal')
        self.declare_parameter('hover_height', 1.0)
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)
        self.declare_parameter('match_yaw', True)
        self.declare_parameter('use_vicon', True)
        
        self.cf_name = self.get_parameter('cf_name').value
        self.jackal_name = self.get_parameter('jackal_name').value
        self.hover_height = self.get_parameter('hover_height').value
        self.offset_x = self.get_parameter('offset_x').value
        self.offset_y = self.get_parameter('offset_y').value
        self.offset_z = self.get_parameter('offset_z').value
        self.match_yaw = self.get_parameter('match_yaw').value
        
        topic = f'/vicon/{self.jackal_name}/{self.jackal_name}' if self.get_parameter('use_vicon').value else f'/{self.jackal_name}/pose'
        self.sub = self.create_subscription(PoseStamped, topic, self.callback, 10)
        self.pub = self.create_publisher(FullState, f'/{self.cf_name}/cmd_full_state', 10)
        self.last_log = self.get_clock().now()
        self.get_logger().info(f'Hovering above {self.jackal_name} at {self.hover_height}m')
    
    def callback(self, msg):
        jp = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        jq = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        offset = R.from_quat(jq).apply([self.offset_x, self.offset_y, self.offset_z])
        tp = jp + offset; tp[2] = self.hover_height
        tq = jq if self.match_yaw else np.array([0,0,0,1])
        
        cmd = FullState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'world'
        cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z = tp
        cmd.pose.orientation.x, cmd.pose.orientation.y, cmd.pose.orientation.z, cmd.pose.orientation.w = tq
        self.pub.publish(cmd)
        
        if (self.get_clock().now() - self.last_log).nanoseconds > 1e9:
            self.get_logger().info(f'J:({jp[0]:.2f},{jp[1]:.2f}) → CF:({tp[0]:.2f},{tp[1]:.2f},{tp[2]:.2f})')
            self.last_log = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(HoverAboveJackalNode())
    except KeyboardInterrupt:
        print('\nStopped')
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()