#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import FullState
from scipy.spatial.transform import Rotation as R
import numpy as np
from collections import deque

class CircularTrackingNode(Node):
    def __init__(self):
        super().__init__('circular_tracking')
        self.declare_parameter('cf_name', 'cf1')
        self.declare_parameter('jackal_name', 'jackal')
        self.declare_parameter('hover_height', 1.2)
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)
        self.declare_parameter('match_jackal_yaw', True)
        self.declare_parameter('face_center', False)
        self.declare_parameter('estimate_velocity', True)
        self.declare_parameter('use_vicon', True)
        
        self.cf_name = self.get_parameter('cf_name').value
        self.jackal_name = self.get_parameter('jackal_name').value
        self.hover_height = self.get_parameter('hover_height').value
        self.offset_x = self.get_parameter('offset_x').value
        self.offset_y = self.get_parameter('offset_y').value
        self.offset_z = self.get_parameter('offset_z').value
        self.match_yaw = self.get_parameter('match_jackal_yaw').value
        self.face_center = self.get_parameter('face_center').value
        self.estimate_vel = self.get_parameter('estimate_velocity').value
        
        self.pos_hist = deque(maxlen=5)
        self.time_hist = deque(maxlen=5)
        self.count = 0
        
        topic = f'/vicon/{self.jackal_name}/{self.jackal_name}' if self.get_parameter('use_vicon').value else f'/{self.jackal_name}/pose'
        self.sub = self.create_subscription(PoseStamped, topic, self.callback, 10)
        self.pub = self.create_publisher(FullState, f'/{self.cf_name}/cmd_full_state', 10)
        self.get_logger().info(f'Tracking {self.jackal_name}, height={self.hover_height}m')
    
    def estimate_vel(self, pos, time):
        if len(self.pos_hist) < 2: return np.zeros(3)
        dt = (time - self.time_hist[-1]).nanoseconds / 1e9
        if dt < 0.001: return np.zeros(3)
        vel = (pos - self.pos_hist[-1]) / dt
        if hasattr(self, 'last_vel'):
            vel = 0.3 * vel + 0.7 * self.last_vel
        self.last_vel = vel
        return vel
    
    def callback(self, msg):
        t = self.get_clock().now()
        jp = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        jq = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
        self.pos_hist.append(jp)
        self.time_hist.append(t)
        
        offset = R.from_quat(jq).apply([self.offset_x, self.offset_y, self.offset_z])
        tp = jp + offset; tp[2] = self.hover_height
        tv = self.estimate_vel(jp, t) if self.estimate_vel else np.zeros(3)
        
        if self.face_center:
            yaw = np.arctan2(-tp[1], -tp[0])
            tq = R.from_euler('z', yaw).as_quat()
        elif self.match_yaw:
            tq = jq
        else:
            tq = np.array([0,0,0,1])
        
        cmd = FullState()
        cmd.header.stamp = t.to_msg()
        cmd.header.frame_id = 'world'
        cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z = tp
        cmd.pose.orientation.x, cmd.pose.orientation.y, cmd.pose.orientation.z, cmd.pose.orientation.w = tq
        cmd.twist.linear.x, cmd.twist.linear.y = tv[0], tv[1]
        self.pub.publish(cmd)
        
        self.count += 1
        if self.count % 30 == 0:
            self.get_logger().info(f'J:({jp[0]:.2f},{jp[1]:.2f}) → CF:({tp[0]:.2f},{tp[1]:.2f},{tp[2]:.2f}, v={np.linalg.norm(tv):.2f})')

def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(CircularTrackingNode())
    except KeyboardInterrupt:
        print('\nStopped')
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()