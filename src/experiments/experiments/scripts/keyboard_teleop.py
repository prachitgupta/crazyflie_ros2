#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('cf_name', 'cf1')
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('vertical_speed', 0.3)
        
        self.cf_name = self.get_parameter('cf_name').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.vertical_speed = self.get_parameter('vertical_speed').value
        
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.cf_name}/cmd_vel_legacy', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_twist = Twist()
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Keyboard Teleop: w/s/a/d/q/e/i/k/x, CTRL-C to quit')
    
    def get_key(self, timeout=0.1):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def timer_callback(self):
        self.cmd_vel_pub.publish(self.current_twist)
        key = self.get_key(timeout=0.05)
        if key: self.process_key(key)
    
    def process_key(self, key):
        twist = Twist()
        if key == 'w': twist.linear.x = self.linear_speed
        elif key == 's': twist.linear.x = -self.linear_speed
        elif key == 'a': twist.linear.y = self.linear_speed
        elif key == 'd': twist.linear.y = -self.linear_speed
        elif key == 'q': twist.angular.z = self.angular_speed
        elif key == 'e': twist.angular.z = -self.angular_speed
        elif key == 'i': twist.linear.z = self.vertical_speed
        elif key == 'k': twist.linear.z = -self.vertical_speed
        elif key == 'x': twist = Twist()
        elif key == '\x03': raise KeyboardInterrupt
        else: return
        self.current_twist = twist
    
    def cleanup(self):
        self.current_twist = Twist()
        self.cmd_vel_pub.publish(self.current_twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nStopped')
    finally:
        if node: node.cleanup(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
