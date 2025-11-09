#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from crazyflie_interfaces.srv import Land
from rclpy.duration import Duration
import time

class LandNode(Node):
    def __init__(self):
        super().__init__('land_node')
        self.declare_parameter('height', 0.0)
        self.declare_parameter('duration', 3.0)
        self.declare_parameter('cf_name', 'cf1')
        self.declare_parameter('group_mask', 0)
        
        self.height = self.get_parameter('height').value
        self.duration = self.get_parameter('duration').value
        self.cf_name = self.get_parameter('cf_name').value
        self.group_mask = self.get_parameter('group_mask').value
        
        self.client = self.create_client(Land, f'/{self.cf_name}/land')
        self.get_logger().info('Waiting for land service...')
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Service not available')
        self.get_logger().info('✓ Service ready')
    
    def send_land(self):
        req = Land.Request()
        req.height = float(self.height)
        req.duration = Duration(seconds=self.duration).to_msg()
        req.group_mask = self.group_mask
        self.get_logger().info(f'Landing in {self.duration}s')
        return self.client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = LandNode()
        future = node.send_land()
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result():
            node.get_logger().info('✓ Land sent!')
            time.sleep(node.duration + 1)
            node.get_logger().info('✓ Complete!')
        else:
            node.get_logger().error('✗ Failed')
    except KeyboardInterrupt:
        print('\nStopped')
    finally:
        if rclpy.ok(): node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()