#!/user/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class SpawnClient(Node):
    def __init__(self, name):
        super().__init__(name)
        self.cli = self.create_client(Spawn, '/spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Spawn.Request()

    def send_request(self):
        self.req.x = 2.0
        self.req.y = 2.0
        self.req.theta = 0.0
        self.req.name = 'turtle2'
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

def main(args=None):
    rclpy.init(args=args)
    node = SpawnClient('turtle_spawner')
    node.send_request()

    while rclpy.ok():
        rclpy.spin(node)

        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                    node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                node.get_logger().info(f'service done, response: {response}')
            break
    
    node.destroy_node()
    rclpy.shutdown()