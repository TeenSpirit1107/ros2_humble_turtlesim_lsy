#!/user/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, pi, atan2
from turtlesim.srv import Spawn, Kill
import random

class GoToGoal(Node):
    pose_x = 0.0
    pose_y = 0.0
    theta = 0.0
    def __init__(self, name):
        super().__init__(name)
        # 发布速度 创建发布者对象（消息类型， 话题名，队列长度）
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 创建计时器，定时执行函数
        self.timer = self.create_timer(0.05, self.go_to_goal)
        # 创建乌龟位置的订阅者对象（消息类型， 话题名，回调函数，队列长度）
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # 初始化目标位置

        # 调用/spawn服务，创建新乌龟
        self.cli1 = self.create_client(Spawn, '/spawn')
        while not self.cli1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service_sqawn not available, waiting again...')
        self.req = Spawn.Request()

        self.cli2 = self.create_client(Kill, '/kill')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service_kill not available, waiting again...')
        self.req2 = Kill.Request()

        self.count =2
        self.set_goal()

    def spawn(self):
        self.req.x = random.uniform(0.0, 11.0)
        self.req.y = random.uniform(0.0, 11.0)
        self.req.theta = 0.0
        self.req.name = f'turtle{self.count}'
        self.future = self.cli1.call_async(self.req)
        self.get_logger().info(f'spawning new turtle at ({self.req.x}, {self.req.y})')

    def kill(self, count):
        self.req2.name = f'turtle{count}'
        self.future = self.cli2.call_async(self.req2)
        #rclpy.spin_until_future_complete(self, self.future)

    def set_goal(self):
        # 设置乌龟的目标（将会被设置为新乌龟的坐标）
        self.spawn()
        self.x_goal = self.req.x
        self.y_goal = self.req.y
        self.get_logger().info(f"the goal is at ({self.x_goal}, {self.y_goal})")

    def pose_callback(self, pose_msg=Pose()):
        self.pose_x = pose_msg.x
        self.pose_y = pose_msg.y
        self.theta = pose_msg.theta

    def go_to_goal(self):
        # 计算距离
        self.dist_x = self.x_goal - self.pose_x
        self.dist_y = self.y_goal - self.pose_y
        distance = sqrt(self.dist_x**2 + self.dist_y**2)
        speed = Twist()
        # 配置速度
        if distance > 0.1:
            speed.linear.x = 2.0
            if distance < 2.0:
                speed.linear.x = 1.0

            angle = atan2(self.dist_y, self.dist_x)
            diff = angle - self.theta

            if diff > pi:
                diff -= 2*pi
            elif diff < -pi:
                diff += 2*pi

            speed.angular.z = 6*(diff)
        else:
            speed.angular.z = 0.0
            speed.linear.x = 0.0
            self.get_logger().info("reached the goal")
            self.kill(self.count)
            self.count += 1
            self.set_goal()

        self.velocity_publisher.publish(speed)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal('go_to_goal')
    while rclpy.ok():
        rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()