#!/user/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, pi, atan2,sin, cos
from turtlesim.srv import Spawn, Kill
import random

class GoToGoal(Node):
    SPEED_FAST = 4.0
    SPEED_SLOW = 1.6
    DISTANCE_REACHED = 0.1
    DISTANCE_CLOSE = 1.0
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
        self.cli2 = self.create_client(Kill, '/kill')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service_kill not available, waiting again...')
        self.req2 = Kill.Request()
        self.kill(2)

        self.cli1 = self.create_client(Spawn, '/spawn')
        while not self.cli1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service_sqawn not available, waiting again...')
        self.req = Spawn.Request()

        
        self.count =3
        self.set_goal()

    def spawn(self):
        self.req.x = random.uniform(0.3, 10.7) # if you set it to (0.0,11.0), sometimes the turtle may be spawned at somewhere out of bound and will not be displayed in the window.
        self.req.y = random.uniform(0.3, 10.7)
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
        speed = Twist()
        self.dist_x = self.x_goal - self.pose_x
        self.dist_y = self.y_goal - self.pose_y
        angle = atan2(self.dist_y, self.dist_x)
        distance = sqrt(self.dist_x**2 + self.dist_y**2)
        # 配置速度
        if distance > self.DISTANCE_REACHED:
            speed_magn = self.SPEED_FAST if distance >self.DISTANCE_CLOSE else 1.0
            speed.linear.x = speed_magn*cos(angle)
            speed.linear.y = speed_magn*sin(angle)
            speed.angular.z = 0.0
        else:
            speed.linear.x,speed.linear.y,speed.angular.z=0.0,0.0,0.0
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
