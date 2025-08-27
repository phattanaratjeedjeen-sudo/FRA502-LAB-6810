#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
import numpy as np
from std_msgs.msg import Int64
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty

class DummyNode(Node):
    def __init__(self):
        super().__init__('eater_node')
        # pub turtle1/cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_timer(0.1, self.timer_callback)

        # sub turtle1/pose
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.turtle_pose = np.array([0.0, 0.0, 0.0])

        # sub mouse_position
        self.create_subscription(Point, '/mouse_position', self.mouse_position_callback, 10)
        self.mouse_position = np.array([0.0, 0.0])
        self.mouse_position_q = np.array([5.0, 5.0])

        # client spawn_pizza
        self.spawn_pizza_client = self.create_client(GivePosition, '/spawn_pizza')
        
        # client turtle_eat
        self.eat_pizza_client = self.create_client(Empty, '/turtle1/eat')

        # sub pizza_count
        self.create_subscription(Int64, '/turtle1/pizza_count', self.pizza_count_callback, 10)
        self.pizza_count = 0
        self.pizza_limit = 5

        # sub goal_pose
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.goal_position = np.array([0.0, 0.0])
        self.get_logger().info('eater_node: run')

    def turtle_eat(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)    

    def spawn_pizza(self, x, y):
        position_request = GivePosition.Request()
        position_request.x = x
        position_request.y = y
        self.spawn_pizza_client.call_async(position_request)

    def goal_pose_callback(self, msg):
        self.goal_position[0] = msg.pose.position.x
        self.goal_position[1] = msg.pose.position.y
        # self.goal_x = self.goal_position[0]
        # self.goal_y = self.goal_position[1]
        self.mouse_position_q = np.append(self.mouse_position_q, self.goal_position)

    def pizza_count_callback(self, msg):
        self.pizza_count = msg.data
        self.get_logger().info(f'Eat pizza: {self.pizza_count} from {self.pizza_limit}')

    def mouse_position_callback(self, msg):
        self.mouse_position[0] = msg.x
        self.mouse_position[1] = msg.y
        self.mouse_position_q = np.append(self.mouse_position_q, self.mouse_position)
        self.goal_x = self.mouse_position[0]
        self.goal_y = self.mouse_position[1]

        if self.pizza_count < self.pizza_limit:
            self.spawn_pizza(self.mouse_position[0], self.mouse_position[1])

    def pose_callback(self, msg):
        self.turtle_pose[0] = msg.x
        self.turtle_pose[1] = msg.y
        self.turtle_pose[2] = msg.theta

    def timer_callback(self):
        msg = Twist()
        if self.pizza_count < self.pizza_limit:
            dx = self.mouse_position_q[0] - self.turtle_pose[0]
            dy = self.mouse_position_q[1] - self.turtle_pose[1]
        else:
            dx = self.mouse_position[0] - self.turtle_pose[0]
            dy = self.mouse_position[1] - self.turtle_pose[1]
            # self.get_logger().info(f'x: {self.mouse_position[0]}, y: {self.mouse_position[1]}')
            # self.get_logger().info(f'tx: {self.turtle_pose[0]}, ty: {self.turtle_pose[1]}')
            # self.get_logger().info(f'dx: {dx}, dy: {dy}')

        self.d = np.sqrt(np.power(dx, 2) + np.power(dy, 2))
        alpha = np.arctan2(dy, dx)
        e = alpha - self.turtle_pose[2]
        e = np.arctan2(np.sin(e), np.cos(e))
        
        k1 = 0.8
        k2 = 4
        msg.linear.x = k1 * self.d
        msg.angular.z = k2 * e
        self.cmd_vel_pub.publish(msg)
        
        if self.pizza_count < self.pizza_limit:
            self.turtle_eat()    

        if self.d < 1.5 and len(self.mouse_position_q) > 2:
            self.mouse_position_q = np.delete(self.mouse_position_q, (0, 1))

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
