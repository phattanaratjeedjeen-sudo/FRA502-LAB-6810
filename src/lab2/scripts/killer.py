#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
from turtlesim.srv import Kill
from std_msgs.msg import Int64

class KillerNode(Node):
    def __init__(self):
        super().__init__('killer_node')
        self.create_subscription(Pose, '/turtle1/pose', self.pose1_callback, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.pose2_callback, 10)
        self.create_subscription(Int64, '/turtle1/pizza_count', self.pizza_count_callback, 10)
        self.create_subscription(Int64, '/set_max_pizza', self.set_max_pizza_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.remove_turtle_client = self.create_client(Kill, '/remove_turtle')
        self.create_timer(0.01, self.timer_callback)

        self.turtle1_pose = np.array([0.0, 0.0])
        self.turtle2_pose = np.array([0.0, 0.0, 0.0])
        self.pizza_count = 0
        self.pizza_limit = 5
        self.eat_all = False
        self.ei_dis = 0.0
        self.ei_ang = 0.0

        self.get_logger().info('killer_node: run')

    def set_max_pizza_callback(self, msg:Int64):
        self.pizza_limit = msg.data

    def kill_turtle_callback(self):
        kill_request = Kill.Request()
        kill_request.name = "turtle1"
        self.remove_turtle_client.call_async(kill_request)

    def pizza_count_callback(self, msg:Int64):
        self.pizza_count = msg.data
        if self.pizza_count >= self.pizza_limit:
            self.eat_all = True

    def pose1_callback(self, msg:Pose):
        self.turtle1_pose[0] = msg.x
        self.turtle1_pose[1] = msg.y

    def pose2_callback(self, msg:Pose):
        self.turtle2_pose[0] = msg.x
        self.turtle2_pose[1] = msg.y
        self.turtle2_pose[2] = msg.theta    

    def timer_callback(self):
        if self.eat_all:
            msg = Twist()
            dx = self.turtle1_pose[0] - self.turtle2_pose[0]
            dy = self.turtle1_pose[1] - self.turtle2_pose[1]
            alpha = np.arctan2(dy, dx)
            e_dis = np.sqrt(np.power(dx, 2) + np.power(dy, 2))
            self.ei_dis += e_dis*0.01
            e_ang = alpha - self.turtle2_pose[2]
            e_ang = np.arctan2(np.sin(e_ang), np.cos(e_ang))
            self.ei_ang += e_ang*0.01
            msg.linear.x = 0.5 * e_dis + 0.05 * self.ei_dis
            msg.angular.z = 5.5 * e_ang + 0.05 * self.ei_ang

            if e_dis < 1.0:
                self.kill_turtle_callback()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                
            self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KillerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
