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
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.remove_turtle_client = self.create_client(Kill, '/remove_turtle')
        self.create_timer(0.1, self.timer_callback)

        self.turtle1_pose = np.array([0.0, 0.0, 0.0])
        self.turtle2_pose = np.array([0.0, 0.0, 0.0])
        self.pizza_count = 0
        self.pizza_limit = 5

        self.get_logger().info('killer_node: run')
    
    def kill_turtle_callback(self):
        kill_request = Kill.Request()
        kill_request.name = "turtle1"
        self.remove_turtle_client.call_async(kill_request)

    def pizza_count_callback(self, msg):
        self.pizza_count = msg.data
        self.get_logger().info(f'Eat pizza: {self.pizza_count} from {self.pizza_limit}')

    def pose1_callback(self, msg):
        self.turtle1_pose[0] = msg.x
        self.turtle1_pose[1] = msg.y
        self.turtle1_pose[2] = msg.theta

    def pose2_callback(self, msg):
        self.turtle2_pose[0] = msg.x
        self.turtle2_pose[1] = msg.y
        self.turtle2_pose[2] = msg.theta    

    def timer_callback(self):
        msg = Twist()
        dx = self.turtle1_pose[0] - self.turtle2_pose[0]
        dy = self.turtle1_pose[1] - self.turtle2_pose[1]
        self.d = np.sqrt(np.power(dx, 2) + np.power(dy, 2))
        
        alpha = np.arctan2(dy, dx)
        e = alpha - self.turtle2_pose[2]
        e = np.arctan2(np.sin(e), np.cos(e))
        
        k1 = 0.5
        k2 = 2
        msg.linear.x = k1 * self.d
        msg.angular.z = k2 * e

        if self.pizza_count >= self.pizza_limit:
            self.cmd_vel_pub.publish(msg)
        
        if self.d < 1.0 and self.pizza_count >= self.pizza_limit:
            self.kill_turtle_callback()

def main(args=None):
    rclpy.init(args=args)
    node = KillerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
