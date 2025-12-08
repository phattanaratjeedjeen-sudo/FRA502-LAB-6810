#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
import numpy as np
from std_msgs.msg import Int64
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty

class EaterNode(Node):
    def __init__(self):
        super().__init__('eater_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_subscription(Int64, '/turtle1/pizza_count', self.pizza_count_callback, 10)
        self.create_subscription(Point, '/mouse_position', self.mouse_position_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(Int64, '/set_max_pizza', self.set_max_pizza_callback, 10)
        self.spawn_pizza_client = self.create_client(GivePosition, '/spawn_pizza')
        self.eat_pizza_client = self.create_client(Empty, '/turtle1/eat')
        self.create_timer(0.01, self.timer_callback)

        self.turtle_pose = [0.0, 0.0, 0.0]
        self.target_position= [7.0, 7.0]
        self.target_position_queue = [7.0, 7.0]
        self.pizza_count = 0
        self.pizza_limit = 5
        self.eat_all = False
        self.total_spawned = 1
        self.ei_dis = 0.0
        self.ei_ang = 0.0

        self.get_logger().info(f'Eater_node: run, Default max pizza: {self.pizza_limit}')

    def set_max_pizza_callback(self, msg:Int64):
        if msg.data < self.pizza_count:
            self.get_logger().info(f'Max pizza {msg.data} less than current pizza {self.pizza_count}, ignore')
        else:
            self.get_logger().info(f'Set max pizza from {self.pizza_limit} to {msg.data}')
            self.pizza_limit = msg.data

    def turtle_eat(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)
        self.get_logger().info(f'Eat pizza: {self.pizza_count} from {self.pizza_limit}')

    def spawn_pizza(self, x, y):
        if self.total_spawned <= self.pizza_limit:
            position_request = GivePosition.Request()
            position_request.x = x
            position_request.y = y
            self.spawn_pizza_client.call_async(position_request)
            self.get_logger().info(f'Spawned pizza: {self.total_spawned} from {self.pizza_limit}')
            self.total_spawned += 1

    def goal_pose_callback(self, msg:PoseStamped):
        self.target_position[0] = msg.pose.position.x + 5.4
        self.target_position[1] = msg.pose.position.y + 5.4
        self.target_position_queue = np.append(self.target_position_queue, self.target_position)
        self.spawn_pizza(msg.pose.position.x + 5.4, msg.pose.position.y + 5.4)

    def pizza_count_callback(self, msg:Int64):
        self.pizza_count = msg.data
        if self.pizza_count >= self.pizza_limit:
            self.eat_all = True

    def mouse_position_callback(self, msg:Point):
        self.target_position[0] = msg.x
        self.target_position[1] = msg.y
        self.target_position_queue = np.append(self.target_position_queue, self.target_position)
        self.spawn_pizza(msg.x, msg.y)

    def pose_callback(self, msg:Pose):
        self.turtle_pose[0] = msg.x
        self.turtle_pose[1] = msg.y
        self.turtle_pose[2] = msg.theta

    def timer_callback(self):
        msg = Twist()
        if self.eat_all:
            dx = self.target_position[0] - self.turtle_pose[0]
            dy = self.target_position[1] - self.turtle_pose[1]
        else:
            dx = self.target_position_queue[0] - self.turtle_pose[0]
            dy = self.target_position_queue[1] - self.turtle_pose[1]

        alpha = np.arctan2(dy, dx)
        e_dis = np.sqrt(np.power(dx, 2) + np.power(dy, 2))
        self.ei_dis += e_dis*0.01
        e_ang = alpha - self.turtle_pose[2]
        e_ang = np.arctan2(np.sin(e_ang), np.cos(e_ang))
        self.ei_ang += e_ang*0.01
        msg.linear.x = 0.5 * e_dis + 0.02 * self.ei_dis
        msg.angular.z = 5.5 * e_ang + 0.05 * self.ei_ang
        
        if e_dis < 0.8:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if len(self.target_position_queue) > 2:
                self.target_position_queue = np.delete(self.target_position_queue, [0,1])
                self.turtle_eat()
            elif not self.eat_all:
                self.turtle_eat()

        self.cmd_vel_pub.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = EaterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
