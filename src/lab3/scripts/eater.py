#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from std_msgs.msg import Int64
from controller_interfaces.srv import SetMaxPizza, SetParam
import math


class EaterNode(Node):
    def __init__(self):
        super().__init__('eater_node')

        self.pub_cmdvel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_subscription(Int64, '/turtle1/pizza_count', self.eat_pizza_count_callback, 10)
        self.create_subscription(Point, '/mouse_position', self.mouse_position_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.rviz_position_callback, 10)

        self.create_service(SetParam, '/set_param', self.set_gain_callback)
        self.create_service(SetMaxPizza, '/set_max_pizza', self.set_max_pizza_callback)

        self.spawn_pizza_client = self.create_client(GivePosition, '/spawn_pizza')
        self.eat_pizza_client = self.create_client(Empty, '/turtle1/eat')

        self.declare_parameter('rate', 100.0)
        self.rate = self.get_parameter('rate').get_parameter_value().double_value

        self.max_pizza = 5
        self.pizza_cnt = 0
        self.target_queue = []

        self.kp_linear = 0.5
        self.kp_angular = 2.0

        self.current_target = None
        self.current_pose = [0.0, 0.0, 0.0]
        self.controller_enable = False
        self.is_eat_all = False

        self.create_timer(1/self.rate, self.timer_callback)
        self.get_logger().info(f'Run eater node with default gains kp_linear={self.kp_linear}, kp_angular={self.kp_angular}, rate={self.rate}Hz.')

    def set_gain_callback(self, request: SetParam.Request, response: SetParam.Response):
        self.kp_linear = request.kp_linear.data
        self.kp_angular = request.kp_angular.data
        return response

    def spawn_pizza(self, position):
        position_request = GivePosition.Request()
        position_request.x = position[0]
        position_request.y = position[1]
        self.spawn_pizza_client.call_async(position_request)

    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)

    def eat_pizza_count_callback(self, msg: Int64):
        self.is_eat_all = msg.data == self.max_pizza

    def cmd_vel(self, vx, w):
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = w
        self.pub_cmdvel.publish(cmd_vel)

    def mouse_position_callback(self, msg: Point):
        point = [msg.x, msg.y]
        if self.pizza_cnt < self.max_pizza:
            self.target_queue.append(point)
            self.pizza_cnt += 1
            self.spawn_pizza(point)
        elif self.is_eat_all:
            self.target_queue.append(point)
            self.controller_enable = False
            while len(self.target_queue) > 1:
                self.target_queue.pop(0)
        self.get_logger().info(f'Mouse Position: x={msg.x}, y={msg.y}')

    def rviz_position_callback(self, msg: PoseStamped):
        point = [msg.pose.position.x + 5.40, msg.pose.position.y + 5.38]
        if self.pizza_cnt < self.max_pizza:
            self.target_queue.append(point)
            self.pizza_cnt += 1
            self.spawn_pizza(point)
        elif self.is_eat_all:
            self.target_queue.append(point)
            self.controller_enable = False
            while len(self.target_queue) > 1:
                self.target_queue.pop(0)
        self.get_logger().info(f'RViz Goal Position: x={msg.pose.position.x + 5.40}, y={msg.pose.position.y + 5.38}')

    def set_max_pizza_callback(self, request: SetMaxPizza.Request, response: SetMaxPizza.Response):
        if request.max_pizza.data > self.max_pizza:
            self.max_pizza = request.max_pizza.data
            response.log.data = "success"
        else:
            response.log.data = "failed"
        return response
    
    def pose_callback(self, msg: Pose):
        self.current_pose[0] = msg.x
        self.current_pose[1] = msg.y
        self.current_pose[2] = msg.theta

    def timer_callback(self):
        if len(self.target_queue) > 0 and not self.controller_enable:
            self.current_target = self.target_queue.pop(0)
            self.controller_enable = True

        if self.controller_enable:
            dx = self.current_target[0] - self.current_pose[0]
            dy = self.current_target[1] - self.current_pose[1]

            e_dis = math.hypot(dx, dy)
            e_ori = math.atan2(dy, dx) - self.current_pose[2]
            e_ori = math.atan2(math.sin(e_ori), math.cos(e_ori))

            u_dis = self.kp_linear * e_dis
            u_ori = self.kp_angular * e_ori

            if (abs(dx) < 0.1 and abs(dy) < 0.1):
                self.cmd_vel(0.0, 0.0)
                if not self.is_eat_all:
                    self.eat_pizza()
                self.controller_enable = False
            else:
                self.cmd_vel(u_dis, u_ori)

def main(args=None):
    rclpy.init(args=args)
    node = EaterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
