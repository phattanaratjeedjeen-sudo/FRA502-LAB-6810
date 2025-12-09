#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from std_msgs.msg import Int64
from controller_interfaces.srv import SetParam, SetMaxPizza
import math


class KillerNode(Node):
    def __init__(self):
        super().__init__('killer_node')
        self.pub_cmdvel = self.create_publisher(Twist, 'cmd_vel', 10) 
        self.create_subscription(Pose, 'pose', self.pose_callback, 10)

        self.create_subscription(Pose, '/pose', self.target_callback, 10)
        self.create_subscription(Int64, '/pizza_count', self.pizza_count_callback, 10)

        self.create_service(SetParam, 'set_controller_param', self.set_gain_callback)
        self.create_service(SetMaxPizza, '/set_max_pizza', self.set_max_pizza_callback)

        self.eat_pizza_client = self.create_client(Kill, '/remove_turtle')
        
        self.declare_parameter('sampling_frequency', 100.0)
        self.declare_parameter('kill_target', 'turtle_eater')
        self.sampling_frequency = self.get_parameter('sampling_frequency').get_parameter_value().double_value
        self.kill_target = self.get_parameter('kill_target').get_parameter_value().string_value

        self.kp_linear = 0.5
        self.kp_angular = 1.0

        self.current_target = None
        self.current_pose = [0.0, 0.0, 0.0]
        self.controller_enable = False
        self.pizza_cnt = 0
        self.max_pizza = 5

        self.timer = self.create_timer(1/self.sampling_frequency, self.timer_callback)
        self.get_logger().info(f'Run killer node with default gains kp_linear={self.kp_linear}, kp_angular={self.kp_angular}, sampling_frequency={self.sampling_frequency}Hz.')

    def set_gain_callback(self, request: SetParam.Request, response: SetParam.Response):
        self.kp_linear = request.kp_linear.data
        self.kp_angular = request.kp_angular.data
        return response

    def target_callback(self, msg: Pose):
        if self.pizza_cnt == self.max_pizza:
            self.current_target = [msg.x, msg.y]
            self.controller_enable = True
        else:
            self.controller_enable = False
            self.cmd_vel(0.0, 0.0)

    def pose_callback(self, msg: Pose):
        self.current_pose[0] = msg.x
        self.current_pose[1] = msg.y
        self.current_pose[2] = msg.theta

    def pizza_count_callback(self, msg: Int64):
        self.pizza_cnt = msg.data

    def set_max_pizza_callback(self, request: SetMaxPizza.Request, response: SetMaxPizza.Response):
        if request.max_pizza.data > self.max_pizza:
            self.max_pizza = request.max_pizza.data
            response.log.data = "success"
        else:
            response.log.data = "failed"
        return response

    def kill_turtle(self, name : str):
        kill_request = Kill.Request()
        kill_request.name = name
        self.eat_pizza_client.call_async(kill_request)

    def cmd_vel(self, vx, w):
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = w
        self.pub_cmdvel.publish(cmd_vel)

    def timer_callback(self):
        if self.controller_enable:
            dx = self.current_target[0] - self.current_pose[0]
            dy = self.current_target[1] - self.current_pose[1]

            e_dis = math.hypot(dx, dy)
            e_ori = math.atan2(dy, dx) - self.current_pose[2]
            e_ori = math.atan2(math.sin(e_ori), math.cos(e_ori))

            u_dis = self.kp_linear * e_dis
            u_ori = self.kp_angular * e_ori

            if e_dis < 0.5:
                self.cmd_vel(0.0, 0.0)
                self.kill_turtle(self.kill_target)
                self.controller_enable = False
            else:
                self.cmd_vel(u_dis, u_ori)

def main(args=None):
    rclpy.init(args=args)
    node = KillerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
