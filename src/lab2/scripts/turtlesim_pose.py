#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from turtlesim.msg import Pose
from tf_transformations import quaternion_from_euler

class DummyNode(Node):
    def __init__(self):
        super().__init__('odom_pub')
        self.odom1_publisher = self.create_publisher(Odometry, '/odom1', 10)
        self.odom2_publisher = self.create_publisher(Odometry, '/odom2', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose1_callback, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.pose2_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.pub_static_tf()
        self.get_logger().info('odom_pub: run')


    def pub_static_tf(self):
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'frame0'
        t1.child_frame_id = 'odom1'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.w = 1.0

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'frame0'
        t2.child_frame_id = 'odom2'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform([t1, t2])


    def odom_tf_pub(self, msg, turtle_id, publisher):
        x = msg.x
        y = msg.y
        theta = msg.theta

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = f'odom{turtle_id}'
        odom_msg.child_frame_id = f'turtle{turtle_id}'

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        publisher.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = odom_msg.header.frame_id
        t.child_frame_id = odom_msg.child_frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


    def pose1_callback(self, msg):
        self.odom_tf_pub(msg, 1, self.odom1_publisher)


    def pose2_callback(self, msg):
        self.odom_tf_pub(msg, 2, self.odom2_publisher)


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
