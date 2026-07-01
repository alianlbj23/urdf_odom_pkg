import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__("wheel_odom_node")

        self.wheel_radius = 0.050350

        self.wheel_separation = 0.210000

        self.left_joints = [
            "Revolute_35",
            "Revolute_38",
        ]

        self.right_joints = [
            "Revolute_29",
            "Revolute_32",
        ]

        self.reverse_left = False
        self.reverse_right = False

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.wheel_velocity_pub = self.create_publisher(
            Float64MultiArray,
            "/wheel_joint_velocities",
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.get_logger().info("wheel_odom_node started")

    def get_average_velocity(self, msg, joint_names):
        velocities = []

        for joint_name in joint_names:
            if joint_name not in msg.name:
                self.get_logger().warn(f"Joint not found: {joint_name}")
                return None

            index = msg.name.index(joint_name)

            if index >= len(msg.velocity):
                self.get_logger().warn(f"No velocity for joint: {joint_name}")
                return None

            velocities.append(msg.velocity[index])

        return sum(velocities) / len(velocities)

    def joint_state_callback(self, msg):
        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0:
            return

        left_rad_s = self.get_average_velocity(msg, self.left_joints)
        right_rad_s = self.get_average_velocity(msg, self.right_joints)

        if left_rad_s is None or right_rad_s is None:
            return

        self.publish_wheel_velocities(msg, left_rad_s, right_rad_s)

        if self.reverse_left:
            left_rad_s *= -1.0

        if self.reverse_right:
            right_rad_s *= -1.0

        v_left = left_rad_s * self.wheel_radius
        v_right = right_rad_s * self.wheel_radius

        linear_v = (v_right + v_left) / 2.0
        angular_w = (v_right - v_left) / self.wheel_separation

        self.x += linear_v * math.cos(self.theta) * dt
        self.y += linear_v * math.sin(self.theta) * dt
        self.theta += angular_w * dt

        qx, qy, qz, qw = yaw_to_quaternion(self.theta)

        self.publish_odom(now, linear_v, angular_w, qx, qy, qz, qw)
        self.publish_tf(now, qx, qy, qz, qw)

    def publish_wheel_velocities(self, msg, left_average, right_average):
        velocity_msg = Float64MultiArray()
        velocity_msg.layout.dim = [
            MultiArrayDimension(
                label="left_0_left_1_right_0_right_1_left_avg_right_avg",
                size=6,
                stride=6,
            )
        ]

        left_velocities = [
            msg.velocity[msg.name.index(joint_name)]
            for joint_name in self.left_joints
        ]
        right_velocities = [
            msg.velocity[msg.name.index(joint_name)]
            for joint_name in self.right_joints
        ]

        velocity_msg.data = [
            left_velocities[0],
            left_velocities[1],
            right_velocities[0],
            right_velocities[1],
            left_average,
            right_average,
        ]
        self.wheel_velocity_pub.publish(velocity_msg)

    def publish_odom(self, now, linear_v, angular_w, qx, qy, qz, qw):
        odom = Odometry()

        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = linear_v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_w

        self.odom_pub.publish(odom)

    def publish_tf(self, now, qx, qy, qz, qw):
        tf_msg = TransformStamped()

        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = WheelOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
