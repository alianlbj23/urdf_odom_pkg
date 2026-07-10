import math

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class WheelTwistNode(Node):
    """Convert measured wheel angular velocities into a planar body twist."""

    def __init__(self):
        super().__init__("wheel_twist_node")

        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("wheel_twist_topic", "/wheel_twist")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("left_front_joint", "Revolute_35")
        self.declare_parameter("left_rear_joint", "")
        self.declare_parameter("right_front_joint", "Revolute_38")
        self.declare_parameter("right_rear_joint", "")
        self.declare_parameter("wheel_radius", 0.050350)
        self.declare_parameter("wheel_separation", 0.210000)
        self.declare_parameter("reverse_left", False)
        self.declare_parameter("reverse_right", False)
        self.declare_parameter("linear_x_variance", 0.0025)
        self.declare_parameter("angular_z_variance", 0.25)

        self.joint_states_topic = self.get_parameter(
            "joint_states_topic"
        ).value
        wheel_twist_topic = self.get_parameter("wheel_twist_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.left_joints = self.get_joint_group(
            "left_front_joint",
            "left_rear_joint",
        )
        self.right_joints = self.get_joint_group(
            "right_front_joint",
            "right_rear_joint",
        )
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(
            self.get_parameter("wheel_separation").value
        )
        self.reverse_left = bool(self.get_parameter("reverse_left").value)
        self.reverse_right = bool(self.get_parameter("reverse_right").value)
        self.linear_x_variance = float(
            self.get_parameter("linear_x_variance").value
        )
        self.angular_z_variance = float(
            self.get_parameter("angular_z_variance").value
        )

        if self.wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be greater than zero")
        if self.wheel_separation <= 0.0:
            raise ValueError("wheel_separation must be greater than zero")
        if not self.left_joints:
            raise ValueError("at least one left wheel joint must be configured")
        if not self.right_joints:
            raise ValueError("at least one right wheel joint must be configured")
        if self.linear_x_variance < 0.0 or self.angular_z_variance < 0.0:
            raise ValueError("twist covariance values cannot be negative")

        self.publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            wheel_twist_topic,
            10,
        )
        self.subscription = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_callback,
            qos_profile_sensor_data,
        )
        self.last_warning_ns = 0

        self.get_logger().info(
            f"Converting measured wheel velocities from "
            f"{self.joint_states_topic} to {wheel_twist_topic}"
        )
        self.get_logger().info(
            f"wheel_radius={self.wheel_radius:.6f}, "
            f"wheel_separation={self.wheel_separation:.6f}, "
            f"left_joints={self.left_joints}, right_joints={self.right_joints}, "
            f"reverse_left={self.reverse_left}, reverse_right={self.reverse_right}"
        )

    def get_joint_group(self, front_parameter, rear_parameter):
        joints = (
            self.get_parameter(front_parameter).value,
            self.get_parameter(rear_parameter).value,
        )
        return tuple(joint for joint in joints if joint)

    def warn_throttled(self, message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warning_ns >= 5_000_000_000:
            self.get_logger().warn(message)
            self.last_warning_ns = now_ns

    def get_joint_velocity(self, msg, joint_name, name_to_index):
        index = name_to_index.get(joint_name)
        if index is None:
            self.warn_throttled(
                f"Joint '{joint_name}' is missing from {self.joint_states_topic}"
            )
            return None
        if index >= len(msg.velocity):
            self.warn_throttled(
                f"Joint '{joint_name}' has no velocity value "
                f"(name size={len(msg.name)}, velocity size={len(msg.velocity)})"
            )
            return None

        velocity = float(msg.velocity[index])
        if not math.isfinite(velocity):
            self.warn_throttled(
                f"Joint '{joint_name}' has a non-finite velocity"
            )
            return None
        return velocity

    def joint_state_callback(self, msg):
        name_to_index = {name: index for index, name in enumerate(msg.name)}
        required_joints = self.left_joints + self.right_joints
        velocities = {
            name: self.get_joint_velocity(msg, name, name_to_index)
            for name in required_joints
        }
        if any(value is None for value in velocities.values()):
            return

        left_rad_s = sum(velocities[name] for name in self.left_joints)
        left_rad_s /= len(self.left_joints)
        right_rad_s = sum(
            velocities[name] for name in self.right_joints
        )
        right_rad_s /= len(self.right_joints)

        if self.reverse_left:
            left_rad_s *= -1.0
        if self.reverse_right:
            right_rad_s *= -1.0

        v_left = left_rad_s * self.wheel_radius
        v_right = right_rad_s * self.wheel_radius

        output = TwistWithCovarianceStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.base_frame
        output.twist.twist.linear.x = (v_left + v_right) / 2.0
        output.twist.twist.angular.z = (
            v_right - v_left
        ) / self.wheel_separation

        # Twist covariance order is [vx, vy, vz, vroll, vpitch, vyaw].
        # Unmeasured axes get large variances. Wheel yaw is deliberately noisy
        # for a skid-steer platform and is not fused by the initial EKF config.
        covariance = [0.0] * 36
        covariance[0] = self.linear_x_variance
        covariance[7] = 1.0e3
        covariance[14] = 1.0e3
        covariance[21] = 1.0e3
        covariance[28] = 1.0e3
        covariance[35] = self.angular_z_variance
        output.twist.covariance = covariance

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelTwistNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
