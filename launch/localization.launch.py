import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("urdf_odom_pkg")
    ekf_config = os.path.join(package_share, "config", "ekf.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    wheel_twist_topic = LaunchConfiguration("wheel_twist_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    base_frame = LaunchConfiguration("base_frame")
    left_front_joint = LaunchConfiguration("left_front_joint")
    left_rear_joint = LaunchConfiguration("left_rear_joint")
    right_front_joint = LaunchConfiguration("right_front_joint")
    right_rear_joint = LaunchConfiguration("right_rear_joint")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    reverse_left = LaunchConfiguration("reverse_left")
    reverse_right = LaunchConfiguration("reverse_right")

    wheel_twist_node = Node(
        package="urdf_odom_pkg",
        executable="wheel_twist_node",
        name="wheel_twist_node",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "joint_states_topic": joint_states_topic,
            "wheel_twist_topic": wheel_twist_topic,
            "base_frame": base_frame,
            "left_front_joint": left_front_joint,
            "left_rear_joint": left_rear_joint,
            "right_front_joint": right_front_joint,
            "right_rear_joint": right_rear_joint,
            "wheel_radius": ParameterValue(wheel_radius, value_type=float),
            "wheel_separation": ParameterValue(
                wheel_separation,
                value_type=float,
            ),
            "reverse_left": ParameterValue(reverse_left, value_type=bool),
            "reverse_right": ParameterValue(reverse_right, value_type=bool),
        }],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[
            ("/wheel_twist", wheel_twist_topic),
            ("/imu/data", imu_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/joint_states",
        ),
        DeclareLaunchArgument(
            "wheel_twist_topic",
            default_value="/wheel_twist",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/imu",
            description=(
                "Actual Isaac Sim IMU topic; override this because no IMU "
                "topic is defined in the current workspace"
            ),
        ),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("left_front_joint", default_value="Revolute_35"),
        DeclareLaunchArgument("left_rear_joint", default_value=""),
        DeclareLaunchArgument("right_front_joint", default_value="Revolute_38"),
        DeclareLaunchArgument("right_rear_joint", default_value=""),
        DeclareLaunchArgument("wheel_radius", default_value="0.050350"),
        DeclareLaunchArgument("wheel_separation", default_value="0.210000"),
        DeclareLaunchArgument("reverse_left", default_value="false"),
        DeclareLaunchArgument("reverse_right", default_value="false"),
        wheel_twist_node,
        ekf_node,
    ])
