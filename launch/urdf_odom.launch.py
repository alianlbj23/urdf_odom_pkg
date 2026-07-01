from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    reverse_left = LaunchConfiguration("reverse_left")
    reverse_right = LaunchConfiguration("reverse_right")

    base_to_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar_tf",
        arguments=[
            "-0.029563", "0.335426", "0.216507",
            "0", "0", "0",
            "base_link", "laser_frame",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    wheel_odom_node = Node(
        package="urdf_odom_pkg",
        executable="wheel_odom_node",
        name="wheel_odom_node",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "wheel_radius": ParameterValue(wheel_radius, value_type=float),
            "wheel_separation": ParameterValue(wheel_separation, value_type=float),
            "reverse_left": ParameterValue(reverse_left, value_type=bool),
            "reverse_right": ParameterValue(reverse_right, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("wheel_radius", default_value="0.050350"),
        DeclareLaunchArgument("wheel_separation", default_value="0.210000"),
        DeclareLaunchArgument("reverse_left", default_value="false"),
        DeclareLaunchArgument("reverse_right", default_value="false"),
        base_to_lidar_tf,
        wheel_odom_node,
    ])
