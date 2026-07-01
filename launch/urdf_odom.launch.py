from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

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
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        base_to_lidar_tf,
        wheel_odom_node,
    ])
