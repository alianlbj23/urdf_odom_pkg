import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    imu_topic = LaunchConfiguration("imu_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    left_front_joint = LaunchConfiguration("left_front_joint")
    left_rear_joint = LaunchConfiguration("left_rear_joint")
    right_front_joint = LaunchConfiguration("right_front_joint")
    right_rear_joint = LaunchConfiguration("right_rear_joint")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    reverse_left = LaunchConfiguration("reverse_left")
    reverse_right = LaunchConfiguration("reverse_right")
    pkg_share = get_package_share_directory("urdf_odom_pkg")

    slam_params_file = os.path.join(
        pkg_share,
        "config",
        "slam_config.yaml",
    )
    localization_launch_file = os.path.join(
        pkg_share,
        "launch",
        "localization.launch.py",
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "imu_topic": imu_topic,
            "left_front_joint": left_front_joint,
            "left_rear_joint": left_rear_joint,
            "right_front_joint": right_front_joint,
            "right_rear_joint": right_rear_joint,
            "wheel_radius": wheel_radius,
            "wheel_separation": wheel_separation,
            "reverse_left": reverse_left,
            "reverse_right": reverse_right,
        }.items(),
    )

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
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    base_to_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_tf",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "base_link", "sim_imu",
        ],
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "scan_topic": scan_topic,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("imu_topic", default_value="/imu"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("left_front_joint", default_value="Revolute_35"),
        DeclareLaunchArgument("left_rear_joint", default_value=""),
        DeclareLaunchArgument("right_front_joint", default_value="Revolute_38"),
        DeclareLaunchArgument("right_rear_joint", default_value=""),
        DeclareLaunchArgument("wheel_radius", default_value="0.050350"),
        DeclareLaunchArgument("wheel_separation", default_value="0.210000"),
        DeclareLaunchArgument("reverse_left", default_value="false"),
        DeclareLaunchArgument("reverse_right", default_value="false"),
        localization_launch,
        base_to_lidar_tf,
        base_to_imu_tf,
        slam_toolbox_node,
    ])
