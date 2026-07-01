import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = LaunchConfiguration("scan_topic")
    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    laser_frame = LaunchConfiguration("laser_frame")

    pkg_share = get_package_share_directory("urdf_odom_pkg")
    slam_params_file = os.path.join(
        pkg_share,
        "config",
        "slam_config.yaml",
    )

    base_to_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar_tf",
        arguments=[
            "-0.029563", "0.335426", "0.216507",
            "0", "0", "0",
            base_frame, laser_frame,
        ],
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    lidar_odom_node = Node(
        package="ros2_laser_scan_matcher",
        executable="laser_scan_matcher",
        name="laser_scan_matcher",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "publish_odom": "/odom",
            "publish_tf": True,
            "base_frame": base_frame,
            "odom_frame": odom_frame,
            "laser_frame": laser_frame,
        }],
        remappings=[
            ("/scan", scan_topic),
        ],
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {
                "use_sim_time": use_sim_time,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "scan_topic": scan_topic,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("laser_frame", default_value="laser_frame"),
        base_to_lidar_tf,
        lidar_odom_node,
        slam_toolbox_node,
    ])
