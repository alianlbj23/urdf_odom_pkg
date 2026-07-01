import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg_share = get_package_share_directory("urdf_odom_pkg")

    slam_params_file = os.path.join(
        pkg_share,
        "config",
        "slam_config.yaml",
    )
    odom_launch_file = os.path.join(
        pkg_share,
        "launch",
        "urdf_odom.launch.py",
    )

    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odom_launch_file),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        odom_launch,
        slam_toolbox_node,
    ])
