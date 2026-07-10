import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def save_map_action(context, *args, **kwargs):
    map_name = LaunchConfiguration("map_name").perform(context)
    map_dir = LaunchConfiguration("map_dir").perform(context)

    if map_dir == "":
        pkg_share = get_package_share_directory("urdf_odom_pkg")
        map_dir = os.path.join(pkg_share, "maps")

    os.makedirs(map_dir, exist_ok=True)

    map_path = os.path.join(map_dir, map_name)

    return [
        ExecuteProcess(
            cmd=[
                "ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", map_path,
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_name",
            default_value="v6_map",
            description="Map file name without .yaml or .pgm",
        ),
        DeclareLaunchArgument(
            "map_dir",
            default_value="",
            description="Directory to save map. Empty means package share maps directory.",
        ),
        OpaqueFunction(function=save_map_action),
    ])