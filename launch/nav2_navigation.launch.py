from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("indoor_bot")
    nav2_launch = PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
    mapping_launch = PathJoinSubstitution([pkg_share, "launch", "cartographer_mapping.launch.py"])

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    start_mapping = LaunchConfiguration("start_mapping")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    configuration_basename = LaunchConfiguration("configuration_basename")
    map_resolution = LaunchConfiguration("map_resolution")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"]),
            ),
            DeclareLaunchArgument(
                "start_mapping",
                default_value="false",
                description=(
                    "If true, includes cartographer_mapping.launch.py. "
                    "Keep false when SLAM is already running to avoid duplicate map/TF publishers."
                ),
            ),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("world", default_value="ros2_indoor.world"),
            DeclareLaunchArgument("x_pose", default_value="0.0"),
            DeclareLaunchArgument("y_pose", default_value="0.0"),
            DeclareLaunchArgument("z_pose", default_value="0.05"),
            DeclareLaunchArgument(
                "configuration_basename",
                default_value="indoor_bot_lidar_slam_2d_gazebo.lua",
            ),
            DeclareLaunchArgument("map_resolution", default_value="0.03"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution([pkg_share, "rviz", "indoor_bot.rviz"]),
            ),
            LogInfo(
                msg=(
                    "Navigation mode: stop teleop and explore_lite first so Nav2 is the only "
                    "/cmd_vel publisher."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch),
                condition=IfCondition(start_mapping),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "gui": gui,
                    "rviz": rviz,
                    "world": world,
                    "x_pose": x_pose,
                    "y_pose": y_pose,
                    "z_pose": z_pose,
                    "configuration_basename": configuration_basename,
                    "map_resolution": map_resolution,
                    "rviz_config": rviz_config,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )
