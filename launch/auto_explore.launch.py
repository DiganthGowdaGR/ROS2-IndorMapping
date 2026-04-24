from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("indoor_bot")
    mapping_launch = PathJoinSubstitution([pkg_share, "launch", "cartographer_mapping.launch.py"])

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    configuration_basename = LaunchConfiguration("configuration_basename")
    map_resolution = LaunchConfiguration("map_resolution")
    rviz_config = LaunchConfiguration("rviz_config")
    require_motion_for_scan = LaunchConfiguration("require_motion_for_scan")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("world", default_value="ros2_indoor.world"),
            DeclareLaunchArgument("x_pose", default_value="0.0"),
            DeclareLaunchArgument("y_pose", default_value="0.0"),
            DeclareLaunchArgument("z_pose", default_value="0.05"),
            DeclareLaunchArgument("require_motion_for_scan", default_value="false"),
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
                    "Auto mode: ensure manual teleop and Nav2 goal navigation are not running "
                    "so explore_lite is the only /cmd_vel publisher."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch),
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
                    "require_motion_for_scan": require_motion_for_scan,
                }.items(),
            ),
            Node(
                package="explore_lite",
                executable="explore",
                name="explore_lite",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "costmap_topic": "/map",
                        "robot_base_frame": "robot_footprint",
                    }
                ],
                remappings=[("/cmd_vel", "/cmd_vel")],
            ),
        ]
    )
