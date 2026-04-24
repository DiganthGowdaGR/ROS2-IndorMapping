from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("indoor_bot")
    nav2_bringup_share = FindPackageShare("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")
    map_resolution = LaunchConfiguration("map_resolution")
    configuration_basename = LaunchConfiguration("configuration_basename")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    auto_save_map = LaunchConfiguration("auto_save_map")
    save_map_url = LaunchConfiguration("save_map_url")
    nav2_start_delay = LaunchConfiguration("nav2_start_delay")

    cartographer_launch = PathJoinSubstitution([pkg_share, "launch", "cartographer_mapping.launch.py"])
    navigation_launch = PathJoinSubstitution([nav2_bringup_share, "launch", "navigation_launch.py"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "indoor_bot_autonomous.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("world", default_value="ros2_indoor.world"),
            DeclareLaunchArgument("map_resolution", default_value="0.03"),
            DeclareLaunchArgument(
                "configuration_basename",
                default_value="indoor_bot_lidar_slam_2d_gazebo.lua",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution([pkg_share, "config", "nav2_autonomous_params.yaml"]),
            ),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("auto_save_map", default_value="true"),
            DeclareLaunchArgument("nav2_start_delay", default_value="12.0"),
            DeclareLaunchArgument(
                "save_map_url",
                default_value="/home/sharath/indoor_bot_ws/auto_explore_map",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cartographer_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "gui": gui,
                    "rviz": "false",
                    "world": world,
                    "configuration_basename": configuration_basename,
                    "map_resolution": map_resolution,
                }.items(),
            ),
            TimerAction(
                period=nav2_start_delay,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(navigation_launch),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "autostart": autostart,
                            "params_file": params_file,
                            "use_composition": "False",
                            "use_respawn": "False",
                            "container_name": "nav2_container",
                            "log_level": "info",
                        }.items(),
                    ),
                    Node(
                        package="nav2_map_server",
                        executable="map_saver_server",
                        name="map_saver",
                        output="screen",
                        parameters=[
                            params_file,
                            {
                                "use_sim_time": use_sim_time,
                            },
                        ],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_map_saver",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "autostart": autostart,
                                "node_names": ["map_saver"],
                            }
                        ],
                    ),
                    Node(
                        package="indoor_bot",
                        executable="frontier_explorer",
                        name="frontier_explorer",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "auto_save_map": auto_save_map,
                                "save_map_url": save_map_url,
                            }
                        ],
                    ),
                    Node(
                        condition=IfCondition(rviz),
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="screen",
                        parameters=[{"use_sim_time": use_sim_time}],
                        arguments=["-d", rviz_config],
                    ),
                ],
            ),
        ]
    )
