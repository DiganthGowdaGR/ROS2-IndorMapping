# ============================================================
# ROS2 LAUNCH FILE - CARTOGRAPHER MAPPING (MANUAL MODE)
# Project: Indoor Mapping Robot
# Description:
# Launches Gazebo simulation, robot model, SLAM (Cartographer),
# scan gating system, and optional RViz visualization.
# ============================================================

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ========================================================
    # PACKAGE PATHS
    # ========================================================
    pkg_share = FindPackageShare("indoor_bot")

    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("gazebo_ros"),
        "launch",
        "gazebo.launch.py"
    ])

    # ========================================================
    # LAUNCH ARGUMENTS (USER CONFIGURABLE)
    # ========================================================
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")

    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    yaw = LaunchConfiguration("yaw")

    require_motion_for_scan = LaunchConfiguration("require_motion_for_scan")

    configuration_basename = LaunchConfiguration("configuration_basename")
    map_resolution = LaunchConfiguration("map_resolution")
    rviz_config = LaunchConfiguration("rviz_config")

    # ========================================================
    # FILE PATHS
    # ========================================================
    world_path = PathJoinSubstitution([pkg_share, "worlds", world])
    xacro_path = PathJoinSubstitution([
        pkg_share, "models", "urdf", "indoor_bot_ros2.xacro"
    ])
    config_dir = PathJoinSubstitution([pkg_share, "config"])
    model_path = PathJoinSubstitution([pkg_share, "models"])

    # ========================================================
    # ROBOT DESCRIPTION (URDF via XACRO)
    # ========================================================
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_path]),
        value_type=str,
    )

    # Convert launch argument to boolean
    require_motion_for_scan_param = ParameterValue(
        require_motion_for_scan, value_type=bool
    )

    # ========================================================
    # LAUNCH DESCRIPTION
    # ========================================================
    return LaunchDescription([

        # ----------------------------------------------------
        # DECLARE ARGUMENTS
        # ----------------------------------------------------
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("world", default_value="ros2_indoor.world"),

        DeclareLaunchArgument("x_pose", default_value="-4.0"),
        DeclareLaunchArgument("y_pose", default_value="-3.8"),
        DeclareLaunchArgument("z_pose", default_value="0.05"),
        DeclareLaunchArgument("yaw", default_value="3.14"),

        DeclareLaunchArgument("require_motion_for_scan", default_value="true"),

        DeclareLaunchArgument(
            "configuration_basename",
            default_value="indoor_bot_lidar_slam_2d_gazebo.lua",
        ),

        DeclareLaunchArgument("map_resolution", default_value="0.03"),

        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                pkg_share, "rviz", "indoor_bot.rviz"
            ]),
        ),

        # ----------------------------------------------------
        # LOG MESSAGES (USER WARNINGS)
        # ----------------------------------------------------
        LogInfo(msg="Manual SLAM mode active. Use teleop for movement."),
        LogInfo(msg="Scan gate enabled: Map starts only after movement."),

        # ----------------------------------------------------
        # ENVIRONMENT VARIABLES (GAZEBO PATHS)
        # ----------------------------------------------------
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            [model_path, ":", EnvironmentVariable("GAZEBO_MODEL_PATH", default_value="")],
        ),
        SetEnvironmentVariable(
            "GAZEBO_RESOURCE_PATH",
            [pkg_share, ":", EnvironmentVariable("GAZEBO_RESOURCE_PATH", default_value="")],
        ),

        # ----------------------------------------------------
        # START GAZEBO SIMULATION
        # ----------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "gui": gui,
                "world": world_path,
                "verbose": "true",
            }.items(),
        ),

        # ----------------------------------------------------
        # ROBOT STATE PUBLISHER (TF)
        # ----------------------------------------------------
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }],
        ),

        # ----------------------------------------------------
        # SPAWN ROBOT IN GAZEBO
        # ----------------------------------------------------
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "indoor_bot",
                "-topic", "robot_description",
                "-x", x_pose,
                "-y", y_pose,
                "-z", z_pose,
                "-Y", yaw,
            ],
        ),

        # ----------------------------------------------------
        # SCAN MOTION GATE (YOUR CUSTOM LOGIC)
        # ----------------------------------------------------
        Node(
            package="indoor_bot",
            executable="scan_motion_gate",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_scan_topic": "/scan",
                "output_scan_topic": "/scan_for_slam",
                "odom_topic": "/odom",
                "cmd_vel_topic": "/cmd_vel",
                "require_motion_for_scan": require_motion_for_scan_param,
            }],
        ),

        # ----------------------------------------------------
        # CARTOGRAPHER SLAM NODE
        # ----------------------------------------------------
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-configuration_directory", config_dir,
                "-configuration_basename", configuration_basename,
            ],
            remappings=[
                ("scan", "scan_for_slam"),
            ],
        ),

        # ----------------------------------------------------
        # MAP GENERATION NODE
        # ----------------------------------------------------
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=["-resolution", map_resolution],
        ),

        # ----------------------------------------------------
        # RVIZ (OPTIONAL)
        # ----------------------------------------------------
        Node(
            condition=IfCondition(rviz),
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
        ),
    ])