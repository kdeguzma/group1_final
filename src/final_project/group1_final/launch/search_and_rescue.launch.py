import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("group1_final")
    map_file = os.path.join(pkg_share, "maps", "final_project_map.yaml")
    nav2_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    mission_params = os.path.join(pkg_share, "config", "mission_params.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "nav2.rviz")

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz with the package's nav2 view.",
    )
    
    tick_rate_arg = DeclareLaunchArgument(
        "tick_rate_hz",
        default_value="2.0",
        description="Tick Rate of Behavior Tree in Hz."        
        )
    

    # --- Nav2 bringup, adapted from map_nav.launch.py ---
    # Pass launch_arguments as a list of tuples (NOT dict.items())
    # so each value's static type is narrowed independently.
    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_launch = os.path.join(nav2_share, "launch", "bringup_launch.py")    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments=[
            ("map", map_file),
            ("params_file", nav2_params),
            ("use_sim_time", "true"),
            ("autostart", "true"),
        ],
    )

    # --- behavior tree entry point ---
    bt_node = Node(
        package="group1_final",
        executable="search_and_rescue_exe",  # registered in setup.py
        name="search_and_rescue",
        output="screen",
        emulate_tty=True,
        parameters=[mission_params,{"tick_rate_hz":
            LaunchConfiguration("tick_rate_hz")}],
    )

    # --- simulated service servers ---
    detect_server = Node(
        package="group1_final",
        executable="detect_survivor_server_exe",
        name="detect_survivor_server",
        output="screen",
        emulate_tty=True,
    )
    report_server = Node(
        package="group1_final",
        executable="report_survivor_server_exe",
        name="report_survivor_server",
        output="screen",
        emulate_tty=True,
    )

    # --- RViz (optional, gated on a launch arg) ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],  # gated here
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    

    return LaunchDescription(
        [
            rviz_arg,
            tick_rate_arg,
            nav2_bringup,
            rviz_node,
            detect_server,
            report_server,            
            bt_node,
        ]
    )
