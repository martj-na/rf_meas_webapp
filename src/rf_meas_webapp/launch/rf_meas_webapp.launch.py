"""
RF Measurements Manager + WebSocket Bridge launch file.

dotX Automation s.r.l. <info@dotxautomation.com>

March 2025
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # ------------------------------------------------------------
    # Configuration file default path
    # ------------------------------------------------------------
    config = os.path.join(
        get_package_share_directory("rf_meas_webapp"),
        "config",
        "rf_meas_webapp.yaml"
    )

    # ------------------------------------------------------------
    # Declare launch arguments
    # ------------------------------------------------------------
    ns = LaunchConfiguration("namespace")
    cf = LaunchConfiguration("cf")
    ws_port = LaunchConfiguration("ws_port")

    ld.add_action(DeclareLaunchArgument(
        "namespace",
        default_value=""
    ))

    ld.add_action(DeclareLaunchArgument(
        "cf",
        default_value=config
    ))

    ld.add_action(DeclareLaunchArgument(
        "ws_port",
        default_value="9090"
    ))

 

    # ------------------------------------------------------------
    # WebSocket Bridge: rosbridge_websocket
    # ------------------------------------------------------------
    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        namespace=ns,
        output="screen",
        emulate_tty=True,
        parameters=[{
            "port": ws_port
        }]
    )

    ld.add_action(rosbridge_node)

    return ld