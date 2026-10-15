from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(
        get_package_share_directory("robotx_mission_state_machine")
    )
    config_file = package_share / "config" / "mission_state_machine.yaml"

    return LaunchDescription(
        [
            Node(
                package="robotx_mission_state_machine",
                executable="mission_planner",
                name="mission_planner",
                output="screen",
                parameters=[str(config_file)],
            )
        ]
    )
