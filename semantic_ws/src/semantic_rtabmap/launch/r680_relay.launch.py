from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="semantic_rtabmap",
            executable="ros2_relay_node",
            name="r680_ros2_relay",
            parameters=[os.path.join(get_package_share_directory("semantic_rtabmap"), "config", "r680_relay.yaml")],
            output="screen",
        ),
    ])
