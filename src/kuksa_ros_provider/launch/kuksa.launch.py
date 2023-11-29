import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    output = "screen"
    kuksa_config = os.path.join(get_package_share_directory(
        "kuksa_ros_provider"), "config", "kuksa_ros_provider.yaml")

    node_kuksa = Node(
        package="kuksa_ros_provider",
        executable="ros_provider",
        name="kuksa_ros_provider",
        output=output,
        parameters=[kuksa_config]
    )

    ld = LaunchDescription()
    ld.add_action(node_kuksa)
    return ld