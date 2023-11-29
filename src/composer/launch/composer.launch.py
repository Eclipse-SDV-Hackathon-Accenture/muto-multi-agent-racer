import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    output = "screen"
    composer_config = os.path.join(get_package_share_directory(
        "composer"), "config", "composer.yaml")

    node_composer = Node(
        package="composer",
        executable="muto_composer",
        output=output,
        parameters=[composer_config]
    )

    node_compose_plugin = Node(
        package="composer",
        executable="compose_plugin",
        output=output,
        parameters=[composer_config]
    )

    node_launch_plugin = Node(
        package="composer",
        executable="launch_plugin",
        output=output,
        parameters=[composer_config]
    )

    ld = LaunchDescription()
    ld.add_action(node_composer)
    ld.add_action(node_compose_plugin)
    ld.add_action(node_launch_plugin)
    return ld
