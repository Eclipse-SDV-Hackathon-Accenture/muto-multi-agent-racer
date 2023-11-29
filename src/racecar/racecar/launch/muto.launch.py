#
#  Copyright (c) 2023 Composiv.ai
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# Licensed under the  Eclipse Public License v2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v20.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai - initial API and implementation
#
#

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

output = "screen"

def generate_launch_description():

    # Files
    current_dir = os.path.dirname(__file__)
    muto_config = os.path.join(current_dir, '..', 'config', 'muto.yaml')
    mutons = os.getenv("MUTONS")
    ld = LaunchDescription()

    try:
        # Nodes
        node_agent = Node(
            name="muto_agent",
            package="agent",
            namespace=mutons,
            executable="muto_agent",
            output=output,
            parameters=[muto_config]
        )

        node_mqtt_gateway = Node(
            name="mqtt_gateway",
            package="agent",
            namespace=mutons,
            executable="mqtt",
            output=output,
            parameters=[muto_config]
        )

        node_commands = Node(
            name="commands_plugin",
            package="agent",
            namespace=mutons,
            executable="commands",
            output=output,
            parameters=[muto_config]
        )

        node_twin = Node(
            name="core_twin",
            package="core",
            namespace=mutons,
            executable="twin",
            output=output,
            parameters=[muto_config]
        )

        node_composer = Node(
            package="composer",
            namespace=mutons,
            executable="muto_composer",
            output=output,
            parameters=[muto_config]
        )

        node_compose_plugin = Node(
            package="composer",
            namespace=mutons,
            executable="compose_plugin",
            output=output,
            parameters=[muto_config]
        )

        node_launch_plugin = Node(
            package="composer",
            namespace=mutons,
            executable="launch_plugin",
            output=output,
            parameters=[muto_config]
        )

        ld.add_action(node_agent)
        ld.add_action(node_mqtt_gateway)
        ld.add_action(node_commands)
        ld.add_action(node_twin)
        ld.add_action(node_composer)
        ld.add_action(node_compose_plugin)
        ld.add_action(node_launch_plugin)
    except Exception as e:
        print(f"Can't find muto {e}")

    return ld
