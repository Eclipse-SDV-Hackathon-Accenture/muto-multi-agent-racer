#
#  Copyright (c) 2023 Composiv.ai, Eteration A.S. and others
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v10.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai, Eteration A.S. - initial API and implementation
#

import os
import yaml
import json
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from composer.stack import Stack
from composer.twin import Twin
from composer.edge_device import EdgeDevice
from muto_msgs.srv import ComposePlugin
from muto_msgs.msg import PluginResponse, StackManifest, PlanManifest


class MutoDefaultComposePlugin(Node):
    def __init__(self):
        super().__init__("compose_plugin")

        self.declare_parameter("name", "example-01")
        self.declare_parameter("namespace", "org.eclipse.muto.sandbox")
        self.declare_parameter("stack_topic", "stack")
        self.declare_parameter("twin_topic", "twin")
        self.declare_parameter("anonymous", False)
        self.declare_parameter(
            "twin_url", "http://ditto:ditto@sandbox.composiv.ai")

        self.name = self.get_parameter("name").value
        self.namespace = self.get_parameter("namespace").value
        self.stack_topic = self.get_parameter("stack_topic").value
        self.twin_topic = self.get_parameter("twin_topic").value
        self.twin_url = self.get_parameter("twin_url").value
        self.anonymous = self.get_parameter("anonymous").value

        self.muto = {
            "name": self.name,
            "namespace": self.namespace,
            "stack_topic": self.stack_topic,
            "twin_topic": self.twin_topic,
            "twin_url": self.twin_url,
            "anonymous": self.anonymous
        }

        self.srv = self.create_service(
            ComposePlugin, "muto_compose", self.handle_compose
        )

        self.twin = Twin(node='muto_compose_plugin',
                         config=self.muto, publisher=None)
        self.edge_device = EdgeDevice(twin=self.twin)

    def handle_compose(self, req, res):
        plan = req.input

        st = json.loads(plan.current.stack)
        current_stack = Stack(self.edge_device, st, None)
        st = json.loads(plan.next.stack)

        if st.get('stackId', None) is not None:
            manifest = self.twin.stack(st['stackId'])
            nextStack = Stack(self.edge_device, manifest, None)
        else:
            nextStack = Stack(self.edge_device, st, None)

        merged = current_stack.merge(nextStack)

        res.output = PlanManifest(
            current=plan.current,
            next=plan.next,
            pipeline=plan.pipeline,
            planned=StackManifest(
                type="json",
                stack=json.dumps(merged.manifest)
            ),
            result=PluginResponse(
                result_code=0, error_message="", error_description=""
            )
        )
        return res


def main(args=None):
    rclpy.init(args=args)
    node = MutoDefaultComposePlugin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
