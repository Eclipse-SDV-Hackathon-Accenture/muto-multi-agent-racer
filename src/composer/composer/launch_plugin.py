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

import json
import rclpy
import composer.twin as twin
import composer.edge_device as edge
from rclpy.node import Node
from std_msgs.msg import String
from muto_msgs.srv import ComposePlugin
from muto_msgs.msg import PluginResponse, PlanManifest


class MutoDefaultLaunchPlugin(Node):
    def __init__(self):
        super().__init__("launch_plugin")

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

        self.device = None

        self.twinPublisher = self.create_publisher(
            String,
            self.twin_topic,
            10,
        )

        self.srv_apply = self.create_service(
            ComposePlugin, "muto_apply_stack", self.handle_apply
        )
        self.srv_kill = self.create_service(
            ComposePlugin, "muto_kill_stack", self.handle_kill
        )
        self.srv_start = self.create_service(
            ComposePlugin, "muto_start_stack", self.handle_start
        )

        self.bootstrap()

    def bootstrap(self):
        if self.device is None:
            self.twin = twin.Twin(
                node='MutoLaunchPlugin', config=self.muto, publisher=None)
            self.device = edge.EdgeDevice(self.twin)

    def handle_apply(self, req, res):
        plan = req.input
        if plan.planned:
            stack = json.loads(plan.planned.stack)
            self.device.apply(stack)
            res.output = PlanManifest(
                result=PluginResponse(
                    result_code=0,
                    error_message="",
                    error_description=""), planned=plan.planned)
        else:
            res.output = PlanManifest(
                result=PluginResponse(
                    result_code=1000,
                    error_message="Could not handle launch",
                    error_description="Could not handle launch"))
        return res

    def handle_kill(self, req, res):
        plan = req.input
        if plan.planned:
            try:
                stack = json.loads(plan.planned.stack)
                self.device.kill(stack)
            except Exception as error:
                self.get_logger().error('Cannot parse stack: {}'.format(error))
            res.output = PlanManifest(
                result=PluginResponse(
                    result_code=0,
                    error_message="",
                    error_description=""),
                planned=plan.planned)
        else:
            res.output = PlanManifest(
                result=PluginResponse(
                    result_code=1000,
                    error_message="Could not handle launch",
                    error_description="Could not handle launch"))
        return res

    def handle_start(self, req, res):
        plan = req.input
        if plan.planned:
            stack = json.loads(plan.planned.stack)
            self.device.activate(stack)
            res.output = PlanManifest(
                result=PluginResponse(
                    result_code=0,
                    error_message="",
                    error_description=""),
                planned=plan.planned)
        else:
            res.output = PlanManifest(
                result=PluginResponse(
                    result_code=1000,
                    error_message="Could not handle launch",
                    error_description="Could not handle launch")
            )
        return res

    def handle_restart(self, req, res):
        plan = req.input
        if plan.planned:
            stack = json.loads(plan.planned.stack)
            self.device.restart(stack)

            res.output = PlanManifest(
                result=PluginResponse(
                    result_code=0,
                    error_message="",
                    error_description=""),
                plan=plan.planned)
        else:
            res.output = PlanManifest(
                result=PluginResponse(
                    result_code=1000,
                    error_message="Could not handle launch",
                    error_description="Could not handle launch"))
        return res


def main(args=None):
    rclpy.init(args=args)
    node = MutoDefaultLaunchPlugin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
