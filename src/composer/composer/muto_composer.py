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
import json
import yaml
import rclpy
from std_msgs.msg import String
from muto_msgs.msg import MutoAction
from composer.twin import Twin
from composer.router import Router
from composer.pipeline import Pipeline
from composer.edge_device import EdgeDevice
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class MutoComposer(Node):
    def __init__(self):
        super().__init__("muto_composer")
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

        pipeline_file_path = os.path.join(
            get_package_share_directory("composer"), "config", "pipeline.yaml"
        )

        with open(pipeline_file_path, "r") as f:
            pipelines = yaml.safe_load(f)

        self.pipelines = pipelines
        self.edge_device = None

        self.twin_publisher = self.create_publisher(
            String,
            self.twin_topic,
            10,
        )

        self.create_subscription(
            MutoAction,
            self.stack_topic,
            self.on_stack_callback,
            10,
        )

        self.bootstrap(self.muto)

    def bootstrap(self, context):
        try:
            if self.edge_device == None:
                self.twin = Twin(node='muto_composer',
                                      config=context, publisher=self.twin_publisher)
                self.edge_device = EdgeDevice(twin=self.twin)
                pipelinesSpec = self.pipelines["pipelines"]
                for pipelineItem in pipelinesSpec:
                    pipeline = Pipeline(
                        self.edge_device, pipelineItem["name"], pipelineItem["pipeline"], pipelineItem.get("compensation", None))
                    self.pipelines[pipelineItem["name"]] = pipeline
                self.edge_device.bootstrap()
                self.router = Router(self.edge_device, self.pipelines)
        except Exception as e:
            print(f'An exception occured in muto_composer bootstrap: {e}')

    def on_stack_callback(self, msg):
        if msg:
            stack = json.loads(msg.payload)
            self.router.route(msg.method, stack)


def main(args=None):
    rclpy.init(args=args)
    muto_composer_node = MutoComposer()
    rclpy.spin(muto_composer_node)
    muto_composer_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
