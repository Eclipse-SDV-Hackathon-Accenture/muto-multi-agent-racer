#!/usr/bin/env python3
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
#

import rclpy
from rclpy.node import Node
import json
from muto_msgs.msg import StackManifest, PlanManifest
from muto_msgs.srv import ComposePlugin

PLUGINS = {
    "ComposePlugin": ComposePlugin
}
INFO = 'INFO'
WARN = 'WARN'
ERROR = 'ERROR'


class Pipeline(Node):
    def __init__(self, device, name, steps, compensation=None):
        super().__init__(f"{name}_pipeline_node")
        self.device = device
        self.actions = {}
        self.actions[name] = steps
        self.compensation = compensation
        self._class_name = self.__class__.__name__

    def execute(self, command, current, next):
        print(f'Execute pipeline for command: {command}')
        cstack = StackManifest(type="json", stack=json.dumps(current))
        pstack = StackManifest(type="json", stack=json.dumps(next))
        plan = PlanManifest(current=cstack, next=pstack, pipeline=command)

        pipeline = self.actions[command]
        for items in pipeline:
            if items["sequence"]:
                for step in items["sequence"]:
                    try:
                        response = self.executeStep(plan, step)
                        if response.output.result.result_code == 0:
                            plan = response.output
                            print(f'Step passed: {step}')
                    except Exception as e:
                        print(f'Step failed: {step}, {e}')
                        if self.compensation is not None:
                            for step in self.compensation:
                                self.executeStep(plan, step)
                        return

    def executeStep(self, plan, step):
        cli = self.create_client(PLUGINS[step["plugin"]], step["service"])
        req = PLUGINS[step["plugin"]].Request()
        req.input = plan
        future = cli.call_async(req)

        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.output.result.result_code == 1000:
                print(f'Step failed: {step}')
            return response
        else:
            print(f'Service call failed: {future.exception()}')
