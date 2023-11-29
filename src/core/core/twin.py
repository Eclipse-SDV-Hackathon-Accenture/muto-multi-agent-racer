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

import rclpy
from rclpy.node import Node

from core.twin_services import TwinServices

import uuid
import json
import requests


class Twin(Node):
    """TODO add documentation."""

    def __init__(self):
        super().__init__("core_twin")

        # Declare Parameters
        self.declare_parameter("twin_url", "")
        self.declare_parameter("anonymous", False)
        self.declare_parameter("namespace", "")
        self.declare_parameter("name", "")
        self.declare_parameter("type", "")
        self.declare_parameter("unique_name", "")
        self.declare_parameter("attributes", json.dumps({}))
        self.declare_parameter("definition", "org.eclipse.muto:EdgeDevice:0.0.1")
        self.declare_parameter("topic", "")
        self.declare_parameter("thing_id", "")

        # Initialize Parameters
        self.twin_url = self.get_parameter("twin_url").value
        self.anonymous = self.get_parameter("anonymous").value
        self.namespace = self.get_parameter("namespace").value
        self.name = self.get_parameter("name").value
        self.type = self.get_parameter("type").value
        self.unique_name = f"{self.type}.{str(uuid.uuid4())}" if self.anonymous else self.name
        self.attributes = json.loads(self.get_parameter("attributes").value)
        self.definition = self.get_parameter("definition").value
        self.topic = f"{self.namespace}:{self.unique_name}"
        self.thing_id = f"{self.namespace}:{self.name}"

        # Services
        TwinServices(self, self.get_name())

        # Register Device
        self.register_device()

    def get_current_properties(self):
        return self.stack(self.thing_id)

    def get_stack_definition(self, stack_id):
        return self.stack(stack_id)

    def stack(self, thing_id):
        try:
            r = requests.get(self.twin_url + "/api/2/things/" +
                             thing_id + '/features/stack')
            print("Status Code: %d, Response: %s" % (r.status_code, r.text))
            if r.status_code >= 300:
                return {}
            payload = json.loads(r.text)
            return payload.get('properties', {})
        except Exception as e:
            self.get_logger().error(f'Could not get stack from twins repo {e}')
        return None

    def set_current_stack(self, stack, state='unknown'):
        if not stack:
            return
        deftn = stack.manifest
        self.get_logger().info(f"Setting current stack to {deftn}")
        stack_id = deftn.get('stackId', None)
        headers = {'Content-type': 'application/json'}

        if not stack_id is None:
            r = requests.put(self.twin_url + "/api/2/things/{}/features/stack/properties/current".format(self.thingId),
                             headers=headers, json={"stackId": stack_id, "state": state})
            print("Status Code: %d, Response: %s" % (r.status_code, r.text))
            return

        stacks = deftn.get('stack', [])
        for s in stacks:
            id = s.get('thingId', '')
            if id:
                r = requests.post(self.twin_url + "/api/2/things/{}/features/stack/properties/current".format(self.thingId),
                                  headers=headers, json={"stackId": id, "state": state})
                print("Status Code: %d, Response: %s" %
                      (r.status_code, r.text))

    def get_context(self):
        """Return information about the device."""
        context = {}

        context["namespace"] = self.namespace
        context["topic"] = self.topic
        context["twin_url"] = self.twin_url
        context["type"] = self.type
        context["unique_name"] = self.unique_name
        context["thing_id"] = self.thing_id
        context["anonymous"] = self.anonymous

        return context

    def register_device(self):
        """ # TODO add docs."""
        res = requests.patch(
            f"{self.twin_url}/api/2/things/{self.thing_id}",
            headers={"Content-type": "application/merge-patch+json"},
            json=self.device_register_data()
        )

        if res.status_code == 400 or res.status_code == 404:
            data = self.device_register_data()
            data['policyId'] = self.thing_id
            res = requests.put(
                f"{self.twin_url}/api/2/things/{self.thing_id}",
                headers={"Content-type": "application/json"},
                json=data
            )

        if res.status_code == 201 or res.status_code == 204:
            self.get_logger().info(f"Device registered successfully.")
        else:
            self.get_logger().warn(
                f"Device registration was unsuccessful. Status Code: {res.status_code}.")

        return res.status_code

    def get_registered_telemetries(self):
        """ # TODO add docs."""
        res = requests.get(
            f"{self.twin_url}/api/2/things/{self.thing_id}/features/telemetry/properties",
            headers={"Content-type": "application/json"}
        )

        if res.status_code == 200:
            self.get_logger().info(f"Telemetry properties received successfully.")
        else:
            self.get_logger().warn(
                f"Getting telemetry properties was unsuccessful - {res.status_code} {res.text}.")

        payload = json.loads(res.text)

        return payload

    def register_telemetry(self, telemetry_to_register):
        """ # TODO add docs."""
        req_telemetry = json.loads(telemetry_to_register)

        registered_telemetries = self.get_registered_telemetries()
        current_definition = registered_telemetries.get("definition", None)

        new_definition = []

        if current_definition != None:
            filtered = filter(
                lambda x: True if x.get(
                    "topic") != req_telemetry.get("topic") else False,
                current_definition
            )
            new_definition = list(filtered)

        new_definition.append(req_telemetry)

        res = requests.put(
            f"{self.twin_url}/api/2/things/{self.thing_id}/features/telemetry/properties/definition",
            headers={"Content-type": "application/json"},
            json=new_definition
        )

        if res.status_code == 201:
            self.get_logger().info(f"Telemetry registered successfully.")
        elif res.status_code == 204:
            self.get_logger().info(f"Telemetry modified successfully.")
        else:
            self.get_logger().warn(
                f"Telemetry registration was unsuccessful - {res.status_code}.")

        return res.status_code

    def delete_telemtry(self, telemetry_to_delete):
        """ # TODO add docs."""
        req_telemetry = json.loads(telemetry_to_delete)

        registered_telemetries = self.get_registered_telemetries()
        current_definition = registered_telemetries.get("definition", None)

        new_definition = []

        if current_definition != None:
            filtered = filter(
                lambda x: True if x.get(
                    "topic") != req_telemetry.get("topic") else False,
                current_definition
            )
            new_definition = list(filtered)

        res = requests.put(
            f"{self.twin_url}/api/2/things/{self.thing_id}/features/telemetry/properties/definition",
            headers={"Content-type": "application/json"},
            json=new_definition
        )

        if res.status_code == 204:
            self.get_logger().info(f"Telemetry deleted successfully.")
        else:
            self.get_logger().warn(
                f"Telemetry deletion was unsuccessful - {res.status_code}.")

        return res.status_code

    def device_register_data(self):
        """ # TODO add docs."""
        data = {
            "definition": self.definition,
            "attributes": self.attributes,
            "features": {
                "context": {
                    "properties": {}
                },
                "stack": {
                    "properties": {}
                },
                "telemetry": {
                    "properties": {}
                }
            }
        }

        return data


def main():
    rclpy.init()
    twin = Twin()
    rclpy.spin(twin)


if __name__ == '__main__':
    main()
