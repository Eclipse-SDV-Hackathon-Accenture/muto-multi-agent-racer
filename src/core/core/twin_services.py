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

from muto_msgs.srv import CoreTwin

import json


class TwinServices():
    """ # TODO add docs."""

    def __init__(self, node, nname):
        self.node = node
        self.nname = nname

        self.node.create_service(CoreTwin, f"{self.nname}/get_current_properties", self.callback_get_current_properties)
        self.node.create_service(CoreTwin, f"{self.nname}/get_stack_definition", self.callback_get_stack_definition)
        self.node.create_service(CoreTwin, f"{self.nname}/set_current_stack", self.callback_set_current_stack)
        self.node.create_service(CoreTwin, f"{self.nname}/get_context", self.callback_get_context)
        self.node.create_service(CoreTwin, f"{self.nname}/register_device", self.callback_register_device)
        self.node.create_service(CoreTwin, f"{self.nname}/get_registered_telemetries", self.callback_get_registered_telemetries)
        self.node.create_service(CoreTwin, f"{self.nname}/register_telemetry", self.callback_register_telemetry)
        self.node.create_service(CoreTwin, f"{self.nname}/delete_telemetry", self.callback_delete_telemetry)



    def callback_get_current_properties(self, request, response):
        """ # TODO add docs."""
        stack = self.node.get_current_properties()
        
        response.output = json.dumps(stack)

        return response
    
    def callback_get_stack_definition(self, request, response):
        """ # TODO add docs."""
        stack_id = request.input
        definition = self.node.get_stack_definition(stack_id)
        
        response.output = json.dumps(definition)

        return response
    
    def callback_set_current_stack(self, request, response):
        """ # TODO add docs."""
        payload = request.input
        status_code = self.node.set_current_stack(payload)

        response.output = str(status_code)

        return response
    
    def callback_get_context(self, request, response):
        """ # TODO add docs."""
        context = self.node.get_context()
        
        response.output = json.dumps(context)

        return response

    def callback_register_device(self, request, response):
        """ # TODO add docs."""
        status_code = self.node.register_device()

        response.output = str(status_code)

        return response
    
    def callback_get_registered_telemetries(self, request, response):
        """ # TODO add docs."""
        telemetry_data = self.node.get_telemetry()

        response.output = telemetry_data

        return response
    
    def callback_register_telemetry(self, request, response):
        """ # TODO add docs."""
        payload = request.input
        status_code = self.node.register_telemetry(payload)

        response.output = str(status_code)

        return response

    def callback_delete_telemetry(self, request, response):
        """ # TODO add docs."""
        payload = request.input
        status_code = self.node.delete_telemtry(payload)

        response.output = str(status_code)

        return response
        