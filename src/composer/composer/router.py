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

class Router():
    def __init__(self, device, pipelines):
        self.device = device
        self.pipelines = pipelines

    def route(self, action, payload):
        print(f"Compose route: {action, ' ', payload}")
        if self.pipelines.get(action) is not None:
            self.device.bootstrap()
            current_stack = self.device.current_stack
            manifest = {}
            if current_stack:
                manifest = current_stack.manifest
            self.pipelines[action].execute(action, manifest, payload)
