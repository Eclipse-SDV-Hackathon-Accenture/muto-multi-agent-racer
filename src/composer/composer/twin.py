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
import uuid
import requests

INFO = 'INFO'
WARN = 'WARN'
ERROR = 'ERROR'


class Twin():
    """
    The class that handles Muto Digital Twin
    """

    def __init__(self, node, config, publisher=None):
        self.twin_url = config['twin_url']
        self.publisher = publisher
        self.namespace = config.get("namespace", "")
        self.name = config.get("name", "")
        self.thingId = self.namespace + ":" + self.name
        self.requested_stacks = []
        
        # unique_name = config.get(
            # "uniqueName",  config['type']+"."+str(uuid.uuid4()))
        # self._unique_name = unique_name


    def get_current_stack(self):
        return self.stack(self.thingId)

    def set_current_stack(self, stack, state='unknown'):
        if stack is None:
            print("No stack to set")
            return
        try:
            deftn = stack._manifest
            stack_id = deftn.get('stackId', None)
            headers = {'Content-type': 'application/json'}

            if not stack_id is None:
                r = requests.put(self.twin_url + "/api/2/things/{}/features/stack/properties/current".format(self.thingId),
                                 headers=headers, json={"stackId": stack_id, "state": state})
                print(
                    f"Setting stack ended for stack {stack_id} with status code: {r.status_code}")
                return

            stacks = deftn.get('stack', [])
            for s in stacks:
                id = s.get('thingId', '')
                if id:
                    r = requests.post(self.twin_url + "/api/2/things/{}/features/stack/properties/current".format(self.thingId),
                                      headers=headers, json={"stackId": id, "state": state})
                    print(
                        f"Setting stack ended for stack {stack_id} with status code: {r.status_code}")
        except Exception as e:
            print(f"Setting stack ended with exception: {e}")

    def stack(self, thingId):
        try:
            for prop in self.requested_stacks:
                if thingId == prop['stackId']:
                    print(f'Skipped getting {thingId} since it is in memory')
                    return prop
            r = requests.get(self.twin_url + '/api/2/things/' +
                             thingId + '/features/stack')
            print(
                f"Stack getting for {thingId} from repo ended with status code: {r.status_code}")
            if r.status_code >= 300:
                return {}
            payload = json.loads(r.text)
            props = payload.get('properties', {})
            if props not in self.requested_stacks:
                if props.get('stackId', None) is not None:
                    self.requested_stacks.append(props)
            return props
        except Exception as e:
            print(
                f"Stack getting for {thingId} from repo ended with exception: {e}")
