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

from composer.stack import Stack
from composer.launcher import Ros2LaunchParent

UNKNOWN = 'unknown'
ACTIVE = 'active'
KILLED = 'killed'


class EdgeDevice():
    def __init__(self, twin):
        self.twin = twin
        self.definition = None
        self.state = None
        self.current_stack = None
        self.launcher = Ros2LaunchParent()

    def bootstrap(self):
        try:
            print("Edge Device bootstrapping...")
            current_definition = self.twin.get_current_stack()
            if current_definition is None:
                current_definition = {}
            current_definition = current_definition.get('current', None)

            if not current_definition is None:
                self.definition = self.twin.stack(
                    current_definition.get('stackId', None))
                self.state = current_definition.get('state', UNKNOWN)
            if not self.definition is None:
                self.current_stack = Stack(
                    self, self.definition, None)
            print('Edge Device bootstrap done.')
        except Exception as e:
            print(
                f"An exception occurred in device bootstrapping: {e}")

    def activate(self, current=None):
        try:
            if self.current_stack is not None:
                self.current_stack.kill_all(self.launcher)
            if not current is None:
                self.current_stack = Stack(
                    self, current, None)

            self.current_stack.launch(self.launcher)
            self.twin.set_current_stack(self.current_stack, state=ACTIVE)
            self.state = ACTIVE
        except Exception as e:
            print(
                f'An exception occurred in edge_device activate: {e}')

    def apply(self, current=None):
        try:
            if self.current_stack is not None:
                self.current_stack.kill_diff(self.launcher, self.current_stack)
            if not current is None:
                self.current_stack = Stack(self, current, None)
            self.current_stack.apply(self.launcher)
            self.twin.set_current_stack(self.current_stack, state=ACTIVE)
            self.state = ACTIVE
        except Exception as e:
            print(f'An exception occurred in edge_device apply: {e}')

    def kill(self, payload=None):
        try:
            if self.current_stack is not None:
                self.current_stack.kill_all(self.launcher)
            if not payload is None:
                self.current_stack = Stack(self, payload, None)
            self.twin.set_current_stack(self.current_stack, state=KILLED)
            self.state = KILLED
        except Exception as e:
            print(f"Failed to kill. Exception: {e}")

    def stack(self, stackId):
        return self.twin.stack(stackId)
