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

import subprocess
import ros2param.api as parameter_api
from rclpy.parameter import Parameter
import shlex


class Param():
    def __init__(self, stack, manifest={}):
        self.stack = stack
        self._manifest = manifest
        self._param = manifest.get('param', [])  # recursive
        self._name = manifest.get('name', '')
        self._value = manifest.get('value', '')
        self._sep = manifest.get('sep', '')
        self._from_file = manifest.get('from', '')
        self._namespace = manifest.get('namespace', '/')
        self._command = manifest.get('command', '')

        try:
            if self._from_file:
                resolved_expression = stack.resolveExpression(self._from_file)
                self.from_file = resolved_expression
                self._value = resolved_expression
            elif self._command:
                resolved_expression = stack.resolveExpression(self._command)
                args = shlex.split(resolved_expression)
                val = subprocess.check_output(args).decode('utf-8')
                self._value = val
            elif 'value' in manifest.keys():
                if (type(self._value)) is list:
                    resolved_expression = self._value
                else:
                    resolved_expression = stack.resolveExpression(self._value)
                    if (resolved_expression.lower() == 'true'):
                        resolved_expression = True
                    elif (resolved_expression.lower() == 'false'):
                        resolved_expression = False
                    else:
                        resolved_expression = self.convert_string_to_int_or_double(
                            resolved_expression)

                    self._value = resolved_expression
        except Exception as e:
            print(f"Exception occurred in parameter parsing: {e}")
            

    def convert_string_to_int_or_double(self, value):
        try:
            return int(value)
        except ValueError:
            try:
                return float(value)
            except ValueError:
                return value

    def toManifest(self):
        if not self._value is None:
            manifest = {"name": self._name, "value": self._value,
                        "sep": self._sep, "from": self._from_file,
                        "namespace": self._namespace,
                        "param": []}
        else:
            manifest = {"name": self._name,
                        "sep": self._sep, "from": self._from_file,
                        "namespace": self._namespace,
                        "command": self._command, "param": []}
        for p in self._param:
            manifest["param"].append(p.toManifest())
        return manifest

    def __eq__(self, other):
        """Overrides the default implementation"""
        if isinstance(other, Param):
            if self.name != other.name:
                return False
            if self._value != other.value:
                return False
            if self.from_file != other.from_file:
                return False
            if self.namespace != other.namespace:
                return False
            if self.command != other.command:  # no getter setter
                return False
            return True
        return False

    def __ne__(self, other):
        """Overrides the default implementation (unnecessary in Python 3)"""
        return not self.__eq__(other)

    @property
    def manifest(self):
        return self._manifest

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, n):
        self._name = n

    @property
    def namespace(self):
        return self._namespace

    @namespace.setter
    def namespace(self, n):
        self._namespace = n

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, n):
        self._value = n

    @property
    def sep(self):
        return self._sep

    @sep.setter
    def sep(self, n):
        self._sep = n

    @property
    def from_file(self):
        return self._from_file

    @from_file.setter
    def from_file(self, n):
        self._from_file = n

    @property
    def command(self):
        return self._command

    @command.setter
    def command(self, n):
        self._command = n
