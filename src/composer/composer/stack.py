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
import os
import re
import uuid
import rclpy
import composer.node as node
import composer.param as param
from composer.introspector import Introspector
from collections import namedtuple
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.node import HIDDEN_NODE_PREFIX
from rclpy.utilities import get_default_context
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from ament_index_python.packages import get_package_share_directory


NodeName = namedtuple('NodeName', ('name', 'namespace', 'full_name'))

NOACTION = 'none'
STARTACTION = 'start'
STOPACTION = 'stop'


class Stack():
    def __init__(self, edge_device, manifest={}, parent=None):
        self._manifest = manifest
        self.parent = parent
        self.edge_device = edge_device
        self._name = manifest.get('name', '')
        self._context = manifest.get('context', '')
        self._stackId = manifest.get('stackId', '')
        self._param = manifest.get('param', [])
        self._process = None
        self.arg = manifest.get('arg', [])
        self.arg = self.resolveArgs(self.arg)
        self._details = []

        params = []
        for pDef in self._param:
            params.append(param.Param(self, pDef))
        self._param = params

        self.initialize()

    @property
    def param(self):
        return self._param

    @property
    def manifest(self):
        return self._manifest

    @property
    def node(self):
        return self._node

    @property
    def stack(self):
        return self._stack

    def resolve_namespace(self, pns=""):
        ns = '/'
        if not ns.startswith('/'):
            ns = '/' + ns
        if ns.endswith('/'):
            return ns + pns + '/'
        return ns + '/' + pns + '/'

    def initialize(self):
        self._stack = []
        referencedStacks = self.manifest.get('stack', [])
        for stackRef in referencedStacks:
            stackDef = self.edge_device.stack(stackRef['thingId'])
            stack = Stack(self.edge_device, stackDef, self)
            self._stack.append(stack)

        self._node = []
        for nDef in self.manifest.get('node', []):
            sn = node.Node(self, nDef)
            self._node.append(sn)

    def compare(self, other):
        nodeSet = set(self.flattenNodes([]))
        otherNodeSet = set(other.flattenNodes([]))
        common = nodeSet.intersection(otherNodeSet)
        difference = nodeSet.difference(otherNodeSet)
        added = otherNodeSet.difference(nodeSet)
        return common, difference, added

    def flattenNodes(self, list):
        for n in self._node:
            list.append(n)
        for s in self._stack:
            s.flattenNodes(list)
        return list

    def calculate_ros_params_differences(self, current, other):
        differences = {}
        for node_i in current._node:
            for node_j in other._node:
                if node_i.exect == node_j.exect and node_i._pkg == node_j._pkg:
                    diff = self.compare_ros_params(
                        node_i.ros_params, node_j.ros_params)
                    if diff:
                        differences[(node_i.name, node_j.name)] = diff
        return differences

    @staticmethod
    def compare_ros_params(params1, params2):
        diff = []

        def params_to_flat_dict(params):
            flat_dict = {}
            for param in params:
                if isinstance(param, dict):
                    for key in param:
                        flat_dict[key] = param.get(key)
            return flat_dict

        dict_params1 = params_to_flat_dict(params1)
        dict_params2 = params_to_flat_dict(params2)

        all_keys = set(dict_params1).union(set(dict_params2))

        for key in all_keys:
            val1 = dict_params1.get(key, None)
            val2 = dict_params2.get(key, None)

            if val1 != val2:
                diff_entry = {'key': key, 'in_node1': val1, 'in_node2': val2}
                diff.append(diff_entry)

        return diff

    def merge(self, other):
        common, difference, added = self.compare(other)
        for n in common:
            n.action = 'none'
        for n in added:
            n.action = 'start'
        for n in difference:
            n.action = 'stop'
        all = common.union(added).union(difference)
        otherParams = {}
        merged = Stack(self.edge_device, manifest={}, parent=None)
        merged._name = other._name
        merged._context = other._context
        merged._stackId = other._stackId
        merged._stack = []
        merged._node = all
        merged._param = []
        for pn, pv in otherParams.items():
            merged._param.append(param.Param(self, {"name": pn, "value": pv}))
        merged.arg = other.arg
        merged._manifest = merged.toManifest()
        try:
            referencedStacks = self.manifest.get('stack', [])
            otherReferencedStacks = other.manifest.get('stack', [])
            self.differentStacks = []
            for i in otherReferencedStacks:
                if i not in referencedStacks:
                    differentStackDef = self.edge_device.stack(i['thingId'])
                    differentStack = Stack(
                        self.edge_device, differentStackDef, self)
                    for j in referencedStacks:
                        if j not in otherReferencedStacks:
                            differentStackDef2 = self.edge_device.stack(
                                j['thingId'])
                            differentStack2 = Stack(
                                self.edge_device, differentStackDef2, self)
                            self.differentStacks.append(
                                [differentStack, differentStack2])

            for d in self.differentStacks:
                param_difference = self.calculate_ros_params_differences(
                    d[0], d[1])
                self.change_params_at_runtime(param_difference)
            param_difference = self.calculate_ros_params_differences(
                other, self)
            self.change_params_at_runtime(param_difference)
        except Exception as e:
            print(
                f"Exception while calculating parameter difference between stacks: {e}")
        return merged

    def kill_all(self, launcher):
        intrspc = Introspector()
        for n in launcher._active_nodes:
            for name, pid in n.items():
                intrspc.kill(name, pid)

    def kill_diff(self, launcher, stack):
        intrspc = Introspector()
        for n in stack._node:
            for e in launcher._active_nodes:
                for exec_name, pid in e.items():
                    if n._exec in exec_name and n._action == STOPACTION:
                        intrspc.kill(exec_name, pid)

    def change_params_at_runtime(self, param_differences):
        try:
            for key, val in param_differences.items():
                for i in range(len(val)):
                    subprocess.run(['ros2', 'param', 'set', str(key[0]), str(
                        val[i]['key']), str(val[i]['in_node1'])])
        except Exception as e:
            print(
                f'Exception occurred while changing parameters at runtime: {e}')

    def get_node_names(self, include_hidden_nodes=False):
        context = get_default_context()
        __node = _rclpy.Node(
            "ababa",
            "xxxx",
            context.handle,
            None,
            False,
            False
        )
        node_names_and_namespaces = __node.get_node_names_and_namespaces()
        return [
            NodeName(
                name=t[0],
                namespace=t[1],
                full_name=t[1] + ('' if t[1].endswith('/') else '/') + t[0])
            for t in node_names_and_namespaces
            if (
                include_hidden_nodes or
                (t[0] and not t[0].startswith(HIDDEN_NODE_PREFIX))
            )
        ]

    def toShallowManifest(self):
        manifest = {"name": self._name,
                    "context": self._context,
                    "stackId": self._stackId,
                    "param": [],
                    "arg": [],
                    "stack": [],
                    "node": []}
        return manifest

    def toManifest(self):
        manifest = self.toShallowManifest()
        for p in self._param:
            manifest["param"].append(p.toManifest())
        for a in self.arg:
            manifest["arg"].append(self.arg[a])
        for s in self._stack:
            manifest["stack"].append(s.toShallowManifest())
        for n in self._node:
            manifest["node"].append(n.toManifest())
        return manifest

    def launch(self, launcher):
        self.launchMaster()
        launch_description = LaunchDescription()

        try:
            for s in self._stack:
                s.launch(launcher)
            for n in self._node:
                remaps = []
                if (n.remap != []):
                    for rmp in n.remap:
                        remap = (rmp['from'], rmp['to'])
                        remaps.append(remap)
                if n._action == STARTACTION:
                    launch_description.add_action(Node(
                        package=n.pkg,
                        executable=n.exect,
                        name=n.name,
                        namespace=n.namespace,
                        parameters=n.ros_params,
                        arguments=n.args.split(),
                        remappings=remaps
                    ))

                elif n._action == NOACTION:
                    active_nodes = self.get_node_names()
                    is_active = False
                    for nn in active_nodes:
                        if n.name == nn.name:
                            is_active = True
                            break
                    if not is_active:
                        launch_description.add_action(Node(
                            package=n.pkg,
                            executable=n.exect,
                            name=n.name,
                            namespace=n.namespace,
                            parameters=n.ros_params,
                            arguments=n.args.split(),
                            remappings=remaps
                        ))
                elif n._action == STOPACTION:
                    self.kill_diff(launcher, self)
                else:
                    print(f"Unexpected action ({n._action}) in stack launch.")
        except Exception as e:
            print(f'Stack launching ended with exception: {e}')

        launcher.start(launch_description)

    def apply(self, launcher):
        self.launchMaster()
        self.launch(launcher)

    def launchMaster(self):
        if not rclpy.ok():
            rclpy.init(args=None)
            print("ROS 2 is initialized")

    def resolveExpression(self, v=""):
        value = str(v)
        if value is None:
            return ""
        expressions = re.findall('\$\(([\s0-9a-zA-Z_-]+)\)', value)
        vals = []
        for match in expressions:
            expr, var = match.split()
            if expr == 'find':
                try:
                    vals.append(get_package_share_directory(var))
                except Exception as e:
                    print(f'Excepion occured: {e}')
            elif expr == 'env':
                v = os.environ.get(var)
                if v is None:
                    raise Exception(value + ' does not exist', 'param')
                vals.append(v)
            elif expr == 'optenv':
                defv = var
                if defv is None:
                    defv = ''
                # return re.sub('\$\((.*)\)', os.environ.get(var,defv) , value)
                vals.append(os.environ.get(var, defv))
            elif expr == 'arg':
                a = self.arg.get(var)
                if a is not None:
                    vals.append(a['value'])
            elif expr == 'anon':
                if var in self.anon.keys():
                    return self.anon[var]
                self.anon['key'] = var + uuid.uuid1().hex
                # return re.sub('\$\((.*)\)', self.anon['key'] , value)
                vals.append(self.anon['key'])
            elif expr == 'eval':
                raise Exception(value + ' NOT SUPPORTED IN MUTO', 'param')
            else:
                return value
        result = value
        for v in vals:
            result = re.sub('\$\(([\s0-9a-zA-Z_-]+)\)', v, result, count=1)
        return result

    def resolveParamExpression(self, param={}):
        name = ''
        value = None
        valKey = None
        for k in param.keys():
            if 'name' == k:
                name = param['name']
            else:
                value = param[k]
                valKey = k
        if not valKey is None:
            return (name, valKey, self.resolveExpression(value))
        return None

    def resolveArgs(self, array=[]):
        result = {}
        self.arg = {}
        for item in array:
            name, key, value = self.resolveParamExpression(item)
            p = {"name": name}
            p[key] = value
            result[name] = p
            self.arg[name] = p

        return result
