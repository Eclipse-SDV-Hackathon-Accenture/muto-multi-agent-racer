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

import unittest
from unittest.mock import patch

import rclpy
from rclpy.executors import MultiThreadedExecutor

import agent.commands
from agent.ros.node_commands import NodeCommands
from agent.ros.topic_commands import TopicCommands
from agent.ros.param_commands import ParamCommands

from muto_msgs.msg import MutoAction, MutoActionMeta, CommandInput, CommandOutput, PluginResponse
from muto_msgs.srv import CommandPlugin

import json


file_path = "/home/alpsarica/Desktop/muto_ros2/src/agent/config/agent.yaml"

class TestCommandsPlugin(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        self.dummy_service_to_call = "rostopic_list"
        self.dummy_plugin = "CommandPlugin"
        self.node = agent.commands.ROSCommandsPlugin()

    def tearDown(self):
        self.node.destroy_node()

    def test_commands_node_create(self):
        assert self.node != None, "Node couldn't be created."

    @patch("agent.commands.Command")
    def test_agent_msg_callback(self, Command):
        meta_msg = MutoActionMeta()
        meta_msg.response_topic = "db-org.eclipse.muto.sandbox:f1tenth/agent/fa30fe44-153f-4541-b78e-3f71863a331d"
        meta_msg.correlation_data = "fa30fe44-153f-4541-b78e-3f71863a331d"        
        
        action_msg = MutoAction()
        action_msg.context = ""
        action_msg.method = "ros/topic"
        action_msg.payload = ""
        action_msg.meta = meta_msg

        command = Command(self.node, self.dummy_service_to_call, self.dummy_plugin)

        self.node.commands = {"ros/topic": command}

        self.node.agent_msg_callback(action_msg)
        
        command.execute.assert_called(), "Command didn't executed."


    def test_publish_executed_command_result(self):
        meta_msg = MutoActionMeta()
        meta_msg.response_topic = "db-org.eclipse.muto.sandbox:f1tenth/agent/fa30fe44-153f-4541-b78e-3f71863a331d"
        meta_msg.correlation_data = "fa30fe44-153f-4541-b78e-3f71863a331d"

        plugin_msg = CommandPlugin.Response()
        plugin_msg.output.payload = '{"nodes": [{"name": "muto_agent"}]}'

        self.node.received_message = None
        self.node.create_subscription(
            MutoAction,
            self.node.get_parameter("commands_to_agent_topic").value,
            lambda msg: setattr(self.node, "received_message", msg),
            10
        )

        self.node.publish_executed_command_result(plugin_msg, meta_msg)

        rclpy.spin_once(self.node, timeout_sec=3)

        assert type(self.node.received_message) == MutoAction, "Received message has to be MutoAction type."
        assert self.node.received_message.context == ""
        assert self.node.received_message.method == ""
        assert self.node.received_message.payload == plugin_msg.output.payload, "Payload shouldn't be changed."
        assert self.node.received_message.meta == meta_msg, "Meta shouldn't be changed."

    def test_construct_command_output_message(self):
        pass


class TestCommand(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        self.dummy_service_to_call = "rosnode_list"
        self.dummy_plugin = "CommandPlugin"
        self.commnands_plugin_node = agent.commands.ROSCommandsPlugin()
        self.command = agent.commands.Command(self.commnands_plugin_node, self.dummy_service_to_call, self.dummy_plugin)

    def tearDown(self):
        self.commnands_plugin_node.destroy_node()
    
    def test_command_client_created(self):
        clients = self.commnands_plugin_node.clients
        clients = [x.srv_name for x in clients]

        assert self.dummy_service_to_call in clients, "Client not created"
    
    def test_command_execute(self):
        # TODO
        pass

class TestNodeCommands(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        self.commnands_plugin_node = agent.commands.ROSCommandsPlugin()
        self.node_commands = NodeCommands(self.commnands_plugin_node)

    def tearDown(self):
        self.commnands_plugin_node.destroy_node()

    @patch("agent.ros.node_commands.NodeCommands.get_discovered_nodes")
    def test_callback_rosnode_list_wo_nodes(self, get_discovered_nodes):
        req = CommandPlugin.Request()
        res = CommandPlugin.Response()

        get_discovered_nodes.return_value = []

        expected_response = PluginResponse()
        expected_response.result_code = 0
        expected_response.error_message = ""
        expected_response.error_description = ""


        result = self.node_commands.callback_rosnode_list(req, res)

        assert type(result) == CommandPlugin.Response
        assert type(result.output) == CommandOutput
        assert type(result.output.result) == PluginResponse
        assert result.output.payload == '{"nodes": []}'
        assert result.output.result == expected_response

    @patch("agent.ros.node_commands.NodeCommands.get_node_info")
    @patch("agent.ros.node_commands.NodeCommands.get_discovered_nodes")
    def test_callback_rosnode_list_w_nodes(self, get_discovered_nodes, get_node_info):
        req = CommandPlugin.Request()
        res = CommandPlugin.Response()

        nodes_list = ['muto_agent', 'mqtt_gateway']
        node_info = {
            'name': 'mqtt_gateway', 
            'pubs': [
                {'topic': '/msg_2', 'type': ['muto_msgs/msg/Gateway']},
                {'topic': '/parameter_events', 'type': ['rcl_interfaces/msg/ParameterEvent']},
                {'topic': '/rosout', 'type': ['rcl_interfaces/msg/Log']}
            ],
            'subs': [
                {'topic': '/msg1', 'type': ['muto_msgs/msg/Gateway']}
            ],
            'services': [
                {'topic': '/mqtt_gateway/describe_parameters', 'type': ['rcl_interfaces/srv/DescribeParameters']},
                {'topic': '/mqtt_gateway/get_parameter_types', 'type': ['rcl_interfaces/srv/GetParameterTypes']},
                {'topic': '/mqtt_gateway/get_parameters', 'type': ['rcl_interfaces/srv/GetParameters']},
                {'topic': '/mqtt_gateway/list_parameters', 'type': ['rcl_interfaces/srv/ListParameters']},
                {'topic': '/mqtt_gateway/set_parameters', 'type': ['rcl_interfaces/srv/SetParameters']},
                {'topic': '/mqtt_gateway/set_parameters_atomically', 'type': ['rcl_interfaces/srv/SetParametersAtomically']}
            ]
        }

        get_discovered_nodes.return_value = nodes_list
        get_node_info.return_value = node_info

        expected_response = PluginResponse()
        expected_response.result_code = 0
        expected_response.error_message = ""
        expected_response.error_description = ""
        expected_payload = {
            "nodes": [
                {"name": "muto_agent", "info": node_info},
                {"name": "mqtt_gateway", "info": node_info}
            ]
        }


        result = self.node_commands.callback_rosnode_list(req, res)

        assert type(result) == CommandPlugin.Response
        assert type(result.output) == CommandOutput
        assert type(result.output.result) == PluginResponse
        assert result.output.payload == json.dumps(expected_payload)
        assert result.output.result == expected_response


class TestParamCommands(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        self.commnands_plugin_node = agent.commands.ROSCommandsPlugin()
        self.param_commands = ParamCommands(self.commnands_plugin_node)

    def tearDown(self):
        self.commnands_plugin_node.destroy_node()

    def test_callback_rosparam_list_wo_nodes(self):
        req = CommandPlugin.Request()
        res = CommandPlugin.Response()

        expected_response = PluginResponse()
        expected_response.result_code = 0
        expected_response.error_message = ""
        expected_response.error_description = ""
        expected_payload = {
            "params": [
                {"name": "use_sim_time", "value": False},
                {"name": "agent_to_commands_topic", "value": "msg_3"},
                {"name": "commands_to_agent_topic", "value": "msg4"}
            ]
        }


        result = self.param_commands.callback_rosparam_list(req, res)

        assert type(result) == CommandPlugin.Response
        assert type(result.output) == CommandOutput
        assert type(result.output.result) == PluginResponse
        assert result.output.payload == json.dumps(expected_payload)
        assert result.output.result == expected_response

    def test_callack_rosparam_get(self):
        req = CommandPlugin.Request()
        res = CommandPlugin.Response()

        req.input.payload = json.dumps({"param": "use_sim_time"})

        expected_response = PluginResponse()
        expected_response.result_code = 0
        expected_response.error_message = ""
        expected_response.error_description = ""
        expected_payload = {
            "name": "use_sim_time",
            "value": False
        }


        result = self.param_commands.callback_rosparam_get(req, res)

        assert type(result) == CommandPlugin.Response
        assert type(result.output) == CommandOutput
        assert type(result.output.result) == PluginResponse
        assert result.output.payload == json.dumps(expected_payload)
        assert result.output.result == expected_response


class TestTopicCommands(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        self.commnands_plugin_node = agent.commands.ROSCommandsPlugin()
        self.topic_commands = TopicCommands(self.commnands_plugin_node)

    def tearDown(self):
        self.commnands_plugin_node.destroy_node()

    @patch("agent.ros.topic_commands.TopicCommands.construct_subscribers")
    @patch("agent.ros.topic_commands.TopicCommands.construct_publishers")
    def test_callback_rostopic_list(self, pubs_func, subs_func):
        req = CommandPlugin.Request()
        res = CommandPlugin.Response()

        dummy_pubs = [
            ('/msg1', ['muto_msgs/msg/Gateway'], ['muto_agent']),
            ('/msg4', ['muto_msgs/msg/MutoAction'], ['commands_plugin']),
            ('/msg_2', ['muto_msgs/msg/Gateway'], ['mqtt_gateway']),
            ('/msg_3', ['muto_msgs/msg/MutoAction'], ['muto_agent']),
            ('/parameter_events', ['rcl_interfaces/msg/ParameterEvent'], ['muto_agent', 'mqtt_gateway', 'commands_plugin']),
            ('/rosout', ['rcl_interfaces/msg/Log'], ['muto_agent', 'mqtt_gateway', 'commands_plugin']),
            ('/stack', ['muto_msgs/msg/MutoAction'], ['muto_agent']),
            ('/twin', ['std_msgs/msg/String'], [])
        ]

        dummy_subs = [
            ('/msg1', ['muto_msgs/msg/Gateway'], ['mqtt_gateway']),
            ('/msg4', ['muto_msgs/msg/MutoAction'], ['muto_agent']),
            ('/msg_2', ['muto_msgs/msg/Gateway'], ['muto_agent']),
            ('/msg_3', ['muto_msgs/msg/MutoAction'], ['commands_plugin']),
            ('/parameter_events', ['rcl_interfaces/msg/ParameterEvent'], []),
            ('/rosout', ['rcl_interfaces/msg/Log'], []),
            ('/stack', ['muto_msgs/msg/MutoAction'], []),
            ('/twin', ['std_msgs/msg/String'], ['muto_agent'])
        ]

        pubs_func.return_value = dummy_pubs
        subs_func.return_value = dummy_subs

        expected_response = PluginResponse()
        expected_response.result_code = 0
        expected_response.error_message = ""
        expected_response.error_description = ""
        expected_payload = {
            "pubs": dummy_pubs,
            "subs": dummy_subs
        }

        result = self.topic_commands.callback_rostopic_list(req, res)

        assert type(result) == CommandPlugin.Response
        assert type(result.output) == CommandOutput
        assert type(result.output.result) == PluginResponse
        assert result.output.payload == json.dumps(expected_payload)
        assert result.output.result == expected_response

    @patch("agent.ros.topic_commands.TopicCommands.get_topic_info")
    def test_callback_rostopic_info(self, topic_info):
        req = CommandPlugin.Request()
        res = CommandPlugin.Response()

        req.input.payload = json.dumps({"topic": "dummy_topic"})

        topic_info.return_value = {
            "topic": "dummy_topic",
            "types": None,
            "pubs": ["mqtt_gateway", "commands_plugin"],
            "subs": []
        }

        expected_response = PluginResponse()
        expected_response.result_code = 0
        expected_response.error_message = ""
        expected_response.error_description = ""
        expected_payload = {
            "topic": "dummy_topic",
            "types": None,
            "pubs": ["mqtt_gateway", "commands_plugin"],
            "subs": []
        }

        result = self.topic_commands.callback_rostopic_info(req, res)

        assert type(result) == CommandPlugin.Response
        assert type(result.output) == CommandOutput
        assert type(result.output.result) == PluginResponse
        assert result.output.payload == json.dumps(expected_payload)
        assert result.output.result == expected_response
    
    def test_rostopic_echo(self):
        # TODO
        pass

