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
from muto_msgs.srv import CommandPlugin

import json


class NodeCommands():
    """
    Class for managing rosnode commands.

    This class provides functionality to handle rosnode commands such as
    listing nodes, retrieving node information, and pinging nodes. Also
    services are defined here.

    Args:
        node: RosCommnadsPlugin class which is a subclass of Node.

    """

    def __init__(self, node):
        self.node = node

        self.rosnode_list = self.node.create_service(
            CommandPlugin, "rosnode_list", self.callback_rosnode_list)
        self.rosnode_info = self.node.create_service(
            CommandPlugin, "rosnode_info", self.callback_rosnode_info)
        self.rosnode_ping = self.node.create_service(
            CommandPlugin, "rosnode_ping", self.callback_rosnode_ping)

    def callback_rosnode_list(self, request, response):
        """
        Callback function for handling rosnode_list service request.

        Retrieves the list of nodes and their associated publishers and
        subscribers, constructs a JSON-formatted result, and sends it as the
        output of the response.

        Args:
            request: The request message object.
            response: The response message object.

        Returns:
            The modified response message object.
        """
        result = {"nodes": []}
        try:
            discovered_nodes = self.get_discovered_nodes()
        except:
            self.node.get_logger().error(f"Couldn't get the list of running nodes.")

        for node in discovered_nodes:
            try:
                info = self.get_node_info(node)
            except:
                self.node.get_logger().error(f"Couldn't get the information about the {node[0]}.")
            result["nodes"].append({"name": node[1]+"/"+node[0], "info": info})


        response.output = self.node.construct_command_output_message(result)

        return response

    def callback_rosnode_info(self, request, response):
        """
        Callback function for handling rosnode_info service request.

        This function extracts the requested node name from the request payload
        and retrieves the corresponding node information. The retrieved node
        information is then serialized and added to the response payload.


        Args:
            request: The request message object.
            response: The response message object.

        Returns:
            The modified response message object.
        """
        payload = json.loads(request.input.payload)
        result = {}

        requested_node = payload["node"]
        if requested_node != None:
            try:
                result = self.get_node_info(requested_node)
            except:
                self.node.get_logger().error(f"Couldn't get the information about the {requested_node}.")


        response.output = self.node.construct_command_output_message(result)
        return response

    def callback_rosnode_ping(self, request, response):
        """
        Callback function for handling rosnode_ping service request.

        This function extracts the requested node name from the request payload
        and checks if the node is discovered. The response payload indicates
        the status of the requested node, whether it is found or not.

        Args:
            request: The request message object.
            response: The response message object.

        Returns:
            The modified response message object.
        """
        payload = json.loads(request.input.payload)
        result = {"status": "NOTFOUND"}

        requested_node = payload["node"]

        try:
            discovered_nodes = self.get_discovered_nodes()
        except:
            self.node.get_logger().error(f"Couldn't get the list of running nodes.")

        if (requested_node != None) and (requested_node in discovered_nodes):
            result = {"status": True, "node": requested_node}


        response.output = self.node.construct_command_output_message(result)

        return response

    def get_discovered_nodes(self):
        """
        Get list of nodes.

        Retrieves a list of discovered nodes. Removes nodes starting with an
        underscore which are considered internal nodes.

        Returns:
            A list of discovered node names.
        """
        node_list = self.node.get_node_names_and_namespaces()
        node_list = [x for x in node_list if not x[0].startswith("_")]

        return node_list

    def get_node_info(self, n):
        """
        Get information about a node.

        This function creates a dictionary object containing information about
        a node. The node information includes the name of the node, as well as
        list for publishers, subscriptions, and services associated with the
        node.

        Args:
            node_name: Name of the node to get information for.
            name_space: Namespace of the node.

        Returns:
            A dictionary including publishers, subscribers and services as keys
            and corresponding values as values.
        """
        node_name = n[0]
        name_space = n[1]
        info = {"name": node_name, "pubs": [], "subs": [], "services": []}

        publishers = self.node.get_publisher_names_and_types_by_node(
            node_name, name_space)
        subscribers = self.node.get_subscriber_names_and_types_by_node(
            node_name, name_space)
        services = self.node.get_service_names_and_types_by_node(
            node_name, name_space)

        for topic, type in publishers:
            info["pubs"].append({"topic": topic, "type": type})

        for topic, type in subscribers:
            info["subs"].append({"topic": topic, "type": type})

        for service, type in services:
            info["services"].append({"topic": service, "type": type})

        return info
