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

import os
import uuid
from rclpy.node import Node
from muto_msgs.msg import MutoAction
from ament_index_python.packages import get_package_share_directory


class LogPublisher(Node):
    """
    Class for outputting useful data to multiple streams
    (to a file, to /composer_to_agent topic and to stdout) 
    for both debugging and observing a running program
    """

    def __init__(self):
        unique_suffix = str(uuid.uuid4()).replace("-", "")[:12]  # Take the first 12 characters
        unique_name = 'log_publisher_' + unique_suffix
        super().__init__(unique_name)
        self.publisher_ = self.create_publisher(
            MutoAction, '/composer_to_agent', 10)  # Ensure the message type matches the topic type
        self.log_message = ''
        self.log_method = {'INFO': self.get_logger().info,
                           'ERROR': self.get_logger().error,
                           'WARN': self.get_logger().warn}
        self.log_file_name = os.path.join(
            get_package_share_directory('composer'),
            'log',
            'composer.log'
        )

        if not os.path.isfile(self.log_file_name):
            open(self.log_file_name, 'a').close()

    def write_to_log_file_and_publish(self, message, log_method, sender_name):
        """
        First, it outputs log data to local log file (can be found under package share directory)
        Second, it publishes the same data to /compose_to_agent topic
        """
        data = f'[{sender_name}]: {message}\n'
        with open(self.log_file_name, 'a') as log_file:
            log_file.write(data)

        msg = MutoAction(context='', method=log_method,
                         payload=data)
        self.publisher_.publish(msg)

    def print_log(self, log, log_method, sender_name):
        """
        Prints logs for both developer and client
        """
        try:
            self.log_method[log_method](
                f'[{sender_name}] | {log}')  # For developer
            self.write_to_log_file_and_publish(
            message=log, log_method=log_method, sender_name=sender_name)  # For client
        except Exception as e:
            print(f"Logger failed with exception: {e}")
