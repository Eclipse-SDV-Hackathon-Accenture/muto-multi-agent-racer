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


import subprocess
from rclpy.node import Node


class Introspector(Node):
    def __init__(self):
        super().__init__('introspector')

    def kill(self, name, pid):
        print(f'Killing {name} with PID: {pid}')
        try:
            subprocess.run(['kill', str(pid)])
        except Exception as e:
            print(f'Kill was not successful for {name}. Exception msg: {e}')
