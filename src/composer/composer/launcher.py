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

import asyncio
import multiprocessing

from launch import LaunchDescription, LaunchService
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import RegisterEventHandler


class Ros2LaunchParent:
    def __init__(self):
        self.manager = multiprocessing.Manager()
        self._active_nodes = self.manager.list()
        self._lock = self.manager.Lock()

    def __del__(self):
        self.manager.shutdown()

    def start(self, launch_description: LaunchDescription):
        self._stop_event = multiprocessing.Event()
        self._process = multiprocessing.Process(target=self._run_process, args=(
            self._stop_event, launch_description), daemon=True)
        self._process.start()

    def shutdown(self):
        self._stop_event.set()
        self._process.join(timeout=20.0)
        if self._process.is_alive():
            self._process.terminate()
            print(
                "The process did not terminate gracefully and was terminated forcefully.")

    def on_process_start(self, event, context):
        details = event.action.process_details
        with self._lock:
            self._active_nodes.append({details['name']: details['pid']})
        print(f"Active Nodes after start: {self._active_nodes}")

    def on_process_exit(self, event, context):
        details = event.action.process_details
        with self._lock:
            for node in self._active_nodes[:]:  # make a copy of the list
                if node.get(details['name']) == details['pid']:
                    self._active_nodes.remove(node)
                    break
        print(f"Active Nodes after exit: {self._active_nodes}")
        if not self._active_nodes:
            self.shutdown()

    def _run_process(self, stop_event, launch_description):
        try:
            asyncio.set_event_loop(asyncio.new_event_loop())
            loop = asyncio.get_event_loop()
            launch_description.add_action(
                RegisterEventHandler(
                    OnProcessStart(
                        on_start=self.on_process_start,
                    )
                )
            )
            launch_description.add_action(
                RegisterEventHandler(
                    OnProcessExit(
                        on_exit=self.on_process_exit
                    )
                )
            )
            launch_service = LaunchService(debug=False, noninteractive=True)
            launch_service.include_launch_description(launch_description)
            launch_task = loop.create_task(launch_service.run_async())

            async def wait_for_stop_event():
                while not stop_event.is_set():
                    await asyncio.sleep(0.1)
                launch_service.shutdown()

            loop.run_until_complete(asyncio.gather(
                launch_task,
                wait_for_stop_event()
            ))
        except Exception as e:
            print(f"An exception occurred: {e}")
        finally:
            loop.close()
