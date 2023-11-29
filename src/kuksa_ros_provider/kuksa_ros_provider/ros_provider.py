import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive


import asyncio
import csv
import argparse
import logging
import os
from pathlib import Path
import random

from kuksa_client.grpc import Datapoint
from kuksa_client.grpc import DataEntry
from kuksa_client.grpc import EntryUpdate
from kuksa_client.grpc import Field
from kuksa_client.grpc import VSSClientError
from kuksa_client.grpc.aio import VSSClient


class KuksaRosProvider(Node):
    def __init__(self):
        super().__init__("kuksa_ros_provider")
        self.declare_parameter("muto_vin", "40MUT05848Z411439")
        self.declare_parameter("racecar_namespace", "racecar1")
        self.declare_parameter("drive_topic", "drive")
        self.declare_parameter("databroker", "databroker-racer")
        self.declare_parameter("port", 55556)

        self.vin = self.get_parameter("muto_vin").value
        self.racecar1_drive_topic = self.get_parameter("drive_topic").value
        self.databroker = self.get_parameter("databroker").value
        self.port = self.get_parameter("port").value

        self.create_subscription(
            AckermannDriveStamped, self.racecar1_drive_topic, self.racecar1_drive_callback, 10)

        # current,Vehicle.VehicleIdentification.VIN,YV2E4C3A5VB180691,0.5
        self.send_id(self.vin)

    def send_id(self, vin):
        async def asyncfunc():
            await self.feed_kuksa(vin, 'Vehicle.VehicleIdentification.VIN')
        asyncio.run(asyncfunc())

    def racecar1_drive_callback(self, msg):
        # self.get_logger().info(f" ackermann: {msg.drive.speed}")
        async def asyncfunc():
            await self.feed_kuksa(msg.drive.speed * 3.6, 'Vehicle.Speed')
            await self.feed_kuksa(msg.drive.speed * 3.6, 'Vehicle.Tachograph.VehicleSpeed')
            await self.feed_kuksa(msg.drive.steering_angle, 'Vehicle.Chassis.SteeringWheel.Angle')
            await self.feed_kuksa(abs((msg.drive.speed / (3.14 * 0.0005)) - random.random() * 500), 'Vehicle.Powertrain.CombustionEngine.Speed')

        asyncio.run(asyncfunc())

    async def feed_kuksa(self, value, entry):
        # self.get_logger().info(f"to_kuksa: {value}")
        entry = DataEntry(
            entry,
            value=Datapoint(value=value)
        )
        updates = (EntryUpdate(entry, (Field.VALUE,)),)
        try:
            async with VSSClient(self.databroker, self.port, root_certificates=None,
                                 tls_server_name=None) as client:
                await client.set(updates=updates)
        except VSSClientError:
            logging.error("Could not connect to the kuksa.val databroker at %s:%s."
                          " Make sure to set the correct connection details using --address and --port"
                          " and that the kuksa.val databroker is running.", self.databroker, self.port)


def main(args=None):
    rclpy.init(args=args)
    kuksa_ros_provider = KuksaRosProvider()
    rclpy.spin(kuksa_ros_provider)
    kuksa_ros_provider.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
