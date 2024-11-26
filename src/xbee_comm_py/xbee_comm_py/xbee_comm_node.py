#!/usr/bin/env python3

import json
import random
import rclpy
from rclpy.node import Node
from digi.xbee.devices import XBeeDevice
import os
import importlib.resources

class XBeeCommNode(Node):
    def __init__(self):
        super().__init__('xbee_comm_node')

        # Load configuration from file
        config = self.load_config()
        self.port = config.get('port', '/dev/ttyXbee')
        self.baud_rate = config.get('baud_rate', 115200)
        self.boat_name = config.get('boat_name', 'MyBoat')

        # Initialize XBee device
        try:
            self.device = XBeeDevice(self.port, self.baud_rate)
            self.device.open()
            self.get_logger().info(f"Opened XBee device on {self.port} at {self.baud_rate} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open XBee device: {e}")
            rclpy.shutdown()
            return

        # Set up data received callback
        self.device.add_data_received_callback(self.data_receive_callback)

        # Send registration message
        self.send_registration_message()

        # Start heartbeat timer
        self.create_timer(10.0, self.send_heartbeat_message)

        self.get_logger().info("XBeeCommNode is up and running.")

    def load_config(self):
        try:
            with importlib.resources.open_text('xbee_comm_py', 'config.json') as config_file:
                config = json.load(config_file)
                self.get_logger().info("Configuration loaded successfully.")
                return config
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            return {}

    def send_registration_message(self):
        registration_message = json.dumps({
            "type": "registration",
            "boat_name": self.boat_name
        })
        try:
            self.device.send_data_broadcast(registration_message)
            self.get_logger().info(f"Sent registration message: {registration_message}")
        except Exception as e:
            self.get_logger().error(f"Failed to send registration message: {e}")

    def send_heartbeat_message(self):
        heartbeat_message = json.dumps({
            "type": "heartbeat",
            "boat_name": self.boat_name
        })
        try:
            self.device.send_data_broadcast(heartbeat_message)
            self.get_logger().info(f"Sent heartbeat message: {heartbeat_message}")
        except Exception as e:
            self.get_logger().error(f"Failed to send heartbeat message: {e}")

    def data_receive_callback(self, xbee_message):
        try:
            data_str = xbee_message.data.decode('utf-8')
            self.get_logger().info(f"Received data: {data_str}")

            # Decode the JSON message
            data = json.loads(data_str)

            # Check if the message is intended for this boat
            target_boat_name = data.get('boat_name')
            if target_boat_name not in [self.boat_name, 'all']:
                # Message is not for this boat
                return

            # Print out the received data
            self.get_logger().info(f"Data received from backend: {data}")

        except (ValueError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Error decoding data: {e}")
            self.get_logger().error("Invalid data received. Please send valid JSON.")

    def destroy_node(self):
        super().destroy_node()
        if self.device is not None and self.device.is_open():
            self.device.close()
            self.get_logger().info("Closed XBee device.")

def main(args=None):
    rclpy.init(args=args)
    node = XBeeCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down XBeeCommNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
