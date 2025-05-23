#!/usr/bin/env python3
from pathlib import Path
import json
import rclpy
from rclpy.node import Node
from digi.xbee.devices import XBeeDevice
from std_msgs.msg import String
import traceback

CONFIG_FILE = Path("/home/boat/Desktop/python/TAFLAB_boatpi_roshumble/src/config.json")

class XBeeCommunicationNode(Node):
    def __init__(self):
        super().__init__('xbee_communication_node')
        self.get_logger().info('XBee Communication Node Initialized')

        # Load configuration
        self.config = self.load_config()
        self.xbee_port = self.config.get('xbee_port', '/dev/ttyXbee')
        self.xbee_baud_rate = self.config.get('xbee_baud_rate', 115200)
        self.device = None

        # Initialize XBee device
        try:
            self.device = XBeeDevice(self.xbee_port, self.xbee_baud_rate)
            self.device.open()
            self.device.add_data_received_callback(self.xbee_data_receive_callback)
            self.get_logger().info(f"XBee device opened on {self.xbee_port}")
        except Exception as e:
            self.get_logger().error(f'Failed to open XBee device: {e}')
            rclpy.shutdown()
            return

        # Publisher for incoming XBee data
        self.xbee_data_publisher = self.create_publisher(String, '/xbee_data', 10)

        # Subscriber to listen to commands from the main logic node
        self.create_subscription(String, '/xbee_commands', self.xbee_command_callback, 10)

    def load_config(self):
        """Load configuration from the specified JSON file."""
        try:
            if CONFIG_FILE.exists():
                with open(CONFIG_FILE, 'r') as file:
                    config = json.load(file)
                    self.get_logger().info(f"Loaded configuration from {CONFIG_FILE}")
                    return config
            else:
                self.get_logger().error(f"Configuration file not found: {CONFIG_FILE}")
                rclpy.shutdown()
                return {}
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            traceback.print_exc()
            rclpy.shutdown()
            return {}

    def xbee_data_receive_callback(self, xbee_message):
        try:
            data_str = xbee_message.data.decode('utf-8')
            self.get_logger().info(f"Received data from XBee: {data_str}")
            # Publish incoming XBee data on a ROS topic
            self.xbee_data_publisher.publish(String(data=data_str))
        except Exception as e:
            self.get_logger().error(f"Error in xbee_data_receive_callback: {e}")
            traceback.print_exc()

    def xbee_command_callback(self, msg):
        """Callback to handle commands from the main logic node."""
        try:
            self.device.send_data_broadcast(msg.data)
            self.get_logger().info(f"Sent command via XBee: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command via XBee: {e}")

    def destroy_node(self):
        """Ensure XBee device is closed on shutdown."""
        super().destroy_node()
        if self.device and self.device.is_open():
            self.device.close()
            self.get_logger().info("Closed XBee device.")

def main(args=None):
    rclpy.init(args=args)
    node = XBeeCommunicationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down XBeeCommunicationNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
