#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from digi.xbee.devices import XBeeDevice
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import NavSatFix  # Removed MagneticField import
import math

# Boat class to manage boat's state and updates
class Boat:
    def __init__(self, boat_id):
        self.boat_id = boat_id
        self.lat = 0.0
        self.lng = 0.0
        self.wndvn = 0.0
        self.temperature = 0.0
        self.status = "station-keeping"
        self.notification = None
        # Removed magnetometer attributes

    def get_data_transfer(self):
        """Generate compact data transfer payload."""
        return {
            "t": "dt",   # type: "data_transfer"
            "id": self.boat_id,
            "lat": round(self.lat, 5),
            "lng": round(self.lng, 5),
            "w": round(self.wndvn, 2),
            "temp": round(self.temperature, 1)
            # Magnetometer data omitted to reduce size
        }

    def get_heartbeat(self):
        """Generate compact heartbeat payload."""
        return {
            "t": "hb",  # type: "heartbeat"
            "id": self.boat_id,
            "s": self.status,
            "n": self.notification or ""
        }

# Main logic node
class MainLogicNode(Node):
    def __init__(self):
        super().__init__('main_logic_node')
        self.get_logger().info('Main Logic Node Initialized')

        # Inline configuration
        config = {
            "port": "/dev/ttyXbee",
            "baud_rate": 115200,
            "boat_id": "Boat1"
        }
        self.get_logger().info(f'Config loaded: {config}')
        self.xbee_port = config.get('port', '/dev/ttyXbee')
        self.xbee_baud_rate = config.get('baud_rate', 115200)
        self.boat_id = config.get('boat_id', 'Boat_Pi')

        # Initialize Boat instance
        self.boat = Boat(self.boat_id)

        # Initialize XBee device
        try:
            self.device = XBeeDevice(self.xbee_port, self.xbee_baud_rate)
            self.device.open()
            self.device.add_data_received_callback(self.xbee_data_receive_callback)
            self.get_logger().info(f"XBee device opened on {self.xbee_port} at {self.xbee_baud_rate} baud.")
        except Exception as e:
            self.get_logger().error(f'Failed to open XBee device: {e}')
            rclpy.shutdown()
            return

        # Set up publishers and subscribers
        self.control_publisher = self.create_publisher(Float32MultiArray, '/boatcontrol', 10)
        self.create_subscription(Float32, '/as5600_angle', self.angle_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        # Removed magnetometer subscription to reduce data size

        # Send registration message and start heartbeat timer
        self.send_registration_message()
        self.create_timer(10.0, self.send_heartbeat_message)

        # Send data transfer every 2 seconds (optional)
        # self.create_timer(2.0, self.send_data_transfer)

    def send_registration_message(self):
        """Send registration message to the backend."""
        registration_message = json.dumps({
            "t": "reg",
            "id": self.boat_id
        })
        try:
            self.device.send_data_broadcast(registration_message)
            self.get_logger().info(f"Sent registration message: {registration_message}")
        except Exception as e:
            self.get_logger().error(f"Failed to send registration message: {e}")

    def send_heartbeat_message(self):
        """Send heartbeat message to the backend."""
        heartbeat_message = json.dumps(self.boat.get_heartbeat())
        try:
            self.device.send_data_broadcast(heartbeat_message)
            self.get_logger().info(f"Sent heartbeat message: {heartbeat_message}")
        except Exception as e:
            self.get_logger().error(f"Failed to send heartbeat message: {e}")

    def send_data_transfer(self):
        """Send current sensor data to the backend."""
        data_transfer_message = json.dumps(self.boat.get_data_transfer())
        try:
            self.device.send_data_broadcast(data_transfer_message)
            self.get_logger().info(f"Sent data transfer message: {data_transfer_message}")
        except Exception as e:
            self.get_logger().error(f"Failed to send data transfer message: {e}")

    def angle_callback(self, msg):
        """Update the boat's wind angle from /as5600_angle."""
        self.boat.wndvn = msg.data

    def gps_callback(self, msg):
        """Update the boat's latitude and longitude from /gps/fix."""
        self.boat.lat = msg.latitude if not math.isnan(msg.latitude) else 0.0
        self.boat.lng = msg.longitude if not math.isnan(msg.longitude) else 0.0

    def xbee_data_receive_callback(self, xbee_message):
        """Process incoming XBee messages from the backend."""
        try:
            data_str = xbee_message.data.decode('utf-8')
            self.get_logger().info(f"Received data: {data_str}")

            # Decode JSON message
            data = json.loads(data_str)

            # Check if the message is intended for this boat
            target_boat_id = data.get('id')
            if target_boat_id not in [self.boat_id, 'all']:
                return  # Message is not for this boat

            # Process different message types
            message_type = data.get("t")
            if message_type == "cmd":
                self.handle_backend_command(data)
            elif message_type == "dr":
                self.send_data_transfer()
        except (ValueError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Error decoding data: {e}")
            self.get_logger().error("Invalid data received. Please send valid JSON.")

    def handle_backend_command(self, data):
        """Handle control commands from backend."""
        control_msg = Float32MultiArray()
        control_msg.data = [
            float(data.get('r', 0.0)),   # Abbreviated 'rudd' to 'r'
            float(data.get('s', 0.0)),   # Abbreviated 'sail' to 's'
            float(data.get('th', 0.0))   # Abbreviated 'throt' to 'th'
        ]
        self.control_publisher.publish(control_msg)
        self.get_logger().info(f"Published control commands to /boatcontrol")

        # Update target lat/long for autonomous mode
        self.boat.target_lat = data.get('tlat', self.boat.lat)
        self.boat.target_lng = data.get('tlng', self.boat.lng)
        self.boat.status = data.get("cmd", "station-keeping")

    def destroy_node(self):
        """Ensure XBee device is closed on shutdown."""
        super().destroy_node()
        if self.device is not None and self.device.is_open():
            self.device.close()
            self.get_logger().info("Closed XBee device.")

def main(args=None):
    rclpy.init(args=args)
    node = MainLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MainLogicNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
