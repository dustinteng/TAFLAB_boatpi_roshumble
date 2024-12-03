#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from digi.xbee.devices import XBeeDevice
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import NavSatFix
import math

class Boat:
    def __init__(self, boat_id):
        self.boat_id = boat_id
        self.lat = 0.0
        self.lng = 0.0
        self.wndvn = 0.0
        self.temperature = 0.0
        self.status = "station-keeping"
        self.notification = None
        self.target_lat = None
        self.target_lng = None

    def get_data_transfer(self):
        return {
            "t": "dt",
            "id": self.boat_id,
            "lat": round(self.lat, 5),
            "lng": round(self.lng, 5),
            "w": round(self.wndvn, 2),
            "temp": round(self.temperature, 1)
        }

    def get_heartbeat(self):
        return {
            "t": "hb",
            "id": self.boat_id,
            "s": self.status,
            "n": self.notification or ""
        }

class MainLogicNode(Node):
    def __init__(self):
        super().__init__('main_logic_node')
        self.get_logger().info('Main Logic Node Initialized')

        config = {
            "port": "/dev/ttyXbee",
            "baud_rate": 115200,
            "boat_id": "Boat1"
        }
        self.xbee_port = config.get('port', '/dev/ttyXbee')
        self.xbee_baud_rate = config.get('baud_rate', 115200)
        self.boat_id = config.get('boat_id', 'Boat_Pi')

        self.boat = Boat(self.boat_id)

        try:
            self.device = XBeeDevice(self.xbee_port, self.xbee_baud_rate)
            self.device.open()
            self.device.add_data_received_callback(self.xbee_data_receive_callback)
            self.get_logger().info(f"XBee device opened on {self.xbee_port}")
        except Exception as e:
            self.get_logger().error(f'Failed to open XBee device: {e}')
            rclpy.shutdown()
            return

        self.control_publisher = self.create_publisher(Float32MultiArray, '/boatcontrol', 10)
        self.create_subscription(Float32, '/as5600_angle', self.angle_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)

        self.send_registration_message()
        self.create_timer(10.0, self.send_heartbeat_message)
        self.create_timer(2.0, self.send_data_transfer)

    def send_registration_message(self):
        message = json.dumps({"t": "reg", "id": self.boat_id})
        self._send_xbee_message(message, "Sent registration message")

    def send_heartbeat_message(self):
        message = json.dumps(self.boat.get_heartbeat())
        self._send_xbee_message(message, "Sent heartbeat message")

    def send_data_transfer(self):
        message = json.dumps(self.boat.get_data_transfer())
        self._send_xbee_message(message, "Sent data transfer message")

    def angle_callback(self, msg):
        self.boat.wndvn = msg.data

    def gps_callback(self, msg):
        self.boat.lat = msg.latitude if not math.isnan(msg.latitude) else 0.0
        self.boat.lng = msg.longitude if not math.isnan(msg.longitude) else 0.0

    def xbee_data_receive_callback(self, xbee_message):
        try:
            data_str = xbee_message.data.decode('utf-8')
            self.get_logger().info(f"Received data: {data_str}")
            data = json.loads(data_str)

            # Ensure message is intended for this boat or all boats
            target_boat_id = data.get('id')
            if target_boat_id not in [self.boat_id, 'all']:
                self.get_logger().info(f"Ignored message for boat ID {target_boat_id}")
                return

            # Process command message type
            if data.get("t") == "cmd":
                self.handle_backend_command(data)
            else:
                self.get_logger().warning(f"Unhandled message type: {data.get('t')}")
        except (ValueError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Error decoding data: {e}")

    def handle_backend_command(self, data):
        # Determine mode from the incoming data
        mode = data.get('md', 'auto')  # Default to autonomous if 'md' key is missing

        if mode == 'mnl':
            # Manual mode
            control_msg = Float32MultiArray()
            control_msg.data = [
                float(data.get('r', 0.0)),   # Rudder angle
                float(data.get('s', 0.0)),   # Sail angle
                float(data.get('th', 0.0))   # Throttle
            ]
            self.control_publisher.publish(control_msg)
            self.boat.status = "manual control"
            self.get_logger().info("Manual mode: Control command sent.")
            
        elif mode == 'auto':
            # Autonomous mode
            self.boat.target_lat = data.get('tlat', self.boat.lat)
            self.boat.target_lng = data.get('tlng', self.boat.lng)
            self.boat.status = "autonomous navigation"
            self.get_logger().info("Autonomous mode: Target coordinates updated.")
            
        else:
            # Handle any unknown mode
            self.get_logger().warning(f"Unknown mode '{mode}' received. Ignoring command.")

        # Log mode for debugging purposes
        self.get_logger().info(f"Mode: {mode}, Status: {self.boat.status}")



    def _send_xbee_message(self, message, log_message):
        try:
            self.device.send_data_broadcast(message)
            self.get_logger().info(f"{log_message}: {message}")
        except Exception as e:
            self.get_logger().error(f"Failed to send {log_message}: {e}")

    def destroy_node(self):
        super().destroy_node()
        if self.device.is_open():
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
