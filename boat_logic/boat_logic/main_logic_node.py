#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from digi.xbee.devices import XBeeDevice
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import MagneticField
import importlib.resources
import math
import uuid

# Constants
BASE_LAT = 37.871866
BASE_LON = -122.311821
MAX_SPEED = 0.0005  # Adjust as needed

# Boat class to manage boat's state and updates
class Boat:
    def __init__(self, boat_id):
        self.boat_id = boat_id
        self.boat_number = 1  # Assuming single boat; adjust if needed
        self.lat = BASE_LAT
        self.lng = BASE_LON
        self.velocity = {"lat": 0.0, "lng": 0.0}
        self.u_wind = 0.0
        self.v_wind = 0.0
        self.temperature = 0.0
        self.chaos = 0.0
        self.status = "station-keeping"
        self.notification = None
        self.wind_angle = 0.0
        # Magnetic field components
        self.magnetic_field_x = 0.0
        self.magnetic_field_y = 0.0
        self.magnetic_field_z = 0.0

        # Target coordinates for autonomous mode
        self.target_lat = None
        self.target_lng = None
        self.start_lat = self.lat
        self.start_lng = self.lng

    def update(self):
        # Update function to adjust boat state
        if self.target_lat is not None and self.target_lng is not None:
            # Compute direction towards the target
            dir_lat = self.target_lat - self.lat
            dir_lng = self.target_lng - self.lng

            # Compute distance to the target
            distance = math.hypot(dir_lat, dir_lng)

            if distance > 0:
                # Normalize the direction vector
                dir_lat /= distance
                dir_lng /= distance

                # Determine movement distance (don't overshoot the target)
                move_dist = min(distance, MAX_SPEED)

                # Set velocity towards the target
                self.velocity["lat"] = dir_lat * move_dist
                self.velocity["lng"] = dir_lng * move_dist

                # Update position based on velocity
                self.lat += self.velocity["lat"]
                self.lng += self.velocity["lng"]

                # Update status with progress percentage
                total_distance = math.hypot(self.target_lat - self.start_lat, self.target_lng - self.start_lng)
                progress = 100 - (distance / total_distance * 100)
                progress = max(0, min(100, progress))  # Clamp progress between 0 and 100
                self.status = f"In Progress ({progress:.1f}%)"

                # Check if the boat has reached the target
                if distance <= MAX_SPEED:
                    # Target reached
                    print(f"{self.boat_id} has reached the target.")
                    self.lat = self.target_lat
                    self.lng = self.target_lng
                    self.target_lat = None
                    self.target_lng = None
                    # Stop the boat
                    self.velocity["lat"] = 0
                    self.velocity["lng"] = 0
                    self.status = "Reached Destination"
                    # Set notification
                    self.notification = {
                        "id": str(uuid.uuid4()),
                        "type": "reached"
                    }
            else:
                # Target reached (distance is zero)
                self.lat = self.target_lat
                self.lng = self.target_lng
                self.target_lat = None
                self.target_lng = None
                self.velocity["lat"] = 0
                self.velocity["lng"] = 0
                self.status = "Reached Destination"
                # Set notification
                self.notification = {
                    "id": str(uuid.uuid4()),
                    "type": "reached"
                }
        else:
            # No target set; boat remains idle
            self.velocity["lat"] = 0
            self.velocity["lng"] = 0
            self.status = "station-keeping"

        # Update chaos (could be based on speed)
        speed = math.hypot(self.velocity["lat"], self.velocity["lng"])
        self.chaos = speed * 1e4  # Scale appropriately

        # Update temperature (could be random fluctuations)
        self.temperature += 0.1  # Simulate slight change

        # Return the current state
        return self.get_state()

    def get_state(self):
        state = {
            "boat_id": self.boat_id,
            "boat_number": self.boat_number,
            "lat": round(self.lat, 6),
            "lng": round(self.lng, 6),
            "u-wind": round(self.u_wind, 2),
            "v-wind": round(self.v_wind, 2),
            "chaos": round(self.chaos, 2),
            "temperature": round(float(self.temperature), 1),
            "velocity": {
                "lat": round(self.velocity["lat"], 2),
                "lng": round(self.velocity["lng"], 2),
            },
            "status": self.status,
            "wind_angle": self.wind_angle,
            "magnetic_field": {
                "x": self.magnetic_field_x,
                "y": self.magnetic_field_y,
                "z": self.magnetic_field_z,
            }
        }
        if self.notification:
            state["notification"] = self.notification
            self.notification = None  # Reset notification after sending
        return state

# Main logic node
class MainLogicNode(Node):
    def __init__(self):
        super().__init__('main_logic_node')
        self.get_logger().info('Main Logic Node Initialized')

        # Load configuration
        config = self.load_config()
        self.xbee_port = config.get('port', '/dev/ttyXbee')
        self.xbee_baud_rate = config.get('baud_rate', 115200)
        self.boat_name = config.get('boat_name', 'Boat_Pi')

        # Initialize Boat instance
        self.boat = Boat(self.boat_name)

        # Initialize control mode
        self.mode = 'manual'  # Default mode

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
        self.magnetometer_subscriber = self.create_subscription(MagneticField, '/magnetic_field', self.magnetometer_callback, 10)

        # Send registration message and start heartbeat timer
        self.send_registration_message()
        self.create_timer(10.0, self.send_heartbeat_message)

        # Send sensor data every 2 seconds
        self.create_timer(2.0, self.send_sensor_data)

    def load_config(self):
        """Load configuration from a JSON file."""
        try:
            with importlib.resources.open_text('boat_logic', 'config.json') as config_file:
                config = json.load(config_file)
                self.get_logger().info("Configuration loaded successfully.")
                return config
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            return {}

    def send_registration_message(self):
        """Send registration message to the backend."""
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
        """Send heartbeat message to the backend."""
        heartbeat_message = json.dumps({
            "type": "heartbeat",
            "boat_name": self.boat_name
        })
        try:
            self.device.send_data_broadcast(heartbeat_message)
            self.get_logger().info(f"Sent heartbeat message: {heartbeat_message}")
        except Exception as e:
            self.get_logger().error(f"Failed to send heartbeat message: {e}")

    def send_sensor_data(self):
        """Send current sensor data to the backend."""
        # Update boat state
        self.boat.update()

        # Prepare sensor data
        sensor_data = {
            "type": "sensor_data",
            "boat_name": self.boat_name,
            **self.boat.get_state()
        }

        sensor_data_json = json.dumps(sensor_data)
        try:
            self.device.send_data_broadcast(sensor_data_json)
            self.get_logger().info(f"Sent sensor data: {sensor_data_json}")
        except Exception as e:
            self.get_logger().error(f"Failed to send sensor data: {e}")

    def xbee_data_receive_callback(self, xbee_message):
        """Process incoming XBee messages from the backend."""
        try:
            data_str = xbee_message.data.decode('utf-8')
            self.get_logger().info(f"Received data: {data_str}")

            # Decode JSON message
            data = json.loads(data_str)

            # Check if the message is intended for this boat
            target_boat_name = data.get('boat_name')
            if target_boat_name not in [self.boat_name, 'all']:
                return  # Message is not for this boat

            # Process different message types
            message_type = data.get("type")
            if message_type == "command":
                self.mode = data.get('command_mode', 'manual')
                self.handle_backend_command(data)
            elif message_type == "data_request":
                self.send_sensor_data()
        except (ValueError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Error decoding data: {e}")
            self.get_logger().error("Invalid data received. Please send valid JSON.")

    def handle_backend_command(self, data):
        """Handle commands received from the backend based on mode."""
        if self.mode == 'manual':
            # In manual mode, directly control actuators
            control_msg = Float32MultiArray()
            control_msg.data = [
                data.get('rudder_angle', 0),
                data.get('sail_angle', 0),
                data.get('throttle', 0)
            ]
            self.control_publisher.publish(control_msg)
            self.get_logger().info('Published manual control commands to /boatcontrol')
        elif self.mode == 'autonomous':
            # Set target coordinates for autonomous navigation
            self.boat.target_lat = data.get('target_lat')
            self.boat.target_lng = data.get('target_lng')
            self.boat.start_lat = self.boat.lat
            self.boat.start_lng = self.boat.lng
            self.boat.status = "In Progress (0%)"
            self.get_logger().info(f"Autonomous mode: Navigating to ({self.boat.target_lat}, {self.boat.target_lng})")

    def magnetometer_callback(self, msg):
        """Process data from the magnetometer."""
        # Store all magnetic field components
        self.boat.magnetic_field_x = msg.magnetic_field.x
        self.boat.magnetic_field_y = msg.magnetic_field.y
        self.boat.magnetic_field_z = msg.magnetic_field.z

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
