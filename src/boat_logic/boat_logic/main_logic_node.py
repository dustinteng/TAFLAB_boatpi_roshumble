import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from taflab_msgs.msg import ControlData, CalibrationData  # Import CalibrationData
from sensor_msgs.msg import NavSatFix, MagneticField
from std_msgs.msg import String, Float32, Bool
from queue import Queue
import threading
import math
import os

CONFIG_FILE = Path("/home/boat/Desktop/python/TAFLAB_boatpi_roshumble/src/config.json")


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
        self.heading = 0.0


    def get_heartbeat(self):
        return {
            "t": "hb",
            "id": self.boat_id,
            "s": self.status,
            "n": self.notification or ""
        }

    def get_data_transfer_1(self):
        return {
            "t": "dt1",
            "id": self.boat_id,
            "lt": round(self.lat, 4),
            "lg": round(self.lng, 4)
        }

    def get_data_transfer_2(self):
        return {
            "t": "dt2",
            "id": self.boat_id,
            "h": round(self.heading,2),
            "w": round(self.wndvn, 2),
            "tp": round(self.temperature, 1)
        }

class MainLogicNode(Node):
    def __init__(self):
        super().__init__('main_logic_node')
        self.get_logger().info('Main Logic Node Initialized')
        # Load configuration
        self.config = self.load_config()
        self.get_logger().info(f"Loaded configuration: {self.config}")

        # Boat configuration
        self.boat = Boat(self.config.get("boat_name", "default_boat"))
        self.in_autonomous_mode = False

        # Latest autonomous control values
        self.rudder_auto = 0.0
        self.sail_auto = 0.0
        self.esc_auto = 0.0

        # Publisher to send commands to XBeeCommunicationNode
        self.xbee_command_publisher = self.create_publisher(String, '/xbee_commands', 10)

        # Publisher for boat control commands and target coordinates
        self.control_publisher = self.create_publisher(ControlData, '/boatcontrol', 10)
        self.auto_state_publisher = self.create_publisher(String, '/auto_init', 10)
        self.target_coords_publisher = self.create_publisher(NavSatFix, '/boat/target_coordinates', 10)
        
        # Publisher for calibration data
        self.calibration_publisher = self.create_publisher(CalibrationData, '/calibration', 10)

        # Subscriptions
        self.create_subscription(String, '/xbee_data', self.xbee_data_callback, 10)
        self.create_subscription(Float32, '/as5600_angle', self.angle_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        # self.create_subscription(MagneticField, '/magnetic_field', self.magnetometer_callback, 10)
        self.create_subscription(Float32, '/witmotion_heading', self.heading_callback, 10)
        self.create_subscription(Float32, '/rud_cmd_auto', self.rud_callback_auto, 10)
        self.create_subscription(Float32, '/sail_cmd_auto', self.sail_callback_auto, 10)
        self.create_subscription(Float32, '/esc_cmd_auto', self.esc_callback_auto, 10)

        # Message queue and worker thread for XBee communication
        self.message_queue = Queue()
        self.worker_thread = threading.Thread(target=self.xbee_worker)
        self.worker_thread.daemon = True
        self.worker_thread.start()

        # Send registration message
        self.queue_message(self.boat.get_heartbeat(), "Sent registration message")
        
        # Timers for periodic messages
        self.create_timer(10.0, self.queue_heartbeat_message)
        self.create_timer(2.0, self.queue_data_transfer_1)
        self.create_timer(2.0, self.queue_data_transfer_2)

    def load_config(self):
        """Load configuration from the JSON file."""
        try:
            with open(CONFIG_FILE, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            self.get_logger().error(f"Configuration file not found at {CONFIG_FILE}. Using default values.")
            return {"port": "/dev/ttyAMA0", "baud_rate": 115200, "boat_name": "default_boat"}
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding JSON: {e}. Using default values.")
            return {"port": "/dev/ttyAMA0", "baud_rate": 115200, "boat_name": "default_boat"}
        
    # Queue and worker for XBee communication
    def queue_message(self, message_data, log_message):
        self.message_queue.put((json.dumps(message_data), log_message))

    def xbee_worker(self):
        while True:
            message, log_message = self.message_queue.get()
            try:
                self._send_xbee_message(message, log_message)
            except Exception as e:
                self.get_logger().error(f"Failed to send {log_message}: {e}")
            self.message_queue.task_done()

    # Timed queue functions
    def queue_heartbeat_message(self):
        self.queue_message(self.boat.get_heartbeat(), "Sent heartbeat message")

    def queue_data_transfer_1(self):
        self.queue_message(self.boat.get_data_transfer_1(), "Sent data transfer 1 message")

    def queue_data_transfer_2(self):
        self.queue_message(self.boat.get_data_transfer_2(), "Sent data transfer 2 message")

    def handle_backend_command(self, data):
        mode = data.get('md', 'auto')
        self.get_logger().info(f"Handling backend command in mode: {mode}")

        if mode == 'mnl':
            self.in_autonomous_mode = False
            self.auto_state_publisher.publish(String(data=str(self.in_autonomous_mode)))
            control_msg = ControlData()
            control_msg.servo_rudder = float(data.get('r', 0.0))
            control_msg.servo_sail = float(data.get('s', 0.0))
            control_msg.esc = float(data.get('th', 0.0))
            self.control_publisher.publish(control_msg)
            self.boat.status = "manual"
            self.get_logger().info("Manual mode: Control command sent.")

        elif mode == 'auto':
            self.in_autonomous_mode = True
            self.auto_state_publisher.publish(String(data=str(self.in_autonomous_mode)))
            self.boat.target_lat = data.get('tlat', self.boat.lat)
            self.boat.target_lng = data.get('tlng', self.boat.lng)
            self.boat.status = "autonomous"
            self.publish_target_coordinates()
            self.get_logger().info("Autonomous mode: Target coordinates updated.")
        else:
            self.get_logger().warning(f"Unknown mode '{mode}' received. Ignoring command.")

    def handle_cal_test(self, data):
        if "rudder" in data:
            value_type = "rudder"
            value = float(data["rudder"])
        elif "sail" in data:
            value_type = "sail"
            value = float(data["sail"])
        elif "throttle" in data:
            value_type = "throttle"
            value = float(data["throttle"])
        else:
            self.get_logger().warning("Unknown calibration value type received.")
            return

        control_msg = ControlData()
        if value_type == "rudder":
            control_msg.servo_rudder = value
        elif value_type == "sail":
            control_msg.servo_sail = value
        elif value_type == "throttle":
            control_msg.esc = value

        self.control_publisher.publish(control_msg)
        self.get_logger().info(f"Calibration test command sent: {value_type} = {value}")

    def handle_calibration_data(self, data):
        # Prepare calibration data message for publishing
        calibration_msg = CalibrationData()
        calibration_msg.rudder_min = float(data.get('rm', -90))
        calibration_msg.rudder_max = float(data.get('rx', 90))
        calibration_msg.sail_min = float(data.get('sm', -180))
        calibration_msg.sail_max = float(data.get('sx', 180))
        calibration_msg.esc_min = float(data.get('em', -100))
        calibration_msg.esc_max = float(data.get('ex', 100))
        
        # Publish calibration data on the /calibration topic
        self.calibration_publisher.publish(calibration_msg)
        self.get_logger().info("Calibration data published to /calibration topic")

    # New function to handle requests for calibration data
    def handle_calibration_request(self, boat_id):
        # Implement logic to retrieve calibration data from boat_control_node
        file_path = f'src/calibration_data.json'  # Adjust path as needed
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                calibration_data = json.load(f)
            self.get_logger().info(f"Calibration data for {boat_id} retrieved successfully.")
            return calibration_data
        else:
            self.get_logger().warning(f"Calibration data for {boat_id} not found.")
            return None

    def publish_autonomous_control(self):
        if self.in_autonomous_mode:
            control_msg = ControlData()
            control_msg.servo_rudder = self.rudder_auto
            control_msg.servo_sail = self.sail_auto
            control_msg.esc = self.esc_auto
            self.control_publisher.publish(control_msg)
            self.get_logger().info(f"Autonomous control command sent: Rudder={self.rudder_auto}, Sail={self.sail_auto}, ESC={self.esc_auto}")

    def publish_target_coordinates(self):
        target_msg = NavSatFix()
        target_msg.latitude = self.boat.target_lat
        target_msg.longitude = self.boat.target_lng
        target_msg.altitude = 0.0
        target_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.target_coords_publisher.publish(target_msg)
        self.get_logger().info(f"Published target coordinates: lat={self.boat.target_lat}, lng={self.boat.target_lng}")

    def angle_callback(self, msg):
        self.boat.wndvn = msg.data

    def gps_callback(self, msg):
        self.boat.lat = msg.latitude if not math.isnan(msg.latitude) else 0.0
        self.boat.lng = msg.longitude if not math.isnan(msg.longitude) else 0.0

    def heading_callback(self, msg):
        self.boat.heading = msg.data
        self.get_logger().info(f"Updated heading: {self.boat.heading}")

    # def magnetometer_callback(self, msg):
    #     self.boat.magnetic_field_x = msg.magnetic_field.x
    #     self.boat.magnetic_field_y = msg.magnetic_field.y
    #     self.boat.magnetic_field_z = msg.magnetic_field.z
    

    def xbee_data_callback(self, msg):
        try:
            data = json.loads(msg.data)
            target_boat_id = data.get('id')
            if target_boat_id not in [self.boat.boat_id, 'all']:
                self.get_logger().info(f"Ignored message for boat ID {target_boat_id}")
                return

            msg_type = data.get("t")
            if msg_type == "cmd":
                self.handle_backend_command(data)
            elif msg_type == "cal_test":
                self.handle_cal_test(data)
            elif msg_type == "cal":
                self.handle_calibration_data(data)
            elif msg_type == "req_cal_data":
                calibration_data = self.handle_calibration_request(target_boat_id)
                # Emit calibration data to the backend
                self.calibration_publisher.publish(calibration_data)
            else:
                self.get_logger().warning(f"Unhandled message type: {msg_type}")
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding data: {e}")

    def _send_xbee_message(self, message, log_message):
        self.xbee_command_publisher.publish(String(data=message))
        self.get_logger().info(f"{log_message}: {message}")

    # Autonomous mode callbacks to update control data
    def rud_callback_auto(self, msg):
        self.rudder_auto = msg.data
        self.publish_autonomous_control()

    def sail_callback_auto(self, msg):
        self.sail_auto = msg.data
        self.publish_autonomous_control()

    def esc_callback_auto(self, msg):
        self.esc_auto = msg.data
        self.publish_autonomous_control()

def main(args=None):
    rclpy.init(args=args)
    node = MainLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MainLogicNode.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
