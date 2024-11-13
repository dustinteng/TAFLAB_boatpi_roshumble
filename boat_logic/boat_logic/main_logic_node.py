import json
import rclpy
from rclpy.node import Node
from taflab_msgs.msg import ControlData
from sensor_msgs.msg import NavSatFix, MagneticField
from std_msgs.msg import String, Float32
from queue import Queue
import threading
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
        self.magnetic_field_x = 0.0
        self.magnetic_field_y = 0.0
        self.magnetic_field_z = 0.0

    def get_heartbeat(self):
        return {
            "t": "hb",
            "id": self.boat_id,
            "s": self.status,
            "n": self.notification or ""
        }

    def get_location_data_transfer(self):
        return {
            "t": "dt1",
            "id": self.boat_id,
            "lt": round(self.lat, 4),
            "lg": round(self.lng, 4),
            "w": round(self.wndvn, 2),
            "tp": round(self.temperature, 1)
        }

    def get_magnetic_data_transfer(self):
        return {
            "t": "dt2",
            "id": self.boat_id,
            "mx": round(self.magnetic_field_x, 3),
            "my": round(self.magnetic_field_y, 3),
            "mz": round(self.magnetic_field_z, 3)
        }

class MainLogicNode(Node):
    def __init__(self):
        super().__init__('main_logic_node')
        self.get_logger().info('Main Logic Node Initialized')

        # Boat configuration
        self.boat = Boat("B1")
        self.in_autonomous_mode = False

        # Latest autonomous control values
        self.rudder_auto = 0.0
        self.sail_auto = 0.0
        self.esc_auto = 0.0

        # Publisher to send commands to XBeeCommunicationNode
        self.xbee_command_publisher = self.create_publisher(String, '/xbee_commands', 10)

        # Publisher for boat control commands and target coordinates
        self.control_publisher = self.create_publisher(ControlData, '/boatcontrol', 10)
        self.target_coords_publisher = self.create_publisher(NavSatFix, '/boat/target_coordinates', 10)

        # Subscriptions
        self.create_subscription(String, '/xbee_data', self.xbee_data_callback, 10)
        self.create_subscription(Float32, '/as5600_angle', self.angle_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(MagneticField, '/magnetic_field', self.magnetometer_callback, 10)
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
        self.create_timer(2.0, self.queue_location_data_transfer)
        self.create_timer(2.0, self.queue_magnetic_data_transfer)

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

    def queue_location_data_transfer(self):
        self.queue_message(self.boat.get_location_data_transfer(), "Sent location data transfer message")

    def queue_magnetic_data_transfer(self):
        self.queue_message(self.boat.get_magnetic_data_transfer(), "Sent magnetic data transfer message")

    def handle_backend_command(self, data):
        mode = data.get('md', 'auto')
        self.get_logger().info(f"Handling backend command in mode: {mode}")

        if mode == 'mnl':
            self.in_autonomous_mode = False
            control_msg = ControlData()
            control_msg.servo_rudder = float(data.get('r', 0.0))
            control_msg.servo_sail = float(data.get('s', 0.0))
            control_msg.esc = float(data.get('th', 0.0))
            self.control_publisher.publish(control_msg)
            self.boat.status = "manual"
            self.get_logger().info("Manual mode: Control command sent.")

        elif mode == 'auto':
            self.in_autonomous_mode = True
            self.boat.target_lat = data.get('tlat', self.boat.lat)
            self.boat.target_lng = data.get('tlng', self.boat.lng)
            self.boat.status = "autonomous"
            
            # Publish target coordinates to '/boat/target_coordinates'
            self.publish_target_coordinates()
            self.get_logger().info("Autonomous mode: Target coordinates updated.")
        else:
            self.get_logger().warning(f"Unknown mode '{mode}' received. Ignoring command.")

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

    def publish_autonomous_control(self):
        """Publish the autonomous control commands if in autonomous mode."""
        if self.in_autonomous_mode:
            control_msg = ControlData()
            control_msg.servo_rudder = self.rudder_auto
            control_msg.servo_sail = self.sail_auto
            control_msg.esc = self.esc_auto
            self.control_publisher.publish(control_msg)
            self.get_logger().info(f"Autonomous control command sent: Rudder={self.rudder_auto}, Sail={self.sail_auto}, ESC={self.esc_auto}")

    def publish_target_coordinates(self):
        """Publish the target latitude and longitude as a NavSatFix message."""
        target_msg = NavSatFix()
        target_msg.latitude = self.boat.target_lat
        target_msg.longitude = self.boat.target_lng
        target_msg.altitude = 0.0  # Set altitude to 0 as it's not used here
        target_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.target_coords_publisher.publish(target_msg)
        self.get_logger().info(f"Published target coordinates: lat={self.boat.target_lat}, lng={self.boat.target_lng}")

    # Callback functions
    def angle_callback(self, msg):
        self.boat.wndvn = msg.data

    def gps_callback(self, msg):
        self.boat.lat = msg.latitude if not math.isnan(msg.latitude) else 0.0
        self.boat.lng = msg.longitude if not math.isnan(msg.longitude) else 0.0

    def magnetometer_callback(self, msg):
        self.boat.magnetic_field_x = msg.magnetic_field.x
        self.boat.magnetic_field_y = msg.magnetic_field.y
        self.boat.magnetic_field_z = msg.magnetic_field.z
    
    def xbee_data_callback(self, msg):
        """Process incoming data from XBee."""
        try:
            data = json.loads(msg.data)
            target_boat_id = data.get('id')
            if target_boat_id not in [self.boat.boat_id, 'all']:
                self.get_logger().info(f"Ignored message for boat ID {target_boat_id}")
                return

            if data.get("t") == "cmd":
                self.handle_backend_command(data)
            else:
                self.get_logger().warning(f"Unhandled message type: {data.get('t')}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding data: {e}")

    def _send_xbee_message(self, message, log_message):
        self.xbee_command_publisher.publish(String(data=message))
        self.get_logger().info(f"{log_message}: {message}")

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
