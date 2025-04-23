#!/usr/bin/env python3
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from taflab_msgs.msg import ControlData, CalibrationData
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Float32
from queue import Queue
import threading
import math
import os

CONFIG_FILE = Path(
    "/home/boat/Desktop/version4/TAFLAB_boatpi_roshumble/src/config.json"
)

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
        self.accelerometer = Vector3()
        self.angular_velocity = Vector3()

    def get_heartbeat(self):
        return {"t": "hb", "id": self.boat_id, "s": self.status, "n": self.notification or ""}

    def get_data_transfer_1(self):
        return {"t": "dt1", "id": self.boat_id, "lt": round(self.lat, 4), "lg": round(self.lng, 4), "w": round(self.wndvn, 2), "tp": round(self.temperature, 1)}

    def get_data_transfer_2(self):
        return {"t": "dt2", "id": self.boat_id, "h": round(self.heading, 2), "ax": round(self.accelerometer.x, 2), "ay": round(self.accelerometer.y, 2), "az": round(self.accelerometer.z, 2)}

class MainLogicNode(Node):
    def __init__(self):
        super().__init__('main_logic_node')
        self.get_logger().info('Main Logic Node Initialized')

        # Load config
        self.config = self.load_config()
        self.get_logger().info(f"Loaded configuration: {self.config}")

        # Boat state
        self.boat = Boat(self.config.get("boat_name", "default_boat"))
        self.in_autonomous_mode = True

        # Control values
        self.rudder_auto = 0.0
        self.sail_auto = 90.0
        self.esc_auto = 0.0
        self.rudder_manual = 0.0
        self.esc_manual = 0.0

        # Publishers
        self.xbee_command_publisher   = self.create_publisher(String, '/xbee_commands', 10)
        self.control_publisher         = self.create_publisher(ControlData, '/boatcontrol', 10)
        self.auto_state_publisher      = self.create_publisher(String, '/auto_init', 10)
        self.target_coords_publisher   = self.create_publisher(NavSatFix, '/boat/target_coordinates', 10)
        self.calibration_publisher     = self.create_publisher(CalibrationData, '/calibration', 10)

        # Subscriptions
        self.create_subscription(String,  '/xbee_data',        self.xbee_data_callback,    10)
        self.create_subscription(Float32, '/as5600_angle',     self.angle_callback,        10)
        self.create_subscription(NavSatFix,'/gps/fix',          self.gps_callback,          10)
        self.create_subscription(Float32, '/witmotion_heading', self.heading_callback,      10)
        self.create_subscription(Imu,      '/imu/data',         self.imu_callback,          10)
        self.create_subscription(Float32, '/rud_cmd_auto',     self.rud_callback_auto,     10)
        self.create_subscription(Float32, '/sail_cmd_auto',    self.sail_callback_auto,    10)
        self.create_subscription(Float32, '/esc_cmd_auto',     self.esc_callback_auto,     10)

        # XBee thread
        self.message_queue = Queue()
        thread = threading.Thread(target=self.xbee_worker)
        thread.daemon = True
        thread.start()

        # Heartbeat timer
        self.queue_message(self.boat.get_heartbeat(), "Sent registration message")
        self.create_timer(6.0, self.queue_heartbeat_message)

        # Publish control at 10 Hz
        self.create_timer(0.1, self.publish_control)

    def load_config(self):
        try:
            with open(CONFIG_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(f"Error loading config ({e}), using defaults.")
            return {"port":"/dev/ttyAMA0","baud_rate":115200,"boat_name":"default_boat"}

    # XBee helpers
    def queue_message(self, message_data, log_message):
        self.message_queue.put((json.dumps(message_data), log_message))

    def xbee_worker(self):
        while True:
            msg, log = self.message_queue.get()
            try:
                self._send_xbee_message(msg, log)
            except Exception as e:
                self.get_logger().error(f"Failed to send {log}: {e}")
            self.message_queue.task_done()

    def queue_heartbeat_message(self):
        self.queue_message(self.boat.get_heartbeat(), "Sent heartbeat message")

    def queue_data_transfer_1(self):
        self.queue_message(self.boat.get_data_transfer_1(), "Sent data transfer 1 message")

    def queue_data_transfer_2(self):
        self.queue_message(self.boat.get_data_transfer_2(), "Sent data transfer 2 message")

    # Central control publisher
    def publish_control(self):
        msg = ControlData()
        if self.in_autonomous_mode:
            msg.servo_rudder = float(self.rudder_auto)
            msg.servo_sail   = float(self.sail_auto)
            msg.esc          = float(self.esc_auto)
        else:
            msg.servo_rudder = float(self.rudder_manual)
            msg.servo_sail   = float(self.sail_auto)  # always from auto subscription
            msg.esc          = float(self.esc_manual)
        self.control_publisher.publish(msg)

    # Command handler (from XBee)
    def handle_backend_command(self, data):
        mode = data.get('md', 'auto')
        if mode == 'mnl':
            # Switch to manual mode and publish state
            self.in_autonomous_mode = False
            self.auto_state_publisher.publish(String(data='False'))

            # Set manual control values from XBee data
            self.rudder_manual = float(data.get('r', 0.0))
            self.esc_manual    = float(data.get('th', 0.0))
            self.boat.status   = "manual"

            # Publish a ControlData message immediately for manual control
            msg = ControlData()
            msg.servo_rudder = self.rudder_manual
            msg.servo_sail   = float(self.sail_auto)  # always sail_cmd_auto
            msg.esc          = self.esc_manual
            self.control_publisher.publish(msg)
        else:
            # Switch to autonomous mode and set target
            self.in_autonomous_mode = True
            self.auto_state_publisher.publish(String(data='True'))
            self.boat.target_lat = data.get('tlat', self.boat.lat)
            self.boat.target_lng = data.get('tlng', self.boat.lng)
            self.boat.status = "autonomous"
            self.publish_target_coordinates()

    # Calibration & misc handlers
    def handle_cal_test(self, data):
        msg = ControlData()
        if 'rudder' in data:
            msg.servo_rudder = float(data['rudder'])
        elif 'sail' in data:
            msg.servo_sail = float(data['sail'])
        elif 'throttle' in data:
            msg.esc = float(data['throttle'])
        self.control_publisher.publish(msg)

    def handle_calibration_data(self, data):
        msg = CalibrationData()
        msg.rudder_min = float(data.get('rm', -90))
        msg.rudder_max = float(data.get('rx',  90))
        msg.sail_min   = float(data.get('sm', -180))
        msg.sail_max   = float(data.get('sx', 180))
        msg.esc_min    = float(data.get('em', -100))
        msg.esc_max    = float(data.get('ex', 100))
        self.calibration_publisher.publish(msg)

    def publish_target_coordinates(self):
        msg = NavSatFix()
        msg.latitude  = self.boat.target_lat
        msg.longitude = self.boat.target_lng
        msg.altitude  = 0.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.target_coords_publisher.publish(msg)

    # Sensor callbacks
    def angle_callback(self, msg):
        angle = msg.data
        self.boat.wndvn = angle
        # update auto sail
        if 15 < angle < 90:
            target = angle - 15
        elif 270 < angle < 345:
            target = angle + 15 - 180
        else:
            target = max(0, min(180, angle / 2))
        self.sail_auto = (target - 90 + 180) % 180

    def gps_callback(self, msg):
        self.boat.lat = 0.0 if math.isnan(msg.latitude) else msg.latitude
        self.boat.lng = 0.0 if math.isnan(msg.longitude) else msg.longitude

    def heading_callback(self, msg):
        self.boat.heading = msg.data

    def imu_callback(self, msg: Imu):
        self.boat.accelerometer   = msg.linear_acceleration
        self.boat.angular_velocity = msg.angular_velocity
        self.get_logger().debug(
            f"IMU → acc=({msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}) "
            f"gyro=({msg.angular_velocity.x:.2f}, {msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f})"
        )

    def xbee_data_callback(self, msg):
        try:
            data = json.loads(msg.data)
            bid = data.get('id')
            if bid not in (self.boat.boat_id, 'all'):
                return
            t = data.get('t')
            if t == 'cmd':
                self.handle_backend_command(data)
            elif t == 'cal_test':
                self.handle_cal_test(data)
            elif t == 'cal':
                self.handle_calibration_data(data)
            elif t == 'req_cal_data':
                cal = self.handle_calibration_request(bid)
                if cal:
                    cm = CalibrationData()
                    cm.rudder_min = float(cal.get('rm', -90))
                    cm.rudder_max = float(cal.get('rx',  90))
                    cm.sail_min   = float(cal.get('sm', -180))
                    cm.sail_max   = float(cal.get('sx', 180))
                    cm.esc_min    = float(cal.get('em', -100))
                    cm.esc_max    = float(cal.get('ex', 100))
                    self.calibration_publisher.publish(cm)
            elif t == 'data_req':
                self.queue_data_transfer_1()
                self.queue_data_transfer_2()
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in XBee data")

    def _send_xbee_message(self, message, log_message):
        self.xbee_command_publisher.publish(String(data=message))
        self.get_logger().info(f"{log_message}: {message}")

    def rud_callback_auto(self, msg):
        self.rudder_auto = msg.data

    def sail_callback_auto(self, msg):
        self.sail_auto = msg.data

    def esc_callback_auto(self, msg):
        self.esc_auto = msg.data


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
