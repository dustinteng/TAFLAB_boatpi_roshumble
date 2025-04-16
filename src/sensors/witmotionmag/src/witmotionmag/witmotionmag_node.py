import rclpy
import os
import json
import numpy as np  # For matrix operations
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import MagneticField, Imu  # ⬅️ added Imu import
import math
import platform

import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver

config_file = "/home/boat/Desktop/python/TAFLAB_boatpi_roshumble/src/config.json"

class WitMotionMagNode(Node):
    
    def __init__(self):
        super().__init__('witmotionmag_node')

        # ROS2 Publishers
        self.angle_publisher = self.create_publisher(Float32, 'witmotion_heading', 10)
        self.mag_publisher   = self.create_publisher(MagneticField, 'magnetic_field', 10)
        self.imu_publisher   = self.create_publisher(Imu, 'imu/data', 10)  # ⬅️ added IMU publisher

        # Timer (publishes data at 10Hz)
        self.timer = self.create_timer(0.1, self.updateData)  # Every 0.1s

        # magnetometer value
        self.mag_port = "/dev/ttyUSB0"
        
        # Store latest magnetometer readings
        self.last_mx = 0.0
        self.last_my = 0.0
        self.last_mz = 0.0
        self.last_heading_degrees = 0.0

        # Store latest IMU readings  ⬅️ added
        self.last_ax = 0.0
        self.last_ay = 0.0
        self.last_az = 0.0
        self.last_gx = 0.0
        self.last_gy = 0.0
        self.last_gz = 0.0

        # We'll keep the most recent corrected mag vector in a class attribute:
        self.corrected_mag_vector = np.array([0.0, 0.0, 0.0])  

        # Calibration file path
        self.config_file = "/home/boat/Desktop/python/TAFLAB_boatpi_roshumble/src/config.json"

        # Default calibration values
        self.mag_offset = np.zeros(3)  # Offset vector [offset_x, offset_y, offset_z]
        self.mag_matrix = np.eye(3)    # 3×3 identity matrix as default scaling
        self.heading_offset = 0.0      # Adjustable heading correction

        # Load calibration values
        self.load_calibration()

        # Initialize the device
        self.device = deviceModel.DeviceModel(
            "myJY901",
            WitProtocolResolver(),
            JY901SDataProcessor(),
            "51_0"
        )

        # Set serial port based on OS
        if platform.system().lower() == 'linux':
            self.device.serialConfig.portName = self.mag_port
            self.get_logger().info(f"portttttttt {self.mag_port}.")
        else:
            self.device.serialConfig.portName = "/dev/ttyUSB1"
            print("USB0 not getting the mag_port from config")
        
        self.device.serialConfig.baud = 9600
        self.device.openDevice()

        # Register update callback (fires whenever new sensor data is available)
        self.device.dataProcessor.onVarChanged.append(self.onUpdate)

    def get_mag_port(self):
        try:
            with open(config_file, 'r') as file:
                config = json.load(file)
                mag_port = config.get("mag_port", "/dev/ttyUSB1")
                return mag_port
        except FileNotFoundError:
            print("Error: Config file not found.")
            return None
        except json.JSONDecodeError:
            print("Error: Failed to decode JSON.")
            return None

    def load_calibration(self):
        """
        Loads calibration data from 'config.json'.
        If the file doesn't exist, it creates a default one.
        """
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    data = json.load(f)
                    self.mag_offset = np.array(data.get("mag_offset", [0.0, 0.0, 0.0]))
                    self.mag_matrix = np.array(data.get("mag_matrix", np.eye(3).tolist()))
                    self.heading_offset = data.get("heading_offset", 0.0)

                    self.get_logger().info(f"Loaded calibration values from: {self.config_file}.")
            except Exception as e:
                self.get_logger().error(f"Error loading calibration file: {e}")
        else:
            self.save_calibration()
            self.get_logger().info(f"Created default: {self.config_file}.")

    def save_calibration(self):
        """
        Saves the current calibration values to 'config.json' without altering existing settings.
        """
        try:
            with open(self.config_file, 'r') as f:
                config_data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            config_data = {}

        config_data.update({
            "mag_offset": self.mag_offset.tolist(),
            "mag_matrix": self.mag_matrix.tolist(),
            "heading_offset": self.heading_offset
        })

        with open(self.config_file, 'w') as f:
            json.dump(config_data, f, indent=4)

        self.get_logger().info(f"Saved calibration values to: {self.config_file}.")

    def onUpdate(self, deviceModel):
        """
        Callback function triggered whenever new sensor data arrives.
        Stores the latest magnetometer readings, heading, and IMU.
        """
        # --- MAGNETOMETER (mG → T) ---
        raw_mx = float(deviceModel.getDeviceData("magX") or 0.0) * 1e-2
        raw_my = float(deviceModel.getDeviceData("magY") or 0.0) * 1e-2
        raw_mz = float(deviceModel.getDeviceData("magZ") or 0.0) * 1e-2
        raw_mag_vector = np.array([raw_mx, raw_my, raw_mz])
        self.corrected_mag_vector = np.dot(self.mag_matrix, (raw_mag_vector + self.mag_offset))
        self.last_mx, self.last_my, self.last_mz = self.corrected_mag_vector

        heading_radians = math.atan2(self.last_my, self.last_mx)
        heading_degrees = heading_radians * 180.0 / math.pi + self.heading_offset
        if heading_degrees < 0:
            heading_degrees += 360.0
        self.last_heading_degrees = heading_degrees

        # --- ACCELEROMETER (g → m/s²) ---  ⬅️ added
        ax_g = float(deviceModel.getDeviceData("accX") or 0.0)
        ay_g = float(deviceModel.getDeviceData("accY") or 0.0)
        az_g = float(deviceModel.getDeviceData("accZ") or 0.0)
        self.last_ax = ax_g * 9.80665
        self.last_ay = ay_g * 9.80665
        self.last_az = az_g * 9.80665

        # --- GYROSCOPE (°/s → rad/s) ---  ⬅️ added
        gx_dps = float(deviceModel.getDeviceData("gyroX") or 0.0)
        gy_dps = float(deviceModel.getDeviceData("gyroY") or 0.0)
        gz_dps = float(deviceModel.getDeviceData("gyroZ") or 0.0)
        self.last_gx = gx_dps * math.pi / 180.0
        self.last_gy = gy_dps * math.pi / 180.0
        self.last_gz = gz_dps * math.pi / 180.0

        self.get_logger().info(f"Heading: {self.last_heading_degrees}")

    def updateData(self):
        """
        Timer callback (10 Hz) to publish the latest magnetometer, heading, and IMU data.
        """
        now = self.get_clock().now().to_msg()

        # 1) Publish MagneticField
        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = "witmotion_link"
        mag_msg.magnetic_field.x = self.last_mx
        mag_msg.magnetic_field.y = self.last_my
        mag_msg.magnetic_field.z = self.last_mz
        mag_msg.magnetic_field_covariance = [0.0] * 9
        self.mag_publisher.publish(mag_msg)

        # 2) Publish Heading
        heading_msg = Float32()
        heading_msg.data = self.last_heading_degrees
        self.angle_publisher.publish(heading_msg)

        # 3) Publish IMU  ⬅️ added
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "witmotion_link"
        imu_msg.orientation_covariance[0] = -1.0  # no orientation
        imu_msg.linear_acceleration.x = self.last_ax
        imu_msg.linear_acceleration.y = self.last_ay
        imu_msg.linear_acceleration.z = self.last_az
        imu_msg.linear_acceleration_covariance = [0.0] * 9
        imu_msg.angular_velocity.x = self.last_gx
        imu_msg.angular_velocity.y = self.last_gy
        imu_msg.angular_velocity.z = self.last_gz
        imu_msg.angular_velocity_covariance = [0.0] * 9
        self.imu_publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WitMotionMagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
