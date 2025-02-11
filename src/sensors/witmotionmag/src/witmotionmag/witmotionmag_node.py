import rclpy
import os
import json
import numpy as np  # For matrix operations
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import MagneticField
import math
import platform

import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver

class WitMotionMagNode(Node):
    
    def __init__(self):
        super().__init__('witmotionmag_node')

        # ROS2 Publishers
        self.angle_publisher = self.create_publisher(Float32, 'witmotion_heading', 10)
        self.mag_publisher = self.create_publisher(MagneticField, 'magnetic_field', 10)

        # Timer (publishes data at 10Hz)
        self.timer = self.create_timer(0.1, self.updateData)  # Every 0.1s

        # Store latest magnetometer readings
        self.last_mx = 0.0
        self.last_my = 0.0
        self.last_mz = 0.0
        self.last_heading_degrees = 0.0

        # We'll keep the most recent corrected mag vector in a class attribute:
        self.corrected_mag_vector = np.array([0.0, 0.0, 0.0])  

        # Calibration file path
        self.calibration_file = "/home/boat/Desktop/python/TAFLAB_boatpi_roshumble/src/config.json"

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
            self.device.serialConfig.portName = "/dev/ttyMag"
        else:
            self.device.serialConfig.portName = "/dev/ttyUSB0"

        self.device.serialConfig.baud = 9600
        self.device.openDevice()

        # Register update callback (fires whenever new sensor data is available)
        self.device.dataProcessor.onVarChanged.append(self.onUpdate)

    def load_calibration(self):
        """
        Loads calibration data from 'wit_calibration.json'.
        If the file doesn't exist, it creates a default one.
        """
        if os.path.exists(self.calibration_file):
            try:
                with open(self.calibration_file, 'r') as f:
                    data = json.load(f)
                    self.mag_offset = np.array(data.get("mag_offset", [0.0, 0.0, 0.0]))
                    self.mag_matrix = np.array(data.get("mag_matrix", np.eye(3).tolist()))
                    self.heading_offset = data.get("heading_offset", 0.0)

                    self.get_logger().info(f"Loaded calibration values from: {self.calibration_file}.")
            except Exception as e:
                self.get_logger().error(f"Error loading calibration file: {e}")
        else:
            self.save_calibration()
            self.get_logger().info(f"Created default: {self.calibration_file}.")


    def save_calibration(self):
        """
        Saves the current calibration values to 'config.json' without altering existing settings.
        """
        # Load existing configuration if the file exists
        try:
            with open(self.calibration_file, 'r') as f:
                config_data = json.load(f)  # Load existing JSON data
        except (FileNotFoundError, json.JSONDecodeError):
            config_data = {}  # Start with an empty dict if the file doesn't exist or is corrupted

        # Update only the calibration values
        config_data.update({
            "mag_offset": self.mag_offset.tolist(),
            "mag_matrix": self.mag_matrix.tolist(),
            "heading_offset": self.heading_offset
        })

        # Save back to the same file
        with open(self.calibration_file, 'w') as f:
            json.dump(config_data, f, indent=4)

        self.get_logger().info(f"Saved calibration values to: {self.calibration_file}.")


    def onUpdate(self, deviceModel):
        """
        Callback function triggered whenever new sensor data arrives.
        Stores the latest magnetometer readings and heading.
        """
        # Convert raw milligauss (mG) to Tesla (T) by multiplying by 1e-7
        raw_mx = float(deviceModel.getDeviceData("magX") or 0.0) * 1e-2
        raw_my = float(deviceModel.getDeviceData("magY") or 0.0) * 1e-2
        raw_mz = float(deviceModel.getDeviceData("magZ") or 0.0) * 1e-2

        # Convert raw vector to numpy array
        raw_mag_vector = np.array([raw_mx, raw_my, raw_mz])

        # Apply full calibration: (raw - offset) × calibration_matrix
        self.corrected_mag_vector = np.dot(self.mag_matrix, (raw_mag_vector + self.mag_offset))

        # Unpack corrected vector to class variables
        self.last_mx, self.last_my, self.last_mz = self.corrected_mag_vector

        # Compute heading with offset
        heading_radians = math.atan2(self.last_my, self.last_mx)
        heading_degrees = heading_radians * 180.0 / math.pi
        heading_degrees += self.heading_offset  # Apply user-defined offset

        if heading_degrees < 0:
            heading_degrees += 360.0  # Convert to range [0, 360)

        self.last_heading_degrees = heading_degrees
        self.get_logger().info(f"Heading: {self.last_heading_degrees}")

    def updateData(self):
        """
        Timer callback (10 Hz) to publish the latest magnetometer data.
        """
        # 1) Publish MagneticField message
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = "witmotion_link"

        mag_msg.magnetic_field.x = self.last_mx
        mag_msg.magnetic_field.y = self.last_my
        mag_msg.magnetic_field.z = self.last_mz
        mag_msg.magnetic_field_covariance = [0.0] * 9  # No known covariance

        self.mag_publisher.publish(mag_msg)

        # 2) Publish Heading as Float32 message
        heading_msg = Float32()
        heading_msg.data = self.last_heading_degrees
        self.angle_publisher.publish(heading_msg)

        # Log for debugging
        # self.get_logger().info(f"Published Heading: {self.last_heading_degrees:.2f}° (Offset: {self.heading_offset:.2f}°)")
        # self.get_logger().info(f"Published Magnetic Field (T): {self.corrected_mag_vector}")

def main(args=None):
    rclpy.init(args=args)
    node = WitMotionMagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
