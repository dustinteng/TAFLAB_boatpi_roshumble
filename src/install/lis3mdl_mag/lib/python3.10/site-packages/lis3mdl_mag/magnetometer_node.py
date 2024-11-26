#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import smbus2
import time

class LIS3MDLNode(Node):
    def __init__(self):
        super().__init__('lis3mdl_node')
        self.declare_parameter('i2c_address', 0x1c)  # Parameter for I2C address
        self.declare_parameter('sensitivity', 6842.0)  # Sensitivity for +/-4 gauss
        self.address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.sensitivity = self.get_parameter('sensitivity').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(MagneticField, 'magnetic_field', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize the I2C bus and magnetometer
        try:
            self.bus = smbus2.SMBus(1)
            self.init_magnetometer()
            self.get_logger().info("LIS3MDL magnetometer initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize magnetometer: {e}")

    def init_magnetometer(self):
        # Configure the magnetometer registers
        try:
            self.bus.write_byte_data(self.address, 0x20, 0x70)  # CTRL_REG1
            self.bus.write_byte_data(self.address, 0x21, 0x00)  # CTRL_REG2
            self.bus.write_byte_data(self.address, 0x22, 0x00)  # CTRL_REG3
            self.bus.write_byte_data(self.address, 0x23, 0x0C)  # CTRL_REG4
            self.bus.write_byte_data(self.address, 0x24, 0x40)  # CTRL_REG5
            self.get_logger().debug("Magnetometer registers configured.")
        except Exception as e:
            self.get_logger().error(f"Error configuring magnetometer: {e}")

    def read_magnetometer(self):
        try:
            # Read 6 bytes of data from the magnetometer
            data = self.bus.read_i2c_block_data(self.address, 0x28 | 0x80, 6)
            # Combine the readings
            x = self.combine_data(data[0], data[1])
            y = self.combine_data(data[2], data[3])
            z = self.combine_data(data[4], data[5])
            
            # Convert to gauss using sensitivity parameter
            x_gauss = x / self.sensitivity
            y_gauss = y / self.sensitivity
            z_gauss = z / self.sensitivity
            
            # Log the raw and converted data at debug level
            self.get_logger().debug(f"Raw data: x={x}, y={y}, z={z}")
            self.get_logger().debug(f"Converted to Gauss: x={x_gauss}, y={y_gauss}, z={z_gauss}")
            
            return x_gauss, y_gauss, z_gauss

        except Exception as e:
            self.get_logger().error(f"Error reading magnetometer: {e}")
            return None, None, None  # Return None to handle in callback

    def combine_data(self, low, high):
        # Combines two bytes of data and converts to signed integer
        value = low | (high << 8)
        if value > 32767:
            value -= 65536
        return value

    def timer_callback(self):
        x, y, z = self.read_magnetometer()
        
        # Only publish if data is valid
        if x is not None and y is not None and z is not None:
            msg = MagneticField()
            msg.magnetic_field.x = x
            msg.magnetic_field.y = y
            msg.magnetic_field.z = z
            msg.magnetic_field_covariance = [0.0] * 9  # Assume covariance is unknown
            self.publisher_.publish(msg)
            # self.get_logger().info(f'Published: x={x:.4f}, y={y:.4f}, z={z:.4f}')
        else:
            self.get_logger().warn("Magnetometer data is invalid, skipping publish.")

def main(args=None):
    rclpy.init(args=args)
    lis3mdl_node = LIS3MDLNode()
    rclpy.spin(lis3mdl_node)
    lis3mdl_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
