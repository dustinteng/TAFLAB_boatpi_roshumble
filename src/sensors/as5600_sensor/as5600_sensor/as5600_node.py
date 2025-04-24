import os
import json
import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from std_msgs.msg import Float32

AS5600_ADDRESS = 0x36
RAW_ANGLE_REGISTER = 0x0C

class AS5600Node(Node):
    def __init__(self):
        super().__init__('as5600_node')

        self.bus = SMBus(1)  # Raspberry Pi I2C bus
        self.angle_publisher = self.create_publisher(Float32, 'as5600_angle', 10)
        self.angle_total_publisher = self.create_publisher(Float32, 'windvane_and_sail_heading', 10)
        self.create_subscription(Float32, '/currentSailPos', self.saveCurrentSailPos, 10)
        self.timer = self.create_timer(0.25, self.read_angle)  # 4 Hz

        self.currentAngle = 0
        self.currentSailPos = 0

        # Load config values
        self.load_config()

    def load_config(self):
        config_path = "/home/boat/Desktop/version4/TAFLAB_boatpi_roshumble/src/config.json"
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
                self.offset = config.get("windvane_offset", -3132)
                reverse_str = config.get("windvane_reverse", "True")
                self.reverse = reverse_str.lower() == "true"
                self.get_logger().info(f"Loaded config: offset={self.offset}, reverse={self.reverse}")
        except Exception as e:
            self.get_logger().warn(f"Failed to load config.json: {e}")
            self.offset = 0
            self.reverse = False

    def read_angle(self):
        try:
            # Read 2 bytes of raw angle data
            angle_data = self.bus.read_i2c_block_data(AS5600_ADDRESS, RAW_ANGLE_REGISTER, 2)
            raw_angle = ((angle_data[0] << 8) | angle_data[1]) + self.offset

            # Normalize raw angle
            raw_angle %= 4096

            # Convert to degrees
            windVaneAngle = (raw_angle / 4096.0) * 360.0

            # Reverse direction if needed
            if self.reverse:
                windVaneAngle = 360.0 - windVaneAngle

            # Add sail offset and wrap around 360
            self.currentAngle = (windVaneAngle + (self.currentSailPos - 90)) % 360

            # Publish both values
            self.angle_publisher.publish(Float32(data=windVaneAngle))
            self.angle_total_publisher.publish(Float32(data=self.currentAngle))

            self.get_logger().info(f"Windvane Angle: {windVaneAngle:.2f}° | Sail Pos: {self.currentSailPos:.2f}° | Total: {self.currentAngle:.2f}°")

        except Exception as e:
            self.get_logger().error(f"Error reading AS5600: {e}")

    def saveCurrentSailPos(self, msg):
        self.currentSailPos = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = AS5600Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
