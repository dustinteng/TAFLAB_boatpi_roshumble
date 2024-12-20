import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from std_msgs.msg import Float32

# AS5600's I2C address and register map
AS5600_ADDRESS = 0x36
RAW_ANGLE_REGISTER = 0x0C




class AS5600Node(Node):
    def __init__(self):
        super().__init__('as5600_node')
        self.bus = SMBus(1)  # Raspberry Pi I2C bus
        self.angle_publisher = self.create_publisher(Float32, 'as5600_angle', 10)
        self.timer = self.create_timer(0.1, self.read_angle)  # Publish every 0.1 seconds
        #self.olderdata array - save older 10. here to filter sensors
    def read_angle(self):
        try:
            # Read 2 bytes of the raw angle
            angle_data = self.bus.read_i2c_block_data(AS5600_ADDRESS, RAW_ANGLE_REGISTER, 2)
            
            # Debug: log raw data
            self.get_logger().debug(f"Raw angle data: {angle_data}")
            
            # Calculate angle in degrees from the raw value
            raw_angle = (angle_data[0] << 8) | angle_data[1]
            angle_degrees = (raw_angle / 4096) * 360.0  # AS5600 has 12-bit resolution

            # Check for potentially erroneous values
            if angle_degrees < 0 or angle_degrees > 360:
                self.get_logger().warn(f"Unexpected angle value: {angle_degrees} degrees")

            # Publish the angle
            #if self.angle_data < ## 
            self.angle_publisher.publish(Float32(data=angle_degrees))
            # self.get_logger().info(f"Published angle: {angle_degrees:.2f} degrees")
            #

        except Exception as e:
            self.get_logger().error(f"Error reading angle: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AS5600Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
