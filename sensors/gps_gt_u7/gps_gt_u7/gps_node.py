
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        
        # Parameters /dev/ttyS0 or /dev/ttyAMA0
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 9600)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Initialize Serial Connection
        try:
            self.serial = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Connected to GPS module on {port} at {baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            rclpy.shutdown()
            return
        
        # Publisher
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        
        # Timer for periodic reading
        self.timer = self.create_timer(1.0, self.read_gps_data)
        
    def read_gps_data(self):
        if self.serial.in_waiting > 0:
            line = self.serial.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GPGGA'):
                try:
                    msg = pynmea2.parse(line)
                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = self.get_clock().now().to_msg()
                    
                    # Set latitude and longitude
                    gps_msg.latitude = float(msg.latitude) if msg.latitude else float('0.0')
                    gps_msg.longitude = float(msg.longitude) if msg.longitude else float('0.0')
                    
                    # Set altitude and handle missing/invalid values
                    try:
                        gps_msg.altitude = float(msg.altitude) if msg.altitude else float('0.0')
                    except ValueError:
                        gps_msg.altitude = float(0.0)
                    
                    self.gps_pub.publish(gps_msg)
                    self.get_logger().info(f"Published GPS data: {gps_msg}")
                except pynmea2.ParseError as e:
                    self.get_logger().error(f"Failed to parse NMEA sentence: {e}")

    def destroy_node(self):
        self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    def destroy_node(self):
        self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
