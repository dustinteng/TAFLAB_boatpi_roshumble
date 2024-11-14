import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2
from collections import Counter
from datetime import datetime, timedelta

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
        self.gps_history = []
        
    def read_gps_data(self):
        if self.serial.in_waiting > 0:
            line = self.serial.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GPGGA'):
                try:
                    msg = pynmea2.parse(line)
                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = self.get_clock().now().to_msg()
                    
                    # Set latitude and longitude
                    latitude = float(msg.latitude) if msg.latitude else float('0.0')
                    longitude = float(msg.longitude) if msg.longitude else float('0.0')
                    filtered_lat, filtered_lon = self.filter_gps_data(latitude, longitude)
                    gps_msg.latitude = filtered_lat
                    gps_msg.longitude = filtered_lon
                    # Set altitude and handle missing/invalid values
                    try:
                        gps_msg.altitude = float(msg.altitude) if msg.altitude else float('0.0')
                    except ValueError:
                        gps_msg.altitude = float(0.0)
                    
                    self.gps_pub.publish(gps_msg)
                    self.get_logger().info(f"Published GPS data: {gps_msg}")
                except pynmea2.ParseError as e:
                    self.get_logger().error(f"Failed to parse NMEA sentence: {e}")
    def filter_gps_data(self, latitude, longitude):
        # Define valid GPS bounds
        min_lat, max_lat = -90.0, 90.0
        min_lon, max_lon = -180.0, 180.0

        # Record current time
        current_time = datetime.now()

        # Remove outdated data from history
        self.gps_history = [
            (lat, lon, ts) for lat, lon, ts in self.gps_history 
            if ts > current_time - timedelta(seconds=10)
        ]

        # Add new data to history
        self.gps_history.append((latitude, longitude, current_time))

        # Check bounds
        if not (min_lat <= latitude <= max_lat and min_lon <= longitude <= max_lon):
            self.get_logger().warning("GPS data out of bounds, replacing with mode.")
            latitude, longitude = self.get_mode()

        # Check if data deviates substantially
        avg_lat = sum(lat for lat, lon, ts in self.gps_history) / len(self.gps_history)
        avg_lon = sum(lon for lat, lon, ts in self.gps_history) / len(self.gps_history)
        substantial_deviation = 0.01  # Example threshold
        if (abs(latitude - avg_lat) > substantial_deviation or 
            abs(longitude - avg_lon) > substantial_deviation):
            self.get_logger().warning("GPS data deviates substantially, replacing with mode.")
            latitude, longitude = self.get_mode()

        return latitude, longitude

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
