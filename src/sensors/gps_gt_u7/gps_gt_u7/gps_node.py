#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        
        # Declare parameters for the serial port and baudrate.
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
        
        # Create the publisher for NavSatFix messages.
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Timer for periodic reading (every 0.1 sec)
        self.timer = self.create_timer(0.1, self.read_gps_data)
        
    def read_gps_data(self):
        # Only attempt to read if there is data waiting
        if self.serial.in_waiting > 0:
            try:
                # Read one line from the serial port.
                line = self.serial.readline().decode('ascii', errors='replace').strip()
            except Exception as e:
                self.get_logger().error(f"Error reading serial data: {e}")
                return
            
            # Log the raw line at DEBUG level.
            self.get_logger().debug(f"Received line: {line}")
            
            # Only process lines that start with $GPGGA.
            if line.startswith('$GPGGA'):
                try:
                    msg = pynmea2.parse(line)
                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = self.get_clock().now().to_msg()
                    
                    # Convert and assign latitude and longitude.
                    try:
                        gps_msg.latitude = float(msg.latitude) if msg.latitude else 0.0
                        gps_msg.longitude = float(msg.longitude) if msg.longitude else 0.0
                    except ValueError:
                        gps_msg.latitude = 0.0
                        gps_msg.longitude = 0.0
                    
                    # Convert and assign altitude.
                    try:
                        gps_msg.altitude = float(msg.altitude) if msg.altitude else 0.0
                    except ValueError:
                        gps_msg.altitude = 0.0
                    
                    self.gps_pub.publish(gps_msg)
                    self.get_logger().info(
                        f"Published GPS data: lat {gps_msg.latitude}, lon {gps_msg.longitude}, alt {gps_msg.altitude}"
                    )
                except pynmea2.ParseError as e:
                    self.get_logger().error(f"Failed to parse NMEA sentence: {e}")
            else:
                # If the sentence is not $GPGGA, log it at DEBUG level.
                self.get_logger().debug(f"Ignored non-GPGGA sentence: {line}")

    def destroy_node(self):
        # Ensure the serial port is closed during shutdown.
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
