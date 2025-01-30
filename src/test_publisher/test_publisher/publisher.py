import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from taflab_msgs.msg import ControlData  # Custom message for rudder control
import math

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')

        # Publishers for required topics
        self.gps_publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.wind_publisher = self.create_publisher(Float32, '/wind_data', 10)
        self.init_publisher = self.create_publisher(String, '/auto_init', 10)
        self.heading_publisher = self.create_publisher(Float32, '/as5600_angle', 10)
        self.waypoint_publisher = self.create_publisher(NavSatFix, '/rudder_servo_control', 10)

        # Subscriber to read rudder commands
        self.rudder_subscriber = self.create_subscription(ControlData, '/boatcontrol', self.rudder_callback, 10)

        # Timer for publishing sensor data
        self.timer = self.create_timer(1.0, self.publish_messages)  # Publish every second

        # Simulated heading and rudder angle
        self.current_heading = 90.0  # Start facing east
        self.current_rudder_angle = 0.0  # Rudder is initially neutral

    def publish_messages(self):
        # Simulated GPS Data
        gps_msg = NavSatFix()
        gps_msg.latitude = 37.7749
        gps_msg.longitude = -122.4194
        gps_msg.altitude = 10.0
        self.gps_publisher.publish(gps_msg)
        self.get_logger().info('[TEST] Published GPS Data')

        # Simulated Wind Data
        wind_msg = Float32()
        wind_msg.data = 45.5
        self.wind_publisher.publish(wind_msg)
        self.get_logger().info('[TEST] Published Wind Data')

        # Simulated Heading (updated based on rudder)
        self.update_heading()
        heading_msg = Float32()
        heading_msg.data = self.current_heading
        self.heading_publisher.publish(heading_msg)
        self.get_logger().info(f'[TEST] Published Updated Heading: {self.current_heading:.2f}')

        # Simulated Initialization Message
        init_msg = String()
        init_msg.data = 'True'
        self.init_publisher.publish(init_msg)
        self.get_logger().info('[TEST] Published Init Message')

        # Simulated Waypoint Data
        waypoint_msg = NavSatFix()
        waypoint_msg.latitude = 37.7750  # Slightly north to create a turning scenario
        waypoint_msg.longitude = -122.4195
        waypoint_msg.altitude = 10.0
        self.waypoint_publisher.publish(waypoint_msg)
        self.get_logger().info('[TEST] Published Waypoint Data')

    def rudder_callback(self, msg):
        """ Receives rudder commands from the RudderServoControlNode """
        self.current_rudder_angle = msg.servo_rudder
        self.get_logger().info(f'[TEST] Received Rudder Angle Command: {self.current_rudder_angle:.2f}°')

    def update_heading(self):
        """ Simulates the boat's response to rudder input by changing heading """
        if abs(self.current_rudder_angle) > 1.0:
            # Simulated turning speed (degrees per second)
            turn_rate = self.current_rudder_angle * 0.05  
            
            # Update heading, ensuring it wraps around 0-360 degrees
            self.current_heading = (self.current_heading + turn_rate) % 360

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
