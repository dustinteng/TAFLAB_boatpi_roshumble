import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Bool

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.gps_publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.wind_publisher = self.create_publisher(Float32, '/wind_data', 10)
        self.init_publisher = self.create_publisher(String, '/auto_init', 10)
        self.timer = self.create_timer(1.0, self.publish_messages)  # Publish every second

    def publish_messages(self):
        # GPS Data
        gps_msg = NavSatFix()
        gps_msg.latitude = 37.7749
        gps_msg.longitude = -122.4194
        gps_msg.altitude = 10.0
        self.gps_publisher.publish(gps_msg)
        self.get_logger().info('Published GPS Data')

        # Wind Data
        wind_msg = Float32()
        wind_msg.data = 45.5
        self.wind_publisher.publish(wind_msg)
        self.get_logger().info('Published Wind Data')

        # Initialization
        init_msg = String()
        init_msg.data = 'True'
        self.init_publisher.publish(init_msg)
        self.get_logger().info('Published Init Message')

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
