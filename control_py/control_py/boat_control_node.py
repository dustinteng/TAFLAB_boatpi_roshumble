import rclpy
from rclpy.node import Node
from taflab_msgs.msg import ControlData  # Import ControlData message

class BoatControlNode(Node):
    def __init__(self):
        super().__init__('boat_control_node')
        
        # Subscriber to listen to /boatcontrol topic
        self.subscription = self.create_subscription(
            ControlData,  # Update to use ControlData message type
            '/boatcontrol',
            self.control_callback,
            10
        )
                                                                           
        # Actuator control setup (placeholder for actual servo/ESC control logic)
        self.get_logger().info("BoatControlNode has been initialized and listening to /boatcontrol")

    def control_callback(self, msg):
        # Access control data directly from the message fields
        rudder_value = msg.servo_rudder
        sail_value = msg.servo_sail
        throttle_value = msg.esc

        # Control the actuators based on received values
        self.control_rudder(rudder_value)
        self.control_sail(sail_value)
        self.control_esc(throttle_value)

    def control_rudder(self, value):
        # Implement rudder servo control logic
        self.get_logger().info(f"Rudder set to: {value}")

    def control_sail(self, value):
        # Implement sail servo control logic
        self.get_logger().info(f"Sail set to: {value}")

    def control_esc(self, value):
        # Implement ESC control logic
        self.get_logger().info(f"ESC set to: {value}")

def main(args=None):
    rclpy.init(args=args)
    node = BoatControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
