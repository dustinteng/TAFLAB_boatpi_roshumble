"""
test_publisher Node for ROS2 Package: Full System Test

This node simulates a full navigation system for testing purposes. It publishes simulated sensor data
and control signals that represent the boat's state during autonomous navigation. Key functionalities include:

- Publishing simulated GPS data (using sensor_msgs/NavSatFix) to the "/gps/fix" topic.
- Publishing simulated wind data (Float32) to the "/wind_data" topic.
- Sending an initialization message (String) to the "/auto_init" topic to trigger autonomous mode.
- Publishing simulated heading data (Float32) to the "/as5600_angle" topic.
- Publishing waypoints (NavSatFix) to the "/ground_targets" topic as part of the navigation target queue.
- Publishing execution state (Bool) to the "/executing_state" topic to control the flow of the test.

Additionally, the node subscribes to:
- Rudder control commands (ControlData) from the "/boatcontrol" topic to update the boat's rudder angle.
- Execution state updates (Bool) from the "/executing_state" topic to trigger waypoint publication.

The node utilizes a timer callback to regularly publish messages, simulating continuous sensor updates and
control feedback. The rudder callback function simulates heading changes in response to control inputs, providing
a dynamic behavior for testing navigation logic and control data processing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Bool
from taflab_msgs.msg import ControlData  # Custom message for rudder control
import time

class FullSystemTest(Node):
    def __init__(self):
        super().__init__('full_system_test')

        # Publishers
        self.gps_publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.wind_publisher = self.create_publisher(Float32, '/wind_data', 10)
        self.init_publisher = self.create_publisher(String, '/auto_init', 10)
        self.heading_publisher = self.create_publisher(Float32, '/as5600_angle', 10)
        self.waypoint_queue_publisher = self.create_publisher(NavSatFix, '/ground_targets', 10)
        self.execution_state_publisher = self.create_publisher(Bool, '/executing_state', 10)

        # Subscribers
        self.rudder_subscriber = self.create_subscription(ControlData, '/boatcontrol', self.rudder_callback, 10)
        self.execution_state_subscriber = self.create_subscription(Bool, '/executing_state', self.execution_state_callback, 10)

        self.timer = self.create_timer(1.0, self.publish_messages)

        # Waypoints for testing
        # First waypoint is set directly north of the initial position:
        # initial position: (37.7749, -122.4194)
        # waypoint 1: (37.7849, -122.4194) – about 1.1 km to the north.
        self.waypoints = [
            (37.7849, -122.4194),  # Directly north, forcing a 90° left turn from east
            (37.7854, -122.4189),  # Slightly northeast of the first waypoint
            (37.7860, -122.4185),  # Continuing the route
        ]
        self.current_waypoint_idx = 0

        # Simulated heading and rudder angle
        self.current_heading = 90.0  # Start facing east
        self.current_rudder_angle = 0.0  # Rudder is initially neutral
        self.autonomous_mode = False  # Track the state of navigation

        self.get_logger().info("[TEST] Full system test initialized.")

    def execution_state_callback(self, msg):
        """Monitor the execution state to determine when to send the next waypoint."""
        if msg.data:  # Execution state is True
            if self.current_waypoint_idx < len(self.waypoints):
                self.get_logger().info(f"[TEST] Execution state true, sending waypoint {self.current_waypoint_idx + 1}")
                self.publish_waypoint(self.waypoints[self.current_waypoint_idx])
                self.current_waypoint_idx += 1
        else:
            self.get_logger().info("[TEST] Execution state set to false. No more waypoints to send.")

    def publish_messages(self):
        """Publish test data to simulate full navigation."""
        self.publish_gps_data()
        self.publish_wind_data()
        self.publish_heading()
        self.publish_init_message()

        # Publish execution state: True if there are remaining waypoints; otherwise, publish False.
        if self.current_waypoint_idx < len(self.waypoints):
            self.publish_execution_state(True)
        else:
            self.publish_execution_state(False)

    def publish_execution_state(self, state):
        """Publish execution state (True to allow publishing, False to stop)."""
        exec_msg = Bool()
        exec_msg.data = state
        self.execution_state_publisher.publish(exec_msg)
        self.get_logger().info(f"[TEST] Published Execution State: {exec_msg.data}")

    def publish_gps_data(self):
        """Simulate GPS data."""
        gps_msg = NavSatFix()
        if self.current_waypoint_idx > 0:
            # Simulate that the boat has reached the previous waypoint.
            gps_msg.latitude, gps_msg.longitude = self.waypoints[self.current_waypoint_idx - 1]
        else:
            gps_msg.latitude, gps_msg.longitude = (37.7749, -122.4194)  # Initial position

        gps_msg.altitude = 10.0
        self.gps_publisher.publish(gps_msg)
        self.get_logger().info(f"[TEST] Published GPS Data: {gps_msg.latitude}, {gps_msg.longitude}")

    def publish_wind_data(self):
        """Simulate wind data."""
        wind_msg = Float32()
        wind_msg.data = 45.0  # Example wind angle
        self.wind_publisher.publish(wind_msg)
        self.get_logger().info(f"[TEST] Published Wind Data: {wind_msg.data}")

    def publish_heading(self):
        """Simulate magnetometer heading data."""
        heading_msg = Float32()
        heading_msg.data = self.current_heading
        self.heading_publisher.publish(heading_msg)
        self.get_logger().info(f"[TEST] Published Heading: {heading_msg.data}")

    def publish_init_message(self):
        """Send initialization message."""
        init_msg = String()
        init_msg.data = 'True'
        self.init_publisher.publish(init_msg)

        if not self.autonomous_mode:
            self.get_logger().info("[TEST] Published Init Message: Starting Autonomous Mode")
            self.autonomous_mode = True

    def publish_waypoint(self, waypoint):
        """Publish a waypoint to the waypoint queue node."""
        waypoint_msg = NavSatFix()
        waypoint_msg.latitude, waypoint_msg.longitude = waypoint
        waypoint_msg.altitude = 10.0
        self.waypoint_queue_publisher.publish(waypoint_msg)
        self.get_logger().info(f"[TEST] Published Waypoint to Queue: {waypoint_msg.latitude}, {waypoint_msg.longitude}")

    def rudder_callback(self, msg):
        """Receives rudder commands from the RudderServoControlNode."""
        self.current_rudder_angle = msg.servo_rudder
        self.get_logger().info(f"[TEST] Received Rudder Angle Command: {self.current_rudder_angle:.2f}°")
        self.update_heading()

    def update_heading(self):
        """Simulates the boat's response to rudder input by changing heading.
           A nonzero rudder command will change the current heading based on a simple turn_rate.
        """
        if abs(self.current_rudder_angle) > 1.0:
            # Simulated turning speed (degrees per second)
            turn_rate = self.current_rudder_angle * 0.05  
            self.current_heading = (self.current_heading + turn_rate) % 360
            self.get_logger().info(f"[TEST] Updated Heading: {self.current_heading:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = FullSystemTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
