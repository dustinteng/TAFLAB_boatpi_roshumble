#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Bool
from taflab_msgs.msg import ControlData
import time

class FullSystemTest(Node):
    def __init__(self):
        super().__init__('full_system_test')

        # Publishers (correct topics)
        self.gps_publisher        = self.create_publisher(NavSatFix,   '/gps/fix',            10)
        self.wind_publisher       = self.create_publisher(Float32,    '/as5600_angle',       10)
        self.heading_publisher    = self.create_publisher(Float32,    '/witmotion_heading',  10)
        self.init_publisher       = self.create_publisher(String,      '/auto_init',          10)
        self.waypoint_publisher   = self.create_publisher(NavSatFix,   '/ground_targets',     10)
        self.exec_state_publisher = self.create_publisher(Bool,        '/executing_state',    10)

        # Subscribers
        self.rudder_sub = self.create_subscription(
            ControlData, '/boatcontrol', self.rudder_callback, 10)
        self.exec_sub = self.create_subscription(
            Bool, '/executing_state', self.execution_state_callback, 10)
        self.reached_sub = self.create_subscription(
            Bool, '/reached_state', self.reached_callback, 10)

        # Fire off our timer every second
        self.timer = self.create_timer(1.0, self.publish_messages)

        # Test waypoints
        self.waypoints = [
            (37.7750, -122.4195),
            (37.7755, -122.4197),
            (37.7760, -122.4200),
        ]
        self.current_wp = 0

        # Simulated state
        self.current_heading = 90.0
        self.current_rudder  = 0.0
        self.autonomous_mode = False

        self.get_logger().info("[TEST] Full system test initialized.")

    def publish_messages(self):
        # Publish all the sensors + kick off auto
        self.publish_gps()
        self.publish_wind()
        self.publish_heading()
        self.publish_init()
        self.publish_exec_state(True)

    def publish_gps(self):
        msg = NavSatFix()
        if self.current_wp > 0:
            msg.latitude, msg.longitude = self.waypoints[self.current_wp-1]
        else:
            msg.latitude, msg.longitude = (37.7749, -122.4194)
        msg.altitude = 10.0
        self.gps_publisher.publish(msg)
        self.get_logger().info(f"[TEST] GPS: {msg.latitude:.6f}, {msg.longitude:.6f}")

    def publish_wind(self):
        msg = Float32()
        msg.data = 45.0
        self.wind_publisher.publish(msg)
        self.get_logger().info(f"[TEST] Wind (/as5600_angle): {msg.data}")

    def publish_heading(self):
        msg = Float32()
        msg.data = self.current_heading
        self.heading_publisher.publish(msg)
        self.get_logger().info(f"[TEST] Heading (/witmotion_heading): {msg.data}")

    def publish_init(self):
        msg = String()
        msg.data = 'True'
        self.init_publisher.publish(msg)
        if not self.autonomous_mode:
            self.get_logger().info("[TEST] Init→True (autonomous mode)")
            self.autonomous_mode = True

    def publish_exec_state(self, ok: bool):
        msg = Bool()
        msg.data = ok
        self.exec_state_publisher.publish(msg)
        self.get_logger().info(f"[TEST] Exec state: {msg.data}")

    def execution_state_callback(self, msg: Bool):
        if msg.data and self.current_wp < len(self.waypoints):
            wp = self.waypoints[self.current_wp]
            self.get_logger().info(f"[TEST] Exec→True, sending waypoint {self.current_wp+1}")
            out = NavSatFix()
            out.latitude, out.longitude = wp
            out.altitude = 10.0
            self.waypoint_publisher.publish(out)
            self.get_logger().info(f"[TEST] Waypoint→/ground_targets: {out.latitude:.6f}, {out.longitude:.6f}")
            self.current_wp += 1
        else:
            self.get_logger().info("[TEST] Exec=False or no more waypoints")

    def rudder_callback(self, msg: ControlData):
        self.current_rudder = msg.servo_rudder
        self.get_logger().info(f"[TEST] Received rudder cmd: {self.current_rudder:.2f}°")
        self.update_heading()

    def reached_callback(self, msg: Bool):
        self.get_logger().info(f"[TEST] Reached state: {msg.data}")

    def update_heading(self):
        if abs(self.current_rudder) > 1.0:
            turn_rate = self.current_rudder * 0.05
            self.current_heading = (self.current_heading + turn_rate) % 360

def main(args=None):
    rclpy.init(args=args)
    node = FullSystemTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
