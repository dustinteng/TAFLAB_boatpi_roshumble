import rclpy
from rclpy.node import Node
from taflab_msgs.msg import ControlData, CalibrationData
from sensor_msgs.msg import NavSatFix, MagneticField
from std_msgs.msg import Float32
import math

class AutonomousControlNode(Node):
    def __init__(self):
        super().__init__('autonomous_control')
        self.get_logger().info("Autonomous Control Node has been started")

        # Initialization
        self.waypoints = []  # Queue of waypoints (latitude, longitude)
        self.heading = 0  # Current heading of the boat in degrees
        self.windAngle = 0  # Wind angle relative to the boat in degrees
        self.straight = 90  # Neutral rudder position
        self.range = 25  # Range for one-sided rudder movement in degrees
        self.currentSailPos = 0

        # Subscriptions
        self.create_subscription(NavSatFix, '/gps/fix', self.navigateToDestination, 10)
        self.create_subscription(NavSatFix, '/boat/target_coordinates', self.setNewTarget, 10)
        self.create_subscription(Float32, '/as5600_angle', self.setWindAngle, 10)
        self.create_subscription(Float32, '/witmotion_heading', self.setHeading, 10)


        # Publishers
        self.rudderPublisher = self.create_publisher(Float32, '/rud_cmd_auto', 10)
        self.sailPublisher = self.create_publisher(Float32, '/sail_cmd_auto', 10)

    def navigateToDestination(self, msg):
        """
        Navigate the boat to the next waypoint in the queue.
        """
        currentLat = msg.latitude
        currentLon = msg.longitude

        if self.waypoints:
            targetLat, targetLon = self.waypoints[0]
            distance = self.calculateDistance(currentLat, currentLon, targetLat, targetLon)
            targetBearing = self.calculateBearing(currentLat, currentLon, targetLat, targetLon)

            self.turnRudderTo(targetBearing)

            if distance < 5:  # Close enough to the target waypoint
                self.get_logger().info("Arrived at waypoint: ({}, {})".format(targetLat, targetLon))
                self.waypoints.pop(0)

    def turnRudderTo(self, targetBearing):
        """
        Adjust the rudder to steer the boat toward the target bearing.
        """
        delta = self.heading - targetBearing
        if delta > 0:
            if delta > 180:
                target_rudder_pos = self.straight + math.pow(((360 - delta) / 180), 0.5) * self.range
            else:
                target_rudder_pos = self.straight - math.pow((delta / 180), 0.5) * self.range
        elif delta < 0:
            if delta < -180:
                target_rudder_pos = self.straight - math.pow(((360 + delta) / 180), 0.5) * self.range
            else:
                target_rudder_pos = self.straight + math.pow((-delta / 180), 0.5) * self.range
        else:
            target_rudder_pos = self.straight

        self.rudderPublisher.publish(Float32(data=target_rudder_pos))

    def turnSailTo(self):
        """
        Adjust the sail based on the current wind angle.
        """
        if 15 < self.windAngle < 90:
            target_sail_pos = self.windAngle - 15
        elif 270 < self.windAngle < 345:
            target_sail_pos = self.windAngle + 15 - 180
        elif self.windAngle >= 90 or self.windAngle <= 270:
            target_sail_pos = self.windAngle / 2
        else:
            target_sail_pos = 0

        target_sail_pos -= 90
        if target_sail_pos < 0:
            target_sail_pos += 180

        self.sailPublisher.publish(Float32(data=target_sail_pos))

    def setNewTarget(self, msg):
        """
        Add a new waypoint to the navigation queue.
        """
        self.waypoints.append((msg.latitude, msg.longitude))
        self.get_logger().info("New target added: ({}, {})".format(msg.latitude, msg.longitude))


    def setHeading(self, msg):
        """
        Update the current heading from /witmotion_heading topic.
        """
        self.heading = msg.data
        # self.get_logger().info(f"Heading: {self.heading}")

    def setWindAngle(self, msg):
        """
        Update the wind angle and adjust the sail position accordingly.
        """
        self.windAngle = msg.data
        self.turnSailTo()

    def calculateDistance(self, lat1, lon1, lat2, lon2):
        """
        Calculate the distance between two GPS coordinates using the Haversine formula.
        """
        R = 6371000  # Radius of the Earth in meters
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        a = math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        return distance

    def calculateBearing(self, lat1, lon1, lat2, lon2):
        """
        Calculate the bearing between two GPS coordinates.
        """
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        y = math.sin(lon2_rad - lon1_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon2_rad - lon1_rad)
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()