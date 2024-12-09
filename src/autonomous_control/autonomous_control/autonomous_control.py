import rclpy
from rclpy.node import Node
from taflab_msgs.msg import ControlData, CalibrationData
from sensor_msgs.msg import NavSatFix, MagneticField
from std_msgs.msg import String, Float32, Bool
from queue import Queue
import threading
import math
import os

class AutonomousControlNode(Node):    
    def __init__(self):
        super().__init__('autonomous_control')
        self.get_logger().info("Autonomous Control Node has been started")
        
        # Create queue for waypoints
        self.waypoints = []
        self.heading = 0
        self.windAngle = 0 # Wind angle relative to boat
        self.straight = 90
        self.range = 25 # Range of one sided rudder movement
        
        # Create subcriptions
        self.create_subscription(NavSatFix, '/gps/fix', self.navigateToDestination, 10) # Run navigation when new GPS coordinates are published
        self.create_subscription(NavSatFix, '/boat/target_coordinates', self.setNewTarget, 10) # Append new target to list
        self.create_subscription(MagneticField, '/magnetic_field', self.setHeading, 10) # Calculate heading
        self.create_subscription(Float32, '/as5600_angle', self.setWindAngle, 10) # Calculate wind angle
        
        # Create publishers
        self.rudderPublisher = self.create_publisher(Float32, '/rud_cmd_auto', 10) # Publish rudder command for controller
        self.sailPublisher = self.create_publisher(Float32, '/sail_cmd_auto', 10) # Publish sail command for controller
        
    def navigateToDestination(self, msg):
        currentLat = msg.latitude
        currentLon = msg.longitude
        self.turnSailTo()
        if (self.waypoints):
            targetLat, targetLon = self.waypoints[0]
            distance = self.calculateDistance(currentLat, currentLon, targetLat, targetLon)
            targetBearing = self.calculateBearing(currentLat, currentLon, targetLat, targetLon)
            self.turnRudderTo(targetBearing)
            if (distance < 5): # Arrived at destination, remove target
                self.waypoints.pop(0)
        
    def turnRudderTo(self, targetBearing):
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
            
        self.rudderPublisher.publish(target_rudder_pos)
        
    def turnSailTo(self):
        if 15 < self.windAngle < 90:
            target_sail_pos = self.windAngle - 15
        elif 270 < windAngle < 345:
            target_sail_pos = self.windAngle + 15 - 180
        elif windAngle >= 90 or windAngle <= 270:
            target_sail_pos = self.windAngle / 2
        else:
            target_sail_pos = 0

        target_sail_pos -= 90
        if target_sail_pos < 0:
            target_sail_pos += 180
            
        self.sailPublisher.publish(target_sail_pos)
        
    def setNewTarget(self, msg):
        self.waypoints.append((msg.latitude, msg.longitude))
        
    def setHeading(self, msg):
        self.heading = math.atan2(msg.magnetic_field_y, msg.magnetic_field_x) * 180 / math.PI
        if (self.heading < 0): 
            self.heading += 360
        if (self.heading > 360): 
            self.heading -= 360
        
    def setWindAngle(self, msg):
        from_range = 4095
        to_range = 360
        scaled_value = msg.data / from_range
        self.windAngle = scaled_value * to_range
        
    def calculateDistance(self, lat1, lon1, lat2, lon2):
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
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        y = math.sin(lon2_rad - lon1_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon2_rad - lon1_rad)
        bearing = math.degrees(math.atan2(y, x))

        if bearing < 0:
            bearing += 360

        return bearing
        
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
